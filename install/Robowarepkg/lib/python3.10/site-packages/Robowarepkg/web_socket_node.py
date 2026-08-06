import json
import threading
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from altair_interfaces.msg import CanStatus

from fastapi import FastAPI, WebSocket as FastAPIWebSocket
from fastapi.responses import HTMLResponse
import uvicorn


# IPアドレスとポートの指定
ipadress_ = '0.0.0.0'
port_ = 8080

# HTMLファイルのパス
path = '/home/altair/Person_Tracking_Roboware/UI.txt'

# mdd_params.json のパス
_MDD_PARAMS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'mdd_params.json'
)

# FastAPIのインスタンスを作成
app = FastAPI()

# HTMLファイルが存在するか確認し、読み込み
if not os.path.exists(path):
    raise FileNotFoundError(f'File not found: {path}')

with open(path, 'r') as f:
    html = f.read()

# ROS2 ノードの定義
class WebSocketNode(Node):
    def __init__(self):
        msg = String()
        super().__init__('web_socket_node')
        self.send_data = ''

        # CAN 接続状態キャッシュ
        self._can_connected = False
        self._can_port = 'None'

        # パブリッシャーを作成
        self.pub = self.create_publisher(String, 'web_socket_pub', 10)

        # サブスクリプションを作成し、コールバック関数を設定
        self.sub = self.create_subscription(
            Float32MultiArray, 'estimated_position', self.callback, 10
        )

        # CAN 接続状態サブスクライバ
        self.can_status_sub = self.create_subscription(
            CanStatus, '/altair/can/status',
            self._handle_can_status, 10
        )

        # パラメータ再送信サービスクライアント
        self.send_params_client = self.create_client(
            Trigger, '/roboware/can/send_params'
        )

        # FastAPIルートの定義
        node_ref = self  # クロージャ用に参照を保持

        @app.get('/')
        async def get():
            return HTMLResponse(html)

        @app.websocket('/ws')
        async def websocket_endpoint(websocket: FastAPIWebSocket):
            await websocket.accept()
            try:
                while True:
                    # クライアントからのデータを受信
                    receive_data = await websocket.receive_text()

                    # JSON 形式のデータは MDD パラメータ更新として処理
                    try:
                        data = json.loads(receive_data)
                        if data.get('type') == 'mdd_params':
                            node_ref._handle_mdd_params_update(data.get('params', {}))
                            # 現在のパラメータを返す
                            response = json.dumps({
                                'type': 'mdd_params_ack',
                                'params': node_ref._get_current_params(),
                                'can_connected': node_ref._can_connected,
                                'can_port': node_ref._can_port,
                            })
                            await websocket.send_text(response)
                            continue
                        elif data.get('type') == 'get_mdd_params':
                            # GUI 起動時にパラメータと状態を返す
                            response = json.dumps({
                                'type': 'mdd_params_ack',
                                'params': node_ref._get_current_params(),
                                'can_connected': node_ref._can_connected,
                                'can_port': node_ref._can_port,
                            })
                            await websocket.send_text(response)
                            continue
                    except json.JSONDecodeError:
                        pass

                    # 通常のゲームパッド / モード操作コマンド (CSV) として処理
                    msg.data = receive_data
                    node_ref.pub.publish(msg)

                    # サブスクライブしたデータをクライアントに送信
                    string_send_data = ','.join(map(str, node_ref.send_data))
                    await websocket.send_text(string_send_data)

            except Exception as e:
                print(f'WebSocket error: {str(e)}')
            finally:
                print('WebSocket disconnected')

    # サブスクリプションのコールバック関数
    def callback(self, sub_msg):
        self.send_data = sub_msg.data

    def _handle_can_status(self, msg: CanStatus):
        """CAN 接続状態を更新する。"""
        self._can_connected = msg.is_connected
        self._can_port = msg.active_port

    def _get_current_params(self) -> dict:
        """mdd_params.json の現在値を読み込んで返す。"""
        params_path = os.path.normpath(_MDD_PARAMS_PATH)
        try:
            with open(params_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {
                'ch1': {'p': 1.0, 'i': 0.0, 'd': 0.0, 'direction': 1, 'diameter_mm': 100.0},
                'ch2': {'p': 1.0, 'i': 0.0, 'd': 0.0, 'direction': 1, 'diameter_mm': 100.0},
            }

    def _handle_mdd_params_update(self, new_params: dict):
        """
        GUI から受信したパラメータを mdd_params.json に保存し、
        can_node にパラメータ再送信を要求する。
        """
        params_path = os.path.normpath(_MDD_PARAMS_PATH)
        try:
            os.makedirs(os.path.dirname(params_path), exist_ok=True)
            with open(params_path, 'w', encoding='utf-8') as f:
                json.dump(new_params, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f'MDD パラメータを保存しました: {params_path}')
        except Exception as e:
            self.get_logger().error(f'MDD パラメータ保存エラー: {e}')
            return

        # can_node のパラメータ再送信サービスを非同期で呼び出す
        if self.send_params_client.service_is_ready():
            req = Trigger.Request()
            future = self.send_params_client.call_async(req)
            future.add_done_callback(self._on_send_params_done)
        else:
            self.get_logger().warn(
                '/roboware/can/send_params サービスが利用できません。'
                'can_node が起動しているか確認してください。'
            )

    def _on_send_params_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'パラメータ再送信: {result.message}')
            else:
                self.get_logger().warn(f'パラメータ再送信失敗: {result.message}')
        except Exception as e:
            self.get_logger().error(f'パラメータ再送信サービス呼び出しエラー: {e}')


# ROS2 ノードを実行する関数
def run_ros2():
    rclpy.init()
    node = WebSocketNode()
    rclpy.spin(node)
    rclpy.shutdown()


# FastAPI サーバーを実行する関数
def run_fastapi():
    config = uvicorn.Config(app, host=ipadress_, port=port_, log_level='info')
    server = uvicorn.Server(config)
    server.run()


# メイン関数
def main():
    ros2_thread = threading.Thread(target=run_ros2)
    ros2_thread.start()

    fastapi_thread = threading.Thread(target=run_fastapi)
    fastapi_thread.start()

    ros2_thread.join()
    fastapi_thread.join()


if __name__ == '__main__':
    main()