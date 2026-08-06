import json
import math
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from can_msgs.msg import Frame
from altair_interfaces.msg import CanStatus


# MDDの CAN ID (altair_framework MDD仕様準拠)
MDD_BASE_ID = 0x200
# パラメータ送信先: 0x200 (ch1), 0x201 (ch2)
# モード設定:       0x210
# 速度目標値:       0x220
# ステータス受信:   0x230
# 速度フィードバック受信: 0x250

CMD_PARAM_CH1_ID  = MDD_BASE_ID + 0x00   # 0x200
CMD_PARAM_CH2_ID  = MDD_BASE_ID + 0x01   # 0x201
CMD_MODE_ID       = MDD_BASE_ID + 0x10   # 0x210
CMD_TARGET_ID     = MDD_BASE_ID + 0x20   # 0x220
FEEDBACK_STATUS_ID = MDD_BASE_ID + 0x30  # 0x230
FEEDBACK_SPEED_ID  = MDD_BASE_ID + 0x50  # 0x250

# mdd_params.json のパス (このファイルの一階層上のディレクトリ)
_DEFAULT_PARAMS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'mdd_params.json'
)

# 制御状態
STATE_WAITING = 'WAITING'    # CAN 接続待ち
STATE_SETUP   = 'SETUP'      # パラメータ送信中
STATE_RUNNING = 'RUNNING'    # 制御実行中

# 目標値送信周期 (秒)
CONTROL_PERIOD = 0.01  # 10ms = 100Hz


def _clamp_int16(v):
    return max(-32768, min(32767, int(v)))


class CanNode(Node):
    """
    シリアル通信ノードの代替。
    altair_can_bridge が提供する /altair/can/tx /altair/can/rx を通じて
    MDD (0x200) に速度指令・PIDパラメータを送受信する。
    """

    def __init__(self):
        super().__init__('can_node')
        self.callback_group = ReentrantCallbackGroup()

        # 状態初期化
        self._state = STATE_WAITING
        self._state_lock = threading.Lock()

        # 目標速度キャッシュ [ch1_mm_s, ch2_mm_s]
        self._target_ch1_mm_s = 0.0
        self._target_ch2_mm_s = 0.0
        self._target_lock = threading.Lock()

        # パラメータのロード
        self._params_path = _DEFAULT_PARAMS_PATH
        self._params = self._load_params()

        # パブリッシャ: CAN 送信
        self.can_tx_pub = self.create_publisher(Frame, '/altair/can/tx', 100)

        # パブリッシャ: 速度フィードバック (元の wheel_feedback トピックに互換)
        self.wheel_feedback_pub = self.create_publisher(
            Float32MultiArray, 'wheel_feedback', 10
        )

        # サブスクライバ: CAN 受信
        self.can_rx_sub = self.create_subscription(
            Frame, '/altair/can/rx',
            self._handle_can_rx,
            100,
            callback_group=self.callback_group
        )

        # サブスクライバ: CAN 接続状態
        self.can_status_sub = self.create_subscription(
            CanStatus, '/altair/can/status',
            self._handle_can_status,
            10,
            callback_group=self.callback_group
        )

        # サブスクライバ: wheel_targets (Roboware_node から mm/s 単位で届く)
        self.wheel_targets_sub = self.create_subscription(
            Float32MultiArray, 'wheel_targets',
            self._handle_wheel_targets,
            10,
            callback_group=self.callback_group
        )

        # サービスサーバー: GUI からのパラメータ再送信トリガー
        self.send_params_srv = self.create_service(
            Trigger, '/roboware/can/send_params',
            self._handle_send_params_service,
            callback_group=self.callback_group
        )

        # 10ms 周期の速度目標値送信タイマー
        self.control_timer = self.create_timer(
            CONTROL_PERIOD,
            self._publish_target,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            f'can_node 起動。CAN 接続待ち... (パラメータファイル: {self._params_path})'
        )

    # --- パラメータ管理 ---

    def _load_params(self):
        """mdd_params.json を読み込む。ファイルがなければデフォルト値を返す。"""
        path = os.path.normpath(self._params_path)
        try:
            with open(path, 'r', encoding='utf-8') as f:
                params = json.load(f)
            self.get_logger().info(f'パラメータをロードしました: {path}')
            return params
        except FileNotFoundError:
            self.get_logger().warn(
                f'パラメータファイルが見つかりません: {path} デフォルト値を使用します。'
            )
            return {
                'ch1': {'p': 1.0, 'i': 0.0, 'd': 0.0, 'direction': 1, 'diameter_mm': 100.0},
                'ch2': {'p': 1.0, 'i': 0.0, 'd': 0.0, 'direction': 1, 'diameter_mm': 100.0},
            }
        except Exception as e:
            self.get_logger().error(f'パラメータファイル読み込みエラー: {e}')
            return {
                'ch1': {'p': 1.0, 'i': 0.0, 'd': 0.0, 'direction': 1, 'diameter_mm': 100.0},
                'ch2': {'p': 1.0, 'i': 0.0, 'd': 0.0, 'direction': 1, 'diameter_mm': 100.0},
            }

    def save_params(self, new_params: dict):
        """GUIからの更新内容を mdd_params.json に保存する。"""
        path = os.path.normpath(self._params_path)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(new_params, f, indent=2, ensure_ascii=False)
            self._params = new_params
            self.get_logger().info(f'パラメータを保存しました: {path}')
        except Exception as e:
            self.get_logger().error(f'パラメータ保存エラー: {e}')

    # --- 速度換算 ---

    def _mm_s_to_int16(self, mm_s: float, ch: str) -> int:
        """
        mm/s を MDD 送信用 int16 に変換する。
        rps = mm_s / (pi * diameter_mm)
        int16 = clamp(int(rps * 10), -32768, 32767)
        direction パラメータで符号を反転できる。
        """
        p = self._params.get(ch, {})
        diameter_mm = float(p.get('diameter_mm', 100.0))
        direction = int(p.get('direction', 1))
        if diameter_mm <= 0:
            return 0
        rps = mm_s / (math.pi * diameter_mm)
        return _clamp_int16(rps * 10.0 * direction)

    # --- CAN フレーム送信ユーティリティ ---

    def _send_frame(self, can_id: int, payload: bytes, dlc: int = 8):
        frame = Frame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.id = can_id
        frame.dlc = dlc
        frame.is_extended = False
        frame.is_rtr = False
        # data は 8 要素の配列にパディング
        data_list = list(payload)
        data_list += [0] * (8 - len(data_list))
        frame.data = data_list[:8]
        self.can_tx_pub.publish(frame)

    # --- パラメータ & モード送信 ---

    def _send_pid_params(self):
        """ch1 / ch2 の PIDパラメータと車輪径 (方向込み) を送信する。"""
        for idx, ch in enumerate(['ch1', 'ch2']):
            p = self._params.get(ch, {})
            p_gain = _clamp_int16(float(p.get('p', 1.0)) * 1000)
            i_gain = _clamp_int16(float(p.get('i', 0.0)) * 1000)
            d_gain = _clamp_int16(float(p.get('d', 0.0)) * 1000)
            diameter_mm = float(p.get('diameter_mm', 100.0))
            direction = int(p.get('direction', 1))
            diameter_dir = _clamp_int16(diameter_mm * direction)

            payload = struct.pack('<hhhh', p_gain, i_gain, d_gain, diameter_dir)
            can_id = CMD_PARAM_CH1_ID + idx
            self._send_frame(can_id, payload)
            self.get_logger().info(
                f'PIDパラメータ送信: {ch} (ID: {hex(can_id)}) '
                f'P={p_gain} I={i_gain} D={d_gain} 車輪={diameter_dir}'
            )
            # マイコン側のバッファ溢れを防ぐ小待機
            time.sleep(0.02)

    def _send_mode(self):
        """速度制御モード (0) を ch1/ch2/ch3/ch4 に設定する。"""
        # 4 チャンネル分の mode (0=速度制御) を送信
        payload = bytes([0, 0, 0, 0])
        self._send_frame(CMD_MODE_ID, payload, dlc=4)
        self.get_logger().info(f'モード設定送信 (ID: {hex(CMD_MODE_ID)}): 速度制御モード')

    def _do_setup(self):
        """パラメータ送信 -> モード設定 -> RUNNING 遷移を実行する。"""
        self.get_logger().info('パラメータ送信を開始します...')
        try:
            self._params = self._load_params()
            self._send_pid_params()
            self._send_mode()
            with self._state_lock:
                self._state = STATE_RUNNING
            self.get_logger().info('パラメータ送信完了。制御モードに移行しました。')
        except Exception as e:
            self.get_logger().error(f'パラメータ送信中にエラーが発生しました: {e}')
            with self._state_lock:
                self._state = STATE_SETUP

    # --- コールバック ---

    def _handle_can_status(self, msg: CanStatus):
        """CAN 接続状態の変化を監視して状態機械を更新する。"""
        with self._state_lock:
            current_state = self._state

        if msg.is_connected and current_state == STATE_WAITING:
            self.get_logger().info(
                f'CAN 接続確認 (ポート: {msg.active_port})。パラメータ送信を開始します...'
            )
            with self._state_lock:
                self._state = STATE_SETUP
            # パラメータ送信はブロッキングするため別スレッドで実行
            threading.Thread(target=self._do_setup, daemon=True).start()

        elif not msg.is_connected and current_state != STATE_WAITING:
            self.get_logger().warn('CAN 接続が切断されました。再接続待ちに戻ります。')
            with self._state_lock:
                self._state = STATE_WAITING
            # 目標値をゼロにリセット
            with self._target_lock:
                self._target_ch1_mm_s = 0.0
                self._target_ch2_mm_s = 0.0

    def _handle_wheel_targets(self, msg: Float32MultiArray):
        """Roboware_node からの wheel_targets (mm/s) を受け取りキャッシュする。"""
        if len(msg.data) < 2:
            self.get_logger().error('wheel_targets のデータ数が不正です (2 要素必要)。')
            return
        with self._target_lock:
            self._target_ch1_mm_s = float(msg.data[0])
            self._target_ch2_mm_s = float(msg.data[1])

    def _handle_can_rx(self, frame: Frame):
        """CAN 受信フレームを処理する。"""
        can_id = frame.id

        # 速度フィードバック (0x250)
        if can_id == FEEDBACK_SPEED_ID:
            if frame.dlc < 8:
                return
            try:
                speeds_raw = struct.unpack('<hhhh', bytes(frame.data[:8]))
                # 0.01 rps 単位 -> rps に変換
                speeds_rps = [float(s) / 100.0 for s in speeds_raw]
                # ch1/ch2 を wheel_feedback (rps) としてパブリッシュ
                fb_msg = Float32MultiArray()
                fb_msg.data = [speeds_rps[0], speeds_rps[1]]
                self.wheel_feedback_pub.publish(fb_msg)
            except Exception as e:
                self.get_logger().error(f'速度フィードバックのデコードに失敗しました: {e}')

        # ステータス受信 (0x230) - マイコンリセット検知
        elif can_id == FEEDBACK_STATUS_ID:
            if frame.dlc < 6:
                return
            system_status = frame.data[5]
            with self._state_lock:
                current_state = self._state
            if system_status == 0 and current_state == STATE_RUNNING:
                self.get_logger().warn(
                    'マイコンの待機状態リセットを検知しました。SETUP 状態に戻ります。'
                )
                with self._state_lock:
                    self._state = STATE_SETUP
                threading.Thread(target=self._do_setup, daemon=True).start()

    def _publish_target(self):
        """10ms 周期で速度目標値を 0x220 に送信する (RUNNING 状態のみ)。"""
        with self._state_lock:
            current_state = self._state

        if current_state != STATE_RUNNING:
            # 制御中以外はゼロ指令を送り続けて安全を確保
            payload = struct.pack('<hhhh', 0, 0, 0, 0)
            self._send_frame(CMD_TARGET_ID, payload)
            return

        with self._target_lock:
            ch1_mm_s = self._target_ch1_mm_s
            ch2_mm_s = self._target_ch2_mm_s

        ch1_int16 = self._mm_s_to_int16(ch1_mm_s, 'ch1')
        ch2_int16 = self._mm_s_to_int16(ch2_mm_s, 'ch2')

        # ch3/ch4 は未使用なのでゼロ
        payload = struct.pack('<hhhh', ch1_int16, ch2_int16, 0, 0)
        self._send_frame(CMD_TARGET_ID, payload)

    def _handle_send_params_service(self, request, response):
        """GUI からのパラメータ再送信トリガーを受け付けるサービスハンドラー。"""
        self.get_logger().info('パラメータ再送信サービスが呼ばれました。')
        with self._state_lock:
            current_state = self._state

        if current_state == STATE_WAITING:
            response.success = False
            response.message = 'CAN が未接続のためパラメータを送信できません。'
            return response

        with self._state_lock:
            self._state = STATE_SETUP
        threading.Thread(target=self._do_setup, daemon=True).start()
        response.success = True
        response.message = 'パラメータ再送信を開始しました。'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CanNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
