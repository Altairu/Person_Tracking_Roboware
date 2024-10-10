import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from fastapi import FastAPI, WebSocket as FastAPIWebSocket
from fastapi.responses import HTMLResponse
import uvicorn
import os

# IPアドレスとポートの指定
ipadress_ = '192.168.137.216'
port_ = 8080

# HTMLファイルのパスを指定
path = '/home/altair/Roboware/UI.txt'

# FastAPIのインスタンスを作成
app = FastAPI()

# HTMLファイルが存在するか確認し、読み込み
if not os.path.exists(path):
    raise FileNotFoundError(f'File not found: {path}')

with open(path, 'r') as f:
    html = f.read()

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('web_socket_node')
        self.send_data = ''
        self.pub = self.create_publisher(String, 'web_socket_pub', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'estimated_position', self.callback, 10)

        @app.get("/")
        async def get():
            return HTMLResponse(html)

        @app.websocket('/ws')
        async def websocket_endpoint(websocket: FastAPIWebSocket):
            await websocket.accept()
            try:
                while True:
                    receive_data = await websocket.receive_text()
                    msg = String()
                    msg.data = receive_data
                    self.pub.publish(msg)
                    await websocket.send_text(",".join(map(str, self.send_data)))
            except Exception as e:
                print(f'WebSocket error: {str(e)}')
            finally:
                print('WebSocket disconnected')

    def callback(self, sub_msg):
        self.send_data = sub_msg.data

def run_ros2():
    rclpy.init()
    node = WebSocketNode()
    rclpy.spin(node)
    rclpy.shutdown()

def run_fastapi():
    config = uvicorn.Config(app, host=ipadress_, port=port_, log_level="info")
    server = uvicorn.Server(config)
    server.run()

def main():
    ros2_thread = threading.Thread(target=run_ros2)
    ros2_thread.start()

    fastapi_thread = threading.Thread(target=run_fastapi)
    fastapi_thread.start()

    ros2_thread.join()
    fastapi_thread.join()

if __name__ == '__main__':
    main()
