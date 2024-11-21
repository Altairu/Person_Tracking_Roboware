import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from fastapi import FastAPI, WebSocket as FastAPIWebSocket
from fastapi.responses import HTMLResponse
import uvicorn
import os

ipadress_ = '192.168.2.212'
port_ = 8080
path = '/home/altair/Roboware/UI.txt'

app = FastAPI()

if not os.path.exists(path):
    raise FileNotFoundError(f'File not found: {path}')

with open(path, 'r') as f:
    html = f.read()

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('web_socket_node')
        self.pub = self.create_publisher(String, 'web_socket_commands', 10)
        self.send_data = ''

        @app.get("/")
        async def get():
            return HTMLResponse(html)

        @app.websocket('/ws')
        async def websocket_endpoint(websocket: FastAPIWebSocket):
            await websocket.accept()
            try:
                while True:
                    data = await websocket.receive_text()
                    self.pub.publish(String(data=data))
                    await websocket.send_text(f"Received: {data}")
            except Exception as e:
                print(f'WebSocket error: {str(e)}')

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
    fastapi_thread = threading.Thread(target=run_fastapi)

    ros2_thread.start()
    fastapi_thread.start()

    ros2_thread.join()
    fastapi_thread.join()

if __name__ == '__main__':
    main()
