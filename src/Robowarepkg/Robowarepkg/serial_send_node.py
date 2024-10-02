import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')
        self.sub = self.create_subscription(Float32MultiArray, 'wheel_rps', self.callback, 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # シリアルポート設定

    def callback(self, msg):
        vr = msg.data[0]  # 右車輪速度
        vl = msg.data[1]  # 左車輪速度
        data = struct.pack('>ff', vr, vl)  # 速度データをパック
        self.serial_port.write(b'\xA5\xA5' + data)  # ヘッダーと速度データを送信
        self.get_logger().info(f'Sent data: vr={vr}, vl={vl}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
