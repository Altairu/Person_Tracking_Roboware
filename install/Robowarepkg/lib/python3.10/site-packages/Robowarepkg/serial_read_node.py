import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

# シリアルポート設定
port = "/dev/ttyACM0"  # STM32と通信するポート
baudrate = 115200

class SerialReadNode(Node):
    def __init__(self):
        super().__init__('serial_read_node')
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

        self.timer = self.create_timer(0.1, self.read_from_serial)

    def read_from_serial(self):
        if self.ser is None:
            return

        # シリアルから6バイトを受信
        if self.ser.in_waiting >= 6:
            received_data = self.ser.read(6)
            if len(received_data) == 6 and received_data[0] == 0xA5 and received_data[1] == 0xA5:
                _, _, right_speed, left_speed = struct.unpack('>BBhh', received_data)

                # データをfloat型に変換してパブリッシュ
                msg = Float32MultiArray()
                msg.data = [float(right_speed), float(left_speed)]
                self.pub.publish(msg)

                # デバッグ用ログ
                self.get_logger().info(f"Received: Right Speed={right_speed}, Left Speed={left_speed}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
