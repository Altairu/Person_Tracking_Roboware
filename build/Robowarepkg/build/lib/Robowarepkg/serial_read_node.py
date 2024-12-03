import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialReadNode(Node):
    def __init__(self):
        super().__init__('serial_read_node')

        # シリアルポート設定
        self.serial_port = "/dev/ttyACM0"
        self.baudrate = 115200
        self.timeout = 0.01

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            self.get_logger().info(f"Connected to serial port: {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.ser = None

        # パブリッシャ設定
        self.publisher = self.create_publisher(Float32MultiArray, 'wheel_feedback', 10)

        # タイマーで定期的にデータを読む
        self.timer = self.create_timer(0.01, self.read_from_serial)

    def read_from_serial(self):
        if self.ser is not None and self.ser.in_waiting >= 6:
            try:
                received_data = self.ser.read(6)
                if len(received_data) == 6 and received_data[0] == 0xA5 and received_data[1] == 0xA5:
                    _, _, right_speed, left_speed = struct.unpack('>BBhh', received_data)
                    msg = Float32MultiArray()
                    msg.data = [float(right_speed), float(left_speed)]
                    self.publisher.publish(msg)

                    # デバッグログ
                    self.get_logger().info(f"Received from serial: Right={right_speed}, Left={left_speed}")
                else:
                    self.get_logger().error("Invalid data format received from serial.")
            except Exception as e:
                self.get_logger().error(f"Error reading data from serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReadNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
