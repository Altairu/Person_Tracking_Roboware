import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_speeds',
            self.send_to_serial,
            10
        )

        # シリアルポート設定
        self.serial_port = "/dev/ttyACM0"  # 実際のポートに置き換える
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to serial port: {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.ser = None

    def send_to_serial(self, msg):
        if self.ser is None:
            self.get_logger().error("Serial port is not initialized. Cannot send data.")
            return

        # モータースピードの送信
        if len(msg.data) != 2:
            self.get_logger().error("Expected exactly 2 motor speeds, got: {len(msg.data)}")
            return

        right_speed = int(msg.data[0])
        left_speed = int(msg.data[1])

        try:
            # シリアル送信データの構築
            data = struct.pack('>BBhh', 0xA5, 0xA5, right_speed, left_speed)
            self.ser.write(data)

            # ログで送信内容を確認
            self.get_logger().info(f"Sent to serial: Right={right_speed}, Left={left_speed}")
        except Exception as e:
            self.get_logger().error(f"Failed to send data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
