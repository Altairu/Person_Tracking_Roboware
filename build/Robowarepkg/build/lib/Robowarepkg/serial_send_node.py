import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')

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

        # サブスクライバ設定
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_controls',
            self.control_callback,
            10
        )

    def control_callback(self, msg):
        if self.ser is not None:
            try:
                if len(msg.data) == 2:
                    right_control, left_control = msg.data
                    send_data = struct.pack('>BBhh', 0xA5, 0xA5, int(right_control), int(left_control))
                    self.ser.write(send_data)

                    # デバッグログ
                    self.get_logger().info(f"Sent to serial: Right={right_control}, Left={left_control}")
                else:
                    self.get_logger().error("Invalid control data length. Expected 2 values.")
            except Exception as e:
                self.get_logger().error(f"Error sending data to serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
