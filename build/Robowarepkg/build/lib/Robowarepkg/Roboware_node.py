import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class RobowareNode(Node):
    def __init__(self):
        super().__init__('Roboware_node')
        self.subscription = self.create_subscription(
            String,
            'web_socket_pub',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float32MultiArray, 'wheel_targets', 10)

        self.mode = 0  # Initial mode (Control: 0, Follow: 1)
        self.target_right = 0.0
        self.target_left = 0.0

    def listener_callback(self, msg):
        try:
            data = list(map(float, msg.data.split(',')))
            if len(data) != 6:
                self.get_logger().warn("Received invalid data length.")
                return

            mode, rx, ry, lx, ly, stop = data
            self.mode = int(mode)

            if stop == 1:
                self.target_right = 0.0
                self.target_left = 0.0
                self.get_logger().info("Emergency stop activated.")
            elif self.mode == 0:  # Control Mode
                V = float((int(ry) - 106)*100)
                omega = float(-1*(int(lx) - 102) *50)
                self.target_right = V + omega
                self.target_left = V - omega
                self.get_logger().info(f"Control mode | V={V}, Omega={omega} | Target Right={self.target_right}, Left={self.target_left}")
            elif self.mode == 1:  # Follow Mode
                # Placeholder for Follow mode implementation
                self.target_right = 0.0
                self.target_left = 0.0
                self.get_logger().info("Follow mode is not yet implemented.")

            # Publish the wheel targets
            self.publish_targets()

        except ValueError as e:
            self.get_logger().error(f"Error parsing message: {e}")

    def publish_targets(self):
        msg = Float32MultiArray()
        msg.data = [self.target_right, self.target_left]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobowareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
