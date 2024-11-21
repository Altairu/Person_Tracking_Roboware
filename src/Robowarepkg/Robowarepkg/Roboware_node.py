import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class RobowareNode(Node):
    def __init__(self):
        super().__init__('Roboware_node')
        self.subscription_control = self.create_subscription(
            String,
            'web_socket_pub',
            self.command_callback,
            10
        )
        self.subscription_follow = self.create_subscription(
            Float32MultiArray,
            'follow_speeds',
            self.follow_callback,
            10
        )
        self.publisher_speeds = self.create_publisher(
            Float32MultiArray,
            'target_speeds',
            10
        )

        self.mode = "STOP"  # 初期モード
        self.target_right = 0.0  # 浮動小数点に変更
        self.target_left = 0.0   # 浮動小数点に変更

    def command_callback(self, msg):
        command = msg.data
        if command.startswith("MODE:"):
            self.mode = command.split(":")[1]
            self.get_logger().info(f"Mode switched to: {self.mode}")
        elif command.startswith("SPEED:"):
            speeds = list(map(float, command.split(":")[1].split(",")))  # 浮動小数点に変換
            self.target_right = speeds[0]
            self.target_left = speeds[1]

    def follow_callback(self, msg):
        if self.mode == "FOLLOW":
            self.target_right, self.target_left = msg.data

    def publish_speeds(self):
        msg = Float32MultiArray()
        msg.data = [float(self.target_right), float(self.target_left)]  # 必ず浮動小数点として設定
        self.publisher_speeds.publish(msg)
        self.get_logger().info(
            f"Published Target Speeds: Right={self.target_right}, Left={self.target_left}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RobowareNode()
    timer = node.create_timer(0.1, node.publish_speeds)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
