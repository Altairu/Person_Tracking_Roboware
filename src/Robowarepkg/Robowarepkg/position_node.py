import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class PositionNode(Node):
    def __init__(self):
        super().__init__('position_node')
        self.pub = self.create_publisher(Float32MultiArray, 'estimated_position', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'robot_position', self.callback, 10)

        self.x = 0.0  # x座標 [m]
        self.y = 0.0  # y座標 [m]
        self.theta = 0.0  # 向き [rad]

    def callback(self, msg):
        omega_r = msg.data[0]  # 右車輪のRPS
        omega_l = msg.data[1]  # 左車輪のRPS

        # RPSからV（mm/s）、ω（rad/s）を計算
        velocity = (omega_r + omega_l) / 2.0 * 100.0  # mm/s
        angular_velocity = (omega_r - omega_l) / 500.0  # rad/s（車輪間距離500mm）

        # mmをmに変換して自己位置を更新
        self.update_position(velocity / 1000, angular_velocity)

        position_msg = Float32MultiArray(data=[self.x, self.y, self.theta])
        self.pub.publish(position_msg)
        self.get_logger().info(f'Estimated position: x={self.x}m, y={self.y}m, theta={self.theta}rad')

    def update_position(self, velocity, angular_velocity):
        dt = 0.1  # タイムステップ [s]
        self.x += velocity * dt * math.cos(self.theta)  # x座標 [m]
        self.y += velocity * dt * math.sin(self.theta)  # y座標 [m]
        self.theta += angular_velocity * dt  # 向き [rad]

def main(args=None):
    rclpy.init(args=args)
    node = PositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
