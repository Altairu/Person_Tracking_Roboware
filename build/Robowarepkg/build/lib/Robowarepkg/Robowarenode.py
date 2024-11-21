import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import math

class Roboware_node(Node):
    def __init__(self):
        super().__init__('roboware_node')
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_rps', 10)
        self.sub = self.create_subscription(String, 'web_socket_pub', self.callback, 10)

        # 二輪車両のパラメータ (車輪間距離、車輪半径) - mm単位
        self.wheel_distance = 500.0  # 車輪間距離[mm] 
        self.wheel_radius = 100.0    # 車輪半径[mm]

    def callback(self, msg):
        try:
            data = msg.data.split(',')
            velocity = float(data[0])  # 指令速度V[mm/s]
            omega = float(data[1])     # 指令角速度ω[rad/s]

            # 車輪のRPS計算（単位はすべてmm）
            vr = (2 * velocity + omega * self.wheel_distance) / (2 * self.wheel_radius)
            vl = (2 * velocity - omega * self.wheel_distance) / (2 * self.wheel_radius)

            rps_msg = Float32MultiArray(data=[vr, vl])
            self.pub.publish(rps_msg)
            self.get_logger().info(f'Calculated RPS: vr={vr}, vl={vl}')
        except Exception as e:
            self.get_logger().error(f'Error in processing: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = Roboware_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
