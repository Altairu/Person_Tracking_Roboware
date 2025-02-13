import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import csv
import os
import time

class RobowareNode(Node):
    def __init__(self):
        super().__init__('Roboware_node')
        self.subscription = self.create_subscription(
            String,
            'web_socket_pub',
            self.listener_callback,
            10
        )
        self.position_subscription = self.create_subscription(
            Float32MultiArray,
            'camera_data',
            self.position_callback,
            10
        )
        self.publisher = self.create_publisher(Float32MultiArray, 'wheel_targets', 10)

        self.mode = 0  # Initial mode (Control: 0, Follow: 1)
        self.target_right = 0.0
        self.target_left = 0.0
        self.person_distance = 0.0
        self.person_offset = 0.0
        self.kp_v = 5000.0  # Proportional gain for velocity 前50000
        self.kp_omega = 50.0  # Proportional gain for angular velocity
        self.navigation_constant = 2.0  # Proportional navigation constant (N)

        # CSVファイルの設定
        self.csv_file = "PN_data1.csv"
        self.initialize_csv()

        # タイマー設定
        self.timer = self.create_timer(0.1, self.record_data)  # 0.1秒ごとに実行
        self.recording = False  # 記録中フラグ

    def initialize_csv(self):
        """CSVファイルを初期化し、ヘッダーを記録"""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'Time', 'PersonDistance', 'PersonOffset', 'V', 'Omega', 
                    'TargetRight', 'TargetLeft'
                ])

    def save_to_csv(self, time, distance, offset, v, omega, right_speed, left_speed):
        """データをCSVファイルに追記"""
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([time, distance, offset, v, omega, right_speed, left_speed])


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
            elif self.mode == 0:  # Control Mode (手動モード)
                V = float((int(ry) - 106) * 200)
                omega = float(-1 * (int(lx) - 102) * 100)
                self.target_right = V + omega
                self.target_left = V - omega
                self.recording = False  # 記録停止
                self.get_logger().info(f"Control mode | V={V}, Omega={omega} | Target Right={self.target_right}, Left={self.target_left}")
            elif self.mode == 1:  # Follow Mode (画像処理モード)
                self.follow_person()
                self.recording = True  # 記録開始

            # Publish the wheel targets
            self.publish_targets()

        except ValueError as e:
            self.get_logger().error(f"Error parsing message: {e}")

    def position_callback(self, msg):
        if len(msg.data) == 2:  # Assuming message contains [distance, offset]
            self.person_distance = msg.data[0]
            self.person_offset = msg.data[1]
            self.get_logger().info(f"Updated position: Distance={self.person_distance}, Offset={self.person_offset}")
        else:
            self.get_logger().warn("Invalid position data received.")

    def follow_person(self):
        # Proportional Navigation Algorithm (PN)
        V = self.kp_v * (self.person_distance - 1.0)
        omega = -1 * (self.navigation_constant) * self.kp_omega * self.person_offset / max((self.person_distance), 1.0)
        # Clamp the values to maximum limits
        V = max(min(V, 30000.0), -30000.0)  # Max forward/backward velocity
        omega = max(min(omega, 15000.0), -15000.0)  # Max rotational velocity

        self.current_v =V
        self.current_omega =omega

        # Calculate target velocities for left and right wheels
        self.target_right = V + omega
        self.target_left = V - omega

        self.get_logger().info(f"Follow mode | Distance={self.person_distance}, Offset={self.person_offset} | V={V}, Omega={omega} | Target Right={self.target_right}, Left={self.target_left}")

    def publish_targets(self):
        msg = Float32MultiArray()
        msg.data = [self.target_right, self.target_left]
        self.publisher.publish(msg)


    def record_data(self):
        """0.1秒ごとにデータをCSVに記録"""
        if self.recording:  # 記録中のみデータを保存
            current_time = self.get_clock().now().to_msg().sec  # 現在時刻を秒単位で取得
            self.save_to_csv(
                current_time, 
                self.person_distance, 
                self.person_offset, 
                self.current_v, 
                self.current_omega, 
                self.target_right, 
                self.target_left
            )
            
def main(args=None):
    rclpy.init(args=args)
    node = RobowareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
