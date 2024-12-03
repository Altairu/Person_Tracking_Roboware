import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, target, current, dt):
        if target == 0.0 and current == 0.0:
            self.reset()
            return 0.0  # 特別条件: 目標値と現在値が0の場合は出力も0

        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

class PIDNode(Node):
    def __init__(self):
        super().__init__('PID_node')

        self.create_subscription(Float32MultiArray, 'wheel_targets', self.target_callback, 10)
        self.create_subscription(Float32MultiArray, 'wheel_feedback', self.feedback_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, 'wheel_controls', 10)

        self.pid_right = PIDController(kp=0.003, ki=0.01, kd=0.0)
        self.pid_left = PIDController(kp=0.003, ki=0.01, kd=0.0)

        self.target_right = 0.0
        self.target_left = 0.0
        self.current_right = 0.0
        self.current_left = 0.0

        self.create_timer(0.1, self.control_loop)
        self.last_time = self.get_clock().now()

    def target_callback(self, msg):
        if len(msg.data) == 2:
            self.target_right = msg.data[0]
            self.target_left = msg.data[1]
        else:
            self.get_logger().error(f"Invalid data received in 'wheel_targets'. Expected 2 floats, got {len(msg.data)}")

    def feedback_callback(self, msg):
        if len(msg.data) == 2:
            self.current_right = msg.data[0]
            self.current_left = msg.data[1]
        else:
            self.get_logger().error(f"Invalid data received in 'wheel_feedback'. Expected 2 floats, got {len(msg.data)}")

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        control_signal_right = self.pid_right.compute(self.target_right, self.current_right, dt)
        control_signal_left = self.pid_left.compute(self.target_left, self.current_left, dt)

        # 制御信号が有効な範囲内か検証
        if not (self.is_valid_float(control_signal_right) and self.is_valid_float(control_signal_left)):
            self.get_logger().error("Control signals contain invalid values. Skipping this loop.")
            return

        # 制御信号をパブリッシュ
        control_msg = Float32MultiArray()
        control_msg.data = [float(control_signal_right), float(control_signal_left)]
        self.pub.publish(control_msg)

        # デバッグ情報を出力
        self.get_logger().info(
            f"Target: Right={self.target_right}, Left={self.target_left} | "
            f"Current: Right={self.current_right}, Left={self.current_left} | "
            f"Control: Right={control_signal_right}, Left={control_signal_left}"
        )

    def is_valid_float(self, value):
        try:
            float_value = float(value)
            return float('-inf') < float_value < float('inf')  # 有限数の検証
        except ValueError:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
