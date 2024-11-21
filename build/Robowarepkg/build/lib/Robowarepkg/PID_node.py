import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.prev_error = 0
        self.integral = 0

class PIDNode(Node):
    def __init__(self):
        super().__init__('PID_node')
        self.subscription_target = self.create_subscription(
            Float32MultiArray,
            'target_speeds',
            self.target_callback,
            10
        )
        self.subscription_current = self.create_subscription(
            Float32MultiArray,
            'current_speeds',
            self.current_callback,
            10
        )
        self.publisher_motor = self.create_publisher(
            Float32MultiArray,
            'motor_speeds',
            10
        )

        self.target_right = 0
        self.target_left = 0
        self.current_right = 0
        self.current_left = 0
        self.last_time = self.get_clock().now()

        # PIDインスタンス
        self.pid_right = PID(kp=0.03, ki=0.1, kd=0.0)
        self.pid_left = PID(kp=0.03, ki=0.1, kd=0.0)

    def target_callback(self, msg):
        self.target_right, self.target_left = msg.data

    def current_callback(self, msg):
        self.current_right, self.current_left = msg.data

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        control_right = self.pid_right.compute(self.target_right, self.current_right, dt)
        control_left = self.pid_left.compute(self.target_left, self.current_left, dt)

        motor_msg = Float32MultiArray()
        motor_msg.data = [control_right, control_left]
        self.publisher_motor.publish(motor_msg)
        self.get_logger().info(
            f"Control Outputs: Right={control_right}, Left={control_left} "
            f"Target: Right={self.target_right}, Left={self.target_left} "
            f"Current: Right={self.current_right}, Left={self.current_left}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    timer = node.create_timer(0.1, node.control_loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
