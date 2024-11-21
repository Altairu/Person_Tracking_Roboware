import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
import torch

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('RealSense_node')
        self.pub = self.create_publisher(Float32MultiArray, 'person_position', 10)

        # RealSenseのセットアップ
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # YOLOv5モデルのロード
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

        # Timerでループ処理
        self.timer = self.create_timer(0.1, self.process_frames)

    def process_frames(self):
        # フレーム取得
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # YOLOでの検出
        results = self.model(color_image)
        for *box, confidence, cls in results.xyxy[0]:
            if int(cls) == 0 and confidence > 0.5:  # クラスID 0: 'person'
                x1, y1, x2, y2 = map(int, box)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                distance = depth_frame.get_distance(center_x, center_y)
                offset_x = center_x - (color_image.shape[1] // 2)

                # データをパブリッシュ
                msg = Float32MultiArray()
                msg.data = [float(distance), float(offset_x)]  # 明示的にfloat型に変換
                self.pub.publish(msg)

                # デバッグログ
                self.get_logger().info(f"Distance: {distance:.2f}m, Offset: {offset_x}px")

                # 検出結果の描画
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f'Distance: {distance:.2f}m', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(color_image, f'Offset: {offset_x}px', (x1, y1 - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 結果表示
        cv2.imshow("RealSense", color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
