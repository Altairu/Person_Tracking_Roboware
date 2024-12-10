import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs
import numpy as np
import torch
import cv2

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'camera_data', 10)

        # RealSense設定
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # YOLOモデルロード
        self.model = torch.hub.load('/home/altair/Roboware/ultralytics/yolov5', 'custom', 
                                    path='/home/altair/Roboware/ultralytics/yolov5/yolov5s.pt', 
                                    source='local')
        self.create_timer(0.1, self.process_frames)

        # 直近の有効な距離を保存
        self.previous_valid_distance = None

    def process_frames(self):
        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                return

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            results = self.model(color_image)

            for result in results.xyxy[0]:  # 検出結果をループ
                box, conf, cls = result[:4], result[4], int(result[5])
                if cls == 0:  # クラス0（人物）のみ処理
                    x1, y1, x2, y2 = map(int, box)
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                    distance = depth_frame.get_distance(center_x, center_y)
                    offset_x = center_x - (color_image.shape[1] // 2)

                    # 距離が無効（0.0）なら直近の有効な値を使用
                    if distance == 0.0:
                        if self.previous_valid_distance is not None:
                            distance = self.previous_valid_distance
                            self.get_logger().warn("Invalid distance detected. Using previous valid value.")
                        else:
                            self.get_logger().warn("Invalid distance detected. Skipping frame.")
                            continue  # 最初のフレームで無効値の場合スキップ

                    # 有効な距離を保存
                    self.previous_valid_distance = distance

                    # Publish camera data
                    msg = Float32MultiArray()
                    msg.data = [distance, float(offset_x)]
                    self.publisher.publish(msg)

                    self.get_logger().info(f"Published camera data: Distance={distance}, Offset={offset_x}")

                    # 画面に検出結果を描画
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, f"Distance: {distance:.2f}m", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Offset: {offset_x}", (x1, y1 - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    break

            # カメラ映像を表示
            cv2.imshow("RealSense Detection", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing frames: {str(e)}")

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()  # OpenCVのウィンドウを閉じる
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
