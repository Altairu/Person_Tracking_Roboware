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

        # ウィンドウをフルスクリーンで表示する設定
        cv2.namedWindow("RealSense Detection", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("RealSense Detection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # YOLOモデルロード
        self.model = torch.hub.load('/home/altair/Roboware/ultralytics/yolov5', 'custom', 
                                    path='/home/altair/Roboware/ultralytics/yolov5/yolov5s.pt', 
                                    source='local')
        self.create_timer(0.1, self.process_frames)

        # フィルタ用変数
        self.previous_distance = 0.0  # 前回の距離値
        self.distance_history = []  # 距離履歴
        self.history_size = 5  # 移動平均履歴の最大サイズ

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

                    raw_distance = depth_frame.get_distance(center_x, center_y)
                    offset_x = center_x - (color_image.shape[1] // 2)

                    # 距離の処理
                    filtered_distance = self.filter_distance(raw_distance)

                    # Publish camera data
                    msg = Float32MultiArray()
                    msg.data = [filtered_distance, float(offset_x)]
                    self.publisher.publish(msg)

                    self.get_logger().info(f"Published camera data: Distance={filtered_distance:.2f}, Offset={offset_x}")

                    # 画面に検出結果を描画
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, f"Distance: {filtered_distance:.2f}m", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Offset: {offset_x}", (x1, y1 - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    self.previous_distance = filtered_distance  # 前回の距離を更新
                    break

            # カメラ映像を表示
            cv2.imshow("RealSense Detection", color_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing frames: {str(e)}")

    def filter_distance(self, current_distance):
        """
        距離が近づく場合はそのまま、遠ざかる場合はフィルタリングする。
        """
        if current_distance == 0.0:  # 無効値は無視
            return self.previous_distance

        if current_distance < self.previous_distance:
            # 近づいている場合: そのままの値を使用
            self.distance_history = [current_distance]  # 履歴をリセット
            return current_distance
        else:
            # 遠ざかる場合: 移動平均フィルタを適用
            self.distance_history.append(current_distance)
            if len(self.distance_history) > self.history_size:
                self.distance_history.pop(0)  # 古い値を削除
            return sum(self.distance_history) / len(self.distance_history)

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
