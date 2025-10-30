import math
import random
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node


class FaceAnimationNode(Node):
    def __init__(self):
        super().__init__('face_animation_node')
        self.window_name = 'Robot Face'
        self.frame_width = 1280
        self.frame_height = 720

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.start_time = time.time()
        self.expression = 'neutral'
        self.expression_end_time = None
        self.next_expression_time = self._schedule_next_expression(0.0)
        self.wink_left = True

        self.pupil_offset = np.zeros(2, dtype=np.float32)
        self.pupil_target = np.zeros(2, dtype=np.float32)
        self.next_pupil_update = 0.5

        self.create_timer(1.0 / 60.0, self.update_frame)

    def _schedule_next_expression(self, now):
        return now + random.uniform(6.0, 10.0)

    def update_frame(self):
        now = time.time() - self.start_time

        if self.expression != 'neutral' and self.expression_end_time and now >= self.expression_end_time:
            self.expression = 'neutral'
            self.expression_end_time = None
            self.next_expression_time = self._schedule_next_expression(now)

        if self.expression == 'neutral' and now >= self.next_expression_time:
            self.expression = random.choice(['blink', 'wink', 'happy', 'sparkle'])
            durations = {
                'blink': 0.25,
                'wink': 0.8,
                'happy': 1.4,
                'sparkle': 1.2,
            }
            self.expression_end_time = now + durations[self.expression]
            if self.expression == 'wink':
                self.wink_left = random.choice([True, False])

        if now >= self.next_pupil_update:
            max_offset = (self.frame_height // 6) * 0.35
            self.pupil_target = np.array([
                random.uniform(-max_offset, max_offset),
                random.uniform(-max_offset * 0.6, max_offset * 0.6),
            ], dtype=np.float32)
            self.next_pupil_update = now + random.uniform(0.6, 1.6)

        self.pupil_offset += (self.pupil_target - self.pupil_offset) * 0.12

        frame = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
        frame[:] = (18, 18, 22)

        center_x = self.frame_width // 2
        center_y = self.frame_height // 2

        eye_spacing = self.frame_width // 5
        eye_radius = self.frame_height // 6
        eye_outline = max(8, eye_radius // 3)
        pupil_radius = max(14, eye_radius // 4)
        whisker_span = self.frame_width // 5

        line_color = (255, 255, 255)
        pupil_color = (30, 45, 70)
        highlight_color = (240, 240, 255)
        cheek_color = (210, 120, 150)

        left_eye_shape = right_eye_shape = 'open'
        mouth_shape = 'neutral'
        eyebrow_raise = 0
        sparkle = False

        if self.expression == 'blink':
            left_eye_shape = right_eye_shape = 'closed'
        elif self.expression == 'wink':
            left_eye_shape = 'closed' if self.wink_left else 'open'
            right_eye_shape = 'open' if self.wink_left else 'closed'
            eyebrow_raise = 8
            mouth_shape = 'smile'
        elif self.expression == 'happy':
            left_eye_shape = right_eye_shape = 'smile'
            eyebrow_raise = 12
            mouth_shape = 'smile'

        eyebrow_base_y = center_y - eye_radius - 50
        eyebrow_length = eye_radius
        eyebrow_thickness = max(6, eye_outline // 2)

        cheeks_y = center_y + eye_radius // 2
        cheek_radius = max(20, eye_radius // 3)
        cv2.circle(frame, (center_x - eye_spacing // 2, cheeks_y), cheek_radius, cheek_color, -1, lineType=cv2.LINE_AA)
        cv2.circle(frame, (center_x + eye_spacing // 2, cheeks_y), cheek_radius, cheek_color, -1, lineType=cv2.LINE_AA)

        def draw_eye(eye_center, shape):
            if shape == 'open':
                cv2.circle(frame, eye_center, eye_radius, line_color, eye_outline, lineType=cv2.LINE_AA)
                inner_radius = eye_radius - eye_outline // 2
                cv2.circle(frame, eye_center, inner_radius, (18, 18, 22), -1, lineType=cv2.LINE_AA)
                pupil_center = (
                    int(eye_center[0] + self.pupil_offset[0]),
                    int(eye_center[1] + self.pupil_offset[1]),
                )
                cv2.circle(frame, pupil_center, pupil_radius, pupil_color, -1, lineType=cv2.LINE_AA)
                highlight_pos = (
                    pupil_center[0] - pupil_radius // 2,
                    pupil_center[1] - pupil_radius // 2,
                )
                cv2.circle(frame, highlight_pos, max(4, pupil_radius // 4), highlight_color, -1, lineType=cv2.LINE_AA)
            elif shape == 'closed':
                start = (eye_center[0] - eye_radius, eye_center[1])
                end = (eye_center[0] + eye_radius, eye_center[1])
                cv2.line(frame, start, end, line_color, eye_outline, lineType=cv2.LINE_AA)
                cv2.ellipse(frame, eye_center, (eye_radius, eye_outline), 0, 0, 180, line_color, 2, lineType=cv2.LINE_AA)
            elif shape == 'smile':
                cv2.ellipse(frame, eye_center, (eye_radius, eye_radius // 2), 0, 0, 180, line_color, eye_outline, lineType=cv2.LINE_AA)
            else:
                cv2.circle(frame, eye_center, eye_radius, line_color, eye_outline, lineType=cv2.LINE_AA)
                for angle in (0, 90, 180, 270):
                    rad = math.radians(angle)
                    pt = (
                        int(eye_center[0] + math.cos(rad) * (eye_radius - eye_outline // 2)),
                        int(eye_center[1] + math.sin(rad) * (eye_radius - eye_outline // 2)),
                    )
                    cv2.circle(frame, pt, max(4, eye_outline // 3), highlight_color, -1, lineType=cv2.LINE_AA)

        left_center = (center_x - eye_spacing, center_y)
        right_center = (center_x + eye_spacing, center_y)
        draw_eye(left_center, left_eye_shape)
        draw_eye(right_center, right_eye_shape)

        for direction, eye_center in ((-1, left_center), (1, right_center)):
            brow_center_y = eyebrow_base_y - eyebrow_raise
            start = (eye_center[0] - eyebrow_length // 2, brow_center_y - direction * 4)
            end = (eye_center[0] + eyebrow_length // 2, brow_center_y + direction * 4)
            cv2.line(frame, start, end, line_color, eyebrow_thickness, lineType=cv2.LINE_AA)

        nose_radius = max(8, eye_radius // 5)
        cv2.circle(frame, (center_x, center_y + eye_radius // 3), nose_radius, line_color, -1, lineType=cv2.LINE_AA)

        mouth_width = self.frame_width // 12
        base_height = mouth_width // 2
        variance = mouth_width // 6
        if mouth_shape == 'wow':
            cv2.circle(frame, (center_x, center_y + self.frame_height // 5), mouth_width // 2, line_color, eye_outline, lineType=cv2.LINE_AA)
        else:
            wave = math.sin(now * 1.2) * variance
            mouth_height = base_height + (variance if mouth_shape == 'smile' else wave)
            mouth_center = (center_x, center_y + self.frame_height // 5)
            cv2.ellipse(frame, mouth_center, (mouth_width, int(mouth_height)), 0, 0, 180, line_color, eye_outline, lineType=cv2.LINE_AA)

        whisker_offsets = (-40, -5, 30)
        base_gap = eye_radius // 2
        for direction in (-1, 1):
            base_x = center_x + direction * (eye_spacing + base_gap)
            for offset in whisker_offsets:
                start = (base_x, center_y + offset)
                end = (base_x + direction * whisker_span, center_y + offset + direction * 6)
                cv2.line(frame, start, end, line_color, max(4, eye_outline // 2), lineType=cv2.LINE_AA)

        if sparkle:
            star_radius = pupil_radius
            star_center = (center_x, center_y - eye_radius // 2)
            cv2.circle(frame, star_center, star_radius, highlight_color, eye_outline // 2, lineType=cv2.LINE_AA)
            cv2.putText(frame, 'Kira!', (star_center[0] + star_radius + 10, star_center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, line_color, 2, lineType=cv2.LINE_AA)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceAnimationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
