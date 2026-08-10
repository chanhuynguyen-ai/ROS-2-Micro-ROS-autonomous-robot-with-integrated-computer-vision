#!/usr/bin/env python3

import os
import time
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node


class CameraRecordNode(Node):
    def __init__(self):
        super().__init__('camera_record_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('save_dir', '/root/yahboomcar_ws/videos')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 20.0)

        # Nếu True: cửa sổ xem có nút REC/STOP/QUIT.
        # Video lưu ra vẫn luôn sạch, không có chữ.
        self.declare_parameter('show_overlay', True)

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.show_overlay = bool(self.get_parameter('show_overlay').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.cap = cv2.VideoCapture(self.camera_id)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera id {self.camera_id}')
            raise RuntimeError(f'Cannot open camera id {self.camera_id}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.window_name = 'SimpleRobot Camera Recorder'

        self.recording = False
        self.video_writer = None
        self.current_video_path = None
        self.frame_count = 0

        self.button_rec = (20, 20, 150, 70)
        self.button_stop = (170, 20, 340, 70)
        self.button_quit = (360, 20, 490, 70)

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.timer = self.create_timer(1.0 / self.fps, self.loop)

        self.get_logger().info('Camera recorder started')
        self.get_logger().info('Saved video is CLEAN: no text, no buttons, no overlay')
        self.get_logger().info('Keyboard: r=REC, s=STOP/SAVE, q=QUIT')
        self.get_logger().info(f'Video save directory: {self.save_dir}')

    def inside_button(self, x, y, button):
        x1, y1, x2, y2 = button
        return x1 <= x <= x2 and y1 <= y <= y2

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.show_overlay and self.inside_button(x, y, self.button_rec):
            self.start_recording()

        elif self.show_overlay and self.inside_button(x, y, self.button_stop):
            self.stop_recording()

        elif self.show_overlay and self.inside_button(x, y, self.button_quit):
            self.get_logger().info('Quit button clicked')
            self.shutdown_node()

    def start_recording(self):
        if self.recording:
            self.get_logger().warn('Already recording')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'simplerobot_record_{timestamp}.mp4'
        self.current_video_path = os.path.join(self.save_dir, filename)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        self.video_writer = cv2.VideoWriter(
            self.current_video_path,
            fourcc,
            self.fps,
            (self.frame_width, self.frame_height)
        )

        if not self.video_writer.isOpened():
            self.get_logger().error('Cannot create video writer')
            self.video_writer = None
            self.current_video_path = None
            return

        self.recording = True
        self.frame_count = 0
        self.get_logger().info(f'Start recording clean video: {self.current_video_path}')

    def stop_recording(self):
        if not self.recording:
            self.get_logger().warn('Not recording')
            return

        self.recording = False

        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

        self.get_logger().info(f'Saved clean video: {self.current_video_path}')
        self.current_video_path = None

    def draw_button(self, frame, button, text):
        x1, y1, x2, y2 = button
        cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 40, 40), -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(
            frame,
            text,
            (x1 + 15, y1 + 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

    def draw_overlay_for_preview_only(self, frame):
        self.draw_button(frame, self.button_rec, 'REC')
        self.draw_button(frame, self.button_stop, 'STOP/SAVE')
        self.draw_button(frame, self.button_quit, 'QUIT')

        if self.recording:
            status = f'RECORDING frames: {self.frame_count}'
            color = (0, 0, 255)
        else:
            status = 'READY'
            color = (0, 255, 0)

        cv2.putText(
            frame,
            status,
            (20, self.frame_height - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2
        )

    def loop(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Failed to read camera frame')
            return

        clean_frame = cv2.resize(frame, (self.frame_width, self.frame_height))

        # Quan trọng:
        # File video lưu bằng clean_frame, không dùng frame đã vẽ chữ/nút.
        if self.recording and self.video_writer is not None:
            self.video_writer.write(clean_frame)
            self.frame_count += 1

        preview_frame = clean_frame.copy()

        # Overlay chỉ để nhìn trên màn hình, không ghi vào video.
        if self.show_overlay:
            self.draw_overlay_for_preview_only(preview_frame)

        cv2.imshow(self.window_name, preview_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            self.start_recording()
        elif key == ord('s'):
            self.stop_recording()
        elif key == ord('q'):
            self.shutdown_node()

    def shutdown_node(self):
        if self.recording:
            self.stop_recording()

        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()

        self.get_logger().info('Camera recorder stopped')

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = CameraRecordNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(e)
    finally:
        if node is not None:
            if node.recording:
                node.stop_recording()

            if node.cap is not None:
                node.cap.release()

            cv2.destroyAllWindows()

            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
