#!/usr/bin/env python3

import os
import time
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


MOVE_BINDINGS = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'k': (0, 0, 0, 0),
    ' ': (0, 0, 0, 0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def clamp(value, low, high):
    return max(low, min(high, value))


class DriveCaptureNode(Node):
    def __init__(self):
        super().__init__('drive_capture_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('save_dir', '/root/yahboomcar_ws/demo_imgs')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 20.0)

        self.declare_parameter('capture_interval', 10.0)

        self.declare_parameter('speed', 0.20)
        self.declare_parameter('turn', 1.00)
        self.declare_parameter('linear_speed_limit', 1.00)
        self.declare_parameter('angular_speed_limit', 5.00)

        self.declare_parameter('show_overlay', True)

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.capture_interval = float(self.get_parameter('capture_interval').value)

        self.speed = float(self.get_parameter('speed').value)
        self.turn = float(self.get_parameter('turn').value)
        self.linear_speed_limit = float(self.get_parameter('linear_speed_limit').value)
        self.angular_speed_limit = float(self.get_parameter('angular_speed_limit').value)

        self.show_overlay = bool(self.get_parameter('show_overlay').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera id {self.camera_id}')
            raise RuntimeError(f'Cannot open camera id {self.camera_id}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.window_name = 'SimpleRobot Drive Capture'

        self.capturing = False
        self.last_capture_time = 0.0
        self.image_count = 0
        self.last_saved_path = ''

        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0

        self.latest_clean_frame = None

        self.button_start = (20, 20, 180, 70)
        self.button_stop = (200, 20, 360, 70)
        self.button_quit = (380, 20, 510, 70)

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.timer = self.create_timer(1.0 / self.fps, self.loop)

        self.get_logger().info('drive_capture_node started')
        self.get_logger().info(f'Images save directory: {self.save_dir}')
        self.get_logger().info(f'Capture interval: {self.capture_interval} seconds')
        self.print_help()

    def print_help(self):
        help_text = """
Keyboard control:
---------------------------
Moving:
   u    i    o
   j    k    l
   m    ,    .

space/k : stop robot

Speed:
q/z : increase/decrease both linear and angular speed
w/x : increase/decrease linear speed only
e/c : increase/decrease angular speed only

Capture:
r : start auto capture
p : stop auto capture
t : capture one image immediately
ESC : quit node

Mouse:
START / STOP / QUIT buttons
---------------------------
"""
        self.get_logger().info(help_text)

    def inside_button(self, x, y, button):
        x1, y1, x2, y2 = button
        return x1 <= x <= x2 and y1 <= y <= y2

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.show_overlay and self.inside_button(x, y, self.button_start):
            self.start_capture()

        elif self.show_overlay and self.inside_button(x, y, self.button_stop):
            self.stop_capture()

        elif self.show_overlay and self.inside_button(x, y, self.button_quit):
            self.shutdown_node()

    def start_capture(self):
        if self.capturing:
            self.get_logger().warn('Auto capture already started')
            return

        self.capturing = True
        self.last_capture_time = 0.0
        self.get_logger().info('Auto capture started')

    def stop_capture(self):
        if not self.capturing:
            self.get_logger().warn('Auto capture is not running')
            return

        self.capturing = False
        self.get_logger().info('Auto capture stopped')

    def save_image(self):
        if self.latest_clean_frame is None:
            self.get_logger().warn('No camera frame available')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'simplerobot_img_{timestamp}_{self.image_count:04d}.jpg'
        path = os.path.join(self.save_dir, filename)

        ok = cv2.imwrite(path, self.latest_clean_frame)

        if ok:
            self.image_count += 1
            self.last_saved_path = path
            self.get_logger().info(f'Saved image: {path}')
        else:
            self.get_logger().error(f'Failed to save image: {path}')

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = float(self.x * self.speed)
        cmd.linear.y = float(self.y * self.speed)
        cmd.linear.z = float(self.z * self.speed)
        cmd.angular.z = float(self.th * self.turn)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.cmd_pub.publish(Twist())

    def handle_key(self, key):
        if key == 255:
            return

        if key == 27:
            self.shutdown_node()
            return

        char = chr(key)

        if char in MOVE_BINDINGS:
            self.x, self.y, self.z, self.th = MOVE_BINDINGS[char]
            self.publish_cmd()

        elif char in SPEED_BINDINGS:
            linear_scale, angular_scale = SPEED_BINDINGS[char]

            self.speed = clamp(
                self.speed * linear_scale,
                0.0,
                self.linear_speed_limit
            )

            self.turn = clamp(
                self.turn * angular_scale,
                0.0,
                self.angular_speed_limit
            )

            self.get_logger().info(
                f'Current speed: linear={self.speed:.3f}, angular={self.turn:.3f}'
            )

            self.publish_cmd()

        elif char == 'r':
            self.start_capture()

        elif char == 'p':
            self.stop_capture()

        elif char == 't':
            self.save_image()

    def draw_button(self, frame, button, text):
        x1, y1, x2, y2 = button
        cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 40, 40), -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(
            frame,
            text,
            (x1 + 12, y1 + 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

    def draw_overlay_for_preview_only(self, frame):
        self.draw_button(frame, self.button_start, 'START')
        self.draw_button(frame, self.button_stop, 'STOP')
        self.draw_button(frame, self.button_quit, 'QUIT')

        if self.capturing:
            status = f'CAPTURING every {self.capture_interval:.0f}s | count={self.image_count}'
            color = (0, 0, 255)
        else:
            status = f'READY | count={self.image_count}'
            color = (0, 255, 0)

        cv2.putText(
            frame,
            status,
            (20, self.frame_height - 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2
        )

        info = f'v={self.speed:.2f} w={self.turn:.2f} | r=START p=STOP t=SHOT ESC=QUIT'
        cv2.putText(
            frame,
            info,
            (20, self.frame_height - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1
        )

    def loop(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn('Failed to read camera frame')
            return

        clean_frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        self.latest_clean_frame = clean_frame.copy()

        now = time.time()

        if self.capturing:
            if self.last_capture_time <= 0.0:
                self.save_image()
                self.last_capture_time = now
            elif now - self.last_capture_time >= self.capture_interval:
                self.save_image()
                self.last_capture_time = now

        preview_frame = clean_frame.copy()

        if self.show_overlay:
            self.draw_overlay_for_preview_only(preview_frame)

        cv2.imshow(self.window_name, preview_frame)

        key = cv2.waitKey(1) & 0xFF
        self.handle_key(key)

        # Publish /cmd_vel continuously while robot is moving
        if self.x != 0 or self.y != 0 or self.z != 0 or self.th != 0:
            self.publish_cmd()

    def shutdown_node(self):
        self.stop_robot()

        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()

        self.get_logger().info('drive_capture_node stopped')

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = DriveCaptureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(e)
    finally:
        if node is not None:
            node.stop_robot()

            if node.cap is not None:
                node.cap.release()

            cv2.destroyAllWindows()

            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
