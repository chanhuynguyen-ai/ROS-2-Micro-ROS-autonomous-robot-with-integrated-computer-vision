#!/usr/bin/env python3

import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class LineCameraTargetNode(Node):
    def __init__(self):
        super().__init__('line_camera_target_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 20.0)

        self.declare_parameter('lane_target_topic', '/lane_target')
        self.declare_parameter('lane_marker_topic', '/simplerobot_lane_target_marker')

        self.declare_parameter('show_preview', True)
        self.declare_parameter('roi_top_ratio', 0.45)

        self.declare_parameter('min_mask_area', 500)
        self.declare_parameter('near_y_ratio', 0.85)
        self.declare_parameter('far_y_ratio', 0.58)
        self.declare_parameter('band_height', 35)

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('marker_forward_m', 0.55)
        self.declare_parameter('marker_lateral_scale_m', 0.35)

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.fps = float(self.get_parameter('fps').value)

        self.lane_target_topic = str(self.get_parameter('lane_target_topic').value)
        self.lane_marker_topic = str(self.get_parameter('lane_marker_topic').value)

        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)

        self.min_mask_area = int(self.get_parameter('min_mask_area').value)
        self.near_y_ratio = float(self.get_parameter('near_y_ratio').value)
        self.far_y_ratio = float(self.get_parameter('far_y_ratio').value)
        self.band_height = int(self.get_parameter('band_height').value)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.marker_forward_m = float(self.get_parameter('marker_forward_m').value)
        self.marker_lateral_scale_m = float(self.get_parameter('marker_lateral_scale_m').value)

        self.cap = cv2.VideoCapture(self.camera_id)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera id {self.camera_id}')
            raise RuntimeError(f'Cannot open camera id {self.camera_id}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.target_pub = self.create_publisher(Point, self.lane_target_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.lane_marker_topic, 10)

        self.window_name = 'SimpleRobot Lane Camera'
        if self.show_preview:
            cv2.namedWindow(self.window_name)

        self.timer = self.create_timer(1.0 / self.fps, self.loop)

        self.get_logger().info('Line camera target node started')
        self.get_logger().info(f'Publish lane target: {self.lane_target_topic}')
        self.get_logger().info('Target format: Point.x=e_y, Point.y=e_heading, Point.z=valid')

    def threshold_line(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White line mask
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 80, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Yellow line mask
        lower_yellow = np.array([15, 70, 80])
        upper_yellow = np.array([40, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask = cv2.bitwise_or(mask_white, mask_yellow)

        roi_top = int(self.frame_height * self.roi_top_ratio)
        mask[:roi_top, :] = 0

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def band_center_x(self, mask, y_center):
        y1 = max(0, int(y_center - self.band_height // 2))
        y2 = min(mask.shape[0], int(y_center + self.band_height // 2))

        band = mask[y1:y2, :]
        ys, xs = np.where(band > 0)

        if len(xs) < 20:
            return None

        return float(np.mean(xs))

    def publish_lane_marker(self, e_y, valid):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.base_frame
        marker.ns = 'simplerobot_lane_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # ROS base_link convention: x forward, y left.
        # e_y positive means line is on right image side, so marker y is negative.
        marker.pose.position.x = self.marker_forward_m
        marker.pose.position.y = -e_y * self.marker_lateral_scale_m
        marker.pose.position.z = 0.10

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        if valid > 0.5:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6

        self.marker_pub.publish(marker)

    def loop(self):
        ret, frame = self.cap.read()

        out = Point()
        out.z = 0.0

        if not ret:
            self.get_logger().warn('Failed to read camera frame')
            self.target_pub.publish(out)
            self.publish_lane_marker(0.0, 0.0)
            return

        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        mask = self.threshold_line(frame)

        area = int(cv2.countNonZero(mask))

        near_y = int(self.frame_height * self.near_y_ratio)
        far_y = int(self.frame_height * self.far_y_ratio)

        near_x = self.band_center_x(mask, near_y)
        far_x = self.band_center_x(mask, far_y)

        valid = 0.0

        if area >= self.min_mask_area and near_x is not None:
            if far_x is None:
                far_x = near_x

            center_x = self.frame_width / 2.0

            e_y = (near_x - center_x) / center_x

            # Heading error from near point to far point.
            # Positive means the line goes to the right in image.
            dy = max(float(near_y - far_y), 1.0)
            e_heading = math.atan2(far_x - near_x, dy)

            out.x = float(e_y)
            out.y = float(e_heading)
            out.z = 1.0
            valid = 1.0

            self.publish_lane_marker(e_y, valid)

            if self.show_preview:
                cv2.circle(frame, (int(near_x), near_y), 8, (0, 255, 0), -1)
                cv2.circle(frame, (int(far_x), far_y), 8, (255, 0, 0), -1)
                cv2.line(frame, (int(near_x), near_y), (int(far_x), far_y), (0, 255, 255), 3)
                cv2.line(frame, (int(center_x), self.frame_height), (int(center_x), 0), (255, 255, 255), 1)
                cv2.putText(frame, f'e_y={e_y:.3f} e_h={e_heading:.3f}',
                            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            self.publish_lane_marker(0.0, 0.0)
            if self.show_preview:
                cv2.putText(frame, 'LANE LOST', (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self.target_pub.publish(out)

        if self.show_preview:
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            preview = np.hstack((frame, mask_bgr))
            cv2.imshow(self.window_name, preview)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                rclpy.shutdown()

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LineCameraTargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(e)
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
