#!/usr/bin/env python3

import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class RealCameraPublisherNode(Node):
    def __init__(self):
        super().__init__('real_camera_publisher_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('show_preview', False)

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)

        self.pub = self.create_publisher(Image, self.image_topic, 10)

        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open camera /dev/video{self.camera_id}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.timer = self.create_timer(1.0 / self.fps, self.loop)

        self.get_logger().info(f'Real camera publisher started: /dev/video{self.camera_id}')
        self.get_logger().info(f'Publish image: {self.image_topic}')
        self.get_logger().info(f'Size: {self.frame_width}x{self.frame_height}, fps={self.fps}')

    def cv2_to_imgmsg(self, frame):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read /dev/video camera frame')
            return

        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        self.pub.publish(self.cv2_to_imgmsg(frame))

        if self.show_preview:
            cv2.imshow('real_camera_publisher_node', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RealCameraPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
