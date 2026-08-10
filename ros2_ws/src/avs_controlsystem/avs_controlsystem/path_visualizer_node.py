#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathVisualizerNode(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')

        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('path_topic', '/simplerobot_path')
        self.declare_parameter('heading_marker_topic', '/simplerobot_heading')

        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('max_path_length', 5000)
        self.declare_parameter('min_distance_step', 0.02)

        self.declare_parameter('front_arrow_offset', 0.20)
        self.declare_parameter('arrow_length', 0.45)
        self.declare_parameter('arrow_height', 0.18)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.heading_marker_topic = str(self.get_parameter('heading_marker_topic').value)

        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.max_path_length = int(self.get_parameter('max_path_length').value)
        self.min_distance_step = float(self.get_parameter('min_distance_step').value)

        self.front_arrow_offset = float(self.get_parameter('front_arrow_offset').value)
        self.arrow_length = float(self.get_parameter('arrow_length').value)
        self.arrow_height = float(self.get_parameter('arrow_height').value)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.fixed_frame

        self.last_x = None
        self.last_y = None

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.heading_marker_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        self.get_logger().info('Clean path visualizer started')
        self.get_logger().info(f'Odom:   {self.odom_topic}')
        self.get_logger().info(f'Path:   {self.path_topic}')
        self.get_logger().info(f'Arrow:  {self.heading_marker_topic}')
        self.get_logger().info('RViz will show path plus ONE current heading arrow only.')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.publish_tf(msg)
        self.publish_current_heading_arrow(msg)

        if self.last_x is not None and self.last_y is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_distance_step:
                return

        self.last_x = x
        self.last_y = y

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.fixed_frame
        pose.pose = msg.pose.pose

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.header.frame_id = self.fixed_frame
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > self.max_path_length:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_length:]

        self.path_pub.publish(self.path_msg)

    def publish_tf(self, odom_msg):
        if not self.publish_tf:
            return

        now = self.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.fixed_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = odom_msg.pose.pose.position.x
        tf.transform.translation.y = odom_msg.pose.pose.position.y
        tf.transform.translation.z = odom_msg.pose.pose.position.z
        tf.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)

    def publish_current_heading_arrow(self, odom_msg):
        now = self.get_clock().now().to_msg()

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q)

        start_x = x + self.front_arrow_offset * math.cos(yaw)
        start_y = y + self.front_arrow_offset * math.sin(yaw)

        end_x = start_x + self.arrow_length * math.cos(yaw)
        end_y = start_y + self.arrow_length * math.sin(yaw)

        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.fixed_frame

        # Cùng namespace + id = RViz chỉ update 1 mũi tên, không tạo mũi tên cũ.
        marker.ns = 'simplerobot_current_heading'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        p_start = Point()
        p_start.x = start_x
        p_start.y = start_y
        p_start.z = self.arrow_height

        p_end = Point()
        p_end.x = end_x
        p_end.y = end_y
        p_end.z = self.arrow_height

        marker.points = [p_start, p_end]

        marker.scale.x = 0.05  # shaft diameter
        marker.scale.y = 0.12  # head diameter
        marker.scale.z = 0.18  # head length

        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
