#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


class CmdVelOdomNode(Node):
    def __init__(self):
        super().__init__('cmdvel_odom_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rate_hz', 30.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_time = time.time()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer(1.0 / self.rate_hz, self.update_odom)

        self.get_logger().info('cmdvel_odom_node started')
        self.get_logger().info(f'Subscribe: {self.cmd_vel_topic}')
        self.get_logger().info(f'Publish:   {self.odom_topic}')
        self.get_logger().info('This is open-loop odometry from /cmd_vel, not encoder odometry.')

    def cmd_callback(self, msg):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)

    def update_odom(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Body velocity -> odom frame
        dx = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        dy = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        dtheta = self.wz * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0

            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
