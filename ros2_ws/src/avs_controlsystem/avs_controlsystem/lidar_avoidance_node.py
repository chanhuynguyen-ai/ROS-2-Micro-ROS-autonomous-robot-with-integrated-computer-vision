#!/usr/bin/env python3

import math
import time
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(value, low, high):
    return max(low, min(high, value))


class LidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_node')

        self.declare_parameter('linear_max', 0.5)
        self.declare_parameter('linear_min', 0.04)
        self.declare_parameter('angular_max', 0.75)

        self.declare_parameter('emergency_distance', 0.18)
        self.declare_parameter('stop_distance', 0.32)
        self.declare_parameter('slow_distance', 0.7)

        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('side_min_angle_deg', 35.0)
        self.declare_parameter('side_max_angle_deg', 110.0)

        self.declare_parameter('k_avoid', 0.45)
        self.declare_parameter('k_center', 0.25)
        self.declare_parameter('invert_angular', False)

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.linear_max = float(self.get_parameter('linear_max').value)
        self.linear_min = float(self.get_parameter('linear_min').value)
        self.angular_max = float(self.get_parameter('angular_max').value)

        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.side_min_angle_deg = float(self.get_parameter('side_min_angle_deg').value)
        self.side_max_angle_deg = float(self.get_parameter('side_max_angle_deg').value)

        self.k_avoid = float(self.get_parameter('k_avoid').value)
        self.k_center = float(self.get_parameter('k_center').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.latest_scan = None
        self.scan_received = False
        self.shutting_down = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        self.get_logger().info('LiDAR avoidance node started')
        self.get_logger().info(f'Subscribe: {self.scan_topic}')
        self.get_logger().info(f'Publish:   {self.cmd_vel_topic}')
        self.get_logger().info('Ctrl+C/SIGTERM will publish repeated zero Twist.')

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f'Received signal {signum}. Stopping robot now...')
        self.safe_stop_robot()
        self.shutting_down = True
        if rclpy.ok():
            rclpy.shutdown()

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received = True

    def get_sector_values(self, scan, min_deg, max_deg):
        values = []
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)

        angle = scan.angle_min

        for r in scan.ranges:
            if min_rad <= angle <= max_rad:
                if math.isfinite(r) and scan.range_min < r < scan.range_max:
                    values.append(float(r))
            angle += scan.angle_increment

        return values

    def robust_min(self, values, default=9.9):
        if not values:
            return default

        values = sorted(values)
        idx = max(0, int(len(values) * 0.15))
        return values[idx]

    def make_cmd(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        return cmd

    def safe_stop_robot(self):
        stop = Twist()

        # Gửi nhiều lần vì một số firmware cần nhận liên tục mới dừng chắc.
        for _ in range(30):
            self.cmd_pub.publish(stop)
            time.sleep(0.03)

        self.get_logger().warn('STOP command published repeatedly.')

    def control_loop(self):
        if self.shutting_down:
            self.safe_stop_robot()
            return

        if not self.scan_received or self.latest_scan is None:
            self.cmd_pub.publish(Twist())
            return

        scan = self.latest_scan

        half_front = self.front_angle_deg / 2.0

        front_values = self.get_sector_values(scan, -half_front, half_front)
        left_values = self.get_sector_values(scan, self.side_min_angle_deg, self.side_max_angle_deg)
        right_values = self.get_sector_values(scan, -self.side_max_angle_deg, -self.side_min_angle_deg)

        front_dist = self.robust_min(front_values)
        left_dist = self.robust_min(left_values)
        right_dist = self.robust_min(right_values)

        right_inv = 1.0 / max(right_dist, 0.05)
        left_inv = 1.0 / max(left_dist, 0.05)

        avoid_turn = self.k_avoid * (right_inv - left_inv)
        center_turn = self.k_center * (left_dist - right_dist)

        angular = avoid_turn + center_turn
        angular = clamp(angular, -self.angular_max, self.angular_max)

        if self.invert_angular:
            angular = -angular

        if front_dist < self.emergency_distance:
            linear = 0.0

            if left_dist >= right_dist:
                angular = abs(self.angular_max)
            else:
                angular = -abs(self.angular_max)

            if self.invert_angular:
                angular = -angular

            self.cmd_pub.publish(self.make_cmd(linear, angular))
            return

        if front_dist < self.stop_distance:
            linear = 0.0

            if left_dist >= right_dist:
                angular = abs(self.angular_max * 0.75)
            else:
                angular = -abs(self.angular_max * 0.75)

            if self.invert_angular:
                angular = -angular

            self.cmd_pub.publish(self.make_cmd(linear, angular))
            return

        if front_dist < self.slow_distance:
            ratio = (front_dist - self.stop_distance) / max(
                self.slow_distance - self.stop_distance,
                1e-6
            )
            ratio = clamp(ratio, 0.0, 1.0)
            linear = self.linear_min + ratio * (self.linear_max - self.linear_min)
        else:
            linear = self.linear_max

        if abs(angular) > 0.7 * self.angular_max:
            linear *= 0.6

        self.cmd_pub.publish(self.make_cmd(linear, angular))


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
