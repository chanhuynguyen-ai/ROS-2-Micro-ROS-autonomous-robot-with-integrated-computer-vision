#!/usr/bin/env python3

import math
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker


def clamp(value, low, high):
    return max(low, min(high, value))


class LaneLidarFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_lidar_follower_node')

        self.declare_parameter('lane_target_topic', '/lane_target')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('obstacle_marker_topic', '/simplerobot_obstacle_points')

        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('v_max', 0.12)
        self.declare_parameter('v_min', 0.035)
        self.declare_parameter('omega_max', 0.75)

        self.declare_parameter('kp_y', 1.15)
        self.declare_parameter('kp_heading', 1.55)
        self.declare_parameter('kd_heading', 0.06)

        self.declare_parameter('lane_lost_timeout', 0.7)
        self.declare_parameter('search_omega', 0.25)

        self.declare_parameter('emergency_distance', 0.18)
        self.declare_parameter('stop_distance', 0.32)
        self.declare_parameter('slow_distance', 0.70)
        self.declare_parameter('side_alert_distance', 0.45)

        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('side_min_angle_deg', 35.0)
        self.declare_parameter('side_max_angle_deg', 110.0)

        self.declare_parameter('avoid_gain', 0.24)
        self.declare_parameter('invert_angular', False)

        self.declare_parameter('marker_max_range', 1.5)
        self.declare_parameter('marker_decimation', 4)

        self.lane_target_topic = str(self.get_parameter('lane_target_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.obstacle_marker_topic = str(self.get_parameter('obstacle_marker_topic').value)

        self.base_frame = str(self.get_parameter('base_frame').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.omega_max = float(self.get_parameter('omega_max').value)

        self.kp_y = float(self.get_parameter('kp_y').value)
        self.kp_heading = float(self.get_parameter('kp_heading').value)
        self.kd_heading = float(self.get_parameter('kd_heading').value)

        self.lane_lost_timeout = float(self.get_parameter('lane_lost_timeout').value)
        self.search_omega = float(self.get_parameter('search_omega').value)

        self.emergency_distance = float(self.get_parameter('emergency_distance').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.side_alert_distance = float(self.get_parameter('side_alert_distance').value)

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.side_min_angle_deg = float(self.get_parameter('side_min_angle_deg').value)
        self.side_max_angle_deg = float(self.get_parameter('side_max_angle_deg').value)

        self.avoid_gain = float(self.get_parameter('avoid_gain').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)

        self.marker_max_range = float(self.get_parameter('marker_max_range').value)
        self.marker_decimation = int(self.get_parameter('marker_decimation').value)

        self.e_y = 0.0
        self.e_heading = 0.0
        self.lane_valid = 0.0
        self.last_lane_time = 0.0

        self.prev_heading_error = 0.0
        self.prev_t = time.time()

        self.latest_scan = None
        self.scan_received = False
        self.shutting_down = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.lane_sub = self.create_subscription(
            Point,
            self.lane_target_topic,
            self.lane_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.obstacle_marker_topic, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        self.get_logger().info('Lane + LiDAR follower started')
        self.get_logger().info(f'Lane target: {self.lane_target_topic}')
        self.get_logger().info(f'Scan:        {self.scan_topic}')
        self.get_logger().info(f'Cmd vel:     {self.cmd_vel_topic}')

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f'Received signal {signum}. Stopping robot...')
        self.safe_stop_robot()
        self.shutting_down = True

        if rclpy.ok():
            rclpy.shutdown()

    def lane_callback(self, msg):
        self.e_y = float(msg.x)
        self.e_heading = float(msg.y)
        self.lane_valid = float(msg.z)

        if self.lane_valid > 0.5:
            self.last_lane_time = time.time()

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received = True
        self.publish_obstacle_marker(msg)

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
        for _ in range(35):
            self.cmd_pub.publish(stop)
            time.sleep(0.03)

        self.get_logger().warn('Repeated zero /cmd_vel sent.')

    def publish_obstacle_marker(self, scan):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.base_frame
        marker.ns = 'simplerobot_current_obstacles'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.045
        marker.scale.y = 0.045

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):
            if i % max(self.marker_decimation, 1) != 0:
                angle += scan.angle_increment
                continue

            if math.isfinite(r) and scan.range_min < r < min(scan.range_max, self.marker_max_range):
                p = Point()
                p.x = float(r * math.cos(angle))
                p.y = float(r * math.sin(angle))
                p.z = 0.08
                marker.points.append(p)

            angle += scan.angle_increment

        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)

    def control_loop(self):
        if self.shutting_down:
            self.safe_stop_robot()
            return

        now = time.time()
        dt = max(now - self.prev_t, 1e-3)
        self.prev_t = now

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

        lane_lost = self.lane_valid < 0.5 or (now - self.last_lane_time) > self.lane_lost_timeout

        # Obstacle emergency
        if front_dist < self.emergency_distance:
            if left_dist >= right_dist:
                omega = abs(self.omega_max)
            else:
                omega = -abs(self.omega_max)

            if self.invert_angular:
                omega = -omega

            self.cmd_pub.publish(self.make_cmd(0.0, omega))
            return

        # Obstacle close
        if front_dist < self.stop_distance:
            if left_dist >= right_dist:
                omega = abs(self.omega_max * 0.75)
            else:
                omega = -abs(self.omega_max * 0.75)

            if self.invert_angular:
                omega = -omega

            self.cmd_pub.publish(self.make_cmd(0.0, omega))
            return

        # Lane control
        if lane_lost:
            v = 0.0

            if left_dist >= right_dist:
                omega = self.search_omega
            else:
                omega = -self.search_omega

            if self.invert_angular:
                omega = -omega

            self.cmd_pub.publish(self.make_cmd(v, omega))
            return

        d_heading = (self.e_heading - self.prev_heading_error) / dt
        self.prev_heading_error = self.e_heading

        # e_y positive means line is on right in image.
        # ROS angular.z positive turns left, so lane correction sign is negative.
        omega_lane = -(
            self.kp_y * self.e_y
            + self.kp_heading * self.e_heading
            + self.kd_heading * d_heading
        )

        # Obstacle side avoidance
        avoid_omega = 0.0

        if left_dist < self.side_alert_distance or right_dist < self.side_alert_distance:
            right_inv = 1.0 / max(right_dist, 0.05)
            left_inv = 1.0 / max(left_dist, 0.05)
            avoid_omega = self.avoid_gain * (right_inv - left_inv)

        omega = omega_lane + avoid_omega
        omega = clamp(omega, -self.omega_max, self.omega_max)

        if self.invert_angular:
            omega = -omega

        # Speed profile
        if front_dist < self.slow_distance:
            ratio = (front_dist - self.stop_distance) / max(self.slow_distance - self.stop_distance, 1e-6)
            ratio = clamp(ratio, 0.0, 1.0)
            v = self.v_min + ratio * (self.v_max - self.v_min)
        else:
            v = self.v_max

        # Slow down when line error or turning is large
        v *= math.exp(-1.0 * abs(self.e_y))
        if abs(omega) > 0.7 * self.omega_max:
            v *= 0.65

        v = clamp(v, self.v_min, self.v_max)

        self.cmd_pub.publish(self.make_cmd(v, omega))


def main(args=None):
    rclpy.init(args=args)
    node = LaneLidarFollowerNode()

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
