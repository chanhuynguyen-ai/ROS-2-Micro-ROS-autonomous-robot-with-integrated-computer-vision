
import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def rate_limit(target: float, current: float, max_delta: float) -> float:
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


class SmoothLaneLidarFollowerNode(Node):
    '''
    Smooth lane follower for 4-wheel differential / skid-steer robot.

    Input:
      /lane_target geometry_msgs/Point
        x = lateral error, normalized or meter depending on parser
        y = heading error rad
        z = valid flag, >0.5 means lane detected

      /scan sensor_msgs/LaserScan

    Output:
      /cmd_vel geometry_msgs/Twist
        linear.x  = forward speed
        angular.z = yaw rate

      /avs/control_state std_msgs/String JSON debug
    '''

    def __init__(self):
        super().__init__('smooth_lane_lidar_follower_node')

        # Topics
        self.declare_parameter('lane_target_topic', '/lane_target')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_state_topic', '/avs/control_state')

        # Safety
        self.declare_parameter('enable_motion', True)
        self.declare_parameter('lane_lost_timeout', 1.2)
        self.declare_parameter('stop_on_lane_lost', True)

        # Speed limits
        self.declare_parameter('v_max', 0.10)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('omega_max', 0.30)

        # Controller gains
        self.declare_parameter('kp_y', 0.35)
        self.declare_parameter('kp_heading', 0.20)
        self.declare_parameter('kd_y', 0.08)
        self.declare_parameter('kd_heading', 0.04)

        # Smoothing
        self.declare_parameter('error_filter_alpha', 0.18)
        self.declare_parameter('v_rate_limit', 0.12)        # m/s^2
        self.declare_parameter('omega_rate_limit', 0.55)    # rad/s^2
        self.declare_parameter('control_rate_hz', 20.0)

        # Deadband
        self.declare_parameter('e_y_deadband', 0.025)
        self.declare_parameter('e_heading_deadband', 0.015)
        self.declare_parameter('omega_deadband', 0.015)

        # Direction sign
        self.declare_parameter('invert_angular', False)

        # Slow down when error is large
        self.declare_parameter('slow_error_gain', 1.2)

        # Anti-pivot differential mixing protection
        # Firmware roughly maps:
        #   left_speed  = v - omega * wheel_mix_factor
        #   right_speed = v + omega * wheel_mix_factor
        # To avoid inner wheels stopping:
        #   |omega| <= v * (1 - inner_min_fraction) / wheel_mix_factor
        self.declare_parameter('wheel_mix_factor', 0.18)
        self.declare_parameter('inner_min_fraction', 0.35)

        # LiDAR safety
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('emergency_distance', 0.18)
        self.declare_parameter('stop_distance', 0.32)
        self.declare_parameter('slow_distance', 0.70)

        lane_target_topic = self.get_parameter('lane_target_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        control_state_topic = self.get_parameter('control_state_topic').value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, control_state_topic, 10)

        self.lane_sub = self.create_subscription(
            Point,
            lane_target_topic,
            self.lane_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.latest_lane = Point()
        self.lane_valid = False
        self.last_lane_time = None

        self.front_min: Optional[float] = None

        self.e_y_f = 0.0
        self.e_h_f = 0.0
        self.prev_e_y_f = 0.0
        self.prev_e_h_f = 0.0

        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        self.last_control_time = None
        self.mode = 'init'

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        period = 1.0 / max(1.0, control_rate_hz)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('smooth_lane_lidar_follower_node started')
        self.get_logger().info(f'sub lane: {lane_target_topic}, scan: {scan_topic}, pub cmd_vel: {cmd_vel_topic}')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def lane_callback(self, msg: Point):
        self.latest_lane = msg
        self.lane_valid = msg.z > 0.5
        self.last_lane_time = self.now_sec()

    def scan_callback(self, msg: LaserScan):
        front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        values = []

        angle = msg.angle_min
        for r in msg.ranges:
            if abs(angle) <= front_angle:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    values.append(float(r))
            angle += msg.angle_increment

        self.front_min = min(values) if values else None

    def publish_zero(self, mode: str):
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.mode = mode
        self.publish_state(0.0, 0.0, 0.0, 0.0)

    def publish_state(self, e_y: float, e_heading: float, v: float, omega: float):
        wheel_mix_factor = float(self.get_parameter('wheel_mix_factor').value)
        inner_speed_est = max(0.0, abs(v) - abs(omega) * wheel_mix_factor)

        state = {
            'enabled': bool(self.get_parameter('enable_motion').value),
            'mode': self.mode,
            'lane_valid': bool(self.lane_valid),
            'e_y_raw': float(self.latest_lane.x),
            'e_heading_raw': float(self.latest_lane.y),
            'e_y_filtered': float(e_y),
            'e_heading_filtered': float(e_heading),
            'v_cmd': float(v),
            'omega_cmd': float(omega),
            'front_min_m': None if self.front_min is None else float(self.front_min),
            'wheel_mix_factor': float(wheel_mix_factor),
            'inner_speed_est': float(inner_speed_est),
        }

        msg = String()
        msg.data = json.dumps(state, separators=(',', ':'))
        self.state_pub.publish(msg)

    def control_loop(self):
        now = self.now_sec()

        if self.last_control_time is None:
            self.last_control_time = now
            return

        dt = max(1e-3, now - self.last_control_time)
        self.last_control_time = now

        enable_motion = bool(self.get_parameter('enable_motion').value)
        lane_lost_timeout = float(self.get_parameter('lane_lost_timeout').value)
        stop_on_lane_lost = bool(self.get_parameter('stop_on_lane_lost').value)

        if not enable_motion:
            self.publish_zero('disabled')
            return

        if self.last_lane_time is None or (now - self.last_lane_time) > lane_lost_timeout or not self.lane_valid:
            if stop_on_lane_lost:
                self.publish_zero('lane_lost_stop')
                return

        # LiDAR safety
        emergency_distance = float(self.get_parameter('emergency_distance').value)
        stop_distance = float(self.get_parameter('stop_distance').value)
        slow_distance = float(self.get_parameter('slow_distance').value)

        obstacle_factor = 1.0

        if self.front_min is not None:
            if self.front_min <= emergency_distance:
                self.publish_zero('emergency_stop')
                return
            if self.front_min <= stop_distance:
                self.publish_zero('obstacle_stop')
                return
            if self.front_min < slow_distance:
                obstacle_factor = clamp(
                    (self.front_min - stop_distance) / max(1e-3, slow_distance - stop_distance),
                    0.25,
                    1.0
                )

        # Read parameters
        v_max = float(self.get_parameter('v_max').value)
        v_min = float(self.get_parameter('v_min').value)
        omega_max = float(self.get_parameter('omega_max').value)

        kp_y = float(self.get_parameter('kp_y').value)
        kp_heading = float(self.get_parameter('kp_heading').value)
        kd_y = float(self.get_parameter('kd_y').value)
        kd_heading = float(self.get_parameter('kd_heading').value)

        error_filter_alpha = float(self.get_parameter('error_filter_alpha').value)
        error_filter_alpha = clamp(error_filter_alpha, 0.01, 1.0)

        e_y_deadband = float(self.get_parameter('e_y_deadband').value)
        e_heading_deadband = float(self.get_parameter('e_heading_deadband').value)
        omega_deadband = float(self.get_parameter('omega_deadband').value)

        invert_angular = bool(self.get_parameter('invert_angular').value)
        slow_error_gain = float(self.get_parameter('slow_error_gain').value)

        v_rate_limit = float(self.get_parameter('v_rate_limit').value)
        omega_rate_limit = float(self.get_parameter('omega_rate_limit').value)

        wheel_mix_factor = float(self.get_parameter('wheel_mix_factor').value)
        inner_min_fraction = float(self.get_parameter('inner_min_fraction').value)
        wheel_mix_factor = max(1e-3, wheel_mix_factor)
        inner_min_fraction = clamp(inner_min_fraction, 0.0, 0.85)

        # Raw error from lane_target
        e_y = float(self.latest_lane.x)
        e_heading = float(self.latest_lane.y)

        if abs(e_y) < e_y_deadband:
            e_y = 0.0
        if abs(e_heading) < e_heading_deadband:
            e_heading = 0.0

        # Low-pass filter errors
        self.e_y_f = error_filter_alpha * e_y + (1.0 - error_filter_alpha) * self.e_y_f
        self.e_h_f = error_filter_alpha * e_heading + (1.0 - error_filter_alpha) * self.e_h_f

        de_y = (self.e_y_f - self.prev_e_y_f) / dt
        de_h = (self.e_h_f - self.prev_e_h_f) / dt

        self.prev_e_y_f = self.e_y_f
        self.prev_e_h_f = self.e_h_f

        # Lane controller
        # Convention:
        #   e_y > 0 means lane target is on the right side of image.
        #   ROS angular.z < 0 turns right.
        omega_target = -(
            kp_y * self.e_y_f
            + kp_heading * self.e_h_f
            + kd_y * de_y
            + kd_heading * de_h
        )

        if invert_angular:
            omega_target = -omega_target

        if abs(omega_target) < omega_deadband:
            omega_target = 0.0

        # Smooth speed profile
        abs_error = abs(self.e_y_f) + 0.6 * abs(self.e_h_f)
        v_target = v_max / (1.0 + slow_error_gain * abs_error)
        v_target = clamp(v_target, v_min, v_max)
        v_target *= obstacle_factor

        # Anti-pivot limit:
        # keep inner side moving, not stopping.
        omega_no_pivot_limit = abs(v_target) * (1.0 - inner_min_fraction) / wheel_mix_factor
        omega_limit = min(omega_max, max(0.03, omega_no_pivot_limit))

        omega_target = clamp(omega_target, -omega_limit, omega_limit)

        # Rate limit command to reduce left-right jerking
        self.v_cmd = rate_limit(v_target, self.v_cmd, v_rate_limit * dt)
        self.omega_cmd = rate_limit(omega_target, self.omega_cmd, omega_rate_limit * dt)

        cmd = Twist()
        cmd.linear.x = float(self.v_cmd)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(self.omega_cmd)

        self.cmd_pub.publish(cmd)

        self.mode = 'tracking'
        self.publish_state(self.e_y_f, self.e_h_f, self.v_cmd, self.omega_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SmoothLaneLidarFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
