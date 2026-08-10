#!/usr/bin/env python3

import json
import math
import signal
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(value, low, high):
    return max(low, min(high, value))


class LaneFollowControllerNode(Node):
    def __init__(self):
        super().__init__('lane_follow_controller_node')

        self.declare_parameter('lane_target_topic', '/avs/lane_target')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_state_topic', '/avs/control_state')
        self.declare_parameter('control_log_topic', '/avs/control_log')
        self.declare_parameter('scan_topic', '/scan')

        self.declare_parameter('enable_cmd', False)
        self.declare_parameter('controller_mode', 'pid')  # pid | pd_backstepping
        self.declare_parameter('control_hz', 20.0)

        self.declare_parameter('v_max', 0.35)
        self.declare_parameter('v_min', 0.08)
        self.declare_parameter('w_max', 1.50)

        # PID lateral + heading.
        self.declare_parameter('kp', 1.60)
        self.declare_parameter('ki', 0.00)
        self.declare_parameter('kd', 0.18)
        self.declare_parameter('k_heading', 0.90)

        # PD Backstepping simplified for lane target.
        self.declare_parameter('k_y', 1.80)
        self.declare_parameter('k_theta', 1.20)
        self.declare_parameter('k_dy', 0.20)

        self.declare_parameter('integral_limit', 0.50)
        self.declare_parameter('target_timeout_s', 0.50)

        # Search mode.
        self.declare_parameter('search_angular', 0.35)
        self.declare_parameter('search_linear', 0.00)

        # Stop-line logic.
        self.declare_parameter('stop_line_hold_s', 2.0)

        # Safety.
        self.declare_parameter('use_lidar_safety', True)
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('obstacle_stop_distance', 0.28)
        self.declare_parameter('obstacle_slow_distance', 0.60)

        self.lane_target_topic = str(self.get_parameter('lane_target_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.control_state_topic = str(self.get_parameter('control_state_topic').value)
        self.control_log_topic = str(self.get_parameter('control_log_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)

        self.enable_cmd = bool(self.get_parameter('enable_cmd').value)
        self.controller_mode = str(self.get_parameter('controller_mode').value)

        self.control_hz = float(self.get_parameter('control_hz').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.k_heading = float(self.get_parameter('k_heading').value)

        self.k_y = float(self.get_parameter('k_y').value)
        self.k_theta = float(self.get_parameter('k_theta').value)
        self.k_dy = float(self.get_parameter('k_dy').value)

        self.integral_limit = float(self.get_parameter('integral_limit').value)
        self.target_timeout_s = float(self.get_parameter('target_timeout_s').value)

        self.search_angular = float(self.get_parameter('search_angular').value)
        self.search_linear = float(self.get_parameter('search_linear').value)

        self.stop_line_hold_s = float(self.get_parameter('stop_line_hold_s').value)

        self.use_lidar_safety = bool(self.get_parameter('use_lidar_safety').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.obstacle_stop_distance = float(self.get_parameter('obstacle_stop_distance').value)
        self.obstacle_slow_distance = float(self.get_parameter('obstacle_slow_distance').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, self.control_state_topic, 10)
        self.log_pub = self.create_publisher(String, self.control_log_topic, 10)

        self.target_sub = self.create_subscription(
            String,
            self.lane_target_topic,
            self.target_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.last_target = None
        self.last_target_time = 0.0
        self.last_error = 0.0
        self.error_integral = 0.0
        self.last_control_time = time.time()

        self.last_seen_side = 1.0
        self.stop_until = 0.0

        self.front_obstacle_dist = 9.9

        self.mode = 'IDLE'

        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.get_logger().info('lane_follow_controller_node started')
        self.get_logger().info(f'Subscribe target: {self.lane_target_topic}')
        self.get_logger().info(f'Publish cmd_vel:  {self.cmd_vel_topic}')
        self.get_logger().warn(f'enable_cmd = {self.enable_cmd}. Set true only when testing on real robot.')

    def signal_handler(self, signum, frame):
        self.get_logger().warn(f'Received signal {signum}, stopping robot.')
        self.safe_stop()
        if rclpy.ok():
            rclpy.shutdown()

    def publish_log(self, level, message, extra=None):
        payload = {
            'time': time.time(),
            'node': 'lane_follow_controller_node',
            'level': level,
            'message': message,
            'extra': extra or {},
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.log_pub.publish(msg)

    def make_twist(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        return msg

    def publish_cmd(self, v, w):
        if not self.enable_cmd:
            return

        self.cmd_pub.publish(self.make_twist(v, w))

    def safe_stop(self):
        stop = self.make_twist(0.0, 0.0)
        for _ in range(30):
            self.cmd_pub.publish(stop)
            time.sleep(0.02)

    def target_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid lane target JSON: {e}')
            return

        self.last_target = data
        self.last_target_time = time.time()

        e = float(data.get('lateral_error_m', 0.0))
        if abs(e) > 0.02:
            self.last_seen_side = 1.0 if e > 0.0 else -1.0

    def scan_callback(self, msg):
        half = math.radians(self.front_angle_deg / 2.0)
        angle = msg.angle_min
        values = []

        for r in msg.ranges:
            if -half <= angle <= half:
                if math.isfinite(r) and msg.range_min < r < msg.range_max:
                    values.append(float(r))
            angle += msg.angle_increment

        if values:
            values = sorted(values)
            idx = max(0, int(len(values) * 0.15))
            self.front_obstacle_dist = values[idx]
        else:
            self.front_obstacle_dist = 9.9

    def compute_speed_scale(self, abs_error, abs_heading):
        scale = 1.0

        if abs_error > 0.25:
            scale *= 0.45
        elif abs_error > 0.15:
            scale *= 0.65
        elif abs_error > 0.08:
            scale *= 0.85

        if abs_heading > 0.45:
            scale *= 0.55
        elif abs_heading > 0.25:
            scale *= 0.75

        return clamp(scale, 0.20, 1.0)

    def apply_lidar_safety(self, v):
        if not self.use_lidar_safety:
            return v, False, False

        if self.front_obstacle_dist <= self.obstacle_stop_distance:
            return 0.0, True, False

        if self.front_obstacle_dist <= self.obstacle_slow_distance:
            ratio = (
                (self.front_obstacle_dist - self.obstacle_stop_distance) /
                max(self.obstacle_slow_distance - self.obstacle_stop_distance, 1e-6)
            )
            ratio = clamp(ratio, 0.20, 1.0)
            return v * ratio, False, True

        return v, False, False

    def compute_pid_cmd(self, target, dt):
        e = float(target.get('lateral_error_m', 0.0))
        heading = float(target.get('heading_error_rad', 0.0))

        de = (e - self.last_error) / max(dt, 1e-6)

        self.error_integral += e * dt
        self.error_integral = clamp(
            self.error_integral,
            -self.integral_limit,
            self.integral_limit
        )

        # e > 0 nghĩa là làn nằm bên phải, xe cần quay phải => angular.z âm.
        w = -(
            self.kp * e +
            self.ki * self.error_integral +
            self.kd * de +
            self.k_heading * heading
        )

        w = clamp(w, -self.w_max, self.w_max)

        scale = self.compute_speed_scale(abs(e), abs(heading))
        v = self.v_min + scale * (self.v_max - self.v_min)

        self.last_error = e

        return v, w, e, heading

    def compute_backstepping_cmd(self, target, dt):
        x_t = float(target.get('target_x_m', 0.0))
        y_t = max(float(target.get('target_y_m', 0.50)), 0.05)
        heading = float(target.get('heading_error_rad', 0.0))

        e = x_t
        de = (e - self.last_error) / max(dt, 1e-6)

        # Góc tới điểm nhìn trước. Dương nghĩa target ở bên phải.
        alpha_right = math.atan2(x_t, y_t)

        # ROS angular.z dương là trái, nên đổi dấu.
        w = -(
            self.k_y * x_t +
            self.k_theta * alpha_right +
            self.k_dy * de +
            0.50 * heading
        )

        w = clamp(w, -self.w_max, self.w_max)

        scale = self.compute_speed_scale(abs(x_t), abs(alpha_right))
        v = self.v_min + scale * (self.v_max - self.v_min)

        self.last_error = e

        return v, w, e, alpha_right

    def publish_state(self, mode, v, w, target=None, reason=''):
        payload = {
            'time': time.time(),
            'mode': mode,
            'controller_mode': self.controller_mode,
            'enable_cmd': self.enable_cmd,
            'cmd_linear_x': float(v),
            'cmd_angular_z': float(w),
            'front_obstacle_dist': float(self.front_obstacle_dist),
            'reason': reason,
            'target': target or {},
            'gains': {
                'kp': self.kp,
                'ki': self.ki,
                'kd': self.kd,
                'k_heading': self.k_heading,
                'k_y': self.k_y,
                'k_theta': self.k_theta,
                'k_dy': self.k_dy,
            }
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def control_loop(self):
        now = time.time()
        dt = now - self.last_control_time
        self.last_control_time = now

        if self.last_target is None or now - self.last_target_time > self.target_timeout_s:
            self.mode = 'SEARCH_LANE'

            # Mất main-lane thì quay nhẹ theo hướng lần cuối thấy lane.
            # last_seen_side > 0 nghĩa lane ở phải, cần quay phải => w âm.
            w = -self.last_seen_side * abs(self.search_angular)
            v = self.search_linear

            self.publish_cmd(v, w)
            self.publish_state(self.mode, v, w, reason='target_timeout')
            return

        target = self.last_target
        valid = bool(target.get('valid', False))

        if not valid:
            self.mode = 'SEARCH_LANE'
            w = -self.last_seen_side * abs(self.search_angular)
            v = self.search_linear

            self.publish_cmd(v, w)
            self.publish_state(self.mode, v, w, target, reason='invalid_target')
            return

        if bool(target.get('stop_line_close', False)):
            self.stop_until = max(self.stop_until, now + self.stop_line_hold_s)

        if now < self.stop_until:
            self.mode = 'STOP_LINE'
            v = 0.0
            w = 0.0

            self.publish_cmd(v, w)
            self.publish_state(self.mode, v, w, target, reason='stop_line_hold')
            return

        if self.controller_mode == 'pd_backstepping':
            v, w, e, heading = self.compute_backstepping_cmd(target, dt)
            self.mode = 'LANE_FOLLOW_BACKSTEPPING'
        else:
            v, w, e, heading = self.compute_pid_cmd(target, dt)
            self.mode = 'LANE_FOLLOW_PID'

        # Logic line vàng: giảm tốc khi thấy vạch vàng và sai số lớn.
        if bool(target.get('solid_yellow_visible', False)) and abs(e) > 0.10:
            v *= 0.70

        # Logic other lane: nếu thấy other-lane và main-lane vẫn thấy thì vẫn đi main-lane, giảm tốc nhẹ.
        if bool(target.get('other_lane_visible', False)) and abs(e) > 0.18:
            v *= 0.80

        # Vehicle class: giảm tốc. Dừng thật nên ưu tiên LiDAR.
        if bool(target.get('vehicle_visible', False)):
            v *= 0.60

        v, obstacle_stop, obstacle_slow = self.apply_lidar_safety(v)

        if obstacle_stop:
            self.mode = 'OBSTACLE_STOP'
            w = 0.0
        elif obstacle_slow:
            self.mode = self.mode + '_OBSTACLE_SLOW'

        self.publish_cmd(v, w)
        self.publish_state(self.mode, v, w, target)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
