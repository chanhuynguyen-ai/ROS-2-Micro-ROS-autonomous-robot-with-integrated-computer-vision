#!/usr/bin/env python3

import json
import math
import signal
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


def approach(current, target, max_delta):
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


def finite_float(value, default=None):
    try:
        v = float(value)
        if math.isfinite(v):
            return v
    except Exception:
        pass
    return default


def parse_bool(value, default=True):
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() not in ["false", "0", "no", "none", "invalid", ""]

    if value is None:
        return default

    try:
        return bool(value)
    except Exception:
        return default


def yaw_from_quat(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


class PDBackstepingCmdVelNode(Node):
    """
    v5_center_first_pd_backstepping

    Mục tiêu:
      - Bám e_x_mm và theta_rad về 0.
      - Khi xe đang giữa làn thẳng: không bẻ lái lung tung.
      - Đoạn thẳng chạy nhanh hơn, đoạn cua tự giảm tốc.
      - Chỉ tăng omega khi có bằng chứng cua ổn định nhiều frame.
      - Không rẽ theo một frame perception nhảy sai ở ngã tư.
    """

    def __init__(self):
        super().__init__("pd_backsteping_cmdvel_node")

        # Topics
        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("state_topic", "/avs/pd_backsteping_state")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom_raw")

        self.declare_parameter("control_hz", 50.0)

        # Timing
        self.declare_parameter("fresh_s", 1.8)
        self.declare_parameter("blind_hold_s", 3.0)
        self.declare_parameter("lost_stop_s", 6.0)

        # Startup guard: khi vừa đặt xe xuống làn, đi thẳng ngắn hạn để perception ổn định.
        self.declare_parameter("startup_straight_s", 1.0)
        self.declare_parameter("startup_v", 0.045)

        # FPS adaptive speed
        self.declare_parameter("fps_init", 2.0)
        self.declare_parameter("fps_min", 0.8)
        self.declare_parameter("fps_max", 20.0)
        self.declare_parameter("fps_tau_s", 1.0)
        self.declare_parameter("target_distance_per_frame_m", 0.033)

        # Speed limits
        self.declare_parameter("v_max", 0.115)
        self.declare_parameter("v_min", 0.050)
        self.declare_parameter("v_slow", 0.040)
        self.declare_parameter("v_blind", 0.048)
        self.declare_parameter("v_curve_max", 0.052)
        self.declare_parameter("v_recover_max", 0.070)

        # Steering limits by zone
        self.declare_parameter("omega_abs_max", 0.58)
        self.declare_parameter("omega_center_max", 0.000)
        self.declare_parameter("omega_near_max", 0.025)
        self.declare_parameter("omega_mid_max", 0.160)
        self.declare_parameter("omega_curve_max", 0.440)
        self.declare_parameter("min_turn_radius_m", 0.150)

        # Center-first hysteresis
        self.declare_parameter("x_deadband_m", 0.035)
        self.declare_parameter("theta_deadband_rad", 0.050)
        self.declare_parameter("center_x_m", 0.060)
        self.declare_parameter("center_theta_rad", 0.110)
        self.declare_parameter("near_x_m", 0.090)
        self.declare_parameter("near_theta_rad", 0.180)

        # Curve confirmation: chỉ rẽ mạnh khi lỗi ổn định nhiều frame.
        self.declare_parameter("curve_enter_x_m", 0.110)
        self.declare_parameter("curve_enter_theta_rad", 0.240)
        self.declare_parameter("curve_enter_curvature", 1.25)
        self.declare_parameter("curve_confirm_frames", 2)
        self.declare_parameter("curve_release_frames", 3)

        # Sign convention and bias
        self.declare_parameter("x_bias_m", 0.0)
        self.declare_parameter("epsilon_sign", 1.0)
        self.declare_parameter("theta_sign", 1.0)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("invert_angular", False)

        # Outlier and branch guard
        self.declare_parameter("max_abs_e_y_m", 0.45)
        self.declare_parameter("max_abs_theta_rad", 1.10)
        self.declare_parameter("control_clip_e_y_m", 0.18)
        self.declare_parameter("control_clip_theta_rad", 0.42)

        self.declare_parameter("branch_guard_enable", True)
        self.declare_parameter("branch_jump_e_y_m", 0.18)
        self.declare_parameter("branch_jump_theta_rad", 0.45)
        self.declare_parameter("branch_extreme_e_y_m", 0.32)
        self.declare_parameter("branch_extreme_theta_rad", 0.80)
        self.declare_parameter("branch_hold_s", 0.90)
        self.declare_parameter("branch_hold_v", 0.050)

        # Filtering
        self.declare_parameter("median_window", 5)
        self.declare_parameter("error_filter_tau_s", 0.75)
        self.declare_parameter("derivative_filter_tau_s", 1.00)

        # Lookahead
        self.declare_parameter("lookahead_default_m", 0.42)
        self.declare_parameter("lookahead_min_m", 0.26)
        self.declare_parameter("lookahead_max_m", 0.85)

        # PD-Backstepping gains
        self.declare_parameter("k_pp", 0.82)
        self.declare_parameter("k_y", 0.50)
        self.declare_parameter("k_dy", 0.000)
        self.declare_parameter("k_theta", 0.24)
        self.declare_parameter("k_dtheta", 0.000)
        self.declare_parameter("lambda_y", 0.95)

        # Speed reduction factors
        self.declare_parameter("k_slow_y", 1.70)
        self.declare_parameter("k_slow_theta", 1.10)
        self.declare_parameter("k_slow_curvature", 0.140)

        # Rate limits
        self.declare_parameter("v_ref_rate_up", 0.14)
        self.declare_parameter("v_ref_rate_down", 0.38)
        self.declare_parameter("omega_ref_rate_center", 0.18)
        self.declare_parameter("omega_ref_rate_mid", 0.55)
        self.declare_parameter("omega_ref_rate_curve", 0.95)

        # Calibration
        self.declare_parameter("enable_calibration", True)
        self.declare_parameter("linear_cmd_scale", 1.245)

        # v5 dùng nhân angular_cmd_scale.
        # 0.85 nghĩa là giảm omega đầu ra một chút so với omega_ref.
        self.declare_parameter("angular_cmd_scale", 0.88)

        # Skid-steer constraint
        self.declare_parameter("track_width_m", 0.135)
        self.declare_parameter("wheel_radius_m", 0.0225)
        self.declare_parameter("allow_pivot_turn", False)
        self.declare_parameter("inner_wheel_min_fraction", 0.24)

        # LiDAR safety
        self.declare_parameter("enable_lidar_safety", True)
        self.declare_parameter("emergency_distance", 0.12)
        self.declare_parameter("stop_distance", 0.18)
        self.declare_parameter("slow_distance", 0.38)
        self.declare_parameter("front_angle_deg", 16.0)

        # Publishing safety
        self.declare_parameter("enable_cmd", False)
        self.declare_parameter("check_cmd_vel_conflict", True)
        self.declare_parameter("allow_cmd_vel_conflict", False)
        self.declare_parameter("publish_zero_on_conflict", True)

        self.declare_parameter("stop_burst_count", 45)
        self.declare_parameter("stop_burst_dt", 0.025)

        self.control_error_topic = str(self.get_parameter("control_error_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(String, self.control_error_topic, self.control_error_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)

        # Raw perception state
        self.last_msg_time = -1.0
        self.last_valid_time = -1.0
        self.first_valid_time = -1.0
        self.last_frame_time = None

        self.raw_valid = False
        self.raw_reason = "waiting"
        self.raw_lane_state = "UNKNOWN"
        self.raw_confidence = 0.0
        self.raw_e_y = 0.0
        self.raw_e_theta = 0.0
        self.raw_lookahead = float(self.get_parameter("lookahead_default_m").value)
        self.raw_source_y = "none"
        self.raw_source_theta = "none"
        self.raw_source_lookahead = "default"

        # Accepted control-error buffers
        mw = int(self.get_parameter("median_window").value)
        self.e_y_buffer = deque(maxlen=mw)
        self.e_theta_buffer = deque(maxlen=mw)

        self.last_accepted_e_y = None
        self.last_accepted_e_theta = None

        self.branch_hold_until = -1.0
        self.branch_hold_reason = "none"

        # Filtered errors
        self.e_y_f = 0.0
        self.e_theta_f = 0.0
        self.de_y_f = 0.0
        self.de_theta_f = 0.0
        self.prev_e_y_used = 0.0
        self.prev_e_theta_used = 0.0

        # Curve evidence
        self.curve_count = 0
        self.curve_release_count = 0
        self.curve_sign = 0
        self.curve_confirmed = False

        # Speed/command state
        self.fps_est = float(self.get_parameter("fps_init").value)

        self.v_ref_prev = 0.0
        self.omega_ref_prev = 0.0
        self.v_cmd_prev = 0.0
        self.omega_cmd_prev = 0.0

        # Sensor state
        self.front_min = float("inf")
        self.last_scan_time = -1.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_v = 0.0
        self.odom_omega = 0.0
        self.last_odom_time = -1.0

        self.prev_loop_time = self.now_s()

        hz = max(5.0, float(self.get_parameter("control_hz").value))
        self.create_timer(1.0 / hz, self.control_loop)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.get_logger().info("pd_backsteping_cmdvel_node v5_center_first_pd_backstepping started")
        self.get_logger().info(f"Subscribe: {self.control_error_topic}")
        self.get_logger().info(f"Publish:   {self.cmd_vel_topic}")
        self.get_logger().info("enable_cmd initial: %s" % bool(self.get_parameter("enable_cmd").value))

    def now_s(self):
        return time.time()

    def pfloat(self, name):
        return float(self.get_parameter(name).value)

    def pint(self, name):
        return int(self.get_parameter(name).value)

    def pbool(self, name):
        return bool(self.get_parameter(name).value)

    def alpha_from_tau(self, dt, tau):
        tau = max(1e-3, float(tau))
        return 1.0 - math.exp(-dt / tau)

    def make_cmd(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        return msg

    def signal_handler(self, signum, frame):
        self.get_logger().warn("Signal received. Sending stop burst.")
        self.publish_stop_burst()
        if rclpy.ok():
            rclpy.shutdown()

    def publish_stop_burst(self):
        stop = self.make_cmd(0.0, 0.0)
        count = max(5, self.pint("stop_burst_count"))
        dt = max(0.005, self.pfloat("stop_burst_dt"))

        for _ in range(count):
            self.cmd_pub.publish(stop)
            time.sleep(dt)

        self.v_ref_prev = 0.0
        self.omega_ref_prev = 0.0
        self.v_cmd_prev = 0.0
        self.omega_cmd_prev = 0.0

    def publish_state(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def update_dynamic_window(self):
        n = max(1, min(9, self.pint("median_window")))

        if self.e_y_buffer.maxlen != n:
            old_y = list(self.e_y_buffer)[-n:]
            old_t = list(self.e_theta_buffer)[-n:]
            self.e_y_buffer = deque(old_y, maxlen=n)
            self.e_theta_buffer = deque(old_t, maxlen=n)

    def update_fps(self, now, data):
        fps_msg = finite_float(data.get("fps", data.get("fps_est", data.get("vision_fps", None))), None)

        if fps_msg is not None and fps_msg > 0.1:
            fps_now = clamp(fps_msg, self.pfloat("fps_min"), self.pfloat("fps_max"))
            dt_est = 1.0 / max(fps_now, 1e-3)
        else:
            if self.last_frame_time is None:
                self.last_frame_time = now
                return

            dt_est = max(now - self.last_frame_time, 1e-3)
            fps_now = clamp(1.0 / dt_est, self.pfloat("fps_min"), self.pfloat("fps_max"))

        self.last_frame_time = now

        alpha = self.alpha_from_tau(dt_est, self.pfloat("fps_tau_s"))
        self.fps_est = (1.0 - alpha) * self.fps_est + alpha * fps_now

    def extract_errors(self, data):
        e_y = None
        source_y = "none"

        for key in ["lateral_error_m", "lateral_error", "e_y_m", "e_lat_m", "x_error_m"]:
            if key in data:
                e_y = finite_float(data.get(key), None)
                source_y = key
                break

        if e_y is None:
            for key in ["epsilon_x_mm", "x_mm", "e_y_mm", "e_lat_mm"]:
                val = finite_float(data.get(key), None)
                if val is not None:
                    e_y = val / 1000.0
                    source_y = key
                    break

        if e_y is None:
            e_y = 0.0

        e_y = self.pfloat("epsilon_sign") * e_y + self.pfloat("x_bias_m")

        e_theta = None
        source_theta = "none"

        for key in ["heading_error_rad", "heading_error", "e_theta_rad", "theta_error_rad", "theta_rad"]:
            if key in data:
                e_theta = finite_float(data.get(key), None)
                source_theta = key
                break

        if e_theta is None:
            e_theta = 0.0

        e_theta = self.pfloat("theta_sign") * e_theta

        lookahead = None
        source_lookahead = "default"

        for key in ["lookahead_m", "lookahead_d_m", "epsilon_y_m", "target_y_m"]:
            if key in data:
                lookahead = finite_float(data.get(key), None)
                source_lookahead = key
                break

        if lookahead is None:
            for key in ["lookahead_d_mm", "epsilon_y_mm", "target_y_mm"]:
                val = finite_float(data.get(key), None)
                if val is not None:
                    lookahead = val / 1000.0
                    source_lookahead = key
                    break

        if lookahead is None:
            lookahead = self.pfloat("lookahead_default_m")

        lookahead = clamp(lookahead, self.pfloat("lookahead_min_m"), self.pfloat("lookahead_max_m"))

        return e_y, e_theta, lookahead, source_y, source_theta, source_lookahead

    def extract_valid(self, data, e_y, e_theta):
        lane_state = str(data.get("lane_state", data.get("state", ""))).upper()

        if lane_state in ["LOST", "INVALID", "NO_LANE", "NONE"]:
            return False, 0.0, lane_state, "lane_state_invalid"

        valid = parse_bool(data.get("valid", True), True)
        lane_valid = parse_bool(data.get("lane_valid", True), True)
        confidence = finite_float(data.get("confidence", data.get("conf", data.get("prob", 1.0))), 1.0)

        if abs(e_y) > self.pfloat("max_abs_e_y_m"):
            return False, confidence, lane_state, "e_y_outlier"

        if abs(e_theta) > self.pfloat("max_abs_theta_rad"):
            return False, confidence, lane_state, "theta_outlier"

        if lane_state == "FOLLOW_MAIN":
            return True, confidence, lane_state, "ok_follow_main"

        ok = valid and lane_valid
        return ok, confidence, lane_state, "ok" if ok else "invalid_flags"

    def branch_guard_triggered(self, e_y, e_theta):
        if not self.pbool("branch_guard_enable"):
            return False, "disabled"

        extreme = (
            abs(e_y) > self.pfloat("branch_extreme_e_y_m")
            or abs(e_theta) > self.pfloat("branch_extreme_theta_rad")
        )

        if extreme:
            return True, f"extreme raw_e={e_y:.3f}, raw_theta={e_theta:.3f}"

        if self.last_accepted_e_y is None or self.last_accepted_e_theta is None:
            return False, "first_valid"

        jump_e = abs(e_y - self.last_accepted_e_y)
        jump_t = abs(e_theta - self.last_accepted_e_theta)

        jump = (
            jump_e > self.pfloat("branch_jump_e_y_m")
            or jump_t > self.pfloat("branch_jump_theta_rad")
        )

        if jump:
            return True, f"jump_e={jump_e:.3f}, jump_theta={jump_t:.3f}"

        return False, "normal"

    def control_error_callback(self, msg):
        self.update_dynamic_window()
        now = self.now_s()

        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid control_error JSON: {exc}")
            return

        e_y, e_theta, lookahead, source_y, source_theta, source_lookahead = self.extract_errors(data)
        valid, confidence, lane_state, reason = self.extract_valid(data, e_y, e_theta)

        self.update_fps(now, data)

        self.raw_valid = valid
        self.raw_reason = reason
        self.raw_lane_state = lane_state
        self.raw_confidence = confidence
        self.raw_e_y = e_y
        self.raw_e_theta = e_theta
        self.raw_lookahead = lookahead
        self.raw_source_y = source_y
        self.raw_source_theta = source_theta
        self.raw_source_lookahead = source_lookahead

        self.last_msg_time = now

        if not valid:
            return

        if self.first_valid_time < 0.0:
            self.first_valid_time = now

        self.last_valid_time = now

        branch, branch_reason = self.branch_guard_triggered(e_y, e_theta)

        if branch:
            self.branch_hold_until = now + self.pfloat("branch_hold_s")
            self.branch_hold_reason = branch_reason
            return

        self.branch_hold_reason = "none"

        e_y_ctrl = clamp(e_y, -self.pfloat("control_clip_e_y_m"), self.pfloat("control_clip_e_y_m"))
        e_theta_ctrl = clamp(e_theta, -self.pfloat("control_clip_theta_rad"), self.pfloat("control_clip_theta_rad"))

        self.e_y_buffer.append(e_y_ctrl)
        self.e_theta_buffer.append(e_theta_ctrl)

        self.last_accepted_e_y = e_y
        self.last_accepted_e_theta = e_theta

    def scan_callback(self, msg):
        front_angle = math.radians(self.pfloat("front_angle_deg"))

        vals = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                if abs(angle) <= front_angle:
                    vals.append(float(r))
            angle += msg.angle_increment

        self.front_min = min(vals) if vals else float("inf")
        self.last_scan_time = self.now_s()

    def odom_callback(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.odom_v = float(msg.twist.twist.linear.x)
        self.odom_omega = float(msg.twist.twist.angular.z)
        self.last_odom_time = self.now_s()

    def get_filtered_errors(self, dt, use_measurement):
        if use_measurement and len(self.e_y_buffer) > 0:
            e_y_target = statistics.median(self.e_y_buffer)
            e_theta_target = statistics.median(self.e_theta_buffer)
        else:
            e_y_target = 0.0
            e_theta_target = 0.0

        alpha = self.alpha_from_tau(dt, self.pfloat("error_filter_tau_s"))

        self.e_y_f = (1.0 - alpha) * self.e_y_f + alpha * e_y_target
        self.e_theta_f = (1.0 - alpha) * self.e_theta_f + alpha * e_theta_target

        if abs(self.e_y_f) < self.pfloat("x_deadband_m"):
            e_y_used = 0.0
        else:
            e_y_used = self.e_y_f

        if abs(self.e_theta_f) < self.pfloat("theta_deadband_rad"):
            e_theta_used = 0.0
        else:
            e_theta_used = self.e_theta_f

        de_y_raw = (e_y_used - self.prev_e_y_used) / max(dt, 1e-3)
        de_theta_raw = (e_theta_used - self.prev_e_theta_used) / max(dt, 1e-3)

        self.prev_e_y_used = e_y_used
        self.prev_e_theta_used = e_theta_used

        d_alpha = self.alpha_from_tau(dt, self.pfloat("derivative_filter_tau_s"))
        self.de_y_f = (1.0 - d_alpha) * self.de_y_f + d_alpha * de_y_raw
        self.de_theta_f = (1.0 - d_alpha) * self.de_theta_f + d_alpha * de_theta_raw

        return e_y_used, e_theta_used

    def update_curve_evidence(self, e_y, e_theta, curvature):
        curve_input = (
            abs(e_y) > self.pfloat("curve_enter_x_m")
            or abs(e_theta) > self.pfloat("curve_enter_theta_rad")
            or abs(curvature) > self.pfloat("curve_enter_curvature")
        )

        if abs(e_theta) > 0.08:
            sign_src = e_theta
        else:
            sign_src = e_y

        sign = 1 if sign_src > 0 else -1 if sign_src < 0 else 0

        if curve_input and sign != 0:
            if sign == self.curve_sign:
                self.curve_count += 1
            else:
                self.curve_count = 1
                self.curve_sign = sign

            self.curve_release_count = 0

            if self.curve_count >= self.pint("curve_confirm_frames"):
                self.curve_confirmed = True
        else:
            self.curve_release_count += 1

            if self.curve_release_count >= self.pint("curve_release_frames"):
                self.curve_confirmed = False
                self.curve_count = 0
                self.curve_sign = 0

        return self.curve_confirmed

    def speed_base_from_fps(self):
        v_base = self.fps_est * self.pfloat("target_distance_per_frame_m")
        return clamp(v_base, self.pfloat("v_min"), self.pfloat("v_max"))

    def compute_tracking_command(self, e_y, e_theta):
        lookahead = max(0.05, self.raw_lookahead)
        curvature = -2.0 * e_y / max(lookahead * lookahead, 1e-4)

        curve_confirmed = self.update_curve_evidence(e_y, e_theta, curvature)

        center_zone = (
            abs(self.e_y_f) < self.pfloat("center_x_m")
            and abs(self.e_theta_f) < self.pfloat("center_theta_rad")
        )

        near_zone = (
            abs(self.e_y_f) < self.pfloat("near_x_m")
            and abs(self.e_theta_f) < self.pfloat("near_theta_rad")
        )

        v_base = self.speed_base_from_fps()

        theta_virtual = math.atan(self.pfloat("lambda_y") * e_y)
        e_theta_bs = e_theta + theta_virtual

        omega_pp = self.pfloat("k_pp") * v_base * curvature

        omega_raw = (
            omega_pp
            - self.pfloat("k_y") * e_y
            - self.pfloat("k_dy") * self.de_y_f
            - self.pfloat("k_theta") * e_theta_bs
            - self.pfloat("k_dtheta") * self.de_theta_f
        )

        omega_raw *= self.pfloat("steering_sign")

        if self.pbool("invert_angular"):
            omega_raw = -omega_raw

        slow_factor = math.exp(
            -self.pfloat("k_slow_y") * abs(self.e_y_f)
            -self.pfloat("k_slow_theta") * abs(self.e_theta_f)
            -self.pfloat("k_slow_curvature") * abs(curvature)
        )

        if center_zone:
            mode_detail = "center_cruise"
            omega_limit = self.pfloat("omega_center_max")
            v_des = v_base
            omega_des = 0.0

        elif near_zone and not curve_confirmed:
            mode_detail = "near_center_recover"
            omega_limit = self.pfloat("omega_near_max")
            v_des = min(v_base, self.pfloat("v_recover_max"))
            omega_des = clamp(omega_raw, -omega_limit, omega_limit)

        elif curve_confirmed:
            mode_detail = "curve_tracking"
            omega_limit = self.pfloat("omega_curve_max")
            radius_limit = abs(v_base) / max(0.05, self.pfloat("min_turn_radius_m"))
            omega_limit = min(omega_limit, self.pfloat("omega_abs_max"), max(0.10, radius_limit))
            v_des = clamp(v_base * slow_factor, self.pfloat("v_slow"), self.pfloat("v_curve_max"))
            omega_des = clamp(omega_raw, -omega_limit, omega_limit)

        else:
            mode_detail = "straight_recover"
            omega_limit = self.pfloat("omega_mid_max")
            radius_limit = abs(v_base) / max(0.05, self.pfloat("min_turn_radius_m"))
            omega_limit = min(omega_limit, self.pfloat("omega_abs_max"), max(0.08, radius_limit))
            v_des = clamp(v_base * slow_factor, self.pfloat("v_slow"), self.pfloat("v_recover_max"))
            omega_des = clamp(omega_raw, -omega_limit, omega_limit)

        # Chống đảo lái liên tục khi chưa xác nhận đang vào cua.
        if not curve_confirmed and omega_des * self.omega_ref_prev < 0.0:
            omega_des = 0.0

        return {
            "mode_detail": mode_detail,
            "curve_confirmed": curve_confirmed,
            "center_zone": center_zone,
            "near_zone": near_zone,
            "v_base": v_base,
            "v_des": v_des,
            "omega_raw": omega_raw,
            "omega_des": omega_des,
            "omega_limit": omega_limit,
            "curvature": curvature,
            "e_theta_bs": e_theta_bs,
            "slow_factor": slow_factor,
        }

    def apply_no_pivot_limit(self, v, omega):
        if self.pbool("allow_pivot_turn"):
            return v, omega, "pivot_allowed"

        if v <= 0.0:
            return 0.0, 0.0, "no_pivot_zero_v"

        B = max(0.05, self.pfloat("track_width_m"))
        inner_frac = clamp(self.pfloat("inner_wheel_min_fraction"), 0.0, 0.95)

        max_omega = 2.0 * v * (1.0 - inner_frac) / B
        omega_limited = clamp(omega, -max_omega, max_omega)

        return v, omega_limited, "no_pivot_limited"

    def apply_lidar_safety(self, v, omega):
        if not self.pbool("enable_lidar_safety"):
            return v, omega, False, "lidar_disabled"

        if not math.isfinite(self.front_min):
            return v, omega, False, "lidar_no_data"

        emergency = self.pfloat("emergency_distance")
        stop = self.pfloat("stop_distance")
        slow = self.pfloat("slow_distance")

        if self.front_min < emergency:
            return 0.0, 0.0, True, "lidar_emergency"

        if self.front_min < stop:
            return 0.0, 0.0, True, "lidar_stop"

        if self.front_min < slow and v > 0.0:
            ratio = clamp((self.front_min - stop) / max(1e-6, slow - stop), 0.30, 1.0)
            return v * ratio, omega, False, "lidar_slow"

        return v, omega, False, "lidar_clear"

    def cmd_vel_conflict_detected(self):
        if not self.pbool("check_cmd_vel_conflict"):
            return False, []

        if self.pbool("allow_cmd_vel_conflict"):
            return False, []

        infos = self.get_publishers_info_by_topic(self.cmd_vel_topic)
        names = []

        for info in infos:
            if info.node_namespace == "/":
                names.append(info.node_name)
            else:
                names.append(f"{info.node_namespace}/{info.node_name}")

        if self.cmd_vel_topic == "/cmd_vel" and len(infos) > 1:
            return True, names

        return False, names

    def publish_cmd_if_enabled(self, v, omega):
        enabled = self.pbool("enable_cmd")
        conflict, publishers = self.cmd_vel_conflict_detected()

        if not enabled:
            return False, conflict, publishers, "enable_cmd_false"

        if conflict:
            if self.pbool("publish_zero_on_conflict"):
                self.cmd_pub.publish(self.make_cmd(0.0, 0.0))
            return False, conflict, publishers, "cmd_vel_conflict"

        self.cmd_pub.publish(self.make_cmd(v, omega))
        return True, conflict, publishers, "published"

    def control_loop(self):
        now = self.now_s()
        dt = max(now - self.prev_loop_time, 1e-3)
        self.prev_loop_time = now

        msg_age = now - self.last_msg_time if self.last_msg_time > 0.0 else 999.0
        valid_age = now - self.last_valid_time if self.last_valid_time > 0.0 else 999.0

        has_fresh_valid = self.raw_valid and msg_age <= self.pfloat("fresh_s")
        branch_hold_active = now < self.branch_hold_until

        startup_active = (
            self.first_valid_time > 0.0
            and now - self.first_valid_time < self.pfloat("startup_straight_s")
        )

        e_y_used = 0.0
        e_theta_used = 0.0

        info = {
            "mode_detail": "none",
            "curve_confirmed": False,
            "center_zone": False,
            "near_zone": False,
            "v_base": 0.0,
            "v_des": 0.0,
            "omega_raw": 0.0,
            "omega_des": 0.0,
            "omega_limit": 0.0,
            "curvature": 0.0,
            "e_theta_bs": 0.0,
            "slow_factor": 1.0,
        }

        if branch_hold_active:
            mode = "branch_straight_hold"
            e_y_used, e_theta_used = self.get_filtered_errors(dt, False)
            v_des = self.pfloat("branch_hold_v")
            omega_des = 0.0
            info["mode_detail"] = "branch_guard"

        elif startup_active:
            mode = "startup_straight"
            e_y_used, e_theta_used = self.get_filtered_errors(dt, True)
            v_des = min(self.speed_base_from_fps(), self.pfloat("startup_v"))
            omega_des = 0.0
            info["mode_detail"] = "startup_straight"

        elif has_fresh_valid:
            mode = "tracking"
            e_y_used, e_theta_used = self.get_filtered_errors(dt, True)
            info = self.compute_tracking_command(e_y_used, e_theta_used)
            v_des = info["v_des"]
            omega_des = info["omega_des"]

        elif valid_age <= self.pfloat("blind_hold_s"):
            mode = "blind_hold"
            e_y_used, e_theta_used = self.get_filtered_errors(dt, False)
            v_des = self.pfloat("v_blind")
            omega_des = 0.0
            info["mode_detail"] = "blind_straight"

        elif valid_age <= self.pfloat("lost_stop_s"):
            mode = "lost_slow_stop"
            e_y_used, e_theta_used = self.get_filtered_errors(dt, False)
            v_des = 0.0
            omega_des = 0.0
            info["mode_detail"] = "lost_slow_stop"

        else:
            mode = "lost_stop"
            e_y_used, e_theta_used = self.get_filtered_errors(dt, False)
            v_des = 0.0
            omega_des = 0.0
            info["mode_detail"] = "lost_stop"

        if v_des >= self.v_ref_prev:
            v_rate = self.pfloat("v_ref_rate_up")
        else:
            v_rate = self.pfloat("v_ref_rate_down")

        if info["mode_detail"] in ["center_cruise", "near_center_recover", "startup_straight", "blind_straight", "branch_guard"]:
            omega_rate = self.pfloat("omega_ref_rate_center")
        elif info["mode_detail"] == "curve_tracking":
            omega_rate = self.pfloat("omega_ref_rate_curve")
        else:
            omega_rate = self.pfloat("omega_ref_rate_mid")

        v_ref = approach(self.v_ref_prev, v_des, abs(v_rate) * dt)
        omega_ref = approach(self.omega_ref_prev, omega_des, abs(omega_rate) * dt)

        self.v_ref_prev = v_ref
        self.omega_ref_prev = omega_ref

        v_cmd_target = v_ref
        omega_cmd_target = omega_ref

        if self.pbool("enable_calibration"):
            v_cmd_target = v_cmd_target / max(1e-6, self.pfloat("linear_cmd_scale"))
            omega_cmd_target = omega_cmd_target * self.pfloat("angular_cmd_scale")

        v_cmd_target, omega_cmd_target, pivot_mode = self.apply_no_pivot_limit(v_cmd_target, omega_cmd_target)
        v_cmd_target, omega_cmd_target, lidar_stop, lidar_mode = self.apply_lidar_safety(v_cmd_target, omega_cmd_target)

        if mode in ["lost_stop", "lost_slow_stop"]:
            v_cmd_target = 0.0
            omega_cmd_target = 0.0

        v_cmd = approach(self.v_cmd_prev, v_cmd_target, abs(v_rate) * dt)
        omega_cmd = approach(self.omega_cmd_prev, omega_cmd_target, abs(omega_rate) * dt)

        if lidar_stop:
            v_cmd = 0.0
            omega_cmd = 0.0

        self.v_cmd_prev = v_cmd
        self.omega_cmd_prev = omega_cmd

        cmd_published, conflict, publishers, publish_reason = self.publish_cmd_if_enabled(v_cmd, omega_cmd)

        B = max(0.05, self.pfloat("track_width_m"))
        R = max(1e-6, self.pfloat("wheel_radius_m"))

        v_left_cmd = v_cmd - omega_cmd * B * 0.5
        v_right_cmd = v_cmd + omega_cmd * B * 0.5

        odom_timeout = self.last_odom_time < 0.0 or (now - self.last_odom_time) > 1.5

        self.publish_state({
            "node": "pd_backsteping_cmdvel_node",
            "version": "v5_center_first_pd_backstepping",
            "time": now,

            "enabled": self.pbool("enable_cmd"),
            "cmd_published": cmd_published,
            "publish_reason": publish_reason,
            "cmd_vel_topic": self.cmd_vel_topic,
            "cmd_vel_conflict": conflict,
            "cmd_vel_publishers": publishers,

            "mode": mode,
            "mode_detail": info["mode_detail"],
            "raw_valid": self.raw_valid,
            "raw_reason": self.raw_reason,
            "lane_state": self.raw_lane_state,
            "confidence": self.raw_confidence,
            "msg_age_s": msg_age,
            "valid_age_s": valid_age,

            "branch_hold_active": branch_hold_active,
            "branch_hold_reason": self.branch_hold_reason,

            "startup_active": startup_active,
            "curve_confirmed": info["curve_confirmed"],
            "curve_count": self.curve_count,
            "curve_sign": self.curve_sign,
            "center_zone": info["center_zone"],
            "near_zone": info["near_zone"],

            "fps_est": self.fps_est,
            "v_base": info["v_base"],
            "slow_factor": info["slow_factor"],

            "source_y": self.raw_source_y,
            "source_theta": self.raw_source_theta,
            "source_lookahead": self.raw_source_lookahead,

            "e_y_m": self.raw_e_y,
            "e_y_mm": self.raw_e_y * 1000.0,
            "epsilon_x_mm": self.raw_e_y * 1000.0,
            "e_y_f_m": self.e_y_f,
            "e_y_used_m": e_y_used,
            "e_y_used_mm": e_y_used * 1000.0,

            "e_theta_rad": self.raw_e_theta,
            "theta_rad": self.raw_e_theta,
            "e_theta_f_rad": self.e_theta_f,
            "e_theta_used_rad": e_theta_used,

            "lookahead_m": self.raw_lookahead,
            "curvature": info["curvature"],
            "e_theta_bs": info["e_theta_bs"],

            "omega_raw": info["omega_raw"],
            "omega_limit": info["omega_limit"],

            "de_y": self.de_y_f,
            "de_theta": self.de_theta_f,

            "v_ref": v_ref,
            "omega_ref": omega_ref,
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,

            "v_left_cmd": v_left_cmd,
            "v_right_cmd": v_right_cmd,
            "v_left_est": v_left_cmd,
            "v_right_est": v_right_cmd,
            "wheel_left_radps_est": v_left_cmd / R,
            "wheel_right_radps_est": v_right_cmd / R,

            "pivot_mode": pivot_mode,

            "lidar_stop": lidar_stop,
            "lidar_mode": lidar_mode,
            "front_min_m": self.front_min if math.isfinite(self.front_min) else None,

            "odom_timeout": odom_timeout,
            "odom_x": self.odom_x,
            "odom_y": self.odom_y,
            "odom_yaw": self.odom_yaw,
            "odom_v": self.odom_v,
            "odom_omega": self.odom_omega,

            "epsilon_sign": self.pfloat("epsilon_sign"),
            "theta_sign": self.pfloat("theta_sign"),
            "steering_sign": self.pfloat("steering_sign"),
            "invert_angular": self.pbool("invert_angular"),
            "angular_calibration_mode": "omega_cmd = omega_ref * angular_cmd_scale",
        })


def main(args=None):
    rclpy.init(args=args)
    node = PDBackstepingCmdVelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().warn("Shutdown: sending stop burst.")
        node.publish_stop_burst()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
