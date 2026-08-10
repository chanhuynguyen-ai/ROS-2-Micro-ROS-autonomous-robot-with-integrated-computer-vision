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
from std_msgs.msg import String, Bool


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


class CascadeControllerAvoid(Node):
    def __init__(self):
        super().__init__("cascade_controller_avoid")

        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("lane_state_topic", "/avs/lane_state")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("ref_topic", "/avs/cascade_controller_avoid_ref")
        self.declare_parameter("state_topic", "/avs/cascade_controller_avoid_state")
        self.declare_parameter("runtime_enable_topic", "/avs/cascade_avoid_enable_cmd")
        self.declare_parameter("emergency_stop_topic", "/avs/cascade_avoid_emergency_stop")

        self.declare_parameter("enable_cmd", False)
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("error_timeout_s", 1.35)
        self.declare_parameter("blind_hold_s", 0.45)
        self.declare_parameter("startup_straight_s", 0.25)
        self.declare_parameter("startup_v", 0.045)

        self.declare_parameter("fps_init", 3.0)
        self.declare_parameter("fps_min", 0.8)
        self.declare_parameter("fps_max", 25.0)
        self.declare_parameter("fps_tau_s", 1.0)
        self.declare_parameter("target_distance_per_frame_m", 0.040)

        self.declare_parameter("v_max", 0.140)
        self.declare_parameter("v_center", 0.115)
        self.declare_parameter("v_min", 0.040)
        self.declare_parameter("v_curve_max", 0.078)
        self.declare_parameter("v_recover_max", 0.090)
        self.declare_parameter("v_large_error", 0.040)
        self.declare_parameter("v_blind", 0.035)

        self.declare_parameter("track_width_m", 0.135)
        self.declare_parameter("wheel_radius_m", 0.0225)

        self.declare_parameter("epsilon_sign", 1.0)
        self.declare_parameter("theta_sign", 1.0)
        self.declare_parameter("theta_control_sign", -1.0)
        self.declare_parameter("invert_angular", False)
        self.declare_parameter("x_bias_m", 0.0)

        self.declare_parameter("median_window", 7)
        self.declare_parameter("error_filter_tau_s", 0.58)
        self.declare_parameter("derivative_filter_tau_s", 1.15)

        self.declare_parameter("x_deadband_m", 0.016)
        self.declare_parameter("theta_deadband_rad", 0.028)

        self.declare_parameter("center_x_m", 0.045)
        self.declare_parameter("center_theta_rad", 0.080)
        self.declare_parameter("near_x_m", 0.085)
        self.declare_parameter("near_theta_rad", 0.170)
        self.declare_parameter("large_error_x_m", 0.240)
        self.declare_parameter("large_error_theta_rad", 0.800)

        self.declare_parameter("curve_enter_x_m", 0.110)
        self.declare_parameter("curve_enter_theta_rad", 0.230)
        self.declare_parameter("curve_enter_kappa", 1.20)
        self.declare_parameter("curve_confirm_frames", 3)
        self.declare_parameter("curve_release_frames", 7)

        self.declare_parameter("max_abs_x_m", 0.60)
        self.declare_parameter("max_abs_theta_rad", 1.50)
        self.declare_parameter("min_valid_lookahead_m", 0.16)

        self.declare_parameter("jump_guard_enable", True)
        self.declare_parameter("jump_x_m", 0.320)
        self.declare_parameter("jump_theta_rad", 0.820)

        self.declare_parameter("soft_replan_enable", True)
        self.declare_parameter("soft_replan_step_x_m", 0.060)
        self.declare_parameter("soft_replan_step_theta_rad", 0.160)
        self.declare_parameter("soft_replan_hold_s", 0.45)
        self.declare_parameter("soft_replan_v_max", 0.052)
        self.declare_parameter("soft_replan_omega_max", 0.200)

        self.declare_parameter("k_pp_near", 0.08)
        self.declare_parameter("k_lat_near", 0.16)
        self.declare_parameter("k_theta_near", 0.08)

        self.declare_parameter("k_pp_mid", 0.58)
        self.declare_parameter("k_lat_mid", 0.52)
        self.declare_parameter("k_theta_mid", 0.20)

        self.declare_parameter("k_pp_curve", 1.05)
        self.declare_parameter("k_lat_curve", 0.72)
        self.declare_parameter("k_theta_curve", 0.28)

        self.declare_parameter("k_pp_large", 0.18)
        self.declare_parameter("k_lat_large", 0.34)
        self.declare_parameter("k_theta_large", 0.16)

        self.declare_parameter("kd_lat", 0.002)
        self.declare_parameter("kd_theta", 0.003)

        self.declare_parameter("k_slow_x", 0.90)
        self.declare_parameter("k_slow_theta", 0.65)
        self.declare_parameter("k_slow_kappa", 0.040)

        self.declare_parameter("omega_center_max", 0.000)
        self.declare_parameter("omega_near_max", 0.025)
        self.declare_parameter("omega_mid_max", 0.220)
        self.declare_parameter("omega_curve_max", 0.480)
        self.declare_parameter("omega_large_error_max", 0.270)
        self.declare_parameter("omega_abs_max", 0.520)
        self.declare_parameter("omega_deadband", 0.008)

        self.declare_parameter("v_rate_up", 0.17)
        self.declare_parameter("v_rate_down", 0.38)
        self.declare_parameter("omega_rate_center", 0.30)
        self.declare_parameter("omega_rate_mid", 0.55)
        self.declare_parameter("omega_rate_curve", 0.78)

        self.declare_parameter("ld_min_m", 0.320)
        self.declare_parameter("ld_max_m", 0.90)
        self.declare_parameter("default_lookahead_m", 0.48)
        self.declare_parameter("max_abs_kappa", 2.3)

        self.declare_parameter("v_max_cmd", 0.145)
        self.declare_parameter("omega_max_cmd", 0.70)
        self.declare_parameter("wheel_speed_rate", 0.125)
        self.declare_parameter("wheel_speed_decel_rate", 0.280)
        self.declare_parameter("same_direction_inner_fraction", 0.55)
        self.declare_parameter("same_direction_min_inner_mps", 0.028)
        self.declare_parameter("inner_omega_deadband", 0.018)

        self.declare_parameter("enable_calibration", True)
        self.declare_parameter("linear_cmd_scale", 1.245)
        self.declare_parameter("angular_cmd_scale", 0.92)

        self.declare_parameter("enable_lidar_safety", True)
        self.declare_parameter("front_angle_deg", 14.0)
        self.declare_parameter("emergency_distance", 0.035)
        self.declare_parameter("stop_distance", 0.100)
        self.declare_parameter("slow_distance", 0.300)

        self.declare_parameter("enable_obstacle_avoidance", True)
        self.declare_parameter("avoidance_direction", "right")
        self.declare_parameter("avoid_trigger_distance", 0.155)
        self.declare_parameter("avoid_clear_distance", 0.430)
        self.declare_parameter("avoid_clear_hold_s", 0.70)
        self.declare_parameter("avoid_min_duration_s", 1.20)
        self.declare_parameter("avoid_lateral_offset_m", 0.220)
        self.declare_parameter("avoid_offset_rate_mps", 0.090)
        self.declare_parameter("avoidance_v_max", 0.060)
        self.declare_parameter("avoidance_omega_max", 0.300)
        self.declare_parameter("avoidance_min_omega", 0.075)
        self.declare_parameter("avoid_lidar_min_ratio", 0.45)

        self.declare_parameter("avoid_reverse_enable", True)
        self.declare_parameter("avoid_reverse_trigger_distance", 0.105)
        self.declare_parameter("avoid_reverse_duration_s", 0.55)
        self.declare_parameter("avoid_reverse_v", 0.028)
        self.declare_parameter("avoid_reverse_omega", 0.0)

        self.declare_parameter("check_cmd_vel_conflict", True)
        self.declare_parameter("allow_cmd_vel_conflict", False)
        self.declare_parameter("publish_zero_on_conflict", True)
        self.declare_parameter("stop_burst_count", 45)
        self.declare_parameter("stop_burst_dt", 0.025)
        self.declare_parameter("always_publish_stop_on_exit", True)

        self.control_error_topic = self.pstr("control_error_topic")
        self.lane_state_topic = self.pstr("lane_state_topic")
        self.odom_topic = self.pstr("odom_topic")
        self.scan_topic = self.pstr("scan_topic")
        self.cmd_vel_topic = self.pstr("cmd_vel_topic")
        self.ref_topic = self.pstr("ref_topic")
        self.state_topic = self.pstr("state_topic")

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.ref_pub = self.create_publisher(Twist, self.ref_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.create_subscription(String, self.control_error_topic, self.control_error_cb, 10)
        self.create_subscription(String, self.lane_state_topic, self.lane_state_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, self.pstr("runtime_enable_topic"), self.runtime_enable_cb, 10)
        self.create_subscription(Bool, self.pstr("emergency_stop_topic"), self.emergency_stop_cb, 10)

        self.runtime_enable = False
        self.emergency_stop = False
        self.last_error_time = -1.0
        self.first_valid_time = -1.0
        self.last_frame_time = None

        self.raw_valid = False
        self.raw_valid_reason = "waiting"
        self.raw_lane_state = ""
        self.raw_confidence = 0.0
        self.raw_e_lat = 0.0
        self.raw_e_theta = 0.0
        self.raw_lookahead = self.pfloat("default_lookahead_m")
        self.raw_kappa = 0.0

        self.last_accepted_e = None
        self.last_accepted_theta = None

        self.lane_state_debug = {}
        self.last_lane_state_time = -1.0
        self.intent_hint = "UNKNOWN"
        self.trajectory_hint = "UNKNOWN"
        self.planner_status_hint = "UNKNOWN"

        mw = max(1, self.pint("median_window"))
        self.e_buf = deque(maxlen=mw)
        self.theta_buf = deque(maxlen=mw)

        self.e_f = 0.0
        self.theta_f = 0.0
        self.de_f = 0.0
        self.dtheta_f = 0.0
        self.prev_e_used = 0.0
        self.prev_theta_used = 0.0
        self.fps_est = self.pfloat("fps_init")

        self.curve_confirmed = False
        self.curve_count = 0
        self.curve_release_count = 0
        self.curve_sign = 0

        self.soft_replan_until = -1.0
        self.soft_replan_reason = "none"

        self.odom_v = 0.0
        self.odom_omega = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = -1.0

        self.front_min = float("inf")
        self.last_scan_time = -1.0

        self.avoidance_mode = "lane_follow"
        self.avoidance_offset_current = 0.0
        self.avoidance_offset_target = 0.0
        self.avoidance_start_time = -1.0
        self.avoidance_clear_start_time = -1.0

        self.v_ref = 0.0
        self.omega_ref = 0.0
        self.v_left_cmd_prev = 0.0
        self.v_right_cmd_prev = 0.0
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.prev_loop_time = time.time()

        hz = max(5.0, self.pfloat("control_hz"))
        self.create_timer(1.0 / hz, self.control_loop)

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        self.get_logger().info("cascade_controller_avoid started")

    def now_s(self):
        return time.time()

    def pfloat(self, name):
        return float(self.get_parameter(name).value)

    def pint(self, name):
        return int(self.get_parameter(name).value)

    def pbool(self, name):
        return bool(self.get_parameter(name).value)

    def pstr(self, name):
        return str(self.get_parameter(name).value)

    def alpha_from_tau(self, dt, tau):
        tau = max(1e-3, float(tau))
        return 1.0 - math.exp(-dt / tau)

    def make_twist(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        return msg

    def signal_stop_handler(self, signum, frame):
        self.publish_stop_burst()
        if rclpy.ok():
            rclpy.shutdown()

    def publish_stop_burst(self):
        stop = self.make_twist(0.0, 0.0)
        count = max(5, self.pint("stop_burst_count"))
        dt = max(0.005, self.pfloat("stop_burst_dt"))
        for _ in range(count):
            self.cmd_pub.publish(stop)
            time.sleep(dt)
        self.v_ref = 0.0
        self.omega_ref = 0.0
        self.v_left_cmd_prev = 0.0
        self.v_right_cmd_prev = 0.0
        self.v_cmd = 0.0
        self.omega_cmd = 0.0

    def publish_state(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def publish_ref(self, v_ref, omega_ref):
        self.ref_pub.publish(self.make_twist(v_ref, omega_ref))

    def is_enabled(self):
        return self.pbool("enable_cmd") or self.runtime_enable

    def runtime_enable_cb(self, msg):
        self.runtime_enable = bool(msg.data)
        if not self.runtime_enable and not self.pbool("enable_cmd"):
            self.publish_stop_burst()

    def emergency_stop_cb(self, msg):
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.publish_stop_burst()

    def odom_cb(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.odom_v = float(msg.twist.twist.linear.x)
        self.odom_omega = float(msg.twist.twist.angular.z)
        self.last_odom_time = self.now_s()

    def scan_cb(self, msg):
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

    def lane_state_cb(self, msg):
        try:
            d = json.loads(msg.data)
            if not isinstance(d, dict):
                return
        except Exception:
            return

        self.lane_state_debug = d
        self.last_lane_state_time = self.now_s()
        text = json.dumps(d, ensure_ascii=False).upper()

        if "LANE_CHANGE" in text or "CHANGE_LANE" in text:
            self.intent_hint = "LANE_CHANGE"
        elif "TURN_LEFT" in text or "LEFT" in text:
            self.intent_hint = "TURN_LEFT"
        elif "TURN_RIGHT" in text or "RIGHT" in text:
            self.intent_hint = "TURN_RIGHT"
        elif "FOLLOW_MAIN" in text or "MAIN" in text:
            self.intent_hint = "FOLLOW_MAIN"
        else:
            self.intent_hint = "UNKNOWN"

        if "COMMITTED" in text:
            self.trajectory_hint = "COMMITTED"
        elif "HOLD" in text:
            self.trajectory_hint = "HOLD"
        elif "RECOVERY" in text:
            self.trajectory_hint = "RECOVERY"
        else:
            self.trajectory_hint = "UNKNOWN"

        if "BLOCKED_BY_MARKING" in text:
            self.planner_status_hint = "BLOCKED_BY_MARKING"
        elif "DROPOUT" in text:
            self.planner_status_hint = "DROPOUT"
        elif "REPLAN" in text:
            self.planner_status_hint = "REPLAN"
        elif "HOLD" in text:
            self.planner_status_hint = "HOLD"
        else:
            self.planner_status_hint = "UNKNOWN"

    def update_fps(self, now, data):
        fps_msg = finite_float(data.get("fps", data.get("fps_est", data.get("vision_fps", None))), None)
        if fps_msg is not None and fps_msg > 0.1:
            fps_now = clamp(fps_msg, self.pfloat("fps_min"), self.pfloat("fps_max"))
            dt_frame = 1.0 / max(fps_now, 1e-3)
        else:
            if self.last_frame_time is None:
                self.last_frame_time = now
                return
            dt_frame = max(now - self.last_frame_time, 1e-3)
            fps_now = clamp(1.0 / dt_frame, self.pfloat("fps_min"), self.pfloat("fps_max"))

        self.last_frame_time = now
        alpha = self.alpha_from_tau(dt_frame, self.pfloat("fps_tau_s"))
        self.fps_est = (1.0 - alpha) * self.fps_est + alpha * fps_now

    def extract_control_error(self, data):
        e_lat = None
        for key in ["lateral_error_m", "lateral_error", "e_lat_m", "e_y_m", "x_error_m"]:
            if key in data:
                e_lat = finite_float(data.get(key), None)
                break
        if e_lat is None:
            for key in ["epsilon_x_mm", "x_mm", "e_y_mm", "e_lat_mm"]:
                if key in data:
                    v = finite_float(data.get(key), None)
                    if v is not None:
                        e_lat = v / 1000.0
                        break
        if e_lat is None:
            e_lat = 0.0

        e_lat = self.pfloat("epsilon_sign") * e_lat + self.pfloat("x_bias_m")

        e_theta = None
        for key in ["heading_error_rad", "heading_error", "e_theta_rad", "theta_error_rad", "theta_rad"]:
            if key in data:
                e_theta = finite_float(data.get(key), None)
                break
        if e_theta is None:
            e_theta = 0.0

        e_theta = self.pfloat("theta_sign") * e_theta

        lookahead = None
        for key in ["lookahead_m", "lookahead_d_m", "epsilon_y_m", "target_y_m"]:
            if key in data:
                lookahead = finite_float(data.get(key), None)
                break
        if lookahead is None:
            for key in ["lookahead_d_mm", "epsilon_y_mm", "target_y_mm", "y_mm"]:
                if key in data:
                    v = finite_float(data.get(key), None)
                    if v is not None:
                        lookahead = v / 1000.0
                        break
        if lookahead is None:
            lookahead = self.pfloat("default_lookahead_m")

        lookahead = clamp(abs(lookahead), self.pfloat("ld_min_m"), self.pfloat("ld_max_m"))

        curvature_inv_mm = finite_float(data.get("curvature_inv_mm", 0.0), 0.0)
        kappa = clamp(curvature_inv_mm * 1000.0, -self.pfloat("max_abs_kappa"), self.pfloat("max_abs_kappa"))

        confidence = finite_float(data.get("confidence", data.get("conf", data.get("prob", 1.0))), 1.0)
        lane_state = str(data.get("lane_state", "")).upper()
        raw_valid = parse_bool(data.get("valid", True), True)
        lane_valid = parse_bool(data.get("lane_valid", True), True)

        if lane_state in ["LOST", "INVALID", "NO_LANE", "NONE"]:
            return False, "lane_state_invalid", e_lat, e_theta, lookahead, kappa, confidence, lane_state
        if abs(e_lat) > self.pfloat("max_abs_x_m"):
            return False, "e_lat_outlier", e_lat, e_theta, lookahead, kappa, confidence, lane_state
        if abs(e_theta) > self.pfloat("max_abs_theta_rad"):
            return False, "theta_outlier", e_lat, e_theta, lookahead, kappa, confidence, lane_state
        if lookahead < self.pfloat("min_valid_lookahead_m"):
            return False, "lookahead_too_near", e_lat, e_theta, lookahead, kappa, confidence, lane_state

        valid = raw_valid and lane_valid
        if lane_state == "FOLLOW_MAIN":
            valid = True

        return valid, "ok" if valid else "invalid_flags", e_lat, e_theta, lookahead, kappa, confidence, lane_state

    def jump_guard_triggered(self, e, theta):
        if not self.pbool("jump_guard_enable"):
            return False, "disabled"
        if self.last_accepted_e is None or self.last_accepted_theta is None:
            return False, "first_valid"

        de = abs(e - self.last_accepted_e)
        dt = abs(theta - self.last_accepted_theta)

        if de > self.pfloat("jump_x_m") or dt > self.pfloat("jump_theta_rad"):
            return True, f"jump_e={de:.3f}, jump_theta={dt:.3f}"
        return False, "normal"

    def control_error_cb(self, msg):
        now = self.now_s()
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except Exception as exc:
            self.get_logger().warn(f"Invalid control_error JSON: {exc}")
            return

        valid, reason, e, theta, ld, kappa, conf, lane_state = self.extract_control_error(data)
        self.update_fps(now, data)

        self.raw_valid = valid
        self.raw_valid_reason = reason
        self.raw_e_lat = e
        self.raw_e_theta = theta
        self.raw_lookahead = ld
        self.raw_kappa = kappa
        self.raw_confidence = conf
        self.raw_lane_state = lane_state
        self.last_error_time = now

        if not valid:
            return

        if self.first_valid_time < 0.0:
            self.first_valid_time = now

        jump, jump_reason = self.jump_guard_triggered(e, theta)

        if jump:
            if self.pbool("soft_replan_enable") and self.last_accepted_e is not None and self.last_accepted_theta is not None:
                step_x = abs(self.pfloat("soft_replan_step_x_m"))
                step_th = abs(self.pfloat("soft_replan_step_theta_rad"))

                e = self.last_accepted_e + clamp(e - self.last_accepted_e, -step_x, step_x)
                theta = self.last_accepted_theta + clamp(theta - self.last_accepted_theta, -step_th, step_th)

                self.soft_replan_until = now + self.pfloat("soft_replan_hold_s")
                self.soft_replan_reason = jump_reason
            else:
                return
        else:
            self.soft_replan_reason = "none"

        self.last_accepted_e = e
        self.last_accepted_theta = theta

        self.update_median_window()
        self.e_buf.append(e)
        self.theta_buf.append(theta)

    def update_median_window(self):
        mw = max(1, self.pint("median_window"))
        if self.e_buf.maxlen != mw:
            self.e_buf = deque(list(self.e_buf)[-mw:], maxlen=mw)
            self.theta_buf = deque(list(self.theta_buf)[-mw:], maxlen=mw)

    def filtered_error(self, dt, use_measurement):
        if use_measurement and len(self.e_buf) > 0:
            target_e = statistics.median(self.e_buf) + self.avoidance_offset_current
            target_theta = statistics.median(self.theta_buf)
        else:
            target_e = self.avoidance_offset_current
            target_theta = 0.0

        alpha = self.alpha_from_tau(dt, self.pfloat("error_filter_tau_s"))

        self.e_f = (1.0 - alpha) * self.e_f + alpha * target_e
        self.theta_f = (1.0 - alpha) * self.theta_f + alpha * target_theta

        e_used = 0.0 if abs(self.e_f) < self.pfloat("x_deadband_m") else self.e_f
        theta_used = 0.0 if abs(self.theta_f) < self.pfloat("theta_deadband_rad") else self.theta_f

        de_raw = (e_used - self.prev_e_used) / max(dt, 1e-3)
        dtheta_raw = (theta_used - self.prev_theta_used) / max(dt, 1e-3)

        self.prev_e_used = e_used
        self.prev_theta_used = theta_used

        d_alpha = self.alpha_from_tau(dt, self.pfloat("derivative_filter_tau_s"))

        self.de_f = (1.0 - d_alpha) * self.de_f + d_alpha * de_raw
        self.dtheta_f = (1.0 - d_alpha) * self.dtheta_f + d_alpha * dtheta_raw

        return e_used, theta_used

    def update_curve_state(self, e, theta, kappa):
        candidate = (
            abs(e) >= self.pfloat("curve_enter_x_m")
            or abs(theta) >= self.pfloat("curve_enter_theta_rad")
            or abs(kappa) >= self.pfloat("curve_enter_kappa")
        )

        sign_src = theta if abs(theta) > 0.06 else e
        sign = 1 if sign_src > 0 else -1 if sign_src < 0 else 0

        if candidate and sign != 0:
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

    def speed_from_fps(self):
        v = self.fps_est * self.pfloat("target_distance_per_frame_m")
        return clamp(v, self.pfloat("v_min"), self.pfloat("v_max"))

    def compute_outer(self, e, theta):
        ld = max(self.pfloat("ld_min_m"), self.raw_lookahead)
        kappa = self.raw_kappa
        curvature_from_error = -2.0 * e / max(ld * ld, 1e-4)

        curve = self.update_curve_state(e, theta, kappa)

        center = (
            abs(self.e_f) <= self.pfloat("center_x_m")
            and abs(self.theta_f) <= self.pfloat("center_theta_rad")
            and self.avoidance_mode == "lane_follow"
        )

        near = (
            abs(self.e_f) <= self.pfloat("near_x_m")
            and abs(self.theta_f) <= self.pfloat("near_theta_rad")
        )

        large_error = (
            abs(self.e_f) >= self.pfloat("large_error_x_m")
            or abs(self.theta_f) >= self.pfloat("large_error_theta_rad")
        )

        v_base = self.speed_from_fps()

        slow_factor = math.exp(
            -self.pfloat("k_slow_x") * abs(self.e_f)
            -self.pfloat("k_slow_theta") * abs(self.theta_f)
            -self.pfloat("k_slow_kappa") * abs(kappa)
        )

        theta_sign = self.pfloat("theta_control_sign")

        if center:
            mode = "center_cruise"
            v_des = min(v_base, self.pfloat("v_center"))
            omega_limit = self.pfloat("omega_center_max")
            k_pp = k_lat = k_theta = 0.0
            omega_raw = 0.0

        elif near and not curve:
            mode = "near_center_cascade"
            v_des = min(v_base, self.pfloat("v_recover_max"))
            omega_limit = self.pfloat("omega_near_max")
            k_pp = self.pfloat("k_pp_near")
            k_lat = self.pfloat("k_lat_near")
            k_theta = self.pfloat("k_theta_near")
            omega_raw = (
                k_pp * v_des * curvature_from_error
                - k_lat * e
                + theta_sign * k_theta * theta
                - self.pfloat("kd_lat") * self.de_f
                - self.pfloat("kd_theta") * self.dtheta_f
            )

        elif large_error:
            mode = "large_error_slow_recover"
            v_des = self.pfloat("v_large_error")
            omega_limit = self.pfloat("omega_large_error_max")
            k_pp = self.pfloat("k_pp_large")
            k_lat = self.pfloat("k_lat_large")
            k_theta = self.pfloat("k_theta_large")
            omega_raw = (
                k_pp * v_des * curvature_from_error
                - k_lat * e
                + theta_sign * k_theta * theta
                - self.pfloat("kd_lat") * self.de_f
                - self.pfloat("kd_theta") * self.dtheta_f
            )

        elif curve:
            mode = "curve_committed_fast_response"
            v_des = clamp(v_base * slow_factor, self.pfloat("v_min"), self.pfloat("v_curve_max"))
            omega_limit = self.pfloat("omega_curve_max")
            k_pp = self.pfloat("k_pp_curve")
            k_lat = self.pfloat("k_lat_curve")
            k_theta = self.pfloat("k_theta_curve")
            omega_raw = (
                k_pp * v_des * curvature_from_error
                - k_lat * e
                + theta_sign * k_theta * theta
                - self.pfloat("kd_lat") * self.de_f
                - self.pfloat("kd_theta") * self.dtheta_f
            )

        else:
            mode = "mid_cascade_tracking"
            v_des = clamp(v_base * slow_factor, self.pfloat("v_min"), self.pfloat("v_recover_max"))
            omega_limit = self.pfloat("omega_mid_max")
            k_pp = self.pfloat("k_pp_mid")
            k_lat = self.pfloat("k_lat_mid")
            k_theta = self.pfloat("k_theta_mid")
            omega_raw = (
                k_pp * v_des * curvature_from_error
                - k_lat * e
                + theta_sign * k_theta * theta
                - self.pfloat("kd_lat") * self.de_f
                - self.pfloat("kd_theta") * self.dtheta_f
            )

        if self.pbool("invert_angular"):
            omega_raw = -omega_raw

        if abs(omega_raw) < self.pfloat("omega_deadband"):
            omega_raw = 0.0

        omega_limit = min(abs(omega_limit), self.pfloat("omega_abs_max"))
        omega_des = clamp(omega_raw, -omega_limit, omega_limit)

        if near and not curve and omega_des * self.omega_ref < 0.0:
            omega_des = 0.0

        return {
            "outer_mode": mode,
            "v_des": v_des,
            "omega_des": omega_des,
            "omega_raw": omega_raw,
            "omega_limit": omega_limit,
            "curvature_from_error": curvature_from_error,
            "center_zone": center,
            "near_zone": near,
            "large_error": large_error,
            "curve_confirmed": curve,
            "slow_factor": slow_factor,
            "k_pp_used": k_pp,
            "k_lat_used": k_lat,
            "k_theta_used": k_theta,
        }

    def avoidance_lateral_sign(self):
        return 1.0 if self.pstr("avoidance_direction").strip().lower() == "right" else -1.0

    def avoidance_angular_sign(self):
        return -1.0 if self.pstr("avoidance_direction").strip().lower() == "right" else 1.0

    def avoidance_info(self, active):
        return {
            "active": active,
            "mode": self.avoidance_mode,
            "offset_m": self.avoidance_offset_current,
            "target_offset_m": self.avoidance_offset_target,
            "angular_sign": self.avoidance_angular_sign() if active else 0.0,
        }

    def update_obstacle_avoidance(self, now, dt):
        if not self.pbool("enable_obstacle_avoidance"):
            self.avoidance_mode = "disabled"
            self.avoidance_offset_target = 0.0
            self.avoidance_offset_current = approach(
                self.avoidance_offset_current,
                0.0,
                self.pfloat("avoid_offset_rate_mps") * dt,
            )
            return self.avoidance_info(False)

        trigger = self.pfloat("avoid_trigger_distance")
        reverse_trigger = self.pfloat("avoid_reverse_trigger_distance")
        clear = self.pfloat("avoid_clear_distance")
        front_valid = math.isfinite(self.front_min)

        if not front_valid:
            self.avoidance_mode = "lane_follow"
            self.avoidance_offset_target = 0.0

        elif self.avoidance_mode in ["lane_follow", "disabled"]:
            self.avoidance_offset_target = 0.0

            if self.pbool("avoid_reverse_enable") and self.front_min <= reverse_trigger:
                self.avoidance_mode = "avoid_reverse"
                self.avoidance_start_time = now
                self.avoidance_clear_start_time = -1.0

            elif self.front_min <= trigger:
                self.avoidance_mode = "avoid_shift_out"
                self.avoidance_start_time = now
                self.avoidance_clear_start_time = -1.0
                self.avoidance_offset_target = self.avoidance_lateral_sign() * abs(self.pfloat("avoid_lateral_offset_m"))

        elif self.avoidance_mode == "avoid_reverse":
            self.avoidance_offset_target = 0.0

            if (now - self.avoidance_start_time) >= self.pfloat("avoid_reverse_duration_s"):
                self.avoidance_mode = "avoid_shift_out"
                self.avoidance_start_time = now
                self.avoidance_offset_target = self.avoidance_lateral_sign() * abs(self.pfloat("avoid_lateral_offset_m"))

        elif self.avoidance_mode == "avoid_shift_out":
            self.avoidance_offset_target = self.avoidance_lateral_sign() * abs(self.pfloat("avoid_lateral_offset_m"))

            if abs(self.avoidance_offset_current - self.avoidance_offset_target) < 0.025:
                self.avoidance_mode = "avoid_bypass"

        elif self.avoidance_mode == "avoid_bypass":
            self.avoidance_offset_target = self.avoidance_lateral_sign() * abs(self.pfloat("avoid_lateral_offset_m"))
            enough_time = (now - self.avoidance_start_time) >= self.pfloat("avoid_min_duration_s")
            clear_now = front_valid and self.front_min >= clear

            if enough_time and clear_now:
                if self.avoidance_clear_start_time < 0.0:
                    self.avoidance_clear_start_time = now
                elif (now - self.avoidance_clear_start_time) >= self.pfloat("avoid_clear_hold_s"):
                    self.avoidance_mode = "avoid_return"
            else:
                self.avoidance_clear_start_time = -1.0

        elif self.avoidance_mode == "avoid_return":
            self.avoidance_offset_target = 0.0

            if front_valid and self.front_min <= reverse_trigger and self.pbool("avoid_reverse_enable"):
                self.avoidance_mode = "avoid_reverse"
                self.avoidance_start_time = now
                self.avoidance_clear_start_time = -1.0

            elif front_valid and self.front_min <= trigger:
                self.avoidance_mode = "avoid_bypass"
                self.avoidance_offset_target = self.avoidance_lateral_sign() * abs(self.pfloat("avoid_lateral_offset_m"))

            elif abs(self.avoidance_offset_current) < 0.010:
                self.avoidance_mode = "lane_follow"
                self.avoidance_start_time = -1.0
                self.avoidance_clear_start_time = -1.0

        else:
            self.avoidance_mode = "lane_follow"
            self.avoidance_offset_target = 0.0

        self.avoidance_offset_current = approach(
            self.avoidance_offset_current,
            self.avoidance_offset_target,
            self.pfloat("avoid_offset_rate_mps") * dt,
        )

        active = self.avoidance_mode in ["avoid_reverse", "avoid_shift_out", "avoid_bypass", "avoid_return"]
        return self.avoidance_info(active)

    def lidar_scale(self):
        if not self.pbool("enable_lidar_safety"):
            return 1.0, False, "lidar_disabled"

        if not math.isfinite(self.front_min):
            return 1.0, False, "lidar_no_data"

        if self.avoidance_mode == "avoid_reverse":
            return 1.0, False, "lidar_reverse"

        if self.front_min < self.pfloat("emergency_distance"):
            return 0.0, True, "lidar_emergency"

        avoid_active = self.avoidance_mode in ["avoid_shift_out", "avoid_bypass", "avoid_return"]
        stop = self.pfloat("stop_distance")
        slow = self.pfloat("slow_distance")

        if avoid_active and self.front_min < stop:
            return max(0.25, self.pfloat("avoid_lidar_min_ratio")), False, "lidar_avoid_close"

        if self.front_min < stop:
            return 0.0, True, "lidar_stop"

        if avoid_active and self.front_min < slow:
            ratio = clamp((self.front_min - stop) / max(1e-6, slow - stop), self.pfloat("avoid_lidar_min_ratio"), 1.0)
            return ratio, False, "lidar_avoid_slow"

        if self.front_min < slow:
            ratio = clamp((self.front_min - stop) / max(1e-6, slow - stop), 0.25, 1.0)
            return ratio, False, "lidar_slow"

        return 1.0, False, "lidar_clear"

    def mix_wheels(self, v_ref, omega_ref, dt):
        b = max(0.05, self.pfloat("track_width_m"))
        reverse_limit = abs(self.pfloat("avoid_reverse_v")) if self.avoidance_mode == "avoid_reverse" else 0.0

        v_ref = clamp(v_ref, -reverse_limit, abs(self.pfloat("v_max_cmd")))
        omega_ref = clamp(omega_ref, -abs(self.pfloat("omega_max_cmd")), abs(self.pfloat("omega_max_cmd")))

        if abs(omega_ref) < self.pfloat("inner_omega_deadband"):
            omega_ref = 0.0

        if v_ref < 0.0:
            v_left_des = v_ref
            v_right_des = v_ref
            mix_mode = "reverse"

        elif v_ref <= 0.0:
            v_left_des = 0.0
            v_right_des = 0.0
            mix_mode = "zero"

        else:
            v_left_des = v_ref - omega_ref * b * 0.5
            v_right_des = v_ref + omega_ref * b * 0.5

            inner_frac = clamp(self.pfloat("same_direction_inner_fraction"), 0.0, 0.95)
            inner_min = abs(self.pfloat("same_direction_min_inner_mps"))

            if omega_ref > 0.0:
                outer = max(v_right_des, inner_min)
                inside = max(v_left_des, max(inner_min, outer * inner_frac))
                v_left_des = inside
                v_right_des = outer
                mix_mode = "same_direction_left_turn"

            elif omega_ref < 0.0:
                outer = max(v_left_des, inner_min)
                inside = max(v_right_des, max(inner_min, outer * inner_frac))
                v_left_des = outer
                v_right_des = inside
                mix_mode = "same_direction_right_turn"

            else:
                v_left_des = v_ref
                v_right_des = v_ref
                mix_mode = "straight"

        lidar_ratio, lidar_stop, lidar_mode = self.lidar_scale()

        if self.emergency_stop:
            lidar_ratio = 0.0
            lidar_stop = True
            lidar_mode = "emergency_stop_topic"

        v_left_des *= lidar_ratio
        v_right_des *= lidar_ratio

        left_rate = abs(self.pfloat("wheel_speed_rate")) if abs(v_left_des) >= abs(self.v_left_cmd_prev) else abs(self.pfloat("wheel_speed_decel_rate"))
        right_rate = abs(self.pfloat("wheel_speed_rate")) if abs(v_right_des) >= abs(self.v_right_cmd_prev) else abs(self.pfloat("wheel_speed_decel_rate"))

        v_left_cmd = approach(self.v_left_cmd_prev, v_left_des, left_rate * dt)
        v_right_cmd = approach(self.v_right_cmd_prev, v_right_des, right_rate * dt)

        if lidar_stop or self.emergency_stop:
            v_left_cmd = 0.0
            v_right_cmd = 0.0

        self.v_left_cmd_prev = v_left_cmd
        self.v_right_cmd_prev = v_right_cmd

        v_cmd = 0.5 * (v_left_cmd + v_right_cmd)
        omega_cmd = (v_right_cmd - v_left_cmd) / b

        if self.pbool("enable_calibration"):
            v_cmd = v_cmd / max(1e-6, self.pfloat("linear_cmd_scale"))
            omega_cmd = omega_cmd / max(1e-6, self.pfloat("angular_cmd_scale"))

        return {
            "v_left_des": v_left_des,
            "v_right_des": v_right_des,
            "v_left_cmd": v_left_cmd,
            "v_right_cmd": v_right_cmd,
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "mix_mode": mix_mode,
            "lidar_ratio": lidar_ratio,
            "lidar_stop": lidar_stop,
            "lidar_mode": lidar_mode,
        }

    def cmd_vel_conflict_detected(self):
        if not self.pbool("check_cmd_vel_conflict") or self.pbool("allow_cmd_vel_conflict"):
            return False, []

        infos = self.get_publishers_info_by_topic(self.cmd_vel_topic)
        names = []

        for info in infos:
            names.append(info.node_name if info.node_namespace == "/" else f"{info.node_namespace}/{info.node_name}")

        if self.cmd_vel_topic == "/cmd_vel" and len(infos) > 1:
            return True, names

        return False, names

    def maybe_publish_cmd(self, v, omega):
        enabled = self.is_enabled()
        conflict, publishers = self.cmd_vel_conflict_detected()

        if not enabled:
            return False, conflict, publishers, "enable_cmd_false"

        if conflict:
            if self.pbool("publish_zero_on_conflict"):
                self.cmd_pub.publish(self.make_twist(0.0, 0.0))
            return False, conflict, publishers, "cmd_vel_conflict"

        self.cmd_pub.publish(self.make_twist(v, omega))
        return True, conflict, publishers, "published"

    def control_loop(self):
        now = self.now_s()
        dt = max(now - self.prev_loop_time, 1e-3)
        self.prev_loop_time = now

        age = now - self.last_error_time if self.last_error_time > 0.0 else 999.0
        fresh = self.raw_valid and age <= self.pfloat("error_timeout_s")
        startup = self.first_valid_time > 0.0 and (now - self.first_valid_time) <= self.pfloat("startup_straight_s")
        soft_replan_active = now < self.soft_replan_until
        avoid_info = self.update_obstacle_avoidance(now, dt)

        outer = {
            "outer_mode": "none",
            "v_des": 0.0,
            "omega_des": 0.0,
            "omega_raw": 0.0,
            "omega_limit": 0.0,
            "curvature_from_error": 0.0,
            "center_zone": False,
            "near_zone": False,
            "large_error": False,
            "curve_confirmed": False,
            "slow_factor": 1.0,
            "k_pp_used": 0.0,
            "k_lat_used": 0.0,
            "k_theta_used": 0.0,
        }

        if avoid_info["mode"] == "avoid_reverse":
            e_used, theta_used = self.filtered_error(dt, False)
            mode = "avoid_reverse"
            v_des = -abs(self.pfloat("avoid_reverse_v"))
            omega_des = 0.0
            outer["outer_mode"] = mode

        elif startup:
            e_used, theta_used = self.filtered_error(dt, True)
            mode = "startup_straight"
            v_des = min(self.speed_from_fps(), self.pfloat("startup_v"))
            omega_des = 0.0
            outer["outer_mode"] = mode

        elif fresh:
            e_used, theta_used = self.filtered_error(dt, True)
            outer = self.compute_outer(e_used, theta_used)
            mode = outer["outer_mode"]
            v_des = outer["v_des"]
            omega_des = outer["omega_des"]

            if soft_replan_active:
                mode = "soft_replan_tracking"
                outer["outer_mode"] = mode
                v_des = min(v_des, self.pfloat("soft_replan_v_max"))
                omega_lim = abs(self.pfloat("soft_replan_omega_max"))
                omega_des = clamp(omega_des, -omega_lim, omega_lim)
                outer["omega_limit"] = min(abs(outer.get("omega_limit", omega_lim)), omega_lim)

        elif age <= self.pfloat("blind_hold_s"):
            e_used, theta_used = self.filtered_error(dt, False)
            mode = "blind_straight_hold"
            v_des = self.pfloat("v_blind")
            omega_des = 0.0
            outer["outer_mode"] = mode

        else:
            e_used, theta_used = self.filtered_error(dt, False)
            mode = "control_error_timeout"
            v_des = 0.0
            omega_des = 0.0
            outer["outer_mode"] = mode

        if avoid_info["active"] and mode not in ["control_error_timeout", "avoid_reverse"]:
            if mode == "center_cruise":
                mode = avoid_info["mode"]
                outer["outer_mode"] = mode

            v_des = min(v_des, self.pfloat("avoidance_v_max"))

            avoid_w_lim = abs(self.pfloat("avoidance_omega_max"))
            omega_des = clamp(omega_des, -avoid_w_lim, avoid_w_lim)
            outer["omega_limit"] = min(abs(outer.get("omega_limit", avoid_w_lim)), avoid_w_lim)

            min_w = abs(self.pfloat("avoidance_min_omega"))
            if self.avoidance_mode == "avoid_shift_out" and abs(omega_des) < min_w:
                omega_des = avoid_info["angular_sign"] * min_w

        v_rate = self.pfloat("v_rate_up") if v_des >= self.v_ref else self.pfloat("v_rate_down")

        if mode in ["center_cruise", "near_center_cascade", "startup_straight", "blind_straight_hold"]:
            omega_rate = self.pfloat("omega_rate_center")
        elif mode in ["curve_committed_fast_response", "avoid_shift_out", "avoid_bypass", "avoid_return", "avoid_reverse"]:
            omega_rate = self.pfloat("omega_rate_curve")
        else:
            omega_rate = self.pfloat("omega_rate_mid")

        self.v_ref = approach(self.v_ref, v_des, v_rate * dt)
        self.omega_ref = approach(self.omega_ref, omega_des, omega_rate * dt)
        self.publish_ref(self.v_ref, self.omega_ref)

        inner = self.mix_wheels(self.v_ref, self.omega_ref, dt)

        self.v_cmd = inner["v_cmd"]
        self.omega_cmd = inner["omega_cmd"]

        if mode == "control_error_timeout":
            self.v_cmd = 0.0
            self.omega_cmd = 0.0

        cmd_published, conflict, publishers, publish_reason = self.maybe_publish_cmd(self.v_cmd, self.omega_cmd)

        b = max(0.05, self.pfloat("track_width_m"))
        v_left_odom = self.odom_v - self.odom_omega * b * 0.5
        v_right_odom = self.odom_v + self.odom_omega * b * 0.5
        odom_timeout = self.last_odom_time < 0.0 or (now - self.last_odom_time) > 1.5

        self.publish_state({
            "node": "cascade_controller_avoid",
            "version": "cascade_controller_avoid_v1",
            "time": now,
            "enabled": self.is_enabled(),
            "param_enable_cmd": self.pbool("enable_cmd"),
            "runtime_enable": self.runtime_enable,
            "emergency_stop": self.emergency_stop,
            "cmd_published": cmd_published,
            "publish_reason": publish_reason,
            "cmd_vel_conflict": conflict,
            "cmd_vel_publishers": publishers,
            "mode": mode,
            "outer_mode": outer["outer_mode"],
            "mix_mode": inner["mix_mode"],
            "raw_valid": self.raw_valid,
            "raw_valid_reason": self.raw_valid_reason,
            "lane_state": self.raw_lane_state,
            "confidence": self.raw_confidence,
            "error_age_s": age,
            "intent_hint": self.intent_hint,
            "trajectory_hint": self.trajectory_hint,
            "planner_status_hint": self.planner_status_hint,
            "soft_replan_active": soft_replan_active,
            "soft_replan_reason": self.soft_replan_reason,
            "avoidance_mode": avoid_info["mode"],
            "avoidance_active": avoid_info["active"],
            "avoidance_offset_m": avoid_info["offset_m"],
            "avoidance_target_offset_m": avoid_info["target_offset_m"],
            "avoidance_direction": self.pstr("avoidance_direction"),
            "fps_est": self.fps_est,
            "epsilon_x_mm": self.raw_e_lat * 1000.0,
            "theta_rad": self.raw_e_theta,
            "lookahead_m": self.raw_lookahead,
            "kappa_m": self.raw_kappa,
            "e_f_m": self.e_f,
            "e_f_mm": self.e_f * 1000.0,
            "theta_f_rad": self.theta_f,
            "e_used_m": e_used,
            "e_used_mm": e_used * 1000.0,
            "theta_used_rad": theta_used,
            "de_f": self.de_f,
            "dtheta_f": self.dtheta_f,
            "curve_confirmed": outer["curve_confirmed"],
            "curve_count": self.curve_count,
            "curve_sign": self.curve_sign,
            "center_zone": outer["center_zone"],
            "near_zone": outer["near_zone"],
            "large_error": outer["large_error"],
            "curvature_from_error": outer["curvature_from_error"],
            "slow_factor": outer["slow_factor"],
            "k_pp_used": outer["k_pp_used"],
            "k_lat_used": outer["k_lat_used"],
            "k_theta_used": outer["k_theta_used"],
            "v_des": v_des,
            "omega_des": omega_des,
            "omega_raw": outer["omega_raw"],
            "omega_limit": outer["omega_limit"],
            "v_ref": self.v_ref,
            "omega_ref": self.omega_ref,
            "v_left_des": inner["v_left_des"],
            "v_right_des": inner["v_right_des"],
            "v_left_cmd": inner["v_left_cmd"],
            "v_right_cmd": inner["v_right_cmd"],
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "odom_v": self.odom_v,
            "odom_omega": self.odom_omega,
            "v_left_odom": v_left_odom,
            "v_right_odom": v_right_odom,
            "odom_timeout": odom_timeout,
            "odom_x": self.odom_x,
            "odom_y": self.odom_y,
            "odom_yaw": self.odom_yaw,
            "front_min_m": self.front_min if math.isfinite(self.front_min) else None,
            "lidar_ratio": inner["lidar_ratio"],
            "lidar_stop": inner["lidar_stop"],
            "lidar_mode": inner["lidar_mode"],
        })


def main(args=None):
    rclpy.init(args=args)
    node = CascadeControllerAvoid()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if bool(node.get_parameter("always_publish_stop_on_exit").value):
            node.publish_stop_burst()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
