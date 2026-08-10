#!/usr/bin/env python3

import json
import math
import signal
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
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


class PDControllerV1(Node):
    """
    PD_controller_v1

    Controller mới cho /avs/control_error của vision branch optimize.

    Mục tiêu:
      - Đường thẳng: hết lắc, omega gần 0 khi xe đã ở giữa làn.
      - Vào cua: phản ứng nhanh hơn nhưng không pivot/trượt quá mạnh.
      - Khi lệch lớn: giảm tốc và giới hạn omega để không văng qua làn khác.
      - Khi /avs/control_error nhảy bất thường ở giao lộ: giữ thẳng ngắn hạn.
    """

    def __init__(self):
        super().__init__("PD_controller_v1")

        # Topic
        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("lane_state_topic", "/avs/lane_state")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/avs/PD_controller_v1_debug")

        # Runtime
        self.declare_parameter("enable_motion", True)
        self.declare_parameter("control_rate_hz", 40.0)
        self.declare_parameter("error_timeout_s", 1.2)
        self.declare_parameter("blind_hold_s", 1.0)

        # Startup: tránh vừa đặt xe xuống là bẻ lái vì frame đầu chưa ổn định.
        self.declare_parameter("startup_straight_s", 0.45)
        self.declare_parameter("startup_v", 0.040)

        # Speed
        self.declare_parameter("v_max", 0.105)
        self.declare_parameter("v_center", 0.090)
        self.declare_parameter("v_min", 0.035)
        self.declare_parameter("v_curve_max", 0.055)
        self.declare_parameter("v_recover_max", 0.060)
        self.declare_parameter("v_large_error", 0.042)
        self.declare_parameter("v_blind", 0.035)

        # FPS adaptive speed
        self.declare_parameter("fps_init", 3.0)
        self.declare_parameter("fps_min", 0.8)
        self.declare_parameter("fps_max", 25.0)
        self.declare_parameter("fps_tau_s", 1.0)
        self.declare_parameter("target_distance_per_frame_m", 0.030)

        # Geometry
        self.declare_parameter("wheel_separation_m", 0.135)
        self.declare_parameter("max_delta_v", 0.070)
        self.declare_parameter("inner_wheel_min_fraction", 0.38)

        # Input convention
        self.declare_parameter("epsilon_sign", 1.0)
        self.declare_parameter("theta_sign", 1.0)
        self.declare_parameter("invert_angular", False)
        self.declare_parameter("x_bias_m", 0.0)

        # Filters
        self.declare_parameter("median_window", 5)
        self.declare_parameter("error_filter_tau_s", 0.42)
        self.declare_parameter("derivative_filter_tau_s", 0.80)

        # Deadband / zone
        self.declare_parameter("x_deadband_m", 0.014)
        self.declare_parameter("theta_deadband_rad", 0.025)

        self.declare_parameter("center_x_m", 0.030)
        self.declare_parameter("center_theta_rad", 0.060)

        self.declare_parameter("near_x_m", 0.065)
        self.declare_parameter("near_theta_rad", 0.140)

        self.declare_parameter("large_error_x_m", 0.150)
        self.declare_parameter("large_error_theta_rad", 0.420)

        # Curve detection
        self.declare_parameter("curve_enter_x_m", 0.085)
        self.declare_parameter("curve_enter_theta_rad", 0.180)
        self.declare_parameter("curve_enter_kappa", 0.75)
        self.declare_parameter("curve_confirm_frames", 2)
        self.declare_parameter("curve_release_frames", 4)

        # Outlier / jump guard
        self.declare_parameter("max_abs_x_m", 0.42)
        self.declare_parameter("max_abs_theta_rad", 1.20)

        self.declare_parameter("jump_guard_enable", True)
        self.declare_parameter("jump_x_m", 0.160)
        self.declare_parameter("jump_theta_rad", 0.420)
        self.declare_parameter("jump_hold_s", 0.55)
        self.declare_parameter("jump_hold_v", 0.040)

        # PD gains
        # Gần tâm: gain nhỏ để hết lắc.
        self.declare_parameter("k_lat_near", 0.28)
        self.declare_parameter("k_theta_near", 0.16)

        # Vùng tracking thường.
        self.declare_parameter("k_lat", 0.62)
        self.declare_parameter("k_theta", 0.24)

        # Vùng cua: tăng P ngang và heading để rẽ sớm hơn.
        self.declare_parameter("k_lat_curve", 0.86)
        self.declare_parameter("k_theta_curve", 0.30)

        # Vùng lệch quá lớn: không tăng quá mạnh để tránh văng qua làn.
        self.declare_parameter("k_lat_large", 0.46)
        self.declare_parameter("k_theta_large", 0.22)

        # Derivative nhẹ, chủ yếu để dập dao động.
        self.declare_parameter("kd_lat", 0.010)
        self.declare_parameter("kd_theta", 0.014)

        # Preview nhỏ theo lookahead để vào cua sớm hơn.
        # Set 0.0 nếu muốn PD tuyệt đối.
        self.declare_parameter("k_preview", 0.18)

        # Omega limits
        self.declare_parameter("omega_center_max", 0.000)
        self.declare_parameter("omega_near_max", 0.035)
        self.declare_parameter("omega_mid_max", 0.200)
        self.declare_parameter("omega_curve_max", 0.420)
        self.declare_parameter("omega_large_error_max", 0.260)
        self.declare_parameter("omega_abs_max", 0.480)
        self.declare_parameter("omega_deadband", 0.006)

        # Rate limits
        self.declare_parameter("v_rate_up", 0.14)
        self.declare_parameter("v_rate_down", 0.36)
        self.declare_parameter("omega_rate_center", 0.22)
        self.declare_parameter("omega_rate_mid", 0.58)
        self.declare_parameter("omega_rate_curve", 0.95)

        # Lookahead / curvature
        # ld_min_m phải <= lookahead_d_mm mà perception phát (hiện 0.14 m theo
        # docker-compose.prod.yml): epsilon_x đo tại khoảng cách đó, clamp ld
        # lên cao hơn sẽ làm preview 2*e_x/ld^2 yếu đi sai lệch.
        self.declare_parameter("ld_min_m", 0.08)
        self.declare_parameter("ld_max_m", 0.85)
        self.declare_parameter("default_lookahead_m", 0.42)
        self.declare_parameter("max_abs_kappa", 3.0)

        # Speed reduction
        self.declare_parameter("k_slow_x", 1.90)
        self.declare_parameter("k_slow_theta", 1.10)
        self.declare_parameter("k_slow_kappa", 0.16)

        # Safety against multiple /cmd_vel publishers
        self.declare_parameter("check_cmd_vel_conflict", True)
        self.declare_parameter("allow_cmd_vel_conflict", False)
        self.declare_parameter("publish_zero_on_conflict", True)

        self.declare_parameter("stop_burst_count", 40)
        self.declare_parameter("stop_burst_dt", 0.025)

        self.control_error_topic = str(self.get_parameter("control_error_topic").value)
        self.lane_state_topic = str(self.get_parameter("lane_state_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.create_subscription(String, self.control_error_topic, self.control_error_cb, 10)
        self.create_subscription(String, self.lane_state_topic, self.lane_state_cb, 10)

        self.last_error_time = -1.0
        self.first_valid_time = -1.0
        self.last_frame_time = None

        self.raw_valid = False
        self.raw_valid_reason = "waiting"
        self.raw_lane_state = ""
        self.raw_confidence = 0.0

        self.raw_e_x = 0.0
        self.raw_theta = 0.0
        self.raw_ld = float(self.get_parameter("default_lookahead_m").value)
        self.raw_kappa = 0.0

        self.last_accepted_raw_x = None
        self.last_accepted_raw_theta = None

        self.jump_hold_until = -1.0
        self.jump_reason = "none"

        self.lane_state_debug = {}
        self.last_lane_state_time = -1.0

        mw = int(self.get_parameter("median_window").value)
        self.x_buf = deque(maxlen=max(1, mw))
        self.theta_buf = deque(maxlen=max(1, mw))

        self.e_x_f = 0.0
        self.theta_f = 0.0
        self.de_x_f = 0.0
        self.dtheta_f = 0.0

        self.prev_e_x_used = 0.0
        self.prev_theta_used = 0.0

        self.fps_est = float(self.get_parameter("fps_init").value)

        self.curve_confirmed = False
        self.curve_count = 0
        self.curve_release_count = 0
        self.curve_sign = 0

        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.v_ref = 0.0
        self.omega_ref = 0.0

        self.prev_loop_time = time.time()

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        hz = max(5.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / hz, self.control_loop)

        self.get_logger().info("PD_controller_v1 started")
        self.get_logger().info(f"Subscribe control_error: {self.control_error_topic}")
        self.get_logger().info(f"Subscribe lane_state   : {self.lane_state_topic}")
        self.get_logger().info(f"Publish cmd_vel        : {self.cmd_vel_topic}")
        self.get_logger().info(f"Publish debug          : {self.debug_topic}")

    def now(self):
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

    def publish_stop_burst(self):
        stop = self.make_cmd(0.0, 0.0)
        count = max(5, self.pint("stop_burst_count"))
        dt = max(0.005, self.pfloat("stop_burst_dt"))

        for _ in range(count):
            self.cmd_pub.publish(stop)
            time.sleep(dt)

        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.v_ref = 0.0
        self.omega_ref = 0.0

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Sending zero /cmd_vel burst.")
        self.publish_stop_burst()
        if rclpy.ok():
            rclpy.shutdown()

    def publish_debug(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def cmd_vel_conflict_detected(self):
        if not self.pbool("check_cmd_vel_conflict"):
            return False, []

        if self.pbool("allow_cmd_vel_conflict"):
            return False, []

        infos = self.get_publishers_info_by_topic(self.cmd_vel_topic)
        names = []

        for info in infos:
            ns = info.node_namespace
            if ns == "/":
                names.append(info.node_name)
            else:
                names.append(f"{ns}/{info.node_name}")

        # Node này cũng là 1 publisher. Nếu >1 nghĩa là có publisher khác.
        if self.cmd_vel_topic == "/cmd_vel" and len(infos) > 1:
            return True, names

        return False, names

    def publish_cmd(self, v, omega):
        conflict, publishers = self.cmd_vel_conflict_detected()

        if conflict:
            if self.pbool("publish_zero_on_conflict"):
                self.cmd_pub.publish(self.make_cmd(0.0, 0.0))
            return False, conflict, publishers, "cmd_vel_conflict"

        if not self.pbool("enable_motion"):
            self.cmd_pub.publish(self.make_cmd(0.0, 0.0))
            return False, conflict, publishers, "enable_motion_false"

        self.cmd_pub.publish(self.make_cmd(v, omega))
        return True, conflict, publishers, "published"

    def lane_state_cb(self, msg):
        try:
            d = json.loads(msg.data)
            if isinstance(d, dict):
                self.lane_state_debug = d
                self.last_lane_state_time = self.now()
        except Exception:
            pass

    def update_fps(self, now, data):
        fps_msg = finite_float(
            data.get("fps", data.get("fps_est", data.get("vision_fps", None))),
            None
        )

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
        e_x_mm = finite_float(data.get("epsilon_x_mm", data.get("x_mm", 0.0)), 0.0)
        e_x = e_x_mm / 1000.0

        e_x = self.pfloat("epsilon_sign") * e_x + self.pfloat("x_bias_m")

        default_ld_mm = self.pfloat("default_lookahead_m") * 1000.0
        ld_mm = finite_float(
            data.get(
                "epsilon_y_mm",
                data.get("lookahead_d_mm", data.get("y_mm", default_ld_mm))
            ),
            default_ld_mm
        )

        ld = clamp(abs(ld_mm) / 1000.0, self.pfloat("ld_min_m"), self.pfloat("ld_max_m"))

        theta = finite_float(
            data.get("theta_rad", data.get("heading_error_rad", data.get("e_theta_rad", 0.0))),
            0.0
        )
        theta = self.pfloat("theta_sign") * theta

        curvature_inv_mm = finite_float(data.get("curvature_inv_mm", 0.0), 0.0)

        # Trong hệ hiện tại, code cũ đang dùng curvature_inv_mm * 1000.
        # Clamp để tránh một frame curvature phi lý làm giảm tốc hoặc rẽ quá mạnh.
        kappa_m = clamp(curvature_inv_mm * 1000.0, -self.pfloat("max_abs_kappa"), self.pfloat("max_abs_kappa"))

        confidence = finite_float(data.get("confidence", data.get("conf", data.get("prob", 1.0))), 1.0)
        lane_state = str(data.get("lane_state", "")).upper()

        raw_valid = parse_bool(data.get("valid", True), True)
        lane_valid = parse_bool(data.get("lane_valid", True), True)

        if lane_state in ["LOST", "INVALID", "NO_LANE", "NONE"]:
            return False, "lane_state_invalid", e_x, theta, ld, kappa_m, confidence, lane_state

        if abs(e_x) > self.pfloat("max_abs_x_m"):
            return False, "x_outlier", e_x, theta, ld, kappa_m, confidence, lane_state

        if abs(theta) > self.pfloat("max_abs_theta_rad"):
            return False, "theta_outlier", e_x, theta, ld, kappa_m, confidence, lane_state

        valid = raw_valid and lane_valid
        if lane_state == "FOLLOW_MAIN":
            valid = True

        return valid, "ok" if valid else "invalid_flags", e_x, theta, ld, kappa_m, confidence, lane_state

    def jump_guard_triggered(self, e_x, theta):
        if not self.pbool("jump_guard_enable"):
            return False, "disabled"

        if self.last_accepted_raw_x is None or self.last_accepted_raw_theta is None:
            return False, "first_valid"

        dx = abs(e_x - self.last_accepted_raw_x)
        dt = abs(theta - self.last_accepted_raw_theta)

        if dx > self.pfloat("jump_x_m") or dt > self.pfloat("jump_theta_rad"):
            return True, f"jump dx={dx:.3f}, dtheta={dt:.3f}"

        return False, "normal"

    def control_error_cb(self, msg):
        now = self.now()

        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid /avs/control_error JSON: {exc}")
            return

        valid, reason, e_x, theta, ld, kappa_m, confidence, lane_state = self.extract_control_error(data)
        self.update_fps(now, data)

        self.raw_valid = valid
        self.raw_valid_reason = reason
        self.raw_e_x = e_x
        self.raw_theta = theta
        self.raw_ld = ld
        self.raw_kappa = kappa_m
        self.raw_confidence = confidence
        self.raw_lane_state = lane_state

        self.last_error_time = now

        if not valid:
            return

        if self.first_valid_time < 0.0:
            self.first_valid_time = now

        jump, jump_reason = self.jump_guard_triggered(e_x, theta)

        if jump:
            self.jump_hold_until = now + self.pfloat("jump_hold_s")
            self.jump_reason = jump_reason
            return

        self.jump_reason = "none"

        self.last_accepted_raw_x = e_x
        self.last_accepted_raw_theta = theta

        self.update_median_window()
        self.x_buf.append(e_x)
        self.theta_buf.append(theta)

    def update_median_window(self):
        mw = max(1, self.pint("median_window"))

        if self.x_buf.maxlen != mw:
            old_x = list(self.x_buf)[-mw:]
            old_t = list(self.theta_buf)[-mw:]

            self.x_buf = deque(old_x, maxlen=mw)
            self.theta_buf = deque(old_t, maxlen=mw)

    def filtered_error(self, dt, use_measurement):
        if use_measurement and len(self.x_buf) > 0:
            target_x = statistics.median(self.x_buf)
            target_theta = statistics.median(self.theta_buf)
        else:
            target_x = 0.0
            target_theta = 0.0

        alpha = self.alpha_from_tau(dt, self.pfloat("error_filter_tau_s"))

        self.e_x_f = (1.0 - alpha) * self.e_x_f + alpha * target_x
        self.theta_f = (1.0 - alpha) * self.theta_f + alpha * target_theta

        if abs(self.e_x_f) < self.pfloat("x_deadband_m"):
            e_x_used = 0.0
        else:
            e_x_used = self.e_x_f

        if abs(self.theta_f) < self.pfloat("theta_deadband_rad"):
            theta_used = 0.0
        else:
            theta_used = self.theta_f

        raw_de_x = (e_x_used - self.prev_e_x_used) / max(dt, 1e-3)
        raw_dtheta = (theta_used - self.prev_theta_used) / max(dt, 1e-3)

        self.prev_e_x_used = e_x_used
        self.prev_theta_used = theta_used

        d_alpha = self.alpha_from_tau(dt, self.pfloat("derivative_filter_tau_s"))
        self.de_x_f = (1.0 - d_alpha) * self.de_x_f + d_alpha * raw_de_x
        self.dtheta_f = (1.0 - d_alpha) * self.dtheta_f + d_alpha * raw_dtheta

        return e_x_used, theta_used

    def update_curve_state(self, e_x, theta, kappa):
        curve_candidate = (
            abs(e_x) >= self.pfloat("curve_enter_x_m")
            or abs(theta) >= self.pfloat("curve_enter_theta_rad")
            or abs(kappa) >= self.pfloat("curve_enter_kappa")
        )

        sign_src = theta if abs(theta) > 0.06 else e_x
        sign = 1 if sign_src > 0 else -1 if sign_src < 0 else 0

        if curve_candidate and sign != 0:
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

    def compute_tracking(self, e_x, theta):
        kappa = self.raw_kappa
        ld = max(self.pfloat("ld_min_m"), self.raw_ld)

        curve = self.update_curve_state(e_x, theta, kappa)

        center = (
            abs(self.e_x_f) <= self.pfloat("center_x_m")
            and abs(self.theta_f) <= self.pfloat("center_theta_rad")
        )

        near = (
            abs(self.e_x_f) <= self.pfloat("near_x_m")
            and abs(self.theta_f) <= self.pfloat("near_theta_rad")
        )

        large_error = (
            abs(self.e_x_f) >= self.pfloat("large_error_x_m")
            or abs(self.theta_f) >= self.pfloat("large_error_theta_rad")
        )

        v_base = min(self.speed_from_fps(), self.pfloat("v_max"))

        slow_factor = math.exp(
            -self.pfloat("k_slow_x") * abs(self.e_x_f)
            -self.pfloat("k_slow_theta") * abs(self.theta_f)
            -self.pfloat("k_slow_kappa") * abs(kappa)
        )

        if center:
            mode = "center_cruise"
            v_des = min(v_base, self.pfloat("v_center"))
            omega_limit = self.pfloat("omega_center_max")
            omega_target = 0.0
            k_lat = 0.0
            k_theta = 0.0

        elif near and not curve:
            mode = "near_center_pd"
            v_des = min(v_base, self.pfloat("v_recover_max"))
            omega_limit = self.pfloat("omega_near_max")
            k_lat = self.pfloat("k_lat_near")
            k_theta = self.pfloat("k_theta_near")

            omega_target = (
                -k_lat * e_x
                -k_theta * theta
                -self.pfloat("kd_lat") * self.de_x_f
                -self.pfloat("kd_theta") * self.dtheta_f
            )

        elif large_error:
            mode = "large_error_slow_recover"
            v_des = self.pfloat("v_large_error")
            omega_limit = self.pfloat("omega_large_error_max")
            k_lat = self.pfloat("k_lat_large")
            k_theta = self.pfloat("k_theta_large")

            omega_target = (
                -k_lat * e_x
                -k_theta * theta
                -self.pfloat("kd_lat") * self.de_x_f
                -self.pfloat("kd_theta") * self.dtheta_f
            )

        elif curve:
            mode = "curve_pd_fast_response"
            v_des = clamp(v_base * slow_factor, self.pfloat("v_min"), self.pfloat("v_curve_max"))
            omega_limit = self.pfloat("omega_curve_max")
            k_lat = self.pfloat("k_lat_curve")
            k_theta = self.pfloat("k_theta_curve")

            # Preview nhỏ giúp vào cua sớm hơn.
            preview = -self.pfloat("k_preview") * v_des * (2.0 * e_x / max(ld * ld, 1e-4))

            omega_target = (
                -k_lat * e_x
                -k_theta * theta
                -self.pfloat("kd_lat") * self.de_x_f
                -self.pfloat("kd_theta") * self.dtheta_f
                + preview
            )

        else:
            mode = "mid_pd_tracking"
            v_des = clamp(v_base * slow_factor, self.pfloat("v_min"), self.pfloat("v_recover_max"))
            omega_limit = self.pfloat("omega_mid_max")
            k_lat = self.pfloat("k_lat")
            k_theta = self.pfloat("k_theta")

            preview = -0.5 * self.pfloat("k_preview") * v_des * (2.0 * e_x / max(ld * ld, 1e-4))

            omega_target = (
                -k_lat * e_x
                -k_theta * theta
                -self.pfloat("kd_lat") * self.de_x_f
                -self.pfloat("kd_theta") * self.dtheta_f
                + preview
            )

        if self.pbool("invert_angular"):
            omega_target = -omega_target

        if abs(omega_target) < self.pfloat("omega_deadband"):
            omega_target = 0.0

        omega_limit = min(abs(omega_limit), self.pfloat("omega_abs_max"))

        # Giới hạn theo chênh lệch vận tốc hai bên để giảm trượt/pivot.
        B = max(0.05, self.pfloat("wheel_separation_m"))
        max_delta_v = abs(self.pfloat("max_delta_v"))
        omega_from_delta = max_delta_v / B

        inner_frac = clamp(self.pfloat("inner_wheel_min_fraction"), 0.0, 0.90)
        omega_from_inner = 2.0 * max(v_des, 0.01) * (1.0 - inner_frac) / B

        omega_limit_final = min(omega_limit, omega_from_delta, max(0.08, omega_from_inner))

        omega_target = clamp(omega_target, -omega_limit_final, omega_limit_final)

        # Chống đổi dấu liên tục gần tâm.
        if near and not curve and omega_target * self.omega_ref < 0.0:
            omega_target = 0.0

        return {
            "mode": mode,
            "v_des": v_des,
            "omega_des": omega_target,
            "omega_limit": omega_limit_final,
            "curve_confirmed": curve,
            "center_zone": center,
            "near_zone": near,
            "large_error": large_error,
            "slow_factor": slow_factor,
            "k_lat_used": k_lat,
            "k_theta_used": k_theta,
        }

    def control_loop(self):
        now = self.now()
        dt = max(now - self.prev_loop_time, 1e-3)
        self.prev_loop_time = now

        age = now - self.last_error_time if self.last_error_time > 0.0 else 999.0
        fresh = self.raw_valid and age <= self.pfloat("error_timeout_s")

        jump_hold_active = now < self.jump_hold_until

        startup_active = (
            self.first_valid_time > 0.0
            and now - self.first_valid_time <= self.pfloat("startup_straight_s")
        )

        e_x_used = 0.0
        theta_used = 0.0

        info = {
            "mode": "none",
            "v_des": 0.0,
            "omega_des": 0.0,
            "omega_limit": 0.0,
            "curve_confirmed": False,
            "center_zone": False,
            "near_zone": False,
            "large_error": False,
            "slow_factor": 1.0,
            "k_lat_used": 0.0,
            "k_theta_used": 0.0,
        }

        if jump_hold_active:
            e_x_used, theta_used = self.filtered_error(dt, False)
            mode = "jump_guard_straight"
            v_des = self.pfloat("jump_hold_v")
            omega_des = 0.0
            info["mode"] = mode

        elif startup_active:
            e_x_used, theta_used = self.filtered_error(dt, True)
            mode = "startup_straight"
            v_des = min(self.speed_from_fps(), self.pfloat("startup_v"))
            omega_des = 0.0
            info["mode"] = mode

        elif fresh:
            e_x_used, theta_used = self.filtered_error(dt, True)
            info = self.compute_tracking(e_x_used, theta_used)
            mode = info["mode"]
            v_des = info["v_des"]
            omega_des = info["omega_des"]

        elif age <= self.pfloat("blind_hold_s"):
            e_x_used, theta_used = self.filtered_error(dt, False)
            mode = "blind_straight_hold"
            v_des = self.pfloat("v_blind")
            omega_des = 0.0
            info["mode"] = mode

        else:
            e_x_used, theta_used = self.filtered_error(dt, False)
            mode = "control_error_timeout"
            v_des = 0.0
            omega_des = 0.0
            info["mode"] = mode

        if v_des >= self.v_ref:
            v_rate = self.pfloat("v_rate_up")
        else:
            v_rate = self.pfloat("v_rate_down")

        if mode in ["center_cruise", "near_center_pd", "startup_straight", "blind_straight_hold", "jump_guard_straight"]:
            omega_rate = self.pfloat("omega_rate_center")
        elif mode == "curve_pd_fast_response":
            omega_rate = self.pfloat("omega_rate_curve")
        else:
            omega_rate = self.pfloat("omega_rate_mid")

        self.v_ref = approach(self.v_ref, v_des, v_rate * dt)
        self.omega_ref = approach(self.omega_ref, omega_des, omega_rate * dt)

        self.v_cmd = approach(self.v_cmd, self.v_ref, v_rate * dt)
        self.omega_cmd = approach(self.omega_cmd, self.omega_ref, omega_rate * dt)

        published, conflict, publishers, publish_reason = self.publish_cmd(self.v_cmd, self.omega_cmd)

        B = max(0.05, self.pfloat("wheel_separation_m"))

        v_left_est = self.v_cmd - self.omega_cmd * B * 0.5
        v_right_est = self.v_cmd + self.omega_cmd * B * 0.5
        delta_v_cmd = v_right_est - v_left_est

        self.publish_debug({
            "node": "PD_controller_v1",
            "version": "PD_controller_v1_center_stable_curve_fast",
            "time": now,

            "enable_motion": self.pbool("enable_motion"),
            "cmd_published": published,
            "publish_reason": publish_reason,
            "cmd_vel_conflict": conflict,
            "cmd_vel_publishers": publishers,

            "mode": mode,
            "raw_valid": self.raw_valid,
            "raw_valid_reason": self.raw_valid_reason,
            "lane_state": self.raw_lane_state,
            "confidence": self.raw_confidence,
            "error_age_s": age,

            "jump_hold_active": jump_hold_active,
            "jump_reason": self.jump_reason,

            "fps_est": self.fps_est,

            "epsilon_x_mm": self.raw_e_x * 1000.0,
            "theta_rad": self.raw_theta,
            "L_d_m": self.raw_ld,
            "kappa_m": self.raw_kappa,

            "e_x_f_m": self.e_x_f,
            "e_x_f_mm": self.e_x_f * 1000.0,
            "theta_f_rad": self.theta_f,

            "e_x_used_m": e_x_used,
            "e_x_used_mm": e_x_used * 1000.0,
            "theta_used_rad": theta_used,

            "de_x_f": self.de_x_f,
            "dtheta_f": self.dtheta_f,

            "curve_confirmed": info["curve_confirmed"],
            "curve_count": self.curve_count,
            "curve_sign": self.curve_sign,
            "center_zone": info["center_zone"],
            "near_zone": info["near_zone"],
            "large_error": info["large_error"],

            "slow_factor": info["slow_factor"],
            "k_lat_used": info["k_lat_used"],
            "k_theta_used": info["k_theta_used"],

            "v_des": v_des,
            "omega_des": omega_des,
            "v_ref": self.v_ref,
            "omega_ref": self.omega_ref,
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "omega_limit": info["omega_limit"],

            "delta_v_cmd": delta_v_cmd,
            "v_left_est": v_left_est,
            "v_right_est": v_right_est,

            "lane_state_debug_age_s": now - self.last_lane_state_time if self.last_lane_state_time > 0 else -1.0,
            "lane_state_debug": self.lane_state_debug,
        })


def main(args=None):
    rclpy.init(args=args)
    node = PDControllerV1()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().warn("Shutdown: sending zero /cmd_vel burst.")
        node.publish_stop_burst()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
