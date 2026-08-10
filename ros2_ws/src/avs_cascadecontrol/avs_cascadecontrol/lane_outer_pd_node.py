#!/usr/bin/env python3

import json
import math
import signal
import time

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
    return bool(value)


class LaneOuterPDNode(Node):
    def __init__(self):
        super().__init__("lane_outer_pd_node")

        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("secondary_control_error_topic", "/control_error")
        self.declare_parameter("lane_ref_topic", "/avs/lane_ref_cmd")
        self.declare_parameter("lane_state_topic", "/avs/lane_pd_state")

        self.declare_parameter("outer_hz", 25.0)
        self.declare_parameter("outer_timeout_s", 2.0)
        self.declare_parameter("stale_hold_s", 0.55)

        self.declare_parameter("fps_init", 4.0)
        self.declare_parameter("fps_min", 0.8)
        self.declare_parameter("fps_max", 20.0)
        self.declare_parameter("fps_tau_s", 0.8)

        # Xe đi nhanh/chậm theo FPS thực tế.
        self.declare_parameter("target_distance_per_frame_m", 0.020)

        self.declare_parameter("v_max", 0.105)
        self.declare_parameter("v_min", 0.045)
        self.declare_parameter("v_turn_min", 0.055)
        self.declare_parameter("v_intersection", 0.045)

        self.declare_parameter("omega_max", 0.62)

        self.declare_parameter("lookahead_default_m", 0.42)
        self.declare_parameter("lookahead_min_m", 0.22)
        self.declare_parameter("lookahead_max_m", 0.85)

        self.declare_parameter("x_deadband_m", 0.018)
        self.declare_parameter("theta_deadband_rad", 0.018)

        # Nếu xe luôn đi lệch một bên, chỉnh x_bias_m.
        # x_bias_m > 0 làm xe sửa như thể lane nằm bên phải hơn.
        # x_bias_m < 0 làm xe sửa như thể lane nằm bên trái hơn.
        self.declare_parameter("x_bias_m", 0.0)

        # Loại bỏ sai số vision quá bất thường.
        self.declare_parameter("max_abs_e_lat_m", 0.42)
        self.declare_parameter("max_abs_theta_rad", 0.90)
        self.declare_parameter("min_valid_lookahead_m", 0.18)

        # Qua ngã tư: không quay vòng tìm lane, đi thẳng chậm một đoạn.
        self.declare_parameter("intersection_hold_s", 1.20)

        self.declare_parameter("error_filter_tau_s", 0.28)
        self.declare_parameter("derivative_filter_tau_s", 0.50)

        self.declare_parameter("k_pp", 1.75)
        self.declare_parameter("k_lat", 0.62)
        self.declare_parameter("kd_lat", 0.008)
        self.declare_parameter("k_theta", 0.18)
        self.declare_parameter("kd_theta", 0.004)

        self.declare_parameter("k_slow_lat", 1.50)
        self.declare_parameter("k_slow_theta", 0.55)
        self.declare_parameter("k_slow_curvature", 0.040)

        self.declare_parameter("v_ref_rate", 0.16)
        self.declare_parameter("omega_ref_rate", 0.95)

        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("invert_angular", False)

        self.declare_parameter("test_mode", False)
        self.declare_parameter("test_hz", 4.0)
        self.declare_parameter("test_e_lat_amp_m", 0.08)
        self.declare_parameter("test_e_theta_amp_rad", 0.05)

        self.control_error_topic = str(self.get_parameter("control_error_topic").value)
        self.secondary_control_error_topic = str(self.get_parameter("secondary_control_error_topic").value)
        self.lane_ref_topic = str(self.get_parameter("lane_ref_topic").value)
        self.lane_state_topic = str(self.get_parameter("lane_state_topic").value)

        self.ref_pub = self.create_publisher(Twist, self.lane_ref_topic, 10)
        self.state_pub = self.create_publisher(String, self.lane_state_topic, 10)

        self.create_subscription(String, self.control_error_topic, self.control_error_callback, 10)

        if self.secondary_control_error_topic and self.secondary_control_error_topic != self.control_error_topic:
            self.create_subscription(String, self.secondary_control_error_topic, self.control_error_callback, 10)

        now = self.now_s()

        self.last_rx_time = -1.0
        self.last_valid_lane_time = -1.0
        self.last_frame_time = None

        self.raw_valid = False
        self.raw_lane_state = ""
        self.raw_confidence = 1.0
        self.raw_e_lat = 0.0
        self.raw_e_theta = 0.0
        self.raw_lookahead = float(self.get_parameter("lookahead_default_m").value)
        self.raw_lat_source = "none"
        self.raw_theta_source = "none"
        self.raw_lookahead_source = "default"

        self.e_lat_f = 0.0
        self.e_theta_f = 0.0
        self.de_lat_f = 0.0
        self.de_theta_f = 0.0
        self.prev_e_lat_used = 0.0
        self.prev_e_theta_used = 0.0

        self.fps_est = float(self.get_parameter("fps_init").value)

        self.v_ref_prev = 0.0
        self.omega_ref_prev = 0.0
        self.prev_loop_time = now

        outer_hz = max(5.0, float(self.get_parameter("outer_hz").value))
        self.create_timer(1.0 / outer_hz, self.control_loop)
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("test_hz").value)), self.test_timer)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.get_logger().info("lane_outer_pd_node adaptive v2 started")
        self.get_logger().info(f"Subscribe primary:   {self.control_error_topic}")
        self.get_logger().info(f"Subscribe secondary: {self.secondary_control_error_topic}")
        self.get_logger().info(f"Publish lane ref:    {self.lane_ref_topic}")
        self.get_logger().info(f"Publish state:       {self.lane_state_topic}")

    def now_s(self):
        return time.time()

    def signal_handler(self, signum, frame):
        self.publish_ref(0.0, 0.0)
        if rclpy.ok():
            rclpy.shutdown()

    def publish_ref(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.ref_pub.publish(msg)

    def publish_state(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def alpha_from_tau(self, dt, tau):
        tau = max(1e-3, float(tau))
        return 1.0 - math.exp(-dt / tau)

    def test_timer(self):
        if not bool(self.get_parameter("test_mode").value):
            return

        t = self.now_s()
        e_lat = float(self.get_parameter("test_e_lat_amp_m").value) * math.sin(0.7 * t)
        e_theta = float(self.get_parameter("test_e_theta_amp_rad").value) * math.sin(0.45 * t)

        fake = {
            "valid": True,
            "lateral_error_m": e_lat,
            "heading_error_rad": e_theta,
            "epsilon_y_mm": 450.0,
            "confidence": 1.0,
            "lane_state": "TEST_MODE",
        }

        msg = String()
        msg.data = json.dumps(fake)
        self.control_error_callback(msg)

    def extract_errors(self, data):
        e_lat = None
        lat_source = "none"

        for key in ["lateral_error_m", "lateral_error", "e_lat_m", "x_error_m"]:
            if key in data:
                e_lat = finite_float(data.get(key), None)
                lat_source = key
                break

        if e_lat is None:
            for key in ["epsilon_x_mm", "x_mm"]:
                if key in data:
                    x_mm = finite_float(data.get(key), None)
                    if x_mm is not None:
                        e_lat = x_mm / 1000.0
                        lat_source = key
                        break

        if e_lat is None:
            e_lat = 0.0

        # Bias để chỉnh tâm làn thực nghiệm.
        e_lat += float(self.get_parameter("x_bias_m").value)

        e_theta = None
        theta_source = "none"

        for key in ["heading_error_rad", "heading_error", "e_theta_rad", "theta_error_rad", "theta_rad"]:
            if key in data:
                e_theta = finite_float(data.get(key), None)
                theta_source = key
                break

        if e_theta is None:
            e_theta = 0.0

        lookahead = None
        lookahead_source = "default"

        for key in ["lookahead_m", "lookahead_d_m", "epsilon_y_m", "target_y_m"]:
            if key in data:
                lookahead = finite_float(data.get(key), None)
                lookahead_source = key
                break

        if lookahead is None:
            for key in ["lookahead_d_mm", "epsilon_y_mm", "target_y_mm"]:
                if key in data:
                    y_mm = finite_float(data.get(key), None)
                    if y_mm is not None:
                        lookahead = y_mm / 1000.0
                        lookahead_source = key
                        break

        if lookahead is None:
            lookahead = float(self.get_parameter("lookahead_default_m").value)

        lookahead = clamp(
            lookahead,
            float(self.get_parameter("lookahead_min_m").value),
            float(self.get_parameter("lookahead_max_m").value),
        )

        return float(e_lat), float(e_theta), float(lookahead), lat_source, theta_source, lookahead_source

    def extract_valid(self, data, e_lat, e_theta, lookahead):
        lane_state = str(data.get("lane_state", "")).upper()

        if lane_state in ["LOST", "INVALID", "NO_LANE", "NONE"]:
            return False, 0.0, lane_state, "lane_state_invalid"

        valid = parse_bool(data.get("valid", True), True)
        lane_valid = parse_bool(data.get("lane_valid", True), True)
        confidence = finite_float(
            data.get("confidence", data.get("conf", data.get("prob", 1.0))),
            1.0,
        )

        min_conf = float(self.get_parameter("min_confidence").value)

        if abs(e_lat) > float(self.get_parameter("max_abs_e_lat_m").value):
            return False, confidence, lane_state, "e_lat_outlier"

        if abs(e_theta) > float(self.get_parameter("max_abs_theta_rad").value):
            return False, confidence, lane_state, "theta_outlier"

        if lookahead < float(self.get_parameter("min_valid_lookahead_m").value):
            return False, confidence, lane_state, "lookahead_too_near"

        if lane_state == "FOLLOW_MAIN":
            return True, confidence, lane_state, "ok_follow_main"

        ok = valid and lane_valid and confidence >= min_conf
        return ok, confidence, lane_state, "ok" if ok else "invalid_flags"

    def update_fps_from_frame(self, now):
        if self.last_frame_time is None:
            self.last_frame_time = now
            return

        dt_frame = max(now - self.last_frame_time, 1e-3)
        self.last_frame_time = now

        fps_now = 1.0 / dt_frame
        fps_now = clamp(
            fps_now,
            float(self.get_parameter("fps_min").value),
            float(self.get_parameter("fps_max").value),
        )

        tau = float(self.get_parameter("fps_tau_s").value)
        alpha = self.alpha_from_tau(dt_frame, tau)
        self.fps_est = (1.0 - alpha) * self.fps_est + alpha * fps_now

    def control_error_callback(self, msg):
        now = self.now_s()

        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid JSON on control_error: {exc}")
            return

        e_lat, e_theta, lookahead, lat_source, theta_source, lookahead_source = self.extract_errors(data)
        valid, confidence, lane_state, valid_reason = self.extract_valid(data, e_lat, e_theta, lookahead)

        self.update_fps_from_frame(now)

        self.raw_valid = valid
        self.raw_valid_reason = valid_reason
        self.raw_lane_state = lane_state
        self.raw_confidence = confidence
        self.raw_e_lat = e_lat
        self.raw_e_theta = e_theta
        self.raw_lookahead = lookahead
        self.raw_lat_source = lat_source
        self.raw_theta_source = theta_source
        self.raw_lookahead_source = lookahead_source

        self.last_rx_time = now

        if valid:
            self.last_valid_lane_time = now

    def control_loop(self):
        now = self.now_s()
        dt = max(now - self.prev_loop_time, 1e-3)
        self.prev_loop_time = now

        if self.last_rx_time < 0.0:
            self.publish_ref(0.0, 0.0)
            self.publish_state({
                "valid": False,
                "mode": "waiting_control_error",
                "fps_est": self.fps_est,
                "v_ref": 0.0,
                "omega_ref": 0.0,
            })
            return

        age = now - self.last_rx_time
        time_since_valid = now - self.last_valid_lane_time if self.last_valid_lane_time > 0.0 else 999.0

        timeout_s = float(self.get_parameter("outer_timeout_s").value)
        stale_hold_s = float(self.get_parameter("stale_hold_s").value)
        intersection_hold_s = float(self.get_parameter("intersection_hold_s").value)

        raw_valid_now = self.raw_valid and age <= stale_hold_s

        if raw_valid_now:
            mode = "tracking"
            target_e_lat = self.raw_e_lat
            target_e_theta = self.raw_e_theta
            v_floor = float(self.get_parameter("v_min").value)
        elif time_since_valid <= intersection_hold_s:
            # Qua ngã tư hoặc mất lane ngắn: đi thẳng chậm, không quay vòng.
            mode = "intersection_hold"
            target_e_lat = 0.0
            target_e_theta = 0.0
            v_floor = float(self.get_parameter("v_intersection").value)
        elif age <= timeout_s:
            mode = "stale_slow"
            target_e_lat = 0.0
            target_e_theta = 0.0
            v_floor = float(self.get_parameter("v_min").value)
        else:
            mode = "lane_timeout"
            self.v_ref_prev = approach(self.v_ref_prev, 0.0, float(self.get_parameter("v_ref_rate").value) * dt)
            self.omega_ref_prev = approach(self.omega_ref_prev, 0.0, float(self.get_parameter("omega_ref_rate").value) * dt)
            self.publish_ref(self.v_ref_prev, self.omega_ref_prev)
            self.publish_state({
                "valid": False,
                "mode": mode,
                "age_s": age,
                "time_since_valid_s": time_since_valid,
                "fps_est": self.fps_est,
                "v_ref": self.v_ref_prev,
                "omega_ref": self.omega_ref_prev,
                "lane_timeout": True,
            })
            return

        err_alpha = self.alpha_from_tau(dt, float(self.get_parameter("error_filter_tau_s").value))
        self.e_lat_f = (1.0 - err_alpha) * self.e_lat_f + err_alpha * target_e_lat
        self.e_theta_f = (1.0 - err_alpha) * self.e_theta_f + err_alpha * target_e_theta

        x_deadband = float(self.get_parameter("x_deadband_m").value)
        theta_deadband = float(self.get_parameter("theta_deadband_rad").value)

        e_lat_used = 0.0 if abs(self.e_lat_f) < x_deadband else self.e_lat_f
        e_theta_used = 0.0 if abs(self.e_theta_f) < theta_deadband else self.e_theta_f

        de_lat_raw = (e_lat_used - self.prev_e_lat_used) / dt
        de_theta_raw = (e_theta_used - self.prev_e_theta_used) / dt
        self.prev_e_lat_used = e_lat_used
        self.prev_e_theta_used = e_theta_used

        d_alpha = self.alpha_from_tau(dt, float(self.get_parameter("derivative_filter_tau_s").value))
        self.de_lat_f = (1.0 - d_alpha) * self.de_lat_f + d_alpha * de_lat_raw
        self.de_theta_f = (1.0 - d_alpha) * self.de_theta_f + d_alpha * de_theta_raw

        lookahead = self.raw_lookahead

        curvature = -2.0 * e_lat_used / max(lookahead * lookahead, 1e-4)

        target_dist_per_frame = float(self.get_parameter("target_distance_per_frame_m").value)
        v_fps = target_dist_per_frame * self.fps_est

        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v_turn_min = float(self.get_parameter("v_turn_min").value)

        v_base = clamp(v_fps, v_min, v_max)

        k_slow_lat = float(self.get_parameter("k_slow_lat").value)
        k_slow_theta = float(self.get_parameter("k_slow_theta").value)
        k_slow_curv = float(self.get_parameter("k_slow_curvature").value)

        slow_factor = math.exp(
            -k_slow_lat * abs(self.e_lat_f)
            -k_slow_theta * abs(self.e_theta_f)
            -k_slow_curv * abs(curvature)
        )

        v_des = clamp(v_base * slow_factor, v_floor, v_max)

        if abs(self.e_lat_f) >= x_deadband and raw_valid_now:
            v_des = max(v_des, v_turn_min)

        # Nếu mất lane/ngã tư thì không quay.
        if mode != "tracking":
            omega_des = 0.0
        else:
            k_pp = float(self.get_parameter("k_pp").value)
            k_lat = float(self.get_parameter("k_lat").value)
            kd_lat = float(self.get_parameter("kd_lat").value)
            k_theta = float(self.get_parameter("k_theta").value)
            kd_theta = float(self.get_parameter("kd_theta").value)

            omega_raw = (
                k_pp * v_des * curvature
                - k_lat * e_lat_used
                - kd_lat * self.de_lat_f
                + k_theta * e_theta_used
                + kd_theta * self.de_theta_f
            )

            if bool(self.get_parameter("invert_angular").value):
                omega_raw = -omega_raw

            omega_max = abs(float(self.get_parameter("omega_max").value))
            omega_des = clamp(omega_raw, -omega_max, omega_max)

            if abs(self.e_lat_f) < x_deadband and abs(self.e_theta_f) < theta_deadband:
                omega_des = 0.0

        v_rate = abs(float(self.get_parameter("v_ref_rate").value))
        omega_rate = abs(float(self.get_parameter("omega_ref_rate").value))

        v_ref = approach(self.v_ref_prev, v_des, v_rate * dt)
        omega_ref = approach(self.omega_ref_prev, omega_des, omega_rate * dt)

        self.v_ref_prev = v_ref
        self.omega_ref_prev = omega_ref

        self.publish_ref(v_ref, omega_ref)

        self.publish_state({
            "valid": raw_valid_now,
            "raw_valid": self.raw_valid,
            "valid_reason": getattr(self, "raw_valid_reason", ""),
            "mode": mode,
            "lane_state": self.raw_lane_state,
            "confidence": self.raw_confidence,
            "age_s": age,
            "time_since_valid_s": time_since_valid,

            "fps_est": self.fps_est,
            "v_fps": v_fps,
            "v_base": v_base,

            "lat_source": self.raw_lat_source,
            "theta_source": self.raw_theta_source,
            "lookahead_source": self.raw_lookahead_source,

            "epsilon_x_mm": self.raw_e_lat * 1000.0,
            "e_lat_m": self.raw_e_lat,
            "e_lat_f_m": self.e_lat_f,
            "e_lat_used_m": e_lat_used,

            "e_theta_rad": self.raw_e_theta,
            "e_theta_f_rad": self.e_theta_f,
            "e_theta_used_rad": e_theta_used,

            "lookahead_m": lookahead,
            "curvature": curvature,

            "de_lat": self.de_lat_f,
            "de_theta": self.de_theta_f,

            "slow_factor": slow_factor,
            "v_ref": v_ref,
            "omega_ref": omega_ref,
            "lane_timeout": mode == "lane_timeout",
            "invert_angular": bool(self.get_parameter("invert_angular").value),
            "x_bias_m": float(self.get_parameter("x_bias_m").value),
        })


def main(args=None):
    rclpy.init(args=args)
    node = LaneOuterPDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_ref(0.0, 0.0)
        node.publish_state({
            "valid": False,
            "mode": "shutdown",
            "v_ref": 0.0,
            "omega_ref": 0.0,
        })
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
