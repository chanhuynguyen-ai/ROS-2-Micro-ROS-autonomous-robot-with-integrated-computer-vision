#!/usr/bin/env python3

import json
import math
import time
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# Label ids theo config/label_mapping.json (model 22 class).
LABEL_MAIN_LANE = 6
LABEL_TURN_LANE = 20


def clamp(value, low, high):
    return max(low, min(high, value))


def rate_limit(target, current, max_delta):
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


class AvsLaneCmdvelNode(Node):
    def __init__(self):
        super().__init__("avs_lane_cmdvel_node")

        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("telemetry_topic", "/avs/telemetry_realworld")
        self.declare_parameter("lane_state_topic", "/avs/lane_state")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/avs/lane_cmdvel_debug")

        self.declare_parameter("enable_motion", False)
        self.declare_parameter("control_rate_hz", 30.0)

        self.declare_parameter("control_error_timeout", 0.8)
        self.declare_parameter("telemetry_timeout", 1.5)
        self.declare_parameter("stop_on_timeout", True)
        self.declare_parameter("timeout_decel_rate", 0.35)

        self.declare_parameter("target_label", LABEL_MAIN_LANE)
        self.declare_parameter("allow_telemetry_fallback", True)
        self.declare_parameter("allow_label_fallback", True)

        self.declare_parameter("v_max", 0.08)
        self.declare_parameter("v_min", 0.030)
        self.declare_parameter("v_turn_max", 0.050)
        self.declare_parameter("omega_max", 0.28)

        self.declare_parameter("k_pursuit", 1.0)
        self.declare_parameter("heading_gain", 0.26)
        self.declare_parameter("kd_lateral", 0.008)
        self.declare_parameter("kd_heading", 0.012)

        self.declare_parameter("filter_alpha", 0.10)
        self.declare_parameter("v_rate_limit", 0.12)
        self.declare_parameter("omega_rate_limit", 0.22)

        self.declare_parameter("x_deadband_m", 0.010)
        self.declare_parameter("theta_deadband_rad", 0.014)
        self.declare_parameter("omega_deadband", 0.008)

        self.declare_parameter("min_lookahead_m", 0.12)
        self.declare_parameter("max_lookahead_m", 0.85)

        self.declare_parameter("max_lateral_accel", 0.16)
        self.declare_parameter("slow_x_gain", 1.20)
        self.declare_parameter("slow_theta_gain", 0.90)
        self.declare_parameter("slow_curvature_gain", 0.08)

        self.declare_parameter("wheel_separation_m", 0.36)
        self.declare_parameter("inner_min_fraction", 0.45)
        self.declare_parameter("allow_pivot_turn", False)

        self.declare_parameter("invert_angular", False)

        self.declare_parameter("use_lidar_safety", True)
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("emergency_distance", 0.18)
        self.declare_parameter("stop_distance", 0.32)
        self.declare_parameter("slow_distance", 0.70)

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            self.get_parameter("debug_topic").value,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter("control_error_topic").value,
            self.control_error_callback,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter("telemetry_topic").value,
            self.telemetry_callback,
            10
        )

        self.create_subscription(
            String,
            self.get_parameter("lane_state_topic").value,
            self.lane_state_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.control_error_target: Optional[Dict[str, Any]] = None
        self.telemetry_target: Optional[Dict[str, Any]] = None

        self.last_control_error_time = -1.0
        self.last_telemetry_time = -1.0
        self.last_scan_time = -1.0

        self.lane_state = "UNKNOWN"
        self.main_lane_detected = False
        self.other_lane_detected = False
        self.turn_lane_detected = False
        self.stop_line_detected = False

        self.front_min_distance = float("inf")

        self.x_f = 0.0
        self.theta_f = 0.0
        self.prev_x_f = 0.0
        self.prev_theta_f = 0.0

        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.last_loop_time = time.time()

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.loop)

        self.get_logger().info("avs_lane_cmdvel_node started")
        self.get_logger().info("Prefer /avs/control_error; fallback /avs/telemetry_realworld")
        self.get_logger().info("Output /cmd_vel only: linear.x and angular.z")

    def now(self):
        return time.time()

    def control_error_callback(self, msg):
        try:
            data = json.loads(msg.data)

            x_mm = float(data.get("epsilon_x_mm", 0.0))
            y_mm = float(data.get("lookahead_d_mm", data.get("epsilon_y_mm", 0.0)))
            theta = float(data.get("theta_rad", 0.0))
            label = int(data.get("target_label", -1))
            curvature_inv_mm = float(data.get("curvature_inv_mm", 0.0))

            if not all(math.isfinite(v) for v in [x_mm, y_mm, theta]):
                return

            self.control_error_target = {
                "source": "control_error",
                "x_mm": x_mm,
                "y_mm": y_mm,
                "theta_rad": theta,
                "label": label,
                "prob": 1.0,
                "curvature_inv_mm": curvature_inv_mm,
                "fallback_used": bool(data.get("fallback_used", False)),
            }

            self.last_control_error_time = self.now()

        except Exception as e:
            self.get_logger().warn(f"control_error parse error: {e}")

    def has_lookahead(self, obj):
        return (
            "lookahead_x_mm" in obj
            and "lookahead_d_mm" in obj
            and "lookahead_theta_rad" in obj
        )

    def telemetry_callback(self, msg):
        try:
            data = json.loads(msg.data)
            objects = data.get("objects", [])

            wanted_label = int(self.get_parameter("target_label").value)
            allow_label_fallback = bool(self.get_parameter("allow_label_fallback").value)

            best = None
            best_prob = -1.0
            fallback = None
            fallback_prob = -1.0

            for obj in objects:
                if not self.has_lookahead(obj):
                    continue

                label = int(obj.get("label", -1))
                prob = float(obj.get("prob", 0.0))

                if label == wanted_label and prob > best_prob:
                    best = obj
                    best_prob = prob

                if allow_label_fallback and label in (3, 4, 10) and prob > fallback_prob:
                    fallback = obj
                    fallback_prob = prob

            target = best
            fallback_used = False

            if target is None:
                target = fallback
                fallback_used = target is not None

            if target is None:
                return

            x_mm = float(target.get("lookahead_x_mm", 0.0))
            y_mm = float(target.get("lookahead_d_mm", 0.0))
            theta = float(target.get("lookahead_theta_rad", 0.0))

            if not all(math.isfinite(v) for v in [x_mm, y_mm, theta]):
                return

            self.telemetry_target = {
                "source": "telemetry_realworld",
                "x_mm": x_mm,
                "y_mm": y_mm,
                "theta_rad": theta,
                "label": int(target.get("label", -1)),
                "prob": float(target.get("prob", 0.0)),
                "curvature_inv_mm": float(target.get("curvature_inv_mm", 0.0)),
                "fallback_used": fallback_used,
            }

            self.last_telemetry_time = self.now()

        except Exception as e:
            self.get_logger().warn(f"telemetry parse error: {e}")

    def lane_state_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.lane_state = data.get("lane_state", self.lane_state)
            self.main_lane_detected = bool(data.get("main_lane_detected", False))
            self.other_lane_detected = bool(data.get("other_lane_detected", False))
            self.turn_lane_detected = bool(data.get("turn_lane_detected", False))
            self.stop_line_detected = bool(data.get("stop_line_detected", False))
        except Exception:
            pass

    def scan_callback(self, msg):
        front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        front_min = float("inf")

        angle = msg.angle_min
        for r in msg.ranges:
            if -front_angle <= angle <= front_angle:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    front_min = min(front_min, float(r))
            angle += msg.angle_increment

        self.front_min_distance = front_min
        self.last_scan_time = self.now()

    def select_target(self):
        now = self.now()

        ce_timeout = float(self.get_parameter("control_error_timeout").value)
        telemetry_timeout = float(self.get_parameter("telemetry_timeout").value)

        if (
            self.control_error_target is not None
            and self.last_control_error_time > 0.0
            and now - self.last_control_error_time <= ce_timeout
        ):
            return self.control_error_target

        if not bool(self.get_parameter("allow_telemetry_fallback").value):
            return None

        if (
            self.telemetry_target is not None
            and self.last_telemetry_time > 0.0
            and now - self.last_telemetry_time <= telemetry_timeout
        ):
            return self.telemetry_target

        return None

    def publish_cmd(self, v, omega):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def publish_debug(self, mode, target=None, **extra):
        data = {
            "enabled": bool(self.get_parameter("enable_motion").value),
            "mode": mode,
            "lane_state": self.lane_state,
            "main_lane_detected": self.main_lane_detected,
            "other_lane_detected": self.other_lane_detected,
            "turn_lane_detected": self.turn_lane_detected,
            "stop_line_detected": self.stop_line_detected,
            "front_min_distance": self.front_min_distance,
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "last_control_error_age_s": self.now() - self.last_control_error_time if self.last_control_error_time > 0 else -1.0,
            "last_telemetry_age_s": self.now() - self.last_telemetry_time if self.last_telemetry_time > 0 else -1.0,
        }

        if target is not None:
            data.update({
                "target_source": target.get("source", ""),
                "target_label": target.get("label", -1),
                "target_prob": target.get("prob", 0.0),
                "fallback_used": target.get("fallback_used", False),
                "x_mm": target.get("x_mm", 0.0),
                "y_mm": target.get("y_mm", 0.0),
                "theta_rad": target.get("theta_rad", 0.0),
                "curvature_inv_mm": target.get("curvature_inv_mm", 0.0),
            })

        data.update(extra)

        msg = String()
        msg.data = json.dumps(data)
        self.debug_pub.publish(msg)

    def hard_stop(self, mode):
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.publish_cmd(0.0, 0.0)
        self.publish_debug(mode, None)

    def ramp_stop(self, dt, mode):
        decel = max(0.01, float(self.get_parameter("timeout_decel_rate").value))
        omega_rate = max(0.01, float(self.get_parameter("omega_rate_limit").value))

        self.v_cmd = rate_limit(0.0, self.v_cmd, decel * dt)
        self.omega_cmd = rate_limit(0.0, self.omega_cmd, omega_rate * dt)

        self.publish_cmd(self.v_cmd, self.omega_cmd)
        self.publish_debug(mode, None)

    def loop(self):
        now = self.now()
        dt = max(1e-3, now - self.last_loop_time)
        self.last_loop_time = now

        if not bool(self.get_parameter("enable_motion").value):
            self.hard_stop("disabled")
            return

        target = self.select_target()

        if target is None:
            if bool(self.get_parameter("stop_on_timeout").value):
                self.ramp_stop(dt, "target_timeout_ramp_stop")
            else:
                self.publish_debug("target_timeout_no_stop", None)
            return

        if bool(self.get_parameter("use_lidar_safety").value):
            emergency = float(self.get_parameter("emergency_distance").value)
            stop_d = float(self.get_parameter("stop_distance").value)

            if self.front_min_distance < emergency:
                self.hard_stop("lidar_emergency_stop")
                return

            if self.front_min_distance < stop_d:
                self.hard_stop("lidar_stop")
                return

        x_m = float(target["x_mm"]) / 1000.0
        y_m = float(target["y_mm"]) / 1000.0
        theta = float(target["theta_rad"])

        if abs(x_m) < float(self.get_parameter("x_deadband_m").value):
            x_m = 0.0

        if abs(theta) < float(self.get_parameter("theta_deadband_rad").value):
            theta = 0.0

        alpha = clamp(float(self.get_parameter("filter_alpha").value), 0.01, 1.0)

        self.x_f = alpha * x_m + (1.0 - alpha) * self.x_f
        self.theta_f = alpha * theta + (1.0 - alpha) * self.theta_f

        dx = (self.x_f - self.prev_x_f) / dt
        dtheta = (self.theta_f - self.prev_theta_f) / dt

        self.prev_x_f = self.x_f
        self.prev_theta_f = self.theta_f

        lookahead_m = abs(y_m)
        if lookahead_m < 1e-3:
            lookahead_m = math.hypot(x_m, y_m)

        lookahead_m = clamp(
            lookahead_m,
            float(self.get_parameter("min_lookahead_m").value),
            float(self.get_parameter("max_lookahead_m").value)
        )

        curvature = -2.0 * self.x_f / max(1e-4, lookahead_m * lookahead_m)

        target_label = int(target.get("label", -1))
        is_turn = target_label == LABEL_TURN_LANE or self.lane_state == "TURNING"

        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v_turn_max = float(self.get_parameter("v_turn_max").value)

        active_v_max = min(v_max, v_turn_max) if is_turn else v_max

        max_lat_accel = max(0.02, float(self.get_parameter("max_lateral_accel").value))
        abs_curv = abs(curvature)

        v_curve_limit = active_v_max
        if abs_curv > 1e-4:
            v_curve_limit = math.sqrt(max_lat_accel / abs_curv)

        v_curve_limit = clamp(v_curve_limit, v_min, active_v_max)

        if bool(self.get_parameter("use_lidar_safety").value):
            slow_d = float(self.get_parameter("slow_distance").value)
            stop_d = float(self.get_parameter("stop_distance").value)

            if self.front_min_distance < slow_d:
                scale = clamp((self.front_min_distance - stop_d) / max(1e-3, slow_d - stop_d), 0.20, 1.0)
                v_curve_limit *= scale

        slow_factor = (
            1.0
            + float(self.get_parameter("slow_x_gain").value) * abs(self.x_f)
            + float(self.get_parameter("slow_theta_gain").value) * abs(self.theta_f)
            + float(self.get_parameter("slow_curvature_gain").value) * abs_curv
        )

        v_target = clamp(v_curve_limit / slow_factor, v_min, active_v_max)

        omega_target = (
            float(self.get_parameter("k_pursuit").value) * v_target * curvature
            - float(self.get_parameter("heading_gain").value) * self.theta_f
            - float(self.get_parameter("kd_lateral").value) * dx
            - float(self.get_parameter("kd_heading").value) * dtheta
        )

        if bool(self.get_parameter("invert_angular").value):
            omega_target = -omega_target

        if abs(omega_target) < float(self.get_parameter("omega_deadband").value):
            omega_target = 0.0

        omega_limit = abs(float(self.get_parameter("omega_max").value))

        if not bool(self.get_parameter("allow_pivot_turn").value):
            wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))
            inner_min = clamp(float(self.get_parameter("inner_min_fraction").value), 0.0, 0.85)
            omega_no_pivot = abs(v_target) * (1.0 - inner_min) / (wheel_sep * 0.5)
            omega_limit = min(omega_limit, max(0.05, omega_no_pivot))

        omega_target = clamp(omega_target, -omega_limit, omega_limit)

        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        omega_rate = max(0.01, float(self.get_parameter("omega_rate_limit").value))

        self.v_cmd = rate_limit(v_target, self.v_cmd, v_rate * dt)
        self.omega_cmd = rate_limit(omega_target, self.omega_cmd, omega_rate * dt)

        self.publish_cmd(self.v_cmd, self.omega_cmd)

        self.publish_debug(
            "tracking_lane",
            target,
            x_m_filtered=self.x_f,
            theta_filtered=self.theta_f,
            lookahead_m=lookahead_m,
            curvature=curvature,
            v_target=v_target,
            omega_target=omega_target,
            omega_limit=omega_limit,
        )


def main(args=None):
    rclpy.init(args=args)
    node = AvsLaneCmdvelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
