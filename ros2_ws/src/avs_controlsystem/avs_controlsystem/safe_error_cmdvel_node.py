#!/usr/bin/env python3

import json
import math
import signal
import time
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def rate_limit(target: float, current: float, max_delta: float) -> float:
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


class SafeErrorCmdvelNode(Node):
    """
    avs_controlsystem node.

    Input:
      /avs/safe_control_error hoặc /safe_control_error
      /avs/control_error hoặc /control_error
      /scan optional

    Output:
      /cmd_vel geometry_msgs/Twist
        linear.x  = v
        angular.z = omega

    Convention:
      epsilon_x_mm > 0: target/lane nằm bên phải robot.
      ROS angular.z > 0: rẽ trái.
      Vì vậy khi epsilon_x_mm > 0 thì omega phải âm để rẽ phải.
    """

    def __init__(self):
        super().__init__("safe_error_cmdvel_node")

        self.declare_parameter("safe_error_topic", "/avs/safe_control_error")
        self.declare_parameter("safe_error_alias_topic", "/safe_control_error")
        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("control_error_alias_topic", "/control_error")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/avs/safe_error_cmdvel_debug")

        self.declare_parameter("enable_motion", False)
        self.declare_parameter("control_rate_hz", 30.0)

        self.declare_parameter("safe_error_timeout_s", 0.8)
        self.declare_parameter("control_error_timeout_s", 0.8)
        self.declare_parameter("allow_raw_control_error_fallback", True)
        self.declare_parameter("stop_on_timeout", True)

        self.declare_parameter("v_max", 0.08)
        self.declare_parameter("v_min", 0.025)
        self.declare_parameter("v_turn_max", 0.045)
        self.declare_parameter("omega_max", 0.28)
        self.declare_parameter("k_c", 0.22)

        self.declare_parameter("k_pursuit", 1.00)
        self.declare_parameter("heading_gain", 0.24)
        self.declare_parameter("kd_lateral", 0.006)
        self.declare_parameter("kd_heading", 0.010)

        self.declare_parameter("filter_alpha", 0.12)
        self.declare_parameter("v_rate_limit", 0.10)
        self.declare_parameter("omega_rate_limit", 0.18)
        self.declare_parameter("timeout_decel_rate", 0.35)

        self.declare_parameter("x_deadband_m", 0.010)
        self.declare_parameter("theta_deadband_rad", 0.014)
        self.declare_parameter("omega_deadband", 0.008)
        self.declare_parameter("ld_min_m", 0.12)
        self.declare_parameter("ld_max_m", 0.85)

        self.declare_parameter("wheel_separation_m", 0.36)
        self.declare_parameter("inner_min_fraction", 0.45)
        self.declare_parameter("allow_pivot_turn", False)

        self.declare_parameter("invert_angular", False)

        self.declare_parameter("use_lidar_safety", True)
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("side_min_angle_deg", 35.0)
        self.declare_parameter("side_max_angle_deg", 110.0)
        self.declare_parameter("emergency_distance", 0.18)
        self.declare_parameter("stop_distance", 0.32)
        self.declare_parameter("slow_distance", 0.70)

        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10
        )
        self.debug_pub = self.create_publisher(
            String,
            str(self.get_parameter("debug_topic").value),
            10
        )

        safe_topic = str(self.get_parameter("safe_error_topic").value)
        safe_alias = str(self.get_parameter("safe_error_alias_topic").value)
        raw_topic = str(self.get_parameter("control_error_topic").value)
        raw_alias = str(self.get_parameter("control_error_alias_topic").value)

        self.create_subscription(String, safe_topic, self.safe_error_callback, 10)
        if safe_alias and safe_alias != safe_topic:
            self.create_subscription(String, safe_alias, self.safe_error_callback, 10)

        self.create_subscription(String, raw_topic, self.raw_error_callback, 10)
        if raw_alias and raw_alias != raw_topic:
            self.create_subscription(String, raw_alias, self.raw_error_callback, 10)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos
        )

        self.safe_error: Optional[Dict[str, Any]] = None
        self.raw_error: Optional[Dict[str, Any]] = None
        self.last_safe_time = -1.0
        self.last_raw_time = -1.0

        self.front_dist = 9.9
        self.left_dist = 9.9
        self.right_dist = 9.9
        self.last_scan_time = -1.0

        self.x_f = 0.0
        self.theta_f = 0.0
        self.prev_x_f = 0.0
        self.prev_theta_f = 0.0

        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.prev_t = time.time()

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("safe_error_cmdvel_node started in avs_controlsystem")
        self.get_logger().info(f"Safe input: {safe_topic}, alias: {safe_alias}")
        self.get_logger().info(f"Raw input:  {raw_topic}, alias: {raw_alias}")
        self.get_logger().info(f"Output:     {self.get_parameter('cmd_vel_topic').value}")

    def now(self) -> float:
        return time.time()

    def parse_error_json(self, data: Dict[str, Any], source: str) -> Optional[Dict[str, Any]]:
        x_mm = float(data.get("epsilon_x_mm", data.get("x_mm", 0.0)))
        y_mm = float(data.get("epsilon_y_mm", data.get("lookahead_d_mm", data.get("y_mm", 300.0))))
        theta = float(data.get("theta_rad", data.get("heading_error_rad", 0.0)))
        curvature_inv_mm = float(data.get("curvature_inv_mm", 0.0))
        armed = bool(data.get("armed", True))

        if not all(math.isfinite(v) for v in [x_mm, y_mm, theta, curvature_inv_mm]):
            return None

        y_mm = max(50.0, abs(y_mm))

        return {
            "source": source,
            "x_mm": x_mm,
            "y_mm": y_mm,
            "theta_rad": theta,
            "curvature_inv_mm": curvature_inv_mm,
            "armed": armed,
            "raw": data,
        }

    def safe_error_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            parsed = self.parse_error_json(data, "safe_control_error")
            if parsed is None:
                return
            self.safe_error = parsed
            self.last_safe_time = self.now()
        except Exception as exc:
            self.get_logger().warn(f"safe_control_error parse error: {exc}")

    def raw_error_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            parsed = self.parse_error_json(data, "control_error")
            if parsed is None:
                return
            parsed["armed"] = True
            self.raw_error = parsed
            self.last_raw_time = self.now()
        except Exception as exc:
            self.get_logger().warn(f"control_error parse error: {exc}")

    def get_sector_values(self, scan: LaserScan, min_deg: float, max_deg: float):
        vals = []
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        angle = scan.angle_min

        for r in scan.ranges:
            if min_rad <= angle <= max_rad:
                if math.isfinite(r) and scan.range_min < r < scan.range_max:
                    vals.append(float(r))
            angle += scan.angle_increment

        return vals

    @staticmethod
    def robust_min(values, default=9.9):
        if not values:
            return default
        values = sorted(values)
        idx = max(0, int(len(values) * 0.15))
        return values[idx]

    def scan_callback(self, msg: LaserScan):
        half_front = float(self.get_parameter("front_angle_deg").value) / 2.0
        side_min = float(self.get_parameter("side_min_angle_deg").value)
        side_max = float(self.get_parameter("side_max_angle_deg").value)

        self.front_dist = self.robust_min(self.get_sector_values(msg, -half_front, half_front))
        self.left_dist = self.robust_min(self.get_sector_values(msg, side_min, side_max))
        self.right_dist = self.robust_min(self.get_sector_values(msg, -side_max, -side_min))
        self.last_scan_time = self.now()

    def select_error(self) -> Optional[Dict[str, Any]]:
        now = self.now()
        safe_timeout = float(self.get_parameter("safe_error_timeout_s").value)
        raw_timeout = float(self.get_parameter("control_error_timeout_s").value)
        allow_raw = bool(self.get_parameter("allow_raw_control_error_fallback").value)

        safe_recent = (
            self.safe_error is not None
            and self.last_safe_time > 0.0
            and now - self.last_safe_time <= safe_timeout
        )

        if safe_recent:
            return self.safe_error

        raw_recent = (
            self.raw_error is not None
            and self.last_raw_time > 0.0
            and now - self.last_raw_time <= raw_timeout
        )

        if allow_raw and raw_recent:
            return self.raw_error

        return None

    def make_cmd(self, v: float, omega: float) -> Twist:
        msg = Twist()
        msg.linear.x = float(v)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(omega)
        return msg

    def publish_cmd(self, v: float, omega: float):
        self.cmd_pub.publish(self.make_cmd(v, omega))

    def safe_stop_robot(self, repeat: int = 20):
        stop = self.make_cmd(0.0, 0.0)
        for _ in range(repeat):
            self.cmd_pub.publish(stop)
            time.sleep(0.02)

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Publishing repeated zero /cmd_vel.")
        self.safe_stop_robot(35)
        if rclpy.ok():
            rclpy.shutdown()

    def publish_debug(self, mode: str, err: Optional[Dict[str, Any]] = None, **extra):
        payload = {
            "mode": mode,
            "enable_motion": bool(self.get_parameter("enable_motion").value),
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "front_dist": self.front_dist,
            "left_dist": self.left_dist,
            "right_dist": self.right_dist,
            "safe_age_s": self.now() - self.last_safe_time if self.last_safe_time > 0 else -1.0,
            "raw_age_s": self.now() - self.last_raw_time if self.last_raw_time > 0 else -1.0,
        }

        if err is not None:
            payload.update({
                "source": err.get("source"),
                "armed": err.get("armed"),
                "epsilon_x_mm": err.get("x_mm"),
                "epsilon_y_mm": err.get("y_mm"),
                "theta_rad": err.get("theta_rad"),
                "curvature_inv_mm": err.get("curvature_inv_mm"),
            })

        payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def hard_stop(self, mode: str, err: Optional[Dict[str, Any]] = None):
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.publish_cmd(0.0, 0.0)
        self.publish_debug(mode, err)

    def ramp_stop(self, dt: float, mode: str, err: Optional[Dict[str, Any]] = None):
        decel = max(0.01, float(self.get_parameter("timeout_decel_rate").value))
        omega_rate = max(0.01, float(self.get_parameter("omega_rate_limit").value))

        self.v_cmd = rate_limit(0.0, self.v_cmd, decel * dt)
        self.omega_cmd = rate_limit(0.0, self.omega_cmd, omega_rate * dt)

        self.publish_cmd(self.v_cmd, self.omega_cmd)
        self.publish_debug(mode, err)

    def control_loop(self):
        now = self.now()
        dt = max(now - self.prev_t, 1e-3)
        self.prev_t = now

        if not bool(self.get_parameter("enable_motion").value):
            self.hard_stop("disabled")
            return

        err = self.select_error()

        if err is None:
            if bool(self.get_parameter("stop_on_timeout").value):
                self.ramp_stop(dt, "error_timeout")
            else:
                self.publish_debug("error_timeout_no_stop")
            return

        if err.get("source") == "safe_control_error" and not bool(err.get("armed", False)):
            self.hard_stop("safe_error_disarmed", err)
            return

        if bool(self.get_parameter("use_lidar_safety").value):
            emergency = float(self.get_parameter("emergency_distance").value)
            stop_d = float(self.get_parameter("stop_distance").value)
            if self.front_dist < emergency:
                self.hard_stop("lidar_emergency_stop", err)
                return
            if self.front_dist < stop_d:
                self.hard_stop("lidar_stop", err)
                return

        x_m = float(err["x_mm"]) / 1000.0
        ld_m = max(float(err["y_mm"]) / 1000.0, 1e-3)
        theta = float(err["theta_rad"])
        kappa_m = float(err["curvature_inv_mm"]) * 1000.0

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

        ld_m = clamp(
            ld_m,
            float(self.get_parameter("ld_min_m").value),
            float(self.get_parameter("ld_max_m").value),
        )

        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v_turn_max = float(self.get_parameter("v_turn_max").value)
        omega_max = abs(float(self.get_parameter("omega_max").value))
        k_c = float(self.get_parameter("k_c").value)

        v_target = v_max * max(0.0, math.cos(self.theta_f)) / (1.0 + k_c * abs(kappa_m))
        v_target = clamp(v_target, v_min, v_max)

        # x > 0: target bên phải. ROS angular.z âm: rẽ phải.
        pp_curvature = -2.0 * self.x_f / max(1e-4, ld_m * ld_m)

        omega_target = (
            float(self.get_parameter("k_pursuit").value) * v_target * pp_curvature
            - float(self.get_parameter("heading_gain").value) * self.theta_f
            - float(self.get_parameter("kd_lateral").value) * dx
            - float(self.get_parameter("kd_heading").value) * dtheta
        )

        if bool(self.get_parameter("invert_angular").value):
            omega_target = -omega_target

        if abs(omega_target) < float(self.get_parameter("omega_deadband").value):
            omega_target = 0.0

        if abs(omega_target) > 0.70 * omega_max:
            v_target = min(v_target, v_turn_max)

        if bool(self.get_parameter("use_lidar_safety").value):
            slow_d = float(self.get_parameter("slow_distance").value)
            stop_d = float(self.get_parameter("stop_distance").value)
            if self.front_dist < slow_d:
                ratio = (self.front_dist - stop_d) / max(1e-3, slow_d - stop_d)
                ratio = clamp(ratio, 0.20, 1.0)
                v_target *= ratio

        omega_limit = omega_max
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
            "tracking",
            err,
            x_m_filtered=self.x_f,
            theta_filtered=self.theta_f,
            ld_m=ld_m,
            kappa_m=kappa_m,
            pp_curvature=pp_curvature,
            v_target=v_target,
            omega_target=omega_target,
            omega_limit=omega_limit,
        )


def main(args=None):
    rclpy.init(args=args)
    node = SafeErrorCmdvelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop_robot(35)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
