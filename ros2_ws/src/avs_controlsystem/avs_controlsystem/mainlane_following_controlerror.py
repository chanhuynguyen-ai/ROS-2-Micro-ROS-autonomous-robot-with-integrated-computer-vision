#!/usr/bin/env python3

import json
import math
import signal
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


def clamp(value, low, high):
    return max(low, min(high, value))


def rate_limit(target, current, max_delta):
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


class MainLaneFollowingControlError(Node):
    """
    Node điều khiển bám làn chính từ /avs/control_error.

    Subscribe:
      /avs/control_error [std_msgs/String]

    Publish:
      /cmd_vel [geometry_msgs/Twist]

    Quy ước:
      epsilon_x_mm > 0: điểm mục tiêu/làn nằm bên phải robot.
      ROS angular.z > 0: xe quay trái.
      Vì vậy epsilon_x_mm > 0 thì omega phải âm để xe rẽ phải.
    """

    def __init__(self):
        super().__init__("mainlane_following_controlerror")

        # Topics
        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/avs/mainlane_control_debug")

        # Enable
        self.declare_parameter("enable_motion", True)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("error_timeout_s", 1.5)

        # Speed limits
        self.declare_parameter("v_max", 0.10)
        self.declare_parameter("v_min", 0.025)
        self.declare_parameter("v_turn_max", 0.055)
        self.declare_parameter("omega_max", 0.32)

        # Control gains
        self.declare_parameter("k_pursuit", 1.00)
        self.declare_parameter("k_heading", 0.28)
        self.declare_parameter("kd_lateral", 0.006)
        self.declare_parameter("kd_heading", 0.010)

        # Smooth filters
        self.declare_parameter("filter_alpha", 0.12)
        self.declare_parameter("v_rate_limit", 0.12)
        self.declare_parameter("omega_rate_limit", 0.22)
        self.declare_parameter("timeout_decel_rate", 0.35)

        # Deadband
        self.declare_parameter("x_deadband_m", 0.010)
        self.declare_parameter("theta_deadband_rad", 0.014)
        self.declare_parameter("omega_deadband", 0.006)

        # Lookahead
        self.declare_parameter("lookahead_min_m", 0.12)
        self.declare_parameter("lookahead_max_m", 0.85)
        self.declare_parameter("default_lookahead_m", 0.30)

        # Differential-drive smoothing
        # Robot 4 bánh differential/skid-steer nhưng ROS vẫn dùng v, omega.
        # ESP32 tự chia v_left/v_right.
        self.declare_parameter("wheel_separation_m", 0.36)
        self.declare_parameter("inner_wheel_min_fraction", 0.55)
        self.declare_parameter("allow_pivot_turn", False)

        # Speed profile
        self.declare_parameter("slow_lateral_gain", 2.0)
        self.declare_parameter("slow_heading_gain", 0.9)
        self.declare_parameter("slow_curvature_gain", 0.04)

        # Sign
        self.declare_parameter("invert_angular", False)

        self.control_error_topic = str(self.get_parameter("control_error_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.sub = self.create_subscription(
            String,
            self.control_error_topic,
            self.control_error_callback,
            10
        )

        self.last_error = None
        self.last_error_time = -1.0

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

        self.get_logger().info("mainlane_following_controlerror started")
        self.get_logger().info(f"Subscribe: {self.control_error_topic}")
        self.get_logger().info(f"Publish:   {self.cmd_vel_topic}")
        self.get_logger().info("Control source: /avs/control_error only, no arm/disarm")

    def now(self):
        return time.time()

    def control_error_callback(self, msg):
        try:
            data = json.loads(msg.data)

            # Nhận nhiều tên field để tránh lệch format giữa các version control_node.
            x_mm = float(data.get(
                "epsilon_x_mm",
                data.get("lookahead_x_mm", data.get("x_mm", data.get("lateral_error_mm", 0.0)))
            ))

            y_mm = float(data.get(
                "epsilon_y_mm",
                data.get("lookahead_d_mm", data.get("y_mm", 1000.0 * float(self.get_parameter("default_lookahead_m").value)))
            ))

            theta_rad = float(data.get(
                "theta_rad",
                data.get("heading_error_rad", data.get("target_theta_rad", 0.0))
            ))

            curvature_inv_mm = float(data.get("curvature_inv_mm", 0.0))

            valid = data.get("valid", True)
            if isinstance(valid, str):
                valid = valid.lower() not in ("false", "0", "no", "invalid")
            valid = bool(valid)

            if not all(math.isfinite(v) for v in [x_mm, y_mm, theta_rad, curvature_inv_mm]):
                return

            y_mm = max(abs(y_mm), 50.0)

            self.last_error = {
                "valid": valid,
                "epsilon_x_mm": x_mm,
                "epsilon_y_mm": y_mm,
                "theta_rad": theta_rad,
                "curvature_inv_mm": curvature_inv_mm,
                "raw": data,
            }
            self.last_error_time = self.now()

        except Exception as exc:
            self.get_logger().warn(f"Invalid /avs/control_error JSON: {exc}")

    def make_cmd(self, v, omega):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(omega)
        return cmd

    def publish_cmd(self, v, omega):
        self.cmd_pub.publish(self.make_cmd(v, omega))

    def safe_stop_robot(self, repeat=35):
        stop = self.make_cmd(0.0, 0.0)
        for _ in range(repeat):
            self.cmd_pub.publish(stop)
            time.sleep(0.02)

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Sending repeated zero /cmd_vel.")
        self.safe_stop_robot(35)
        if rclpy.ok():
            rclpy.shutdown()

    def publish_debug(self, mode, extra=None):
        payload = {
            "mode": mode,
            "enable_motion": bool(self.get_parameter("enable_motion").value),
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "error_age_s": self.now() - self.last_error_time if self.last_error_time > 0 else -1.0,
        }

        if self.last_error is not None:
            payload.update({
                "valid": self.last_error["valid"],
                "epsilon_x_mm": self.last_error["epsilon_x_mm"],
                "epsilon_y_mm": self.last_error["epsilon_y_mm"],
                "theta_rad": self.last_error["theta_rad"],
                "curvature_inv_mm": self.last_error["curvature_inv_mm"],
            })

        if extra:
            payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(msg)

    def hard_stop(self, mode):
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.publish_cmd(0.0, 0.0)
        self.publish_debug(mode)

    def ramp_stop(self, dt, mode):
        decel = max(0.01, float(self.get_parameter("timeout_decel_rate").value))
        omega_rate = max(0.01, float(self.get_parameter("omega_rate_limit").value))

        self.v_cmd = rate_limit(0.0, self.v_cmd, decel * dt)
        self.omega_cmd = rate_limit(0.0, self.omega_cmd, omega_rate * dt)

        self.publish_cmd(self.v_cmd, self.omega_cmd)
        self.publish_debug(mode)

    def limit_omega_by_differential_speed(self, v_target, omega_target):
        """
        Giới hạn omega để đổi hướng bằng chênh lệch vận tốc hai bên từ từ.

        Differential-drive:
          v_left  = v - omega * W/2
          v_right = v + omega * W/2

        Nếu không cho pivot, giữ bánh trong vẫn chạy tiến tối thiểu:
          v_inner >= inner_fraction * v
        """

        omega_max = abs(float(self.get_parameter("omega_max").value))

        if bool(self.get_parameter("allow_pivot_turn").value):
            omega_limit = omega_max
        else:
            wheel_separation = max(0.05, float(self.get_parameter("wheel_separation_m").value))
            inner_fraction = clamp(float(self.get_parameter("inner_wheel_min_fraction").value), 0.0, 0.90)

            # |omega| <= v * (1 - inner_fraction) / (W/2)
            omega_no_pivot = abs(v_target) * (1.0 - inner_fraction) / (wheel_separation * 0.5)

            # Vẫn cho phép omega nhỏ để xe có thể chỉnh hướng khi v thấp.
            omega_limit = min(omega_max, max(0.05, omega_no_pivot))

        omega_limited = clamp(omega_target, -omega_limit, omega_limit)

        wheel_separation = max(0.05, float(self.get_parameter("wheel_separation_m").value))
        v_left_est = v_target - omega_limited * wheel_separation * 0.5
        v_right_est = v_target + omega_limited * wheel_separation * 0.5

        return omega_limited, omega_limit, v_left_est, v_right_est

    def control_loop(self):
        now = self.now()
        dt = max(now - self.prev_t, 1e-3)
        self.prev_t = now

        if not bool(self.get_parameter("enable_motion").value):
            self.hard_stop("disabled")
            return

        if self.last_error is None or self.last_error_time < 0.0:
            self.hard_stop("no_control_error")
            return

        age = now - self.last_error_time
        timeout_s = float(self.get_parameter("error_timeout_s").value)
        if age > timeout_s:
            self.ramp_stop(dt, "control_error_timeout")
            return

        if not bool(self.last_error.get("valid", True)):
            self.hard_stop("control_error_invalid")
            return

        x_m = float(self.last_error["epsilon_x_mm"]) / 1000.0
        ld_m = float(self.last_error["epsilon_y_mm"]) / 1000.0
        theta = float(self.last_error["theta_rad"])

        if abs(x_m) < float(self.get_parameter("x_deadband_m").value):
            x_m = 0.0

        if abs(theta) < float(self.get_parameter("theta_deadband_rad").value):
            theta = 0.0

        ld_m = clamp(
            abs(ld_m),
            float(self.get_parameter("lookahead_min_m").value),
            float(self.get_parameter("lookahead_max_m").value)
        )

        alpha = clamp(float(self.get_parameter("filter_alpha").value), 0.01, 1.0)
        self.x_f = alpha * x_m + (1.0 - alpha) * self.x_f
        self.theta_f = alpha * theta + (1.0 - alpha) * self.theta_f

        dx = (self.x_f - self.prev_x_f) / dt
        dtheta = (self.theta_f - self.prev_theta_f) / dt

        self.prev_x_f = self.x_f
        self.prev_theta_f = self.theta_f

        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v_turn_max = float(self.get_parameter("v_turn_max").value)
        omega_max = abs(float(self.get_parameter("omega_max").value))

        # Pure pursuit curvature.
        # x > 0 nghĩa là target/lane ở bên phải.
        # Muốn rẽ phải thì angular.z phải âm.
        curvature = -2.0 * self.x_f / max(1e-4, ld_m * ld_m)

        slow_factor = (
            1.0
            + float(self.get_parameter("slow_lateral_gain").value) * abs(self.x_f)
            + float(self.get_parameter("slow_heading_gain").value) * abs(self.theta_f)
            + float(self.get_parameter("slow_curvature_gain").value) * abs(curvature)
        )

        v_target = clamp(v_max / slow_factor, v_min, v_max)

        omega_target = (
            float(self.get_parameter("k_pursuit").value) * v_target * curvature
            - float(self.get_parameter("k_heading").value) * self.theta_f
            - float(self.get_parameter("kd_lateral").value) * dx
            - float(self.get_parameter("kd_heading").value) * dtheta
        )

        if bool(self.get_parameter("invert_angular").value):
            omega_target = -omega_target

        if abs(omega_target) < float(self.get_parameter("omega_deadband").value):
            omega_target = 0.0

        if abs(omega_target) > 0.70 * omega_max:
            v_target = min(v_target, v_turn_max)

        omega_target, omega_limit, v_left_est, v_right_est = self.limit_omega_by_differential_speed(
            v_target,
            omega_target
        )

        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        omega_rate = max(0.01, float(self.get_parameter("omega_rate_limit").value))

        self.v_cmd = rate_limit(v_target, self.v_cmd, v_rate * dt)
        self.omega_cmd = rate_limit(omega_target, self.omega_cmd, omega_rate * dt)

        # Ước lượng lại vận tốc hai bên theo lệnh thực sau rate-limit.
        wheel_separation = max(0.05, float(self.get_parameter("wheel_separation_m").value))
        v_left_cmd_est = self.v_cmd - self.omega_cmd * wheel_separation * 0.5
        v_right_cmd_est = self.v_cmd + self.omega_cmd * wheel_separation * 0.5

        self.publish_cmd(self.v_cmd, self.omega_cmd)

        self.publish_debug(
            "tracking_main_lane",
            {
                "x_m_filtered": self.x_f,
                "theta_filtered": self.theta_f,
                "lookahead_m": ld_m,
                "curvature": curvature,
                "v_target": v_target,
                "omega_target": omega_target,
                "omega_limit": omega_limit,
                "v_left_est": v_left_est,
                "v_right_est": v_right_est,
                "v_left_cmd_est": v_left_cmd_est,
                "v_right_cmd_est": v_right_cmd_est,
            }
        )


def main(args=None):
    rclpy.init(args=args)
    node = MainLaneFollowingControlError()

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
