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


def parse_bool(value, default=True):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() not in ["false", "0", "no", "invalid", "none"]
    if value is None:
        return default
    try:
        return bool(value)
    except Exception:
        return default


class PurPersuitPDMainlaneFollowing(Node):
    """
    Bộ điều khiển Pure Pursuit + PD cho xe bám main-lane.

    Subscribe:
      /avs/control_error [std_msgs/String]

    Publish:
      /cmd_vel [geometry_msgs/Twist]

    Input JSON:
      epsilon_x_mm       : sai số ngang, mm. Dương nếu target/lane ở bên phải xe.
      epsilon_y_mm       : khoảng cách nhìn trước Ld, mm.
      theta_rad          : sai số góc heading, rad.
      curvature_inv_mm   : độ cong làn, mm^-1.

    Pure Pursuit:
      e_x = epsilon_x_mm / 1000
      L_d = epsilon_y_mm / 1000
      kappa_m = curvature_inv_mm * 1000

      v = v_max * cos(theta) / (1 + k_c * abs(kappa_m))

      omega_pp = -2 * v * e_x / L_d^2

    PD correction:
      omega_pd = -Ktheta * theta_f
                 -Kdx * d(e_x_f)/dt
                 -Kdtheta * d(theta_f)/dt

      omega = omega_pp + omega_pd

    Dấu âm:
      epsilon_x > 0: target ở bên phải.
      ROS angular.z < 0: rẽ phải.
    """

    def __init__(self):
        super().__init__("pur_persuit_pd_mainlane_following")

        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/avs/pur_persuit_pd_mainlane_debug")

        self.declare_parameter("enable_motion", True)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("error_timeout_s", 1.5)

        # Tốc độ tối đa xe chạy khoảng 0.1 m/s.
        self.declare_parameter("v_max", 0.10)
        self.declare_parameter("v_min", 0.025)
        self.declare_parameter("v_turn_min", 0.035)

        # Phạt độ cong khi tính tốc độ.
        self.declare_parameter("k_c", 0.20)

        # Pure Pursuit gain.
        self.declare_parameter("k_pp", 1.00)

        # PD gain.
        self.declare_parameter("k_theta", 0.22)
        self.declare_parameter("kd_lateral", 0.004)
        self.declare_parameter("kd_theta", 0.008)

        # Giới hạn yaw rate.
        self.declare_parameter("omega_max", 0.35)

        # Hình học xe.
        self.declare_parameter("wheel_separation_m", 0.36)

        # Giới hạn chênh lệch tốc độ v_right - v_left.
        self.declare_parameter("max_delta_v", 0.085)
        self.declare_parameter("delta_v_rate_limit", 0.16)
        self.declare_parameter("inner_wheel_min_fraction", 0.45)

        # Lọc sai số.
        self.declare_parameter("filter_alpha", 0.18)

        # Lọc derivative để giảm nhiễu.
        self.declare_parameter("derivative_alpha", 0.25)

        # Rate limit tốc độ tiến.
        self.declare_parameter("v_rate_limit", 0.12)

        # Deadband chống rung.
        self.declare_parameter("x_deadband_m", 0.008)
        self.declare_parameter("theta_deadband_rad", 0.012)
        self.declare_parameter("omega_deadband", 0.006)

        # Lookahead bảo vệ chia cho 0.
        self.declare_parameter("ld_min_m", 0.14)
        self.declare_parameter("ld_max_m", 0.85)
        self.declare_parameter("default_lookahead_m", 0.30)

        # Dừng nếu sai số quá lớn.
        self.declare_parameter("max_abs_x_m", 0.45)
        self.declare_parameter("max_abs_theta_rad", 1.30)

        # Đảo dấu nếu chạy thực tế rẽ ngược.
        self.declare_parameter("invert_angular", False)

        self.control_error_topic = str(self.get_parameter("control_error_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.create_subscription(
            String,
            self.control_error_topic,
            self.control_error_callback,
            10
        )

        self.last_error = None
        self.last_error_time = -1.0

        self.e_x_f = 0.0
        self.theta_f = 0.0

        self.prev_e_x_f = 0.0
        self.prev_theta_f = 0.0

        self.de_x_f = 0.0
        self.dtheta_f = 0.0

        self.v_cmd = 0.0
        self.delta_v_cmd = 0.0
        self.omega_cmd = 0.0

        self.prev_t = time.time()

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("pur_persuit_pd_mainlane_following started")
        self.get_logger().info(f"Subscribe: {self.control_error_topic}")
        self.get_logger().info(f"Publish:   {self.cmd_vel_topic}")
        self.get_logger().info("Control law: Pure Pursuit + PD from /avs/control_error")
        self.get_logger().info("Default v_max = 0.10 m/s")

    def now(self):
        return time.time()

    def control_error_callback(self, msg):
        try:
            data = json.loads(msg.data)

            valid = parse_bool(data.get("valid", True), True)
            lane_valid = parse_bool(data.get("lane_valid", True), True)

            e_x_mm = float(data.get("epsilon_x_mm", data.get("x_mm", 0.0)))

            default_ld_mm = float(self.get_parameter("default_lookahead_m").value) * 1000.0
            e_y_mm = float(
                data.get(
                    "epsilon_y_mm",
                    data.get(
                        "lookahead_d_mm",
                        data.get("y_mm", default_ld_mm)
                    )
                )
            )

            theta = float(data.get("theta_rad", data.get("heading_error_rad", 0.0)))
            curvature_inv_mm = float(data.get("curvature_inv_mm", 0.0))
            confidence = float(data.get("confidence", data.get("conf", data.get("prob", 1.0))))

            if not all(math.isfinite(v) for v in [e_x_mm, e_y_mm, theta, curvature_inv_mm, confidence]):
                return

            self.last_error = {
                "valid": valid and lane_valid,
                "epsilon_x_mm": e_x_mm,
                "epsilon_y_mm": max(abs(e_y_mm), 50.0),
                "theta_rad": theta,
                "curvature_inv_mm": curvature_inv_mm,
                "confidence": confidence,
                "raw": data,
            }

            self.last_error_time = self.now()

        except Exception as exc:
            self.get_logger().warn(f"Invalid /avs/control_error JSON: {exc}")

    def make_cmd(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(omega)
        return msg

    def publish_cmd(self, v, omega):
        self.cmd_pub.publish(self.make_cmd(v, omega))

    def publish_debug(self, mode, extra=None):
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))

        v_left_est = self.v_cmd - self.omega_cmd * wheel_sep * 0.5
        v_right_est = self.v_cmd + self.omega_cmd * wheel_sep * 0.5

        payload = {
            "mode": mode,
            "enable_motion": bool(self.get_parameter("enable_motion").value),
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "delta_v_cmd": self.delta_v_cmd,
            "v_left_est": v_left_est,
            "v_right_est": v_right_est,
            "error_age_s": self.now() - self.last_error_time if self.last_error_time > 0 else -1.0,
        }

        if self.last_error is not None:
            payload.update({
                "valid": self.last_error["valid"],
                "epsilon_x_mm": self.last_error["epsilon_x_mm"],
                "epsilon_y_mm": self.last_error["epsilon_y_mm"],
                "theta_rad": self.last_error["theta_rad"],
                "curvature_inv_mm": self.last_error["curvature_inv_mm"],
                "confidence": self.last_error["confidence"],
            })

        if extra:
            payload.update(extra)

        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.debug_pub.publish(out)

    def hard_stop(self, mode):
        self.v_cmd = 0.0
        self.delta_v_cmd = 0.0
        self.omega_cmd = 0.0
        self.publish_cmd(0.0, 0.0)
        self.publish_debug(mode)

    def ramp_stop(self, dt, mode):
        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        delta_rate = max(0.01, float(self.get_parameter("delta_v_rate_limit").value))
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))

        self.v_cmd = rate_limit(0.0, self.v_cmd, v_rate * dt)
        self.delta_v_cmd = rate_limit(0.0, self.delta_v_cmd, delta_rate * dt)
        self.omega_cmd = self.delta_v_cmd / wheel_sep

        self.publish_cmd(self.v_cmd, self.omega_cmd)
        self.publish_debug(mode)

    def safe_stop_robot(self, repeat=35):
        stop = self.make_cmd(0.0, 0.0)
        for _ in range(repeat):
            self.cmd_pub.publish(stop)
            time.sleep(0.02)

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Sending repeated zero /cmd_vel.")
        self.safe_stop_robot()
        if rclpy.ok():
            rclpy.shutdown()

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

        if not bool(self.last_error["valid"]):
            self.ramp_stop(dt, "invalid_control_error")
            return

        # 1. Đọc và đổi đơn vị từ /avs/control_error.
        e_x = self.last_error["epsilon_x_mm"] / 1000.0
        L_d = self.last_error["epsilon_y_mm"] / 1000.0
        theta = self.last_error["theta_rad"]
        kappa_m = self.last_error["curvature_inv_mm"] * 1000.0

        max_abs_x = float(self.get_parameter("max_abs_x_m").value)
        max_abs_theta = float(self.get_parameter("max_abs_theta_rad").value)

        if abs(e_x) > max_abs_x or abs(theta) > max_abs_theta:
            self.ramp_stop(dt, "error_too_large")
            return

        if abs(e_x) < float(self.get_parameter("x_deadband_m").value):
            e_x = 0.0

        if abs(theta) < float(self.get_parameter("theta_deadband_rad").value):
            theta = 0.0

        L_d = clamp(
            abs(L_d),
            float(self.get_parameter("ld_min_m").value),
            float(self.get_parameter("ld_max_m").value)
        )

        # 2. Lọc e_x và theta.
        alpha = clamp(float(self.get_parameter("filter_alpha").value), 0.01, 1.0)
        self.e_x_f = alpha * e_x + (1.0 - alpha) * self.e_x_f
        self.theta_f = alpha * theta + (1.0 - alpha) * self.theta_f

        # 3. Tính đạo hàm đã lọc cho PD.
        raw_de_x = (self.e_x_f - self.prev_e_x_f) / dt
        raw_dtheta = (self.theta_f - self.prev_theta_f) / dt

        d_alpha = clamp(float(self.get_parameter("derivative_alpha").value), 0.01, 1.0)
        self.de_x_f = d_alpha * raw_de_x + (1.0 - d_alpha) * self.de_x_f
        self.dtheta_f = d_alpha * raw_dtheta + (1.0 - d_alpha) * self.dtheta_f

        self.prev_e_x_f = self.e_x_f
        self.prev_theta_f = self.theta_f

        # 4. Tính v theo guide:
        #    v = v_max*cos(theta)/(1+k_c*abs(kappa_m))
        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v_turn_min = float(self.get_parameter("v_turn_min").value)
        k_c = float(self.get_parameter("k_c").value)

        cos_theta = max(0.0, math.cos(self.theta_f))
        v_target = (v_max * cos_theta) / (1.0 + k_c * abs(kappa_m))
        v_target = clamp(v_target, v_min, v_max)

        # 5. Pure Pursuit:
        #    gamma = 2*e_x/L_d^2
        #    omega_pp = v*gamma
        # Dấu âm vì e_x dương là bên phải, ROS omega âm là rẽ phải.
        gamma = -2.0 * self.e_x_f / max(1e-4, L_d * L_d)
        omega_pp = float(self.get_parameter("k_pp").value) * v_target * gamma

        # 6. PD correction:
        #    omega_pd = -Ktheta*theta - Kdx*d(e_x)/dt - Kdtheta*d(theta)/dt
        k_theta = float(self.get_parameter("k_theta").value)
        kd_lateral = float(self.get_parameter("kd_lateral").value)
        kd_theta = float(self.get_parameter("kd_theta").value)

        omega_pd = (
            -k_theta * self.theta_f
            -kd_lateral * self.de_x_f
            -kd_theta * self.dtheta_f
        )

        omega_target = omega_pp + omega_pd

        if bool(self.get_parameter("invert_angular").value):
            omega_target = -omega_target

        if abs(omega_target) < float(self.get_parameter("omega_deadband").value):
            omega_target = 0.0

        omega_max = abs(float(self.get_parameter("omega_max").value))
        omega_target = clamp(omega_target, -omega_max, omega_max)

        # 7. Đổi omega thành chênh lệch vận tốc hai bên:
        #    delta_v = v_right - v_left = omega * wheel_separation.
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))
        delta_v_target = omega_target * wheel_sep

        max_delta_v = abs(float(self.get_parameter("max_delta_v").value))
        delta_v_target = clamp(delta_v_target, -max_delta_v, max_delta_v)

        inner_min_fraction = clamp(
            float(self.get_parameter("inner_wheel_min_fraction").value),
            0.0,
            0.90
        )

        # Không cho bánh trong cua bị chậm quá nhiều hoặc đảo chiều.
        max_delta_from_inner = 2.0 * v_target * (1.0 - inner_min_fraction)
        delta_v_target = clamp(delta_v_target, -max_delta_from_inner, max_delta_from_inner)

        if abs(delta_v_target) > 0.70 * max_delta_v:
            v_target = max(v_turn_min, min(v_target, v_max * 0.75))

        # 8. Làm mượt v và delta_v.
        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        delta_rate = max(0.01, float(self.get_parameter("delta_v_rate_limit").value))

        self.v_cmd = rate_limit(v_target, self.v_cmd, v_rate * dt)
        self.delta_v_cmd = rate_limit(delta_v_target, self.delta_v_cmd, delta_rate * dt)

        self.omega_cmd = self.delta_v_cmd / wheel_sep

        self.publish_cmd(self.v_cmd, self.omega_cmd)

        v_left_est = self.v_cmd - self.omega_cmd * wheel_sep * 0.5
        v_right_est = self.v_cmd + self.omega_cmd * wheel_sep * 0.5

        self.publish_debug(
            "tracking_pure_pursuit_pd_mainlane",
            {
                "e_x_m_raw": e_x,
                "L_d_m": L_d,
                "theta_raw": theta,
                "e_x_m_filtered": self.e_x_f,
                "theta_rad_filtered": self.theta_f,
                "de_x_f": self.de_x_f,
                "dtheta_f": self.dtheta_f,
                "kappa_m": kappa_m,
                "gamma": gamma,
                "v_target": v_target,
                "omega_pp": omega_pp,
                "omega_pd": omega_pd,
                "omega_target": omega_target,
                "delta_v_target": delta_v_target,
                "v_left_est_now": v_left_est,
                "v_right_est_now": v_right_est,
            }
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurPersuitPDMainlaneFollowing()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
