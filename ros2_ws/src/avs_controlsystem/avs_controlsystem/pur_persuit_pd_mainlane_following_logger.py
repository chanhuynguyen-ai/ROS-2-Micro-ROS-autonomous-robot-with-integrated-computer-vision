#!/usr/bin/env python3

import csv
import json
import math
import os
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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


def yaw_from_quaternion(q):
    """
    Convert quaternion to yaw.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurPersuitPDMainlaneFollowingLogger(Node):
    """
    Pure Pursuit + PD mainlane controller có logging quỹ đạo.

    Subscribe:
      /avs/control_error [std_msgs/String]
      /odom              [nav_msgs/Odometry] optional

    Publish:
      /cmd_vel [geometry_msgs/Twist]
      /avs/pur_persuit_pd_mainlane_debug [std_msgs/String]

    Không lưu ảnh camera.
    Chỉ lưu:
      - CSV quỹ đạo
      - CSV vận tốc/góc quay
      - PNG quỹ đạo
      - PNG vận tốc và omega
    """

    def __init__(self):
        super().__init__("pur_persuit_pd_mainlane_following_logger")

        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/avs/pur_persuit_pd_mainlane_debug")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter("enable_motion", True)
        self.declare_parameter("use_odom", True)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("error_timeout_s", 1.5)

        # Tốc độ.
        self.declare_parameter("v_max", 0.10)
        self.declare_parameter("v_min", 0.025)
        self.declare_parameter("v_turn_min", 0.035)

        # Pure Pursuit + PD.
        self.declare_parameter("k_c", 0.20)
        self.declare_parameter("k_pp", 1.00)
        self.declare_parameter("k_theta", 0.22)
        self.declare_parameter("kd_lateral", 0.004)
        self.declare_parameter("kd_theta", 0.008)

        # Giới hạn quay.
        self.declare_parameter("omega_max", 0.35)
        self.declare_parameter("wheel_separation_m", 0.36)
        self.declare_parameter("max_delta_v", 0.085)
        self.declare_parameter("delta_v_rate_limit", 0.16)
        self.declare_parameter("inner_wheel_min_fraction", 0.45)

        # Lọc.
        self.declare_parameter("filter_alpha", 0.18)
        self.declare_parameter("derivative_alpha", 0.25)
        self.declare_parameter("v_rate_limit", 0.12)

        # Deadband.
        self.declare_parameter("x_deadband_m", 0.008)
        self.declare_parameter("theta_deadband_rad", 0.012)
        self.declare_parameter("omega_deadband", 0.006)

        # Lookahead.
        self.declare_parameter("ld_min_m", 0.14)
        self.declare_parameter("ld_max_m", 0.85)
        self.declare_parameter("default_lookahead_m", 0.30)

        # Safety.
        self.declare_parameter("max_abs_x_m", 0.45)
        self.declare_parameter("max_abs_theta_rad", 1.30)
        self.declare_parameter("invert_angular", False)

        # Logging.
        self.declare_parameter("enable_logging", True)
        self.declare_parameter("run_data_dir", "/workspace/ros2_ws/src/avs_controlsystem/run_data")
        self.declare_parameter("save_image_period_s", 2.0)
        self.declare_parameter("csv_flush_period_s", 1.0)
        self.declare_parameter("max_points_in_memory", 20000)

        self.control_error_topic = str(self.get_parameter("control_error_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.create_subscription(String, self.control_error_topic, self.control_error_callback, 10)

        if bool(self.get_parameter("use_odom").value):
            self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

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
        self.start_time = time.time()

        # Pose dùng cho vẽ quỹ đạo.
        # Quy ước log nội bộ:
        # x_right_m: trục X dương sang phải
        # y_forward_m: trục Y dương về phía trước
        # theta_rad: theta = 0 hướng +Y, omega dương quay trái
        self.x_right_m = 0.0
        self.y_forward_m = 0.0
        self.theta_pose_rad = 0.0

        self.have_odom = False
        self.last_odom_time = -1.0
        self.odom_x0 = None
        self.odom_y0 = None
        self.odom_yaw0 = None

        self.last_plot_time = 0.0
        self.last_flush_time = time.time()

        self.trajectory_points = []
        self.command_points = []

        self.setup_run_data()

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("pur_persuit_pd_mainlane_following_logger started")
        self.get_logger().info(f"Subscribe control error: {self.control_error_topic}")
        self.get_logger().info(f"Subscribe odom:          {self.odom_topic}")
        self.get_logger().info(f"Publish cmd_vel:         {self.cmd_vel_topic}")
        self.get_logger().info(f"Run data dir:            {self.run_dir}")

    def setup_run_data(self):
        self.enable_logging = bool(self.get_parameter("enable_logging").value)

        base_dir = Path(str(self.get_parameter("run_data_dir").value))
        base_dir.mkdir(parents=True, exist_ok=True)

        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = base_dir / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.trajectory_csv_path = self.run_dir / "trajectory.csv"
        self.command_csv_path = self.run_dir / "command_log.csv"
        self.summary_json_path = self.run_dir / "summary.json"
        self.trajectory_png_path = self.run_dir / "trajectory.png"
        self.velocity_png_path = self.run_dir / "velocity_omega.png"

        self.traj_file = None
        self.cmd_file = None
        self.traj_writer = None
        self.cmd_writer = None

        if self.enable_logging:
            self.traj_file = open(self.trajectory_csv_path, "w", newline="")
            self.cmd_file = open(self.command_csv_path, "w", newline="")

            self.traj_writer = csv.DictWriter(
                self.traj_file,
                fieldnames=[
                    "t_s",
                    "source",
                    "x_right_m",
                    "y_forward_m",
                    "theta_rad",
                    "theta_deg",
                    "odom_available",
                ]
            )
            self.traj_writer.writeheader()

            self.cmd_writer = csv.DictWriter(
                self.cmd_file,
                fieldnames=[
                    "t_s",
                    "mode",
                    "v_cmd_mps",
                    "omega_cmd_radps",
                    "delta_v_cmd_mps",
                    "v_left_est_mps",
                    "v_right_est_mps",
                    "epsilon_x_mm",
                    "epsilon_y_mm",
                    "theta_rad",
                    "curvature_inv_mm",
                    "confidence",
                    "e_x_m_filtered",
                    "theta_rad_filtered",
                    "de_x_f",
                    "dtheta_f",
                    "kappa_m",
                    "gamma",
                    "v_target_mps",
                    "omega_pp_radps",
                    "omega_pd_radps",
                    "omega_target_radps",
                    "delta_v_target_mps",
                    "error_age_s",
                ]
            )
            self.cmd_writer.writeheader()

            summary = {
                "run_dir": str(self.run_dir),
                "created_at": datetime.now().isoformat(),
                "note": "Không lưu ảnh camera. Chỉ lưu ảnh quỹ đạo, vận tốc, góc quay và CSV.",
                "topics": {
                    "control_error_topic": self.control_error_topic,
                    "cmd_vel_topic": self.cmd_vel_topic,
                    "debug_topic": self.debug_topic,
                    "odom_topic": self.odom_topic,
                },
                "parameters": {
                    "v_max": float(self.get_parameter("v_max").value),
                    "v_min": float(self.get_parameter("v_min").value),
                    "v_turn_min": float(self.get_parameter("v_turn_min").value),
                    "k_c": float(self.get_parameter("k_c").value),
                    "k_pp": float(self.get_parameter("k_pp").value),
                    "k_theta": float(self.get_parameter("k_theta").value),
                    "kd_lateral": float(self.get_parameter("kd_lateral").value),
                    "kd_theta": float(self.get_parameter("kd_theta").value),
                    "omega_max": float(self.get_parameter("omega_max").value),
                    "wheel_separation_m": float(self.get_parameter("wheel_separation_m").value),
                    "max_delta_v": float(self.get_parameter("max_delta_v").value),
                }
            }
            with open(self.summary_json_path, "w") as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)

    def now(self):
        return time.time()

    def run_time(self):
        return self.now() - self.start_time

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

    def odom_callback(self, msg):
        """
        Dùng /odom nếu có để lấy quỹ đạo thực.

        ROS odom thường:
          odom x: hướng tiến
          odom y: hướng trái

        Đổi sang quy ước đồ án:
          y_forward = odom_x - odom_x0
          x_right   = -(odom_y - odom_y0)
        """
        try:
            ox = float(msg.pose.pose.position.x)
            oy = float(msg.pose.pose.position.y)
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)

            if self.odom_x0 is None:
                self.odom_x0 = ox
                self.odom_y0 = oy
                self.odom_yaw0 = yaw

            dx = ox - self.odom_x0
            dy = oy - self.odom_y0
            dyaw = yaw - self.odom_yaw0

            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi

            self.y_forward_m = dx
            self.x_right_m = -dy
            self.theta_pose_rad = dyaw

            self.have_odom = True
            self.last_odom_time = self.now()

        except Exception as exc:
            self.get_logger().warn(f"Invalid odom: {exc}")

    def update_dead_reckoning(self, dt):
        """
        Khi không có /odom thì ước lượng quỹ đạo từ v_cmd và omega_cmd.

        Quy ước:
          x_right_dot   = v*sin(theta)
          y_forward_dot = v*cos(theta)
          theta_dot     = omega
        """
        if bool(self.get_parameter("use_odom").value) and self.have_odom:
            return

        self.theta_pose_rad += self.omega_cmd * dt

        while self.theta_pose_rad > math.pi:
            self.theta_pose_rad -= 2.0 * math.pi
        while self.theta_pose_rad < -math.pi:
            self.theta_pose_rad += 2.0 * math.pi

        self.x_right_m += self.v_cmd * math.sin(self.theta_pose_rad) * dt
        self.y_forward_m += self.v_cmd * math.cos(self.theta_pose_rad) * dt

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

    def wheel_estimates(self):
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))
        v_left_est = self.v_cmd - self.omega_cmd * wheel_sep * 0.5
        v_right_est = self.v_cmd + self.omega_cmd * wheel_sep * 0.5
        return v_left_est, v_right_est

    def publish_debug(self, mode, extra=None):
        v_left_est, v_right_est = self.wheel_estimates()

        payload = {
            "mode": mode,
            "enable_motion": bool(self.get_parameter("enable_motion").value),
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "delta_v_cmd": self.delta_v_cmd,
            "v_left_est": v_left_est,
            "v_right_est": v_right_est,
            "x_right_m": self.x_right_m,
            "y_forward_m": self.y_forward_m,
            "theta_pose_rad": self.theta_pose_rad,
            "run_dir": str(self.run_dir),
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

    def log_data(self, mode, extra=None):
        if not self.enable_logging:
            return

        t_s = self.run_time()
        source = "odom" if self.have_odom else "dead_reckoning"

        traj_row = {
            "t_s": t_s,
            "source": source,
            "x_right_m": self.x_right_m,
            "y_forward_m": self.y_forward_m,
            "theta_rad": self.theta_pose_rad,
            "theta_deg": math.degrees(self.theta_pose_rad),
            "odom_available": self.have_odom,
        }
        self.traj_writer.writerow(traj_row)

        v_left_est, v_right_est = self.wheel_estimates()

        err = self.last_error or {}
        cmd_row = {
            "t_s": t_s,
            "mode": mode,
            "v_cmd_mps": self.v_cmd,
            "omega_cmd_radps": self.omega_cmd,
            "delta_v_cmd_mps": self.delta_v_cmd,
            "v_left_est_mps": v_left_est,
            "v_right_est_mps": v_right_est,
            "epsilon_x_mm": err.get("epsilon_x_mm", ""),
            "epsilon_y_mm": err.get("epsilon_y_mm", ""),
            "theta_rad": err.get("theta_rad", ""),
            "curvature_inv_mm": err.get("curvature_inv_mm", ""),
            "confidence": err.get("confidence", ""),
            "e_x_m_filtered": self.e_x_f,
            "theta_rad_filtered": self.theta_f,
            "de_x_f": self.de_x_f,
            "dtheta_f": self.dtheta_f,
            "kappa_m": "",
            "gamma": "",
            "v_target_mps": "",
            "omega_pp_radps": "",
            "omega_pd_radps": "",
            "omega_target_radps": "",
            "delta_v_target_mps": "",
            "error_age_s": self.now() - self.last_error_time if self.last_error_time > 0 else -1.0,
        }

        if extra:
            mapping = {
                "kappa_m": "kappa_m",
                "gamma": "gamma",
                "v_target": "v_target_mps",
                "omega_pp": "omega_pp_radps",
                "omega_pd": "omega_pd_radps",
                "omega_target": "omega_target_radps",
                "delta_v_target": "delta_v_target_mps",
            }
            for src_key, dst_key in mapping.items():
                if src_key in extra:
                    cmd_row[dst_key] = extra[src_key]

        self.cmd_writer.writerow(cmd_row)

        max_points = int(self.get_parameter("max_points_in_memory").value)
        self.trajectory_points.append(traj_row)
        self.command_points.append(cmd_row)

        if len(self.trajectory_points) > max_points:
            self.trajectory_points = self.trajectory_points[-max_points:]
        if len(self.command_points) > max_points:
            self.command_points = self.command_points[-max_points:]

        now = self.now()
        flush_period = max(0.2, float(self.get_parameter("csv_flush_period_s").value))
        if now - self.last_flush_time >= flush_period:
            self.traj_file.flush()
            self.cmd_file.flush()
            os.fsync(self.traj_file.fileno())
            os.fsync(self.cmd_file.fileno())
            self.last_flush_time = now

    def save_plots_if_needed(self, force=False):
        if not self.enable_logging:
            return

        period = max(0.5, float(self.get_parameter("save_image_period_s").value))
        now = self.now()

        if not force and now - self.last_plot_time < period:
            return

        self.last_plot_time = now

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().warn(f"Cannot import matplotlib, skip plot saving: {exc}")
            return

        try:
            if len(self.trajectory_points) >= 2:
                xs = [p["x_right_m"] for p in self.trajectory_points]
                ys = [p["y_forward_m"] for p in self.trajectory_points]

                plt.figure(figsize=(7, 7))
                plt.plot(xs, ys, linewidth=2)
                plt.scatter([xs[0]], [ys[0]], marker="o", label="Start")
                plt.scatter([xs[-1]], [ys[-1]], marker="x", label="Current/End")
                plt.xlabel("X right (m)")
                plt.ylabel("Y forward (m)")
                plt.title("Robot trajectory")
                plt.axis("equal")
                plt.grid(True)
                plt.legend()
                plt.tight_layout()
                plt.savefig(self.trajectory_png_path, dpi=150)
                plt.close()

            if len(self.command_points) >= 2:
                ts = [p["t_s"] for p in self.command_points]
                vs = [p["v_cmd_mps"] for p in self.command_points]
                ws = [p["omega_cmd_radps"] for p in self.command_points]
                vls = [p["v_left_est_mps"] for p in self.command_points]
                vrs = [p["v_right_est_mps"] for p in self.command_points]

                plt.figure(figsize=(10, 6))
                plt.plot(ts, vs, label="v_cmd linear.x (m/s)", linewidth=2)
                plt.plot(ts, ws, label="omega_cmd angular.z (rad/s)", linewidth=2)
                plt.plot(ts, vls, label="v_left_est (m/s)", linestyle="--")
                plt.plot(ts, vrs, label="v_right_est (m/s)", linestyle="--")
                plt.xlabel("Time (s)")
                plt.ylabel("Value")
                plt.title("Velocity and angular command")
                plt.grid(True)
                plt.legend()
                plt.tight_layout()
                plt.savefig(self.velocity_png_path, dpi=150)
                plt.close()

        except Exception as exc:
            self.get_logger().warn(f"Plot saving failed: {exc}")

    def hard_stop(self, mode):
        self.v_cmd = 0.0
        self.delta_v_cmd = 0.0
        self.omega_cmd = 0.0
        self.publish_cmd(0.0, 0.0)
        self.publish_debug(mode)
        self.log_data(mode)
        self.save_plots_if_needed()

    def ramp_stop(self, dt, mode):
        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        delta_rate = max(0.01, float(self.get_parameter("delta_v_rate_limit").value))
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))

        self.v_cmd = rate_limit(0.0, self.v_cmd, v_rate * dt)
        self.delta_v_cmd = rate_limit(0.0, self.delta_v_cmd, delta_rate * dt)
        self.omega_cmd = self.delta_v_cmd / wheel_sep

        self.update_dead_reckoning(dt)
        self.publish_cmd(self.v_cmd, self.omega_cmd)
        self.publish_debug(mode)
        self.log_data(mode)
        self.save_plots_if_needed()

    def safe_stop_robot(self, repeat=35):
        stop = self.make_cmd(0.0, 0.0)
        for _ in range(repeat):
            self.cmd_pub.publish(stop)
            time.sleep(0.02)

    def close_logs(self):
        try:
            self.save_plots_if_needed(force=True)
        except Exception:
            pass

        try:
            if self.traj_file:
                self.traj_file.flush()
                os.fsync(self.traj_file.fileno())
                self.traj_file.close()
                self.traj_file = None
        except Exception:
            pass

        try:
            if self.cmd_file:
                self.cmd_file.flush()
                os.fsync(self.cmd_file.fileno())
                self.cmd_file.close()
                self.cmd_file = None
        except Exception:
            pass

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Sending repeated zero /cmd_vel.")
        self.safe_stop_robot()
        self.close_logs()
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

        # 4. Tính v:
        #    v = v_max*cos(theta)/(1+k_c*abs(kappa_m))
        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v_turn_min = float(self.get_parameter("v_turn_min").value)
        k_c = float(self.get_parameter("k_c").value)

        cos_theta = max(0.0, math.cos(self.theta_f))
        v_target = (v_max * cos_theta) / (1.0 + k_c * abs(kappa_m))
        v_target = clamp(v_target, v_min, v_max)

        # 5. Pure Pursuit:
        gamma = -2.0 * self.e_x_f / max(1e-4, L_d * L_d)
        omega_pp = float(self.get_parameter("k_pp").value) * v_target * gamma

        # 6. PD correction:
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

        # 7. Đổi omega thành delta_v.
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))
        delta_v_target = omega_target * wheel_sep

        max_delta_v = abs(float(self.get_parameter("max_delta_v").value))
        delta_v_target = clamp(delta_v_target, -max_delta_v, max_delta_v)

        inner_min_fraction = clamp(
            float(self.get_parameter("inner_wheel_min_fraction").value),
            0.0,
            0.90
        )

        max_delta_from_inner = 2.0 * v_target * (1.0 - inner_min_fraction)
        delta_v_target = clamp(delta_v_target, -max_delta_from_inner, max_delta_from_inner)

        if abs(delta_v_target) > 0.70 * max_delta_v:
            v_target = max(v_turn_min, min(v_target, v_max * 0.75))

        # 8. Rate limit.
        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        delta_rate = max(0.01, float(self.get_parameter("delta_v_rate_limit").value))

        self.v_cmd = rate_limit(v_target, self.v_cmd, v_rate * dt)
        self.delta_v_cmd = rate_limit(delta_v_target, self.delta_v_cmd, delta_rate * dt)
        self.omega_cmd = self.delta_v_cmd / wheel_sep

        # 9. Cập nhật quỹ đạo.
        self.update_dead_reckoning(dt)

        # 10. Publish.
        self.publish_cmd(self.v_cmd, self.omega_cmd)

        v_left_est, v_right_est = self.wheel_estimates()

        extra = {
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

        self.publish_debug("tracking_pure_pursuit_pd_mainlane_logger", extra)
        self.log_data("tracking_pure_pursuit_pd_mainlane_logger", extra)
        self.save_plots_if_needed()


def main(args=None):
    rclpy.init(args=args)
    node = PurPersuitPDMainlaneFollowingLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop_robot()
        node.close_logs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
