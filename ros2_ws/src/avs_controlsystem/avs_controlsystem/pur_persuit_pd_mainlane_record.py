#!/usr/bin/env python3

import csv
import json
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from avs_controlsystem.pur_persuit_pd_mainlane_following import (
    PurPersuitPDMainlaneFollowing,
)


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurPersuitPDMainlaneRecord(PurPersuitPDMainlaneFollowing):
    """
    Node record dựa trên node cũ pur_persuit_pd_mainlane_following.

    Điểm quan trọng:
      - Không sửa công thức điều khiển cũ.
      - Không sửa cách tính v_cmd, omega_cmd.
      - Không sửa Pure Pursuit + PD.
      - Chỉ thêm:
          + nhấn ENTER để bắt đầu chạy
          + ghi video camera thành .avi
          + ghi trajectory.csv
          + ghi motor_command.csv
          + lưu trajectory.jpg bằng matplotlib
          + lưu velocity_omega.jpg bằng matplotlib

    Vì vậy xe sẽ chạy giống node cũ.
    """

    def __init__(self):
        super().__init__()

        # Tham số thêm cho record/log.
        self.declare_parameter("wait_for_enter_start", True)
        self.declare_parameter("record_video", True)
        self.declare_parameter("use_odom", True)

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("run_data_dir", "/workspace/ros2_ws/src/avs_controlsystem/run_data")

        self.declare_parameter("video_fps", 20.0)
        self.declare_parameter("wheel_diameter_m", 0.065)
        self.declare_parameter("csv_flush_period_s", 1.0)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.started = not bool(self.get_parameter("wait_for_enter_start").value)
        self.start_time = time.time()

        # Pose log theo hệ đồ án:
        # X phải, Y trước, theta = 0 hướng +Y.
        self.x_right_m = 0.0
        self.y_forward_m = 0.0
        self.theta_pose_rad = 0.0

        self.have_odom = False
        self.odom_x0 = None
        self.odom_y0 = None
        self.odom_yaw0 = None

        self.last_pose_update_t = time.time()
        self.last_flush_t = time.time()

        self.trajectory_points = []
        self.command_points = []

        self.video_writer = None
        self.video_frame_count = 0

        self.setup_record_files()

        if bool(self.get_parameter("use_odom").value):
            self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

        if bool(self.get_parameter("record_video").value):
            self.create_subscription(Image, self.camera_topic, self.camera_callback, 10)

        if bool(self.get_parameter("wait_for_enter_start").value):
            threading.Thread(target=self.wait_for_enter_thread, daemon=True).start()

        # Ghi đè signal handler để Ctrl+C vừa dừng xe vừa đóng video/CSV/ảnh.
        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        self.get_logger().warn("RECORD NODE READY")
        self.get_logger().warn("Core điều khiển giữ nguyên từ pur_persuit_pd_mainlane_following.")
        self.get_logger().warn(f"Run data: {self.run_dir}")
        self.get_logger().warn(f"Camera topic: {self.camera_topic}")
        self.get_logger().warn(f"Odom topic: {self.odom_topic}")

        if bool(self.get_parameter("wait_for_enter_start").value):
            self.get_logger().warn("Nhấn ENTER trong terminal này để bắt đầu chạy xe và quay video.")

    def setup_record_files(self):
        base_dir = Path(str(self.get_parameter("run_data_dir").value))
        base_dir.mkdir(parents=True, exist_ok=True)

        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = base_dir / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.video_path = self.run_dir / "camera_video.avi"
        self.trajectory_csv_path = self.run_dir / "trajectory.csv"
        self.motor_csv_path = self.run_dir / "motor_command.csv"
        self.trajectory_jpg_path = self.run_dir / "trajectory.jpg"
        self.velocity_jpg_path = self.run_dir / "velocity_omega.jpg"
        self.summary_json_path = self.run_dir / "summary.json"

        self.traj_file = open(self.trajectory_csv_path, "w", newline="")
        self.motor_file = open(self.motor_csv_path, "w", newline="")

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
            ],
        )
        self.traj_writer.writeheader()

        self.motor_writer = csv.DictWriter(
            self.motor_file,
            fieldnames=[
                "t_s",
                "mode",
                "v_cmd_mps",
                "omega_cmd_radps",
                "delta_v_cmd_mps",
                "v_left_est_mps",
                "v_right_est_mps",
                "rpm_front_left",
                "rpm_rear_left",
                "rpm_front_right",
                "rpm_rear_right",
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
            ],
        )
        self.motor_writer.writeheader()

        summary = {
            "created_at": datetime.now().isoformat(),
            "run_dir": str(self.run_dir),
            "note": "Node này giữ nguyên control core cũ, chỉ thêm record video/csv/jpg.",
            "files": {
                "camera_video": str(self.video_path),
                "trajectory_csv": str(self.trajectory_csv_path),
                "motor_csv": str(self.motor_csv_path),
                "trajectory_jpg": str(self.trajectory_jpg_path),
                "velocity_jpg": str(self.velocity_jpg_path),
            },
            "topics": {
                "control_error_topic": str(self.get_parameter("control_error_topic").value),
                "cmd_vel_topic": str(self.get_parameter("cmd_vel_topic").value),
                "debug_topic": str(self.get_parameter("debug_topic").value),
                "camera_topic": str(self.get_parameter("camera_topic").value),
                "odom_topic": str(self.get_parameter("odom_topic").value),
            },
            "control_params_at_start": {
                "v_max": float(self.get_parameter("v_max").value),
                "v_min": float(self.get_parameter("v_min").value),
                "v_turn_min": float(self.get_parameter("v_turn_min").value),
                "k_c": float(self.get_parameter("k_c").value),
                "k_pp": float(self.get_parameter("k_pp").value),
                "k_theta": float(self.get_parameter("k_theta").value),
                "kd_lateral": float(self.get_parameter("kd_lateral").value),
                "kd_theta": float(self.get_parameter("kd_theta").value),
                "omega_max": float(self.get_parameter("omega_max").value),
            },
        }

        with open(self.summary_json_path, "w") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

    def wait_for_enter_thread(self):
        print("")
        print("======================================================")
        print("  Node record đã sẵn sàng.")
        print("  Nhấn ENTER để BẮT ĐẦU chạy xe + quay video.")
        print("  Nhấn Ctrl+C để DỪNG xe và lưu video/CSV/JPG.")
        print("======================================================")
        print("")
        try:
            sys.stdin.readline()
            self.started = True
            self.start_time = time.time()
            self.last_pose_update_t = time.time()
            self.get_logger().warn("START: xe bắt đầu chạy, bắt đầu record.")
        except Exception as exc:
            self.get_logger().warn(f"Không đọc được ENTER: {exc}")

    def run_time(self):
        return time.time() - self.start_time

    def control_loop(self):
        # Chưa nhấn ENTER thì luôn publish zero, không chạy controller.
        if not self.started:
            self.hard_stop("waiting_for_enter_start")
            return

        # Sau khi START, dùng nguyên control_loop của code cũ.
        super().control_loop()

    def odom_callback(self, msg):
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
            dyaw = wrap_pi(yaw - self.odom_yaw0)

            # ROS odom thường x tiến, y trái.
            # Đổi sang hệ đồ án: X phải, Y trước.
            self.y_forward_m = dx
            self.x_right_m = -dy
            self.theta_pose_rad = dyaw
            self.have_odom = True

        except Exception as exc:
            self.get_logger().warn(f"Invalid odom: {exc}")

    def update_dead_reckoning(self):
        now = time.time()
        dt = max(now - self.last_pose_update_t, 1e-3)
        self.last_pose_update_t = now

        if bool(self.get_parameter("use_odom").value) and self.have_odom:
            return

        self.theta_pose_rad = wrap_pi(self.theta_pose_rad + self.omega_cmd * dt)
        self.x_right_m += self.v_cmd * math.sin(self.theta_pose_rad) * dt
        self.y_forward_m += self.v_cmd * math.cos(self.theta_pose_rad) * dt

    def image_to_bgr(self, msg):
        try:
            import numpy as np
            import cv2

            h = int(msg.height)
            w = int(msg.width)
            enc = msg.encoding.lower()

            if enc == "bgr8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                return frame.copy()

            if enc == "rgb8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            if enc == "mono8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            if enc == "bgra8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            if enc == "rgba8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            if enc in ["yuyv", "yuv422", "yuyv422"]:
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 2))
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)

            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return None

        except Exception as exc:
            self.get_logger().warn(f"Convert image failed: {exc}")
            return None

    def camera_callback(self, msg):
        if not self.started:
            return

        if not bool(self.get_parameter("record_video").value):
            return

        frame = self.image_to_bgr(msg)
        if frame is None:
            return

        try:
            import cv2

            if self.video_writer is None:
                h, w = frame.shape[:2]
                fps = max(1.0, float(self.get_parameter("video_fps").value))
                fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                self.video_writer = cv2.VideoWriter(str(self.video_path), fourcc, fps, (w, h))

                if not self.video_writer.isOpened():
                    self.get_logger().error(f"Cannot open video writer: {self.video_path}")
                    self.video_writer = None
                    return

                self.get_logger().warn(f"Recording video to: {self.video_path}")

            self.video_writer.write(frame)
            self.video_frame_count += 1

        except Exception as exc:
            self.get_logger().warn(f"Video write failed: {exc}")

    def wheel_estimates_for_log(self):
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))
        wheel_diam = max(0.01, float(self.get_parameter("wheel_diameter_m").value))
        circumference = math.pi * wheel_diam

        v_left = self.v_cmd - self.omega_cmd * wheel_sep * 0.5
        v_right = self.v_cmd + self.omega_cmd * wheel_sep * 0.5

        rpm_left = v_left * 60.0 / circumference
        rpm_right = v_right * 60.0 / circumference

        return v_left, v_right, rpm_left, rpm_right

    def publish_debug(self, mode, extra=None):
        # Gọi debug cũ trước để không đổi hành vi node cũ.
        super().publish_debug(mode, extra)

        # Sau đó mới log dữ liệu.
        if self.started:
            self.update_dead_reckoning()
            self.log_data(mode, extra)

    def log_data(self, mode, extra=None):
        try:
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
            self.trajectory_points.append(traj_row)

            v_left, v_right, rpm_left, rpm_right = self.wheel_estimates_for_log()
            err = self.last_error or {}

            motor_row = {
                "t_s": t_s,
                "mode": mode,
                "v_cmd_mps": self.v_cmd,
                "omega_cmd_radps": self.omega_cmd,
                "delta_v_cmd_mps": self.delta_v_cmd,
                "v_left_est_mps": v_left,
                "v_right_est_mps": v_right,
                "rpm_front_left": rpm_left,
                "rpm_rear_left": rpm_left,
                "rpm_front_right": rpm_right,
                "rpm_rear_right": rpm_right,
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
                "error_age_s": time.time() - self.last_error_time if self.last_error_time > 0 else -1.0,
            }

            if extra:
                keymap = {
                    "kappa_m": "kappa_m",
                    "gamma": "gamma",
                    "v_target": "v_target_mps",
                    "omega_pp": "omega_pp_radps",
                    "omega_pd": "omega_pd_radps",
                    "omega_target": "omega_target_radps",
                    "delta_v_target": "delta_v_target_mps",
                }
                for src, dst in keymap.items():
                    if src in extra:
                        motor_row[dst] = extra[src]

            self.motor_writer.writerow(motor_row)
            self.command_points.append(motor_row)

            now = time.time()
            flush_period = max(0.2, float(self.get_parameter("csv_flush_period_s").value))
            if now - self.last_flush_t >= flush_period:
                self.traj_file.flush()
                self.motor_file.flush()
                os.fsync(self.traj_file.fileno())
                os.fsync(self.motor_file.fileno())
                self.last_flush_t = now

        except Exception as exc:
            self.get_logger().warn(f"Log data failed: {exc}")

    def save_plots(self):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.get_logger().error(f"Không import được matplotlib: {exc}")
            return

        try:
            if len(self.trajectory_points) >= 2:
                xs = [float(p["x_right_m"]) for p in self.trajectory_points]
                ys = [float(p["y_forward_m"]) for p in self.trajectory_points]

                plt.figure(figsize=(7, 7))
                plt.plot(xs, ys, linewidth=2)
                plt.scatter([xs[0]], [ys[0]], marker="o", label="Start")
                plt.scatter([xs[-1]], [ys[-1]], marker="x", label="End")
                plt.xlabel("X right (m)")
                plt.ylabel("Y forward (m)")
                plt.title("Robot trajectory")
                plt.grid(True)
                plt.axis("equal")
                plt.legend()
                plt.tight_layout()
                plt.savefig(self.trajectory_jpg_path, dpi=160)
                plt.close()

            if len(self.command_points) >= 2:
                ts = [float(p["t_s"]) for p in self.command_points]
                vs = [float(p["v_cmd_mps"]) for p in self.command_points]
                ws = [float(p["omega_cmd_radps"]) for p in self.command_points]
                vls = [float(p["v_left_est_mps"]) for p in self.command_points]
                vrs = [float(p["v_right_est_mps"]) for p in self.command_points]

                plt.figure(figsize=(10, 6))
                plt.plot(ts, vs, label="linear.x v_cmd (m/s)", linewidth=2)
                plt.plot(ts, ws, label="angular.z omega_cmd (rad/s)", linewidth=2)
                plt.plot(ts, vls, label="v_left_est (m/s)", linestyle="--")
                plt.plot(ts, vrs, label="v_right_est (m/s)", linestyle="--")
                plt.xlabel("Time (s)")
                plt.ylabel("Value")
                plt.title("Velocity and angular command")
                plt.grid(True)
                plt.legend()
                plt.tight_layout()
                plt.savefig(self.velocity_jpg_path, dpi=160)
                plt.close()

            self.get_logger().warn(f"Saved trajectory image: {self.trajectory_jpg_path}")
            self.get_logger().warn(f"Saved velocity image:   {self.velocity_jpg_path}")

        except Exception as exc:
            self.get_logger().error(f"Save plot failed: {exc}")

    def close_all_outputs(self):
        try:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
                self.get_logger().warn(f"Saved video: {self.video_path}")
        except Exception as exc:
            self.get_logger().warn(f"Release video failed: {exc}")

        try:
            self.traj_file.flush()
            self.motor_file.flush()
            os.fsync(self.traj_file.fileno())
            os.fsync(self.motor_file.fileno())
        except Exception:
            pass

        try:
            self.save_plots()
        except Exception:
            pass

        try:
            self.traj_file.close()
            self.motor_file.close()
        except Exception:
            pass

        try:
            old = {}
            if self.summary_json_path.exists():
                with open(self.summary_json_path, "r") as f:
                    old = json.load(f)

            old.update({
                "finished_at": datetime.now().isoformat(),
                "duration_s": self.run_time(),
                "video_frames": self.video_frame_count,
                "trajectory_points": len(self.trajectory_points),
                "command_points": len(self.command_points),
            })

            with open(self.summary_json_path, "w") as f:
                json.dump(old, f, indent=2, ensure_ascii=False)

        except Exception as exc:
            self.get_logger().warn(f"Update summary failed: {exc}")

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Stop robot and save outputs.")
        self.safe_stop_robot()
        self.close_all_outputs()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PurPersuitPDMainlaneRecord()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop_robot()
        node.close_all_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
