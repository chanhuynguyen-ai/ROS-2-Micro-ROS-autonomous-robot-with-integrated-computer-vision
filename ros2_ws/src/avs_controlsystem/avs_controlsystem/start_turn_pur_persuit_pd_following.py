#!/usr/bin/env python3

import csv
import json
import math
import os
import signal
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
    CV_AVAILABLE = True
except Exception:
    CV_AVAILABLE = False


def clamp(value, low, high):
    return max(low, min(high, value))


def rate_limit(target, current, max_delta):
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


def parse_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ["true", "1", "yes", "valid", "visible"]
    if value is None:
        return default
    try:
        return bool(value)
    except Exception:
        return default


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StartTurnPurPersuitPDFollowing(Node):
    """
    Pure Pursuit + PD controller có logic start -> turn_lane -> main_lane.

    Subscribe:
      /avs/control_error       std_msgs/String
      /odom                    nav_msgs/Odometry, optional
      /camera/image_raw        sensor_msgs/Image, optional

    Publish:
      /cmd_vel                 geometry_msgs/Twist
      /avs/start_turn_debug    std_msgs/String
      /avs/run_path            nav_msgs/Path
      /avs/run_marker          visualization_msgs/Marker

    State logic:
      WAIT_START:
        Nếu thấy start_visible và turn_lane_visible -> FOLLOW_TURN_LANE.
        Nếu không thấy turn_lane -> FOLLOW_MAIN_LANE.

      FOLLOW_TURN_LANE:
        Bám turn_lane nếu có lỗi riêng cho turn_lane.
        Nếu turn_lane mất quá turn_lane_lost_confirm_s -> FOLLOW_MAIN_LANE.

      FOLLOW_MAIN_LANE:
        Bám main_lane.
        Nếu vòng lại thấy start nhưng không thấy turn_lane -> vẫn bám main_lane.
    """

    def __init__(self):
        super().__init__("start_turn_pur_persuit_pd_following")

        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/avs/start_turn_debug")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_topic", "/camera/image_raw")

        self.declare_parameter("path_topic", "/avs/run_path")
        self.declare_parameter("marker_topic", "/avs/run_marker")
        self.declare_parameter("path_frame_id", "odom")

        self.declare_parameter("enable_motion", True)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("error_timeout_s", 1.5)

        self.declare_parameter("v_max", 0.10)
        self.declare_parameter("v_min", 0.025)
        self.declare_parameter("v_turn_min", 0.035)
        self.declare_parameter("k_c", 0.20)

        self.declare_parameter("k_pp", 1.00)
        self.declare_parameter("k_theta", 0.22)
        self.declare_parameter("kd_lateral", 0.004)
        self.declare_parameter("kd_theta", 0.008)

        self.declare_parameter("omega_max", 0.35)
        self.declare_parameter("wheel_separation_m", 0.36)
        self.declare_parameter("max_delta_v", 0.085)
        self.declare_parameter("delta_v_rate_limit", 0.16)
        self.declare_parameter("inner_wheel_min_fraction", 0.45)

        self.declare_parameter("filter_alpha", 0.18)
        self.declare_parameter("derivative_alpha", 0.25)
        self.declare_parameter("v_rate_limit", 0.12)

        self.declare_parameter("x_deadband_m", 0.008)
        self.declare_parameter("theta_deadband_rad", 0.012)
        self.declare_parameter("omega_deadband", 0.006)

        self.declare_parameter("ld_min_m", 0.14)
        self.declare_parameter("ld_max_m", 0.85)
        self.declare_parameter("default_lookahead_m", 0.30)

        self.declare_parameter("max_abs_x_m", 0.45)
        self.declare_parameter("max_abs_theta_rad", 1.30)
        self.declare_parameter("invert_angular", False)

        # Logic start/turn/main.
        self.declare_parameter("start_confirm_s", 0.20)
        self.declare_parameter("turn_lane_lost_confirm_s", 0.80)
        self.declare_parameter("prefer_turn_lane_at_start", True)

        # Logging.
        self.declare_parameter("run_data_dir", "/workspace/ros2_ws/src/avs_controlsystem/run_data")
        self.declare_parameter("save_run_data", True)
        self.declare_parameter("save_images", True)
        self.declare_parameter("image_save_period_s", 0.50)
        self.declare_parameter("csv_save_period_s", 0.05)

        self.control_error_topic = str(self.get_parameter("control_error_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.path_frame_id = str(self.get_parameter("path_frame_id").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        self.create_subscription(String, self.control_error_topic, self.control_error_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.bridge = CvBridge() if CV_AVAILABLE else None
        self.latest_image = None
        self.last_image_save_time = 0.0

        self.create_subscription(Image, self.camera_topic, self.image_callback, 5)

        self.state = "FOLLOW_MAIN_LANE"
        self.has_entered_main_road = False
        self.first_start_seen = False
        self.last_start_seen_time = -1.0
        self.last_turn_seen_time = -1.0

        self.last_error = None
        self.last_error_time = -1.0
        self.selected_lane = "main_lane"

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

        self.odom_received = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.dead_x = 0.0
        self.dead_y = 0.0
        self.dead_yaw = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.path_frame_id
        self.path_decimation_count = 0

        self.run_dir = None
        self.csv_file = None
        self.csv_writer = None
        self.last_csv_save_time = 0.0
        self.setup_run_data()

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info("start_turn_pur_persuit_pd_following started")
        self.get_logger().info(f"Subscribe control_error: {self.control_error_topic}")
        self.get_logger().info(f"Publish cmd_vel:         {self.cmd_vel_topic}")
        self.get_logger().info(f"Publish RViz path:       {self.path_topic}")
        self.get_logger().info(f"Run data:                {self.run_dir}")
        if not CV_AVAILABLE:
            self.get_logger().warn("cv_bridge/cv2 not available, image saving disabled.")

    def now(self):
        return time.time()

    def setup_run_data(self):
        if not bool(self.get_parameter("save_run_data").value):
            return

        base = str(self.get_parameter("run_data_dir").value)
        stamp = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(base, stamp)
        os.makedirs(self.run_dir, exist_ok=True)
        os.makedirs(os.path.join(self.run_dir, "images"), exist_ok=True)

        params = {}
        for p in self._parameters:
            try:
                params[p] = self.get_parameter(p).value
            except Exception:
                pass

        with open(os.path.join(self.run_dir, "params.json"), "w", encoding="utf-8") as f:
            json.dump(params, f, ensure_ascii=False, indent=2)

        self.csv_file = open(os.path.join(self.run_dir, "run_log.csv"), "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "time",
                "state",
                "selected_lane",
                "start_visible",
                "turn_lane_visible",
                "main_lane_visible",
                "valid",
                "epsilon_x_mm",
                "epsilon_y_mm",
                "theta_rad",
                "curvature_inv_mm",
                "e_x_f",
                "theta_f",
                "de_x_f",
                "dtheta_f",
                "kappa_m",
                "L_d",
                "gamma",
                "v_target",
                "omega_pp",
                "omega_pd",
                "omega_target",
                "v_cmd",
                "omega_cmd",
                "delta_v_cmd",
                "v_left_est",
                "v_right_est",
                "odom_x",
                "odom_y",
                "odom_yaw",
            ],
        )
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def close_run_data(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

    def image_callback(self, msg):
        self.latest_image = msg

    def save_image_if_needed(self):
        if not bool(self.get_parameter("save_images").value):
            return

        if self.run_dir is None or self.latest_image is None:
            return

        if not CV_AVAILABLE or self.bridge is None:
            return

        now = self.now()
        period = float(self.get_parameter("image_save_period_s").value)
        if now - self.last_image_save_time < period:
            return

        self.last_image_save_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
            filename = os.path.join(self.run_dir, "images", f"img_{now:.3f}_{self.state}_{self.selected_lane}.jpg")
            cv2.imwrite(filename, cv_img)
        except Exception as exc:
            self.get_logger().warn(f"Cannot save image: {exc}")

    def odom_callback(self, msg):
        self.odom_received = True
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def parse_error_block(self, data, key_name):
        """
        Hỗ trợ nhiều format:
        1) data["turn_lane"] = {"epsilon_x_mm": ...}
        2) data["turn_lane_error"] = {...}
        3) data["turn_lane_epsilon_x_mm"] = ...
        4) fallback: data["epsilon_x_mm"] = ...
        """

        possible_nested_keys = [
            key_name,
            f"{key_name}_error",
            f"{key_name}_control_error",
            f"{key_name}_target",
        ]

        block = None
        for k in possible_nested_keys:
            if isinstance(data.get(k), dict):
                block = data.get(k)
                break

        if block is None:
            block = data

        prefix = "" if block is not data else f"{key_name}_"

        default_ld_mm = float(self.get_parameter("default_lookahead_m").value) * 1000.0

        def get_val(names, default):
            for name in names:
                if name in block:
                    return block[name]
                if prefix and prefix + name in data:
                    return data[prefix + name]
            return default

        e_x_mm = float(get_val(["epsilon_x_mm", "x_mm", "lateral_error_mm"], data.get("epsilon_x_mm", 0.0)))
        e_y_mm = float(get_val(["epsilon_y_mm", "lookahead_d_mm", "y_mm"], data.get("epsilon_y_mm", default_ld_mm)))
        theta = float(get_val(["theta_rad", "heading_error_rad"], data.get("theta_rad", 0.0)))
        curvature_inv_mm = float(get_val(["curvature_inv_mm"], data.get("curvature_inv_mm", 0.0)))
        confidence = float(get_val(["confidence", "conf", "prob"], data.get("confidence", 1.0)))

        if not all(math.isfinite(v) for v in [e_x_mm, e_y_mm, theta, curvature_inv_mm, confidence]):
            return None

        return {
            "epsilon_x_mm": e_x_mm,
            "epsilon_y_mm": max(abs(e_y_mm), 50.0),
            "theta_rad": theta,
            "curvature_inv_mm": curvature_inv_mm,
            "confidence": confidence,
        }

    def control_error_callback(self, msg):
        try:
            data = json.loads(msg.data)

            start_visible = (
                parse_bool(data.get("start_visible", False), False)
                or parse_bool(data.get("start_detected", False), False)
                or str(data.get("label", "")).strip() == "start"
                or str(data.get("class_name", "")).strip() == "start"
            )

            turn_lane_visible = (
                parse_bool(data.get("turn_lane_visible", False), False)
                or parse_bool(data.get("turn-lane_visible", False), False)
                or parse_bool(data.get("turn_lane_detected", False), False)
                or str(data.get("label", "")).strip() in ["turn_lane", "turn-lane"]
                or str(data.get("class_name", "")).strip() in ["turn_lane", "turn-lane"]
            )

            main_lane_visible = (
                parse_bool(data.get("main_lane_visible", True), True)
                or parse_bool(data.get("main-lane_visible", True), True)
                or str(data.get("label", "")).strip() in ["main_lane", "main-lane"]
                or str(data.get("class_name", "")).strip() in ["main_lane", "main-lane"]
            )

            valid = parse_bool(data.get("valid", True), True)
            lane_valid = parse_bool(data.get("lane_valid", True), True)

            if start_visible:
                self.last_start_seen_time = self.now()
                self.first_start_seen = True

            if turn_lane_visible:
                self.last_turn_seen_time = self.now()

            main_error = self.parse_error_block(data, "main_lane")
            turn_error = self.parse_error_block(data, "turn_lane")

            if main_error is None:
                main_error = self.parse_error_block(data, "main-lane")

            if turn_error is None:
                turn_error = self.parse_error_block(data, "turn-lane")

            if main_error is None:
                return

            self.update_lane_state(start_visible, turn_lane_visible)

            if self.state == "FOLLOW_TURN_LANE" and turn_error is not None:
                selected = turn_error
                selected_lane = "turn_lane"
            else:
                selected = main_error
                selected_lane = "main_lane"

            self.selected_lane = selected_lane

            self.last_error = {
                "valid": valid and lane_valid,
                "start_visible": start_visible,
                "turn_lane_visible": turn_lane_visible,
                "main_lane_visible": main_lane_visible,
                "selected_lane": selected_lane,
                "epsilon_x_mm": selected["epsilon_x_mm"],
                "epsilon_y_mm": selected["epsilon_y_mm"],
                "theta_rad": selected["theta_rad"],
                "curvature_inv_mm": selected["curvature_inv_mm"],
                "confidence": selected["confidence"],
                "raw": data,
            }

            self.last_error_time = self.now()

        except Exception as exc:
            self.get_logger().warn(f"Invalid /avs/control_error JSON: {exc}")

    def update_lane_state(self, start_visible, turn_lane_visible):
        now = self.now()
        prefer_turn = bool(self.get_parameter("prefer_turn_lane_at_start").value)
        turn_lost_s = float(self.get_parameter("turn_lane_lost_confirm_s").value)

        if not self.has_entered_main_road:
            if prefer_turn and start_visible and turn_lane_visible:
                self.state = "FOLLOW_TURN_LANE"
                return

            if self.state == "FOLLOW_TURN_LANE":
                if (not turn_lane_visible) and self.last_turn_seen_time > 0.0:
                    if now - self.last_turn_seen_time > turn_lost_s:
                        self.has_entered_main_road = True
                        self.state = "FOLLOW_MAIN_LANE"
                        return

            if self.state != "FOLLOW_TURN_LANE":
                self.state = "FOLLOW_MAIN_LANE"
                return

        # Đã ra đường chính.
        # Nếu vòng lại thấy start nhưng không thấy turn_lane thì vẫn giữ mainlane.
        if self.has_entered_main_road:
            self.state = "FOLLOW_MAIN_LANE"

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
            "state": self.state,
            "selected_lane": self.selected_lane,
            "has_entered_main_road": self.has_entered_main_road,
            "enable_motion": bool(self.get_parameter("enable_motion").value),
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "delta_v_cmd": self.delta_v_cmd,
            "v_left_est": v_left_est,
            "v_right_est": v_right_est,
            "odom_received": self.odom_received,
            "odom_x": self.odom_x,
            "odom_y": self.odom_y,
            "odom_yaw": self.odom_yaw,
            "error_age_s": self.now() - self.last_error_time if self.last_error_time > 0 else -1.0,
            "run_dir": self.run_dir or "",
        }

        if self.last_error is not None:
            payload.update({
                "valid": self.last_error["valid"],
                "start_visible": self.last_error["start_visible"],
                "turn_lane_visible": self.last_error["turn_lane_visible"],
                "main_lane_visible": self.last_error["main_lane_visible"],
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

    def publish_rviz(self):
        now_msg = self.get_clock().now().to_msg()

        if self.odom_received:
            x = self.odom_x
            y = self.odom_y
            yaw = self.odom_yaw
        else:
            x = self.dead_x
            y = self.dead_y
            yaw = self.dead_yaw

        self.path_msg.header.stamp = now_msg
        self.path_msg.header.frame_id = self.path_frame_id

        self.path_decimation_count += 1
        if self.path_decimation_count >= 3:
            self.path_decimation_count = 0
            pose = PoseStamped()
            pose.header.stamp = now_msg
            pose.header.frame_id = self.path_frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)

            self.path_msg.poses.append(pose)
            if len(self.path_msg.poses) > 3000:
                self.path_msg.poses = self.path_msg.poses[-3000:]

        self.path_pub.publish(self.path_msg)

        marker = Marker()
        marker.header.stamp = now_msg
        marker.header.frame_id = self.path_frame_id
        marker.ns = "avs_start_turn_controller"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.text = f"{self.state}\\nselected={self.selected_lane}\\nv={self.v_cmd:.3f}, w={self.omega_cmd:.3f}"
        self.marker_pub.publish(marker)

    def log_csv_if_needed(self, values):
        if self.csv_writer is None:
            return

        now = self.now()
        period = float(self.get_parameter("csv_save_period_s").value)
        if now - self.last_csv_save_time < period:
            return

        self.last_csv_save_time = now
        self.csv_writer.writerow(values)
        self.csv_file.flush()

    def hard_stop(self, mode):
        self.v_cmd = 0.0
        self.delta_v_cmd = 0.0
        self.omega_cmd = 0.0
        self.publish_cmd(0.0, 0.0)
        self.publish_debug(mode)
        self.publish_rviz()

    def ramp_stop(self, dt, mode):
        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        delta_rate = max(0.01, float(self.get_parameter("delta_v_rate_limit").value))
        wheel_sep = max(0.05, float(self.get_parameter("wheel_separation_m").value))

        self.v_cmd = rate_limit(0.0, self.v_cmd, v_rate * dt)
        self.delta_v_cmd = rate_limit(0.0, self.delta_v_cmd, delta_rate * dt)
        self.omega_cmd = self.delta_v_cmd / wheel_sep

        self.publish_cmd(self.v_cmd, self.omega_cmd)
        self.publish_debug(mode)
        self.publish_rviz()

    def safe_stop_robot(self, repeat=35):
        stop = self.make_cmd(0.0, 0.0)
        for _ in range(repeat):
            self.cmd_pub.publish(stop)
            time.sleep(0.02)

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}. Sending repeated zero /cmd_vel.")
        self.safe_stop_robot()
        self.close_run_data()
        if rclpy.ok():
            rclpy.shutdown()

    def update_dead_reckoning(self, dt):
        if self.odom_received:
            return

        # Kinematics cùng quy ước trước đó:
        # theta=0 hướng +Y, x dương sang phải.
        self.dead_x += self.v_cmd * math.sin(self.dead_yaw) * dt
        self.dead_y += self.v_cmd * math.cos(self.dead_yaw) * dt
        self.dead_yaw += self.omega_cmd * dt
        self.dead_yaw = math.atan2(math.sin(self.dead_yaw), math.cos(self.dead_yaw))

    def control_loop(self):
        now = self.now()
        dt = max(now - self.prev_t, 1e-3)
        self.prev_t = now

        self.update_dead_reckoning(dt)

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

        alpha = clamp(float(self.get_parameter("filter_alpha").value), 0.01, 1.0)
        self.e_x_f = alpha * e_x + (1.0 - alpha) * self.e_x_f
        self.theta_f = alpha * theta + (1.0 - alpha) * self.theta_f

        raw_de_x = (self.e_x_f - self.prev_e_x_f) / dt
        raw_dtheta = (self.theta_f - self.prev_theta_f) / dt

        d_alpha = clamp(float(self.get_parameter("derivative_alpha").value), 0.01, 1.0)
        self.de_x_f = d_alpha * raw_de_x + (1.0 - d_alpha) * self.de_x_f
        self.dtheta_f = d_alpha * raw_dtheta + (1.0 - d_alpha) * self.dtheta_f

        self.prev_e_x_f = self.e_x_f
        self.prev_theta_f = self.theta_f

        v_max = float(self.get_parameter("v_max").value)
        v_min = float(self.get_parameter("v_min").value)
        v_turn_min = float(self.get_parameter("v_turn_min").value)
        k_c = float(self.get_parameter("k_c").value)

        cos_theta = max(0.0, math.cos(self.theta_f))
        v_target = (v_max * cos_theta) / (1.0 + k_c * abs(kappa_m))
        v_target = clamp(v_target, v_min, v_max)

        gamma = -2.0 * self.e_x_f / max(1e-4, L_d * L_d)
        omega_pp = float(self.get_parameter("k_pp").value) * v_target * gamma

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

        v_rate = max(0.01, float(self.get_parameter("v_rate_limit").value))
        delta_rate = max(0.01, float(self.get_parameter("delta_v_rate_limit").value))

        self.v_cmd = rate_limit(v_target, self.v_cmd, v_rate * dt)
        self.delta_v_cmd = rate_limit(delta_v_target, self.delta_v_cmd, delta_rate * dt)
        self.omega_cmd = self.delta_v_cmd / wheel_sep

        self.publish_cmd(self.v_cmd, self.omega_cmd)

        v_left_est = self.v_cmd - self.omega_cmd * wheel_sep * 0.5
        v_right_est = self.v_cmd + self.omega_cmd * wheel_sep * 0.5

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

        self.publish_debug("tracking_start_turn_pure_pursuit_pd", extra)
        self.publish_rviz()
        self.save_image_if_needed()

        row = {
            "time": now,
            "state": self.state,
            "selected_lane": self.selected_lane,
            "start_visible": self.last_error["start_visible"],
            "turn_lane_visible": self.last_error["turn_lane_visible"],
            "main_lane_visible": self.last_error["main_lane_visible"],
            "valid": self.last_error["valid"],
            "epsilon_x_mm": self.last_error["epsilon_x_mm"],
            "epsilon_y_mm": self.last_error["epsilon_y_mm"],
            "theta_rad": self.last_error["theta_rad"],
            "curvature_inv_mm": self.last_error["curvature_inv_mm"],
            "e_x_f": self.e_x_f,
            "theta_f": self.theta_f,
            "de_x_f": self.de_x_f,
            "dtheta_f": self.dtheta_f,
            "kappa_m": kappa_m,
            "L_d": L_d,
            "gamma": gamma,
            "v_target": v_target,
            "omega_pp": omega_pp,
            "omega_pd": omega_pd,
            "omega_target": omega_target,
            "v_cmd": self.v_cmd,
            "omega_cmd": self.omega_cmd,
            "delta_v_cmd": self.delta_v_cmd,
            "v_left_est": v_left_est,
            "v_right_est": v_right_est,
            "odom_x": self.odom_x if self.odom_received else self.dead_x,
            "odom_y": self.odom_y if self.odom_received else self.dead_y,
            "odom_yaw": self.odom_yaw if self.odom_received else self.dead_yaw,
        }
        self.log_csv_if_needed(row)


def main(args=None):
    rclpy.init(args=args)
    node = StartTurnPurPersuitPDFollowing()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_stop_robot()
        node.close_run_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
