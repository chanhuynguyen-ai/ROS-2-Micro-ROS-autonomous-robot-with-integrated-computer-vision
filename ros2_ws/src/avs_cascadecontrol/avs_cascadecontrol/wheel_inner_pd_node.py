#!/usr/bin/env python3

import json
import math
import signal
import time

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


class WheelInnerPDNode(Node):
    """
    Vòng trong soft differential mixer.

    Input:
      /avs/lane_ref_cmd: Twist(v_ref, omega_ref)

    Output:
      /cmd_vel

    Cơ chế:
      - Cua nhẹ: hai cụm bánh cùng tiến, cụm trong chậm hơn nhưng không đứng yên.
      - Cua gắt: cụm ngoài tiến, cụm trong lùi nhẹ.
      - Không cho kiểu một cụm đứng yên, một cụm chạy.
      - /odom_raw chỉ dùng để log, không đóng vòng nếu odom chưa chuẩn.
    """

    def __init__(self):
        super().__init__("wheel_inner_pd_node")

        self.declare_parameter("lane_ref_topic", "/avs/lane_ref_cmd")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_state_topic", "/avs/wheel_pd_state")

        self.declare_parameter("runtime_enable_topic", "/avs/cascade_enable_cmd")
        self.declare_parameter("emergency_stop_topic", "/avs/cascade_emergency_stop")

        self.declare_parameter("inner_hz", 50.0)
        self.declare_parameter("ref_timeout_s", 2.0)
        self.declare_parameter("odom_timeout_s", 1.5)

        self.declare_parameter("track_width_m", 0.135)
        self.declare_parameter("wheel_radius_m", 0.0225)

        self.declare_parameter("v_max_cmd", 0.12)
        self.declare_parameter("omega_max_cmd", 0.70)

        # Giới hạn thay đổi để xe không giật.
        self.declare_parameter("wheel_speed_rate", 0.18)
        self.declare_parameter("wheel_speed_decel_rate", 0.35)

        # Cua nhẹ: hai cụm cùng tiến.
        self.declare_parameter("same_direction_inner_fraction", 0.42)
        self.declare_parameter("same_direction_min_inner_mps", 0.020)

        # Cua gắt: cụm trong lùi nhẹ.
        self.declare_parameter("enable_soft_counter_turn", True)
        self.declare_parameter("counter_start_omega", 0.32)
        self.declare_parameter("counter_full_omega", 0.72)
        self.declare_parameter("counter_outer_min_mps", 0.040)
        self.declare_parameter("counter_outer_max_mps", 0.065)
        self.declare_parameter("counter_reverse_max_mps", 0.014)

        # Nếu omega quá nhỏ thì triệt để không lắc.
        self.declare_parameter("omega_deadband", 0.018)

        self.declare_parameter("enable_calibration", False)
        self.declare_parameter("linear_cmd_scale", 1.245)
        self.declare_parameter("angular_cmd_scale", 0.75)

        self.declare_parameter("enable_cmd", False)

        self.declare_parameter("enable_lidar_safety", True)
        self.declare_parameter("emergency_distance", 0.18)
        self.declare_parameter("stop_distance", 0.32)
        self.declare_parameter("slow_distance", 0.70)
        self.declare_parameter("front_angle_deg", 35.0)

        self.declare_parameter("always_publish_stop_on_exit", True)
        self.declare_parameter("stop_burst_count", 40)
        self.declare_parameter("stop_burst_dt", 0.03)

        self.lane_ref_topic = str(self.get_parameter("lane_ref_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.wheel_state_topic = str(self.get_parameter("wheel_state_topic").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, self.wheel_state_topic, 10)

        self.create_subscription(Twist, self.lane_ref_topic, self.ref_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, str(self.get_parameter("runtime_enable_topic").value), self.runtime_enable_callback, 10)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), self.emergency_callback, 10)

        self.runtime_enable = False
        self.emergency_stop = False

        self.v_ref = 0.0
        self.omega_ref = 0.0
        self.last_ref_time = -1.0

        self.v_odom = 0.0
        self.omega_odom = 0.0
        self.last_odom_time = -1.0

        self.front_min = float("inf")
        self.last_scan_time = -1.0

        self.prev_v_left_cmd = 0.0
        self.prev_v_right_cmd = 0.0
        self.prev_timer_time = time.time()

        hz = max(1.0, float(self.get_parameter("inner_hz").value))
        self.create_timer(1.0 / hz, self.control_loop)

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.get_logger().info("wheel_inner_pd_node soft differential mixer started")
        self.get_logger().info(f"Subscribe ref: {self.lane_ref_topic}")
        self.get_logger().info(f"Publish cmd:   {self.cmd_vel_topic}")

    def now_s(self):
        return time.time()

    def is_enabled(self):
        return bool(self.get_parameter("enable_cmd").value) or self.runtime_enable

    def get_track_width(self):
        return max(0.05, float(self.get_parameter("track_width_m").value))

    def signal_handler(self, signum, frame):
        self.get_logger().warn("Signal received. Force publishing stop.")
        self.publish_stop_burst()
        if rclpy.ok():
            rclpy.shutdown()

    def runtime_enable_callback(self, msg):
        self.runtime_enable = bool(msg.data)
        self.get_logger().warn(f"Runtime enable set to: {self.runtime_enable}")
        if not self.runtime_enable and not bool(self.get_parameter("enable_cmd").value):
            self.publish_stop_burst()

    def emergency_callback(self, msg):
        self.emergency_stop = bool(msg.data)
        self.get_logger().error(f"Emergency stop = {self.emergency_stop}")
        if self.emergency_stop:
            self.publish_stop_burst()

    def ref_callback(self, msg):
        self.v_ref = float(msg.linear.x)
        self.omega_ref = float(msg.angular.z)
        self.last_ref_time = self.now_s()

    def odom_callback(self, msg):
        self.v_odom = float(msg.twist.twist.linear.x)
        self.omega_odom = float(msg.twist.twist.angular.z)
        self.last_odom_time = self.now_s()

    def scan_callback(self, msg):
        front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))

        vals = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                if abs(angle) <= front_angle:
                    vals.append(float(r))
            angle += msg.angle_increment

        self.front_min = min(vals) if vals else float("inf")
        self.last_scan_time = self.now_s()

    def make_cmd(self, v, omega):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        return msg

    def publish_cmd(self, v, omega):
        self.cmd_pub.publish(self.make_cmd(v, omega))

    def publish_stop_burst(self):
        stop = self.make_cmd(0.0, 0.0)
        count = int(self.get_parameter("stop_burst_count").value)
        dt = float(self.get_parameter("stop_burst_dt").value)

        for _ in range(max(5, count)):
            self.cmd_pub.publish(stop)
            time.sleep(max(0.005, dt))

        self.prev_v_left_cmd = 0.0
        self.prev_v_right_cmd = 0.0

    def publish_state(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

    def lidar_scale(self):
        if not bool(self.get_parameter("enable_lidar_safety").value):
            return 1.0, False, "lidar_disabled"

        if not math.isfinite(self.front_min):
            return 1.0, False, "lidar_no_data"

        emergency = float(self.get_parameter("emergency_distance").value)
        stop = float(self.get_parameter("stop_distance").value)
        slow = float(self.get_parameter("slow_distance").value)

        if self.front_min < emergency:
            return 0.0, True, "lidar_emergency"

        if self.front_min < stop:
            return 0.0, True, "lidar_stop"

        if self.front_min < slow:
            ratio = clamp((self.front_min - stop) / max(1e-6, slow - stop), 0.20, 1.0)
            return ratio, False, "lidar_slow"

        return 1.0, False, "lidar_clear"

    def mix_wheel_speeds(self, v_ref, omega_ref):
        """
        Trả về v_left_des, v_right_des, mix_mode.

        Quy ước:
          omega > 0: quay trái
            left là cụm trong
            right là cụm ngoài

          omega < 0: quay phải
            right là cụm trong
            left là cụm ngoài
        """

        B = self.get_track_width()

        v_ref = clamp(
            v_ref,
            -abs(float(self.get_parameter("v_max_cmd").value)),
            abs(float(self.get_parameter("v_max_cmd").value)),
        )

        omega_ref = clamp(
            omega_ref,
            -abs(float(self.get_parameter("omega_max_cmd").value)),
            abs(float(self.get_parameter("omega_max_cmd").value)),
        )

        if abs(omega_ref) < float(self.get_parameter("omega_deadband").value):
            omega_ref = 0.0

        if v_ref <= 0.0:
            return 0.0, 0.0, "zero_or_reverse_blocked", omega_ref

        abs_w = abs(omega_ref)
        sign_w = 1.0 if omega_ref > 0.0 else -1.0

        counter_enable = bool(self.get_parameter("enable_soft_counter_turn").value)
        counter_start = abs(float(self.get_parameter("counter_start_omega").value))
        counter_full = abs(float(self.get_parameter("counter_full_omega").value))
        counter_full = max(counter_full, counter_start + 1e-3)

        if counter_enable and abs_w >= counter_start:
            strength = clamp((abs_w - counter_start) / (counter_full - counter_start), 0.0, 1.0)

            outer_min = float(self.get_parameter("counter_outer_min_mps").value)
            outer_max = float(self.get_parameter("counter_outer_max_mps").value)
            reverse_max = abs(float(self.get_parameter("counter_reverse_max_mps").value))

            # Cua càng gắt thì xe giảm tiến, chỉ tạo xoay mềm.
            outer_speed = clamp(v_ref, outer_min, outer_max)
            inner_reverse = reverse_max * strength

            if sign_w > 0.0:
                # Quay trái: trái lùi nhẹ, phải tiến.
                v_left = -inner_reverse
                v_right = outer_speed
            else:
                # Quay phải: trái tiến, phải lùi nhẹ.
                v_left = outer_speed
                v_right = -inner_reverse

            return v_left, v_right, "soft_counter_turn", omega_ref

        # Cua nhẹ: tính theo vi sai chuẩn rồi ép bánh trong vẫn tiến.
        v_left = v_ref - omega_ref * B * 0.5
        v_right = v_ref + omega_ref * B * 0.5

        inner_frac = clamp(float(self.get_parameter("same_direction_inner_fraction").value), 0.0, 0.95)
        inner_min_abs = abs(float(self.get_parameter("same_direction_min_inner_mps").value))

        if omega_ref > 0.0:
            # Quay trái: left inner, right outer.
            outer = max(v_right, inner_min_abs)
            inner_min = max(inner_min_abs, outer * inner_frac)
            v_left = max(v_left, inner_min)
            v_right = outer
        elif omega_ref < 0.0:
            # Quay phải: right inner, left outer.
            outer = max(v_left, inner_min_abs)
            inner_min = max(inner_min_abs, outer * inner_frac)
            v_right = max(v_right, inner_min)
            v_left = outer
        else:
            v_left = v_ref
            v_right = v_ref

        return v_left, v_right, "same_direction_turn", omega_ref

    def control_loop(self):
        now = self.now_s()
        dt = max(now - self.prev_timer_time, 1e-3)
        self.prev_timer_time = now

        enabled = self.is_enabled()
        B = self.get_track_width()

        ref_timeout_s = float(self.get_parameter("ref_timeout_s").value)
        ref_timeout = self.last_ref_time < 0.0 or (now - self.last_ref_time) > ref_timeout_s

        if ref_timeout:
            v_left_des = 0.0
            v_right_des = 0.0
            mix_mode = "ref_timeout"
            omega_ref_limited = 0.0
        else:
            v_left_des, v_right_des, mix_mode, omega_ref_limited = self.mix_wheel_speeds(
                self.v_ref,
                self.omega_ref,
            )

        lidar_ratio, lidar_stop, lidar_mode = self.lidar_scale()

        if self.emergency_stop:
            lidar_ratio = 0.0
            lidar_stop = True
            lidar_mode = "emergency_stop_topic"

        v_left_des *= lidar_ratio
        v_right_des *= lidar_ratio

        # Rate-limit riêng từng cụm bánh để chuyển cua mượt.
        if abs(v_left_des) >= abs(self.prev_v_left_cmd):
            left_rate = abs(float(self.get_parameter("wheel_speed_rate").value))
        else:
            left_rate = abs(float(self.get_parameter("wheel_speed_decel_rate").value))

        if abs(v_right_des) >= abs(self.prev_v_right_cmd):
            right_rate = abs(float(self.get_parameter("wheel_speed_rate").value))
        else:
            right_rate = abs(float(self.get_parameter("wheel_speed_decel_rate").value))

        v_left_cmd = approach(self.prev_v_left_cmd, v_left_des, left_rate * dt)
        v_right_cmd = approach(self.prev_v_right_cmd, v_right_des, right_rate * dt)

        if ref_timeout or self.emergency_stop:
            v_left_cmd = 0.0
            v_right_cmd = 0.0

        self.prev_v_left_cmd = v_left_cmd
        self.prev_v_right_cmd = v_right_cmd

        v_cmd = 0.5 * (v_left_cmd + v_right_cmd)
        omega_cmd = (v_right_cmd - v_left_cmd) / B

        if bool(self.get_parameter("enable_calibration").value):
            linear_scale = max(1e-6, float(self.get_parameter("linear_cmd_scale").value))
            angular_scale = max(1e-6, float(self.get_parameter("angular_cmd_scale").value))
            v_cmd = v_cmd / linear_scale
            omega_cmd = omega_cmd / angular_scale

        odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        odom_timeout = self.last_odom_time < 0.0 or (now - self.last_odom_time) > odom_timeout_s

        v_left_odom = self.v_odom - self.omega_odom * B * 0.5
        v_right_odom = self.v_odom + self.omega_odom * B * 0.5

        cmd_published = False
        if enabled:
            self.publish_cmd(v_cmd, omega_cmd)
            cmd_published = True

        self.publish_state({
            "enabled": enabled,
            "runtime_enable": self.runtime_enable,
            "param_enable_cmd": bool(self.get_parameter("enable_cmd").value),
            "cmd_published": cmd_published,

            "mode": "soft_differential_mixer",
            "mix_mode": mix_mode,
            "feedback_mode": "odom_monitor_only",

            "v_ref": self.v_ref,
            "omega_ref": self.omega_ref,
            "omega_ref_limited": omega_ref_limited,

            "v_left_des": v_left_des,
            "v_right_des": v_right_des,
            "v_left_cmd": v_left_cmd,
            "v_right_cmd": v_right_cmd,

            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,

            "v_left_odom": v_left_odom,
            "v_right_odom": v_right_odom,
            "odom_v": self.v_odom,
            "odom_omega": self.omega_odom,
            "odom_timeout": odom_timeout,
            "ref_timeout": ref_timeout,

            "lidar_ratio": lidar_ratio,
            "lidar_stop": lidar_stop,
            "lidar_mode": lidar_mode,
            "front_min_m": self.front_min if math.isfinite(self.front_min) else None,

            "track_width_m": B,
            "emergency_stop": self.emergency_stop,
        })


def main(args=None):
    rclpy.init(args=args)
    node = WheelInnerPDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if bool(node.get_parameter("always_publish_stop_on_exit").value):
            node.get_logger().warn("Shutdown: force stop burst to /cmd_vel")
            node.publish_stop_burst()

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
