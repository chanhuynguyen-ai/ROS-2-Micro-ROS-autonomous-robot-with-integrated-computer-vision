#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32


def yaw_from_quat(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CascadeControlMonitorNode(Node):
    def __init__(self):
        super().__init__("cascade_control_monitor_node")

        self.declare_parameter("control_error_topic", "/avs/control_error")
        self.declare_parameter("lane_ref_topic", "/avs/lane_ref_cmd")
        self.declare_parameter("lane_state_topic", "/avs/lane_pd_state")
        self.declare_parameter("wheel_state_topic", "/avs/wheel_pd_state")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cascade_state_topic", "/avs/cascade_control_state")
        self.declare_parameter("monitor_hz", 10.0)
        self.declare_parameter("front_angle_deg", 35.0)

        self.latest_control_error = {}
        self.latest_lane_ref = {}
        self.latest_lane_state = {}
        self.latest_wheel_state = {}
        self.latest_cmd_vel = {}
        self.latest_odom = {}
        self.latest_scan = {}

        self.state_pub = self.create_publisher(
            String,
            str(self.get_parameter("cascade_state_topic").value),
            10,
        )

        self.scalar_pubs = {}
        for name in [
            "epsilon_x_mm",
            "e_lat_m",
            "theta_rad",
            "lookahead_m",
            "curvature",
            "v_ref",
            "omega_ref",
            "v_cmd",
            "omega_cmd",
            "cmd_v",
            "cmd_omega",
            "odom_v",
            "odom_omega",
            "v_left_cmd",
            "v_right_cmd",
            "v_left_odom",
            "v_right_odom",
            "front_min_m",
        ]:
            self.scalar_pubs[name] = self.create_publisher(Float32, f"/avs/cascade/{name}", 10)

        self.create_subscription(String, str(self.get_parameter("control_error_topic").value), self.control_error_cb, 10)
        self.create_subscription(Twist, str(self.get_parameter("lane_ref_topic").value), self.lane_ref_cb, 10)
        self.create_subscription(String, str(self.get_parameter("lane_state_topic").value), self.lane_state_cb, 10)
        self.create_subscription(String, str(self.get_parameter("wheel_state_topic").value), self.wheel_state_cb, 10)
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self.scan_cb, qos_profile_sensor_data)

        hz = max(1.0, float(self.get_parameter("monitor_hz").value))
        self.create_timer(1.0 / hz, self.publish_state)

        self.get_logger().info("cascade_control_monitor_node started")

    def now_s(self):
        return time.time()

    def parse_json(self, msg):
        try:
            return json.loads(msg.data)
        except Exception:
            return {"raw": msg.data}

    def publish_float(self, name, value):
        if name not in self.scalar_pubs:
            return
        try:
            v = float(value)
            if not math.isfinite(v):
                return
        except Exception:
            return
        msg = Float32()
        msg.data = v
        self.scalar_pubs[name].publish(msg)

    def control_error_cb(self, msg):
        d = self.parse_json(msg)
        d["_rx_time"] = self.now_s()
        self.latest_control_error = d

    def lane_ref_cb(self, msg):
        self.latest_lane_ref = {
            "v_ref": float(msg.linear.x),
            "omega_ref": float(msg.angular.z),
            "_rx_time": self.now_s(),
        }

    def lane_state_cb(self, msg):
        d = self.parse_json(msg)
        d["_rx_time"] = self.now_s()
        self.latest_lane_state = d

    def wheel_state_cb(self, msg):
        d = self.parse_json(msg)
        d["_rx_time"] = self.now_s()
        self.latest_wheel_state = d

    def cmd_vel_cb(self, msg):
        self.latest_cmd_vel = {
            "cmd_v": float(msg.linear.x),
            "cmd_omega": float(msg.angular.z),
            "_rx_time": self.now_s(),
        }

    def odom_cb(self, msg):
        self.latest_odom = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw": yaw_from_quat(msg.pose.pose.orientation),
            "odom_v": float(msg.twist.twist.linear.x),
            "odom_omega": float(msg.twist.twist.angular.z),
            "_rx_time": self.now_s(),
        }

    def scan_cb(self, msg):
        front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        vals = []
        angle = msg.angle_min

        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                if abs(angle) <= front_angle:
                    vals.append(float(r))
            angle += msg.angle_increment

        self.latest_scan = {
            "front_min_m": min(vals) if vals else None,
            "_rx_time": self.now_s(),
        }

    def publish_state(self):
        payload = {
            "time": self.now_s(),
            "control_error": self.latest_control_error,
            "lane_ref": self.latest_lane_ref,
            "lane_pd_state": self.latest_lane_state,
            "wheel_pd_state": self.latest_wheel_state,
            "cmd_vel": self.latest_cmd_vel,
            "odom_raw": self.latest_odom,
            "scan": self.latest_scan,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)

        self.publish_float("epsilon_x_mm", self.latest_lane_state.get("epsilon_x_mm", self.latest_control_error.get("epsilon_x_mm", 0.0)))
        self.publish_float("e_lat_m", self.latest_lane_state.get("e_lat_m", 0.0))
        self.publish_float("theta_rad", self.latest_lane_state.get("e_theta_rad", self.latest_control_error.get("theta_rad", 0.0)))
        self.publish_float("lookahead_m", self.latest_lane_state.get("lookahead_m", 0.0))
        self.publish_float("curvature", self.latest_lane_state.get("curvature", 0.0))

        self.publish_float("v_ref", self.latest_lane_ref.get("v_ref", self.latest_lane_state.get("v_ref", 0.0)))
        self.publish_float("omega_ref", self.latest_lane_ref.get("omega_ref", self.latest_lane_state.get("omega_ref", 0.0)))

        self.publish_float("v_cmd", self.latest_wheel_state.get("v_cmd", 0.0))
        self.publish_float("omega_cmd", self.latest_wheel_state.get("omega_cmd", 0.0))

        self.publish_float("cmd_v", self.latest_cmd_vel.get("cmd_v", 0.0))
        self.publish_float("cmd_omega", self.latest_cmd_vel.get("cmd_omega", 0.0))

        self.publish_float("odom_v", self.latest_odom.get("odom_v", 0.0))
        self.publish_float("odom_omega", self.latest_odom.get("odom_omega", 0.0))

        self.publish_float("v_left_cmd", self.latest_wheel_state.get("v_left_cmd", 0.0))
        self.publish_float("v_right_cmd", self.latest_wheel_state.get("v_right_cmd", 0.0))
        self.publish_float("v_left_odom", self.latest_wheel_state.get("v_left_odom", 0.0))
        self.publish_float("v_right_odom", self.latest_wheel_state.get("v_right_odom", 0.0))

        self.publish_float("front_min_m", self.latest_scan.get("front_min_m", -1.0))


def main(args=None):
    rclpy.init(args=args)
    node = CascadeControlMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
