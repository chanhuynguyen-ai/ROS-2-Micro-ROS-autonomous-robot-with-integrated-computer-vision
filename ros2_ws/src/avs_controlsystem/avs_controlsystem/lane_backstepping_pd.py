#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point


def clamp(x, low, high):
    return max(low, min(high, x))


class LaneBacksteppingPD(Node):
    def __init__(self):
        super().__init__('lane_backstepping_pd')

        self.declare_parameter('v_max', 0.20)
        self.declare_parameter('v_min', 0.06)
        self.declare_parameter('omega_max', 1.0)

        self.declare_parameter('k_y', 1.5)
        self.declare_parameter('k_heading', 2.0)
        self.declare_parameter('kd_heading', 0.10)
        self.declare_parameter('kd_y', 0.04)

        self.declare_parameter('lost_timeout', 0.5)
        self.declare_parameter('search_omega', 0.20)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.lane_sub = self.create_subscription(
            Point,
            '/lane_target',
            self.lane_callback,
            10
        )

        self.e_y = 0.0
        self.e_theta = 0.0
        self.valid = 0.0
        self.last_lane_time = 0.0

        self.prev_e_y = 0.0
        self.prev_e_heading = 0.0
        self.prev_t = time.time()

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('avs_controlsystem lane_backstepping_pd started')

    def lane_callback(self, msg: Point):
        self.e_y = float(msg.x)
        self.e_theta = float(msg.y)
        self.valid = float(msg.z)

        if self.valid > 0.5:
            self.last_lane_time = time.time()

    def control_loop(self):
        now = time.time()
        dt = max(now - self.prev_t, 1e-3)

        v_max = float(self.get_parameter('v_max').value)
        v_min = float(self.get_parameter('v_min').value)
        omega_max = float(self.get_parameter('omega_max').value)

        k_y = float(self.get_parameter('k_y').value)
        k_heading = float(self.get_parameter('k_heading').value)
        kd_heading = float(self.get_parameter('kd_heading').value)
        kd_y = float(self.get_parameter('kd_y').value)

        lost_timeout = float(self.get_parameter('lost_timeout').value)
        search_omega = float(self.get_parameter('search_omega').value)

        cmd = Twist()

        lane_lost = self.valid < 0.5 or (now - self.last_lane_time) > lost_timeout

        if lane_lost:
            cmd.linear.x = 0.0
            cmd.angular.z = search_omega
            self.cmd_pub.publish(cmd)
            self.prev_t = now
            return

        e_y = self.e_y
        e_theta = self.e_theta

        # Backstepping: lateral error -> desired heading correction
        theta_des = math.atan2(k_y * e_y, max(v_max, 1e-3))
        e_heading = e_theta + theta_des

        # PD damping
        d_e_heading = (e_heading - self.prev_e_heading) / dt
        d_e_y = (e_y - self.prev_e_y) / dt

        omega = (
            k_heading * e_heading
            + kd_heading * d_e_heading
            + kd_y * d_e_y
        )

        omega = clamp(omega, -omega_max, omega_max)

        # Reduce speed when heading error is large
        speed_scale = math.exp(-1.8 * abs(e_heading))
        v = clamp(v_max * speed_scale, v_min, v_max)

        if abs(omega) > 0.8 * omega_max:
            v *= 0.55

        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)

        self.cmd_pub.publish(cmd)

        self.prev_e_y = e_y
        self.prev_e_heading = e_heading
        self.prev_t = now

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LaneBacksteppingPD()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
