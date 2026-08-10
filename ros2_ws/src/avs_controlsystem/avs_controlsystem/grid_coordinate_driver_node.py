#!/usr/bin/env python3

import csv
import math
import os
import re
import signal
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_deg(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GridCoordinateDriverNode(Node):
    def __init__(self):
        super().__init__('grid_coordinate_driver_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('report_dir', '/root/AVScontrol/ros2_ws/demo_reports/grid_coordinate_driver')
        self.declare_parameter('publish_hz', 20.0)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.report_dir = str(self.get_parameter('report_dir').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        os.makedirs(self.report_dir, exist_ok=True)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

        self.odom_ok = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.odom_start_x = 0.0
        self.odom_start_y = 0.0
        self.odom_start_yaw = 0.0

        self.odom_end_x = 0.0
        self.odom_end_y = 0.0
        self.odom_end_yaw = 0.0

        self.grid_x = 0.0
        self.grid_y = 0.0

        # Quy ước: 0 deg = hướng +Y ban đầu, -90 deg = +X bên phải, +90 deg = -X bên trái.
        self.heading_est_deg = 0.0

        self.rows = []
        self.segment_id = 0
        self.shutting_down = False

        signal.signal(signal.SIGINT, self.signal_stop_handler)
        signal.signal(signal.SIGTERM, self.signal_stop_handler)

        self.get_logger().info('Grid coordinate driver started')
        self.get_logger().info(f'Publish cmd_vel: {self.cmd_vel_topic}')
        self.get_logger().info(f'Subscribe odom:  {self.odom_topic}')
        self.get_logger().info(f'Report dir:      {self.report_dir}')

    def odom_callback(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ok = True

    def signal_stop_handler(self, signum, frame):
        self.get_logger().warn(f'Received signal {signum}. Stopping robot...')
        self.shutting_down = True
        self.safe_stop_robot()
        if rclpy.ok():
            rclpy.shutdown()

    def make_cmd(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        return cmd

    def publish_cmd(self, linear, angular):
        self.cmd_pub.publish(self.make_cmd(linear, angular))

    def safe_stop_robot(self):
        for _ in range(35):
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.02)

    def capture_odom_start(self):
        self.odom_start_x = self.odom_x
        self.odom_start_y = self.odom_y
        self.odom_start_yaw = self.odom_yaw

    def capture_odom_end(self):
        self.odom_end_x = self.odom_x
        self.odom_end_y = self.odom_y
        self.odom_end_yaw = self.odom_yaw

    def get_odom_delta(self):
        dx = self.odom_end_x - self.odom_start_x
        dy = self.odom_end_y - self.odom_start_y

        dist_cm = math.hypot(dx, dy) * 100.0

        dyaw = math.atan2(
            math.sin(self.odom_end_yaw - self.odom_start_yaw),
            math.cos(self.odom_end_yaw - self.odom_start_yaw)
        )
        angle_deg = math.degrees(dyaw)

        return dist_cm, angle_deg

    def run_cmd_for_duration(self, linear, angular, duration_s):
        duration_s = max(0.0, float(duration_s))

        self.capture_odom_start()

        start_time = time.time()
        dt = 1.0 / max(self.publish_hz, 1.0)

        while rclpy.ok() and not self.shutting_down:
            now = time.time()
            elapsed = now - start_time

            if elapsed >= duration_s:
                break

            self.publish_cmd(linear, angular)
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(dt)

        self.safe_stop_robot()
        time.sleep(0.15)

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.01)

        self.capture_odom_end()

    def add_report_row(
        self,
        segment_type,
        direction,
        from_grid_x,
        from_grid_y,
        to_grid_x,
        to_grid_y,
        desired_heading_deg,
        cmd_linear,
        cmd_angular,
        duration_s,
        desired_distance_cm,
        desired_angle_deg,
        linear_scale,
        angular_scale
    ):
        odom_distance_cm, odom_angle_deg = self.get_odom_delta()

        self.segment_id += 1

        row = {
            'timestamp': datetime.now().strftime('%Y-%m-%d_%H-%M-%S'),
            'segment_id': self.segment_id,
            'segment_type': segment_type,
            'direction': direction,
            'from_grid_x': from_grid_x,
            'from_grid_y': from_grid_y,
            'to_grid_x': to_grid_x,
            'to_grid_y': to_grid_y,
            'desired_heading_deg': desired_heading_deg,
            'cmd_linear_mps': cmd_linear,
            'cmd_angular_radps': cmd_angular,
            'duration_s': duration_s,
            'desired_distance_cm': desired_distance_cm,
            'desired_angle_deg': desired_angle_deg,
            'odom_distance_cm': odom_distance_cm,
            'odom_angle_deg': odom_angle_deg,
            'linear_scale': linear_scale,
            'angular_scale': angular_scale,
            'heading_est_deg_after': self.heading_est_deg,
            'odom_ok': int(self.odom_ok),
        }

        self.rows.append(row)

        print(
            f"[SEG {self.segment_id}] {segment_type:8s} {direction:8s} | "
            f"cmd_v={cmd_linear:.3f}, cmd_w={cmd_angular:.3f}, t={duration_s:.3f}s | "
            f"target_dist={desired_distance_cm:.1f}cm, target_angle={desired_angle_deg:.1f}deg | "
            f"odom_dist={odom_distance_cm:.1f}cm, odom_angle={odom_angle_deg:.1f}deg"
        )

    def rotate_to_heading(self, desired_heading_deg, angular_speed, angular_scale, settle_time):
        desired_heading_deg = wrap_deg(desired_heading_deg)
        delta_deg = wrap_deg(desired_heading_deg - self.heading_est_deg)

        if abs(delta_deg) < 1.0:
            return

        sign = 1.0 if delta_deg > 0.0 else -1.0
        cmd_angular = sign * abs(angular_speed)

        desired_angle_rad = math.radians(abs(delta_deg))
        actual_angular_speed = abs(angular_speed) * max(angular_scale, 1e-6)
        duration_s = desired_angle_rad / actual_angular_speed

        from_x = self.grid_x
        from_y = self.grid_y

        print(
            f"\n[ROTATE] heading {self.heading_est_deg:.1f} -> {desired_heading_deg:.1f} deg, "
            f"delta={delta_deg:.1f} deg, omega={cmd_angular:.3f}, time={duration_s:.3f}s"
        )

        self.run_cmd_for_duration(0.0, cmd_angular, duration_s)

        self.heading_est_deg = desired_heading_deg

        self.add_report_row(
            segment_type='rotate',
            direction='left' if sign > 0 else 'right',
            from_grid_x=from_x,
            from_grid_y=from_y,
            to_grid_x=self.grid_x,
            to_grid_y=self.grid_y,
            desired_heading_deg=desired_heading_deg,
            cmd_linear=0.0,
            cmd_angular=cmd_angular,
            duration_s=duration_s,
            desired_distance_cm=0.0,
            desired_angle_deg=abs(delta_deg),
            linear_scale=0.0,
            angular_scale=angular_scale
        )

        time.sleep(settle_time)

    def move_forward_distance(self, distance_cm, linear_speed, linear_scale, settle_time):
        distance_cm = abs(float(distance_cm))

        if distance_cm < 0.1:
            return

        desired_distance_m = distance_cm / 100.0
        actual_linear_speed = abs(linear_speed) * max(linear_scale, 1e-6)
        duration_s = desired_distance_m / actual_linear_speed

        from_x = self.grid_x
        from_y = self.grid_y

        print(
            f"\n[MOVE] distance={distance_cm:.1f} cm, v={linear_speed:.3f}, "
            f"linear_scale={linear_scale:.4f}, time={duration_s:.3f}s"
        )

        self.run_cmd_for_duration(abs(linear_speed), 0.0, duration_s)

        self.add_report_row(
            segment_type='move',
            direction='forward',
            from_grid_x=from_x,
            from_grid_y=from_y,
            to_grid_x=self.grid_x,
            to_grid_y=self.grid_y,
            desired_heading_deg=self.heading_est_deg,
            cmd_linear=abs(linear_speed),
            cmd_angular=0.0,
            duration_s=duration_s,
            desired_distance_cm=distance_cm,
            desired_angle_deg=0.0,
            linear_scale=linear_scale,
            angular_scale=0.0
        )

        time.sleep(settle_time)

    def drive_to_grid_target(
        self,
        target_grid_x,
        target_grid_y,
        cell_cm,
        axis_order,
        linear_speed,
        angular_speed,
        linear_scale,
        angular_scale,
        settle_time
    ):
        target_grid_x = float(target_grid_x)
        target_grid_y = float(target_grid_y)

        print(f"\n========== TARGET ({target_grid_x:g}, {target_grid_y:g}) ==========")

        def run_x_axis():
            dx = target_grid_x - self.grid_x

            if abs(dx) < 1e-6:
                return

            desired_heading = -90.0 if dx > 0.0 else 90.0
            distance_cm = abs(dx) * cell_cm

            self.rotate_to_heading(desired_heading, angular_speed, angular_scale, settle_time)
            self.move_forward_distance(distance_cm, linear_speed, linear_scale, settle_time)

            self.grid_x = target_grid_x

        def run_y_axis():
            dy = target_grid_y - self.grid_y

            if abs(dy) < 1e-6:
                return

            desired_heading = 0.0 if dy > 0.0 else 180.0
            distance_cm = abs(dy) * cell_cm

            self.rotate_to_heading(desired_heading, angular_speed, angular_scale, settle_time)
            self.move_forward_distance(distance_cm, linear_speed, linear_scale, settle_time)

            self.grid_y = target_grid_y

        if axis_order.lower() == 'yx':
            run_y_axis()
            run_x_axis()
        else:
            run_x_axis()
            run_y_axis()

        print(
            f"[DONE TARGET] current grid=({self.grid_x:g}, {self.grid_y:g}), "
            f"heading={self.heading_est_deg:.1f} deg"
        )

    def save_report(self):
        if not self.rows:
            print('[REPORT] No data to save.')
            return None

        run_name = datetime.now().strftime('grid_run_%Y-%m-%d_%H-%M-%S')
        run_dir = os.path.join(self.report_dir, run_name)
        os.makedirs(run_dir, exist_ok=True)

        csv_path = os.path.join(run_dir, 'grid_coordinate_run.csv')

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

        summary_path = os.path.join(run_dir, 'summary.txt')
        with open(summary_path, 'w') as f:
            f.write('Grid Coordinate Driver Summary\n')
            f.write('==============================\n')
            f.write(f'Final grid position: ({self.grid_x}, {self.grid_y})\n')
            f.write(f'Final estimated heading deg: {self.heading_est_deg}\n')
            f.write(f'Segment count: {len(self.rows)}\n')
            f.write(f'CSV: {csv_path}\n')

        print(f'\n[REPORT] Saved CSV: {csv_path}')
        print(f'[REPORT] Saved summary: {summary_path}')

        return run_dir

    @staticmethod
    def ask_float(prompt, default, low=None, high=None):
        raw = input(f'{prompt} [{default}]: ').strip()

        if raw == '':
            value = float(default)
        else:
            value = float(raw)

        if low is not None:
            value = max(low, value)
        if high is not None:
            value = min(high, value)

        return value

    @staticmethod
    def ask_text(prompt, default):
        raw = input(f'{prompt} [{default}]: ').strip()
        return raw if raw else str(default)

    @staticmethod
    def parse_targets(text):
        text = text.strip()
        text = text.replace('，', ',')
        chunks = re.split(r';+', text)

        targets = []

        for chunk in chunks:
            chunk = chunk.strip()
            if not chunk:
                continue

            chunk = chunk.replace('(', '').replace(')', '')
            parts = [p.strip() for p in chunk.split(',')]

            if len(parts) != 2:
                raise ValueError(f'Invalid target: {chunk}. Use format: 1,1; 2,1')

            targets.append((float(parts[0]), float(parts[1])))

        if not targets:
            raise ValueError('No target parsed.')

        return targets

    def run_interactive(self):
        print('\n========== GRID COORDINATE DRIVER ==========')
        print('Quy ước:')
        print('  Gốc tọa độ: vị trí xe ban đầu')
        print('  +Y: hướng đầu xe ban đầu')
        print('  +X: bên phải xe')
        print('  Ví dụ step=30cm, target=1,1 => đi phải 30cm, sau đó đi tiến 30cm')
        print('============================================\n')

        cell_cm = self.ask_float('Khoảng cách 1 đơn vị tọa độ, cm', 30.0, low=1.0, high=500.0)
        linear_speed = self.ask_float('Tốc độ chạy thẳng linear.x, m/s', 0.4, low=0.01, high=1.0)
        angular_speed = self.ask_float('Tốc độ quay angular.z, rad/s', 2.5, low=0.05, high=5.0)

        linear_scale = self.ask_float('Linear scale thực tế/lý thuyết', 1.245, low=0.05, high=5.0)
        angular_scale = self.ask_float('Angular scale thực tế/lý thuyết', 0.739198, low=0.05, high=5.0)

        axis_order = self.ask_text('Thứ tự chạy trục: xy hoặc yx', 'xy').lower()
        if axis_order not in ['xy', 'yx']:
            axis_order = 'xy'

        settle_time = self.ask_float('Thời gian nghỉ giữa các đoạn, s', 0.25, low=0.0, high=5.0)

        target_text = self.ask_text('Nhập tọa độ cần đi, ví dụ: 1,1 hoặc 1,1; 2,1; 2,2', '1,1')
        targets = self.parse_targets(target_text)

        print('\n========== RUN CONFIG ==========')
        print(f'cell_cm       = {cell_cm}')
        print(f'linear_speed  = {linear_speed}')
        print(f'angular_speed = {angular_speed}')
        print(f'linear_scale  = {linear_scale}')
        print(f'angular_scale = {angular_scale}')
        print(f'axis_order    = {axis_order}')
        print(f'targets       = {targets}')
        print('===============================\n')

        confirm = input('Bắt đầu chạy? Nhập y để chạy: ').strip().lower()
        if confirm != 'y':
            print('Canceled.')
            self.safe_stop_robot()
            return

        print('\nBắt đầu sau 3 giây...')
        for i in [3, 2, 1]:
            print(i)
            time.sleep(1.0)

        for tx, ty in targets:
            if self.shutting_down or not rclpy.ok():
                break

            self.drive_to_grid_target(
                target_grid_x=tx,
                target_grid_y=ty,
                cell_cm=cell_cm,
                axis_order=axis_order,
                linear_speed=linear_speed,
                angular_speed=angular_speed,
                linear_scale=linear_scale,
                angular_scale=angular_scale,
                settle_time=settle_time
            )

        self.safe_stop_robot()
        self.save_report()


def main(args=None):
    rclpy.init(args=args)
    node = GridCoordinateDriverNode()

    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.safe_stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
