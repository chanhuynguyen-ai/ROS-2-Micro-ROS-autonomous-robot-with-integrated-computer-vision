#!/usr/bin/env python3

import json
import math
import signal
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def clamp(value, low, high):
    return max(low, min(high, value))


def approach(current, target, max_delta):
    max_delta = abs(max_delta)

    if target > current + max_delta:
        return current + max_delta

    if target < current - max_delta:
        return current - max_delta

    return target


def finite_float(value, default=None):
    try:
        value = float(value)

        if math.isfinite(value):
            return value

    except (TypeError, ValueError):
        pass

    return default


def parse_bool(value, default=True):
    if value is None:
        return default

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() not in {
            '',
            '0',
            'false',
            'no',
            'none',
            'invalid',
            'lost',
        }

    return bool(value)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (
        q.w * q.z +
        q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y +
        q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


class CascadeControllerV2(Node):

    VERSION = 'cascade_controller_v2_true_cascade_pd_1_1'

    def __init__(self):

        super().__init__(
            'cascade_controller_v2'
        )

        # ============================================================
        # ROS TOPICS
        # ============================================================

        self.declare_parameter(
            'control_error_topic',
            '/avs/control_error'
        )

        self.declare_parameter(
            'lane_state_topic',
            '/avs/lane_state'
        )

        self.declare_parameter(
            'odom_topic',
            '/odom_raw'
        )

        self.declare_parameter(
            'scan_topic',
            '/scan'
        )

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel'
        )

        self.declare_parameter(
            'ref_topic',
            '/avs/cascade_controller_v2_ref'
        )

        self.declare_parameter(
            'state_topic',
            '/avs/cascade_controller_v2_state'
        )

        self.declare_parameter(
            'runtime_enable_topic',
            '/avs/cascade_v2_enable_cmd'
        )

        self.declare_parameter(
            'emergency_stop_topic',
            '/avs/cascade_v2_emergency_stop'
        )

        # ============================================================
        # GENERAL CONTROL
        # ============================================================

        self.declare_parameter(
            'enable_cmd',
            False
        )

        self.declare_parameter(
            'control_hz',
            50.0
        )

        self.declare_parameter(
            'error_timeout_s',
            1.20
        )

        self.declare_parameter(
            'odom_timeout_s',
            0.60
        )

        self.declare_parameter(
            'blind_hold_s',
            0.25
        )

        self.declare_parameter(
            'blind_v',
            0.030
        )

        self.declare_parameter(
            'startup_hold_s',
            0.20
        )

        self.declare_parameter(
            'startup_v',
            0.050
        )

        # ============================================================
        # ROBOT GEOMETRY
        # ============================================================

        self.declare_parameter(
            'track_width_m',
            0.135
        )

        self.declare_parameter(
            'wheel_radius_m',
            0.0225
        )

        # ============================================================
        # CONTROL ERROR CONVENTION
        #
        # Xe hiện tại:
        #
        # epsilon_x > 0:
        #     mục tiêu nằm bên phải
        #
        # ROS angular.z < 0:
        #     xe quay phải
        #
        # Vì vậy outer_control_sign = -1.
        # ============================================================

        self.declare_parameter(
            'epsilon_sign',
            1.0
        )

        self.declare_parameter(
            'theta_sign',
            1.0
        )

        self.declare_parameter(
            'outer_control_sign',
            -1.0
        )

        self.declare_parameter(
            'invert_angular',
            False
        )

        self.declare_parameter(
            'x_bias_m',
            0.0
        )

        # ============================================================
        # INPUT VALIDATION
        # ============================================================

        self.declare_parameter(
            'max_abs_x_m',
            0.50
        )

        self.declare_parameter(
            'max_abs_theta_rad',
            1.30
        )

        self.declare_parameter(
            'min_confidence',
            0.15
        )

        # ============================================================
        # CONTROL ERROR FILTER
        # ============================================================

        self.declare_parameter(
            'median_window',
            5
        )

        self.declare_parameter(
            'error_filter_tau_s',
            0.30
        )

        self.declare_parameter(
            'derivative_filter_tau_s',
            0.55
        )

        self.declare_parameter(
            'x_deadband_m',
            0.012
        )

        self.declare_parameter(
            'theta_deadband_rad',
            0.020
        )

        self.declare_parameter(
            'derivative_clip_lat_mps',
            0.30
        )

        self.declare_parameter(
            'derivative_clip_theta_rps',
            1.20
        )

        # ============================================================
        # OUTER PD CONTROLLER
        #
        # omega_ref =
        # sign * (
        #     kp_lat * e_lat
        #   + kd_lat * de_lat
        #   + kp_heading * e_heading
        #   + kd_heading * de_heading
        # )
        #
        # Không có Pure Pursuit.
        # Không có -2e/Ld^2.
        # ============================================================

        self.declare_parameter(
            'outer_kp_lat',
            0.55
        )

        self.declare_parameter(
            'outer_kd_lat',
            0.10
        )

        self.declare_parameter(
            'outer_kp_heading',
            0.45
        )

        self.declare_parameter(
            'outer_kd_heading',
            0.08
        )

        self.declare_parameter(
            'omega_ref_max',
            0.38
        )

        self.declare_parameter(
            'omega_ref_deadband',
            0.008
        )

        self.declare_parameter(
            'omega_ref_rate_up',
            0.60
        )

        self.declare_parameter(
            'omega_ref_rate_down',
            0.90
        )

        self.declare_parameter(
            'omega_zero_cross_rate',
            0.34
        )

        # ============================================================
        # LINEAR VELOCITY REFERENCE
        #
        # Curvature chỉ dùng giảm tốc.
        # Không dùng curvature để tính omega_ref.
        # ============================================================

        self.declare_parameter(
            'v_max',
            0.145
        )

        self.declare_parameter(
            'v_min',
            0.048
        )

        self.declare_parameter(
            'v_curve_min',
            0.065
        )

        self.declare_parameter(
            'v_recovery_max',
            0.078
        )

        self.declare_parameter(
            'v_lane_change_max',
            0.082
        )

        self.declare_parameter(
            'speed_slow_lat',
            1.20
        )

        self.declare_parameter(
            'speed_slow_heading',
            0.95
        )

        self.declare_parameter(
            'speed_slow_curvature',
            0.070
        )

        self.declare_parameter(
            'v_ref_rate_up',
            0.18
        )

        self.declare_parameter(
            'v_ref_rate_down',
            0.40
        )

        # ============================================================
        # PLANNER PROFILE
        #
        # Chỉ nhân gain hoặc giới hạn tốc độ.
        # Không đổi công thức điều khiển.
        # ============================================================

        self.declare_parameter(
            'recovery_gain_multiplier',
            1.20
        )

        self.declare_parameter(
            'lane_change_gain_multiplier',
            1.08
        )

        self.declare_parameter(
            'soft_replan_gain_multiplier',
            0.80
        )

        self.declare_parameter(
            'hold_on_planner_hold',
            False
        )

        # ============================================================
        # INNER WHEEL SPEED PD
        #
        # v_wheel_cmd =
        #     v_wheel_ref
        #   + kp * wheel_error
        #   + kd * d_wheel_error
        #
        # v_wheel_ref là feed-forward.
        # PD chỉ bù sai số tốc độ bánh.
        # ============================================================

        self.declare_parameter(
            'inner_kp_left',
            0.25
        )

        self.declare_parameter(
            'inner_kd_left',
            0.02
        )

        self.declare_parameter(
            'inner_kp_right',
            0.25
        )

        self.declare_parameter(
            'inner_kd_right',
            0.02
        )

        self.declare_parameter(
            'inner_derivative_tau_s',
            0.18
        )

        self.declare_parameter(
            'inner_error_deadband_mps',
            0.004
        )

        self.declare_parameter(
            'inner_correction_max_mps',
            0.052
        )

        self.declare_parameter(
            'wheel_ref_max_mps',
            0.175
        )

        self.declare_parameter(
            'wheel_cmd_max_mps',
            0.185
        )

        self.declare_parameter(
            'wheel_cmd_rate_up_mps2',
            0.32
        )

        self.declare_parameter(
            'wheel_cmd_rate_down_mps2',
            0.58
        )

        self.declare_parameter(
            'allow_reverse_wheel',
            False
        )

        self.declare_parameter(
            'minimum_forward_wheel_mps',
            0.018
        )

        # ============================================================
        # OUTPUT CALIBRATION
        # ============================================================

        self.declare_parameter(
            'enable_calibration',
            True
        )

        self.declare_parameter(
            'linear_cmd_scale',
            1.245
        )

        # Đây là phép NHÂN:
        #
        # < 1 giảm quay
        # > 1 tăng quay
        self.declare_parameter(
            'angular_cmd_gain',
            0.92
        )

        self.declare_parameter(
            'linear_cmd_max',
            0.145
        )

        self.declare_parameter(
            'angular_cmd_max',
            0.42
        )

        # ============================================================
        # OPTIONAL LIDAR STOP
        #
        # V2 không chứa tránh vật cản.
        # Chỉ có emergency stop tùy chọn.
        # ============================================================

        self.declare_parameter(
            'enable_lidar_safety',
            False
        )

        self.declare_parameter(
            'front_angle_deg',
            18.0
        )

        self.declare_parameter(
            'lidar_emergency_distance_m',
            0.16
        )

        self.declare_parameter(
            'lidar_stop_distance_m',
            0.24
        )

        self.declare_parameter(
            'lidar_slow_distance_m',
            0.55
        )

        # ============================================================
        # CMD_VEL SAFETY
        # ============================================================

        self.declare_parameter(
            'check_cmd_vel_conflict',
            True
        )

        self.declare_parameter(
            'allow_cmd_vel_conflict',
            False
        )

        self.declare_parameter(
            'publish_zero_on_conflict',
            True
        )

        self.declare_parameter(
            'stop_burst_count',
            30
        )

        self.declare_parameter(
            'stop_burst_dt',
            0.020
        )

        self.declare_parameter(
            'always_publish_stop_on_exit',
            True
        )

        # ============================================================
        # READ TOPIC PARAMETERS
        # ============================================================

        self.control_error_topic = self.pstr(
            'control_error_topic'
        )

        self.lane_state_topic = self.pstr(
            'lane_state_topic'
        )

        self.odom_topic = self.pstr(
            'odom_topic'
        )

        self.scan_topic = self.pstr(
            'scan_topic'
        )

        self.cmd_vel_topic = self.pstr(
            'cmd_vel_topic'
        )

        self.ref_topic = self.pstr(
            'ref_topic'
        )

        self.state_topic = self.pstr(
            'state_topic'
        )

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.ref_pub = self.create_publisher(
            Twist,
            self.ref_topic,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            10
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.create_subscription(
            String,
            self.control_error_topic,
            self.control_error_callback,
            20
        )

        self.create_subscription(
            String,
            self.lane_state_topic,
            self.lane_state_callback,
            20
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Bool,
            self.pstr('runtime_enable_topic'),
            self.runtime_enable_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.pstr('emergency_stop_topic'),
            self.emergency_stop_callback,
            10
        )

        # ============================================================
        # ENABLE STATE
        # ============================================================

        self.runtime_enable = False
        self.emergency_stop = False

        # ============================================================
        # CONTROL ERROR STATE
        # ============================================================

        self.last_error_time = -1.0
        self.last_valid_error_time = -1.0
        self.first_valid_error_time = -1.0
        self.last_measurement_time = None

        self.raw_valid = False
        self.raw_valid_reason = 'waiting'

        self.raw_e_lat = 0.0
        self.raw_e_heading = 0.0
        self.raw_kappa = 0.0
        self.raw_confidence = 0.0
        self.raw_lane_state = ''
        self.raw_lookahead = 0.0

        window = max(
            1,
            self.pint('median_window')
        )

        self.lat_buffer = deque(
            maxlen=window
        )

        self.heading_buffer = deque(
            maxlen=window
        )

        self.e_lat_f = 0.0
        self.e_heading_f = 0.0

        self.de_lat_f = 0.0
        self.de_heading_f = 0.0

        self.previous_lat_measurement = 0.0
        self.previous_heading_measurement = 0.0

        # ============================================================
        # PLANNER STATE
        # ============================================================

        self.intent_hint = 'UNKNOWN'
        self.trajectory_hint = 'UNKNOWN'
        self.planner_status_hint = 'UNKNOWN'

        self.lane_state_debug = {}
        self.last_lane_state_time = -1.0

        # ============================================================
        # ODOM STATE
        # ============================================================

        self.odom_v = 0.0
        self.odom_omega = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.last_odom_time = -1.0

        # ============================================================
        # LIDAR STATE
        # ============================================================

        self.front_min = float('inf')
        self.last_scan_time = -1.0

        # ============================================================
        # OUTER LOOP STATE
        # ============================================================

        self.v_ref = 0.0
        self.omega_ref = 0.0

        # ============================================================
        # INNER LOOP STATE
        # ============================================================

        self.v_left_ref = 0.0
        self.v_right_ref = 0.0

        self.v_left_measured = 0.0
        self.v_right_measured = 0.0

        self.left_error = 0.0
        self.right_error = 0.0

        self.previous_left_error = 0.0
        self.previous_right_error = 0.0

        self.d_left_error_f = 0.0
        self.d_right_error_f = 0.0

        self.left_correction = 0.0
        self.right_correction = 0.0

        self.v_left_cmd = 0.0
        self.v_right_cmd = 0.0

        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        self.control_mode = 'waiting'

        self.previous_loop_time = time.monotonic()

        # ============================================================
        # DYNAMIC PARAMETER VALIDATION
        # ============================================================

        self.add_on_set_parameters_callback(
            self.parameter_update_callback
        )

        # ============================================================
        # CONTROL TIMER
        # ============================================================

        frequency = max(
            5.0,
            self.pfloat('control_hz')
        )

        self.create_timer(
            1.0 / frequency,
            self.control_loop
        )

        signal.signal(
            signal.SIGINT,
            self.signal_handler
        )

        signal.signal(
            signal.SIGTERM,
            self.signal_handler
        )

        self.get_logger().info(
            f'{self.VERSION} started'
        )

        self.get_logger().info(
            f'control_error: {self.control_error_topic}'
        )

        self.get_logger().info(
            f'lane_state: {self.lane_state_topic}'
        )

        self.get_logger().info(
            f'odom: {self.odom_topic}'
        )

        self.get_logger().info(
            f'cmd_vel: {self.cmd_vel_topic}'
        )

        self.get_logger().info(
            'Outer loop: lateral + heading PD'
        )

        self.get_logger().info(
            'Inner loop: left/right wheel-speed PD'
        )

        self.get_logger().info(
            'Pure Pursuit is not used'
        )

    # ================================================================
    # PARAMETER HELPERS
    # ================================================================

    def pfloat(self, name):
        return float(
            self.get_parameter(name).value
        )

    def pint(self, name):
        return int(
            self.get_parameter(name).value
        )

    def pbool(self, name):
        return bool(
            self.get_parameter(name).value
        )

    def pstr(self, name):
        return str(
            self.get_parameter(name).value
        )

    def parameter_update_callback(self, parameters):

        positive_parameters = {
            'control_hz',
            'track_width_m',
            'linear_cmd_scale',
            'linear_cmd_max',
            'angular_cmd_max',
            'wheel_ref_max_mps',
            'wheel_cmd_max_mps',
            'error_timeout_s',
            'odom_timeout_s',
        }

        for parameter in parameters:

            if (
                parameter.name in positive_parameters
                and
                float(parameter.value) <= 0.0
            ):

                return SetParametersResult(
                    successful=False,
                    reason=(
                        f'{parameter.name} '
                        'must be greater than zero'
                    )
                )

        return SetParametersResult(
            successful=True
        )

    @staticmethod
    def alpha_from_tau(dt, tau):

        tau = max(
            0.001,
            float(tau)
        )

        return 1.0 - math.exp(
            -max(dt, 0.0) / tau
        )

    @staticmethod
    def make_twist(v, omega):

        message = Twist()

        message.linear.x = float(v)
        message.angular.z = float(omega)

        return message

    @staticmethod
    def first_text(data, keys):

        for key in keys:

            if (
                key in data
                and
                data[key] is not None
            ):

                value = str(
                    data[key]
                ).strip().upper()

                if value:
                    return value

        return ''

    def now_s(self):
        return time.monotonic()

    # ================================================================
    # ENABLE / SAFETY
    # ================================================================

    def is_enabled(self):

        return (
            self.pbool('enable_cmd')
            or
            self.runtime_enable
        )

    def runtime_enable_callback(self, message):

        self.runtime_enable = bool(
            message.data
        )

        self.get_logger().warn(
            f'runtime_enable={self.runtime_enable}'
        )

        if not self.is_enabled():

            self.reset_controller_state()
            self.publish_stop_burst()

    def emergency_stop_callback(self, message):

        self.emergency_stop = bool(
            message.data
        )

        self.get_logger().error(
            f'emergency_stop={self.emergency_stop}'
        )

        if self.emergency_stop:

            self.reset_controller_state()
            self.publish_stop_burst()

    def signal_handler(self, signum, _frame):

        self.get_logger().warn(
            f'Signal {signum}: sending stop burst'
        )

        self.publish_stop_burst()

        if rclpy.ok():
            rclpy.shutdown()

    def reset_controller_state(self):

        self.v_ref = 0.0
        self.omega_ref = 0.0

        self.v_left_ref = 0.0
        self.v_right_ref = 0.0

        self.left_error = 0.0
        self.right_error = 0.0

        self.previous_left_error = 0.0
        self.previous_right_error = 0.0

        self.d_left_error_f = 0.0
        self.d_right_error_f = 0.0

        self.left_correction = 0.0
        self.right_correction = 0.0

        self.v_left_cmd = 0.0
        self.v_right_cmd = 0.0

        self.v_cmd = 0.0
        self.omega_cmd = 0.0

    def publish_stop_burst(self):

        stop = self.make_twist(
            0.0,
            0.0
        )

        count = max(
            3,
            self.pint('stop_burst_count')
        )

        interval = max(
            0.005,
            self.pfloat('stop_burst_dt')
        )

        for _ in range(count):

            self.cmd_pub.publish(stop)
            time.sleep(interval)

    def cmd_vel_conflict_detected(self):

        if (
            not self.pbool('check_cmd_vel_conflict')
            or
            self.pbool('allow_cmd_vel_conflict')
        ):
            return False, []

        publisher_information = (
            self.get_publishers_info_by_topic(
                self.cmd_vel_topic
            )
        )

        publishers = []

        for information in publisher_information:

            if information.node_namespace == '/':

                publishers.append(
                    information.node_name
                )

            else:

                publishers.append(
                    f'{information.node_namespace}/'
                    f'{information.node_name}'
                )

        return (
            len(publisher_information) > 1,
            publishers
        )

    def maybe_publish_cmd(self, v, omega):

        conflict, publishers = (
            self.cmd_vel_conflict_detected()
        )

        if not self.is_enabled():

            return (
                False,
                conflict,
                publishers,
                'enable_cmd_false'
            )

        if conflict:

            if self.pbool(
                'publish_zero_on_conflict'
            ):

                self.cmd_pub.publish(
                    self.make_twist(
                        0.0,
                        0.0
                    )
                )

            return (
                False,
                True,
                publishers,
                'cmd_vel_conflict'
            )

        self.cmd_pub.publish(
            self.make_twist(
                v,
                omega
            )
        )

        return (
            True,
            False,
            publishers,
            'published'
        )

    # ================================================================
    # LANE STATE PARSING
    # ================================================================

    def lane_state_callback(self, message):

        try:

            data = json.loads(
                message.data
            )

            if not isinstance(data, dict):
                return

        except Exception:
            return

        self.lane_state_debug = data
        self.last_lane_state_time = self.now_s()

        intent_text = self.first_text(
            data,
            [
                'active_intent',
                'committed_intent',
                'intent',
                'current_intent',
                'route_intent',
            ]
        )

        lane_text = self.first_text(
            data,
            [
                'active_lane_state',
                'committed_lane_state',
                'lane_state',
                'trajectory_state',
            ]
        )

        trajectory_text = self.first_text(
            data,
            [
                'trajectory_status',
                'trajectory_mode',
                'manager_mode',
                'commit_state',
            ]
        )

        planner_text = self.first_text(
            data,
            [
                'planner_status',
                'planner_state',
                'replan_reason',
                'status',
            ]
        )

        active_text = (
            f'{intent_text} {lane_text}'
        )

        if (
            'LANE_CHANGE' in active_text
            or
            'CHANGE_LANE' in active_text
        ):

            self.intent_hint = 'LANE_CHANGE'

        elif 'TURN_LEFT' in active_text:

            self.intent_hint = 'TURN_LEFT'

        elif 'TURN_RIGHT' in active_text:

            self.intent_hint = 'TURN_RIGHT'

        elif (
            'FOLLOW_MAIN' in active_text
            or
            lane_text == 'MAIN'
        ):

            self.intent_hint = 'FOLLOW_MAIN'

        else:

            self.intent_hint = 'UNKNOWN'

        trajectory_joined = (
            f'{trajectory_text} {lane_text}'
        )

        if 'RECOVERY' in trajectory_joined:

            self.trajectory_hint = 'RECOVERY'

        elif 'HOLD' in trajectory_joined:

            self.trajectory_hint = 'HOLD'

        elif (
            'SOFT' in trajectory_joined
            or
            'REPLAN' in trajectory_joined
        ):

            self.trajectory_hint = 'SOFT_REPLAN'

        elif (
            'COMMITTED' in trajectory_joined
            or
            'ACTIVE' in trajectory_joined
            or
            lane_text == 'FOLLOW_MAIN'
        ):

            self.trajectory_hint = 'COMMITTED'

        else:

            self.trajectory_hint = 'UNKNOWN'

        planner_joined = (
            f'{planner_text} {trajectory_text}'
        )

        if 'BLOCKED_BY_MARKING' in planner_joined:

            self.planner_status_hint = (
                'BLOCKED_BY_MARKING'
            )

        elif 'DROPOUT' in planner_joined:

            self.planner_status_hint = 'DROPOUT'

        elif 'REPLAN' in planner_joined:

            self.planner_status_hint = 'REPLAN'

        elif 'HOLD' in planner_joined:

            self.planner_status_hint = 'HOLD'

        else:

            self.planner_status_hint = 'UNKNOWN'

    # ================================================================
    # CONTROL ERROR EXTRACTION
    # ================================================================

    def extract_control_error(self, data):

        e_lat = None

        for key in [
            'lateral_error_m',
            'lateral_error',
            'e_lat_m',
            'e_y_m',
            'x_error_m',
        ]:

            if key in data:

                e_lat = finite_float(
                    data.get(key)
                )

                break

        if e_lat is None:

            for key in [
                'epsilon_x_mm',
                'x_mm',
                'e_y_mm',
                'e_lat_mm',
            ]:

                if key in data:

                    value = finite_float(
                        data.get(key)
                    )

                    if value is not None:

                        e_lat = value / 1000.0

                        break

        if e_lat is None:
            e_lat = 0.0

        e_heading = None

        for key in [
            'heading_error_rad',
            'heading_error',
            'e_theta_rad',
            'theta_error_rad',
            'theta_rad',
        ]:

            if key in data:

                e_heading = finite_float(
                    data.get(key)
                )

                break

        if e_heading is None:
            e_heading = 0.0

        e_lat = (
            self.pfloat('epsilon_sign')
            * e_lat
            +
            self.pfloat('x_bias_m')
        )

        e_heading = (
            self.pfloat('theta_sign')
            * e_heading
        )

        curvature_inv_mm = finite_float(
            data.get('curvature_inv_mm'),
            None
        )

        if curvature_inv_mm is not None:

            kappa = (
                curvature_inv_mm
                * 1000.0
            )

        else:

            kappa = finite_float(
                data.get(
                    'curvature_m_inv',
                    data.get(
                        'kappa',
                        data.get(
                            'curvature',
                            0.0
                        )
                    )
                ),
                0.0
            )

        kappa = clamp(
            float(kappa),
            -5.0,
            5.0
        )

        confidence = finite_float(
            data.get(
                'confidence',
                data.get(
                    'conf',
                    data.get(
                        'prob',
                        1.0
                    )
                )
            ),
            1.0
        )

        confidence = clamp(
            float(confidence),
            0.0,
            1.0
        )

        lane_state = str(
            data.get(
                'lane_state',
                ''
            )
        ).strip().upper()

        lookahead = finite_float(
            data.get(
                'lookahead_m',
                data.get(
                    'epsilon_y_m',
                    0.0
                )
            ),
            0.0
        )

        raw_valid = parse_bool(
            data.get('valid'),
            True
        )

        lane_valid = parse_bool(
            data.get('lane_valid'),
            True
        )

        if lane_state in {
            'LOST',
            'INVALID',
            'NO_LANE',
            'NONE',
        }:

            return (
                False,
                'lane_state_invalid',
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                float(lookahead)
            )

        if abs(e_lat) > self.pfloat(
            'max_abs_x_m'
        ):

            return (
                False,
                'lateral_outlier',
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                float(lookahead)
            )

        if abs(e_heading) > self.pfloat(
            'max_abs_theta_rad'
        ):

            return (
                False,
                'heading_outlier',
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                float(lookahead)
            )

        if confidence < self.pfloat(
            'min_confidence'
        ):

            return (
                False,
                'low_confidence',
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                float(lookahead)
            )

        valid = (
            raw_valid
            and
            lane_valid
        )

        if lane_state == 'FOLLOW_MAIN':
            valid = True

        return (
            valid,
            'ok' if valid else 'invalid_flags',
            e_lat,
            e_heading,
            kappa,
            confidence,
            lane_state,
            float(lookahead)
        )

    # ================================================================
    # CONTROL ERROR CALLBACK
    #
    # Đạo hàm chỉ được cập nhật khi có frame perception mới.
    # Không tính derivative lại ở 50 Hz trên cùng một mẫu.
    # ================================================================

    def control_error_callback(self, message):

        now = self.now_s()

        try:

            data = json.loads(
                message.data
            )

            if not isinstance(data, dict):
                return

        except Exception as exception:

            self.get_logger().warn(
                f'Invalid control_error JSON: '
                f'{exception}'
            )

            return

        (
            valid,
            reason,
            e_lat,
            e_heading,
            kappa,
            confidence,
            lane_state,
            lookahead,
        ) = self.extract_control_error(data)

        self.last_error_time = now

        self.raw_valid = valid
        self.raw_valid_reason = reason

        self.raw_e_lat = e_lat
        self.raw_e_heading = e_heading
        self.raw_kappa = kappa
        self.raw_confidence = confidence
        self.raw_lane_state = lane_state
        self.raw_lookahead = lookahead

        if not valid:
            return

        self.last_valid_error_time = now

        if self.first_valid_error_time < 0.0:
            self.first_valid_error_time = now

        desired_window = max(
            1,
            self.pint('median_window')
        )

        if self.lat_buffer.maxlen != desired_window:

            self.lat_buffer = deque(
                list(self.lat_buffer)[
                    -desired_window:
                ],
                maxlen=desired_window
            )

            self.heading_buffer = deque(
                list(self.heading_buffer)[
                    -desired_window:
                ],
                maxlen=desired_window
            )

        self.lat_buffer.append(
            e_lat
        )

        self.heading_buffer.append(
            e_heading
        )

        target_lat = statistics.median(
            self.lat_buffer
        )

        target_heading = statistics.median(
            self.heading_buffer
        )

        if self.last_measurement_time is None:

            measurement_dt = 0.10

        else:

            measurement_dt = clamp(
                now - self.last_measurement_time,
                0.001,
                1.0
            )

        self.last_measurement_time = now

        alpha = self.alpha_from_tau(
            measurement_dt,
            self.pfloat(
                'error_filter_tau_s'
            )
        )

        self.e_lat_f = (
            (1.0 - alpha)
            * self.e_lat_f
            +
            alpha
            * target_lat
        )

        self.e_heading_f = (
            (1.0 - alpha)
            * self.e_heading_f
            +
            alpha
            * target_heading
        )

        lat_for_derivative = (
            0.0
            if abs(self.e_lat_f)
            <
            self.pfloat('x_deadband_m')
            else
            self.e_lat_f
        )

        heading_for_derivative = (
            0.0
            if abs(self.e_heading_f)
            <
            self.pfloat(
                'theta_deadband_rad'
            )
            else
            self.e_heading_f
        )

        de_lat_raw = (
            lat_for_derivative
            -
            self.previous_lat_measurement
        ) / measurement_dt

        de_heading_raw = (
            heading_for_derivative
            -
            self.previous_heading_measurement
        ) / measurement_dt

        self.previous_lat_measurement = (
            lat_for_derivative
        )

        self.previous_heading_measurement = (
            heading_for_derivative
        )

        de_lat_raw = clamp(
            de_lat_raw,
            -self.pfloat(
                'derivative_clip_lat_mps'
            ),
            self.pfloat(
                'derivative_clip_lat_mps'
            )
        )

        de_heading_raw = clamp(
            de_heading_raw,
            -self.pfloat(
                'derivative_clip_theta_rps'
            ),
            self.pfloat(
                'derivative_clip_theta_rps'
            )
        )

        derivative_alpha = self.alpha_from_tau(
            measurement_dt,
            self.pfloat(
                'derivative_filter_tau_s'
            )
        )

        self.de_lat_f = (
            (1.0 - derivative_alpha)
            * self.de_lat_f
            +
            derivative_alpha
            * de_lat_raw
        )

        self.de_heading_f = (
            (1.0 - derivative_alpha)
            * self.de_heading_f
            +
            derivative_alpha
            * de_heading_raw
        )

    # ================================================================
    # ODOM CALLBACK
    # ================================================================

    def odom_callback(self, message):

        self.odom_x = float(
            message.pose.pose.position.x
        )

        self.odom_y = float(
            message.pose.pose.position.y
        )

        self.odom_yaw = yaw_from_quaternion(
            message.pose.pose.orientation
        )

        self.odom_v = float(
            message.twist.twist.linear.x
        )

        self.odom_omega = float(
            message.twist.twist.angular.z
        )

        self.last_odom_time = self.now_s()

    # ================================================================
    # LIDAR CALLBACK
    # ================================================================

    def scan_callback(self, message):

        front_angle = math.radians(
            self.pfloat(
                'front_angle_deg'
            )
        )

        valid_ranges = []

        angle = message.angle_min

        for distance in message.ranges:

            if (
                math.isfinite(distance)
                and
                message.range_min
                <= distance
                <= message.range_max
                and
                abs(angle)
                <= front_angle
            ):

                valid_ranges.append(
                    float(distance)
                )

            angle += message.angle_increment

        self.front_min = (
            min(valid_ranges)
            if valid_ranges
            else
            float('inf')
        )

        self.last_scan_time = self.now_s()

    # ================================================================
    # PLANNER PROFILE
    #
    # Công thức outer PD không thay đổi.
    # Chỉ thay multiplier.
    # ================================================================

    def profile_multiplier(self):

        gain_multiplier = 1.0
        speed_multiplier = 1.0
        profile = 'follow_main'

        if self.trajectory_hint == 'RECOVERY':

            gain_multiplier = self.pfloat(
                'recovery_gain_multiplier'
            )

            speed_multiplier = (
                self.pfloat('v_recovery_max')
                /
                max(
                    self.pfloat('v_max'),
                    0.000001
                )
            )

            profile = 'recovery'

        elif self.intent_hint == 'LANE_CHANGE':

            gain_multiplier = self.pfloat(
                'lane_change_gain_multiplier'
            )

            speed_multiplier = (
                self.pfloat(
                    'v_lane_change_max'
                )
                /
                max(
                    self.pfloat('v_max'),
                    0.000001
                )
            )

            profile = 'lane_change'

        elif (
            self.trajectory_hint
            ==
            'SOFT_REPLAN'
            or
            self.planner_status_hint
            ==
            'REPLAN'
        ):

            gain_multiplier = self.pfloat(
                'soft_replan_gain_multiplier'
            )

            speed_multiplier = 0.70
            profile = 'soft_replan'

        elif (
            self.trajectory_hint == 'HOLD'
            or
            self.planner_status_hint == 'HOLD'
        ):

            if self.pbool(
                'hold_on_planner_hold'
            ):

                speed_multiplier = 0.0

            else:

                speed_multiplier = 0.55

            gain_multiplier = 0.75
            profile = 'planner_hold'

        return (
            gain_multiplier,
            speed_multiplier,
            profile
        )

    # ================================================================
    # OUTER CASCADE PD
    # ================================================================

    def compute_outer_pd(self, dt):

        e_lat = (
            0.0
            if abs(self.e_lat_f)
            <
            self.pfloat('x_deadband_m')
            else
            self.e_lat_f
        )

        e_heading = (
            0.0
            if abs(self.e_heading_f)
            <
            self.pfloat(
                'theta_deadband_rad'
            )
            else
            self.e_heading_f
        )

        (
            gain_multiplier,
            speed_multiplier,
            profile
        ) = self.profile_multiplier()

        p_lat = (
            self.pfloat('outer_kp_lat')
            * e_lat
        )

        d_lat = (
            self.pfloat('outer_kd_lat')
            * self.de_lat_f
        )

        p_heading = (
            self.pfloat(
                'outer_kp_heading'
            )
            * e_heading
        )

        d_heading = (
            self.pfloat(
                'outer_kd_heading'
            )
            * self.de_heading_f
        )

        omega_pd = gain_multiplier * (
            p_lat
            +
            d_lat
            +
            p_heading
            +
            d_heading
        )

        omega_target = (
            self.pfloat(
                'outer_control_sign'
            )
            * omega_pd
        )

        if self.pbool('invert_angular'):
            omega_target = -omega_target

        if abs(omega_target) < self.pfloat(
            'omega_ref_deadband'
        ):
            omega_target = 0.0

        omega_target = clamp(
            omega_target,
            -abs(
                self.pfloat(
                    'omega_ref_max'
                )
            ),
            abs(
                self.pfloat(
                    'omega_ref_max'
                )
            )
        )

        # ------------------------------------------------------------
        # Speed scheduling
        # ------------------------------------------------------------

        speed_factor = math.exp(
            -self.pfloat(
                'speed_slow_lat'
            )
            * abs(e_lat)
            -
            self.pfloat(
                'speed_slow_heading'
            )
            * abs(e_heading)
            -
            self.pfloat(
                'speed_slow_curvature'
            )
            * abs(self.raw_kappa)
        )

        v_target = (
            self.pfloat('v_max')
            * speed_factor
            * speed_multiplier
        )

        if (
            abs(self.raw_kappa) > 0.25
            or
            abs(e_heading) > 0.12
        ):

            v_target = max(
                v_target,
                self.pfloat(
                    'v_curve_min'
                )
            )

        maximum_profile_speed = (
            self.pfloat('v_max')
            * speed_multiplier
        )

        if speed_multiplier > 0.0:

            v_target = clamp(
                v_target,
                min(
                    self.pfloat('v_min'),
                    maximum_profile_speed
                ),
                maximum_profile_speed
            )

        else:

            v_target = 0.0

        # ------------------------------------------------------------
        # v_ref slew rate
        # ------------------------------------------------------------

        if v_target >= self.v_ref:

            v_rate = self.pfloat(
                'v_ref_rate_up'
            )

        else:

            v_rate = self.pfloat(
                'v_ref_rate_down'
            )

        self.v_ref = approach(
            self.v_ref,
            v_target,
            v_rate * dt
        )

        # ------------------------------------------------------------
        # omega_ref slew rate
        # ------------------------------------------------------------

        crossing_zero = (
            self.omega_ref
            * omega_target
            < 0.0
            and
            abs(self.omega_ref)
            > 0.0001
            and
            abs(omega_target)
            > 0.0001
        )

        if crossing_zero:

            omega_rate = self.pfloat(
                'omega_zero_cross_rate'
            )

        elif abs(omega_target) >= abs(
            self.omega_ref
        ):

            omega_rate = self.pfloat(
                'omega_ref_rate_up'
            )

        else:

            omega_rate = self.pfloat(
                'omega_ref_rate_down'
            )

        self.omega_ref = approach(
            self.omega_ref,
            omega_target,
            omega_rate * dt
        )

        debug = {
            'profile': profile,

            'gain_multiplier':
                gain_multiplier,

            'speed_multiplier':
                speed_multiplier,

            'e_lat_used_m':
                e_lat,

            'e_heading_used_rad':
                e_heading,

            'p_lat':
                p_lat,

            'd_lat':
                d_lat,

            'p_heading':
                p_heading,

            'd_heading':
                d_heading,

            'omega_pd':
                omega_pd,

            'omega_target':
                omega_target,

            'v_target':
                v_target,

            'speed_factor':
                speed_factor,
        }

        return debug

    # ================================================================
    # INNER WHEEL-SPEED PD
    # ================================================================

    def compute_inner_wheel_pd(
        self,
        dt
    ):

        track = max(
            0.05,
            self.pfloat(
                'track_width_m'
            )
        )

        # ------------------------------------------------------------
        # Reference wheel-group speeds
        # ------------------------------------------------------------

        self.v_left_ref = (
            self.v_ref
            -
            0.5
            * track
            * self.omega_ref
        )

        self.v_right_ref = (
            self.v_ref
            +
            0.5
            * track
            * self.omega_ref
        )

        wheel_ref_limit = abs(
            self.pfloat(
                'wheel_ref_max_mps'
            )
        )

        self.v_left_ref = clamp(
            self.v_left_ref,
            -wheel_ref_limit,
            wheel_ref_limit
        )

        self.v_right_ref = clamp(
            self.v_right_ref,
            -wheel_ref_limit,
            wheel_ref_limit
        )

        # ------------------------------------------------------------
        # Measured wheel-group speeds from odometry
        # ------------------------------------------------------------

        self.v_left_measured = (
            self.odom_v
            -
            0.5
            * track
            * self.odom_omega
        )

        self.v_right_measured = (
            self.odom_v
            +
            0.5
            * track
            * self.odom_omega
        )

        self.left_error = (
            self.v_left_ref
            -
            self.v_left_measured
        )

        self.right_error = (
            self.v_right_ref
            -
            self.v_right_measured
        )

        error_deadband = self.pfloat(
            'inner_error_deadband_mps'
        )

        if abs(self.left_error) < error_deadband:
            self.left_error = 0.0

        if abs(self.right_error) < error_deadband:
            self.right_error = 0.0

        # ------------------------------------------------------------
        # Derivative wheel errors
        # ------------------------------------------------------------

        d_left_raw = (
            self.left_error
            -
            self.previous_left_error
        ) / max(dt, 0.001)

        d_right_raw = (
            self.right_error
            -
            self.previous_right_error
        ) / max(dt, 0.001)

        self.previous_left_error = (
            self.left_error
        )

        self.previous_right_error = (
            self.right_error
        )

        derivative_alpha = self.alpha_from_tau(
            dt,
            self.pfloat(
                'inner_derivative_tau_s'
            )
        )

        self.d_left_error_f = (
            (1.0 - derivative_alpha)
            * self.d_left_error_f
            +
            derivative_alpha
            * d_left_raw
        )

        self.d_right_error_f = (
            (1.0 - derivative_alpha)
            * self.d_right_error_f
            +
            derivative_alpha
            * d_right_raw
        )

        # ------------------------------------------------------------
        # Inner PD corrections
        # ------------------------------------------------------------

        self.left_correction = (
            self.pfloat(
                'inner_kp_left'
            )
            * self.left_error
            +
            self.pfloat(
                'inner_kd_left'
            )
            * self.d_left_error_f
        )

        self.right_correction = (
            self.pfloat(
                'inner_kp_right'
            )
            * self.right_error
            +
            self.pfloat(
                'inner_kd_right'
            )
            * self.d_right_error_f
        )

        correction_limit = abs(
            self.pfloat(
                'inner_correction_max_mps'
            )
        )

        self.left_correction = clamp(
            self.left_correction,
            -correction_limit,
            correction_limit
        )

        self.right_correction = clamp(
            self.right_correction,
            -correction_limit,
            correction_limit
        )

        left_target = (
            self.v_left_ref
            +
            self.left_correction
        )

        right_target = (
            self.v_right_ref
            +
            self.right_correction
        )

        # ------------------------------------------------------------
        # No reverse wheel during normal lane following
        # ------------------------------------------------------------

        if (
            not self.pbool(
                'allow_reverse_wheel'
            )
            and
            self.v_ref > 0.0
        ):

            minimum_forward = self.pfloat(
                'minimum_forward_wheel_mps'
            )

            left_target = max(
                left_target,
                minimum_forward
            )

            right_target = max(
                right_target,
                minimum_forward
            )

        command_limit = abs(
            self.pfloat(
                'wheel_cmd_max_mps'
            )
        )

        left_target = clamp(
            left_target,
            -command_limit,
            command_limit
        )

        right_target = clamp(
            right_target,
            -command_limit,
            command_limit
        )

        # ------------------------------------------------------------
        # Wheel command slew rate
        # ------------------------------------------------------------

        if abs(left_target) >= abs(
            self.v_left_cmd
        ):

            left_rate = self.pfloat(
                'wheel_cmd_rate_up_mps2'
            )

        else:

            left_rate = self.pfloat(
                'wheel_cmd_rate_down_mps2'
            )

        if abs(right_target) >= abs(
            self.v_right_cmd
        ):

            right_rate = self.pfloat(
                'wheel_cmd_rate_up_mps2'
            )

        else:

            right_rate = self.pfloat(
                'wheel_cmd_rate_down_mps2'
            )

        self.v_left_cmd = approach(
            self.v_left_cmd,
            left_target,
            left_rate * dt
        )

        self.v_right_cmd = approach(
            self.v_right_cmd,
            right_target,
            right_rate * dt
        )

        return {
            'v_left_ref':
                self.v_left_ref,

            'v_right_ref':
                self.v_right_ref,

            'v_left_measured':
                self.v_left_measured,

            'v_right_measured':
                self.v_right_measured,

            'left_error':
                self.left_error,

            'right_error':
                self.right_error,

            'd_left_error':
                self.d_left_error_f,

            'd_right_error':
                self.d_right_error_f,

            'left_correction':
                self.left_correction,

            'right_correction':
                self.right_correction,

            'left_target':
                left_target,

            'right_target':
                right_target,
        }

    # ================================================================
    # LIDAR SCALE
    # ================================================================

    def lidar_scale(self):

        if not self.pbool(
            'enable_lidar_safety'
        ):

            return (
                1.0,
                False,
                'lidar_disabled'
            )

        if not math.isfinite(
            self.front_min
        ):

            return (
                1.0,
                False,
                'lidar_no_data'
            )

        emergency = self.pfloat(
            'lidar_emergency_distance_m'
        )

        stop = self.pfloat(
            'lidar_stop_distance_m'
        )

        slow = self.pfloat(
            'lidar_slow_distance_m'
        )

        if self.front_min <= emergency:

            return (
                0.0,
                True,
                'lidar_emergency'
            )

        if self.front_min <= stop:

            return (
                0.0,
                True,
                'lidar_stop'
            )

        if self.front_min < slow:

            ratio = clamp(
                (
                    self.front_min
                    -
                    stop
                )
                /
                max(
                    slow - stop,
                    0.000001
                ),
                0.20,
                1.0
            )

            return (
                ratio,
                False,
                'lidar_slow'
            )

        return (
            1.0,
            False,
            'lidar_clear'
        )

    # ================================================================
    # WHEEL COMMANDS TO CMD_VEL
    # ================================================================

    def wheel_commands_to_twist(
        self,
        lidar_ratio,
        force_stop
    ):

        track = max(
            0.05,
            self.pfloat(
                'track_width_m'
            )
        )

        left = (
            self.v_left_cmd
            * lidar_ratio
        )

        right = (
            self.v_right_cmd
            * lidar_ratio
        )

        if (
            force_stop
            or
            self.emergency_stop
        ):

            left = 0.0
            right = 0.0

        v_command = (
            0.5
            * (
                left
                +
                right
            )
        )

        omega_command = (
            right
            -
            left
        ) / track

        if self.pbool(
            'enable_calibration'
        ):

            v_command = (
                v_command
                /
                max(
                    self.pfloat(
                        'linear_cmd_scale'
                    ),
                    0.000001
                )
            )

            # Nhân, không chia.
            omega_command = (
                omega_command
                *
                self.pfloat(
                    'angular_cmd_gain'
                )
            )

        v_command = clamp(
            v_command,
            -abs(
                self.pfloat(
                    'linear_cmd_max'
                )
            ),
            abs(
                self.pfloat(
                    'linear_cmd_max'
                )
            )
        )

        omega_command = clamp(
            omega_command,
            -abs(
                self.pfloat(
                    'angular_cmd_max'
                )
            ),
            abs(
                self.pfloat(
                    'angular_cmd_max'
                )
            )
        )

        return (
            v_command,
            omega_command
        )

    # ================================================================
    # PUBLISH REFERENCE
    # ================================================================

    def publish_reference(self):

        self.ref_pub.publish(
            self.make_twist(
                self.v_ref,
                self.omega_ref
            )
        )

    # ================================================================
    # PUBLISH STATE JSON
    # ================================================================

    def publish_state(self, payload):

        message = String()

        message.data = json.dumps(
            payload,
            ensure_ascii=False
        )

        self.state_pub.publish(
            message
        )

    # ================================================================
    # MAIN CONTROL LOOP
    # ================================================================

    def control_loop(self):

        now = self.now_s()

        dt = clamp(
            now - self.previous_loop_time,
            0.001,
            0.20
        )

        self.previous_loop_time = now

        if self.last_valid_error_time > 0.0:

            error_age = (
                now
                -
                self.last_valid_error_time
            )

        else:

            error_age = 999.0

        if self.last_odom_time > 0.0:

            odom_age = (
                now
                -
                self.last_odom_time
            )

        else:

            odom_age = 999.0

        fresh_error = (
            self.raw_valid
            and
            error_age
            <=
            self.pfloat(
                'error_timeout_s'
            )
        )

        fresh_odom = (
            odom_age
            <=
            self.pfloat(
                'odom_timeout_s'
            )
        )

        startup = (
            self.first_valid_error_time > 0.0
            and
            now - self.first_valid_error_time
            <=
            self.pfloat(
                'startup_hold_s'
            )
        )

        outer_debug = {
            'profile': 'none',
            'gain_multiplier': 0.0,
            'speed_multiplier': 0.0,
            'e_lat_used_m': 0.0,
            'e_heading_used_rad': 0.0,
            'p_lat': 0.0,
            'd_lat': 0.0,
            'p_heading': 0.0,
            'd_heading': 0.0,
            'omega_pd': 0.0,
            'omega_target': 0.0,
            'v_target': 0.0,
            'speed_factor': 0.0,
        }

        # ------------------------------------------------------------
        # Controller mode
        # ------------------------------------------------------------

        if self.emergency_stop:

            self.control_mode = (
                'emergency_stop'
            )

            self.v_ref = 0.0
            self.omega_ref = 0.0

        elif not fresh_odom:

            self.control_mode = (
                'odom_timeout'
            )

            self.v_ref = 0.0
            self.omega_ref = 0.0

        elif startup and fresh_error:

            self.control_mode = (
                'startup_straight'
            )

            self.v_ref = approach(
                self.v_ref,
                self.pfloat(
                    'startup_v'
                ),
                self.pfloat(
                    'v_ref_rate_up'
                )
                * dt
            )

            self.omega_ref = approach(
                self.omega_ref,
                0.0,
                self.pfloat(
                    'omega_ref_rate_down'
                )
                * dt
            )

        elif fresh_error:

            self.control_mode = (
                'cascade_pd_tracking'
            )

            outer_debug = (
                self.compute_outer_pd(
                    dt
                )
            )

        elif error_age <= (
            self.pfloat(
                'error_timeout_s'
            )
            +
            self.pfloat(
                'blind_hold_s'
            )
        ):

            self.control_mode = (
                'blind_hold'
            )

            self.v_ref = approach(
                self.v_ref,
                self.pfloat(
                    'blind_v'
                ),
                self.pfloat(
                    'v_ref_rate_down'
                )
                * dt
            )

            self.omega_ref = approach(
                self.omega_ref,
                0.0,
                self.pfloat(
                    'omega_ref_rate_down'
                )
                * dt
            )

        else:

            self.control_mode = (
                'control_error_timeout'
            )

            self.v_ref = approach(
                self.v_ref,
                0.0,
                self.pfloat(
                    'v_ref_rate_down'
                )
                * dt
            )

            self.omega_ref = approach(
                self.omega_ref,
                0.0,
                self.pfloat(
                    'omega_ref_rate_down'
                )
                * dt
            )

        self.publish_reference()

        # ------------------------------------------------------------
        # Inner wheel PD
        # ------------------------------------------------------------

        inner_debug = (
            self.compute_inner_wheel_pd(
                dt
            )
        )

        (
            lidar_ratio,
            lidar_stop,
            lidar_mode
        ) = self.lidar_scale()

        force_stop = (
            self.control_mode
            in {
                'emergency_stop',
                'odom_timeout',
                'control_error_timeout',
            }
            or
            lidar_stop
        )

        (
            self.v_cmd,
            self.omega_cmd
        ) = self.wheel_commands_to_twist(
            lidar_ratio,
            force_stop
        )

        if force_stop:

            self.v_left_cmd = 0.0
            self.v_right_cmd = 0.0

            self.v_cmd = 0.0
            self.omega_cmd = 0.0

        (
            cmd_published,
            conflict,
            publishers,
            publish_reason
        ) = self.maybe_publish_cmd(
            self.v_cmd,
            self.omega_cmd
        )

        # ------------------------------------------------------------
        # State output compatible with dashboard
        # ------------------------------------------------------------

        self.publish_state({
            'node':
                'cascade_controller_v2',

            'version':
                self.VERSION,

            'time_monotonic':
                now,

            'enabled':
                self.is_enabled(),

            'param_enable_cmd':
                self.pbool('enable_cmd'),

            'runtime_enable':
                self.runtime_enable,

            'emergency_stop':
                self.emergency_stop,

            'cmd_published':
                cmd_published,

            'publish_reason':
                publish_reason,

            'cmd_vel_conflict':
                conflict,

            'cmd_vel_publishers':
                publishers,

            'mode':
                self.control_mode,

            'outer_mode':
                outer_debug['profile'],

            'mix_mode':
                'inner_wheel_pd',

            'raw_valid':
                self.raw_valid,

            'raw_valid_reason':
                self.raw_valid_reason,

            'lane_state':
                self.raw_lane_state,

            'confidence':
                self.raw_confidence,

            'error_age_s':
                error_age,

            'odom_age_s':
                odom_age,

            'intent_hint':
                self.intent_hint,

            'trajectory_hint':
                self.trajectory_hint,

            'planner_status_hint':
                self.planner_status_hint,

            'lane_state_debug_age_s':
                (
                    now
                    -
                    self.last_lane_state_time
                    if
                    self.last_lane_state_time
                    > 0.0
                    else
                    -1.0
                ),

            'epsilon_x_mm':
                self.raw_e_lat
                * 1000.0,

            'theta_rad':
                self.raw_e_heading,

            'lookahead_m':
                self.raw_lookahead,

            'kappa_m':
                self.raw_kappa,

            'e_f_m':
                self.e_lat_f,

            'e_f_mm':
                self.e_lat_f
                * 1000.0,

            'theta_f_rad':
                self.e_heading_f,

            'e_used_m':
                outer_debug[
                    'e_lat_used_m'
                ],

            'e_used_mm':
                outer_debug[
                    'e_lat_used_m'
                ]
                * 1000.0,

            'theta_used_rad':
                outer_debug[
                    'e_heading_used_rad'
                ],

            'de_f':
                self.de_lat_f,

            'dtheta_f':
                self.de_heading_f,

            'outer_kp_lat':
                self.pfloat(
                    'outer_kp_lat'
                ),

            'outer_kd_lat':
                self.pfloat(
                    'outer_kd_lat'
                ),

            'outer_kp_heading':
                self.pfloat(
                    'outer_kp_heading'
                ),

            'outer_kd_heading':
                self.pfloat(
                    'outer_kd_heading'
                ),

            'p_lat':
                outer_debug['p_lat'],

            'd_lat':
                outer_debug['d_lat'],

            'p_heading':
                outer_debug[
                    'p_heading'
                ],

            'd_heading':
                outer_debug[
                    'd_heading'
                ],

            'outer_gain_multiplier':
                outer_debug[
                    'gain_multiplier'
                ],

            'speed_factor':
                outer_debug[
                    'speed_factor'
                ],

            'v_des':
                outer_debug[
                    'v_target'
                ],

            'omega_des':
                outer_debug[
                    'omega_target'
                ],

            'omega_raw':
                outer_debug[
                    'omega_pd'
                ],

            'omega_limit':
                self.pfloat(
                    'omega_ref_max'
                ),

            'v_ref':
                self.v_ref,

            'omega_ref':
                self.omega_ref,

            'v_left_ref':
                self.v_left_ref,

            'v_right_ref':
                self.v_right_ref,

            'v_left_des':
                self.v_left_ref,

            'v_right_des':
                self.v_right_ref,

            'v_left_measured':
                self.v_left_measured,

            'v_right_measured':
                self.v_right_measured,

            'v_left_odom':
                self.v_left_measured,

            'v_right_odom':
                self.v_right_measured,

            'left_wheel_error':
                self.left_error,

            'right_wheel_error':
                self.right_error,

            'd_left_wheel_error':
                self.d_left_error_f,

            'd_right_wheel_error':
                self.d_right_error_f,

            'left_pd_correction':
                self.left_correction,

            'right_pd_correction':
                self.right_correction,

            'v_left_cmd':
                self.v_left_cmd,

            'v_right_cmd':
                self.v_right_cmd,

            'v_cmd':
                self.v_cmd,

            'omega_cmd':
                self.omega_cmd,

            'odom_v':
                self.odom_v,

            'odom_omega':
                self.odom_omega,

            'odom_x':
                self.odom_x,

            'odom_y':
                self.odom_y,

            'odom_yaw':
                self.odom_yaw,

            'front_min_m':
                (
                    self.front_min
                    if
                    math.isfinite(
                        self.front_min
                    )
                    else
                    None
                ),

            'lidar_ratio':
                lidar_ratio,

            'lidar_stop':
                lidar_stop,

            'lidar_mode':
                lidar_mode,
        })


def main(args=None):

    rclpy.init(
        args=args
    )

    node = CascadeControllerV2()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        if bool(
            node.get_parameter(
                'always_publish_stop_on_exit'
            ).value
        ):

            node.get_logger().warn(
                'Shutdown: force stop burst'
            )

            node.publish_stop_burst()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
