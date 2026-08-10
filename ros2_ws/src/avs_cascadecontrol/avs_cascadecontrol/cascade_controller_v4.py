#!/usr/bin/env python3

import json
import math
import signal
import statistics
import time
from collections import deque

import rclpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def clamp(value, low, high):
    return max(
        low,
        min(
            high,
            value
        )
    )


def approach(
    current,
    target,
    max_delta
):
    delta = (
        target
        -
        current
    )

    if delta > max_delta:
        return current + max_delta

    if delta < -max_delta:
        return current - max_delta

    return target


def finite_float(
    value,
    default=None
):
    try:
        value = float(value)

        if math.isfinite(value):
            return value

    except (
        TypeError,
        ValueError
    ):
        pass

    return default


def parse_bool(
    value,
    default=True
):
    if value is None:
        return default

    if isinstance(
        value,
        bool
    ):
        return value

    if isinstance(
        value,
        str
    ):
        return (
            value.strip().lower()
            not in {
                "",
                "0",
                "false",
                "no",
                "none",
                "invalid",
                "lost",
            }
        )

    return bool(value)


class CascadeControllerV4(Node):

    VERSION = (
        "cascade_controller_v4_"
        "fast_smooth_true_cascade_pd_1_0"
    )

    def __init__(self):

        super().__init__(
            "cascade_controller_v4"
        )

        # ============================================================
        # TOPICS
        # ============================================================

        self.declare_parameter(
            "control_error_topic",
            "/avs/control_error"
        )

        self.declare_parameter(
            "lane_state_topic",
            "/avs/lane_state"
        )

        self.declare_parameter(
            "odom_topic",
            "/odom_raw"
        )

        self.declare_parameter(
            "scan_topic",
            "/scan"
        )

        self.declare_parameter(
            "cmd_vel_topic",
            "/cmd_vel"
        )

        self.declare_parameter(
            "ref_topic",
            "/avs/cascade_controller_v4_ref"
        )

        self.declare_parameter(
            "state_topic",
            "/avs/cascade_controller_v4_state"
        )

        self.declare_parameter(
            "runtime_enable_topic",
            "/avs/cascade_v4_enable_cmd"
        )

        self.declare_parameter(
            "emergency_stop_topic",
            "/avs/cascade_v3_emergency_stop"
        )

        # ============================================================
        # GENERAL
        # ============================================================

        self.declare_parameter(
            "enable_cmd",
            False
        )

        self.declare_parameter(
            "control_hz",
            50.0
        )

        self.declare_parameter(
            "error_timeout_s",
            0.55
        )

        self.declare_parameter(
            "odom_timeout_s",
            0.45
        )

        # ============================================================
        # CMD_VEL SAFETY
        # ============================================================

        self.declare_parameter(
            "check_cmd_vel_conflict",
            True
        )

        self.declare_parameter(
            "allow_cmd_vel_conflict",
            False
        )

        self.declare_parameter(
            "stop_burst_count",
            20
        )

        self.declare_parameter(
            "stop_burst_dt",
            0.015
        )

        # ============================================================
        # ROBOT GEOMETRY
        # ============================================================

        self.declare_parameter(
            "track_width_m",
            0.135
        )

        self.declare_parameter(
            "wheel_radius_m",
            0.0225
        )

        # ============================================================
        # SIGN CONVENTION
        #
        # epsilon_x > 0 = target bên phải
        # ROS angular.z < 0 = quay phải
        # ============================================================

        self.declare_parameter(
            "epsilon_sign",
            1.0
        )

        self.declare_parameter(
            "theta_sign",
            1.0
        )

        self.declare_parameter(
            "outer_control_sign",
            -1.0
        )

        self.declare_parameter(
            "invert_angular",
            False
        )

        self.declare_parameter(
            "x_bias_m",
            0.0
        )

        # ============================================================
        # INPUT VALIDATION
        # ============================================================

        self.declare_parameter(
            "max_abs_x_m",
            0.45
        )

        self.declare_parameter(
            "max_abs_theta_rad",
            1.15
        )

        self.declare_parameter(
            "min_confidence",
            0.15
        )

        # ============================================================
        # FPS ESTIMATOR
        #
        # FPS được đo trực tiếp từ khoảng thời gian giữa các message
        # /avs/control_error.
        # ============================================================

        self.declare_parameter(
            "fps_filter_tau_s",
            0.50
        )

        self.declare_parameter(
            "fps_initial",
            15.0
        )

        self.declare_parameter(
            "fps_speed_enable",
            True
        )

        self.declare_parameter(
            "target_distance_per_frame_m",
            0.040
        )

        self.declare_parameter(
            "fps_speed_floor",
            0.20
        )

        # ============================================================
        # VISION ERROR FILTER
        #
        # V2 lọc quá nặng.
        #
        # V3:
        # median = 3 frame
        # LPF = 0.14 s
        #
        # phù hợp hơn với 14-18 FPS.
        # ============================================================

        self.declare_parameter(
            "median_window",
            3
        )

        self.declare_parameter(
            "error_filter_tau_s",
            0.14
        )

        self.declare_parameter(
            "derivative_filter_tau_s",
            0.30
        )

        self.declare_parameter(
            "x_deadband_m",
            0.012
        )

        self.declare_parameter(
            "theta_deadband_rad",
            0.025
        )

        # Giới hạn jump perception từng frame
        self.declare_parameter(
            "max_lat_step_per_frame_m",
            0.050
        )

        self.declare_parameter(
            "max_theta_step_per_frame_rad",
            0.20
        )

        self.declare_parameter(
            "derivative_clip_lat_mps",
            0.45
        )

        self.declare_parameter(
            "derivative_clip_theta_rps",
            1.80
        )

        # ============================================================
        # OUTER CASCADE PD
        #
        # omega =
        #
        # sign *
        # (
        #   Kp_lat * e_lat
        # + Kd_lat * de_lat
        # + Kp_theta * e_theta
        # + Kd_theta * de_theta
        # )
        #
        # KHÔNG PURE PURSUIT.
        # ============================================================

        self.declare_parameter(
            "outer_kp_lat",
            0.72
        )

        self.declare_parameter(
            "outer_kd_lat",
            0.35
        )

        self.declare_parameter(
            "outer_kp_heading",
            0.68
        )

        self.declare_parameter(
            "outer_kd_heading",
            0.30
        )

        self.declare_parameter(
            "omega_ref_max",
            0.85
        )

        self.declare_parameter(
            "omega_ref_deadband",
            0.015
        )

        # ============================================================
        # OMEGA SLEW
        #
        # Cho phép cua đủ mạnh,
        # nhưng không nhảy angular tức thời.
        # ============================================================

        # FPS=5Hz dt~0.19s:
        # rate=3.0 -> delta_omega=0.57/frame, dủ để tăng 0->0.40 trong 1 frame
        self.declare_parameter(
            "omega_ref_rate_up",
            3.00
        )

        self.declare_parameter(
            "omega_ref_rate_down",
            2.00
        )

        self.declare_parameter(
            "omega_zero_cross_rate",
            0.50
        )

        # ============================================================
        # LINEAR SPEED
        #
        # Setpoint thẳng = 0.50 m/s
        # ============================================================

        self.declare_parameter(
            "v_setpoint_mps",
            0.32
        )

        # Cua gắt tối thiểu 0.16 m/s
        self.declare_parameter(
            "v_curve_min_mps",
            0.16
        )

        self.declare_parameter(
            "v_min_mps",
            0.10
        )

        # ============================================================
        # CONTINUOUS CURVE SEVERITY
        #
        # Không còn near / mid / curve hard switching.
        #
        # severity tăng liên tục theo:
        # lateral + heading + curvature.
        # ============================================================

        self.declare_parameter(
            "curve_lat_scale_m",
            0.080
        )

        self.declare_parameter(
            "curve_heading_scale_rad",
            0.22
        )

        self.declare_parameter(
            "curve_kappa_scale",
            2.50
        )

        self.declare_parameter(
            "curve_heading_weight",
            0.45
        )

        self.declare_parameter(
            "curve_kappa_weight",
            0.55
        )

        self.declare_parameter(
            "curve_lat_weight",
            0.30
        )

        self.declare_parameter(
            "curve_severity_deadband",
            0.12
        )

        self.declare_parameter(
            "curve_slow_gain",
            1.15
        )

        # Giữ severity lâu hơn khi đã qua cua/về thẳng (0.5s)
        self.declare_parameter(
            "severity_filter_tau_s",
            0.50
        )

        # ============================================================
        # SPEED SLEW
        # ============================================================

        self.declare_parameter(
            "v_ref_rate_up_mps2",
            0.28
        )

        self.declare_parameter(
            "v_ref_rate_down_mps2",
            2.50
        )

        # ============================================================
        # PLANNER PROFILES
        #
        # QUAN TRỌNG:
        #
        # planner_status=HOLD không còn ép giảm tốc.
        #
        # Chỉ trajectory_hint thật sự = HOLD mới dừng.
        # ============================================================

        self.declare_parameter(
            "recovery_gain_multiplier",
            1.12
        )

        self.declare_parameter(
            "recovery_speed_max_mps",
            0.28
        )

        self.declare_parameter(
            "lane_change_gain_multiplier",
            1.05
        )

        self.declare_parameter(
            "lane_change_speed_max_mps",
            0.32
        )

        self.declare_parameter(
            "soft_replan_gain_multiplier",
            0.90
        )

        self.declare_parameter(
            "soft_replan_speed_max_mps",
            0.34
        )

        self.declare_parameter(
            "explicit_hold_speed_mps",
            0.0
        )

        # ============================================================
        # V3.2 - CURVATURE PREVIEW FEED-FORWARD
        #
        # This is NOT Pure Pursuit.
        #
        # omega_ff = k_ff * v_ref * curvature
        #
        # It starts a small steering response before lateral error
        # becomes large.
        # ============================================================

        self.declare_parameter(
            "curvature_ff_sign",
            1.0
        )

        self.declare_parameter(
            "curvature_ff_gain",
            1.20
        )

        self.declare_parameter(
            "curvature_ff_max",
            0.85
        )

        self.declare_parameter(
            "curvature_filter_tau_s",
            0.08
        )

        self.declare_parameter(
            "max_abs_curvature_control",
            3.0
        )

        # ============================================================
        # CONTINUOUS SPEED MAP
        # ============================================================

        # speed map: tighten thresholds for earlier slowdown
        self.declare_parameter(
            "v_mild_mps",
            0.24
        )

        self.declare_parameter(
            "v_medium_mps",
            0.16
        )

        # 0.11 was low enough to stall the inner wheels in a turn.
        #
        # The car is 4WD driven as a left pair and a right pair, so a turn is
        # skid-steered: every wheel has to slip sideways, and the resistance to
        # that is highest exactly when this speed is lowest. Measured from
        # /cmd_vel over run28 (20573 samples): on straights the controller asks
        # for 0.207 m/s and all four wheels turn, but in a sharp turn it sits at
        # 0.110 - this value, to three digits - for 94% of the time, and the
        # inner pair then gets v - omega*0.0675 = 0.048 m/s at the 5th
        # percentile. That is less than half the inner-wheel speed on a straight.
        # Two wheels share one signal but not their friction, so at that duty
        # only the freer one breaks away: the reported "one wheel spins, three
        # stop".
        #
        # 0.16 keeps the inner pair near 0.099 m/s at omega=0.9, about what it
        # gets on a straight. The differential itself was never the problem -
        # measured inner speed never went negative and only once fell below
        # 0.02 m/s in the whole run - so raising the floor, not reshaping the
        # split, is the fix.
        self.declare_parameter(
            "v_sharp_mps",
            0.16
        )

        self.declare_parameter(
            "severity_mild_threshold",
            0.12
        )

        self.declare_parameter(
            "severity_medium_threshold",
            0.35
        )

        self.declare_parameter(
            "severity_sharp_threshold",
            0.65
        )

        # ============================================================
        # STEERING LIMIT BY CURVE SEVERITY
        #
        # High speed / straight:
        #     small omega
        #
        # Strong curve:
        #     robot is already slower, therefore more omega is allowed.
        # ============================================================

        self.declare_parameter(
            "omega_straight_max",
            0.22
        )

        self.declare_parameter(
            "omega_mild_max",
            0.35
        )

        # Python Turn Memory removed, rely on C++ Trajectory Latch.

        self.declare_parameter(
            "omega_medium_max",
            0.55
        )

        self.declare_parameter(
            "omega_sharp_max",
            0.80
        )

        # ============================================================
        # ADAPTIVE STEERING GAIN
        #
        # Straight:
        #     slightly softer -> reduce oscillation.
        #
        # Curves:
        #     progressively stronger -> prevent lane departure.
        # ============================================================

        self.declare_parameter(
            "curve_steer_gain_straight",
            0.90
        )

        self.declare_parameter(
            "curve_steer_gain_mild",
            1.03
        )

        self.declare_parameter(
            "curve_steer_gain_medium",
            1.12
        )

        self.declare_parameter(
            "curve_steer_gain_sharp",
            1.18
        )

        # ============================================================
        # ODOM FILTER FOR INNER LOOP
        # ============================================================

        self.declare_parameter(
            "odom_filter_tau_s",
            0.12
        )

        # ============================================================
        # INNER WHEEL PD
        #
        # v_wheel_cmd =
        #
        # v_wheel_ref
        # + Kp * error
        # + Kd * derivative(error)
        #
        # Gain nhỏ hơn V2 để không bù quá tay.
        # ============================================================

        self.declare_parameter(
            "inner_kp_left",
            0.22
        )

        self.declare_parameter(
            "inner_kd_left",
            0.004
        )

        self.declare_parameter(
            "inner_kp_right",
            0.22
        )

        self.declare_parameter(
            "inner_kd_right",
            0.004
        )

        self.declare_parameter(
            "inner_derivative_tau_s",
            0.15
        )

        self.declare_parameter(
            "inner_error_deadband_mps",
            0.008
        )

        self.declare_parameter(
            "inner_correction_max_mps",
            0.080
        )

        # ============================================================
        # WHEEL LIMIT
        # ============================================================

        self.declare_parameter(
            "wheel_ref_max_mps",
            0.62
        )

        self.declare_parameter(
            "wheel_cmd_max_mps",
            0.68
        )

        self.declare_parameter(
            "wheel_cmd_rate_up_mps2",
            0.75
        )

        self.declare_parameter(
            "wheel_cmd_rate_down_mps2",
            1.20
        )

        self.declare_parameter(
            "allow_reverse_wheel",
            False
        )

        self.declare_parameter(
            "minimum_forward_wheel_mps",
            0.040
        )

        # ============================================================
        # FINAL CMD_VEL
        #
        # Người dùng yêu cầu:
        #
        # linear.x max hệ thống = 1
        # setpoint xe = 0.5
        #
        # V3 hard limit linear.x = 0.50
        # ============================================================

        self.declare_parameter(
            "linear_cmd_scale",
            1.0
        )

        self.declare_parameter(
            "angular_cmd_gain",
            0.90
        )

        self.declare_parameter(
            "linear_cmd_max",
            0.50
        )

        self.declare_parameter(
            "angular_cmd_max",
            0.90
        )

        # ============================================================
        # SPEED-STEERING COUPLING
        #
        # Limit v*|omega| to avoid violent steering while moving fast.
        #
        # omega_dynamic_max =
        #     lateral_accel_limit / max(v_ref, min_speed)
        # ============================================================

        self.declare_parameter(
            "enable_speed_steering_coupling",
            True
        )

        self.declare_parameter(
            "lateral_accel_limit_mps2",
            0.20
        )

        self.declare_parameter(
            "omega_dynamic_floor",
            0.30
        )

        self.declare_parameter(
            "omega_dynamic_ceiling",
            0.90
        )

        # Reduce steering while the robot is still braking
        # into a strong curve.
        self.declare_parameter(
            "brake_before_turn_enable",
            True
        )

        self.declare_parameter(
            "brake_before_turn_severity",
            0.45
        )

        self.declare_parameter(
            "brake_before_turn_speed_margin",
            0.06
        )

        self.declare_parameter(
            "brake_before_turn_omega_scale",
            0.72
        )

        # ============================================================
        # OPTIONAL LIDAR EMERGENCY
        # ============================================================

        self.declare_parameter(
            "enable_lidar_safety",
            False
        )

        self.declare_parameter(
            "front_angle_deg",
            18.0
        )

        self.declare_parameter(
            "lidar_stop_distance_m",
            0.22
        )

        self.declare_parameter(
            "lidar_slow_distance_m",
            0.55
        )

        # ============================================================
        # READ TOPICS
        # ============================================================

        self.control_error_topic = (
            self.pstr(
                "control_error_topic"
            )
        )

        self.lane_state_topic = (
            self.pstr(
                "lane_state_topic"
            )
        )

        self.odom_topic = (
            self.pstr(
                "odom_topic"
            )
        )

        self.scan_topic = (
            self.pstr(
                "scan_topic"
            )
        )

        self.cmd_vel_topic = (
            self.pstr(
                "cmd_vel_topic"
            )
        )

        self.ref_topic = (
            self.pstr(
                "ref_topic"
            )
        )

        self.state_topic = (
            self.pstr(
                "state_topic"
            )
        )

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.cmd_pub = (
            self.create_publisher(
                Twist,
                self.cmd_vel_topic,
                10
            )
        )

        self.ref_pub = (
            self.create_publisher(
                Twist,
                self.ref_topic,
                10
            )
        )

        self.state_pub = (
            self.create_publisher(
                String,
                self.state_topic,
                10
            )
        )

        # ============================================================
        # SUBSCRIBERS
        # ============================================================

        self.create_subscription(
            String,
            self.control_error_topic,
            self.control_error_cb,
            20
        )

        self.create_subscription(
            String,
            self.lane_state_topic,
            self.lane_state_cb,
            20
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Bool,
            self.pstr(
                "runtime_enable_topic"
            ),
            self.enable_cb,
            10
        )

        self.create_subscription(
            Bool,
            self.pstr(
                "emergency_stop_topic"
            ),
            self.estop_cb,
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

        self.raw_valid = False
        self.raw_reason = "waiting"

        self.raw_e_lat = 0.0
        self.raw_e_heading = 0.0
        self.raw_kappa = 0.0
        self.raw_confidence = 0.0
        self.raw_lane_state = ""
        self.raw_lookahead = 0.0

        self.last_error_rx = -1.0
        self.last_valid_error_rx = -1.0

        # ============================================================
        # FPS
        # ============================================================

        self.last_vision_rx = None

        self.vision_fps = (
            self.pfloat(
                "fps_initial"
            )
        )

        # ============================================================
        # ERROR FILTER STATE
        # ============================================================

        median_count = max(
            1,
            self.pint(
                "median_window"
            )
        )

        self.lat_buffer = deque(
            maxlen=median_count
        )

        self.heading_buffer = deque(
            maxlen=median_count
        )

        self.e_lat_f = 0.0
        self.e_heading_f = 0.0

        self.prev_derivative_lat = 0.0
        self.prev_derivative_heading = 0.0

        self.de_lat_f = 0.0
        self.de_heading_f = 0.0

        self._last_valid_frame_time = None

        # ============================================================
        # PLANNER STATE
        # ============================================================

        self.intent_hint = "UNKNOWN"
        self.trajectory_hint = "UNKNOWN"
        self.planner_status_hint = "UNKNOWN"

        self.last_lane_state_rx = -1.0

        # ============================================================
        # ODOM
        # ============================================================

        self.odom_v_raw = 0.0
        self.odom_omega_raw = 0.0

        self.odom_v_f = 0.0
        self.odom_omega_f = 0.0

        self.odom_filter_initialized = False

        self.last_odom_rx = -1.0
        self.last_odom_filter_time = None

        self.odom_x = 0.0
        self.odom_y = 0.0

        # ============================================================
        # LIDAR
        # ============================================================

        self.front_min = math.inf

        # ============================================================
        # OUTER STATE
        # ============================================================

        self.severity_f = 0.0
        self.kappa_f = 0.0

        self.v_target = 0.0
        self.v_ref = 0.0

        self.omega_target = 0.0
        self.omega_ref = 0.0

        # ============================================================
        # INNER STATE
        # ============================================================

        self.v_left_ref = 0.0
        self.v_right_ref = 0.0

        self.v_left_meas = 0.0
        self.v_right_meas = 0.0

        self.left_error = 0.0
        self.right_error = 0.0

        self.prev_left_error = 0.0
        self.prev_right_error = 0.0

        self.d_left_f = 0.0
        self.d_right_f = 0.0

        self.inner_correction_max = 0.0

        self.left_correction = 0.0
        self.right_correction = 0.0

        self.v_left_cmd = 0.0
        self.v_right_cmd = 0.0

        self.v_cmd = 0.0
        self.omega_cmd = 0.0

        self.outer_debug = {}

        self.control_mode = "waiting"

        self.previous_loop_time = (
            time.monotonic()
        )

        # ============================================================
        # PARAMETER CALLBACK
        # ============================================================

        self.add_on_set_parameters_callback(
            self.parameter_cb
        )

        # ============================================================
        # TIMER
        # ============================================================

        hz = max(
            10.0,
            self.pfloat(
                "control_hz"
            )
        )

        self.create_timer(
            1.0 / hz,
            self.control_loop
        )

        # ============================================================
        # SIGNALS
        # ============================================================

        signal.signal(
            signal.SIGINT,
            self.signal_handler
        )

        signal.signal(
            signal.SIGTERM,
            self.signal_handler
        )

        self.get_logger().info(
            self.VERSION
        )

        self.get_logger().info(
            "Outer loop = lateral/heading PD, no Pure Pursuit"
        )

        self.get_logger().info(
            "Inner loop = left/right wheel-group speed PD"
        )

        self.get_logger().info(
            "Straight speed setpoint = 0.50 m/s"
        )

    # ================================================================
    # PARAM HELPERS
    # ================================================================

    def pfloat(
        self,
        name
    ):
        return float(
            self.get_parameter(
                name
            ).value
        )

    def pint(
        self,
        name
    ):
        return int(
            self.get_parameter(
                name
            ).value
        )

    def pbool(
        self,
        name
    ):
        return bool(
            self.get_parameter(
                name
            ).value
        )

    def pstr(
        self,
        name
    ):
        return str(
            self.get_parameter(
                name
            ).value
        )

    @staticmethod
    def alpha(
        dt,
        tau
    ):
        return (
            1.0
            -
            math.exp(
                -max(
                    dt,
                    0.0
                )
                /
                max(
                    tau,
                    0.0001
                )
            )
        )

    @staticmethod
    def twist(
        v,
        omega
    ):
        message = Twist()

        message.linear.x = float(v)
        message.angular.z = float(omega)

        return message

    @staticmethod
    def first_text(
        data,
        keys
    ):
        for key in keys:

            value = data.get(
                key
            )

            if value is not None:

                value = str(
                    value
                ).strip().upper()

                if value:
                    return value

        return ""

    # ================================================================
    # PARAMETER VALIDATION
    # ================================================================

    def parameter_cb(
        self,
        parameters
    ):
        for parameter in parameters:

            if (
                parameter.name
                in {
                    "control_hz",
                    "track_width_m",
                    "v_setpoint_mps",
                    "linear_cmd_scale",
                    "linear_cmd_max",
                    "angular_cmd_max",
                }
                and
                float(
                    parameter.value
                )
                <= 0.0
            ):

                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"{parameter.name} "
                        "must be > 0"
                    )
                )

        return SetParametersResult(
            successful=True
        )

    # ================================================================
    # ENABLE / STOP
    # ================================================================

    def is_enabled(self):

        return (
            self.pbool(
                "enable_cmd"
            )
            or
            self.runtime_enable
        )

    def enable_cb(
        self,
        message
    ):

        self.runtime_enable = bool(
            message.data
        )

        if not self.is_enabled():
            self.stop_burst()

    def estop_cb(
        self,
        message
    ):

        self.emergency_stop = bool(
            message.data
        )

        if self.emergency_stop:
            self.stop_burst()

    def signal_handler(
        self,
        _signal,
        _frame
    ):

        self.stop_burst()

        if rclpy.ok():
            rclpy.shutdown()

    def stop_burst(self):

        message = self.twist(
            0.0,
            0.0
        )

        count = max(
            3,
            self.pint(
                "stop_burst_count"
            )
        )

        dt = max(
            0.005,
            self.pfloat(
                "stop_burst_dt"
            )
        )

        for _ in range(count):

            self.cmd_pub.publish(
                message
            )

            time.sleep(
                dt
            )

    # ================================================================
    # CMD_VEL CONFLICT
    # ================================================================

    def conflict(self):

        if (
            not self.pbool(
                "check_cmd_vel_conflict"
            )
            or
            self.pbool(
                "allow_cmd_vel_conflict"
            )
        ):
            return False, []

        infos = (
            self.get_publishers_info_by_topic(
                self.cmd_vel_topic
            )
        )

        names = []

        for info in infos:

            if info.node_namespace == "/":

                names.append(
                    info.node_name
                )

            else:

                names.append(
                    f"{info.node_namespace}/"
                    f"{info.node_name}"
                )

        return (
            len(infos) > 1,
            names
        )

    # ================================================================
    # LANE STATE
    # ================================================================

    def lane_state_cb(
        self,
        message
    ):

        try:

            data = json.loads(
                message.data
            )

            if not isinstance(
                data,
                dict
            ):
                return

        except Exception:
            return

        self.last_lane_state_rx = (
            time.monotonic()
        )

        intent = self.first_text(
            data,
            [
                "active_intent",
                "committed_intent",
                "intent",
                "current_intent",
            ]
        )

        lane = self.first_text(
            data,
            [
                "active_lane_state",
                "committed_lane_state",
                "lane_state",
            ]
        )

        trajectory = self.first_text(
            data,
            [
                "trajectory_status",
                "trajectory_mode",
                "manager_mode",
                "commit_state",
            ]
        )

        planner = self.first_text(
            data,
            [
                "planner_status",
                "planner_state",
                "replan_reason",
                "status",
            ]
        )

        active = (
            f"{intent} {lane}"
        )

        if (
            "LANE_CHANGE" in active
            or
            "CHANGE_LANE" in active
        ):

            self.intent_hint = (
                "LANE_CHANGE"
            )

        elif "TURN_LEFT" in active:

            self.intent_hint = (
                "TURN_LEFT"
            )

        elif "TURN_RIGHT" in active:

            self.intent_hint = (
                "TURN_RIGHT"
            )

        elif (
            "FOLLOW_MAIN" in active
            or
            lane == "MAIN"
        ):

            self.intent_hint = (
                "FOLLOW_MAIN"
            )

        else:

            self.intent_hint = (
                "UNKNOWN"
            )

        joined = (
            f"{trajectory} {lane}"
        )

        if "RECOVERY" in joined:

            self.trajectory_hint = (
                "RECOVERY"
            )

        elif "HOLD" in joined:

            self.trajectory_hint = (
                "HOLD"
            )

        elif (
            "SOFT" in joined
            or
            "REPLAN" in joined
        ):

            self.trajectory_hint = (
                "SOFT_REPLAN"
            )

        elif (
            "COMMITTED" in joined
            or
            "ACTIVE" in joined
            or
            lane == "FOLLOW_MAIN"
        ):

            self.trajectory_hint = (
                "COMMITTED"
            )

        else:

            self.trajectory_hint = (
                "UNKNOWN"
            )

        # Planner status chỉ dùng debug.
        # Không trực tiếp thay đổi tốc độ khi trajectory COMMITTED.

        if "DROPOUT" in planner:

            self.planner_status_hint = (
                "DROPOUT"
            )

        elif "HOLD" in planner:

            self.planner_status_hint = (
                "HOLD"
            )

        elif "REPLAN" in planner:

            self.planner_status_hint = (
                "REPLAN"
            )

        elif (
            "BLOCKED_BY_MARKING"
            in planner
        ):

            self.planner_status_hint = (
                "BLOCKED_BY_MARKING"
            )

        else:

            self.planner_status_hint = (
                "UNKNOWN"
            )

    # ================================================================
    # EXTRACT CONTROL ERROR
    # ================================================================

    def extract_error(
        self,
        data
    ):

        e_lat = None

        for key in (
            "lateral_error_m",
            "e_lat_m",
            "e_y_m",
            "x_error_m",
        ):

            if key in data:

                e_lat = finite_float(
                    data.get(
                        key
                    )
                )

                break

        if e_lat is None:

            for key in (
                "epsilon_x_mm",
                "x_mm",
                "e_y_mm",
                "e_lat_mm",
            ):

                if key in data:

                    value = finite_float(
                        data.get(
                            key
                        )
                    )

                    if value is not None:

                        e_lat = (
                            value
                            /
                            1000.0
                        )

                        break

        if e_lat is None:
            e_lat = 0.0

        e_heading = None

        for key in (
            "heading_error_rad",
            "e_theta_rad",
            "theta_error_rad",
            "theta_rad",
        ):

            if key in data:

                e_heading = finite_float(
                    data.get(
                        key
                    )
                )

                break

        if e_heading is None:
            e_heading = 0.0

        e_lat = (
            self.pfloat(
                "epsilon_sign"
            )
            *
            e_lat
            +
            self.pfloat(
                "x_bias_m"
            )
        )

        e_heading = (
            self.pfloat(
                "theta_sign"
            )
            *
            e_heading
        )

        inverse_mm = finite_float(
            data.get(
                "curvature_inv_mm"
            )
        )

        if inverse_mm is not None:

            kappa = (
                inverse_mm
                *
                1000.0
            )

        else:

            kappa = finite_float(
                data.get(
                    "curvature_m_inv",
                    data.get(
                        "kappa",
                        data.get(
                            "curvature",
                            0.0
                        )
                    )
                ),
                0.0
            )

        kappa = clamp(
            float(kappa),
            -8.0,
            8.0
        )

        confidence = finite_float(
            data.get(
                "confidence",
                data.get(
                    "conf",
                    1.0
                )
            ),
            1.0
        )

        confidence = clamp(
            confidence,
            0.0,
            1.0
        )

        lane_state = str(
            data.get(
                "lane_state",
                ""
            )
        ).strip().upper()

        lookahead = finite_float(
            data.get(
                "lookahead_m",
                data.get(
                    "epsilon_y_m",
                    0.0
                )
            ),
            0.0
        )

        valid = (
            parse_bool(
                data.get(
                    "valid"
                ),
                True
            )
            and
            parse_bool(
                data.get(
                    "lane_valid"
                ),
                True
            )
        )

        if lane_state == "FOLLOW_MAIN":
            valid = True

        if lane_state in {
            "LOST",
            "INVALID",
            "NO_LANE",
            "NONE",
        }:

            return (
                False,
                "lane_state_invalid",
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                lookahead,
            )

        if abs(e_lat) > self.pfloat(
            "max_abs_x_m"
        ):

            return (
                False,
                "lateral_outlier",
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                lookahead,
            )

        if abs(e_heading) > self.pfloat(
            "max_abs_theta_rad"
        ):

            return (
                False,
                "heading_outlier",
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                lookahead,
            )

        if confidence < self.pfloat(
            "min_confidence"
        ):

            return (
                False,
                "low_confidence",
                e_lat,
                e_heading,
                kappa,
                confidence,
                lane_state,
                lookahead,
            )

        return (
            valid,
            "ok" if valid else "invalid",
            e_lat,
            e_heading,
            kappa,
            confidence,
            lane_state,
            lookahead,
        )

    # ================================================================
    # CONTROL ERROR CALLBACK
    # ================================================================

    def control_error_cb(
        self,
        message
    ):

        now = (
            time.monotonic()
        )

        try:

            data = json.loads(
                message.data
            )

            if not isinstance(
                data,
                dict
            ):
                return

        except Exception:
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
        ) = self.extract_error(
            data
        )

        self.last_error_rx = now

        self.raw_valid = valid
        self.raw_reason = reason

        self.raw_e_lat = e_lat
        self.raw_e_heading = e_heading
        self.raw_kappa = kappa
        self.raw_confidence = confidence
        self.raw_lane_state = lane_state
        self.raw_lookahead = lookahead

        # ============================================================
        # FPS ESTIMATION
        # ============================================================

        if (
            self.last_vision_rx
            is not None
        ):

            frame_dt = clamp(
                now
                -
                self.last_vision_rx,
                0.01,
                1.0
            )

            fps_raw = (
                1.0
                /
                frame_dt
            )

            fps_alpha = self.alpha(
                frame_dt,
                self.pfloat(
                    "fps_filter_tau_s"
                )
            )

            self.vision_fps = (
                (
                    1.0
                    -
                    fps_alpha
                )
                *
                self.vision_fps
                +
                fps_alpha
                *
                fps_raw
            )

        self.last_vision_rx = now

        if not valid:
            return

        self.last_valid_error_rx = now

        # ============================================================
        # MEDIAN WINDOW
        # ============================================================

        desired_window = max(
            1,
            self.pint(
                "median_window"
            )
        )

        if (
            self.lat_buffer.maxlen
            !=
            desired_window
        ):

            self.lat_buffer = deque(
                list(
                    self.lat_buffer
                )[
                    -desired_window:
                ],
                maxlen=desired_window
            )

            self.heading_buffer = deque(
                list(
                    self.heading_buffer
                )[
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

        lat_target = (
            statistics.median(
                self.lat_buffer
            )
        )

        heading_target = (
            statistics.median(
                self.heading_buffer
            )
        )

        # ============================================================
        # FRAME DT
        # ============================================================

        if (
            self._last_valid_frame_time
            is None
        ):

            dt = (
                1.0
                /
                max(
                    self.vision_fps,
                    5.0
                )
            )

        else:

            dt = clamp(
                now
                -
                self._last_valid_frame_time,
                0.01,
                0.5
            )

        self._last_valid_frame_time = now

        # ============================================================
        # PER-FRAME JUMP LIMITER
        #
        # Không dùng LPF quá nặng như V2.
        # Chặn riêng jump segmentation.
        # ============================================================

        lat_target = clamp(
            lat_target,

            self.e_lat_f
            -
            self.pfloat(
                "max_lat_step_per_frame_m"
            ),

            self.e_lat_f
            +
            self.pfloat(
                "max_lat_step_per_frame_m"
            )
        )

        heading_target = clamp(
            heading_target,

            self.e_heading_f
            -
            self.pfloat(
                "max_theta_step_per_frame_rad"
            ),

            self.e_heading_f
            +
            self.pfloat(
                "max_theta_step_per_frame_rad"
            )
        )

        # ============================================================
        # FAST LPF
        # ============================================================

        alpha_error = self.alpha(
            dt,
            self.pfloat(
                "error_filter_tau_s"
            )
        )

        self.e_lat_f = (
            (
                1.0
                -
                alpha_error
            )
            *
            self.e_lat_f
            +
            alpha_error
            *
            lat_target
        )

        self.e_heading_f = (
            (
                1.0
                -
                alpha_error
            )
            *
            self.e_heading_f
            +
            alpha_error
            *
            heading_target
        )

        # ============================================================
        # DERIVATIVE
        # ============================================================

        e_for_derivative = (
            0.0
            if
            abs(
                self.e_lat_f
            )
            <
            self.pfloat(
                "x_deadband_m"
            )
            else
            self.e_lat_f
        )

        theta_for_derivative = (
            0.0
            if
            abs(
                self.e_heading_f
            )
            <
            self.pfloat(
                "theta_deadband_rad"
            )
            else
            self.e_heading_f
        )

        de_raw = (
            e_for_derivative
            -
            self.prev_derivative_lat
        ) / dt

        dtheta_raw = (
            theta_for_derivative
            -
            self.prev_derivative_heading
        ) / dt

        self.prev_derivative_lat = (
            e_for_derivative
        )

        self.prev_derivative_heading = (
            theta_for_derivative
        )

        de_raw = clamp(
            de_raw,
            -self.pfloat(
                "derivative_clip_lat_mps"
            ),
            self.pfloat(
                "derivative_clip_lat_mps"
            )
        )

        dtheta_raw = clamp(
            dtheta_raw,
            -self.pfloat(
                "derivative_clip_theta_rps"
            ),
            self.pfloat(
                "derivative_clip_theta_rps"
            )
        )

        derivative_alpha = self.alpha(
            dt,
            self.pfloat(
                "derivative_filter_tau_s"
            )
        )

        self.de_lat_f = (
            (
                1.0
                -
                derivative_alpha
            )
            *
            self.de_lat_f
            +
            derivative_alpha
            *
            de_raw
        )

        self.de_heading_f = (
            (
                1.0
                -
                derivative_alpha
            )
            *
            self.de_heading_f
            +
            derivative_alpha
            *
            dtheta_raw
        )

    # ================================================================
    # ODOM CALLBACK
    # ================================================================

    def odom_cb(
        self,
        message
    ):

        now = (
            time.monotonic()
        )

        self.odom_x = float(
            message.pose.pose.position.x
        )

        self.odom_y = float(
            message.pose.pose.position.y
        )

        self.odom_v_raw = float(
            message.twist.twist.linear.x
        )

        self.odom_omega_raw = float(
            message.twist.twist.angular.z
        )

        self.last_odom_rx = now

        if (
            self.last_odom_filter_time
            is None
        ):

            dt = 0.02

        else:

            dt = clamp(
                now
                -
                self.last_odom_filter_time,
                0.001,
                0.2
            )

        self.last_odom_filter_time = (
            now
        )

        if (
            not
            self.odom_filter_initialized
        ):

            self.odom_v_f = (
                self.odom_v_raw
            )

            self.odom_omega_f = (
                self.odom_omega_raw
            )

            self.odom_filter_initialized = (
                True
            )

        else:

            alpha_odom = self.alpha(
                dt,
                self.pfloat(
                    "odom_filter_tau_s"
                )
            )

            self.odom_v_f = (
                (
                    1.0
                    -
                    alpha_odom
                )
                *
                self.odom_v_f
                +
                alpha_odom
                *
                self.odom_v_raw
            )

            self.odom_omega_f = (
                (
                    1.0
                    -
                    alpha_odom
                )
                *
                self.odom_omega_f
                +
                alpha_odom
                *
                self.odom_omega_raw
            )

    # ================================================================
    # LIDAR
    # ================================================================

    def scan_cb(
        self,
        message
    ):

        half_angle = math.radians(
            self.pfloat(
                "front_angle_deg"
            )
        )

        values = []

        angle = (
            message.angle_min
        )

        for distance in message.ranges:

            if (
                math.isfinite(
                    distance
                )
                and
                message.range_min
                <= distance
                <= message.range_max
                and
                abs(angle)
                <= half_angle
            ):

                values.append(
                    float(
                        distance
                    )
                )

            angle += (
                message.angle_increment
            )

        self.front_min = (
            min(
                values
            )
            if values
            else
            math.inf
        )

    # ================================================================
    # ACTIVE PROFILE
    #
    # V3 FIX:
    #
    # planner_status = HOLD
    # KHÔNG làm giảm speed nữa.
    #
    # trajectory_hint = HOLD
    # mới thật sự hold.
    # ================================================================

    def active_profile(self):

        gain = 1.0

        speed_max = self.pfloat(
            "v_setpoint_mps"
        )

        profile = "follow_main"

        if (
            self.trajectory_hint
            ==
            "HOLD"
        ):

            gain = 0.0

            speed_max = self.pfloat(
                "explicit_hold_speed_mps"
            )

            profile = "explicit_hold"

        elif (
            self.trajectory_hint
            ==
            "RECOVERY"
        ):

            gain = self.pfloat(
                "recovery_gain_multiplier"
            )

            speed_max = self.pfloat(
                "recovery_speed_max_mps"
            )

            profile = "recovery"

        elif (
            self.trajectory_hint
            ==
            "SOFT_REPLAN"
        ):

            gain = self.pfloat(
                "soft_replan_gain_multiplier"
            )

            speed_max = self.pfloat(
                "soft_replan_speed_max_mps"
            )

            profile = "soft_replan"

        elif (
            self.intent_hint
            ==
            "LANE_CHANGE"
        ):

            gain = self.pfloat(
                "lane_change_gain_multiplier"
            )

            speed_max = self.pfloat(
                "lane_change_speed_max_mps"
            )

            profile = "lane_change"

        return (
            gain,
            speed_max,
            profile
        )

    # ================================================================
    # OUTER TRUE PD
    # ================================================================

    def compute_outer(self, dt):

        # ------------------------------------------------------------
        # Control errors
        # ------------------------------------------------------------

        e_lat = (
            0.0
            if abs(self.e_lat_f)
            <
            self.pfloat("x_deadband_m")
            else
            self.e_lat_f
        )

        e_heading = (
            0.0
            if abs(self.e_heading_f)
            <
            self.pfloat("theta_deadband_rad")
            else
            self.e_heading_f
        )

        (
            gain_multiplier,
            profile_speed_max,
            profile
        ) = self.active_profile()

        # ============================================================
        # FILTER TRAJECTORY CURVATURE
        #
        # Curvature gives preview of the committed trajectory.
        # It is intentionally clipped and filtered before use.
        # ============================================================

        kappa_target = clamp(
            self.raw_kappa,
            -self.pfloat(
                "max_abs_curvature_control"
            ),
            self.pfloat(
                "max_abs_curvature_control"
            )
        )

        kappa_alpha = self.alpha(
            dt,
            self.pfloat(
                "curvature_filter_tau_s"
            )
        )

        self.kappa_f = (
            (1.0 - kappa_alpha)
            * self.kappa_f
            +
            kappa_alpha
            * kappa_target
        )

        # ============================================================
        # CURVE SEVERITY
        #
        # Curvature receives more weight than before.
        #
        # Reason:
        # lateral / heading error are reactive.
        # curvature is trajectory preview.
        # ============================================================

        lateral_ratio = (
            abs(e_lat)
            /
            max(
                self.pfloat(
                    "curve_lat_scale_m"
                ),
                0.001
            )
        )

        heading_ratio = (
            abs(e_heading)
            /
            max(
                self.pfloat(
                    "curve_heading_scale_rad"
                ),
                0.001
            )
        )

        curvature_ratio = (
            abs(self.kappa_f)
            /
            max(
                self.pfloat(
                    "curve_kappa_scale"
                ),
                0.001
            )
        )

        severity_raw = (
            self.pfloat(
                "curve_lat_weight"
            )
            * lateral_ratio
            +
            self.pfloat(
                "curve_heading_weight"
            )
            * heading_ratio
            +
            self.pfloat(
                "curve_kappa_weight"
            )
            * curvature_ratio
        )

        severity_raw = max(
            0.0,
            severity_raw
            -
            self.pfloat(
                "curve_severity_deadband"
            )
        )

        severity_raw = clamp(
            severity_raw,
            0.0,
            2.0
        )

        severity_alpha = self.alpha(
            dt,
            self.pfloat(
                "severity_filter_tau_s"
            )
        )

        if severity_raw > self.severity_f:
            # Tăng đột ngột -> phanh ngay lập tức
            self.severity_f = severity_raw
        else:
            # Giảm từ từ -> giữ tốc độ thấp cho đến khi ổn định
            self.severity_f = (
                (1.0 - severity_alpha)
                * self.severity_f
                +
                severity_alpha
                * severity_raw
            )

        s = self.severity_f

        # ============================================================
        # SPEED MAP
        #
        # straight       0.30
        # mild           0.25
        # medium         0.18
        # sharp          0.12
        #
        # Continuous interpolation prevents velocity steps.
        # ============================================================

        s1 = self.pfloat(
            "severity_mild_threshold"
        )

        s2 = self.pfloat(
            "severity_medium_threshold"
        )

        s3 = self.pfloat(
            "severity_sharp_threshold"
        )

        v0 = min(
            self.pfloat(
                "v_setpoint_mps"
            ),
            profile_speed_max
        )

        v1 = min(
            self.pfloat(
                "v_mild_mps"
            ),
            profile_speed_max
        )

        v2 = min(
            self.pfloat(
                "v_medium_mps"
            ),
            profile_speed_max
        )

        v3 = min(
            self.pfloat(
                "v_sharp_mps"
            ),
            profile_speed_max
        )

        def blend(a, b, ratio):

            ratio = clamp(
                ratio,
                0.0,
                1.0
            )

            return (
                a
                +
                (b - a)
                * ratio
            )

        if profile_speed_max <= 0.0:

            v_target = 0.0
            curve_zone = "hold"

        elif s <= s1:

            v_target = blend(
                v0,
                v1,
                s
                /
                max(
                    s1,
                    0.001
                )
            )

            curve_zone = "straight"

        elif s <= s2:

            v_target = blend(
                v1,
                v2,
                (
                    s - s1
                )
                /
                max(
                    s2 - s1,
                    0.001
                )
            )

            curve_zone = "mild"

        elif s <= s3:

            v_target = blend(
                v2,
                v3,
                (
                    s - s2
                )
                /
                max(
                    s3 - s2,
                    0.001
                )
            )

            curve_zone = "medium"

        else:

            v_target = v3
            curve_zone = "sharp"

        # ------------------------------------------------------------
        # Brake faster into a turn.
        # Accelerate gently when returning to straight.
        # ------------------------------------------------------------

        if v_target >= self.v_ref:

            v_rate = self.pfloat(
                "v_ref_rate_up_mps2"
            )

        else:

            v_rate = self.pfloat(
                "v_ref_rate_down_mps2"
            )

        self.v_ref = approach(
            self.v_ref,
            v_target,
            v_rate
            * dt
        )

        # ============================================================
        # TRUE OUTER PD FEEDBACK
        # ============================================================

        p_lat = (
            self.pfloat(
                "outer_kp_lat"
            )
            * e_lat
        )

        d_lat = (
            self.pfloat(
                "outer_kd_lat"
            )
            * self.de_lat_f
        )

        p_heading = (
            self.pfloat(
                "outer_kp_heading"
            )
            * e_heading
        )

        d_heading = (
            self.pfloat(
                "outer_kd_heading"
            )
            * self.de_heading_f
        )

        omega_feedback = (
            self.pfloat(
                "outer_control_sign"
            )
            *
            gain_multiplier
            *
            (
                p_lat
                +
                d_lat
                +
                p_heading
                +
                d_heading
            )
        )

        # ============================================================
        # CURVATURE FEED-FORWARD
        #
        # omega_ff = k_ff * v * curvature
        #
        # No Pure Pursuit equation.
        # ============================================================

        omega_ff = (
            self.pfloat(
                "curvature_ff_sign"
            )
            *
            self.pfloat(
                "curvature_ff_gain"
            )
            *
            self.v_ref
            *
            self.kappa_f
        )

        omega_ff = clamp(
            omega_ff,
            -self.pfloat(
                "curvature_ff_max"
            ),
            self.pfloat(
                "curvature_ff_max"
            )
        )

        omega_target = (
            omega_ff
            +
            omega_feedback
        )

        # ============================================================
        # ADAPTIVE CURVE STEERING
        #
        # Do not increase steering everywhere.
        #
        # Straight is deliberately softer.
        # Steering authority grows progressively with curve severity.
        # ============================================================

        if curve_zone == "straight":

            curve_steer_gain = self.pfloat(
                "curve_steer_gain_straight"
            )

        elif curve_zone == "mild":

            curve_steer_gain = self.pfloat(
                "curve_steer_gain_mild"
            )

        elif curve_zone == "medium":

            curve_steer_gain = self.pfloat(
                "curve_steer_gain_medium"
            )

        elif curve_zone == "sharp":

            curve_steer_gain = self.pfloat(
                "curve_steer_gain_sharp"
            )

        else:

            curve_steer_gain = 1.0

        omega_target *= (
            curve_steer_gain
        )

        if self.pbool(
            "invert_angular"
        ):

            omega_target = (
                -omega_target
            )

        # ============================================================
        # ANGULAR LIMIT BY CURVE ZONE
        #
        # At 0.30 m/s, steering remains limited.
        # When the robot has slowed, more yaw authority is allowed.
        # ============================================================

        if curve_zone == "straight":

            omega_limit = self.pfloat(
                "omega_straight_max"
            )

        elif curve_zone == "mild":

            omega_limit = self.pfloat(
                "omega_mild_max"
            )

        elif curve_zone == "medium":

            omega_limit = self.pfloat(
                "omega_medium_max"
            )

        elif curve_zone == "sharp":

            omega_limit = self.pfloat(
                "omega_sharp_max"
            )

        else:

            omega_limit = 0.0

        omega_target = clamp(
            omega_target,
            -omega_limit,
            omega_limit
        )

        if (
            abs(omega_target)
            <
            self.pfloat(
                "omega_ref_deadband"
            )
        ):

            omega_target = 0.0

        # ============================================================
        # SMOOTH OMEGA REFERENCE
        # ============================================================

        crossing_zero = (
            self.omega_ref
            * omega_target
            < 0.0
            and
            abs(self.omega_ref)
            > 0.001
            and
            abs(omega_target)
            > 0.001
        )

        if crossing_zero:

            omega_rate = self.pfloat(
                "omega_zero_cross_rate"
            )

        elif abs(omega_target) >= abs(
            self.omega_ref
        ):

            omega_rate = self.pfloat(
                "omega_ref_rate_up"
            )

        else:

            omega_rate = self.pfloat(
                "omega_ref_rate_down"
            )

        self.omega_ref = approach(
            self.omega_ref,
            omega_target,
            omega_rate
            * dt
        )

        self.v_target = v_target
        self.omega_target = omega_target

        return {
            "profile":
                profile,

            "curve_zone":
                curve_zone,

            "curve_severity":
                self.severity_f,

            "curve_steer_gain":
                curve_steer_gain,

            "omega_limit":
                omega_limit,

            "gain_multiplier":
                gain_multiplier,

            "speed_multiplier":
                (
                    v_target
                    /
                    max(
                        self.pfloat(
                            "v_setpoint_mps"
                        ),
                        0.001
                    )
                ),

            "e_lat_used_m":
                e_lat,

            "e_heading_used_rad":
                e_heading,

            "p_lat":
                p_lat,

            "d_lat":
                d_lat,

            "p_heading":
                p_heading,

            "d_heading":
                d_heading,

            "omega_ff":
                omega_ff,

            "omega_fb":
                omega_feedback,

            # Keep existing logger compatibility.
            "omega_pd":
                omega_target,

            "omega_target":
                omega_target,

            "v_target":
                v_target,

            "speed_factor":
                (
                    v_target
                    /
                    max(
                        self.pfloat(
                            "v_setpoint_mps"
                        ),
                        0.001
                    )
                ),
        }

    def compute_inner(
        self,
        dt
    ):

        track = max(
            self.pfloat(
                "track_width_m"
            ),
            0.05
        )

        # ============================================================
        # WHEEL REFERENCES
        # ============================================================

        self.v_left_ref = (
            self.v_ref
            -
            0.5
            *
            track
            *
            self.omega_ref
        )

        self.v_right_ref = (
            self.v_ref
            +
            0.5
            *
            track
            *
            self.omega_ref
        )

        ref_limit = self.pfloat(
            "wheel_ref_max_mps"
        )

        self.v_left_ref = clamp(
            self.v_left_ref,
            -ref_limit,
            ref_limit
        )

        self.v_right_ref = clamp(
            self.v_right_ref,
            -ref_limit,
            ref_limit
        )

        # ============================================================
        # MEASURED WHEEL GROUP SPEED
        #
        # Dùng odom đã lọc.
        # ============================================================

        self.v_left_meas = (
            self.odom_v_f
            -
            0.5
            *
            track
            *
            self.odom_omega_f
        )

        self.v_right_meas = (
            self.odom_v_f
            +
            0.5
            *
            track
            *
            self.odom_omega_f
        )

        # ============================================================
        # ERROR
        # ============================================================

        self.left_error = (
            self.v_left_ref
            -
            self.v_left_meas
        )

        self.right_error = (
            self.v_right_ref
            -
            self.v_right_meas
        )

        deadband = self.pfloat(
            "inner_error_deadband_mps"
        )

        if (
            abs(
                self.left_error
            )
            <
            deadband
        ):

            self.left_error = 0.0

        if (
            abs(
                self.right_error
            )
            <
            deadband
        ):

            self.right_error = 0.0

        # ============================================================
        # INNER DERIVATIVE
        # ============================================================

        d_left_raw = (
            self.left_error
            -
            self.prev_left_error
        ) / max(
            dt,
            0.001
        )

        d_right_raw = (
            self.right_error
            -
            self.prev_right_error
        ) / max(
            dt,
            0.001
        )

        self.prev_left_error = (
            self.left_error
        )

        self.prev_right_error = (
            self.right_error
        )

        derivative_alpha = self.alpha(
            dt,
            self.pfloat(
                "inner_derivative_tau_s"
            )
        )

        self.d_left_f = (
            (
                1.0
                -
                derivative_alpha
            )
            *
            self.d_left_f

            +

            derivative_alpha
            *
            d_left_raw
        )

        self.d_right_f = (
            (
                1.0
                -
                derivative_alpha
            )
            *
            self.d_right_f

            +

            derivative_alpha
            *
            d_right_raw
        )

        # ============================================================
        # INNER PD CORRECTIONS
        # ============================================================

        self.left_correction = (
            self.pfloat(
                "inner_kp_left"
            )
            *
            self.left_error

            +

            self.pfloat(
                "inner_kd_left"
            )
            *
            self.d_left_f
        )

        self.right_correction = (
            self.pfloat(
                "inner_kp_right"
            )
            *
            self.right_error

            +

            self.pfloat(
                "inner_kd_right"
            )
            *
            self.d_right_f
        )

        correction_limit = self.pfloat(
            "inner_correction_max_mps"
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

        # ============================================================
        # DO NOT REVERSE INNER WHEEL
        # ============================================================

        if (
            not
            self.pbool(
                "allow_reverse_wheel"
            )
            and
            self.v_ref
            >
            0.0
        ):

            wheel_min = self.pfloat(
                "minimum_forward_wheel_mps"
            )

            left_target = max(
                left_target,
                wheel_min
            )

            right_target = max(
                right_target,
                wheel_min
            )

        # ============================================================
        # WHEEL CMD LIMIT
        # ============================================================

        command_limit = self.pfloat(
            "wheel_cmd_max_mps"
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

        # ============================================================
        # WHEEL CMD SLEW
        # ============================================================

        left_rate = (
            self.pfloat(
                "wheel_cmd_rate_up_mps2"
            )
            if
            abs(
                left_target
            )
            >=
            abs(
                self.v_left_cmd
            )
            else
            self.pfloat(
                "wheel_cmd_rate_down_mps2"
            )
        )

        right_rate = (
            self.pfloat(
                "wheel_cmd_rate_up_mps2"
            )
            if
            abs(
                right_target
            )
            >=
            abs(
                self.v_right_cmd
            )
            else
            self.pfloat(
                "wheel_cmd_rate_down_mps2"
            )
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

    # ================================================================
    # PUBLISH STATE
    # ================================================================

    def publish_state(
        self,
        now,
        error_age,
        odom_age,
        cmd_published,
        conflict,
        publishers
    ):

        debug = (
            self.outer_debug
        )

        message = String()

        message.data = json.dumps(
            {
                "node":
                    "cascade_controller_v4",

                "version":
                    self.VERSION,

                "mode":
                    self.control_mode,

                "outer_mode":
                    debug.get(
                        "profile",
                        ""
                    ),

                "mix_mode":
                    "inner_wheel_pd",

                "enabled":
                    self.is_enabled(),

                "cmd_published":
                    cmd_published,

                "cmd_vel_conflict":
                    conflict,

                "cmd_vel_publishers":
                    publishers,

                "emergency_stop":
                    self.emergency_stop,

                "intent_hint":
                    self.intent_hint,

                "trajectory_hint":
                    self.trajectory_hint,

                "planner_status_hint":
                    self.planner_status_hint,

                "lane_state":
                    self.raw_lane_state,

                "raw_valid":
                    self.raw_valid,

                "raw_valid_reason":
                    self.raw_reason,

                "confidence":
                    self.raw_confidence,

                "fps_est":
                    self.vision_fps,

                "error_age_s":
                    error_age,

                "odom_age_s":
                    odom_age,

                "epsilon_x_mm":
                    self.raw_e_lat
                    *
                    1000.0,

                "theta_rad":
                    self.raw_e_heading,

                "kappa_m":
                    self.raw_kappa,

                "lookahead_m":
                    self.raw_lookahead,

                "e_f_m":
                    self.e_lat_f,

                "e_f_mm":
                    self.e_lat_f
                    *
                    1000.0,

                "theta_f_rad":
                    self.e_heading_f,

                "e_used_m":
                    (
                        0.0
                        if
                        abs(
                            self.e_lat_f
                        )
                        <
                        self.pfloat(
                            "x_deadband_m"
                        )
                        else
                        self.e_lat_f
                    ),

                "e_used_mm":
                    (
                        0.0
                        if
                        abs(
                            self.e_lat_f
                        )
                        <
                        self.pfloat(
                            "x_deadband_m"
                        )
                        else
                        self.e_lat_f
                        *
                        1000.0
                    ),

                "theta_used_rad":
                    (
                        0.0
                        if
                        abs(
                            self.e_heading_f
                        )
                        <
                        self.pfloat(
                            "theta_deadband_rad"
                        )
                        else
                        self.e_heading_f
                    ),

                "de_f":
                    self.de_lat_f,

                "dtheta_f":
                    self.de_heading_f,

                "outer_kp_lat":
                    self.pfloat(
                        "outer_kp_lat"
                    ),

                "outer_kd_lat":
                    self.pfloat(
                        "outer_kd_lat"
                    ),

                "outer_kp_heading":
                    self.pfloat(
                        "outer_kp_heading"
                    ),

                "outer_kd_heading":
                    self.pfloat(
                        "outer_kd_heading"
                    ),

                "p_lat":
                    debug.get(
                        "p_lat",
                        0.0
                    ),

                "d_lat":
                    debug.get(
                        "d_lat",
                        0.0
                    ),

                "p_heading":
                    debug.get(
                        "p_heading",
                        0.0
                    ),

                "d_heading":
                    debug.get(
                        "d_heading",
                        0.0
                    ),

                "omega_raw":
                    debug.get(
                        "omega_pd",
                        0.0
                    ),

                "omega_des":
                    self.omega_target,

                "omega_target":
                    self.omega_target,

                "v_des":
                    self.v_target,

                "v_target":
                    self.v_target,

                "curve_severity":
                    self.severity_f,

                "curve_zone":
                    debug.get(
                        "curve_zone",
                        ""
                    ),

                "curve_steer_gain":
                    debug.get(
                        "curve_steer_gain",
                        1.0
                    ),

                "outer_gain_multiplier":
                    debug.get(
                        "gain_multiplier",
                        1.0
                    ),

                "omega_ff":
                    debug.get(
                        "omega_ff",
                        0.0
                    ),

                "omega_fb":
                    debug.get(
                        "omega_fb",
                        0.0
                    ),

                "omega_limit":
                    debug.get(
                        "omega_limit",
                        0.0
                    ),

                "speed_factor":
                    (
                        self.v_target
                        /
                        max(
                            self.pfloat(
                                "v_setpoint_mps"
                            ),
                            0.000001
                        )
                    ),

                "v_ref":
                    self.v_ref,

                "omega_ref":
                    self.omega_ref,

                "v_left_ref":
                    self.v_left_ref,

                "v_right_ref":
                    self.v_right_ref,

                "v_left_des":
                    self.v_left_ref,

                "v_right_des":
                    self.v_right_ref,

                "v_left_measured":
                    self.v_left_meas,

                "v_right_measured":
                    self.v_right_meas,

                "v_left_odom":
                    self.v_left_meas,

                "v_right_odom":
                    self.v_right_meas,

                "left_wheel_error":
                    self.left_error,

                "right_wheel_error":
                    self.right_error,

                "d_left_wheel_error":
                    self.d_left_f,

                "d_right_wheel_error":
                    self.d_right_f,

                "left_pd_correction":
                    self.left_correction,

                "right_pd_correction":
                    self.right_correction,

                "v_left_cmd":
                    self.v_left_cmd,

                "v_right_cmd":
                    self.v_right_cmd,

                "v_cmd":
                    self.v_cmd,

                "omega_cmd":
                    self.omega_cmd,

                "odom_v":
                    self.odom_v_raw,

                "odom_omega":
                    self.odom_omega_raw,

                "odom_v_filtered":
                    self.odom_v_f,

                "odom_omega_filtered":
                    self.odom_omega_f,

                "odom_x":
                    self.odom_x,

                "odom_y":
                    self.odom_y,

                "front_min_m":
                    (
                        self.front_min
                        if
                        math.isfinite(
                            self.front_min
                        )
                        else
                        None
                    ),

                "lidar_mode":
                    (
                        "lidar_disabled"
                        if
                        not
                        self.pbool(
                            "enable_lidar_safety"
                        )
                        else
                        "lidar_enabled"
                    ),

                "lidar_ratio":
                    1.0,

                "lidar_stop":
                    False,

                "time_monotonic":
                    now,
            },
            ensure_ascii=False
        )

        self.state_pub.publish(
            message
        )

    # ================================================================
    # MAIN CONTROL LOOP
    # ================================================================

    def control_loop(self):

        now = (
            time.monotonic()
        )

        dt = clamp(
            now
            -
            self.previous_loop_time,
            0.001,
            0.10
        )

        self.previous_loop_time = (
            now
        )

        error_age = (
            now
            -
            self.last_valid_error_rx
            if
            self.last_valid_error_rx
            >
            0.0
            else
            999.0
        )

        odom_age = (
            now
            -
            self.last_odom_rx
            if
            self.last_odom_rx
            >
            0.0
            else
            999.0
        )

        fresh_error = (
            self.raw_valid
            and
            error_age
            <=
            self.pfloat(
                "error_timeout_s"
            )
        )

        fresh_odom = (
            odom_age
            <=
            self.pfloat(
                "odom_timeout_s"
            )
        )

        self.outer_debug = {}

        # ============================================================
        # MODES
        # ============================================================

        if self.emergency_stop:

            self.control_mode = (
                "emergency_stop"
            )

            self.v_target = 0.0

            self.v_ref = approach(
                self.v_ref,
                0.0,
                self.pfloat(
                    "v_ref_rate_down_mps2"
                )
                *
                dt
            )

            self.omega_target = 0.0

            self.omega_ref = approach(
                self.omega_ref,
                0.0,
                self.pfloat(
                    "omega_ref_rate_down"
                )
                *
                dt
            )

        elif not fresh_odom:

            self.control_mode = (
                "odom_timeout"
            )

            self.v_target = 0.0

            self.v_ref = approach(
                self.v_ref,
                0.0,
                self.pfloat(
                    "v_ref_rate_down_mps2"
                )
                *
                dt
            )

            self.omega_target = 0.0

            self.omega_ref = approach(
                self.omega_ref,
                0.0,
                self.pfloat(
                    "omega_ref_rate_down"
                )
                *
                dt
            )

        elif not fresh_error:

            self.control_mode = (
                "control_error_timeout"
            )

            self.v_target = 0.0

            self.v_ref = approach(
                self.v_ref,
                0.0,
                self.pfloat(
                    "v_ref_rate_down_mps2"
                )
                *
                dt
            )

            self.omega_target = 0.0

            self.omega_ref = approach(
                self.omega_ref,
                0.0,
                self.pfloat(
                    "omega_ref_rate_down"
                )
                *
                dt
            )

        else:

            self.control_mode = (
                "cascade_pd_tracking"
            )

            self.outer_debug = (
                self.compute_outer(
                    dt
                )
            )

        # ============================================================
        # INNER PD
        # ============================================================

        self.compute_inner(
            dt
        )

        # ============================================================
        # WHEEL -> TWIST
        # ============================================================

        track = max(
            self.pfloat(
                "track_width_m"
            ),
            0.05
        )

        self.v_cmd = (
            0.5
            *
            (
                self.v_left_cmd
                +
                self.v_right_cmd
            )
        )

        self.omega_cmd = (
            self.v_right_cmd
            -
            self.v_left_cmd
        ) / track

        # ============================================================
        # OUTPUT SCALE
        # ============================================================

        self.v_cmd = (
            self.v_cmd
            /
            max(
                self.pfloat(
                    "linear_cmd_scale"
                ),
                0.000001
            )
        )

        self.omega_cmd = (
            self.omega_cmd
            *
            self.pfloat(
                "angular_cmd_gain"
            )
        )

        # ============================================================
        # HARD LIMIT
        # ============================================================

        self.v_cmd = clamp(
            self.v_cmd,
            -self.pfloat(
                "linear_cmd_max"
            ),
            self.pfloat(
                "linear_cmd_max"
            )
        )

        self.omega_cmd = clamp(
            self.omega_cmd,
            -self.pfloat(
                "angular_cmd_max"
            ),
            self.pfloat(
                "angular_cmd_max"
            )
        )

        force_stop = (
            self.control_mode
            in {
                "emergency_stop",
                "odom_timeout",
                "control_error_timeout",
            }
        )

        # ============================================================
        # OPTIONAL LIDAR
        # ============================================================

        if (
            self.pbool(
                "enable_lidar_safety"
            )
            and
            math.isfinite(
                self.front_min
            )
        ):

            if (
                self.front_min
                <=
                self.pfloat(
                    "lidar_stop_distance_m"
                )
            ):

                force_stop = True

            elif (
                self.front_min
                <
                self.pfloat(
                    "lidar_slow_distance_m"
                )
            ):

                lidar_ratio = clamp(
                    (
                        self.front_min
                        -
                        self.pfloat(
                            "lidar_stop_distance_m"
                        )
                    )
                    /
                    max(
                        self.pfloat(
                            "lidar_slow_distance_m"
                        )
                        -
                        self.pfloat(
                            "lidar_stop_distance_m"
                        ),
                        0.000001
                    ),
                    0.25,
                    1.0
                )

                self.v_cmd *= (
                    lidar_ratio
                )

                self.omega_cmd *= (
                    lidar_ratio
                )

        if force_stop:

            self.v_cmd = 0.0
            self.omega_cmd = 0.0

            self.v_left_cmd = 0.0
            self.v_right_cmd = 0.0

        # ============================================================
        # PUBLISH REFERENCE
        # ============================================================

        self.ref_pub.publish(
            self.twist(
                self.v_ref,
                self.omega_ref
            )
        )

        # ============================================================
        # CMD CONFLICT
        # ============================================================

        (
            conflict,
            publishers
        ) = self.conflict()

        cmd_published = False

        if (
            self.is_enabled()
            and
            not conflict
        ):

            self.cmd_pub.publish(
                self.twist(
                    self.v_cmd,
                    self.omega_cmd
                )
            )

            cmd_published = True

        elif (
            self.is_enabled()
            and
            conflict
        ):

            self.cmd_pub.publish(
                self.twist(
                    0.0,
                    0.0
                )
            )

        # ============================================================
        # DEBUG STATE
        # ============================================================

        self.publish_state(
            now,
            error_age,
            odom_age,
            cmd_published,
            conflict,
            publishers
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = CascadeControllerV4()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_burst()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()
