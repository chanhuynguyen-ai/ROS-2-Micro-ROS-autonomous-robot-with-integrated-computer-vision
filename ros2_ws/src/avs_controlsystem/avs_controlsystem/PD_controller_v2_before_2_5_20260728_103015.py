#!/usr/bin/env python3

import json
import math
import signal
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


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

    except Exception:
        pass

    return default


def parse_bool(value, default=True):

    if value is None:
        return default

    if isinstance(value, bool):
        return value

    if isinstance(value, str):

        return value.strip().lower() not in {
            "",
            "0",
            "false",
            "no",
            "none",
            "invalid",
            "lost",
        }

    return bool(value)


class PDControllerV2(Node):

    VERSION = "PD_controller_v2_lane_stable_2_4"

    def __init__(self):

        super().__init__(
            "PD_controller_v2"
        )

        # ============================================================
        # ALL PARAMETERS
        #
        # Khai báo tập trung ở MỘT nơi để tránh trùng parameter.
        # ============================================================

        parameters = {

            # --------------------------------------------------------
            # Topics
            # --------------------------------------------------------

            "control_error_topic":
                "/avs/control_error",

            "lane_state_topic":
                "/avs/lane_state",

            "cmd_vel_topic":
                '/cmd_vel',

            "debug_topic":
                "/avs/PD_controller_v2_debug",

            # --------------------------------------------------------
            # Runtime / safety
            # --------------------------------------------------------

            "enable_motion":
                True,

            "control_rate_hz":
                50.0,

            "error_timeout_s":
                0.70,

            "blind_hold_s":
                0.25,

            "check_cmd_vel_conflict":
                True,

            "allow_cmd_vel_conflict":
                False,

            "publish_zero_on_conflict":
                True,

            "stop_burst_count":
                20,

            "stop_burst_dt":
                0.015,

            # --------------------------------------------------------
            # Sign convention
            #
            # epsilon_x > 0:
            # target ở bên phải.
            #
            # ROS angular.z < 0:
            # quay phải.
            #
            # Công thức phía dưới đã có dấu trừ.
            # --------------------------------------------------------

            "epsilon_sign":
                1.0,

            "theta_sign":
                1.0,

            "invert_angular":
                False,

            "x_bias_m":
                0.0,

            # --------------------------------------------------------
            # Input validation
            # --------------------------------------------------------

            "max_abs_x_m":
                0.45,

            "max_abs_theta_rad":
                1.20,

            "min_confidence":
                0.15,

            # --------------------------------------------------------
            # Vision filter
            #
            # Nhanh hơn V1.
            # --------------------------------------------------------

            "median_window":
                3,

            "error_filter_tau_s":
                0.18,

            "derivative_filter_tau_s":
                0.32,

            "derivative_clip_x_mps":
                0.25,

            "derivative_clip_theta_rps":
                0.9,

            "x_deadband_m":
                0.01,

            "theta_deadband_rad":
                0.018,

            # --------------------------------------------------------
            # FPS estimator
            # --------------------------------------------------------

            "fps_init":
                15.0,

            "fps_min":
                3.0,

            "fps_max":
                30.0,

            "fps_filter_tau_s":
                0.40,

            "target_distance_per_frame_m":
                0.034,

            # --------------------------------------------------------
            # Linear speed
            #
            # V1 ~0.10.
            #
            # V2 cho thẳng 0.22-0.24 m/s.
            # --------------------------------------------------------

            "v_max":
                0.18,

            "v_center":
                0.17,

            "v_near":
                0.155,

            "v_mid":
                0.135,

            "v_curve":
                0.105,

            "v_large_error":
                0.08,

            "v_blind":
                0.05,

            "v_min":
                0.075,

            # --------------------------------------------------------
            # Center / near / large zones
            # --------------------------------------------------------

            "center_x_m":
                0.02,

            "center_theta_rad":
                0.04,

            "near_x_m":
                0.055,

            "near_theta_rad":
                0.11,

            "large_error_x_m":
                0.13,

            "large_error_theta_rad":
                0.4,

            # --------------------------------------------------------
            # Curve detection
            #
            # Curvature chỉ dùng phân loại cua + giảm tốc.
            # KHÔNG dùng curvature trong steering law.
            # --------------------------------------------------------

            "curve_enter_x_m":
                0.075,

            "curve_enter_theta_rad":
                0.17,

            "curve_enter_kappa":
                1.1,

            "curve_strong_theta_rad":
                0.27,

            "curve_confirm_frames":
                1,

            "curve_release_frames":
                3,

            # --------------------------------------------------------
            # PURE PD GAINS
            #
            # omega =
            #
            # -Kp_lat * e_x
            # -Kp_theta * theta
            # -Kd_lat * de_x
            # -Kd_theta * dtheta
            # --------------------------------------------------------

            "kp_lat_near":
                0.28,

            "kp_theta_near":
                0.16,

            "kp_lat_mid":
                0.55,

            "kp_theta_mid":
                0.3,

            "kp_lat_curve":
                0.65,

            "kp_theta_curve":
                0.38,

            "kp_lat_large":
                0.72,

            "kp_theta_large":
                0.38,

            "kd_lat":
                0.018,

            "kd_theta":
                0.07,

            # --------------------------------------------------------
            # Angular limits
            # --------------------------------------------------------

            "omega_center_max":
                0.0,

            "omega_near_max":
                0.065,

            "omega_mid_max":
                0.17,

            "omega_curve_max":
                0.24,

            "omega_large_error_max":
                0.26,

            "omega_abs_max":
                0.28,

            "omega_deadband":
                0.005,

            # --------------------------------------------------------
            # Slew rates
            # --------------------------------------------------------

            "v_rate_up_mps2":
                0.18,

            "v_rate_down_mps2":
                0.6,

            "omega_rate_center_rps2":
                0.85,

            "omega_rate_mid_rps2":
                0.45,

            "omega_rate_curve_rps2":
                0.5,

            "omega_reverse_rate_rps2":
                0.3,

            "omega_release_rate_rps2":
                0.85,

            # --------------------------------------------------------
            # Skid-steer feasibility
            # --------------------------------------------------------

            "wheel_separation_m":
                0.135,

            "max_delta_v_mps":
                0.045,

            "inner_wheel_min_fraction":
                0.32,

            # --------------------------------------------------------
            # Adaptive slowdown
            # --------------------------------------------------------

            "slow_k_x":
                0.5,

            "slow_k_theta":
                0.4,

            "slow_k_kappa":
                0.02,

            # --------------------------------------------------------
            # Perception jump guard
            # --------------------------------------------------------

            "jump_guard_enable":
                False,

            "jump_x_m":
                0.18,

            "jump_theta_rad":
                0.50,

            "jump_hold_s":
                0.12,

            "jump_hold_v":
                0.10,

            "jump_hold_omega_decay":
                0.75,
        }

        # ============================================================
        # Mỗi parameter được declare đúng MỘT lần.
        # ============================================================

        for name, value in parameters.items():

            self.declare_parameter(
                name,
                value
            )

        # ============================================================
        # TOPICS
        # ============================================================

        self.control_error_topic = str(
            self.get_parameter(
                "control_error_topic"
            ).value
        )

        self.lane_state_topic = str(
            self.get_parameter(
                "lane_state_topic"
            ).value
        )

        self.cmd_vel_topic = str(
            self.get_parameter(
                "cmd_vel_topic"
            ).value
        )

        self.debug_topic = str(
            self.get_parameter(
                "debug_topic"
            ).value
        )

        # ============================================================
        # PUBLISHERS
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            self.debug_topic,
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

        # ============================================================
        # RAW INPUT STATE
        # ============================================================

        self.raw_valid = False
        self.raw_reason = "waiting"

        self.raw_lane_state = ""
        self.raw_confidence = 0.0

        self.raw_x = 0.0
        self.raw_theta = 0.0
        self.raw_kappa = 0.0

        self.last_error_rx = -1.0

        # ============================================================
        # FPS
        # ============================================================

        self.last_frame_time = None
        self.last_valid_frame_time = None

        self.fps_est = self.pfloat(
            "fps_init"
        )

        # ============================================================
        # FILTERED ERROR
        # ============================================================

        self.x_filtered = 0.0
        self.theta_filtered = 0.0

        self.dx_filtered = 0.0
        self.dtheta_filtered = 0.0

        self.previous_x_for_derivative = 0.0
        self.previous_theta_for_derivative = 0.0

        window = max(
            1,
            self.pint(
                "median_window"
            )
        )

        self.x_buffer = deque(
            maxlen=window
        )

        self.theta_buffer = deque(
            maxlen=window
        )

        # ============================================================
        # JUMP GUARD
        # ============================================================

        self.last_raw_x = None
        self.last_raw_theta = None

        self.jump_hold_until = -1.0
        self.jump_reason = "none"

        # ============================================================
        # CURVE STATE
        # ============================================================

        self.curve_confirmed = False
        self.curve_sign = 0

        self.curve_count = 0
        self.curve_release_count = 0

        # ============================================================
        # LANE DEBUG
        # ============================================================

        self.lane_debug = {}
        self.last_lane_state_rx = -1.0

        # ============================================================
        # CONTROL OUTPUT STATE
        # ============================================================

        self.v_reference = 0.0
        self.omega_reference = 0.0

        self.previous_loop_time = (
            time.monotonic()
        )

        # ============================================================
        # CONTROL TIMER
        # ============================================================

        frequency = max(
            5.0,
            self.pfloat(
                "control_rate_hz"
            )
        )

        self.create_timer(
            1.0 / frequency,
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
            f"{self.VERSION} started"
        )

        self.get_logger().info(
            f"control_error : "
            f"{self.control_error_topic}"
        )

        self.get_logger().info(
            f"lane_state    : "
            f"{self.lane_state_topic}"
        )

        self.get_logger().info(
            f"cmd_vel       : "
            f"{self.cmd_vel_topic}"
        )

        self.get_logger().info(
            f"debug         : "
            f"{self.debug_topic}"
        )

        self.get_logger().info(
            "Steering law  : PURE PD"
        )

        self.get_logger().info(
            "Pure Pursuit  : NOT USED"
        )

    # ================================================================
    # PARAMETER HELPERS
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

    # ================================================================
    # MATH HELPERS
    # ================================================================

    @staticmethod
    def alpha_from_tau(
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
                    0.001
                )
            )
        )

    @staticmethod
    def make_twist(
        linear,
        angular
    ):

        message = Twist()

        message.linear.x = float(
            linear
        )

        message.angular.z = float(
            angular
        )

        return message

    # ================================================================
    # SIGNAL / STOP
    # ================================================================

    def signal_handler(
        self,
        signum,
        _frame
    ):

        self.get_logger().warn(
            f"Signal {signum}: stop robot"
        )

        self.publish_stop_burst()

        if rclpy.ok():
            rclpy.shutdown()

    def publish_stop_burst(
        self
    ):

        message = self.make_twist(
            0.0,
            0.0
        )

        count = max(
            3,
            self.pint(
                "stop_burst_count"
            )
        )

        delay = max(
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
                delay
            )

    # ================================================================
    # CMD_VEL CONFLICT
    # ================================================================

    def check_cmd_vel_conflict(
        self
    ):

        if (
            not
            self.pbool(
                "check_cmd_vel_conflict"
            )
            or
            self.pbool(
                "allow_cmd_vel_conflict"
            )
        ):

            return (
                False,
                []
            )

        publisher_infos = (
            self.get_publishers_info_by_topic(
                self.cmd_vel_topic
            )
        )

        publisher_names = []

        for info in publisher_infos:

            if info.node_namespace == "/":

                publisher_names.append(
                    info.node_name
                )

            else:

                publisher_names.append(
                    f"{info.node_namespace}/"
                    f"{info.node_name}"
                )

        # Chính node PD_controller_v2 là 1 publisher.
        # >1 nghĩa là có controller khác cùng publish /cmd_vel.

        return (
            len(
                publisher_infos
            )
            >
            1,
            publisher_names
        )

    # ================================================================
    # LANE STATE CALLBACK
    # ================================================================

    def lane_state_callback(
        self,
        message
    ):

        try:

            data = json.loads(
                message.data
            )

            if isinstance(
                data,
                dict
            ):

                self.lane_debug = data

                self.last_lane_state_rx = (
                    time.monotonic()
                )

        except Exception:

            pass

    # ================================================================
    # FPS
    # ================================================================

    def update_fps(
        self,
        now,
        data
    ):

        fps_from_message = finite_float(
            data.get(
                "fps",
                data.get(
                    "fps_est",
                    data.get(
                        "vision_fps"
                    )
                )
            )
        )

        if (
            fps_from_message is not None
            and
            fps_from_message > 0.1
        ):

            fps_now = clamp(
                fps_from_message,
                self.pfloat(
                    "fps_min"
                ),
                self.pfloat(
                    "fps_max"
                )
            )

            frame_dt = (
                1.0
                /
                fps_now
            )

        elif self.last_frame_time is not None:

            frame_dt = clamp(
                now
                -
                self.last_frame_time,
                0.01,
                1.0
            )

            fps_now = clamp(
                1.0
                /
                frame_dt,
                self.pfloat(
                    "fps_min"
                ),
                self.pfloat(
                    "fps_max"
                )
            )

        else:

            self.last_frame_time = (
                now
            )

            return

        self.last_frame_time = (
            now
        )

        alpha = self.alpha_from_tau(
            frame_dt,
            self.pfloat(
                "fps_filter_tau_s"
            )
        )

        self.fps_est = (
            (
                1.0
                -
                alpha
            )
            *
            self.fps_est

            +

            alpha
            *
            fps_now
        )

    # ================================================================
    # EXTRACT CONTROL ERROR
    # ================================================================

    def extract_control_error(
        self,
        data
    ):

        # ------------------------------------------------------------
        # Lateral error
        # ------------------------------------------------------------

        lateral = None

        for key in (
            "lateral_error_m",
            "e_lat_m",
            "e_y_m",
            "x_error_m",
        ):

            if key in data:

                lateral = finite_float(
                    data.get(
                        key
                    )
                )

                break

        if lateral is None:

            lateral_mm = finite_float(
                data.get(
                    "epsilon_x_mm",
                    data.get(
                        "x_mm",
                        data.get(
                            "e_y_mm",
                            0.0
                        )
                    )
                ),
                0.0
            )

            lateral = (
                lateral_mm
                /
                1000.0
            )

        # ------------------------------------------------------------
        # Heading error
        # ------------------------------------------------------------

        theta = None

        for key in (
            "theta_rad",
            "heading_error_rad",
            "e_theta_rad",
            "theta_error_rad",
        ):

            if key in data:

                theta = finite_float(
                    data.get(
                        key
                    )
                )

                break

        if theta is None:
            theta = 0.0

        lateral = (
            self.pfloat(
                "epsilon_sign"
            )
            *
            lateral

            +

            self.pfloat(
                "x_bias_m"
            )
        )

        theta = (
            self.pfloat(
                "theta_sign"
            )
            *
            theta
        )

        # ------------------------------------------------------------
        # Curvature
        # ------------------------------------------------------------

        curvature_inverse_mm = (
            finite_float(
                data.get(
                    "curvature_inv_mm"
                )
            )
        )

        if curvature_inverse_mm is not None:

            kappa = (
                curvature_inverse_mm
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
            kappa,
            -5.0,
            5.0
        )

        # ------------------------------------------------------------
        # Confidence
        # ------------------------------------------------------------

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
                lateral,
                theta,
                kappa,
                confidence,
                lane_state
            )

        if abs(
            lateral
        ) > self.pfloat(
            "max_abs_x_m"
        ):

            return (
                False,
                "lateral_outlier",
                lateral,
                theta,
                kappa,
                confidence,
                lane_state
            )

        if abs(
            theta
        ) > self.pfloat(
            "max_abs_theta_rad"
        ):

            return (
                False,
                "heading_outlier",
                lateral,
                theta,
                kappa,
                confidence,
                lane_state
            )

        if confidence < self.pfloat(
            "min_confidence"
        ):

            return (
                False,
                "low_confidence",
                lateral,
                theta,
                kappa,
                confidence,
                lane_state
            )

        return (
            valid,
            (
                "ok"
                if valid
                else
                "invalid_flags"
            ),
            lateral,
            theta,
            kappa,
            confidence,
            lane_state
        )

    # ================================================================
    # CONTROL ERROR CALLBACK
    #
    # Derivative chỉ cập nhật khi có frame perception mới.
    # ================================================================

    def control_error_callback(
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

        self.update_fps(
            now,
            data
        )

        (
            valid,
            reason,
            lateral,
            theta,
            kappa,
            confidence,
            lane_state
        ) = self.extract_control_error(
            data
        )

        self.raw_valid = (
            valid
        )

        self.raw_reason = (
            reason
        )

        self.raw_x = (
            lateral
        )

        self.raw_theta = (
            theta
        )

        self.raw_kappa = (
            kappa
        )

        self.raw_confidence = (
            confidence
        )

        self.raw_lane_state = (
            lane_state
        )

        self.last_error_rx = (
            now
        )

        if not valid:
            return

        # ============================================================
        # JUMP GUARD
        # ============================================================

        if (
            self.pbool(
                "jump_guard_enable"
            )
            and
            self.last_raw_x
            is not None
            and
            self.last_raw_theta
            is not None
        ):

            delta_x = abs(
                lateral
                -
                self.last_raw_x
            )

            delta_theta = abs(
                theta
                -
                self.last_raw_theta
            )

            if (
                delta_x
                >
                self.pfloat(
                    "jump_x_m"
                )
                or
                delta_theta
                >
                self.pfloat(
                    "jump_theta_rad"
                )
            ):

                self.jump_hold_until = (
                    now
                    +
                    self.pfloat(
                        "jump_hold_s"
                    )
                )

                self.jump_reason = (
                    f"jump dx={delta_x:.3f}, "
                    f"dtheta={delta_theta:.3f}"
                )

                return

        self.jump_reason = (
            "none"
        )

        self.last_raw_x = (
            lateral
        )

        self.last_raw_theta = (
            theta
        )

        # ============================================================
        # MEDIAN BUFFER
        # ============================================================

        window = max(
            1,
            self.pint(
                "median_window"
            )
        )

        if (
            self.x_buffer.maxlen
            !=
            window
        ):

            self.x_buffer = deque(
                list(
                    self.x_buffer
                )[
                    -window:
                ],
                maxlen=window
            )

            self.theta_buffer = deque(
                list(
                    self.theta_buffer
                )[
                    -window:
                ],
                maxlen=window
            )

        self.x_buffer.append(
            lateral
        )

        self.theta_buffer.append(
            theta
        )

        target_x = (
            statistics.median(
                self.x_buffer
            )
        )

        target_theta = (
            statistics.median(
                self.theta_buffer
            )
        )

        # ============================================================
        # FRAME DT
        # ============================================================

        if self.last_valid_frame_time is None:

            frame_dt = (
                1.0
                /
                max(
                    self.fps_est,
                    5.0
                )
            )

        else:

            frame_dt = clamp(
                now
                -
                self.last_valid_frame_time,
                0.01,
                0.50
            )

        self.last_valid_frame_time = (
            now
        )

        # ============================================================
        # ERROR LOW-PASS
        # ============================================================

        alpha = self.alpha_from_tau(
            frame_dt,
            self.pfloat(
                "error_filter_tau_s"
            )
        )

        self.x_filtered = (
            (
                1.0
                -
                alpha
            )
            *
            self.x_filtered

            +

            alpha
            *
            target_x
        )

        self.theta_filtered = (
            (
                1.0
                -
                alpha
            )
            *
            self.theta_filtered

            +

            alpha
            *
            target_theta
        )

        # ============================================================
        # DEADBAND
        # ============================================================

        x_used = (
            0.0
            if
            abs(
                self.x_filtered
            )
            <
            self.pfloat(
                "x_deadband_m"
            )
            else
            self.x_filtered
        )

        theta_used = (
            0.0
            if
            abs(
                self.theta_filtered
            )
            <
            self.pfloat(
                "theta_deadband_rad"
            )
            else
            self.theta_filtered
        )

        # ============================================================
        # DERIVATIVE
        # ============================================================

        dx_raw = (
            x_used
            -
            self.previous_x_for_derivative
        ) / max(
            frame_dt,
            0.001
        )

        dtheta_raw = (
            theta_used
            -
            self.previous_theta_for_derivative
        ) / max(
            frame_dt,
            0.001
        )

        self.previous_x_for_derivative = (
            x_used
        )

        self.previous_theta_for_derivative = (
            theta_used
        )

        dx_raw = clamp(
            dx_raw,
            -self.pfloat(
                "derivative_clip_x_mps"
            ),
            self.pfloat(
                "derivative_clip_x_mps"
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

        derivative_alpha = (
            self.alpha_from_tau(
                frame_dt,
                self.pfloat(
                    "derivative_filter_tau_s"
                )
            )
        )

        self.dx_filtered = (
            (
                1.0
                -
                derivative_alpha
            )
            *
            self.dx_filtered

            +

            derivative_alpha
            *
            dx_raw
        )

        self.dtheta_filtered = (
            (
                1.0
                -
                derivative_alpha
            )
            *
            self.dtheta_filtered

            +

            derivative_alpha
            *
            dtheta_raw
        )

    # ================================================================
    # USED ERRORS
    # ================================================================

    def get_used_errors(
        self
    ):

        lateral = (
            0.0
            if
            abs(
                self.x_filtered
            )
            <
            self.pfloat(
                "x_deadband_m"
            )
            else
            self.x_filtered
        )

        theta = (
            0.0
            if
            abs(
                self.theta_filtered
            )
            <
            self.pfloat(
                "theta_deadband_rad"
            )
            else
            self.theta_filtered
        )

        return (
            lateral,
            theta
        )

    # ================================================================
    # CURVE STATE
    # ================================================================

    def update_curve_state(
        self,
        lateral,
        theta,
        kappa
    ):

        lateral_evidence = (
            abs(
                lateral
            )
            >=
            self.pfloat(
                "curve_enter_x_m"
            )
        )

        heading_evidence = (
            abs(
                theta
            )
            >=
            self.pfloat(
                "curve_enter_theta_rad"
            )
        )

        curvature_evidence = (
            abs(
                kappa
            )
            >=
            self.pfloat(
                "curve_enter_kappa"
            )
        )

        strong_heading = (
            abs(
                theta
            )
            >=
            self.pfloat(
                "curve_strong_theta_rad"
            )
        )

        evidence_count = (
            int(
                lateral_evidence
            )
            +
            int(
                heading_evidence
            )
            +
            int(
                curvature_evidence
            )
        )

        candidate = (
            strong_heading
            or
            evidence_count >= 2
        )

        sign_source = (
            theta
            if
            abs(
                theta
            )
            >=
            self.pfloat(
                "theta_deadband_rad"
            )
            else
            lateral
        )

        if sign_source > 0.0:
            sign = 1

        elif sign_source < 0.0:
            sign = -1

        else:
            sign = 0

        if (
            candidate
            and
            sign != 0
        ):

            if sign == self.curve_sign:

                self.curve_count += 1

            else:

                self.curve_sign = (
                    sign
                )

                self.curve_count = 1

            self.curve_release_count = 0

            if (
                self.curve_count
                >=
                self.pint(
                    "curve_confirm_frames"
                )
            ):

                self.curve_confirmed = (
                    True
                )

        else:

            self.curve_release_count += 1

            if (
                self.curve_release_count
                >=
                self.pint(
                    "curve_release_frames"
                )
            ):

                self.curve_confirmed = (
                    False
                )

                self.curve_sign = 0
                self.curve_count = 0

        return (
            self.curve_confirmed,
            evidence_count
        )

    # ================================================================
    # FPS SPEED CAP
    # ================================================================

    def speed_cap_from_fps(
        self
    ):

        speed = (
            self.fps_est
            *
            self.pfloat(
                "target_distance_per_frame_m"
            )
        )

        return clamp(
            speed,
            self.pfloat(
                "v_min"
            ),
            self.pfloat(
                "v_max"
            )
        )

    # ================================================================
    # PURE PD
    # ================================================================

    def pure_pd(
        self,
        lateral,
        theta,
        kp_lat,
        kp_theta
    ):

        p_lat = (
            -kp_lat
            *
            lateral
        )

        p_theta = (
            -kp_theta
            *
            theta
        )

        d_lat = (
            -self.pfloat(
                "kd_lat"
            )
            *
            self.dx_filtered
        )

        d_theta = (
            -self.pfloat(
                "kd_theta"
            )
            *
            self.dtheta_filtered
        )

        omega = (
            p_lat
            +
            p_theta
            +
            d_lat
            +
            d_theta
        )

        if self.pbool(
            "invert_angular"
        ):

            omega = (
                -omega
            )

            p_lat = -p_lat
            p_theta = -p_theta
            d_lat = -d_lat
            d_theta = -d_theta

        if abs(
            omega
        ) < self.pfloat(
            "omega_deadband"
        ):

            omega = 0.0

        return (
            omega,
            p_lat,
            p_theta,
            d_lat,
            d_theta
        )

    # ================================================================
    # TRACKING LOGIC
    # ================================================================

    def compute_tracking(
        self,
        lateral,
        theta
    ):

        (
            curve,
            curve_evidence
        ) = self.update_curve_state(
            lateral,
            theta,
            self.raw_kappa
        )

        center = (
            abs(
                self.x_filtered
            )
            <=
            self.pfloat(
                "center_x_m"
            )
            and
            abs(
                self.theta_filtered
            )
            <=
            self.pfloat(
                "center_theta_rad"
            )
        )

        near = (
            abs(
                self.x_filtered
            )
            <=
            self.pfloat(
                "near_x_m"
            )
            and
            abs(
                self.theta_filtered
            )
            <=
            self.pfloat(
                "near_theta_rad"
            )
        )

        large = (
            abs(
                self.x_filtered
            )
            >=
            self.pfloat(
                "large_error_x_m"
            )
            or
            abs(
                self.theta_filtered
            )
            >=
            self.pfloat(
                "large_error_theta_rad"
            )
        )

        speed_cap = (
            self.speed_cap_from_fps()
        )

        slow_factor = math.exp(
            -self.pfloat(
                "slow_k_x"
            )
            *
            abs(
                self.x_filtered
            )

            -

            self.pfloat(
                "slow_k_theta"
            )
            *
            abs(
                self.theta_filtered
            )

            -

            self.pfloat(
                "slow_k_kappa"
            )
            *
            abs(
                self.raw_kappa
            )
        )

        # ============================================================
        # CENTER
        # ============================================================

        if center:

            mode = (
                "center_cruise"
            )

            v_desired = min(
                speed_cap,
                self.pfloat(
                    "v_center"
                )
            )

            omega_limit = (
                self.pfloat(
                    "omega_center_max"
                )
            )

            kp_lat = 0.0
            kp_theta = 0.0

            omega_raw = 0.0

            p_lat = 0.0
            p_theta = 0.0
            d_lat = 0.0
            d_theta = 0.0

        # ============================================================
        # NEAR CENTER
        # ============================================================

        elif (
            near
            and
            not curve
        ):

            mode = (
                "near_center_pd"
            )

            v_desired = min(
                speed_cap,
                self.pfloat(
                    "v_near"
                )
            )

            omega_limit = (
                self.pfloat(
                    "omega_near_max"
                )
            )

            kp_lat = self.pfloat(
                "kp_lat_near"
            )

            kp_theta = self.pfloat(
                "kp_theta_near"
            )

            (
                omega_raw,
                p_lat,
                p_theta,
                d_lat,
                d_theta
            ) = self.pure_pd(
                lateral,
                theta,
                kp_lat,
                kp_theta
            )

        # ============================================================
        # LARGE ERROR
        # ============================================================

        elif large:

            mode = (
                "large_error_pd"
            )

            v_desired = (
                self.pfloat(
                    "v_large_error"
                )
            )

            omega_limit = (
                self.pfloat(
                    "omega_large_error_max"
                )
            )

            kp_lat = self.pfloat(
                "kp_lat_large"
            )

            kp_theta = self.pfloat(
                "kp_theta_large"
            )

            (
                omega_raw,
                p_lat,
                p_theta,
                d_lat,
                d_theta
            ) = self.pure_pd(
                lateral,
                theta,
                kp_lat,
                kp_theta
            )

        # ============================================================
        # CURVE
        # ============================================================

        elif curve:

            mode = (
                "curve_pd"
            )

            v_desired = clamp(
                min(
                    speed_cap,
                    self.pfloat(
                        "v_curve"
                    )
                )
                *
                slow_factor,
                self.pfloat(
                    "v_min"
                ),
                self.pfloat(
                    "v_curve"
                )
            )

            omega_limit = (
                self.pfloat(
                    "omega_curve_max"
                )
            )

            kp_lat = self.pfloat(
                "kp_lat_curve"
            )

            kp_theta = self.pfloat(
                "kp_theta_curve"
            )

            (
                omega_raw,
                p_lat,
                p_theta,
                d_lat,
                d_theta
            ) = self.pure_pd(
                lateral,
                theta,
                kp_lat,
                kp_theta
            )

        # ============================================================
        # MID
        # ============================================================

        else:

            mode = (
                "mid_pd_tracking"
            )

            v_desired = clamp(
                min(
                    speed_cap,
                    self.pfloat(
                        "v_mid"
                    )
                )
                *
                slow_factor,
                self.pfloat(
                    "v_min"
                ),
                self.pfloat(
                    "v_mid"
                )
            )

            omega_limit = (
                self.pfloat(
                    "omega_mid_max"
                )
            )

            kp_lat = self.pfloat(
                "kp_lat_mid"
            )

            kp_theta = self.pfloat(
                "kp_theta_mid"
            )

            (
                omega_raw,
                p_lat,
                p_theta,
                d_lat,
                d_theta
            ) = self.pure_pd(
                lateral,
                theta,
                kp_lat,
                kp_theta
            )

        # ============================================================
        # SKID-STEER FEASIBILITY
        # ============================================================

        wheel_separation = max(
            0.05,
            self.pfloat(
                "wheel_separation_m"
            )
        )

        omega_from_delta_v = (
            self.pfloat(
                "max_delta_v_mps"
            )
            /
            wheel_separation
        )

        inner_fraction = clamp(
            self.pfloat(
                "inner_wheel_min_fraction"
            ),
            0.0,
            0.90
        )

        omega_from_inner_wheel = (
            2.0
            *
            max(
                v_desired,
                0.01
            )
            *
            (
                1.0
                -
                inner_fraction
            )
            /
            wheel_separation
        )

        omega_limit_final = min(
            abs(
                omega_limit
            ),
            self.pfloat(
                "omega_abs_max"
            ),
            omega_from_delta_v,
            max(
                0.08,
                omega_from_inner_wheel
            )
        )

        omega_desired = clamp(
            omega_raw,
            -omega_limit_final,
            omega_limit_final
        )

        return {
            "mode":
                mode,

            "v_des":
                v_desired,

            "omega_raw":
                omega_raw,

            "omega_des":
                omega_desired,

            "omega_limit":
                omega_limit_final,

            "curve_confirmed":
                curve,

            "curve_evidence":
                curve_evidence,

            "center_zone":
                center,

            "near_zone":
                near,

            "large_error":
                large,

            "slow_factor":
                slow_factor,

            "kp_lat_used":
                kp_lat,

            "kp_theta_used":
                kp_theta,

            "p_lat":
                p_lat,

            "p_theta":
                p_theta,

            "d_lat":
                d_lat,

            "d_theta":
                d_theta,
        }

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(
        self
    ):

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

        if self.last_error_rx > 0.0:

            error_age = (
                now
                -
                self.last_error_rx
            )

        else:

            error_age = 999.0

        fresh = (
            self.raw_valid
            and
            error_age
            <=
            self.pfloat(
                "error_timeout_s"
            )
        )

        jump_hold = (
            now
            <
            self.jump_hold_until
        )

        (
            lateral,
            theta
        ) = self.get_used_errors()

        tracking = {
            "mode":
                "none",

            "v_des":
                0.0,

            "omega_raw":
                0.0,

            "omega_des":
                0.0,

            "omega_limit":
                0.0,

            "curve_confirmed":
                False,

            "curve_evidence":
                0,

            "center_zone":
                False,

            "near_zone":
                False,

            "large_error":
                False,

            "slow_factor":
                1.0,

            "kp_lat_used":
                0.0,

            "kp_theta_used":
                0.0,

            "p_lat":
                0.0,

            "p_theta":
                0.0,

            "d_lat":
                0.0,

            "d_theta":
                0.0,
        }

        # ============================================================
        # JUMP GUARD
        # ============================================================

        if jump_hold:

            mode = (
                "jump_guard_hold"
            )

            v_desired = (
                self.pfloat(
                    "jump_hold_v"
                )
            )

            omega_desired = (
                self.omega_reference
                *
                self.pfloat(
                    "jump_hold_omega_decay"
                )
            )

        # ============================================================
        # NORMAL TRACKING
        # ============================================================

        elif fresh:

            tracking = (
                self.compute_tracking(
                    lateral,
                    theta
                )
            )

            mode = tracking[
                "mode"
            ]

            v_desired = tracking[
                "v_des"
            ]

            omega_desired = tracking[
                "omega_des"
            ]

        # ============================================================
        # SHORT BLIND HOLD
        # ============================================================

        elif (
            error_age
            <=
            self.pfloat(
                "blind_hold_s"
            )
        ):

            mode = (
                "blind_hold"
            )

            v_desired = (
                self.pfloat(
                    "v_blind"
                )
            )

            omega_desired = 0.0

        # ============================================================
        # TIMEOUT
        # ============================================================

        else:

            mode = (
                "control_error_timeout"
            )

            v_desired = 0.0
            omega_desired = 0.0

        # ============================================================
        # LINEAR SLEW
        # ============================================================

        if (
            v_desired
            >=
            self.v_reference
        ):

            v_rate = (
                self.pfloat(
                    "v_rate_up_mps2"
                )
            )

        else:

            v_rate = (
                self.pfloat(
                    "v_rate_down_mps2"
                )
            )

        # ============================================================
        # ANGULAR SLEW
        # ============================================================

        if mode in {
            "center_cruise",
            "near_center_pd",
            "blind_hold",
        }:

            omega_rate = (
                self.pfloat(
                    "omega_rate_center_rps2"
                )
            )

        elif mode == "curve_pd":

            omega_rate = (
                self.pfloat(
                    "omega_rate_curve_rps2"
                )
            )

        else:

            omega_rate = (
                self.pfloat(
                    "omega_rate_mid_rps2"
                )
            )

        # ========================================================
        # V2_4_FAST_RELEASE
        #
        # Khi yêu cầu quay giảm nhưng vẫn cùng chiều,
        # trả steering về target nhanh hơn.
        #
        # Điều này giúp tránh quay dư sau cua.
        # ========================================================

        if (
            self.omega_reference
            *
            omega_desired
            >=
            0.0
            and
            abs(
                omega_desired
            )
            <
            abs(
                self.omega_reference
            )
        ):

            omega_rate = max(
                omega_rate,
                self.pfloat(
                    "omega_release_rate_rps2"
                )
            )

        if (
            self.omega_reference
            *
            omega_desired
            <
            0.0
            and
            abs(
                self.omega_reference
            )
            >
            0.01
            and
            abs(
                omega_desired
            )
            >
            0.01
        ):

            omega_rate = min(
                omega_rate,
                self.pfloat(
                    "omega_reverse_rate_rps2"
                )
            )

        self.v_reference = approach(
            self.v_reference,
            v_desired,
            v_rate
            *
            dt
        )

        self.omega_reference = approach(
            self.omega_reference,
            omega_desired,
            omega_rate
            *
            dt
        )

        # ============================================================
        # CMD_VEL SAFETY
        # ============================================================

        (
            conflict,
            publishers
        ) = self.check_cmd_vel_conflict()

        command_published = False

        if conflict:

            publish_reason = (
                "cmd_vel_conflict"
            )

            if self.pbool(
                "publish_zero_on_conflict"
            ):

                self.cmd_pub.publish(
                    self.make_twist(
                        0.0,
                        0.0
                    )
                )

        elif self.pbool(
            "enable_motion"
        ):

            self.cmd_pub.publish(
                self.make_twist(
                    self.v_reference,
                    self.omega_reference
                )
            )

            command_published = (
                True
            )

            publish_reason = (
                "published"
            )

        else:

            self.cmd_pub.publish(
                self.make_twist(
                    0.0,
                    0.0
                )
            )

            publish_reason = (
                "enable_motion_false"
            )

        # ============================================================
        # LEFT / RIGHT ESTIMATE
        # ============================================================

        wheel_separation = max(
            0.05,
            self.pfloat(
                "wheel_separation_m"
            )
        )

        v_left_est = (
            self.v_reference
            -
            0.5
            *
            wheel_separation
            *
            self.omega_reference
        )

        v_right_est = (
            self.v_reference
            +
            0.5
            *
            wheel_separation
            *
            self.omega_reference
        )

        # ============================================================
        # DEBUG
        # ============================================================

        debug = {

            "node":
                "PD_controller_v2",

            "version":
                self.VERSION,

            "mode":
                mode,

            "enable_motion":
                self.pbool(
                    "enable_motion"
                ),

            "cmd_published":
                command_published,

            "publish_reason":
                publish_reason,

            "cmd_vel_conflict":
                conflict,

            "cmd_vel_publishers":
                publishers,

            "raw_valid":
                self.raw_valid,

            "raw_valid_reason":
                self.raw_reason,

            "lane_state":
                self.raw_lane_state,

            "confidence":
                self.raw_confidence,

            "error_age_s":
                error_age,

            "fps_est":
                self.fps_est,

            "jump_hold_active":
                jump_hold,

            "jump_reason":
                self.jump_reason,

            "epsilon_x_mm":
                self.raw_x
                *
                1000.0,

            "theta_rad":
                self.raw_theta,

            "kappa_m":
                self.raw_kappa,

            "e_x_f_m":
                self.x_filtered,

            "e_x_f_mm":
                self.x_filtered
                *
                1000.0,

            "theta_f_rad":
                self.theta_filtered,

            "e_x_used_m":
                lateral,

            "e_x_used_mm":
                lateral
                *
                1000.0,

            "theta_used_rad":
                theta,

            "de_x_f":
                self.dx_filtered,

            "dtheta_f":
                self.dtheta_filtered,

            "curve_confirmed":
                tracking[
                    "curve_confirmed"
                ],

            "curve_evidence":
                tracking[
                    "curve_evidence"
                ],

            "center_zone":
                tracking[
                    "center_zone"
                ],

            "near_zone":
                tracking[
                    "near_zone"
                ],

            "large_error":
                tracking[
                    "large_error"
                ],

            "slow_factor":
                tracking[
                    "slow_factor"
                ],

            "k_lat_used":
                tracking[
                    "kp_lat_used"
                ],

            "k_theta_used":
                tracking[
                    "kp_theta_used"
                ],

            "kd_lat":
                self.pfloat(
                    "kd_lat"
                ),

            "kd_theta":
                self.pfloat(
                    "kd_theta"
                ),

            "p_lat":
                tracking[
                    "p_lat"
                ],

            "p_theta":
                tracking[
                    "p_theta"
                ],

            "d_lat":
                tracking[
                    "d_lat"
                ],

            "d_theta":
                tracking[
                    "d_theta"
                ],

            "v_des":
                v_desired,

            "omega_raw":
                tracking[
                    "omega_raw"
                ],

            "omega_des":
                omega_desired,

            "omega_limit":
                tracking[
                    "omega_limit"
                ],

            "v_ref":
                self.v_reference,

            "omega_ref":
                self.omega_reference,

            "v_cmd":
                self.v_reference,

            "omega_cmd":
                self.omega_reference,

            "v_left_est":
                v_left_est,

            "v_right_est":
                v_right_est,

            "delta_v_cmd":
                (
                    v_right_est
                    -
                    v_left_est
                ),

            "lane_state_debug_age_s":
                (
                    now
                    -
                    self.last_lane_state_rx

                    if
                    self.last_lane_state_rx
                    >
                    0.0

                    else
                    -1.0
                ),

            "lane_state_debug":
                self.lane_debug,
        }

        debug_message = String()

        debug_message.data = json.dumps(
            debug,
            ensure_ascii=False
        )

        self.debug_pub.publish(
            debug_message
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = PDControllerV2()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        try:

            node.publish_stop_burst()

        except Exception:

            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()
