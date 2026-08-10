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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
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


def yaw_from_quat(q):
    siny_cosp = 2.0 * (
        q.w * q.z
        +
        q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y
        +
        q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


class PDBacksteppingControllerV2(Node):

    VERSION = "pd_backstepping_v2_dynamic_steering_2_3"

    def __init__(self):

        super().__init__(
            "pd_backstepping_controller_v2"
        )

        # ============================================================
        # DEFAULT PARAMETERS
        #
        # Mặc định đã tune trực tiếp trong node.
        # Sau khi build chỉ cần ros2 run.
        # ============================================================

        parameters = {

            # --------------------------------------------------------
            # Topics
            # --------------------------------------------------------

            "control_error_topic":
                "/avs/control_error",

            "cmd_vel_topic":
                "/cmd_vel",

            "state_topic":
                "/avs/pd_backstepping_v2_state",

            "scan_topic":
                "/scan",

            "odom_topic":
                "/odom_raw",

            # --------------------------------------------------------
            # Runtime
            # --------------------------------------------------------

            "control_hz":
                50.0,

            "enable_cmd":
                True,

            "check_cmd_vel_conflict":
                True,

            "allow_cmd_vel_conflict":
                False,

            "publish_zero_on_conflict":
                True,

            # --------------------------------------------------------
            # Timing
            #
            # V1 dùng fresh=1.8 s là quá dài.
            # --------------------------------------------------------

            "fresh_s":
                0.65,

            "blind_hold_s":
                0.30,

            "lost_stop_s":
                0.80,

            "startup_straight_s":
                0.30,

            "startup_v":
                0.10,

            # --------------------------------------------------------
            # Sign convention
            # --------------------------------------------------------

            "epsilon_sign":
                1.0,

            "theta_sign":
                1.0,

            "steering_sign":
                1.0,

            "invert_angular":
                False,

            "x_bias_m":
                0.0,

            # --------------------------------------------------------
            # Validation
            # --------------------------------------------------------

            "min_confidence":
                0.15,

            "max_abs_e_y_m":
                0.45,

            "max_abs_theta_rad":
                1.20,

            "control_clip_e_y_m":
                0.22,

            "control_clip_theta_rad":
                0.60,

            # --------------------------------------------------------
            # FPS estimator
            # --------------------------------------------------------

            "fps_init":
                10.0,

            "fps_min":
                2.0,

            "fps_max":
                30.0,

            "fps_tau_s":
                0.45,

            # 10 FPS -> 0.30 m/s cap
            "target_distance_per_frame_m":
                0.034,

            # --------------------------------------------------------
            # SPEED PROFILE
            #
            # Straight                  0.30
            # gentle curve              0.23
            # medium curve              0.17
            # sharp turn                0.11
            # --------------------------------------------------------

            "v_max":
                0.3,

            "v_center":
                0.285,

            "v_straight_recover":
                0.245,

            "v_gentle_curve":
                0.205,

            "v_medium_curve":
                0.14,

            "v_sharp_curve":
                0.09,

            "v_large_error":
                0.07,

            "v_blind":
                0.070,

            "v_min":
                0.07,

            # --------------------------------------------------------
            # Speed slew
            #
            # giảm tốc nhanh trước cua,
            # tăng tốc lại từ từ.
            # --------------------------------------------------------

            "v_ref_rate_up":
                0.22,

            "v_ref_rate_down":
                0.8,

            # --------------------------------------------------------
            # ERROR FILTER
            #
            # V1:
            # median=5
            # tau=0.75
            #
            # V2 giảm latency mạnh.
            # --------------------------------------------------------

            "median_window":
                3,

            "error_filter_tau_s":
                0.14,

            "derivative_filter_tau_s":
                0.22,

            "derivative_limit_y_mps":
                0.35,

            "derivative_limit_theta_rps":
                1.2,

            # --------------------------------------------------------
            # Deadband / center
            # --------------------------------------------------------

            "x_deadband_m":
                0.012,

            "theta_deadband_rad":
                0.022,

            "center_x_m":
                0.032,

            "center_theta_rad":
                0.060,

            "center_release_x_m":
                0.050,

            "center_release_theta_rad":
                0.095,

            "near_x_m":
                0.070,

            "near_theta_rad":
                0.145,

            # --------------------------------------------------------
            # LARGE ERROR
            # --------------------------------------------------------

            "large_error_x_m":
                0.09,

            "large_error_theta_rad":
                0.3,

            # --------------------------------------------------------
            # Lookahead
            # --------------------------------------------------------

            "lookahead_default_m":
                0.42,

            "lookahead_min_m":
                0.24,

            "lookahead_max_m":
                0.85,

            # --------------------------------------------------------
            # CURVATURE
            #
            # Dùng curvature thật nếu perception có.
            # Nếu không có, fallback curvature từ error.
            # --------------------------------------------------------

            "curvature_input_weight":
                0.1,

            "curvature_error_weight":
                0.9,

            "curvature_filter_tau_s":
                0.18,

            "max_abs_curvature":
                4.0,

            # --------------------------------------------------------
            # CURVE SEVERITY
            #
            # severity dùng cho SPEED PROFILE.
            # Không hard switch theo một frame.
            # --------------------------------------------------------

            "severity_lat_scale_m":
                0.090,

            "severity_heading_scale_rad":
                0.30,

            "severity_curvature_scale":
                2.20,

            "severity_lat_weight":
                0.18,

            "severity_heading_weight":
                0.37,

            "severity_curvature_weight":
                0.45,

            "severity_filter_tau_s":
                0.14,

            "severity_gentle":
                0.18,

            "severity_medium":
                0.45,

            "severity_sharp":
                0.78,

            # --------------------------------------------------------
            # BACKSTEPPING / PD
            #
            # theta_virtual = atan(lambda_y * e_y)
            #
            # e_theta_bs = theta + theta_virtual
            #
            # omega =
            #   k_ff * v * curvature
            # - k_y * e_y
            # - k_dy * de_y
            # - k_theta * e_theta_bs
            # - k_dtheta * de_theta
            # --------------------------------------------------------

            "k_ff":
                0.08,

            "k_y":
                0.68,

            "k_dy":
                0.035,

            "k_theta":
                0.5,

            "k_dtheta":
                0.12,

            "lambda_y":
                0.95,

            # Feedback mạnh hơn nếu lateral error lớn
            "large_error_gain":
                1.05,

            # --------------------------------------------------------
            # Steering limits by curve severity
            # --------------------------------------------------------

            "omega_center_max":
                0.015,

            "omega_straight_max":
                0.15,

            "omega_gentle_max":
                0.26,

            "omega_medium_max":
                0.35,

            "omega_sharp_max":
                0.38,

            "omega_large_error_max":
                0.37,

            "omega_abs_max":
                0.4,

            "omega_deadband":
                0.006,

            # --------------------------------------------------------
            # Omega rate
            #
            # Nhả steering về 0 nhanh hơn.
            # --------------------------------------------------------

            "omega_rate_center":
                1.0,

            "omega_rate_straight":
                0.58,

            "omega_rate_gentle":
                0.72,

            "omega_rate_medium":
                0.9,

            "omega_rate_sharp":
                0.95,

            "omega_reverse_rate":
                0.46,

            # --------------------------------------------------------
            # Calibration
            #
            # Giữ convention của V1:
            # linear / scale
            # angular * scale
            # --------------------------------------------------------

            "enable_calibration":
                True,

            "linear_cmd_scale":
                1.245,

            "angular_cmd_scale":
                0.78,

            # --------------------------------------------------------
            # Skid steer
            # --------------------------------------------------------

            # --------------------------------------------------------
            # DYNAMIC ANGULAR OUTPUT GAIN
            #
            # Không tăng steering đồng loạt.
            # Càng cong / lệch lớn thì gain output càng tăng.
            # --------------------------------------------------------

            "angular_zone_gain_center":
                0.90,

            "angular_zone_gain_straight":
                0.95,

            "angular_zone_gain_gentle":
                1.05,

            "angular_zone_gain_medium":
                1.12,

            "angular_zone_gain_sharp":
                1.16,

            "angular_zone_gain_large":
                1.18,

            "track_width_m":
                0.135,

            "wheel_radius_m":
                0.0225,

            "allow_pivot_turn":
                False,

            "inner_wheel_min_fraction":
                0.30,

            # --------------------------------------------------------
            # Branch/jump protection
            #
            # V1 branch hold 0.9 s quá lâu.
            #
            # V2 chỉ reject extreme frame rất ngắn.
            # --------------------------------------------------------

            # --------------------------------------------------------
            # FROZEN CONTROL-ERROR GUARD
            #
            # Nếu xe đang chạy nhưng e_y/theta lớn gần như đứng yên
            # bất thường, không tiếp tục lái theo target đó.
            # --------------------------------------------------------

            "freeze_guard_enable":
                True,

            "freeze_same_e_y_m":
                0.004,

            "freeze_same_theta_rad":
                0.010,

            "freeze_trigger_e_y_m":
                0.070,

            "freeze_trigger_theta_rad":
                0.140,

            "freeze_timeout_s":
                1.00,

            "freeze_min_motion_mps":
                0.050,

            "branch_guard_enable":
                True,

            "branch_jump_e_y_m":
                0.25,

            "branch_jump_theta_rad":
                0.65,

            "branch_extreme_e_y_m":
                0.38,

            "branch_extreme_theta_rad":
                1.00,

            "branch_hold_s":
                0.10,

            "branch_hold_v":
                0.10,

            # --------------------------------------------------------
            # LiDAR
            # --------------------------------------------------------

            "enable_lidar_safety":
                False,

            "emergency_distance":
                0.12,

            "stop_distance":
                0.18,

            "slow_distance":
                0.38,

            "front_angle_deg":
                16.0,

            # --------------------------------------------------------
            # Shutdown
            # --------------------------------------------------------

            "stop_burst_count":
                20,

            "stop_burst_dt":
                0.015,
        }

        for name, value in parameters.items():
            self.declare_parameter(
                name,
                value
            )

        # ============================================================
        # TOPICS
        # ============================================================

        self.control_error_topic = self.pstr(
            "control_error_topic"
        )

        self.cmd_vel_topic = self.pstr(
            "cmd_vel_topic"
        )

        self.state_topic = self.pstr(
            "state_topic"
        )

        self.scan_topic = self.pstr(
            "scan_topic"
        )

        self.odom_topic = self.pstr(
            "odom_topic"
        )

        # ============================================================
        # PUB/SUB
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            10
        )

        self.create_subscription(
            String,
            self.control_error_topic,
            self.control_error_callback,
            20
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )

        # ============================================================
        # RAW PERCEPTION
        # ============================================================

        self.raw_valid = False
        self.raw_reason = "waiting"

        self.raw_lane_state = "UNKNOWN"
        self.raw_confidence = 0.0

        self.raw_e_y = 0.0
        self.raw_e_theta = 0.0

        self.raw_lookahead = self.pfloat(
            "lookahead_default_m"
        )

        self.raw_curvature = 0.0
        self.raw_curvature_available = False

        self.last_msg_time = -1.0
        self.last_valid_time = -1.0
        self.first_valid_time = -1.0

        # ============================================================
        # FPS
        # ============================================================

        self.last_frame_time = None

        self.fps_est = self.pfloat(
            "fps_init"
        )

        # ============================================================
        # ERROR FILTER
        # ============================================================

        window = max(
            1,
            self.pint(
                "median_window"
            )
        )

        self.e_y_buffer = deque(
            maxlen=window
        )

        self.e_theta_buffer = deque(
            maxlen=window
        )

        self.e_y_f = 0.0
        self.e_theta_f = 0.0

        self.e_y_used = 0.0
        self.e_theta_used = 0.0

        self.de_y_f = 0.0
        self.de_theta_f = 0.0

        self.prev_frame_e_y_used = 0.0
        self.prev_frame_e_theta_used = 0.0

        self.last_valid_frame_time = None

        self.center_latched = False

        # ============================================================
        # CURVATURE / SEVERITY
        # ============================================================

        self.curvature_f = 0.0
        self.severity_f = 0.0

        # ============================================================
        # BRANCH GUARD
        # ============================================================

        self.last_accepted_e_y = None
        self.last_accepted_e_theta = None

        self.branch_hold_until = -1.0
        self.branch_hold_reason = "none"

        # ============================================================
        # FROZEN CONTROL ERROR
        # ============================================================

        self.freeze_last_e_y = None
        self.freeze_last_theta = None

        self.freeze_start_time = None
        self.control_error_frozen = False
        self.freeze_age_s = 0.0

        # ============================================================
        # COMMAND STATE
        # ============================================================

        self.v_ref_prev = 0.0
        self.omega_ref_prev = 0.0

        self.v_cmd_prev = 0.0
        self.omega_cmd_prev = 0.0

        # ============================================================
        # ODOM
        # ============================================================

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.odom_v = 0.0
        self.odom_omega = 0.0

        self.last_odom_time = -1.0

        # ============================================================
        # LIDAR
        # ============================================================

        self.front_min = math.inf
        self.last_scan_time = -1.0

        # ============================================================
        # LOOP
        # ============================================================

        self.prev_loop_time = time.monotonic()

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
            f"control_error: {self.control_error_topic}"
        )

        self.get_logger().info(
            f"state        : {self.state_topic}"
        )

        self.get_logger().info(
            f"cmd_vel      : {self.cmd_vel_topic}"
        )

        self.get_logger().info(
            "Default straight speed = 0.30 m/s"
        )

    # ================================================================
    # PARAM HELPERS
    # ================================================================

    def pfloat(self, name):
        return float(
            self.get_parameter(
                name
            ).value
        )

    def pint(self, name):
        return int(
            self.get_parameter(
                name
            ).value
        )

    def pbool(self, name):
        return bool(
            self.get_parameter(
                name
            ).value
        )

    def pstr(self, name):
        return str(
            self.get_parameter(
                name
            ).value
        )

    @staticmethod
    def alpha_from_tau(dt, tau):
        return (
            1.0
            -
            math.exp(
                -max(dt, 0.0)
                /
                max(tau, 0.001)
            )
        )

    @staticmethod
    def make_cmd(v, omega):
        msg = Twist()

        msg.linear.x = float(v)
        msg.angular.z = float(omega)

        return msg

    # ================================================================
    # STOP
    # ================================================================

    def signal_handler(
        self,
        _signum,
        _frame
    ):
        self.publish_stop_burst()

        if rclpy.ok():
            rclpy.shutdown()

    def publish_stop_burst(self):

        stop = self.make_cmd(
            0.0,
            0.0
        )

        for _ in range(
            max(
                3,
                self.pint(
                    "stop_burst_count"
                )
            )
        ):
            self.cmd_pub.publish(
                stop
            )

            time.sleep(
                max(
                    0.005,
                    self.pfloat(
                        "stop_burst_dt"
                    )
                )
            )

    # ================================================================
    # INPUT PARSING
    # ================================================================

    def extract_errors(self, data):

        e_y = None

        for key in (
            "lateral_error_m",
            "lateral_error",
            "e_y_m",
            "e_lat_m",
            "x_error_m",
        ):
            if key in data:
                e_y = finite_float(
                    data.get(key)
                )
                break

        if e_y is None:

            for key in (
                "epsilon_x_mm",
                "x_mm",
                "e_y_mm",
                "e_lat_mm",
            ):
                value = finite_float(
                    data.get(key)
                )

                if value is not None:
                    e_y = value / 1000.0
                    break

        if e_y is None:
            e_y = 0.0

        e_y = (
            self.pfloat(
                "epsilon_sign"
            )
            *
            e_y
            +
            self.pfloat(
                "x_bias_m"
            )
        )

        e_theta = None

        for key in (
            "heading_error_rad",
            "heading_error",
            "e_theta_rad",
            "theta_error_rad",
            "theta_rad",
        ):
            if key in data:

                e_theta = finite_float(
                    data.get(key)
                )

                break

        if e_theta is None:
            e_theta = 0.0

        e_theta *= self.pfloat(
            "theta_sign"
        )

        # ------------------------------------------------------------
        # Lookahead
        # ------------------------------------------------------------

        lookahead = None

        for key in (
            "lookahead_m",
            "lookahead_d_m",
            "epsilon_y_m",
            "target_y_m",
        ):
            if key in data:

                lookahead = finite_float(
                    data.get(key)
                )

                break

        if lookahead is None:

            for key in (
                "lookahead_d_mm",
                "epsilon_y_mm",
                "target_y_mm",
            ):

                value = finite_float(
                    data.get(key)
                )

                if value is not None:
                    lookahead = value / 1000.0
                    break

        if lookahead is None:
            lookahead = self.pfloat(
                "lookahead_default_m"
            )

        lookahead = clamp(
            abs(lookahead),
            self.pfloat(
                "lookahead_min_m"
            ),
            self.pfloat(
                "lookahead_max_m"
            )
        )

        # ------------------------------------------------------------
        # Curvature from perception
        # ------------------------------------------------------------

        curvature = None

        inverse_mm = finite_float(
            data.get(
                "curvature_inv_mm"
            )
        )

        if inverse_mm is not None:
            curvature = (
                inverse_mm
                *
                1000.0
            )

        if curvature is None:

            for key in (
                "curvature_m_inv",
                "curvature",
                "kappa",
            ):
                if key in data:

                    curvature = finite_float(
                        data.get(key)
                    )

                    if curvature is not None:
                        break

        curvature_available = (
            curvature is not None
        )

        if curvature is None:
            curvature = 0.0

        curvature = clamp(
            curvature,
            -self.pfloat(
                "max_abs_curvature"
            ),
            self.pfloat(
                "max_abs_curvature"
            )
        )

        return (
            e_y,
            e_theta,
            lookahead,
            curvature,
            curvature_available
        )

    def extract_valid(
        self,
        data,
        e_y,
        e_theta
    ):

        lane_state = str(
            data.get(
                "lane_state",
                data.get(
                    "state",
                    ""
                )
            )
        ).upper()

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

        if lane_state in {
            "LOST",
            "INVALID",
            "NO_LANE",
            "NONE",
        }:
            return (
                False,
                confidence,
                lane_state,
                "lane_state_invalid"
            )

        if (
            abs(e_y)
            >
            self.pfloat(
                "max_abs_e_y_m"
            )
        ):
            return (
                False,
                confidence,
                lane_state,
                "lateral_outlier"
            )

        if (
            abs(e_theta)
            >
            self.pfloat(
                "max_abs_theta_rad"
            )
        ):
            return (
                False,
                confidence,
                lane_state,
                "theta_outlier"
            )

        if (
            confidence
            <
            self.pfloat(
                "min_confidence"
            )
        ):
            return (
                False,
                confidence,
                lane_state,
                "low_confidence"
            )

        valid = (
            parse_bool(
                data.get(
                    "valid",
                    True
                )
            )
            and
            parse_bool(
                data.get(
                    "lane_valid",
                    True
                )
            )
        )

        if lane_state == "FOLLOW_MAIN":
            valid = True

        return (
            valid,
            confidence,
            lane_state,
            "ok" if valid else "invalid"
        )

    # ================================================================
    # FPS
    # ================================================================

    def update_fps(
        self,
        now,
        data
    ):

        fps_msg = finite_float(
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
            fps_msg is not None
            and
            fps_msg > 0.1
        ):

            fps_now = clamp(
                fps_msg,
                self.pfloat(
                    "fps_min"
                ),
                self.pfloat(
                    "fps_max"
                )
            )

            frame_dt = 1.0 / max(
                fps_now,
                0.001
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
                1.0 / frame_dt,
                self.pfloat(
                    "fps_min"
                ),
                self.pfloat(
                    "fps_max"
                )
            )

        else:

            self.last_frame_time = now
            return

        self.last_frame_time = now

        alpha = self.alpha_from_tau(
            frame_dt,
            self.pfloat(
                "fps_tau_s"
            )
        )

        self.fps_est = (
            (1.0 - alpha)
            *
            self.fps_est
            +
            alpha
            *
            fps_now
        )

    # ================================================================
    # BRANCH GUARD
    # ================================================================

    def branch_guard_triggered(
        self,
        e_y,
        e_theta
    ):

        if not self.pbool(
            "branch_guard_enable"
        ):
            return False, "disabled"

        if (
            abs(e_y)
            >
            self.pfloat(
                "branch_extreme_e_y_m"
            )
            or
            abs(e_theta)
            >
            self.pfloat(
                "branch_extreme_theta_rad"
            )
        ):
            return True, "extreme"

        if (
            self.last_accepted_e_y
            is None
            or
            self.last_accepted_e_theta
            is None
        ):
            return False, "first"

        if (
            abs(
                e_y
                -
                self.last_accepted_e_y
            )
            >
            self.pfloat(
                "branch_jump_e_y_m"
            )
            or
            abs(
                e_theta
                -
                self.last_accepted_e_theta
            )
            >
            self.pfloat(
                "branch_jump_theta_rad"
            )
        ):
            return True, "jump"

        return False, "normal"

    # ================================================================
    # FROZEN CONTROL ERROR GUARD
    # ================================================================

    def update_freeze_guard(
        self,
        now,
        e_y,
        e_theta
    ):

        if not self.pbool(
            "freeze_guard_enable"
        ):

            self.control_error_frozen = False
            self.freeze_start_time = None
            self.freeze_age_s = 0.0
            return

        # Only care about a frozen value when the error is meaningful.
        dangerous_error = (
            abs(e_y)
            >=
            self.pfloat(
                "freeze_trigger_e_y_m"
            )
            or
            abs(e_theta)
            >=
            self.pfloat(
                "freeze_trigger_theta_rad"
            )
        )

        if (
            self.freeze_last_e_y
            is None
            or
            self.freeze_last_theta
            is None
        ):

            self.freeze_last_e_y = e_y
            self.freeze_last_theta = e_theta
            self.freeze_start_time = None
            self.control_error_frozen = False
            self.freeze_age_s = 0.0
            return

        same_error = (
            abs(
                e_y
                -
                self.freeze_last_e_y
            )
            <=
            self.pfloat(
                "freeze_same_e_y_m"
            )
            and
            abs(
                e_theta
                -
                self.freeze_last_theta
            )
            <=
            self.pfloat(
                "freeze_same_theta_rad"
            )
        )

        robot_moving = (
            abs(self.odom_v)
            >=
            self.pfloat(
                "freeze_min_motion_mps"
            )
            or
            abs(self.odom_omega)
            >=
            0.05
        )

        if (
            dangerous_error
            and
            same_error
            and
            robot_moving
        ):

            if self.freeze_start_time is None:
                self.freeze_start_time = now

            self.freeze_age_s = (
                now
                -
                self.freeze_start_time
            )

            self.control_error_frozen = (
                self.freeze_age_s
                >=
                self.pfloat(
                    "freeze_timeout_s"
                )
            )

        else:

            self.freeze_start_time = None
            self.freeze_age_s = 0.0
            self.control_error_frozen = False

        self.freeze_last_e_y = e_y
        self.freeze_last_theta = e_theta


    # ================================================================
    # VISION CALLBACK
    #
    # Điểm khác V1:
    # derivative cập nhật tại đây, theo frame perception.
    # Không tính derivative 50 lần/s từ cùng một frame.
    # ================================================================

    def control_error_callback(
        self,
        msg
    ):

        now = time.monotonic()

        try:
            data = json.loads(
                msg.data
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
            e_y,
            e_theta,
            lookahead,
            curvature,
            curvature_available
        ) = self.extract_errors(
            data
        )

        (
            valid,
            confidence,
            lane_state,
            reason
        ) = self.extract_valid(
            data,
            e_y,
            e_theta
        )

        self.raw_valid = valid
        self.raw_reason = reason
        self.raw_confidence = confidence
        self.raw_lane_state = lane_state

        self.raw_e_y = e_y
        self.raw_e_theta = e_theta

        self.raw_lookahead = lookahead
        self.raw_curvature = curvature
        self.raw_curvature_available = (
            curvature_available
        )

        self.last_msg_time = now

        if not valid:
            return

        if self.first_valid_time < 0.0:
            self.first_valid_time = now

        self.last_valid_time = now

        # Detect a large control error that stays almost perfectly
        # unchanged while odometry says the robot is moving.
        self.update_freeze_guard(
            now,
            e_y,
            e_theta
        )

        branch, branch_reason = (
            self.branch_guard_triggered(
                e_y,
                e_theta
            )
        )

        if branch:

            self.branch_hold_until = (
                now
                +
                self.pfloat(
                    "branch_hold_s"
                )
            )

            self.branch_hold_reason = (
                branch_reason
            )

            return

        self.branch_hold_reason = "none"

        self.last_accepted_e_y = e_y
        self.last_accepted_e_theta = e_theta

        # ------------------------------------------------------------
        # Dynamic median window
        # ------------------------------------------------------------

        n = max(
            1,
            self.pint(
                "median_window"
            )
        )

        if self.e_y_buffer.maxlen != n:

            self.e_y_buffer = deque(
                list(
                    self.e_y_buffer
                )[-n:],
                maxlen=n
            )

            self.e_theta_buffer = deque(
                list(
                    self.e_theta_buffer
                )[-n:],
                maxlen=n
            )

        self.e_y_buffer.append(
            clamp(
                e_y,
                -self.pfloat(
                    "control_clip_e_y_m"
                ),
                self.pfloat(
                    "control_clip_e_y_m"
                )
            )
        )

        self.e_theta_buffer.append(
            clamp(
                e_theta,
                -self.pfloat(
                    "control_clip_theta_rad"
                ),
                self.pfloat(
                    "control_clip_theta_rad"
                )
            )
        )

        y_target = statistics.median(
            self.e_y_buffer
        )

        theta_target = statistics.median(
            self.e_theta_buffer
        )

        # ------------------------------------------------------------
        # Frame dt
        # ------------------------------------------------------------

        if self.last_valid_frame_time is None:

            dt = 1.0 / max(
                self.fps_est,
                5.0
            )

        else:

            dt = clamp(
                now
                -
                self.last_valid_frame_time,
                0.01,
                0.50
            )

        self.last_valid_frame_time = now

        # ------------------------------------------------------------
        # Error LPF
        # ------------------------------------------------------------

        alpha = self.alpha_from_tau(
            dt,
            self.pfloat(
                "error_filter_tau_s"
            )
        )

        self.e_y_f = (
            (1.0 - alpha)
            *
            self.e_y_f
            +
            alpha
            *
            y_target
        )

        self.e_theta_f = (
            (1.0 - alpha)
            *
            self.e_theta_f
            +
            alpha
            *
            theta_target
        )

        # ------------------------------------------------------------
        # Center hysteresis
        # ------------------------------------------------------------

        if self.center_latched:

            if (
                abs(self.e_y_f)
                >
                self.pfloat(
                    "center_release_x_m"
                )
                or
                abs(self.e_theta_f)
                >
                self.pfloat(
                    "center_release_theta_rad"
                )
            ):
                self.center_latched = False

        elif (
            abs(self.e_y_f)
            <=
            self.pfloat(
                "center_x_m"
            )
            and
            abs(self.e_theta_f)
            <=
            self.pfloat(
                "center_theta_rad"
            )
        ):
            self.center_latched = True

        # ------------------------------------------------------------
        # Used errors
        # ------------------------------------------------------------

        self.e_y_used = (
            0.0
            if
            abs(self.e_y_f)
            <
            self.pfloat(
                "x_deadband_m"
            )
            else
            self.e_y_f
        )

        self.e_theta_used = (
            0.0
            if
            abs(self.e_theta_f)
            <
            self.pfloat(
                "theta_deadband_rad"
            )
            else
            self.e_theta_f
        )

        # ------------------------------------------------------------
        # DERIVATIVE PER VISION FRAME
        # ------------------------------------------------------------

        de_y_raw = (
            self.e_y_used
            -
            self.prev_frame_e_y_used
        ) / max(
            dt,
            0.001
        )

        de_theta_raw = (
            self.e_theta_used
            -
            self.prev_frame_e_theta_used
        ) / max(
            dt,
            0.001
        )

        self.prev_frame_e_y_used = (
            self.e_y_used
        )

        self.prev_frame_e_theta_used = (
            self.e_theta_used
        )

        de_y_raw = clamp(
            de_y_raw,
            -self.pfloat(
                "derivative_limit_y_mps"
            ),
            self.pfloat(
                "derivative_limit_y_mps"
            )
        )

        de_theta_raw = clamp(
            de_theta_raw,
            -self.pfloat(
                "derivative_limit_theta_rps"
            ),
            self.pfloat(
                "derivative_limit_theta_rps"
            )
        )

        d_alpha = self.alpha_from_tau(
            dt,
            self.pfloat(
                "derivative_filter_tau_s"
            )
        )

        self.de_y_f = (
            (1.0 - d_alpha)
            *
            self.de_y_f
            +
            d_alpha
            *
            de_y_raw
        )

        self.de_theta_f = (
            (1.0 - d_alpha)
            *
            self.de_theta_f
            +
            d_alpha
            *
            de_theta_raw
        )

    # ================================================================
    # ODOM / LIDAR
    # ================================================================

    def odom_callback(
        self,
        msg
    ):

        self.odom_x = float(
            msg.pose.pose.position.x
        )

        self.odom_y = float(
            msg.pose.pose.position.y
        )

        self.odom_yaw = yaw_from_quat(
            msg.pose.pose.orientation
        )

        self.odom_v = float(
            msg.twist.twist.linear.x
        )

        self.odom_omega = float(
            msg.twist.twist.angular.z
        )

        self.last_odom_time = (
            time.monotonic()
        )

    def scan_callback(
        self,
        msg
    ):

        front_angle = math.radians(
            self.pfloat(
                "front_angle_deg"
            )
        )

        values = []

        angle = msg.angle_min

        for distance in msg.ranges:

            if (
                math.isfinite(distance)
                and
                msg.range_min
                <=
                distance
                <=
                msg.range_max
                and
                abs(angle)
                <=
                front_angle
            ):
                values.append(
                    float(distance)
                )

            angle += msg.angle_increment

        self.front_min = (
            min(values)
            if values
            else math.inf
        )

        self.last_scan_time = (
            time.monotonic()
        )

    # ================================================================
    # SPEED / CURVATURE
    # ================================================================

    def speed_from_fps(self):

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

    def effective_curvature(self, dt):

        lookahead = clamp(
            self.raw_lookahead,
            self.pfloat(
                "lookahead_min_m"
            ),
            self.pfloat(
                "lookahead_max_m"
            )
        )

        error_curvature = (
            -2.0
            *
            self.e_y_used
            /
            max(
                lookahead
                *
                lookahead,
                0.0001
            )
        )

        if self.raw_curvature_available:

            target = (
                self.pfloat(
                    "curvature_input_weight"
                )
                *
                self.raw_curvature
                +
                self.pfloat(
                    "curvature_error_weight"
                )
                *
                error_curvature
            )

        else:

            target = error_curvature

        target = clamp(
            target,
            -self.pfloat(
                "max_abs_curvature"
            ),
            self.pfloat(
                "max_abs_curvature"
            )
        )

        alpha = self.alpha_from_tau(
            dt,
            self.pfloat(
                "curvature_filter_tau_s"
            )
        )

        self.curvature_f = (
            (1.0 - alpha)
            *
            self.curvature_f
            +
            alpha
            *
            target
        )

        return (
            self.curvature_f,
            error_curvature,
            lookahead
        )

    def compute_severity(
        self,
        curvature,
        dt
    ):

        raw = (
            self.pfloat(
                "severity_lat_weight"
            )
            *
            abs(
                self.e_y_used
            )
            /
            max(
                self.pfloat(
                    "severity_lat_scale_m"
                ),
                0.001
            )

            +

            self.pfloat(
                "severity_heading_weight"
            )
            *
            abs(
                self.e_theta_used
            )
            /
            max(
                self.pfloat(
                    "severity_heading_scale_rad"
                ),
                0.001
            )

            +

            self.pfloat(
                "severity_curvature_weight"
            )
            *
            abs(
                curvature
            )
            /
            max(
                self.pfloat(
                    "severity_curvature_scale"
                ),
                0.001
            )
        )

        raw = clamp(
            raw,
            0.0,
            2.0
        )

        alpha = self.alpha_from_tau(
            dt,
            self.pfloat(
                "severity_filter_tau_s"
            )
        )

        self.severity_f = (
            (1.0 - alpha)
            *
            self.severity_f
            +
            alpha
            *
            raw
        )

        return raw

    def interpolate(
        self,
        a,
        b,
        ratio
    ):
        ratio = clamp(
            ratio,
            0.0,
            1.0
        )

        return (
            a
            +
            (b - a)
            *
            ratio
        )

    def speed_and_zone(
        self,
        speed_cap
    ):

        s = self.severity_f

        s1 = self.pfloat(
            "severity_gentle"
        )

        s2 = self.pfloat(
            "severity_medium"
        )

        s3 = self.pfloat(
            "severity_sharp"
        )

        if self.center_latched:

            return (
                min(
                    speed_cap,
                    self.pfloat(
                        "v_center"
                    )
                ),
                "center"
            )

        if (
            abs(self.e_y_used)
            >
            self.pfloat(
                "large_error_x_m"
            )
            or
            abs(self.e_theta_used)
            >
            self.pfloat(
                "large_error_theta_rad"
            )
        ):

            return (
                self.pfloat(
                    "v_large_error"
                ),
                "large_error"
            )

        if s <= s1:

            ratio = s / max(
                s1,
                0.001
            )

            speed = self.interpolate(
                self.pfloat(
                    "v_straight_recover"
                ),
                self.pfloat(
                    "v_gentle_curve"
                ),
                ratio
            )

            return (
                min(
                    speed_cap,
                    speed
                ),
                "straight"
            )

        if s <= s2:

            ratio = (
                s - s1
            ) / max(
                s2 - s1,
                0.001
            )

            speed = self.interpolate(
                self.pfloat(
                    "v_gentle_curve"
                ),
                self.pfloat(
                    "v_medium_curve"
                ),
                ratio
            )

            return (
                min(
                    speed_cap,
                    speed
                ),
                "gentle"
            )

        if s <= s3:

            ratio = (
                s - s2
            ) / max(
                s3 - s2,
                0.001
            )

            speed = self.interpolate(
                self.pfloat(
                    "v_medium_curve"
                ),
                self.pfloat(
                    "v_sharp_curve"
                ),
                ratio
            )

            return (
                min(
                    speed_cap,
                    speed
                ),
                "medium"
            )

        return (
            self.pfloat(
                "v_sharp_curve"
            ),
            "sharp"
        )

    # ================================================================
    # BACKSTEPPING TRACKING
    # ================================================================

    def compute_tracking(
        self,
        dt
    ):

        speed_cap = self.speed_from_fps()

        (
            curvature,
            error_curvature,
            lookahead
        ) = self.effective_curvature(
            dt
        )

        severity_raw = (
            self.compute_severity(
                curvature,
                dt
            )
        )

        (
            v_des,
            curve_zone
        ) = self.speed_and_zone(
            speed_cap
        )

        # ------------------------------------------------------------
        # Backstepping virtual heading
        # ------------------------------------------------------------

        theta_virtual = math.atan(
            self.pfloat(
                "lambda_y"
            )
            *
            self.e_y_used
        )

        e_theta_bs = (
            self.e_theta_used
            +
            theta_virtual
        )

        # ------------------------------------------------------------
        # Components
        # ------------------------------------------------------------

        omega_ff = (
            self.pfloat(
                "k_ff"
            )
            *
            v_des
            *
            curvature
        )

        p_y = (
            -self.pfloat(
                "k_y"
            )
            *
            self.e_y_used
        )

        d_y = (
            -self.pfloat(
                "k_dy"
            )
            *
            self.de_y_f
        )

        p_theta = (
            -self.pfloat(
                "k_theta"
            )
            *
            e_theta_bs
        )

        d_theta = (
            -self.pfloat(
                "k_dtheta"
            )
            *
            self.de_theta_f
        )

        feedback_gain = 1.0

        if curve_zone == "large_error":
            feedback_gain = self.pfloat(
                "large_error_gain"
            )

        omega_raw = (
            omega_ff
            +
            feedback_gain
            *
            (
                p_y
                +
                d_y
                +
                p_theta
                +
                d_theta
            )
        )

        omega_raw *= self.pfloat(
            "steering_sign"
        )

        if self.pbool(
            "invert_angular"
        ):
            omega_raw = (
                -omega_raw
            )

        # ------------------------------------------------------------
        # Zone-specific omega limits
        # ------------------------------------------------------------

        if curve_zone == "center":

            omega_limit = self.pfloat(
                "omega_center_max"
            )

        elif curve_zone == "straight":

            omega_limit = self.pfloat(
                "omega_straight_max"
            )

        elif curve_zone == "gentle":

            omega_limit = self.pfloat(
                "omega_gentle_max"
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

            omega_limit = self.pfloat(
                "omega_large_error_max"
            )

        omega_limit = min(
            omega_limit,
            self.pfloat(
                "omega_abs_max"
            )
        )

        omega_des = clamp(
            omega_raw,
            -omega_limit,
            omega_limit
        )

        if (
            abs(omega_des)
            <
            self.pfloat(
                "omega_deadband"
            )
        ):
            omega_des = 0.0

        return {
            "curve_zone":
                curve_zone,

            "speed_cap":
                speed_cap,

            "v_des":
                v_des,

            "omega_des":
                omega_des,

            "omega_raw":
                omega_raw,

            "omega_limit":
                omega_limit,

            "omega_ff":
                omega_ff,

            "p_y":
                p_y,

            "d_y":
                d_y,

            "p_theta":
                p_theta,

            "d_theta":
                d_theta,

            "theta_virtual":
                theta_virtual,

            "e_theta_bs":
                e_theta_bs,

            "curvature":
                curvature,

            "error_curvature":
                error_curvature,

            "lookahead":
                lookahead,

            "severity_raw":
                severity_raw,

            "severity":
                self.severity_f,

            "feedback_gain":
                feedback_gain,
        }

    # ================================================================
    # OUTPUT CONSTRAINTS
    # ================================================================

    def apply_no_pivot_limit(
        self,
        v,
        omega
    ):

        if self.pbool(
            "allow_pivot_turn"
        ):
            return (
                v,
                omega,
                "pivot_allowed"
            )

        if v <= 0.0:
            return (
                0.0,
                0.0,
                "zero_v"
            )

        B = max(
            0.05,
            self.pfloat(
                "track_width_m"
            )
        )

        fraction = clamp(
            self.pfloat(
                "inner_wheel_min_fraction"
            ),
            0.0,
            0.95
        )

        max_omega = (
            2.0
            *
            v
            *
            (1.0 - fraction)
            /
            B
        )

        return (
            v,
            clamp(
                omega,
                -max_omega,
                max_omega
            ),
            "no_pivot_limit"
        )

    def apply_lidar_safety(
        self,
        v,
        omega
    ):

        if not self.pbool(
            "enable_lidar_safety"
        ):
            return (
                v,
                omega,
                False,
                "disabled"
            )

        if not math.isfinite(
            self.front_min
        ):
            return (
                v,
                omega,
                False,
                "no_data"
            )

        if (
            self.front_min
            <
            self.pfloat(
                "emergency_distance"
            )
        ):
            return (
                0.0,
                0.0,
                True,
                "emergency"
            )

        if (
            self.front_min
            <
            self.pfloat(
                "stop_distance"
            )
        ):
            return (
                0.0,
                0.0,
                True,
                "stop"
            )

        if (
            self.front_min
            <
            self.pfloat(
                "slow_distance"
            )
        ):

            ratio = clamp(
                (
                    self.front_min
                    -
                    self.pfloat(
                        "stop_distance"
                    )
                )
                /
                max(
                    self.pfloat(
                        "slow_distance"
                    )
                    -
                    self.pfloat(
                        "stop_distance"
                    ),
                    0.001
                ),
                0.30,
                1.0
            )

            return (
                v * ratio,
                omega,
                False,
                "slow"
            )

        return (
            v,
            omega,
            False,
            "clear"
        )

    # ================================================================
    # CMD CONFLICT
    # ================================================================

    def cmd_vel_conflict_detected(
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
    # MAIN LOOP
    # ================================================================

    def control_loop(self):

        now = time.monotonic()

        dt = clamp(
            now
            -
            self.prev_loop_time,
            0.001,
            0.10
        )

        self.prev_loop_time = now

        msg_age = (
            now
            -
            self.last_msg_time
            if
            self.last_msg_time
            >
            0.0
            else
            999.0
        )

        valid_age = (
            now
            -
            self.last_valid_time
            if
            self.last_valid_time
            >
            0.0
            else
            999.0
        )

        startup = (
            self.first_valid_time
            >
            0.0
            and
            now
            -
            self.first_valid_time
            <
            self.pfloat(
                "startup_straight_s"
            )
        )

        branch_hold = (
            now
            <
            self.branch_hold_until
        )

        fresh = (
            self.raw_valid
            and
            msg_age
            <=
            self.pfloat(
                "fresh_s"
            )
        )

        info = {
            "curve_zone": "none",
            "speed_cap": 0.0,
            "v_des": 0.0,
            "omega_des": 0.0,
            "omega_raw": 0.0,
            "omega_limit": 0.0,
            "omega_ff": 0.0,
            "p_y": 0.0,
            "d_y": 0.0,
            "p_theta": 0.0,
            "d_theta": 0.0,
            "theta_virtual": 0.0,
            "e_theta_bs": 0.0,
            "curvature": self.curvature_f,
            "error_curvature": 0.0,
            "lookahead": self.raw_lookahead,
            "severity_raw": 0.0,
            "severity": self.severity_f,
            "feedback_gain": 1.0,
        }

        # ------------------------------------------------------------
        # MODE
        # ------------------------------------------------------------

        if self.control_error_frozen:

            # --------------------------------------------------------
            # The perception target is no longer trustworthy.
            #
            # Do NOT keep steering toward a potentially adjacent lane.
            # --------------------------------------------------------

            mode = "frozen_control_error"

            v_des = 0.0
            omega_des = 0.0

        elif branch_hold:

            mode = "branch_hold"

            v_des = self.pfloat(
                "branch_hold_v"
            )

            omega_des = (
                self.omega_ref_prev
                *
                0.55
            )

        elif startup:

            mode = "startup"

            v_des = self.pfloat(
                "startup_v"
            )

            omega_des = 0.0

        elif fresh:

            mode = "tracking"

            info = self.compute_tracking(
                dt
            )

            v_des = info[
                "v_des"
            ]

            omega_des = info[
                "omega_des"
            ]

        elif (
            valid_age
            <=
            self.pfloat(
                "blind_hold_s"
            )
        ):

            mode = "blind_hold"

            v_des = self.pfloat(
                "v_blind"
            )

            omega_des = 0.0

        else:

            mode = "stop"

            v_des = 0.0
            omega_des = 0.0

        # ------------------------------------------------------------
        # V slew
        # ------------------------------------------------------------

        if v_des >= self.v_ref_prev:

            v_rate = self.pfloat(
                "v_ref_rate_up"
            )

        else:

            v_rate = self.pfloat(
                "v_ref_rate_down"
            )

        v_ref = approach(
            self.v_ref_prev,
            v_des,
            v_rate
            *
            dt
        )

        # ------------------------------------------------------------
        # Omega slew
        # ------------------------------------------------------------

        zone = info[
            "curve_zone"
        ]

        if zone == "center":

            omega_rate = self.pfloat(
                "omega_rate_center"
            )

        elif zone == "straight":

            omega_rate = self.pfloat(
                "omega_rate_straight"
            )

        elif zone == "gentle":

            omega_rate = self.pfloat(
                "omega_rate_gentle"
            )

        elif zone == "medium":

            omega_rate = self.pfloat(
                "omega_rate_medium"
            )

        else:

            omega_rate = self.pfloat(
                "omega_rate_sharp"
            )

        # Chậm đổi dấu để tránh trái-phải liên tục.
        if (
            self.omega_ref_prev
            *
            omega_des
            <
            0.0
            and
            abs(
                self.omega_ref_prev
            )
            >
            0.01
            and
            abs(
                omega_des
            )
            >
            0.01
        ):

            omega_rate = min(
                omega_rate,
                self.pfloat(
                    "omega_reverse_rate"
                )
            )

        omega_ref = approach(
            self.omega_ref_prev,
            omega_des,
            omega_rate
            *
            dt
        )

        self.v_ref_prev = v_ref
        self.omega_ref_prev = omega_ref

        # ------------------------------------------------------------
        # Calibration
        # ------------------------------------------------------------

        v_cmd_target = v_ref
        omega_cmd_target = omega_ref

        if self.pbool(
            "enable_calibration"
        ):

            v_cmd_target = (
                v_cmd_target
                /
                max(
                    self.pfloat(
                        "linear_cmd_scale"
                    ),
                    0.001
                )
            )

            # ====================================================
            # DYNAMIC ANGULAR OUTPUT GAIN
            #
            # Straight:
            #     steering output nhẹ để tránh lắc.
            #
            # Curve / recovery:
            #     tăng output để xe quay sớm hơn,
            #     không đợi đến khi đã ra khỏi lane.
            # ====================================================

            if zone == "center":

                angular_zone_gain = self.pfloat(
                    "angular_zone_gain_center"
                )

            elif zone == "straight":

                angular_zone_gain = self.pfloat(
                    "angular_zone_gain_straight"
                )

            elif zone == "gentle":

                angular_zone_gain = self.pfloat(
                    "angular_zone_gain_gentle"
                )

            elif zone == "medium":

                angular_zone_gain = self.pfloat(
                    "angular_zone_gain_medium"
                )

            elif zone == "sharp":

                angular_zone_gain = self.pfloat(
                    "angular_zone_gain_sharp"
                )

            elif zone == "large_error":

                angular_zone_gain = self.pfloat(
                    "angular_zone_gain_large"
                )

            else:

                angular_zone_gain = 1.0

            omega_cmd_target = (
                omega_cmd_target
                *
                self.pfloat(
                    "angular_cmd_scale"
                )
                *
                angular_zone_gain
            )

        # ------------------------------------------------------------
        # Constraints
        # ------------------------------------------------------------

        (
            v_cmd_target,
            omega_cmd_target,
            pivot_mode
        ) = self.apply_no_pivot_limit(
            v_cmd_target,
            omega_cmd_target
        )

        (
            v_cmd_target,
            omega_cmd_target,
            lidar_stop,
            lidar_mode
        ) = self.apply_lidar_safety(
            v_cmd_target,
            omega_cmd_target
        )

        if mode == "stop":

            v_cmd_target = 0.0
            omega_cmd_target = 0.0

        # ------------------------------------------------------------
        # Final command smoothing
        #
        # Không lọc thêm quá nặng.
        # ------------------------------------------------------------

        v_cmd = approach(
            self.v_cmd_prev,
            v_cmd_target,
            v_rate
            *
            dt
        )

        omega_cmd = approach(
            self.omega_cmd_prev,
            omega_cmd_target,
            omega_rate
            *
            dt
        )

        if lidar_stop:
            v_cmd = 0.0
            omega_cmd = 0.0

        self.v_cmd_prev = v_cmd
        self.omega_cmd_prev = omega_cmd

        # ------------------------------------------------------------
        # Publisher conflict
        # ------------------------------------------------------------

        conflict, publishers = (
            self.cmd_vel_conflict_detected()
        )

        cmd_published = False

        if (
            self.pbool(
                "enable_cmd"
            )
            and
            not conflict
        ):

            self.cmd_pub.publish(
                self.make_cmd(
                    v_cmd,
                    omega_cmd
                )
            )

            cmd_published = True

            publish_reason = (
                "published"
            )

        elif conflict:

            publish_reason = (
                "cmd_vel_conflict"
            )

            if self.pbool(
                "publish_zero_on_conflict"
            ):

                self.cmd_pub.publish(
                    self.make_cmd(
                        0.0,
                        0.0
                    )
                )

        else:

            publish_reason = (
                "enable_cmd_false"
            )

        # ------------------------------------------------------------
        # Wheel-group estimate
        # ------------------------------------------------------------

        B = self.pfloat(
            "track_width_m"
        )

        R = self.pfloat(
            "wheel_radius_m"
        )

        v_left = (
            v_cmd
            -
            0.5
            *
            B
            *
            omega_cmd
        )

        v_right = (
            v_cmd
            +
            0.5
            *
            B
            *
            omega_cmd
        )

        # ------------------------------------------------------------
        # STATE
        # ------------------------------------------------------------

        state = {
            "node":
                "pd_backstepping_controller_v2",

            "version":
                self.VERSION,

            "time":
                time.time(),

            "mode":
                mode,

            "curve_zone":
                zone,

            "enabled":
                self.pbool(
                    "enable_cmd"
                ),

            "cmd_published":
                cmd_published,

            "publish_reason":
                publish_reason,

            "cmd_vel_conflict":
                conflict,

            "cmd_vel_publishers":
                publishers,

            "raw_valid":
                self.raw_valid,

            "raw_reason":
                self.raw_reason,

            "lane_state":
                self.raw_lane_state,

            "confidence":
                self.raw_confidence,

            "msg_age_s":
                msg_age,

            "valid_age_s":
                valid_age,

            "fps_est":
                self.fps_est,

            # Error
            "e_y_raw_m":
                self.raw_e_y,

            "e_y_raw_mm":
                self.raw_e_y
                *
                1000.0,

            "e_y_f_m":
                self.e_y_f,

            "e_y_f_mm":
                self.e_y_f
                *
                1000.0,

            "e_y_used_m":
                self.e_y_used,

            "e_y_used_mm":
                self.e_y_used
                *
                1000.0,

            "theta_raw_rad":
                self.raw_e_theta,

            "theta_f_rad":
                self.e_theta_f,

            "theta_used_rad":
                self.e_theta_used,

            "de_y":
                self.de_y_f,

            "de_theta":
                self.de_theta_f,

            # Backstepping
            "theta_virtual":
                info[
                    "theta_virtual"
                ],

            "e_theta_bs":
                info[
                    "e_theta_bs"
                ],

            "omega_ff":
                info[
                    "omega_ff"
                ],

            "p_y":
                info[
                    "p_y"
                ],

            "d_y":
                info[
                    "d_y"
                ],

            "p_theta":
                info[
                    "p_theta"
                ],

            "d_theta":
                info[
                    "d_theta"
                ],

            "feedback_gain":
                info[
                    "feedback_gain"
                ],

            # Geometry
            "lookahead_m":
                info[
                    "lookahead"
                ],

            "raw_curvature":
                self.raw_curvature,

            "curvature":
                info[
                    "curvature"
                ],

            "error_curvature":
                info[
                    "error_curvature"
                ],

            "severity_raw":
                info[
                    "severity_raw"
                ],

            "severity":
                info[
                    "severity"
                ],

            # Speed
            "speed_cap":
                info[
                    "speed_cap"
                ],

            "v_des":
                v_des,

            "v_ref":
                v_ref,

            "v_cmd":
                v_cmd,

            # Steering
            "omega_raw":
                info[
                    "omega_raw"
                ],

            "omega_des":
                omega_des,

            "omega_limit":
                info[
                    "omega_limit"
                ],

            "omega_ref":
                omega_ref,

            "omega_cmd":
                omega_cmd,

            "angular_zone_gain":
                (
                    angular_zone_gain
                    if
                    self.pbool(
                        "enable_calibration"
                    )
                    else
                    1.0
                ),

            # Wheels
            "v_left_cmd":
                v_left,

            "v_right_cmd":
                v_right,

            "wheel_left_radps":
                v_left
                /
                max(
                    R,
                    0.001
                ),

            "wheel_right_radps":
                v_right
                /
                max(
                    R,
                    0.001
                ),

            # Odom
            "odom_x":
                self.odom_x,

            "odom_y":
                self.odom_y,

            "odom_yaw":
                self.odom_yaw,

            "odom_v":
                self.odom_v,

            "odom_omega":
                self.odom_omega,

            # Safety
            "center_latched":
                self.center_latched,

            "branch_hold":
                branch_hold,

            "branch_reason":
                self.branch_hold_reason,

            "control_error_frozen":
                self.control_error_frozen,

            "freeze_age_s":
                self.freeze_age_s,

            "pivot_mode":
                pivot_mode,

            "lidar_stop":
                lidar_stop,

            "lidar_mode":
                lidar_mode,

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
        }

        msg = String()

        msg.data = json.dumps(
            state,
            ensure_ascii=False
        )

        self.state_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        PDBacksteppingControllerV2()
    )

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
