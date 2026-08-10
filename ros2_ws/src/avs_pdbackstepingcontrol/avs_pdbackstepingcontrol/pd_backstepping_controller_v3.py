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

    cosy_cosp = (
        1.0
        -
        2.0
        *
        (
            q.y * q.y
            +
            q.z * q.z
        )
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


class PDBacksteppingControllerV3(Node):

    VERSION = "pd_backstepping_v3_lane_tracking_3_0"

    def __init__(self):

        super().__init__(
            "pd_backstepping_controller_v3"
        )

        # ============================================================
        # V3.0
        #
        # Mục tiêu:
        #
        # - không freeze
        # - không branch hold
        # - không pre-turn steering
        # - curvature KHÔNG tạo omega
        # - Backstepping + PD chịu trách nhiệm steering
        # - curvature chỉ dùng speed scheduling
        # - derivative cập nhật theo frame vision
        # - chỉ một tầng angular slew
        # ============================================================

        parameters = {

            # --------------------------------------------------------
            # TOPICS
            # --------------------------------------------------------

            "control_error_topic":
                "/avs/control_error",

            "cmd_vel_topic":
                "/cmd_vel",

            "state_topic":
                "/avs/pd_backstepping_v3_state",

            "scan_topic":
                "/scan",

            "odom_topic":
                "/odom_raw",

            # --------------------------------------------------------
            # RUNTIME
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
            # PERCEPTION TIMING
            # --------------------------------------------------------

            "fresh_s":
                0.65,

            "blind_hold_s":
                0.30,

            "startup_straight_s":
                0.25,

            "startup_v":
                0.10,

            # --------------------------------------------------------
            # SHUTDOWN
            # --------------------------------------------------------

            "stop_burst_count":
                20,

            "stop_burst_dt":
                0.015,

            # --------------------------------------------------------
            # SIGN
            #
            # epsilon_x > 0 : target bên phải
            # ROS angular.z < 0 : quay phải
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
            # INPUT VALIDITY
            # --------------------------------------------------------

            "min_confidence":
                0.15,

            "max_abs_e_y_m":
                0.45,

            "max_abs_theta_rad":
                1.20,

            # Giá trị thực sự được đưa vào controller.
            "control_clip_e_y_m":
                0.22,

            "control_clip_theta_rad":
                0.70,

            # --------------------------------------------------------
            # INPUT STEP LIMIT
            #
            # KHÔNG freeze frame.
            #
            # Nếu perception nhảy mạnh:
            # chỉ giới hạn bước thay đổi mỗi frame.
            # --------------------------------------------------------

            "max_lat_step_per_frame_m":
                0.060,

            "max_theta_step_per_frame_rad":
                0.20,

            # --------------------------------------------------------
            # FPS
            #
            # 5 FPS * 0.045 = 0.225 m/s
            # 6 FPS * 0.045 = 0.270 m/s
            # --------------------------------------------------------

            "fps_init":
                6.0,

            "fps_min":
                2.0,

            "fps_max":
                30.0,

            "fps_tau_s":
                0.45,

            "target_distance_per_frame_m":
                0.045,

            # --------------------------------------------------------
            # ERROR FILTER
            # --------------------------------------------------------

            "median_window":
                3,

            "error_filter_tau_s":
                0.16,

            "derivative_filter_tau_s":
                0.32,

            "derivative_limit_y_mps":
                0.32,

            "derivative_limit_theta_rps":
                1.15,

            # --------------------------------------------------------
            # DEADBAND / CENTER
            # --------------------------------------------------------

            "x_deadband_m":
                0.010,

            "theta_deadband_rad":
                0.020,

            "center_x_m":
                0.026,

            "center_theta_rad":
                0.055,

            "center_release_x_m":
                0.045,

            "center_release_theta_rad":
                0.085,

            # --------------------------------------------------------
            # RECOVERY
            #
            # Đây KHÔNG phải curve mode.
            # Chỉ dùng khi thật sự lệch lớn.
            # --------------------------------------------------------

            "recovery_x_m":
                0.130,

            "recovery_theta_rad":
                0.50,

            # --------------------------------------------------------
            # SPEED PROFILE
            #
            # Speed reference vật lý mong muốn.
            # --------------------------------------------------------

            "v_max":
                0.28,

            "v_center":
                0.260,

            "v_straight":
                0.230,

            "v_gentle":
                0.190,

            "v_medium":
                0.150,

            "v_sharp":
                0.110,

            "v_recovery":
                0.100,

            "v_blind":
                0.070,

            "v_min":
                0.100,

            # --------------------------------------------------------
            # LINEAR SLEW
            # --------------------------------------------------------

            "v_ref_rate_up":
                0.18,

            "v_ref_rate_down":
                0.55,

            # --------------------------------------------------------
            # LOOKAHEAD
            #
            # Chỉ log.
            # KHÔNG steering.
            # --------------------------------------------------------

            "lookahead_default_m":
                0.42,

            "lookahead_min_m":
                0.20,

            "lookahead_max_m":
                0.90,

            # --------------------------------------------------------
            # CURVATURE
            #
            # CHỈ SPEED SCHEDULING.
            # --------------------------------------------------------

            "curvature_filter_tau_s":
                0.25,

            "max_abs_curvature":
                4.0,

            # --------------------------------------------------------
            # CURVE SEVERITY
            #
            # heading là tín hiệu chính.
            # curvature chỉ hỗ trợ giảm tốc.
            #
            # lateral weight thấp vì lateral error lớn có thể
            # chỉ là xe bị lệch, không có nghĩa đường đang cong.
            # --------------------------------------------------------

            "severity_lat_scale_m":
                0.120,

            "severity_heading_scale_rad":
                0.35,

            "severity_curvature_scale":
                2.50,

            "severity_lat_weight":
                0.15,

            "severity_heading_weight":
                0.55,

            "severity_curvature_weight":
                0.30,

            "severity_filter_tau_s":
                0.50,

            "severity_gentle":
                0.18,

            "severity_medium":
                0.45,

            "severity_sharp":
                0.78,

            # --------------------------------------------------------
            # BACKSTEPPING + PD
            #
            # theta_virtual = atan(lambda_y * e_y)
            #
            # e_theta_bs =
            #     theta + theta_virtual
            #
            # omega =
            #   - k_y       * e_y
            #   - k_dy      * de_y
            #   - k_theta   * e_theta_bs
            #   - k_dtheta  * de_theta_bs
            #
            # KHÔNG omega_ff.
            # KHÔNG Pure Pursuit.
            # KHÔNG curvature steering.
            # --------------------------------------------------------

            "lambda_y":
                0.78,

            "k_y":
                0.55,

            "k_dy":
                0.35,

            "k_theta":
                0.42,

            "k_dtheta_bs":
                0.30,

            # --------------------------------------------------------
            # OMEGA LIMIT
            # --------------------------------------------------------

            "omega_straight_max":
                0.130,

            "omega_gentle_max":
                0.200,

            "omega_medium_max":
                0.550,

            "omega_sharp_max":
                0.800,

            "omega_recovery_max":
                0.800,

            "omega_abs_max":
                0.900,

            "omega_deadband":
                0.006,

            # --------------------------------------------------------
            # ANGULAR SLEW
            #
            # Một tầng duy nhất.
            # --------------------------------------------------------

            "omega_rate_release":
                0.85,

            "omega_rate_straight":
                0.40,

            "omega_rate_gentle":
                0.48,

            "omega_rate_medium":
                0.58,

            "omega_rate_sharp":
                0.62,

            "omega_rate_recovery":
                0.65,

            "omega_reverse_rate":
                0.32,

            # --------------------------------------------------------
            # CALIBRATION
            # --------------------------------------------------------

            "enable_calibration":
                True,

            "linear_cmd_scale":
                1.245,

            "angular_cmd_scale":
                0.78,

            # --------------------------------------------------------
            # SKID STEER
            # --------------------------------------------------------

            "track_width_m":
                0.135,

            "wheel_radius_m":
                0.0225,

            "allow_pivot_turn":
                False,

            "inner_wheel_min_fraction":
                0.30,

            # --------------------------------------------------------
            # LIDAR
            #
            # Chỉ safety stop.
            # Không avoidance steering.
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
        }

        # ============================================================
        # DECLARE
        # ============================================================

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
        # PUB / SUB
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

        self.last_msg_time = -1.0
        self.last_valid_time = -1.0
        self.first_valid_time = -1.0

        # ============================================================
        # FPS
        # ============================================================

        self.last_frame_time = None
        self.last_valid_frame_time = None

        self.fps_est = self.pfloat(
            "fps_init"
        )

        # ============================================================
        # FILTER
        # ============================================================

        n = max(
            1,
            self.pint(
                "median_window"
            )
        )

        self.e_y_buffer = deque(
            maxlen=n
        )

        self.e_theta_buffer = deque(
            maxlen=n
        )

        self.e_y_f = 0.0
        self.e_theta_f = 0.0

        self.e_y_used = 0.0
        self.e_theta_used = 0.0

        self.de_y_f = 0.0
        self.de_theta_f = 0.0

        self.prev_frame_e_y_used = 0.0
        self.prev_frame_e_theta_used = 0.0

        self.filter_initialized = False

        # ============================================================
        # STEP-LIMITED INPUT
        #
        # Đây thay cho freeze / branch hold.
        # ============================================================

        self.accepted_e_y = None
        self.accepted_e_theta = None

        # ============================================================
        # STATE
        # ============================================================

        self.center_latched = False

        self.curvature_f = 0.0
        self.severity_f = 0.0

        # ============================================================
        # CONTROL
        #
        # Chỉ một tầng reference smoothing.
        # ============================================================

        self.v_ref = 0.0
        self.omega_ref = 0.0

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
        # TIMER
        # ============================================================

        self.prev_loop_time = (
            time.monotonic()
        )

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
            "Steering: Backstepping + PD only"
        )

        self.get_logger().info(
            "Curvature steering / pre-turn: OFF"
        )

        self.get_logger().info(
            "Freeze / branch hold: REMOVED"
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
    def make_cmd(
        v,
        omega
    ):

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

    def publish_stop_burst(
        self
    ):

        stop = self.make_cmd(
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
                stop
            )

            time.sleep(
                delay
            )

    # ================================================================
    # CMD_VEL CONFLICT
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

            return (
                False,
                []
            )

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
    # INPUT PARSER
    # ================================================================

    def extract_errors(
        self,
        data
    ):

        # ------------------------------------------------------------
        # LATERAL
        # ------------------------------------------------------------

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
                    data.get(
                        key
                    )
                )

                if e_y is not None:
                    break

        if e_y is None:

            for key in (
                "epsilon_x_mm",
                "x_mm",
                "e_y_mm",
                "e_lat_mm",
            ):

                value = finite_float(
                    data.get(
                        key
                    )
                )

                if value is not None:

                    e_y = (
                        value
                        /
                        1000.0
                    )

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

        # ------------------------------------------------------------
        # HEADING
        # ------------------------------------------------------------

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
                    data.get(
                        key
                    )
                )

                if e_theta is not None:
                    break

        if e_theta is None:
            e_theta = 0.0

        e_theta *= self.pfloat(
            "theta_sign"
        )

        # ------------------------------------------------------------
        # LOOKAHEAD
        #
        # Logger only.
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
                    data.get(
                        key
                    )
                )

                if lookahead is not None:
                    break

        if lookahead is None:

            for key in (
                "lookahead_d_mm",
                "epsilon_y_mm",
                "target_y_mm",
            ):

                value = finite_float(
                    data.get(
                        key
                    )
                )

                if value is not None:

                    lookahead = (
                        value
                        /
                        1000.0
                    )

                    break

        if lookahead is None:

            lookahead = self.pfloat(
                "lookahead_default_m"
            )

        lookahead = clamp(
            abs(
                lookahead
            ),
            self.pfloat(
                "lookahead_min_m"
            ),
            self.pfloat(
                "lookahead_max_m"
            )
        )

        # ------------------------------------------------------------
        # CURVATURE
        #
        # SPEED ONLY.
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
                        data.get(
                            key
                        )
                    )

                    if curvature is not None:
                        break

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
            curvature
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

        confidence = clamp(
            confidence,
            0.0,
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
            abs(
                e_y
            )
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
            abs(
                e_theta
            )
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
            (
                "ok"
                if valid
                else
                "invalid"
            )
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

            frame_dt = (
                1.0
                /
                max(
                    fps_now,
                    0.001
                )
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
    # STEP LIMIT
    #
    # Không reject/freeze frame.
    # Chỉ không cho error teleport quá xa trong 1 frame.
    # ================================================================

    def step_limit_measurement(
        self,
        e_y,
        e_theta
    ):

        if self.accepted_e_y is None:

            self.accepted_e_y = e_y
            self.accepted_e_theta = e_theta

            return (
                e_y,
                e_theta
            )

        max_y = self.pfloat(
            "max_lat_step_per_frame_m"
        )

        max_theta = self.pfloat(
            "max_theta_step_per_frame_rad"
        )

        e_y_limited = clamp(
            e_y,
            self.accepted_e_y
            -
            max_y,
            self.accepted_e_y
            +
            max_y
        )

        e_theta_limited = clamp(
            e_theta,
            self.accepted_e_theta
            -
            max_theta,
            self.accepted_e_theta
            +
            max_theta
        )

        self.accepted_e_y = (
            e_y_limited
        )

        self.accepted_e_theta = (
            e_theta_limited
        )

        return (
            e_y_limited,
            e_theta_limited
        )

    # ================================================================
    # VISION CALLBACK
    #
    # Derivative tính đúng theo frame mới.
    # ================================================================

    def control_error_callback(
        self,
        msg
    ):

        now = (
            time.monotonic()
        )

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
            curvature
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

        self.raw_confidence = (
            confidence
        )

        self.raw_lane_state = (
            lane_state
        )

        self.raw_e_y = e_y
        self.raw_e_theta = e_theta

        self.raw_lookahead = (
            lookahead
        )

        self.raw_curvature = (
            curvature
        )

        self.last_msg_time = (
            now
        )

        if not valid:
            return

        if self.first_valid_time < 0.0:
            self.first_valid_time = now

        self.last_valid_time = now

        # ------------------------------------------------------------
        # Smooth sudden measurement jumps.
        #
        # No freeze.
        # ------------------------------------------------------------

        (
            e_y,
            e_theta
        ) = self.step_limit_measurement(
            e_y,
            e_theta
        )

        e_y = clamp(
            e_y,
            -self.pfloat(
                "control_clip_e_y_m"
            ),
            self.pfloat(
                "control_clip_e_y_m"
            )
        )

        e_theta = clamp(
            e_theta,
            -self.pfloat(
                "control_clip_theta_rad"
            ),
            self.pfloat(
                "control_clip_theta_rad"
            )
        )

        # ------------------------------------------------------------
        # MEDIAN
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
                )[
                    -n:
                ],
                maxlen=n
            )

            self.e_theta_buffer = deque(
                list(
                    self.e_theta_buffer
                )[
                    -n:
                ],
                maxlen=n
            )

        self.e_y_buffer.append(
            e_y
        )

        self.e_theta_buffer.append(
            e_theta
        )

        target_y = statistics.median(
            self.e_y_buffer
        )

        target_theta = statistics.median(
            self.e_theta_buffer
        )

        # ------------------------------------------------------------
        # FRAME DT
        # ------------------------------------------------------------

        if self.last_valid_frame_time is None:

            dt = (
                1.0
                /
                max(
                    self.fps_est,
                    4.0
                )
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
        # ERROR FILTER
        #
        # First frame initializes directly.
        # ------------------------------------------------------------

        if not self.filter_initialized:

            self.e_y_f = target_y
            self.e_theta_f = target_theta

            self.filter_initialized = True

        else:

            alpha = self.alpha_from_tau(
                dt,
                self.pfloat(
                    "error_filter_tau_s"
                )
            )

            self.e_y_f = (
                (
                    1.0
                    -
                    alpha
                )
                *
                self.e_y_f
                +
                alpha
                *
                target_y
            )

            self.e_theta_f = (
                (
                    1.0
                    -
                    alpha
                )
                *
                self.e_theta_f
                +
                alpha
                *
                target_theta
            )

        # ------------------------------------------------------------
        # CENTER HYSTERESIS
        # ------------------------------------------------------------

        if self.center_latched:

            if (
                abs(
                    self.e_y_f
                )
                >
                self.pfloat(
                    "center_release_x_m"
                )
                or
                abs(
                    self.e_theta_f
                )
                >
                self.pfloat(
                    "center_release_theta_rad"
                )
            ):

                self.center_latched = (
                    False
                )

        elif (
            abs(
                self.e_y_f
            )
            <=
            self.pfloat(
                "center_x_m"
            )
            and
            abs(
                self.e_theta_f
            )
            <=
            self.pfloat(
                "center_theta_rad"
            )
        ):

            self.center_latched = (
                True
            )

        # ------------------------------------------------------------
        # USED ERROR
        # ------------------------------------------------------------

        self.e_y_used = (
            0.0
            if
            abs(
                self.e_y_f
            )
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
            abs(
                self.e_theta_f
            )
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
            (
                1.0
                -
                d_alpha
            )
            *
            self.de_y_f
            +
            d_alpha
            *
            de_y_raw
        )

        self.de_theta_f = (
            (
                1.0
                -
                d_alpha
            )
            *
            self.de_theta_f
            +
            d_alpha
            *
            de_theta_raw
        )

        # ------------------------------------------------------------
        # CURVATURE FILTER
        #
        # SPEED ONLY.
        # ------------------------------------------------------------

        c_alpha = self.alpha_from_tau(
            dt,
            self.pfloat(
                "curvature_filter_tau_s"
            )
        )

        self.curvature_f = (
            (
                1.0
                -
                c_alpha
            )
            *
            self.curvature_f
            +
            c_alpha
            *
            curvature
        )

        # ------------------------------------------------------------
        # SEVERITY FILTER
        # ------------------------------------------------------------

        severity_raw = (
            self.compute_severity_raw()
        )

        s_alpha = self.alpha_from_tau(
            dt,
            self.pfloat(
                "severity_filter_tau_s"
            )
        )

        if severity_raw > self.severity_f:
            self.severity_f = severity_raw
        else:
            self.severity_f = (
                (
                    1.0
                    -
                    s_alpha
                )
                *
                self.severity_f
                +
                s_alpha
                *
                severity_raw
            )

    # ================================================================
    # ODOM
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

    # ================================================================
    # LIDAR
    # ================================================================

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
                math.isfinite(
                    distance
                )
                and
                msg.range_min
                <=
                distance
                <=
                msg.range_max
                and
                abs(
                    angle
                )
                <=
                front_angle
            ):

                values.append(
                    float(
                        distance
                    )
                )

            angle += msg.angle_increment

        self.front_min = (
            min(
                values
            )
            if
            values
            else
            math.inf
        )

        self.last_scan_time = (
            time.monotonic()
        )

    # ================================================================
    # SPEED
    # ================================================================

    def speed_from_fps(
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
    # CURVE SEVERITY
    #
    # Không dùng để steering.
    # ================================================================

    def compute_severity_raw(
        self
    ):

        lat = (
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
        )

        heading = (
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
        )

        curvature = (
            abs(
                self.curvature_f
            )
            /
            max(
                self.pfloat(
                    "severity_curvature_scale"
                ),
                0.001
            )
        )

        raw = (
            self.pfloat(
                "severity_lat_weight"
            )
            *
            lat

            +

            self.pfloat(
                "severity_heading_weight"
            )
            *
            heading

            +

            self.pfloat(
                "severity_curvature_weight"
            )
            *
            curvature
        )

        return clamp(
            raw,
            0.0,
            1.5
        )

    @staticmethod
    def lerp(
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
            (
                b
                -
                a
            )
            *
            ratio
        )

    # ================================================================
    # SPEED ZONE
    # ================================================================

    def speed_zone(
        self
    ):

        # ------------------------------------------------------------
        # RECOVERY
        #
        # Lệch thật sự lớn.
        # ------------------------------------------------------------

        if (
            abs(
                self.e_y_used
            )
            >=
            self.pfloat(
                "recovery_x_m"
            )
            or
            abs(
                self.e_theta_used
            )
            >=
            self.pfloat(
                "recovery_theta_rad"
            )
        ):

            return (
                self.pfloat(
                    "v_recovery"
                ),
                "recovery"
            )

        # ------------------------------------------------------------
        # CENTER
        # ------------------------------------------------------------

        if self.center_latched:

            return (
                self.pfloat(
                    "v_center"
                ),
                "center"
            )

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

        # ------------------------------------------------------------
        # STRAIGHT -> GENTLE
        # ------------------------------------------------------------

        if s <= s1:

            ratio = (
                s
                /
                max(
                    s1,
                    0.001
                )
            )

            return (
                self.lerp(
                    self.pfloat(
                        "v_straight"
                    ),
                    self.pfloat(
                        "v_gentle"
                    ),
                    ratio
                ),
                "straight"
            )

        # ------------------------------------------------------------
        # GENTLE -> MEDIUM
        # ------------------------------------------------------------

        if s <= s2:

            ratio = (
                s
                -
                s1
            ) / max(
                s2
                -
                s1,
                0.001
            )

            return (
                self.lerp(
                    self.pfloat(
                        "v_gentle"
                    ),
                    self.pfloat(
                        "v_medium"
                    ),
                    ratio
                ),
                "gentle"
            )

        # ------------------------------------------------------------
        # MEDIUM -> SHARP
        # ------------------------------------------------------------

        if s <= s3:

            ratio = (
                s
                -
                s2
            ) / max(
                s3
                -
                s2,
                0.001
            )

            return (
                self.lerp(
                    self.pfloat(
                        "v_medium"
                    ),
                    self.pfloat(
                        "v_sharp"
                    ),
                    ratio
                ),
                "medium"
            )

        return (
            self.pfloat(
                "v_sharp"
            ),
            "sharp"
        )

    # ================================================================
    # BACKSTEPPING + PD
    #
    # NO PRE-TURN.
    # NO CURVATURE FF.
    # ================================================================

    def compute_tracking(
        self
    ):

        speed_cap = (
            self.speed_from_fps()
        )

        (
            v_profile,
            zone
        ) = self.speed_zone()

        v_des = min(
            speed_cap,
            v_profile
        )

        # ------------------------------------------------------------
        # BACKSTEPPING VIRTUAL HEADING
        # ------------------------------------------------------------

        lam = self.pfloat(
            "lambda_y"
        )

        theta_virtual = math.atan(
            lam
            *
            self.e_y_used
        )

        e_theta_bs = (
            self.e_theta_used
            +
            theta_virtual
        )

        # ------------------------------------------------------------
        # DERIVATIVE OF VIRTUAL HEADING
        #
        # d/dt atan(lambda * e_y)
        # ------------------------------------------------------------

        dtheta_virtual = (
            lam
            /
            (
                1.0
                +
                (
                    lam
                    *
                    self.e_y_used
                )
                **
                2
            )
            *
            self.de_y_f
        )

        de_theta_bs = (
            self.de_theta_f
            +
            dtheta_virtual
        )

        # ------------------------------------------------------------
        # PD COMPONENTS
        # ------------------------------------------------------------

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
                "k_dtheta_bs"
            )
            *
            de_theta_bs
        )

        # ============================================================
        # NO FEED-FORWARD TURN
        # ============================================================

        omega_ff = 0.0

        omega_raw = (
            p_y
            +
            d_y
            +
            p_theta
            +
            d_theta
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

            p_y = -p_y
            d_y = -d_y
            p_theta = -p_theta
            d_theta = -d_theta

        # ------------------------------------------------------------
        # CENTER HARD ZERO
        # ------------------------------------------------------------

        if self.center_latched:

            omega_limit = 0.0
            omega_des = 0.0

        else:

            # --------------------------------------------------------
            # LIMIT BY ZONE
            # --------------------------------------------------------

            if zone == "straight":

                omega_limit = self.pfloat(
                    "omega_straight_max"
                )

            elif zone == "gentle":

                omega_limit = self.pfloat(
                    "omega_gentle_max"
                )

            elif zone == "medium":

                omega_limit = self.pfloat(
                    "omega_medium_max"
                )

            elif zone == "sharp":

                omega_limit = self.pfloat(
                    "omega_sharp_max"
                )

            else:

                omega_limit = self.pfloat(
                    "omega_recovery_max"
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
                abs(
                    omega_des
                )
                <
                self.pfloat(
                    "omega_deadband"
                )
            ):

                omega_des = 0.0

        return {

            "curve_zone":
                zone,

            "speed_cap":
                speed_cap,

            "v_des":
                v_des,

            "omega_ff":
                omega_ff,

            "omega_raw":
                omega_raw,

            "omega_des":
                omega_des,

            "omega_limit":
                omega_limit,

            "theta_virtual":
                theta_virtual,

            "e_theta_bs":
                e_theta_bs,

            "de_theta_bs":
                de_theta_bs,

            "p_y":
                p_y,

            "d_y":
                d_y,

            "p_theta":
                p_theta,

            "d_theta":
                d_theta,

            "lookahead":
                self.raw_lookahead,

            "curvature":
                self.curvature_f,

            "severity_raw":
                self.compute_severity_raw(),

            "severity":
                self.severity_f,
        }

    # ================================================================
    # OMEGA SLEW
    # ================================================================

    def omega_rate_for_zone(
        self,
        zone
    ):

        if zone == "center":

            return self.pfloat(
                "omega_rate_release"
            )

        if zone == "straight":

            return self.pfloat(
                "omega_rate_straight"
            )

        if zone == "gentle":

            return self.pfloat(
                "omega_rate_gentle"
            )

        if zone == "medium":

            return self.pfloat(
                "omega_rate_medium"
            )

        if zone == "sharp":

            return self.pfloat(
                "omega_rate_sharp"
            )

        return self.pfloat(
            "omega_rate_recovery"
        )

    def slew_omega(
        self,
        current,
        target,
        zone,
        dt
    ):

        # ------------------------------------------------------------
        # RELEASE
        #
        # Khi steering đang quá lớn so với target mới:
        # nhả nhanh.
        # ------------------------------------------------------------

        if (
            abs(
                target
            )
            <
            abs(
                current
            )
            and
            current
            *
            target
            >=
            0.0
        ):

            rate = self.pfloat(
                "omega_rate_release"
            )

            return approach(
                current,
                target,
                rate
                *
                dt
            )

        # ------------------------------------------------------------
        # ZERO-CROSS
        #
        # Không nhảy thẳng từ trái sang phải.
        # Trước hết về zero.
        # ------------------------------------------------------------

        if (
            current
            *
            target
            <
            0.0
            and
            abs(
                current
            )
            >
            0.005
        ):

            rate = self.pfloat(
                "omega_reverse_rate"
            )

            return approach(
                current,
                0.0,
                rate
                *
                dt
            )

        rate = self.omega_rate_for_zone(
            zone
        )

        return approach(
            current,
            target,
            rate
            *
            dt
        )

    # ================================================================
    # NO PIVOT
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
            (
                1.0
                -
                fraction
            )
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

    # ================================================================
    # LIDAR
    # ================================================================

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
                v
                *
                ratio,
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
    # MAIN LOOP
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
            self.prev_loop_time,
            0.001,
            0.10
        )

        self.prev_loop_time = (
            now
        )

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

            "curve_zone":
                "none",

            "speed_cap":
                0.0,

            "v_des":
                0.0,

            "omega_ff":
                0.0,

            "omega_raw":
                0.0,

            "omega_des":
                0.0,

            "omega_limit":
                0.0,

            "theta_virtual":
                0.0,

            "e_theta_bs":
                0.0,

            "de_theta_bs":
                0.0,

            "p_y":
                0.0,

            "d_y":
                0.0,

            "p_theta":
                0.0,

            "d_theta":
                0.0,

            "lookahead":
                self.raw_lookahead,

            "curvature":
                self.curvature_f,

            "severity_raw":
                self.compute_severity_raw(),

            "severity":
                self.severity_f,
        }

        # ------------------------------------------------------------
        # STARTUP
        # ------------------------------------------------------------

        if startup:

            mode = "startup"
            zone = "center"

            v_des = self.pfloat(
                "startup_v"
            )

            omega_des = 0.0

        # ------------------------------------------------------------
        # NORMAL TRACKING
        # ------------------------------------------------------------

        elif fresh:

            mode = "tracking"

            info = (
                self.compute_tracking()
            )

            zone = info[
                "curve_zone"
            ]

            v_des = info[
                "v_des"
            ]

            omega_des = info[
                "omega_des"
            ]

        # ------------------------------------------------------------
        # SHORT SENSOR GAP
        #
        # Đây không phải freeze logic.
        # ------------------------------------------------------------

        elif (
            valid_age
            <=
            self.pfloat(
                "blind_hold_s"
            )
        ):

            mode = "blind_hold"
            zone = "center"

            v_des = self.pfloat(
                "v_blind"
            )

            omega_des = 0.0

        # ------------------------------------------------------------
        # REAL TIMEOUT
        # ------------------------------------------------------------

        else:

            mode = "stop"
            zone = "center"

            v_des = 0.0
            omega_des = 0.0

        # ============================================================
        # ONE LINEAR SLEW
        # ============================================================

        if (
            v_des
            >=
            self.v_ref
        ):

            v_rate = self.pfloat(
                "v_ref_rate_up"
            )

        else:

            v_rate = self.pfloat(
                "v_ref_rate_down"
            )

        self.v_ref = approach(
            self.v_ref,
            v_des,
            v_rate
            *
            dt
        )

        # ============================================================
        # ONE ANGULAR SLEW
        # ============================================================

        self.omega_ref = self.slew_omega(
            self.omega_ref,
            omega_des,
            zone,
            dt
        )

        # ============================================================
        # CALIBRATION
        #
        # Không có tầng smoothing thứ hai.
        # ============================================================

        v_cmd = self.v_ref
        omega_cmd = self.omega_ref

        if self.pbool(
            "enable_calibration"
        ):

            v_cmd = (
                v_cmd
                /
                max(
                    self.pfloat(
                        "linear_cmd_scale"
                    ),
                    0.001
                )
            )

            omega_cmd = (
                omega_cmd
                *
                self.pfloat(
                    "angular_cmd_scale"
                )
            )

        # ============================================================
        # PHYSICAL CONSTRAINTS
        # ============================================================

        (
            v_cmd,
            omega_cmd,
            pivot_mode
        ) = self.apply_no_pivot_limit(
            v_cmd,
            omega_cmd
        )

        (
            v_cmd,
            omega_cmd,
            lidar_stop,
            lidar_mode
        ) = self.apply_lidar_safety(
            v_cmd,
            omega_cmd
        )

        if mode == "stop":

            v_cmd = 0.0
            omega_cmd = 0.0

        # ============================================================
        # CMD_VEL SAFETY
        # ============================================================

        (
            conflict,
            publishers
        ) = self.cmd_vel_conflict_detected()

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
            publish_reason = "published"

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

        # ============================================================
        # WHEEL GROUP ESTIMATE
        # ============================================================

        B = self.pfloat(
            "track_width_m"
        )

        R = max(
            self.pfloat(
                "wheel_radius_m"
            ),
            0.001
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

        # ============================================================
        # STATE
        #
        # Giữ schema tương thích logger cũ.
        # ============================================================

        state = {

            "node":
                "pd_backstepping_controller_v3",

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

            # --------------------------------------------------------
            # PERCEPTION
            # --------------------------------------------------------

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

            # --------------------------------------------------------
            # ERROR
            # --------------------------------------------------------

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

            # --------------------------------------------------------
            # BACKSTEPPING
            # --------------------------------------------------------

            "theta_virtual":
                info[
                    "theta_virtual"
                ],

            "e_theta_bs":
                info[
                    "e_theta_bs"
                ],

            "de_theta_bs":
                info[
                    "de_theta_bs"
                ],

            # Always zero in V3.
            "omega_ff":
                0.0,

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

            # --------------------------------------------------------
            # GEOMETRY
            # --------------------------------------------------------

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

            "severity_raw":
                info[
                    "severity_raw"
                ],

            "severity":
                info[
                    "severity"
                ],

            # --------------------------------------------------------
            # LINEAR
            # --------------------------------------------------------

            "speed_cap":
                info[
                    "speed_cap"
                ],

            "v_des":
                v_des,

            "v_ref":
                self.v_ref,

            "v_cmd":
                v_cmd,

            # --------------------------------------------------------
            # ANGULAR
            # --------------------------------------------------------

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
                self.omega_ref,

            "omega_cmd":
                omega_cmd,

            # --------------------------------------------------------
            # WHEELS
            # --------------------------------------------------------

            "v_left_cmd":
                v_left,

            "v_right_cmd":
                v_right,

            "wheel_left_radps":
                (
                    v_left
                    /
                    R
                ),

            "wheel_right_radps":
                (
                    v_right
                    /
                    R
                ),

            # --------------------------------------------------------
            # ODOM
            # --------------------------------------------------------

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

            # --------------------------------------------------------
            # FLAGS
            # --------------------------------------------------------

            "center_latched":
                self.center_latched,

            # Compatibility with old logger.
            # V3 never activates these.
            "branch_hold":
                False,

            "control_error_frozen":
                False,

            "lidar_stop":
                lidar_stop,

            "lidar_mode":
                lidar_mode,

            "pivot_mode":
                pivot_mode,

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

        state_msg = String()

        state_msg.data = json.dumps(
            state,
            ensure_ascii=False
        )

        self.state_pub.publish(
            state_msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        PDBacksteppingControllerV3()
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
