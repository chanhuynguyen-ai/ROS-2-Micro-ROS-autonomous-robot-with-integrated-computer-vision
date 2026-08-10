#!/usr/bin/env python3

import argparse
import csv
import json
import math
import signal
import sys
import time

from collections import OrderedDict
from pathlib import Path


import matplotlib

if "--no-live" in sys.argv:
    matplotlib.use("Agg")

try:
    matplotlib.rcParams["figure.raise_window"] = False
except Exception:
    pass

import matplotlib.pyplot as plt


import rclpy

from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ================================================================
# CONTROLLERS
#
# V1 cố ý là "backsteping" một chữ p vì node cũ publish như vậy.
# ================================================================

CONTROLLERS = OrderedDict([
    (
        "v1",
        {
            "topic":
                "/avs/pd_backsteping_state",

            "title":
                "PD-Backstepping V1 - Center First",
        },
    ),

    (
        "v2",
        {
            "topic":
                "/avs/pd_backstepping_v2_state",

            "title":
                "PD-Backstepping V2 - Smooth Early",
        },
    ),

    (
        "v3",
        {
            "topic":
                "/avs/pd_backstepping_v3_state",

            "title":
                "PD-Backstepping V3 - Lane Tracking",
        },
    ),
])


# ================================================================
# HELPERS
# ================================================================

def now_s():

    return time.time()


def finite(
    value,
    default=math.nan
):

    try:

        value = float(value)

        if math.isfinite(value):
            return value

    except Exception:
        pass

    return default


def first_finite(
    *values,
    default=math.nan
):

    for value in values:

        result = finite(value)

        if math.isfinite(result):
            return result

    return default


def parse_json(text):

    try:

        value = json.loads(text)

        if isinstance(value, dict):
            return value

    except Exception:
        pass

    return {}


def bint(value):

    if isinstance(value, bool):
        return int(value)

    if isinstance(value, str):

        return (
            0
            if value.strip().lower()
            in {
                "",
                "0",
                "false",
                "no",
                "none",
                "invalid",
                "lost",
            }
            else
            1
        )

    if value is None:
        return 0

    return int(
        bool(value)
    )


def yaw_from_quaternion(q):

    return math.atan2(
        2.0
        *
        (
            q.w
            *
            q.z
            +
            q.x
            *
            q.y
        ),

        1.0
        -
        2.0
        *
        (
            q.y
            *
            q.y
            +
            q.z
            *
            q.z
        ),
    )


def csv_scalar(value):

    if value is None:
        return ""

    if isinstance(
        value,
        (
            str,
            int,
            float,
            bool,
        )
    ):
        return value

    try:

        return json.dumps(
            value,
            ensure_ascii=False,
            separators=(
                ",",
                ":"
            ),
        )

    except Exception:

        return str(value)


# ================================================================
# LOGGER
# ================================================================

class PDBacksteppingUnifiedLogger(Node):

    def __init__(
        self,
        args
    ):

        super().__init__(
            "PD_backstepping_logger"
        )

        self.a = args

        self.t0 = now_s()

        self.stop_requested = False
        self.window_closed = False

        self.last_log = 0.0
        self.last_plot = 0.0
        self.last_autosave = 0.0

        self.last_active = ""

        self.last_multi_warn = 0.0

        # ============================================================
        # INPUT STATE
        # ============================================================

        self.states = {
            name: {}
            for name
            in CONTROLLERS
        }

        self.cmd = {}
        self.odom = {}

        self.records = []
        self.raw_records = []

        # ============================================================
        # MAPS
        # ============================================================

        self.maps = {

            "mode":
                OrderedDict(),

            "detail":
                OrderedDict(),

            "zone":
                OrderedDict(),

            "lidar":
                OrderedDict(),

            "pivot":
                OrderedDict(),

            "lane":
                OrderedDict(),

            "reason":
                OrderedDict(),
        }

        # ============================================================
        # LOW LATENCY QoS
        # ============================================================

        self.latest_qos = QoSProfile(

            history=(
                QoSHistoryPolicy.KEEP_LAST
            ),

            depth=1,

            reliability=(
                QoSReliabilityPolicy.BEST_EFFORT
            ),
        )

        # ============================================================
        # AUTO-DETECT
        #
        # Subscribe cả V1 + V2 + V3.
        # ============================================================

        for (
            controller,
            info
        ) in CONTROLLERS.items():

            self.create_subscription(

                String,

                info[
                    "topic"
                ],

                lambda msg, c=controller:
                    self.state_callback(
                        c,
                        msg
                    ),

                self.latest_qos,
            )

        # ============================================================
        # CMD VEL
        # ============================================================

        self.create_subscription(

            Twist,

            args.cmd_vel_topic,

            self.cmd_callback,

            self.latest_qos,
        )

        # ============================================================
        # ODOM
        # ============================================================

        self.create_subscription(

            Odometry,

            args.odom_topic,

            self.odom_callback,

            qos_profile_sensor_data,
        )

        # ============================================================
        # OUTPUT
        # ============================================================

        stamp = time.strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

        self.run_name = (
            f"PD_backstepping_unified_{stamp}"
        )

        self.output_dir = (

            Path(
                args.output_dir
            )
            .expanduser()
            .resolve()

            /

            self.run_name
        )

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        self.csv_path = (
            self.output_dir
            /
            f"{self.run_name}.csv"
        )

        self.png_path = (
            self.output_dir
            /
            f"{self.run_name}.png"
        )

        self.map_path = (
            self.output_dir
            /
            "mode_map.json"
        )

        self.raw_path = (
            self.output_dir
            /
            "raw_state.jsonl"
        )

        # ============================================================
        # PLOT
        # ============================================================

        self.fig = None
        self.axes = None
        self.lines = None
        self.status_text = None

        if not args.no_live:

            plt.ion()

            (
                self.fig,
                self.axes,
                self.lines,
                self.status_text,
            ) = self.make_dashboard(
                "Waiting for PD-Backstepping V1/V2/V3..."
            )

            self.fig.canvas.mpl_connect(
                "close_event",
                self.on_close
            )

            plt.show(
                block=False
            )

            self.configure_no_focus()

        # ============================================================
        # INFO
        # ============================================================

        self.get_logger().info(
            "Unified PD-Backstepping V1/V2/V3 logger started"
        )

        for (
            controller,
            info
        ) in CONTROLLERS.items():

            self.get_logger().info(
                f"{controller}: "
                f"{info['topic']}"
            )

        self.get_logger().info(
            f"Output: {self.output_dir}"
        )

        self.get_logger().info(
            "Logger is subscribe-only; "
            "it never publishes /cmd_vel"
        )

    # ================================================================
    # AGE
    # ================================================================

    @staticmethod
    def age(data):

        if not isinstance(
            data,
            dict
        ):
            return math.inf

        rx = finite(
            data.get(
                "_rx"
            )
        )

        if not math.isfinite(rx):
            return math.inf

        return (
            now_s()
            -
            rx
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def state_callback(
        self,
        controller,
        msg
    ):

        data = parse_json(
            msg.data
        )

        if not data:
            return

        data[
            "_rx"
        ] = now_s()

        data[
            "_controller"
        ] = controller

        self.states[
            controller
        ] = data

        # ------------------------------------------------------------
        # Raw telemetry
        # ------------------------------------------------------------

        self.raw_records.append({

            "time_wall_s":
                now_s(),

            "controller":
                controller,

            "state":
                {
                    key: value

                    for key, value
                    in data.items()

                    if not key.startswith(
                        "_"
                    )
                },
        })

    def cmd_callback(
        self,
        msg
    ):

        self.cmd = {

            "v":
                float(
                    msg.linear.x
                ),

            "omega":
                float(
                    msg.angular.z
                ),

            "_rx":
                now_s(),
        }

    def odom_callback(
        self,
        msg
    ):

        self.odom = {

            "x":
                float(
                    msg.pose.pose.position.x
                ),

            "y":
                float(
                    msg.pose.pose.position.y
                ),

            "yaw":
                yaw_from_quaternion(
                    msg.pose.pose.orientation
                ),

            "v":
                float(
                    msg.twist.twist.linear.x
                ),

            "omega":
                float(
                    msg.twist.twist.angular.z
                ),

            "_rx":
                now_s(),
        }

    # ================================================================
    # AUTO-DETECT ACTIVE CONTROLLER
    # ================================================================

    def active_controller(self):

        # ------------------------------------------------------------
        # Forced version
        # ------------------------------------------------------------

        if (
            self.a.controller
            !=
            "auto"
        ):

            controller = (
                self.a.controller
            )

            state = self.states.get(
                controller,
                {}
            )

            if (
                state
                and
                self.age(state)
                <=
                self.a.active_timeout_s
                *
                3.0
            ):

                return controller

            return ""

        # ------------------------------------------------------------
        # AUTO
        # ------------------------------------------------------------

        fresh = [

            (
                state.get(
                    "_rx",
                    0.0
                ),
                controller
            )

            for (
                controller,
                state
            )
            in self.states.items()

            if (
                self.age(state)
                <=
                self.a.active_timeout_s
            )
        ]

        if not fresh:

            self.last_active = ""

            return ""

        fresh.sort(
            reverse=True
        )

        fresh_names = [
            controller
            for _, controller
            in fresh
        ]

        # ------------------------------------------------------------
        # Nếu controller hiện tại vẫn fresh thì giữ nó.
        #
        # Tránh dashboard nhấp nháy V1/V2 nếu vô tình chạy 2 node.
        # ------------------------------------------------------------

        if (
            self.last_active
            not in
            fresh_names
        ):

            self.last_active = (
                fresh[0][1]
            )

        # ------------------------------------------------------------
        # Multiple controller warning
        # ------------------------------------------------------------

        if (
            len(fresh) > 1
            and
            now_s()
            -
            self.last_multi_warn
            >
            3.0
        ):

            details = ", ".join(

                f"{controller}="
                f"{self.age(self.states[controller]):.2f}s"

                for _, controller
                in fresh
            )

            self.get_logger().warn(

                "MULTIPLE FRESH PD-BACKSTEPPING CONTROLLERS: "
                +
                details
                +
                "; dashboard shows "
                +
                self.last_active
            )

            self.last_multi_warn = (
                now_s()
            )

        return self.last_active

    def fresh_controllers(self):

        return [

            controller

            for (
                controller,
                state
            )
            in self.states.items()

            if (
                self.age(state)
                <=
                self.a.active_timeout_s
            )
        ]

    # ================================================================
    # CODE MAP
    # ================================================================

    def encode(
        self,
        group,
        value
    ):

        value = str(
            value
            or
            ""
        )

        if not value:
            return 0

        mapping = self.maps[
            group
        ]

        if value not in mapping:

            mapping[
                value
            ] = (
                len(mapping)
                +
                1
            )

        return mapping[
            value
        ]

    # ================================================================
    # NORMALIZED ROW
    # ================================================================

    def make_row(self):

        controller = (
            self.active_controller()
        )

        if not controller:
            return None

        state = self.states.get(
            controller,
            {}
        )

        if not state:
            return None

        cmd = self.cmd
        odom = self.odom

        # ============================================================
        # MODE
        # ============================================================

        mode = str(
            state.get(
                "mode",
                ""
            )
        )

        mode_detail = str(
            state.get(
                "mode_detail",
                ""
            )
        )

        zone = str(
            state.get(
                "curve_zone",
                ""
            )
        )

        # ------------------------------------------------------------
        # V1 không có curve_zone.
        # mode_detail chính là center_cruise/curve_tracking/...
        # ------------------------------------------------------------

        display_zone = (
            zone
            if zone
            else
            mode_detail
        )

        # ============================================================
        # COMMAND / ODOM
        # ============================================================

        cmd_v = finite(
            cmd.get(
                "v"
            )
        )

        cmd_omega = finite(
            cmd.get(
                "omega"
            )
        )

        odom_v = first_finite(

            state.get(
                "odom_v"
            ),

            odom.get(
                "v"
            ),
        )

        odom_omega = first_finite(

            state.get(
                "odom_omega"
            ),

            odom.get(
                "omega"
            ),
        )

        # ============================================================
        # LATERAL ERROR
        #
        # Normalize V1 vs V2/V3 schema.
        # ============================================================

        e_y_raw_mm = first_finite(

            state.get(
                "e_y_raw_mm"
            ),

            state.get(
                "e_y_mm"
            ),

            state.get(
                "epsilon_x_mm"
            ),
        )

        if not math.isfinite(
            e_y_raw_mm
        ):

            e_y_raw_m = first_finite(

                state.get(
                    "e_y_raw_m"
                ),

                state.get(
                    "e_y_m"
                ),
            )

            if math.isfinite(
                e_y_raw_m
            ):

                e_y_raw_mm = (
                    e_y_raw_m
                    *
                    1000.0
                )

        # ------------------------------------------------------------

        e_y_f_mm = first_finite(
            state.get(
                "e_y_f_mm"
            )
        )

        if not math.isfinite(
            e_y_f_mm
        ):

            value = finite(
                state.get(
                    "e_y_f_m"
                )
            )

            if math.isfinite(
                value
            ):

                e_y_f_mm = (
                    value
                    *
                    1000.0
                )

        # ------------------------------------------------------------

        e_y_used_mm = first_finite(
            state.get(
                "e_y_used_mm"
            )
        )

        if not math.isfinite(
            e_y_used_mm
        ):

            value = finite(
                state.get(
                    "e_y_used_m"
                )
            )

            if math.isfinite(
                value
            ):

                e_y_used_mm = (
                    value
                    *
                    1000.0
                )

        # ============================================================
        # HEADING ERROR
        # ============================================================

        theta_raw = first_finite(

            state.get(
                "theta_raw_rad"
            ),

            state.get(
                "e_theta_rad"
            ),

            state.get(
                "theta_rad"
            ),
        )

        theta_f = first_finite(

            state.get(
                "theta_f_rad"
            ),

            state.get(
                "e_theta_f_rad"
            ),
        )

        theta_used = first_finite(

            state.get(
                "theta_used_rad"
            ),

            state.get(
                "e_theta_used_rad"
            ),
        )

        # ============================================================
        # WHEEL GROUP
        # ============================================================

        v_left = first_finite(

            state.get(
                "v_left_cmd"
            ),

            state.get(
                "v_left_est"
            ),
        )

        v_right = first_finite(

            state.get(
                "v_right_cmd"
            ),

            state.get(
                "v_right_est"
            ),
        )

        # ============================================================
        # ROW
        # ============================================================

        row = {

            # --------------------------------------------------------
            # TIME
            # --------------------------------------------------------

            "time_wall_s":
                now_s(),

            "t_s":
                now_s()
                -
                self.t0,

            # --------------------------------------------------------
            # CONTROLLER
            # --------------------------------------------------------

            "active_controller":
                controller,

            "controller_node":
                str(
                    state.get(
                        "node",
                        ""
                    )
                ),

            "controller_version":
                str(
                    state.get(
                        "version",
                        ""
                    )
                ),

            # --------------------------------------------------------
            # AGES
            # --------------------------------------------------------

            "state_age_s":
                self.age(
                    state
                ),

            "cmd_age_s":
                self.age(
                    cmd
                ),

            "odom_age_local_s":
                self.age(
                    odom
                ),

            "msg_age_s":
                finite(
                    state.get(
                        "msg_age_s"
                    )
                ),

            "valid_age_s":
                finite(
                    state.get(
                        "valid_age_s"
                    )
                ),

            # --------------------------------------------------------
            # MODE
            # --------------------------------------------------------

            "mode":
                mode,

            "mode_detail":
                mode_detail,

            "curve_zone":
                zone,

            "display_zone":
                display_zone,

            "mode_code":
                self.encode(
                    "mode",
                    mode
                ),

            "mode_detail_code":
                self.encode(
                    "detail",
                    mode_detail
                ),

            "zone_code":
                self.encode(
                    "zone",
                    display_zone
                ),

            # --------------------------------------------------------
            # STATE
            # --------------------------------------------------------

            "enabled":
                bint(
                    state.get(
                        "enabled",
                        state.get(
                            "enable_cmd",
                            False
                        )
                    )
                ),

            "cmd_published":
                bint(
                    state.get(
                        "cmd_published"
                    )
                ),

            "publish_reason":
                str(
                    state.get(
                        "publish_reason",
                        ""
                    )
                ),

            "publish_reason_code":
                self.encode(
                    "reason",
                    state.get(
                        "publish_reason",
                        ""
                    )
                ),

            "cmd_vel_conflict":
                bint(
                    state.get(
                        "cmd_vel_conflict"
                    )
                ),

            "raw_valid":
                bint(
                    state.get(
                        "raw_valid"
                    )
                ),

            "raw_reason":
                str(
                    state.get(
                        "raw_reason",
                        ""
                    )
                ),

            "lane_state":
                str(
                    state.get(
                        "lane_state",
                        ""
                    )
                ),

            "lane_code":
                self.encode(
                    "lane",
                    state.get(
                        "lane_state",
                        ""
                    )
                ),

            "confidence":
                finite(
                    state.get(
                        "confidence"
                    )
                ),

            "fps_est":
                finite(
                    state.get(
                        "fps_est"
                    )
                ),

            # --------------------------------------------------------
            # ERROR
            # --------------------------------------------------------

            "e_y_raw_mm":
                e_y_raw_mm,

            "e_y_f_mm":
                e_y_f_mm,

            "e_y_used_mm":
                e_y_used_mm,

            "theta_raw_rad":
                theta_raw,

            "theta_f_rad":
                theta_f,

            "theta_used_rad":
                theta_used,

            "de_y":
                finite(
                    state.get(
                        "de_y"
                    )
                ),

            "de_theta":
                finite(
                    state.get(
                        "de_theta"
                    )
                ),

            "de_theta_bs":
                finite(
                    state.get(
                        "de_theta_bs"
                    )
                ),

            # --------------------------------------------------------
            # BACKSTEPPING
            # --------------------------------------------------------

            "theta_virtual":
                finite(
                    state.get(
                        "theta_virtual"
                    )
                ),

            "e_theta_bs":
                finite(
                    state.get(
                        "e_theta_bs"
                    )
                ),

            "omega_ff":
                finite(
                    state.get(
                        "omega_ff"
                    )
                ),

            "p_y":
                finite(
                    state.get(
                        "p_y"
                    )
                ),

            "d_y":
                finite(
                    state.get(
                        "d_y"
                    )
                ),

            "p_theta":
                finite(
                    state.get(
                        "p_theta"
                    )
                ),

            "d_theta":
                finite(
                    state.get(
                        "d_theta"
                    )
                ),

            "feedback_gain":
                finite(
                    state.get(
                        "feedback_gain"
                    )
                ),

            "angular_zone_gain":
                finite(
                    state.get(
                        "angular_zone_gain"
                    )
                ),

            # --------------------------------------------------------
            # GEOMETRY
            # --------------------------------------------------------

            "lookahead_m":
                finite(
                    state.get(
                        "lookahead_m"
                    )
                ),

            "raw_curvature":
                finite(
                    state.get(
                        "raw_curvature"
                    )
                ),

            "curvature":
                finite(
                    state.get(
                        "curvature"
                    )
                ),

            "error_curvature":
                finite(
                    state.get(
                        "error_curvature"
                    )
                ),

            "severity_raw":
                finite(
                    state.get(
                        "severity_raw"
                    )
                ),

            "severity":
                finite(
                    state.get(
                        "severity"
                    )
                ),

            "slow_factor":
                finite(
                    state.get(
                        "slow_factor"
                    )
                ),

            # --------------------------------------------------------
            # LINEAR
            # --------------------------------------------------------

            "speed_cap":
                first_finite(

                    state.get(
                        "speed_cap"
                    ),

                    state.get(
                        "v_base"
                    ),
                ),

            "v_des":
                finite(
                    state.get(
                        "v_des"
                    )
                ),

            "v_ref":
                finite(
                    state.get(
                        "v_ref"
                    )
                ),

            "v_cmd_internal":
                finite(
                    state.get(
                        "v_cmd"
                    )
                ),

            "cmd_v":
                cmd_v,

            "odom_v":
                odom_v,

            # --------------------------------------------------------
            # ANGULAR
            # --------------------------------------------------------

            "omega_raw":
                finite(
                    state.get(
                        "omega_raw"
                    )
                ),

            "omega_des":
                finite(
                    state.get(
                        "omega_des"
                    )
                ),

            "omega_limit":
                finite(
                    state.get(
                        "omega_limit"
                    )
                ),

            "omega_ref":
                finite(
                    state.get(
                        "omega_ref"
                    )
                ),

            "omega_cmd_internal":
                finite(
                    state.get(
                        "omega_cmd"
                    )
                ),

            "cmd_omega":
                cmd_omega,

            "odom_omega":
                odom_omega,

            # --------------------------------------------------------
            # WHEELS
            # --------------------------------------------------------

            "v_left_cmd":
                v_left,

            "v_right_cmd":
                v_right,

            "wheel_left_radps":
                first_finite(

                    state.get(
                        "wheel_left_radps"
                    ),

                    state.get(
                        "wheel_left_radps_est"
                    ),
                ),

            "wheel_right_radps":
                first_finite(

                    state.get(
                        "wheel_right_radps"
                    ),

                    state.get(
                        "wheel_right_radps_est"
                    ),
                ),

            "delta_v_cmd":
                (
                    v_right
                    -
                    v_left

                    if
                    math.isfinite(
                        v_left
                    )
                    and
                    math.isfinite(
                        v_right
                    )

                    else
                    math.nan
                ),

            # --------------------------------------------------------
            # ODOM
            # --------------------------------------------------------

            "odom_x":
                first_finite(

                    state.get(
                        "odom_x"
                    ),

                    odom.get(
                        "x"
                    ),
                ),

            "odom_y":
                first_finite(

                    state.get(
                        "odom_y"
                    ),

                    odom.get(
                        "y"
                    ),
                ),

            "odom_yaw":
                first_finite(

                    state.get(
                        "odom_yaw"
                    ),

                    odom.get(
                        "yaw"
                    ),
                ),

            # --------------------------------------------------------
            # FLAGS
            # --------------------------------------------------------

            "center_latched":
                bint(
                    state.get(
                        "center_latched"
                    )
                ),

            "center_zone":
                bint(
                    state.get(
                        "center_zone"
                    )
                ),

            "near_zone":
                bint(
                    state.get(
                        "near_zone"
                    )
                ),

            "curve_confirmed":
                bint(
                    state.get(
                        "curve_confirmed"
                    )
                ),

            "curve_count":
                finite(
                    state.get(
                        "curve_count"
                    )
                ),

            "curve_sign":
                finite(
                    state.get(
                        "curve_sign"
                    )
                ),

            "branch_hold":
                bint(
                    state.get(
                        "branch_hold",
                        state.get(
                            "branch_hold_active",
                            False
                        )
                    )
                ),

            "branch_reason":
                str(
                    state.get(
                        "branch_reason",
                        state.get(
                            "branch_hold_reason",
                            ""
                        )
                    )
                ),

            "control_error_frozen":
                bint(
                    state.get(
                        "control_error_frozen"
                    )
                ),

            "freeze_age_s":
                finite(
                    state.get(
                        "freeze_age_s"
                    )
                ),

            "startup_active":
                bint(
                    state.get(
                        "startup_active"
                    )
                ),

            "lidar_stop":
                bint(
                    state.get(
                        "lidar_stop"
                    )
                ),

            "lidar_mode":
                str(
                    state.get(
                        "lidar_mode",
                        ""
                    )
                ),

            "lidar_mode_code":
                self.encode(
                    "lidar",
                    state.get(
                        "lidar_mode",
                        ""
                    )
                ),

            "front_min_m":
                finite(
                    state.get(
                        "front_min_m"
                    )
                ),

            "pivot_mode":
                str(
                    state.get(
                        "pivot_mode",
                        ""
                    )
                ),

            "pivot_mode_code":
                self.encode(
                    "pivot",
                    state.get(
                        "pivot_mode",
                        ""
                    )
                ),
        }

        # ============================================================
        # TRACKING ERROR
        # ============================================================

        row[
            "linear_tracking_error"
        ] = (
            cmd_v
            -
            odom_v

            if
            math.isfinite(
                cmd_v
            )
            and
            math.isfinite(
                odom_v
            )

            else
            math.nan
        )

        row[
            "angular_tracking_error"
        ] = (
            cmd_omega
            -
            odom_omega

            if
            math.isfinite(
                cmd_omega
            )
            and
            math.isfinite(
                odom_omega
            )

            else
            math.nan
        )

        # ============================================================
        # PRESERVE ALL ORIGINAL STATE FIELDS
        # ============================================================

        for (
            key,
            value
        ) in state.items():

            if not key.startswith(
                "_"
            ):

                row[
                    f"state__{key}"
                ] = csv_scalar(
                    value
                )

        return row

    # ================================================================
    # LOG
    # ================================================================

    def log_snapshot(self):

        row = self.make_row()

        if row is not None:

            self.records.append(
                row
            )

    # ================================================================
    # WINDOW
    # ================================================================

    def window_records(self):

        if not self.records:
            return []

        if self.a.window_s <= 0:
            return self.records

        cutoff = (
            self.records[-1][
                "t_s"
            ]
            -
            self.a.window_s
        )

        return [

            row

            for row
            in self.records

            if row[
                "t_s"
            ]
            >=
            cutoff
        ]

    @staticmethod
    def column(
        records,
        key
    ):

        return [

            finite(
                row.get(
                    key
                )
            )

            for row
            in records
        ]

    @staticmethod
    def setup_axis(
        axis,
        title,
        ylabel
    ):

        axis.set_title(
            title
        )

        axis.set_xlabel(
            "time [s]"
        )

        axis.set_ylabel(
            ylabel
        )

        axis.grid(
            True
        )

    # ================================================================
    # DASHBOARD
    # ================================================================

    def make_dashboard(
        self,
        title
    ):

        (
            fig,
            array
        ) = plt.subplots(
            4,
            3,
            figsize=(
                19,
                12
            )
        )

        fig.subplots_adjust(
            left=0.055,
            right=0.985,
            top=0.94,
            bottom=0.06,
            hspace=0.40,
            wspace=0.26,
        )

        fig.suptitle(
            title,
            fontsize=15
        )

        try:

            fig.canvas.manager.set_window_title(
                "AVS PD-Backstepping Unified Logger"
            )

        except Exception:
            pass

        names = [

            "lateral",
            "heading",
            "components",

            "linear",
            "angular",
            "derivative",

            "geometry",
            "modes",
            "tracking",

            "wheels",
            "path",
            "status",
        ]

        axes = OrderedDict(
            zip(
                names,
                array.flatten()
            )
        )

        lines = {

            name:
                OrderedDict()

            for name
            in names
        }

        def add(
            panel,
            key,
            label
        ):

            line, = axes[
                panel
            ].plot(
                [],
                [],
                label=label
            )

            lines[
                panel
            ][key] = line

        # ============================================================
        # 1. LATERAL
        # ============================================================

        self.setup_axis(
            axes[
                "lateral"
            ],
            "1. Lateral error",
            "mm"
        )

        for key, label in [

            (
                "e_y_raw_mm",
                "raw"
            ),

            (
                "e_y_f_mm",
                "filtered"
            ),

            (
                "e_y_used_mm",
                "used"
            ),

        ]:

            add(
                "lateral",
                key,
                label
            )

        # ============================================================
        # 2. HEADING
        # ============================================================

        self.setup_axis(

            axes[
                "heading"
            ],

            "2. Heading / backstepping error",

            "rad"
        )

        for key, label in [

            (
                "theta_raw_rad",
                "raw"
            ),

            (
                "theta_f_rad",
                "filtered"
            ),

            (
                "theta_used_rad",
                "used"
            ),

            (
                "theta_virtual",
                "theta virtual"
            ),

            (
                "e_theta_bs",
                "BS error"
            ),

        ]:

            add(
                "heading",
                key,
                label
            )

        # ============================================================
        # 3. COMPONENTS
        # ============================================================

        self.setup_axis(

            axes[
                "components"
            ],

            "3. Backstepping / PD steering",

            "rad/s contribution"
        )

        for key, label in [

            (
                "omega_ff",
                "feed-forward"
            ),

            (
                "p_y",
                "P lateral"
            ),

            (
                "d_y",
                "D lateral"
            ),

            (
                "p_theta",
                "P heading"
            ),

            (
                "d_theta",
                "D heading"
            ),

            (
                "omega_raw",
                "omega raw"
            ),

            (
                "omega_des",
                "omega desired"
            ),

            (
                "omega_limit",
                "omega limit"
            ),

        ]:

            add(
                "components",
                key,
                label
            )

        # ============================================================
        # 4. LINEAR
        # ============================================================

        self.setup_axis(

            axes[
                "linear"
            ],

            "4. Linear velocity",

            "m/s"
        )

        for key, label in [

            (
                "speed_cap",
                "speed cap/base"
            ),

            (
                "v_des",
                "desired"
            ),

            (
                "v_ref",
                "reference"
            ),

            (
                "v_cmd_internal",
                "controller cmd"
            ),

            (
                "cmd_v",
                "/cmd_vel"
            ),

            (
                "odom_v",
                "odom"
            ),

        ]:

            add(
                "linear",
                key,
                label
            )

        # ============================================================
        # 5. ANGULAR
        # ============================================================

        self.setup_axis(

            axes[
                "angular"
            ],

            "5. Angular velocity",

            "rad/s"
        )

        for key, label in [

            (
                "omega_raw",
                "raw"
            ),

            (
                "omega_des",
                "desired"
            ),

            (
                "omega_ref",
                "reference"
            ),

            (
                "omega_cmd_internal",
                "controller cmd"
            ),

            (
                "cmd_omega",
                "/cmd_vel"
            ),

            (
                "odom_omega",
                "odom"
            ),

        ]:

            add(
                "angular",
                key,
                label
            )

        # ============================================================
        # 6. DERIVATIVE
        # ============================================================

        self.setup_axis(

            axes[
                "derivative"
            ],

            "6. Derivative signals",

            "value / s"
        )

        for key, label in [

            (
                "de_y",
                "de_y"
            ),

            (
                "de_theta",
                "dtheta"
            ),

            (
                "de_theta_bs",
                "dtheta BS"
            ),

        ]:

            add(
                "derivative",
                key,
                label
            )

        # ============================================================
        # 7. GEOMETRY / SEVERITY
        # ============================================================

        self.setup_axis(

            axes[
                "geometry"
            ],

            "7. Geometry / severity",

            "value"
        )

        for key, label in [

            (
                "raw_curvature",
                "raw curvature"
            ),

            (
                "curvature",
                "filtered curvature"
            ),

            (
                "error_curvature",
                "error curvature"
            ),

            (
                "severity_raw",
                "severity raw"
            ),

            (
                "severity",
                "severity"
            ),

            (
                "slow_factor",
                "slow factor"
            ),

            (
                "lookahead_m",
                "lookahead"
            ),

            (
                "feedback_gain",
                "feedback gain"
            ),

            (
                "angular_zone_gain",
                "angular gain"
            ),

        ]:

            add(
                "geometry",
                key,
                label
            )

        # ============================================================
        # 8. MODES
        # ============================================================

        self.setup_axis(

            axes[
                "modes"
            ],

            "8. Modes / perception / safety",

            "value / code"
        )

        for key, label in [

            (
                "fps_est",
                "FPS"
            ),

            (
                "confidence",
                "confidence"
            ),

            (
                "mode_code",
                "mode"
            ),

            (
                "zone_code",
                "zone/detail"
            ),

            (
                "center_latched",
                "center latch"
            ),

            (
                "center_zone",
                "center"
            ),

            (
                "near_zone",
                "near"
            ),

            (
                "curve_confirmed",
                "curve"
            ),

            (
                "branch_hold",
                "branch hold"
            ),

            (
                "control_error_frozen",
                "frozen"
            ),

            (
                "lidar_stop",
                "lidar stop"
            ),

        ]:

            add(
                "modes",
                key,
                label
            )

        # ============================================================
        # 9. TRACKING ERROR
        # ============================================================

        self.setup_axis(

            axes[
                "tracking"
            ],

            "9. Command tracking error",

            "error"
        )

        add(
            "tracking",
            "linear_tracking_error",
            "linear cmd-odom"
        )

        add(
            "tracking",
            "angular_tracking_error",
            "angular cmd-odom"
        )

        # ============================================================
        # 10. WHEELS
        # ============================================================

        self.setup_axis(

            axes[
                "wheels"
            ],

            "10. Left/right wheel-group command",

            "m/s"
        )

        for key, label in [

            (
                "v_left_cmd",
                "left"
            ),

            (
                "v_right_cmd",
                "right"
            ),

            (
                "delta_v_cmd",
                "right-left"
            ),

        ]:

            add(
                "wheels",
                key,
                label
            )

        # ============================================================
        # 11. ODOM PATH
        # ============================================================

        axes[
            "path"
        ].set_title(
            "11. Odom path"
        )

        axes[
            "path"
        ].set_xlabel(
            "x [m]"
        )

        axes[
            "path"
        ].set_ylabel(
            "y [m]"
        )

        axes[
            "path"
        ].grid(
            True
        )

        path_line, = axes[
            "path"
        ].plot(
            [],
            [],
            label="odom path"
        )

        lines[
            "path"
        ][
            "path"
        ] = path_line

        axes[
            "path"
        ].legend(
            fontsize=7
        )

        axes[
            "path"
        ].set_aspect(
            "equal",
            adjustable="datalim"
        )

        # ============================================================
        # 12. STATUS
        # ============================================================

        axes[
            "status"
        ].set_title(
            "12. Current PD-Backstepping status"
        )

        axes[
            "status"
        ].axis(
            "off"
        )

        status = axes[
            "status"
        ].text(

            0.01,
            0.99,

            "Waiting...",

            transform=(
                axes[
                    "status"
                ].transAxes
            ),

            va="top",
            ha="left",

            family="monospace",

            fontsize=7.8
        )

        # ============================================================
        # LEGENDS
        # ============================================================

        for (
            panel,
            axis
        ) in axes.items():

            if (
                panel !=
                "status"
                and
                lines[
                    panel
                ]
            ):

                axis.legend(
                    fontsize=6.5,
                    loc="best"
                )

        return (
            fig,
            axes,
            lines,
            status
        )

    # ================================================================
    # VISIBILITY
    # ================================================================

    @staticmethod
    def set_visibility(
        lines,
        panel,
        keys
    ):

        wanted = set(
            keys
        )

        for (
            key,
            line
        ) in lines[
            panel
        ].items():

            line.set_visible(
                key
                in
                wanted
            )

    # ================================================================
    # VERSION-SPECIFIC DASHBOARD
    # ================================================================

    def adapt_dashboard(
        self,
        controller,
        axes,
        lines
    ):

        # ============================================================
        # V1
        #
        # V1 không publish P/D breakdown riêng.
        # ============================================================

        if controller == "v1":

            axes[
                "components"
            ].set_title(
                "3. V1 Backstepping steering result"
            )

            self.set_visibility(
                lines,
                "components",
                [
                    "omega_raw",
                    "omega_des",
                    "omega_limit",
                ]
            )

            axes[
                "geometry"
            ].set_title(
                "7. V1 Geometry / curve slowdown"
            )

            self.set_visibility(
                lines,
                "geometry",
                [
                    "curvature",
                    "slow_factor",
                    "lookahead_m",
                ]
            )

            axes[
                "derivative"
            ].set_title(
                "6. V1 derivative signals"
            )

            self.set_visibility(
                lines,
                "derivative",
                [
                    "de_y",
                    "de_theta",
                ]
            )

        # ============================================================
        # V2
        # ============================================================

        elif controller == "v2":

            axes[
                "components"
            ].set_title(
                "3. V2 PD-Backstepping components + FF"
            )

            self.set_visibility(
                lines,
                "components",
                [
                    "omega_ff",
                    "p_y",
                    "d_y",
                    "p_theta",
                    "d_theta",
                    "omega_raw",
                    "omega_des",
                    "omega_limit",
                ]
            )

            axes[
                "geometry"
            ].set_title(
                "7. V2 Curvature / severity / adaptive gains"
            )

            self.set_visibility(
                lines,
                "geometry",
                [
                    "raw_curvature",
                    "curvature",
                    "error_curvature",
                    "severity_raw",
                    "severity",
                    "lookahead_m",
                    "feedback_gain",
                    "angular_zone_gain",
                ]
            )

            axes[
                "derivative"
            ].set_title(
                "6. V2 derivative signals"
            )

            self.set_visibility(
                lines,
                "derivative",
                [
                    "de_y",
                    "de_theta",
                ]
            )

        # ============================================================
        # V3
        # ============================================================

        elif controller == "v3":

            axes[
                "components"
            ].set_title(
                "3. V3 Backstepping + PD components (no turn FF)"
            )

            self.set_visibility(
                lines,
                "components",
                [
                    "p_y",
                    "d_y",
                    "p_theta",
                    "d_theta",
                    "omega_raw",
                    "omega_des",
                    "omega_limit",
                ]
            )

            axes[
                "geometry"
            ].set_title(
                "7. V3 Curvature / severity for speed scheduling"
            )

            self.set_visibility(
                lines,
                "geometry",
                [
                    "raw_curvature",
                    "curvature",
                    "severity_raw",
                    "severity",
                    "lookahead_m",
                ]
            )

            axes[
                "derivative"
            ].set_title(
                "6. V3 derivative + Backstepping derivative"
            )

            self.set_visibility(
                lines,
                "derivative",
                [
                    "de_y",
                    "de_theta",
                    "de_theta_bs",
                ]
            )

        else:

            for panel in (
                "components",
                "geometry",
                "derivative",
            ):

                self.set_visibility(
                    lines,
                    panel,
                    lines[
                        panel
                    ].keys()
                )

        # ============================================================
        # REBUILD LEGENDS
        # ============================================================

        for panel in (

            "components",
            "geometry",
            "derivative",

        ):

            old = axes[
                panel
            ].get_legend()

            if old:
                old.remove()

            visible = [

                line

                for line
                in lines[
                    panel
                ].values()

                if line.get_visible()
            ]

            if visible:

                axes[
                    panel
                ].legend(
                    fontsize=6.5,
                    loc="best"
                )

    # ================================================================
    # RENDER
    # ================================================================

    def render(
        self,
        fig,
        axes,
        lines,
        status,
        records
    ):

        if not records:
            return

        latest = records[
            -1
        ]

        controller = str(
            latest.get(
                "active_controller",
                ""
            )
            or
            ""
        )

        self.adapt_dashboard(
            controller,
            axes,
            lines
        )

        controller_title = (
            CONTROLLERS.get(
                controller,
                {}
            ).get(
                "title",
                "PD-Backstepping Controller"
            )
        )

        version = str(
            latest.get(
                "controller_version",
                ""
            )
            or
            ""
        )

        fig.suptitle(

            f"AVS {controller_title}"
            +
            (
                f" | {version}"
                if version
                else
                ""
            ),

            fontsize=15
        )

        t = self.column(
            records,
            "t_s"
        )

        # ============================================================
        # TIME PANELS
        # ============================================================

        for panel in (

            "lateral",
            "heading",
            "components",

            "linear",
            "angular",
            "derivative",

            "geometry",
            "modes",
            "tracking",

            "wheels",

        ):

            for (
                key,
                line
            ) in lines[
                panel
            ].items():

                line.set_data(
                    t,
                    self.column(
                        records,
                        key
                    )
                )

            axes[
                panel
            ].relim(
                visible_only=True
            )

            axes[
                panel
            ].autoscale_view()

        # ============================================================
        # PATH
        # ============================================================

        lines[
            "path"
        ][
            "path"
        ].set_data(

            self.column(
                records,
                "odom_x"
            ),

            self.column(
                records,
                "odom_y"
            )
        )

        axes[
            "path"
        ].relim()

        axes[
            "path"
        ].autoscale_view()

        # ============================================================
        # MODES TITLE
        # ============================================================

        axes[
            "modes"
        ].set_title(

            "8. Modes / perception / safety"

            f" | mode="
            f"{latest.get('mode', '')}"

            f" | zone="
            f"{latest.get('display_zone', '')}"
        )

        # ============================================================
        # STATUS
        # ============================================================

        multiple = (

            "  !!! MULTIPLE FRESH !!!"

            if
            len(
                self.fresh_controllers()
            )
            >
            1

            else
            ""
        )

        component_note = ""

        if controller == "v1":

            component_note = (
                "\nV1: no separate P/D component telemetry"
            )

        elif controller == "v3":

            component_note = (
                "\nV3: omega_ff=0; curvature used for speed only"
            )

        status.set_text(

            f"active       : "
            f"{controller}"
            f"{multiple}\n"

            f"node         : "
            f"{latest.get('controller_node', '')}\n"

            f"version      : "
            f"{version}\n"

            f"state age    : "
            f"{finite(latest.get('state_age_s')):7.3f} s\n"

            f"\n"

            f"mode/detail  : "
            f"{latest.get('mode', '')} / "
            f"{latest.get('mode_detail', '')}\n"

            f"zone         : "
            f"{latest.get('display_zone', '')}\n"

            f"valid/cmd    : "
            f"{latest.get('raw_valid', 0)} / "
            f"{latest.get('cmd_published', 0)}\n"

            f"publish      : "
            f"{latest.get('publish_reason', '')}\n"

            f"conflict     : "
            f"{latest.get('cmd_vel_conflict', 0)}\n"

            f"lane         : "
            f"{latest.get('lane_state', '')}\n"

            f"fps/conf     : "
            f"{finite(latest.get('fps_est')):7.2f} / "
            f"{finite(latest.get('confidence')):5.2f}\n"

            f"\n"

            f"e_y          : "
            f"{finite(latest.get('e_y_used_mm')):8.2f} mm\n"

            f"theta        : "
            f"{finite(latest.get('theta_used_rad')):8.3f} rad\n"

            f"theta virt   : "
            f"{finite(latest.get('theta_virtual')):8.3f} rad\n"

            f"theta BS     : "
            f"{finite(latest.get('e_theta_bs')):8.3f} rad\n"

            f"de_y/dtheta  : "
            f"{finite(latest.get('de_y')):7.3f} / "
            f"{finite(latest.get('de_theta')):7.3f}\n"

            f"\n"

            f"FF           : "
            f"{finite(latest.get('omega_ff')):8.3f}\n"

            f"P y/theta    : "
            f"{finite(latest.get('p_y')):8.3f} / "
            f"{finite(latest.get('p_theta')):8.3f}\n"

            f"D y/theta    : "
            f"{finite(latest.get('d_y')):8.3f} / "
            f"{finite(latest.get('d_theta')):8.3f}\n"

            f"omega limit  : "
            f"{finite(latest.get('omega_limit')):8.3f}\n"

            f"\n"

            f"v cap/des/ref: "
            f"{finite(latest.get('speed_cap')):7.3f} / "
            f"{finite(latest.get('v_des')):7.3f} / "
            f"{finite(latest.get('v_ref')):7.3f}\n"

            f"v cmd/odom   : "
            f"{finite(latest.get('cmd_v')):7.3f} / "
            f"{finite(latest.get('odom_v')):7.3f}\n"

            f"w des/ref    : "
            f"{finite(latest.get('omega_des')):7.3f} / "
            f"{finite(latest.get('omega_ref')):7.3f}\n"

            f"w cmd/odom   : "
            f"{finite(latest.get('cmd_omega')):7.3f} / "
            f"{finite(latest.get('odom_omega')):7.3f}\n"

            f"\n"

            f"curvature    : "
            f"{finite(latest.get('curvature')):8.3f}\n"

            f"severity     : "
            f"{finite(latest.get('severity')):8.3f}\n"

            f"slow factor  : "
            f"{finite(latest.get('slow_factor')):8.3f}\n"

            f"center/curve : "
            f"{latest.get('center_latched', 0) or latest.get('center_zone', 0)} / "
            f"{latest.get('curve_confirmed', 0)}\n"

            f"branch/freeze: "
            f"{latest.get('branch_hold', 0)} / "
            f"{latest.get('control_error_frozen', 0)}\n"

            f"lidar        : "
            f"{latest.get('lidar_mode', '')} "
            f"stop={latest.get('lidar_stop', 0)}"

            f"{component_note}"
        )

        try:

            fig.canvas.draw_idle()

        except Exception:
            pass

    # ================================================================
    # LIVE
    # ================================================================

    def update_live(self):

        if (
            self.fig is None
            or
            self.window_closed
        ):
            return

        records = (
            self.window_records()
        )

        if not records:
            return

        self.render(

            self.fig,

            self.axes,

            self.lines,

            self.status_text,

            records
        )

        try:

            self.fig.canvas.flush_events()

        except Exception:
            pass

    # ================================================================
    # WINDOW DOES NOT STEAL FOCUS
    # ================================================================

    def configure_no_focus(self):

        try:

            window = getattr(
                self.fig.canvas.manager,
                "window",
                None
            )

            if window is None:
                return

            try:

                from matplotlib.backends.qt_compat import QtCore

                try:

                    window.setWindowFlag(
                        QtCore.Qt.WindowStaysOnTopHint,
                        False
                    )

                except Exception:
                    pass

                try:

                    window.setAttribute(
                        QtCore.Qt.WA_ShowWithoutActivating,
                        True
                    )

                except Exception:
                    pass

            except Exception:
                pass

        except Exception:
            pass

    # ================================================================
    # CLOSE
    #
    # Không save figure đang bị destroy.
    # Final PNG được dựng lại từ records.
    # ================================================================

    def on_close(
        self,
        _event
    ):

        self.window_closed = True
        self.stop_requested = True

    # ================================================================
    # CSV
    # ================================================================

    def save_csv(self):

        if not self.records:
            return

        preferred = [

            "time_wall_s",
            "t_s",

            "active_controller",
            "controller_node",
            "controller_version",

            "mode",
            "mode_detail",
            "curve_zone",
            "display_zone",

            "enabled",
            "raw_valid",
            "cmd_published",
            "publish_reason",
            "cmd_vel_conflict",
            "raw_reason",
            "lane_state",

            "confidence",
            "fps_est",

            "e_y_raw_mm",
            "e_y_f_mm",
            "e_y_used_mm",

            "theta_raw_rad",
            "theta_f_rad",
            "theta_used_rad",

            "theta_virtual",
            "e_theta_bs",

            "de_y",
            "de_theta",
            "de_theta_bs",

            "omega_ff",

            "p_y",
            "d_y",
            "p_theta",
            "d_theta",

            "feedback_gain",
            "angular_zone_gain",

            "lookahead_m",
            "raw_curvature",
            "curvature",
            "error_curvature",

            "severity_raw",
            "severity",
            "slow_factor",

            "speed_cap",

            "v_des",
            "v_ref",
            "v_cmd_internal",
            "cmd_v",
            "odom_v",

            "linear_tracking_error",

            "omega_raw",
            "omega_des",
            "omega_limit",
            "omega_ref",
            "omega_cmd_internal",
            "cmd_omega",
            "odom_omega",

            "angular_tracking_error",

            "v_left_cmd",
            "v_right_cmd",
            "delta_v_cmd",

            "wheel_left_radps",
            "wheel_right_radps",

            "odom_x",
            "odom_y",
            "odom_yaw",

            "center_latched",
            "center_zone",
            "near_zone",

            "curve_confirmed",
            "curve_count",
            "curve_sign",

            "branch_hold",
            "branch_reason",

            "control_error_frozen",
            "freeze_age_s",

            "startup_active",

            "lidar_stop",
            "lidar_mode",
            "front_min_m",

            "pivot_mode",

            "state_age_s",
            "cmd_age_s",
            "odom_age_local_s",

            "msg_age_s",
            "valid_age_s",
        ]

        fields = []
        seen = set()

        for key in preferred:

            if key not in seen:

                fields.append(
                    key
                )

                seen.add(
                    key
                )

        # ============================================================
        # state__* dynamic columns
        # ============================================================

        for row in self.records:

            for key in row:

                if key not in seen:

                    fields.append(
                        key
                    )

                    seen.add(
                        key
                    )

        with self.csv_path.open(
            "w",
            newline="",
            encoding="utf-8"
        ) as handle:

            writer = csv.DictWriter(
                handle,
                fieldnames=fields
            )

            writer.writeheader()

            writer.writerows(
                self.records
            )

    # ================================================================
    # MODE MAP
    # ================================================================

    def save_maps(self):

        data = {

            group:
                {
                    str(code):
                        name

                    for (
                        name,
                        code
                    )
                    in mapping.items()
                }

            for (
                group,
                mapping
            )
            in self.maps.items()
        }

        self.map_path.write_text(

            json.dumps(
                data,
                indent=2,
                ensure_ascii=False
            ),

            encoding="utf-8"
        )

    # ================================================================
    # RAW STATE
    # ================================================================

    def save_raw(self):

        with self.raw_path.open(
            "w",
            encoding="utf-8"
        ) as handle:

            for item in self.raw_records:

                handle.write(

                    json.dumps(
                        item,
                        ensure_ascii=False
                    )

                    +

                    "\n"
                )

    # ================================================================
    # FINAL PNG
    #
    # Dựng figure mới.
    #
    # Vì vậy:
    # - Ctrl+C vẫn lưu PNG
    # - bấm X vẫn lưu PNG
    # ================================================================

    def save_png(self):

        if not self.records:
            return

        was_interactive = (
            plt.isinteractive()
        )

        plt.ioff()

        (
            fig,
            axes,
            lines,
            status
        ) = self.make_dashboard(
            "Final PD-Backstepping Dashboard"
        )

        try:

            self.render(
                fig,
                axes,
                lines,
                status,
                self.records
            )

            fig.savefig(
                self.png_path,
                dpi=self.a.dpi,
                bbox_inches="tight"
            )

        finally:

            plt.close(
                fig
            )

            if (
                was_interactive
                and
                not self.a.no_live
            ):

                plt.ion()

    # ================================================================
    # AUTOSAVE
    # ================================================================

    def autosave(self):

        # Không save PNG liên tục vì savefig gây khựng GUI.

        self.save_csv()

        self.save_maps()

        self.save_raw()

    def save_all(self):

        self.autosave()

        self.save_png()

        print()

        print(
            "=" * 62
        )

        print(
            "PD-BACKSTEPPING UNIFIED LOG SAVED"
        )

        print(
            f"Folder    : "
            f"{self.output_dir}"
        )

        print(
            f"CSV       : "
            f"{self.csv_path}"
        )

        print(
            f"PNG       : "
            f"{self.png_path}"
        )

        print(
            f"Mode map  : "
            f"{self.map_path}"
        )

        print(
            f"Raw state : "
            f"{self.raw_path}"
        )

        print(
            "=" * 62
        )

        print()


# ================================================================
# ARGUMENTS
# ================================================================

def build_parser():

    parser = argparse.ArgumentParser(

        description=(
            "Unified realtime logger for "
            "PD-Backstepping V1/V2/V3"
        )
    )

    parser.add_argument(

        "--output-dir",

        default=(
            "/home/bluedstar/"
            "SimpleRobot/"
            "terminal-run/"
            "plot_logger"
        )
    )

    parser.add_argument(

        "--controller",

        choices=[
            "auto",
            "v1",
            "v2",
            "v3",
        ],

        default="auto"
    )

    parser.add_argument(
        "--active-timeout-s",
        type=float,
        default=1.0
    )

    parser.add_argument(
        "--cmd-vel-topic",
        default="/cmd_vel"
    )

    parser.add_argument(
        "--odom-topic",
        default="/odom_raw"
    )

    parser.add_argument(
        "--window-s",
        type=float,
        default=90.0
    )

    parser.add_argument(
        "--log-hz",
        type=float,
        default=20.0
    )

    parser.add_argument(
        "--plot-hz",
        type=float,
        default=4.0
    )

    parser.add_argument(
        "--autosave-s",
        type=float,
        default=10.0
    )

    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0
    )

    parser.add_argument(
        "--dpi",
        type=int,
        default=160
    )

    parser.add_argument(
        "--no-live",
        action="store_true"
    )

    return parser


# ================================================================
# MAIN
# ================================================================

def main():

    args = (
        build_parser()
        .parse_args()
    )

    rclpy.init()

    node = (
        PDBacksteppingUnifiedLogger(
            args
        )
    )

    def stop_handler(
        *_args
    ):

        node.stop_requested = True

    signal.signal(
        signal.SIGINT,
        stop_handler
    )

    signal.signal(
        signal.SIGTERM,
        stop_handler
    )

    log_period = (
        1.0
        /
        max(
            args.log_hz,
            1.0
        )
    )

    plot_period = (
        1.0
        /
        max(
            args.plot_hz,
            0.5
        )
    )

    try:

        while (
            rclpy.ok()
            and
            not node.stop_requested
        ):

            rclpy.spin_once(
                node,
                timeout_sec=0.005
            )

            current = (
                now_s()
            )

            # --------------------------------------------------------
            # LOG
            # --------------------------------------------------------

            if (
                current
                -
                node.last_log
                >=
                log_period
            ):

                node.log_snapshot()

                node.last_log = (
                    current
                )

            # --------------------------------------------------------
            # LIVE PLOT
            # --------------------------------------------------------

            if (
                not args.no_live
                and
                not node.window_closed
                and
                current
                -
                node.last_plot
                >=
                plot_period
            ):

                node.update_live()

                node.last_plot = (
                    current
                )

            # --------------------------------------------------------
            # AUTOSAVE
            # --------------------------------------------------------

            if (
                args.autosave_s
                >
                0.0
                and
                current
                -
                node.last_autosave
                >=
                args.autosave_s
            ):

                node.autosave()

                node.last_autosave = (
                    current
                )

            # --------------------------------------------------------
            # DURATION
            # --------------------------------------------------------

            if (
                args.duration_s
                >
                0.0
                and
                current
                -
                node.t0
                >=
                args.duration_s
            ):

                break

            # --------------------------------------------------------
            # GUI EVENTS
            # --------------------------------------------------------

            if (
                not args.no_live
                and
                not node.window_closed
                and
                node.fig
                is not None
            ):

                try:

                    node.fig.canvas.flush_events()

                except Exception:
                    pass

            time.sleep(
                0.003
            )

    except KeyboardInterrupt:

        pass

    finally:

        try:

            node.save_all()

        finally:

            node.destroy_node()

            if rclpy.ok():

                rclpy.shutdown()

            try:

                plt.close(
                    "all"
                )

            except Exception:
                pass


if __name__ == "__main__":

    main()
