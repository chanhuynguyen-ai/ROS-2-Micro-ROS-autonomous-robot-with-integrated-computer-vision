#!/usr/bin/env python3
import argparse, csv, json, math, signal, sys, time
from collections import OrderedDict
from pathlib import Path

import matplotlib
if '--no-live' in sys.argv:
    matplotlib.use('Agg')
try:
    matplotlib.rcParams['figure.raise_window'] = False
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


CONTROLLERS = OrderedDict([
    (
        'v1',
        {
            'topic': '/avs/PD_controller_v1_debug',
            'title': 'PD Controller V1 - Center Stable / Curve Fast',
        },
    ),
    (
        'v2',
        {
            'topic': '/avs/PD_controller_v2_debug',
            'title': 'PD Controller V2 - Pure PD',
        },
    ),
    (
        'v3',
        {
            'topic': '/avs/PD_controller_v3_debug',
            'title': 'PD Controller V3 - Fine Tuned PD',
        },
    ),
])


def now_s():
    return time.time()


def finite(v, default=math.nan):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except Exception:
        return default


def first_finite(*vals, default=math.nan):
    for v in vals:
        x = finite(v)
        if math.isfinite(x):
            return x
    return default


def first_value(*vals, default=None):
    for v in vals:
        if v is not None and v != '':
            return v
    return default


def bint(v):
    if isinstance(v, bool):
        return int(v)

    if isinstance(v, str):
        return (
            0
            if v.strip().lower() in {
                '',
                '0',
                'false',
                'no',
                'none',
                'invalid',
                'lost',
            }
            else 1
        )

    return int(bool(v)) if v is not None else 0


def parse_json(text):
    try:
        d = json.loads(text)
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z),
    )


def csv_scalar(v):
    if v is None:
        return ''

    if isinstance(v, (str, int, float, bool)):
        return v

    try:
        return json.dumps(
            v,
            ensure_ascii=False,
            separators=(',', ':'),
        )
    except Exception:
        return str(v)


class PDLogger(Node):

    def __init__(self, args):
        super().__init__('PD_controller_logger')

        self.a = args

        self.t0 = now_s()

        self.stop = False
        self.window_closed = False

        self.last_log = 0.0
        self.last_plot = 0.0
        self.last_save = 0.0

        # ============================================================
        # DATA SOURCES
        # ============================================================

        self.debug = {
            k: {}
            for k in CONTROLLERS
        }

        self.ce = {}
        self.cmd = {}
        self.odom = {}

        self.records = []
        self.raw_records = []

        self.last_active = ''
        self.last_warn = 0.0

        self.mode_maps = {
            k: OrderedDict()
            for k in [
                'mode',
                'lane',
                'publish_reason',
            ]
        }

        # ============================================================
        # LOW LATENCY QoS
        # ============================================================

        self.q = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        # ============================================================
        # AUTO-DETECT:
        #
        # Subscribe BOTH V1 and V2.
        #
        # Node nào có debug mới nhất sẽ là active controller.
        # ============================================================

        for c, info in CONTROLLERS.items():

            self.create_subscription(
                String,
                info['topic'],
                lambda m, cc=c: self.debug_cb(cc, m),
                self.q,
            )

        self.create_subscription(
            String,
            args.control_error_topic,
            self.ce_cb,
            self.q,
        )

        self.create_subscription(
            Twist,
            args.cmd_vel_topic,
            self.cmd_cb,
            self.q,
        )

        self.create_subscription(
            Odometry,
            args.odom_topic,
            self.odom_cb,
            qos_profile_sensor_data,
        )

        # ============================================================
        # OUTPUT
        # ============================================================

        stamp = time.strftime(
            '%Y-%m-%d_%H-%M-%S'
        )

        self.run_name = (
            f'PD_controller_unified_{stamp}'
        )

        self.run_dir = (
            Path(args.output_dir)
            .expanduser()
            .resolve()
            /
            self.run_name
        )

        self.run_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.csv_path = (
            self.run_dir
            /
            f'{self.run_name}.csv'
        )

        self.png_path = (
            self.run_dir
            /
            f'{self.run_name}.png'
        )

        self.map_path = (
            self.run_dir
            /
            'mode_map.json'
        )

        self.raw_path = (
            self.run_dir
            /
            'raw_debug.jsonl'
        )

        # ============================================================
        # LIVE DASHBOARD
        # ============================================================

        self.fig = None
        self.axes = None
        self.lines = None
        self.status = None

        if not args.no_live:

            plt.ion()

            (
                self.fig,
                self.axes,
                self.lines,
                self.status,
            ) = self.make_dashboard(
                'Waiting for PD V1/V2...'
            )

            self.fig.canvas.mpl_connect(
                'close_event',
                self.on_close,
            )

            plt.show(
                block=False
            )

            self.no_focus()

        self.get_logger().info(
            'Unified PD V1/V2 logger started'
        )

        self.get_logger().info(
            'Auto sources: '
            '/avs/PD_controller_v1_debug, '
            '/avs/PD_controller_v2_debug, '
            '/avs/PD_controller_v3_debug'
        )

        self.get_logger().info(
            f'Output: {self.run_dir}'
        )

        self.get_logger().info(
            'Subscribe-only: '
            'this logger never publishes /cmd_vel'
        )

    # ================================================================
    # AGE
    # ================================================================

    @staticmethod
    def age(d):

        if not isinstance(d, dict):
            return math.inf

        t = finite(
            d.get('_rx')
        )

        if not math.isfinite(t):
            return math.inf

        return (
            now_s()
            -
            t
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def debug_cb(self, c, m):

        d = parse_json(
            m.data
        )

        if not d:
            return

        d['_rx'] = now_s()
        d['_controller'] = c

        self.debug[c] = d

        # Lưu nguyên debug JSON để không mất bất kỳ field nào.
        self.raw_records.append({
            'time_wall_s':
                now_s(),

            'controller':
                c,

            'debug':
                {
                    k: v
                    for k, v
                    in d.items()
                    if not k.startswith('_')
                },
        })

    def ce_cb(self, m):

        d = parse_json(
            m.data
        )

        if d:
            d['_rx'] = now_s()

        self.ce = d

    def cmd_cb(self, m):

        self.cmd = {
            'v':
                float(
                    m.linear.x
                ),

            'w':
                float(
                    m.angular.z
                ),

            '_rx':
                now_s(),
        }

    def odom_cb(self, m):

        self.odom = {
            'x':
                float(
                    m.pose.pose.position.x
                ),

            'y':
                float(
                    m.pose.pose.position.y
                ),

            'yaw':
                yaw(
                    m.pose.pose.orientation
                ),

            'v':
                float(
                    m.twist.twist.linear.x
                ),

            'w':
                float(
                    m.twist.twist.angular.z
                ),

            '_rx':
                now_s(),
        }

    # ================================================================
    # AUTO-DETECT ACTIVE CONTROLLER
    # ================================================================

    def active(self):

        # Có thể ép logger chỉ xem V1 hoặc V2.
        if self.a.controller != 'auto':
            return self.a.controller

        fresh = [
            (
                d.get(
                    '_rx',
                    0.0
                ),
                c
            )

            for c, d
            in self.debug.items()

            if self.age(d)
            <=
            self.a.active_timeout_s
        ]

        if fresh:

            # Newest message wins.
            fresh.sort(
                reverse=True
            )

            self.last_active = (
                fresh[0][1]
            )

            # Nếu cả V1 và V2 cùng đang chạy thì cảnh báo.
            if (
                len(fresh) > 1
                and
                now_s()
                -
                self.last_warn
                >
                3.0
            ):

                text = ', '.join(
                    f'{c}='
                    f'{self.age(self.debug[c]):.2f}s'

                    for _, c
                    in fresh
                )

                self.get_logger().warn(
                    'MULTIPLE FRESH PD CONTROLLERS: '
                    +
                    text
                    +
                    '; showing '
                    +
                    self.last_active
                )

                self.last_warn = (
                    now_s()
                )

        return self.last_active

    def fresh_list(self):

        return [
            c

            for c, d
            in self.debug.items()

            if self.age(d)
            <=
            self.a.active_timeout_s
        ]

    # ================================================================
    # MODE CODE
    # ================================================================

    def encode(
        self,
        group,
        value
    ):

        value = str(
            value
            or
            ''
        )

        if not value:
            return 0

        mp = self.mode_maps[
            group
        ]

        if value not in mp:

            mp[value] = (
                len(mp)
                +
                1
            )

        return mp[
            value
        ]

    # ================================================================
    # BUILD COMMON ROW FOR V1 + V2
    # ================================================================

    def make_row(self):

        c = self.active()

        d = (
            self.debug.get(
                c,
                {}
            )
            if c
            else
            {}
        )

        ce = self.ce
        cmd = self.cmd
        od = self.odom

        epsilon_x_mm = first_finite(
            d.get(
                'epsilon_x_mm'
            ),

            ce.get(
                'epsilon_x_mm'
            ),

            ce.get(
                'x_mm'
            ),
        )

        theta_raw = first_finite(
            d.get(
                'theta_rad'
            ),

            ce.get(
                'theta_rad'
            ),

            ce.get(
                'heading_error_rad'
            ),

            ce.get(
                'e_theta_rad'
            ),
        )

        # ------------------------------------------------------------
        # Filtered lateral
        # ------------------------------------------------------------

        e_x_f_mm = first_finite(
            d.get(
                'e_x_f_mm'
            )
        )

        if not math.isfinite(
            e_x_f_mm
        ):

            value = first_finite(
                d.get(
                    'e_x_f_m'
                )
            )

            e_x_f_mm = (
                value * 1000.0
                if math.isfinite(value)
                else math.nan
            )

        # ------------------------------------------------------------
        # Used lateral
        # ------------------------------------------------------------

        e_x_used_mm = first_finite(
            d.get(
                'e_x_used_mm'
            )
        )

        if not math.isfinite(
            e_x_used_mm
        ):

            value = first_finite(
                d.get(
                    'e_x_used_m'
                )
            )

            e_x_used_mm = (
                value * 1000.0
                if math.isfinite(value)
                else math.nan
            )

        cmd_v = finite(
            cmd.get('v')
        )

        cmd_omega = finite(
            cmd.get('w')
        )

        odom_v = finite(
            od.get('v')
        )

        odom_omega = finite(
            od.get('w')
        )

        mode = str(
            d.get(
                'mode',
                ''
            )
        )

        lane = str(
            first_value(
                d.get(
                    'lane_state'
                ),

                ce.get(
                    'lane_state'
                ),

                default=''
            )
        )

        reason = str(
            d.get(
                'publish_reason',
                ''
            )
        )

        row = {

            # --------------------------------------------------------
            # TIME / CONTROLLER
            # --------------------------------------------------------

            'time_wall_s':
                now_s(),

            't_s':
                now_s()
                -
                self.t0,

            'active_controller':
                c,

            'controller_node':
                str(
                    d.get(
                        'node',
                        ''
                    )
                ),

            'controller_version':
                str(
                    d.get(
                        'version',
                        ''
                    )
                ),

            # --------------------------------------------------------
            # AGE
            # --------------------------------------------------------

            'debug_age_s':
                self.age(
                    d
                ),

            'control_error_age_s':
                self.age(
                    ce
                ),

            'cmd_vel_age_s':
                self.age(
                    cmd
                ),

            'odom_age_s':
                self.age(
                    od
                ),

            'controller_error_age_s':
                finite(
                    d.get(
                        'error_age_s'
                    )
                ),

            # --------------------------------------------------------
            # MODE / STATUS
            # --------------------------------------------------------

            'mode':
                mode,

            'mode_code':
                self.encode(
                    'mode',
                    mode
                ),

            'lane_state':
                lane,

            'lane_state_code':
                self.encode(
                    'lane',
                    lane
                ),

            'publish_reason':
                reason,

            'publish_reason_code':
                self.encode(
                    'publish_reason',
                    reason
                ),

            'enable_motion':
                bint(
                    d.get(
                        'enable_motion'
                    )
                ),

            'raw_valid':
                bint(
                    d.get(
                        'raw_valid',
                        ce.get(
                            'valid',
                            False
                        )
                    )
                ),

            'cmd_published':
                bint(
                    d.get(
                        'cmd_published'
                    )
                ),

            'cmd_vel_conflict':
                bint(
                    d.get(
                        'cmd_vel_conflict'
                    )
                ),

            'raw_valid_reason':
                str(
                    d.get(
                        'raw_valid_reason',
                        ''
                    )
                ),

            'jump_hold_active':
                bint(
                    d.get(
                        'jump_hold_active'
                    )
                ),

            'jump_reason':
                str(
                    d.get(
                        'jump_reason',
                        ''
                    )
                ),

            # --------------------------------------------------------
            # PERCEPTION
            # --------------------------------------------------------

            'fps_est':
                first_finite(
                    d.get(
                        'fps_est'
                    ),

                    ce.get(
                        'fps'
                    ),

                    ce.get(
                        'fps_est'
                    ),

                    ce.get(
                        'vision_fps'
                    ),
                ),

            'confidence':
                first_finite(
                    d.get(
                        'confidence'
                    ),

                    ce.get(
                        'confidence'
                    ),
                ),

            'epsilon_x_mm':
                epsilon_x_mm,

            'e_x_f_mm':
                e_x_f_mm,

            'e_x_used_mm':
                e_x_used_mm,

            'theta_raw_rad':
                theta_raw,

            'theta_f_rad':
                finite(
                    d.get(
                        'theta_f_rad'
                    )
                ),

            'theta_used_rad':
                finite(
                    d.get(
                        'theta_used_rad'
                    )
                ),

            'de_x_f':
                finite(
                    d.get(
                        'de_x_f'
                    )
                ),

            'dtheta_f':
                finite(
                    d.get(
                        'dtheta_f'
                    )
                ),

            # --------------------------------------------------------
            # GAINS / PD
            #
            # V1: p/d components = NaN vì V1 chưa publish chúng.
            # V2: đầy đủ.
            # --------------------------------------------------------

            'k_lat_used':
                finite(
                    d.get(
                        'k_lat_used'
                    )
                ),

            'k_theta_used':
                finite(
                    d.get(
                        'k_theta_used'
                    )
                ),

            'kd_lat':
                finite(
                    d.get(
                        'kd_lat'
                    )
                ),

            'kd_theta':
                finite(
                    d.get(
                        'kd_theta'
                    )
                ),

            'p_lat':
                finite(
                    d.get(
                        'p_lat'
                    )
                ),

            'p_theta':
                finite(
                    d.get(
                        'p_theta'
                    )
                ),

            'd_lat':
                finite(
                    d.get(
                        'd_lat'
                    )
                ),

            'd_theta':
                finite(
                    d.get(
                        'd_theta'
                    )
                ),

            # --------------------------------------------------------
            # CURVE / ZONES
            # --------------------------------------------------------

            'curve_confirmed':
                bint(
                    d.get(
                        'curve_confirmed'
                    )
                ),

            'curve_evidence':
                finite(
                    d.get(
                        'curve_evidence'
                    )
                ),

            'curve_count':
                finite(
                    d.get(
                        'curve_count'
                    )
                ),

            'curve_sign':
                finite(
                    d.get(
                        'curve_sign'
                    )
                ),

            'center_zone':
                bint(
                    d.get(
                        'center_zone'
                    )
                ),

            'near_zone':
                bint(
                    d.get(
                        'near_zone'
                    )
                ),

            'large_error':
                bint(
                    d.get(
                        'large_error'
                    )
                ),

            'slow_factor':
                finite(
                    d.get(
                        'slow_factor'
                    )
                ),

            # --------------------------------------------------------
            # GEOMETRY
            # --------------------------------------------------------

            'kappa_m':
                finite(
                    d.get(
                        'kappa_m'
                    )
                ),

            'L_d_m':
                first_finite(
                    d.get(
                        'L_d_m'
                    ),

                    ce.get(
                        'lookahead_m'
                    ),
                ),

            # --------------------------------------------------------
            # LINEAR
            # --------------------------------------------------------

            'v_des':
                finite(
                    d.get(
                        'v_des'
                    )
                ),

            'v_ref':
                finite(
                    d.get(
                        'v_ref'
                    )
                ),

            'v_cmd_internal':
                finite(
                    d.get(
                        'v_cmd'
                    )
                ),

            'cmd_v':
                cmd_v,

            'odom_v':
                odom_v,

            # --------------------------------------------------------
            # ANGULAR
            # --------------------------------------------------------

            'omega_raw':
                finite(
                    d.get(
                        'omega_raw'
                    )
                ),

            'omega_des':
                finite(
                    d.get(
                        'omega_des'
                    )
                ),

            'omega_limit':
                finite(
                    d.get(
                        'omega_limit'
                    )
                ),

            'omega_ref':
                finite(
                    d.get(
                        'omega_ref'
                    )
                ),

            'omega_cmd_internal':
                finite(
                    d.get(
                        'omega_cmd'
                    )
                ),

            'cmd_omega':
                cmd_omega,

            'odom_omega':
                odom_omega,

            # --------------------------------------------------------
            # WHEEL ESTIMATE
            # --------------------------------------------------------

            'v_left_est':
                finite(
                    d.get(
                        'v_left_est'
                    )
                ),

            'v_right_est':
                finite(
                    d.get(
                        'v_right_est'
                    )
                ),

            'delta_v_cmd':
                finite(
                    d.get(
                        'delta_v_cmd'
                    )
                ),

            # --------------------------------------------------------
            # ODOM PATH
            # --------------------------------------------------------

            'odom_x':
                finite(
                    od.get(
                        'x'
                    )
                ),

            'odom_y':
                finite(
                    od.get(
                        'y'
                    )
                ),

            'odom_yaw':
                finite(
                    od.get(
                        'yaw'
                    )
                ),
        }

        # ============================================================
        # TRACKING ERROR
        # ============================================================

        row[
            'linear_tracking_error'
        ] = (
            cmd_v
            -
            odom_v

            if
            math.isfinite(cmd_v)
            and
            math.isfinite(odom_v)

            else
            math.nan
        )

        row[
            'angular_tracking_error'
        ] = (
            cmd_omega
            -
            odom_omega

            if
            math.isfinite(cmd_omega)
            and
            math.isfinite(odom_omega)

            else
            math.nan
        )

        # ============================================================
        # SAVE ALL ORIGINAL DEBUG FIELDS INTO CSV
        #
        # Example:
        #
        # debug__curve_count
        # debug__lane_state_debug
        # debug__cmd_vel_publishers
        #
        # => không mất telemetry khi controller thay đổi sau này.
        # ============================================================

        for k, v in d.items():

            if not k.startswith('_'):

                row[
                    f'debug__{k}'
                ] = csv_scalar(
                    v
                )

        return row

    def log_snapshot(self):

        self.records.append(
            self.make_row()
        )

    # ================================================================
    # DISPLAY WINDOW
    # ================================================================

    def window(self):

        if (
            not self.records
            or
            self.a.window_s <= 0
        ):
            return self.records

        cutoff = (
            self.records[-1]['t_s']
            -
            self.a.window_s
        )

        return [
            row

            for row
            in self.records

            if row['t_s']
            >=
            cutoff
        ]

    @staticmethod
    def col(
        rows,
        key
    ):

        return [
            finite(
                row.get(
                    key
                )
            )

            for row
            in rows
        ]

    @staticmethod
    def axis_setup(
        axis,
        title,
        ylabel
    ):

        axis.set_title(
            title
        )

        axis.set_xlabel(
            'time [s]'
        )

        axis.set_ylabel(
            ylabel
        )

        axis.grid(
            True
        )

    # ================================================================
    # BUILD DASHBOARD
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
            left=.055,
            right=.985,
            top=.94,
            bottom=.06,
            hspace=.38,
            wspace=.25
        )

        fig.suptitle(
            title,
            fontsize=15
        )

        try:

            fig.canvas.manager.set_window_title(
                'AVS Unified PD Controller Logger'
            )

        except Exception:
            pass

        names = [
            'lat',
            'head',
            'specific',

            'linear',
            'angular',
            'deriv',

            'zones',
            'geom',
            'tracking',

            'wheels',
            'path',
            'status',
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

        self.axis_setup(
            axes['lat'],
            '1. Lateral error',
            'mm'
        )

        for k, label in [
            (
                'epsilon_x_mm',
                'raw'
            ),
            (
                'e_x_f_mm',
                'filtered'
            ),
            (
                'e_x_used_mm',
                'used'
            ),
        ]:

            add(
                'lat',
                k,
                label
            )

        # ============================================================
        # 2. HEADING
        # ============================================================

        self.axis_setup(
            axes['head'],
            '2. Heading error',
            'rad'
        )

        for k, label in [
            (
                'theta_raw_rad',
                'raw'
            ),
            (
                'theta_f_rad',
                'filtered'
            ),
            (
                'theta_used_rad',
                'used'
            ),
        ]:

            add(
                'head',
                k,
                label
            )

        # ============================================================
        # 3. CONTROLLER SPECIFIC
        #
        # V1:
        #   K lat / K theta / omega desired / omega limit
        #
        # V2:
        #   P lat / P theta / D lat / D theta
        # ============================================================

        self.axis_setup(
            axes['specific'],
            '3. Controller-specific steering',
            'value'
        )

        for k, label in [
            (
                'p_lat',
                'P lateral'
            ),
            (
                'p_theta',
                'P heading'
            ),
            (
                'd_lat',
                'D lateral'
            ),
            (
                'd_theta',
                'D heading'
            ),
            (
                'k_lat_used',
                'K lateral'
            ),
            (
                'k_theta_used',
                'K heading'
            ),
            (
                'omega_des',
                'omega desired'
            ),
            (
                'omega_limit',
                'omega limit'
            ),
        ]:

            add(
                'specific',
                k,
                label
            )

        # ============================================================
        # 4. LINEAR
        # ============================================================

        self.axis_setup(
            axes['linear'],
            '4. Linear velocity',
            'm/s'
        )

        for k, label in [
            (
                'v_des',
                'v desired'
            ),
            (
                'v_ref',
                'v reference'
            ),
            (
                'v_cmd_internal',
                'controller'
            ),
            (
                'cmd_v',
                '/cmd_vel'
            ),
            (
                'odom_v',
                'odom'
            ),
        ]:

            add(
                'linear',
                k,
                label
            )

        # ============================================================
        # 5. ANGULAR
        # ============================================================

        self.axis_setup(
            axes['angular'],
            '5. Angular velocity',
            'rad/s'
        )

        for k, label in [
            (
                'omega_raw',
                'PD raw'
            ),
            (
                'omega_des',
                'desired'
            ),
            (
                'omega_ref',
                'reference'
            ),
            (
                'omega_cmd_internal',
                'controller'
            ),
            (
                'cmd_omega',
                '/cmd_vel'
            ),
            (
                'odom_omega',
                'odom'
            ),
        ]:

            add(
                'angular',
                k,
                label
            )

        # ============================================================
        # 6. DERIVATIVE
        # ============================================================

        self.axis_setup(
            axes['deriv'],
            '6. Derivative signals',
            'value / s'
        )

        add(
            'deriv',
            'de_x_f',
            'de_x'
        )

        add(
            'deriv',
            'dtheta_f',
            'dtheta'
        )

        # ============================================================
        # 7. ZONES
        # ============================================================

        self.axis_setup(
            axes['zones'],
            '7. Controller zones',
            'value / code'
        )

        for k, label in [
            (
                'curve_confirmed',
                'curve'
            ),
            (
                'curve_evidence',
                'evidence'
            ),
            (
                'curve_count',
                'count'
            ),
            (
                'center_zone',
                'center'
            ),
            (
                'near_zone',
                'near'
            ),
            (
                'large_error',
                'large error'
            ),
            (
                'mode_code',
                'mode code'
            ),
        ]:

            add(
                'zones',
                k,
                label
            )

        # ============================================================
        # 8. VISION / GEOMETRY
        # ============================================================

        self.axis_setup(
            axes['geom'],
            '8. Vision / geometry',
            'value'
        )

        for k, label in [
            (
                'fps_est',
                'FPS'
            ),
            (
                'confidence',
                'confidence'
            ),
            (
                'kappa_m',
                'curvature'
            ),
            (
                'L_d_m',
                'lookahead'
            ),
            (
                'slow_factor',
                'speed factor'
            ),
        ]:

            add(
                'geom',
                k,
                label
            )

        # ============================================================
        # 9. TRACKING ERROR
        # ============================================================

        self.axis_setup(
            axes['tracking'],
            '9. Command tracking error',
            'error'
        )

        add(
            'tracking',
            'linear_tracking_error',
            'linear cmd-odom'
        )

        add(
            'tracking',
            'angular_tracking_error',
            'angular cmd-odom'
        )

        # ============================================================
        # 10. WHEEL ESTIMATE
        # ============================================================

        self.axis_setup(
            axes['wheels'],
            '10. Estimated left/right group command',
            'm/s'
        )

        add(
            'wheels',
            'v_left_est',
            'left'
        )

        add(
            'wheels',
            'v_right_est',
            'right'
        )

        add(
            'wheels',
            'delta_v_cmd',
            'right-left'
        )

        # ============================================================
        # 11. ODOM PATH
        # ============================================================

        axes[
            'path'
        ].set_title(
            '11. Odom path'
        )

        axes[
            'path'
        ].set_xlabel(
            'x [m]'
        )

        axes[
            'path'
        ].set_ylabel(
            'y [m]'
        )

        axes[
            'path'
        ].grid(
            True
        )

        path_line, = axes[
            'path'
        ].plot(
            [],
            [],
            label='odom path'
        )

        lines[
            'path'
        ][
            'path'
        ] = path_line

        axes[
            'path'
        ].legend(
            fontsize=7
        )

        axes[
            'path'
        ].set_aspect(
            'equal',
            adjustable='datalim'
        )

        # ============================================================
        # 12. STATUS
        # ============================================================

        axes[
            'status'
        ].set_title(
            '12. Current PD status'
        )

        axes[
            'status'
        ].axis(
            'off'
        )

        status = axes[
            'status'
        ].text(
            .01,
            .99,
            'Waiting...',
            transform=(
                axes[
                    'status'
                ].transAxes
            ),
            va='top',
            ha='left',
            family='monospace',
            fontsize=8.2
        )

        for panel, axis in (
            axes.items()
        ):

            if (
                panel != 'status'
                and
                lines[panel]
            ):

                axis.legend(
                    fontsize=7
                )

        return (
            fig,
            axes,
            lines,
            status
        )

    # ================================================================
    # SHOW/HIDE VERSION-SPECIFIC SIGNALS
    # ================================================================

    @staticmethod
    def visibility(
        lines,
        panel,
        keys
    ):

        wanted = set(
            keys
        )

        for k, line in (
            lines[
                panel
            ].items()
        ):

            line.set_visible(
                k
                in
                wanted
            )

    def adapt(
        self,
        c,
        axes,
        lines
    ):

        if c == 'v1':

            axes[
                'specific'
            ].set_title(
                '3. V1 steering gains / limits'
            )

            self.visibility(
                lines,
                'specific',
                [
                    'k_lat_used',
                    'k_theta_used',
                    'omega_des',
                    'omega_limit',
                ]
            )

            zone_title = (
                '7. V1 curve / controller zones'
            )

            self.visibility(
                lines,
                'zones',
                [
                    'curve_confirmed',
                    'curve_count',
                    'center_zone',
                    'near_zone',
                    'large_error',
                    'mode_code',
                ]
            )

        elif c in ('v2', 'v3'):

            axes[
                'specific'
            ].set_title(
                '3. V2 pure PD steering components'
            )

            self.visibility(
                lines,
                'specific',
                [
                    'p_lat',
                    'p_theta',
                    'd_lat',
                    'd_theta',
                    'omega_des',
                    'omega_limit',
                ]
            )

            zone_title = (
                '7. V2 curve evidence / controller zones'
            )

            self.visibility(
                lines,
                'zones',
                [
                    'curve_confirmed',
                    'curve_evidence',
                    'center_zone',
                    'near_zone',
                    'large_error',
                    'mode_code',
                ]
            )

        else:

            axes[
                'specific'
            ].set_title(
                '3. Controller-specific steering'
            )

            self.visibility(
                lines,
                'specific',
                lines[
                    'specific'
                ].keys()
            )

            zone_title = (
                '7. Controller zones'
            )

            self.visibility(
                lines,
                'zones',
                lines[
                    'zones'
                ].keys()
            )

        # Rebuild legends because hidden lines should not appear.
        for panel in [
            'specific',
            'zones',
        ]:

            old_legend = (
                axes[
                    panel
                ].get_legend()
            )

            if old_legend:
                old_legend.remove()

            visible_lines = [
                line

                for line
                in lines[
                    panel
                ].values()

                if line.get_visible()
            ]

            if visible_lines:

                axes[
                    panel
                ].legend(
                    fontsize=7
                )

        return zone_title

    # ================================================================
    # RENDER
    # ================================================================

    def render(
        self,
        fig,
        axes,
        lines,
        status,
        rows
    ):

        if not rows:
            return

        latest = rows[
            -1
        ]

        controller = str(
            latest.get(
                'active_controller'
            )
            or
            ''
        )

        zone_title = self.adapt(
            controller,
            axes,
            lines
        )

        controller_title = (
            CONTROLLERS.get(
                controller,
                {}
            ).get(
                'title',
                'PD Controller'
            )
        )

        version = str(
            latest.get(
                'controller_version'
            )
            or
            ''
        )

        fig.suptitle(
            'AVS '
            +
            controller_title
            +
            (
                f' | {version}'
                if version
                else
                ''
            ),
            fontsize=15
        )

        t = self.col(
            rows,
            't_s'
        )

        for panel in [
            'lat',
            'head',
            'specific',
            'linear',
            'angular',
            'deriv',
            'zones',
            'geom',
            'tracking',
            'wheels',
        ]:

            for k, line in (
                lines[
                    panel
                ].items()
            ):

                line.set_data(
                    t,
                    self.col(
                        rows,
                        k
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
        # ODOM PATH
        # ============================================================

        lines[
            'path'
        ][
            'path'
        ].set_data(
            self.col(
                rows,
                'odom_x'
            ),

            self.col(
                rows,
                'odom_y'
            )
        )

        axes[
            'path'
        ].relim()

        axes[
            'path'
        ].autoscale_view()

        axes[
            'zones'
        ].set_title(
            zone_title
            +
            ' | mode='
            +
            str(
                latest.get(
                    'mode',
                    ''
                )
            )
        )

        # ============================================================
        # STATUS PANEL
        # ============================================================

        multiple = (
            ' !!! MULTIPLE FRESH !!!'
            if len(
                self.fresh_list()
            ) > 1
            else
            ''
        )

        status.set_text(

            f"active      : "
            f"{controller}"
            f"{multiple}\n"

            f"node        : "
            f"{latest.get('controller_node', '')}\n"

            f"version     : "
            f"{version}\n"

            f"debug age   : "
            f"{finite(latest.get('debug_age_s')):7.3f} s\n"

            f"\n"

            f"mode        : "
            f"{latest.get('mode', '')}\n"

            f"valid/cmd   : "
            f"{latest.get('raw_valid', 0)} / "
            f"{latest.get('cmd_published', 0)}\n"

            f"publish     : "
            f"{latest.get('publish_reason', '')}\n"

            f"conflict    : "
            f"{latest.get('cmd_vel_conflict', 0)}\n"

            f"lane        : "
            f"{latest.get('lane_state', '')}\n"

            f"FPS/conf    : "
            f"{finite(latest.get('fps_est')):7.2f} / "
            f"{finite(latest.get('confidence')):5.2f}\n"

            f"\n"

            f"e_x         : "
            f"{finite(latest.get('e_x_used_mm')):8.2f} mm\n"

            f"theta       : "
            f"{finite(latest.get('theta_used_rad')):8.3f} rad\n"

            f"de_x/dtheta : "
            f"{finite(latest.get('de_x_f')):7.3f} / "
            f"{finite(latest.get('dtheta_f')):7.3f}\n"

            f"\n"

            f"K lat/theta : "
            f"{finite(latest.get('k_lat_used')):7.3f} / "
            f"{finite(latest.get('k_theta_used')):7.3f}\n"

            f"P lat/theta : "
            f"{finite(latest.get('p_lat')):7.3f} / "
            f"{finite(latest.get('p_theta')):7.3f}\n"

            f"D lat/theta : "
            f"{finite(latest.get('d_lat')):7.3f} / "
            f"{finite(latest.get('d_theta')):7.3f}\n"

            f"\n"

            f"v des/ref   : "
            f"{finite(latest.get('v_des')):7.3f} / "
            f"{finite(latest.get('v_ref')):7.3f}\n"

            f"v cmd/odom  : "
            f"{finite(latest.get('cmd_v')):7.3f} / "
            f"{finite(latest.get('odom_v')):7.3f}\n"

            f"w des/ref   : "
            f"{finite(latest.get('omega_des')):7.3f} / "
            f"{finite(latest.get('omega_ref')):7.3f}\n"

            f"w cmd/odom  : "
            f"{finite(latest.get('cmd_omega')):7.3f} / "
            f"{finite(latest.get('odom_omega')):7.3f}\n"

            f"\n"

            f"curve       : "
            f"{latest.get('curve_confirmed', 0)} "

            f"evidence="
            f"{finite(latest.get('curve_evidence')):3.0f} "

            f"count="
            f"{finite(latest.get('curve_count')):3.0f}\n"

            f"omega limit : "
            f"{finite(latest.get('omega_limit')):7.3f}\n"

            f"slow factor : "
            f"{finite(latest.get('slow_factor')):7.3f}\n"

            f"jump hold   : "
            f"{latest.get('jump_hold_active', 0)}"
        )

        try:
            fig.canvas.draw_idle()
        except Exception:
            pass

    # ================================================================
    # LIVE UPDATE
    # ================================================================

    def update_live(self):

        if (
            self.fig is None
            or
            self.window_closed
        ):
            return

        rows = self.window()

        if not rows:
            return

        self.render(
            self.fig,
            self.axes,
            self.lines,
            self.status,
            rows
        )

        try:
            self.fig.canvas.flush_events()
        except Exception:
            pass

    # ================================================================
    # PREVENT PLOT WINDOW STEALING FOCUS
    # ================================================================

    def no_focus(self):

        try:

            window = getattr(
                self.fig.canvas.manager,
                'window',
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
    # Không save trực tiếp figure đang bị destroy.
    # finally sẽ dựng figure mới và lưu PNG.
    # ================================================================

    def on_close(self, _):

        self.window_closed = True
        self.stop = True

    # ================================================================
    # CSV
    # ================================================================

    def save_csv(self):

        if not self.records:
            return

        preferred = [

            'time_wall_s',
            't_s',

            'active_controller',
            'controller_node',
            'controller_version',

            'mode',
            'lane_state',
            'publish_reason',

            'enable_motion',
            'raw_valid',
            'cmd_published',
            'cmd_vel_conflict',
            'raw_valid_reason',

            'jump_hold_active',
            'jump_reason',

            'fps_est',
            'confidence',

            'epsilon_x_mm',
            'e_x_f_mm',
            'e_x_used_mm',

            'theta_raw_rad',
            'theta_f_rad',
            'theta_used_rad',

            'de_x_f',
            'dtheta_f',

            'k_lat_used',
            'k_theta_used',
            'kd_lat',
            'kd_theta',

            'p_lat',
            'p_theta',
            'd_lat',
            'd_theta',

            'curve_confirmed',
            'curve_evidence',
            'curve_count',
            'curve_sign',

            'center_zone',
            'near_zone',
            'large_error',

            'slow_factor',

            'kappa_m',
            'L_d_m',

            'v_des',
            'v_ref',
            'v_cmd_internal',
            'cmd_v',
            'odom_v',
            'linear_tracking_error',

            'omega_raw',
            'omega_des',
            'omega_limit',
            'omega_ref',
            'omega_cmd_internal',
            'cmd_omega',
            'odom_omega',
            'angular_tracking_error',

            'v_left_est',
            'v_right_est',
            'delta_v_cmd',

            'odom_x',
            'odom_y',
            'odom_yaw',

            'debug_age_s',
            'control_error_age_s',
            'cmd_vel_age_s',
            'odom_age_s',
            'controller_error_age_s',
        ]

        columns = []
        seen = set()

        for k in preferred:

            if k not in seen:
                columns.append(k)
                seen.add(k)

        # Include all debug__* keys discovered during run.
        for row in self.records:

            for k in row:

                if k not in seen:
                    columns.append(k)
                    seen.add(k)

        with self.csv_path.open(
            'w',
            newline='',
            encoding='utf-8'
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=columns
            )

            writer.writeheader()

            writer.writerows(
                self.records
            )

    # ================================================================
    # MODE MAP
    # ================================================================

    def save_maps(self):

        obj = {
            group:
                {
                    str(code):
                        name

                    for name, code
                    in mapping.items()
                }

            for group, mapping
            in self.mode_maps.items()
        }

        self.map_path.write_text(
            json.dumps(
                obj,
                indent=2,
                ensure_ascii=False
            ),
            encoding='utf-8'
        )

    # ================================================================
    # RAW DEBUG JSONL
    # ================================================================

    def save_raw(self):

        with self.raw_path.open(
            'w',
            encoding='utf-8'
        ) as file:

            for item in self.raw_records:

                file.write(
                    json.dumps(
                        item,
                        ensure_ascii=False
                    )
                    +
                    '\n'
                )

    # ================================================================
    # FINAL PNG
    #
    # Dựng FIGURE MỚI từ toàn bộ records.
    #
    # Vì vậy:
    #   Ctrl+C  -> vẫn lưu PNG
    #   bấm X   -> vẫn lưu PNG
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
            status,
        ) = self.make_dashboard(
            'Final Unified PD Dashboard'
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
                bbox_inches='tight'
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

        # Không save PNG liên tục để tránh giật GUI.
        self.save_csv()
        self.save_maps()
        self.save_raw()

    def save_all(self):

        self.autosave()

        self.save_png()

        print()
        print(
            '=' * 60
        )

        print(
            'UNIFIED PD CONTROLLER LOG SAVED'
        )

        print(
            f'Folder    : '
            f'{self.run_dir}'
        )

        print(
            f'CSV       : '
            f'{self.csv_path}'
        )

        print(
            f'PNG       : '
            f'{self.png_path}'
        )

        print(
            f'Mode map  : '
            f'{self.map_path}'
        )

        print(
            f'Raw debug : '
            f'{self.raw_path}'
        )

        print(
            '=' * 60
        )

        print()


# ================================================================
# ARGUMENTS
# ================================================================

def build_parser():

    parser = argparse.ArgumentParser(
        description=(
            'Unified realtime logger for '
            'PD_controller_v1/v2'
        )
    )

    parser.add_argument(
        '--output-dir',
        default=(
            '/home/bluedstar/'
            'SimpleRobot/'
            'terminal-run/'
            'plot_logger'
        )
    )

    parser.add_argument(
        '--controller',
        choices=[
            'auto',
            'v1',
            'v2',
            'v3',
        ],
        default='auto'
    )

    parser.add_argument(
        '--active-timeout-s',
        type=float,
        default=1.0
    )

    parser.add_argument(
        '--control-error-topic',
        default='/avs/control_error'
    )

    parser.add_argument(
        '--cmd-vel-topic',
        default='/cmd_vel'
    )

    parser.add_argument(
        '--odom-topic',
        default='/odom_raw'
    )

    parser.add_argument(
        '--window-s',
        type=float,
        default=90.0
    )

    parser.add_argument(
        '--log-hz',
        type=float,
        default=20.0
    )

    parser.add_argument(
        '--plot-hz',
        type=float,
        default=4.0
    )

    parser.add_argument(
        '--autosave-s',
        type=float,
        default=10.0
    )

    parser.add_argument(
        '--duration-s',
        type=float,
        default=0.0
    )

    parser.add_argument(
        '--dpi',
        type=int,
        default=160
    )

    parser.add_argument(
        '--no-live',
        action='store_true'
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

    node = PDLogger(
        args
    )

    def stop_handler(
        *_args
    ):

        node.stop = True

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
            not node.stop
        ):

            rclpy.spin_once(
                node,
                timeout_sec=.005
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
            # AUTOSAVE CSV / RAW
            # --------------------------------------------------------

            if (
                args.autosave_s > 0
                and
                current
                -
                node.last_save
                >=
                args.autosave_s
            ):

                node.autosave()

                node.last_save = (
                    current
                )

            # --------------------------------------------------------
            # OPTIONAL DURATION
            # --------------------------------------------------------

            if (
                args.duration_s > 0
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
                node.fig is not None
            ):

                try:

                    node.fig.canvas.flush_events()

                except Exception:
                    pass

            time.sleep(
                .003
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
                    'all'
                )

            except Exception:
                pass


if __name__ == '__main__':

    main()
