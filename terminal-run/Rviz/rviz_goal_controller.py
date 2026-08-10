#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


# ================================================================
# UTILITIES
# ================================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


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


class RVizGoalController(Node):

    def __init__(self):

        super().__init__(
            'rviz_goal_controller'
        )

        # ============================================================
        # TOPICS / FRAMES
        # ============================================================

        self.declare_parameter(
            'goal_topic',
            '/goal_pose'
        )

        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel'
        )

        # Chính là Fixed Frame đang dùng trong RViz
        self.declare_parameter(
            'fixed_frame',
            'odom_frame'
        )

        # Robot pose được lấy từ:
        #
        # odom_frame -> base_footprint
        #
        # TF này do odom_raw_to_tf.py của bạn tạo.
        self.declare_parameter(
            'base_frame',
            'base_footprint'
        )

        # ============================================================
        # CONTROL FREQUENCY
        # ============================================================

        self.declare_parameter(
            'control_hz',
            30.0
        )

        # ============================================================
        # LINEAR SPEED
        # ============================================================

        self.declare_parameter(
            'v_max',
            0.10
        )

        self.declare_parameter(
            'v_min',
            0.030
        )

        # Khi đang xoay về phía goal
        self.declare_parameter(
            'v_turn_max',
            0.055
        )

        # ============================================================
        # ANGULAR SPEED
        # ============================================================

        self.declare_parameter(
            'omega_max',
            0.32
        )

        # Khi xoay tại chỗ trước khi đi
        self.declare_parameter(
            'omega_rotate_max',
            0.26
        )

        # Tốc độ quay tối thiểu để thắng ma sát
        self.declare_parameter(
            'omega_min',
            0.065
        )

        # ============================================================
        # CONTROLLER GAINS
        # ============================================================

        # Điều khiển khoảng cách
        self.declare_parameter(
            'kp_distance',
            0.55
        )

        # Điều khiển hướng khi đang chạy
        self.declare_parameter(
            'kp_heading_drive',
            0.75
        )

        # Điều khiển hướng khi quay tại chỗ
        self.declare_parameter(
            'kp_heading_rotate',
            0.80
        )

        # ============================================================
        # GOAL TOLERANCE
        # ============================================================

        # Đến cách goal <= 5 cm -> coi như đến
        self.declare_parameter(
            'goal_tolerance_m',
            0.05
        )

        # ============================================================
        # ROTATE-FIRST STATE MACHINE
        #
        # Xe quay trước nếu sai hướng lớn.
        #
        # Dùng 2 threshold khác nhau để tạo hysteresis,
        # tránh liên tục DRIVE <-> ROTATE gây lắc.
        # ============================================================

        # > 35 deg:
        # chuyển sang ROTATE
        self.declare_parameter(
            'rotate_enter_angle_deg',
            35.0
        )

        # < 8 deg:
        # kết thúc ROTATE và bắt đầu chạy
        self.declare_parameter(
            'rotate_exit_angle_deg',
            8.0
        )

        # ============================================================
        # DRIVE HEADING
        # ============================================================

        # Nếu sai góc > mức này khi chạy:
        # giảm tốc rất mạnh.
        self.declare_parameter(
            'large_heading_error_deg',
            25.0
        )

        # ============================================================
        # SLOW DOWN NEAR GOAL
        # ============================================================

        self.declare_parameter(
            'slow_distance_m',
            0.30
        )

        # ============================================================
        # FINAL HEADING
        #
        # false:
        #   tới đúng vị trí -> STOP.
        #
        # true:
        #   tới vị trí rồi quay theo hướng người dùng
        #   kéo trong 2D Goal Pose.
        #
        # Hiện tại để FALSE theo yêu cầu của bạn.
        # ============================================================

        self.declare_parameter(
            'align_final_heading',
            False
        )

        self.declare_parameter(
            'final_heading_tolerance_deg',
            6.0
        )

        # ============================================================
        # SMOOTHING / RATE LIMIT
        # ============================================================

        self.declare_parameter(
            'max_linear_accel',
            0.16
        )

        self.declare_parameter(
            'max_linear_decel',
            0.30
        )

        self.declare_parameter(
            'max_angular_accel',
            0.45
        )

        # ============================================================
        # CALIBRATION
        #
        # Giữ theo thông số robot của bạn.
        # ============================================================

        self.declare_parameter(
            'linear_cmd_scale',
            1.245
        )

        self.declare_parameter(
            'angular_cmd_scale',
            0.75
        )

        # ============================================================
        # READ PARAMETER NAMES
        # ============================================================

        self.goal_topic = str(
            self.get_parameter(
                'goal_topic'
            ).value
        )

        self.cmd_vel_topic = str(
            self.get_parameter(
                'cmd_vel_topic'
            ).value
        )

        self.fixed_frame = str(
            self.get_parameter(
                'fixed_frame'
            ).value
        )

        self.base_frame = str(
            self.get_parameter(
                'base_frame'
            ).value
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # PUBLISHER
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # ============================================================
        # GOAL SUBSCRIBER
        # ============================================================

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )

        # ============================================================
        # GOAL STATE
        # ============================================================

        self.has_goal = False

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        # ============================================================
        # CONTROL STATE
        # ============================================================

        # WAIT_GOAL
        # ROTATE_TO_GOAL
        # DRIVE_TO_GOAL
        # ALIGN_FINAL
        self.control_state = 'WAIT_GOAL'

        # ============================================================
        # LAST COMMAND
        # ============================================================

        self.last_v = 0.0
        self.last_w = 0.0

        # ============================================================
        # LOG CONTROL
        # ============================================================

        self.last_status_log_ns = 0

        # ============================================================
        # TIMER
        # ============================================================

        hz = float(
            self.get_parameter(
                'control_hz'
            ).value
        )

        self.timer = self.create_timer(
            1.0 / hz,
            self.control_loop
        )

        # ============================================================
        # STARTUP LOG
        # ============================================================

        self.get_logger().info(
            '======================================='
        )

        self.get_logger().info(
            'RViz Goal Controller started'
        )

        self.get_logger().info(
            f'Goal topic  : {self.goal_topic}'
        )

        self.get_logger().info(
            f'Cmd topic   : {self.cmd_vel_topic}'
        )

        self.get_logger().info(
            f'Fixed frame : {self.fixed_frame}'
        )

        self.get_logger().info(
            f'Base frame  : {self.base_frame}'
        )

        self.get_logger().info(
            'Robot pose source: TF'
        )

        self.get_logger().info(
            f'{self.fixed_frame} -> {self.base_frame}'
        )

        self.get_logger().info(
            'Final heading alignment: OFF by default'
        )

        self.get_logger().info(
            '======================================='
        )

    # ================================================================
    # GOAL CALLBACK
    # ================================================================

    def goal_callback(self, msg):

        # ------------------------------------------------------------
        # Kiểm tra frame.
        #
        # Với RViz Fixed Frame = odom_frame,
        # goal cũng phải ở odom_frame.
        # ------------------------------------------------------------

        goal_frame = (
            msg.header.frame_id.strip()
        )

        if (
            goal_frame
            and
            goal_frame != self.fixed_frame
        ):

            self.get_logger().error(
                f'Goal received in frame "{goal_frame}", '
                f'but controller uses "{self.fixed_frame}". '
                f'Set RViz Fixed Frame = {self.fixed_frame}.'
            )

            self.stop_immediately()

            self.has_goal = False

            return

        self.goal_x = float(
            msg.pose.position.x
        )

        self.goal_y = float(
            msg.pose.position.y
        )

        self.goal_yaw = yaw_from_quaternion(
            msg.pose.orientation
        )

        self.has_goal = True

        # Goal mới luôn đánh giá lại hướng.
        self.control_state = 'ROTATE_TO_GOAL'

        self.get_logger().info(
            '---------------------------------------'
        )

        self.get_logger().info(
            'NEW RVIZ GOAL'
        )

        self.get_logger().info(
            f'x = {self.goal_x:.3f} m'
        )

        self.get_logger().info(
            f'y = {self.goal_y:.3f} m'
        )

        self.get_logger().info(
            f'yaw = '
            f'{math.degrees(self.goal_yaw):.1f} deg'
        )

        self.get_logger().info(
            f'frame = {self.fixed_frame}'
        )

        self.get_logger().info(
            '---------------------------------------'
        )

    # ================================================================
    # GET ROBOT POSE FROM TF
    # ================================================================

    def get_robot_pose(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(
                        seconds=0.05
                    )
                )
            )

        except Exception:
            return None

        x = float(
            transform.transform.translation.x
        )

        y = float(
            transform.transform.translation.y
        )

        yaw = yaw_from_quaternion(
            transform.transform.rotation
        )

        return (
            x,
            y,
            yaw
        )

    # ================================================================
    # RATE LIMIT
    # ================================================================

    def rate_limit(
        self,
        target,
        current,
        rate_up,
        rate_down,
        dt
    ):

        delta = (
            target -
            current
        )

        # ------------------------------------------------------------
        # Nếu độ lớn command đang tăng -> rate_up
        # Nếu đang giảm -> rate_down
        # ------------------------------------------------------------

        if abs(target) > abs(current):
            max_change = (
                rate_up *
                dt
            )

        else:
            max_change = (
                rate_down *
                dt
            )

        delta = clamp(
            delta,
            -max_change,
            max_change
        )

        return (
            current +
            delta
        )

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # Không có goal
        # ------------------------------------------------------------

        if not self.has_goal:
            return

        # ------------------------------------------------------------
        # Lấy robot pose từ TF
        # ------------------------------------------------------------

        pose = self.get_robot_pose()

        if pose is None:

            self.stop_immediately()

            return

        robot_x, robot_y, robot_yaw = pose

        # ============================================================
        # ERROR
        # ============================================================

        dx = (
            self.goal_x -
            robot_x
        )

        dy = (
            self.goal_y -
            robot_y
        )

        distance = math.hypot(
            dx,
            dy
        )

        # ============================================================
        # Góc đường thẳng từ robot đến goal
        # ============================================================

        desired_yaw = math.atan2(
            dy,
            dx
        )

        heading_error = normalize_angle(
            desired_yaw -
            robot_yaw
        )

        # ============================================================
        # PARAMETERS
        # ============================================================

        goal_tol = float(
            self.get_parameter(
                'goal_tolerance_m'
            ).value
        )

        rotate_enter = math.radians(
            float(
                self.get_parameter(
                    'rotate_enter_angle_deg'
                ).value
            )
        )

        rotate_exit = math.radians(
            float(
                self.get_parameter(
                    'rotate_exit_angle_deg'
                ).value
            )
        )

        # ============================================================
        # GOAL POSITION REACHED
        # ============================================================

        if distance <= goal_tol:

            align_final = bool(
                self.get_parameter(
                    'align_final_heading'
                ).value
            )

            # --------------------------------------------------------
            # Không cần final orientation
            # --------------------------------------------------------

            if not align_final:

                self.stop_immediately()

                self.has_goal = False

                self.control_state = 'WAIT_GOAL'

                self.get_logger().info(
                    'GOAL REACHED'
                )

                self.get_logger().info(
                    f'Position error = '
                    f'{distance * 100.0:.1f} cm'
                )

                return

            # --------------------------------------------------------
            # Có yêu cầu final orientation
            # --------------------------------------------------------

            self.control_state = (
                'ALIGN_FINAL'
            )

            self.align_final_heading(
                robot_yaw
            )

            return

        # ============================================================
        # STATE TRANSITION
        # ============================================================

        # ------------------------------------------------------------
        # Đang chạy nhưng hướng lệch quá nhiều
        # -> quay lại ROTATE
        # ------------------------------------------------------------

        if (
            self.control_state ==
            'DRIVE_TO_GOAL'
            and
            abs(heading_error)
            >
            rotate_enter
        ):

            self.control_state = (
                'ROTATE_TO_GOAL'
            )

        # ------------------------------------------------------------
        # Đang ROTATE và đã gần đúng hướng
        # -> DRIVE
        # ------------------------------------------------------------

        if (
            self.control_state ==
            'ROTATE_TO_GOAL'
            and
            abs(heading_error)
            <=
            rotate_exit
        ):

            self.control_state = (
                'DRIVE_TO_GOAL'
            )

        # ============================================================
        # ROTATE TO GOAL
        # ============================================================

        if (
            self.control_state ==
            'ROTATE_TO_GOAL'
        ):

            self.rotate_to_goal(
                heading_error
            )

        # ============================================================
        # DRIVE TO GOAL
        # ============================================================

        else:

            self.drive_to_goal(
                distance,
                heading_error
            )

        # ============================================================
        # STATUS LOG
        # ============================================================

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        if (
            now_ns -
            self.last_status_log_ns
            >
            1_000_000_000
        ):

            self.last_status_log_ns = (
                now_ns
            )

            self.get_logger().info(
                f'state={self.control_state} | '
                f'robot=({robot_x:.2f},{robot_y:.2f}) | '
                f'goal=({self.goal_x:.2f},{self.goal_y:.2f}) | '
                f'd={distance:.2f}m | '
                f'e_heading={math.degrees(heading_error):.1f}deg'
            )

    # ================================================================
    # ROTATE FIRST
    # ================================================================

    def rotate_to_goal(
        self,
        heading_error
    ):

        kp = float(
            self.get_parameter(
                'kp_heading_rotate'
            ).value
        )

        omega_max = float(
            self.get_parameter(
                'omega_rotate_max'
            ).value
        )

        omega_min = float(
            self.get_parameter(
                'omega_min'
            ).value
        )

        # ------------------------------------------------------------
        # P controller
        # ------------------------------------------------------------

        w_target = (
            kp *
            heading_error
        )

        w_target = clamp(
            w_target,
            -omega_max,
            omega_max
        )

        # ------------------------------------------------------------
        # Minimum effective angular speed
        # ------------------------------------------------------------

        if abs(w_target) > 0.001:

            if abs(w_target) < omega_min:

                w_target = math.copysign(
                    omega_min,
                    w_target
                )

        # Không chạy tới trong giai đoạn quay
        self.publish_command(
            0.0,
            w_target
        )

    # ================================================================
    # DRIVE TO GOAL
    # ================================================================

    def drive_to_goal(
        self,
        distance,
        heading_error
    ):

        v_max = float(
            self.get_parameter(
                'v_max'
            ).value
        )

        v_min = float(
            self.get_parameter(
                'v_min'
            ).value
        )

        v_turn_max = float(
            self.get_parameter(
                'v_turn_max'
            ).value
        )

        omega_max = float(
            self.get_parameter(
                'omega_max'
            ).value
        )

        kp_distance = float(
            self.get_parameter(
                'kp_distance'
            ).value
        )

        kp_heading = float(
            self.get_parameter(
                'kp_heading_drive'
            ).value
        )

        slow_distance = float(
            self.get_parameter(
                'slow_distance_m'
            ).value
        )

        large_heading = math.radians(
            float(
                self.get_parameter(
                    'large_heading_error_deg'
                ).value
            )
        )

        # ============================================================
        # LINEAR VELOCITY
        # ============================================================

        v_target = (
            kp_distance *
            distance
        )

        v_target = clamp(
            v_target,
            v_min,
            v_max
        )

        # ------------------------------------------------------------
        # Smooth slowdown close to goal
        # ------------------------------------------------------------

        if distance < slow_distance:

            ratio = clamp(
                distance /
                slow_distance,
                0.0,
                1.0
            )

            v_target = (
                v_min +
                (
                    v_max -
                    v_min
                ) *
                ratio
            )

        # ============================================================
        # HEADING-DEPENDENT SPEED
        #
        # Sai góc càng lớn -> chạy càng chậm.
        #
        # Điều này giúp xe chạy thành đường thẳng tới goal,
        # thay vì vừa chạy nhanh vừa bẻ góc tạo vòng cung lớn.
        # ============================================================

        angle_abs = abs(
            heading_error
        )

        if angle_abs >= large_heading:

            v_target = min(
                v_target,
                v_turn_max
            )

        else:

            heading_factor = (
                1.0 -
                0.65 *
                (
                    angle_abs /
                    max(
                        large_heading,
                        0.001
                    )
                )
            )

            heading_factor = clamp(
                heading_factor,
                0.35,
                1.0
            )

            v_target *= (
                heading_factor
            )

        # ============================================================
        # ANGULAR VELOCITY
        # ============================================================

        w_target = (
            kp_heading *
            heading_error
        )

        w_target = clamp(
            w_target,
            -omega_max,
            omega_max
        )

        self.publish_command(
            v_target,
            w_target
        )

    # ================================================================
    # FINAL HEADING
    # ================================================================

    def align_final_heading(
        self,
        robot_yaw
    ):

        error = normalize_angle(
            self.goal_yaw -
            robot_yaw
        )

        tolerance = math.radians(
            float(
                self.get_parameter(
                    'final_heading_tolerance_deg'
                ).value
            )
        )

        if abs(error) <= tolerance:

            self.stop_immediately()

            self.has_goal = False
            self.control_state = 'WAIT_GOAL'

            self.get_logger().info(
                'GOAL POSITION + HEADING REACHED'
            )

            return

        kp = float(
            self.get_parameter(
                'kp_heading_rotate'
            ).value
        )

        omega_max = float(
            self.get_parameter(
                'omega_rotate_max'
            ).value
        )

        w = clamp(
            kp * error,
            -omega_max,
            omega_max
        )

        self.publish_command(
            0.0,
            w
        )

    # ================================================================
    # PUBLISH COMMAND
    # ================================================================

    def publish_command(
        self,
        v_target,
        w_target
    ):

        hz = float(
            self.get_parameter(
                'control_hz'
            ).value
        )

        dt = (
            1.0 /
            max(
                hz,
                1.0
            )
        )

        linear_accel = float(
            self.get_parameter(
                'max_linear_accel'
            ).value
        )

        linear_decel = float(
            self.get_parameter(
                'max_linear_decel'
            ).value
        )

        angular_accel = float(
            self.get_parameter(
                'max_angular_accel'
            ).value
        )

        # ============================================================
        # LINEAR RATE LIMIT
        # ============================================================

        v_cmd = self.rate_limit(
            v_target,
            self.last_v,
            linear_accel,
            linear_decel,
            dt
        )

        # ============================================================
        # ANGULAR RATE LIMIT
        # ============================================================

        max_dw = (
            angular_accel *
            dt
        )

        dw = clamp(
            w_target -
            self.last_w,
            -max_dw,
            max_dw
        )

        w_cmd = (
            self.last_w +
            dw
        )

        # ============================================================
        # CALIBRATION
        # ============================================================

        linear_scale = float(
            self.get_parameter(
                'linear_cmd_scale'
            ).value
        )

        angular_scale = float(
            self.get_parameter(
                'angular_cmd_scale'
            ).value
        )

        linear_scale = max(
            linear_scale,
            0.001
        )

        angular_scale = max(
            angular_scale,
            0.001
        )

        msg = Twist()

        msg.linear.x = (
            v_cmd /
            linear_scale
        )

        msg.angular.z = (
            w_cmd *
            angular_scale
        )

        self.cmd_pub.publish(
            msg
        )

        self.last_v = (
            v_cmd
        )

        self.last_w = (
            w_cmd
        )

    # ================================================================
    # STOP IMMEDIATELY
    # ================================================================

    def stop_immediately(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        # Publish vài lần để chắc chắn ESP32 nhận stop
        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)
        self.cmd_pub.publish(msg)

        self.last_v = 0.0
        self.last_w = 0.0


def main(args=None):

    rclpy.init(
        args=args
    )

    node = RVizGoalController()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_immediately()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()
