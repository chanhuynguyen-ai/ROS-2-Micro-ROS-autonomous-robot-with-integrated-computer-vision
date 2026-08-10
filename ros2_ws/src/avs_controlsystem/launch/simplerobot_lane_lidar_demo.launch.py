from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_simplerobot = get_package_share_directory('avs_controlsystem')
    rviz_config = os.path.join(pkg_simplerobot, 'rviz', 'simplerobot_lane_lidar.rviz')

    robot_description = ''

    try:
        pkg_description = get_package_share_directory('yahboomcar_description')

        urdf_path = os.path.join(pkg_description, 'urdf', 'MicroROS.urdf')

        if not os.path.exists(urdf_path):
            urdf_path = os.path.join(pkg_description, 'urdf', 'yahboomcar_robot2.urdf')

        if os.path.exists(urdf_path):
            with open(urdf_path, 'r') as f:
                robot_description = f.read()
        else:
            robot_description = '<robot name="simplerobot"></robot>'

    except Exception:
        robot_description = '<robot name="simplerobot"></robot>'

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    line_camera = Node(
        package='avs_controlsystem',
        executable='line_camera_target_node',
        name='line_camera_target_node',
        output='screen',
        parameters=[
            {
                'camera_id': 0,
                'frame_width': 640,
                'frame_height': 480,
                'fps': 20.0,
                'lane_target_topic': '/lane_target',
                'lane_marker_topic': '/simplerobot_lane_target_marker',
                'show_preview': True,
                'roi_top_ratio': 0.45,
                'min_mask_area': 500,
                'near_y_ratio': 0.85,
                'far_y_ratio': 0.58,
                'band_height': 35,
                'base_frame': 'base_link',
            }
        ]
    )

    lane_lidar_follower = Node(
        package='avs_controlsystem',
        executable='lane_lidar_follower_node',
        name='lane_lidar_follower_node',
        output='screen',
        parameters=[
            {
                'lane_target_topic': '/lane_target',
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'obstacle_marker_topic': '/simplerobot_obstacle_points',
                'base_frame': 'base_link',

                'v_max': 0.11,
                'v_min': 0.035,
                'omega_max': 0.70,

                'kp_y': 1.15,
                'kp_heading': 1.55,
                'kd_heading': 0.06,

                'lane_lost_timeout': 0.7,
                'search_omega': 0.22,

                'emergency_distance': 0.18,
                'stop_distance': 0.32,
                'slow_distance': 0.70,
                'side_alert_distance': 0.45,

                'front_angle_deg': 35.0,
                'side_min_angle_deg': 35.0,
                'side_max_angle_deg': 110.0,

                'avoid_gain': 0.24,
                'invert_angular': False,

                'marker_max_range': 1.5,
                'marker_decimation': 4,
            }
        ]
    )

    path_visualizer = Node(
        package='avs_controlsystem',
        executable='path_visualizer_node',
        name='path_visualizer_node',
        output='screen',
        parameters=[
            {
                'odom_topic': '/odom_raw',
                'path_topic': '/simplerobot_path',
                'heading_marker_topic': '/simplerobot_heading',
                'fixed_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'max_path_length': 5000,
                'min_distance_step': 0.02,
                'front_arrow_offset': 0.20,
                'arrow_length': 0.45,
                'arrow_height': 0.18,
            }
        ]
    )

    run_report = Node(
        package='avs_controlsystem',
        executable='run_report_node',
        name='run_report_node',
        output='screen',
        parameters=[
            {
                'odom_topic': '/odom_raw',
                'cmd_vel_topic': '/cmd_vel',
                'scan_topic': '/scan',
                'save_root': '/root/yahboomcar_ws/demo_reports',
                'track_width': 0.18,
                'wheel_radius': 0.0325,
                'sample_rate': 10.0,
                'obstacle_max_range': 1.5,
                'obstacle_decimation': 8,
                'max_obstacle_points': 20000,
            }
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher,
        line_camera,
        path_visualizer,
        run_report,
        lane_lidar_follower,
        rviz,
    ])
