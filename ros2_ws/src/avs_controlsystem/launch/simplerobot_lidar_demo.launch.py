from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_simplerobot = get_package_share_directory('avs_controlsystem')
    rviz_config = os.path.join(pkg_simplerobot, 'rviz', 'simplerobot_lidar.rviz')

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

    lidar_avoidance = Node(
        package='avs_controlsystem',
        executable='lidar_avoidance_node',
        name='lidar_avoidance_node',
        output='screen',
        parameters=[
            {
                'linear_max': 0.12,
                'linear_min': 0.04,
                'angular_max': 0.75,
                'emergency_distance': 0.18,
                'stop_distance': 0.32,
                'slow_distance': 0.70,
                'front_angle_deg': 35.0,
                'side_min_angle_deg': 35.0,
                'side_max_angle_deg': 110.0,
                'k_avoid': 0.45,
                'k_center': 0.25,
                'invert_angular': False,
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
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
                'save_root': '/root/yahboomcar_ws/demo_reports',
                'track_width': 0.18,
                'wheel_radius': 0.0325,
                'sample_rate': 10.0,
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
        path_visualizer,
        run_report,
        lidar_avoidance,
        rviz,
    ])
