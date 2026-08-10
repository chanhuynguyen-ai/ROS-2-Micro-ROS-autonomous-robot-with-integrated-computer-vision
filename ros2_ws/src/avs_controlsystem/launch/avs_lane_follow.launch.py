from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_cmd = LaunchConfiguration('enable_cmd')
    controller_mode = LaunchConfiguration('controller_mode')
    telemetry_topic = LaunchConfiguration('telemetry_topic')
    v_max = LaunchConfiguration('v_max')

    return LaunchDescription([
        DeclareLaunchArgument('enable_cmd', default_value='false'),
        DeclareLaunchArgument('controller_mode', default_value='pid'),
        DeclareLaunchArgument('telemetry_topic', default_value='/avs/telemetry_realworld'),
        DeclareLaunchArgument('v_max', default_value='0.25'),

        Node(
            package='avs_controlsystem',
            executable='lane_target_from_telemetry_node',
            name='lane_target_from_telemetry_node',
            output='screen',
            parameters=[{
                'telemetry_topic': telemetry_topic,
                'fallback_telemetry_topic': '/avs/telemetry',
                'lane_target_topic': '/avs/lane_target',
                'lookahead_m': 0.65,
                'stop_line_distance_m': 0.35,
            }]
        ),

        Node(
            package='avs_controlsystem',
            executable='lane_follow_controller_node',
            name='lane_follow_controller_node',
            output='screen',
            parameters=[{
                'enable_cmd': enable_cmd,
                'controller_mode': controller_mode,
                'lane_target_topic': '/avs/lane_target',
                'cmd_vel_topic': '/cmd_vel',
                'v_max': v_max,
                'v_min': 0.06,
                'w_max': 1.20,
                'kp': 1.2,
                'ki': 0.0,
                'kd': 0.12,
                'k_heading': 0.7,
                'k_y': 1.5,
                'k_theta': 1.0,
                'k_dy': 0.15,
                'use_lidar_safety': True,
                'obstacle_stop_distance': 0.28,
                'obstacle_slow_distance': 0.60,
            }]
        ),
    ])
