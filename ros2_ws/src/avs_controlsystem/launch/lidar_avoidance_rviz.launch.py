from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import os


def generate_launch_description():
    rviz_config = '/root/AVScontrol/ros2_ws/src/avs_controlsystem/rviz/lidar_avoidance_view.rviz'

    linear_max = LaunchConfiguration('linear_max')
    linear_min = LaunchConfiguration('linear_min')
    angular_max = LaunchConfiguration('angular_max')

    slow_distance = LaunchConfiguration('slow_distance')
    stop_distance = LaunchConfiguration('stop_distance')
    emergency_distance = LaunchConfiguration('emergency_distance')

    actions = [
        DeclareLaunchArgument('linear_max', default_value='0.5'),
        DeclareLaunchArgument('linear_min', default_value='0.08'),
        DeclareLaunchArgument('angular_max', default_value='1.0'),

        DeclareLaunchArgument('slow_distance', default_value='1.20'),
        DeclareLaunchArgument('stop_distance', default_value='0.55'),
        DeclareLaunchArgument('emergency_distance', default_value='0.28'),

        LogInfo(msg='Starting LiDAR avoidance with RViz...'),

        Node(
            package='avs_controlsystem',
            executable='lidar_avoidance_node',
            name='lidar_avoidance_node',
            output='screen',
            parameters=[{
                'linear_max': linear_max,
                'linear_min': linear_min,
                'angular_max': angular_max,
                'slow_distance': slow_distance,
                'stop_distance': stop_distance,
                'emergency_distance': emergency_distance,
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_lidar_avoidance',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ]

    # Optional robot model if yahboomcar_description exists.
    try:
        pkg_desc = get_package_share_directory('yahboomcar_description')
        urdf_candidates = [
            os.path.join(pkg_desc, 'urdf', 'MicroROS.urdf'),
            os.path.join(pkg_desc, 'urdf', 'yahboomcar_robot2.urdf'),
        ]

        urdf_path = None
        for p in urdf_candidates:
            if os.path.exists(p):
                urdf_path = p
                break

        if urdf_path is not None:
            with open(urdf_path, 'r') as f:
                robot_description = f.read()

            actions.insert(
                1,
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': robot_description}]
                )
            )
    except PackageNotFoundError:
        actions.insert(
            1,
            LogInfo(msg='yahboomcar_description not found. RViz will show scan/odom, but RobotModel may not appear.')
        )

    return LaunchDescription(actions)
