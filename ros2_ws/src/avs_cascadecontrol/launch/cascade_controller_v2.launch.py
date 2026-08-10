#!/usr/bin/env python3

import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument
)

from launch.substitutions import (
    LaunchConfiguration
)

from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory(
        'avs_cascadecontrol'
    )

    default_config = os.path.join(
        package_share,
        'config',
        'cascade_controller_v2.yaml'
    )

    params_file = LaunchConfiguration(
        'params_file'
    )

    enable_cmd = LaunchConfiguration(
        'enable_cmd'
    )

    cmd_vel_topic = LaunchConfiguration(
        'cmd_vel_topic'
    )

    invert_angular = LaunchConfiguration(
        'invert_angular'
    )

    allow_cmd_vel_conflict = LaunchConfiguration(
        'allow_cmd_vel_conflict'
    )

    enable_lidar_safety = LaunchConfiguration(
        'enable_lidar_safety'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=default_config
        ),

        DeclareLaunchArgument(
            'enable_cmd',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel'
        ),

        DeclareLaunchArgument(
            'invert_angular',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'allow_cmd_vel_conflict',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'enable_lidar_safety',
            default_value='false'
        ),

        Node(
            package='avs_cascadecontrol',

            executable='cascade_controller_v2',

            name='cascade_controller_v2',

            output='screen',

            parameters=[
                params_file,

                {
                    'enable_cmd':
                        enable_cmd,

                    'cmd_vel_topic':
                        cmd_vel_topic,

                    'invert_angular':
                        invert_angular,

                    'allow_cmd_vel_conflict':
                        allow_cmd_vel_conflict,

                    'enable_lidar_safety':
                        enable_lidar_safety,
                }
            ]
        )
    ])
