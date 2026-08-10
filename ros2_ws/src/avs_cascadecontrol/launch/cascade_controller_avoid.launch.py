from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("avs_cascadecontrol")
    config = os.path.join(pkg, "config", "cascade_controller_avoid.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("enable_cmd", default_value="false"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("allow_cmd_vel_conflict", default_value="false"),
        DeclareLaunchArgument("invert_angular", default_value="false"),

        LogInfo(msg=[
            "cascade_controller_avoid launch. enable_cmd=",
            LaunchConfiguration("enable_cmd"),
            ", cmd_vel_topic=",
            LaunchConfiguration("cmd_vel_topic"),
        ]),

        Node(
            package="avs_cascadecontrol",
            executable="cascade_controller_avoid",
            name="cascade_controller_avoid",
            output="screen",
            parameters=[
                config,
                {
                    "enable_cmd": ParameterValue(LaunchConfiguration("enable_cmd"), value_type=bool),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "allow_cmd_vel_conflict": ParameterValue(LaunchConfiguration("allow_cmd_vel_conflict"), value_type=bool),
                    "invert_angular": ParameterValue(LaunchConfiguration("invert_angular"), value_type=bool),
                },
            ],
        )
    ])
