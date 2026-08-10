from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("avs_pdbackstepingcontrol")
    config_file = os.path.join(pkg_share, "config", "pd_backsteping.yaml")

    enable_cmd_arg = DeclareLaunchArgument(
        "enable_cmd",
        default_value="false",
        description="true: publish moving cmd_vel; false: log only",
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="cmd_vel output topic. Use /cmd_vel for direct test or /avs/cmd_vel/pd_backsteping for mux.",
    )

    invert_angular_arg = DeclareLaunchArgument(
        "invert_angular",
        default_value="false",
        description="true: invert angular command sign",
    )

    allow_conflict_arg = DeclareLaunchArgument(
        "allow_cmd_vel_conflict",
        default_value="false",
        description="false: do not move if multiple /cmd_vel publishers are detected",
    )

    enable_cmd = LaunchConfiguration("enable_cmd")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    invert_angular = LaunchConfiguration("invert_angular")
    allow_cmd_vel_conflict = LaunchConfiguration("allow_cmd_vel_conflict")

    node = Node(
        package="avs_pdbackstepingcontrol",
        executable="pd_backsteping_cmdvel_node",
        name="pd_backsteping_cmdvel_node",
        output="screen",
        parameters=[
            config_file,
            {
                "enable_cmd": ParameterValue(enable_cmd, value_type=bool),
                "cmd_vel_topic": cmd_vel_topic,
                "invert_angular": ParameterValue(invert_angular, value_type=bool),
                "allow_cmd_vel_conflict": ParameterValue(allow_cmd_vel_conflict, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        enable_cmd_arg,
        cmd_vel_topic_arg,
        invert_angular_arg,
        allow_conflict_arg,
        LogInfo(msg=[
            "PD-Backsteping launch. enable_cmd=",
            enable_cmd,
            ", cmd_vel_topic=",
            cmd_vel_topic,
        ]),
        node,
    ])
