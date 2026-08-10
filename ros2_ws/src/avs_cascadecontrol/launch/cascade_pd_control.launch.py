from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("avs_cascadecontrol")
    config_file = os.path.join(pkg_share, "config", "cascade_pd.yaml")

    enable_cmd_arg = DeclareLaunchArgument(
        "enable_cmd",
        default_value="false",
        description="true: wheel_inner_pd_node publishes /cmd_vel",
    )

    test_mode_arg = DeclareLaunchArgument(
        "test_mode",
        default_value="false",
        description="true: generate fake lane error",
    )

    invert_angular_arg = DeclareLaunchArgument(
        "invert_angular",
        default_value="false",
        description="true: invert omega_ref sign",
    )

    allow_pivot_arg = DeclareLaunchArgument(
        "allow_pivot_turn",
        default_value="false",
        description="true: allow pivot turn; false: both wheel groups keep moving",
    )

    enable_cmd = LaunchConfiguration("enable_cmd")
    test_mode = LaunchConfiguration("test_mode")
    invert_angular = LaunchConfiguration("invert_angular")
    allow_pivot_turn = LaunchConfiguration("allow_pivot_turn")

    lane_outer = Node(
        package="avs_cascadecontrol",
        executable="lane_outer_pd_node",
        name="lane_outer_pd_node",
        output="screen",
        parameters=[
            config_file,
            {
                "test_mode": ParameterValue(test_mode, value_type=bool),
                "invert_angular": ParameterValue(invert_angular, value_type=bool),
            },
        ],
    )

    wheel_inner = Node(
        package="avs_cascadecontrol",
        executable="wheel_inner_pd_node",
        name="wheel_inner_pd_node",
        output="screen",
        parameters=[
            config_file,
            {
                "enable_cmd": ParameterValue(enable_cmd, value_type=bool),
                "allow_pivot_turn": ParameterValue(allow_pivot_turn, value_type=bool),
            },
        ],
    )

    monitor = Node(
        package="avs_cascadecontrol",
        executable="cascade_control_monitor_node",
        name="cascade_control_monitor_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        enable_cmd_arg,
        test_mode_arg,
        invert_angular_arg,
        allow_pivot_arg,
        LogInfo(msg=["Cascade lane control launch. enable_cmd=", enable_cmd, ", test_mode=", test_mode]),
        lane_outer,
        wheel_inner,
        monitor,
    ])
