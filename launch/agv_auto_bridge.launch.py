from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("agv2_pkg"),
            "config",
            "agv_auto_bridge.yaml",
        ]),
        description="Path to agv_auto_bridge parameter yaml",
    )

    bridge = Node(
        package="agv2_pkg",
        executable="agv_auto_bridge.py",
        name="agv_auto_bridge",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, bridge])
