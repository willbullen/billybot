#!/usr/bin/env python3
"""Camera launch: camera_driver_node with optional depth.

Usage:
  ros2 launch by_your_command camera.launch.py
  ros2 launch by_your_command camera.launch.py simulate:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("by_your_command")
    camera_config = os.path.join(pkg_share, "config", "camera.yaml")

    sim_arg = DeclareLaunchArgument(
        "simulate",
        default_value="false",
        description="Run camera in simulation mode (no hardware).",
    )

    simulate = LaunchConfiguration("simulate", default="false")

    camera_node = Node(
        package="by_your_command",
        executable="camera_driver_node",
        name="camera_driver_node",
        output="screen",
        parameters=[camera_config, {"simulate": simulate}],
    )

    log_msg = LogInfo(msg="Camera launch: camera_driver_node (check simulate param).")

    return LaunchDescription([
        sim_arg,
        log_msg,
        camera_node,
    ])
