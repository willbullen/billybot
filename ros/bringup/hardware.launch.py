#!/usr/bin/env python3
"""
Hardware launch: robot_state_publisher (URDF) + ddsm_driver_node + st3215_driver_node.

Usage:
  ros2 launch by_your_command hardware.launch.py
  ros2 launch by_your_command hardware.launch.py simulate:=true

For Jetson with real hardware, ensure serial devices are available and simulate:=false.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_urdf_path():
    pkg_share = get_package_share_directory("by_your_command")
    return os.path.join(pkg_share, "urdf", "billybot.urdf.xml")


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        "simulate",
        default_value="false",
        description="If true, run DDSM and ST3215 drivers in simulation (no serial).",
    )

    pkg_share = get_package_share_directory("by_your_command")
    urdf_path = os.path.join(pkg_share, "urdf", "billybot.urdf.xml")
    hardware_config = os.path.join(pkg_share, "config", "hardware.yaml")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    simulate = LaunchConfiguration("simulate", default="false")
    ddsm_node = Node(
        package="by_your_command",
        executable="ddsm_driver_node",
        name="ddsm_driver_node",
        output="screen",
        parameters=[hardware_config, {"simulate": simulate}],
    )
    st3215_node = Node(
        package="by_your_command",
        executable="st3215_driver_node",
        name="st3215_driver_node",
        output="screen",
        parameters=[hardware_config, {"simulate": simulate}],
    )

    log_sim = LogInfo(msg="Hardware launch: robot_state_publisher + ddsm_driver + st3215_driver (check simulate param).")

    return LaunchDescription([
        sim_arg,
        log_sim,
        robot_state_publisher,
        ddsm_node,
        st3215_node,
    ])
