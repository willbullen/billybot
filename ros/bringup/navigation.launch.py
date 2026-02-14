#!/usr/bin/env python3
"""Navigation launch: camera + hardware + SLAM + nav2.

Brings up the full navigation stack:
  - robot_state_publisher (URDF)
  - DDSM + ST3215 hardware drivers
  - Camera driver
  - SLAM Toolbox (mapping mode)
  - Nav2 (controller, planner, behavior trees)

Usage:
  ros2 launch by_your_command navigation.launch.py
  ros2 launch by_your_command navigation.launch.py simulate:=true
  ros2 launch by_your_command navigation.launch.py slam_mode:=localization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("by_your_command")
    nav2_config = os.path.join(pkg_share, "config", "nav2_params.yaml")

    # Launch arguments
    sim_arg = DeclareLaunchArgument(
        "simulate", default_value="false",
        description="Run all drivers in simulation mode.",
    )
    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode", default_value="mapping",
        description="SLAM mode: mapping or localization.",
    )
    use_nav2_arg = DeclareLaunchArgument(
        "use_nav2", default_value="true",
        description="Launch the Nav2 navigation stack.",
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value="",
        description="Path to saved map for localization mode.",
    )

    simulate = LaunchConfiguration("simulate")
    slam_mode = LaunchConfiguration("slam_mode")
    use_nav2 = LaunchConfiguration("use_nav2")

    # Include hardware launch (URDF + motors + servos)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "bringup", "hardware.launch.py")
        ),
        launch_arguments={"simulate": simulate}.items(),
    )

    # Include camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "bringup", "camera.launch.py")
        ),
        launch_arguments={"simulate": simulate}.items(),
    )

    # Depth to laser scan (for SLAM toolbox which expects LaserScan)
    # Converts depth image to 2D laser scan
    depth_to_scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",
        parameters=[{
            "scan_time": 0.033,
            "range_min": 0.3,
            "range_max": 8.0,
            "scan_height": 1,
            "output_frame_id": "camera_depth_frame",
        }],
        remappings=[
            ("depth", "/camera/depth/image_rect_raw"),
            ("depth_camera_info", "/camera/camera_info"),
            ("scan", "/scan"),
        ],
    )

    # SLAM Toolbox
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[nav2_config, {"mode": slam_mode}],
    )

    # Nav2 lifecycle manager controls all nav2 nodes
    nav2_nodes = []

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_config],
        condition=IfCondition(use_nav2),
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_config],
        condition=IfCondition(use_nav2),
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_config],
        condition=IfCondition(use_nav2),
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "bt_navigator",
            ],
        }],
        condition=IfCondition(use_nav2),
    )

    log_msg = LogInfo(
        msg="Navigation launch: hardware + camera + SLAM + Nav2"
    )

    return LaunchDescription([
        sim_arg,
        slam_mode_arg,
        use_nav2_arg,
        map_file_arg,
        log_msg,
        hardware_launch,
        camera_launch,
        depth_to_scan,
        slam_node,
        controller_server,
        planner_server,
        bt_navigator,
        lifecycle_manager,
    ])
