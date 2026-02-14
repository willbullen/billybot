"""Launch behavior manager with navigation stack."""

import os

from ament_cmake.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('by_your_command')

    simulate_arg = DeclareLaunchArgument(
        'simulate', default_value='false',
        description='Enable hardware simulation mode'
    )

    # Include full navigation stack
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'bringup', 'navigation.launch.py')
        ),
        launch_arguments={'simulate': LaunchConfiguration('simulate')}.items()
    )

    # Behavior manager node
    behavior_manager = Node(
        package='by_your_command',
        executable='behavior_manager_node',
        name='behavior_manager',
        parameters=[os.path.join(pkg_dir, 'config', 'behaviors.yaml')],
        output='screen',
    )

    return LaunchDescription([
        simulate_arg,
        nav_launch,
        behavior_manager,
    ])
