#!/usr/bin/env python3
"""The Lite3 driver alone, in the robot namespace (lite3_bringup starts everything)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare namespace and params_file, start lite3_driver."""
    default_params = os.path.join(
        get_package_share_directory('lite3_driver'), 'config', 'driver_params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='lite3_driver',
            executable='driver_node',
            name='lite3_driver',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
