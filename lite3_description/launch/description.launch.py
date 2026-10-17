#!/usr/bin/env python3
"""robot_state_publisher with the Lite3 URDF, in the robot namespace."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare namespace, camera and lidar; start robot_state_publisher."""
    xacro_file = os.path.join(
        get_package_share_directory('lite3_description'), 'urdf', 'lite3.urdf.xacro'
    )
    description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' camera:=', LaunchConfiguration('camera'),
            ' lidar:=', LaunchConfiguration('lidar'),
        ]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot'),
        DeclareLaunchArgument(
            'camera', default_value='rtsp', description='rtsp | realsense | none'
        ),
        DeclareLaunchArgument('lidar', default_value='mid360', description='mid360 | none'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'robot_description': description}],
            output='screen',
        ),
    ])
