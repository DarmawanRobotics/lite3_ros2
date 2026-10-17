#!/usr/bin/env python3
"""
The Lite3 robot package in one launch: driver, URDF, Livox Mid-360 and the camera.

    ros2 launch lite3_bringup bringup.launch.py                     # namespace robot, RTSP camera
    ros2 launch lite3_bringup bringup.launch.py camera:=realsense lidar:=false
    ros2 launch lite3_bringup bringup.launch.py lidar_ip:=192.168.1.150 host_ip:=192.168.1.110

Molly starts it with namespace:=<ROBOT_NS> (ROBOT_BRINGUP=lite3_bringup). It starts no Molly
node. The driver, the URDF and the camera live in the namespace; the Livox driver runs in the
root namespace so the cloud stays on /livox/lidar and /livox/imu.
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from lite3_bringup.bringup import (
    resolve_camera,
    resolve_ip,
    resolve_namespace,
    rtsp_url,
    truthy,
    write_livox_config,
)

CAMERA_TOPICS = '/camera/camera/color'
LIVOX_CONFIG_OUT = '~/.lite3/MID360_config.json'


def share(package: str, *path: str) -> str:
    """Return a file in the installed share directory of a package."""
    return os.path.join(get_package_share_directory(package), *path)


def installed(package: str) -> bool:
    """Return True when a package is built or installed."""
    try:
        get_package_share_directory(package)
    except PackageNotFoundError:
        return False
    return True


def launch_setup(context):
    """Validate the arguments and build the actions."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    ns = resolve_namespace(arg('namespace'))
    camera = resolve_camera(arg('camera'))
    lidar = truthy(arg('lidar'))
    motion_host = resolve_ip(arg('motion_host'), 'motion_host')
    lidar_ip = resolve_ip(arg('lidar_ip'), 'lidar_ip')
    host_ip = resolve_ip(arg('host_ip'), 'host_ip')

    driver_overrides = {'motion_host_ip': motion_host} if motion_host else {}
    actions = [
        LogInfo(msg=f'Lite3 namespace={ns} camera={camera} lidar={lidar} '
                    f'motion_host={motion_host or "(driver params)"}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                share('lite3_description', 'launch', 'description.launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'camera': camera,
                'lidar': 'mid360' if lidar else 'none',
            }.items(),
        ),
        Node(
            package='lite3_driver',
            executable='driver_node',
            name='lite3_driver',
            namespace=ns,
            parameters=[arg('driver_params'), driver_overrides],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ]

    if lidar and installed('livox_ros_driver2'):
        config = write_livox_config(
            share('lite3_bringup', 'config', 'MID360_config.json'), lidar_ip, host_ip,
            LIVOX_CONFIG_OUT,
        )
        actions.append(Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            parameters=[{
                'xfer_format': 1,             # Livox CustomMsg: FAST-LIO and the sectors read it
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'lvx_file_path': '',
                'user_config_path': config,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ))
    elif lidar:
        actions.append(LogInfo(msg='WARNING livox_ros_driver2 is not built: no LiDAR'))

    if camera == 'rtsp':
        camera_overrides = {'url': rtsp_url(motion_host)} if motion_host else {}
        actions.append(Node(
            package='lite3_camera',
            executable='rtsp_camera_node',
            name='front_camera',
            namespace=ns,
            parameters=[arg('camera_params'), camera_overrides],
            remappings=[
                ('image_raw', f'{CAMERA_TOPICS}/image_raw'),
                ('image_raw/compressed', f'{CAMERA_TOPICS}/image_raw/compressed'),
                ('camera_info', f'{CAMERA_TOPICS}/camera_info'),
            ],
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ))
    elif camera == 'realsense' and installed('realsense2_camera'):
        actions.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'camera_name': 'camera',
                'base_frame_id': 'camera_link',
                'publish_tf': True,
                'enable_color': True,
                'enable_depth': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.color_profile': '640,480,15',
            }],
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ))
    elif camera == 'realsense':
        actions.append(LogInfo(msg='WARNING realsense2_camera is not installed: no camera'))

    if truthy(arg('rviz')):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', share('lite3_description', 'rviz', 'lite3.rviz')],
            output='log',
        ))
    return actions


def generate_launch_description():
    """Declare the arguments; most default to the environment."""
    env = lambda name, default='': EnvironmentVariable(name, default_value=default)  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=env('ROBOT_NS', 'robot')),
        DeclareLaunchArgument(
            'camera', default_value=env('LITE3_CAMERA', 'rtsp'),
            description='rtsp (the front camera of the robot) | realsense | none',
        ),
        DeclareLaunchArgument(
            'lidar', default_value=env('LITE3_LIDAR', 'true'), description='Livox Mid-360'
        ),
        DeclareLaunchArgument(
            'motion_host', default_value=env('LITE3_MOTION_HOST'),
            description='motion host IP for the driver and the camera; empty: the params files',
        ),
        DeclareLaunchArgument(
            'lidar_ip', default_value=env('LIVOX_LIDAR_IP'),
            description='Mid-360 IP; empty: config/MID360_config.json',
        ),
        DeclareLaunchArgument(
            'host_ip', default_value=env('LIVOX_HOST_IP'),
            description='IP of this computer for the Mid-360; empty: config/MID360_config.json',
        ),
        DeclareLaunchArgument(
            'driver_params',
            default_value=env(
                'LITE3_DRIVER_PARAMS', share('lite3_driver', 'config', 'driver_params.yaml')
            ),
        ),
        DeclareLaunchArgument(
            'camera_params',
            default_value=env(
                'LITE3_CAMERA_PARAMS', share('lite3_camera', 'config', 'camera_params.yaml')
            ),
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
