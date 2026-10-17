"""Pure helpers of the Lite3 bringup: argument checks and the Livox configuration."""

import copy
import ipaddress
import json
import os
import re

CAMERAS = ('rtsp', 'realsense', 'none')
NAMESPACE_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$')
RTSP_PORT = 8554
RTSP_PATH = 'test'


class BringupError(ValueError):
    """A launch argument is invalid."""


def truthy(value) -> bool:
    """Interpret a launch argument as a bool."""
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def resolve_namespace(value: str) -> str:
    """Return the robot namespace without slashes around it."""
    ns = str(value).strip().strip('/')
    if not NAMESPACE_RE.match(ns):
        raise BringupError(f'namespace:={value!r} is not a valid ROS namespace')
    return ns


def resolve_camera(value: str) -> str:
    """Return rtsp, realsense or none."""
    camera = str(value).strip().lower()
    if camera not in CAMERAS:
        raise BringupError(f'camera:={value!r}, use one of {", ".join(CAMERAS)}')
    return camera


def resolve_ip(value: str, name: str) -> str:
    """Return an IPv4 address, or '' when unset."""
    text = str(value).strip()
    if not text:
        return ''
    try:
        return str(ipaddress.IPv4Address(text))
    except ValueError as error:
        raise BringupError(f'{name}:={value!r} is not an IPv4 address') from error


def rtsp_url(host: str) -> str:
    """Return the front camera's stream on a motion host (document 1.2.11)."""
    return f'rtsp://{host}:{RTSP_PORT}/{RTSP_PATH}'


def livox_config(template: dict, lidar_ip: str = '', host_ip: str = '') -> dict:
    """Return the Livox driver configuration with the LiDAR and host addresses filled in."""
    config = copy.deepcopy(template)
    if host_ip:
        host = config['MID360']['host_net_info']
        for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
            host[key] = host_ip
    if lidar_ip:
        config['lidar_configs'][0]['ip'] = lidar_ip
    return config


def write_livox_config(template_path: str, lidar_ip: str, host_ip: str, out_path: str) -> str:
    """Write the Livox configuration for these addresses; return the path to use."""
    if not lidar_ip and not host_ip:
        return template_path
    with open(template_path) as f:
        template = json.load(f)
    out_path = os.path.expanduser(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump(livox_config(template, lidar_ip, host_ip), f, indent=2)
    return out_path
