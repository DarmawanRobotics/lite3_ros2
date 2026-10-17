"""Bringup helpers and the launch description of bringup.launch.py."""

import json
import os

from lite3_bringup.bringup import (
    BringupError,
    livox_config,
    resolve_camera,
    resolve_ip,
    resolve_namespace,
    rtsp_url,
    truthy,
    write_livox_config,
)
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
TEMPLATE = os.path.join(PACKAGE, 'config', 'MID360_config.json')


def test_arguments():
    """Namespaces, cameras and addresses are checked before anything starts."""
    assert resolve_namespace('/robot/') == 'robot'
    assert resolve_camera(' RTSP ') == 'rtsp'
    assert resolve_ip('192.168.1.120', 'motion_host') == '192.168.1.120'
    assert resolve_ip('  ', 'x') == ''
    assert truthy('True') and not truthy('0')
    assert rtsp_url('192.168.2.1') == 'rtsp://192.168.2.1:8554/test'
    for call in (
        lambda: resolve_namespace('dog-2'),
        lambda: resolve_camera('usb'),
        lambda: resolve_ip('192.168.1', 'host_ip'),
    ):
        with pytest.raises(BringupError):
            call()


def test_livox_config_fills_both_addresses(tmp_path):
    """The Mid-360 configuration gets the LiDAR and host IPs; empty keeps the template."""
    with open(TEMPLATE) as f:
        template = json.load(f)
    config = livox_config(template, '192.168.1.151', '192.168.1.111')
    assert config['lidar_configs'][0]['ip'] == '192.168.1.151'
    host = config['MID360']['host_net_info']
    assert {host[k] for k in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip')} == {
        '192.168.1.111'
    }
    assert template['lidar_configs'][0]['ip'] == '192.168.1.150'  # not modified
    assert write_livox_config(TEMPLATE, '', '', str(tmp_path / 'x.json')) == TEMPLATE
    out = write_livox_config(TEMPLATE, '192.168.1.152', '', str(tmp_path / 'livox.json'))
    with open(out) as f:
        assert json.load(f)['lidar_configs'][0]['ip'] == '192.168.1.152'


def load_launch_module():
    """Import launch/bringup.launch.py as a module."""
    import importlib.util

    path = os.path.join(PACKAGE, 'launch', 'bringup.launch.py')
    spec = importlib.util.spec_from_file_location('lite3_bringup_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_context(module, **overrides):
    """Return a LaunchContext with the declared defaults plus overrides."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value
            )
    context.launch_configurations.update(overrides)
    return context


def started(actions) -> dict:
    """Return {executable: node} of the Node actions."""
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    def text(value):
        return value if isinstance(value, str) else perform_substitutions(None, value)

    return {text(a.node_executable): a for a in actions if isinstance(a, Node)}


@pytest.mark.parametrize('camera,expected', [
    ('rtsp', {'driver_node', 'rtsp_camera_node'}),
    ('none', {'driver_node'}),
])
def test_launch_starts_the_robot_parts(camera, expected):
    """The driver always, the camera node with camera:=rtsp, never a Molly node."""
    pytest.importorskip('launch_ros')
    pytest.importorskip('lite3_driver')
    module = load_launch_module()
    context = launch_context(module, camera=camera, lidar='false', namespace='dog')
    nodes = started(module.launch_setup(context))
    assert set(nodes) == expected
    assert not any(name.startswith('molly') for name in nodes)


def test_launch_rejects_bad_arguments():
    """A wrong camera, namespace or address stops the launch with a clear error."""
    pytest.importorskip('launch_ros')
    pytest.importorskip('lite3_driver')
    module = load_launch_module()
    for overrides in ({'camera': 'webcam'}, {'namespace': '1dog'}, {'motion_host': 'robot'}):
        with pytest.raises(BringupError):
            module.launch_setup(launch_context(module, **overrides))
