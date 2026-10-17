"""The Lite3 URDF: legs, sensor frames and the camera / LiDAR variants."""

import math
from pathlib import Path
from xml.etree import ElementTree

import pytest

xacro = pytest.importorskip('xacro')

XACRO = Path(__file__).resolve().parents[1] / 'urdf' / 'lite3.urdf.xacro'
JOINTS = [
    f'{leg}_{joint}' for leg in ('FL', 'FR', 'HL', 'HR') for joint in ('HipX', 'HipY', 'Knee')
]
# FAST-LIO's extrinsic_T in Molly (molly_odometry/config/fast_lio.yaml): the LiDAR in the IMU
FAST_LIO_EXTRINSIC_T = (-0.011, -0.02329, 0.04412)


def urdf(**args) -> ElementTree.Element:
    """Expand the xacro with these arguments."""
    return ElementTree.fromstring(xacro.process_file(str(XACRO), mappings=args).toxml())


def joint(root, name):
    """Return a joint element by name."""
    found = root.find(f"joint[@name='{name}']")
    assert found is not None, f'no joint {name}'
    return found


def origin(element) -> tuple:
    """Return the xyz of an origin element."""
    return tuple(float(v) for v in element.find('origin').get('xyz').split())


def links(root) -> set:
    """Return every link name."""
    return {link.get('name') for link in root.findall('link')}


def parent_of(root, child: str) -> str:
    """Return the parent link of a link."""
    for j in root.findall('joint'):
        if j.find('child').get('link') == child:
            return j.find('parent').get('link')
    return ''


def test_legs_match_the_document():
    """12 revolute joints with the driver's names, positions and ranges of the document."""
    root = urdf()
    revolute = [j.get('name') for j in root.findall("joint[@type='revolute']")]
    assert sorted(revolute) == sorted(JOINTS)
    assert origin(joint(root, 'FL_HipX')) == (0.1745, 0.062, 0.0)
    assert origin(joint(root, 'HR_HipX')) == (-0.1745, -0.062, 0.0)
    assert origin(joint(root, 'FR_HipY')) == (0.0, -0.0985, 0.0)
    assert origin(joint(root, 'HL_Knee')) == (0.0, 0.0, -0.2)
    assert origin(joint(root, 'FL_foot_joint')) == (0.0, 0.0, -0.21)
    limit = joint(root, 'FL_Knee').find('limit')
    assert float(limit.get('lower')) == pytest.approx(math.radians(34.5))
    assert float(limit.get('upper')) == pytest.approx(math.radians(156.0))
    limit = joint(root, 'HR_HipY').find('limit')
    assert float(limit.get('lower')) == pytest.approx(math.radians(-200.0))


def test_sensor_frames_and_fast_lio_extrinsic():
    """IMU and Mid-360 where the document puts them; the LiDAR IMU agrees with FAST-LIO."""
    root = urdf()
    assert origin(joint(root, 'imu_joint')) == (0.005, 0.0, 0.0)
    assert origin(joint(root, 'livox_joint')) == (0.18678, 0.0, 0.12498)
    imu = origin(joint(root, 'livox_imu_joint'))
    assert imu == pytest.approx(tuple(-v for v in FAST_LIO_EXTRINSIC_T))
    assert parent_of(root, 'livox_imu_frame') == 'livox_frame'


@pytest.mark.parametrize('camera,expected', [
    ('rtsp', {'camera_link', 'camera_color_optical_frame'}),
    ('realsense', {'camera_link'}),
    ('none', set()),
])
def test_camera_variants(camera, expected):
    """The RTSP camera brings its optical frame; the RealSense driver publishes its own."""
    root = urdf(camera=camera)
    assert links(root) & {'camera_link', 'camera_color_optical_frame'} == expected
    if camera == 'realsense':
        assert origin(joint(root, 'camera_joint')) == (0.25228, -0.0005, 0.0741)
    if camera == 'rtsp':
        rpy = joint(root, 'camera_optical_joint').find('origin').get('rpy').split()
        assert [float(v) for v in rpy] == pytest.approx([-math.pi / 2, 0.0, -math.pi / 2])


def test_lidar_can_be_left_out():
    """lidar:=none drops the Livox frames."""
    assert not links(urdf(lidar='none')) & {'livox_frame', 'livox_imu_frame'}


def test_tree_rooted_at_base_link_without_foreign_frames():
    """One tree from base_link; no frame owned by FAST-LIO or Molly."""
    root = urdf()
    names = links(root)
    children = [j.find('child').get('link') for j in root.findall('joint')]
    assert len(children) == len(set(children))
    assert names - set(children) == {'base_link'}
    assert not names & {'body', 'odom', 'map', 'camera_init'}
