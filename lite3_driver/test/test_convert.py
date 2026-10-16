"""Conversions from the motion host's reports to ROS conventions."""

import math

from lite3_driver.convert import (
    attitude,
    finite_or,
    odometry_orientation,
    quaternion_from_rpy,
    ultrasound_range,
)
from lite3_driver.protocol import StateReport
import pytest


def test_quaternion_of_a_yaw():
    """A pure yaw is a rotation about z."""
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5)) and w == pytest.approx(math.sqrt(0.5))


def test_quaternion_is_unit_length():
    """Any attitude gives a unit quaternion."""
    q = quaternion_from_rpy(0.3, -0.2, 2.5)
    assert sum(v * v for v in q) == pytest.approx(1.0)


def test_attitude_from_degrees_and_odometry_yaw():
    """The report carries degrees; odometry takes its yaw from pos_world."""
    report = StateReport(
        0, 0, 0, (90.0, 0.0, 180.0), (0, 0, 0), (0, 0, 0), (0.0, 0.0, 0.0), (0, 0, 0),
        (0, 0, 0), 'natural',
    )
    assert attitude(report) == pytest.approx((math.pi / 2, 0.0, math.pi))
    x, y, z, w = odometry_orientation(report)
    assert x == pytest.approx(math.sqrt(0.5)) and z == pytest.approx(0.0)


def test_ultrasound_range_and_finite():
    """Readings are clamped to the sensor range; missing ones read as max range."""
    assert ultrasound_range(0.1, 0.28, 4.5) == 0.28
    assert ultrasound_range(None, 0.28, 4.5) == 4.5
    assert ultrasound_range(float('inf'), 0.28, 4.5) == 4.5
    assert finite_or(float('nan'), 1.0) == 1.0
