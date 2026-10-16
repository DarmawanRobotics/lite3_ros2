"""Conversions between the motion host's reports and ROS conventions (no ROS imports)."""

import math
from typing import Optional, Tuple

from lite3_driver import protocol as p


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, ...]:
    """Return (x, y, z, w) of the ZYX Euler angles in radians."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def attitude(report: p.StateReport) -> Tuple[float, float, float]:
    """Return the IMU roll, pitch and yaw in radians (the report carries degrees)."""
    return tuple(math.radians(v) for v in report.rpy_deg)


def odometry_orientation(report: p.StateReport) -> Tuple[float, ...]:
    """Return the body orientation of the leg odometry: IMU roll and pitch, odometry yaw."""
    roll, pitch, _ = attitude(report)
    return quaternion_from_rpy(roll, pitch, report.pos_world[2])


def finite_or(value: Optional[float], default: float) -> float:
    """Return the value when it is a finite number, the default otherwise."""
    if value is None or not math.isfinite(value):
        return default
    return float(value)


def ultrasound_range(value: Optional[float], minimum: float, maximum: float) -> float:
    """Clamp an ultrasound reading to the sensor's valid range (0.28-4.5 m)."""
    return min(maximum, max(minimum, finite_or(value, maximum)))
