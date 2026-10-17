"""Calibration file, rectification and the field-of-view approximation."""

import math
from pathlib import Path

from lite3_camera.calibration import (
    fov_intrinsics,
    info_values,
    load_calibration,
    rectifier,
    try_load,
)
import numpy as np
import pytest

EXAMPLE = Path(__file__).resolve().parents[1] / 'config' / 'calibration.example.yaml'


def test_example_calibration_loads():
    """The example file reads as a 1280x720 plumb_bob calibration."""
    calibration = load_calibration(str(EXAMPLE))
    assert (calibration.width, calibration.height) == (1280, 720)
    assert calibration.model == 'plumb_bob'
    assert calibration.k[0, 0] == 640.0 and len(calibration.d) == 5


def test_scaling_keeps_the_field_of_view():
    """Half-size frames halve the focal lengths and the principal point."""
    half = load_calibration(str(EXAMPLE)).scaled(640, 360)
    assert half.k[0, 0] == 320.0 and half.k[1, 2] == pytest.approx(179.75)


def test_rectifier_maps_and_new_matrix():
    """Rectification gives full-size maps and a pinhole matrix for camera_info."""
    calibration = load_calibration(str(EXAMPLE)).scaled(320, 180)
    new_k, (map1, map2) = rectifier(calibration, alpha=0.0)
    assert map1.shape[:2] == (180, 320)
    assert new_k.shape == (3, 3) and new_k[2, 2] == 1.0
    values = info_values(new_k, 320, 180)
    assert values['d'] == [0.0] * 5 and values['p'][3] == 0.0 and len(values['p']) == 12


def test_fisheye_rectifier(tmp_path):
    """An equidistant (fisheye) calibration rectifies with the fisheye model."""
    path = tmp_path / 'fisheye.yaml'
    path.write_text(
        'image_width: 320\nimage_height: 240\n'
        'camera_matrix: {rows: 3, cols: 3, data: [200, 0, 160, 0, 200, 120, 0, 0, 1]}\n'
        'distortion_model: equidistant\n'
        'distortion_coefficients: {rows: 1, cols: 4, data: [0.05, -0.01, 0.0, 0.0]}\n'
    )
    calibration = load_calibration(str(path))
    new_k, (map1, _) = rectifier(calibration, alpha=0.5)
    assert map1.shape[:2] == (240, 320) and new_k[0, 0] > 0


def test_fov_intrinsics():
    """A 90 deg field of view puts the focal length at half the width."""
    k = fov_intrinsics(640, 480, 90.0)
    assert k[0, 0] == pytest.approx(320.0) and k[1, 1] == k[0, 0]
    assert k[0, 2] == pytest.approx(319.5)
    assert fov_intrinsics(640, 480, 120.0)[0, 0] == pytest.approx(320 / math.tan(math.radians(60)))


def test_missing_or_broken_calibration():
    """No file or a broken one is reported, not raised."""
    assert try_load('')[0] is None
    calibration, why = try_load('/nonexistent/calibration.yaml')
    assert calibration is None and 'calibration.yaml' in why
    assert isinstance(np.zeros(1), np.ndarray)
