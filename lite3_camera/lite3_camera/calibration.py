"""Camera calibration: the camera_calibration YAML, rectification maps and approximations."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml

FISHEYE = 'equidistant'


@dataclass
class Calibration:
    """Intrinsics of one camera at one image size."""

    width: int
    height: int
    k: np.ndarray                 # 3 x 3
    d: np.ndarray                 # distortion coefficients
    model: str = 'plumb_bob'      # plumb_bob, rational_polynomial or equidistant

    def scaled(self, width: int, height: int) -> 'Calibration':
        """Return the calibration for frames of another size (same field of view)."""
        if (width, height) == (self.width, self.height):
            return self
        sx, sy = width / self.width, height / self.height
        k = self.k.copy()
        k[0, 0] *= sx
        k[0, 2] *= sx
        k[1, 1] *= sy
        k[1, 2] *= sy
        return Calibration(width, height, k, self.d.copy(), self.model)


def matrix(entry, rows: int, cols: int) -> np.ndarray:
    """Read a {rows, cols, data} matrix of the calibration file."""
    data = entry['data'] if isinstance(entry, dict) else entry
    return np.array(data, dtype=np.float64).reshape(rows, cols)


def load_calibration(path: str) -> Calibration:
    """Read a camera_calibration / camera_info_manager YAML file."""
    with open(path) as f:
        data = yaml.safe_load(f)
    d = data['distortion_coefficients']
    coefficients = np.array(d['data'] if isinstance(d, dict) else d, dtype=np.float64)
    return Calibration(
        width=int(data['image_width']),
        height=int(data['image_height']),
        k=matrix(data['camera_matrix'], 3, 3),
        d=coefficients,
        model=str(data.get('distortion_model', 'plumb_bob')),
    )


def fov_intrinsics(width: int, height: int, hfov_deg: float) -> np.ndarray:
    """Return a pinhole camera matrix from the horizontal field of view (no calibration)."""
    f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[f, 0.0, (width - 1) / 2.0], [0.0, f, (height - 1) / 2.0], [0, 0, 1.0]])


def rectifier(calibration: Calibration, alpha: float) -> Tuple[np.ndarray, tuple]:
    """Return (new camera matrix, remap maps) that undistort frames of the calibration's size."""
    size = (calibration.width, calibration.height)
    alpha = min(1.0, max(0.0, float(alpha)))
    if calibration.model == FISHEYE:
        d = calibration.d[:4].reshape(4, 1)
        new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            calibration.k, d, size, np.eye(3), balance=alpha
        )
        maps = cv2.fisheye.initUndistortRectifyMap(
            calibration.k, d, np.eye(3), new_k, size, cv2.CV_16SC2
        )
    else:
        new_k, _ = cv2.getOptimalNewCameraMatrix(calibration.k, calibration.d, size, alpha, size)
        maps = cv2.initUndistortRectifyMap(
            calibration.k, calibration.d, None, new_k, size, cv2.CV_16SC2
        )
    return new_k, maps


def info_values(k: np.ndarray, width: int, height: int) -> dict:
    """Return the camera_info fields of an ideal (rectified) pinhole camera."""
    p = np.zeros((3, 4))
    p[:, :3] = k
    return {
        'width': int(width),
        'height': int(height),
        'distortion_model': 'plumb_bob',
        'd': [0.0] * 5,
        'k': [float(v) for v in k.flatten()],
        'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'p': [float(v) for v in p.flatten()],
    }


def try_load(path: str) -> Tuple[Optional[Calibration], str]:
    """Return (calibration, '') or (None, why it could not be read)."""
    if not path:
        return None, 'no calibration_file'
    try:
        return load_calibration(path), ''
    except (OSError, KeyError, ValueError, TypeError, yaml.YAMLError) as error:
        return None, f'{path}: {error}'
