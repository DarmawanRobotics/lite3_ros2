# lite3_description

URDF of the Deep Robotics Jueying Lite3 with its sensor frames, as simple shapes (no meshes).

```bash
ros2 launch lite3_description display.launch.py              # RViz with joint sliders
ros2 launch lite3_description description.launch.py namespace:=robot camera:=rtsp lidar:=mid360
```

| Argument | Values | |
|---|---|---|
| `camera` | `rtsp` (default), `realsense`, `none` | `rtsp`: the robot's own wide-angle front camera, `camera_link` + `camera_color_optical_frame`; `realsense`: a D435i, only `camera_link` (realsense2_camera publishes its optical frames) |
| `lidar` | `mid360` (default), `none` | Livox Mid-360: `livox_frame` + `livox_imu_frame` |

## Frames (relative to `base_link`, metres)

`base_link` is the centre of the body: x forward, y left, z up. The body box and its inertia hang
on a fixed `trunk` link, so the root carries no inertia.

| Frame | Position | Source |
|---|---|---|
| `FL_HipX` ... `HR_HipX` joints | (±0.1745, ±0.062, 0) | Motion System 1.1.2 |
| `*_HipY` joints | (0, ±0.0985, 0) from HipX | 1.1.2 |
| `*_Knee` joints | (0, 0, −0.20) from HipY | 1.1.2 |
| `*_FOOT` | (0, 0, −0.21) from Knee, r 0.02 | 1.1.2, 1.2.2 |
| `imu_link` | (0.005, 0, 0) | 1.1.3 |
| `livox_frame` (Mid-360) | (0.18678, 0, 0.12498) | 1.1.3 and the measured TF drawings |
| `livox_imu_frame` | (0.011, 0.02329, −0.04412) from `livox_frame` | Mid-360 internal IMU; FAST-LIO's `extrinsic_T` is its negative |
| `camera_link` (D435i, `camera:=realsense`) | (0.25228, −0.0005, 0.0741) | 1.1.3 and the measured TF drawings |
| `camera_link` (front camera, `camera:=rtsp`) | (0.280, 0, 0.0) | **estimate**: measure it |
| `ultrasound_front`, `ultrasound_rear` | (±0.285, 0, −0.03), rear turned 180° | **estimates** |

Joint ranges are those of section 1.2.4 (HipX ±24°, HipY −200°…20°, Knee 34.5°…156°), torques
and speeds from the same table. Joint axes (HipX −x, HipY and Knee −y) and the leg inertials
come from Deep Robotics' Lite3 URDF
([DeepRoboticsLab/Lite3_rl_training](https://github.com/DeepRoboticsLab/Lite3_rl_training),
BSD-3-Clause); they match the signs of the angles `lite3_driver` reports.

No link may be called `body`, `odom`, `map` or `camera_init`: those frames belong to FAST-LIO
and Molly. A test checks it.

## Test

```bash
colcon test --packages-select lite3_description && colcon test-result --verbose
```
