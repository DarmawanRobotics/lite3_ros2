# lite3_description

URDF of the Deep Robotics Jueying Lite3: Deep Robotics' own model with its meshes, plus the
sensor frames of the manual V2.2.1, as xacro.

## Usage

```bash
ros2 launch lite3_description display.launch.py              # RViz with joint sliders
ros2 launch lite3_description description.launch.py namespace:=robot camera:=rtsp lidar:=mid360
```

## Launch

`description.launch.py` (`robot_state_publisher`, also takes `namespace`, default `robot`) and
`display.launch.py` (adds RViz and, with `gui:=true` (default), joint sliders):

| Argument | Values | |
|---|---|---|
| `camera` | `rtsp` (default), `realsense`, `none` | `rtsp`: the robot's own wide-angle front camera, `camera_link` + `camera_color_optical_frame`; `realsense`: a D435i, only `camera_link` (realsense2_camera publishes its optical frames) |
| `lidar` | `mid360` (default), `robosense`, `none` | Livox Mid-360: `livox_frame` + `livox_imu_frame`; RoboSense (LiDAR edition): `rslidar` |

## Model

Links, joints, meshes, collisions and inertials are those of
[deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model) `Lite3/urdf/Lite3.urdf`
(BSD-3-Clause, [LICENSE.deep_robotics_model](LICENSE.deep_robotics_model)), unchanged and with
the official names: `TORSO`, `{FL,FR,HL,HR}_{HIP,THIGH,SHANK,FOOT}`, revolute joints
`*_HipX_joint`, `*_HipY_joint`, `*_Knee_joint` (the names `lite3_driver` publishes) and fixed
`*_Ankle`. `base_link` is the root, with `TORSO` on it by an identity joint.

## Frames (relative to `base_link`, metres)

`base_link` is the centre of the body: x forward, y left, z up.

| Frame | Position | Source |
|---|---|---|
| `*_HipX_joint` | (±0.1745, ±0.062, 0) | model (= manual 1.1.2) |
| `*_HipY_joint` | (0, ±0.09735, 0) from HipX | model (manual: 0.0985) |
| `*_Knee_joint` | (0, 0, −0.20) from HipY | model (= manual) |
| `*_FOOT` | (0, 0, −0.21012) from Knee, collision sphere r 0.022 | model (manual: 0.21) |
| `imu_link` | (0.005, 0, 0) | manual 1.1.3 |
| `livox_frame` (Mid-360) | (0.18678, 0, 0.12498) | manual 1.1.3 |
| `livox_imu_frame` | (0.011, 0.02329, −0.04412) from `livox_frame` | Mid-360 internal IMU; FAST-LIO's `extrinsic_T` is its negative |
| `rslidar` (RoboSense, `lidar:=robosense`) | (0.2033, 0, 0.12906) | manual 1.1.3 |
| `camera_link` (D435i, `camera:=realsense`) | (0.25228, −0.0005, 0.0741) | manual 1.1.3 |
| `camera_link` (front camera, `camera:=rtsp`) | (0.280, 0, 0.0) | **estimate**: measure it |
| `ultrasound_front`, `ultrasound_rear` | (±0.285, 0, −0.03), rear turned 180° | **estimates** |

Joint limits are the model's (HipX ±0.523, HipY −2.67…0.314, Knee 0.524…2.792 rad); the manual
lists slightly different ranges (1.2.4: HipX ±24°, HipY −200°…20°, Knee 34.5°…156°). The axes
(HipX −x, HipY and Knee −y) match the signs of the angles the motion host reports.

No link may be called `body`, `odom`, `map` or `camera_init`: those frames belong to FAST-LIO
and Molly. A test checks it.

## Test

```bash
colcon test --packages-select lite3_description && colcon test-result --verbose
```

The test expands every variant, checks the legs against the official model's numbers, the
sensor frames against the manual, that every mesh is in the package, and the tree.
