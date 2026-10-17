# lite3_bringup

The Lite3 robot package in one launch, as [Molly](https://github.com/DarmawanRobotics/Molly)
expects it (`src/xtras/README.md` there): the driver, `robot_state_publisher` with the URDF, the
Livox Mid-360 and the camera. It starts no Molly node.

## Usage

```bash
ros2 launch lite3_bringup bringup.launch.py                          # namespace robot, RTSP camera
ros2 launch lite3_bringup bringup.launch.py camera:=realsense
ros2 launch lite3_bringup bringup.launch.py lidar_ip:=192.168.1.150 host_ip:=192.168.1.110
ros2 launch lite3_bringup bringup.launch.py motion_host:=192.168.2.1   # other robot network
```

## Launch

`launch/bringup.launch.py`:

| Argument | Default (environment) | |
|---|---|---|
| `namespace` | `robot` (`ROBOT_NS`) | driver topics and services, the URDF and the camera node |
| `camera` | `rtsp` (`LITE3_CAMERA`) | `rtsp`: the robot's own front camera through `lite3_camera`; `realsense`: a D435i through `realsense2_camera`; `none` |
| `lidar` | `true` (`LITE3_LIDAR`) | Livox Mid-360 through `livox_ros_driver2`, when that package is built |
| `motion_host` | empty (`LITE3_MOTION_HOST`) | motion host IP for the driver and the RTSP url; empty: the params files (`192.168.1.120`) |
| `lidar_ip`, `host_ip` | empty (`LIVOX_LIDAR_IP`, `LIVOX_HOST_IP`) | Mid-360 and this computer's address; when set, the launch writes `~/.lite3/MID360_config.json` from [config/MID360_config.json](config/MID360_config.json) |
| `driver_params` | [`lite3_driver/config/driver_params.yaml`](../lite3_driver/config/driver_params.yaml) (`LITE3_DRIVER_PARAMS`) | a copy with this robot's settings, e.g. `invert_wz` |
| `camera_params` | [`lite3_camera/config/camera_params.yaml`](../lite3_camera/config/camera_params.yaml) (`LITE3_CAMERA_PARAMS`) | a copy with the calibration file |
| `rviz` | `false` | RViz with the robot model |

## Nodes and topics

| Node | Namespace | Output |
|---|---|---|
| `robot_state_publisher` | `robot` | `base_link → legs, imu_link, livox_frame → livox_imu_frame, camera_link, ultrasound_*` |
| `lite3_driver` | `robot` | `cmd_vel` in; `odom`, `imu`, `battery`, `joint_states`, `ultrasound/*`, `state`, `diagnostics`; Trigger services ([lite3_driver](../lite3_driver/README.md)) |
| `livox_lidar_publisher` | root | `/livox/lidar` (CustomMsg, `xfer_format` 1), `/livox/imu`, frame `livox_frame` |
| `front_camera` (`camera:=rtsp`) | `robot` | `/camera/camera/color/image_raw`, `.../image_raw/compressed`, `.../camera_info` |
| `camera` (`camera:=realsense`) | `camera` | the same topics from `realsense2_camera` (color 640×480 at 15 Hz, depth off) |

The parameter files use `/**:` keys: a bare node name would not match a node in a namespace.

## Setup script

[`scripts/lite3_setup.sh`](scripts/lite3_setup.sh) is the `ROBOT_SETUP` hook of Molly's `script/setup.sh`:

```bash
lite3_bringup/scripts/lite3_setup.sh check            # motion host, its state reports, GStreamer, OpenCV
lite3_bringup/scripts/lite3_setup.sh install          # apt packages (REALSENSE=1: the D435i driver too)
lite3_bringup/scripts/lite3_setup.sh network          # the network.toml change for this computer
lite3_bringup/scripts/lite3_setup.sh network --apply  # make it on the motion host over ssh, with a backup
```

The motion host only sends its state to the computers listed in `~/jy_exe/conf/network.toml`
(Deep Robotics *SDK Deploy* 1.3.2). `network` prints the two lines to change for this computer's
IP (`ips` and `ports`, keeping the perception host's entry); `--apply` edits the file over ssh
(`ysc@<motion host>`) after a timestamped backup. Restart the robot afterwards. Nothing on the
robot changes without `--apply`.

Environment: `MOTION_HOST` (`ROBOT_IP`, else `192.168.1.120`), `MOTION_USER` (`ysc`),
`LOCAL_PORT` (`43897`), `ROS_DISTRO` (`humble`).
