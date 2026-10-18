# Changelog

All notable changes to lite3_ros2. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

### Fixed
- `lite3_driver`: navigation mode is no longer re-sent every 2 s while `cmd_vel` streams, which
  took the robot back from an operator who had switched it to manual on the remote. It goes out
  when motion starts after a quiet stream (`mode_repeats` 3, `mode_repeat_sec` 0.2, replacing
  `mode_refresh_sec`); `manual_mode` drops the rest of the repeats.
- `lite3_driver`: `emergency_stop` sends the soft e-stop three times.
- `lite3_driver`: `emergency_stop`, `sit_down` and `save_data` hold `cmd_vel` until `stand_up`,
  and then until `cmd_vel` has been zero or quiet once; before, the next `cmd_vel` drove again,
  and a stream still running when the robot stood up drove it off at once.
- `lite3_driver`: a `cmd_vel` with NaN or inf in any field is a stop; before, only that component
  was dropped and the rest was executed.

### Changed
- `lite3_driver`: the yaw rate of `cmd_vel` goes out negated (`0x0141`): the motion host turns
  right for a positive value, and Deep Robotics' own bridge (Lite3_ROS) sends `-angular.z`.
  Before, `{angular: {z: 0.3}}` turned the robot clockwise unless `invert_wz` was set.
- `lite3_description`: Deep Robotics' official model (deep_robotics_model `Lite3.urdf`, BSD-3)
  with its meshes, collisions and inertials, as xacro; joints renamed to the official
  `*_HipX_joint`, `*_HipY_joint`, `*_Knee_joint` (and `lite3_driver`'s `joint_states` with them),
  body link `TORSO` under `base_link`. Sensor frames stay those of the manual V2.2.1.

### Added
- `lite3_driver`: the 200 Hz IMU datagram `0x010901` (decoded by Deep Robotics' bridge, not in
  the manual) feeds `imu` while it arrives (`fast_imu`).
- `lite3_description`: `lidar:=robosense` for the LiDAR edition's RoboSense (manual 1.1.3).

### Docs
- References updated to the manual V2.2.1 (UDP Commands V1.1.0-0); motion host address on
  Ethernet and Wi-Fi; SDK mode note; README without the diagram.

### Added
- `lite3_driver`: the motion host's UDP protocol (UDP Commands V1.1.0). `cmd_vel` as speed
  commands in navigation mode with a 0.5 s watchdog; state reports as `odom`, `imu`, `battery`,
  `joint_states`, `ultrasound/front` / `rear`, `state` and `diagnostics`; Trigger services for
  posture, modes, gaits, actions, the speaker and saving data; blocking gestures (`hello`,
  `nod`, `bow`, `look_left`, `look_right`). The state report is decoded by its length (natural,
  `pack(4)` or `pack(1)` layout).
- `lite3_interfaces`: `RobotState`.
- `lite3_description`: URDF with the legs, IMU, Mid-360, D435i or front camera and ultrasound
  frames from the Motion System document; the front camera and ultrasound positions are
  estimates.
- `lite3_camera`: the front RTSP camera with Jetson hardware decoding, reconnects with back-off
  and rectification from a calibration file.
- `lite3_bringup`: one launch for the robot package contract of Molly, and `lite3_setup.sh`
  (`check`, `install`, `network [--apply]` for the motion host's `network.toml`).

### Docs
- README: Quick start (network.toml, build, launch, check) and a "Verify on the real robot"
  checklist (state layout, navigation mode, `invert_wz` / `invert_vy`, camera calibration); fixed
  the `lite3_setup.sh check` path, which ran from the wrong directory.
- Package READMEs share one layout (Nodes, Topics, Services, Parameters, Launch, Usage); added
  the `lite3_driver` launch arguments, the `lite3_camera` parameter table, the `display.launch.py`
  `gui` argument and the `RobotState` fields; relative links checked.
