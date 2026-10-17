# Changelog

All notable changes to lite3_ros2. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

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
