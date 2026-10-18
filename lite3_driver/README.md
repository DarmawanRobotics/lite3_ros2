# lite3_driver

ROS 2 driver of the Deep Robotics Jueying Lite3. It speaks the motion host's UDP protocol
(Deep Robotics manual V2.2.1, *UDP Commands* V1.1.0-0): velocity as speed commands in navigation mode, the 50 Hz
state report and the 100 Hz joint reports as ROS topics, every robot command as a
`std_srvs/Trigger` service, and gestures that block until the robot is done.

## Nodes

| Node | Executable | |
|---|---|---|
| `lite3_driver` | `driver_node` | UDP link to the motion host, ROS topics and services |

## Usage

```bash
ros2 launch lite3_driver driver.launch.py namespace:=robot
ros2 topic echo /robot/state
ros2 service call /robot/stand_up std_srvs/srv/Trigger
```

## Topics (in the launch namespace, `robot` by default)

| | Name | Type | |
|---|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` | `base_link`, REP-103: `linear.x` forward, `linear.y` left, `angular.z` counter-clockwise; zeros after `cmd_timeout_sec` without a command |
| pub | `odom` | `nav_msgs/Odometry` | leg odometry: pose from `pos_world`, twist in `base_link` from `vel_body`; **no TF** |
| pub | `imu` | `sensor_msgs/Imu` | attitude (report degrees → quaternion), rates, acceleration, frame `imu_link`; from the 200 Hz IMU datagram `0x010901` when the motion host sends it, else from the state report |
| pub | `battery` | `sensor_msgs/BatteryState` | `percentage` 0–1, `battery_hz` |
| pub | `joint_states` | `sensor_msgs/JointState` | `FL_HipX_joint` ... `HR_Knee_joint` (the official model's names): angles and velocities |
| pub | `ultrasound/front`, `ultrasound/rear` | `sensor_msgs/Range` | 0.28–4.5 m, frames `ultrasound_front`, `ultrasound_rear` |
| pub | `state` | `lite3_interfaces/RobotState` | basic state, gait, motion, battery, ultrasound, link, control mode, gesture |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | ERROR without state reports, WARN on protection, lost balance, low battery |

## Services (`std_srvs/Trigger`)

| Service | Sends | Notes |
|---|---|---|
| `stand_up`, `sit_down` | stand/sit toggle `0x21010202` | state aware: nothing to do when already there, refused while walking or in a transition |
| `emergency_stop` | soft e-stop `0x21020C0E` | also zeroes the velocity and stops a gesture |
| `navigation_mode`, `manual_mode` | `0x21010C03`, `0x21010C02` | navigation: speed commands are obeyed; manual: the remote drives |
| `move_mode`, `pose_mode` | `0x21010D06`, `0x21010D05` | |
| `gait_slow`, `gait_medium`, `gait_fast`, `gait_crawl` | flat gaits `0x21010300`, `0x21010307`, `0x21010303`, `0x21010406` | not in AI state |
| `gait_general`, `gait_grip`, `gait_high_step` | rough-ground gaits `0x21010401`, `0x21010402`, `0x21010407` | not in AI state |
| `reset_zero` | `0x21010C05` | refused while standing |
| `stop_action` | `0x21010C0B` with value 0 and 1 | |
| `keep_stepping_on`, `keep_stepping_off` | `0x21010C06` -1 / 2 | |
| `speaker_on`, `speaker_off` | `0x2101030D` 1 / 0 | |
| `save_data` | `0x21010C01` | saves 100 s of data, then the robot soft-stops and **quits its motion program** |
| `hello` | `0x21010507` | sits down first when standing (the action needs sitting), waves, stands up again; blocks until done |
| `nod`, `bow`, `look_left`, `look_right` | pose-mode axis keyframes | standing still only; manual + pose mode while it runs, move (and navigation) mode after; blocks until done |

Gesture services run on their own callback group: the driver keeps streaming and publishing
while one runs. Any non-zero `cmd_vel` stops a running gesture.

## Velocity

`cmd_vel` is clamped to `max_*` and the protocol's ranges (1.0 m/s, 0.5 m/s, 1.5 rad/s) and sent
as the three speed commands `0x0140` (x), `0x0145` (y), `0x0141` (yaw) at `cmd_rate_hz`. x and y
go out as they are; the yaw rate is negated, because the motion host turns right for a positive
value (like its axis commands): Deep Robotics' own ROS 2 bridge
([Lite3_ROS](https://github.com/DeepRoboticsLab/Lite3_ROS), `Jetson2Motion.cpp`) sends
`-angular.z` too. The robot only
obeys them in navigation mode: with `auto_navigation_mode` the driver sends `0x21010C03` when
motion starts after an idle stream, and every `mode_refresh_sec` while moving. When `cmd_vel`
goes quiet for `cmd_timeout_sec` the driver sends zeros for `stop_burst_sec`, then nothing.

If the robot still turns or strafes the wrong way, set `invert_wz` / `invert_vy` ([Verify on the real robot](../README.md#verify-on-the-real-robot)).

## Fast IMU

The manual lists no IMU datagram of its own, but Deep Robotics' bridge also decodes `0x010901`:
a `uint32` timestamp and nine floats (roll, pitch, yaw in degrees, rates, acceleration) at
200 Hz. While those arrive (`fast_imu: true`), `imu` is published from each of them and no longer
from the 50 Hz state report; `diagnostics` shows which source is in use (`imu_source`).

## State report layouts

The document gives `RobotStateUpload` as a plain C struct. The codec recognises it by its
length: 208 bytes (natural alignment, what an aarch64 or x86-64 motion program sends), 204
(`pack(4)`) and 201 (`pack(1)`). Any other length of at least 156 bytes still gives the attitude,
odometry and velocities (`layout: prefix`, logged once). `rpy` is in degrees, `battery_level` is
read as 0–1 or as a percentage.

## Parameters

[config/driver_params.yaml](config/driver_params.yaml), every one commented:

| Parameter | Default | |
|---|---|---|
| `motion_host_ip`, `motion_host_port` | `192.168.1.120`, `43893` | where commands go |
| `local_ip`, `local_port` | `0.0.0.0`, `43897` | where reports arrive (listed in the motion host's `network.toml`) |
| `heartbeat_hz` | 5.0 | at least 2 Hz |
| `cmd_rate_hz`, `cmd_timeout_sec`, `stop_burst_sec` | 20.0, 0.5, 0.5 | velocity stream and watchdog |
| `auto_navigation_mode`, `mode_refresh_sec` | true, 2.0 | |
| `max_vx`, `max_vy`, `max_wz` | 1.0, 0.5, 1.5 | |
| `invert_vy`, `invert_wz` | false | sign fixes, see above |
| `odom_frame`, `base_frame`, `imu_frame` | `odom`, `base_link`, `imu_link` | message frames (no TF) |
| `odom_hz`, `joint_state_hz`, `state_hz`, `battery_hz`, `ultrasound_hz` | 50, 50, 10, 1, 10 | |
| `state_timeout_sec` | 1.0 | no report for this long: disconnected |
| `low_battery` | 0.2 | diagnostics warn below |
| `fast_imu` | true | publish `imu` from `0x010901` datagrams when they arrive |
| `gesture.*` | | keyframe amplitudes and timing, hello timeouts |

## Launch

`launch/driver.launch.py`:

| Argument | Default | |
|---|---|---|
| `namespace` | `robot` | topics and services |
| `params_file` | [config/driver_params.yaml](config/driver_params.yaml) | a copy with this robot's settings |

The full robot (URDF, LiDAR, camera) starts from [lite3_bringup](../lite3_bringup/README.md).

## Test

```bash
colcon test --packages-select lite3_driver && colcon test-result --verbose
```

The codec is checked against the document's own worked example (appendix 2.1), all three
struct layouts and the fast IMU datagram; the velocity policy, the gestures (against a simulated motion host) and the node
(against a fake motion host on the loopback interface) have their own tests.
