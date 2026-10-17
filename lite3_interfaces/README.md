# lite3_interfaces

Messages of [lite3_driver](../lite3_driver/README.md). No nodes, topics, services or parameters of
its own.

## Messages

| Kind | Name | Published on |
|---|---|---|
| msg | [`RobotState`](msg/RobotState.msg) | `/<namespace>/state` (`state_hz`, 10 Hz by default) |

`RobotState` fields:

| Field | |
|---|---|
| `header`, `connected`, `age_sec` | a state report arrived within `state_timeout_sec`; time since the last one |
| `basic_state`, `basic_state_name`, `standing` | posture (6 standing) |
| `gait_state`, `gait_name`, `policy_state` | current gait |
| `motion_state`, `motion_name`, `moving` | still, stepping, ... |
| `balance_lost`, `zeroed` | `is_robot_need_move`, `zero_position_flag` |
| `battery` | 0–1 |
| `ultrasound_front`, `ultrasound_rear` | m, 0.28–4.5 |
| `control_mode` | `navigation`, `manual` or `unknown`: the last mode this driver commanded |
| `gesture` | gesture or action running, empty when none |
| `layout` | state report layout: `natural`, `pack4`, `pack1` or `prefix` |

The state codes follow the Deep Robotics *UDP Commands* document (section 1.3.1 and appendix 2.2).
