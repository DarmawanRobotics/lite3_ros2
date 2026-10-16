# lite3_driver -> state: the Lite3 motion host's state report (0x0901) in readable form.

std_msgs/Header header
bool connected            # a state datagram arrived within state_timeout_sec
float32 age_sec           # since the last state datagram
# robot_basic_state: 1 sitting, 4 prepare, 5 sit-to-stand, 6 standing (torque control),
# 7 stand-to-sit, 8 lose-control protection, 9 posture adjustment, 11 flipping over, 16 AI,
# 17 reset to zero, 18 backflip, 20 hello
int32 basic_state
# sitting, prepare, standing_up, standing, sitting_down, protection, posture_adjustment,
# flipping_over, ai, reset_to_zero, backflip, hello, unknown
string basic_state_name
int32 gait_state
# flat_slow, rug_general, flat_medium, flat_fast, rug_grip, moonwalk, rug_high_step, unknown
string gait_name
int32 policy_state
int32 motion_state        # 0 still, 1 stepping, 2 twist, 4 twist jump, 11 long jump
string motion_name
bool standing             # basic_state 6
bool moving               # motion_state 1
bool balance_lost         # is_robot_need_move
bool zeroed               # zero_position_flag
float32 battery           # 0-1
float32 ultrasound_front  # m, 0.28-4.5
float32 ultrasound_rear   # m, 0.28-4.5
string control_mode       # navigation | manual | unknown: the last mode this driver commanded
string gesture            # gesture or action running, "" none
string layout             # RobotStateUpload layout detected: natural, pack4, pack1, prefix
