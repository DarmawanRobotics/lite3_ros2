"""
Codec of the Lite3 motion host's UDP protocol (Deep Robotics "UDP Commands" V1.1.0).

Every datagram starts with a little-endian CommandHead {uint32 code, uint32 paramters_size,
uint32 type}. Simple commands (type 0) carry their value in paramters_size; complex commands
(type 1) carry paramters_size bytes of payload. No ROS imports: unit-tested on its own.
"""

from dataclasses import dataclass
import math
import struct
from typing import Optional, Tuple

HEAD = struct.Struct('<III')
HEAD_SIZE = HEAD.size
TYPE_SIMPLE = 0
TYPE_COMPLEX = 1
MAX_PAYLOAD = 1024  # the receiving buffer of the document's own example

# ---------------------------------------------------------------- commands to the motion host

HEARTBEAT = 0x21040001          # at least 2 Hz
STAND_SIT = 0x21010202          # toggles sitting and standing
SOFT_ESTOP = 0x21020C0E         # software emergency stop
RESET_ZERO = 0x21010C05         # initialize the joints
ENTER_AI = 0x21010528
EXIT_AI = 0x2101052B

# Axis commands, values in [-AXIS_MAX, AXIS_MAX]. Pose mode: posture; move mode: joystick.
AXIS_MAX = 32767
AXIS_ROLL = 0x21010131          # pose: positive rolls right
AXIS_PITCH = 0x21010130         # pose: positive lowers the head
AXIS_HEIGHT = 0x21010102        # pose: positive raises the body
AXIS_YAW = 0x21010135           # pose: positive rotates right
AXIS_LATERAL = AXIS_ROLL        # move: positive moves right
AXIS_FORWARD = AXIS_PITCH       # move: positive moves forward
AXIS_TURN = AXIS_YAW            # move: positive turns right
AXIS_DEAD_ZONES = {AXIS_ROLL: 12553, AXIS_PITCH: 6553, AXIS_HEIGHT: 20000, AXIS_YAW: 9553}

POSE_MODE = 0x21010D05
MOVE_MODE = 0x21010D06
NAVIGATION_MODE = 0x21010C03    # speed commands from the perception side are obeyed
MANUAL_MODE = 0x21010C02        # the remote controller drives

GAIT_FLAT_SLOW = 0x21010300
GAIT_FLAT_MEDIUM = 0x21010307
GAIT_FLAT_FAST = 0x21010303
GAIT_FLAT_CRAWL = 0x21010406
GAIT_RUG_GRIP = 0x21010402
GAIT_RUG_GENERAL = 0x21010401
GAIT_RUG_HIGH_STEP = 0x21010407

ACTION_TWIST = 0x21010204       # standing
ACTION_TURN_OVER = 0x21010205   # sitting
ACTION_MOONWALK = 0x2101030C    # standing
ACTION_BACKFLIP = 0x21010502    # sitting
ACTION_HELLO = 0x21010507       # sitting
ACTION_LONG_JUMP = 0x2101050B   # sitting
ACTION_TWIST_JUMP = 0x2101020D  # standing
STOP_ACTION = 0x21010C0B        # values 0 and 1 must both be sent

SAVE_DATA = 0x21010C01          # saves 100 s of data, then soft e-stop and motion program exit
KEEP_STEPPING = 0x21010C06
KEEP_STEPPING_ON = -1
KEEP_STEPPING_OFF = 2
VOICE_COMMAND = 0x21010C0A
SPEAKER = 0x2101030D
SPEAKER_OFF = 0
SPEAKER_ON = 1
SPEAKER_QUERY = 2
PERCEPTION_SETTINGS = 0x21012109
PERCEPTION_OFF = 0x00
PERCEPTION_AUTO_STOP = 0x20
PERCEPTION_TRACKING = 0xC0

AI_GAIT_BASIC = 0x2101052A
AI_GAIT_FAST = 0x2101052E
AI_GAIT_JUMP = 0x21010529
AI_GAIT_STAND = 0x2101052C
AI_ACTION = 0x2101030E

# Speed commands (complex, one double each, navigation mode only)
SPEED_VX = 0x0140               # m/s, [-1.0, 1.0]
SPEED_WZ = 0x0141               # rad/s, [-1.5, 1.5]
SPEED_VY = 0x0145               # m/s, [-0.5, 0.5]
SPEED_LIMITS = {SPEED_VX: 1.0, SPEED_VY: 0.5, SPEED_WZ: 1.5}

# ---------------------------------------------------------------- reports from the motion host

STATE = 0x0901                  # RobotStateUpload, 50 Hz
JOINT_ANGLES = 0x0902           # 12 doubles, rad, 100 Hz
JOINT_VELOCITIES = 0x0903       # 12 doubles, rad/s, 100 Hz
SPEAKER_STATUS = 0x11050F08     # simple: 0 off, 1 on

# Not in the document: Deep Robotics' own ROS 2 bridge (Lite3_ROS, transfer/protocol.hpp)
# decodes a 200 Hz IMU datagram too: uint32 timestamp, roll/pitch/yaw (degrees), rates (rad/s)
# and acceleration (m/s^2) as floats. Used when the motion host sends it.
IMU_FAST = 0x010901
IMU_FAST_STRUCT = struct.Struct('<I9f')  # 40 bytes

# The report's order (FL, FR, HL, HR x HipX, HipY, Knee) with the joint names of Deep Robotics'
# model (deep_robotics_model Lite3.urdf), which lite3_description uses.
JOINT_NAMES = tuple(
    f'{leg}_{joint}_joint'
    for leg in ('FL', 'FR', 'HL', 'HR')
    for joint in ('HipX', 'HipY', 'Knee')
)
JOINT_ARRAY = struct.Struct('<12d')

BASIC_STATES = {
    1: 'sitting',
    4: 'prepare',
    5: 'standing_up',
    6: 'standing',
    7: 'sitting_down',
    8: 'protection',
    9: 'posture_adjustment',
    11: 'flipping_over',
    16: 'ai',
    17: 'reset_to_zero',
    18: 'backflip',
    20: 'hello',
}
GAIT_STATES = {
    0: 'flat_slow',
    2: 'rug_general',
    4: 'flat_medium',
    5: 'flat_fast',
    6: 'rug_grip',
    12: 'moonwalk',
    13: 'rug_high_step',
}
POLICY_STATES = {0: 'ai_basic', 16: 'ai_jump', 18: 'ai_stand', 20: 'ai_fast'}
MOTION_STATES = {0: 'still', 1: 'stepping', 2: 'twist', 4: 'twist_jump', 11: 'long_jump'}

STATE_SITTING = 1
STATE_STANDING = 6
STATE_AI = 16
STATE_HELLO = 20
MOTION_STEPPING = 1

# RobotStateUpload: the document gives a plain C struct. The motion host sends it with the
# compiler's natural alignment (208 bytes); pack(4) and pack(1) builds are recognised too.
PREFIX = ('basic_state', 'gait_state', 'policy_state')
DOUBLES = ('rpy', 'rpy_vel', 'xyz_acc', 'pos_world', 'vel_world', 'vel_body')
TAIL = (
    'touch_down', 'is_charging', 'error_state', 'motion_state', 'battery_level', 'task_state',
    'need_move', 'zero_position', 'after_first_start', 'voice_enabled', 'ultrasound_front',
    'ultrasound_rear',
)
STATE_LAYOUTS = {
    'natural': struct.Struct('<iii4x18dI?3xIidi????2d'),
    'pack4': struct.Struct('<iii18dI?3xIidi????2d'),
    'pack1': struct.Struct('<iii18dI?Iidi????2d'),
}
STATE_SIZES = {layout.size: name for name, layout in STATE_LAYOUTS.items()}
PREFIX_NATURAL = struct.Struct('<iii4x18d')   # through vel_body, 160 bytes
PREFIX_PACKED = struct.Struct('<iii18d')      # through vel_body, 156 bytes


@dataclass(frozen=True)
class Datagram:
    """One datagram from the motion host."""

    code: int
    kind: int
    value: int        # simple: the command value; complex: the payload length
    payload: bytes


@dataclass(frozen=True)
class StateReport:
    """A decoded RobotStateUpload; tail fields are None when the layout is unknown."""

    basic_state: int
    gait_state: int
    policy_state: int
    rpy_deg: Tuple[float, float, float]
    rpy_vel: Tuple[float, float, float]
    xyz_acc: Tuple[float, float, float]
    pos_world: Tuple[float, float, float]
    vel_world: Tuple[float, float, float]
    vel_body: Tuple[float, float, float]
    layout: str
    motion_state: Optional[int] = None
    battery: Optional[float] = None
    error_state: Optional[int] = None
    balance_lost: Optional[bool] = None
    zeroed: Optional[bool] = None
    first_start: Optional[bool] = None
    voice_enabled: Optional[bool] = None
    ultrasound: Optional[Tuple[float, float]] = None

    @property
    def standing(self) -> bool:
        """Return True in the torque-control (standing) state."""
        return self.basic_state == STATE_STANDING

    @property
    def sitting(self) -> bool:
        """Return True in the sitting state."""
        return self.basic_state == STATE_SITTING

    @property
    def stepping(self) -> bool:
        """Return True while the legs step (the robot walks or treads)."""
        return self.motion_state == MOTION_STEPPING


@dataclass(frozen=True)
class ImuReport:
    """A decoded fast IMU datagram (0x010901)."""

    timestamp: int
    rpy_deg: Tuple[float, float, float]
    rpy_vel: Tuple[float, float, float]
    xyz_acc: Tuple[float, float, float]


def u32(value: int) -> int:
    """Return a signed command value as the uint32 the head carries (two's complement)."""
    return int(value) & 0xFFFFFFFF


def simple(code: int, value: int = 0) -> bytes:
    """Encode a simple command."""
    return HEAD.pack(code, u32(value), TYPE_SIMPLE)


def complex_command(code: int, payload: bytes) -> bytes:
    """Encode a complex command with its payload."""
    return HEAD.pack(code, len(payload), TYPE_COMPLEX) + payload


def speed(code: int, value: float) -> bytes:
    """Encode one speed command, clamped to its range; a non-finite value is sent as 0."""
    limit = SPEED_LIMITS[code]
    value = float(value) if math.isfinite(value) else 0.0
    return complex_command(code, struct.pack('<d', max(-limit, min(limit, value))))


def axis(code: int, value: int) -> bytes:
    """Encode an axis command, clamped to [-AXIS_MAX, AXIS_MAX]."""
    return simple(code, max(-AXIS_MAX, min(AXIS_MAX, int(value))))


def parse_datagram(data: bytes) -> Optional[Datagram]:
    """Split a datagram into head and payload; None when it is malformed."""
    if len(data) < HEAD_SIZE:
        return None
    code, size, kind = HEAD.unpack_from(data)
    if kind == TYPE_SIMPLE:
        return Datagram(code, kind, size, b'')
    if kind != TYPE_COMPLEX or size > MAX_PAYLOAD or len(data) < HEAD_SIZE + size:
        return None
    return Datagram(code, kind, size, bytes(data[HEAD_SIZE:HEAD_SIZE + size]))


def signed(value: int) -> int:
    """Return the uint32 value of a simple command as a signed int."""
    return value - (1 << 32) if value & 0x80000000 else value


def triples(values) -> tuple:
    """Group 18 doubles into the six 3-vectors of the state report."""
    return tuple(tuple(values[i:i + 3]) for i in range(0, 18, 3))


def normalize_battery(value) -> Optional[float]:
    """Return the battery level as 0-1; the report may carry 0-1 or a percentage."""
    if value is None or not math.isfinite(value):
        return None
    if value > 1.0:
        value /= 100.0
    return max(0.0, min(1.0, value))


def decode_state(payload: bytes) -> Optional[StateReport]:
    """Decode a RobotStateUpload payload by its length; None when it is too short."""
    layout = STATE_SIZES.get(len(payload))
    if layout is not None:
        fields = STATE_LAYOUTS[layout].unpack(payload)
        head, vectors, tail = fields[:3], triples(fields[3:21]), fields[21:]
        values = dict(zip(TAIL, tail))
        return StateReport(
            *head,
            *vectors,
            layout=layout,
            motion_state=values['motion_state'],
            battery=normalize_battery(values['battery_level']),
            error_state=values['error_state'],
            balance_lost=bool(values['need_move']),
            zeroed=bool(values['zero_position']),
            first_start=bool(values['after_first_start']),
            voice_enabled=bool(values['voice_enabled']),
            ultrasound=(values['ultrasound_front'], values['ultrasound_rear']),
        )
    if len(payload) >= PREFIX_NATURAL.size:
        fields = PREFIX_NATURAL.unpack_from(payload)
    elif len(payload) >= PREFIX_PACKED.size:
        fields = PREFIX_PACKED.unpack_from(payload)
    else:
        return None
    return StateReport(*fields[:3], *triples(fields[3:21]), layout='prefix')


def decode_joints(payload: bytes) -> Optional[Tuple[float, ...]]:
    """Decode 12 joint values (angles or velocities); None when the payload is short."""
    if len(payload) < JOINT_ARRAY.size:
        return None
    return JOINT_ARRAY.unpack_from(payload)


def decode_imu(payload: bytes) -> Optional[ImuReport]:
    """Decode a fast IMU payload; None when it is short."""
    if len(payload) < IMU_FAST_STRUCT.size:
        return None
    values = IMU_FAST_STRUCT.unpack_from(payload)
    return ImuReport(values[0], tuple(values[1:4]), tuple(values[4:7]), tuple(values[7:10]))


def name_of(table: dict, value: Optional[int]) -> str:
    """Return the readable name of a state code, 'unknown' when it is not in the table."""
    return table.get(value, 'unknown') if value is not None else 'unknown'


def encode_state(report: StateReport, layout: str = 'natural') -> bytes:
    """Encode a state report as the motion host would (tests and the simulator)."""
    battery = report.battery if report.battery is not None else 0.0
    front, rear = report.ultrasound if report.ultrasound is not None else (4.5, 4.5)
    vectors = [v for vector in (
        report.rpy_deg, report.rpy_vel, report.xyz_acc, report.pos_world, report.vel_world,
        report.vel_body,
    ) for v in vector]
    payload = STATE_LAYOUTS[layout].pack(
        report.basic_state, report.gait_state, report.policy_state, *vectors,
        0, False, report.error_state or 0, report.motion_state or 0, battery, 0,
        bool(report.balance_lost), bool(report.zeroed), bool(report.first_start),
        bool(report.voice_enabled), front, rear,
    )
    return complex_command(STATE, payload)


def encode_imu(report: ImuReport) -> bytes:
    """Encode a fast IMU datagram as the motion host would (tests and the simulator)."""
    return complex_command(IMU_FAST, IMU_FAST_STRUCT.pack(
        report.timestamp, *report.rpy_deg, *report.rpy_vel, *report.xyz_acc))


def encode_joints(code: int, values) -> bytes:
    """Encode 12 joint values as the motion host would (tests and the simulator)."""
    return complex_command(code, JOINT_ARRAY.pack(*values))
