"""Codec tests against the Deep Robotics UDP Commands document."""

import math
import struct

from lite3_driver import protocol as p
import pytest

# Appendix 2.1: a joint-angle datagram as the document prints it, 16-bit words in byte order.
APPENDIX = (
    '0209 0000 6000 0000 0100 0000 2efd 3ccf 47e4 e1bf f130 4992 f49e e7bf c3d7 eefe '
    '8567 f13f 9ae4 b66d 6583 f1bf 24d8 f33c 3d00 f13f 8c16 222a 0b1f 0140 e8ba aaaa '
    '3803 ef3f d009 0000 60bd d2bf db04 3353 1150 e73f 7d0b 0000 72f0 e53f 0a3d 0cc3 '
    'f071 e73f c3d7 eefe 7b52 f93f'
)
APPENDIX_ANGLES = [
    -0.5591162726995316, -0.7381537301203293, 1.0877742727584383,
    -1.0945791516990426, 1.0625584011991203, 2.1401580135014395,
    0.9691432317102597, -0.2928085327149832, 0.7285238862024487,
    0.6856012344363617, 0.7326587495353192, 1.5826377828902742,
]


def report(**overrides) -> p.StateReport:
    """Return a state report with distinct values in every field."""
    values = {
        'basic_state': 6, 'gait_state': 4, 'policy_state': 0,
        'rpy_deg': (1.5, -2.0, 90.0), 'rpy_vel': (0.01, 0.02, 0.3), 'xyz_acc': (0.1, 0.2, 9.8),
        'pos_world': (1.25, -0.5, 0.75), 'vel_world': (0.2, 0.1, 0.3),
        'vel_body': (0.3, -0.05, 0.4), 'layout': 'natural', 'motion_state': 1, 'battery': 0.82,
        'error_state': 0, 'balance_lost': True, 'zeroed': True, 'first_start': False,
        'voice_enabled': True, 'ultrasound': (1.2, 3.4),
    }
    values.update(overrides)
    return p.StateReport(**values)


def test_appendix_joint_angles():
    """The document's own worked example decodes to its 12 angles."""
    raw = bytes.fromhex(APPENDIX.replace(' ', ''))
    assert len(raw) == 108
    datagram = p.parse_datagram(raw)
    assert datagram.code == p.JOINT_ANGLES
    assert datagram.kind == p.TYPE_COMPLEX
    assert len(datagram.payload) == 96
    assert list(p.decode_joints(datagram.payload)) == APPENDIX_ANGLES


def test_simple_command_bytes():
    """A simple command is code, value, type 0, little-endian."""
    assert p.simple(p.HEARTBEAT) == bytes.fromhex('01000421' '00000000' '00000000')
    assert p.simple(p.KEEP_STEPPING, p.KEEP_STEPPING_ON)[4:8] == b'\xff\xff\xff\xff'
    assert p.signed(struct.unpack('<I', b'\xff\xff\xff\xff')[0]) == -1


def test_speed_commands_clamp_and_carry_one_double():
    """Speed commands are complex, 8-byte payloads, clamped to the document's ranges."""
    for code, limit in p.SPEED_LIMITS.items():
        data = p.speed(code, 99.0)
        assert struct.unpack('<III', data[:12]) == (code, 8, 1)
        assert struct.unpack('<d', data[12:])[0] == limit
        assert struct.unpack('<d', p.speed(code, -99.0)[12:])[0] == -limit
    assert struct.unpack('<d', p.speed(p.SPEED_VX, float('nan'))[12:])[0] == 0.0
    assert struct.unpack('<d', p.speed(p.SPEED_WZ, 0.25)[12:])[0] == 0.25


def test_axis_values_are_clamped_and_signed():
    """Axis values stay in [-32767, 32767] and negative ones are two's complement."""
    assert p.signed(struct.unpack('<III', p.axis(p.AXIS_YAW, -50000))[1]) == -p.AXIS_MAX
    assert struct.unpack('<III', p.axis(p.AXIS_PITCH, 20000))[1] == 20000


@pytest.mark.parametrize('layout,size', [('natural', 208), ('pack4', 204), ('pack1', 201)])
def test_state_layouts_round_trip(layout, size):
    """Each struct layout is recognised by its length and decodes every field."""
    sent = report(layout=layout)
    data = p.encode_state(sent, layout)
    datagram = p.parse_datagram(data)
    assert datagram.code == p.STATE and len(datagram.payload) == size
    got = p.decode_state(datagram.payload)
    assert got == sent


def test_natural_layout_offsets():
    """The natural layout puts rpy at 16, battery at 176 and ultrasound at 192."""
    payload = p.encode_state(report(), 'natural')[12:]
    assert struct.unpack_from('<3d', payload, 16) == (1.5, -2.0, 90.0)
    assert struct.unpack_from('<i', payload, 172)[0] == 1
    assert struct.unpack_from('<d', payload, 176)[0] == pytest.approx(0.82)
    assert struct.unpack_from('<2d', payload, 192) == (1.2, 3.4)


def test_unknown_length_keeps_the_prefix():
    """Other lengths decode through vel_body and leave the tail empty."""
    payload = p.encode_state(report(), 'natural')[12:] + b'\x00' * 16
    got = p.decode_state(payload)
    assert got.layout == 'prefix'
    assert got.vel_body == (0.3, -0.05, 0.4)
    assert got.battery is None and got.ultrasound is None and got.motion_state is None
    packed = p.encode_state(report(), 'pack1')[12:12 + 157]
    assert p.decode_state(packed).pos_world == (1.25, -0.5, 0.75)
    assert p.decode_state(b'\x00' * 100) is None


def test_battery_percent_or_fraction():
    """A battery level above 1 is a percentage."""
    assert p.normalize_battery(82.0) == pytest.approx(0.82)
    assert p.normalize_battery(0.5) == 0.5
    assert p.normalize_battery(150.0) == 1.0
    assert p.normalize_battery(float('nan')) is None


def test_malformed_datagrams_are_rejected():
    """Short heads, unknown types and truncated payloads are dropped."""
    assert p.parse_datagram(b'\x01\x02') is None
    assert p.parse_datagram(struct.pack('<III', p.STATE, 0, 7)) is None
    assert p.parse_datagram(struct.pack('<III', p.STATE, 208, 1) + b'\x00' * 10) is None
    assert p.parse_datagram(struct.pack('<III', p.STATE, 5000, 1)) is None
    status = p.parse_datagram(p.simple(p.SPEAKER_STATUS, 1))
    assert status.kind == p.TYPE_SIMPLE and status.value == 1


def test_state_names():
    """State codes have readable names, unknown codes say so."""
    assert p.name_of(p.BASIC_STATES, 6) == 'standing'
    assert p.name_of(p.GAIT_STATES, 13) == 'rug_high_step'
    assert p.name_of(p.MOTION_STATES, 99) == 'unknown'
    assert p.name_of(p.BASIC_STATES, None) == 'unknown'
    sample = report()
    assert sample.standing and not sample.sitting and sample.stepping


def test_joint_round_trip():
    """Joint arrays encode and decode in the document's joint order."""
    values = [math.radians(i) for i in range(12)]
    data = p.encode_joints(p.JOINT_VELOCITIES, values)
    datagram = p.parse_datagram(data)
    assert datagram.code == p.JOINT_VELOCITIES
    assert list(p.decode_joints(datagram.payload)) == values
    assert p.decode_joints(b'\x00' * 40) is None
    # the document's order, with the joint names of the official model (lite3_description)
    assert p.JOINT_NAMES[:3] == ('FL_HipX_joint', 'FL_HipY_joint', 'FL_Knee_joint')
    assert p.JOINT_NAMES[-1] == 'HR_Knee_joint' and len(p.JOINT_NAMES) == 12


def test_fast_imu_round_trip():
    """The 200 Hz IMU datagram of Deep Robotics' bridge: uint32 timestamp and 9 floats."""
    report = p.ImuReport(7, (1.5, -2.0, 90.0), (0.1, 0.2, 0.3), (0.0, 0.5, 9.75))
    datagram = p.parse_datagram(p.encode_imu(report))
    assert (datagram.code, datagram.kind, datagram.value) == (p.IMU_FAST, p.TYPE_COMPLEX, 40)
    decoded = p.decode_imu(datagram.payload)
    assert decoded.timestamp == 7
    assert decoded.rpy_deg == (1.5, -2.0, 90.0)
    assert decoded.xyz_acc == (0.0, 0.5, 9.75)
    assert p.decode_imu(datagram.payload[:39]) is None
