"""Velocity streaming policy: fresh commands, watchdog, navigation mode, limits."""

import struct

from lite3_driver import protocol as p
from lite3_driver.motion import VelocityConfig, VelocityPolicy
import pytest


def speeds(datagrams) -> dict:
    """Return {code: value} of the speed commands in a tick."""
    out = {}
    for data in datagrams:
        datagram = p.parse_datagram(data)
        if datagram.code in p.SPEED_LIMITS:
            out[datagram.code] = struct.unpack('<d', datagram.payload)[0]
    return out


def codes(datagrams) -> list:
    """Return the command codes of a tick."""
    return [p.parse_datagram(d).code for d in datagrams]


def test_silent_before_the_first_command():
    """Nothing is sent before anyone commands the robot."""
    assert VelocityPolicy(VelocityConfig()).tick(0.0) == []


def test_fresh_command_streams_with_navigation_mode_first():
    """A new motion asks for navigation mode, then streams the three speeds as-is (REP-103)."""
    policy = VelocityPolicy(VelocityConfig())
    policy.command(0.3, 0.1, 0.5, now=0.0)
    first = policy.tick(0.0)
    assert codes(first) == [p.NAVIGATION_MODE, p.SPEED_VX, p.SPEED_VY, p.SPEED_WZ]
    assert speeds(first) == {p.SPEED_VX: 0.3, p.SPEED_VY: 0.1, p.SPEED_WZ: 0.5}
    assert codes(policy.tick(0.05)) == [p.SPEED_VX, p.SPEED_VY, p.SPEED_WZ]
    policy.command(0.3, 0.0, 0.0, now=2.0)
    assert p.NAVIGATION_MODE in codes(policy.tick(2.0))  # refreshed while moving


def test_watchdog_sends_zeros_then_goes_silent():
    """A stale command turns into zeros for the stop burst, then into silence."""
    policy = VelocityPolicy(VelocityConfig(cmd_timeout_sec=0.5, stop_burst_sec=0.5))
    policy.command(0.5, 0.0, 0.0, now=0.0)
    assert speeds(policy.tick(0.4))[p.SPEED_VX] == 0.5
    assert speeds(policy.tick(0.6)) == {p.SPEED_VX: 0.0, p.SPEED_VY: 0.0, p.SPEED_WZ: 0.0}
    assert policy.tick(1.1) == []
    policy.command(0.2, 0.0, 0.0, now=5.0)
    assert codes(policy.tick(5.0))[0] == p.NAVIGATION_MODE  # motion after idle asks again


def test_zero_commands_do_not_switch_modes():
    """Zeros (e.g. the e-stop hold) stream without touching the control mode."""
    policy = VelocityPolicy(VelocityConfig())
    policy.command(0.0, 0.0, 0.0, now=0.0)
    assert p.NAVIGATION_MODE not in codes(policy.tick(0.0))
    assert not policy.moving(0.0)


def test_limits_and_non_finite_values():
    """Commands are clamped to the configured and protocol limits; NaN counts as 0."""
    policy = VelocityPolicy(VelocityConfig(max_vx=0.6, max_vy=2.0, max_wz=1.0))
    policy.command(5.0, -5.0, float('nan'), now=0.0)
    assert policy.current == (0.6, -0.5, 0.0)
    assert speeds(policy.tick(0.0)) == {p.SPEED_VX: 0.6, p.SPEED_VY: -0.5, p.SPEED_WZ: 0.0}


def test_inversions_for_a_robot_that_disagrees():
    """invert_vy / invert_wz flip what is sent, not the command itself."""
    policy = VelocityPolicy(VelocityConfig(invert_vy=True, invert_wz=True))
    policy.command(0.1, 0.2, 0.3, now=0.0)
    sent = speeds(policy.tick(0.0))
    assert sent[p.SPEED_VY] == -0.2 and sent[p.SPEED_WZ] == -0.3
    assert policy.current == (0.1, 0.2, 0.3)


def test_stop_and_motion_memory():
    """stop() zeroes at once; recently_moved remembers the last motion."""
    policy = VelocityPolicy(VelocityConfig())
    policy.command(0.4, 0.0, 0.0, now=0.0)
    assert policy.moving(0.1) and policy.recently_moved(0.5, 1.0)
    policy.stop(0.2)
    assert not policy.moving(0.2)
    assert speeds(policy.tick(0.2))[p.SPEED_VX] == 0.0
    assert not policy.recently_moved(1.5, 1.0)
    assert policy.sent_command(5.0) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('auto', [True, False])
def test_auto_navigation_mode_switch(auto):
    """auto_navigation_mode false never sends the navigation mode command."""
    policy = VelocityPolicy(VelocityConfig(auto_navigation_mode=auto))
    policy.command(0.2, 0.0, 0.0, now=0.0)
    assert (p.NAVIGATION_MODE in codes(policy.tick(0.0))) is auto
