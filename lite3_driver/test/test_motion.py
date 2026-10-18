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
    """A new motion asks for navigation mode, then streams the speeds in the robot's signs."""
    policy = VelocityPolicy(VelocityConfig())
    policy.command(0.3, 0.1, 0.5, now=0.0)
    first = policy.tick(0.0)
    assert codes(first) == [p.NAVIGATION_MODE, p.SPEED_VX, p.SPEED_VY, p.SPEED_WZ]
    # +angular.z (counter-clockwise) goes out negative: the motion host turns right for positive
    assert speeds(first) == {p.SPEED_VX: 0.3, p.SPEED_VY: 0.1, p.SPEED_WZ: -0.5}
    assert codes(policy.tick(0.05)) == [p.SPEED_VX, p.SPEED_VY, p.SPEED_WZ]


def test_navigation_mode_only_when_motion_starts():
    """Navigation mode goes out mode_repeats times as motion starts, never while it streams."""
    policy = VelocityPolicy(VelocityConfig(mode_repeats=3, mode_repeat_sec=0.2))
    sent = []
    for i in range(200):  # 10 s of cmd_vel at 20 Hz
        now = i * 0.05 + 0.01
        policy.command(0.3, 0.0, 0.0, now=now)
        if p.NAVIGATION_MODE in codes(policy.tick(now)):
            sent.append(now)
    # an operator who switches the robot to manual on the remote keeps it while cmd_vel streams
    assert len(sent) == 3 and sent[-1] < 0.6


def test_a_pause_with_zeros_is_the_same_stream():
    """Zeros keep the stream alive: motion after them does not ask for navigation mode again."""
    policy = VelocityPolicy(VelocityConfig(mode_repeats=1))
    policy.command(0.3, 0.0, 0.0, now=0.0)
    assert p.NAVIGATION_MODE in codes(policy.tick(0.0))
    for i in range(1, 60):
        policy.command(0.0, 0.0, 0.0, now=i * 0.1)
        policy.tick(i * 0.1)
    policy.command(0.3, 0.0, 0.0, now=6.0)
    assert p.NAVIGATION_MODE not in codes(policy.tick(6.0))


def test_manual_mode_holds_until_the_stream_goes_idle():
    """After manual_mode no navigation mode goes out until cmd_vel has gone quiet."""
    policy = VelocityPolicy(VelocityConfig(cmd_timeout_sec=0.5, stop_burst_sec=0.5))
    policy.command(0.3, 0.0, 0.0, now=0.0)
    assert p.NAVIGATION_MODE in codes(policy.tick(0.0))
    policy.hold_manual()
    policy.command(0.3, 0.0, 0.0, now=0.25)
    assert p.NAVIGATION_MODE not in codes(policy.tick(0.25))  # the remaining repeats are dropped
    assert speeds(policy.tick(0.8))[p.SPEED_VX] == 0.0
    assert policy.tick(1.3) == []  # idle
    policy.command(0.3, 0.0, 0.0, now=2.0)
    assert codes(policy.tick(2.0))[0] == p.NAVIGATION_MODE


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
    """Commands are clamped to the configured and protocol limits; NaN anywhere is a stop."""
    policy = VelocityPolicy(VelocityConfig(max_vx=0.6, max_vy=2.0, max_wz=1.0))
    policy.command(5.0, -5.0, 0.0, now=0.0)
    assert policy.current == (0.6, -0.5, 0.0)
    assert speeds(policy.tick(0.0)) == {p.SPEED_VX: 0.6, p.SPEED_VY: -0.5, p.SPEED_WZ: 0.0}
    policy.command(0.0, 0.0, -3.0, now=1.0)
    assert speeds(policy.tick(1.0))[p.SPEED_WZ] == 1.0  # clockwise, clamped to 1.0
    policy.command(0.5, float('nan'), 0.2, now=2.0)
    assert policy.current == (0.0, 0.0, 0.0)  # not x 0.5 with y dropped
    assert speeds(policy.tick(2.0)) == {p.SPEED_VX: 0.0, p.SPEED_VY: 0.0, p.SPEED_WZ: 0.0}


def test_inversions_for_a_robot_that_disagrees():
    """invert_vy / invert_wz flip what is sent, not the command itself."""
    policy = VelocityPolicy(VelocityConfig(invert_vy=True, invert_wz=True))
    policy.command(0.1, 0.2, 0.3, now=0.0)
    sent = speeds(policy.tick(0.0))
    assert sent[p.SPEED_VY] == -0.2 and sent[p.SPEED_WZ] == 0.3  # wz back to REP-103's sign
    assert policy.current == (0.1, 0.2, 0.3)


def test_hold_and_motion_memory():
    """hold() zeroes at once; recently_moved remembers the last motion that was sent."""
    policy = VelocityPolicy(VelocityConfig())
    policy.command(0.4, 0.0, 0.0, now=0.0)
    assert policy.moving(0.1) and policy.recently_moved(0.5, 1.0)
    policy.hold('emergency_stop', 0.2)
    assert not policy.moving(0.2)
    assert speeds(policy.tick(0.2))[p.SPEED_VX] == 0.0
    policy.command(0.4, 0.0, 0.0, now=0.3)  # held: not sent, not remembered
    assert not policy.recently_moved(1.5, 1.0)
    assert policy.sent_command(0.3) == (0.0, 0.0, 0.0)


def test_a_hold_ignores_a_running_stream_until_release_and_a_zero():
    """After emergency_stop or sit_down a stream that keeps running never drives the robot."""
    policy = VelocityPolicy(VelocityConfig(cmd_timeout_sec=0.5, stop_burst_sec=0.5))
    policy.command(0.3, 0.0, 0.0, now=0.0)
    assert speeds(policy.tick(0.0))[p.SPEED_VX] == 0.3
    policy.hold('emergency_stop', 0.1)
    assert policy.gate == 'hold:emergency_stop'
    sent = []
    for i in range(2, 60):  # the stream goes on for 6 s
        now = i * 0.1
        policy.command(0.3, 0.0, 0.0, now=now)
        sent += policy.tick(now)
    assert set(speeds(sent).values()) == {0.0}  # only the stop burst
    assert codes(policy.tick(5.95)) == []
    policy.release()  # stand_up
    policy.command(0.3, 0.0, 0.0, now=6.0)
    assert policy.tick(6.0) == []  # still the old stream: no drive-off
    assert policy.gate.startswith('waiting')
    policy.command(0.0, 0.0, 0.0, now=6.1)  # the commander stopped once
    assert policy.gate == 'ok'
    policy.command(0.3, 0.0, 0.0, now=6.2)
    first = policy.tick(6.2)
    assert speeds(first)[p.SPEED_VX] == 0.3
    assert p.NAVIGATION_MODE not in codes(first)  # the same stream: no navigation mode again


def test_a_quiet_stream_after_the_release_also_rearms():
    """cmd_vel that went quiet after stand_up counts like a zero."""
    policy = VelocityPolicy(VelocityConfig(cmd_timeout_sec=0.5, stop_burst_sec=0.5))
    policy.command(0.3, 0.0, 0.0, now=0.0)
    policy.hold('sit_down', 0.0)
    policy.release()
    assert set(speeds(policy.tick(0.2)).values()) == {0.0}  # the stop burst
    assert policy.tick(1.2) == []  # quiet: armed again
    policy.command(0.2, 0.0, 0.0, now=2.0)
    assert speeds(policy.tick(2.0))[p.SPEED_VX] == 0.2


@pytest.mark.parametrize('auto', [True, False])
def test_auto_navigation_mode_switch(auto):
    """auto_navigation_mode false never sends the navigation mode command."""
    policy = VelocityPolicy(VelocityConfig(auto_navigation_mode=auto))
    policy.command(0.2, 0.0, 0.0, now=0.0)
    assert (p.NAVIGATION_MODE in codes(policy.tick(0.0))) is auto


def run_stream(policy, until, moving, events=None, step=0.05):
    """Drive one never-quiet stream at 20 Hz; return when navigation mode went out."""
    events = events or {}
    sent = []
    for i in range(int(round(until / step))):
        now = round(i * step, 2)
        policy.command(0.3 if moving(now) else 0.0, 0.0, 0.0, now=now)
        if now in events:
            events[now](now)
        if p.NAVIGATION_MODE in codes(policy.tick(now)):
            sent.append(now)
    return sent


def test_the_repeats_follow_the_first_one_not_the_next_motion():
    """One short motion, then a minute of zeros: the remaining repeats do not wait for motion."""
    policy = VelocityPolicy(VelocityConfig(mode_repeats=3, mode_repeat_sec=0.2))
    sent = run_stream(policy, 62.0, lambda t: t < 0.01 or t >= 60.0)
    assert sent and max(sent) < 1.0  # nothing at 60 s, when the same stream moves again


def test_a_hold_does_not_start_a_new_stream():
    """sit_down and stand_up in a stream that never went quiet: no navigation mode after them."""
    policy = VelocityPolicy(VelocityConfig())
    events = {3.0: lambda t: policy.hold('sit_down', t), 5.0: lambda t: policy.release()}
    sent = run_stream(policy, 10.0, lambda t: t < 2.0 or t > 6.0, events)
    assert sent and max(sent) < 1.0


def test_manual_mode_survives_a_hold():
    """manual_mode, then sit_down and stand_up while cmd_vel never went quiet: still manual."""
    policy = VelocityPolicy(VelocityConfig())
    events = {1.0: lambda t: policy.hold_manual(), 3.0: lambda t: policy.hold('sit_down', t),
              5.0: lambda t: policy.release()}
    sent = run_stream(policy, 10.0, lambda t: t < 2.0 or t > 6.0, events)
    assert sent and max(sent) < 1.0
