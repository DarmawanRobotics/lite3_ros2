"""Gestures against the simulated motion host."""

from fake_robot import FakeClock, FakeRobot
from lite3_driver import protocol as p
from lite3_driver.gestures import GestureConfig, GestureRunner, keyframes
import pytest


def runner(robot, moving=False, connected=True, config=None):
    """Return a runner wired to the fake robot and a fake clock."""
    clock = FakeClock(robot)
    return GestureRunner(
        config or GestureConfig(),
        send=robot.send,
        state=robot.state,
        connected=lambda: connected,
        moving=lambda: moving,
        clock=clock.time,
        sleep=clock.sleep,
    )


@pytest.mark.parametrize('name', ['nod', 'bow', 'look_left', 'look_right'])
def test_pose_gestures_stream_axes_in_pose_mode(name):
    """Pose gestures go manual, pose, stream axes beyond the dead zone, then move again."""
    robot = FakeRobot()
    ok, message = runner(robot).run(name, control_mode='navigation')
    assert ok, message
    sent = robot.codes()
    assert sent[:2] == [p.MANUAL_MODE, p.POSE_MODE]
    assert sent[-2:] == [p.MOVE_MODE, p.NAVIGATION_MODE]
    axes = [p.parse_datagram(d) for d in robot.sent if p.parse_datagram(d).code in (
        p.AXIS_PITCH, p.AXIS_YAW)]
    peak = max(abs(p.signed(d.value)) for d in axes)
    assert peak > max(p.AXIS_DEAD_ZONES[p.AXIS_PITCH], p.AXIS_DEAD_ZONES[p.AXIS_YAW])
    assert peak <= p.AXIS_MAX
    assert robot.mode == 'move' and robot.control == 'navigation'


def test_look_directions():
    """Positive yaw rotates right, so looking left sends a negative yaw."""
    config = GestureConfig()
    assert keyframes('look_left', config)[0].yaw < 0 < keyframes('look_right', config)[0].yaw
    assert keyframes('bow', config)[0].pitch > 0  # positive lowers the head


def test_gestures_are_refused_when_unsafe():
    """No gesture while walking, without state, or (pose gestures) while sitting."""
    assert 'walking' in runner(FakeRobot(), moving=True).run('nod', 'navigation')[1]
    assert 'motion host' in runner(FakeRobot(), connected=False).run('nod', 'manual')[1]
    assert 'stand up first' in runner(FakeRobot(p.STATE_SITTING)).run('bow', 'manual')[1]
    assert 'unknown' in runner(FakeRobot()).run('backflip', 'manual')[1]
    assert FakeRobot().sent == []


def test_hello_sits_waves_and_stands_up_again():
    """Hello needs the sitting state: sit, wave, stand up, back to navigation mode."""
    robot = FakeRobot(p.STATE_STANDING)
    ok, message = runner(robot).run('hello', control_mode='navigation')
    assert ok, message
    assert robot.codes() == [p.STAND_SIT, p.ACTION_HELLO, p.STAND_SIT, p.NAVIGATION_MODE]
    assert robot.report.basic_state == p.STATE_STANDING


def test_hello_from_sitting_stays_seated():
    """A sitting robot waves without standing up."""
    robot = FakeRobot(p.STATE_SITTING)
    ok, _ = runner(robot).run('hello', control_mode='manual')
    assert ok
    assert robot.codes() == [p.ACTION_HELLO]
    assert robot.report.basic_state == p.STATE_SITTING


def test_hello_that_never_starts_still_stands_up():
    """When the robot ignores hello, the runner reports it and stands the robot up again."""
    robot = FakeRobot(p.STATE_STANDING, ignore={p.ACTION_HELLO})
    ok, message = runner(robot).run('hello', control_mode='manual')
    assert not ok and 'did not start' in message
    assert robot.report.basic_state == p.STATE_STANDING


def test_abort_ends_a_gesture_in_move_mode():
    """An abort (somebody drives) stops the keyframes and leaves pose mode."""
    robot = FakeRobot()
    gesture = runner(robot)
    calls = {'n': 0}
    original = robot.send

    def send(data):
        calls['n'] += 1
        if calls['n'] == 20:
            gesture.abort()
        return original(data)

    gesture._send = send
    ok, message = gesture.run('bow', control_mode='manual')
    assert not ok and 'stopped early' in message
    assert robot.mode == 'move'
