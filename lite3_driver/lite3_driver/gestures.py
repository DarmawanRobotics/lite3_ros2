"""
Gestures of the Lite3: posture keyframes streamed in pose mode, and the hello action.

The runner gets its I/O injected (send, state, clock, sleep), so it is tested with a simulated
motion host. Pose-mode axis commands must keep coming, or the robot stands normally again
after one second (document 1.2.3.1); hello needs the sitting state (document 1.2.6).
"""

from dataclasses import dataclass
import threading
from typing import Callable, List, Optional, Tuple

from lite3_driver import protocol as p

AXIS_GESTURES = ('nod', 'bow', 'look_left', 'look_right')
GESTURES = ('hello',) + AXIS_GESTURES


@dataclass(frozen=True)
class Keyframe:
    """Posture held for `duration` seconds; axis values in [-32767, 32767]."""

    duration: float
    pitch: int = 0        # positive lowers the head
    yaw: int = 0          # positive turns the body right
    roll: int = 0         # positive rolls right
    height: int = 0       # positive raises the body

    def datagrams(self) -> List[bytes]:
        """Return the four axis commands of this posture."""
        return [
            p.axis(p.AXIS_ROLL, self.roll),
            p.axis(p.AXIS_PITCH, self.pitch),
            p.axis(p.AXIS_HEIGHT, self.height),
            p.axis(p.AXIS_YAW, self.yaw),
        ]


NEUTRAL = Keyframe(0.0)


@dataclass
class GestureConfig:
    """Shape and timing of the gestures."""

    rate_hz: float = 20.0
    nod_pitch: int = 22000
    nod_cycles: int = 2
    nod_period_sec: float = 0.9
    bow_pitch: int = 26000
    bow_hold_sec: float = 1.5
    look_yaw: int = 16000
    look_hold_sec: float = 1.5
    settle_sec: float = 0.5
    use_manual_mode: bool = True
    sit_timeout_sec: float = 8.0
    hello_start_timeout_sec: float = 3.0
    hello_timeout_sec: float = 15.0
    stand_timeout_sec: float = 8.0
    quiet_sec: float = 1.0


def keyframes(name: str, config: GestureConfig) -> List[Keyframe]:
    """Return the keyframes of a pose-mode gesture."""
    if name == 'nod':
        half = config.nod_period_sec / 2.0
        frames = []
        for _ in range(max(1, int(config.nod_cycles))):
            frames += [Keyframe(half, pitch=config.nod_pitch), Keyframe(half)]
        return frames
    if name == 'bow':
        return [Keyframe(config.bow_hold_sec, pitch=config.bow_pitch), Keyframe(config.settle_sec)]
    if name == 'look_left':
        return [Keyframe(config.look_hold_sec, yaw=-config.look_yaw), Keyframe(config.settle_sec)]
    if name == 'look_right':
        return [Keyframe(config.look_hold_sec, yaw=config.look_yaw), Keyframe(config.settle_sec)]
    raise KeyError(f'not a pose-mode gesture: {name}')


class GestureRunner:
    """Runs one gesture at a time to its end and restores the robot's mode afterwards."""

    def __init__(
        self,
        config: GestureConfig,
        send: Callable[[bytes], bool],
        state: Callable[[], Optional[p.StateReport]],
        connected: Callable[[], bool],
        moving: Callable[[], bool],
        clock: Callable[[], float],
        sleep: Callable[[float], None],
    ):
        """Keep the injected I/O."""
        self.config = config
        self._send = send
        self._state = state
        self._connected = connected
        self._moving = moving
        self._clock = clock
        self._sleep = sleep
        self._abort = threading.Event()
        self._busy = threading.Lock()
        self.active = ''

    def abort(self):
        """Ask the running gesture to stop at its next step."""
        self._abort.set()

    def check(self, name: str) -> Optional[str]:
        """Return why the gesture cannot run now, or None when it can."""
        if name not in GESTURES:
            return f'unknown gesture "{name}"'
        if not self._connected():
            return 'no state from the motion host'
        report = self._state()
        if report is None:
            return 'no state from the motion host yet'
        if self._moving() or report.stepping:
            return 'the robot is walking'
        state = p.name_of(p.BASIC_STATES, report.basic_state)
        if name == 'hello':
            if not (report.standing or report.sitting):
                return f'the robot is busy ({state})'
        elif not report.standing:
            return f'stand up first (the robot is {state})'
        return None

    def run(self, name: str, control_mode: str) -> Tuple[bool, str]:
        """Run a gesture to its end; return (success, message)."""
        if not self._busy.acquire(blocking=False):
            return False, f'busy with {self.active or "a gesture"}'
        try:
            reason = self.check(name)
            if reason:
                return False, reason
            self._abort.clear()
            self.active = name
            if name == 'hello':
                return self._hello(control_mode)
            return self._pose_gesture(name, control_mode)
        finally:
            self.active = ''
            self._busy.release()

    def _pose_gesture(self, name: str, control_mode: str) -> Tuple[bool, str]:
        """Stream the keyframes in pose mode, then back to move mode."""
        c = self.config
        period = 1.0 / max(1.0, c.rate_hz)
        if c.use_manual_mode:
            self._send(p.simple(p.MANUAL_MODE))
        self._send(p.simple(p.POSE_MODE))
        aborted = False
        for frame in keyframes(name, c):
            end = self._clock() + frame.duration
            while self._clock() < end:
                if self._abort.is_set():
                    aborted = True
                    break
                for datagram in frame.datagrams():
                    self._send(datagram)
                self._sleep(period)
            if aborted:
                break
        for datagram in NEUTRAL.datagrams():
            self._send(datagram)
        self._send(p.simple(p.MOVE_MODE))
        if control_mode == 'navigation':
            self._send(p.simple(p.NAVIGATION_MODE))
        return (False, f'{name} stopped early') if aborted else (True, f'{name} done')

    def _wait(self, predicate, timeout: float, abortable: bool = True) -> bool:
        """Wait until the state report satisfies the predicate; False on timeout or abort."""
        end = self._clock() + timeout
        while True:
            report = self._state()
            if report is not None and predicate(report):
                return True
            if self._clock() >= end or (abortable and self._abort.is_set()):
                return False
            self._sleep(0.05)

    def _hello(self, control_mode: str) -> Tuple[bool, str]:
        """Sit when standing, wave, and stand up again."""
        c = self.config
        was_standing = self._state().standing
        if was_standing:
            self._send(p.simple(p.STAND_SIT))
            if not self._wait(lambda r: r.sitting, c.sit_timeout_sec):
                return self._restore(False, 'did not sit down for hello', was_standing,
                                     control_mode)
        self._send(p.simple(p.ACTION_HELLO))
        if not self._wait(lambda r: r.basic_state == p.STATE_HELLO, c.hello_start_timeout_sec):
            return self._restore(False, 'hello did not start', was_standing, control_mode)
        if not self._wait(lambda r: r.basic_state != p.STATE_HELLO, c.hello_timeout_sec):
            return self._restore(False, 'hello did not finish', was_standing, control_mode)
        return self._restore(True, 'hello done', was_standing, control_mode)

    def _restore(self, ok: bool, message: str, was_standing: bool, control_mode: str):
        """Stand up again when the robot stood before, then restore the control mode."""
        c = self.config
        report = self._state()
        if was_standing and (report is None or not report.standing):
            settled = self._wait(lambda r: r.sitting or r.standing, c.sit_timeout_sec, False)
            if settled and self._state().sitting:
                self._send(p.simple(p.STAND_SIT))
            if not self._wait(lambda r: r.standing, c.stand_timeout_sec, abortable=False):
                return False, f'{message}; did not stand up again'
        if control_mode == 'navigation':
            self._send(p.simple(p.NAVIGATION_MODE))
        return ok, message
