"""Velocity streaming policy: which speed commands go to the motion host, and when."""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from lite3_driver import protocol as p

EPSILON = 1e-3
NAVIGATION_DATAGRAM = p.simple(p.NAVIGATION_MODE)
WZ_SIGN = -1.0  # REP-103 counter-clockwise -> the motion host's positive = turn right


@dataclass
class VelocityConfig:
    """Limits and timing of the velocity stream."""

    max_vx: float = 1.0
    max_vy: float = 0.5
    max_wz: float = 1.5
    cmd_timeout_sec: float = 0.5
    stop_burst_sec: float = 0.5
    auto_navigation_mode: bool = True
    mode_repeats: int = 3
    mode_repeat_sec: float = 0.2
    invert_vy: bool = False
    invert_wz: bool = False


def clamp(value: float, limit: float) -> float:
    """Clamp to [-limit, limit]; a non-finite value becomes 0."""
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def is_motion(cmd: Tuple[float, float, float]) -> bool:
    """Return True when any component asks for motion."""
    return any(abs(v) > EPSILON for v in cmd)


class VelocityPolicy:
    """Streams cmd_vel while it is fresh, then zeros for a moment, then nothing."""

    def __init__(self, config: VelocityConfig):
        """Start idle: nothing is sent before the first command."""
        self.config = config
        self._cmd = (0.0, 0.0, 0.0)
        self._stamp: Optional[float] = None
        self._last_motion: Optional[float] = None
        self._mode_sent: Optional[float] = None
        self._mode_first: Optional[float] = None
        self._mode_count = 0
        self._manual_hold = False
        self._hold: Optional[str] = None  # set by emergency_stop, sit_down, save_data
        self._armed = True  # False after a hold until cmd_vel was zero or quiet once
        self._burst_until: Optional[float] = None

    def limits(self) -> Tuple[float, float, float]:
        """Return the effective limits: the configured ones within the protocol's ranges."""
        c = self.config
        return (
            min(abs(c.max_vx), p.SPEED_LIMITS[p.SPEED_VX]),
            min(abs(c.max_vy), p.SPEED_LIMITS[p.SPEED_VY]),
            min(abs(c.max_wz), p.SPEED_LIMITS[p.SPEED_WZ]),
        )

    def command(self, vx: float, vy: float, wz: float, now: float):
        """Take a new cmd_vel (base_link, REP-103); a non-finite component makes it a stop."""
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            vx = vy = wz = 0.0
        lx, ly, lw = self.limits()
        self._cmd = (clamp(vx, lx), clamp(vy, ly), clamp(wz, lw))
        self._stamp = now
        if not is_motion(self._cmd):
            if self._hold is None:
                self._armed = True  # a zero command: what follows is no stale stream
        elif self.driving:
            self._last_motion = now

    @property
    def driving(self) -> bool:
        """Return True when a motion command would be sent: no hold, armed."""
        return self._hold is None and self._armed

    @property
    def hold_reason(self) -> Optional[str]:
        """Return the service that holds cmd_vel, or None."""
        return self._hold

    @property
    def gate(self) -> str:
        """Return why motion is or is not sent: ok, hold:<reason> or waiting for a zero cmd_vel."""
        if self._hold is not None:
            return f'hold:{self._hold}'
        return 'ok' if self._armed else 'waiting for zero or quiet cmd_vel'

    def hold(self, reason: str, now: float):
        """
        Stop and ignore cmd_vel until release() and a zero or quiet cmd_vel after it.

        Zeros go out for stop_burst_sec, then nothing: a stream that keeps running after an
        emergency stop or a sit down never drives the robot.
        """
        self._hold = reason
        self._armed = False
        self._cmd = (0.0, 0.0, 0.0)
        self._burst_until = now + self.config.stop_burst_sec

    def release(self):
        """End a hold; motion is sent again once cmd_vel has been zero or quiet."""
        self._hold = None

    def hold_manual(self):
        """Send no navigation mode until the stream has gone idle (a manual takeover)."""
        self._manual_hold = True

    @property
    def current(self) -> Tuple[float, float, float]:
        """Return the last command after clamping."""
        return self._cmd

    def fresh(self, now: float) -> bool:
        """Return True while the last command is younger than the timeout."""
        return self._stamp is not None and now - self._stamp <= self.config.cmd_timeout_sec

    def moving(self, now: float) -> bool:
        """Return True while a fresh command asks for motion (sent or not)."""
        return self.fresh(now) and is_motion(self._cmd)

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a non-zero command arrived within the window."""
        return self._last_motion is not None and now - self._last_motion <= window

    def sent_command(self, now: float) -> Tuple[float, float, float]:
        """Return what the stream sends now: the command, zeros, or zeros when silent."""
        return self._cmd if self.fresh(now) and self.driving else (0.0, 0.0, 0.0)

    def tick(self, now: float) -> List[bytes]:
        """Return the datagrams for this period of the command timer."""
        c = self.config
        if self._stamp is None:
            return []
        age = now - self._stamp
        if not self.driving:
            if self._hold is None and age > c.cmd_timeout_sec + c.stop_burst_sec:
                self._armed = True  # cmd_vel went quiet after the release
            elif self._burst_until is not None and now <= self._burst_until:
                return self.speeds(0.0, 0.0, 0.0)
            else:
                return []
        if age <= c.cmd_timeout_sec:
            out = []
            if self._navigation_due(now):
                out.append(NAVIGATION_DATAGRAM)
                if self._mode_count == 0:
                    self._mode_first = now
                self._mode_sent = now
                self._mode_count += 1
            return out + self.speeds(*self._cmd)
        if age <= c.cmd_timeout_sec + c.stop_burst_sec:
            return self.speeds(0.0, 0.0, 0.0)
        # idle: the next motion asks for navigation mode again
        self._mode_sent = None
        self._mode_first = None
        self._mode_count = 0
        self._manual_hold = False
        return []

    def _navigation_due(self, now: float) -> bool:
        """
        Return True when navigation mode goes out in this period.

        The first time when motion starts after a quiet stream; the other mode_repeats - 1
        follow on their schedule right after it, zeros or not (a UDP datagram may get lost);
        then never again until cmd_vel has gone quiet. The state report does not say which
        mode the robot is in, so this is how an operator who switches it to manual on the
        remote keeps it while cmd_vel streams.
        """
        c = self.config
        if not c.auto_navigation_mode or self._manual_hold or self._mode_count >= c.mode_repeats:
            return False
        if self._mode_count == 0:
            return is_motion(self._cmd)
        return (now - self._mode_sent >= c.mode_repeat_sec
                and now - self._mode_first <= c.mode_repeats * c.mode_repeat_sec)

    def speeds(self, vx: float, vy: float, wz: float) -> List[bytes]:
        """
        Encode the three speed commands in the motion host's signs.

        x forward and y left are sent as they are. The motion host turns right for a positive
        angular speed, as for its axis commands: Deep Robotics' own bridge (Lite3_ROS,
        transfer/Jetson2Motion.cpp) sends -angular.z, so the counter-clockwise +z of REP-103 is
        negated here. invert_vy / invert_wz flip a sign that turns out wrong on a robot.
        """
        c = self.config
        return [
            p.speed(p.SPEED_VX, vx),
            p.speed(p.SPEED_VY, -vy if c.invert_vy else vy),
            p.speed(p.SPEED_WZ, wz * WZ_SIGN * (-1.0 if c.invert_wz else 1.0)),
        ]
