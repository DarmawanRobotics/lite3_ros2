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
        self._mode_count = 0
        self._manual_hold = False

    def limits(self) -> Tuple[float, float, float]:
        """Return the effective limits: the configured ones within the protocol's ranges."""
        c = self.config
        return (
            min(abs(c.max_vx), p.SPEED_LIMITS[p.SPEED_VX]),
            min(abs(c.max_vy), p.SPEED_LIMITS[p.SPEED_VY]),
            min(abs(c.max_wz), p.SPEED_LIMITS[p.SPEED_WZ]),
        )

    def command(self, vx: float, vy: float, wz: float, now: float):
        """Take a new cmd_vel (base_link, REP-103)."""
        lx, ly, lw = self.limits()
        self._cmd = (clamp(vx, lx), clamp(vy, ly), clamp(wz, lw))
        self._stamp = now
        if is_motion(self._cmd):
            self._last_motion = now

    def hold_manual(self):
        """Send no navigation mode until the stream has gone idle (a manual takeover)."""
        self._manual_hold = True

    def stop(self, now: float):
        """Drop the current command; zeros are streamed until the stream runs out."""
        self._cmd = (0.0, 0.0, 0.0)
        self._stamp = now

    @property
    def current(self) -> Tuple[float, float, float]:
        """Return the last command after clamping."""
        return self._cmd

    def fresh(self, now: float) -> bool:
        """Return True while the last command is younger than the timeout."""
        return self._stamp is not None and now - self._stamp <= self.config.cmd_timeout_sec

    def moving(self, now: float) -> bool:
        """Return True while a fresh command asks for motion."""
        return self.fresh(now) and is_motion(self._cmd)

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a non-zero command arrived within the window."""
        return self._last_motion is not None and now - self._last_motion <= window

    def sent_command(self, now: float) -> Tuple[float, float, float]:
        """Return what the stream sends now: the command, zeros, or zeros when silent."""
        return self._cmd if self.fresh(now) else (0.0, 0.0, 0.0)

    def tick(self, now: float) -> List[bytes]:
        """Return the datagrams for this period of the command timer."""
        c = self.config
        if self._stamp is None:
            return []
        age = now - self._stamp
        if age <= c.cmd_timeout_sec:
            out = []
            # Navigation mode goes out when motion starts after idle, mode_repeats times (UDP
            # may drop one), and never again while the stream lasts: an operator who switches
            # the robot to manual on the remote keeps it until the stream goes idle.
            if (c.auto_navigation_mode and is_motion(self._cmd) and not self._manual_hold
                    and self._mode_count < c.mode_repeats
                    and (self._mode_sent is None or now - self._mode_sent >= c.mode_repeat_sec)):
                out.append(NAVIGATION_DATAGRAM)
                self._mode_sent = now
                self._mode_count += 1
            return out + self.speeds(*self._cmd)
        if age <= c.cmd_timeout_sec + c.stop_burst_sec:
            return self.speeds(0.0, 0.0, 0.0)
        # idle: the next motion asks for navigation mode again
        self._mode_sent = None
        self._mode_count = 0
        self._manual_hold = False
        return []

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
