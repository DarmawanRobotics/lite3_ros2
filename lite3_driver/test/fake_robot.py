"""A simulated Lite3 motion host for the tests: reacts to commands like the real one."""

from dataclasses import replace

from lite3_driver import protocol as p


class FakeClock:
    """Time that only moves when the code under test sleeps."""

    def __init__(self, robot=None):
        """Start at zero; the robot (if any) advances with the clock."""
        self.now = 0.0
        self.robot = robot

    def time(self) -> float:
        """Return the current fake time."""
        return self.now

    def sleep(self, seconds: float):
        """Advance the time and let the robot catch up."""
        self.now += max(seconds, 1e-3)
        if self.robot is not None:
            self.robot.advance(self.now)


class FakeRobot:
    """State machine of the motion host: stand/sit toggle, hello, modes and axis commands."""

    TRANSITION_SEC = 1.5
    HELLO_SEC = 3.0

    def __init__(self, basic_state=p.STATE_STANDING, ignore=()):
        """Start in a basic state; commands in `ignore` have no effect."""
        self.report = p.StateReport(
            basic_state=basic_state, gait_state=0, policy_state=0,
            rpy_deg=(0.0, 0.0, 0.0), rpy_vel=(0.0, 0.0, 0.0), xyz_acc=(0.0, 0.0, 9.8),
            pos_world=(0.0, 0.0, 0.0), vel_world=(0.0, 0.0, 0.0), vel_body=(0.0, 0.0, 0.0),
            layout='natural', motion_state=0, battery=0.9, ultrasound=(2.0, 2.0),
        )
        self.ignore = set(ignore)
        self.sent = []
        self.mode = 'move'
        self.control = 'manual'
        self._pending = None  # (time, basic_state) of the next transition
        self.now = 0.0

    def send(self, data: bytes) -> bool:
        """Take one datagram as the motion host would."""
        self.sent.append(data)
        datagram = p.parse_datagram(data)
        code = datagram.code
        if code in self.ignore:
            return True
        state = self.report.basic_state
        if code == p.STAND_SIT and state == p.STATE_SITTING:
            self._set(5)
            self._pending = (self.now + self.TRANSITION_SEC, p.STATE_STANDING)
        elif code == p.STAND_SIT and state == p.STATE_STANDING:
            self._set(7)
            self._pending = (self.now + self.TRANSITION_SEC, p.STATE_SITTING)
        elif code == p.ACTION_HELLO and state == p.STATE_SITTING:
            self._set(p.STATE_HELLO)
            self._pending = (self.now + self.HELLO_SEC, p.STATE_SITTING)
        elif code == p.POSE_MODE:
            self.mode = 'pose'
        elif code == p.MOVE_MODE:
            self.mode = 'move'
        elif code == p.NAVIGATION_MODE:
            self.control = 'navigation'
        elif code == p.MANUAL_MODE:
            self.control = 'manual'
        return True

    def advance(self, now: float):
        """Finish a transition whose time has come."""
        self.now = now
        if self._pending is not None and now >= self._pending[0]:
            self._set(self._pending[1])
            self._pending = None

    def state(self):
        """Return the latest state report."""
        return self.report

    def _set(self, basic_state: int):
        """Change the basic state."""
        self.report = replace(self.report, basic_state=basic_state)

    def codes(self) -> list:
        """Return the command codes received so far, in order."""
        return [p.parse_datagram(d).code for d in self.sent]
