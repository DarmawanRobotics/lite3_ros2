"""In-process tests of the driver against a fake motion host on the loopback interface."""

from dataclasses import replace
import math
import socket
import struct
import threading
import time

from fake_robot import FakeRobot
from lite3_driver import protocol as p
import pytest

rclpy = pytest.importorskip('rclpy')
Parameter = pytest.importorskip('rclpy.parameter').Parameter
MultiThreadedExecutor = pytest.importorskip('rclpy.executors').MultiThreadedExecutor
Twist = pytest.importorskip('geometry_msgs.msg').Twist
Vector3 = pytest.importorskip('geometry_msgs.msg').Vector3
Odometry = pytest.importorskip('nav_msgs.msg').Odometry
sensor_msgs = pytest.importorskip('sensor_msgs.msg')
Trigger = pytest.importorskip('std_srvs.srv').Trigger
RobotState = pytest.importorskip('lite3_interfaces.msg').RobotState
Lite3DriverNode = pytest.importorskip('lite3_driver.driver_node').Lite3DriverNode

NAMESPACE = f'lite3_test_{int(time.time() * 1000) % 100000}'


def free_port() -> int:
    """Return a UDP port nobody listens on."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def wait_for(predicate, timeout: float = 3.0) -> bool:
    """Poll until the predicate holds."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class MotionHost:
    """Loopback stand-in for the motion host: records commands, sends reports."""

    def __init__(self):
        """Bind a socket and start recording."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.received = []
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        """Record every datagram from the driver."""
        while self._running:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                continue
            self.received.append((time.monotonic(), p.parse_datagram(data)))

    def codes(self, since: float = 0.0) -> list:
        """Return the command codes received after a time."""
        return [d.code for t, d in list(self.received) if t >= since and d is not None]

    def speeds(self, since: float = 0.0) -> list:
        """Return (code, value) of the speed commands received after a time."""
        return [
            (d.code, struct.unpack('<d', d.payload)[0])
            for t, d in list(self.received)
            if t >= since and d is not None and d.code in p.SPEED_LIMITS
        ]

    def report(self, port: int, report):
        """Send a state report to the driver."""
        self.sock.sendto(p.encode_state(report), ('127.0.0.1', port))

    def close(self):
        """Stop recording."""
        self._running = False
        self._thread.join(timeout=1.0)
        self.sock.close()


@pytest.fixture
def rig():
    """Run a driver, a fake motion host and a listener on their own threads."""
    host = MotionHost()
    driver_port = free_port()
    rclpy.init()
    node = Lite3DriverNode(
        namespace=NAMESPACE,
        parameter_overrides=[
            Parameter('motion_host_ip', value='127.0.0.1'),
            Parameter('motion_host_port', value=host.port),
            Parameter('local_ip', value='127.0.0.1'),
            Parameter('local_port', value=driver_port),
            Parameter('gesture.nod_period_sec', value=0.4),
            Parameter('gesture.nod_cycles', value=1),
        ],
    )
    listener = rclpy.create_node('lite3_test_listener', namespace=NAMESPACE)
    got = {'odom': [], 'imu': [], 'battery': [], 'state': [], 'joints': [], 'front': []}
    listener.create_subscription(Odometry, 'odom', got['odom'].append, 10)
    listener.create_subscription(sensor_msgs.Imu, 'imu', got['imu'].append, 10)
    listener.create_subscription(sensor_msgs.BatteryState, 'battery', got['battery'].append, 10)
    listener.create_subscription(RobotState, 'state', got['state'].append, 10)
    listener.create_subscription(sensor_msgs.JointState, 'joint_states', got['joints'].append, 10)
    listener.create_subscription(sensor_msgs.Range, 'ultrasound/front', got['front'].append, 10)
    cmd = listener.create_publisher(Twist, 'cmd_vel', 10)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(listener)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    class Rig:
        pass

    r = Rig()
    r.host, r.node, r.listener, r.got, r.cmd, r.port = host, node, listener, got, cmd, driver_port

    def call(name: str, timeout: float = 15.0):
        client = listener.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=3.0), name
        future = client.call_async(Trigger.Request())
        assert wait_for(future.done, timeout), f'{name} did not answer'
        return future.result()

    r.call = call
    yield r
    executor.shutdown()
    thread.join(timeout=2.0)
    listener.destroy_node()
    node.destroy_node()
    host.close()
    rclpy.shutdown()


def standing_report(**overrides):
    """Return a standing robot's report."""
    robot = FakeRobot(p.STATE_STANDING)
    values = {
        'pos_world': (1.0, 2.0, math.pi / 2), 'vel_body': (0.3, 0.1, 0.2),
        'rpy_deg': (0.0, 0.0, 90.0), 'rpy_vel': (0.0, 0.0, 0.2), 'battery': 0.8,
        'ultrasound': (1.5, 4.0),
    }
    values.update(overrides)
    return replace(robot.report, **values)


def test_reports_become_odometry_imu_battery_and_state(rig):
    """A state report shows up as odom, imu, battery, ultrasound and RobotState."""
    rig.host.report(rig.port, standing_report())
    assert wait_for(lambda: rig.got['odom'] and rig.got['imu'] and rig.got['front'])
    odom = rig.got['odom'][-1]
    assert (odom.pose.pose.position.x, odom.pose.pose.position.y) == (1.0, 2.0)
    assert odom.twist.twist.linear.x == pytest.approx(0.3)
    assert odom.twist.twist.angular.z == pytest.approx(0.2)
    assert odom.child_frame_id == 'base_link'
    q = rig.got['imu'][-1].orientation
    assert q.z == pytest.approx(math.sin(math.pi / 4))
    assert q.w == pytest.approx(math.cos(math.pi / 4))
    assert rig.got['front'][-1].range == pytest.approx(1.5)
    assert wait_for(lambda: any(s.connected and s.standing for s in rig.got['state']))
    state = [s for s in rig.got['state'] if s.connected][-1]
    assert state.basic_state_name == 'standing' and state.layout == 'natural'
    rig.host.report(rig.port, standing_report())
    assert wait_for(lambda: rig.got['battery'], timeout=2.5)
    assert rig.got['battery'][-1].percentage == pytest.approx(0.8)
    rig.host.sock.sendto(p.encode_joints(p.JOINT_ANGLES, [0.1] * 12), ('127.0.0.1', rig.port))
    assert wait_for(lambda: rig.got['joints'])
    assert list(rig.got['joints'][-1].name) == list(p.JOINT_NAMES)


def test_cmd_vel_streams_speeds_then_zeros_then_silence(rig):
    """cmd_vel becomes navigation mode and speed commands; the watchdog stops the robot."""
    assert wait_for(lambda: p.HEARTBEAT in rig.host.codes())
    start = time.monotonic()
    for _ in range(10):
        rig.cmd.publish(Twist(linear=Vector3(x=0.2, y=0.1), angular=Vector3(z=0.3)))
        time.sleep(0.05)
    assert wait_for(lambda: (p.SPEED_WZ, 0.3) in rig.host.speeds(start))
    assert p.NAVIGATION_MODE in rig.host.codes(start)
    assert (p.SPEED_VX, 0.2) in rig.host.speeds(start)
    assert (p.SPEED_VY, 0.1) in rig.host.speeds(start)
    stopped = time.monotonic()
    time.sleep(0.6)
    assert (p.SPEED_VX, 0.0) in rig.host.speeds(stopped)  # watchdog zeros
    time.sleep(0.8)
    quiet = time.monotonic()
    time.sleep(0.4)
    assert rig.host.speeds(quiet) == []  # then silence
    assert p.HEARTBEAT in rig.host.codes(quiet)


def test_stand_up_and_sit_down_follow_the_state(rig):
    """Stand up toggles only from sitting; sit down is refused without state."""
    assert not rig.call('sit_down').success  # no state yet
    rig.host.report(rig.port, standing_report(basic_state=p.STATE_SITTING))
    assert wait_for(lambda: rig.node.latest_report() is not None)
    since = time.monotonic()
    answer = rig.call('stand_up')
    assert answer.success and answer.message == 'standing up'
    assert wait_for(lambda: p.STAND_SIT in rig.host.codes(since))
    rig.host.report(rig.port, standing_report())
    assert wait_for(lambda: rig.node.latest_report().standing)
    since = time.monotonic()
    assert rig.call('stand_up').message == 'already standing'
    time.sleep(0.2)
    assert p.STAND_SIT not in rig.host.codes(since)


def test_commands_and_a_blocking_gesture(rig):
    """Plain commands go out; a gesture streams pose-mode axes and answers when done."""
    since = time.monotonic()
    assert rig.call('gait_slow').success
    assert rig.call('stop_action').success
    assert wait_for(lambda: rig.host.codes(since).count(p.STOP_ACTION) == 2)
    assert p.GAIT_FLAT_SLOW in rig.host.codes(since)

    stop = threading.Event()

    def keep_reporting():
        while not stop.is_set():
            rig.host.report(rig.port, standing_report(vel_body=(0.0, 0.0, 0.0)))
            time.sleep(0.02)

    reporter = threading.Thread(target=keep_reporting, daemon=True)
    reporter.start()
    try:
        assert wait_for(lambda: rig.node.connected())
        since = time.monotonic()
        answer = rig.call('nod')
        assert answer.success, answer.message
        codes = rig.host.codes(since)
        assert codes[0] == p.MANUAL_MODE and p.POSE_MODE in codes and p.MOVE_MODE in codes
        assert codes.count(p.AXIS_PITCH) >= 5
        assert rig.call('bow', timeout=10.0).success
    finally:
        stop.set()
        reporter.join(timeout=1.0)
    assert wait_for(lambda: not rig.node.connected())
    answer = rig.call('look_left')  # reports stopped: no gesture without state
    assert not answer.success and 'motion host' in answer.message
