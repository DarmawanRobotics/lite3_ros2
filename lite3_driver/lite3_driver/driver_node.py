#!/usr/bin/env python3
"""
ROS 2 driver of the Deep Robotics Jueying Lite3 over the motion host's UDP protocol.

cmd_vel becomes speed commands in navigation mode; the motion host's reports become odometry,
IMU, battery, joint states, ultrasound and the RobotState message. Every robot command is a
std_srvs/Trigger service; gestures block until they are done.
"""

import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from lite3_driver import protocol as p
from lite3_driver.convert import (
    attitude,
    finite_or,
    odometry_orientation,
    quaternion_from_rpy,
    ultrasound_range,
)
from lite3_driver.gestures import GestureConfig, GestureRunner, GESTURES
from lite3_driver.link import MotionLink
from lite3_driver.motion import NAVIGATION_DATAGRAM, VelocityConfig, VelocityPolicy
from lite3_interfaces.msg import RobotState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState, Range
from std_srvs.srv import Trigger

ULTRASOUND_MIN = 0.28
ULTRASOUND_MAX = 4.5
ULTRASOUND_FOV = 0.5  # rad, nominal cone of the Lite3 ultrasound sensors

# Plain commands: service name -> (code, value) datagrams sent in this order.
COMMANDS = {
    'reset_zero': ((p.RESET_ZERO, 0),),
    'navigation_mode': ((p.NAVIGATION_MODE, 0),),
    'manual_mode': ((p.MANUAL_MODE, 0),),
    'move_mode': ((p.MOVE_MODE, 0),),
    'pose_mode': ((p.POSE_MODE, 0),),
    'gait_slow': ((p.GAIT_FLAT_SLOW, 0),),
    'gait_medium': ((p.GAIT_FLAT_MEDIUM, 0),),
    'gait_fast': ((p.GAIT_FLAT_FAST, 0),),
    'gait_crawl': ((p.GAIT_FLAT_CRAWL, 0),),
    'gait_general': ((p.GAIT_RUG_GENERAL, 0),),
    'gait_grip': ((p.GAIT_RUG_GRIP, 0),),
    'gait_high_step': ((p.GAIT_RUG_HIGH_STEP, 0),),
    'stop_action': ((p.STOP_ACTION, 0), (p.STOP_ACTION, 1)),
    'keep_stepping_on': ((p.KEEP_STEPPING, p.KEEP_STEPPING_ON),),
    'keep_stepping_off': ((p.KEEP_STEPPING, p.KEEP_STEPPING_OFF),),
    'speaker_on': ((p.SPEAKER, p.SPEAKER_ON),),
    'speaker_off': ((p.SPEAKER, p.SPEAKER_OFF),),
    'save_data': ((p.SAVE_DATA, 0),),
}
GAITS = {name for name in COMMANDS if name.startswith('gait_')}
MODES = {'navigation_mode': 'navigation', 'manual_mode': 'manual'}


class Lite3DriverNode(Node):
    """Bridges ROS 2 and the Lite3 motion host."""

    def __init__(self, **kwargs):
        """Declare parameters, open the UDP link and create every interface."""
        super().__init__('lite3_driver', **kwargs)
        declare = self.declare_parameter
        declare('motion_host_ip', '192.168.1.120')
        declare('motion_host_port', 43893)
        declare('local_ip', '0.0.0.0')
        declare('local_port', 43897)
        declare('heartbeat_hz', 5.0)
        declare('cmd_rate_hz', 20.0)
        declare('cmd_timeout_sec', 0.5)
        declare('stop_burst_sec', 0.5)
        declare('auto_navigation_mode', True)
        declare('mode_refresh_sec', 2.0)
        declare('max_vx', 1.0)
        declare('max_vy', 0.5)
        declare('max_wz', 1.5)
        declare('invert_vy', False)
        declare('invert_wz', False)
        declare('odom_frame', 'odom')
        declare('base_frame', 'base_link')
        declare('imu_frame', 'imu_link')
        declare('odom_hz', 50.0)
        declare('joint_state_hz', 50.0)
        declare('state_hz', 10.0)
        declare('battery_hz', 1.0)
        declare('ultrasound_hz', 10.0)
        declare('state_timeout_sec', 1.0)
        declare('low_battery', 0.2)
        gesture_defaults = GestureConfig()
        for field in gesture_defaults.__dataclass_fields__:
            declare(f'gesture.{field}', getattr(gesture_defaults, field))

        value = lambda name: self.get_parameter(name).value  # noqa: E731
        self._odom_frame = value('odom_frame')
        self._base_frame = value('base_frame')
        self._imu_frame = value('imu_frame')
        self._state_timeout = float(value('state_timeout_sec'))
        self._low_battery = float(value('low_battery'))
        self._policy = VelocityPolicy(VelocityConfig(
            max_vx=float(value('max_vx')),
            max_vy=float(value('max_vy')),
            max_wz=float(value('max_wz')),
            cmd_timeout_sec=float(value('cmd_timeout_sec')),
            stop_burst_sec=float(value('stop_burst_sec')),
            auto_navigation_mode=bool(value('auto_navigation_mode')),
            mode_refresh_sec=float(value('mode_refresh_sec')),
            invert_vy=bool(value('invert_vy')),
            invert_wz=bool(value('invert_wz')),
        ))
        gesture_config = GestureConfig(**{
            field: type(getattr(gesture_defaults, field))(value(f'gesture.{field}'))
            for field in gesture_defaults.__dataclass_fields__
        })

        self._data_lock = threading.Lock()
        self._report = None
        self._report_time = None
        self._new_report = False
        self._angles = None
        self._velocities = None
        self._speaker = None
        self._prefix_warned = False
        self._control_mode = 'unknown'
        self._ultrasound_due = 0.0

        self._link = MotionLink(
            value('motion_host_ip'),
            int(value('motion_host_port')),
            value('local_ip'),
            int(value('local_port')),
            self.on_datagram,
        )
        self._runner = GestureRunner(
            gesture_config,
            send=self._link.send,
            state=self.latest_report,
            connected=self.connected,
            moving=lambda: self._policy.recently_moved(time.monotonic(),
                                                       gesture_config.quiet_sec),
            clock=time.monotonic,
            sleep=time.sleep,
        )

        # reliable everywhere: reliable publishers also reach best-effort subscribers (RViz,
        # rosbridge), the other way round they would not
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu', 10)
        self._battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._front_pub = self.create_publisher(Range, 'ultrasound/front', 10)
        self._rear_pub = self.create_publisher(Range, 'ultrasound/rear', 10)
        self._state_pub = self.create_publisher(RobotState, 'state', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.create_service(Trigger, 'stand_up', self.on_stand_up)
        self.create_service(Trigger, 'sit_down', self.on_sit_down)
        self.create_service(Trigger, 'emergency_stop', self.on_emergency_stop)
        for name, datagrams in COMMANDS.items():
            self.create_service(Trigger, name, self.command_callback(name, datagrams))
        gestures = ReentrantCallbackGroup()
        for name in GESTURES:
            self.create_service(
                Trigger, name, self.gesture_callback(name), callback_group=gestures
            )

        self.create_timer(1.0 / float(value('heartbeat_hz')), self.send_heartbeat)
        self.create_timer(1.0 / float(value('cmd_rate_hz')), self.send_velocity)
        self.create_timer(1.0 / float(value('odom_hz')), self.publish_motion)
        self.create_timer(1.0 / float(value('joint_state_hz')), self.publish_joints)
        self.create_timer(1.0 / float(value('state_hz')), self.publish_state)
        self.create_timer(1.0 / float(value('battery_hz')), self.publish_battery)
        self.create_timer(1.0, self.publish_diagnostics)
        self._ultrasound_period = 1.0 / float(value('ultrasound_hz'))

        self._link.start()
        self.get_logger().info(
            f'Lite3 motion host {self._link.remote[0]}:{self._link.remote[1]}, reports on '
            f'{self._link.local[0]}:{self._link.local[1]} (the robot must send them here: '
            'network.toml on the motion host)'
        )

    # ------------------------------------------------------------------ link

    def on_datagram(self, data: bytes, _address):
        """Store one report from the motion host (receive thread)."""
        datagram = p.parse_datagram(data)
        if datagram is None:
            return
        if datagram.code == p.STATE and datagram.kind == p.TYPE_COMPLEX:
            report = p.decode_state(datagram.payload)
            if report is None:
                return
            if report.layout == 'prefix' and not self._prefix_warned:
                self._prefix_warned = True
                self.get_logger().warn(
                    f'state report of {len(datagram.payload)} bytes: unknown layout, battery, '
                    'motion state and ultrasound are left out'
                )
            with self._data_lock:
                self._report = report
                self._report_time = time.monotonic()
                self._new_report = True
        elif datagram.code == p.JOINT_ANGLES:
            angles = p.decode_joints(datagram.payload)
            if angles is not None:
                with self._data_lock:
                    self._angles = angles
        elif datagram.code == p.JOINT_VELOCITIES:
            velocities = p.decode_joints(datagram.payload)
            if velocities is not None:
                with self._data_lock:
                    self._velocities = velocities
        elif datagram.code == p.SPEAKER_STATUS:
            self._speaker = bool(datagram.value)

    def latest_report(self):
        """Return the latest state report, or None."""
        with self._data_lock:
            return self._report

    def age(self) -> float:
        """Return seconds since the last state report."""
        with self._data_lock:
            stamp = self._report_time
        return float('inf') if stamp is None else time.monotonic() - stamp

    def connected(self) -> bool:
        """Return True while state reports keep arriving."""
        return self.age() <= self._state_timeout

    def send_commands(self, datagrams) -> bool:
        """Send simple commands; True when all went out."""
        return self._link.send_all(p.simple(code, value) for code, value in datagrams) == len(
            datagrams
        )

    # ------------------------------------------------------------------ motion

    def on_cmd_vel(self, msg: Twist):
        """Take a velocity command; any motion stops a running gesture."""
        now = time.monotonic()
        self._policy.command(msg.linear.x, msg.linear.y, msg.angular.z, now)
        if self._runner.active and self._policy.moving(now):
            self._runner.abort()

    def send_heartbeat(self):
        """Keep the connection alive (the motion host wants at least 2 Hz)."""
        self._link.send(p.simple(p.HEARTBEAT))

    def send_velocity(self):
        """Stream the speed commands; gestures own the link while they run."""
        if self._runner.active:
            return
        datagrams = self._policy.tick(time.monotonic())
        if NAVIGATION_DATAGRAM in datagrams:
            self._control_mode = 'navigation'
        self._link.send_all(datagrams)

    # ------------------------------------------------------------------ services

    def on_stand_up(self, _request, response):
        """Stand up from sitting; nothing to do when already standing."""
        report = self.latest_report()
        if report is None or not self.connected():
            response.success, response.message = False, 'no state from the motion host'
        elif report.standing:
            response.success, response.message = True, 'already standing'
        elif report.sitting:
            response.success = self.send_commands(((p.STAND_SIT, 0),))
            response.message = 'standing up' if response.success else self._link.last_error
        else:
            response.success = False
            response.message = f'busy ({p.name_of(p.BASIC_STATES, report.basic_state)})'
        return response

    def on_sit_down(self, _request, response):
        """Sit down from standing, never while walking."""
        report = self.latest_report()
        if report is None or not self.connected():
            response.success, response.message = False, 'no state from the motion host'
        elif report.sitting:
            response.success, response.message = True, 'already sitting'
        elif self.walking(report):
            response.success, response.message = False, 'the robot is walking'
        elif report.standing:
            response.success = self.send_commands(((p.STAND_SIT, 0),))
            response.message = 'sitting down' if response.success else self._link.last_error
        else:
            response.success = False
            response.message = f'busy ({p.name_of(p.BASIC_STATES, report.basic_state)})'
        return response

    def on_emergency_stop(self, _request, response):
        """Zero the velocity, stop any gesture and send the soft emergency stop."""
        self._policy.stop(time.monotonic())
        self._runner.abort()
        self._link.send_all(self._policy.speeds(0.0, 0.0, 0.0))
        response.success = self.send_commands(((p.SOFT_ESTOP, 0),))
        response.message = 'soft emergency stop sent' if response.success else (
            self._link.last_error
        )
        return response

    def walking(self, report) -> bool:
        """Return True while the robot walks or was told to in the last second."""
        return self._policy.recently_moved(time.monotonic(), 1.0) or report.stepping

    def command_callback(self, name: str, datagrams):
        """Return the service callback of a plain command."""
        def callback(_request, response):
            report = self.latest_report()
            if name in GAITS and report is not None and report.basic_state == p.STATE_AI:
                response.success, response.message = False, 'gaits cannot change in AI state'
                return response
            if name == 'reset_zero' and report is not None and (
                report.standing or self.walking(report)
            ):
                response.success, response.message = False, 'sit down first'
                return response
            response.success = self.send_commands(datagrams)
            if response.success and name in MODES:
                self._control_mode = MODES[name]
            note = '' if self.connected() else ' (no state from the motion host)'
            response.message = f'{name} sent{note}' if response.success else self._link.last_error
            return response
        return callback

    def gesture_callback(self, name: str):
        """Return the blocking service callback of a gesture."""
        def callback(_request, response):
            mode = self._control_mode
            response.success, response.message = self._runner.run(name, mode)
            if mode == 'navigation':
                self._control_mode = 'navigation'
            elif self._runner.config.use_manual_mode and name != 'hello':
                self._control_mode = 'manual'
            return response
        return callback

    # ------------------------------------------------------------------ publishing

    def publish_motion(self):
        """Publish odometry, IMU and (slower) ultrasound for each new state report."""
        with self._data_lock:
            report, fresh = self._report, self._new_report
            self._new_report = False
        if report is None or not fresh:
            return
        stamp = self.get_clock().now().to_msg()
        self._odom_pub.publish(self.odometry(report, stamp))
        self._imu_pub.publish(self.imu(report, stamp))
        now = time.monotonic()
        if report.ultrasound is not None and now >= self._ultrasound_due:
            self._ultrasound_due = now + self._ultrasound_period
            self._front_pub.publish(self.range_msg('ultrasound_front', report.ultrasound[0],
                                                   stamp))
            self._rear_pub.publish(self.range_msg('ultrasound_rear', report.ultrasound[1],
                                                  stamp))

    def odometry(self, report, stamp) -> Odometry:
        """Leg odometry: pose in the robot's own world frame, twist in base_link."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = float(report.pos_world[0])
        msg.pose.pose.position.y = float(report.pos_world[1])
        q = odometry_orientation(report)
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        msg.twist.twist.linear.x = float(report.vel_body[0])
        msg.twist.twist.linear.y = float(report.vel_body[1])
        msg.twist.twist.angular.z = float(report.vel_body[2])
        for i, variance in enumerate((0.01, 0.01, 1e6, 1e6, 1e6, 0.05)):
            msg.pose.covariance[i * 7] = variance
        for i, variance in enumerate((0.02, 0.02, 1e6, 1e6, 1e6, 0.05)):
            msg.twist.covariance[i * 7] = variance
        return msg

    def imu(self, report, stamp) -> Imu:
        """IMU attitude, rates and acceleration as the motion host reports them."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame
        q = quaternion_from_rpy(*attitude(report))
        o = msg.orientation
        o.x, o.y, o.z, o.w = q
        av, la = msg.angular_velocity, msg.linear_acceleration
        av.x, av.y, av.z = (float(v) for v in report.rpy_vel)
        la.x, la.y, la.z = (float(v) for v in report.xyz_acc)
        return msg

    def range_msg(self, frame: str, value, stamp) -> Range:
        """One ultrasound reading, clamped to the sensor's 0.28-4.5 m."""
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = ULTRASOUND_FOV
        msg.min_range = ULTRASOUND_MIN
        msg.max_range = ULTRASOUND_MAX
        msg.range = ultrasound_range(value, ULTRASOUND_MIN, ULTRASOUND_MAX)
        return msg

    def publish_joints(self):
        """Publish the joint angles and velocities in the URDF's joint names."""
        with self._data_lock:
            angles, velocities = self._angles, self._velocities
        if angles is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(p.JOINT_NAMES)
        msg.position = [float(v) for v in angles]
        if velocities is not None:
            msg.velocity = [float(v) for v in velocities]
        self._joint_pub.publish(msg)

    def publish_battery(self):
        """Publish the battery level (0-1) when the report carries it."""
        report = self.latest_report()
        if report is None or report.battery is None or not self.connected():
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(report.battery)
        msg.voltage = float('nan')
        msg.current = float('nan')
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self._battery_pub.publish(msg)

    def publish_state(self):
        """Publish the readable robot state."""
        report = self.latest_report()
        age = self.age()
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = age <= self._state_timeout
        msg.age_sec = float(min(age, 1e6))
        msg.control_mode = self._control_mode
        msg.gesture = self._runner.active
        if report is not None:
            msg.basic_state = int(report.basic_state)
            msg.basic_state_name = p.name_of(p.BASIC_STATES, report.basic_state)
            msg.gait_state = int(report.gait_state)
            msg.gait_name = p.name_of(p.GAIT_STATES, report.gait_state)
            msg.policy_state = int(report.policy_state)
            msg.motion_state = int(report.motion_state or 0)
            msg.motion_name = p.name_of(p.MOTION_STATES, report.motion_state)
            msg.standing = report.standing
            msg.moving = report.stepping
            msg.balance_lost = bool(report.balance_lost)
            msg.zeroed = bool(report.zeroed)
            msg.battery = finite_or(report.battery, 0.0)
            if report.ultrasound is not None:
                msg.ultrasound_front = ultrasound_range(report.ultrasound[0], ULTRASOUND_MIN,
                                                        ULTRASOUND_MAX)
                msg.ultrasound_rear = ultrasound_range(report.ultrasound[1], ULTRASOUND_MIN,
                                                       ULTRASOUND_MAX)
            msg.layout = report.layout
        else:
            msg.basic_state_name = msg.gait_name = msg.motion_name = 'unknown'
        self._state_pub.publish(msg)

    def publish_diagnostics(self):
        """Publish the driver's health for the handheld and the dashboard."""
        report = self.latest_report()
        age = self.age()
        status = DiagnosticStatus(name='lite3_driver', hardware_id='lite3')
        if report is None or age > self._state_timeout:
            status.level = DiagnosticStatus.ERROR
            host = self._link.remote[0]
            status.message = (
                f'no state from {host} for {age:.1f} s' if report is not None
                else f'no state from {host} yet (network.toml on the motion host?)'
            )
        else:
            state = p.name_of(p.BASIC_STATES, report.basic_state)
            gait = p.name_of(p.GAIT_STATES, report.gait_state)
            status.message = f'{state}, {gait}'
            low = report.battery is not None and report.battery < self._low_battery
            if report.basic_state == 8 or report.balance_lost or low:
                status.level = DiagnosticStatus.WARN
                status.message += ', balance lost' if report.balance_lost else ''
                status.message += ', protection' if report.basic_state == 8 else ''
                status.message += ', low battery' if low else ''
            else:
                status.level = DiagnosticStatus.OK
        values = {
            'rx': self._link.rx_count,
            'tx': self._link.tx_count,
            'tx_errors': self._link.tx_errors,
            'control_mode': self._control_mode,
            'gesture': self._runner.active or '-',
            'speaker': '-' if self._speaker is None else ('on' if self._speaker else 'off'),
            'cmd': ' '.join(f'{v:.2f}' for v in self._policy.sent_command(time.monotonic())),
        }
        if report is not None:
            values.update(
                basic_state=report.basic_state,
                gait_state=report.gait_state,
                motion_state=report.motion_state,
                battery=f'{report.battery:.2f}' if report.battery is not None else '-',
                layout=report.layout,
            )
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._diag_pub.publish(msg)

    def destroy_node(self):
        """Send zero speeds once and close the link."""
        try:
            self._link.send_all(self._policy.speeds(0.0, 0.0, 0.0))
        finally:
            self._link.stop()
        super().destroy_node()


def main(args=None):
    """Spin the driver; gestures run on their own executor threads."""
    rclpy.init(args=args)
    node = Lite3DriverNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise  # a real error while running
        # else: Ctrl-C shut the context down under spin(), a normal stop
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
