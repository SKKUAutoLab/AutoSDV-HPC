"""Fail-safe command arbiter for manual and autonomous drive commands."""

from concurrent.futures import ThreadPoolExecutor
import fcntl
import os
import signal
import subprocess
import threading
import time

from interfaces_pkg.msg import GuiHeartbeat, MotionCommand, SafetyStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


SOURCE_NONE = 'none'
SOURCE_MANUAL = 'manual'
SOURCE_AUTO = 'auto'
VALID_SOURCES = (SOURCE_NONE, SOURCE_MANUAL, SOURCE_AUTO)


class SafetySupervisorNode(Node):
    """Select one command source and force zero output on safety faults."""

    def __init__(self):
        super().__init__('safety_supervisor_node')

        self._instance_lock_fd = os.open(
            f'/tmp/autosdv_safety_supervisor_{os.getuid()}.lock',
            os.O_CREAT | os.O_RDWR,
            0o600,
        )
        try:
            fcntl.flock(
                self._instance_lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            os.close(self._instance_lock_fd)
            self._instance_lock_fd = None
            raise RuntimeError(
                'Another AutoSDV Safety Supervisor is already running') from exc

        self.manual_topic = self.declare_parameter(
            'manual_topic', '/control/manual_cmd').value
        self.auto_topic = self.declare_parameter(
            'auto_topic', '/control/auto_cmd').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/topic_control_signal').value
        self.heartbeat_topic = self.declare_parameter(
            'heartbeat_topic', '/gui/heartbeat').value
        self.status_topic = self.declare_parameter(
            'status_topic', '/safety/status').value
        self.zcu_ips = list(self.declare_parameter(
            'zcu_ips', ['10.0.0.2', '10.0.0.3', '10.0.0.4', '10.0.0.5']).value)
        self.zcu_probe_period = float(self.declare_parameter(
            'zcu_probe_period_sec', 0.25).value)
        self.zcu_disconnect_timeout = float(self.declare_parameter(
            'zcu_disconnect_timeout_sec', 1.0).value)
        self.output_period = float(self.declare_parameter(
            'output_period_sec', 0.05).value)
        self.owner_pid = int(self.declare_parameter('owner_pid', 0).value)

        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._lock = threading.Lock()
        self._requested_source = SOURCE_NONE
        self._gui_armed = False
        self._stop_ticks_remaining = 0
        self._commands = {SOURCE_MANUAL: None, SOURCE_AUTO: None}
        self._estop_latched = False
        self._estop_reason = ''

        self._zcu_last_success = [None] * len(self.zcu_ips)
        self._zcu_connected = [False] * len(self.zcu_ips)
        self._ping_pool = ThreadPoolExecutor(max_workers=max(1, len(self.zcu_ips)))
        self._ping_futures = [None] * len(self.zcu_ips)

        self.output_pub = self.create_publisher(
            MotionCommand, self.output_topic, self.qos)
        self.status_pub = self.create_publisher(
            SafetyStatus, self.status_topic, 10)
        self.create_subscription(
            MotionCommand, self.manual_topic,
            lambda msg: self._command_callback(SOURCE_MANUAL, msg), self.qos)
        self.create_subscription(
            MotionCommand, self.auto_topic,
            lambda msg: self._command_callback(SOURCE_AUTO, msg), self.qos)
        self.create_subscription(
            GuiHeartbeat, self.heartbeat_topic, self._heartbeat_callback, 10)

        self.create_service(
            Trigger, '/safety/emergency_stop', self._estop_service)
        self.create_service(
            Trigger, '/safety/reset_emergency_stop', self._reset_service)

        self.create_timer(self.output_period, self._control_tick)
        self.create_timer(0.2, self._publish_status)
        self.create_timer(self.zcu_probe_period, self._zcu_probe_tick)
        self.create_timer(0.2, self._owner_watchdog_tick)

        self.get_logger().info(
            f'Safety supervisor ready: {self.manual_topic}, {self.auto_topic} '
            f'-> {self.output_topic}')

    def _heartbeat_callback(self, msg):
        source = msg.requested_source.strip().lower()
        if source not in VALID_SOURCES:
            self._latch_estop(f'Invalid control source: {source}')
            source = SOURCE_NONE

        with self._lock:
            self._requested_source = source
            self._gui_armed = bool(msg.armed and source != SOURCE_NONE)

    def _command_callback(self, source, msg):
        with self._lock:
            self._commands[source] = msg

    @staticmethod
    def _ping(ip):
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', ip],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.2,
                check=False,
            )
            return result.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def _zcu_probe_tick(self):
        now = time.monotonic()
        for index, ip in enumerate(self.zcu_ips):
            future = self._ping_futures[index]
            if future is not None and future.done():
                try:
                    if future.result():
                        self._zcu_last_success[index] = now
                except Exception as exc:
                    self.get_logger().warn(f'ZCU probe failed for {ip}: {exc}')
                self._ping_futures[index] = None

            last_success = self._zcu_last_success[index]
            self._zcu_connected[index] = (
                last_success is not None
                and (now - last_success) <= self.zcu_disconnect_timeout
            )

            if self._ping_futures[index] is None:
                self._ping_futures[index] = self._ping_pool.submit(
                    self._ping, ip)

    def _owner_watchdog_tick(self):
        if self.owner_pid <= 0:
            return
        if self._owner_process_alive(self.owner_pid):
            return
        self.get_logger().error(
            'GUI owner process disappeared; stopping supervisor')
        self.publish_shutdown_stop()
        rclpy.try_shutdown()

    @staticmethod
    def _owner_process_alive(owner_pid):
        try:
            os.kill(owner_pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return False

        try:
            with open(f'/proc/{owner_pid}/stat', encoding='utf-8') as stream:
                process_state = stream.read().rsplit(')', 1)[1].split()[0]
            return process_state != 'Z'
        except (FileNotFoundError, PermissionError, IndexError):
            return False

    def _latch_estop(self, reason):
        with self._lock:
            if self._estop_latched:
                return
            self._estop_latched = True
            self._estop_reason = reason
        self.get_logger().error(f'EMERGENCY STOP: {reason}')

    def _estop_service(self, request, response):
        del request
        self._latch_estop('Emergency stop requested by GUI')
        response.success = True
        response.message = 'Emergency stop latched'
        return response

    def _reset_service(self, request, response):
        del request
        with self._lock:
            source_stopped = (
                self._requested_source == SOURCE_NONE and not self._gui_armed)
            if not source_stopped:
                message = 'Stop the active control node before reset'
            else:
                self._estop_latched = False
                self._estop_reason = ''
                response.success = True
                response.message = 'Emergency stop reset'
                return response

        response.success = False
        response.message = message
        return response

    @staticmethod
    def _zero_command():
        return MotionCommand(steering=0, left_speed=0, right_speed=0)

    def _control_tick(self):
        with self._lock:
            source = self._requested_source
            armed = self._gui_armed
            command = self._commands.get(source)
            estop = self._estop_latched

        # Manual Drive experiments may publish with some or all ZCUs offline.
        if armed and source == SOURCE_AUTO:
            if not all(self._zcu_connected):
                failed = [
                    self.zcu_ips[i] for i, connected
                    in enumerate(self._zcu_connected) if not connected
                ]
                self._latch_estop('ZCU disconnected: ' + ', '.join(failed))
                estop = True

        if estop:
            self.output_pub.publish(self._zero_command())
        elif not armed or source not in (SOURCE_MANUAL, SOURCE_AUTO):
            # Send a short stop burst, then leave the control topic silent.
            if self._stop_ticks_remaining > 0:
                self.output_pub.publish(self._zero_command())
                self._stop_ticks_remaining -= 1
        else:
            self._stop_ticks_remaining = 5
            self.output_pub.publish(
                command if command is not None else self._zero_command())

    def _publish_status(self):
        with self._lock:
            armed = self._gui_armed
            estop = self._estop_latched
            source = self._requested_source if armed and not estop else SOURCE_NONE
            reason = self._estop_reason

        status = SafetyStatus()
        status.active_source = source
        status.armed = armed and not estop
        status.emergency_stop = estop
        status.reason = reason
        status.zcu_connected = list(self._zcu_connected)
        self.status_pub.publish(status)

    def publish_shutdown_stop(self, repeats=10):
        for _ in range(repeats):
            self.output_pub.publish(self._zero_command())
            time.sleep(0.02)

    def destroy_node(self):
        self._ping_pool.shutdown(wait=False, cancel_futures=True)
        if self._instance_lock_fd is not None:
            os.close(self._instance_lock_fd)
            self._instance_lock_fd = None
        super().destroy_node()


def main(args=None):
    stop_requested = threading.Event()

    def handle_signal(signum, frame):
        del signum, frame
        stop_requested.set()

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    node = None
    try:
        rclpy.init(
            args=args,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        node = SafetySupervisorNode()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_shutdown_stop()
            node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
