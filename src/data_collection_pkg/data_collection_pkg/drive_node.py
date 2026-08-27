"""Manual drive node with GUI and optional terminal input."""

import os
import select
import signal
import sys
import termios
import threading
import time
import tty

from interfaces_pkg.msg import DriveInput, MotionCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions


def _move_toward_zero(value, amount):
    if value > 0.0:
        return max(0.0, value - amount)
    if value < 0.0:
        return min(0.0, value + amount)
    return 0.0


class DriveController:
    """Thread-safe manual control state and acceleration model."""

    def __init__(
            self,
            max_steering=7,
            max_speed=255,
            acceleration_rate=50.0,
            deceleration_rate=60.0,
            steering_rate=10.0,
            input_timeout=0.35):
        self.max_steering = float(max_steering)
        self.max_speed = float(max_speed)
        self.acceleration_rate = float(acceleration_rate)
        self.deceleration_rate = float(deceleration_rate)
        self.steering_rate = float(steering_rate)
        self.input_timeout = float(input_timeout)
        self._lock = threading.Lock()
        self._exit_event = threading.Event()
        self._last_input_time = None
        self._forward = False
        self._reverse = False
        self._left = False
        self._right = False
        self._terminal_drive_until = 0.0
        self._terminal_direction = 0
        self.steering = 0.0
        self.speed = 0.0

    def set_gui_input(self, msg):
        with self._lock:
            self._last_input_time = time.monotonic()
            self._forward = bool(msg.forward)
            self._reverse = bool(msg.reverse)
            self._left = bool(msg.steer_left)
            self._right = bool(msg.steer_right)
            if msg.reset:
                self.steering = 0.0
            if msg.emergency_stop:
                self.steering = 0.0
                self.speed = 0.0

    def process_terminal_key(self, key):
        key = key.lower()
        now = time.monotonic()
        with self._lock:
            if key in ('q', '\x03'):
                self.speed = 0.0
                self.steering = 0.0
                self._exit_event.set()
                return 'Exiting'
            if key == 'w':
                self._terminal_direction = 1
                self._terminal_drive_until = now + 0.18
                return 'Forward acceleration'
            if key == 's':
                self._terminal_direction = -1
                self._terminal_drive_until = now + 0.18
                return 'Reverse acceleration'
            if key == 'a':
                self.steering = max(
                    -self.max_steering, self.steering - 1.0)
                return f'Steering={round(self.steering)}'
            if key == 'd':
                self.steering = min(
                    self.max_steering, self.steering + 1.0)
                return f'Steering={round(self.steering)}'
            if key == ' ':
                self.speed = 0.0
                self.steering = 0.0
                return 'Emergency stop'
            if key == 'r':
                self.steering = 0.0
                return 'Steering centered'
        return None

    def update(self, dt):
        now = time.monotonic()
        with self._lock:
            gui_fresh = (
                self._last_input_time is not None
                and now - self._last_input_time <= self.input_timeout
            )
            forward = self._forward if gui_fresh else False
            reverse = self._reverse if gui_fresh else False
            left = self._left if gui_fresh else False
            right = self._right if gui_fresh else False

            if now <= self._terminal_drive_until:
                forward = self._terminal_direction > 0
                reverse = self._terminal_direction < 0

            if forward == reverse:
                self.speed = _move_toward_zero(
                    self.speed, self.deceleration_rate * dt)
            elif forward:
                self.speed = min(
                    self.max_speed, self.speed + self.acceleration_rate * dt)
            else:
                self.speed = max(
                    -self.max_speed, self.speed - self.acceleration_rate * dt)

            if left != right:
                direction = -1.0 if left else 1.0
                self.steering += direction * self.steering_rate * dt
                self.steering = max(
                    -self.max_steering,
                    min(self.max_steering, self.steering),
                )

            return round(self.steering), round(self.speed)

    def request_exit(self):
        with self._lock:
            self.speed = 0.0
            self.steering = 0.0
            self._exit_event.set()

    def exit_requested(self):
        return self._exit_event.is_set()


class DriveNode(Node):
    """Publish manual motion commands generated from DriveInput messages."""

    def __init__(self):
        super().__init__('drive_node')

        input_topic = self.declare_parameter(
            'input_topic', '/gui/manual_input').value
        output_topic = self.declare_parameter(
            'output_topic', '/control/manual_cmd').value
        terminal_input = bool(self.declare_parameter(
            'terminal_input', True).value)
        control_period = float(self.declare_parameter(
            'control_period_sec', 0.05).value)
        max_steering = int(self.declare_parameter(
            'max_steering', 7).value)
        max_speed = int(self.declare_parameter(
            'max_speed', 255).value)
        acceleration_rate = float(self.declare_parameter(
            'acceleration_rate', 50.0).value)
        deceleration_rate = float(self.declare_parameter(
            'deceleration_rate', 60.0).value)
        steering_rate = float(self.declare_parameter(
            'steering_rate', 10.0).value)
        input_timeout = float(self.declare_parameter(
            'input_timeout_sec', 0.35).value)

        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.publisher = self.create_publisher(
            MotionCommand, output_topic, self.qos)
        self.subscription = self.create_subscription(
            DriveInput, input_topic, self._input_callback, 10)
        self.controller = DriveController(
            max_steering=max_steering,
            max_speed=max_speed,
            acceleration_rate=acceleration_rate,
            deceleration_rate=deceleration_rate,
            steering_rate=steering_rate,
            input_timeout=input_timeout,
        )
        self._control_period = control_period
        self._last_update = time.monotonic()
        self._cleanup_lock = threading.Lock()
        self._cleaned_up = False
        self.create_timer(control_period, self._control_tick)

        self._input_thread = None
        if terminal_input:
            if sys.stdin.isatty():
                self._input_thread = threading.Thread(
                    target=self._terminal_loop,
                    name='drive-node-keyboard-input',
                    daemon=True,
                )
                self._input_thread.start()
                self._print_instructions()
            else:
                self.get_logger().warn(
                    'Terminal input requested but stdin is not a TTY; '
                    'GUI topic input remains available.')

        self.get_logger().info(
            f'Manual drive ready: {input_topic} -> {output_topic}')

    def _input_callback(self, msg):
        self.controller.set_gui_input(msg)

    def _control_tick(self):
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self._last_update))
        self._last_update = now
        steering, speed = self.controller.update(dt)
        msg = MotionCommand()
        msg.steering = steering
        msg.left_speed = speed
        msg.right_speed = speed
        self.publisher.publish(msg)

    def _print_instructions(self):
        print('\nROBOT MOTION CONTROL')
        print('  W/S  : Forward/reverse acceleration')
        print('  A/D  : Steer left/right')
        print('  SPACE: Emergency stop')
        print('  R    : Center steering')
        print('  Q    : Quit\n')

    def _terminal_loop(self):
        fd = None
        old_settings = None
        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            while rclpy.ok() and not self.controller.exit_requested():
                readable, _, _ = select.select([fd], [], [], 0.1)
                if not readable:
                    continue
                key_bytes = os.read(fd, 1)
                if not key_bytes:
                    break
                status = self.controller.process_terminal_key(
                    key_bytes.decode('utf-8', errors='ignore'))
                if status:
                    print(status)
        except (OSError, ValueError, termios.error) as exc:
            self.get_logger().error(f'Terminal input failed: {exc}')
        finally:
            if fd is not None and old_settings is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                except (OSError, termios.error):
                    pass

    def publish_stop(self, repeats=5):
        stop = MotionCommand(steering=0, left_speed=0, right_speed=0)
        for _ in range(repeats):
            self.publisher.publish(stop)
            time.sleep(0.02)

    def cleanup(self):
        with self._cleanup_lock:
            if self._cleaned_up:
                return
            self._cleaned_up = True
        self.controller.request_exit()
        if self._input_thread is not None and self._input_thread.is_alive():
            self._input_thread.join(timeout=0.5)
        if rclpy.ok():
            self.publish_stop()


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
        node = DriveNode()
        while (rclpy.ok()
               and not stop_requested.is_set()
               and not node.controller.exit_requested()):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
