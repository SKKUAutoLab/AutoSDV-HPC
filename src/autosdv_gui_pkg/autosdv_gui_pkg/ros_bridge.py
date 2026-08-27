"""Thread-safe ROS bridge used by the Qt GUI."""

from collections import deque
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from interfaces_pkg.action import RunIvnDiagnostics
from interfaces_pkg.msg import (
    DriveInput,
    GuiHeartbeat,
    MotionCommand,
    SafetyStatus,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger


class GuiRosBridge(Node):
    def __init__(self):
        super().__init__('autosdv_gui')
        self._lock = threading.Lock()
        self._latest_command = MotionCommand()
        self._latest_safety = None
        self._latest_safety_time = None
        self._events = deque()
        self._preview_subscription = None
        self._preview_topic = ''
        self._preview_generation = 0
        self._preview_sequence = 0
        self._latest_preview = None
        self._preview_listener = None
        self._preview_notification_pending = False
        self._safety_supervisor_count = 0
        self._latest_ivn_diagnostics = None
        self._latest_ivn_diagnostics_time = None
        self._ivn_diagnostics_generation = 0
        self._manual_diagnostics = {
            'generation': 0,
            'state': 'idle',
            'completed': 0,
            'total': 0,
            'current': '',
            'summary': '',
            'results': [],
        }
        self._manual_diagnostics_goal = None
        self._preview_callback_group = MutuallyExclusiveCallbackGroup()
        self._graph_callback_group = MutuallyExclusiveCallbackGroup()

        command_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.preview_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        preview_source_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.drive_pub = self.create_publisher(
            DriveInput, '/gui/manual_input', 10)
        self.heartbeat_pub = self.create_publisher(
            GuiHeartbeat, '/gui/heartbeat', 10)
        self.fallback_stop_pub = self.create_publisher(
            MotionCommand, '/topic_control_signal', command_qos)
        self.preview_source_pub = self.create_publisher(
            String, '/autosdv_gui/preview/source', preview_source_qos)
        self.create_subscription(
            MotionCommand, '/topic_control_signal',
            self._command_callback, command_qos)
        self.create_subscription(
            SafetyStatus, '/safety/status',
            self._safety_callback, 10)
        self.create_subscription(
            DiagnosticArray, '/ivn/diagnostics/status',
            self._ivn_diagnostics_callback, 10)
        self.ivn_diagnostics_client = ActionClient(
            self, RunIvnDiagnostics, '/ivn/diagnostics/run_manual')
        self.estop_client = self.create_client(
            Trigger, '/safety/emergency_stop')
        self.reset_client = self.create_client(
            Trigger, '/safety/reset_emergency_stop')
        # ROS graph queries can occasionally block. Keep them off Qt's GUI
        # thread and expose only the cached result to the window.
        self.create_timer(
            0.5,
            self._update_safety_supervisor_count,
            callback_group=self._graph_callback_group,
        )

    def _command_callback(self, msg):
        with self._lock:
            self._latest_command = msg

    def _safety_callback(self, msg):
        with self._lock:
            self._latest_safety = msg
            self._latest_safety_time = time.monotonic()

    def _ivn_diagnostics_callback(self, msg):
        with self._lock:
            self._latest_ivn_diagnostics = msg
            self._latest_ivn_diagnostics_time = time.monotonic()
            self._ivn_diagnostics_generation += 1

    def snapshot(self):
        with self._lock:
            command = self._latest_command
            safety = self._latest_safety
            safety_time = self._latest_safety_time
            events = list(self._events)
            self._events.clear()
        safety_age = (
            None if safety_time is None else time.monotonic() - safety_time)
        return command, safety, safety_age, events

    def safety_supervisor_count(self):
        with self._lock:
            return self._safety_supervisor_count

    def diagnostic_snapshot(self):
        with self._lock:
            message = self._latest_ivn_diagnostics
            received_at = self._latest_ivn_diagnostics_time
            generation = self._ivn_diagnostics_generation
            manual = dict(self._manual_diagnostics)
            manual['results'] = list(self._manual_diagnostics['results'])
        age = None if received_at is None else time.monotonic() - received_at
        return message, age, generation, manual

    def run_manual_diagnostics(self):
        if not self.ivn_diagnostics_client.server_is_ready():
            return False, 'IVN diagnostics action is unavailable'
        with self._lock:
            if self._manual_diagnostics['state'] in ('requesting', 'running'):
                return False, 'A manual IVN check is already running'
            generation = self._manual_diagnostics['generation'] + 1
            self._manual_diagnostics = {
                'generation': generation,
                'state': 'requesting',
                'completed': 0,
                'total': 0,
                'current': 'Sending manual deep-check request',
                'summary': '',
                'results': [],
            }

        goal = RunIvnDiagnostics.Goal()
        goal.check_data_flow = True
        future = self.ivn_diagnostics_client.send_goal_async(
            goal, feedback_callback=self._manual_diagnostics_feedback)
        future.add_done_callback(self._manual_diagnostics_goal_response)
        return True, ''

    def _manual_diagnostics_goal_response(self, completed):
        try:
            goal_handle = completed.result()
        except Exception as exc:
            self._set_manual_diagnostics_failed(f'Request failed: {exc}')
            return
        if not goal_handle.accepted:
            self._set_manual_diagnostics_failed(
                'Manual IVN check was rejected by the diagnostics node')
            return
        with self._lock:
            self._manual_diagnostics_goal = goal_handle
            self._manual_diagnostics['state'] = 'running'
            self._manual_diagnostics['generation'] += 1
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._manual_diagnostics_result)

    def _manual_diagnostics_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        with self._lock:
            self._manual_diagnostics['state'] = 'running'
            self._manual_diagnostics['completed'] = int(
                feedback.completed_checks)
            self._manual_diagnostics['total'] = int(feedback.total_checks)
            self._manual_diagnostics['current'] = feedback.current_check
            self._manual_diagnostics['generation'] += 1

    def _manual_diagnostics_result(self, completed):
        try:
            result = completed.result().result
        except Exception as exc:
            self._set_manual_diagnostics_failed(f'Manual check failed: {exc}')
            return
        with self._lock:
            self._manual_diagnostics_goal = None
            self._manual_diagnostics['state'] = (
                'complete' if result.success else 'failed')
            self._manual_diagnostics['summary'] = result.summary
            self._manual_diagnostics['results'] = list(result.results)
            self._manual_diagnostics['current'] = ''
            self._manual_diagnostics['generation'] += 1

    def _set_manual_diagnostics_failed(self, message):
        with self._lock:
            self._manual_diagnostics_goal = None
            self._manual_diagnostics['state'] = 'failed'
            self._manual_diagnostics['summary'] = str(message)
            self._manual_diagnostics['current'] = ''
            self._manual_diagnostics['generation'] += 1

    def cancel_manual_diagnostics(self):
        with self._lock:
            goal_handle = self._manual_diagnostics_goal
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def _update_safety_supervisor_count(self):
        count = len(self.get_publishers_info_by_topic('/safety/status'))
        with self._lock:
            self._safety_supervisor_count = count

    def set_preview_listener(self, listener):
        """Set a lightweight callback used to wake the Qt GUI thread."""
        with self._lock:
            self._preview_listener = listener
            if listener is None:
                self._preview_notification_pending = False

    def select_preview_topic(self, topic):
        requested_topic = str(topic or '')
        with self._lock:
            if requested_topic == self._preview_topic:
                return
            old_subscription = self._preview_subscription
            self._preview_subscription = None
            self._preview_topic = requested_topic
            self._preview_generation += 1
            generation = self._preview_generation
            self._latest_preview = None
            self._preview_sequence += 1
            self._preview_notification_pending = False

        if old_subscription is not None:
            self.destroy_subscription(old_subscription)
        if not requested_topic:
            return

        subscription = self.create_subscription(
            CompressedImage,
            requested_topic,
            lambda msg, current=generation: self._preview_callback(
                msg, current),
            self.preview_qos,
            callback_group=self._preview_callback_group,
        )
        with self._lock:
            if generation == self._preview_generation:
                self._preview_subscription = subscription
                return
        self.destroy_subscription(subscription)

    def select_preview_source(self, topic):
        self.preview_source_pub.publish(String(data=str(topic or '')))

    def _preview_callback(self, msg, generation):
        listener = None
        with self._lock:
            if generation != self._preview_generation:
                return
            self._latest_preview = msg
            self._preview_sequence += 1
            # Queue at most one Qt notification. If Qt is temporarily busy,
            # subsequent ROS frames replace the pending frame instead of
            # building a stale-frame backlog.
            if (self._preview_listener is not None
                    and not self._preview_notification_pending):
                self._preview_notification_pending = True
                listener = self._preview_listener
        if listener is not None:
            listener()

    def preview_snapshot(self):
        with self._lock:
            return (
                self._latest_preview,
                self._preview_sequence,
                self._preview_topic,
            )

    def take_preview_snapshot(self):
        """Return the newest frame and allow one future GUI notification."""
        with self._lock:
            self._preview_notification_pending = False
            return (
                self._latest_preview,
                self._preview_sequence,
                self._preview_topic,
            )

    def publish_drive_input(self, state):
        msg = DriveInput()
        msg.forward = bool(state.get('forward'))
        msg.reverse = bool(state.get('reverse'))
        msg.steer_left = bool(state.get('left'))
        msg.steer_right = bool(state.get('right'))
        msg.reset = bool(state.get('reset'))
        msg.emergency_stop = bool(state.get('emergency_stop'))
        self.drive_pub.publish(msg)

    def publish_heartbeat(self, source):
        msg = GuiHeartbeat()
        msg.requested_source = source
        msg.armed = source in ('manual', 'auto')
        self.heartbeat_pub.publish(msg)

    def publish_fallback_stop(self):
        self.fallback_stop_pub.publish(
            MotionCommand(steering=0, left_speed=0, right_speed=0))

    def request_emergency_stop(self):
        self._call_trigger(self.estop_client, 'Emergency stop')

    def request_estop_reset(self):
        self._call_trigger(self.reset_client, 'E-stop reset')

    def _call_trigger(self, client, label):
        if not client.service_is_ready():
            with self._lock:
                self._events.append((False, f'{label} service is unavailable'))
            return
        future = client.call_async(Trigger.Request())

        def done(completed):
            try:
                response = completed.result()
                event = (response.success, response.message)
            except Exception as exc:
                event = (False, f'{label} failed: {exc}')
            with self._lock:
                self._events.append(event)

        future.add_done_callback(done)
