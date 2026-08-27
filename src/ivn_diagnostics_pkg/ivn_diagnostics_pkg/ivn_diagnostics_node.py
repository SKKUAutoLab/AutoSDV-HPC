"""Continuous IVN monitor and on-demand end-to-end diagnostic action."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import fcntl
import os
import re
import subprocess
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from interfaces_pkg.action import RunIvnDiagnostics
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class ZoneConfig:
    zone_id: str
    name: str
    ip: str
    camera_topics: tuple


@dataclass(frozen=True)
class HealthRecord:
    received_at: float
    statuses: tuple
    sequence: int = 0


LEVEL_PRIORITY = {
    0: 0,
    1: 1,
    3: 2,
    2: 3,
}


def level_number(level):
    """Normalize ROS byte fields represented as int or one-byte bytes."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    return int(level)


def worst_level(levels):
    """Return diagnostic severity using ERROR > STALE > WARN > OK."""
    values = list(levels)
    if not values:
        return DiagnosticStatus.STALE
    selected = max(values, key=lambda level: LEVEL_PRIORITY.get(
        level_number(level), LEVEL_PRIORITY[3]))
    return {
        0: DiagnosticStatus.OK,
        1: DiagnosticStatus.WARN,
        2: DiagnosticStatus.ERROR,
        3: DiagnosticStatus.STALE,
    }.get(level_number(selected), DiagnosticStatus.STALE)


def _status_values(status):
    return {item.key: item.value for item in status.values}


def status_zone_id(status, expected_zone_ids):
    """Read a valid Zone ID from KeyValue first, then hardware_id."""
    expected = set(expected_zone_ids)
    value_zone = _status_values(status).get('zone_id', '')
    if value_zone in expected:
        return value_zone
    hardware_prefix = 'zcu-'
    if status.hardware_id.startswith(hardware_prefix):
        hardware_zone = status.hardware_id[len(hardware_prefix):]
        if hardware_zone in expected:
            return hardware_zone
    return None


def split_health_statuses(message, expected_zone_ids):
    """Group one shared health-topic message by known Zone ID."""
    grouped = {}
    for status in message.status:
        zone_id = status_zone_id(status, expected_zone_ids)
        if zone_id is not None:
            grouped.setdefault(zone_id, []).append(status)
    return {zone_id: tuple(statuses)
            for zone_id, statuses in grouped.items()}


def aggregate_health_status(zone, record, now, timeout_sec, scope='automatic'):
    """Convert a cached ZCU health report into one Zone-level status."""
    common = {
        'scope': scope,
        'check': 'zcu_health_heartbeat',
        'zone_id': zone.zone_id,
        'zone_name': zone.name,
        'ip': zone.ip,
        'timeout_sec': timeout_sec,
    }
    if record is None:
        return make_status(
            DiagnosticStatus.STALE,
            f'IVN/ZCU/{zone.zone_id}/Health',
            'No ZCU health message received',
            f'zcu-{zone.zone_id}',
            age_sec='unknown',
            **common,
        )

    age = max(0.0, now - record.received_at)
    if age > timeout_sec:
        return make_status(
            DiagnosticStatus.STALE,
            f'IVN/ZCU/{zone.zone_id}/Health',
            f'ZCU health heartbeat stale ({age:.2f} s)',
            f'zcu-{zone.zone_id}',
            age_sec=f'{age:.3f}',
            sequence=record.sequence,
            **common,
        )

    level = worst_level(status.level for status in record.statuses)
    summaries = [
        status for status in record.statuses
        if status.name.endswith('/Summary')
    ]
    matching_summaries = [
        status for status in summaries
        if level_number(status.level) == level_number(level)
    ]
    representative = matching_summaries[0] if matching_summaries else (
        max(
            record.statuses,
            key=lambda status: LEVEL_PRIORITY.get(
                level_number(status.level), LEVEL_PRIORITY[3]),
        ) if record.statuses else None
    )
    message = (
        representative.message if representative is not None
        else 'ZCU health message contained no valid statuses'
    )
    component_errors = sum(
        level_number(status.level) == 2 for status in record.statuses)
    return make_status(
        level,
        f'IVN/ZCU/{zone.zone_id}/Health',
        message,
        f'zcu-{zone.zone_id}',
        age_sec=f'{age:.3f}',
        sequence=record.sequence,
        component_count=len(record.statuses),
        component_errors=component_errors,
        **common,
    )


def health_component_status(record, now, timeout_sec, component,
                            component_id=None):
    """Return one fresh ZCU component status, or why it is unavailable."""
    if record is None:
        return DiagnosticStatus.STALE, 'No ZCU health message received'

    age = max(0.0, now - record.received_at)
    if age > timeout_sec:
        return (
            DiagnosticStatus.STALE,
            f'ZCU health heartbeat stale ({age:.2f} s)',
        )

    expected_component = component.lower()
    expected_id = component_id.lower() if component_id else None
    for status in record.statuses:
        values = _status_values(status)
        reported_component = values.get('component', '').lower()
        if reported_component != expected_component:
            continue
        if expected_id is not None:
            reported_id = values.get(f'{expected_component}_id', '').lower()
            if reported_id != expected_id:
                continue
        return status.level, status.message

    description = component
    if component_id:
        description = f'{component} {component_id}'
    return DiagnosticStatus.STALE, f'{description} health was not reported'


def _key_values(values):
    return [KeyValue(key=str(key), value=str(value))
            for key, value in values.items()]


def make_status(level, name, message, hardware_id, **values):
    return DiagnosticStatus(
        level=level,
        name=name,
        message=message,
        hardware_id=hardware_id,
        values=_key_values(values),
    )


def build_automatic_statuses(zones, ping_results, publisher_counts,
                             health_records, now, health_timeout_sec):
    """Build the lightweight periodic status without sampling topic payloads."""
    statuses = []
    for zone, reachable in zip(zones, ping_results):
        common = {
            'scope': 'automatic',
            'zone_id': zone.zone_id,
            'zone_name': zone.name,
            'ip': zone.ip,
        }
        if reachable is None:
            network_level = DiagnosticStatus.STALE
            network_message = 'Waiting for first reachability probe'
        elif reachable:
            network_level = DiagnosticStatus.OK
            network_message = 'ZCU reachable'
        else:
            network_level = DiagnosticStatus.ERROR
            network_message = 'ZCU unreachable'
        statuses.append(make_status(
            network_level,
            f'IVN/ZCU/{zone.zone_id}/Network',
            network_message,
            f'zcu-{zone.zone_id}',
            check='icmp_reachability',
            **common,
        ))

        discovered = [publisher_counts.get(topic, 0)
                      for topic in zone.camera_topics]
        if reachable is None:
            dds_level = DiagnosticStatus.STALE
            dds_message = 'Waiting for ZCU reachability'
        elif not reachable:
            dds_level = DiagnosticStatus.STALE
            dds_message = 'Not checked because ZCU is unreachable'
        elif all(count > 0 for count in discovered):
            dds_level = DiagnosticStatus.OK
            dds_message = 'Expected camera publishers discovered'
        else:
            dds_level = DiagnosticStatus.ERROR
            dds_message = 'One or more camera publishers are missing'
        statuses.append(make_status(
            dds_level,
            f'IVN/ZCU/{zone.zone_id}/DDS',
            dds_message,
            f'zcu-{zone.zone_id}',
            check='dds_discovery',
            **common,
        ))

        runtime_level, runtime_message = health_component_status(
            health_records.get(zone.zone_id),
            now,
            health_timeout_sec,
            'runtime',
        )
        statuses.append(make_status(
            runtime_level,
            f'IVN/ZCU/{zone.zone_id}/Runtime',
            runtime_message,
            f'zcu-{zone.zone_id}',
            check='zcu_runtime_health',
            **common,
        ))

        for topic, count in zip(zone.camera_topics, discovered):
            image_id = topic.strip('/').replace('_raw', '')
            if reachable is None or not reachable:
                level = DiagnosticStatus.STALE
                message = 'Publisher discovery deferred'
            elif count > 0:
                level = DiagnosticStatus.OK
                message = f'{count} publisher(s) discovered'
            else:
                level = DiagnosticStatus.ERROR
                message = 'No publisher discovered'

            health_level = DiagnosticStatus.STALE
            health_message = 'ZCU health deferred until reachable'
            if reachable:
                health_level, health_message = health_component_status(
                    health_records.get(zone.zone_id),
                    now,
                    health_timeout_sec,
                    'camera',
                    image_id,
                )
                discovery_level = level
                level = worst_level((level, health_level))
                discovery_priority = LEVEL_PRIORITY.get(
                    level_number(discovery_level), LEVEL_PRIORITY[3])
                health_priority = LEVEL_PRIORITY.get(
                    level_number(health_level), LEVEL_PRIORITY[3])
                if health_priority > discovery_priority:
                    message = health_message
                elif health_priority == discovery_priority:
                    if level_number(health_level) != DiagnosticStatus.OK:
                        message = f'{message}; {health_message}'
            statuses.append(make_status(
                level,
                f'IVN/Camera/{image_id}',
                message,
                f'camera-{image_id}',
                scope='automatic',
                check='publisher_discovery_and_zcu_health',
                zone_id=zone.zone_id,
                topic=topic,
                publisher_count=count,
                zcu_health_level=level_number(health_level),
                zcu_health_message=health_message,
            ))
        statuses.append(aggregate_health_status(
            zone,
            health_records.get(zone.zone_id),
            now,
            health_timeout_sec,
        ))

    summary_level = worst_level(status.level for status in statuses)
    if summary_level == DiagnosticStatus.ERROR:
        summary_level = DiagnosticStatus.ERROR
        summary_message = 'One or more IVN components require attention'
    elif summary_level == DiagnosticStatus.STALE:
        summary_level = DiagnosticStatus.STALE
        summary_message = 'One or more IVN states are stale'
    elif summary_level == DiagnosticStatus.WARN:
        summary_level = DiagnosticStatus.WARN
        summary_message = 'IVN is reachable with warnings'
    else:
        summary_level = DiagnosticStatus.OK
        summary_message = 'IVN reachability and discovery are healthy'
    statuses.insert(0, make_status(
        summary_level,
        'IVN/Summary',
        summary_message,
        'ivn-monitor',
        scope='automatic',
        check='summary',
    ))
    return statuses


def build_manual_checks(zones, check_data_flow):
    """Return the on-demand plan; obsolete ECU feedback is not included."""
    checks = [('network', zone, None) for zone in zones]
    if check_data_flow:
        checks.extend(
            ('camera_data', zone, topic)
            for zone in zones for topic in zone.camera_topics
        )
    checks.extend(('zcu_health', zone, None) for zone in zones)
    return checks


class IvnDiagnosticsNode(Node):
    """Publish automatic discovery status and serve manual deep checks."""

    def __init__(self):
        super().__init__('IVN_diagnostics_node')
        self._instance_lock_fd = os.open(
            f'/tmp/autosdv_ivn_diagnostics_{os.getuid()}.lock',
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
                'Another IVN diagnostics node is already running') from exc

        zone_names = list(self.declare_parameter(
            'zone_names', [
                'Front Left', 'Front Right', 'Rear Left', 'Rear Right',
            ]).value)
        zcu_ips = list(self.declare_parameter(
            'zcu_ips', [
                '10.0.0.2', '10.0.0.3', '10.0.0.4', '10.0.0.5',
            ]).value)
        camera_topics = list(self.declare_parameter(
            'camera_topics', [
                '/image_01_raw', '/image_02_raw', '/image_03_raw',
                '/image_04_raw', '/image_05_raw',
            ]).value)
        camera_zone_indices = list(self.declare_parameter(
            'camera_zone_indices', [0, 0, 1, 2, 3]).value)
        expected_zone_ids = list(self.declare_parameter(
            'expected_zone_ids', [
                'zone_1', 'zone_2', 'zone_3', 'zone_4',
            ]).value)
        if len(zone_names) != len(zcu_ips):
            raise ValueError('zone_names and zcu_ips must have equal lengths')
        if len(expected_zone_ids) != len(zcu_ips):
            raise ValueError(
                'expected_zone_ids and zcu_ips must have equal lengths')
        if len(set(expected_zone_ids)) != len(expected_zone_ids):
            raise ValueError('expected_zone_ids must be unique')
        invalid_zone_ids = [
            zone_id for zone_id in expected_zone_ids
            if re.fullmatch(r'zone_[1-9][0-9]*', str(zone_id)) is None
        ]
        if invalid_zone_ids:
            raise ValueError(
                'Invalid expected_zone_ids: ' + ', '.join(invalid_zone_ids))
        if len(camera_topics) != len(camera_zone_indices):
            raise ValueError(
                'camera_topics and camera_zone_indices must have equal lengths')

        cameras_by_zone = [[] for _ in zcu_ips]
        for topic, zone_index in zip(camera_topics, camera_zone_indices):
            index = int(zone_index)
            if index < 0 or index >= len(cameras_by_zone):
                raise ValueError(f'Invalid camera zone index: {index}')
            cameras_by_zone[index].append(str(topic))
        self.zones = tuple(
            ZoneConfig(
                zone_id=str(expected_zone_ids[index]),
                name=str(name),
                ip=str(ip),
                camera_topics=tuple(cameras_by_zone[index]),
            )
            for index, (name, ip) in enumerate(zip(zone_names, zcu_ips))
        )
        self.expected_zone_ids = tuple(
            zone.zone_id for zone in self.zones)
        self.zcu_health_topic = str(self.declare_parameter(
            'zcu_health_topic', '/ivn/zcu_health').value)
        self.zcu_health_timeout = float(self.declare_parameter(
            'zcu_health_timeout_sec', 2.0).value)
        self.monitor_period = float(self.declare_parameter(
            'monitor_period_sec', 1.0).value)
        self.manual_topic_timeout = float(self.declare_parameter(
            'manual_topic_timeout_sec', 3.0).value)
        diagnostics_topic = str(self.declare_parameter(
            'diagnostics_topic', '/ivn/diagnostics/status').value)
        action_name = str(self.declare_parameter(
            'manual_action_name', '/ivn/diagnostics/run_manual').value)

        self._probe_pool = ThreadPoolExecutor(
            max_workers=max(1, len(self.zones)))
        self._ping_futures = [None] * len(self.zones)
        self._ping_results = [None] * len(self.zones)
        self._manual_lock = threading.Lock()
        self._manual_active = False
        self._health_condition = threading.Condition()
        self._health_records = {}

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.status_pub = self.create_publisher(
            DiagnosticArray, diagnostics_topic, status_qos)
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(
            DiagnosticArray,
            self.zcu_health_topic,
            self._zcu_health_callback,
            health_qos,
            callback_group=ReentrantCallbackGroup(),
        )
        action_group = ReentrantCallbackGroup()
        self.manual_action = ActionServer(
            self,
            RunIvnDiagnostics,
            action_name,
            execute_callback=self._execute_manual_check,
            goal_callback=self._manual_goal_callback,
            cancel_callback=self._manual_cancel_callback,
            callback_group=action_group,
        )
        self.create_timer(self.monitor_period, self._automatic_monitor_tick)
        self.get_logger().info(
            f'IVN automatic monitor: {diagnostics_topic}; '
            f'ZCU health: {self.zcu_health_topic}; '
            f'manual deep check: {action_name}')

    @staticmethod
    def _ping(ip, count=1):
        try:
            result = subprocess.run(
                ['ping', '-c', str(count), '-W', '1', ip],
                capture_output=True,
                text=True,
                timeout=max(1.5, count * 1.2),
                check=False,
            )
            match = re.search(r'= [\d.]+/([\d.]+)/', result.stdout)
            if match is None:
                match = re.search(r'time=([\d.]+)\s*ms', result.stdout)
            detail = f'{match.group(1)} ms' if match else ''
            return result.returncode == 0, detail
        except FileNotFoundError:
            return False, 'ping command not found'
        except subprocess.TimeoutExpired:
            return False, 'timeout'

    def _publisher_count(self, topic):
        return len(self.get_publishers_info_by_topic(topic))

    def _zcu_health_callback(self, message):
        grouped = split_health_statuses(message, self.expected_zone_ids)
        if not grouped:
            return
        received_at = time.monotonic()
        with self._health_condition:
            for zone_id, statuses in grouped.items():
                previous = self._health_records.get(zone_id)
                sequence = 1 if previous is None else previous.sequence + 1
                self._health_records[zone_id] = HealthRecord(
                    received_at=received_at,
                    statuses=statuses,
                    sequence=sequence,
                )
            self._health_condition.notify_all()

    def _health_snapshot(self):
        with self._health_condition:
            return dict(self._health_records)

    def _automatic_monitor_tick(self):
        for index, zone in enumerate(self.zones):
            future = self._ping_futures[index]
            if future is not None and future.done():
                try:
                    self._ping_results[index] = bool(future.result()[0])
                except Exception as exc:  # pragma: no cover - defensive log
                    self.get_logger().warn(
                        f'Automatic probe failed for {zone.ip}: {exc}')
                    self._ping_results[index] = False
                self._ping_futures[index] = None
            if self._ping_futures[index] is None:
                self._ping_futures[index] = self._probe_pool.submit(
                    self._ping, zone.ip)

        topics = {
            topic for zone in self.zones for topic in zone.camera_topics
        }
        counts = {topic: self._publisher_count(topic) for topic in topics}
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = build_automatic_statuses(
            self.zones,
            self._ping_results,
            counts,
            self._health_snapshot(),
            time.monotonic(),
            self.zcu_health_timeout,
        )
        self.status_pub.publish(message)

    def _manual_goal_callback(self, goal_request):
        del goal_request
        with self._manual_lock:
            if self._manual_active:
                self.get_logger().warn('Manual IVN check already running')
                return GoalResponse.REJECT
            self._manual_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _manual_cancel_callback(goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _sample_topic(self, topic, best_effort):
        timeout = self.manual_topic_timeout
        command = [
            'timeout', str(timeout), 'ros2', 'topic', 'echo', topic, '--once',
        ]
        if best_effort:
            command.extend(['--qos-reliability', 'best_effort'])
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=timeout + 1.0,
                check=False,
            )
            return result.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    @staticmethod
    def _publish_feedback(goal_handle, completed, total, current):
        feedback = RunIvnDiagnostics.Feedback()
        feedback.completed_checks = completed
        feedback.total_checks = total
        feedback.current_check = current
        goal_handle.publish_feedback(feedback)

    def _wait_for_fresh_zone_health(
            self, zone_id, baseline_sequence, goal_handle):
        deadline = time.monotonic() + self.zcu_health_timeout
        with self._health_condition:
            while True:
                record = self._health_records.get(zone_id)
                if (record is not None
                        and record.sequence > baseline_sequence):
                    return record
                if goal_handle.is_cancel_requested:
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._health_condition.wait(timeout=min(remaining, 0.1))

    def _execute_manual_check(self, goal_handle):
        result = RunIvnDiagnostics.Result()
        statuses = []
        with self._health_condition:
            health_baselines = {
                zone.zone_id: (
                    self._health_records[zone.zone_id].sequence
                    if zone.zone_id in self._health_records else 0
                )
                for zone in self.zones
            }
        checks = build_manual_checks(
            self.zones, goal_handle.request.check_data_flow)
        total = len(checks)
        try:
            for completed, (kind, zone, topic) in enumerate(checks):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.summary = 'Manual IVN deep check canceled'
                    result.results = statuses
                    return result

                if kind == 'network':
                    label = f'{zone.name} network round-trip'
                elif kind == 'camera_data':
                    label = f'{topic} data sample'
                else:
                    label = f'{zone.name} fresh ZCU health heartbeat'
                self._publish_feedback(goal_handle, completed, total, label)

                if kind == 'network':
                    ok, detail = self._ping(zone.ip, count=3)
                    statuses.append(make_status(
                        DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR,
                        f'IVN/Manual/{zone.zone_id}/Network',
                        ('Reachable' + (f' ({detail})' if detail else '')
                         if ok else f'Unreachable{": " + detail if detail else ""}'),
                        f'zcu-{zone.zone_id}',
                        scope='manual',
                        check='three_packet_round_trip',
                        zone_id=zone.zone_id,
                        ip=zone.ip,
                    ))
                elif kind == 'camera_data':
                    ok = self._sample_topic(topic, best_effort=True)
                    image_id = topic.strip('/').replace('_raw', '')
                    statuses.append(make_status(
                        DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR,
                        f'IVN/Manual/Camera/{image_id}',
                        ('Data sample received' if ok
                         else 'No data sample received before timeout'),
                        f'camera-{image_id}',
                        scope='manual',
                        check='payload_sample',
                        zone_id=zone.zone_id,
                        topic=topic,
                    ))
                else:
                    record = self._wait_for_fresh_zone_health(
                        zone.zone_id,
                        health_baselines[zone.zone_id],
                        goal_handle,
                    )
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.success = False
                        result.summary = 'Manual IVN deep check canceled'
                        result.results = statuses
                        return result
                    if record is None:
                        health_status = make_status(
                            DiagnosticStatus.STALE,
                            f'IVN/Manual/ZCU/{zone.zone_id}/Health',
                            'No fresh Zone health heartbeat received',
                            f'zcu-{zone.zone_id}',
                            scope='manual',
                            check='fresh_zcu_health_heartbeat',
                            zone_id=zone.zone_id,
                            timeout_sec=self.zcu_health_timeout,
                        )
                    else:
                        health_status = aggregate_health_status(
                            zone,
                            record,
                            time.monotonic(),
                            self.zcu_health_timeout,
                            scope='manual',
                        )
                        health_status.name = (
                            f'IVN/Manual/ZCU/{zone.zone_id}/Health')
                        for value in health_status.values:
                            if value.key == 'check':
                                value.value = 'fresh_zcu_health_heartbeat'
                    statuses.append(health_status)
                self._publish_feedback(
                    goal_handle, completed + 1, total, label)

            failures = sum(
                level_number(status.level) in (
                    2, 3,
                )
                for status in statuses)
            result.success = failures == 0
            result.summary = (
                f'{len(statuses)}/{len(statuses)} manual checks passed'
                if not failures
                else f'{failures} of {len(statuses)} manual checks failed'
            )
            result.results = statuses
            goal_handle.succeed()
            return result
        finally:
            with self._manual_lock:
                self._manual_active = False

    def destroy_node(self):
        self.manual_action.destroy()
        self._probe_pool.shutdown(wait=False, cancel_futures=True)
        if self._instance_lock_fd is not None:
            os.close(self._instance_lock_fd)
            self._instance_lock_fd = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = IvnDiagnosticsNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
