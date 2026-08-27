"""Qt process management for ROS 2 nodes owned by the GUI."""

from dataclasses import dataclass, field
import os
import signal
import time

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch_pkg.rmw_profile import profile_env
from PyQt5.QtCore import (
    pyqtSignal,
    QObject,
    QProcess,
    QProcessEnvironment,
    QTimer,
)
import yaml


def is_safety_supervisor_command(arguments):
    """Return whether argv belongs to this package's safety supervisor."""
    if not arguments:
        return False

    for index in range(len(arguments) - 3):
        if (os.path.basename(arguments[index]) == 'ros2'
                and arguments[index + 1:index + 4] == [
                    'run', 'autosdv_gui_pkg', 'safety_supervisor_node']):
            return True

    return any(
        os.path.basename(argument) == 'safety_supervisor_node'
        and 'autosdv_gui_pkg' in argument
        for argument in arguments
    )


def _matching_safety_supervisor_pids():
    matches = []
    current_pid = os.getpid()
    try:
        entries = os.listdir('/proc')
    except OSError:
        return matches

    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == current_pid:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as stream:
                arguments = [
                    item.decode('utf-8', errors='replace')
                    for item in stream.read().split(b'\0') if item
                ]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if is_safety_supervisor_command(arguments):
            matches.append(pid)
    return matches


def terminate_stale_safety_supervisors(timeout_sec=1.0):
    """Stop supervisors left behind by a previous GUI process."""
    stale_pids = _matching_safety_supervisor_pids()
    for pid in stale_pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

    deadline = time.monotonic() + timeout_sec
    remaining = _matching_safety_supervisor_pids()
    while remaining and time.monotonic() < deadline:
        time.sleep(0.05)
        remaining = _matching_safety_supervisor_pids()

    original = set(stale_pids)
    remaining = [pid for pid in remaining if pid in original]
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass

    if remaining:
        time.sleep(0.05)
    final_remaining = [
        pid for pid in _matching_safety_supervisor_pids() if pid in original
    ]
    return len(original) - len(final_remaining), final_remaining


@dataclass
class NodeSpec:
    node_id: str
    label: str
    package: str
    executable: str
    node_name: str
    dependencies: list = field(default_factory=list)
    parameters: dict = field(default_factory=dict)
    dds_profile: str = ''


def load_node_specs():
    share = get_package_share_directory('autosdv_gui_pkg')
    path = os.path.join(share, 'config', 'nodes.yaml')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    specs = {}
    for node_id, values in data['nodes'].items():
        specs[node_id] = NodeSpec(
            node_id=node_id,
            label=values['label'],
            package=values['package'],
            executable=values['executable'],
            node_name=values['node_name'],
            dependencies=list(values.get('dependencies', [])),
            parameters=dict(values.get('parameters', {})),
            dds_profile=values.get('dds_profile', ''),
        )
    return specs


def _parameter_text(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return '[' + ','.join(str(item) for item in value) + ']'
    return str(value)


class NodeProcessManager(QObject):
    state_changed = pyqtSignal(str, str)
    process_finished = pyqtSignal(str, bool, int)
    process_error = pyqtSignal(str, str)

    def __init__(self, specs, parent=None):
        super().__init__(parent)
        self.specs = specs
        self.processes = {}
        self.states = {node_id: 'STOPPED' for node_id in specs}
        self._expected_stop = set()
        self._last_stderr = {}
        self.process_groups = {}
        package_prefix = get_package_prefix('autosdv_gui_pkg')
        candidate_root = os.path.dirname(os.path.dirname(package_prefix))
        self.workspace_root = (
            candidate_root if os.path.isdir(
                os.path.join(candidate_root, 'src')) else os.getcwd()
        )

    def state(self, node_id):
        return self.states.get(node_id, 'STOPPED')

    def is_running(self, node_id):
        return self.state(node_id) in ('STARTING', 'RUNNING', 'STOPPING')

    def missing_dependencies(self, node_id):
        return [
            dependency for dependency in self.specs[node_id].dependencies
            if self.state(dependency) != 'RUNNING'
        ]

    def running_dependents(self, node_id):
        return [
            key for key, spec in self.specs.items()
            if node_id in spec.dependencies and self.is_running(key)
        ]

    def _set_state(self, node_id, state):
        self.states[node_id] = state
        self.state_changed.emit(node_id, state)

    def start(self, node_id):
        if self.is_running(node_id):
            return False, f'{self.specs[node_id].label} is already running'
        missing = self.missing_dependencies(node_id)
        if missing:
            labels = ', '.join(self.specs[item].label for item in missing)
            return False, f'Start required node first: {labels}'

        spec = self.specs[node_id]
        process = QProcess(self)
        process.setWorkingDirectory(self.workspace_root)
        process.setProcessChannelMode(QProcess.SeparateChannels)
        environment = QProcessEnvironment.systemEnvironment()
        if spec.dds_profile:
            launch_share = get_package_share_directory('launch_pkg')
            # nodes.yaml names the Fast DDS profile; rmw_profile maps it to the
            # selected middleware's equivalent.
            for name, value in profile_env(launch_share, spec.dds_profile).items():
                environment.insert(name, value)
        process.setProcessEnvironment(environment)

        arguments = [
            'run', spec.package, spec.executable,
            '--ros-args', '-r', f'__node:={spec.node_name}',
        ]
        for name, value in spec.parameters.items():
            arguments.extend(['-p', f'{name}:={_parameter_text(value)}'])

        process.started.connect(
            lambda key=node_id: self._on_started(key))
        process.finished.connect(
            lambda code, status, key=node_id: self._on_finished(key, code))
        process.errorOccurred.connect(
            lambda error, key=node_id: self._on_error(key, error))
        process.readyReadStandardError.connect(
            lambda key=node_id: self._read_stderr(key))

        self.processes[node_id] = process
        self._expected_stop.discard(node_id)
        self._set_state(node_id, 'STARTING')
        # Give every managed node its own Unix process group.  `ros2 run`
        # starts the actual node as a child process, so signaling only the
        # QProcess PID can otherwise leave the ROS node orphaned.
        process.start('setsid', ['ros2', *arguments])
        return True, ''

    def _on_started(self, node_id):
        process = self.processes.get(node_id)
        if process is None:
            return
        self.process_groups[node_id] = int(process.processId())
        self._set_state(node_id, 'RUNNING')

    def stop(self, node_id):
        process = self.processes.get(node_id)
        process_group = self.process_groups.get(node_id)
        group_running = self._process_group_exists(process_group)
        if ((process is None or process.state() == QProcess.NotRunning)
                and not group_running):
            self._set_state(node_id, 'STOPPED')
            return True, ''
        dependents = self.running_dependents(node_id)
        if dependents:
            labels = ', '.join(self.specs[item].label for item in dependents)
            return False, f'Stop dependent node first: {labels}'
        self._expected_stop.add(node_id)
        self._set_state(node_id, 'STOPPING')
        if not self._signal_process_group(node_id, signal.SIGINT):
            if process is not None:
                process.terminate()
        QTimer.singleShot(
            2500,
            lambda key=node_id, group=process_group, owned=process:
            self._ensure_process_group_stopped(key, group, owned),
        )
        return True, ''

    def _signal_process_group(self, node_id, requested_signal):
        process_group = self.process_groups.get(node_id)
        if process_group is None:
            return False
        try:
            os.killpg(process_group, requested_signal)
            return True
        except (ProcessLookupError, PermissionError):
            return False

    @staticmethod
    def _process_group_exists(process_group):
        if process_group is None:
            return False
        try:
            os.killpg(process_group, 0)
            return True
        except (ProcessLookupError, PermissionError):
            return False

    def _ensure_process_group_stopped(self, node_id, process_group, process):
        if self._process_group_exists(process_group):
            try:
                os.killpg(process_group, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        if (process is not None
                and self.processes.get(node_id) is process
                and process.state() != QProcess.NotRunning):
            process.kill()
        if self.process_groups.get(node_id) == process_group:
            self.process_groups.pop(node_id, None)

    def set_ros_parameter(self, node_id, parameter, value):
        if self.state(node_id) != 'RUNNING':
            return False
        spec = self.specs[node_id]
        process = QProcess(self)
        process.finished.connect(process.deleteLater)
        process.start(
            'ros2',
            ['param', 'set', f'/{spec.node_name}', parameter,
             _parameter_text(value)],
        )
        return True

    def _read_stderr(self, node_id):
        process = self.processes.get(node_id)
        if process is None:
            return
        text = bytes(process.readAllStandardError()).decode(
            'utf-8', errors='replace').strip()
        if text:
            self._last_stderr[node_id] = text.splitlines()[-1]

    def _on_finished(self, node_id, exit_code):
        expected = node_id in self._expected_stop
        process_group = self.process_groups.get(node_id)
        if self._process_group_exists(process_group):
            self._set_state(node_id, 'STOPPING' if expected else 'ERROR')
            QTimer.singleShot(
                100,
                lambda key=node_id, planned=expected, code=exit_code,
                group=process_group:
                self._await_process_group_exit(
                    key, planned, code, group, 0),
            )
            return
        self._finalize_finished(node_id, expected, exit_code)

    def _await_process_group_exit(
            self, node_id, expected, exit_code, process_group, attempt):
        if self._process_group_exists(process_group):
            if attempt >= 25:
                try:
                    os.killpg(process_group, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
            QTimer.singleShot(
                100,
                lambda key=node_id, planned=expected, code=exit_code,
                group=process_group, count=attempt + 1:
                self._await_process_group_exit(
                    key, planned, code, group, count),
            )
            return
        self._finalize_finished(node_id, expected, exit_code)

    def _finalize_finished(self, node_id, expected, exit_code):
        self._expected_stop.discard(node_id)
        self.process_groups.pop(node_id, None)
        self._set_state(node_id, 'STOPPED' if expected else 'ERROR')
        if not expected and exit_code != 0:
            detail = self._last_stderr.get(
                node_id, f'Process exited with code {exit_code}')
            self.process_error.emit(node_id, detail)
        self.process_finished.emit(node_id, expected, exit_code)
        process = self.processes.pop(node_id, None)
        if process is not None:
            process.deleteLater()

    def _on_error(self, node_id, error):
        process = self.processes.get(node_id)
        detail = process.errorString() if process is not None else str(error)
        self._set_state(node_id, 'ERROR')
        self.process_error.emit(node_id, detail)

    def shutdown(self, order, timeout_ms=1500):
        for node_id in order:
            process = self.processes.get(node_id)
            process_group = self.process_groups.get(node_id)
            group_running = self._process_group_exists(process_group)
            if ((process is None or process.state() == QProcess.NotRunning)
                    and not group_running):
                continue
            self._expected_stop.add(node_id)
            if not self._signal_process_group(node_id, signal.SIGINT):
                if process is not None:
                    process.terminate()
            if (process is not None
                    and process.state() != QProcess.NotRunning
                    and not process.waitForFinished(timeout_ms)):
                if not self._signal_process_group(node_id, signal.SIGKILL):
                    process.kill()
                process.waitForFinished(500)
            self._ensure_process_group_stopped(
                node_id, process_group, process)
