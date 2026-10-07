import os
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from diagnostic_msgs.msg import DiagnosticStatus
import pytest

from autosdv_gui_pkg.main import (
    compressed_image_to_qimage,
    configure_default_dds_profile,
    MainWindow,
    remove_opencv_qt_plugin_paths,
    ros_image_to_qimage,
    ZCU_LAYOUT_ZONE_IDS,
)
from autosdv_gui_pkg.process_manager import (
    is_safety_supervisor_command,
    load_node_specs,
)
from autosdv_gui_pkg.ros_bridge import GuiRosBridge
from launch_pkg.rmw_profile import image_profile_env
from autosdv_gui_pkg.safety_supervisor_node import SafetySupervisorNode
from data_collection_pkg.drive_node import DriveController
from interfaces_pkg.msg import DriveInput, MotionCommand
from PyQt5.QtCore import QBuffer, QIODevice
from PyQt5.QtWidgets import QMessageBox
from sensor_msgs.msg import CompressedImage, Image


@pytest.fixture(autouse=True)
def isolated_mode_state(monkeypatch, tmp_path):
    # Profile unit tests must not inherit the vehicle's persisted selection.
    monkeypatch.setattr('launch_pkg.communication_mode.STATE', tmp_path / 'mode.json')


def test_drive_controller_accelerates_and_decelerates():
    controller = DriveController(
        acceleration_rate=50.0,
        deceleration_rate=60.0,
        steering_rate=10.0,
    )
    controller.set_gui_input(DriveInput(forward=True))
    steering, speed = controller.update(0.1)
    assert steering == 0
    assert speed == 5

    controller.set_gui_input(DriveInput())
    _, speed = controller.update(0.1)
    assert speed == 0

    controller.set_gui_input(DriveInput(reverse=True, steer_left=True))
    steering, speed = controller.update(0.1)
    assert steering == -1
    assert speed == -5


def test_node_registry_enforces_control_dependencies():
    specs = load_node_specs()
    assert specs['yolo'].dependencies == ['camera_01']
    assert specs['lane'].dependencies == ['yolo']
    assert specs['path_planner'].dependencies == ['lane']
    assert specs['lane'].parameters['publish_roi_image'] is True
    assert specs['yolo_visualizer'].executable == 'yolov8_visualizer_node'
    assert specs['yolo_visualizer'].parameters['image_reliability'] == 2
    assert specs['path_visualizer'].executable == 'path_visualizer_node'
    assert specs['preview_bridge'].executable == 'preview_bridge_node'
    assert specs['motion_planner'].dependencies == ['path_planner']
    assert specs['drive'].dependencies == []
    assert specs['motion_planner'].parameters['pub_topic'] == '/control/auto_cmd'
    assert specs['drive'].parameters['output_topic'] == '/control/manual_cmd'
    assert specs['ivn_diagnostics'].package == 'ivn_diagnostics_pkg'
    assert specs['ivn_diagnostics'].executable == 'IVN_diagnostics_node'
    assert specs['ivn_diagnostics'].parameters['expected_zone_ids'] == [
        'zone_1', 'zone_2', 'zone_3', 'zone_4']
    assert specs['ivn_diagnostics'].dds_profile == (
        'fastdds_whitelist.xml')
    assert set(ZCU_LAYOUT_ZONE_IDS.values()) == {
        'zone_1', 'zone_2', 'zone_3', 'zone_4'}
    assert 'command_timeout_sec' not in specs['safety_supervisor'].parameters
    assert 'gui_heartbeat_timeout_sec' not in (
        specs['safety_supervisor'].parameters)


def test_graph_maps_camera_fault_to_camera_not_zcu():
    statuses = [
        DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='IVN/ZCU/zone_3/Network',
            hardware_id='zcu-zone_3',
        ),
        DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name='IVN/ZCU/zone_3/Runtime',
            hardware_id='zcu-zone_3',
        ),
        DiagnosticStatus(
            level=DiagnosticStatus.ERROR,
            name='IVN/ZCU/zone_3/Health',
            hardware_id='zcu-zone_3',
        ),
        DiagnosticStatus(
            level=DiagnosticStatus.ERROR,
            name='IVN/Camera/image_04',
            hardware_id='camera-image_04',
        ),
    ]

    zone_states, camera_states = MainWindow._graph_diagnostic_states(statuses)

    assert zone_states['zone_3'] == 'ok'
    assert camera_states['image_04'] == 'error'


def test_manual_drive_has_no_missing_pipeline_dependencies():
    from autosdv_gui_pkg.process_manager import NodeProcessManager

    manager = SimpleNamespace(specs=load_node_specs(), state=lambda _: 'STOPPED')
    assert NodeProcessManager.missing_dependencies(manager, 'drive') == []
    assert NodeProcessManager.missing_dependencies(manager, 'motion_planner') == [
        'path_planner']


@pytest.mark.parametrize('source,node,should_stop', [
    ('manual', 'camera_01', False), ('manual', 'lane', False),
    ('manual', 'drive', True), ('auto', 'camera_01', True),
    ('auto', 'lane', True),
])
def test_pipeline_crash_only_stops_relevant_control(source, node, should_stop):
    from unittest.mock import Mock

    window = SimpleNamespace(control_source=source, emergency_stop=Mock())
    MainWindow._process_finished(window, node, False, 1)
    assert window.emergency_stop.called is should_stop


def test_reset_centers_steering_without_resetting_speed():
    controller = DriveController(
        acceleration_rate=50.0,
        deceleration_rate=60.0,
        steering_rate=10.0,
    )
    controller.set_gui_input(DriveInput(forward=True, steer_left=True))
    steering, speed = controller.update(0.2)
    assert steering == -2
    assert speed == 10

    controller.set_gui_input(DriveInput(reset=True))
    steering, speed = controller.update(0.0)
    assert steering == 0
    assert speed == 10


def test_safety_supervisor_process_matching_is_scoped():
    assert is_safety_supervisor_command([
        '/opt/ros/jazzy/bin/ros2', 'run', 'autosdv_gui_pkg',
        'safety_supervisor_node', '--ros-args',
    ])
    assert is_safety_supervisor_command([
        '/workspace/install/autosdv_gui_pkg/lib/autosdv_gui_pkg/'
        'safety_supervisor_node', '--ros-args',
    ])
    assert not is_safety_supervisor_command([
        'ros2', 'run', 'some_other_pkg', 'safety_supervisor_node',
    ])
    assert not is_safety_supervisor_command([
        'ros2', 'run', 'autosdv_gui_pkg', 'autosdv_gui',
    ])


def test_safety_supervisor_owner_liveness_check():
    assert SafetySupervisorNode._owner_process_alive(os.getpid())
    assert not SafetySupervisorNode._owner_process_alive(2 ** 31 - 1)


def test_supervisor_does_not_timeout_selected_control_command():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    for source in ('manual', 'auto'):
        command = MotionCommand(
            steering=3,
            left_speed=77,
            right_speed=77,
        )
        publisher = Publisher()
        supervisor = SimpleNamespace(
            _lock=threading.Lock(),
            _requested_source=source,
            _gui_armed=True,
            _commands={'manual': command, 'auto': command},
            _estop_latched=False,
            _zcu_connected=[True] * 4,
            zcu_ips=['z1', 'z2', 'z3', 'z4'],
            output_pub=publisher,
            _zero_command=SafetySupervisorNode._zero_command,
        )

        SafetySupervisorNode._control_tick(supervisor)

        assert publisher.messages == [command]


@pytest.mark.parametrize('node_id,starts', [
    ('drive', True), ('motion_planner', False),
])
def test_start_with_zcus_offline(node_id, starts):
    window = SimpleNamespace(
        _cancel_estop_resume=Mock(),
        manager=SimpleNamespace(
            is_running=Mock(return_value=False),
            start=Mock(return_value=(True, '')),
        ),
        ros=SimpleNamespace(safety_supervisor_count=lambda: 1),
        latest_safety=SimpleNamespace(
            emergency_stop=False, zcu_connected=[False] * 4),
        _show_error=Mock(),
    )
    MainWindow._start_node(window, node_id)
    assert window.manager.start.called is starts
    assert window._show_error.called is not starts


@pytest.mark.parametrize('connected', [
    [False] * 4, [True, False, True, True], [True] * 4,
])
@pytest.mark.parametrize('source,armed,estop', [
    ('manual', True, False), ('auto', True, False),
    ('manual', True, True), ('manual', False, False),
])
def test_supervisor_zcu_gate_and_emergency_stop(connected, source, armed, estop):
    command = MotionCommand(steering=3, left_speed=77, right_speed=77)
    supervisor = SimpleNamespace(
        _lock=threading.Lock(), _requested_source=source, _gui_armed=armed,
        _commands={source: command}, _estop_latched=estop,
        _stop_ticks_remaining=0,
        _zcu_connected=connected, zcu_ips=['z1', 'z2', 'z3', 'z4'],
        output_pub=Mock(), _latch_estop=Mock(),
        _zero_command=SafetySupervisorNode._zero_command,
    )
    SafetySupervisorNode._control_tick(supervisor)
    zcu_fault = armed and source == 'auto' and not all(connected)
    expected = command if armed and not estop and not zcu_fault else MotionCommand()
    if armed or estop:
        supervisor.output_pub.publish.assert_called_once_with(expected)
    else:
        supervisor.output_pub.publish.assert_not_called()
    assert supervisor._latch_estop.called is zcu_fault


@pytest.mark.parametrize('source', ['manual', 'auto'])
def test_supervisor_stop_burst_then_silence_and_restart(source):
    command = MotionCommand(steering=3, left_speed=77, right_speed=77)
    supervisor = SimpleNamespace(
        _lock=threading.Lock(), _requested_source='none', _gui_armed=False,
        _commands={source: command}, _estop_latched=False,
        _stop_ticks_remaining=0, _zcu_connected=[True] * 4,
        output_pub=Mock(), _zero_command=SafetySupervisorNode._zero_command,
    )
    SafetySupervisorNode._control_tick(supervisor)
    supervisor.output_pub.publish.assert_not_called()

    supervisor._requested_source = source
    supervisor._gui_armed = True
    SafetySupervisorNode._control_tick(supervisor)
    supervisor.output_pub.publish.assert_called_once_with(command)
    supervisor.output_pub.reset_mock()

    supervisor._requested_source = 'none'
    supervisor._gui_armed = False
    for _ in range(25):
        SafetySupervisorNode._control_tick(supervisor)
    assert supervisor.output_pub.publish.call_count == 5
    assert all(call.args == (MotionCommand(),)
               for call in supervisor.output_pub.publish.call_args_list)

    supervisor.output_pub.reset_mock()
    supervisor._requested_source = source
    supervisor._gui_armed = True
    SafetySupervisorNode._control_tick(supervisor)
    supervisor.output_pub.publish.assert_called_once_with(command)


def test_supervisor_estop_keeps_publishing_zero_while_disarmed():
    supervisor = SimpleNamespace(
        _lock=threading.Lock(), _requested_source='none', _gui_armed=False,
        _commands={}, _estop_latched=True, _stop_ticks_remaining=0,
        output_pub=Mock(), _zero_command=SafetySupervisorNode._zero_command,
    )
    for _ in range(10):
        SafetySupervisorNode._control_tick(supervisor)
    assert supervisor.output_pub.publish.call_count == 10
    assert all(call.args == (MotionCommand(),)
               for call in supervisor.output_pub.publish.call_args_list)


@pytest.mark.parametrize('source,armed,can_reset', [
    ('none', False, True), ('manual', True, False),
    ('auto', True, False), ('none', True, False), ('manual', False, False),
])
def test_reset_with_zcus_offline_requires_stopped_source(source, armed, can_reset):
    supervisor = SimpleNamespace(
        _lock=threading.Lock(), _requested_source=source, _gui_armed=armed,
        _zcu_connected=[False] * 4,
        _estop_latched=True, _estop_reason='Emergency stop requested by GUI',
    )
    response = SafetySupervisorNode._reset_service(
        supervisor, None, SimpleNamespace())
    assert response.success is can_reset
    assert supervisor._estop_latched is not can_reset


def test_exit_button_requires_confirmation(monkeypatch):
    window = SimpleNamespace(closed=False)
    window.close = lambda: setattr(window, 'closed', True)

    monkeypatch.setattr(
        QMessageBox, 'question', lambda *args, **kwargs: QMessageBox.No)
    MainWindow._request_exit(window)
    assert not window.closed

    monkeypatch.setattr(
        QMessageBox, 'question', lambda *args, **kwargs: QMessageBox.Yes)
    MainWindow._request_exit(window)
    assert window.closed


def test_ros_image_conversion_does_not_require_opencv():
    image = ros_image_to_qimage(Image(
        width=2,
        height=1,
        encoding='bgr8',
        step=6,
        data=[0, 0, 255, 0, 255, 0],
    ))

    assert image.width() == 2
    assert image.height() == 1
    assert image.pixelColor(0, 0).getRgb()[:3] == (255, 0, 0)
    assert image.pixelColor(1, 0).getRgb()[:3] == (0, 255, 0)


def test_compressed_image_is_decoded_by_qt():
    source = ros_image_to_qimage(Image(
        width=2,
        height=1,
        encoding='bgr8',
        step=6,
        data=[0, 0, 255, 0, 255, 0],
    ))
    encoded = bytearray()
    buffer = QBuffer()
    buffer.open(QIODevice.WriteOnly)
    assert source.save(buffer, 'PNG')
    encoded.extend(buffer.data())

    image = compressed_image_to_qimage(CompressedImage(
        format='png', data=encoded))
    assert image.size() == source.size()


def test_opencv_qt_plugin_paths_are_removed(monkeypatch):
    cv2_path = '/tmp/site-packages/cv2/qt/plugins'
    monkeypatch.setenv('QT_QPA_PLATFORM_PLUGIN_PATH', cv2_path)
    monkeypatch.setenv(
        'QT_PLUGIN_PATH', os.pathsep.join((cv2_path, '/safe/qt/plugins')))

    remove_opencv_qt_plugin_paths()

    assert 'QT_QPA_PLATFORM_PLUGIN_PATH' not in os.environ
    assert os.environ['QT_PLUGIN_PATH'] == '/safe/qt/plugins'


def _write_profiles(tmp_path):
    config_dir = tmp_path / 'config'
    config_dir.mkdir()
    fastdds = config_dir / 'fastdds_whitelist.xml'
    fastdds.write_text('<dds/>')
    cyclonedds = config_dir / 'cyclonedds_ivn.xml'
    cyclonedds.write_text('<CycloneDDS/>')
    zenoh = config_dir / 'zenoh_session_ivn.json5'
    zenoh.write_text('{}')
    return fastdds, cyclonedds, zenoh


def test_gui_sets_wired_fastdds_profile_for_all_child_nodes(
        monkeypatch, tmp_path):
    fastdds, _, _ = _write_profiles(tmp_path)
    monkeypatch.setattr(
        'autosdv_gui_pkg.main.get_package_share_directory',
        lambda package: str(tmp_path),
    )
    monkeypatch.delenv('AUTOSDV_RMW', raising=False)
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE', raising=False)
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)

    selected = configure_default_dds_profile()

    assert selected == str(fastdds)
    assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(fastdds)
    assert os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] == str(fastdds)


def test_gui_sets_wired_cyclonedds_profile_when_selected(monkeypatch, tmp_path):
    _, cyclonedds, _ = _write_profiles(tmp_path)
    monkeypatch.setattr(
        'autosdv_gui_pkg.main.get_package_share_directory',
        lambda package: str(tmp_path),
    )
    monkeypatch.setenv('AUTOSDV_RMW', 'cyclonedds')
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)

    selected = configure_default_dds_profile()

    assert selected == 'file://' + str(cyclonedds)
    assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert os.environ['CYCLONEDDS_URI'] == 'file://' + str(cyclonedds)


def test_gui_sets_wired_zenoh_profile_when_selected(monkeypatch, tmp_path):
    _, _, zenoh = _write_profiles(tmp_path)
    monkeypatch.setattr(
        'autosdv_gui_pkg.main.get_package_share_directory',
        lambda package: str(tmp_path),
    )
    monkeypatch.setenv('AUTOSDV_RMW', 'zenoh')
    monkeypatch.delenv('ZENOH_SESSION_CONFIG_URI', raising=False)

    configure_default_dds_profile()

    assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'
    assert os.environ['ZENOH_SESSION_CONFIG_URI'] == str(zenoh)


def test_image_profile_env_follows_selected_middleware(monkeypatch, tmp_path):
    config_dir = tmp_path / 'config'
    config_dir.mkdir()
    (config_dir / 'fastdds_image_03_participant22.xml').write_text('<dds/>')
    (config_dir / 'cyclonedds_image_03_participant22.xml').write_text('<CycloneDDS/>')
    (config_dir / 'zenoh_image_03_participant22.json5').write_text('{}')

    monkeypatch.delenv('AUTOSDV_RMW', raising=False)
    fastdds_env = image_profile_env(str(tmp_path), 3, 22)
    assert fastdds_env['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert fastdds_env['FASTDDS_DEFAULT_PROFILES_FILE'].endswith(
        'fastdds_image_03_participant22.xml')

    monkeypatch.setenv('AUTOSDV_RMW', 'cyclonedds')
    cyclone_env = image_profile_env(str(tmp_path), 3, 22)
    assert cyclone_env['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert cyclone_env['CYCLONEDDS_URI'].endswith(
        'cyclonedds_image_03_participant22.xml')
    assert 'FASTDDS_DEFAULT_PROFILES_FILE' not in cyclone_env

    monkeypatch.setenv('AUTOSDV_RMW', 'zenoh')
    zenoh_env = image_profile_env(str(tmp_path), 3, 22)
    assert zenoh_env['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'
    assert zenoh_env['ZENOH_SESSION_CONFIG_URI'].endswith(
        'zenoh_image_03_participant22.json5')

    # The bridge environment has no per-node profile: DDS never leaves the
    # host, so every node shares the one loopback config and the bridges carry
    # the traffic. The Fast DDS name is still validated, hence the same call.
    monkeypatch.setenv('AUTOSDV_RMW', 'zenoh_bridge')
    (config_dir / 'cyclonedds_bridge_local.xml').write_text('<CycloneDDS/>')
    bridge_env = image_profile_env(str(tmp_path), 3, 22)
    assert bridge_env['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert bridge_env['CYCLONEDDS_URI'].endswith('cyclonedds_bridge_local.xml')
    assert bridge_env == image_profile_env(str(tmp_path), 5, 24)

    monkeypatch.setenv('AUTOSDV_RMW', 'nonsense')
    with pytest.raises(RuntimeError):
        image_profile_env(str(tmp_path), 3, 22)


def test_preview_notifications_coalesce_to_latest_frame():
    notifications = []
    bridge = SimpleNamespace(
        _lock=threading.Lock(),
        _preview_generation=7,
        _preview_sequence=0,
        _preview_topic='/image_01',
        _latest_preview=None,
        _preview_listener=lambda: notifications.append('ready'),
        _preview_notification_pending=False,
    )

    GuiRosBridge._preview_callback(bridge, 'frame-1', 7)
    GuiRosBridge._preview_callback(bridge, 'frame-2', 7)

    assert notifications == ['ready']
    assert GuiRosBridge.take_preview_snapshot(bridge) == (
        'frame-2', 2, '/image_01')

    GuiRosBridge._preview_callback(bridge, 'frame-3', 7)
    assert notifications == ['ready', 'ready']
