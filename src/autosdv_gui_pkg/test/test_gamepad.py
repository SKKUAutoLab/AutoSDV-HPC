"""Controller checks never publish ROS commands or start vehicle processes."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from autosdv_gui_pkg.gamepad import GamepadGate, XboxController


def sample(**values):
    result = dict(steering=0.0, forward=0.0, reverse=0.0,
                  left=False, right=False, reset=False, stop=False, estop_reset=False)
    result.update(values)
    return result


def test_neutral_required_then_pressure_and_position_without_rb():
    gate = GamepadGate()
    assert not any(gate.update(sample(forward=1), True).values())
    assert not any(gate.update(sample(steering=0.3), True).values())
    gate.update(sample(steering=0.05, forward=0.02), True)
    assert gate.ready
    command = gate.update(sample(forward=0.55, steering=-0.6), True)
    assert command['throttle'] == pytest.approx(0.5)
    assert command['steering'] == pytest.approx(-0.5)
    assert command['analog'] and command['steering_position']
    assert not command['brake'] and not command['right']
    command = gate.update(
        sample(reverse=1, steering=1, reset=True), True)
    assert command['brake'] == 1.0 and command['reset']
    assert command['steering'] == 0.0
    command = gate.update(sample(), True)
    assert command['steering_position'] and not command['steering']
    assert not command['throttle'] and not command['emergency_stop']


def test_startup_waits_for_controller_before_arming():
    gate = GamepadGate()
    assert not any(gate.update(None, True).values())
    assert not gate.ready
    assert not any(gate.update(sample(forward=1), True).values())
    gate.update(sample(), True)
    assert gate.ready
    assert gate.update(None, True)['emergency_stop']


def test_stop_button_still_works_before_neutral():
    assert GamepadGate().update(sample(stop=True), True)['emergency_stop']


def test_gui_startup_without_focus_waits_instead_of_estop(monkeypatch):
    from autosdv_gui_pkg.main import MainWindow, QApplication

    monkeypatch.setattr(QApplication, 'activeModalWidget', lambda: None)
    window = SimpleNamespace(
        drive_input_selector=SimpleNamespace(currentIndex=lambda: 1),
        gamepad=SimpleNamespace(read=lambda: sample(), status='connected'),
        gamepad_gate=GamepadGate(), control_source='manual',
        manager=SimpleNamespace(state=lambda _: 'RUNNING'),
        isActiveWindow=lambda: False, gamepad_status=Mock(),
        drive_input={}, emergency_stop=Mock(),
    )
    MainWindow._poll_gamepad(window)
    window.emergency_stop.assert_not_called()
    assert not any(window.drive_input.values())
    window.isActiveWindow = lambda: True
    MainWindow._poll_gamepad(window)
    assert window.gamepad_gate.ready


@pytest.mark.parametrize('fault', [None, sample(stop=True)])
def test_disconnect_and_stop_button_latch_stop(fault):
    gate = GamepadGate()
    gate.update(sample(), True)
    gate.update(sample(forward=1), True)
    command = gate.update(fault, True)
    assert command['emergency_stop']
    assert not command['forward']
    assert not gate.ready
    assert not any(gate.update(sample(forward=1), True).values())


def test_inactive_and_restart_require_new_neutral():
    gate = GamepadGate()
    gate.update(sample(), True)
    assert not any(gate.update(sample(stop=True), False).values())
    assert not gate.ready
    assert not any(gate.update(sample(forward=1), True).values())


def test_sdl_mapping_and_disconnect():
    reader = XboxController()
    pg = Mock()
    pg.error = RuntimeError
    pg.CONTROLLER_AXIS_LEFTX = 0
    pg.CONTROLLER_AXIS_TRIGGERRIGHT = 5
    pg.CONTROLLER_AXIS_TRIGGERLEFT = 4
    pg.CONTROLLER_BUTTON_DPAD_LEFT = 13
    pg.CONTROLLER_BUTTON_DPAD_RIGHT = 14
    pg.CONTROLLER_BUTTON_Y = 3
    pg.CONTROLLER_BUTTON_X = 2
    pg.CONTROLLER_BUTTON_B = 1
    device = Mock()
    device.name = 'Xbox Series X Controller'
    device.attached.return_value = True
    device.get_axis.side_effect = lambda index: {0: -32768, 5: 32767, 4: 0}[index]
    device.get_button.side_effect = lambda index: index in (13, 3)
    controllers = Mock()
    controllers.get_count.return_value = 1
    controllers.name_forindex.return_value = device.name
    controllers.is_controller.return_value = True
    controllers.Controller.return_value = device
    reader.pygame = pg
    reader.controllers = controllers
    state = reader.read()
    assert state['steering'] == -1
    assert state['forward'] > 0.99 and state['reverse'] == 0
    assert state['left'] and state['estop_reset'] and not state['stop']
    assert not state['right']
    device.attached.return_value = False
    assert reader.read() is None
    assert reader.device is None
    device.quit.assert_called_once()
    controllers.get_count.side_effect = RuntimeError('unplugged')
    assert reader.read() is None
    assert 'unplugged' in reader.status
    controllers.quit.assert_called_once()


def test_gui_controller_input_and_stop(monkeypatch):
    from autosdv_gui_pkg.main import MainWindow, QApplication

    monkeypatch.setattr(QApplication, 'activeModalWidget', lambda: None)
    window = SimpleNamespace(
        drive_input_selector=SimpleNamespace(currentIndex=lambda: 1),
        gamepad=SimpleNamespace(read=lambda: sample(), status='connected'),
        gamepad_gate=GamepadGate(),
        control_source='manual',
        manager=SimpleNamespace(state=lambda _: 'RUNNING'),
        isActiveWindow=lambda: True,
        gamepad_status=Mock(), drive_input={}, emergency_stop=Mock(),
    )
    MainWindow._poll_gamepad(window)
    window.gamepad.read = lambda: sample(forward=1)
    MainWindow._poll_gamepad(window)
    assert window.drive_input['throttle'] == 1.0
    window.isActiveWindow = lambda: False
    MainWindow._poll_gamepad(window)
    window.emergency_stop.assert_called_once()

    window.emergency_stop.reset_mock()
    window.control_source = 'auto'
    window.gamepad.read = lambda: None
    MainWindow._poll_gamepad(window)
    window.emergency_stop.assert_not_called()
    assert not any(window.drive_input.values())


def test_keyboard_is_ignored_in_controller_mode_and_switch_stops():
    from autosdv_gui_pkg.main import MainWindow

    window = SimpleNamespace(
        drive_input_selector=SimpleNamespace(currentIndex=lambda: 1),
        drive_input={'forward': False, 'reset': False},
        control_source='manual', emergency_stop=Mock(),
        _cancel_estop_resume=Mock(),
        _clear_drive_input=Mock(), gamepad_gate=GamepadGate(),
        gamepad=Mock(), gamepad_status=Mock(),
    )
    MainWindow._set_drive_field(window, 'forward', True)
    MainWindow._pulse_reset(window)
    assert not any(window.drive_input.values())
    MainWindow._change_drive_input(window, 0)
    window.emergency_stop.assert_called_once()
    window._clear_drive_input.assert_called_once()
    window.gamepad.close.assert_called_once()


def test_gamepad_commands_use_existing_acceleration_and_stop():
    from data_collection_pkg.drive_node import DriveController
    from interfaces_pkg.msg import DriveInput

    gate = GamepadGate()
    drive = DriveController()

    def apply(state):
        command = gate.update(state, True)
        command['steer_left'] = command.pop('left')
        command['steer_right'] = command.pop('right')
        drive.set_gui_input(DriveInput(**command))
        return drive.update(0.1)

    assert apply(sample()) == (0, 0)
    assert apply(sample(forward=1, steering=-1)) == (-1, 5)
    assert apply(sample(forward=1, reset=True)) == (0, 10)
    assert apply(None) == (0, 0)


def test_drive_panel_builds_and_selects_controller():
    from PyQt5.QtWidgets import QApplication
    from autosdv_gui_pkg.main import MainWindow

    app = QApplication.instance() or QApplication([])
    window = SimpleNamespace(
        ros=Mock(), _value_label=MainWindow._value_label,
        _request_estop_reset=Mock(),
        _set_drive_field=Mock(), _pulse_reset=Mock(), _change_drive_input=Mock(),
    )
    panel = MainWindow._build_drive_panel(window)
    panel.resize(550, 350)
    panel.show()
    app.processEvents()
    assert window.drive_input_selector.currentText() == 'Keyboard / GUI'
    window.drive_input_selector.setCurrentIndex(1)
    window._change_drive_input.assert_called_once_with(1)
    assert not panel.grab().isNull()
    panel.close()


def test_dpad_holds_angle_until_stick_or_center_takes_over():
    gate = GamepadGate()
    gate.update(sample(), True)
    command = gate.update(sample(left=True), True)
    assert command['left'] and not command['steering_position']
    command = gate.update(sample(), True)
    assert not command['left'] and not command['steering_position']
    command = gate.update(sample(left=True, steering=1), True)
    assert command['steering_position'] and command['steering'] == 1.0
    assert not command['left']
    command = gate.update(sample(), True)
    assert command['steering_position'] and command['steering'] == 0.0
    gate.update(sample(right=True), True)
    command = gate.update(sample(reset=True), True)
    assert command['steering_position'] and command['reset']


def test_reset_button_edges_require_neutral_and_stop_has_priority():
    gate = GamepadGate()
    assert gate.reset_requested(sample(estop_reset=True))
    assert not gate.reset_requested(sample(estop_reset=True))
    gate.reset_requested(sample())
    assert not gate.reset_requested(sample(estop_reset=True, forward=1))
    assert not gate.reset_requested(sample(estop_reset=True))
    gate.reset_requested(sample())
    assert not gate.reset_requested(sample(estop_reset=True, stop=True))
    assert gate.update(sample(estop_reset=True, stop=True), True)['emergency_stop']


def test_gui_reset_works_while_stopped_without_starting_drive(monkeypatch):
    from autosdv_gui_pkg.main import MainWindow, QApplication

    monkeypatch.setattr(QApplication, 'activeModalWidget', lambda: None)
    window = SimpleNamespace(
        drive_input_selector=SimpleNamespace(currentIndex=lambda: 1),
        gamepad=SimpleNamespace(read=lambda: sample(estop_reset=True), status='connected'),
        gamepad_gate=GamepadGate(), control_source='none',
        manager=SimpleNamespace(state=lambda _: 'STOPPED'),
        isActiveWindow=lambda: True, gamepad_status=Mock(),
        drive_input={}, emergency_stop=Mock(), ros=Mock(),
    )
    window._request_estop_reset = window.ros.request_estop_reset
    MainWindow._poll_gamepad(window)
    MainWindow._poll_gamepad(window)
    window.ros.request_estop_reset.assert_called_once()
    assert window.control_source == 'none'
    assert not any(window.drive_input.values())
    window.gamepad.read = lambda: sample()
    MainWindow._poll_gamepad(window)
    window.control_source = 'auto'
    window.gamepad.read = lambda: sample(estop_reset=True)
    MainWindow._poll_gamepad(window)
    window.ros.request_estop_reset.assert_called_once()


def test_analog_pressure_brakes_to_zero_then_reverses_and_returns_forward():
    from data_collection_pkg.drive_node import DriveController
    from interfaces_pkg.msg import DriveInput

    drive = DriveController()
    drive.set_gui_input(DriveInput(analog=True, throttle=0.5))
    drive.update(0.1)
    assert drive.speed == 2.5
    drive.set_gui_input(DriveInput(analog=True, brake=0.5))
    assert drive.update(0.1)[1] == 0
    drive.update(0.1)
    assert drive.speed == -2.5
    drive.set_gui_input(DriveInput(analog=True, throttle=1.0))
    assert drive.update(0.1)[1] == 0
    assert drive.update(0.1)[1] == 5
    drive.set_gui_input(DriveInput(analog=True, throttle=1.0, brake=1.0))
    assert drive.update(0.1)[1] == 0
    assert drive.update(0.1)[1] == 0


def test_steering_position_return_to_center_and_digital_hold():
    from data_collection_pkg.drive_node import DriveController
    from interfaces_pkg.msg import DriveInput

    drive = DriveController()
    drive.set_gui_input(DriveInput(analog=True, steering_position=True, steering=0.5))
    for _ in range(5):
        drive.update(0.1)
    assert drive.steering == 3.5
    drive.set_gui_input(DriveInput(analog=True, steering_position=True))
    drive.update(0.1)
    assert drive.steering == 2.5  # Return is rate limited, not an instantaneous jump.
    for _ in range(3):
        drive.update(0.1)
    assert drive.steering == 0
    drive.set_gui_input(DriveInput(analog=True, steer_left=True))
    drive.update(0.1)
    assert drive.steering == -1
    drive.set_gui_input(DriveInput(analog=True))
    drive.update(0.1)
    assert drive.steering == -1


def test_bridge_preserves_analog_values_and_digital_defaults():
    from autosdv_gui_pkg.ros_bridge import GuiRosBridge

    bridge = SimpleNamespace(drive_pub=Mock())
    GuiRosBridge.publish_drive_input(bridge, dict(
        analog=True, throttle=0.25, brake=0.5,
        steering_position=True, steering=-0.75, left=True))
    msg = bridge.drive_pub.publish.call_args.args[0]
    assert msg.analog and msg.steering_position and msg.steer_left
    assert (msg.throttle, msg.brake, msg.steering) == (0.25, 0.5, -0.75)
    GuiRosBridge.publish_drive_input(bridge, {'forward': True})
    msg = bridge.drive_pub.publish.call_args.args[0]
    assert msg.forward and not msg.analog
    assert msg.throttle == 0.0


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_axes_stop_without_crashing(bad):
    from data_collection_pkg.drive_node import DriveController
    from interfaces_pkg.msg import DriveInput

    drive = DriveController()
    drive.speed = 20.0
    drive.set_gui_input(DriveInput(analog=True, throttle=1.0, steering=bad))
    assert drive.update(0.1) == (0, 0)


def test_stop_overrides_held_throttle_and_input_timeout_decelerates():
    from data_collection_pkg.drive_node import DriveController
    from interfaces_pkg.msg import DriveInput

    drive = DriveController()
    drive.set_gui_input(DriveInput(analog=True, throttle=1.0, emergency_stop=True))
    assert drive.update(0.1) == (0, 0)
    drive.set_gui_input(DriveInput(analog=True, throttle=1.0))
    drive.update(0.2)
    assert drive.speed == 10
    drive._last_input_time -= 1.0
    drive.update(0.1)
    assert drive.speed == 4.0


@pytest.fixture
def resume_window(monkeypatch):
    from autosdv_gui_pkg.main import MainWindow, QApplication
    from concurrent.futures import Future

    monkeypatch.setattr(QApplication, 'activeModalWidget', lambda: None)
    window = SimpleNamespace(
        _closing=False, _xbox_resume_after_estop=True,
        _estop_reset_future=None, _estop_reset_succeeded=False,
        control_source='none', _fallback_stop_ticks=0,
        drive_input_selector=SimpleNamespace(currentIndex=lambda: 1),
        latest_safety=SimpleNamespace(emergency_stop=False),
        manager=Mock(), gamepad=Mock(), gamepad_gate=GamepadGate(),
        isActiveWindow=lambda: True, _start_node=Mock(),
        _clear_drive_input=Mock(), ros=Mock(), drive_input={},
    )
    window.manager.state.return_value = 'STOPPED'
    window.manager.stop.return_value = (True, 'Stopped')
    window.gamepad.read.return_value = sample()
    window.ros.request_estop_reset.return_value = Future()
    window._cancel_estop_resume = lambda: MainWindow._cancel_estop_resume(window)
    return window


def test_reset_success_waits_for_stop_status_and_neutral_then_starts_once(resume_window):
    from autosdv_gui_pkg.main import MainWindow

    w = resume_window
    MainWindow._request_estop_reset(w)
    MainWindow._resume_after_estop_reset(w)
    w._start_node.assert_not_called()
    w._estop_reset_future.set_result(SimpleNamespace(success=True))
    w.latest_safety.emergency_stop = True
    MainWindow._resume_after_estop_reset(w)
    w._start_node.assert_not_called()
    w.latest_safety.emergency_stop = False
    w.manager.state.return_value = 'STOPPING'
    MainWindow._resume_after_estop_reset(w)
    w._start_node.assert_not_called()
    w.manager.state.return_value = 'STOPPED'
    w._fallback_stop_ticks = 1
    MainWindow._resume_after_estop_reset(w)
    w._start_node.assert_not_called()
    w._fallback_stop_ticks = 0
    for state in (None, sample(forward=1), sample(estop_reset=True)):
        w.gamepad.read.return_value = state
        MainWindow._resume_after_estop_reset(w)
        w._start_node.assert_not_called()
    w.gamepad.read.return_value = sample()
    MainWindow._resume_after_estop_reset(w)
    MainWindow._resume_after_estop_reset(w)
    w._start_node.assert_called_once_with('drive')
    w._clear_drive_input.assert_called_once()


@pytest.mark.parametrize('outcome', ['failed', 'exception', 'unavailable', 'keyboard', 'stop'])
def test_reset_does_not_restart_without_eligible_success(resume_window, outcome):
    from autosdv_gui_pkg.main import MainWindow

    w = resume_window
    if outcome == 'unavailable':
        w.ros.request_estop_reset.return_value = None
    if outcome == 'keyboard':
        w.drive_input_selector.currentIndex = lambda: 0
    if outcome == 'stop':
        w._xbox_resume_after_estop = False
    MainWindow._request_estop_reset(w)
    future = w._estop_reset_future
    if future is not None:
        if outcome == 'exception':
            future.set_exception(RuntimeError('service failed'))
        else:
            future.set_result(SimpleNamespace(success=False))
    MainWindow._resume_after_estop_reset(w)
    w._start_node.assert_not_called()


@pytest.mark.parametrize('mode,source,eligible', [
    (1, 'manual', True), (0, 'manual', False), (1, 'auto', False)])
def test_only_xbox_manual_estop_arms_restart(resume_window, monkeypatch, mode, source, eligible):
    from autosdv_gui_pkg.main import MainWindow, QTimer

    w = resume_window
    w.control_source = source
    w.drive_input_selector.currentIndex = lambda: mode
    monkeypatch.setattr(QTimer, 'singleShot', lambda *args: None)
    MainWindow.emergency_stop(w)
    assert w._xbox_resume_after_estop is eligible
    assert w.control_source == 'none'


def test_user_stop_cancels_pending_restart(resume_window):
    from autosdv_gui_pkg.main import MainWindow

    w = resume_window
    MainWindow._request_estop_reset(w)
    pending = w._estop_reset_future
    MainWindow._stop_node(w, 'drive')
    pending.set_result(SimpleNamespace(success=True))
    MainWindow._resume_after_estop_reset(w)
    w._start_node.assert_not_called()
    assert not w._xbox_resume_after_estop


def test_reset_bridge_returns_future_and_reports_result():
    import threading
    from concurrent.futures import Future
    from autosdv_gui_pkg.ros_bridge import GuiRosBridge

    future = Future()
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = future
    bridge = SimpleNamespace(_lock=threading.Lock(), _events=[], reset_client=client)
    bridge._call_trigger = lambda *args: GuiRosBridge._call_trigger(bridge, *args)
    assert GuiRosBridge.request_estop_reset(bridge) is future
    future.set_result(SimpleNamespace(success=True, message='Reset complete'))
    assert bridge._events == [(True, 'Reset complete')]
    client.service_is_ready.return_value = False
    assert GuiRosBridge.request_estop_reset(bridge) is None
    assert not bridge._events[-1][0]
