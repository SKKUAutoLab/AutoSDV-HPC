import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtWidgets import QApplication

from autosdv_gui_pkg import mode_dialog as ui


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


class FakeProcess:
    def readAllStandardOutput(self):
        return b'host status\n'

    def deleteLater(self):
        pass


def finish(dialog, action, code):
    dialog.process = FakeProcess()
    dialog._action = action
    dialog.finished(code, QProcess.NormalExit)


def test_apply_requires_successful_check_for_selected_mode(app, monkeypatch):
    monkeypatch.setattr(ui, 'state', lambda: {})
    d = ui.ModeDialog()
    assert d.selector.count() == 4
    assert not d.apply_button.isEnabled()
    finish(d, 'check', 0)
    assert d.apply_button.isEnabled()
    d.selector.setCurrentIndex(1)
    assert not d.apply_button.isEnabled()
    finish(d, 'check', 1)
    assert not d.apply_button.isEnabled()
    assert 'Current mode unchanged' in d.status.text()
    d.close()


def test_failure_after_pause_does_not_resume_driving(app, monkeypatch):
    monkeypatch.setattr(ui, 'state', lambda: {})
    d = ui.ModeDialog()
    d.prepared = True
    finish(d, 'apply', 1)
    assert 'Driving is disabled' in d.status.text()
    assert d.check_button.isEnabled()
    assert not d.apply_button.isEnabled()
    d.close()


def test_cannot_close_during_remote_operation(app, monkeypatch):
    monkeypatch.setattr(ui, 'state', lambda: {})
    d = ui.ModeDialog()
    d.show()
    d.process = FakeProcess()
    d.reject()
    assert d.isVisible()
    d.close()
    assert d.isVisible()
    d.process = None
    d.close()


def test_failed_to_start_recovers_buttons(app, monkeypatch):
    monkeypatch.setattr(ui, 'state', lambda: {})
    d = ui.ModeDialog()
    d.process = FakeProcess()
    d.process.errorString = lambda: 'cannot start python'
    d._action = 'check'
    d.process_error(QProcess.FailedToStart)
    assert d.process is None
    assert d.check_button.isEnabled()
    assert not d.apply_button.isEnabled()
    d.close()


def test_async_check_apply_keeps_event_loop_responsive(app, monkeypatch):
    import sys
    from PyQt5.QtTest import QTest
    monkeypatch.setattr(ui, 'state', lambda: {})
    real_process = QProcess

    class LocalProcess(real_process):
        def start(self, program, arguments):
            super().start(sys.executable, ['-c',
                          'import time; time.sleep(.1); print("simulated host check")'])

    monkeypatch.setattr(ui, 'QProcess', LocalProcess)
    prepared = []
    d = ui.ModeDialog(prepare=lambda: prepared.append(True))
    ticks = []
    timer = QTimer()
    timer.timeout.connect(lambda: ticks.append(1))
    timer.start(10)
    d.start('check')
    for _ in range(100):
        if d.process is None:
            break
        QTest.qWait(10)
    assert d.process is None and d.apply_button.isEnabled()
    assert ticks and not prepared
    d.start('apply')
    assert prepared == [True]
    for _ in range(100):
        if d.process is None:
            break
        QTest.qWait(10)
    timer.stop()
    assert d.process is None and d.result() == d.Accepted
    d.close()
