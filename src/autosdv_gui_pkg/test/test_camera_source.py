import json
import os
import shlex
import subprocess
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import pytest
from PyQt5.QtCore import QProcess, QTimer
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from autosdv_gui_pkg import camera_source as control
from autosdv_gui_pkg import camera_source_dialog as ui


def test_target_mapping_and_remote_path_quoting():
    calls = []
    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout='{"ok":true,"source":"video"}', stderr='', returncode=0)
    path = "/home/pi/camera/a ' $(touch BAD); video.mp4"
    for camera in control.CAMERAS:
        control.request(camera, 'video', path, runner=run)
    assert 'pi@12.0.0.5' in calls[0]
    assert 'HostKeyAlias=rpi-zcu1-webcam' in calls[0]
    assert 'ProxyJump=root@10.0.0.5' in calls[-1]
    for args in calls:
        assert shlex.split(args[-1])[-1] == path
        assert 'BatchMode=yes' in args


def test_failed_switch_reports_actual_old_source(monkeypatch):
    replies = iter([{'ok': False, 'source': 'camera', 'error': 'Bad file'},
                    {'ok': True, 'source': 'camera'}])
    monkeypatch.setattr(control, 'request', lambda *a: next(replies))
    result = control.operate('camera_01', 'video', '/bad.mp4')
    assert result['state']['source'] == 'camera'
    assert result['error'] == 'Bad file'


def test_timeout_requeries_actual_source(monkeypatch):
    def request(camera, action='status', path=''):
        if action != 'status':
            raise subprocess.TimeoutExpired('ssh', 25)
        return {'ok': True, 'source': 'video', 'video_path': '/video.mp4'}
    monkeypatch.setattr(control, 'request', request)
    result = control.operate('camera_01', 'video', '/video.mp4')
    assert result['state']['source'] == 'video'
    assert result['error']


def test_failed_requery_does_not_present_unconfirmed_state(monkeypatch):
    def request(camera, action='status', path=''):
        if action == 'status':
            raise RuntimeError('offline')
        return {'ok': True, 'source': 'camera'}
    monkeypatch.setattr(control, 'request', request)
    result = control.operate('camera_01', 'camera')
    assert result['state'] == {}
    assert 'confirm' in result['error']


def test_dialog_async_open_reads_only_and_shows_partial_failure(monkeypatch):
    app = QApplication.instance() or QApplication([])
    calls = []
    class LocalProcess(QProcess):
        def start(self, program, args):
            calls.append(args)
            camera, action = args[2:4]
            result = {'state': {'ok': True, 'source': 'camera'},
                      'error': 'Bad file' if camera == 'camera_02' and action == 'video' else ''}
            if action == 'video' and not result['error']:
                result['state'].update(source='video', video_path=args[4])
            super().start(program, ['-c', 'import time; time.sleep(0.15); print(' + repr(json.dumps(result)) + ')'])
    monkeypatch.setattr(ui, 'QProcess', LocalProcess)
    dialog = ui.CameraSourceDialog()
    ticks = []
    timer = QTimer()
    timer.timeout.connect(lambda: ticks.append(1))
    timer.start(10)
    dialog.show()
    QTest.qWait(600)
    assert len(calls) == 5 and all(c[3] == 'status' for c in calls)
    assert len(ticks) > 5 and not dialog.processes
    dialog.start_all('video')
    assert all(not b.isEnabled() for b in dialog.bulk_buttons)
    QTest.qWait(600)
    assert 'Camera' in dialog.rows['camera_02'][0].text()
    assert 'Bad file' in dialog.rows['camera_02'][0].text()
    assert 'Video' in dialog.rows['camera_01'][0].text()
    assert all(b.isEnabled() for b in dialog.bulk_buttons)
    timer.stop()
    dialog.close()
