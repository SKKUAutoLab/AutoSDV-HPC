"""Nonblocking per-camera input selection; opening the window only reads state."""
import json
import sys

from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtWidgets import (
    QDialog, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QVBoxLayout,
)

from .camera_source import CAMERAS, DEFAULT_VIDEO


class CameraSourceDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Camera inputs')
        self.resize(1050, 480)
        self.processes = {}
        self.rows = {}
        layout = QVBoxLayout(self)
        help_text = QLabel(
            'Switch live inputs without restarting camera processes. A brief image pause may occur.\n'
            'Video paths refer to files on each Raspberry Pi. Changes last until the camera service restarts.')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        bulk = QHBoxLayout()
        self.bulk_buttons = []
        for title, action in [('Refresh all', 'status'), ('All → Camera', 'camera'),
                              ('All → Video', 'video')]:
            button = QPushButton(title)
            button.setMinimumHeight(40)
            button.clicked.connect(lambda checked=False, a=action: self.start_all(a))
            self.bulk_buttons.append(button)
            bulk.addWidget(button)
        layout.addLayout(bulk)
        grid = QGridLayout()
        for col, title in enumerate(['Camera', 'Actual source / result', 'Video file on Raspberry Pi', 'Actions']):
            grid.addWidget(QLabel(title), 0, col)
        for row, (camera_id, config) in enumerate(CAMERAS.items(), 1):
            state = QLabel('Unknown — refresh to read')
            state.setWordWrap(True)
            state.setMinimumWidth(180)
            state.setTextFormat(Qt.PlainText)  # Qt.PlainText: remote errors are not markup.
            path = QLineEdit(DEFAULT_VIDEO)
            path.setAccessibleName(f'{config[0]} video file on Raspberry Pi')
            buttons = []
            actions = QHBoxLayout()
            for title, action in [('Refresh', 'status'), ('Camera', 'camera'), ('Video', 'video')]:
                button = QPushButton(title)
                button.setMinimumHeight(36)
                button.setAccessibleName(f'{config[0]} {title}')
                button.clicked.connect(
                    lambda checked=False, c=camera_id, a=action: self.start(c, a))
                actions.addWidget(button)
                buttons.append(button)
            grid.addWidget(QLabel(config[0]), row, 0)
            grid.addWidget(state, row, 1)
            grid.addWidget(path, row, 2)
            grid.addLayout(actions, row, 3)
            self.rows[camera_id] = (state, path, buttons)
        grid.setColumnStretch(2, 1)
        layout.addLayout(grid)
        self.summary = QLabel('Actual source is read from each camera; selection alone does not indicate success.')
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        close = QPushButton('Close')
        close.clicked.connect(self.close)
        layout.addWidget(close)

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, lambda: self.start_all('status'))

    def start_all(self, action):
        if self.processes:
            return
        for camera_id in self.rows:
            self.start(camera_id, action)

    def start(self, camera_id, action):
        if camera_id in self.processes:
            return
        state, path, buttons = self.rows[camera_id]
        if action == 'video' and (not path.text().startswith('/') or
                                 any(c in path.text() for c in '\n\r\0')):
            state.setText('Not sent: enter an absolute video file path')
            return
        state.setText('Reading current source…' if action == 'status' else f'Switching to {action}…')
        path.setEnabled(False)
        for button in buttons + self.bulk_buttons:
            button.setEnabled(False)
        process = QProcess(self)
        self.processes[camera_id] = process
        process.finished.connect(lambda code, status, c=camera_id: self.finished(c))
        process.errorOccurred.connect(lambda error, c=camera_id: self.failed(c, error))
        process.start(sys.executable, ['-m', 'autosdv_gui_pkg.camera_source',
                                      camera_id, action, path.text() if action == 'video' else ''])
        self.summary.setText('Working… each camera reports its own result. Whole-system switching is not simultaneous.')

    def failed(self, camera_id, error):
        if error == QProcess.FailedToStart:
            self.finished(camera_id)

    def finished(self, camera_id):
        process = self.processes.pop(camera_id, None)
        if process is None:
            return
        state, path, buttons = self.rows[camera_id]
        try:
            result = json.loads(bytes(process.readAllStandardOutput()).decode())
            actual = result['state']
            error = result['error']
            if actual.get('ok'):
                source = actual.get('source')
                text = {'camera': 'Camera', 'video': 'Video', None: 'No active input'}[source]
                if source == 'video' and not error:
                    path.setText(actual.get('video_path', path.text()))
                if error:
                    text += '\nRequest failed: ' + error
            else:
                text = 'Unknown — ' + (error or 'Could not read current source')
            state.setText(text)
            state.setToolTip(text + '\nActual video file: ' + actual.get('video_path', '—'))
        except (ValueError, KeyError, TypeError):
            detail = bytes(process.readAllStandardError()).decode(errors='replace').strip()
            state.setText('Unknown — ' + (detail or process.errorString()))
        path.setEnabled(True)
        for button in buttons:
            button.setEnabled(True)
        process.deleteLater()
        if not self.processes:
            for button in self.bulk_buttons:
                button.setEnabled(True)
            self.summary.setText('Finished — see each camera’s actual source and any errors above.')
