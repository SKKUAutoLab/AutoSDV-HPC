"""Responsive mode preflight/apply UI shared by normal and recovery startup."""
import os
import sys

from launch_pkg.communication_mode import LABELS, MODES, state
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import (
    QComboBox, QDialog, QHBoxLayout, QLabel, QPushButton, QPlainTextEdit,
    QVBoxLayout,
)


class ModeDialog(QDialog):
    def __init__(self, parent=None, prepare=None):
        super().__init__(parent)
        self.prepare = prepare
        self.prepared = False
        self._pause_complete = False
        self.process = None
        self.checked_mode = None
        self._action = None
        self.setWindowTitle('Communication mode — HPC and 4 ZCUs')
        self.resize(760, 440)
        layout = QVBoxLayout(self)
        saved = state()
        self.current = QLabel(
            f"Saved mode: {saved.get('mode', 'legacy / environment')}  |  "
            f"State: {saved.get('status', 'not configured')}")
        self.current.setWordWrap(True)
        layout.addWidget(self.current)
        row = QHBoxLayout()
        label = QLabel('Target mode:')
        self.selector = QComboBox()
        for mode, title in zip(MODES, LABELS):
            self.selector.addItem(title, mode)
        if saved.get('mode') in MODES:
            self.selector.setCurrentIndex(MODES.index(saved['mode']))
        label.setBuddy(self.selector)
        row.addWidget(label)
        row.addWidget(self.selector, 1)
        layout.addLayout(row)
        description = QLabel(
            'Check all devices first. Apply stops managed nodes and restarts the GUI.\n'
            'Driving remains disabled after switching. Unbuilt modes cannot be applied.')
        description.setWordWrap(True)
        layout.addWidget(description)
        self.status = QLabel('Ready to check prerequisites. No mode change has been made.')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setAccessibleName('Device checks and mode switch log')
        layout.addWidget(self.output, 1)
        buttons = QHBoxLayout()
        self.check_button = QPushButton('Check devices')
        self.apply_button = QPushButton('Apply and restart GUI')
        self.apply_button.setEnabled(False)
        self.close_button = QPushButton('Close')
        for button in (self.check_button, self.apply_button, self.close_button):
            button.setMinimumHeight(36)
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.check_button.clicked.connect(lambda: self.start('check'))
        self.apply_button.clicked.connect(lambda: self.start('apply'))
        self.close_button.clicked.connect(self.reject)
        self.selector.currentIndexChanged.connect(self.invalidate)

    def invalidate(self):
        self.checked_mode = None
        self.apply_button.setEnabled(False)
        self.status.setText('Target changed. Check all devices before applying.')

    def start(self, action):
        if self.process is not None:
            return
        mode = self.selector.currentData()
        if action == 'apply' and self.checked_mode != mode:
            return
        if action == 'apply' and not self._pause_complete:
            self.prepared = True
            if self.prepare is not None:
                try:
                    self.prepare()
                except Exception as exc:
                    self.status.setText(f'Could not stop the current GUI: {exc}')
                    return
            self._pause_complete = True
        self._action = action
        self.selector.setEnabled(False)
        self.check_button.setEnabled(False)
        self.apply_button.setEnabled(False)
        self.close_button.setEnabled(False)
        self.status.setText(f'{"Checking" if action == "check" else "Switching"} {mode}…')
        self.output.clear()
        process = QProcess(self)
        self.process = process
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self.read_output)
        process.finished.connect(self.finished)
        process.errorOccurred.connect(self.process_error)
        process.start(sys.executable, ['-u', '-m', 'launch_pkg.communication_mode',
                      action, mode, '--gui-owner', str(os.getpid())])

    def read_output(self):
        if self.process is not None:
            text = bytes(self.process.readAllStandardOutput()).decode('utf-8', errors='replace')
            self.output.appendPlainText(text.rstrip())

    def process_error(self, error):
        if error == QProcess.FailedToStart:
            self.output.appendPlainText(self.process.errorString())
            self.finished(1, QProcess.CrashExit)

    def finished(self, code, exit_status):
        if self.process is None:
            return
        self.read_output()
        process = self.process
        self.process = None
        process.deleteLater()
        self.selector.setEnabled(True)
        self.check_button.setEnabled(True)
        self.close_button.setEnabled(True)
        if code == 0 and exit_status == QProcess.NormalExit:
            if self._action == 'check':
                self.checked_mode = self.selector.currentData()
                self.apply_button.setEnabled(True)
                self.status.setText('All 5 hosts ready. Apply will stop nodes and restart this GUI.')
            else:
                self.status.setText('Mode applied. Live data validation remains pending.')
                self.accept()
        else:
            self.checked_mode = None
            self.status.setText(
                'Operation failed. See device errors below; correct them and check again.'
                + (' Driving is disabled.' if self.prepared else ' Current mode unchanged.'))

    def reject(self):
        # Do not terminate a coordinator halfway through remote mutations.
        if self.process is None:
            super().reject()

    def closeEvent(self, event):
        if self.process is not None:
            event.ignore()
        else:
            super().closeEvent(event)
