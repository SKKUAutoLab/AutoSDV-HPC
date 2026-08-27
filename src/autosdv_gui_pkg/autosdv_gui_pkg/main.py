"""PyQt5 integrated control GUI for the AutoSDV HPC."""

import html
import os
import sys
import threading

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import (
    pyqtSignal,
    QEvent,
    QLockFile,
    QProcess,
    QRectF,
    Qt,
    QTimer,
)
from PyQt5.QtGui import (
    QColor,
    QFont,
    QImage,
    QPainter,
    QPainterPath,
    QPalette,
    QPen,
)
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.executors import SingleThreadedExecutor

from .process_manager import (
    load_node_specs,
    NodeProcessManager,
    terminate_stale_safety_supervisors,
)
from .ros_bridge import GuiRosBridge


CAMERA_IDS = [f'camera_{index:02d}' for index in range(1, 6)]
PIPELINE_IDS = ['yolo', 'lane', 'path_planner', 'motion_planner', 'drive']
CONTROL_CHAIN = {'camera_01', 'yolo', 'lane', 'path_planner',
                 'motion_planner', 'drive'}
PREVIEW_TOPICS = {
    'camera_01': '/image_01',
    'camera_02': '/image_02',
    'camera_03': '/image_03',
    'camera_04': '/image_04',
    'camera_05': '/image_05',
    'yolo': '/yolov8_visualized_img',
    'lane': '/roi_image',
    'path_planner': '/path_visualized_img',
}
PREVIEW_OUTPUT_TOPIC = '/autosdv_gui/preview/compressed'
SHUTDOWN_ORDER = [
    'drive', 'motion_planner', 'preview_bridge',
    'path_visualizer', 'path_planner', 'lane',
    'yolo_visualizer', 'yolo',
    'camera_05', 'camera_04', 'camera_03', 'camera_02', 'camera_01',
    'ivn_diagnostics', 'safety_supervisor',
]

ZCU_LAYOUT_ZONE_IDS = {
    'FR ZCU': 'zone_2',
    'FL ZCU': 'zone_1',
    'RR ZCU': 'zone_4',
    'RL ZCU': 'zone_3',
}


def configure_default_fastdds_profile():
    """Force GUI and generic child nodes onto the wired IVN interfaces."""
    profile = os.path.join(
        get_package_share_directory('launch_pkg'),
        'config',
        'fastdds_whitelist.xml',
    )
    if not os.path.isfile(profile):
        raise RuntimeError(f'Fast DDS profile not found: {profile}')
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile
    os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = profile
    return profile


def ros_image_to_qimage(message):
    """Convert common sensor_msgs/Image encodings without importing OpenCV."""
    encoding = str(message.encoding).strip().lower()
    formats = {
        'rgb8': (QImage.Format_RGB888, 3),
        'bgr8': (QImage.Format_BGR888, 3),
        'rgba8': (QImage.Format_RGBA8888, 4),
        'bgra8': (QImage.Format_ARGB32, 4),
        'mono8': (QImage.Format_Grayscale8, 1),
        '8uc1': (QImage.Format_Grayscale8, 1),
        '8uc3': (QImage.Format_BGR888, 3),
    }
    if encoding.startswith('bayer_') and encoding.endswith('8'):
        image_format, channels = QImage.Format_Grayscale8, 1
    elif encoding in formats:
        image_format, channels = formats[encoding]
    else:
        raise ValueError(f'Unsupported image encoding: {message.encoding}')

    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0:
        raise ValueError(f'Invalid image size: {width}x{height}')
    bytes_per_line = int(message.step) or width * channels
    if bytes_per_line < width * channels:
        raise ValueError(
            f'Invalid image step {bytes_per_line} for {width}x{encoding}')

    # sensor_msgs stores image data in a contiguous array. Referencing that
    # buffer directly avoids copying a full high-resolution frame in Python.
    data = memoryview(message.data)
    if data.format != 'B':
        data = data.cast('B')
    required_size = bytes_per_line * height
    if data.nbytes < required_size:
        raise ValueError(
            f'Image data is truncated: {data.nbytes} < {required_size}')

    image = QImage(
        data,
        width,
        height,
        bytes_per_line,
        image_format,
    )
    # QImage does not own externally supplied memory. Retain both the view and
    # ROS message for as long as this image is the currently displayed frame.
    image._autosdv_buffer = data
    image._autosdv_message = message
    return image


def compressed_image_to_qimage(message):
    """Decode a compressed preview through Qt's native image decoder."""
    data = bytes(message.data)
    if not data:
        raise ValueError('Compressed preview is empty')
    image = QImage.fromData(data)
    if image.isNull():
        raise ValueError(
            f'Unsupported compressed image format: {message.format}')
    return image


def remove_opencv_qt_plugin_paths():
    """Discard plugin paths injected by pip OpenCV before Qt starts."""
    platform_path = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH', '')
    if '/cv2/qt/plugins' in platform_path:
        os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)

    plugin_path = os.environ.get('QT_PLUGIN_PATH', '')
    if plugin_path:
        safe_paths = [
            path for path in plugin_path.split(os.pathsep)
            if '/cv2/qt/plugins' not in path
        ]
        if safe_paths:
            os.environ['QT_PLUGIN_PATH'] = os.pathsep.join(safe_paths)
        else:
            os.environ.pop('QT_PLUGIN_PATH', None)


class VehicleDiagnosticsWidget(QWidget):
    """Large horizontal vehicle layout driven by automatic IVN status."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(250)
        self._summary_state = 'stale'
        self._zone_states = {}
        self._camera_states = {}

    def set_diagnostics(self, summary_state, zone_states, camera_states):
        self._summary_state = str(summary_state)
        self._zone_states = dict(zone_states)
        self._camera_states = dict(camera_states)
        self.update()

    @staticmethod
    def _state_color(state):
        return QColor({
            'ok': '#39c86a',
            'warn': '#e1a73a',
            'error': '#e25757',
            'stale': '#697887',
        }.get(str(state), '#697887'))

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = float(self.width())
        height = float(self.height())
        available_height = max(180.0, height)
        body_width = min(width * 0.96, available_height * 3.15)
        body_height = min(available_height * 0.96, body_width / 3.15)
        left = (width - body_width) / 2.0
        top = (height - body_height) / 2.0
        right = left + body_width
        bottom = top + body_height
        center_x = (left + right) / 2.0
        center_y = (top + bottom) / 2.0
        scale = max(0.85, min(1.75, body_height / 280.0))

        body_path = QPainterPath()
        body_path.moveTo(left + body_width * 0.16, top)
        body_path.lineTo(right - body_width * 0.10, top)
        body_path.cubicTo(
            right - body_width * 0.025, top,
            right, top + body_height * 0.22,
            right, center_y,
        )
        body_path.cubicTo(
            right, bottom - body_height * 0.22,
            right - body_width * 0.025, bottom,
            right - body_width * 0.10, bottom,
        )
        body_path.lineTo(left + body_width * 0.16, bottom)
        body_path.cubicTo(
            left + body_width * 0.055, bottom,
            left, bottom - body_height * 0.20,
            left, center_y,
        )
        body_path.cubicTo(
            left, top + body_height * 0.20,
            left + body_width * 0.055, top,
            left + body_width * 0.16, top,
        )
        body_path.closeSubpath()

        painter.setPen(QPen(QColor('#65798d'), max(2.0, 2.4 * scale)))
        painter.setBrush(QColor('#202d39'))
        painter.drawPath(body_path)

        cabin = QPainterPath()
        cabin.moveTo(left + body_width * 0.31, top + body_height * 0.13)
        cabin.lineTo(right - body_width * 0.22, top + body_height * 0.13)
        cabin.cubicTo(
            right - body_width * 0.16, top + body_height * 0.13,
            right - body_width * 0.13, top + body_height * 0.29,
            right - body_width * 0.13, center_y,
        )
        cabin.cubicTo(
            right - body_width * 0.13, bottom - body_height * 0.29,
            right - body_width * 0.16, bottom - body_height * 0.13,
            right - body_width * 0.22, bottom - body_height * 0.13,
        )
        cabin.lineTo(left + body_width * 0.31, bottom - body_height * 0.13)
        cabin.cubicTo(
            left + body_width * 0.25, bottom - body_height * 0.18,
            left + body_width * 0.22, bottom - body_height * 0.34,
            left + body_width * 0.22, center_y,
        )
        cabin.cubicTo(
            left + body_width * 0.22, top + body_height * 0.34,
            left + body_width * 0.25, top + body_height * 0.18,
            left + body_width * 0.31, top + body_height * 0.13,
        )
        cabin.closeSubpath()
        painter.setPen(QPen(QColor('#455a6d'), max(1.5, 1.8 * scale)))
        painter.setBrush(QColor('#16222d'))
        painter.drawPath(cabin)

        self._draw_wheels(
            painter, left, right, top, bottom, body_width, body_height, scale)

        upper_y = top + body_height * 0.30
        lower_y = bottom - body_height * 0.30
        front_zcu_x = left + body_width * 0.29
        rear_zcu_x = right - body_width * 0.25
        hpc_x = center_x + body_width * 0.03

        camera_positions = {
            'image_01': (left + body_width * 0.035, center_y),
            'image_02': (left + body_width * 0.10, bottom - body_height * 0.18),
            'image_03': (center_x, top + body_height * 0.025),
            'image_04': (center_x, bottom - body_height * 0.045),
            'image_05': (right - body_width * 0.035, center_y),
        }
        zcu_positions = {
            'FR ZCU': (front_zcu_x, upper_y),
            'FL ZCU': (front_zcu_x, lower_y),
            'RR ZCU': (rear_zcu_x, upper_y),
            'RL ZCU': (rear_zcu_x, lower_y),
        }

        marker_radius = 13.0 * scale
        camera_x, camera_y = camera_positions['image_01']
        route_x = front_zcu_x - 58.0 * scale
        self._set_connection_pen(
            painter, self._camera_states.get('image_01'), scale)
        self._draw_connection_path(painter, (
            (camera_x, camera_y), (route_x, camera_y),
            (route_x, lower_y), (front_zcu_x - marker_radius, lower_y),
        ))

        camera_x, camera_y = camera_positions['image_02']
        self._set_connection_pen(
            painter, self._camera_states.get('image_02'), scale)
        self._draw_connection_path(painter, (
            (camera_x, camera_y), (front_zcu_x, camera_y),
            (front_zcu_x, lower_y + marker_radius),
        ))

        camera_x, camera_y = camera_positions['image_03']
        route_y = top + body_height * 0.075
        route_x = front_zcu_x - 36.0 * scale
        self._set_connection_pen(
            painter, self._camera_states.get('image_03'), scale)
        self._draw_connection_path(painter, (
            (camera_x, camera_y), (camera_x, route_y),
            (route_x, route_y), (route_x, upper_y),
            (front_zcu_x - marker_radius, upper_y),
        ))

        camera_x, camera_y = camera_positions['image_04']
        route_y = bottom - body_height * 0.075
        route_x = rear_zcu_x + 36.0 * scale
        self._set_connection_pen(
            painter, self._camera_states.get('image_04'), scale)
        self._draw_connection_path(painter, (
            (camera_x, camera_y), (camera_x, route_y),
            (route_x, route_y), (route_x, lower_y),
            (rear_zcu_x + marker_radius, lower_y),
        ))

        camera_x, camera_y = camera_positions['image_05']
        route_x = rear_zcu_x + 58.0 * scale
        self._set_connection_pen(
            painter, self._camera_states.get('image_05'), scale)
        self._draw_connection_path(painter, (
            (camera_x, camera_y), (route_x, camera_y),
            (route_x, upper_y), (rear_zcu_x + marker_radius, upper_y),
        ))

        hpc_half_width = 31.0 * scale
        hpc_routes = {
            'FR ZCU': (
                hpc_x - 100.0 * scale,
                center_y - 9.0 * scale,
                hpc_x - hpc_half_width,
            ),
            'FL ZCU': (
                hpc_x - 70.0 * scale,
                center_y + 9.0 * scale,
                hpc_x - hpc_half_width,
            ),
            'RR ZCU': (
                hpc_x + 100.0 * scale,
                center_y - 9.0 * scale,
                hpc_x + hpc_half_width,
            ),
            'RL ZCU': (
                hpc_x + 70.0 * scale,
                center_y + 9.0 * scale,
                hpc_x + hpc_half_width,
            ),
        }
        for label, (elbow_x, attach_y, attach_x) in hpc_routes.items():
            zcu_x, zcu_y = zcu_positions[label]
            hpc_connection_pen = QPen(
                self._state_color(self._zone_states.get(
                    ZCU_LAYOUT_ZONE_IDS[label])),
                max(2.0, 2.2 * scale))
            hpc_connection_pen.setCapStyle(Qt.RoundCap)
            hpc_connection_pen.setJoinStyle(Qt.RoundJoin)
            painter.setPen(hpc_connection_pen)
            self._draw_connection_path(painter, (
                (zcu_x, zcu_y), (elbow_x, zcu_y),
                (elbow_x, attach_y), (attach_x, attach_y),
            ))

        zone_offset = 34.0 * scale
        self._draw_zone_label(
            painter, front_zcu_x, upper_y - zone_offset, 'FR', scale)
        self._draw_zone_label(
            painter, front_zcu_x, lower_y - zone_offset, 'FL', scale)
        self._draw_zone_label(
            painter, rear_zcu_x, upper_y - zone_offset, 'RR', scale)
        self._draw_zone_label(
            painter, rear_zcu_x, lower_y - zone_offset, 'RL', scale)

        for label, (x, y) in zcu_positions.items():
            self._draw_device(
                painter, x, y, label, 'square', scale,
                state=self._zone_states.get(ZCU_LAYOUT_ZONE_IDS[label]))
        for label, (x, y) in camera_positions.items():
            self._draw_device(
                painter, x, y, label, 'circle', scale,
                label_above=(label == 'image_04'),
                state=self._camera_states.get(label))
        self._draw_hpc(
            painter, hpc_x, center_y,
            'stale' if self._summary_state == 'stale' else 'ok', scale)
        painter.end()

    @classmethod
    def _set_connection_pen(cls, painter, state, scale):
        pen = QPen(cls._state_color(state), max(1.2, 1.8 * scale))
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        painter.setPen(pen)

    @staticmethod
    def _draw_connection_path(painter, points):
        for start, end in zip(points, points[1:]):
            painter.drawLine(
                round(start[0]), round(start[1]),
                round(end[0]), round(end[1]))

    @staticmethod
    def _draw_wheels(
            painter, left, right, top, bottom, body_width, body_height, scale):
        wheel_width = body_width * 0.12
        wheel_height = max(7.0 * scale, body_height * 0.045)
        wheel_xs = (
            left + body_width * 0.18,
            right - body_width * 0.30,
        )
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor('#0b1015'))
        for wheel_x in wheel_xs:
            painter.drawRoundedRect(
                QRectF(wheel_x, top - wheel_height * 0.45,
                       wheel_width, wheel_height),
                wheel_height / 2.0, wheel_height / 2.0)
            painter.drawRoundedRect(
                QRectF(wheel_x, bottom - wheel_height * 0.55,
                       wheel_width, wheel_height),
                wheel_height / 2.0, wheel_height / 2.0)

    @staticmethod
    def _draw_zone_label(painter, x, y, label, scale):
        painter.setPen(QColor('#d5e0eb'))
        painter.setFont(QFont(
            'Sans Serif', round(8.0 * scale), QFont.Bold))
        painter.drawText(
            QRectF(
                x - 22.0 * scale, y,
                44.0 * scale, 20.0 * scale,
            ),
            Qt.AlignCenter,
            label,
        )

    @staticmethod
    def _draw_device(
            painter, x, y, label, shape, scale, label_above=False,
            state=None):
        radius = 13.0 * scale
        color = VehicleDiagnosticsWidget._state_color(state)
        painter.setPen(QPen(color, max(2.0, 2.0 * scale)))
        painter.setBrush(QColor('#34414e'))
        marker = QRectF(x - radius, y - radius, radius * 2.0, radius * 2.0)
        if shape == 'circle':
            painter.drawEllipse(marker)
        else:
            painter.drawRoundedRect(marker, 4.0 * scale, 4.0 * scale)
        painter.setPen(QColor('#c2ced9'))
        painter.setFont(QFont('Sans Serif', round(6.0 * scale)))
        label_y = y - 32.0 * scale if label_above else y + 14.0 * scale
        painter.drawText(
            QRectF(
                x - 38.0 * scale, label_y,
                76.0 * scale, 17.0 * scale,
            ),
            Qt.AlignCenter,
            label,
        )

    @staticmethod
    def _draw_hpc(painter, x, y, state, scale):
        marker = QRectF(
            x - 31.0 * scale, y - 18.0 * scale,
            62.0 * scale, 36.0 * scale,
        )
        painter.setPen(QPen(
            VehicleDiagnosticsWidget._state_color(state),
            max(2.0, 2.0 * scale)))
        painter.setBrush(QColor('#34414e'))
        painter.drawRoundedRect(marker, 7.0 * scale, 7.0 * scale)
        painter.setPen(QColor('#d5e0eb'))
        painter.setFont(QFont(
            'Sans Serif', round(8.0 * scale), QFont.Bold))
        painter.drawText(marker, Qt.AlignCenter, 'HPC')


class VideoPreviewLabel(QLabel):
    """Draw the latest QImage directly without creating scaled frame copies."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._source_image = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumHeight(250)
        self.setText('Select one Preview checkbox above')
        self.setStyleSheet(
            'background: #080d12; border: 1px solid #405064; '
            'color: #8291a1; font-size: 15px;')

    def set_frame(self, image):
        self._source_image = image
        self.setText('')
        self.update()

    def show_placeholder(self, text):
        self._source_image = None
        self.setText(text)
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if self._source_image is None or self._source_image.isNull():
            return
        source_size = self._source_image.size()
        source_size.scale(self.contentsRect().size(), Qt.KeepAspectRatio)
        target = QRectF(
            (self.width() - source_size.width()) / 2.0,
            (self.height() - source_size.height()) / 2.0,
            float(source_size.width()),
            float(source_size.height()),
        )
        painter = QPainter(self)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        painter.drawImage(target, self._source_image)
        painter.end()


class MainWindow(QMainWindow):
    preview_frame_ready = pyqtSignal()

    def __init__(self, ros_bridge):
        super().__init__()
        self.ros = ros_bridge
        self.specs = load_node_specs()
        self.specs['safety_supervisor'].parameters['owner_pid'] = os.getpid()
        self.manager = NodeProcessManager(self.specs, self)
        self.manager.state_changed.connect(self._node_state_changed)
        self.manager.process_finished.connect(self._process_finished)
        self.manager.process_error.connect(self._process_error)

        self.control_source = 'none'
        self.drive_input = {
            'forward': False,
            'reverse': False,
            'left': False,
            'right': False,
            'reset': False,
            'emergency_stop': False,
        }
        self.latest_safety = None
        self._fallback_stop_ticks = 0
        self._supervisor_fault_handled = False
        self._closing = False
        self._ivn_diagnostics_generation = -1
        self._manual_diagnostics_generation = -1
        self._ivn_diagnostics_was_stale = True
        self._ssh_process = None
        self._duplicate_supervisor_reported = False

        self.node_status_labels = {}
        self.node_start_buttons = {}
        self.node_stop_buttons = {}
        self.camera_selectors = {}
        self.camera_window_checks = {}
        self.pipeline_window_checks = {}
        self.preview_checks = {}
        self._preview_source = None
        self._preview_checkbox_update = False
        self._preview_sequence = -1
        self.zcu_live_labels = []

        self.setWindowTitle('AutoSDV HPC Control')
        self.resize(1180, 780)
        self.setMinimumSize(980, 680)
        self._build_ui()
        self._apply_style()
        self.preview_frame_ready.connect(
            self._refresh_preview, Qt.QueuedConnection)
        self.ros.set_preview_listener(self.preview_frame_ready.emit)

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self._publish_control)
        self.control_timer.start(50)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_ros_status)
        self.status_timer.start(100)

        stopped, remaining = terminate_stale_safety_supervisors()
        if stopped:
            self.statusBar().showMessage(
                f'Removed {stopped} stale safety supervisor process(es)', 7000)
        if remaining:
            self._show_error(
                'Could not stop stale Safety Supervisor process(es): '
                + ', '.join(str(pid) for pid in remaining))
            return

        ok, message = self.manager.start('safety_supervisor')
        if not ok:
            self._show_error(message)
        ok, message = self.manager.start('ivn_diagnostics')
        if not ok:
            self._show_error(message)

    def _build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel('AutoSDV HPC Control')
        title.setObjectName('title')
        header.addWidget(title)
        header.addStretch()
        self.supervisor_label = QLabel('Supervisor: STARTING')
        self.source_label = QLabel('Active source: NONE')
        self.estop_state_label = QLabel('E-Stop: UNKNOWN')
        header.addWidget(self.supervisor_label)
        header.addWidget(self.source_label)
        header.addWidget(self.estop_state_label)
        self.exit_button = QPushButton('Exit GUI')
        self.exit_button.setObjectName('exitButton')
        self.exit_button.clicked.connect(self._request_exit)
        header.addWidget(self.exit_button)
        layout.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_node_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 6)
        layout.addWidget(splitter, 1)

        estop = QPushButton('EMERGENCY STOP  [SPACE]')
        estop.setObjectName('estopButton')
        estop.setMinimumHeight(62)
        estop.clicked.connect(self.emergency_stop)
        layout.addWidget(estop)

        self.setCentralWidget(root)

    def _build_node_panel(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 5, 0)

        camera_group = QGroupBox('Camera Nodes')
        camera_layout = QGridLayout(camera_group)
        camera_layout.addWidget(QLabel('Select'), 0, 0)
        camera_layout.addWidget(QLabel('Camera'), 0, 1)
        camera_layout.addWidget(QLabel('Window'), 0, 2)
        camera_layout.addWidget(QLabel('Preview'), 0, 3)
        camera_layout.addWidget(QLabel('State'), 0, 4)
        for row, node_id in enumerate(CAMERA_IDS, start=1):
            selector = QCheckBox()
            window_check = QCheckBox()
            preview_check = QCheckBox()
            window_check.setChecked(True)
            status = self._status_label('STOPPED')
            self.camera_selectors[node_id] = selector
            self.camera_window_checks[node_id] = window_check
            self.preview_checks[node_id] = preview_check
            self.node_status_labels[node_id] = status
            window_check.toggled.connect(
                lambda checked, key=node_id: self._camera_window_changed(
                    key, checked))
            preview_check.toggled.connect(
                lambda checked, key=node_id: self._preview_changed(
                    key, checked))
            camera_layout.addWidget(selector, row, 0, alignment=Qt.AlignCenter)
            camera_layout.addWidget(QLabel(self.specs[node_id].label), row, 1)
            camera_layout.addWidget(window_check, row, 2, alignment=Qt.AlignCenter)
            camera_layout.addWidget(preview_check, row, 3, alignment=Qt.AlignCenter)
            camera_layout.addWidget(status, row, 4)

        camera_buttons = QHBoxLayout()
        start_selected = QPushButton('Start Selected')
        stop_selected = QPushButton('Stop Selected')
        start_selected.clicked.connect(self._start_selected_cameras)
        stop_selected.clicked.connect(self._stop_selected_cameras)
        camera_buttons.addWidget(start_selected)
        camera_buttons.addWidget(stop_selected)
        camera_layout.addLayout(camera_buttons, len(CAMERA_IDS) + 1, 0, 1, 5)
        layout.addWidget(camera_group)

        node_group = QGroupBox('Perception and Control Nodes')
        node_layout = QGridLayout(node_group)
        node_layout.addWidget(QLabel('Node'), 0, 0)
        node_layout.addWidget(QLabel('Window'), 0, 1)
        node_layout.addWidget(QLabel('Preview'), 0, 2)
        node_layout.addWidget(QLabel('State'), 0, 3)
        for row, node_id in enumerate(PIPELINE_IDS, start=1):
            status = self._status_label('STOPPED')
            start = QPushButton('Start')
            stop = QPushButton('Stop')
            window_check = QCheckBox()
            preview_check = QCheckBox()
            window_check.setEnabled(node_id in ('lane', 'path_planner'))
            window_check.setChecked(node_id in ('lane', 'path_planner'))
            preview_check.setEnabled(node_id in PREVIEW_TOPICS)
            if node_id in ('lane', 'path_planner'):
                window_check.toggled.connect(
                    lambda checked, key=node_id:
                    self._pipeline_window_changed(key, checked))
            self.pipeline_window_checks[node_id] = window_check
            self.preview_checks[node_id] = preview_check
            preview_check.toggled.connect(
                lambda checked, key=node_id: self._preview_changed(
                    key, checked))
            stop.setEnabled(False)
            start.clicked.connect(
                lambda checked=False, key=node_id: self._start_node(key))
            stop.clicked.connect(
                lambda checked=False, key=node_id: self._stop_node(key))
            self.node_status_labels[node_id] = status
            self.node_start_buttons[node_id] = start
            self.node_stop_buttons[node_id] = stop
            node_layout.addWidget(QLabel(self.specs[node_id].label), row, 0)
            node_layout.addWidget(window_check, row, 1, alignment=Qt.AlignCenter)
            node_layout.addWidget(preview_check, row, 2, alignment=Qt.AlignCenter)
            node_layout.addWidget(status, row, 3)
            node_layout.addWidget(start, row, 4)
            node_layout.addWidget(stop, row, 5)
        layout.addWidget(node_group)
        layout.addWidget(self._build_preview_panel(), 1)
        return container

    def _build_preview_panel(self):
        group = QGroupBox('Single Video Preview')
        layout = QVBoxLayout(group)
        self.preview_source_label = QLabel('Source: NONE')
        self.preview_source_label.setAlignment(Qt.AlignCenter)
        self.video_preview = VideoPreviewLabel()
        layout.addWidget(self.preview_source_label)
        layout.addWidget(self.video_preview, 1)
        return group

    def _build_right_panel(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(5, 0, 0, 0)
        layout.addWidget(self._build_drive_panel())
        layout.addWidget(self._build_diagnostic_panel(), 1)
        layout.addWidget(self._build_zcu_management_panel())
        return container

    def _build_drive_panel(self):
        group = QGroupBox('Driving Control')
        layout = QGridLayout(group)
        self.direction_value = self._value_label('STOPPED')
        self.steering_value = self._value_label('0')
        self.left_speed_value = self._value_label('0')
        self.right_speed_value = self._value_label('0')
        layout.addWidget(QLabel('Direction'), 0, 0)
        layout.addWidget(self.direction_value, 0, 1)
        layout.addWidget(QLabel('Steering'), 0, 2)
        layout.addWidget(self.steering_value, 0, 3)
        layout.addWidget(QLabel('Left speed'), 1, 0)
        layout.addWidget(self.left_speed_value, 1, 1)
        layout.addWidget(QLabel('Right speed'), 1, 2)
        layout.addWidget(self.right_speed_value, 1, 3)

        controls = QHBoxLayout()
        for text, field in (
                ('Left [A]', 'left'), ('Forward [W]', 'forward'),
                ('Reverse [S]', 'reverse'), ('Right [D]', 'right')):
            button = QPushButton(text)
            button.setMinimumHeight(42)
            button.pressed.connect(
                lambda key=field: self._set_drive_field(key, True))
            button.released.connect(
                lambda key=field: self._set_drive_field(key, False))
            controls.addWidget(button)
        layout.addLayout(controls, 2, 0, 1, 4)

        reset = QPushButton('Center Steering [R]')
        reset.clicked.connect(self._pulse_reset)
        reset_estop = QPushButton('Reset E-Stop')
        reset_estop.clicked.connect(self.ros.request_estop_reset)
        layout.addWidget(reset, 3, 0, 1, 2)
        layout.addWidget(reset_estop, 3, 2, 1, 2)
        return group

    def _build_diagnostic_panel(self):
        group = QGroupBox('ZCU Diagnostics')
        layout = QVBoxLayout(group)
        self.vehicle_diagnostics = VehicleDiagnosticsWidget()
        layout.addWidget(self.vehicle_diagnostics, 4)

        status_legend = QLabel(
            '<b>Status</b>&nbsp;&nbsp;'
            '<span style="color:#39c86a;">●</span> OK 정상&nbsp;&nbsp;'
            '<span style="color:#e1a73a;">●</span> WARN 경고&nbsp;&nbsp;'
            '<span style="color:#e25757;">●</span> ERROR 장애&nbsp;&nbsp;'
            '<span style="color:#697887;">●</span> STALE 미수신/시간초과')
        status_legend.setWordWrap(True)
        status_legend.setAlignment(Qt.AlignCenter)
        status_legend.setStyleSheet(
            'background: #101820; border: 1px solid #405064; '
            'border-radius: 4px; padding: 5px; color: #c2ced9;')
        status_legend.setToolTip(
            'OK: all checks normal\n'
            'WARN: component warning\n'
            'ERROR: network, DDS, or ZCU health failure\n'
            'STALE: status not received or timed out')
        layout.addWidget(status_legend)

        run_button = QPushButton('Run Manual Deep Check')
        run_button.clicked.connect(self._run_manual_diagnostics)
        self.diagnostic_button = run_button
        layout.addWidget(run_button)
        self.manual_diagnostics_progress = QLabel('Manual check: IDLE')
        layout.addWidget(self.manual_diagnostics_progress)
        self.diagnostic_output = QTextEdit()
        self.diagnostic_output.setReadOnly(True)
        self.diagnostic_output.setPlaceholderText(
            'Manual deep-check results will appear here.')
        self.diagnostic_output.setMinimumHeight(130)
        layout.addWidget(self.diagnostic_output, 2)
        return group

    def _build_zcu_management_panel(self):
        group = QGroupBox('ZCU Management (SSH)')
        layout = QHBoxLayout(group)
        self.zcu_selector = QComboBox()
        for index, ip in enumerate(
                ('10.0.0.2', '10.0.0.3', '10.0.0.4', '10.0.0.5'), start=1):
            self.zcu_selector.addItem(f'Zone {index} - {ip}', ip)
        layout.addWidget(self.zcu_selector, 1)
        for label, action in (
                ('Service Status', 'status'),
                ('Restart zcu.service', 'restart'),
                ('Reboot ZCU', 'reboot')):
            button = QPushButton(label)
            button.clicked.connect(
                lambda checked=False, requested=action: self._run_zcu_action(
                    requested))
            layout.addWidget(button)
        return group

    @staticmethod
    def _status_label(text):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(82)
        label.setProperty('status', 'neutral')
        return label

    @staticmethod
    def _value_label(text):
        label = QLabel(text)
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        label.setFont(font)
        return label

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #18212b; color: #e8eef5; }
            QLabel#title { font-size: 24px; font-weight: 700; }
            QGroupBox {
                border: 1px solid #405064; border-radius: 6px;
                margin-top: 10px; padding-top: 9px; font-weight: 600;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; }
            QCheckBox::indicator {
                width: 18px; height: 18px; border-radius: 3px;
                border: 2px solid #a9bacb; background: #0b1219;
            }
            QCheckBox::indicator:hover { border-color: #dbe9f6; }
            QCheckBox::indicator:checked {
                background: #2f80ed; border: 2px solid #9ac7ff;
            }
            QCheckBox::indicator:disabled {
                background: #26313c; border-color: #566574;
            }
            QPushButton {
                background: #2c3d50; border: 1px solid #52677e;
                border-radius: 4px; padding: 7px 10px;
            }
            QPushButton:hover { background: #39516a; }
            QPushButton:disabled { color: #66717e; background: #222d38; }
            QPushButton#estopButton {
                background: #a31919; border: 2px solid #ef6565;
                color: white; font-size: 20px; font-weight: 800;
            }
            QPushButton#estopButton:hover { background: #c32121; }
            QTextEdit, QComboBox {
                background: #101820; border: 1px solid #405064;
                border-radius: 4px; padding: 4px;
            }
        """)

    def eventFilter(self, obj, event):
        if event.type() not in (QEvent.KeyPress, QEvent.KeyRelease):
            return super().eventFilter(obj, event)
        if event.isAutoRepeat():
            return True
        pressed = event.type() == QEvent.KeyPress
        key_map = {
            Qt.Key_W: 'forward',
            Qt.Key_S: 'reverse',
            Qt.Key_A: 'left',
            Qt.Key_D: 'right',
        }
        if event.key() in key_map:
            self._set_drive_field(key_map[event.key()], pressed)
            return True
        if pressed and event.key() == Qt.Key_R:
            self._pulse_reset()
            return True
        if pressed and event.key() == Qt.Key_Space:
            self.emergency_stop()
            return True
        return super().eventFilter(obj, event)

    def _set_drive_field(self, field, value):
        if self.control_source != 'manual' or self.manager.state('drive') != 'RUNNING':
            if value:
                self.statusBar().showMessage(
                    'Start Manual Drive before using driving controls', 2500)
            return
        self.drive_input[field] = value

    def _pulse_reset(self):
        if self.control_source == 'manual':
            self.drive_input['reset'] = True

    def _publish_control(self):
        if self._closing:
            return
        self.ros.publish_heartbeat(self.control_source)
        if self.manager.state('drive') == 'RUNNING':
            self.ros.publish_drive_input(self.drive_input)
        if self._fallback_stop_ticks > 0:
            self.ros.publish_fallback_stop()
            self._fallback_stop_ticks -= 1
        self.drive_input['reset'] = False
        self.drive_input['emergency_stop'] = False

    def _refresh_ros_status(self):
        command, safety, safety_age, events = self.ros.snapshot()
        self.latest_safety = safety
        self.steering_value.setText(str(command.steering))
        self.left_speed_value.setText(str(command.left_speed))
        self.right_speed_value.setText(str(command.right_speed))
        average = (command.left_speed + command.right_speed) / 2.0
        direction = 'FORWARD' if average > 0 else 'REVERSE' if average < 0 else 'STOPPED'
        self.direction_value.setText(direction)

        supervisor_publishers = self.ros.safety_supervisor_count()
        duplicate_supervisor = supervisor_publishers > 1
        if duplicate_supervisor:
            self.supervisor_label.setText(
                f'Supervisor: DUPLICATE ({supervisor_publishers})')
            self.source_label.setText('Active source: NONE')
            self.estop_state_label.setText('E-Stop: FAULT')
            self.estop_state_label.setStyleSheet(
                'color: #ff6666; font-weight: bold;')
            if not self._duplicate_supervisor_reported:
                self._duplicate_supervisor_reported = True
                self.statusBar().showMessage(
                    'Multiple Safety Supervisors detected; driving is blocked',
                    10000)
        else:
            self._duplicate_supervisor_reported = False

        if safety is not None and not duplicate_supervisor:
            if (safety_age is not None and safety_age <= 1.0
                    and self.manager.state('safety_supervisor') == 'RUNNING'):
                self._supervisor_fault_handled = False
            self.source_label.setText(
                f'Active source: {safety.active_source.upper()}')
            self.estop_state_label.setText(
                'E-Stop: ACTIVE' if safety.emergency_stop else 'E-Stop: READY')
            self.estop_state_label.setStyleSheet(
                'color: #ff6666; font-weight: bold;' if safety.emergency_stop
                else 'color: #6fe58a; font-weight: bold;')
            for index, label in enumerate(self.zcu_live_labels):
                connected = (
                    index < len(safety.zcu_connected)
                    and safety.zcu_connected[index])
                label.setText(f'Z{index + 1}: ' + ('OK' if connected else 'DOWN'))
                label.setStyleSheet(
                    'color: #6fe58a;' if connected else 'color: #ff6666;')
            if safety.emergency_stop and safety.reason:
                self.statusBar().showMessage(safety.reason)

        for success, message in events:
            self.statusBar().showMessage(message, 5000)
            if not success:
                self._show_error(message)
        self._refresh_ivn_diagnostics()

    @staticmethod
    def _diagnostic_level_state(level):
        if isinstance(level, (bytes, bytearray)):
            level = level[0] if level else 0
        else:
            level = int(level)
        return {
            0: 'ok',
            1: 'warn',
            2: 'error',
            3: 'stale',
        }.get(level, 'stale')

    @staticmethod
    def _worst_diagnostic_state(current, candidate):
        rank = {'ok': 0, 'warn': 1, 'stale': 2, 'error': 3}
        return candidate if rank[candidate] > rank[current] else current

    @classmethod
    def _graph_diagnostic_states(cls, statuses):
        """Map component diagnostics to the matching vehicle-graph item."""
        zone_states = {}
        camera_states = {}
        zone_status_suffixes = ('/Network', '/Runtime')
        for status in statuses:
            state = cls._diagnostic_level_state(status.level)
            if (status.hardware_id.startswith('zcu-zone_')
                    and status.name.endswith(zone_status_suffixes)):
                zone_id = status.hardware_id[len('zcu-'):]
                previous = zone_states.get(zone_id, 'ok')
                zone_states[zone_id] = cls._worst_diagnostic_state(
                    previous, state)
            elif status.hardware_id.startswith('camera-image_'):
                image_id = status.hardware_id[len('camera-'):]
                previous = camera_states.get(image_id, 'ok')
                camera_states[image_id] = cls._worst_diagnostic_state(
                    previous, state)
        return zone_states, camera_states

    def _refresh_ivn_diagnostics(self):
        message, age, generation, manual = self.ros.diagnostic_snapshot()
        stale = message is None or age is None or age > 3.0
        if (generation != self._ivn_diagnostics_generation
                or stale != self._ivn_diagnostics_was_stale):
            self._ivn_diagnostics_generation = generation
            self._ivn_diagnostics_was_stale = stale
            if stale:
                self.vehicle_diagnostics.set_diagnostics('stale', {}, {})
            else:
                summary_state = 'stale'
                for status in message.status:
                    if status.name == 'IVN/Summary':
                        summary_state = self._diagnostic_level_state(
                            status.level)
                zone_states, camera_states = self._graph_diagnostic_states(
                    message.status)
                self.vehicle_diagnostics.set_diagnostics(
                    summary_state, zone_states, camera_states)

        if manual['generation'] == self._manual_diagnostics_generation:
            return
        self._manual_diagnostics_generation = manual['generation']
        state = manual['state']
        running = state in ('requesting', 'running')
        self.diagnostic_button.setEnabled(not running)
        if running:
            completed = manual['completed']
            total = manual['total']
            progress = f'{completed}/{total}' if total else 'requesting'
            current = f" - {manual['current']}" if manual['current'] else ''
            self.manual_diagnostics_progress.setText(
                f'Manual check: RUNNING ({progress}){current}')
            return
        self.manual_diagnostics_progress.setText(
            f'Manual check: {state.upper()}')
        if state not in ('complete', 'failed'):
            return
        lines = [manual['summary'] or 'Manual check finished']
        for status in manual['results']:
            check_state = self._diagnostic_level_state(status.level).upper()
            lines.append(
                f'[{check_state}] {status.name}: {status.message}')
        self.diagnostic_output.setPlainText('\n'.join(lines))

    def _start_selected_cameras(self):
        selected = [
            node_id for node_id, checkbox in self.camera_selectors.items()
            if checkbox.isChecked()
        ]
        if not selected:
            self._show_error('Select at least one camera')
            return
        for node_id in selected:
            if self.manager.state(node_id) in ('STOPPED', 'ERROR'):
                self._start_node(node_id)

    def _stop_selected_cameras(self):
        selected = [
            node_id for node_id, checkbox in self.camera_selectors.items()
            if checkbox.isChecked()
        ]
        for node_id in reversed(selected):
            if self.manager.is_running(node_id):
                self._stop_node(node_id)

    def _start_node(self, node_id):
        if node_id == 'drive' and self.manager.is_running('motion_planner'):
            self._show_error('Stop Motion Planner before starting Manual Drive')
            return
        if node_id == 'motion_planner' and self.manager.is_running('drive'):
            self._show_error('Stop Manual Drive before starting Motion Planner')
            return
        if node_id in ('drive', 'motion_planner'):
            supervisor_count = self.ros.safety_supervisor_count()
            if supervisor_count != 1:
                self._show_error(
                    'Exactly one Safety Supervisor is required before driving '
                    f'(detected: {supervisor_count})')
                return
            if self.latest_safety is None:
                self._show_error('Safety supervisor status is not available yet')
                return
            if self.latest_safety.emergency_stop:
                self._show_error('Reset E-Stop before starting a control node')
                return
            if not all(self.latest_safety.zcu_connected):
                self._show_error('All four ZCUs must be connected before driving')
                return

        ok, message = self.manager.start(node_id)
        if not ok:
            self._show_error(message)

    def _stop_node(self, node_id):
        if node_id == 'path_planner' and self.manager.is_running(
                'path_visualizer'):
            self.manager.stop('path_visualizer')
        if node_id == 'yolo' and self.manager.is_running(
                'yolo_visualizer'):
            self.manager.stop('yolo_visualizer')
        if node_id == 'drive' and self.control_source == 'manual':
            self._clear_drive_input()
            self.control_source = 'none'
            self.ros.publish_heartbeat('none')
        elif node_id == 'motion_planner' and self.control_source == 'auto':
            self.control_source = 'none'
            self.ros.publish_heartbeat('none')
        ok, message = self.manager.stop(node_id)
        if not ok:
            self._show_error(message)

    def _camera_window_changed(self, node_id, checked):
        if checked:
            preview_check = self.preview_checks.get(node_id)
            if preview_check is not None and preview_check.isChecked():
                preview_check.setChecked(False)
        self.specs[node_id].parameters['show_image'] = checked
        self.manager.set_ros_parameter(node_id, 'show_image', checked)

    def _pipeline_window_changed(self, node_id, checked):
        if checked:
            preview_check = self.preview_checks.get(node_id)
            if preview_check is not None and preview_check.isChecked():
                preview_check.setChecked(False)

        if node_id == 'lane':
            self.specs['lane'].parameters['show_image'] = checked
            self.manager.set_ros_parameter('lane', 'show_image', checked)
        elif node_id == 'path_planner':
            self.specs['path_visualizer'].parameters['show_image'] = checked
            if self.manager.state('path_visualizer') == 'RUNNING':
                self.manager.set_ros_parameter(
                    'path_visualizer', 'show_image', checked)
        self._sync_preview_renderers()

    def _preview_changed(self, node_id, checked):
        if self._preview_checkbox_update:
            return
        if checked:
            self._preview_source = node_id
            window_check = self.camera_window_checks.get(node_id)
            if window_check is None:
                window_check = self.pipeline_window_checks.get(node_id)
            if window_check is not None and window_check.isChecked():
                window_check.setChecked(False)

            self._preview_checkbox_update = True
            try:
                for key, checkbox in self.preview_checks.items():
                    if key != node_id and checkbox.isChecked():
                        checkbox.setChecked(False)
            finally:
                self._preview_checkbox_update = False
            topic = PREVIEW_TOPICS[node_id]
            self._preview_sequence = -1
            # A small C++ bridge performs the expensive raw Image
            # deserialization and JPEG encoding. The Python GUI receives only
            # the compact CompressedImage to keep its event loop responsive.
            self.ros.select_preview_topic('')
            self.ros.select_preview_topic(PREVIEW_OUTPUT_TOPIC)
            self.ros.select_preview_source(topic)
            self.preview_source_label.setText(
                f'Source: {self.specs[node_id].label}  ({topic})')
            self.video_preview.show_placeholder(
                f'Waiting for image from {topic}')
        elif self._preview_source == node_id:
            self._preview_source = None
            self._preview_sequence = -1
            self.ros.select_preview_source('')
            self.ros.select_preview_topic('')
            self.preview_source_label.setText('Source: NONE')
            self.video_preview.show_placeholder(
                'Select one Preview checkbox above')
        self._sync_preview_renderers()

    def _refresh_preview(self):
        if self._closing:
            return
        message, sequence, topic = self.ros.take_preview_snapshot()
        if sequence == self._preview_sequence:
            return
        self._preview_sequence = sequence
        if message is None or not topic:
            return
        try:
            image = compressed_image_to_qimage(message)
            self.video_preview.set_frame(image)
        except Exception as exc:
            self.video_preview.show_placeholder(
                f'Preview conversion failed: {exc}')

    def _sync_preview_renderers(self):
        if self._closing:
            return

        self._set_renderer_state(
            'preview_bridge', self._preview_source is not None)

        yolo_renderer_needed = (
            self._preview_source == 'yolo'
            and self.manager.state('yolo') == 'RUNNING'
        )
        self._set_renderer_state(
            'yolo_visualizer', yolo_renderer_needed)

        path_running = self.manager.state('path_planner') == 'RUNNING'
        path_window = self.pipeline_window_checks[
            'path_planner'].isChecked()
        path_renderer_needed = path_running and (
            path_window or self._preview_source == 'path_planner')
        self.specs['path_visualizer'].parameters['show_image'] = path_window
        self._set_renderer_state(
            'path_visualizer', path_renderer_needed)

    def _set_renderer_state(self, node_id, needed):
        state = self.manager.state(node_id)
        if needed and state in ('STOPPED', 'ERROR'):
            ok, message = self.manager.start(node_id)
            if not ok:
                self.statusBar().showMessage(message, 5000)
        elif not needed and state in ('STARTING', 'RUNNING'):
            ok, message = self.manager.stop(node_id)
            if not ok:
                self.statusBar().showMessage(message, 5000)

    def _node_state_changed(self, node_id, state):
        if node_id == 'safety_supervisor':
            self.supervisor_label.setText(f'Supervisor: {state}')
            return
        label = self.node_status_labels.get(node_id)
        if label is not None:
            label.setText(state)
            color = {
                'RUNNING': '#6fe58a', 'ERROR': '#ff6666',
                'STARTING': '#f2c94c', 'STOPPING': '#f2c94c',
            }.get(state, '#aeb9c5')
            label.setStyleSheet(f'color: {color}; font-weight: bold;')
        start = self.node_start_buttons.get(node_id)
        stop = self.node_stop_buttons.get(node_id)
        if start is not None:
            start.setEnabled(state in ('STOPPED', 'ERROR'))
        if stop is not None:
            stop.setEnabled(state in ('STARTING', 'RUNNING'))

        if state == 'RUNNING' and node_id == 'drive':
            self.control_source = 'manual'
        elif state == 'RUNNING' and node_id == 'motion_planner':
            self.control_source = 'auto'
        if (node_id in ('yolo', 'path_planner')
                or (node_id in ('yolo_visualizer', 'path_visualizer')
                    and state == 'STOPPED')):
            QTimer.singleShot(0, self._sync_preview_renderers)
        if (node_id == self._preview_source
                and state in ('STOPPED', 'ERROR')):
            topic = PREVIEW_TOPICS[node_id]
            self.video_preview.show_placeholder(
                f'Waiting for image from {topic}')

    def _process_finished(self, node_id, expected, exit_code):
        del exit_code
        if (node_id == 'path_planner'
                and self.manager.is_running('path_visualizer')):
            self.manager.stop('path_visualizer')
        if (node_id == 'yolo'
                and self.manager.is_running('yolo_visualizer')):
            self.manager.stop('yolo_visualizer')
        if expected:
            return
        if node_id == 'safety_supervisor':
            self._handle_supervisor_fault('Safety supervisor stopped unexpectedly')
        elif node_id in CONTROL_CHAIN and self.control_source != 'none':
            self.emergency_stop()

    def _process_error(self, node_id, detail):
        self.statusBar().showMessage(
            f'{self.specs[node_id].label}: {detail}', 7000)

    def emergency_stop(self):
        active_node = (
            'drive' if self.control_source == 'manual'
            else 'motion_planner' if self.control_source == 'auto'
            else None
        )
        self.drive_input['emergency_stop'] = True
        self.ros.publish_drive_input(self.drive_input)
        self.ros.request_emergency_stop()
        self._fallback_stop_ticks = max(self._fallback_stop_ticks, 10)
        self.control_source = 'none'
        self.ros.publish_heartbeat('none')
        self._clear_drive_input()
        if active_node is not None and self.manager.is_running(active_node):
            QTimer.singleShot(100, lambda: self._stop_node(active_node))

    def _handle_supervisor_fault(self, reason):
        self._supervisor_fault_handled = True
        self._fallback_stop_ticks = max(self._fallback_stop_ticks, 40)
        active_node = (
            'drive' if self.control_source == 'manual'
            else 'motion_planner' if self.control_source == 'auto'
            else None
        )
        self.control_source = 'none'
        self._clear_drive_input()
        if active_node is not None and self.manager.is_running(active_node):
            self.manager.stop(active_node)
        self._show_error(reason + '. Fallback stop commands are being published.')

    def _clear_drive_input(self):
        for key in self.drive_input:
            self.drive_input[key] = False
        self.ros.publish_drive_input(self.drive_input)

    def _run_manual_diagnostics(self):
        ok, message = self.ros.run_manual_diagnostics()
        if not ok:
            self.manual_diagnostics_progress.setText(
                'Manual check: UNAVAILABLE')
            self.diagnostic_output.setPlainText(message)
            return
        self.diagnostic_output.setPlainText(
            'Manual deep check requested. Waiting for progress...')

    def _run_zcu_action(self, action):
        if self._ssh_process is not None:
            self._show_error('Another ZCU command is still running')
            return
        ip = self.zcu_selector.currentData()
        commands = {
            'status': ['systemctl', 'is-active', 'zcu.service'],
            'restart': ['systemctl', 'restart', 'zcu.service'],
            'reboot': ['systemctl', 'reboot'],
        }
        if action != 'status':
            description = 'restart zcu.service' if action == 'restart' else 'reboot the ZCU'
            answer = QMessageBox.question(
                self, 'Confirm ZCU action',
                f'Do you want to {description} on {ip}?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return

        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.finished.connect(
            lambda code, status, requested=action, host=ip:
            self._zcu_action_finished(requested, host, code))
        arguments = [
            '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
            f'root@{ip}', *commands[action],
        ]
        self._ssh_process = process
        self.statusBar().showMessage(f'Running {action} on {ip}...')
        process.start('ssh', arguments)

    def _zcu_action_finished(self, action, ip, exit_code):
        process = self._ssh_process
        output = '' if process is None else bytes(
            process.readAllStandardOutput()).decode('utf-8', errors='replace').strip()
        self._ssh_process = None
        if process is not None:
            process.deleteLater()
        if exit_code == 0:
            message = output or f'{action} completed on {ip}'
            self.statusBar().showMessage(message, 7000)
        else:
            self._show_error(output or f'{action} failed on {ip} (exit {exit_code})')

    def _show_error(self, message):
        QMessageBox.warning(self, 'AutoSDV', html.escape(str(message)))

    def _request_exit(self):
        answer = QMessageBox.question(
            self,
            'Exit AutoSDV',
            'Stop all managed nodes and exit the GUI?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.close()

    def closeEvent(self, event):
        if self._closing:
            event.accept()
            return
        self._closing = True
        self.ros.set_preview_listener(None)
        self.ros.select_preview_source('')
        self.ros.select_preview_topic('')
        self._clear_drive_input()
        self.ros.request_emergency_stop()
        self.control_source = 'none'
        self.ros.publish_heartbeat('none')
        QApplication.processEvents()
        self.ros.cancel_manual_diagnostics()
        ssh_process = self._ssh_process
        if (ssh_process is not None
                and ssh_process.state() != QProcess.NotRunning):
            ssh_process.terminate()
            if not ssh_process.waitForFinished(1000):
                ssh_process.kill()
                ssh_process.waitForFinished(500)
        self.manager.shutdown(SHUTDOWN_ORDER)
        event.accept()


def main(args=None):
    configure_default_fastdds_profile()
    remove_opencv_qt_plugin_paths()
    app = QApplication(sys.argv)
    app.setApplicationName('AutoSDV HPC Control')
    instance_lock = QLockFile(f'/tmp/autosdv_hpc_gui_{os.getuid()}.lock')
    instance_lock.setStaleLockTime(0)
    if not instance_lock.tryLock(0):
        QMessageBox.critical(
            None, 'AutoSDV',
            'AutoSDV HPC Control is already running.\n'
            'Close the existing GUI before starting another instance.')
        return 1

    rclpy.init(args=args)
    ros_bridge = GuiRosBridge()
    # MultiThreadedExecutor busy-spins at roughly one full CPU core when a
    # 30 FPS image subscription is active on this ROS 2/Python stack. All GUI
    # callbacks are intentionally lightweight, so one executor thread is both
    # sufficient and substantially smoother.
    executor = SingleThreadedExecutor()
    executor.add_node(ros_bridge)
    ros_thread = threading.Thread(
        target=executor.spin, name='autosdv-gui-ros', daemon=True)
    ros_thread.start()

    palette = app.palette()
    palette.setColor(QPalette.Highlight, QColor('#477ba8'))
    app.setPalette(palette)
    window = MainWindow(ros_bridge)
    app.installEventFilter(window)
    window.show()
    result = app.exec_()

    executor.shutdown(timeout_sec=1.0)
    ros_bridge.destroy_node()
    rclpy.try_shutdown()
    ros_thread.join(timeout=1.0)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
