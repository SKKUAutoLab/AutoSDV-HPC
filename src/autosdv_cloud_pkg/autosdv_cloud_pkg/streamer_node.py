"""Camera 01 ROS image to H.264 WHIP streamer."""

import os
import random
import threading
import time
import uuid

import gi
gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst  # noqa: E402,I201
from sensor_msgs.msg import Image  # noqa: E402,I100,I201
import rclpy  # noqa: E402,I100,I201
from rclpy.node import Node  # noqa: E402,I201
from rclpy.qos import qos_profile_sensor_data  # noqa: E402,I201

from .contract import CAMERA_ID, SCHEMA_VERSION, VEHICLE_ID  # noqa: E402
from .mqtt_client import MqttPublisher  # noqa: E402


ENCODINGS = {
    'bgr8': ('BGR', 3),
    'rgb8': ('RGB', 3),
    'mono8': ('GRAY8', 1),
    '8uc1': ('GRAY8', 1),
    '8uc3': ('BGR', 3),
}


def classify_error(text):
    """Map GStreamer and WHIP errors to the agreed error codes."""
    lower = text.lower()
    if '401' in lower or '403' in lower:
        return 'AUTH_FAILED'
    if '404' in lower:
        return 'PATH_REJECTED'
    if any(code in lower for code in ('400', '406', '415')):
        return 'SDP_REJECTED'
    if any(f'{code}' in lower for code in range(500, 600)):
        return 'SERVER_ERROR'
    if ('encoder' in lower or 'nvv4l2h264enc' in lower
            or 'not-negotiated' in lower):
        return 'ENCODER_ERROR'
    if 'ice' in lower:
        return 'ICE_FAILED'
    return 'SIGNALING_TIMEOUT'


class CameraStreamerNode(Node):
    """Encode the newest Camera 01 frame and publish it through WHIP."""

    def __init__(self):
        """Configure ROS, MQTT, and the GLib event loop."""
        super().__init__('autosdv_camera_streamer')
        Gst.init(None)
        self.vehicle_id = str(self.declare_parameter(
            'vehicle_id', VEHICLE_ID).value)
        self.camera_id = str(self.declare_parameter(
            'camera_id', CAMERA_ID).value)
        self.image_topic = str(self.declare_parameter(
            'image_topic', '/image_01').value)
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = int(self.declare_parameter('fps', 30).value)
        self.bitrate = int(self.declare_parameter(
            'bitrate_kbps', 1500).value)
        default_whip_url = (
            f'http://100.74.147.75:8889/autosdv/'
            f'{self.vehicle_id}/{self.camera_id}/whip')
        self.whip_url = str(self.declare_parameter(
            'whip_url', os.environ.get('WHIP_URL', default_whip_url)).value)
        host = str(self.declare_parameter(
            'mqtt_host', '100.74.147.75').value)
        port = int(self.declare_parameter('mqtt_port', 1883).value)
        mqtt_user = os.environ.get('AUTOSDV_MQTT_USER', 'vehicle')
        mqtt_password = os.environ.get('AUTOSDV_MQTT_PASSWORD', '')
        self.whip_token = ':'.join((
            os.environ.get('AUTOSDV_WHIP_USER', ''),
            os.environ.get('AUTOSDV_WHIP_PASSWORD', ''))).strip(':')
        self.status_topic = (
            f'autosdv/{self.vehicle_id}/streamer/status')
        self.seq = 0
        self.session_id = None
        self.state = 'STARTING'
        self.error_code = None
        self.pipeline = None
        self.appsrc = None
        self._ice_refs = []
        self.last_frame_at = None
        self.started_at = None
        self.frame_count = 0
        self._reconnect_delay = 1.0
        self._retry_at = 0.0
        self.mqtt = MqttPublisher(
            host, port, f'autosdv-{self.vehicle_id}-streamer',
            mqtt_user, mqtt_password, self.status_topic,
            {'schema_version': SCHEMA_VERSION, 'online': False},
            on_connected=self._publish_status)
        self.mqtt.start()
        self.create_subscription(
            Image, self.image_topic, self._image_callback,
            qos_profile_sensor_data)
        self.create_timer(1.0, self._tick)
        self._glib_loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(
            target=self._glib_loop.run, daemon=True)
        self._glib_thread.start()
        self.get_logger().info(
            f'Camera stream: {self.image_topic} -> {self.whip_url}')

    def _pipeline_description(self, gst_format):
        interval = max(1, self.fps)
        return (
            'appsrc name=source is-live=true format=time do-timestamp=true '
            'block=false max-buffers=1 leaky-type=downstream '
            f'caps=video/x-raw,format={gst_format},width={self.width},'
            f'height={self.height},framerate={self.fps}/1 ! '
            'queue max-size-buffers=1 leaky=downstream ! videoconvert ! '
            'video/x-raw,format=I420 ! nvvidconv ! '
            'video/x-raw(memory:NVMM),format=NV12 ! '
            f'nvv4l2h264enc profile=0 bitrate={self.bitrate * 1000} '
            'control-rate=1 num-B-Frames=0 insert-sps-pps=true '
            f'iframeinterval={interval} idrinterval={interval} '
            'tuning-info-id=2 ! h264parse config-interval=-1 ! '
            'capssetter replace=true '
            'caps="video/x-h264,profile=constrained-baseline,'
            'level=(string)3,stream-format=byte-stream,alignment=au,'
            f'width={self.width},height={self.height},'
            f'framerate={self.fps}/1" ! '
            f'whipclientsink name=whip video-caps=video/x-h264 '
            f'min-bitrate=300000 start-bitrate={self.bitrate * 1000} '
            f'max-bitrate={self.bitrate * 1000}')

    def _start_pipeline(self, gst_format):
        self._stop_pipeline()
        try:
            pipeline = Gst.parse_launch(self._pipeline_description(gst_format))
            whip = pipeline.get_by_name('whip')
            whip.set_property('stun-server', None)
            signaller = whip.get_property('signaller')
            signaller.set_property('whip-endpoint', self.whip_url)
            if self.whip_token:
                signaller.set_property('auth-token', self.whip_token)
            signaller.connect('webrtcbin-ready', self._webrtcbin_ready)
            whip.connect('consumer-added', self._consumer_added)
            whip.connect('consumer-removed', self._consumer_removed)
            bus = pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message', self._bus_message)
            self.pipeline = pipeline
            self.appsrc = pipeline.get_by_name('source')
            self.session_id = str(uuid.uuid4())
            self.started_at = time.monotonic()
            self._set_status('CONNECTING')
            result = pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError('GStreamer pipeline failed to start')
        except (GLib.Error, RuntimeError) as exc:
            self.get_logger().error(f'WHIP pipeline start failed: {exc}')
            self._schedule_reconnect(classify_error(str(exc)))

    def _webrtcbin_ready(self, signaller, peer_id, webrtcbin):
        """Disable UPnP discovery; the direct Tailscale path does not need it."""
        del signaller, peer_id
        ice = webrtcbin.get_property('ice-agent')
        agent = ice.get_property('agent')
        agent.set_property('upnp', False)
        self._ice_refs.append((ice, agent))
        webrtcbin.connect(
            'notify::connection-state', self._peer_connection_state_changed)

    def _peer_connection_state_changed(self, webrtcbin, unused_pspec):
        del unused_pspec
        state = webrtcbin.get_property('connection-state').value_nick
        if state == 'connected':
            self._reconnect_delay = 1.0
            self.get_logger().info(f'WHIP connected: {self.whip_url}')
            self._set_status('STREAMING')
        elif state == 'failed':
            self._schedule_reconnect('ICE_FAILED')

    def _stop_pipeline(self):
        pipeline, self.pipeline, self.appsrc = self.pipeline, None, None
        if pipeline is not None:
            pipeline.set_state(Gst.State.NULL)
        self._ice_refs.clear()

    def _consumer_added(self, element, consumer_id, pipeline):
        del element, consumer_id, pipeline

    def _consumer_removed(self, element, consumer_id, pipeline):
        del element, consumer_id, pipeline
        if self.state not in ('STOPPED', 'RECONNECTING'):
            self._schedule_reconnect('ICE_FAILED')

    def _bus_message(self, bus, message):
        del bus
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            detail = f'{error}: {debug or ""}'
            self.get_logger().error(detail)
            self._schedule_reconnect(classify_error(detail))

    def _schedule_reconnect(self, error_code):
        if self.state == 'RECONNECTING' and self.error_code:
            return
        self._set_status('RECONNECTING', error_code)
        self._stop_pipeline()
        self._retry_at = (
            time.monotonic()
            + self._reconnect_delay * random.uniform(0.8, 1.2))
        self._reconnect_delay = min(10.0, self._reconnect_delay * 2.0)

    def _set_status(self, state, error_code=None):
        changed = state != self.state or error_code != self.error_code
        self.state, self.error_code = state, error_code
        if changed:
            self._publish_status()

    def _publish_status(self):
        stream = {
            'camera_id': self.camera_id,
            'path': f'autosdv/{self.vehicle_id}/{self.camera_id}',
            'session_id': self.session_id,
            'state': self.state,
            'error_code': self.error_code,
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'bitrate_kbps': self.bitrate,
        }
        payload = {
            'schema_version': SCHEMA_VERSION,
            'vehicle_id': self.vehicle_id,
            'online': True,
            'seq': self.seq,
            'ts_ms': time.time_ns() // 1_000_000,
            'streams': [stream],
        }
        if self.mqtt.publish(self.status_topic, payload, qos=1, retain=True):
            self.seq += 1

    def _image_callback(self, message):
        encoding = str(message.encoding).lower()
        format_info = ENCODINGS.get(encoding)
        if format_info is None:
            self._set_status('ERROR', 'CAMERA_UNAVAILABLE')
            return
        if (int(message.width) != self.width
                or int(message.height) != self.height):
            self._set_status('ERROR', 'CAMERA_UNAVAILABLE')
            return
        gst_format, channels = format_info
        self.last_frame_at = time.monotonic()
        if self.pipeline is None and self.last_frame_at >= self._retry_at:
            self._start_pipeline(gst_format)
        if self.appsrc is None:
            return
        row_bytes = self.width * channels
        source = bytes(message.data)
        if int(message.step) == row_bytes:
            data = source[:row_bytes * self.height]
        else:
            data = b''.join(
                source[
                    row * int(message.step):
                    row * int(message.step) + row_bytes]
                for row in range(self.height))
        buffer = Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        if self.appsrc.emit('push-buffer', buffer) == Gst.FlowReturn.OK:
            self.frame_count += 1

    def _tick(self):
        now = time.monotonic()
        if self.last_frame_at is None or now - self.last_frame_at > 3.0:
            if self.pipeline is not None:
                self._stop_pipeline()
            self._set_status('RECONNECTING', 'CAMERA_UNAVAILABLE')
        elif self.pipeline is None and now >= self._retry_at:
            # The image callback starts it with the actual encoding.
            pass
        if self.started_at and now - self.started_at >= 30.0:
            self._reconnect_delay = 1.0
        last_status_at = getattr(self, '_last_status_at', 0.0)
        if self.seq == 0 or now - last_status_at >= 5.0:
            self._last_status_at = now
            self._publish_status()

    def destroy_node(self):
        """Stop WHIP and publish the explicit streamer offline state."""
        self._set_status('STOPPED')
        self._stop_pipeline()
        self.mqtt.close(
            self.status_topic,
            {'schema_version': SCHEMA_VERSION, 'online': False})
        self._glib_loop.quit()
        self._glib_thread.join(timeout=2)
        return super().destroy_node()


def main(args=None):
    """Run the camera streamer node."""
    rclpy.init(args=args)
    node = CameraStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
