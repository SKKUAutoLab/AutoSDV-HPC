import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from .p2_image_receive_logger import P2ImageReceiveCsvLogger


class H264Decoder:
    def __init__(self, timeout_ms=20):
        import gi

        gi.require_version('Gst', '1.0')
        from gi.repository import Gst

        self.Gst = Gst
        self.timeout_ns = int(timeout_ms * 1_000_000)
        Gst.init(None)

        decoder = os.environ.get('AUTOSDV_H264_GST_DECODER', 'avdec_h264')
        pipeline_desc = (
            'appsrc name=src is-live=true format=time do-timestamp=true block=false '
            'caps=video/x-h264,stream-format=byte-stream,alignment=au '
            '! h264parse '
            f'! {decoder} '
            '! videoconvert '
            '! video/x-raw,format=BGR '
            '! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true'
        )
        self.pipeline = Gst.parse_launch(pipeline_desc)
        self.appsrc = self.pipeline.get_by_name('src')
        self.appsink = self.pipeline.get_by_name('sink')
        self.pipeline.set_state(Gst.State.PLAYING)

    def push_packet(self, packet):
        Gst = self.Gst
        buffer = Gst.Buffer.new_allocate(None, len(packet), None)
        buffer.fill(0, packet)
        result = self.appsrc.emit('push-buffer', buffer)
        if result != Gst.FlowReturn.OK:
            raise RuntimeError(f'GStreamer push-buffer failed: {result.value_name}')

        latest_frame = None
        while True:
            sample = self.appsink.emit('try-pull-sample', self.timeout_ns)
            if sample is None:
                return latest_frame

            caps = sample.get_caps().get_structure(0)
            width = caps.get_value('width')
            height = caps.get_value('height')
            frame_buffer = sample.get_buffer()
            frame_bytes = frame_buffer.extract_dup(0, frame_buffer.get_size())
            latest_frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape(
                (height, width, 3)).copy()

    def close(self):
        self.pipeline.set_state(self.Gst.State.NULL)


class DDSImageListener(Node):
    def __init__(self):
        super().__init__('dds_image_publisher')
        self.bridge = CvBridge()

        # 'image_number'라는 이름의 파라미터를 선언하고 기본값을 'image_topic'으로 설정합니다.
        self.declare_parameter('image', 'image_01')
        self.declare_parameter('show_image', False)
        self.declare_parameter('p2_log_enabled', False)
        self.declare_parameter('p2_log_path', '')
        self.declare_parameter('p2_log_run_id', '')
        self.declare_parameter('p2_log_start_ns', 0)
        self.declare_parameter('h264_decode_timeout_ms', 20)
        self.declare_parameter(
            'encoded_qos_depth',
            int(os.environ.get('AUTOSDV_ENCODED_QOS_DEPTH', '1')))
        # 파라미터 값을 가져옵니다.
        topic_name = self.get_parameter('image').get_parameter_value().string_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        self.p2_log_enabled = (
            self.get_parameter('p2_log_enabled').get_parameter_value().bool_value
        )
        self.p2_log_path = self.get_parameter('p2_log_path').get_parameter_value().string_value
        self.p2_log_run_id = (
            self.get_parameter('p2_log_run_id').get_parameter_value().string_value
        )
        self.p2_log_start_ns = (
            self.get_parameter('p2_log_start_ns').get_parameter_value().integer_value
        )
        self.h264_decode_timeout_ms = (
            self.get_parameter('h264_decode_timeout_ms').get_parameter_value().integer_value
        )
        self.encoded_qos_depth = max(
            1,
            self.get_parameter('encoded_qos_depth').get_parameter_value().integer_value)
        self.image_window_name = f'DDS Image Viewer ({topic_name})'
        self.image_window_created = False
        self.h264_decoder = None

        output_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        input_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=self.encoded_qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 가져온 파라미터 값(topic_name)을 사용하여 퍼블리셔를 생성합니다.
        self.publisher_ = self.create_publisher(Image, topic_name, output_qos_profile)
        self.get_logger().info(f'Publishing to topic: {topic_name}')

        self.input_topic_name = topic_name + '_raw'
        self.p2_logger = None
        self.p2_logger_lock = threading.Lock()
        self.p2_log_run_index = 0
        if self.p2_log_enabled:
            self._set_p2_logging(
                True,
                self.p2_log_path,
                self.p2_log_run_id,
                self.p2_log_start_ns)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        # DDS와 유사한 토픽(DDS 입력을 시뮬레이션하는 ROS 2 토픽)을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            self.input_topic_name,
            self.listener_callback,
            input_qos_profile
        )
        self.get_logger().info('DDS Image Listener Node has started.')

    def _resolve_p2_log_path(self, path):
        if path:
            return path
        return os.environ.get('AUTOSDV_P2_LOG_PATH', '/tmp/hpc_image_receive_p2.csv')

    def _set_p2_logging(self, enabled, path, run_id, run_start_ns):
        resolved_path = self._resolve_p2_log_path(path)
        old_logger = None

        with self.p2_logger_lock:
            if enabled:
                self.p2_log_run_index += 1
                resolved_run_id = run_id or f'run_{self.p2_log_run_index:02d}'
                new_logger = P2ImageReceiveCsvLogger(
                    resolved_path,
                    self.input_topic_name,
                    resolved_run_id,
                    run_start_ns)
                old_logger = self.p2_logger
                self.p2_logger = new_logger
                self.p2_log_enabled = True
                self.p2_log_path = path
                self.p2_log_run_id = run_id
                self.p2_log_start_ns = run_start_ns
            else:
                old_logger = self.p2_logger
                self.p2_logger = None
                self.p2_log_enabled = False
                self.p2_log_path = path
                self.p2_log_run_id = run_id
                self.p2_log_start_ns = run_start_ns

        if old_logger is not None:
            old_logger.close()

        if enabled:
            self.get_logger().info(
                f'P2 image receive logging enabled: {resolved_path} ({resolved_run_id})')
        else:
            self.get_logger().info('P2 image receive logging disabled.')

    def _on_parameter_update(self, params):
        p2_log_enabled = self.p2_log_enabled
        p2_log_path = self.p2_log_path
        p2_log_run_id = self.p2_log_run_id
        p2_log_start_ns = self.p2_log_start_ns
        show_image = self.show_image

        for param in params:
            if param.name == 'show_image':
                if param.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='show_image must be a bool')
                show_image = param.value
            elif param.name == 'p2_log_enabled':
                if param.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason='p2_log_enabled must be a bool')
                p2_log_enabled = param.value
            elif param.name == 'p2_log_path':
                if param.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason='p2_log_path must be a string')
                p2_log_path = param.value
            elif param.name == 'p2_log_run_id':
                if param.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason='p2_log_run_id must be a string')
                p2_log_run_id = param.value
            elif param.name == 'p2_log_start_ns':
                if param.type_ != Parameter.Type.INTEGER:
                    return SetParametersResult(
                        successful=False,
                        reason='p2_log_start_ns must be an integer')
                p2_log_start_ns = param.value
            elif param.name == 'h264_decode_timeout_ms':
                if param.type_ != Parameter.Type.INTEGER:
                    return SetParametersResult(
                        successful=False,
                        reason='h264_decode_timeout_ms must be an integer')
                self.h264_decode_timeout_ms = param.value

        try:
            if show_image != self.show_image:
                self._set_show_image(show_image)
            if (p2_log_enabled != self.p2_log_enabled or
                    p2_log_path != self.p2_log_path or
                    p2_log_run_id != self.p2_log_run_id or
                    p2_log_start_ns != self.p2_log_start_ns):
                self._set_p2_logging(
                    p2_log_enabled,
                    p2_log_path,
                    p2_log_run_id,
                    p2_log_start_ns)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        return SetParametersResult(successful=True)

    def _set_show_image(self, enabled):
        self.show_image = enabled
        if enabled:
            self.get_logger().info(f'Image display enabled: {self.image_window_name}')
        else:
            if self.image_window_created:
                cv2.destroyWindow(self.image_window_name)
                self.image_window_created = False
            self.get_logger().info('Image display disabled.')

    def _write_p2_log(self, msg):
        with self.p2_logger_lock:
            if self.p2_logger is None:
                return
            try:
                self.p2_logger.write(msg)
            except Exception as exc:
                self.get_logger().error(f'P2 image receive logging failed: {exc}')
                failed_logger = self.p2_logger
                self.p2_logger = None
                self.p2_log_enabled = False
                try:
                    failed_logger.close()
                except Exception as close_exc:
                    self.get_logger().error(f'P2 image receive logger close failed: {close_exc}')

    def _is_h264(self, encoding):
        normalized = encoding.strip().lower().replace('-', '_')
        return normalized in ('h264', 'h264_frame')

    def _decode_h264(self, msg):
        if self.h264_decoder is None:
            self.h264_decoder = H264Decoder(self.h264_decode_timeout_ms)
            self.get_logger().info('H.264 decoder initialized.')
        access_unit = bytes(msg.data)
        return self.h264_decoder.push_packet(access_unit)

    def listener_callback(self, msg):
        try:
            self._write_p2_log(msg)

            if self._is_h264(msg.encoding):
                cv_image = self._decode_h264(msg)
                if cv_image is None:
                    return
            else:
                # ROS 2 이미지 메시지에서 직접 JPEG 데이터를 디코딩합니다.
                jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error(f'Failed to decode image encoding={msg.encoding}.')
                return

            if self.show_image:
                if not self.image_window_created:
                    cv2.namedWindow(self.image_window_name, cv2.WINDOW_NORMAL)
                    self.image_window_created = True
                cv2.imshow(self.image_window_name, cv_image)
                cv2.waitKey(1)

            # 선택적으로 디코딩된 이미지를 다시 발행합니다.
            decoded_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            # E2E latency 측정용 frame_id pass-through: ZCU camera_server가 박은 header.frame_id 보존.
            decoded_msg.header = msg.header
            self.publisher_.publish(decoded_msg)
            # self.get_logger().info(f'Image received and republished: {cv_image.shape[1]}x{cv_image.shape[0]}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def destroy_node(self):
        with self.p2_logger_lock:
            if self.p2_logger is not None:
                self.p2_logger.close()
                self.p2_logger = None
        if self.image_window_created:
            cv2.destroyWindow(self.image_window_name)
            self.image_window_created = False
        if self.h264_decoder is not None:
            self.h264_decoder.close()
            self.h264_decoder = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DDSImageListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DDS Image Listener Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
