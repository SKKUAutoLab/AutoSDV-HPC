import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class DDSImageListener(Node):
    def __init__(self):
        super().__init__('dds_image_publisher')
        self.bridge = CvBridge()

        # 'image_number'라는 이름의 파라미터를 선언하고 기본값을 'image_topic'으로 설정합니다.
        self.declare_parameter('image', 'image_01')
        # 파라미터 값을 가져옵니다.
        topic_name = self.get_parameter('image').get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 가져온 파라미터 값(topic_name)을 사용하여 퍼블리셔를 생성합니다.
        self.publisher_ = self.create_publisher(Image, topic_name, qos_profile)
        self.get_logger().info(f'Publishing to topic: {topic_name}')

        self._sub_topic = topic_name + '_raw'

        # DDS와 유사한 토픽(DDS 입력을 시뮬레이션하는 ROS 2 토픽)을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            self._sub_topic,
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info('DDS Image Listener Node has started.')

        # AUTOSDV_TRACE_SESSION 환경변수가 설정된 경우만 CSV 로깅 활성화 (평시 운영 영향 0)
        self._trace_csv = None
        trace_session = os.environ.get('AUTOSDV_TRACE_SESSION')
        if trace_session:
            trace_base = os.environ.get('AUTOSDV_TRACE_BASE', '/tmp/autosdv_traces')
            csv_dir = Path(trace_base) / trace_session / 'csv'
            csv_dir.mkdir(parents=True, exist_ok=True)
            csv_path = csv_dir / f'entry_{topic_name}_{os.getpid()}.csv'
            self._trace_csv = open(csv_path, 'w', buffering=1)
            self._trace_csv.write('frame_id,topic,event_type,steady_clock_ns,wall_clock_ns\n')
            self.get_logger().info(f'Trace CSV logging enabled: {csv_path}')

        # 측정 시 cv2.imshow off (X server 렌더링 부하 제거) — AUTOSDV_NO_GUI=1 또는 trace 모드 자동
        self._show_window = (
            os.environ.get('AUTOSDV_NO_GUI', '0') != '1'
            and not trace_session
        )
        if not self._show_window:
            self.get_logger().info('GUI window disabled (measurement mode)')

    def listener_callback(self, msg):
        # ZCU image_NN_raw take 직후 — HPC 진입 시각 기록 (T_HPC_in)
        if self._trace_csv is not None:
            self._trace_csv.write(
                f'{msg.header.frame_id},{self._sub_topic},in,'
                f'{time.monotonic_ns()},{time.time_ns()}\n'
            )

        try:
            # ROS 2 이미지 메시지에서 직접 JPEG 데이터를 디코딩합니다.
            jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error('Failed to decode JPEG image.')
                return

            # 이미지를 처리합니다 (예: 화면에 표시하거나 수정).
            if self._show_window:
                cv2.imshow("DDS Image Viewer", cv_image)
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
        if self._trace_csv is not None:
            try:
                self._trace_csv.close()
            except Exception:
                pass
        cv2.destroyAllWindows()
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
