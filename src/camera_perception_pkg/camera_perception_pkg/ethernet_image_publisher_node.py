import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from .p2_image_receive_logger import P2ImageReceiveCsvLogger


class DDSImageListener(Node):
    def __init__(self):
        super().__init__('dds_image_publisher')
        self.bridge = CvBridge()

        # 'image_number'라는 이름의 파라미터를 선언하고 기본값을 'image_topic'으로 설정합니다.
        self.declare_parameter('image', 'image_01')
        self.declare_parameter('show_image', False)
        self.declare_parameter('p2_log_enabled', False)
        self.declare_parameter('p2_log_path', '')
        # 파라미터 값을 가져옵니다.
        topic_name = self.get_parameter('image').get_parameter_value().string_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        self.p2_log_enabled = (
            self.get_parameter('p2_log_enabled').get_parameter_value().bool_value
        )
        p2_log_path = self.get_parameter('p2_log_path').get_parameter_value().string_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 가져온 파라미터 값(topic_name)을 사용하여 퍼블리셔를 생성합니다.
        self.publisher_ = self.create_publisher(Image, topic_name, qos_profile)
        self.get_logger().info(f'Publishing to topic: {topic_name}')

        self.input_topic_name = topic_name + '_raw'
        self.p2_logger = None
        if self.p2_log_enabled:
            if not p2_log_path:
                p2_log_path = os.environ.get(
                    'AUTOSDV_P2_LOG_PATH',
                    '/tmp/hpc_image_receive_p2.csv')
            self.p2_logger = P2ImageReceiveCsvLogger(
                p2_log_path,
                self.input_topic_name)

        # DDS와 유사한 토픽(DDS 입력을 시뮬레이션하는 ROS 2 토픽)을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            self.input_topic_name,
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info('DDS Image Listener Node has started.')

    def listener_callback(self, msg):
        try:
            if self.p2_logger is not None:
                self.p2_logger.write(msg)

            # ROS 2 이미지 메시지에서 직접 JPEG 데이터를 디코딩합니다.
            jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error('Failed to decode JPEG image.')
                return

            if self.show_image:
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
        if self.p2_logger is not None:
            self.p2_logger.close()
            self.p2_logger = None
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
