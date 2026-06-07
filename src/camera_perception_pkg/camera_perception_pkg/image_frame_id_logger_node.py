import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

from .p2_image_receive_logger import P2ImageReceiveCsvLogger


class ImageFrameIdLogger(Node):
    def __init__(self):
        super().__init__('image_frame_id_logger')

        self.declare_parameter('input_topic', 'image_01_raw')
        self.declare_parameter('log_path', '')
        self.declare_parameter('log_dir', '')
        self.declare_parameter('run_id', 'run_01')
        self.declare_parameter('flush_every', 1)
        self.declare_parameter('log_enabled', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.log_path = self._resolve_log_path(
            self.get_parameter('log_path').value,
            self.get_parameter('log_dir').value,
            self.input_topic)
        self.run_id = self.get_parameter('run_id').value
        self.flush_every = max(1, int(self.get_parameter('flush_every').value))
        self.log_enabled = self.get_parameter('log_enabled').value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.logger = None
        if self.log_enabled:
            self.logger = P2ImageReceiveCsvLogger(
                self.log_path,
                self.input_topic,
                self.run_id,
            )
        self.received_count = 0

        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.listener_callback,
            qos_profile,
        )

        if self.log_enabled:
            self.get_logger().info(
                f'Frame ID logging started: topic={self.input_topic}, '
                f'path={self.log_path}, run_id={self.run_id}, '
                f'flush_every={self.flush_every}')
        else:
            self.get_logger().info(
                f'Frame ID subscribe-only started: topic={self.input_topic}')

    def _resolve_log_path(self, path, log_dir, topic):
        if path:
            return os.path.abspath(os.path.expanduser(path))

        if not log_dir:
            log_dir = os.environ.get('AUTOSDV_P2_LOG_DIR', '')
        if not log_dir:
            env_path = os.environ.get('AUTOSDV_P2_LOG_PATH', '')
            if env_path:
                log_dir = os.path.dirname(os.path.abspath(os.path.expanduser(env_path)))
        if not log_dir:
            log_dir = '/tmp'

        topic_name = re.sub(r'[^A-Za-z0-9_.-]+', '_', topic.strip('/'))
        if not topic_name:
            topic_name = 'image_frame_id'
        return os.path.join(
            os.path.abspath(os.path.expanduser(log_dir)),
            f'{topic_name}.csv')

    def listener_callback(self, msg):
        self.received_count += 1
        if self.logger is None:
            return
        self.logger.write(msg)
        if self.received_count % self.flush_every == 0:
            self.logger.flush()

    def destroy_node(self):
        if self.logger is not None:
            self.logger.close()
            self.logger = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageFrameIdLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down image frame ID logger node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
