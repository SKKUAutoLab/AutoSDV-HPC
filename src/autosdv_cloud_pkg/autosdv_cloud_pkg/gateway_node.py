"""ROS diagnostics to MQTT gateway."""

import os
import time

import rclpy  # noqa: I100,I201
from diagnostic_msgs.msg import DiagnosticArray  # noqa: I100,I201
from rclpy.node import Node  # noqa: I201

from .contract import VEHICLE_ID, diagnostics_payload
from .mqtt_client import MqttPublisher


class CloudGatewayNode(Node):
    """Publish the AutoSDV dashboard diagnostic summary over MQTT."""

    def __init__(self):
        """Configure the ROS subscription and MQTT connection."""
        super().__init__('autosdv_cloud_gateway')
        self.vehicle_id = str(self.declare_parameter(
            'vehicle_id', VEHICLE_ID).value)
        host = str(self.declare_parameter(
            'mqtt_host', '100.74.147.75').value)
        port = int(self.declare_parameter('mqtt_port', 1883).value)
        username = os.environ.get('AUTOSDV_MQTT_USER', 'vehicle')
        password = os.environ.get('AUTOSDV_MQTT_PASSWORD', '')
        self.topic_root = f'autosdv/{self.vehicle_id}'
        self.status_topic = f'{self.topic_root}/status'
        self.diagnostics_topic = f'{self.topic_root}/diagnostics'
        self.seq = 0
        self.latest_diagnostics = None
        self.last_diagnostics_at = None
        self.mqtt = MqttPublisher(
            host, port, f'autosdv-{self.vehicle_id}-gateway',
            username, password, self.status_topic, 'offline',
            on_connected=self._publish_online)
        self.mqtt.start()
        self.create_subscription(
            DiagnosticArray, '/ivn/diagnostics/status',
            self._diagnostics_callback, 10)
        self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f'Cloud diagnostics: {self.diagnostics_topic} via {host}:{port}')

    def _publish_online(self):
        self.mqtt.publish(self.status_topic, 'online', qos=1, retain=True)

    def _diagnostics_callback(self, message):
        self.latest_diagnostics = message
        self.last_diagnostics_at = time.monotonic()

    def _publish_diagnostics(self):
        """Publish at 1 Hz, using an all-STALE payload without fresh input."""
        now = time.monotonic()
        message = self.latest_diagnostics
        if (message is None or self.last_diagnostics_at is None
                or now - self.last_diagnostics_at > 3.0):
            message = DiagnosticArray()
        payload = diagnostics_payload(message, self.seq, self.vehicle_id)
        if self.mqtt.publish(
                self.diagnostics_topic, payload, qos=0, retain=False):
            self.seq += 1

    def destroy_node(self):
        """Publish the explicit offline state before disconnecting."""
        self.mqtt.close(self.status_topic, 'offline')
        return super().destroy_node()


def main(args=None):
    """Run the cloud gateway node."""
    rclpy.init(args=args)
    node = CloudGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
