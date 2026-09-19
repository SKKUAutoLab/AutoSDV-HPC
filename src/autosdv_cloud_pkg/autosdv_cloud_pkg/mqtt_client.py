"""Small MQTT v5 publisher shared by the two vehicle processes."""

import json

import paho.mqtt.client as mqtt


class MqttPublisher:
    """Own one reconnecting MQTT connection with a retained LWT."""

    def __init__(self, host, port, client_id, username, password,
                 will_topic, will_payload, on_connected=None):
        """Start a clean MQTT v5 connection in Paho's network thread."""
        self.connected = False
        self._on_connected = on_connected
        self.client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)
        if username:
            self.client.username_pw_set(username, password)
        self.client.will_set(
            will_topic, self.encode(will_payload), qos=1, retain=True)
        self.client.on_connect = self._connected
        self.client.on_disconnect = self._disconnected
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self._host = host
        self._port = int(port)

    def start(self):
        """Start networking after the owner has stored this instance."""
        self.client.connect_async(self._host, self._port, keepalive=5)
        self.client.loop_start()

    @staticmethod
    def encode(payload):
        """Encode JSON payloads while leaving status strings unchanged."""
        if isinstance(payload, str):
            return payload
        return json.dumps(payload, separators=(',', ':'))

    def _connected(
            self, client, userdata, flags, reason_code, properties=None):
        del client, userdata, flags, properties
        self.connected = reason_code == 0
        if self.connected and self._on_connected:
            self._on_connected()

    def _disconnected(self, client, userdata, reason_code, properties=None):
        del client, userdata, reason_code, properties
        self.connected = False

    def publish(self, topic, payload, qos, retain):
        """Publish only while connected; old messages are never queued."""
        if not self.connected:
            return False
        result = self.client.publish(
            topic, self.encode(payload), qos=qos, retain=retain)
        return result.rc == mqtt.MQTT_ERR_SUCCESS

    def close(self, topic, payload):
        """Publish the normal shutdown state and stop the network loop."""
        if self.connected:
            info = self.client.publish(
                topic, self.encode(payload), qos=1, retain=True)
            info.wait_for_publish(2)
        self.client.disconnect()
        try:
            self.client.loop_stop()
        except KeyboardInterrupt:
            # A launch-wide SIGINT can arrive directly and through launch.
            pass
