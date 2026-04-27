# Copyright 2026 SKKU AutoSDV.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Per-camera BEV transformation node (one instance per fisheye camera)."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from bev_perception_pkg.lib.bev_perception_func_lib import (
    get_combined_map,
    load_bev_calibration,
    make_image_qos,
)


class BEVNode(Node):
    """Subscribe to a decoded fisheye image and publish its BEV view."""

    def __init__(self):
        super().__init__('bev_node')

        self.declare_parameter('position', 'front')
        self.declare_parameter('input_topic', 'image_02')
        self.declare_parameter('output_topic', 'image_front_bev')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('input_qos', 'BEST_EFFORT')
        self.declare_parameter('output_qos', 'BEST_EFFORT')

        self._position = self._param_str('position')
        input_topic = self._param_str('input_topic')
        output_topic = self._param_str('output_topic')
        calibration_file = self._param_str('calibration_file')
        input_qos = self._param_str('input_qos')
        output_qos = self._param_str('output_qos')

        if not calibration_file:
            raise RuntimeError(
                'bev_node[{}]: calibration_file parameter is required'
                .format(self._position))

        self._bridge = CvBridge()
        self._calib = load_bev_calibration(calibration_file)

        if self._calib['position'] != self._position:
            self.get_logger().warn(
                "Position mismatch: parameter '{}' vs YAML '{}'"
                .format(self._position, self._calib['position']))

        self._map1, self._map2 = get_combined_map(
            self._calib['camera_matrix'],
            self._calib['dist_coeffs'],
            self._calib['src_points'],
            self._calib['dst_points'],
            self._calib['bev_size'],
            self._calib['resolution'],
        )

        self._sub = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            make_image_qos(input_qos),
        )
        self._pub = self.create_publisher(
            Image,
            output_topic,
            make_image_qos(output_qos),
        )

        self.get_logger().info(
            "bev_node[{pos}] up: {sub_t} -> {pub_t} "
            "(in={qin}, out={qout}, bev_size={bw}x{bh})"
            .format(
                pos=self._position,
                sub_t=input_topic, pub_t=output_topic,
                qin=input_qos, qout=output_qos,
                bw=self._calib['bev_size'][0],
                bh=self._calib['bev_size'][1]))

    def _param_str(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _image_callback(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 — log and drop bad frame
            self.get_logger().error(
                'bev_node[{}] cv_bridge decode failed: {}'
                .format(self._position, exc))
            return

        try:
            bev = cv2.remap(
                frame, self._map1, self._map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                'bev_node[{}] remap failed: {}'
                .format(self._position, exc))
            return

        out_msg = self._bridge.cv2_to_imgmsg(bev, encoding='bgr8')
        out_msg.header = msg.header
        self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BEVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('bev_node interrupted, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
