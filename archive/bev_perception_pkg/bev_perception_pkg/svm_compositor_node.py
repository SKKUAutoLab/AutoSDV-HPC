# Copyright 2026 SKKU AutoSDV.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Surround-View (SVM) compositor — fuses 4 BEV channels into one canvas."""

import time

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from bev_perception_pkg.lib.bev_perception_func_lib import (
    composite_canvas,
    load_svm_layout,
    make_image_qos,
)


_POSITIONS = ('front', 'left', 'right', 'back')


class SVMCompositorNode(Node):
    """Time-sync 4 BEV topics and emit a single surround-view image."""

    def __init__(self):
        super().__init__('svm_compositor_node')

        self.declare_parameter('front_topic', 'image_front_bev')
        self.declare_parameter('left_topic', 'image_left_bev')
        self.declare_parameter('right_topic', 'image_right_bev')
        self.declare_parameter('back_topic', 'image_back_bev')
        self.declare_parameter('output_topic', 'bev_surround')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('input_qos', 'BEST_EFFORT')
        self.declare_parameter('output_qos', 'RELIABLE')
        self.declare_parameter('watchdog_period_sec', 1.0)
        self.declare_parameter('watchdog_timeout_sec', 5.0)

        layout_file = self._param_str('layout_file')
        if not layout_file:
            raise RuntimeError(
                'svm_compositor_node: layout_file parameter is required')

        self._layout = load_svm_layout(layout_file)
        canvas_w, canvas_h = self._layout['output_size']
        self._canvas_template = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        self._bridge = CvBridge()

        sync_queue = int(self.get_parameter(
            'sync_queue_size').get_parameter_value().integer_value)
        sync_slop = float(self.get_parameter(
            'sync_slop').get_parameter_value().double_value)
        input_qos = make_image_qos(self._param_str('input_qos'))
        output_qos = make_image_qos(self._param_str('output_qos'))

        topics = {
            'front': self._param_str('front_topic'),
            'left': self._param_str('left_topic'),
            'right': self._param_str('right_topic'),
            'back': self._param_str('back_topic'),
        }
        self._subs = [
            message_filters.Subscriber(
                self, Image, topics[pos], qos_profile=input_qos)
            for pos in _POSITIONS
        ]

        self._sync = message_filters.ApproximateTimeSynchronizer(
            self._subs, queue_size=sync_queue, slop=sync_slop)
        self._sync.registerCallback(self._synced_callback)

        self._pub = self.create_publisher(
            Image, self._param_str('output_topic'), output_qos)

        self._last_sync_ts = time.monotonic()
        self._watchdog_timeout = float(self.get_parameter(
            'watchdog_timeout_sec').get_parameter_value().double_value)
        watchdog_period = float(self.get_parameter(
            'watchdog_period_sec').get_parameter_value().double_value)
        self._watchdog_warned = False
        self._watchdog_timer = self.create_timer(
            watchdog_period, self._watchdog_tick)

        self.get_logger().info(
            'svm_compositor_node up: {topics} -> {out} '
            '(canvas={cw}x{ch}, slop={slop}s, queue={q})'
            .format(
                topics=topics, out=self._param_str('output_topic'),
                cw=canvas_w, ch=canvas_h,
                slop=sync_slop, q=sync_queue))

    def _param_str(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _synced_callback(self, front_msg, left_msg, right_msg, back_msg):
        msgs = {
            'front': front_msg,
            'left': left_msg,
            'right': right_msg,
            'back': back_msg,
        }

        canvas = self._canvas_template.copy()
        rois = self._layout['rois']
        for pos in self._layout['overlay_order']:
            if pos not in rois or pos not in msgs:
                continue
            try:
                frame = self._bridge.imgmsg_to_cv2(
                    msgs[pos], desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    'svm decode failed for {}: {}'.format(pos, exc))
                continue
            composite_canvas(canvas, frame, rois[pos])

        out_msg = self._bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        out_msg.header = front_msg.header
        self._pub.publish(out_msg)

        self._last_sync_ts = time.monotonic()
        if self._watchdog_warned:
            self.get_logger().info('svm sync recovered.')
            self._watchdog_warned = False

    def _watchdog_tick(self):
        elapsed = time.monotonic() - self._last_sync_ts
        if elapsed > self._watchdog_timeout and not self._watchdog_warned:
            self.get_logger().warn(
                'svm sync stalled: no synchronized 4-camera frame '
                'in {:.1f}s'.format(elapsed))
            self._watchdog_warned = True


def main(args=None):
    rclpy.init(args=args)
    node = SVMCompositorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('svm_compositor_node interrupted, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
