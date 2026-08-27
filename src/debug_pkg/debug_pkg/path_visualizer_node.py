"""Visualize a planned spline path over the lane ROI image."""

import cv2
from cv_bridge import CvBridge
from interfaces_pkg.msg import PathPlanningResult
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image


class PathVisualizerNode(Node):
    """Draw path points, publish the result, and optionally show a window."""

    def __init__(self):
        super().__init__('path_visualizer_node')

        roi_topic = self.declare_parameter(
            'sub_roi_image_topic', 'roi_image').value
        path_topic = self.declare_parameter(
            'sub_spline_path_topic', 'path_planning_result').value
        output_topic = self.declare_parameter(
            'pub_topic', 'path_visualized_img').value
        self.show_image = self.declare_parameter('show_image', True).value
        self.window_name = 'Path Planner'

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.bridge = CvBridge()
        self.roi_image = None
        self.spline_path = None
        self.create_subscription(
            Image, roi_topic, self._roi_image_callback, qos)
        self.create_subscription(
            PathPlanningResult, path_topic, self._path_callback, qos)
        self.publisher = self.create_publisher(Image, output_topic, qos)
        self.add_on_set_parameters_callback(self._parameter_callback)

    def _parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'show_image':
                self.show_image = bool(parameter.value)
                if not self.show_image:
                    try:
                        cv2.destroyWindow(self.window_name)
                    except cv2.error:
                        pass
        return SetParametersResult(successful=True)

    def _roi_image_callback(self, msg):
        try:
            self.roi_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert ROI image: {exc}')

    def _path_callback(self, msg):
        self.spline_path = list(zip(msg.x_points, msg.y_points))
        if self.roi_image is not None:
            self._visualize_path()

    def _visualize_path(self):
        visualized_image = self.roi_image.copy()
        for x, y in self.spline_path:
            cv2.circle(
                visualized_image, (int(x), int(y)), 5, (0, 0, 255), -1)

        if self.show_image:
            cv2.imshow(self.window_name, visualized_image)
            cv2.waitKey(1)

        try:
            output_msg = self.bridge.cv2_to_imgmsg(
                visualized_image, encoding='bgr8')
            self.publisher.publish(output_msg)
        except Exception as exc:
            self.get_logger().error(
                f'Failed to publish path visualization: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
