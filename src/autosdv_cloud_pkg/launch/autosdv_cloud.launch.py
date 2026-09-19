"""Start the AutoSDV MQTT gateway and Camera 01 WHIP streamer."""

from launch import LaunchDescription  # noqa: I201
from launch_ros.actions import Node  # noqa: I201


def generate_launch_description():
    """Return the two independent vehicle-to-server processes."""
    return LaunchDescription([
        Node(
            package='autosdv_cloud_pkg',
            executable='cloud_gateway_node',
            name='autosdv_cloud_gateway',
            output='screen',
        ),
        Node(
            package='autosdv_cloud_pkg',
            executable='camera_streamer_node',
            name='autosdv_camera_streamer',
            output='screen',
        ),
    ])
