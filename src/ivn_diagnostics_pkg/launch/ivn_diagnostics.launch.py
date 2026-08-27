from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ivn_diagnostics_pkg',
            executable='IVN_diagnostics_node',
            name='IVN_diagnostics_node',
            output='screen',
        ),
    ])
