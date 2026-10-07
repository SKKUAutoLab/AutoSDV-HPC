# Copyright 2026 SKKU AutoSDV.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Launch 4 bev_node instances + svm_compositor_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


_BEV_CONFIGS = (
    ('front', 'image_02'),
    ('left',  'image_03'),
    ('right', 'image_04'),
    ('back',  'image_05'),
)


def generate_launch_description():
    pkg_share = get_package_share_directory('bev_perception_pkg')
    config_dir = os.path.join(pkg_share, 'config')

    bev_nodes = [
        Node(
            package='bev_perception_pkg',
            executable='bev_node',
            name='bev_node_{}'.format(pos),
            output='screen',
            parameters=[{
                'position': pos,
                'input_topic': topic,
                'output_topic': 'image_{}_bev'.format(pos),
                'calibration_file': os.path.join(
                    config_dir, 'bev_{}.yaml'.format(pos)),
                'input_qos': 'BEST_EFFORT',
                'output_qos': 'BEST_EFFORT',
            }],
        )
        for pos, topic in _BEV_CONFIGS
    ]

    svm_node = Node(
        package='bev_perception_pkg',
        executable='svm_compositor_node',
        name='svm_compositor_node',
        output='screen',
        parameters=[{
            'front_topic': 'image_front_bev',
            'left_topic': 'image_left_bev',
            'right_topic': 'image_right_bev',
            'back_topic': 'image_back_bev',
            'output_topic': 'bev_surround',
            'layout_file': os.path.join(config_dir, 'svm_layout.yaml'),
            'sync_slop': 0.1,
            'sync_queue_size': 10,
            'input_qos': 'BEST_EFFORT',
            'output_qos': 'RELIABLE',
            'watchdog_period_sec': 1.0,
            'watchdog_timeout_sec': 5.0,
        }],
    )

    return LaunchDescription(bev_nodes + [svm_node])
