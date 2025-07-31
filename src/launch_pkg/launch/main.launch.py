from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_perception_pkg',
            executable='ethernet_image_publisher_node',
            name='ethernet_image_publisher_node_1',  #카메라 추가되면 노드 이름 중복 방지를 위해 변경
            output='screen',
            parameters=[{'image_num': 'image_01'}]  #카메라 추가되면 변경해야 함
        ),
        # 아래 예시대로 사용해야 함.
        # ex)
        # Node(
        #     package='camera_perception_pkg',
        #     executable='image_publisher_node',
        #     name='ethernet_image_publisher_node_2',  
        #     output='screen',
        #     parameters=[{'image_num': 'image_02'}]  
        # ),
        Node(
            package='camera_perception_pkg',
            executable='ethernet_image_publisher_node',
            name='ethernet_image_publisher_node_2',
            output='screen',
            parameters=[{'image_num': 'image_02'}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='ethernet_image_publisher_node',
            name='ethernet_image_publisher_node_3',
            output='screen',
            parameters=[{'image_num': 'image_03'}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen'
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='lane_info_extractor_node',
            output='screen'
        ),
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        # Node(
        #    package='serial_communication_pkg',
        #    executable='serial_sender_node',
        #    name='serial_sender_node',
        #    output='screen'
        # ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen'
        ),
    ])