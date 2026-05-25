import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def fastdds_profile_env(share_dir, image_index, participant_id):
    profile = os.path.join(
        share_dir,
        'config',
        f'fastdds_image_{image_index:02d}_participant{participant_id}.xml')
    return {
        'FASTRTPS_DEFAULT_PROFILES_FILE': profile,
        'FASTDDS_DEFAULT_PROFILES_FILE': profile,
    }


def generate_launch_description():
    launch_pkg_share = get_package_share_directory('launch_pkg')
    bev_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bev_perception_pkg'),
                'launch', 'bev.launch.py')))

    return LaunchDescription([
        Node(
            package='camera_perception_pkg',
            executable='ethernet_image_publisher_node',
            name='ethernet_image_publisher_node_1',  #카메라 추가되면 노드 이름 중복 방지를 위해 변경
            output='screen',
            additional_env=fastdds_profile_env(launch_pkg_share, 1, 20),
            parameters=[{
                'image': 'image_01',
                'show_image': True,
                'p2_log_enabled': False,
                'p2_log_path': os.environ.get(
                    'AUTOSDV_P2_LOG_PATH',
                    '/tmp/hpc_image_receive_p2.csv'),
            }]  #카메라 추가되면 변경해야 함
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
            additional_env=fastdds_profile_env(launch_pkg_share, 2, 21),
            parameters=[{'image': 'image_02'}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='ethernet_image_publisher_node',
            name='ethernet_image_publisher_node_3',
            output='screen',
            additional_env=fastdds_profile_env(launch_pkg_share, 3, 22),
            parameters=[{'image': 'image_03'}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='ethernet_image_publisher_node',
            name='ethernet_image_publisher_node_4',
            output='screen',
            additional_env=fastdds_profile_env(launch_pkg_share, 4, 23),
            parameters=[{'image': 'image_04'}]
        ),
        Node(
            package='camera_perception_pkg',
            executable='ethernet_image_publisher_node',
            name='ethernet_image_publisher_node_5',
            output='screen',
            additional_env=fastdds_profile_env(launch_pkg_share, 5, 24),
            parameters=[{'image':'image_05'}]
        ),

        # BEV/SVM perception (image_02..05 -> bev_surround)
        bev_launch,

        #Node(
        #    package='camera_perception_pkg',
        #    executable='yolov8_node',
        #    name='yolov8_node',
        #    output='screen'
        #),
        #Node(
        #    package='camera_perception_pkg',
        #    executable='lane_info_extractor_node',
        #    name='lane_info_extractor_node',
        #    output='screen'
        #),
        #Node(
        #    package='decision_making_pkg',
        #    executable='path_planner_node',
        #    name='path_planner_node',
        #    output='screen'
        #),
        ## Node(
        ##    package='serial_communication_pkg',
        ##    executable='serial_sender_node',
        ##    name='serial_sender_node',
        ##    output='screen'
        ## ),
        #Node(
        #    package='decision_making_pkg',
        #    executable='motion_planner_node',
        #    name='motion_planner_node',
        #    output='screen'
        #),
    ])
