import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    input_topic = LaunchConfiguration('input_topic')
    log_path = LaunchConfiguration('log_path')
    run_id = LaunchConfiguration('run_id')
    flush_every = LaunchConfiguration('flush_every')

    def logger_node(image_index, participant_id, topic, log_enabled, node_name):
        return Node(
            package='camera_perception_pkg',
            executable='image_frame_id_logger_node',
            name=node_name,
            output='screen',
            additional_env=fastdds_profile_env(
                launch_pkg_share,
                image_index,
                participant_id),
            parameters=[{
                'input_topic': topic,
                'log_path': log_path,
                'run_id': run_id,
                'flush_every': ParameterValue(flush_every, value_type=int),
                'log_enabled': log_enabled,
            }],
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='image_01_raw'),
        DeclareLaunchArgument(
            'log_path',
            default_value=os.environ.get(
                'AUTOSDV_P2_LOG_PATH',
                '/home/autolab/update/P2_log/hpc_image_receive_p2.csv')),
        DeclareLaunchArgument(
            'run_id',
            default_value='run_01'),
        DeclareLaunchArgument(
            'flush_every',
            default_value='1'),
        logger_node(
            1,
            20,
            input_topic,
            True,
            'image_frame_id_logger_node_1'),
        logger_node(
            2,
            21,
            'image_02_raw',
            False,
            'image_frame_id_logger_node_2'),
        logger_node(
            3,
            22,
            'image_03_raw',
            False,
            'image_frame_id_logger_node_3'),
        logger_node(
            4,
            23,
            'image_04_raw',
            False,
            'image_frame_id_logger_node_4'),
        logger_node(
            5,
            24,
            'image_05_raw',
            False,
            'image_frame_id_logger_node_5'),
    ])
