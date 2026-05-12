"""HPC ROS2 Tracing 측정용 launch.

기존 main.launch.py는 평소 운영용(BEV + image_publisher만)으로 유지하고,
본 launch는 측정 전용으로 제어 path 5단계 노드 전체와 LTTng userspace trace를 묶어 실행한다.

측정 끝점:
  T_HPC_in  = ethernet_image_publisher_node 의 image_NN_raw rmw_take
  T_HPC_out = motion_planner_node 의 topic_control_signal rmw_publish

CSV 로깅:
  AUTOSDV_TRACE_SESSION 환경변수를 본 launch가 노드 환경에 주입 →
  ethernet_image_publisher_node / motion_planner_node 가 자동으로 frame_id+ts CSV 기록.

자세한 설계: .claude/plans/hpc-ros2-tracing-plan.md
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tracetools_launch.action import Trace


def generate_launch_description():
    # ── 인자 ───────────────────────────────────────────────
    session = LaunchConfiguration('session_name')
    enable_bev = LaunchConfiguration('enable_bev')
    trace_base_path = LaunchConfiguration('trace_base_path')

    decls = [
        DeclareLaunchArgument(
            'session_name', default_value='autosdv_hpc_run',
            description='LTTng UST session name (CSV/trace 디렉토리 prefix가 됨)'),
        DeclareLaunchArgument(
            'enable_bev', default_value='false',
            description='BEV branch 동시 실행 여부 (본측정은 false; future work으로 true 옵션 유지)'),
        DeclareLaunchArgument(
            'trace_base_path', default_value='/tmp/autosdv_traces',
            description='Trace/CSV 출력 베이스 경로'),
    ]

    # ── 노드 환경변수 (CSV 로깅 활성화 트리거) ─────────────
    set_session_env = SetEnvironmentVariable(
        name='AUTOSDV_TRACE_SESSION', value=session)
    set_base_env = SetEnvironmentVariable(
        name='AUTOSDV_TRACE_BASE', value=trace_base_path)

    # ── LTTng userspace trace ─────────────────────────────
    trace = Trace(
        session_name=session,
        events_ust=[
            'ros2:*',          # rclcpp/rmw 전부 (callback, rmw_publish, rmw_take 등)
            # 'dds:*',         # FastDDS instrument이 있다면 활성 (보너스, 없어도 OK)
        ],
        events_kernel=[],      # userspace only (lttng-modules 불필요)
        base_path=trace_base_path,
    )

    # ── 카메라 입력 (5채널) ───────────────────────────────
    image_publishers = [
        Node(package='camera_perception_pkg',
             executable='ethernet_image_publisher_node',
             name=f'ethernet_image_publisher_node_{i}',
             output='screen',
             parameters=[{'image': f'image_0{i}'}])
        for i in range(1, 6)
    ]

    # ── 제어 path (frame_id pass-through 경로) ────────────
    control_pipeline = [
        Node(package='camera_perception_pkg', executable='yolov8_node',
             name='yolov8_node', output='screen'),
        Node(package='camera_perception_pkg', executable='lane_info_extractor_node',
             name='lane_info_extractor_node', output='screen'),
        Node(package='decision_making_pkg', executable='path_planner_node',
             name='path_planner_node', output='screen'),
        Node(package='decision_making_pkg', executable='motion_planner_node',
             name='motion_planner_node', output='screen'),
    ]

    # ── BEV (옵션, 본측정 기본 OFF) ───────────────────────
    bev = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bev_perception_pkg'),
                         'launch', 'bev.launch.py')),
        condition=IfCondition(enable_bev))

    return LaunchDescription(
        decls
        + [set_session_env, set_base_env, trace]
        + image_publishers
        + control_pipeline
        + [bev]
    )
