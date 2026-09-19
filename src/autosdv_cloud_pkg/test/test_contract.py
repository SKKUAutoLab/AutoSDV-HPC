"""Contract and pipeline configuration checks."""

from autosdv_cloud_pkg.contract import diagnostics_payload
from autosdv_cloud_pkg.streamer_node import (  # noqa: I201
    CameraStreamerNode,
    classify_error,
)
from diagnostic_msgs.msg import (  # noqa: I201
    DiagnosticArray,
    DiagnosticStatus,
)


def status(level, name, hardware_id):
    """Build a compact ROS diagnostic status for tests."""
    return DiagnosticStatus(
        level=level, name=name, hardware_id=hardware_id)


def test_diagnostics_match_autosdv_gui_summary():
    """Only GUI-visible states are included and missing zones stay stale."""
    message = DiagnosticArray(status=[
        status(1, 'IVN/Summary', 'ivn-monitor'),
        status(0, 'IVN/ZCU/zone_1/Network', 'zcu-zone_1'),
        status(2, 'IVN/ZCU/zone_1/Runtime', 'zcu-zone_1'),
        status(0, 'IVN/ZCU/zone_2/Network', 'zcu-zone_2'),
        status(1, 'IVN/ZCU/zone_2/DDS', 'zcu-zone_2'),
        status(0, 'IVN/Camera/image_01', 'camera-image_01'),
        status(2, 'IVN/Camera/image_02', 'camera-image_02'),
    ])

    payload = diagnostics_payload(message, 7, now_ms=123)

    assert payload['vehicle_id'] == 'a0000'
    assert payload['seq'] == 7
    assert payload['ts_ms'] == 123
    assert payload['overall'] == 'WARN'
    assert payload['zones'] == [
        {'zone_id': 'zone_1', 'label': 'FL ZCU', 'status': 'ERROR'},
        {'zone_id': 'zone_2', 'label': 'FR ZCU', 'status': 'OK'},
        {'zone_id': 'zone_3', 'label': 'RL ZCU', 'status': 'STALE'},
        {'zone_id': 'zone_4', 'label': 'RR ZCU', 'status': 'STALE'},
    ]
    assert payload['cameras'] == [
        {'camera_id': 'camera_01', 'status': 'OK'}]


def test_empty_diagnostics_are_all_stale():
    """A missing ROS input still produces the complete stale schema."""
    payload = diagnostics_payload(DiagnosticArray(), 0, now_ms=123)

    assert payload['overall'] == 'STALE'
    assert [zone['status'] for zone in payload['zones']] == ['STALE'] * 4
    assert payload['cameras'][0]['status'] == 'STALE'


def test_whip_errors_map_to_contract_codes():
    """WHIP and GStreamer failures use the agreed error vocabulary."""
    assert classify_error('HTTP 401 Unauthorized') == 'AUTH_FAILED'
    assert classify_error('HTTP 404 Not Found') == 'PATH_REJECTED'
    assert classify_error('ICE connection failed') == 'ICE_FAILED'
    assert classify_error('nvv4l2h264enc failed') == 'ENCODER_ERROR'


def test_pipeline_uses_thor_supported_baseline_profile():
    """Thor profile 0 produces Constrained Baseline without B-frames."""
    node = object.__new__(CameraStreamerNode)
    node.width = 640
    node.height = 480
    node.fps = 30
    node.bitrate = 1500

    pipeline = node._pipeline_description('BGR')

    assert 'nvv4l2h264enc profile=0 ' in pipeline
    assert 'num-B-Frames=0' in pipeline
    assert 'profile=constrained-baseline' in pipeline
    assert 'capssetter replace=true' in pipeline
