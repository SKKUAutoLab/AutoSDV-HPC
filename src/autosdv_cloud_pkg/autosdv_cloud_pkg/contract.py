"""Vehicle-server payload constants and diagnostic reduction."""

import time


SCHEMA_VERSION = '1.0'
VEHICLE_ID = 'a0000'
CAMERA_ID = 'camera_01'
ZONE_LABELS = {
    'zone_1': 'FL ZCU',
    'zone_2': 'FR ZCU',
    'zone_3': 'RL ZCU',
    'zone_4': 'RR ZCU',
}
STATE_BY_LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
STATE_RANK = {'OK': 0, 'WARN': 1, 'STALE': 2, 'ERROR': 3}


def state_for_level(level):
    """Convert a ROS diagnostic level to the interface state string."""
    if isinstance(level, (bytes, bytearray)):
        level = level[0] if level else 3
    return STATE_BY_LEVEL.get(int(level), 'STALE')


def worst_state(current, candidate):
    """Return the state with the same priority used by autosdv_gui."""
    if STATE_RANK[candidate] > STATE_RANK[current]:
        return candidate
    return current


def diagnostics_payload(message, seq, vehicle_id=VEHICLE_ID, now_ms=None):
    """Reduce DiagnosticArray to the fields displayed by autosdv_gui."""
    overall = 'STALE'
    zones = {zone_id: 'STALE' for zone_id in ZONE_LABELS}
    seen_zones = set()
    camera = 'STALE'
    seen_camera = False

    for status in message.status:
        state = state_for_level(status.level)
        if status.name == 'IVN/Summary':
            overall = state
        if (status.hardware_id.startswith('zcu-zone_')
                and status.name.endswith(('/Network', '/Runtime'))):
            zone_id = status.hardware_id[len('zcu-'):]
            if zone_id in zones:
                if zone_id not in seen_zones:
                    zones[zone_id] = state
                    seen_zones.add(zone_id)
                else:
                    zones[zone_id] = worst_state(zones[zone_id], state)
        if status.hardware_id == 'camera-image_01':
            camera = state if not seen_camera else worst_state(camera, state)
            seen_camera = True

    return {
        'schema_version': SCHEMA_VERSION,
        'vehicle_id': vehicle_id,
        'seq': int(seq),
        'ts_ms': int(
            now_ms if now_ms is not None else time.time_ns() // 1_000_000),
        'overall': overall,
        'zones': [
            {
                'zone_id': zone_id,
                'label': label,
                'status': zones[zone_id],
            }
            for zone_id, label in ZONE_LABELS.items()
        ],
        'cameras': [{'camera_id': CAMERA_ID, 'status': camera}],
    }
