from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from ivn_diagnostics_pkg.ivn_diagnostics_node import (
    build_automatic_statuses,
    build_manual_checks,
    HealthRecord,
    make_status,
    split_health_statuses,
    worst_level,
    ZoneConfig,
)


def _zones():
    return (
        ZoneConfig('zone_1', 'Front Left', '10.0.0.2',
                   ('/image_01_raw', '/image_02_raw')),
        ZoneConfig('zone_2', 'Front Right', '10.0.0.3',
                   ('/image_03_raw',)),
    )


def _health_status(zone_id, level=DiagnosticStatus.OK,
                   component='summary', component_id=None):
    values = {
        'zone_id': zone_id,
        'component': component.lower(),
    }
    if component_id is not None:
        values[f'{component.lower()}_id'] = component_id
    return make_status(
        level,
        f'IVN/ZCU/{zone_id}/{component}',
        'healthy' if level == DiagnosticStatus.OK else 'component fault',
        f'zcu-{zone_id}',
        **values,
    )


def _record(now, zone_id, level=DiagnosticStatus.OK, age=0.1):
    camera_ids = {
        'zone_1': ('image_01', 'image_02'),
        'zone_2': ('image_03',),
    }.get(zone_id, ())
    return HealthRecord(
        received_at=now - age,
        statuses=(
            _health_status(zone_id, level),
            _health_status(zone_id, level, 'runtime'),
            *(
                _health_status(
                    zone_id, level, 'camera', camera_id)
                for camera_id in camera_ids
            ),
        ),
        sequence=3,
    )


def _by_name(statuses):
    return {status.name: status for status in statuses}


def test_automatic_monitor_combines_network_discovery_and_health():
    now = 100.0
    zones = _zones()
    statuses = _by_name(build_automatic_statuses(
        zones,
        [True, False],
        {
            '/image_01_raw': 1,
            '/image_02_raw': 0,
            '/image_03_raw': 1,
        },
        {
            'zone_1': _record(now, 'zone_1'),
            'zone_2': _record(now, 'zone_2'),
        },
        now,
        2.0,
    ))

    assert statuses['IVN/ZCU/zone_1/Network'].level == DiagnosticStatus.OK
    assert statuses['IVN/ZCU/zone_1/DDS'].level == DiagnosticStatus.ERROR
    assert statuses['IVN/ZCU/zone_1/Health'].level == DiagnosticStatus.OK
    assert statuses['IVN/ZCU/zone_2/Network'].level == DiagnosticStatus.ERROR
    assert statuses['IVN/Camera/image_03'].level == DiagnosticStatus.STALE
    assert statuses['IVN/Summary'].level == DiagnosticStatus.ERROR


def test_missing_and_timed_out_health_are_stale_then_recover():
    now = 200.0
    zones = (_zones()[0],)
    counts = {'/image_01_raw': 1, '/image_02_raw': 1}

    missing = _by_name(build_automatic_statuses(
        zones, [True], counts, {}, now, 2.0))
    assert missing['IVN/ZCU/zone_1/Health'].level == DiagnosticStatus.STALE

    expired = _by_name(build_automatic_statuses(
        zones,
        [True],
        counts,
        {'zone_1': _record(now, 'zone_1', age=2.1)},
        now,
        2.0,
    ))
    assert expired['IVN/ZCU/zone_1/Health'].level == DiagnosticStatus.STALE

    recovered = _by_name(build_automatic_statuses(
        zones,
        [True],
        counts,
        {'zone_1': _record(now, 'zone_1', age=0.0)},
        now,
        2.0,
    ))
    assert recovered['IVN/ZCU/zone_1/Health'].level == DiagnosticStatus.OK
    assert recovered['IVN/Summary'].level == DiagnosticStatus.OK


def test_zcu_component_error_is_reflected_in_summary():
    now = 300.0
    zone = _zones()[0]
    record = HealthRecord(
        received_at=now,
        statuses=(
            _health_status('zone_1', DiagnosticStatus.OK),
            _health_status('zone_1', DiagnosticStatus.OK, 'runtime'),
            _health_status(
                'zone_1', DiagnosticStatus.ERROR, 'CAN/can0'),
        ),
        sequence=5,
    )
    statuses = _by_name(build_automatic_statuses(
        (zone,),
        [True],
        {'/image_01_raw': 1, '/image_02_raw': 1},
        {'zone_1': record},
        now,
        2.0,
    ))

    assert statuses['IVN/ZCU/zone_1/Health'].level == DiagnosticStatus.ERROR
    assert statuses['IVN/Summary'].level == DiagnosticStatus.ERROR


def test_camera_fault_colors_camera_without_marking_zcu_runtime_failed():
    now = 350.0
    zone = ZoneConfig(
        'zone_3', 'Rear Left', '10.0.0.4', ('/image_04_raw',))
    record = HealthRecord(
        received_at=now,
        statuses=(
            _health_status('zone_3', DiagnosticStatus.ERROR),
            _health_status('zone_3', DiagnosticStatus.OK, 'runtime'),
            _health_status(
                'zone_3', DiagnosticStatus.ERROR, 'camera', 'image_04'),
            _health_status('zone_3', DiagnosticStatus.OK, 'can', 'can0'),
        ),
        sequence=7,
    )

    statuses = _by_name(build_automatic_statuses(
        (zone,),
        [True],
        {'/image_04_raw': 1},
        {'zone_3': record},
        now,
        2.0,
    ))

    assert statuses['IVN/ZCU/zone_3/Network'].level == DiagnosticStatus.OK
    assert statuses['IVN/ZCU/zone_3/Runtime'].level == DiagnosticStatus.OK
    assert statuses['IVN/Camera/image_04'].level == DiagnosticStatus.ERROR
    assert statuses['IVN/ZCU/zone_3/Health'].level == DiagnosticStatus.ERROR
    assert statuses['IVN/Summary'].level == DiagnosticStatus.ERROR


def test_shared_health_topic_is_grouped_by_zone_and_unknown_is_ignored():
    message = DiagnosticArray(status=[
        _health_status('zone_1'),
        _health_status('zone_2'),
        _health_status('zone_9'),
        make_status(
            DiagnosticStatus.OK,
            'IVN/ZCU/bad/Runtime',
            'bad hardware id',
            'not-a-zcu',
        ),
    ])

    grouped = split_health_statuses(message, ('zone_1', 'zone_2'))

    assert set(grouped) == {'zone_1', 'zone_2'}
    assert grouped['zone_1'][0].hardware_id == 'zcu-zone_1'
    assert grouped['zone_2'][0].hardware_id == 'zcu-zone_2'


def test_topic_status_signal_is_not_part_of_automatic_or_manual_checks():
    now = 400.0
    zone = _zones()[0]
    statuses = build_automatic_statuses(
        (zone,),
        [True],
        {'/image_01_raw': 1, '/image_02_raw': 1},
        {'zone_1': _record(now, 'zone_1', age=0.0)},
        now,
        2.0,
    )
    manual_checks = build_manual_checks((zone,), check_data_flow=True)

    assert all('ECU' not in status.name for status in statuses)
    assert all('/topic_status_signal' not in status.message
               for status in statuses)
    assert [kind for kind, _, _ in manual_checks] == [
        'network', 'camera_data', 'camera_data', 'zcu_health',
    ]
    assert _by_name(statuses)['IVN/Summary'].level == DiagnosticStatus.OK


def test_explicit_severity_order_prioritizes_error_over_stale():
    assert worst_level([
        DiagnosticStatus.OK,
        DiagnosticStatus.STALE,
        DiagnosticStatus.ERROR,
    ]) == DiagnosticStatus.ERROR
