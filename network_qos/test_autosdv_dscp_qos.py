import pytest
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name('autosdv_dscp_qos.py')
SPEC = importlib.util.spec_from_file_location('autosdv_dscp_qos', MODULE_PATH)
qos = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(qos)


def test_rtps_user_ports_are_odd_and_exclude_discovery_ports():
    ports = qos.rtps_user_unicast_ports(0, 3)
    assert ports == (7411, 7413, 7415, 7417)
    assert all(port % 2 == 1 for port in ports)
    assert 7400 not in ports
    assert 7410 not in ports
    assert 7412 not in ports


def test_ruleset_defaults_to_cs1_and_overrides_control_with_cs6():
    rules = qos.render_ruleset('enp4s0', ['10.0.0.2'], 0, 1)
    assert 'oifname "enp4s0" ip dscp set cs1' in rules
    assert 'udp sport @control_udp_sports' in rules
    assert 'udp dport @rtps_user_dports' in rules
    assert 'ip dscp set cs6' in rules
    assert 'elements = { 7411, 7413 }' in rules


def test_ros2_wrapper_is_not_treated_as_actual_supervisor():
    assert not qos.is_actual_safety_supervisor([
        '/usr/bin/ros2', 'run', 'autosdv_gui_pkg', 'safety_supervisor_node',
    ])
    assert qos.is_actual_safety_supervisor([
        '/usr/bin/python3',
        '/workspace/install/autosdv_gui_pkg/lib/autosdv_gui_pkg/'
        'safety_supervisor_node',
    ])


def test_parse_udp_table_selects_only_owned_socket_inodes():
    table = '''   sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode
  10: 0A00000A:D8E7 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 12345 2 0000000000000000 0
  11: 0A00000A:1CF5 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 99999 2 0000000000000000 0
'''
    assert qos.parse_udp_table(table, {12345}) == {55527}


def test_source_port_update_is_atomic_batch_text():
    update = qos.render_source_port_update([55527, 7413, 55527])
    assert update.startswith('flush set ip autosdv_dscp control_udp_sports\n')
    assert '{ 7413, 55527 }' in update


def test_apply_does_not_require_effective_uid_zero(monkeypatch):
    calls = []
    monkeypatch.setattr(qos, 'delete_owned_table', lambda binary: calls.append('delete'))
    monkeypatch.setattr(qos, 'run_nft', lambda binary, rules: calls.append('apply'))
    monkeypatch.setattr(qos.os, 'geteuid', lambda: 1000)

    args = type('Args', (), {
        'interface': 'enp4s0',
        'zcu': ['10.0.0.2'],
        'domain_id': 0,
        'max_participant_id': 1,
        'nft': '/usr/sbin/nft',
        'middleware': qos.FASTDDS,
    })()
    qos.apply_rules(args)

    assert calls == ['delete', 'apply']


def test_zenoh_ruleset_matches_the_router_control_port():
    ruleset = qos.render_ruleset(
        "enp4s0", ("10.0.0.5",), 0, 119, qos.ZENOH)

    # Every HPC node reaches a ZCU through zenohd, so there is no per-process
    # source port to follow and no RTPS destination port to calculate.
    assert qos.CONTROL_SPORT_SET not in ruleset
    assert qos.RTPS_USER_DPORT_SET not in ruleset
    assert f"th sport {qos.ZENOH_CONTROL_PORT}" in ruleset
    assert "ip dscp set cs6" in ruleset
    assert "ip dscp set cs1" in ruleset
    # Camera ports must not be marked CS6; they are CS5 at the ZCUs already.
    assert all(str(port) not in ruleset for port in (17451, 17453, 17455, 17457, 17459))


def test_zenoh_bridge_takes_the_same_rule_as_native_zenoh():
    # The bridge environment keeps DDS on loopback and puts the same two
    # listening ports on the wire, so the classification is identical and the
    # two environments stay comparable.
    assert (
        qos.render_ruleset("enp4s0", ("10.0.0.5",), 0, 119, qos.ZENOH_BRIDGE)
        == qos.render_ruleset("enp4s0", ("10.0.0.5",), 0, 119, qos.ZENOH))


def test_rtps_rulesets_are_unchanged_for_both_dds_middlewares():
    fastdds = qos.render_ruleset(
        "enp4s0", ("10.0.0.5",), 0, 119, qos.FASTDDS)
    cyclonedds = qos.render_ruleset(
        "enp4s0", ("10.0.0.5",), 0, 119, qos.CYCLONEDDS)
    assert fastdds == cyclonedds
    assert qos.CONTROL_SPORT_SET in fastdds
    assert qos.RTPS_USER_DPORT_SET in fastdds


def test_unknown_middleware_is_rejected():
    with pytest.raises(ValueError):
        qos.render_ruleset("enp4s0", ("10.0.0.5",), 0, 119, "rti")
