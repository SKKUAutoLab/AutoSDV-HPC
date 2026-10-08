import pytest
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name('autosdv_dscp_qos.py')
SPEC = importlib.util.spec_from_file_location('autosdv_dscp_qos', MODULE_PATH)
qos = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(qos)


def _rules(middleware):
    return qos.render_ruleset('enP2p1s0', ['10.0.0.2', '10.0.0.5'], middleware)


@pytest.mark.parametrize('middleware', qos.MIDDLEWARES)
def test_no_catch_all_rule_that_would_erase_socket_marks(middleware):
    # A rule without a port match would rewrite the TOS the shim set on the
    # node's socket. Every rule must be narrowed to a source port.
    for line in _rules(middleware).splitlines():
        if 'ip dscp set' in line:
            assert 'th sport' in line
    assert 'cs1' not in _rules(middleware)


@pytest.mark.parametrize('middleware', (qos.CYCLONEDDS, qos.FASTDDS))
def test_rtps_middlewares_install_no_rule(middleware):
    # Cyclone/Fast DDS nodes are marked per process by the shim.
    assert 'ip dscp set' not in _rules(middleware)
    assert 'table ip autosdv_dscp' in _rules(middleware)


@pytest.mark.parametrize('middleware', qos.PORT_CLASSIFIED)
def test_zenoh_control_port_is_cs6_and_camera_ports_are_cs5(middleware):
    rules = _rules(middleware)
    assert 'th sport 7448 ip dscp set cs6' in rules
    assert 'th sport { 17451, 17453, 17455, 17457, 17459 } ip dscp set cs5' in rules
    assert 'oifname "enP2p1s0" ip daddr @zcu_addresses' in rules


@pytest.mark.parametrize('middleware', qos.PORT_CLASSIFIED)
def test_health_port_stays_cs0(middleware):
    assert str(qos.ZENOH_HEALTH_PORT) not in _rules(middleware)


def test_apply_does_not_require_effective_uid_zero(monkeypatch):
    calls = []
    monkeypatch.setattr(qos, 'delete_owned_table', lambda binary: calls.append('delete'))
    monkeypatch.setattr(qos, 'run_nft', lambda binary, rules: calls.append('apply'))
    monkeypatch.setattr(qos.os, 'geteuid', lambda: 1000)

    args = type('Args', (), {
        'interface': 'enP2p1s0',
        'zcu': ['10.0.0.2'],
        'nft': '/usr/sbin/nft',
        'middleware': qos.ZENOH,
    })()
    qos.apply_rules(args)

    assert calls == ['delete', 'apply']


def test_unknown_middleware_is_rejected():
    with pytest.raises(ValueError):
        _rules('zenoh_bridge_pico')


def test_invalid_interface_is_rejected():
    with pytest.raises(ValueError):
        qos.render_ruleset('enp4s0; drop', ['10.0.0.2'], qos.ZENOH)
