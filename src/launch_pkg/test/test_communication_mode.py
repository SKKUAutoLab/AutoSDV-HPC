import importlib.util
import json
from pathlib import Path
import threading

import pytest

from launch_pkg import communication_mode as cm
from launch_pkg import rmw_profile


class Hosts:
    def __init__(self, fail=None):
        self.calls = []
        self.lock = threading.Lock()
        self.fail = fail

    def __call__(self, host, action, mode):
        with self.lock:
            self.calls.append((host, action, mode))
        if (host, action) == self.fail:
            raise RuntimeError('injected failure')
        return {'mode': mode}


def test_preflight_failure_does_not_mutate():
    hosts = Hosts(('10.0.0.4', 'check'))
    with pytest.raises(RuntimeError, match='10.0.0.4'):
        cm.Coordinator(hosts, lambda _: None).apply('cyclonedds')
    assert len(hosts.calls) == 5
    assert {action for _, action, _ in hosts.calls} == {'check'}


@pytest.mark.parametrize('failure', [('10.0.0.3', 'prepare'),
                                     ('10.0.0.5', 'commit'), ('HPC', 'start')])
def test_partial_failure_marks_all_hosts_failed_and_stops(failure):
    hosts = Hosts(failure)
    with pytest.raises(RuntimeError):
        cm.Coordinator(hosts, lambda _: None).apply('zenoh')
    assert {host for host, action, _ in hosts.calls if action == 'fail'} == set(cm.HOSTS)
    assert ('HPC', 'commit', 'zenoh') not in hosts.calls


def test_success_order_and_last_commit():
    hosts = Hosts()
    cm.Coordinator(hosts, lambda _: None).apply('cyclonedds')
    actions = [action for _, action, _ in hosts.calls]
    assert max(i for i, x in enumerate(actions) if x == 'check') < actions.index('begin')
    assert max(i for i, x in enumerate(actions) if x == 'begin') < actions.index('stop')
    assert hosts.calls.index(('HPC', 'verify', 'cyclonedds')) < min(
        i for i, (h, a, _) in enumerate(hosts.calls) if h != 'HPC' and a == 'start')
    assert hosts.calls[-1] == ('HPC', 'commit', 'cyclonedds')


def test_saved_selection_and_failed_state_block_launch(tmp_path, monkeypatch):
    path = tmp_path / 'state.json'
    monkeypatch.setattr(cm, 'STATE', path)
    monkeypatch.delenv('AUTOSDV_RMW', raising=False)
    path.write_text(json.dumps({'status': 'ready', 'mode': 'zenoh_bridge_pico'}))
    assert rmw_profile.middleware() == 'zenoh_bridge_pico'
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config/cyclonedds_bridge_local.xml').write_text('<CycloneDDS/>')
    assert rmw_profile.default_profile_env(str(tmp_path))['ROS_LOCALHOST_ONLY'] == ''
    assert rmw_profile.profile_filename('fastdds_image_01_participant20.xml') == 'cyclonedds_bridge_local.xml'
    monkeypatch.setenv('AUTOSDV_RMW', 'cyclonedds')
    with pytest.raises(RuntimeError, match='conflicts'):
        rmw_profile.middleware()
    path.write_text(json.dumps({'status': 'failed', 'mode': 'cyclonedds'}))
    with pytest.raises(RuntimeError, match='incomplete'):
        rmw_profile.middleware()


@pytest.fixture
def host_adapter(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location('communication_host', root / 'scripts/communication_host.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'STATE', tmp_path / 'state.json')
    monkeypatch.setattr(module, 'RUNTIME', tmp_path / 'runtime')
    return module


def test_uncommitted_mode_cannot_restore_after_boot(host_adapter):
    h = host_adapter
    h.save(h.STATE, {'status': 'applying', 'mode': 'zenoh', 'requested': 'cyclonedds'})
    with pytest.raises(RuntimeError, match='Incomplete'):
        h.serve({'role': 'zcu'})


def test_boot_restores_only_committed_mode(host_adapter, monkeypatch):
    h = host_adapter
    h.save(h.STATE, {'status': 'ready', 'mode': 'zenoh'})
    calls = []
    monkeypatch.setattr(h, 'qos', lambda config, mode: calls.append(mode))
    h.serve({'role': 'zcu'})
    assert calls == ['zenoh']


def test_verify_rejects_wrong_effective_mode(host_adapter):
    h = host_adapter
    h.RUNTIME.write_text('zenoh')
    with pytest.raises(RuntimeError, match='does not match'):
        h.verify({'role': 'hpc'}, 'cyclonedds')


def test_zcu_library_check_without_ldd(host_adapter, monkeypatch):
    calls = []

    def run(*args):
        calls.append(args)
        if args[0] == 'ldd':
            raise FileNotFoundError('ldd')
        return 'libddsc.so.0 => /usr/local/lib64/libddsc.so.0'

    monkeypatch.setattr(host_adapter, 'run', run)
    assert 'libddsc' in host_adapter.linked_libraries('/tmp/zcu')
    assert calls[-1] == ('/lib/ld-linux-aarch64.so.1', '--list', '/tmp/zcu')


def test_library_error_is_not_hidden(host_adapter, monkeypatch):
    def run(*args):
        raise RuntimeError('libddsc.so.0 not found')

    monkeypatch.setattr(host_adapter, 'run', run)
    with pytest.raises(RuntimeError, match='not found'):
        host_adapter.linked_libraries('/tmp/zcu')


def test_hpc_qos_uses_actual_ivn_interface(host_adapter, monkeypatch):
    interfaces = [
        {'ifname': 'wlan0', 'addr_info': [{'local': '192.168.0.10'}]},
        {'ifname': 'enP2p1s0', 'addr_info': [{'local': '10.0.0.10'}]},
    ]
    monkeypatch.setattr(host_adapter, 'run', lambda *args: json.dumps(interfaces))
    command = host_adapter.qos({'role': 'hpc', 'workspace': '/tmp/hpc'}, 'zenoh')
    assert command[command.index('--interface') + 1] == 'enP2p1s0'


def test_hpc_qos_rejects_missing_ivn_address(host_adapter, monkeypatch):
    monkeypatch.setattr(host_adapter, 'run', lambda *args: '[]')
    with pytest.raises(RuntimeError, match='10.0.0.10'):
        host_adapter.qos({'role': 'hpc', 'workspace': '/tmp/hpc'}, 'cyclonedds')


@pytest.mark.parametrize('mode', ['cyclonedds', 'zenoh', 'zenoh_bridge', 'zenoh_bridge_pico'])
def test_zcu_qos_installs_no_tc_rule_and_retires_old_ones(host_adapter, monkeypatch, tmp_path,
                                                         mode):
    # ZCU egress is marked per process by the DSCP shim in the units; a tc
    # port rule would re-mark flows, so qos() only deletes the old prefs.
    (tmp_path / '.zone').write_text('zone_1')
    calls = []
    monkeypatch.setattr(host_adapter, 'run', lambda *args, **kwargs: calls.append(args) or '')
    host_adapter.qos({'role': 'zcu', 'workspace': str(tmp_path)}, mode)
    tc = [args for args in calls if args[0] == '/sbin/tc']
    assert tc and all(args[1:3] == ('filter', 'del') for args in tc)
    assert {args[args.index('pref') + 1] for args in tc} == {'10', '101', '102', '103', '104', '105'}


def test_zcu_units_are_one_per_traffic_class(host_adapter, tmp_path):
    (tmp_path / '.zone').write_text('zone_3')
    assert host_adapter.units({'role': 'zcu', 'workspace': str(tmp_path)}) == [
        'zcu-camera.service', 'zcu-control.service', 'zcu-health@zone_3.service',
        host_adapter.UNIT]


def test_wired_cyclone_profiles_use_unicast():
    import xml.etree.ElementTree as ET
    root = Path(__file__).resolve().parents[3]
    files = [root / 'config/dds/cyclonedds_ivn.xml'] + sorted(
        (root / 'src/launch_pkg/config').glob('cyclonedds_image_*.xml'))
    assert len(files) == 6
    for path in files:
        tree = ET.parse(path)
        ns = {'c': 'https://cdds.io/config'}
        assert tree.findtext('.//c:AllowMulticast', namespaces=ns) == 'false', path
        assert '10.0.0.10' in [peer.attrib['address'] for peer in
                              tree.findall('.//c:Peers/c:Peer', ns)], path
        index = tree.findtext('.//c:ParticipantIndex', namespaces=ns)
        assert index == ('auto' if path.name == 'cyclonedds_ivn.xml'
                         else str(19 + int(path.name.split('_')[2]))), path


def test_pico_check_rejects_non_pico_health_binary(host_adapter, tmp_path, monkeypatch):
    build = tmp_path / 'build-zenoh_bridge_pico'
    build.mkdir()
    (build / host_adapter.ZCU_SHIM).touch()
    for name in host_adapter.ZCU_BINARIES:
        binary = build / name
        binary.touch()
        binary.chmod(0o755)
    monkeypatch.setattr(host_adapter, 'linked_libraries', lambda binary:
                        'libzenohc.so' if binary.name == 'zcu_health_node' else 'libzenohpico.so')
    with pytest.raises(RuntimeError, match='zcu_health_node'):
        host_adapter.check({'role': 'zcu', 'workspace': str(tmp_path)}, 'zenoh_bridge_pico')


def test_zcu_check_requires_the_dscp_shim(host_adapter, tmp_path):
    build = tmp_path / 'build-zenoh'
    build.mkdir()
    for name in host_adapter.ZCU_BINARIES:
        (build / name).touch()
        (build / name).chmod(0o755)
    with pytest.raises(RuntimeError, match='libautosdv_dscp_shim.so'):
        host_adapter.check({'role': 'zcu', 'workspace': str(tmp_path)}, 'zenoh')


@pytest.mark.parametrize('traffic_class,tos', [('control', '0xc0'), ('camera', '0xa0')])
def test_dscp_env_preloads_the_shim_with_the_class_tos(monkeypatch, tmp_path, traffic_class, tos):
    shim = tmp_path / 'libautosdv_dscp_shim.so'
    shim.touch()
    monkeypatch.setattr(rmw_profile, 'dscp_shim_path', lambda: str(shim))
    env = rmw_profile.dscp_env(traffic_class, inherited_preload='/opt/other.so')
    assert env == {'LD_PRELOAD': f'{shim} /opt/other.so', 'AUTOSDV_IP_TOS': tos}


def test_dscp_env_leaves_unclassified_nodes_alone():
    assert rmw_profile.dscp_env('') == {}


def test_dscp_env_rejects_unknown_class_and_missing_shim(monkeypatch, tmp_path):
    monkeypatch.setattr(rmw_profile, 'dscp_shim_path', lambda: str(tmp_path / 'missing.so'))
    with pytest.raises(RuntimeError, match='traffic_class'):
        rmw_profile.dscp_env('health')
    with pytest.raises(RuntimeError, match='rebuild launch_pkg'):
        rmw_profile.dscp_env('camera')


def _settle_run(host_adapter, samples, address=True, hold=10.0):
    """Feed (time, carrier) samples; return the times a restart was requested."""
    settle = host_adapter.LinkSettle(hold=hold)
    return [t for t, up in samples if settle.update(up, lambda: address, t)]


def test_link_settle_restarts_once_after_the_link_is_stable(host_adapter):
    samples = [(0, True), (1, False), (2, False), (3, True)] + [(3 + i, True) for i in range(1, 20)]
    assert _settle_run(host_adapter, samples) == [13]


def test_link_settle_waits_out_a_flapping_switch(host_adapter):
    # Down/up twice within 20 s, as an SJA1110 reboot does: one restart, after
    # the second recovery has held for the full window.
    samples = ([(0, False), (4, True), (8, True), (12, False), (16, True)]
               + [(16 + i, True) for i in range(1, 15)])
    assert _settle_run(host_adapter, samples) == [26]


def test_link_settle_waits_for_the_ivn_address(host_adapter):
    samples = [(0, False), (1, True)] + [(1 + i, True) for i in range(1, 30)]
    assert _settle_run(host_adapter, samples, address=False) == []


def test_link_settle_never_restarts_without_a_drop(host_adapter):
    assert _settle_run(host_adapter, [(i, True) for i in range(60)]) == []
