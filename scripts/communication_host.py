#!/usr/bin/env python3
"""Privileged, installed host adapter for coordinated AutoSDV mode changes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

MODES = ('cyclonedds', 'zenoh', 'zenoh_bridge', 'zenoh_bridge_pico')
CYCLONE_CAMERA_PORTS_BY_ZONE = {'zone_1': [7451, 7453], 'zone_2': [7455],
                                'zone_3': [7457], 'zone_4': [7459]}
ZENOH_CAMERA_PORTS_BY_ZONE = {'zone_1': [17451, 17453], 'zone_2': [17455],
                              'zone_3': [17457], 'zone_4': [17459]}
ZENOH_UDP_PORTS = (7447, 7448, 17451, 17453, 17455, 17457, 17459)
CONFIG = Path('/etc/autosdv/communication-host.json')
STATE = Path('/var/lib/autosdv/communication-mode.json')
RUNTIME = Path('/run/autosdv-communication-mode')
UNIT = 'autosdv-communication.service'


def run(*args, check=True):
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, timeout=90)
    if check and result.returncode:
        raise RuntimeError(f'{shlex.join(args)}: {result.stdout.strip()}')
    return result.stdout.strip()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value) + '\n')
    temp.chmod(0o644)
    os.replace(temp, path)


def read_state():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def linked_libraries(binary):
    try:
        return run('ldd', str(binary))
    except FileNotFoundError:
        # The ZCU's minimal AArch64 Linux image omits the ldd wrapper.
        return run('/lib/ld-linux-aarch64.so.1', '--list', str(binary))


def hpc_ivn_interface():
    interfaces = json.loads(run('ip', '-j', '-4', 'addr', 'show'))
    matches = [item['ifname'] for item in interfaces
               if any(addr.get('local') == '10.0.0.10'
                      for addr in item.get('addr_info', []))]
    if len(matches) != 1:
        raise RuntimeError('Exactly one HPC interface with IVN address 10.0.0.10 required')
    return matches[0]


def check(config, mode):
    root = Path(config['workspace'])
    if config['role'] == 'zcu':
        for name in ('zcu', 'zcu_health_node'):
            binary = root / f'build-{mode}' / name
            if not os.access(binary, os.X_OK):
                raise RuntimeError(f'Not built: {binary}')
            linked = linked_libraries(binary)
            if 'not found' in linked:
                raise RuntimeError(f'Missing library: {linked}')
            if mode == 'zenoh_bridge_pico' and 'libzenohpico.so' not in linked:
                raise RuntimeError(f'Pico backend linkage not verified for {name}; shared libzenohpico required')
        zone = (root / '.zone').read_text().strip()
        if zone not in ('zone_1', 'zone_2', 'zone_3', 'zone_4'):
            raise RuntimeError('Invalid .zone')
        if not (root / 'config' / f'{zone}.conf').is_file():
            raise RuntimeError(f'Missing zone configuration: {zone}')
        for unit in ('zcu.service', f'zcu-health@{zone}.service'):
            run('systemctl', 'cat', unit)
        run('/sbin/tc', '-V')
    else:
        hpc_ivn_interface()
        rmw = 'rmw_zenoh_cpp' if mode == 'zenoh' else 'rmw_cyclonedds_cpp'
        run('bash', '-c', f'source /opt/ros/jazzy/setup.bash; ros2 pkg prefix {rmw}')
        required = ['install/local_setup.bash', 'network_qos/autosdv_dscp_qos.py']
        if mode == 'zenoh':
            required += ['scripts/run_zenoh_router.sh', 'zenoh_router_ivn.json5']
        elif mode.startswith('zenoh_bridge'):
            required += ['scripts/run_zenoh_bridge.sh', 'cyclonedds_bridge_local.xml',
                         'zenoh_bridge_camera_ivn.json5', 'zenoh_bridge_control_ivn.json5',
                         'tools/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds']
            version = run(str(root / required[-1]), '--version')
            if 'v1.10.0' not in version:
                raise RuntimeError(f'Bridge 1.10.0 required: {version}')
        for file in required:
            if not (root / file).is_file():
                raise RuntimeError(f'Missing: {root / file}')
        run('/usr/sbin/nft', '--version')
    run('systemctl', 'cat', UNIT)
    if config['role'] == 'hpc':
        profile = ('zenoh_session_ivn.json5' if mode == 'zenoh' else
                   'cyclonedds_bridge_local.xml' if mode.startswith('zenoh_bridge') else
                   'cyclonedds_ivn.xml')
        installed = root / 'install/launch_pkg/share/launch_pkg/config'
        profiles = [profile]
        if not mode.startswith('zenoh_bridge'):
            prefix = 'zenoh' if mode == 'zenoh' else 'cyclonedds'
            suffix = 'json5' if mode == 'zenoh' else 'xml'
            profiles += [f'{prefix}_image_{i:02d}_participant{i+19}.{suffix}'
                         for i in range(1, 6)]
        for file in profiles:
            if not (installed / file).is_file():
                raise RuntimeError(f'Rebuild launch_pkg: missing installed {file}')
        active = run('systemctl', 'is-active', UNIT, check=False) == 'active'
        sockets = run('ss', '-H', '-lun')
        if not active and any(f':{port} ' in sockets for port in (7447, 7448)):
            raise RuntimeError('Unmanaged Zenoh listener on 7447/7448; stop it before switching')
    return {'host': config['role'], 'mode': mode, 'ready': True}


def units(config):
    if config['role'] == 'hpc':
        return [UNIT]
    zone = (Path(config['workspace']) / '.zone').read_text().strip()
    return ['zcu.service', f'zcu-health@{zone}.service', UNIT]


def stop(config):
    run('systemctl', 'stop', *units(config))
    if config['role'] == 'hpc':
        # Stop only executables registered by the installer from this workspace.
        pids = []
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                argv = (proc / 'cmdline').read_bytes().decode().split('\0')
                if any(arg in config.get('node_executables', []) for arg in argv):
                    os.kill(int(proc.name), signal.SIGINT)
                    pids.append(proc)
            except (OSError, UnicodeError):
                continue
        deadline = time.monotonic() + 8
        while any(p.exists() for p in pids) and time.monotonic() < deadline:
            time.sleep(.1)
        if any(p.exists() for p in pids):
            raise RuntimeError('HPC node did not stop; switch aborted')
        if run('systemctl', 'show', '-p', 'LoadState', '--value',
               'autosdv-dscp-qos.service', check=False) == 'loaded':
            run('systemctl', 'disable', '--now', 'autosdv-dscp-qos.service')


def qos(config, mode):
    root = Path(config['workspace'])
    if config['role'] == 'hpc':
        mapped = 'zenoh_bridge' if mode == 'zenoh_bridge_pico' else mode
        return [sys.executable, str(root / 'network_qos/autosdv_dscp_qos.py'),
                'watch', '--middleware', mapped, '--interface', hpc_ivn_interface(),
                '--cleanup-on-exit']
    # Retire only the existing AutoSDV DSCP units and their reserved pref 10.
    old = run('systemctl', 'list-units', '--all', '--plain', '--no-legend',
              'zcu-camera-dscp*').splitlines()
    names = [line.split()[0] for line in old if line.split()]
    if names:
        run('systemctl', 'disable', '--now', *names)
    run('/sbin/tc', 'qdisc', 'add', 'dev', 'pfe0', 'clsact', check=False)
    for pref in (10, 101, 102, 103, 104, 105):
        run('/sbin/tc', 'filter', 'del', 'dev', 'pfe0', 'egress', 'protocol',
            'ip', 'pref', str(pref), check=False)
    zone = (root / '.zone').read_text().strip()
    ports = (CYCLONE_CAMERA_PORTS_BY_ZONE if mode == 'cyclonedds'
             else ZENOH_CAMERA_PORTS_BY_ZONE)[zone]
    for index, port in enumerate(ports, 101):
        run('/sbin/tc', 'filter', 'replace', 'dev', 'pfe0', 'egress', 'protocol',
            'ip', 'pref', str(index), 'flower', 'ip_proto', 'udp',
            'dst_ip', '10.0.0.10', 'dst_port', str(port), 'action', 'pedit',
            'ex', 'munge', 'ip', 'dsfield', 'set', '0xa0', 'retain', '0xfc',
            'pipe', 'action', 'csum', 'ip')


def serve(config):
    state = read_state()
    if config['role'] == 'zcu' and not state and not RUNTIME.exists():
        return  # Before the first switch, preserve the existing boot policy.
    mode = RUNTIME.read_text().strip() if RUNTIME.exists() else state.get('mode')
    if not RUNTIME.exists() and state.get('status') != 'ready':
        raise RuntimeError('Incomplete mode switch; boot restore is blocked')
    if mode not in MODES:
        raise RuntimeError('No committed communication mode')
    if config['role'] == 'zcu':
        qos(config, mode)
        return
    root = Path(config['workspace'])
    env = dict(os.environ)
    for key in list(env):
        if key.startswith(('ZENOH_', 'ROS_', 'CYCLONEDDS_', 'FASTRTPS_', 'FASTDDS_')):
            env.pop(key)
    env.update(AUTOSDV_RMW=mode, ROS_DOMAIN_ID='0', ROS_DISTRO='jazzy')
    children = []
    def terminate(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    try:
        children.append(subprocess.Popen(qos(config, mode), env=env))
        launcher = ('run_zenoh_router.sh' if mode == 'zenoh' else
                    'run_zenoh_bridge.sh' if mode.startswith('zenoh_bridge') else None)
        if launcher:
            children.append(subprocess.Popen(['bash', str(root / 'scripts' / launcher)], env=env))
        while all(p.poll() is None for p in children):
            time.sleep(.2)
        raise RuntimeError('A communication child exited')
    finally:
        for p in children:
            if p.poll() is None:
                p.terminate()
        for p in children:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill(); p.wait()


def verify(config, mode):
    effective = RUNTIME.read_text().strip() if RUNTIME.exists() else read_state().get('mode')
    if effective != mode:
        raise RuntimeError(f'Effective mode {effective!r} does not match {mode}')
    for unit in units(config):
        run('systemctl', 'is-active', '--quiet', unit)
    root = Path(config['workspace'])
    if config['role'] == 'zcu':
        zone = (root / '.zone').read_text().strip()
        for name, unit in [('zcu', 'zcu.service'),
                           ('zcu_health_node', f'zcu-health@{zone}.service')]:
            pid = run('systemctl', 'show', '-p', 'MainPID', '--value', unit)
            if digest(f'/proc/{pid}/exe') != digest(root / f'build-{mode}' / name):
                raise RuntimeError(f'{unit}: running binary does not match {mode}')
    elif mode != 'cyclonedds':
        sockets = run('ss', '-H', '-lun')
        if not all(f':{port} ' in sockets for port in ZENOH_UDP_PORTS):
            raise RuntimeError('Zenoh UDP listeners not ready on all camera/control ports')
    return {'mode': mode, 'processes_verified': True,
            'data_path_verified': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'begin', 'stop', 'prepare', 'start',
                                         'verify', 'commit', 'fail', 'status', 'serve'))
    parser.add_argument('mode', nargs='?', choices=MODES)
    args = parser.parse_args()
    if os.geteuid() != 0 and args.action != 'status':
        parser.error('root privileges required')
    config = json.loads(CONFIG.read_text())
    mode = args.mode
    if args.action not in ('status', 'serve', 'stop', 'fail') and mode is None:
        parser.error('mode required')
    if args.action == 'check':
        result = check(config, mode)
    elif args.action == 'begin':
        previous = read_state().get('mode')
        save(STATE, {'status': 'applying', 'mode': previous, 'requested': mode})
        result = {'applying': mode}
    elif args.action == 'stop':
        stop(config); result = {'stopped': True}
    elif args.action == 'prepare':
        check(config, mode)
        previous = read_state().get('mode')
        save(STATE, {'status': 'applying', 'mode': previous, 'requested': mode})
        if config['role'] == 'zcu':
            root = Path(config['workspace'])
            for name in ('zcu', 'zcu_health_node'):
                run('install', '-m', '0755', str(root / f'build-{mode}' / name),
                    str(root / 'build' / name))
        RUNTIME.write_text(mode + '\n')
        result = {'prepared': mode}
    elif args.action == 'start':
        run('systemctl', 'start', *reversed(units(config)))
        result = {'started': mode}
    elif args.action == 'verify':
        result = verify(config, mode)
    elif args.action == 'commit':
        verify(config, mode)
        run('systemctl', 'enable', *units(config))
        save(STATE, {'status': 'ready', 'mode': mode, 'data_path_verified': False})
        if config['role'] == 'zcu':
            (Path(config['workspace']) / '.middleware').write_text(mode + '\n')
        result = read_state()
    elif args.action == 'fail':
        state = read_state(); state['status'] = 'failed'; save(STATE, state)
        RUNTIME.unlink(missing_ok=True)
        stop(config); result = read_state()
    elif args.action == 'serve':
        serve(config); result = {'restored': True}
    else:
        result = read_state()
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr, flush=True)
        sys.exit(1)
