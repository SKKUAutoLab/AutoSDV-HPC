"""Common CLI orchestration. No ROS or Qt imports; usable before RMW initialization."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

MODES = ('cyclonedds', 'zenoh', 'zenoh_bridge', 'zenoh_bridge_pico')
LABELS = ('CycloneDDS', 'Zenoh', 'Zenoh–DDS Bridge', 'Zenoh–DDS Bridge + Pico')
HOSTS = ('HPC', '10.0.0.2', '10.0.0.3', '10.0.0.4', '10.0.0.5')
HELPER = '/usr/local/libexec/autosdv-communication-host'
STATE = Path('/var/lib/autosdv/communication-mode.json')


def state():
    if not STATE.exists():
        return {}
    return json.loads(STATE.read_text())


def selected_mode():
    saved = state()
    if saved and saved.get('status') != 'ready':
        raise RuntimeError('Communication switch incomplete. Open mode setup to recover.')
    return saved.get('mode', 'fastdds')


def call_host(host, action, mode):
    command = [HELPER, action, mode]
    if host == 'HPC' and action == 'daemon':
        command = ['bash', '-c', 'source /opt/ros/jazzy/setup.bash; ros2 daemon stop']
    elif host == 'HPC':
        command = ['sudo', '-n', *command]
    else:
        command = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
                   '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
                   f'root@{host}', shlex.join(command)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=100)
    if result.returncode:
        raise RuntimeError((result.stderr or result.stdout).strip() or
                           f'exit {result.returncode}')
    return {'daemon_stopped': True} if action == 'daemon' else json.loads(result.stdout)


class Coordinator:
    def __init__(self, call=call_host, report=print):
        self.call = call
        self.report = report

    def phase(self, action, mode, hosts=HOSTS):
        errors = []
        with ThreadPoolExecutor(max_workers=5) as pool:
            pending = {pool.submit(self.call, host, action, mode): host for host in hosts}
            for future in as_completed(pending):
                host = pending[future]
                try:
                    result = future.result()
                    self.report(f'{host}: {action} OK — {json.dumps(result)}')
                except Exception as exc:
                    errors.append(f'{host}: {action}: {exc}')
                    self.report(errors[-1])
        if errors:
            raise RuntimeError('\n'.join(errors))

    def apply(self, mode):
        self.phase('check', mode)  # All hosts pass before any mutation.
        try:
            self.phase('begin', mode)
            self.phase('stop', mode)
            self.phase('daemon', mode, ('HPC',))
            self.phase('prepare', mode)
            self.phase('start', mode, ('HPC',))
            self.wait_ready(mode, ('HPC',))
            self.phase('start', mode, HOSTS[1:])
            self.wait_ready(mode, HOSTS[1:])
            # Commit HPC last: it is the overall success marker read by the GUI.
            self.phase('commit', mode, HOSTS[1:])
            self.phase('commit', mode, ('HPC',))
        except BaseException:
            try:
                self.phase('fail', mode)
            except Exception as exc:
                self.report(f'Cleanup incomplete; manual recovery required: {exc}')
            raise
        self.report('Mode applied. Processes verified; live image/control/health validation pending.')

    def wait_ready(self, mode, hosts):
        deadline = time.monotonic() + 20
        while True:
            try:
                self.phase('verify', mode, hosts)
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(1)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=('check', 'apply', 'status'))
    p.add_argument('mode', nargs='?', choices=MODES)
    p.add_argument('--gui-owner', type=int, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.action == 'status':
        print(json.dumps(state(), indent=2)); return 0
    if args.mode is None:
        p.error('mode required')
    gui_locks = Path('/tmp').glob('autosdv_hpc_gui_*.lock') if args.action == 'apply' else ()
    for gui_lock in gui_locks:
        try:
            pid = int(gui_lock.read_text().splitlines()[0])
        except (OSError, ValueError, IndexError):
            pid = -1
        if Path(f'/proc/{pid}').exists() and not (pid == args.gui_owner == os.getppid()):
            print('GUI is running. Switch from its Communication mode panel.', file=sys.stderr)
            return 1
    lock_path = Path('/var/lib/autosdv/communication-switch.lock')
    if args.action == 'check':
        try:
            Coordinator(report=lambda msg: print(msg, flush=True)).phase('check', args.mode)
            return 0
        except Exception as exc:
            print(str(exc), file=sys.stderr); return 1
    try:
        lock = lock_path.open('r')
    except OSError:
        print('Install the HPC communication host adapter first.', file=sys.stderr)
        return 1
    with lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Another mode operation is running.', file=sys.stderr); return 1
        coordinator = Coordinator(report=lambda msg: print(msg, flush=True))
        try:
            coordinator.apply(args.mode)
        except Exception as exc:
            print(str(exc), file=sys.stderr); return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
