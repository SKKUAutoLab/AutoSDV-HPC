#!/usr/bin/env python3
"""Install host mode adapter; does not start/stop vehicle services."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('role', choices=('hpc', 'zcu'))
p.add_argument('--workspace', type=Path, required=True)
a = p.parse_args()
if os.geteuid() != 0:
    p.error('run with sudo (root on ZCU)')
root = a.workspace.resolve()
if not (root / 'src').is_dir():
    p.error('workspace must contain src/')
helper = Path('/usr/local/libexec/autosdv-communication-host')
helper.parent.mkdir(parents=True, exist_ok=True)
shutil.copyfile(Path(__file__).with_name('communication_host.py'), helper)
helper.chmod(0o755)
config = {'role': a.role, 'workspace': str(root)}
if a.role == 'hpc':
    config['node_executables'] = [str(f) for f in (root / 'install').glob('*/lib/*/*')
                                  if f.is_file() and os.access(f, os.X_OK)
                                  and f.name != 'autosdv_gui']
Path('/etc/autosdv').mkdir(exist_ok=True)
Path('/var/lib/autosdv').mkdir(parents=True, exist_ok=True)
lock = Path('/var/lib/autosdv/communication-switch.lock')
lock.touch(exist_ok=True)
lock.chmod(0o644)
Path('/etc/autosdv/communication-host.json').write_text(json.dumps(config) + '\n')
service = '''[Unit]
Description=AutoSDV selected communication mode
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/local/libexec/autosdv-communication-host serve
TimeoutStopSec=15
KillMode=control-group
'''
service += ('Type=oneshot\nRemainAfterExit=yes\n' if a.role == 'zcu' else
            'Type=simple\nRestart=on-failure\nRestartSec=2\n')
service += '\n[Install]\nWantedBy=multi-user.target\n'
Path('/etc/systemd/system/autosdv-communication.service').write_text(service)
if a.role == 'zcu':
    for unit in ('zcu.service', 'zcu-health@.service'):
        d = Path('/etc/systemd/system') / (unit + '.d')
        d.mkdir(exist_ok=True)
        (d / 'communication-mode.conf').write_text(
            '[Unit]\nRequires=autosdv-communication.service\n'
            'After=autosdv-communication.service\n')
subprocess.run(['systemctl', 'daemon-reload'], check=True)
print('Installed. Run preflight/apply from HPC. Services are enabled on successful commit; no services started.')
