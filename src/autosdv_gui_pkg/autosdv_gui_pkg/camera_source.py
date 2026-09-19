"""Camera input control over the existing SSH jump hosts (no ROS dependency)."""
import argparse
import json
import os
import shlex
import subprocess

# camera_id: (label, ZCU, Pi address, host-key alias, local control port)
CAMERAS = {
    'camera_01': ('Image 01 · ZCU1 webcam', '10.0.0.2', '12.0.0.5', 'rpi-zcu1-webcam', 8586),
    **{f'camera_{n+1:02d}': (f'Image {n+1:02d} · ZCU{n}', f'10.0.0.{n+1}',
                            '11.0.0.5', f'rpi-zcu{n}', 8585) for n in range(1, 5)},
}
DEFAULT_VIDEO = '/home/pi/camera/driving_simulation.mp4'


def request(camera_id, action='status', path='', runner=subprocess.run):
    if action not in ('status', 'camera', 'video'):
        raise ValueError('Unknown camera action')
    if action == 'video' and (not path.startswith('/') or any(c in path for c in '\n\r\0')):
        raise ValueError('Enter an absolute video path on the Raspberry Pi')
    _, jump, host, alias, port = CAMERAS[camera_id]
    remote = ['python3', '/home/pi/camera/camera_control.py', '127.0.0.1',
              str(port), '--timeout', '15', action]
    if action == 'video':
        remote.append(path)
    result = runner([
        'ssh', '-i', os.path.expanduser('~/.ssh/id_ed25519_rpi'),
        '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
        '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=5',
        '-o', 'ServerAliveCountMax=2', '-o', f'ProxyJump=root@{jump}',
        '-o', f'HostKeyAlias={alias}', f'pi@{host}', shlex.join(remote),
    ], capture_output=True, text=True, timeout=25)
    try:
        reply = json.loads(result.stdout)
    except (ValueError, TypeError) as exc:
        raise RuntimeError((result.stderr or result.stdout or 'No camera reply').strip()) from exc
    if (not isinstance(reply, dict) or type(reply.get('ok')) is not bool
            or reply.get('source') not in (None, 'camera', 'video')):
        raise RuntimeError('Invalid camera reply')
    if result.returncode and reply['ok']:
        raise RuntimeError(result.stderr.strip() or 'SSH command failed')
    return reply


def operate(camera_id, action='status', path=''):
    """Reconcile actual state even after a rejected or timed-out switch."""
    try:
        reply = request(camera_id, action, path)
        error = '' if reply['ok'] else reply.get('error', 'Source change rejected')
    except Exception as exc:
        reply, error = {}, str(exc)
    if action != 'status':
        try:
            actual = request(camera_id)
            if not actual['ok']:
                raise RuntimeError(actual.get('error', 'Status query failed'))
            reply = actual
            if not error and (actual['source'] != action or
                              (action == 'video' and actual.get('video_path') != path)):
                error = 'Requested source was not confirmed'
        except Exception as exc:
            reply = {}
            error = f'{error + "; " if error else ""}Could not confirm current source: {exc}'
    return {'camera_id': camera_id, 'state': reply, 'error': error}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera_id', choices=CAMERAS)
    parser.add_argument('action', choices=('status', 'camera', 'video'))
    parser.add_argument('path', nargs='?', default='')
    args = parser.parse_args()
    result = operate(args.camera_id, args.action, args.path)
    print(json.dumps(result))
    return 1 if result['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
