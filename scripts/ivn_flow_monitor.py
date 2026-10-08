#!/usr/bin/env python3
"""Log IVN UDP flows per interval, by direction, ZCU, HPC port and TOS.

Used to watch the per-role DSCP policy while the SJA1110 gates are changed:
every interval it prints one line per flow with its packet count, plus the IVN
carrier state, so link flaps, router restarts and blocked classes show up on a
single timeline.

usage: sudo python3 scripts/ivn_flow_monitor.py [--interface enP2p1s0]
       [--interval 5] [--duration 1200] [--log FILE]
"""
import argparse
import collections
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HPC = '10.0.0.10'
LINE = re.compile(r'^\S+ IP \(tos (0x[0-9a-f]+).*?\)\s*$')
ADDR = re.compile(r'^\s*(\d+\.\d+\.\d+\.\d+)\.(\d+) > (\d+\.\d+\.\d+\.\d+)\.(\d+):')


def flow_key(src, sport, dst, dport, tos):
    if src == HPC and dst != HPC:
        return f'HPC->zcu{int(dst.split(".")[3]) - 1} sport {sport} tos {tos}'
    if dst == HPC and src != HPC:
        return f'zcu{int(src.split(".")[3]) - 1}->HPC dport {dport} tos {tos}'
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--interface', default='enP2p1s0')
    parser.add_argument('--interval', type=float, default=5.0)
    parser.add_argument('--duration', type=float, default=1200.0)
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()

    out = open(args.log, 'a', buffering=1) if args.log else sys.stdout
    counts = collections.Counter()
    lock = threading.Lock()
    proc = subprocess.Popen(
        ['tcpdump', '-i', args.interface, '-n', '-v', '-l', 'udp and net 10.0.0.0/24'],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    def reader():
        tos = None
        for line in proc.stdout:
            match = LINE.match(line)
            if match:
                tos = match.group(1)
                continue
            match = ADDR.match(line)
            if match and tos:
                key = flow_key(match[1], match[2], match[3], match[4], tos)
                if key:
                    with lock:
                        counts[key] += 1
                tos = None

    threading.Thread(target=reader, daemon=True).start()
    carrier = Path(f'/sys/class/net/{args.interface}/carrier')
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end and proc.poll() is None:
            time.sleep(args.interval)
            try:
                link = 'up' if carrier.read_text().strip() == '1' else 'DOWN'
            except OSError:
                link = 'DOWN'
            with lock:
                snapshot = sorted(counts.items())
                counts.clear()
            stamp = time.strftime('%H:%M:%S')
            print(f'--- {stamp} link={link} flows={len(snapshot)}', file=out)
            for key, count in snapshot:
                print(f'{stamp} {count:6d} {key}', file=out)
    finally:
        proc.terminate()


if __name__ == '__main__':
    main()
