#!/usr/bin/env python3
"""Apply the initial AutoSDV IPv4 DSCP policy on the wired IVN.

Policy:
  * every IPv4 packet leaving the wired IVN uses CS1 (SJA1110 Q1)
  * RTPS user-data packets sent by the actual safety_supervisor process to a
    ZCU use CS6 (SJA1110 Q6)

Fast DDS uses an ephemeral UDP send socket, so the Q6 rule cannot rely on a
fixed source port.  This daemon follows the safety supervisor's UDP socket
inodes through procfs and keeps an nftables source-port set up to date.

With the default RTPS port mapping, metatraffic/discovery unicast ports are
even and user-data unicast ports are odd.  The Q6 rule contains only the
calculated user-data ports; discovery therefore remains CS1.  Cyclone DDS uses
the same mapping, so --middleware cyclonedds needs no other change.

Zenoh is different in kind.  Every HPC node reaches a ZCU through the zenohd
router, so nothing a ZCU receives originates from the safety supervisor's own
socket and the procfs tracking has nothing to attach to.  What is left to match
on is the router's listening port, which is per flow rather than per node:
7447 carries camera images and 7448 carries control together with health.
--middleware zenoh therefore marks CS6 on egress from the control port, which
is wider than the Fast DDS rule -- it covers health diagnostics as well.  Give
the safety supervisor its own router port if the experiment needs them apart.
"""

from __future__ import annotations

import argparse
import ipaddress
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Iterable


TABLE_FAMILY = "ip"
TABLE_NAME = "autosdv_dscp"
CONTROL_SPORT_SET = "control_udp_sports"
RTPS_USER_DPORT_SET = "rtps_user_dports"
ZCU_SET = "zcu_addresses"

DEFAULT_INTERFACE = "enp4s0"
DEFAULT_ZCU_ADDRESSES = (
    "10.0.0.2",
    "10.0.0.3",
    "10.0.0.4",
    "10.0.0.5",
)
DEFAULT_POLL_INTERVAL = 0.25
DEFAULT_MAX_PARTICIPANT_ID = 119

FASTDDS = "fastdds"
CYCLONEDDS = "cyclonedds"
ZENOH = "zenoh"
MIDDLEWARES = (FASTDDS, CYCLONEDDS, ZENOH)

# Router ports from AutoSDV-HPC/zenoh_router_ivn.json5.
ZENOH_CAMERA_PORT = 7447
ZENOH_CONTROL_PORT = 7448

_INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.:-]+$")
_SOCKET_INODE_RE = re.compile(r"^socket:\[(\d+)\]$")


def validate_interface(interface: str) -> str:
    if not _INTERFACE_RE.fullmatch(interface):
        raise ValueError(f"invalid interface name: {interface!r}")
    return interface


def validate_zcu_addresses(addresses: Iterable[str]) -> tuple[str, ...]:
    validated = tuple(str(ipaddress.IPv4Address(address)) for address in addresses)
    if not validated:
        raise ValueError("at least one ZCU IPv4 address is required")
    return validated


def rtps_user_unicast_ports(
    domain_id: int,
    max_participant_id: int,
) -> tuple[int, ...]:
    """Return default DDSI-RTPS user-unicast ports for one ROS domain.

    The default mapping is PB + DG * domain + d3 + PG * participant, where
    PB=7400, DG=250, d3=11 and PG=2.
    """
    if not 0 <= domain_id <= 232:
        raise ValueError("ROS domain ID must be in the range 0..232")
    if not 0 <= max_participant_id <= 119:
        raise ValueError("maximum participant ID must be in the range 0..119")
    base = 7400 + 250 * domain_id + 11
    ports = tuple(base + 2 * participant for participant in range(max_participant_id + 1))
    if ports[-1] > 65535:
        raise ValueError("calculated RTPS port exceeds 65535")
    return ports


def _nft_elements(values: Iterable[object]) -> str:
    return ", ".join(str(value) for value in values)


def render_ruleset(
    interface: str,
    zcu_addresses: Iterable[str],
    domain_id: int,
    max_participant_id: int,
    middleware: str = FASTDDS,
) -> str:
    """Render the owned nftables table.

    Rule order is intentional: CS1 is the baseline and the second rule
    overwrites only the control traffic with CS6.  `ip dscp set` changes only
    the six DSCP bits and preserves ECN.

    For the RTPS middlewares the second rule is the intersection of the safety
    supervisor's live source ports (kept current by this daemon) and the
    calculated RTPS user-data destination ports.  For zenoh neither set exists,
    so it matches the router's control port instead; see the module docstring
    for why that is wider.
    """
    interface = validate_interface(interface)
    zcus = validate_zcu_addresses(zcu_addresses)
    if middleware not in MIDDLEWARES:
        raise ValueError(f"unknown middleware: {middleware!r}")

    if middleware == ZENOH:
        control_rule = (
            f'oifname "{interface}" ip daddr @{ZCU_SET} meta l4proto {{ tcp, udp }} '
            f"th sport {ZENOH_CONTROL_PORT} ip dscp set cs6 counter "
            f'comment "AutoSDV control Q6 (zenoh router control port)"'
        )
        sets = f"""    set {ZCU_SET} {{
        type ipv4_addr
        elements = {{ {_nft_elements(zcus)} }}
    }}
"""
    else:
        user_ports = rtps_user_unicast_ports(domain_id, max_participant_id)
        control_rule = (
            f'oifname "{interface}" ip daddr @{ZCU_SET} meta l4proto udp '
            f"udp sport @{CONTROL_SPORT_SET} udp dport @{RTPS_USER_DPORT_SET} "
            f'ip dscp set cs6 counter comment "AutoSDV control Q6"'
        )
        sets = f"""    set {ZCU_SET} {{
        type ipv4_addr
        elements = {{ {_nft_elements(zcus)} }}
    }}

    set {CONTROL_SPORT_SET} {{
        type inet_service
    }}

    set {RTPS_USER_DPORT_SET} {{
        type inet_service
        elements = {{ {_nft_elements(user_ports)} }}
    }}
"""

    return f"""table {TABLE_FAMILY} {TABLE_NAME} {{
{sets}
    chain output {{
        type route hook output priority mangle; policy accept;

        oifname \"{interface}\" ip dscp set cs1 counter comment \"AutoSDV default Q1\"
        {control_rule}
    }}
}}
"""


def is_actual_safety_supervisor(arguments: list[str]) -> bool:
    """Match the installed node process, excluding the `ros2 run` wrapper."""
    if not arguments:
        return False
    if any(
        os.path.basename(arguments[index]) == "ros2"
        and arguments[index + 1:index + 4]
        == ["run", "autosdv_gui_pkg", "safety_supervisor_node"]
        for index in range(max(0, len(arguments) - 3))
    ):
        return False
    return any(
        os.path.basename(argument) == "safety_supervisor_node"
        and "autosdv_gui_pkg" in argument
        for argument in arguments
    )


def find_safety_supervisor_pids(proc_root: Path = Path("/proc")) -> tuple[int, ...]:
    matches: list[int] = []
    try:
        entries = tuple(proc_root.iterdir())
    except OSError:
        return ()
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            arguments = [
                item.decode("utf-8", errors="replace")
                for item in (entry / "cmdline").read_bytes().split(b"\0")
                if item
            ]
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        if is_actual_safety_supervisor(arguments):
            matches.append(int(entry.name))
    return tuple(sorted(matches))


def socket_inodes_for_pids(
    pids: Iterable[int],
    proc_root: Path = Path("/proc"),
) -> set[int]:
    inodes: set[int] = set()
    for pid in pids:
        fd_directory = proc_root / str(pid) / "fd"
        try:
            entries = tuple(fd_directory.iterdir())
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        for entry in entries:
            try:
                target = os.readlink(entry)
            except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
                continue
            match = _SOCKET_INODE_RE.fullmatch(target)
            if match:
                inodes.add(int(match.group(1)))
    return inodes


def parse_udp_table(text: str, wanted_inodes: set[int]) -> set[int]:
    """Return local UDP ports whose socket inode belongs to the target process."""
    ports: set[int] = set()
    for line in text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 10:
            continue
        try:
            local_port = int(fields[1].rsplit(":", 1)[1], 16)
            inode = int(fields[9])
        except (IndexError, ValueError):
            continue
        if inode in wanted_inodes:
            ports.add(local_port)
    return ports


def safety_supervisor_udp_ports(proc_root: Path = Path("/proc")) -> tuple[int, ...]:
    pids = find_safety_supervisor_pids(proc_root)
    inodes = socket_inodes_for_pids(pids, proc_root)
    if not inodes:
        return ()
    try:
        udp_table = (proc_root / "net" / "udp").read_text(encoding="ascii")
    except (FileNotFoundError, PermissionError, OSError):
        return ()
    return tuple(sorted(parse_udp_table(udp_table, inodes)))


def render_source_port_update(ports: Iterable[int]) -> str:
    ports = tuple(sorted(set(int(port) for port in ports)))
    for port in ports:
        if not 0 < port <= 65535:
            raise ValueError(f"invalid UDP port: {port}")
    commands = [f"flush set {TABLE_FAMILY} {TABLE_NAME} {CONTROL_SPORT_SET}"]
    if ports:
        commands.append(
            f"add element {TABLE_FAMILY} {TABLE_NAME} {CONTROL_SPORT_SET} "
            f"{{ {_nft_elements(ports)} }}"
        )
    return "\n".join(commands) + "\n"


def run_nft(nft_binary: str, input_text: str) -> None:
    subprocess.run(
        [nft_binary, "-f", "-"],
        input=input_text,
        text=True,
        check=True,
    )


def delete_owned_table(nft_binary: str, check: bool = False) -> None:
    subprocess.run(
        [nft_binary, "delete", "table", TABLE_FAMILY, TABLE_NAME],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=check,
    )


def apply_rules(args: argparse.Namespace) -> None:
    ruleset = render_ruleset(
        args.interface,
        args.zcu,
        args.domain_id,
        args.max_participant_id,
        args.middleware,
    )
    delete_owned_table(args.nft)
    run_nft(args.nft, ruleset)


def update_ports(args: argparse.Namespace) -> tuple[int, ...]:
    ports = safety_supervisor_udp_ports(Path(args.proc_root))
    run_nft(args.nft, render_source_port_update(ports))
    return ports


def watch(args: argparse.Namespace) -> int:
    apply_rules(args)
    if args.middleware == ZENOH:
        # The zenoh rule matches the router's fixed control port, so there is no
        # per-process source-port set to keep current and nothing to poll.
        print(
            f"AutoSDV DSCP policy active on {args.interface}: "
            f"default=CS1/Q1, control=CS6/Q6 (zenoh router port "
            f"{ZENOH_CONTROL_PORT})",
            flush=True,
        )
        signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
        while True:
            time.sleep(args.poll_interval)

    stopping = False

    def request_stop(signum: int, frame: object) -> None:
        del signum, frame
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    previous_ports: tuple[int, ...] | None = None
    print(
        f"AutoSDV DSCP policy active on {args.interface}: "
        "default=CS1/Q1, control=CS6/Q6",
        flush=True,
    )
    try:
        while not stopping:
            ports = safety_supervisor_udp_ports(Path(args.proc_root))
            if ports != previous_ports:
                run_nft(args.nft, render_source_port_update(ports))
                print(f"safety supervisor UDP source ports: {ports or 'none'}", flush=True)
                previous_ports = ports
            time.sleep(args.poll_interval)
    finally:
        if args.cleanup_on_exit:
            delete_owned_table(args.nft)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        choices=("render", "once", "watch", "ports", "remove", "status"),
    )
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument(
        "--middleware",
        choices=MIDDLEWARES,
        default=os.environ.get("AUTOSDV_RMW", FASTDDS),
        help="selects how control traffic is identified; see the module docstring",
    )
    parser.add_argument("--zcu", action="append", default=None)
    parser.add_argument(
        "--domain-id",
        type=int,
        default=int(os.environ.get("ROS_DOMAIN_ID", "0")),
    )
    parser.add_argument(
        "--max-participant-id",
        type=int,
        default=DEFAULT_MAX_PARTICIPANT_ID,
    )
    parser.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL)
    parser.add_argument("--proc-root", default="/proc")
    parser.add_argument("--nft", default="/usr/sbin/nft")
    parser.add_argument("--cleanup-on-exit", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.zcu is None:
        args.zcu = list(DEFAULT_ZCU_ADDRESSES)
    if args.poll_interval <= 0:
        parser.error("--poll-interval must be positive")

    try:
        if args.command == "render":
            print(
                render_ruleset(
                    args.interface,
                    args.zcu,
                    args.domain_id,
                    args.max_participant_id,
                    args.middleware,
                ),
                end="",
            )
        elif args.command == "ports":
            print(" ".join(str(port) for port in safety_supervisor_udp_ports(
                Path(args.proc_root))))
        elif args.command == "once":
            apply_rules(args)
            if args.middleware == ZENOH:
                print(
                    "zenoh: control marked by router port "
                    f"{ZENOH_CONTROL_PORT}; no source-port tracking needed"
                )
            else:
                ports = update_ports(args)
                print(f"active safety supervisor UDP source ports: {ports or 'none'}")
        elif args.command == "watch":
            return watch(args)
        elif args.command == "remove":
            delete_owned_table(args.nft, check=True)
        elif args.command == "status":
            subprocess.run(
                [args.nft, "-a", "list", "table", TABLE_FAMILY, TABLE_NAME],
                check=True,
            )
    except (ValueError, PermissionError, subprocess.CalledProcessError) as exc:
        print(f"autosdv-dscp-qos: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
