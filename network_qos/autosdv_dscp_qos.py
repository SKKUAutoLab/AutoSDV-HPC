#!/usr/bin/env python3
"""Apply the AutoSDV per-flow DSCP policy on the wired IVN (HPC side).

Policy (the SJA1110 classifies by DSCP into its queues):
  * control  - CS6 (TOS 0xc0, Q6): MotionCommand and the control session's
               discovery, subscription and keep-alive traffic
  * camera   - CS5 (TOS 0xa0, Q5): images and the camera sessions' upkeep
  * health and everything else - CS0 (Q0); no rule, the kernel default

There is deliberately no catch-all rule. Per-process marking (below) sets the
TOS on the socket, and a catch-all rewrite would erase it.

How each middleware is classified:

cyclonedds -- per process. The HPC nodes that own a flow are started with
  LD_PRELOAD=libautosdv_dscp_shim.so and AUTOSDV_IP_TOS (see the GUI node
  specs: safety_supervisor 0xc0, image nodes 0xa0). The shim sets IP_TOS on
  every UDP socket the process creates, so discovery and data of that node
  share its class from the first packet. This module installs no rule.

zenoh, zenoh_bridge -- per HPC port. Every ZCU flow is a separate session to
  a separate HPC port of the router (zenoh) or of a dedicated bridge process
  (zenoh_bridge), so the router's/bridge's egress is classified by source port:
      7448                      control  CS6
      17451..17459 (odd)        cameras  CS5
      7449                      health   CS0 (no rule)
  zenoh_bridge_pico uses the same HPC bridges and takes the zenoh_bridge rule.

The ZCU side marks its own egress per process with the same shim.
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import re
import signal
import subprocess
import sys
import time
from typing import Iterable


TABLE_FAMILY = "ip"
TABLE_NAME = "autosdv_dscp"
ZCU_SET = "zcu_addresses"

DEFAULT_INTERFACE = "enP2p1s0"
DEFAULT_ZCU_ADDRESSES = (
    "10.0.0.2",
    "10.0.0.3",
    "10.0.0.4",
    "10.0.0.5",
)

FASTDDS = "fastdds"
CYCLONEDDS = "cyclonedds"
ZENOH = "zenoh"
ZENOH_BRIDGE = "zenoh_bridge"
MIDDLEWARES = (FASTDDS, CYCLONEDDS, ZENOH, ZENOH_BRIDGE)

# Middlewares whose flows are identified by a fixed HPC port.
PORT_CLASSIFIED = (ZENOH, ZENOH_BRIDGE)

# HPC ports from config/zenoh/zenoh_router_ivn.json5 and the bridge configs.
ZENOH_CONTROL_PORT = 7448
ZENOH_HEALTH_PORT = 7449
ZENOH_CAMERA_PORTS = (17451, 17453, 17455, 17457, 17459)

_INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.:-]+$")


def validate_interface(interface: str) -> str:
    if not _INTERFACE_RE.fullmatch(interface):
        raise ValueError(f"invalid interface name: {interface!r}")
    return interface


def validate_zcu_addresses(addresses: Iterable[str]) -> tuple[str, ...]:
    validated = tuple(str(ipaddress.IPv4Address(address)) for address in addresses)
    if not validated:
        raise ValueError("at least one ZCU IPv4 address is required")
    return validated


def _nft_elements(values: Iterable[object]) -> str:
    return ", ".join(str(value) for value in values)


def render_ruleset(
    interface: str,
    zcu_addresses: Iterable[str],
    middleware: str = CYCLONEDDS,
) -> str:
    """Render the owned nftables table.

    `ip dscp set` changes only the six DSCP bits and preserves ECN.
    """
    interface = validate_interface(interface)
    zcus = validate_zcu_addresses(zcu_addresses)
    if middleware not in MIDDLEWARES:
        raise ValueError(f"unknown middleware: {middleware!r}")

    rules = ""
    if middleware in PORT_CLASSIFIED:
        match = f'oifname "{interface}" ip daddr @{ZCU_SET} meta l4proto {{ tcp, udp }}'
        rules = (
            f"        {match} th sport {ZENOH_CONTROL_PORT} ip dscp set cs6 counter "
            f'comment "AutoSDV control CS6 (port {ZENOH_CONTROL_PORT})"\n'
            f"        {match} th sport {{ {_nft_elements(ZENOH_CAMERA_PORTS)} }} "
            f'ip dscp set cs5 counter comment "AutoSDV camera CS5"\n'
        )

    return f"""table {TABLE_FAMILY} {TABLE_NAME} {{
    set {ZCU_SET} {{
        type ipv4_addr
        elements = {{ {_nft_elements(zcus)} }}
    }}

    chain output {{
        type route hook output priority mangle; policy accept;
{rules}    }}
}}
"""


def describe(middleware: str) -> str:
    if middleware in PORT_CLASSIFIED:
        return (f"control=CS6 (sport {ZENOH_CONTROL_PORT}), camera=CS5 "
                f"(sport {ZENOH_CAMERA_PORTS[0]}-{ZENOH_CAMERA_PORTS[-1]}), "
                f"health/other=CS0")
    return "per-process via libautosdv_dscp_shim.so; no nftables rule"


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
    ruleset = render_ruleset(args.interface, args.zcu, args.middleware)
    delete_owned_table(args.nft)
    run_nft(args.nft, ruleset)


def watch(args: argparse.Namespace) -> int:
    """Apply once and stay alive, so a supervisor can remove the table on stop."""
    apply_rules(args)
    print(f"AutoSDV DSCP policy active on {args.interface}: {describe(args.middleware)}",
          flush=True)

    stopping = False

    def request_stop(signum: int, frame: object) -> None:
        del signum, frame
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while not stopping:
            time.sleep(0.5)
    finally:
        if args.cleanup_on_exit:
            delete_owned_table(args.nft)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("render", "once", "watch", "remove", "status"))
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument(
        "--middleware",
        choices=MIDDLEWARES,
        default=os.environ.get("AUTOSDV_RMW", CYCLONEDDS),
        help="selects how flows are identified; see the module docstring",
    )
    parser.add_argument("--zcu", action="append", default=None)
    parser.add_argument("--nft", default="/usr/sbin/nft")
    parser.add_argument("--cleanup-on-exit", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.zcu is None:
        args.zcu = list(DEFAULT_ZCU_ADDRESSES)

    try:
        if args.command == "render":
            print(render_ruleset(args.interface, args.zcu, args.middleware), end="")
        elif args.command == "once":
            apply_rules(args)
            print(describe(args.middleware))
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
