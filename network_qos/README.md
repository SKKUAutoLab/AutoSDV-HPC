# AutoSDV DSCP policy (HPC side)

Traffic is classified per role, in both directions:

| Role | Includes | DSCP / TOS | SJA1110 |
| --- | --- | --- | --- |
| control | MotionCommand and the control node's/session's discovery and upkeep | CS6 / `0xc0` | Q6 |
| camera | images and the camera nodes'/sessions' discovery and upkeep | CS5 / `0xa0` | Q5 |
| health and everything else | ZCU health, diagnostics ping, SSH, OTA, ... | CS0 / `0x00` | Q0 |

See [the SJA1110 mapping](../docs/sja1110-stream-identification-tc-mapping.md)
for ports per middleware.

## How the HPC marks it

- **Cyclone DDS:** per process. The GUI starts the nodes that own a class with
  `LD_PRELOAD=install/launch_pkg/lib/libautosdv_dscp_shim.so` and
  `AUTOSDV_IP_TOS` (`traffic_class` in `autosdv_gui_pkg/config/nodes.yaml`:
  `safety_supervisor` is `control`, the five camera nodes are `camera`). The shim
  sets `IP_TOS` on every UDP socket the process creates, so discovery and data
  share the class from the first packet. `autosdv_dscp_qos.py` installs no rule.
- **Zenoh, Zenoh bridge, Pico:** the router or bridges carry all IVN traffic,
  one HPC port per flow, so `autosdv_dscp_qos.py` marks their egress by source
  port: `7448` CS6, `17451-17459` CS5, `7449` (health) no rule.

There is no catch-all rule: rewriting every packet would erase the shim's marks.
The ZCUs mark their own egress per process with the same shim.

## Inspect

```bash
python3 network_qos/autosdv_dscp_qos.py render --middleware zenoh
sudo nft list table ip autosdv_dscp
```

The communication mode service (`autosdv-communication.service`) runs
`autosdv_dscp_qos.py watch` for the selected mode and removes the table on stop;
the standalone `autosdv-dscp-qos.service` is retired by the mode switch.

For a wire-level check, capture on the HPC (TX offload can make host-side
captures misleading; an SJA1110 mirror port is authoritative):

```bash
sudo tcpdump -i enP2p1s0 -nn -v 'ip and host 10.0.0.2'
```

Expect `tos 0xc0` for control, `tos 0xa0` for camera and `tos 0x0` for health.

## Remove

```bash
sudo network_qos/uninstall.sh
```

Stopping the service removes only the owned `ip autosdv_dscp` nftables table.
