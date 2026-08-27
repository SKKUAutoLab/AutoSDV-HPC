# AutoSDV initial DSCP policy

This configuration is intentionally limited to the first wire-level test:

- IPv4 packets leaving `enp4s0`: CS1 (`DSCP 8`, TOS `0x20`, SJA1110 Q1)
- `/topic_control_signal` RTPS user data from `safety_supervisor_node` to
  `10.0.0.2` through `10.0.0.5`: CS6 (`DSCP 48`, TOS `0xC0`, SJA1110 Q6)
- DDS discovery and all other IPv4 traffic: CS1/Q1

The nftables rule modifies only the DSCP bits, preserving ECN.

## Why a small watcher is needed

Fast DDS uses a dynamic UDP send port. The watcher identifies the actual
installed `safety_supervisor_node` process, follows its UDP socket inodes in
`/proc`, and updates an nftables source-port set.

The default DDSI-RTPS mapping assigns discovery/metatraffic to even unicast
ports and user data to odd unicast ports. Only the calculated user-data ports
can receive the CS6 override. The repository currently uses the default port
mapping and ROS domain 0.

The practical topic-level assumption is that `/topic_control_signal` is the
only safety-supervisor user-data topic with matching readers on the ZCUs.
If a ZCU later subscribes to `/safety/status` or another topic from this same
DDS participant, that traffic will also use Q6. Exact arbitrary ROS-topic
classification would require a dedicated egress DDS participant.

## Inspect without changing the host

```bash
python3 network_qos/autosdv_dscp_qos.py render
python3 network_qos/autosdv_dscp_qos.py ports
```

## Install

The installer changes the host nftables state and therefore requires root:

```bash
sudo network_qos/install.sh
```

The service itself runs as the invoking `sudo` user (normally `ecu`) with
only `CAP_NET_ADMIN`. Running it as root with only that capability prevents
procfs from exposing the GUI user's socket descriptors and leaves the Q6
source-port set empty.

Check the live counters and tracked source ports:

```bash
sudo /usr/local/libexec/autosdv-dscp-qos status
/usr/local/libexec/autosdv-dscp-qos ports
```

The `AutoSDV control Q6` counter should increase while the safety supervisor
publishes `/topic_control_signal`. The default-Q1 counter includes discovery.

For a wire-level check, use an external TAP or SJA1110 port mirror when
possible. On the HPC:

```bash
sudo tcpdump -i enp4s0 -nn -vv 'ip and host 10.0.0.2'
```

Tcpdump prints `tos 0xc0` for Q6 and `tos 0x20` for Q1. TX offload can make
host-side captures misleading, so the external capture is authoritative.

## Remove

```bash
sudo network_qos/uninstall.sh
```

Stopping the service removes only the owned `ip autosdv_dscp` nftables table.
