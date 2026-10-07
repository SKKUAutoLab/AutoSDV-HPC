#!/bin/bash
# Starts the HPC-side zenoh-DDS bridges for AUTOSDV_RMW=zenoh_bridge.
#
# With this environment the HPC nodes are ordinary Cyclone DDS nodes confined
# to the loopback interface, and these bridges are the only processes on the
# host that touch the wired IVN.  Nothing reaches a ZCU until they are running,
# the same way the native zenoh environment needs scripts/run_zenoh_router.sh.
#
# One camera bridge serves five camera sessions, each with its own 5-tuple:
#   camera   17451/17453/17455/17457/17459   /image_01_raw .. /image_05_raw
#   control  7448   /topic_control_signal, /topic_status_signal, /ivn/zcu_health
#
# usage: run_zenoh_bridge.sh [camera|control|all]      (default: all)
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FLOW="${1:-all}"

BINARY="${ZENOH_BRIDGE_ROS2DDS_BIN:-$WORKSPACE/tools/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds}"
if [ ! -x "$BINARY" ]; then
    BINARY="$(command -v zenoh-bridge-ros2dds || true)"
fi
if [ -z "$BINARY" ] || [ ! -x "$BINARY" ]; then
    echo "zenoh-bridge-ros2dds not found; run scripts/install_zenoh_bridge.sh" >&2
    exit 1
fi

# The bridge reads ROS_DISTRO to pick the ros_discovery_info layout, which is
# how it learns which node owns which DDS endpoint. Left unset it assumes iron
# and logs a warning; on jazzy that silently costs the type hash field, so set
# it here rather than requiring the operator to have sourced ROS.
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# The bridge embeds Cyclone DDS, so it reads CYCLONEDDS_URI like any node.
# Pointing it at the same loopback profile the nodes use is what keeps DDS off
# enP2p1s0 -- without it the bridge would pick an interface automatically,
# reach the ZCUs as plain RTPS, and the measurement would not be of a bridge.
DDS_CONFIG="file://$WORKSPACE/config/dds/cyclonedds_bridge_local.xml"
export CYCLONEDDS_URI="$DDS_CONFIG"

bridge_config() {
    local config="$WORKSPACE/config/zenoh/zenoh_bridge_${1}_ivn.json5"
    if [ ! -f "$config" ]; then
        echo "bridge config not found: $config" >&2
        exit 1
    fi
    printf '%s' "$config"
}

case "$FLOW" in
    camera|control)
        config="$(bridge_config "$FLOW")"
        echo "starting zenoh-bridge-ros2dds ($FLOW) with $config"
        # exec, so a Ctrl-C or a systemd stop reaches the bridge itself instead
        # of killing this wrapper and orphaning it.
        exec "$BINARY" -c "$config"
        ;;
    all)
        # Both bridges share this shell's lifetime: stopping it takes the IVN
        # down as a whole rather than leaving one flow half-connected.
        pids=()
        trap 'kill "${pids[@]}" 2>/dev/null || true' EXIT INT TERM
        for flow in camera control; do
            config="$(bridge_config "$flow")"
            echo "starting zenoh-bridge-ros2dds ($flow) with $config"
            "$BINARY" -c "$config" &
            pids+=("$!")
        done
        wait -n
        echo "a bridge exited; stopping the other" >&2
        exit 1
        ;;
    *)
        echo "usage: $0 [camera|control|all]" >&2
        exit 1
        ;;
esac
