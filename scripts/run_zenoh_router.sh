#!/bin/bash
# Starts the Zenoh router for the wired IVN.
#
# rmw_zenoh disables multicast discovery, so with AUTOSDV_RMW=zenoh nothing
# finds anything until this is running. Each camera has its own UDP endpoint:
#   17451/17453/17455/17457/17459  image_01..05
#   7447  legacy/local clients
#   7448  control
#   7449  health
# See AutoSDV-HPC/config/zenoh/zenoh_router_ivn.json5 for the full config.
set -euo pipefail

# The ROS setup scripts read AMENT_TRACE_SETUP_FILES unguarded, which aborts
# under `set -u`, so nounset is lifted only across the sourcing.
set +u
source /opt/ros/jazzy/setup.bash
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WORKSPACE/install/local_setup.bash" ]; then
    source "$WORKSPACE/install/local_setup.bash"
fi
set -u

CONFIG="${ZENOH_ROUTER_CONFIG_URI:-$WORKSPACE/config/zenoh/zenoh_router_ivn.json5}"
if [ ! -f "$CONFIG" ]; then
    echo "router config not found: $CONFIG" >&2
    exit 1
fi

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI="$CONFIG"
echo "starting zenoh router with $CONFIG"
exec ros2 run rmw_zenoh_cpp rmw_zenohd
