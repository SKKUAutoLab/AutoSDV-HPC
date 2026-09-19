#!/bin/bash
# Fetches the zenoh-bridge-ros2dds binary used by AUTOSDV_RMW=zenoh_bridge.
#
# The bridge is not part of the ROS apt repositories, so it is pulled from the
# Eclipse zenoh release assets and kept under tools/ inside the workspace
# rather than installed system-wide.  The version is pinned: the bridges on
# the HPC and on the four ZCUs have to speak the same zenoh protocol, and a
# floating "latest" would silently break that on the next release.
#
# Only the HPC runs a bridge: the ZCUs are plain zenoh clients and need nothing
# from this script.  The aarch64 glibc build needs at most GLIBC_2.30, well
# under the host's 2.39.
#
# usage: install_zenoh_bridge.sh
set -euo pipefail

VERSION="1.10.0"
ASSET="zenoh-plugin-ros2dds-${VERSION}-aarch64-unknown-linux-gnu-standalone.zip"
URL="https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${VERSION}/${ASSET}"

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="$WORKSPACE/tools/zenoh-bridge-ros2dds"
BINARY="$PREFIX/zenoh-bridge-ros2dds"

if [ $# -gt 0 ]; then
    echo "usage: $0" >&2
    exit 1
fi

if [ ! -x "$BINARY" ]; then
    mkdir -p "$PREFIX"
    WORK="$(mktemp -d)"
    trap 'rm -rf "$WORK"' EXIT
    echo "downloading $ASSET"
    curl -fL --retry 3 -o "$WORK/bridge.zip" "$URL"
    unzip -o -q "$WORK/bridge.zip" -d "$PREFIX"
    chmod +x "$BINARY"
fi

# The binary logs its version to stderr on every start, including this one.
"$BINARY" --version
