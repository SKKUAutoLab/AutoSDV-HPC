#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

AUTOSDV_USER=${SUDO_USER:-}
if [ -z "$AUTOSDV_USER" ] || [ "$AUTOSDV_USER" = root ]; then
    AUTOSDV_USER=$(stat -c %U "$SCRIPT_DIR")
fi
if ! id "$AUTOSDV_USER" >/dev/null 2>&1; then
    echo "Cannot resolve AutoSDV runtime user: $AUTOSDV_USER" >&2
    exit 1
fi

install -D -m 0755 \
    "$SCRIPT_DIR/autosdv_dscp_qos.py" \
    /usr/local/libexec/autosdv-dscp-qos
sed "s/@AUTOSDV_USER@/$AUTOSDV_USER/g" \
    "$SCRIPT_DIR/autosdv-dscp-qos.service" \
    | install -D -m 0644 /dev/stdin \
        /etc/systemd/system/autosdv-dscp-qos.service

systemctl daemon-reload
systemctl enable autosdv-dscp-qos.service
systemctl reset-failed autosdv-dscp-qos.service 2>/dev/null || true
systemctl restart autosdv-dscp-qos.service
systemctl --no-pager --full status autosdv-dscp-qos.service
