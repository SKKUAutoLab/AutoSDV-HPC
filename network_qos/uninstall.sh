#!/bin/sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "Run this uninstaller with sudo." >&2
    exit 1
fi

systemctl disable --now autosdv-dscp-qos.service 2>/dev/null || true
/usr/local/libexec/autosdv-dscp-qos remove 2>/dev/null || true
rm -f /etc/systemd/system/autosdv-dscp-qos.service
rm -f /usr/local/libexec/autosdv-dscp-qos
systemctl daemon-reload
