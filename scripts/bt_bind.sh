#!/usr/bin/env bash
# Linux: bind a paired e-puck to /dev/rfcommN (see docs/bluetooth.md).
# Usage: scripts/bt_bind.sh <MAC> [N=0] [channel=1]
set -euo pipefail
MAC="${1:?usage: $0 <MAC> [N] [channel]}"; N="${2:-0}"; CH="${3:-1}"
sudo rfcomm release "$N" 2>/dev/null || true
sudo rfcomm bind "$N" "$MAC" "$CH"
ls -l "/dev/rfcomm$N"
