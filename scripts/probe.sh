#!/usr/bin/env bash
# Link check: firmware version, sensors, round-trip latency.
# Real robot: selector at 3 (asercom), paired, port bound (scripts/bt_bind.sh).
# Usage: scripts/probe.sh <mock|sim|sim-gui|port> [--n 100 ...]
set -euo pipefail
source "$(dirname "$0")/_target.sh"; shift
"$PY" -m epuck_remote.cli probe "${BACKEND[@]}" "$@"
