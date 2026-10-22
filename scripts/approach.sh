#!/usr/bin/env bash
# IR calibration: drive slowly toward a wall, log front IR vs distance.
# Real robot: place it facing a wall with its front --start-dist m away (default 0.10).
# Mock/sim place the robot themselves.
# Usage: scripts/approach.sh <mock|sim|port> [--start-dist 0.10 --speed 100]
set -euo pipefail
source "$(dirname "$0")/_target.sh"; shift
"$PY" -m epuck_remote.cli approach "${BACKEND[@]}" --out "calib/approach_${TAG}_${STAMP}.csv" "$@"
