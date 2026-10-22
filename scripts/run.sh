#!/usr/bin/env bash
# Run a controller (default: Braitenberg avoidance) and plot the log.
# Usage: scripts/run.sh <mock|sim|sim-gui|port> [run options]
#   e.g. scripts/run.sh sim-gui --duration 60
#        scripts/run.sh /dev/rfcomm0 --calib calib/baseline_dev_rfcomm0.json --duration 30
set -euo pipefail
source "$(dirname "$0")/_target.sh"; shift
mkdir -p runs
OUT="runs/${STAMP}_${TAG}"
"$PY" -m epuck_remote.cli run "${BACKEND[@]}" --log "${OUT}.csv" "$@"
"$PY" -m epuck_remote.plot run --log "${OUT}.csv" --out "$OUT"
