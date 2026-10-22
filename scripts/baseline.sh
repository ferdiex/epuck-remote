#!/usr/bin/env bash
# IR background in open space (nothing within ~15 cm) -> calib/baseline_<target>.json
# Usage: scripts/baseline.sh <mock|sim|port> [--n 200 --calibrate ...]
set -euo pipefail
source "$(dirname "$0")/_target.sh"; shift
"$PY" -m epuck_remote.cli baseline "${BACKEND[@]}" --out "calib/baseline_${TAG}.json" "$@"
