#!/usr/bin/env bash
# Speed / turn calibration: fixed command, compare expected vs measured motion.
# Usage: scripts/drive.sh <mock|sim|port> [--left 500 --right 500 --seconds 5]
#   straight: --left 500 --right 500 ; spin: --left -300 --right 300
set -euo pipefail
source "$(dirname "$0")/_target.sh"; shift
"$PY" -m epuck_remote.cli drive "${BACKEND[@]}" "$@"
