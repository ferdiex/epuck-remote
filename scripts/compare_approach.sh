#!/usr/bin/env bash
# Overlay approach logs with the model curve and print the estimated --ir-shift.
# Usage: scripts/compare_approach.sh calib/approach_a.csv [calib/approach_b.csv ...] [-- --measured-end 1.0 ...]
#   --measured-end: final distance measured with a ruler [cm], one per file
set -euo pipefail
cd "$(dirname "$0")/.."
files=(); extra=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--" ]; then shift; extra=("$@"); break; fi
  files+=("$1"); shift
done
labels=(); for f in "${files[@]}"; do b=$(basename "$f" .csv); labels+=("${b#approach_}"); done
"${PYTHON:-python3}" -m epuck_remote.plot approach --csv "${files[@]}" --labels "${labels[@]}" \
    --model --out "calib/ir_approach_comparison" "${extra[@]}"
