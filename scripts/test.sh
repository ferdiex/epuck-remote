#!/usr/bin/env bash
# Smoke tests, no robot needed (sim tests are skipped if PyBullet is missing).
# Usage: scripts/test.sh
set -euo pipefail
cd "$(dirname "$0")/.."
"${PYTHON:-python3}" tests/test_smoke.py
