#!/usr/bin/env bash
# Install the dependencies.
#
# Usage: scripts/setup.sh [auto|conda|venv] [env_name_or_dir]
#   auto  (default) conda if available, otherwise venv
#   conda create or update the conda env from environment.yml
#         (default name: epuck-remote). Recommended on macOS: conda-forge
#         ships prebuilt pybullet, so nothing is compiled.
#   venv  python venv (default dir: .venv). PYTHON=python3.x chooses the
#         interpreter; SYSTEM_SITE=1 reuses its installed packages.
#         pybullet only from a prebuilt wheel, which does not exist for
#         every Python version; mock and real work without it.
# Both modes finish by running the smoke tests.
set -euo pipefail
cd "$(dirname "$0")/.."
MODE="${1:-auto}"

CONDA=""
for c in mamba "${CONDA_EXE:-}" conda; do
    if [ -n "$c" ] && command -v "$c" >/dev/null 2>&1; then CONDA="$c"; break; fi
done
if [ "$MODE" = auto ]; then
    if [ -n "$CONDA" ]; then MODE=conda; else MODE=venv; fi
fi

if [ "$MODE" = conda ]; then
    [ -n "$CONDA" ] || { echo "conda/mamba not found (install miniforge, or use: $0 venv)"; exit 1; }
    NAME="${2:-epuck-remote}"
    echo "using $CONDA, env '$NAME'"
    if "$CONDA" env list | awk '{print $1}' | grep -qx "$NAME"; then
        "$CONDA" env update -n "$NAME" -f environment.yml --prune
    else
        "$CONDA" env create -n "$NAME" -f environment.yml
    fi
    "$CONDA" run -n "$NAME" python tests/test_smoke.py
    echo
    echo "ready: conda activate $NAME"
    exit 0
fi

if [ "$MODE" != venv ]; then
    echo "unknown mode '$MODE' (auto, conda or venv)"; exit 1
fi
V="${2:-.venv}"
PY="${PYTHON:-python3}"
echo "interpreter: $("$PY" -c 'import sys; print(sys.executable, sys.version.split()[0])')"
ARGS=()
[ "${SYSTEM_SITE:-0}" = 1 ] && ARGS+=(--system-site-packages)
"$PY" -m venv "${ARGS[@]}" "$V"
"$V/bin/pip" install -q -U pip
"$V/bin/pip" install -r requirements.txt
if "$V/bin/python" -c "import pybullet" 2>/dev/null; then
    echo "pybullet: already available"
elif "$V/bin/pip" install --only-binary=:all: pybullet; then
    echo "pybullet: installed from a prebuilt wheel"
else
    echo
    echo "WARNING: no prebuilt pybullet wheel for this Python: the sim backend"
    echo "will not work (mock and real do). Use '$0 conda' (miniforge), or a"
    echo "Python with pybullet: PYTHON=... SYSTEM_SITE=1 $0 venv"
fi
"$V/bin/python" tests/test_smoke.py
echo
echo "ready: source $V/bin/activate"
