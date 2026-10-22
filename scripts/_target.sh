# Sourced by the other scripts. Maps the first argument to backend options:
#   mock | sim | sim-gui | <serial port> (e.g. /dev/rfcomm0) | <profile.json>
cd "$(dirname "${BASH_SOURCE[0]}")/.."
TARGET="${1:?first argument: mock | sim | sim-gui | <serial port>}"
case "$TARGET" in
  mock)    BACKEND=(--backend mock) ;;
  sim)     BACKEND=(--backend sim) ;;
  sim-gui) BACKEND=(--backend sim --gui) ;;
  *.json)  BACKEND=(--profile "$TARGET") ;;
  *)       BACKEND=(--backend real --port "$TARGET") ;;
esac
TAG="$(basename "${TARGET%.json}")"; TAG="${TAG//[^A-Za-z0-9-]/_}"
STAMP=$(date +%Y%m%d_%H%M%S)
PY="${PYTHON:-python3}"
