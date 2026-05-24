#!/usr/bin/env bash
# Check that the selected conda environment can import the modules needed for a
# robot-only, camera-only, dataset-only, or full JoyCapture-UR5 run.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
if [[ -f "$ROOT_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/.env"
  set +a
fi

CONDA_BIN="${UR5_CONDA_BIN:-$(command -v conda || true)}"
ENV_NAME="${UR5_ENV_NAME:-UR_xbox}"
MODE="robot"
if [[ $# -gt 0 && "$1" != --* ]]; then
  MODE="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --conda-bin)
      CONDA_BIN="$2"
      shift 2
      ;;
    --env-name)
      ENV_NAME="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: ./scripts/check_urxbox_env.sh [robot|camera|dataset|all] [--conda-bin PATH] [--env-name NAME]"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ ! -x "$CONDA_BIN" ]]; then
  echo "conda not found. Use --conda-bin PATH or set UR5_CONDA_BIN." >&2
  exit 1
fi

case "$MODE" in
  robot)
    MODS_PY='["rtde_control", "rtde_receive", "inputs", "evdev"]'
    ;;
  camera)
    MODS_PY='["pyrealsense2", "cv2", "numpy"]'
    ;;
  dataset)
    MODS_PY='["h5py", "cv2", "numpy"]'
    ;;
  all)
    MODS_PY='["rtde_control", "rtde_receive", "inputs", "evdev", "pyrealsense2", "cv2", "numpy", "h5py"]'
    ;;
  *)
    echo "Usage: $0 [robot|camera|dataset|all]" >&2
    exit 2
    ;;
esac

VENDOR_BASE="$ROOT_DIR/.vendor/$ENV_NAME/lib"
PY_MM="$("$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<'PY'
import sys
print(f"python{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
export PYTHONPATH="$ROOT_DIR/src:$VENDOR_BASE/$PY_MM/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export UR5_ENV_NAME="$ENV_NAME"

"$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<PY
mods = $MODS_PY
missing = []
for name in mods:
    try:
        __import__(name)
        print(f"{name}: OK")
    except Exception as exc:
        missing.append((name, str(exc)))
        print(f"{name}: MISSING -> {exc}")

if missing:
    raise SystemExit(1)
print("Requested modules are available.")
PY
