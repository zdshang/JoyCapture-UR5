#!/usr/bin/env bash
# Launch JoyCapture-UR5 inside the configured conda environment.
#
# This wrapper loads optional .env overrides, verifies dependencies, then hands
# off to the Python launcher that starts camera services and teleoperation.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$ROOT_DIR/.env" ]]; then
  # Load lab-specific paths/IPs without committing them to the repository.
  set -a
  # shellcheck disable=SC1091
  source "$ROOT_DIR/.env"
  set +a
fi

CONDA_BIN="${UR5_CONDA_BIN:-$(command -v conda || true)}"
ENV_NAME="${UR5_ENV_NAME:-UR_xbox}"
CONFIG_PATH="${UR5_CONFIG_PATH:-}"
OUTPUT_DIR="${UR5_OUTPUT_DIR:-}"
PLAYBACK_DIR="${UR5_PLAYBACK_DIR:-}"
PLAYBACK_SESSION="${UR5_PLAYBACK_SESSION:-}"

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
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --playback-dir)
      PLAYBACK_DIR="$2"
      shift 2
      ;;
    --playback-session)
      PLAYBACK_SESSION="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage: ./run_ur5_xbox_ubuntu.sh [options]

Options:
  --conda-bin PATH   Path to conda executable. Default: $UR5_CONDA_BIN or conda from PATH.
  --env-name NAME    Conda environment name. Default: $UR5_ENV_NAME or UR_xbox.
  --config PATH      Launcher config JSON. Default: config/teleop_launcher_config.local.json if present, else config/teleop_launcher_config.json.
  --output-dir PATH  Output directory. Default: config output_dir, then ./paths.
  --playback-dir PATH
                     Raw paths directory or session_manifest_*.json used by Back playback. Default: output dir.
  --playback-session ID|latest
                     Session id used by Back playback. Default: latest.
EOF
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

cd "$ROOT_DIR"
PY_MM="$("$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<'PY'
import sys
print(f"python{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
export PYTHONPATH="$ROOT_DIR/src:$ROOT_DIR/.vendor/$ENV_NAME/lib/$PY_MM/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export UR5_ENV_NAME="$ENV_NAME"
if [[ -z "$CONFIG_PATH" ]]; then
  if [[ -f "$ROOT_DIR/config/teleop_launcher_config.local.json" ]]; then
    CONFIG_PATH="$ROOT_DIR/config/teleop_launcher_config.local.json"
  else
    CONFIG_PATH="$ROOT_DIR/config/teleop_launcher_config.json"
  fi
fi

CAMERA_ENABLE="$("$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<PY
import json
from pathlib import Path
cfg = json.loads(Path("$CONFIG_PATH").read_text(encoding="utf-8"))
print("1" if bool(cfg.get("camera_enable", True)) else "0")
PY
)"

if [[ "$CAMERA_ENABLE" == "1" ]]; then
  # Camera mode needs RealSense/OpenCV in addition to robot-control packages.
  "$ROOT_DIR/scripts/check_urxbox_env.sh" all --conda-bin "$CONDA_BIN" --env-name "$ENV_NAME"
else
  "$ROOT_DIR/scripts/check_urxbox_env.sh" robot --conda-bin "$CONDA_BIN" --env-name "$ENV_NAME"
fi

LAUNCH_ARGS=(--config "$CONFIG_PATH")
if [[ -n "$OUTPUT_DIR" ]]; then
  LAUNCH_ARGS+=(--output-dir "$OUTPUT_DIR")
fi
if [[ -n "$PLAYBACK_DIR" ]]; then
  LAUNCH_ARGS+=(--playback-dir "$PLAYBACK_DIR")
fi
if [[ -n "$PLAYBACK_SESSION" ]]; then
  LAUNCH_ARGS+=(--playback-session "$PLAYBACK_SESSION")
fi

exec "$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python "$ROOT_DIR/src/launch_ur5_xbox_ubuntu.py" "${LAUNCH_ARGS[@]}"
