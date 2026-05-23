#!/usr/bin/env bash
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
mkdir -p "$ROOT_DIR/.pip-cache" "$ROOT_DIR/.tmp"
export PIP_CACHE_DIR="$ROOT_DIR/.pip-cache"
export TMPDIR="$ROOT_DIR/.tmp"

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
      echo "Usage: ./scripts/setup_urxbox_env.sh [robot|camera|dataset|all] [--conda-bin PATH] [--env-name NAME]"
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

REQ_FILE="$ROOT_DIR/requirements/robot.txt"
case "$MODE" in
  robot)
    REQ_FILE="$ROOT_DIR/requirements/robot.txt"
    ;;
  camera)
    REQ_FILE="$ROOT_DIR/requirements/camera.txt"
    ;;
  dataset)
    REQ_FILE="$ROOT_DIR/requirements/dataset.txt"
    ;;
  all)
    REQ_FILE="$ROOT_DIR/requirements/all.txt"
    ;;
  *)
    echo "Usage: $0 [robot|camera|dataset|all]" >&2
    exit 2
    ;;
esac

PY_MM="$("$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<'PY'
import sys
print(f"python{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
VENDOR_SITE="$ROOT_DIR/.vendor/$ENV_NAME/lib/$PY_MM/site-packages"
mkdir -p "$VENDOR_SITE"

"$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python -m pip install --upgrade --target "$VENDOR_SITE" -r "$REQ_FILE"

echo "Dependency installation completed for env=$ENV_NAME mode=$MODE"
echo "Local vendor site-packages: $VENDOR_SITE"
