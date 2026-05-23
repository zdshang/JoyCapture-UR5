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

if [[ ! -x "$CONDA_BIN" ]]; then
  echo "conda not found. Use UR5_CONDA_BIN in .env or put conda on PATH." >&2
  exit 1
fi

PY_MM="$("$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python - <<'PY'
import sys
print(f"python{sys.version_info.major}.{sys.version_info.minor}")
PY
)"

export PYTHONPATH="$ROOT_DIR/src:$ROOT_DIR/.vendor/$ENV_NAME/lib/$PY_MM/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export UR5_ENV_NAME="$ENV_NAME"

exec "$CONDA_BIN" run --no-capture-output -n "$ENV_NAME" python "$ROOT_DIR/src/postprocess_realsense_bag.py" "$@"
