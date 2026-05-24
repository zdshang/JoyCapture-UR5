#!/usr/bin/env bash
# Build the native L515 recorder service when needed, then run it with the
# same arguments passed by the Python launcher.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BIN="$ROOT_DIR/.tmp/l515_system_recorder"
SRC="$ROOT_DIR/src/l515_system_recorder.cpp"

mkdir -p "$ROOT_DIR/.tmp"

if [[ ! -x "$BIN" || "$SRC" -nt "$BIN" ]]; then
  g++ -std=c++17 -O2 "$SRC" -o "$BIN" $(pkg-config --cflags --libs realsense2)
fi

exec "$BIN" "$@"
