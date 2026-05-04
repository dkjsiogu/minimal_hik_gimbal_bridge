#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)

BRIDGE_BIN=${BRIDGE_BIN:-$PROJECT_DIR/build/minimal_hik_gimbal_bridge}
MVS_RUNTIME_PATH=${MVS_RUNTIME_PATH:-/opt/MVS/bin:/opt/MVS/lib/64}

if [[ ! -x "$BRIDGE_BIN" ]]; then
  echo "缺少 bridge 二进制: $BRIDGE_BIN" >&2
  echo "先执行: cmake -S . -B build && cmake --build build -j" >&2
  exit 1
fi

export LD_LIBRARY_PATH="$MVS_RUNTIME_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec "$BRIDGE_BIN" "$@"