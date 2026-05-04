#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)

BRIDGE_BIN=${BRIDGE_BIN:-$PROJECT_DIR/build/minimal_hik_gimbal_bridge}
MVS_RUNTIME_PATH=${MVS_RUNTIME_PATH:-/opt/MVS/bin:/opt/MVS/lib/64}
LOG_PATH=${RM_BRIDGE_LOG_PATH:-$PROJECT_DIR/logs/bridge.log}

if [[ "${RM_BRIDGE_LOG_TO_FILE:-0}" == "1" || "${RM_BRIDGE_LOG_TO_FILE:-false}" == "true" ]]; then
  mkdir -p -- "$(dirname -- "$LOG_PATH")"
  touch "$LOG_PATH"
  exec >>"$LOG_PATH" 2>&1
  echo "[$(date '+%F %T')] bridge launcher start"
fi

STARTUP_DELAY=${RM_BRIDGE_STARTUP_DELAY:-0}
if [[ "$STARTUP_DELAY" =~ ^[0-9]+$ ]] && (( STARTUP_DELAY > 0 )); then
  sleep "$STARTUP_DELAY"
fi

if [[ ! -x "$BRIDGE_BIN" ]]; then
  echo "缺少 bridge 二进制: $BRIDGE_BIN" >&2
  echo "先执行: cmake -S . -B build && cmake --build build -j" >&2
  exit 1
fi

export LD_LIBRARY_PATH="$MVS_RUNTIME_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "[$(date '+%F %T')] exec $BRIDGE_BIN $*"
exec "$BRIDGE_BIN" "$@"