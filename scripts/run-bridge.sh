#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)

case "$(uname -m)" in
  aarch64|arm64|ARM64)
    HIK_LIB_ARCH=arm64
    ;;
  *)
    HIK_LIB_ARCH=amd64
    ;;
esac

BRIDGE_BIN=${BRIDGE_BIN:-$PROJECT_DIR/build/minimal_hik_gimbal_bridge}
MVS_RUNTIME_PATH=${MVS_RUNTIME_PATH:-$PROJECT_DIR/third_party/hikrobot/lib/$HIK_LIB_ARCH}
SYSTEM_MVS_RUNTIME_PATH=${SYSTEM_MVS_RUNTIME_PATH:-/opt/MVS/lib/64}
LOG_PATH=${RM_BRIDGE_LOG_PATH:-$PROJECT_DIR/logs/bridge.log}
DEFAULT_CONFIG_PATH="$PROJECT_DIR/config/bridge.yaml"

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

MVS_LIBRARY_PATHS=()
if [[ -d "$SYSTEM_MVS_RUNTIME_PATH" ]]; then
  MVS_LIBRARY_PATHS+=("$SYSTEM_MVS_RUNTIME_PATH")
fi
if [[ -d "$MVS_RUNTIME_PATH" && "$MVS_RUNTIME_PATH" != "$SYSTEM_MVS_RUNTIME_PATH" ]]; then
  MVS_LIBRARY_PATHS+=("$MVS_RUNTIME_PATH")
fi

MVS_LIBRARY_PATH=$(IFS=:; echo "${MVS_LIBRARY_PATHS[*]}")
if [[ -n "$MVS_LIBRARY_PATH" ]]; then
  export LD_LIBRARY_PATH="$MVS_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

BRIDGE_ARGS=("$@")
if [[ -f "$DEFAULT_CONFIG_PATH" ]]; then
  HAS_CONFIG_ARG=0
  for arg in "$@"; do
    if [[ "$arg" == "--config" ]]; then
      HAS_CONFIG_ARG=1
      break
    fi
  done
  if (( HAS_CONFIG_ARG == 0 )); then
    BRIDGE_ARGS=(--config "$DEFAULT_CONFIG_PATH" "$@")
  fi
fi

echo "[$(date '+%F %T')] exec $BRIDGE_BIN ${BRIDGE_ARGS[*]}"
exec "$BRIDGE_BIN" "${BRIDGE_ARGS[@]}"