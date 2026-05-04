#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
AUTOSTART_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/autostart"
BIN_DIR="$PROJECT_DIR/bin"
BUILD_DIR="$PROJECT_DIR/build"
SOURCE_BIN="$BUILD_DIR/minimal_hik_gimbal_bridge"
SOURCE_LAUNCHER="$PROJECT_DIR/scripts/run-bridge.sh"
TARGET_BIN="$BIN_DIR/minimal_hik_gimbal_bridge"
TARGET_LAUNCHER="$BIN_DIR/minimal-hik-gimbal-bridge-run"
TEMPLATE="$PROJECT_DIR/deploy/autostart/minimal-hik-gimbal-bridge.desktop.in"
AUTOSTART_DELAY="${RM_BRIDGE_STARTUP_DELAY:-5}"
LOG_DIR="$PROJECT_DIR/logs"
LOG_FILE="$LOG_DIR/bridge-autostart.log"
SKIP_BUILD=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=true
      ;;
    --help|-h)
      cat <<'EOF'
用法: ./scripts/install-autostart.sh [--skip-build]

会执行以下操作：
  - 如有需要自动构建 bridge
  - 复制 bridge 二进制和启动脚本到 bin/
  - 安装桌面自启动项 RoboMaster Hik Bridge
  - 开机自启日志写到 logs/bridge-autostart.log

默认会自动构建；如果你已经手动编译过，可加 --skip-build。
EOF
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 1
      ;;
  esac
  shift
done

if [[ "$SKIP_BUILD" != true ]]; then
  cmake -S "$PROJECT_DIR" -B "$BUILD_DIR"
  cmake --build "$BUILD_DIR" -j
fi

if [[ ! -x "$SOURCE_BIN" ]]; then
  echo "缺少 bridge 二进制: $SOURCE_BIN" >&2
  echo "先执行: cmake -S . -B build && cmake --build build -j" >&2
  exit 1
fi

if [[ ! -f "$SOURCE_LAUNCHER" ]]; then
  echo "缺少启动脚本: $SOURCE_LAUNCHER" >&2
  exit 1
fi

mkdir -p "$BIN_DIR" "$AUTOSTART_DIR"
mkdir -p "$LOG_DIR"
cp "$SOURCE_BIN" "$TARGET_BIN"
cp "$SOURCE_LAUNCHER" "$TARGET_LAUNCHER"
chmod +x "$TARGET_BIN"
chmod +x "$TARGET_LAUNCHER"

rm -f \
  "$AUTOSTART_DIR/minimal-hik-gimbal-bridge.desktop" \
  "$AUTOSTART_DIR/rm-hik-bridge.desktop"

DESKTOP_FILE="$AUTOSTART_DIR/minimal-hik-gimbal-bridge.desktop"
EXEC_LINE="env RM_BRIDGE_STARTUP_DELAY=${AUTOSTART_DELAY} RM_BRIDGE_LOG_TO_FILE=1 RM_BRIDGE_LOG_PATH=${LOG_FILE} $TARGET_LAUNCHER"

sed \
  -e "s|%APP_NAME%|RoboMaster Hik Bridge|g" \
  -e "s|%APP_COMMENT%|Autostart bridge for Hik camera to official 0x0310 video link|g" \
  -e "s|%EXEC_PATH%|${EXEC_LINE}|g" \
  -e "s|%WORK_DIR%|${PROJECT_DIR}|g" \
  -e "s|%AUTOSTART_ENABLED%|true|g" \
  "$TEMPLATE" > "$DESKTOP_FILE"

chmod 644 "$DESKTOP_FILE"

echo "已安装 Startup 项: $DESKTOP_FILE"
echo "  启动命令: $EXEC_LINE"
echo "  日志文件: $LOG_FILE"