#!/bin/bash
# 在 macOS 上把 AFU 协议监控打成通用 .app（Intel x86_64 + Apple 芯片 arm64）。
# 需要 python.org 的 universal2 Python，它自带 Tk。
set -euo pipefail
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"
# 13.7 是 Ventura。不设这项时，构建机会把最低系统写成自己的新版本。
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-13.0}"

echo "Python: $PY"
"$PY" -c 'import platform,sys; print(platform.platform()); print(platform.machine())'
file "$PY" || true

"$PY" -m pip install -r requirements.txt pyinstaller
"$PY" -m PyInstaller \
  --windowed \
  --noconfirm \
  --clean \
  --target-architecture universal2 \
  --name "AFU协议监控" \
  --osx-bundle-identifier com.afu.wearable.monitor \
  --hidden-import serial \
  --hidden-import serial.tools.list_ports \
  afu_monitor.py

APP="dist/AFU协议监控.app"
BIN="$APP/Contents/MacOS/AFU协议监控"
lipo -info "$BIN"
ARCHS="$(lipo -archs "$BIN")"
echo "主程序架构: $ARCHS"
echo "$ARCHS" | grep -q "x86_64"
echo "$ARCHS" | grep -q "arm64"

/usr/libexec/PlistBuddy -c "Print :LSMinimumSystemVersion" "$APP/Contents/Info.plist" || true
MINOS_LIST="$(otool -l "$BIN" | awk '/minos/{print $2}')"
echo "最低系统版本 minos: $MINOS_LIST"
echo "目标系统版本: $MACOSX_DEPLOYMENT_TARGET"
MINOS_LIST="$MINOS_LIST" "$PY" - <<'PY'
import os
import sys
target = tuple(int(x) for x in os.environ["MACOSX_DEPLOYMENT_TARGET"].split("."))
found = os.environ.get("MINOS_LIST", "").split()
if not found:
    sys.exit("没有读到主程序的 minos，无法确认能否在 macOS 13 上打开")
for item in found:
    parts = tuple(int(x) for x in item.split("."))
    if parts > target:
        sys.exit(f"最低系统版本 {item} 高于目标 {os.environ['MACOSX_DEPLOYMENT_TARGET']}")
print("架构和最低系统版本检查通过")
PY

codesign --force --deep --sign - "$APP"
ditto -c -k --keepParent "$APP" "dist/AFU协议监控-macos.zip"
echo "已生成 $APP"
echo "已生成 dist/AFU协议监控-macos.zip"
