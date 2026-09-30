#!/bin/bash
# 在 macOS 上把 AFU 协议监控打成 .app。需要带 Tk 的 python.org Python。
set -euo pipefail
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"

"$PY" -m pip install -r requirements.txt pyinstaller
"$PY" -m PyInstaller \
  --windowed \
  --noconfirm \
  --clean \
  --name "AFU协议监控" \
  --osx-bundle-identifier com.afu.wearable.monitor \
  --hidden-import serial \
  --hidden-import serial.tools.list_ports \
  afu_monitor.py

codesign --force --deep --sign - "dist/AFU协议监控.app"
ditto -c -k --keepParent "dist/AFU协议监控.app" "dist/AFU协议监控-macos.zip"
echo "已生成 dist/AFU协议监控.app"
echo "已生成 dist/AFU协议监控-macos.zip"
