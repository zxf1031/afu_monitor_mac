#!/bin/bash
# AFU 协议监控（macOS）。双击本文件，或在终端执行：
#   ./afu_monitor.command
#   ./afu_monitor.command /dev/cu.usbserial-XXXX 2000000
cd "$(dirname "$0")" || exit 1

PORT="${1:-}"
BAUD="${2:-2000000}"

PY=""
# 优先用 python.org 安装到本机的框架版，它自带 Tk。按版本号取最新的一个。
if [ -d /Library/Frameworks/Python.framework/Versions ]; then
  fw=$(ls -d /Library/Frameworks/Python.framework/Versions/3.* 2>/dev/null | sort -V | tail -n 1)
  if [ -n "$fw" ] && [ -x "$fw/bin/python3" ]; then
    PY="$fw/bin/python3"
  fi
fi
if [ -z "$PY" ]; then
  for candidate in python3 /usr/local/bin/python3 /opt/homebrew/bin/python3; do
    if command -v "$candidate" >/dev/null 2>&1; then
      PY="$candidate"
      break
    fi
  done
fi
if [ -z "$PY" ]; then
  echo "未找到 python3。"
  echo "请在这台 Mac 上安装一次官方包（自带 Tk）："
  echo "  https://www.python.org/ftp/python/3.14.6/python-3.14.6-macos11.pkg"
  echo "装完后重新双击本文件。"
  echo "按回车关闭。"
  read -r
  exit 1
fi

if ! "$PY" -c "import tkinter" >/dev/null 2>&1; then
  echo "当前 Python 没有 Tk。请改用 python.org 安装包，不要用缺少 tkinter 的精简环境。"
  echo "按回车关闭。"
  read -r
  exit 1
fi

if ! "$PY" -c "import serial" >/dev/null 2>&1; then
  echo "正在安装 pyserial ..."
  "$PY" -m pip install --user -r requirements.txt || {
    echo "pyserial 安装失败。按回车关闭。"
    read -r
    exit 1
  }
fi

if [ -n "$PORT" ]; then
  exec "$PY" afu_monitor.py --port "$PORT" --baud "$BAUD"
else
  exec "$PY" afu_monitor.py --baud "$BAUD"
fi
