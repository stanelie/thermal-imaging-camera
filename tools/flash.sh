#!/usr/bin/env bash
# Build and flash, then optionally monitor:  tools/flash.sh [seconds-to-monitor]
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
export PICO_SDK_PATH="${PICO_SDK_PATH:-/home/stanelie/Desktop/pico-sdk}"
cmake --build "$ROOT/build-fps" 2>&1 | grep -E "error|FAILED|Linking" || true
sudo "$HERE/picotool" load -f -x "$ROOT/build-fps/thermocam.uf2" 2>&1 | tr '\r' '\n' | tail -1
sleep 7
[ -n "$1" ] && python3 "$HERE/monitor.py" "$1" --grep 'fps \|'
