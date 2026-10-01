#!/usr/bin/env bash
# picotool with USB support; the copy the Pico SDK fetches is built without it
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$(dirname "$HERE")/build-fps/_deps/picotool-src"
export PICO_SDK_PATH="${PICO_SDK_PATH:-/home/stanelie/Desktop/pico-sdk}"
cmake -S "$SRC" -B "$HERE/.build" -G Ninja -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$HERE/.build" >/dev/null
cp "$HERE/.build/picotool" "$HERE/picotool"
"$HERE/picotool" version
