#!/usr/bin/env bash
# Builds lidar_bridge against the rplidar_sdk driver on macOS / Linux.
# Untested on macOS - written to mirror bridge/build.bat.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../vendor" && pwd)"

# 1. Build the SDK static lib. The top-level Makefile picks the platform up from
#    uname, emitting output/<Platform>/Release.
make -C "$ROOT/rplidar_sdk"

PLATFORM="$(uname -s)"   # Darwin or Linux
LIB="$ROOT/rplidar_sdk/output/$PLATFORM/Release/librplidar_sdk.a"

if [ ! -f "$LIB" ]; then
  echo "[build] expected SDK lib not found at: $LIB" >&2
  echo "[build] check what the SDK Makefile produced under rplidar_sdk/output/" >&2
  exit 1
fi

mkdir -p "$HERE/build"

# 2. Compile the bridge.
c++ -O2 -std=c++11 -pthread \
  -I "$ROOT/rplidar_sdk/sdk/include" \
  -I "$ROOT/rplidar_sdk/sdk/src" \
  "$HERE/lidar_bridge.cxx" \
  "$LIB" \
  -o "$HERE/build/lidar_bridge"

echo "[build] OK -> $HERE/build/lidar_bridge"
