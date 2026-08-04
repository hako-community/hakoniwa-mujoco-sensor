#!/bin/bash
# Build the M6 A-2 sensor SHM bridge (links mujoco-sensor + hakoniwa external SHM API).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

MUJOCO_ROOT="${MUJOCO_ROOT:-$REPO/../hakoniwa-mujoco-robots/src/cmake-build/_deps/mujoco_precompiled-src}"
MJ_INC="$MUJOCO_ROOT/include"; MJ_LIB="$MUJOCO_ROOT/lib"
REGISTRY_DIR="${REGISTRY_DIR:-$REPO/../hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types}"
# nlohmann/json の場所。NLOHMANN_DIR で明示指定するのが基本で、未指定のときだけ下の順で探す。
#   1) 親リポの third_party/ に submodule として並んでいる場合（hakoniwa-humanoid の構成）
#   2) システムに入っている場合（nlohmann-json3-dev など）
find_nlohmann() {
    for c in "$REPO/../nlohmann-json/single_include" \
             "$REPO/../../third_party/nlohmann-json/single_include" \
             "/usr/local/include" "/usr/include"; do
        if [ -f "$c/nlohmann/json.hpp" ]; then echo "$c"; return 0; fi
    done
    return 1
}
NLOHMANN_DIR="${NLOHMANN_DIR:-$(find_nlohmann)}"
HAKO_ROOT="${HAKO_ROOT:-/usr/local/hakoniwa}"

for f in "$MJ_INC/mujoco/mujoco.h" "$MJ_LIB/libmujoco.so" "$NLOHMANN_DIR/nlohmann/json.hpp" \
         "$HAKO_ROOT/include/hakoniwa/hako_asset.h" "$HAKO_ROOT/lib/libassets.so"; do
    [ -e "$f" ] || { echo "ERROR: missing $f"; exit 2; }
done

OUT="$HERE/sensor_bridge_multi"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" -isystem "$NLOHMANN_DIR" \
    -I "$HAKO_ROOT/include" -I "$HAKO_ROOT/include/hakoniwa" \
    "$HERE/sensor_bridge_multi.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar3d_sensor.cpp" \
    "$REPO/src/sensors/lidar/lidar_scan_sensor.cpp" \
    "$REPO/src/sensors/radar/radar_sensor.cpp" \
    "$REPO/src/sensors/noise/range_noise.cpp" \
    "$REPO/src/sensors/noise/axis_noise.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -L "$HAKO_ROOT/lib" -lassets -lconductor -lshakoc -Wl,-rpath,"$HAKO_ROOT/lib" \
    -lpthread \
    -o "$OUT"
echo "built: $OUT"
