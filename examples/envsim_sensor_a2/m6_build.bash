#!/bin/bash
# Build the M6 A-2 sensor SHM bridge (links mujoco-sensor + hakoniwa external SHM API).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

# hakoniwa-mujoco-runtime（下位レイヤ）の include。2026-08-02 の 2 層化以降、
# primitive_types.hpp / physics.hpp などはこちらにある。
find_runtime_inc() {
    for c in "$REPO/../hakoniwa-mujoco-runtime/include" \
             "$REPO/../../third_party/hakoniwa-mujoco-runtime/include"; do
        if [ -f "$c/primitive_types.hpp" ]; then (cd "$c" && pwd) && return 0; fi
    done
    return 1
}
RUNTIME_INC="${RUNTIME_INC:-$(find_runtime_inc)}"
[ -f "$RUNTIME_INC/primitive_types.hpp" ] || { echo "ERROR: primitive_types.hpp not found under '$RUNTIME_INC' (set RUNTIME_INC; hakoniwa-mujoco-runtime が要る)"; exit 2; }

# MuJoCo の場所（include/mujoco/mujoco.h と lib/libmujoco.so を同一バージョンで持つ root）。
# MUJOCO_ROOT で明示指定するのが基本で、未指定のときだけ下の順で探す。
#   1) 親リポの CMake FetchContent キャッシュ（hakoniwa-humanoid の .cache/deps/mujoco_bin-src）
#   2) システム導入（/usr/local, /opt/mujoco）
# 第 1 引数に 1 を渡すと lib/libmujoco.so の存在も条件にする。
find_mujoco_root() {
    local need_lib="${1:-0}" c
    for c in "$REPO/../../.cache/deps/mujoco_bin-src" \
             "$REPO/../../../.cache/deps/mujoco_bin-src" \
             "/usr/local" "/opt/mujoco"; do
        [ -f "$c/include/mujoco/mujoco.h" ] || continue
        if [ "$need_lib" = "1" ] && [ ! -e "$c/lib/libmujoco.so" ]; then continue; fi
        (cd "$c" && pwd) && return 0
    done
    return 1
}
MUJOCO_ROOT="${MUJOCO_ROOT:-$(find_mujoco_root 1)}"
MJ_INC="$MUJOCO_ROOT/include"; MJ_LIB="$MUJOCO_ROOT/lib"
# PDU 型ヘッダ（hakoniwa-pdu-registry）の場所。REGISTRY_DIR で明示指定するのが基本。
#   1) 隣に hakoniwa-core-pro を並べている場合
#   2) 箱庭を install 済みの場合（/usr/local/hakoniwa/include/hakoniwa/pdu。中身は同じ）
find_registry_dir() {
    for c in "$REPO/../../hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types" \
             "$REPO/../hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types" \
             "${HAKO_ROOT:-/usr/local/hakoniwa}/include/hakoniwa/pdu"; do
        if [ -d "$c/builtin_interfaces" ]; then (cd "$c" && pwd) && return 0; fi
    done
    return 1
}
REGISTRY_DIR="${REGISTRY_DIR:-$(find_registry_dir)}"
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

OUT="$HERE/m6_sensor_bridge"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -I "$RUNTIME_INC" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" -isystem "$NLOHMANN_DIR" \
    -I "$HAKO_ROOT/include" -I "$HAKO_ROOT/include/hakoniwa" \
    "$HERE/m6_sensor_bridge.cpp" \
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
