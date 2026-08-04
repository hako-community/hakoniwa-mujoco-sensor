#!/bin/bash
# Build the C-ABI shared library (formal Phase 2 deliverable) and its backend-
# free smoke test. Mirrors examples/envsim_sensor_a2/build.bash's direct-g++
# style so it works without a full CMake configure on this machine.
#
# The .so does NOT link libmujoco: the sensor model .cpp files only include
# mujoco.h for POD types; the world ray cast is injected via the C callback.
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

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
MUJOCO_ROOT="${MUJOCO_ROOT:-$(find_mujoco_root 0)}"
MJ_INC="$MUJOCO_ROOT/include"
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

[ -f "$MJ_INC/mujoco/mujoco.h" ]      || { echo "ERROR: mujoco.h not found under $MJ_INC (set MUJOCO_ROOT)"; exit 2; }
[ -f "$NLOHMANN_DIR/nlohmann/json.hpp" ] || { echo "ERROR: nlohmann/json.hpp not found under $NLOHMANN_DIR (set NLOHMANN_DIR)"; exit 2; }
[ -f "$RUNTIME_INC/primitive_types.hpp" ] || { echo "ERROR: primitive_types.hpp not found under $RUNTIME_INC (set RUNTIME_INC; hakoniwa-mujoco-runtime が要る)"; exit 2; }

SO="$HERE/libhako_mujoco_sensor_capi.so"
g++ -std=c++20 -O2 -Wall -Wextra -fPIC -shared \
    -I "$REPO/include" -I "$RUNTIME_INC" -isystem "$MJ_INC" -isystem "$NLOHMANN_DIR" \
    "$HERE/hako_sensor_capi.cpp" \
    "$REPO/src/sensors/lidar/lidar3d_sensor.cpp" \
    "$REPO/src/sensors/radar/radar_sensor.cpp" \
    "$REPO/src/sensors/noise/range_noise.cpp" \
    "$REPO/src/sensors/noise/axis_noise.cpp" \
    -o "$SO"
echo "built: $SO"

SMOKE="$HERE/smoke_capi"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$HERE" \
    "$HERE/smoke_capi.cpp" \
    -L "$HERE" -lhako_mujoco_sensor_capi -Wl,-rpath,"$HERE" \
    -o "$SMOKE"
echo "built: $SMOKE"
echo "run: $SMOKE"
