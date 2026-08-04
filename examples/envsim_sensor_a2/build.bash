#!/bin/bash
# Build the M2 A-2 LiDAR demo.
#
# Requires a matched MuJoCo header+lib pair. Searches the parent repository's
# CMake FetchContent cache and the usual system prefixes; override with
# MUJOCO_ROOT (must contain include/mujoco/mujoco.h and lib/libmujoco.so of the
# SAME version).
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
MJ_INC="$MUJOCO_ROOT/include"
MJ_LIB="$MUJOCO_ROOT/lib"

if [ ! -f "$MJ_INC/mujoco/mujoco.h" ]; then
    echo "ERROR: mujoco.h not found under $MJ_INC (set MUJOCO_ROOT)"; exit 2
fi
if [ ! -f "$MJ_LIB/libmujoco.so" ]; then
    echo "ERROR: libmujoco.so not found under $MJ_LIB (set MUJOCO_ROOT)"; exit 2
fi

OUT="$HERE/lidar_a2_demo"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -I "$RUNTIME_INC" -isystem "$MJ_INC" \
    "$HERE/lidar_a2_demo.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar_scan_sensor.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT"
echo "built: $OUT"

# M3 PDU publisher: needs the hakoniwa PDU registry type headers (header-only).
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
if [ ! -f "$REGISTRY_DIR/sensor_msgs/pdu_cpptype_conv_LaserScan.hpp" ]; then
    echo "WARN: PDU registry types not found under $REGISTRY_DIR -- skipping lidar_a2_pdu (set REGISTRY_DIR)"
    exit 0
fi
OUT_PDU="$HERE/lidar_a2_pdu"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -I "$RUNTIME_INC" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" \
    "$HERE/lidar_a2_pdu.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar_scan_sensor.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT_PDU"
echo "built: $OUT_PDU"

# A-1: 3D LiDAR -> PointCloud2 PDU
OUT_PDU3D="$HERE/lidar3d_a2_pdu"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -I "$RUNTIME_INC" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" \
    "$HERE/lidar3d_a2_pdu.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar3d_sensor.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT_PDU3D"
echo "built: $OUT_PDU3D"

# B-1: manifest-driven sensor runtime (needs nlohmann/json).
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

if [ ! -f "$NLOHMANN_DIR/nlohmann/json.hpp" ]; then
    echo "WARN: nlohmann/json.hpp not found under $NLOHMANN_DIR -- skipping sensor_runtime_demo (set NLOHMANN_DIR)"
    exit 0
fi
OUT_RT="$HERE/sensor_runtime_demo"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -I "$RUNTIME_INC" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" -isystem "$NLOHMANN_DIR" \
    "$HERE/sensor_runtime_demo.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar3d_sensor.cpp" \
    "$REPO/src/sensors/lidar/lidar_scan_sensor.cpp" \
    "$REPO/src/sensors/radar/radar_sensor.cpp" \
    "$REPO/src/sensors/noise/range_noise.cpp" \
    "$REPO/src/sensors/noise/axis_noise.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT_RT"
echo "built: $OUT_RT"

# M4: env-XML-presence sensing-mode switch
OUT_SW="$HERE/sensing_switch_demo"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -I "$RUNTIME_INC" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" -isystem "$NLOHMANN_DIR" \
    "$HERE/sensing_switch_demo.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar3d_sensor.cpp" \
    "$REPO/src/sensors/lidar/lidar_scan_sensor.cpp" \
    "$REPO/src/sensors/radar/radar_sensor.cpp" \
    "$REPO/src/sensors/noise/range_noise.cpp" \
    "$REPO/src/sensors/noise/axis_noise.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT_SW"
echo "built: $OUT_SW"
echo "verify: python3 verify_pdu.py ; verify_pdu_3d.py ; verify_runtime.py ; verify_b2.py ; verify_m4.py"
