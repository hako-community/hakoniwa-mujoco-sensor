#!/bin/bash
# Build the M2 A-2 LiDAR demo.
#
# Requires a matched MuJoCo header+lib pair. Defaults to the 3.9.0 pair fetched
# by hakoniwa-mujoco-robots; override with MUJOCO_ROOT (must contain
# include/mujoco/mujoco.h and lib/libmujoco.so of the SAME version).
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

MUJOCO_ROOT="${MUJOCO_ROOT:-$REPO/../hakoniwa-mujoco-robots/src/cmake-build/_deps/mujoco_precompiled-src}"
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
    -I "$REPO/include" -isystem "$MJ_INC" \
    "$HERE/lidar_a2_demo.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar_scan_sensor.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT"
echo "built: $OUT"

# M3 PDU publisher: needs the hakoniwa PDU registry type headers (header-only).
REGISTRY_DIR="${REGISTRY_DIR:-$REPO/../hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types}"
if [ ! -f "$REGISTRY_DIR/sensor_msgs/pdu_cpptype_conv_LaserScan.hpp" ]; then
    echo "WARN: PDU registry types not found under $REGISTRY_DIR -- skipping lidar_a2_pdu (set REGISTRY_DIR)"
    exit 0
fi
OUT_PDU="$HERE/lidar_a2_pdu"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" \
    "$HERE/lidar_a2_pdu.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar_scan_sensor.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT_PDU"
echo "built: $OUT_PDU"

# A-1: 3D LiDAR -> PointCloud2 PDU
OUT_PDU3D="$HERE/lidar3d_a2_pdu"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" \
    "$HERE/lidar3d_a2_pdu.cpp" \
    "$REPO/src/sensors/backend/mujoco_ray_caster.cpp" \
    "$REPO/src/sensors/lidar/lidar3d_sensor.cpp" \
    -L "$MJ_LIB" -lmujoco -Wl,-rpath,"$MJ_LIB" \
    -o "$OUT_PDU3D"
echo "built: $OUT_PDU3D"

# B-1: manifest-driven sensor runtime (needs nlohmann/json).
NLOHMANN_DIR="${NLOHMANN_DIR:-$REPO/../hakoniwa-mujoco-robots/thirdparty/nolman/single_include}"
if [ ! -f "$NLOHMANN_DIR/nlohmann/json.hpp" ]; then
    echo "WARN: nlohmann/json.hpp not found under $NLOHMANN_DIR -- skipping sensor_runtime_demo (set NLOHMANN_DIR)"
    exit 0
fi
OUT_RT="$HERE/sensor_runtime_demo"
g++ -std=c++20 -O2 -Wall -Wextra \
    -I "$REPO/include" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" -isystem "$NLOHMANN_DIR" \
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
    -I "$REPO/include" -isystem "$MJ_INC" -isystem "$REGISTRY_DIR" -isystem "$NLOHMANN_DIR" \
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
