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

MUJOCO_ROOT="${MUJOCO_ROOT:-$REPO/../hakoniwa-mujoco-robots/src/cmake-build/_deps/mujoco_precompiled-src}"
MJ_INC="$MUJOCO_ROOT/include"
NLOHMANN_DIR="${NLOHMANN_DIR:-$REPO/../hakoniwa-mujoco-robots/thirdparty/nolman/single_include}"

[ -f "$MJ_INC/mujoco/mujoco.h" ]      || { echo "ERROR: mujoco.h not found under $MJ_INC (set MUJOCO_ROOT)"; exit 2; }
[ -f "$NLOHMANN_DIR/nlohmann/json.hpp" ] || { echo "ERROR: nlohmann/json.hpp not found under $NLOHMANN_DIR (set NLOHMANN_DIR)"; exit 2; }

SO="$HERE/libhako_mujoco_sensor_capi.so"
g++ -std=c++20 -O2 -Wall -Wextra -fPIC -shared \
    -I "$REPO/include" -isystem "$MJ_INC" -isystem "$NLOHMANN_DIR" \
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
