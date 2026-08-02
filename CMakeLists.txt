cmake_minimum_required(VERSION 3.16)

project(hakoniwa-mujoco-sensor
    VERSION 2.0.0
    DESCRIPTION "Backend-agnostic sensor library (upper layer). Depends on hakoniwa-mujoco-runtime."
    LANGUAGES C CXX
)

# =============================================================================
# レイヤ構成（2026-08-02 再編）
#   hakoniwa-mujoco-sensor （本リポ・センサ）
#           │ 依存
#           ▼
#   hakoniwa-mujoco-runtime（MuJoCo バックエンド + 箱庭アセット基盤 + 共通土台）
#           │ 依存
#           ▼
#   hakoniwa-core / hakoniwa_pdu_endpoint / MuJoCo
#
# 以下は runtime へ移設済み（本リポには無い）:
#   primitive_types.hpp / physics.hpp / actuator.hpp
#   config/json_config_utils.hpp
#   sensors/common/{update_scheduler,json_utils}.hpp -> runtime の common/（ns: hako::robots::common）
# =============================================================================
set(HAKO_MUJOCO_RUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-mujoco-runtime"
    CACHE PATH "hakoniwa-mujoco-runtime のルート（下位レイヤ）")
if(NOT EXISTS "${HAKO_MUJOCO_RUNTIME_DIR}/include/physics.hpp")
    message(FATAL_ERROR
        "hakoniwa-mujoco-runtime が見つかりません: ${HAKO_MUJOCO_RUNTIME_DIR}\n"
        "  -DHAKO_MUJOCO_RUNTIME_DIR=<path> で指定してください。")
endif()

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(HAKO_SENSOR_BUILD_TESTS "Build sensor unit tests" ON)

# MuJoCo headers are required to COMPILE the library (primitive_types.hpp and the
# MuJoCo ray-caster backend include <mujoco/mujoco.h>). Linking libmujoco is the
# responsibility of the final executable / integration layer, not this archive.
set(MUJOCO_INCLUDE_DIR "" CACHE PATH "Directory containing mujoco/mujoco.h")

# Hakoniwa core install prefix (provides generated PDU type headers used by the
# PDU converters/adapters). Defaults to the standard install location.
set(HAKONIWA_INSTALL_PREFIX "/usr/local/hakoniwa" CACHE PATH "hakoniwa-core install prefix")

# Vendored nlohmann/json (single header). Falls back to system package.
set(NLOHMANN_JSON_INCLUDE_DIR "" CACHE PATH "Directory containing nlohmann/json.hpp")

# ---------------------------------------------------------------------------
# Dependency discovery (best-effort, overridable)
# ---------------------------------------------------------------------------
if(NOT MUJOCO_INCLUDE_DIR)
    foreach(_cand
        "${CMAKE_CURRENT_SOURCE_DIR}/../../sensor/mujoco/include"
        "/usr/local/include")
        if(EXISTS "${_cand}/mujoco/mujoco.h")
            set(MUJOCO_INCLUDE_DIR "${_cand}")
            break()
        endif()
    endforeach()
endif()

if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    foreach(_cand
        "${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-mujoco-robots/thirdparty/nolman/single_include"
        "/usr/include")
        if(EXISTS "${_cand}/nlohmann/json.hpp")
            set(NLOHMANN_JSON_INCLUDE_DIR "${_cand}")
            break()
        endif()
    endforeach()
endif()

set(HAKO_PDU_TYPES_INCLUDE_DIR "${HAKONIWA_INSTALL_PREFIX}/include/hakoniwa/pdu")

message(STATUS "MUJOCO_INCLUDE_DIR        = ${MUJOCO_INCLUDE_DIR}")
message(STATUS "NLOHMANN_JSON_INCLUDE_DIR = ${NLOHMANN_JSON_INCLUDE_DIR}")
message(STATUS "HAKO_PDU_TYPES_INCLUDE_DIR= ${HAKO_PDU_TYPES_INCLUDE_DIR}")

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(hako_mujoco_sensor STATIC
    src/sensors/backend/mujoco_ray_caster.cpp
    src/sensors/radar/radar_sensor.cpp
    src/sensors/lidar/lidar_2d_sensor.cpp
    src/sensors/lidar/lidar_scan_sensor.cpp
    src/sensors/lidar/lidar3d_sensor.cpp
    src/sensors/ultrasonic/ultrasonic_sensor.cpp
    src/sensors/imu/imu_sensor.cpp
    src/sensors/odometry/odometry_sensor.cpp
    src/sensors/joint_state/joint_state_sensor.cpp
    src/sensors/tf/tf_publisher.cpp
    src/sensors/noise/range_noise.cpp
    src/sensors/noise/axis_noise.cpp
)
target_compile_features(hako_mujoco_sensor PUBLIC cxx_std_20)

target_include_directories(hako_mujoco_sensor PUBLIC
    ${HAKO_MUJOCO_RUNTIME_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(hako_mujoco_sensor SYSTEM PUBLIC
    ${MUJOCO_INCLUDE_DIR}
    ${NLOHMANN_JSON_INCLUDE_DIR}
    ${HAKO_PDU_TYPES_INCLUDE_DIR}
)

if(MSVC)
    target_compile_options(hako_mujoco_sensor PRIVATE /W4)
else()
    target_compile_options(hako_mujoco_sensor PRIVATE -Wall -Wextra)
endif()

# Allow the static archive to be linked into the C-ABI shared library below.
set_target_properties(hako_mujoco_sensor PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# C-ABI shared library (Phase 2 / 4th-slide deliverable).
#
# A stable, consumer-agnostic C interface (capi/hako_sensor_capi.h) that lets any
# consumer -- the Godot sample, a test harness, another engine -- drive the
# LiDAR/Radar models in-process WITHOUT core-pro/PDU/SHM. World ray casts are
# injected through a C callback, so this layer never depends on Godot and the
# model source (include/src) stays untouched. Produces
# libhako_mujoco_sensor_capi.so. It does NOT link libmujoco: the sensor .cpp only
# use mujoco.h for POD types; ray casting is the injected callback's job.
# ---------------------------------------------------------------------------
add_library(hako_mujoco_sensor_capi SHARED capi/hako_sensor_capi.cpp)
target_compile_features(hako_mujoco_sensor_capi PRIVATE cxx_std_20)
target_include_directories(hako_mujoco_sensor_capi PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/capi)
target_link_libraries(hako_mujoco_sensor_capi PRIVATE hako_mujoco_sensor)
if(NOT MSVC)
    target_compile_options(hako_mujoco_sensor_capi PRIVATE -Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# Tests (CTest). The radar model test is backend-free: no MuJoCo/PDU link.
# ---------------------------------------------------------------------------
if(HAKO_SENSOR_BUILD_TESTS)
    enable_testing()
    add_executable(radar_math_test
        tests/radar_math_test.cpp
        src/sensors/radar/radar_sensor.cpp
        src/sensors/noise/range_noise.cpp
        src/sensors/noise/axis_noise.cpp
    )
    target_compile_features(radar_math_test PRIVATE cxx_std_20)
    target_include_directories(radar_math_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_include_directories(radar_math_test SYSTEM PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${NLOHMANN_JSON_INCLUDE_DIR}
    )
    add_test(NAME radar_math_test COMMAND radar_math_test)

    # C-ABI smoke: dummy ray caster -> both lidar3d and radar return points.
    # Backend-free (links only the .so), no MuJoCo/PDU/Godot.
    add_executable(hako_sensor_capi_smoke capi/smoke_capi.cpp)
    target_compile_features(hako_sensor_capi_smoke PRIVATE cxx_std_20)
    target_include_directories(hako_sensor_capi_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/capi)
    target_link_libraries(hako_sensor_capi_smoke PRIVATE hako_mujoco_sensor_capi)
    add_test(NAME hako_sensor_capi_smoke COMMAND hako_sensor_capi_smoke)

    # Real-physics integration test: requires a linkable libmujoco.
    # Auto-detect alongside the headers; override with -DMUJOCO_LIB_DIR=...
    set(MUJOCO_LIB_DIR "" CACHE PATH "Directory containing libmujoco.so (enables the MuJoCo integration test)")
    if(NOT MUJOCO_LIB_DIR AND MUJOCO_INCLUDE_DIR)
        get_filename_component(_mj_root "${MUJOCO_INCLUDE_DIR}" DIRECTORY)
        if(EXISTS "${_mj_root}/lib/libmujoco.so")
            set(MUJOCO_LIB_DIR "${_mj_root}/lib")
        endif()
    endif()
    find_library(MUJOCO_LIBRARY NAMES mujoco PATHS "${MUJOCO_LIB_DIR}")
    if(MUJOCO_LIBRARY)
        message(STATUS "MuJoCo lib found: ${MUJOCO_LIBRARY} -- building mujoco_radar_integration_test")
        add_executable(mujoco_radar_integration_test
            tests/mujoco_radar_integration_test.cpp
            src/sensors/backend/mujoco_ray_caster.cpp
            src/sensors/radar/radar_sensor.cpp
            src/sensors/noise/range_noise.cpp
            src/sensors/noise/axis_noise.cpp
        )
        target_compile_features(mujoco_radar_integration_test PRIVATE cxx_std_20)
        target_include_directories(mujoco_radar_integration_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        target_include_directories(mujoco_radar_integration_test SYSTEM PRIVATE
            ${MUJOCO_INCLUDE_DIR} ${NLOHMANN_JSON_INCLUDE_DIR})
        target_link_libraries(mujoco_radar_integration_test PRIVATE ${MUJOCO_LIBRARY})
        add_test(NAME mujoco_radar_integration_test COMMAND mujoco_radar_integration_test)

        # M2 A-2 LiDAR demo: sense an envsim-sensor generated world (env.xml).
        add_executable(lidar_a2_demo
            examples/envsim_sensor_a2/lidar_a2_demo.cpp
            src/sensors/backend/mujoco_ray_caster.cpp
            src/sensors/lidar/lidar_scan_sensor.cpp
        )
        target_compile_features(lidar_a2_demo PRIVATE cxx_std_20)
        target_include_directories(lidar_a2_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
        target_include_directories(lidar_a2_demo SYSTEM PRIVATE ${MUJOCO_INCLUDE_DIR})
        target_link_libraries(lidar_a2_demo PRIVATE ${MUJOCO_LIBRARY})
        add_test(NAME lidar_a2_demo COMMAND lidar_a2_demo
            "${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-envsim-sensor/examples/simple_room/generated/env.xml")
    else()
        message(STATUS "libmujoco not found -- skipping mujoco_radar_integration_test (set MUJOCO_LIB_DIR)")
    endif()
endif()
