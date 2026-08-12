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
# カメラは GLFW / OpenGL を要求するため既定 OFF。
# カメラを使わない利用者のビルドを壊さないこと（IMPL_PLAN §4.4 の受け入れ基準）。
option(HAKO_SENSOR_BUILD_CAMERA "Build camera sensors (requires GLFW/OpenGL)" OFF)

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
# MuJoCo の場所。ヘッダと libmujoco.so は**必ず同じ root から組で**選ぶこと。
# 版が食い違うと構造体レイアウトがずれ、ビルドは通って実行時に SEGV する
# （実際に 3.10.0-dev ヘッダ + 3.9.0 ライブラリで mujoco_radar_integration_test と
#  lidar_a2_demo が落ちた）。そこで探索順は
#   1) include と lib が揃っている root（＝リリース tarball。multi_build.bash の
#      find_mujoco_root と同じ考え方）
#   2) ヘッダしか無い場所（未ビルドのソースチェックアウト等。コンパイルは通るので
#      ライブラリを要らない利用者のために残すが、優先はしない）
if(NOT MUJOCO_INCLUDE_DIR)
    foreach(_root
        "${CMAKE_CURRENT_SOURCE_DIR}/../../.cache/deps/mujoco_bin-src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../.cache/deps/mujoco_bin-src"
        "/usr/local"
        "/opt/mujoco")
        if(EXISTS "${_root}/include/mujoco/mujoco.h" AND EXISTS "${_root}/lib/libmujoco.so")
            # lib は下の「ヘッダの隣を見る」ロジックが同じ root から拾う。
            set(MUJOCO_INCLUDE_DIR "${_root}/include")
            break()
        endif()
    endforeach()
endif()
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

# nlohmann/json の場所。-DNLOHMANN_JSON_INCLUDE_DIR で明示指定するのが基本で、
# 未指定のときだけ下の順で探す。
#   1) 親リポの third_party/ に submodule として並んでいる場合（hakoniwa-humanoid の構成）
#   2) システムに入っている場合（Debian/Ubuntu の nlohmann-json3-dev など）
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    foreach(_cand
        "${CMAKE_CURRENT_SOURCE_DIR}/../nlohmann-json/single_include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/nlohmann-json/single_include"
        "/usr/local/include"
        "/usr/include")
        if(EXISTS "${_cand}/nlohmann/json.hpp")
            set(NLOHMANN_JSON_INCLUDE_DIR "${_cand}")
            break()
        endif()
    endforeach()
endif()
if(NOT NLOHMANN_JSON_INCLUDE_DIR)
    message(WARNING
        "nlohmann/json.hpp が見つかりません。JSON 設定を読む機能はビルドできません。\n"
        "  -DNLOHMANN_JSON_INCLUDE_DIR=<nlohmann/json.hpp のある include ディレクトリ> を指定してください。")
endif()

set(HAKO_PDU_TYPES_INCLUDE_DIR "${HAKONIWA_INSTALL_PREFIX}/include/hakoniwa/pdu")

message(STATUS "MUJOCO_INCLUDE_DIR        = ${MUJOCO_INCLUDE_DIR}")
message(STATUS "NLOHMANN_JSON_INCLUDE_DIR = ${NLOHMANN_JSON_INCLUDE_DIR}")
message(STATUS "HAKO_PDU_TYPES_INCLUDE_DIR= ${HAKO_PDU_TYPES_INCLUDE_DIR}")

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
set(_camera_srcs "")
if(HAKO_SENSOR_BUILD_CAMERA)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    set(_camera_srcs
        src/sensors/camera/camera_config_loader.cpp
        src/sensors/camera/camera_encoding_utils.cpp
        src/sensors/camera/camera_sensor.cpp
        src/sensors/camera/depth_camera_sensor.cpp
        src/sensors/camera/rgbd_camera_sensor.cpp
        src/sensors/camera/stereo_camera_sensor.cpp
        src/sensors/camera/mujoco_camera_renderer.cpp
        src/sensors/camera/image_frame_writer.cpp
    )
endif()

add_library(hako_mujoco_sensor STATIC
    ${_camera_srcs}
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

# src/ は非公開ヘッダ（camera_encoding_utils.hpp 等）の解決用
target_include_directories(hako_mujoco_sensor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
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
    target_include_directories(radar_math_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${HAKO_MUJOCO_RUNTIME_DIR}/include)
    target_include_directories(radar_math_test SYSTEM PRIVATE
        ${MUJOCO_INCLUDE_DIR}
        ${NLOHMANN_JSON_INCLUDE_DIR}
    )
    add_test(NAME radar_math_test COMMAND radar_math_test)

    # Scan timestamps across the range sensors + their PDU headers.
    # Needs the PDU type headers (it asserts on the converted messages).
    if(EXISTS "${HAKO_PDU_TYPES_INCLUDE_DIR}/builtin_interfaces")
        add_executable(sensor_stamp_test
            tests/sensor_stamp_test.cpp
            src/sensors/lidar/lidar_scan_sensor.cpp
            src/sensors/noise/range_noise.cpp
            src/sensors/noise/axis_noise.cpp
        )
        target_compile_features(sensor_stamp_test PRIVATE cxx_std_20)
        target_include_directories(sensor_stamp_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${HAKO_MUJOCO_RUNTIME_DIR}/include)
        target_include_directories(sensor_stamp_test SYSTEM PRIVATE
            ${MUJOCO_INCLUDE_DIR} ${NLOHMANN_JSON_INCLUDE_DIR} ${HAKO_PDU_TYPES_INCLUDE_DIR})
        add_test(NAME sensor_stamp_test COMMAND sensor_stamp_test)
    else()
        message(STATUS "PDU types missing -- skipping sensor_stamp_test")
    endif()

    # #5: the SHM channel layout is derived from the pdudef, not hand-written.
    # nlohmann/json only -- no MuJoCo, no PDU types, no running master.
    add_executable(pdudef_channels_test tests/pdudef_channels_test.cpp)
    target_compile_features(pdudef_channels_test PRIVATE cxx_std_20)
    target_include_directories(pdudef_channels_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_include_directories(pdudef_channels_test SYSTEM PRIVATE
        ${NLOHMANN_JSON_INCLUDE_DIR})
    add_test(NAME pdudef_channels_test COMMAND pdudef_channels_test)

    # #12: the sensor's own angular velocity -> the mount's lever-arm term.
    # Includes runtime/sensor_runtime.hpp for MakeState, which pulls the PDU
    # converters, hence the same guard. MakeState is inline and no SensorRuntime
    # is constructed, so nothing here links MuJoCo.
    if(EXISTS "${HAKO_PDU_TYPES_INCLUDE_DIR}/builtin_interfaces")
        add_executable(sensor_mount_velocity_test
            tests/sensor_mount_velocity_test.cpp
            src/sensors/radar/radar_sensor.cpp
            src/sensors/noise/range_noise.cpp
            src/sensors/noise/axis_noise.cpp
        )
        target_compile_features(sensor_mount_velocity_test PRIVATE cxx_std_20)
        target_include_directories(sensor_mount_velocity_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${HAKO_MUJOCO_RUNTIME_DIR}/include)
        target_include_directories(sensor_mount_velocity_test SYSTEM PRIVATE
            ${MUJOCO_INCLUDE_DIR} ${NLOHMANN_JSON_INCLUDE_DIR} ${HAKO_PDU_TYPES_INCLUDE_DIR})
        add_test(NAME sensor_mount_velocity_test COMMAND sensor_mount_velocity_test)
    else()
        message(STATUS "PDU types missing -- skipping sensor_mount_velocity_test")
    endif()

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

    # ヘッダとライブラリの版が一致しているか検査する。
    # 不一致でも**コンパイルもリンクも通ってしまい**、実行時に mjModel/mjData の
    # レイアウト差で SEGV する。この環境には未ビルドの 3.10.0-dev ソースツリーと
    # 3.9.0 のリリース版が同居しており、-DMUJOCO_LIB_DIR= だけ渡すと実際に踏む。
    # 静かに落ちるくらいなら configure 時点で止める。
    if(MUJOCO_LIBRARY AND EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
        file(STRINGS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h" _mj_ver_line
             REGEX "^#define +mjVERSION_HEADER +[0-9]+")
        if(_mj_ver_line MATCHES "([0-9]+)$")
            set(_mj_v "${CMAKE_MATCH_1}")
            math(EXPR _mj_major "${_mj_v} / 1000000")
            math(EXPR _mj_minor "(${_mj_v} / 1000) % 1000")
            math(EXPR _mj_patch "${_mj_v} % 1000")
            set(_mj_hdr_ver "${_mj_major}.${_mj_minor}.${_mj_patch}")
            # 実体は libmujoco.so.<version> を指す symlink。
            get_filename_component(_mj_real "${MUJOCO_LIBRARY}" REALPATH)
            if(_mj_real MATCHES "libmujoco\\.so\\.([0-9]+\\.[0-9]+\\.[0-9]+)")
                set(_mj_lib_ver "${CMAKE_MATCH_1}")
                if(NOT _mj_hdr_ver VERSION_EQUAL _mj_lib_ver)
                    message(FATAL_ERROR
                        "MuJoCo のヘッダとライブラリの版が一致しません。\n"
                        "  header ${_mj_hdr_ver}: ${MUJOCO_INCLUDE_DIR}\n"
                        "  lib    ${_mj_lib_ver}: ${_mj_real}\n"
                        "このままビルドすると実行時に SEGV します。"
                        "include と lib を同じ root から組で指定してください:\n"
                        "  cmake -S . -B build -DMUJOCO_INCLUDE_DIR=<root>/include "
                        "-DMUJOCO_LIB_DIR=<root>/lib")
                endif()
                message(STATUS "MuJoCo version check: header/lib とも ${_mj_lib_ver}")
            endif()
        endif()
    endif()

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
        target_include_directories(mujoco_radar_integration_test PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${HAKO_MUJOCO_RUNTIME_DIR}/include)
        target_include_directories(mujoco_radar_integration_test SYSTEM PRIVATE
            ${MUJOCO_INCLUDE_DIR} ${NLOHMANN_JSON_INCLUDE_DIR})
        target_link_libraries(mujoco_radar_integration_test PRIVATE ${MUJOCO_LIBRARY})
        add_test(NAME mujoco_radar_integration_test COMMAND mujoco_radar_integration_test)

        # M2 A-2 LiDAR demo: sense a simenv-data generated world (env.xml).
        add_executable(lidar_a2_demo
            examples/envsim_sensor_a2/lidar_a2_demo.cpp
            src/sensors/backend/mujoco_ray_caster.cpp
            src/sensors/lidar/lidar_scan_sensor.cpp
        )
        target_compile_features(lidar_a2_demo PRIVATE cxx_std_20)
        target_include_directories(lidar_a2_demo PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${HAKO_MUJOCO_RUNTIME_DIR}/include)
        target_include_directories(lidar_a2_demo SYSTEM PRIVATE ${MUJOCO_INCLUDE_DIR})
        target_link_libraries(lidar_a2_demo PRIVATE ${MUJOCO_LIBRARY})
        add_test(NAME lidar_a2_demo COMMAND lidar_a2_demo
            "${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-simenv-data/examples/sensor_envs/simple_room/generated/env.xml")

        # Manifest-loader check: proves sensor_runtime.hpp actually reads the
        # radar-equation and RCS keys. radar_math_test covers the formulas, but a
        # mistyped key there would silently disable the falloff instead of failing.
        if(NLOHMANN_JSON_INCLUDE_DIR AND EXISTS "${HAKO_PDU_TYPES_INCLUDE_DIR}/builtin_interfaces")
            add_executable(check_radar_equation
                examples/envsim_sensor_a2/check_radar_equation.cpp
                src/sensors/radar/radar_sensor.cpp
                src/sensors/lidar/lidar3d_sensor.cpp
                src/sensors/lidar/lidar_scan_sensor.cpp
                src/sensors/lidar/lidar_2d_sensor.cpp
                src/sensors/noise/range_noise.cpp
                src/sensors/noise/axis_noise.cpp
            )
            target_compile_features(check_radar_equation PRIVATE cxx_std_20)
            target_include_directories(check_radar_equation PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${HAKO_MUJOCO_RUNTIME_DIR}/include)
            target_include_directories(check_radar_equation SYSTEM PRIVATE
                ${MUJOCO_INCLUDE_DIR} ${NLOHMANN_JSON_INCLUDE_DIR} ${HAKO_PDU_TYPES_INCLUDE_DIR})
            target_link_libraries(check_radar_equation PRIVATE ${MUJOCO_LIBRARY})
            add_test(NAME check_radar_equation COMMAND check_radar_equation)
        else()
            message(STATUS "nlohmann/json or PDU types missing -- skipping check_radar_equation")
        endif()
    else()
        message(STATUS "libmujoco not found -- skipping mujoco_radar_integration_test (set MUJOCO_LIB_DIR)")
    endif()
endif()
