cmake_minimum_required(VERSION 3.20)
project(sens006_wire_tests LANGUAGES CXX)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR OR CMAKE_BINARY_DIR MATCHES "[/\\\\]third_party[/\\\\]")
    message(FATAL_ERROR "Build outside sources and third_party")
endif()
set(HAKONIWA_INSTALL_PREFIX "" CACHE PATH "Core SDK with standard PDUs")
add_executable(sensor_health_probe codec_probe.cpp)
target_compile_features(sensor_health_probe PRIVATE cxx_std_20)
target_include_directories(sensor_health_probe PRIVATE ../../include
    ${HAKONIWA_INSTALL_PREFIX}/include/hakoniwa/pdu)
if(MSVC)
    target_compile_options(sensor_health_probe PRIVATE /utf-8)
endif()
enable_testing()
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME sensor_health_cross_language COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test_codec.py --probe $<TARGET_FILE:sensor_health_probe>
    --sdk ${HAKONIWA_INSTALL_PREFIX})
