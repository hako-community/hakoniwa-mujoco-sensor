#pragma once

#include <cstdint>
#include <random>

#include "sensors/camera/camera_sensor.hpp"

namespace hako::robots::sensor::camera
{
    // Gaussian RGB noise is specified in 8-bit sample values; depth noise is
    // specified in metres. Invalid depth (NaN) is always preserved.
    bool ApplyImageNoise(ImageFrame& frame, const CameraNoiseConfig& config, std::mt19937& rng);
    bool ApplyDepthNoise(DepthFrame& frame, const CameraNoiseConfig& config, std::mt19937& rng);
}
