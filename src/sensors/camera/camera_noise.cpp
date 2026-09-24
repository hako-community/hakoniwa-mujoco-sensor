#include "sensors/camera/camera_noise.hpp"

#include <algorithm>
#include <cmath>

namespace hako::robots::sensor::camera
{
namespace
{
bool IsNone(const CameraNoiseConfig& config) { return config.type == "none"; }
}

bool ApplyImageNoise(ImageFrame& frame, const CameraNoiseConfig& config, std::mt19937& rng)
{
    if (IsNone(config)) return true;
    if (config.type != "gaussian" || config.stddev < 0.0 || frame.data.empty()) return false;
    std::normal_distribution<double> distribution(config.mean, config.stddev);
    for (auto& sample : frame.data) {
        const double noisy = static_cast<double>(sample) + distribution(rng);
        sample = static_cast<std::uint8_t>(std::lround(std::clamp(noisy, 0.0, 255.0)));
    }
    return true;
}

bool ApplyDepthNoise(DepthFrame& frame, const CameraNoiseConfig& config, std::mt19937& rng)
{
    if (IsNone(config)) return true;
    if (config.type != "gaussian" || config.stddev < 0.0) return false;
    std::normal_distribution<float> distribution(
        static_cast<float>(config.mean), static_cast<float>(config.stddev));
    for (auto& sample : frame.data) {
        if (std::isfinite(sample) && sample > 0.0F) sample += distribution(rng);
    }
    return true;
}
}
