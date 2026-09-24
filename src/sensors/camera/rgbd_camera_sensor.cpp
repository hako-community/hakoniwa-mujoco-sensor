#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/camera_config_loader.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "sensors/camera/camera_encoding_utils.hpp"
#include "sensors/camera/camera_noise.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hako::robots::sensor::camera
{
namespace
{
constexpr double kFovMatchEpsilon = 1.0e-9;
}

RgbdCameraSensor::RgbdCameraSensor(std::shared_ptr<MujocoCameraRenderer> renderer, const std::string& camera_name)
    : renderer_(renderer), camera_name_(camera_name)
{
    if (!renderer) {
        throw std::invalid_argument("Renderer is null");
    }
}

RgbdCameraSensor::~RgbdCameraSensor() = default;

bool RgbdCameraSensor::LoadConfig(const std::string& path)
{
    RgbdCameraConfig config {};
    if (!LoadRgbdCameraConfigFromJson(path, config)) {
        return false;
    }
    if (!LoadConfig(config)) {
        std::cerr << "Failed to validate RgbdCameraConfig loaded from '" << path << "'" << std::endl;
        return false;
    }
    return true;
}

bool RgbdCameraSensor::LoadConfig(const RgbdCameraConfig& config)
{
    if (config.rgb.image.width <= 0 || config.rgb.image.height <= 0) {
        std::cerr << "Invalid RGB image size" << std::endl;
        return false;
    }
    if (config.rgb.update_rate <= 0.0) {
        std::cerr << "Invalid RGB update_rate: " << config.rgb.update_rate << std::endl;
        return false;
    }
    if (config.rgb.horizontal_fov <= 0.0 || config.rgb.horizontal_fov > M_PI) {
        std::cerr << "Invalid RGB horizontal_fov: " << config.rgb.horizontal_fov << std::endl;
        return false;
    }
    if (config.rgb.clip.near <= 0.0 || config.rgb.clip.far <= config.rgb.clip.near) {
        std::cerr << "Invalid RGB clip range: near=" << config.rgb.clip.near
                  << ", far=" << config.rgb.clip.far << std::endl;
        return false;
    }
    if (config.rgb.image.format != "R8G8B8" &&
        config.rgb.image.format != "B8G8R8" &&
        config.rgb.image.format != "L8")
    {
        std::cerr << "Invalid RGB image format: " << config.rgb.image.format << std::endl;
        return false;
    }
    if ((config.rgb.noise.type != "none" && config.rgb.noise.type != "gaussian") || config.rgb.noise.stddev < 0.0) {
        std::cerr << "Invalid RGB camera noise configuration" << std::endl;
        return false;
    }

    if (config.depth.image.width <= 0 || config.depth.image.height <= 0) {
        std::cerr << "Invalid depth image size" << std::endl;
        return false;
    }
    if (config.depth.update_rate <= 0.0) {
        std::cerr << "Invalid depth update_rate: " << config.depth.update_rate << std::endl;
        return false;
    }
    if (config.depth.horizontal_fov <= 0.0 || config.depth.horizontal_fov > M_PI) {
        std::cerr << "Invalid depth horizontal_fov: " << config.depth.horizontal_fov << std::endl;
        return false;
    }
    if (config.depth.clip.near <= 0.0 || config.depth.clip.far <= config.depth.clip.near) {
        std::cerr << "Invalid depth clip range: near=" << config.depth.clip.near
                  << ", far=" << config.depth.clip.far << std::endl;
        return false;
    }
    if (config.depth.image.format != "DEPTH_F32_M" &&
        config.depth.image.format != "DEPTH_U16_MM")
    {
        std::cerr << "Invalid depth image format: " << config.depth.image.format << std::endl;
        return false;
    }
    if ((config.depth.noise.type != "none" && config.depth.noise.type != "gaussian") || config.depth.noise.stddev < 0.0) {
        std::cerr << "Invalid depth camera noise configuration" << std::endl;
        return false;
    }
    if (config.depth_artifact.invalid_probability < 0.0
        || config.depth_artifact.invalid_probability > 1.0) {
        std::cerr << "Invalid material depth artifact probability" << std::endl;
        return false;
    }

    if (config.rgb.image.width != config.depth.image.width ||
        config.rgb.image.height != config.depth.image.height)
    {
        std::cerr << "RGB and depth image dimensions must match" << std::endl;
        return false;
    }
    if (std::abs(config.rgb.horizontal_fov - config.depth.horizontal_fov) > kFovMatchEpsilon) {
        std::cerr << "RGB and depth horizontal_fov must match" << std::endl;
        return false;
    }
    if (config.rgb.update_rate != config.depth.update_rate) {
        std::cerr << "Warning: RGB and depth update_rate differ. Using RGB update_rate." << std::endl;
    }

    config_ = config;
    rgb_noise_rng_.seed(config_.rgb.noise.seed);
    depth_noise_rng_.seed(config_.depth.noise.seed);
    depth_artifact_rng_.seed(config_.depth_artifact.seed);
    StartScheduler(config_.rgb.update_rate);
    return true;
}

const RgbdCameraConfig& RgbdCameraSensor::GetConfig() const
{
    return config_;
}

void RgbdCameraSensor::Capture(ImageFrame& rgb_out, DepthFrame& depth_out)
{
    RawCameraFrame raw;
    const bool success = renderer_->Render(
        camera_name_,
        config_.rgb.image.width,
        config_.rgb.image.height,
        config_.rgb.horizontal_fov,
        config_.depth.clip.near,
        config_.depth.clip.far,
        true,
        true,
        raw);

    if (!success) {
        ClearImageFrame(rgb_out);
        ClearDepthFrame(depth_out);
        return;
    }

    if (!EncodeImage(raw, config_.rgb, rgb_out)) {
        std::cerr << "Failed to encode RGB frame" << std::endl;
        ClearImageFrame(rgb_out);
    } else if (!ApplyImageNoise(rgb_out, config_.rgb.noise, rgb_noise_rng_)) {
        ClearImageFrame(rgb_out);
    }
    if (!EncodeDepth(raw, config_.depth, depth_out)) {
        std::cerr << "Failed to encode depth frame" << std::endl;
        ClearDepthFrame(depth_out);
    } else if (!ApplyDepthNoise(depth_out, config_.depth.noise, depth_noise_rng_)) {
        ClearDepthFrame(depth_out);
    } else if (config_.depth_artifact.enabled) {
        std::bernoulli_distribution invalidate(config_.depth_artifact.invalid_probability);
        const auto targeted = [&](int material_id) {
            return std::find(config_.depth_artifact.material_ids.begin(),
                             config_.depth_artifact.material_ids.end(), material_id)
                   != config_.depth_artifact.material_ids.end();
        };
        const auto count = std::min(depth_out.data.size(), raw.material_ids.size());
        for (std::size_t i = 0; i < count; ++i) {
            if (targeted(raw.material_ids[i]) && invalidate(depth_artifact_rng_)) {
                depth_out.data[i] = 0.0F;
            }
        }
    }
}

}
