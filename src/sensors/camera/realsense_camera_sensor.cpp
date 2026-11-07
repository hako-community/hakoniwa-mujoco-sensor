#include "sensors/camera/realsense/realsense_camera_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

#include "sensors/camera/camera_encoding_utils.hpp"
#include "sensors/camera/camera_noise.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"

namespace hako::robots::sensor::camera::realsense
{
namespace
{
constexpr double kFovMatchEpsilon = 1.0e-9;

PinholeIntrinsics IntrinsicsOf(int width, int height, double hfov_rad)
{
    const double fx = FocalLengthPx(width, hfov_rad);
    return {fx, fx, width * 0.5, height * 0.5};
}
}

RealSenseCameraSensor::RealSenseCameraSensor(
    std::shared_ptr<MujocoCameraRenderer> renderer,
    const std::string& depth_camera_name,
    const std::string& rgb_camera_name)
    : renderer_(std::move(renderer)),
      depth_camera_name_(depth_camera_name),
      rgb_camera_name_(rgb_camera_name.empty() ? depth_camera_name : rgb_camera_name)
{
    if (!renderer_) {
        throw std::invalid_argument("Renderer is null");
    }
}

RealSenseCameraSensor::~RealSenseCameraSensor() = default;

void RealSenseCameraSensor::SetMotionSource(std::shared_ptr<hako::robots::physics::IWorld> world)
{
    motion_world_ = std::move(world);
    fixed_motion_set_ = false;
}

void RealSenseCameraSensor::SetCameraMotion(const CameraMotion& motion)
{
    fixed_motion_ = motion;
    fixed_motion_set_ = true;
}

void RealSenseCameraSensor::SetMaterialSource(std::shared_ptr<hako::robots::physics::IWorld> world)
{
    material_world_ = std::move(world);
    materials_resolved_ = false;
}

void RealSenseCameraSensor::ResolveMaterials()
{
    materials_resolved_ = true;
    materials_by_id_.clear();
    const auto world = material_world_ ? material_world_ : motion_world_;
    const mjModel* m = world ? world->getModel() : nullptr;
    if (!m) {
        std::cerr << "Warning: RealSense texture model has no material source; "
                     "every pixel uses the default material" << std::endl;
        return;
    }
    std::vector<std::string> names;
    names.reserve(static_cast<std::size_t>(m->nmat));
    for (int i = 0; i < m->nmat; ++i) {
        const char* n = mj_id2name(m, mjOBJ_MATERIAL, i);
        names.emplace_back(n ? n : "");
    }
    materials_by_id_ = config_.texture_model.materials.Resolve(names);
}

void RealSenseCameraSensor::Reset()
{
    CameraSensorBase::Reset();
    post_.Reset();
}

bool RealSenseCameraSensor::LoadConfig(const std::string& path)
{
    RealSenseConfig config {};
    if (!LoadRealSenseConfigFromJson(path, config)) {
        return false;
    }
    return LoadConfig(config);
}

bool RealSenseCameraSensor::LoadConfig(const RealSenseConfig& config)
{
    std::vector<std::string> warnings;
    const std::string reason = ValidateConfig(config, &warnings);
    if (!reason.empty()) {
        std::cerr << "Invalid RealSense config (" << config.profile.model << "): " << reason << std::endl;
        return false;
    }
    for (const auto& w : warnings) {
        std::cerr << "Warning: RealSense " << config.profile.model << ": " << w << std::endl;
    }

    config_ = config;
    const auto& p = config_.profile;
    depth_params_ = MakeDepthModelParams(config_);
    edge_params_ = MakeEdgeEffectsParams(config_, depth_params_);
    last_edge_stats_ = EdgeEffectsStats {};
    texture_params_ = MakeTextureModelParams(config_);
    last_texture_stats_ = TextureStats {};
    materials_resolved_ = false;

    // Render config. The clip keeps a margin on both sides so the depth model
    // (not the GL clip) decides the invalid range: background beyond the far
    // plane reads back as exactly `far`, which must be > max_z to become invalid.
    depth_render_config_ = DepthCameraConfig {};
    depth_render_config_.frame_id = config_.DepthFrameId();
    depth_render_config_.update_rate = config_.depth.fps;
    depth_render_config_.horizontal_fov = p.DepthHfovRad();
    depth_render_config_.image = ImageConfig {config_.depth.width, config_.depth.height, config_.depth.format};
    depth_render_config_.clip = ClipConfig {depth_params_.min_z_m * 0.5, depth_params_.max_z_m * 1.1};

    // Output config: same FOV, size after decimation.
    depth_config_ = depth_render_config_;
    int out_w = config_.depth.width;
    int out_h = config_.depth.height;
    PostProcessedSize(config_.post_processing, out_w, out_h);
    depth_config_.image.width = out_w;
    depth_config_.image.height = out_h;
    depth_config_.intrinsics.fx = FocalLengthPx(out_w, p.DepthHfovRad());
    depth_config_.intrinsics.fy = depth_config_.intrinsics.fx;
    depth_config_.intrinsics.cx = out_w * 0.5;
    depth_config_.intrinsics.cy = out_h * 0.5;

    rgb_config_ = CameraConfig {};
    if (config_.RgbEnabled()) {
        rgb_config_.frame_id = config_.ColorFrameId();
        rgb_config_.update_rate = config_.rgb.fps;
        rgb_config_.horizontal_fov = p.RgbHfovRad();
        rgb_config_.image = ImageConfig {config_.rgb.width, config_.rgb.height, config_.rgb.format};
        rgb_config_.clip = ClipConfig {0.01, std::max(p.max_z_m, 10.0)};
        rgb_config_.intrinsics.fx = FocalLengthPx(config_.rgb.width, p.RgbHfovRad());
        rgb_config_.intrinsics.fy = rgb_config_.intrinsics.fx;
        rgb_config_.intrinsics.cx = config_.rgb.width * 0.5;
        rgb_config_.intrinsics.cy = config_.rgb.height * 0.5;
        rgb_config_.noise.type = config_.rgb_noise.type;
        rgb_config_.noise.mean = config_.rgb_noise.mean;
        rgb_config_.noise.stddev = config_.rgb_noise.stddev;
        rgb_config_.noise.seed = config_.rgb_noise.seed;
    }

    depth_rng_.seed(config_.depth_model.seed);
    edge_rng_.seed(config_.edge_effects.seed);
    texture_rng_.seed(config_.texture_model.seed);
    rgb_noise_rng_.seed(config_.rgb_noise.seed);
    post_.Reset();
    StartScheduler(config_.depth.fps);
    return true;
}

CameraMotion RealSenseCameraSensor::MotionOf(const std::string& camera_name) const
{
    if (fixed_motion_set_) return fixed_motion_;
    CameraMotion mo {};
    if (!motion_world_) return mo;
    const mjModel* m = motion_world_->getModel();
    const mjData* d = motion_world_->getData();
    if (!m || !d) return mo;
    const int id = mj_name2id(m, mjOBJ_CAMERA, camera_name.c_str());
    if (id < 0) return mo;
    mjtNum res[6];
    mj_objectVelocity(m, d, mjOBJ_CAMERA, id, res, 1);
    // MuJoCo camera frame (x right, y up, z back) -> optical (x right, y down, z fwd).
    mo.wx = res[0];
    mo.wy = -res[1];
    mo.wz = -res[2];
    mo.vx = res[3];
    mo.vy = -res[4];
    mo.vz = -res[5];
    return mo;
}

bool RealSenseCameraSensor::SharedRender() const
{
    return config_.RgbEnabled()
        && rgb_camera_name_ == depth_camera_name_
        && rgb_config_.image.width == depth_render_config_.image.width
        && rgb_config_.image.height == depth_render_config_.image.height
        && std::abs(rgb_config_.horizontal_fov - depth_render_config_.horizontal_fov) < kFovMatchEpsilon;
}

void RealSenseCameraSensor::ProcessRgb(ImageFrame& rgb, const CameraMotion& motion,
                                       const std::vector<float>* ref_depth)
{
    const double readout = config_.RgbReadoutS();
    if (config_.rolling_shutter.enabled && readout > 0.0 && !rgb.data.empty()) {
        ApplyRollingShutterImage(rgb.data, rgb.width, rgb.height, rgb.channels,
                                 IntrinsicsOf(rgb.width, rgb.height, rgb_config_.horizontal_fov),
                                 motion, readout, ref_depth);
    }
    if (!ApplyImageNoise(rgb, rgb_config_.noise, rgb_noise_rng_)) {
        ClearImageFrame(rgb);
    }
}

bool RealSenseCameraSensor::RenderDepth(DepthFrame& depth_out, ImageFrame* rgb_out)
{
    RawCameraFrame raw;
    const bool ok = renderer_->Render(
        depth_camera_name_,
        depth_render_config_.image.width,
        depth_render_config_.image.height,
        depth_render_config_.horizontal_fov,
        depth_render_config_.clip.near,
        depth_render_config_.clip.far,
        rgb_out != nullptr,
        true,
        raw);
    if (!ok) {
        ClearDepthFrame(depth_out);
        if (rgb_out) ClearImageFrame(*rgb_out);
        return false;
    }

    const CameraMotion motion = MotionOf(depth_camera_name_);
    std::vector<float> ideal_depth;   // reference depth for the RGB translation term

    if (!EncodeDepth(raw, depth_render_config_, depth_out)) {
        std::cerr << "Failed to encode RealSense depth frame" << std::endl;
        ClearDepthFrame(depth_out);
    } else {
        if (rgb_out) ideal_depth = depth_out.data;
        // 1. rolling shutter (exposure) -> 2. edge effects -> 3. stereo matching -> 4. SDK filters
        const double readout = config_.DepthReadoutS();
        if (config_.rolling_shutter.enabled && readout > 0.0) {
            ApplyRollingShutterDepth(depth_out.data, depth_out.width, depth_out.height,
                                     IntrinsicsOf(depth_out.width, depth_out.height,
                                                  depth_render_config_.horizontal_fov),
                                     motion, readout);
        }
        last_edge_stats_ = EdgeEffectsStats {};
        if (edge_params_.occlusion || edge_params_.flying_pixels) {
            last_edge_stats_ = ApplyEdgeEffects(depth_out.data, depth_out.width, depth_out.height,
                                                edge_params_, edge_rng_);
        }
        last_texture_stats_ = TextureStats {};
        const std::vector<float>* sigma_scale = nullptr;
        if (config_.texture_model.enabled) {
            if (!materials_resolved_) ResolveMaterials();
            last_texture_stats_ = ApplyTextureModel(
                depth_out.data, depth_out.width, depth_out.height, &raw.material_ids, materials_by_id_,
                config_.texture_model.materials.default_material, texture_params_, texture_rng_, sigma_scale_);
            sigma_scale = &sigma_scale_;
        }
        ApplyDepthModel(depth_out.data, depth_out.width, depth_out.height,
                        depth_params_, depth_rng_, &raw.material_ids, sigma_scale);
        int w = depth_out.width;
        int h = depth_out.height;
        post_.Apply(depth_out.data, w, h, depth_params_.fx_px, depth_params_.baseline_m,
                    config_.post_processing);
        depth_out.width = w;
        depth_out.height = h;
    }

    if (rgb_out) {
        if (!EncodeImage(raw, rgb_config_, *rgb_out)) {
            ClearImageFrame(*rgb_out);
        } else {
            ProcessRgb(*rgb_out, motion, ideal_depth.empty() ? nullptr : &ideal_depth);
        }
    }
    return true;
}

bool RealSenseCameraSensor::RenderRgb(ImageFrame& rgb_out)
{
    RawCameraFrame raw;
    const bool ok = renderer_->Render(
        rgb_camera_name_,
        rgb_config_.image.width,
        rgb_config_.image.height,
        rgb_config_.horizontal_fov,
        rgb_config_.clip.near,
        rgb_config_.clip.far,
        true,
        false,
        raw);
    if (!ok || !EncodeImage(raw, rgb_config_, rgb_out)) {
        ClearImageFrame(rgb_out);
        return false;
    }
    // Separate render: no aligned depth, rotation-only rolling shutter.
    ProcessRgb(rgb_out, MotionOf(rgb_camera_name_), nullptr);
    return !rgb_out.data.empty();
}

void RealSenseCameraSensor::CaptureDepth(DepthFrame& depth_out)
{
    RenderDepth(depth_out, nullptr);
}

void RealSenseCameraSensor::Capture(ImageFrame& rgb_out, DepthFrame& depth_out)
{
    if (!config_.RgbEnabled()) {
        ClearImageFrame(rgb_out);
        RenderDepth(depth_out, nullptr);
        return;
    }
    if (SharedRender()) {
        RenderDepth(depth_out, &rgb_out);
        return;
    }
    RenderDepth(depth_out, nullptr);
    RenderRgb(rgb_out);
}

}
