#pragma once

// RealSense camera configuration and its JSON mapping.
//
// Pure: nlohmann + profile table, no MuJoCo/GL, so tests and tools can load and
// validate a config without a renderer. Example: config/realsense/*.json
//
// {
//   "model": "D435i",
//   "frame_id": "camera",
//   "depth": { "width": 848, "height": 480, "fps": 30, "format": "DEPTH_F32_M" },
//   "rgb":   { "enabled": true, "width": 1280, "height": 720, "fps": 30, "format": "R8G8B8",
//              "noise": { "type": "gaussian", "mean": 0.0, "stddev": 2.0, "seed": 1 } },
//   "depth_model": { "enabled": true, "noise_model": "typical", "noise_scale": 1.0,
//                    "min_z_mode": "disparity", "disparity_shift_px": 0,
//                    "left_invalid_band": true, "subpixel_step_px": 0.0, "depth_unit_m": 0.001,
//                    "invalid_probability": 0.0, "invalid_material_ids": [],
//                    "material_invalid_probability": 1.0, "seed": 0 },
//   "rolling_shutter": { "enabled": true, "depth_readout_s": null, "rgb_readout_s": null },
//   "edge_effects": { "enabled": false, "occlusion": true,
//                     "flying_pixels": { "enabled": true, "window_px": 7, "jump_ratio": 0.05,
//                                        "probability": 0.5 }, "seed": 0 },
//   "post_processing": { "enabled": false, "decimation": {...}, "threshold": {...},
//                        "spatial": {...}, "temporal": {...}, "hole_filling": {...} },
//   "imu": { "enabled": true, "accel_rate_hz": 250, "gyro_rate_hz": 400,
//            "bias_mode": "calibrated", "seed": 0 },
//   "overrides": { "min_z_m": 0.3, "max_z_m": 6.0 }
// }

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_depth_model.hpp"
#include "sensors/camera/realsense/realsense_edge_effects.hpp"
#include "sensors/camera/realsense/realsense_imu_model.hpp"
#include "sensors/camera/realsense/realsense_post_processing.hpp"
#include "sensors/camera/realsense/realsense_profile.hpp"

namespace hako::robots::sensor::camera::realsense
{
    struct StreamConfig
    {
        bool enabled {true};
        int width {0};
        int height {0};
        double fps {30.0};
        std::string format {};
    };

    struct RgbNoiseConfig
    {
        std::string type {"none"};
        double mean {0.0};
        double stddev {0.0};
        std::uint32_t seed {0};
    };

    struct DepthModelConfig
    {
        bool enabled {true};
        // "typical": disparity RMS = profile.subpixel_rms_px (Intel tuning paper, 0.08 px)
        // "spec"   : catalogue accuracy (e.g. 2% @ 2 m) used as 1 sigma (worst case)
        std::string noise_model {"typical"};
        double noise_scale {1.0};        // multiplies the disparity sigma
        // "disparity": MinZ = fx*B/(max_disparity + shift) -> scales with resolution
        // "fixed"    : MinZ = profile.min_z_m (catalogue value)
        std::string min_z_mode {"disparity"};
        double disparity_shift_px {0.0}; // D4 "disparity shift": nearer MinZ, finite max range
        bool left_invalid_band {true};
        double subpixel_step_px {0.0};
        double depth_unit_m {0.001};     // Z16 default depth unit
        double invalid_probability {0.0};
        std::vector<int> invalid_material_ids {};
        double material_invalid_probability {1.0};
        std::uint32_t seed {0};
    };

    struct RollingShutterConfig
    {
        bool enabled {true};             // applied only to rolling-shutter streams with a motion source
        double depth_readout_s {-1.0};   // < 0: profile value
        double rgb_readout_s {-1.0};
    };

    // RS-1: occlusion shadow + flying pixels at depth edges (default off).
    // window_px / jump_ratio / probability are assumptions (D4 window not published).
    struct FlyingPixelsConfig
    {
        bool enabled {true};
        double window_px {7.0};          // matching window at 848 px width
        double jump_ratio {0.05};        // neighbour step > ratio * nearer depth = discontinuity
        double probability {0.5};
    };

    struct EdgeEffectsConfig
    {
        bool enabled {false};
        bool occlusion {true};
        FlyingPixelsConfig flying_pixels {};
        std::uint32_t seed {0};
    };

    struct RealSenseConfig
    {
        RealSenseProfile profile {};     // built-in profile with overrides applied
        std::string frame_id {"camera"}; // base name (ROS realsense2_camera style)
        StreamConfig depth {true, 848, 480, 30.0, "DEPTH_F32_M"};
        StreamConfig rgb {true, 0, 0, 30.0, "R8G8B8"};  // 0 = profile default size
        RgbNoiseConfig rgb_noise {};
        DepthModelConfig depth_model {};
        RollingShutterConfig rolling_shutter {};
        EdgeEffectsConfig edge_effects {};
        PostProcessingConfig post_processing {};
        ImuModelConfig imu {};

        double DepthReadoutS() const
        {
            return rolling_shutter.depth_readout_s >= 0.0 ? rolling_shutter.depth_readout_s : profile.rolling_readout_s;
        }
        double RgbReadoutS() const
        {
            return rolling_shutter.rgb_readout_s >= 0.0 ? rolling_shutter.rgb_readout_s : profile.rgb_rolling_readout_s;
        }
        bool ImuEnabled() const { return profile.has_imu && imu.enabled; }

        std::string DepthFrameId() const { return frame_id + "_depth_optical_frame"; }
        std::string ColorFrameId() const { return frame_id + "_color_optical_frame"; }
        bool RgbEnabled() const { return profile.has_rgb && rgb.enabled; }
    };

    // Default config for a built-in model. Returns false for an unknown model.
    inline bool MakeDefaultConfig(const std::string& model, RealSenseConfig& out)
    {
        auto p = FindProfile(model);
        if (!p) return false;
        RealSenseConfig c {};
        c.profile = *p;
        if (p->depth_max_width * 3 == p->depth_max_height * 4) {   // 4:3 sensor (D585 Pro)
            c.depth.width = 640;
            c.depth.height = 480;
        }
        c.rgb.width = p->rgb_default_width;
        c.rgb.height = p->rgb_default_height;
        c.rgb.fps = std::min(30.0, p->rgb_max_fps);
        c.rgb.enabled = p->has_rgb;
        c.imu.enabled = p->has_imu;
        c.imu.accel_rate_hz = p->imu.accel_rates_hz.empty() ? 250.0 : p->imu.accel_rates_hz.back();
        c.imu.gyro_rate_hz = p->imu.gyro_rates_hz.empty() ? 400.0 : p->imu.gyro_rates_hz.back();
        out = c;
        return true;
    }

    // Depth model parameters for the configured depth stream.
    inline DepthModelParams MakeDepthModelParams(const RealSenseConfig& c)
    {
        DepthModelParams m {};
        m.fx_px = FocalLengthPx(c.depth.width, c.profile.DepthHfovRad());
        m.baseline_m = c.profile.baseline_m;
        m.min_z_m = c.profile.min_z_m;
        m.max_z_m = c.profile.max_z_m;
        const double base_sigma = c.depth_model.noise_model == "spec"
            ? DisparitySigmaFromSpec(c.profile) : c.profile.subpixel_rms_px;
        m.disparity_sigma_px = base_sigma * c.depth_model.noise_scale;
        if (c.depth_model.min_z_mode == "disparity" && c.profile.max_disparity_px > 0.0) {
            const double fb = m.fx_px * m.baseline_m;
            m.min_z_m = fb / (c.profile.max_disparity_px + c.depth_model.disparity_shift_px);
            if (c.depth_model.disparity_shift_px > 0.0) {
                m.max_z_m = std::min(m.max_z_m, fb / c.depth_model.disparity_shift_px);
            }
        }
        m.subpixel_step_px = c.depth_model.subpixel_step_px;
        m.depth_unit_m = c.depth_model.depth_unit_m;
        m.left_invalid_band = c.depth_model.left_invalid_band;
        m.invalid_probability = c.depth_model.invalid_probability;
        m.invalid_material_ids = c.depth_model.invalid_material_ids;
        m.material_invalid_probability = c.depth_model.material_invalid_probability;
        if (!c.depth_model.enabled) {
            // Ideal depth inside the clip range only.
            m.disparity_sigma_px = 0.0;
            m.subpixel_step_px = 0.0;
            m.depth_unit_m = 0.0;
            m.left_invalid_band = false;
            m.invalid_probability = 0.0;
            m.invalid_material_ids.clear();
        }
        return m;
    }

    // Edge-effect parameters for the configured depth stream (render size).
    inline EdgeEffectsParams MakeEdgeEffectsParams(const RealSenseConfig& c, const DepthModelParams& m)
    {
        EdgeEffectsParams e {};
        e.fx_px = m.fx_px;
        e.baseline_m = m.baseline_m;
        e.occlusion = c.edge_effects.enabled && c.edge_effects.occlusion;
        const auto& f = c.edge_effects.flying_pixels;
        e.flying_pixels = c.edge_effects.enabled && f.enabled;
        e.window_radius_px = FlyingWindowRadiusPx(f.window_px, c.depth.width);
        e.jump_ratio = f.jump_ratio;
        e.probability = f.probability;
        e.min_z_m = m.min_z_m;
        e.max_z_m = m.max_z_m;
        return e;
    }

    // Returns an empty string when valid, otherwise the reason.
    // Requests beyond the catalogue limits (resolution / fps) are allowed but
    // reported through `warnings`.
    inline std::string ValidateConfig(const RealSenseConfig& c, std::vector<std::string>* warnings = nullptr)
    {
        const auto& p = c.profile;
        auto warn = [&](const std::string& w) { if (warnings) warnings->push_back(w); };
        if (p.depth_hfov_deg <= 0.0 || p.depth_hfov_deg >= 180.0) return "invalid depth_hfov_deg";
        if (p.min_z_m <= 0.0 || p.max_z_m <= p.min_z_m) return "invalid min_z_m/max_z_m";
        if (p.baseline_m <= 0.0) return "invalid baseline_m";
        if (p.accuracy_ratio < 0.0 || p.accuracy_ref_distance_m <= 0.0 || p.accuracy_ref_width <= 0)
            return "invalid accuracy spec";
        if (c.depth.width <= 0 || c.depth.height <= 0) return "invalid depth size";
        if (c.depth.fps <= 0.0) return "invalid depth fps";
        if (c.depth.format != "DEPTH_F32_M" && c.depth.format != "DEPTH_U16_MM")
            return "invalid depth format: " + c.depth.format;
        const auto& d = c.depth_model;
        if (d.noise_model != "typical" && d.noise_model != "spec") return "invalid noise_model: " + d.noise_model;
        if (d.min_z_mode != "disparity" && d.min_z_mode != "fixed") return "invalid min_z_mode: " + d.min_z_mode;
        if (d.disparity_shift_px < 0.0 || p.subpixel_rms_px < 0.0) return "invalid disparity parameters";
        const auto& pp = c.post_processing;
        if (pp.decimation.magnitude < 1 || pp.decimation.magnitude > 8) return "invalid decimation magnitude";
        if (pp.threshold.min_m < 0.0 || pp.threshold.max_m <= pp.threshold.min_m) return "invalid threshold range";
        if (pp.spatial.alpha <= 0.0 || pp.spatial.alpha > 1.0 || pp.spatial.iterations < 1 || pp.spatial.delta <= 0.0
            || pp.spatial.holes_fill < 0 || pp.spatial.holes_fill > 5) return "invalid spatial filter";
        if (pp.temporal.alpha < 0.0 || pp.temporal.alpha > 1.0 || pp.temporal.delta <= 0.0
            || pp.temporal.persistency < 0 || pp.temporal.persistency > 8) return "invalid temporal filter";
        if (pp.hole_filling.mode < 0 || pp.hole_filling.mode > 2) return "invalid hole filling mode";
        const auto& fp = c.edge_effects.flying_pixels;
        if (fp.window_px < 1.0 || fp.jump_ratio <= 0.0 || fp.probability < 0.0 || fp.probability > 1.0)
            return "invalid edge_effects.flying_pixels";
        if (c.ImuEnabled()) {
            if (c.imu.accel_rate_hz <= 0.0 || c.imu.gyro_rate_hz <= 0.0) return "invalid imu rate";
            if (c.imu.bias_mode != "none" && c.imu.bias_mode != "calibrated" && c.imu.bias_mode != "uncalibrated")
                return "invalid imu bias_mode: " + c.imu.bias_mode;
            if (c.imu.noise_scale < 0.0) return "invalid imu noise_scale";
            auto listed = [](const std::vector<double>& v, double r) {
                for (double x : v) if (std::fabs(x - r) < 1.0) return true;
                return false;
            };
            if (!listed(p.imu.accel_rates_hz, c.imu.accel_rate_hz)) warn("accel rate is not a catalogue rate of " + p.model);
            if (!listed(p.imu.gyro_rates_hz, c.imu.gyro_rate_hz)) warn("gyro rate is not a catalogue rate of " + p.model);
        }
        if (c.imu.enabled && !p.has_imu) warn(p.model + " has no IMU; imu disabled");
        if (d.noise_scale < 0.0 || d.subpixel_step_px < 0.0 || d.depth_unit_m < 0.0)
            return "invalid depth_model (negative value)";
        if (d.invalid_probability < 0.0 || d.invalid_probability > 1.0
            || d.material_invalid_probability < 0.0 || d.material_invalid_probability > 1.0)
            return "invalid depth_model probability";
        if (c.RgbEnabled()) {
            if (p.rgb_hfov_deg <= 0.0 || p.rgb_hfov_deg >= 180.0) return "invalid rgb_hfov_deg";
            if (c.rgb.width <= 0 || c.rgb.height <= 0) return "invalid rgb size";
            if (c.rgb.fps <= 0.0) return "invalid rgb fps";
            if (c.rgb.format != "R8G8B8" && c.rgb.format != "B8G8R8" && c.rgb.format != "L8")
                return "invalid rgb format: " + c.rgb.format;
            if (c.rgb_noise.type != "none" && c.rgb_noise.type != "gaussian") return "invalid rgb noise type";
            if (c.rgb_noise.stddev < 0.0) return "invalid rgb noise stddev";
            if (c.rgb.fps != c.depth.fps)
                warn("rgb fps differs from depth fps; both streams are captured at the depth rate");
            if (c.rgb.fps > p.rgb_max_fps) warn("rgb fps exceeds the catalogue maximum of " + p.model);
        }
        if (c.rgb.enabled && !p.has_rgb) warn(p.model + " has no RGB sensor; rgb stream disabled");
        if (c.depth.width > p.depth_max_width || c.depth.height > p.depth_max_height)
            warn("depth resolution exceeds the catalogue maximum of " + p.model);
        if (c.depth.fps > p.depth_max_fps) warn("depth fps exceeds the catalogue maximum of " + p.model);
        if (p.preliminary) warn(p.model + " profile is preliminary: " + p.note);
        return {};
    }

    namespace detail
    {
        template <typename T>
        inline void ReadIf(const nlohmann::json& j, const char* key, T& out)
        {
            if (j.contains(key) && !j.at(key).is_null()) out = j.at(key).get<T>();
        }

        inline void ReadStream(const nlohmann::json& j, StreamConfig& s)
        {
            ReadIf(j, "enabled", s.enabled);
            ReadIf(j, "width", s.width);
            ReadIf(j, "height", s.height);
            ReadIf(j, "fps", s.fps);
            ReadIf(j, "format", s.format);
        }

        inline void ApplyOverrides(const nlohmann::json& o, RealSenseProfile& p)
        {
            ReadIf(o, "depth_hfov_deg", p.depth_hfov_deg);
            ReadIf(o, "baseline_m", p.baseline_m);
            ReadIf(o, "accuracy_ratio", p.accuracy_ratio);
            ReadIf(o, "accuracy_ref_distance_m", p.accuracy_ref_distance_m);
            ReadIf(o, "accuracy_ref_width", p.accuracy_ref_width);
            ReadIf(o, "min_z_m", p.min_z_m);
            ReadIf(o, "max_z_m", p.max_z_m);
            ReadIf(o, "rgb_hfov_deg", p.rgb_hfov_deg);
            ReadIf(o, "has_rgb", p.has_rgb);
            ReadIf(o, "has_imu", p.has_imu);
            ReadIf(o, "subpixel_rms_px", p.subpixel_rms_px);
            ReadIf(o, "max_disparity_px", p.max_disparity_px);
            ReadIf(o, "rolling_readout_s", p.rolling_readout_s);
            ReadIf(o, "rgb_rolling_readout_s", p.rgb_rolling_readout_s);
            if (o.contains("imu") && o.at("imu").is_object()) {
                const auto& i = o.at("imu");
                ReadIf(i, "chip", p.imu.chip);
                ReadIf(i, "accel_range_g", p.imu.accel_range_g);
                ReadIf(i, "gyro_range_dps", p.imu.gyro_range_dps);
                ReadIf(i, "accel_bits", p.imu.accel_bits);
                ReadIf(i, "gyro_bits", p.imu.gyro_bits);
                ReadIf(i, "accel_noise_density_ug", p.imu.accel_noise_density_ug);
                ReadIf(i, "gyro_noise_density_dps", p.imu.gyro_noise_density_dps);
                ReadIf(i, "accel_zero_offset_mg", p.imu.accel_zero_offset_mg);
                ReadIf(i, "gyro_zero_offset_dps", p.imu.gyro_zero_offset_dps);
                ReadIf(i, "gyro_bias_instability_dph", p.imu.gyro_bias_instability_dph);
            }
        }

        inline void ReadPostProcessing(const nlohmann::json& j, PostProcessingConfig& c)
        {
            ReadIf(j, "enabled", c.enabled);
            ReadIf(j, "disparity_domain", c.disparity_domain);
            if (j.contains("decimation") && j.at("decimation").is_object()) {
                const auto& d = j.at("decimation");
                ReadIf(d, "enabled", c.decimation.enabled);
                ReadIf(d, "magnitude", c.decimation.magnitude);
            }
            if (j.contains("threshold") && j.at("threshold").is_object()) {
                const auto& d = j.at("threshold");
                ReadIf(d, "enabled", c.threshold.enabled);
                ReadIf(d, "min_m", c.threshold.min_m);
                ReadIf(d, "max_m", c.threshold.max_m);
            }
            if (j.contains("spatial") && j.at("spatial").is_object()) {
                const auto& d = j.at("spatial");
                ReadIf(d, "enabled", c.spatial.enabled);
                ReadIf(d, "iterations", c.spatial.iterations);
                ReadIf(d, "alpha", c.spatial.alpha);
                ReadIf(d, "delta", c.spatial.delta);
                ReadIf(d, "holes_fill", c.spatial.holes_fill);
            }
            if (j.contains("temporal") && j.at("temporal").is_object()) {
                const auto& d = j.at("temporal");
                ReadIf(d, "enabled", c.temporal.enabled);
                ReadIf(d, "alpha", c.temporal.alpha);
                ReadIf(d, "delta", c.temporal.delta);
                ReadIf(d, "persistency", c.temporal.persistency);
            }
            if (j.contains("hole_filling") && j.at("hole_filling").is_object()) {
                const auto& d = j.at("hole_filling");
                ReadIf(d, "enabled", c.hole_filling.enabled);
                ReadIf(d, "mode", c.hole_filling.mode);
            }
        }
    }

    // Parses a config object. Unknown keys are ignored; missing keys keep the
    // model defaults. On error returns false and writes the reason to `error`.
    inline bool RealSenseConfigFromJson(const nlohmann::json& root, RealSenseConfig& out, std::string& error)
    {
        try {
            if (!root.is_object() || !root.contains("model") || !root.at("model").is_string()) {
                error = "missing string field 'model'";
                return false;
            }
            const std::string model = root.at("model").get<std::string>();
            RealSenseConfig c {};
            if (!MakeDefaultConfig(model, c)) {
                error = "unknown RealSense model '" + model + "'";
                return false;
            }
            if (root.contains("overrides") && root.at("overrides").is_object()) {
                detail::ApplyOverrides(root.at("overrides"), c.profile);
                // Keep stream on/off consistent with overridden has_rgb / has_imu.
                if (!c.profile.has_rgb) c.rgb.enabled = false;
                c.imu.enabled = c.profile.has_imu;
                // An explicit MinZ means the catalogue/fixed value is wanted.
                if (root.at("overrides").contains("min_z_m")) c.depth_model.min_z_mode = "fixed";
            }
            detail::ReadIf(root, "frame_id", c.frame_id);
            if (root.contains("depth") && root.at("depth").is_object()) {
                detail::ReadStream(root.at("depth"), c.depth);
            }
            if (root.contains("rgb") && root.at("rgb").is_object()) {
                const auto& rgb = root.at("rgb");
                detail::ReadStream(rgb, c.rgb);
                if (rgb.contains("noise") && rgb.at("noise").is_object()) {
                    const auto& n = rgb.at("noise");
                    detail::ReadIf(n, "type", c.rgb_noise.type);
                    detail::ReadIf(n, "mean", c.rgb_noise.mean);
                    detail::ReadIf(n, "stddev", c.rgb_noise.stddev);
                    detail::ReadIf(n, "seed", c.rgb_noise.seed);
                }
            }
            if (root.contains("depth_model") && root.at("depth_model").is_object()) {
                const auto& d = root.at("depth_model");
                auto& m = c.depth_model;
                detail::ReadIf(d, "enabled", m.enabled);
                detail::ReadIf(d, "noise_model", m.noise_model);
                detail::ReadIf(d, "noise_scale", m.noise_scale);
                detail::ReadIf(d, "min_z_mode", m.min_z_mode);
                detail::ReadIf(d, "disparity_shift_px", m.disparity_shift_px);
                detail::ReadIf(d, "left_invalid_band", m.left_invalid_band);
                detail::ReadIf(d, "subpixel_step_px", m.subpixel_step_px);
                detail::ReadIf(d, "depth_unit_m", m.depth_unit_m);
                detail::ReadIf(d, "invalid_probability", m.invalid_probability);
                detail::ReadIf(d, "invalid_material_ids", m.invalid_material_ids);
                detail::ReadIf(d, "material_invalid_probability", m.material_invalid_probability);
                detail::ReadIf(d, "seed", m.seed);
            }
            if (root.contains("rolling_shutter") && root.at("rolling_shutter").is_object()) {
                const auto& r = root.at("rolling_shutter");
                detail::ReadIf(r, "enabled", c.rolling_shutter.enabled);
                detail::ReadIf(r, "depth_readout_s", c.rolling_shutter.depth_readout_s);
                detail::ReadIf(r, "rgb_readout_s", c.rolling_shutter.rgb_readout_s);
            }
            if (root.contains("edge_effects") && root.at("edge_effects").is_object()) {
                const auto& e = root.at("edge_effects");
                detail::ReadIf(e, "enabled", c.edge_effects.enabled);
                detail::ReadIf(e, "occlusion", c.edge_effects.occlusion);
                detail::ReadIf(e, "seed", c.edge_effects.seed);
                if (e.contains("flying_pixels") && e.at("flying_pixels").is_object()) {
                    const auto& f = e.at("flying_pixels");
                    auto& fp = c.edge_effects.flying_pixels;
                    detail::ReadIf(f, "enabled", fp.enabled);
                    detail::ReadIf(f, "window_px", fp.window_px);
                    detail::ReadIf(f, "jump_ratio", fp.jump_ratio);
                    detail::ReadIf(f, "probability", fp.probability);
                }
            }
            if (root.contains("post_processing") && root.at("post_processing").is_object()) {
                detail::ReadPostProcessing(root.at("post_processing"), c.post_processing);
            }
            if (root.contains("imu") && root.at("imu").is_object()) {
                const auto& i = root.at("imu");
                detail::ReadIf(i, "enabled", c.imu.enabled);
                detail::ReadIf(i, "accel_rate_hz", c.imu.accel_rate_hz);
                detail::ReadIf(i, "gyro_rate_hz", c.imu.gyro_rate_hz);
                detail::ReadIf(i, "bias_mode", c.imu.bias_mode);
                detail::ReadIf(i, "noise_scale", c.imu.noise_scale);
                detail::ReadIf(i, "quantize", c.imu.quantize);
                detail::ReadIf(i, "saturate", c.imu.saturate);
                detail::ReadIf(i, "bias_random_walk", c.imu.bias_random_walk);
                detail::ReadIf(i, "seed", c.imu.seed);
            }
            const std::string reason = ValidateConfig(c);
            if (!reason.empty()) {
                error = reason;
                return false;
            }
            out = c;
            return true;
        } catch (const std::exception& e) {
            error = std::string("JSON type error: ") + e.what();
            return false;
        }
    }

    inline bool LoadRealSenseConfigFromJson(const std::string& path, RealSenseConfig& out)
    {
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            std::cerr << "Failed to open RealSense config '" << path << "'" << std::endl;
            return false;
        }
        nlohmann::json root;
        try {
            ifs >> root;
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse RealSense config '" << path << "': " << e.what() << std::endl;
            return false;
        }
        std::string error;
        if (!RealSenseConfigFromJson(root, out, error)) {
            std::cerr << "Invalid RealSense config '" << path << "': " << error << std::endl;
            return false;
        }
        return true;
    }
}
