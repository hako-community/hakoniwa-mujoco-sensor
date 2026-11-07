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
//   "texture_model": { "enabled": false, "materials": "materials.json" | { ... },
//                      "laser_power_mw": 150, "ambient_ir": 0.1, "seed": 0 },
//   "specular_model": { "enabled": false, "highlight": true, "highlight_width_deg": 8,
//                       "interreflection": true, "max_distance_m": 0.5, "ray_stride_px": 4,
//                       "min_specular": 0.5, "mirror_gain": 1.0, "seed": 0 },
//   "thermal": { "enabled": false, "ambient_c": 25, "ambient_schedule": [[0, 15], [14400, 30]],
//                "steady_rise_k": 15, "time_constant_s": 600, "start_warm": false,
//                "calibration_temp_c": null, "disparity_coeff_px_per_k": 0.005,
//                "accel_tco_mg_per_k": 1.0, "gyro_tco_dps_per_k": 0.015, "seed": 0 },
//   "exposure": { "enabled": false, "auto": true, "depth_exposure_us": 8500, "depth_gain": 16,
//                 "rgb_exposure_us": 10000, "rgb_gain": 16, "max_exposure_us": null, "max_gain": 248,
//                 "rgb_light_level": 1.0, "rgb_gain_noise_dn": 1.0,
//                 "motion_blur": { "enabled": true, "subframes": 4, "disparity_spread_px": 1.0 },
//                 "seed": 0 },
//   "imu_timing": { "enabled": false, "time_offset_s": 0.0, "jitter_s": 0.0, "drop_probability": 0.0,
//                   "position_m": [0, 0, 0], "misalignment_deg": 0.0, "seed": 0 },
//   "frame_timing": { "enabled": false, "jitter_s": 0.0, "drop_probability": 0.0, "seed": 0 },
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
#include "sensors/camera/realsense/realsense_exposure_model.hpp"
#include "sensors/camera/realsense/realsense_imu_model.hpp"
#include "sensors/camera/realsense/realsense_imu_timing.hpp"
#include "sensors/camera/realsense/realsense_post_processing.hpp"
#include "sensors/camera/realsense/realsense_profile.hpp"
#include "sensors/camera/realsense/realsense_specular_model.hpp"
#include "sensors/camera/realsense/realsense_texture_model.hpp"
#include "sensors/camera/realsense/realsense_thermal_model.hpp"

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

    // RS-2: material table + weak-pattern holes (default off). Gains are assumptions.
    struct TextureModelConfig
    {
        bool enabled {false};
        MaterialTable materials {};
        std::string materials_source {};   // file the table was read from ("" = inline / built-in)
        double laser_power_mw {150.0};
        double ambient_ir {0.1};
        double reference_range_m {10.0};
        double steepness {6.0};
        double noise_gain {0.5};
        double ambient_noise_gain {10.0};
        double passive_gain {20.0};
        std::uint32_t seed {0};
    };

    // RS-3: highlights + inter-reflections of shiny materials (default off).
    // Uses texture_model.materials (specular) and texture_model.laser_power_mw.
    struct SpecularModelConfig
    {
        bool enabled {false};
        bool highlight {true};
        double highlight_width_deg {8.0};
        double highlight_gain {1.0};
        std::vector<double> projector_offset_m {};   // empty = (+B/2, 0, 0) (assumed)
        bool interreflection {true};
        double max_distance_m {0.5};
        int ray_stride_px {4};
        double min_specular {0.5};
        double mirror_gain {1.0};
        int max_rays {40000};
        std::uint32_t seed {0};
    };

    // RS-4: warm-up / ambient temperature drift of the depth and the IMU (default off).
    struct ThermalConfig
    {
        bool enabled {false};
        ThermalParams params {};   // laser_power_mw is taken from texture_model
    };

    // RS-5: exposure / gain (auto or manual) and motion blur (default off).
    // Reference exposures (the rendering is correctly exposed there, gain 16):
    // depth 8.5 ms, RGB 10 ms. All values are assumptions.
    struct MotionBlurConfig
    {
        bool enabled {true};
        int subframes {4};
        double disparity_spread_px {1.0};
    };

    struct ExposureConfig
    {
        bool enabled {false};
        bool auto_exposure {true};
        double depth_exposure_us {8500.0};   // manual value (auto: ignored)
        double depth_gain {16.0};
        double rgb_exposure_us {10000.0};
        double rgb_gain {16.0};
        double reference_depth_exposure_us {8500.0};
        double reference_rgb_exposure_us {10000.0};
        double reference_gain {16.0};
        double min_exposure_us {100.0};
        double max_exposure_us {-1.0};       // < 0: 1 / fps
        double max_gain {248.0};
        double ambient_gain {10.0};          // IR level: weight of ambient_ir against the projector
        double rgb_light_level {1.0};        // 1 = rendering correctly exposed at the reference
        double rgb_gain_noise_dn {1.0};      // RGB noise sigma [DN] at the reference gain
        MotionBlurConfig motion_blur {};
        std::uint32_t seed {0};

        double MaxExposureS(double fps) const
        {
            return max_exposure_us > 0.0 ? max_exposure_us * 1e-6 : 1.0 / std::max(fps, 1e-3);
        }
    };

    // RS-6: IMU timestamps / drops / lever arm / misalignment and camera frame
    // timing (default off).
    struct ImuTimingConfig
    {
        bool enabled {false};
        ImuTimingParams params {};
    };

    struct FrameTimingConfig
    {
        bool enabled {false};
        FrameTimingParams params {};
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
        TextureModelConfig texture_model {};
        SpecularModelConfig specular_model {};
        ThermalConfig thermal {};
        ExposureConfig exposure {};
        ImuTimingConfig imu_timing {};
        FrameTimingConfig frame_timing {};
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

    // Texture-model parameters for the configured depth stream (render size).
    inline TextureModelParams MakeTextureModelParams(const RealSenseConfig& c)
    {
        TextureModelParams t {};
        const auto& m = c.texture_model;
        t.laser_power_mw = m.laser_power_mw;
        t.ambient_ir = m.ambient_ir;
        t.reference_range_m = m.reference_range_m;
        t.steepness = m.steepness;
        t.noise_gain = m.noise_gain;
        t.ambient_noise_gain = m.ambient_noise_gain;
        t.passive_gain = m.passive_gain;
        t.fx_px = FocalLengthPx(c.depth.width, c.profile.DepthHfovRad());
        t.fy_px = t.fx_px;
        t.cx_px = c.depth.width * 0.5;
        t.cy_px = c.depth.height * 0.5;
        return t;
    }

    // Specular-model parameters for the configured depth stream (render size).
    inline SpecularModelParams MakeSpecularModelParams(const RealSenseConfig& c)
    {
        SpecularModelParams s {};
        const auto& m = c.specular_model;
        s.highlight = m.highlight;
        s.highlight_width_deg = m.highlight_width_deg;
        s.highlight_gain = m.highlight_gain;
        s.laser_power_mw = c.texture_model.laser_power_mw;
        if (m.projector_offset_m.size() == 3) {
            for (int k = 0; k < 3; ++k) s.projector_offset_m[k] = m.projector_offset_m[static_cast<std::size_t>(k)];
        } else {
            s.projector_offset_m[0] = 0.5 * c.profile.baseline_m;
            s.projector_offset_m[1] = 0.0;
            s.projector_offset_m[2] = 0.0;
        }
        s.interreflection = m.interreflection;
        s.max_distance_m = m.max_distance_m;
        s.ray_stride_px = m.ray_stride_px;
        s.min_specular = m.min_specular;
        s.mirror_gain = m.mirror_gain;
        s.max_rays = static_cast<std::size_t>(std::max(0, m.max_rays));
        s.fx_px = FocalLengthPx(c.depth.width, c.profile.DepthHfovRad());
        s.fy_px = s.fx_px;
        s.cx_px = c.depth.width * 0.5;
        s.cy_px = c.depth.height * 0.5;
        return s;
    }

    inline ThermalParams MakeThermalParams(const RealSenseConfig& c)
    {
        ThermalParams t = c.thermal.params;
        t.laser_power_mw = c.texture_model.laser_power_mw;
        return t;
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
        const auto& tm = c.texture_model;
        if (tm.laser_power_mw < 0.0 || tm.ambient_ir < 0.0 || tm.reference_range_m <= 0.0 || tm.steepness <= 0.0
            || tm.noise_gain < 0.0 || tm.ambient_noise_gain < 0.0 || tm.passive_gain < 0.0)
            return "invalid texture_model";
        const auto& sm = c.specular_model;
        if (sm.highlight_width_deg <= 0.0 || sm.highlight_gain < 0.0 || sm.max_distance_m <= 0.0
            || sm.ray_stride_px < 1 || sm.min_specular < 0.0 || sm.min_specular > 1.0 || sm.mirror_gain < 0.0
            || sm.max_rays < 0 || (!sm.projector_offset_m.empty() && sm.projector_offset_m.size() != 3))
            return "invalid specular_model";
        const auto& th = c.thermal.params;
        if (th.steady_rise_k < 0.0 || th.time_constant_s < 0.0 || th.laser_share < 0.0 || th.laser_share > 1.0
            || th.accel_tco_mg_per_k < 0.0 || th.gyro_tco_dps_per_k < 0.0)
            return "invalid thermal";
        const auto& ex = c.exposure;
        if (ex.depth_exposure_us <= 0.0 || ex.rgb_exposure_us <= 0.0 || ex.depth_gain <= 0.0 || ex.rgb_gain <= 0.0
            || ex.reference_depth_exposure_us <= 0.0 || ex.reference_rgb_exposure_us <= 0.0 || ex.reference_gain <= 0.0
            || ex.min_exposure_us <= 0.0 || ex.max_gain < ex.reference_gain || ex.ambient_gain < 0.0
            || ex.rgb_light_level < 0.0 || ex.rgb_gain_noise_dn < 0.0
            || ex.motion_blur.subframes < 1 || ex.motion_blur.subframes > 64 || ex.motion_blur.disparity_spread_px <= 0.0)
            return "invalid exposure";
        const auto& it = c.imu_timing.params;
        if (it.jitter_s < 0.0 || it.drop_probability < 0.0 || it.drop_probability > 1.0 || it.misalignment_deg < 0.0)
            return "invalid imu_timing";
        const auto& ft = c.frame_timing.params;
        if (ft.jitter_s < 0.0 || ft.drop_probability < 0.0 || ft.drop_probability > 1.0)
            return "invalid frame_timing";
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
    // `base_dir`: directory for relative paths inside the config (the material table).
    inline bool RealSenseConfigFromJson(const nlohmann::json& root, RealSenseConfig& out, std::string& error,
                                        const std::string& base_dir = "")
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
            if (root.contains("texture_model") && root.at("texture_model").is_object()) {
                const auto& t = root.at("texture_model");
                auto& tm = c.texture_model;
                detail::ReadIf(t, "enabled", tm.enabled);
                detail::ReadIf(t, "laser_power_mw", tm.laser_power_mw);
                detail::ReadIf(t, "ambient_ir", tm.ambient_ir);
                detail::ReadIf(t, "reference_range_m", tm.reference_range_m);
                detail::ReadIf(t, "steepness", tm.steepness);
                detail::ReadIf(t, "noise_gain", tm.noise_gain);
                detail::ReadIf(t, "ambient_noise_gain", tm.ambient_noise_gain);
                detail::ReadIf(t, "passive_gain", tm.passive_gain);
                detail::ReadIf(t, "seed", tm.seed);
                if (t.contains("materials")) {
                    const auto& mj = t.at("materials");
                    nlohmann::json table;
                    if (mj.is_string()) {
                        std::string path = mj.get<std::string>();
                        const bool absolute = !path.empty()
                            && (path[0] == '/' || path[0] == '\\' || (path.size() > 1 && path[1] == ':'));
                        if (!absolute && !base_dir.empty()) path = base_dir + "/" + path;
                        std::ifstream ifs(path);
                        if (!ifs.is_open()) {
                            error = "cannot open material table '" + path + "'";
                            return false;
                        }
                        try {
                            ifs >> table;
                        } catch (const std::exception& e) {
                            error = "cannot parse material table '" + path + "': " + e.what();
                            return false;
                        }
                        tm.materials_source = path;
                    } else {
                        table = mj;
                    }
                    if (!MaterialTableFromJson(table, tm.materials, error)) return false;
                }
            }
            if (root.contains("specular_model") && root.at("specular_model").is_object()) {
                const auto& t = root.at("specular_model");
                auto& sm = c.specular_model;
                detail::ReadIf(t, "enabled", sm.enabled);
                detail::ReadIf(t, "highlight", sm.highlight);
                detail::ReadIf(t, "highlight_width_deg", sm.highlight_width_deg);
                detail::ReadIf(t, "highlight_gain", sm.highlight_gain);
                detail::ReadIf(t, "projector_offset_m", sm.projector_offset_m);
                detail::ReadIf(t, "interreflection", sm.interreflection);
                detail::ReadIf(t, "max_distance_m", sm.max_distance_m);
                detail::ReadIf(t, "ray_stride_px", sm.ray_stride_px);
                detail::ReadIf(t, "min_specular", sm.min_specular);
                detail::ReadIf(t, "mirror_gain", sm.mirror_gain);
                detail::ReadIf(t, "max_rays", sm.max_rays);
                detail::ReadIf(t, "seed", sm.seed);
            }
            if (root.contains("thermal") && root.at("thermal").is_object()) {
                const auto& t = root.at("thermal");
                auto& tp = c.thermal.params;
                detail::ReadIf(t, "enabled", c.thermal.enabled);
                detail::ReadIf(t, "ambient_c", tp.ambient_c);
                detail::ReadIf(t, "steady_rise_k", tp.steady_rise_k);
                detail::ReadIf(t, "time_constant_s", tp.time_constant_s);
                detail::ReadIf(t, "laser_share", tp.laser_share);
                detail::ReadIf(t, "start_warm", tp.start_warm);
                detail::ReadIf(t, "calibration_temp_c", tp.calibration_temp_c);
                detail::ReadIf(t, "disparity_coeff_px_per_k", tp.disparity_coeff_px_per_k);
                detail::ReadIf(t, "accel_tco_mg_per_k", tp.accel_tco_mg_per_k);
                detail::ReadIf(t, "gyro_tco_dps_per_k", tp.gyro_tco_dps_per_k);
                detail::ReadIf(t, "seed", tp.seed);
                if (t.contains("ambient_schedule") && t.at("ambient_schedule").is_array()) {
                    tp.ambient_schedule.clear();
                    for (const auto& e : t.at("ambient_schedule")) {
                        if (!e.is_array() || e.size() != 2) {
                            error = "thermal.ambient_schedule entries must be [t_s, deg_c]";
                            return false;
                        }
                        tp.ambient_schedule.emplace_back(e.at(0).get<double>(), e.at(1).get<double>());
                    }
                }
            }
            if (root.contains("exposure") && root.at("exposure").is_object()) {
                const auto& t = root.at("exposure");
                auto& ex = c.exposure;
                detail::ReadIf(t, "enabled", ex.enabled);
                detail::ReadIf(t, "auto", ex.auto_exposure);
                detail::ReadIf(t, "depth_exposure_us", ex.depth_exposure_us);
                detail::ReadIf(t, "depth_gain", ex.depth_gain);
                detail::ReadIf(t, "rgb_exposure_us", ex.rgb_exposure_us);
                detail::ReadIf(t, "rgb_gain", ex.rgb_gain);
                detail::ReadIf(t, "reference_depth_exposure_us", ex.reference_depth_exposure_us);
                detail::ReadIf(t, "reference_rgb_exposure_us", ex.reference_rgb_exposure_us);
                detail::ReadIf(t, "reference_gain", ex.reference_gain);
                detail::ReadIf(t, "min_exposure_us", ex.min_exposure_us);
                detail::ReadIf(t, "max_exposure_us", ex.max_exposure_us);
                detail::ReadIf(t, "max_gain", ex.max_gain);
                detail::ReadIf(t, "ambient_gain", ex.ambient_gain);
                detail::ReadIf(t, "rgb_light_level", ex.rgb_light_level);
                detail::ReadIf(t, "rgb_gain_noise_dn", ex.rgb_gain_noise_dn);
                detail::ReadIf(t, "seed", ex.seed);
                if (t.contains("motion_blur") && t.at("motion_blur").is_object()) {
                    const auto& b = t.at("motion_blur");
                    detail::ReadIf(b, "enabled", ex.motion_blur.enabled);
                    detail::ReadIf(b, "subframes", ex.motion_blur.subframes);
                    detail::ReadIf(b, "disparity_spread_px", ex.motion_blur.disparity_spread_px);
                }
            }
            if (root.contains("imu_timing") && root.at("imu_timing").is_object()) {
                const auto& t = root.at("imu_timing");
                auto& ip = c.imu_timing.params;
                detail::ReadIf(t, "enabled", c.imu_timing.enabled);
                detail::ReadIf(t, "time_offset_s", ip.time_offset_s);
                detail::ReadIf(t, "jitter_s", ip.jitter_s);
                detail::ReadIf(t, "drop_probability", ip.drop_probability);
                detail::ReadIf(t, "misalignment_deg", ip.misalignment_deg);
                detail::ReadIf(t, "seed", ip.seed);
                if (t.contains("position_m")) {
                    const auto v = t.at("position_m").get<std::vector<double>>();
                    if (v.size() != 3) {
                        error = "imu_timing.position_m needs 3 values";
                        return false;
                    }
                    for (int k = 0; k < 3; ++k) ip.position_m[k] = v[static_cast<std::size_t>(k)];
                }
            }
            if (root.contains("frame_timing") && root.at("frame_timing").is_object()) {
                const auto& t = root.at("frame_timing");
                detail::ReadIf(t, "enabled", c.frame_timing.enabled);
                detail::ReadIf(t, "jitter_s", c.frame_timing.params.jitter_s);
                detail::ReadIf(t, "drop_probability", c.frame_timing.params.drop_probability);
                detail::ReadIf(t, "seed", c.frame_timing.params.seed);
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
        const auto slash = path.find_last_of("/\\");
        const std::string base_dir = slash == std::string::npos ? std::string(".") : path.substr(0, slash);
        if (!RealSenseConfigFromJson(root, out, error, base_dir)) {
            std::cerr << "Invalid RealSense config '" << path << "': " << error << std::endl;
            return false;
        }
        return true;
    }
}
