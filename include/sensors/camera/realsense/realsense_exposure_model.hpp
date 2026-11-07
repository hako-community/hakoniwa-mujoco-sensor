#pragma once

// Exposure, gain and motion blur (RS-5). Pure: no MuJoCo/GL.
//
// Exposure / gain (auto or manual):
//   The rendered image corresponds to the reference exposure t_ref at gain g_ref
//   for a scene of light level 1. A scene of level L needs the exposure product
//   E = t_ref * g_ref / L. Auto exposure takes t = clamp(E / g_ref, t_min, t_max)
//   (t_max ~ 1 / fps) and the rest as gain g = clamp(E / t, g_ref, g_max).
//   brightness = t * g * L / (t_ref * g_ref)   (1 when the request fits; < 1 too dark,
//   > 1 too bright -> RGB scaled and clipped). Higher gain -> more noise:
//   disparity sigma and RGB noise scale with sqrt(g / g_ref).
//   - depth (IR) level: L_ir = mean over valid pixels of
//       rho * (P / z^2 + ambient_gain * ambient_ir) / (0.8 * (1 + ambient_gain * 0.1))
//     (= 1 for a white wall at 1 m with the default projector indoors).
//   - RGB level: rgb_light_level (1 = the rendering is correctly exposed at t_ref).
//
// Motion blur (camera ego-motion during the exposure, like the rolling shutter):
//   N sub-frames at s_k = ((k + 0.5) / N - 0.5) * t, k = 0..N-1, each warped from the
//   reference render with the camera pose at that time (rolling sensors add the row
//   time). RGB = mean of the sub-frames. Depth: the stereo match fails when the
//   pattern smears, i.e. when the sub-frame disparities spread more than
//   disparity_spread_px -> invalid; otherwise z = fB / mean(disparity).
//   Moving OBJECTS are not blurred (only the camera motion is known).
//
// Defaults are ASSUMPTIONS (t_ref 8.5 ms depth / 10 ms RGB, gain 16..248,
// sqrt gain noise, spread 1 px, ambient_gain 10).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "sensors/camera/realsense/realsense_rolling_shutter.hpp"
#include "sensors/camera/realsense/realsense_texture_model.hpp"

namespace hako::robots::sensor::camera::realsense
{
    struct ExposureDecision
    {
        double exposure_s {0.0};
        double gain {16.0};
        double brightness {1.0};
    };

    inline ExposureDecision AutoExposure(double level, double t_ref_s, double g_ref,
                                         double t_min_s, double t_max_s, double g_max)
    {
        ExposureDecision d {};
        const double lv = std::max(level, 1e-6);
        const double e = t_ref_s * g_ref / lv;
        d.exposure_s = std::clamp(e / g_ref, t_min_s, std::max(t_min_s, t_max_s));
        d.gain = std::clamp(e / d.exposure_s, g_ref, std::max(g_ref, g_max));
        d.brightness = d.exposure_s * d.gain * lv / (t_ref_s * g_ref);
        return d;
    }

    inline ExposureDecision ManualExposure(double level, double t_s, double gain, double t_ref_s, double g_ref)
    {
        ExposureDecision d {};
        d.exposure_s = t_s;
        d.gain = gain;
        d.brightness = t_s * gain * std::max(level, 0.0) / (t_ref_s * g_ref);
        return d;
    }

    inline double GainNoiseScale(double gain, double g_ref)
    {
        return std::sqrt(std::max(gain, 1e-9) / std::max(g_ref, 1e-9));
    }

    // IR light level of the depth scene (1 = white wall at 1 m, default projector, indoors).
    inline double IrSceneLevel(const std::vector<float>& depth, const std::vector<int>* material_ids,
                               const std::vector<IrMaterial>& materials_by_id, const IrMaterial& fallback,
                               double laser_power_mw, double ambient_ir, double ambient_gain = 10.0)
    {
        const double power = std::max(0.0, laser_power_mw) / 150.0;
        const bool use_ids = material_ids != nullptr && material_ids->size() == depth.size();
        double sum = 0.0;
        std::size_t n = 0;
        for (std::size_t i = 0; i < depth.size(); ++i) {
            const double z = depth[i];
            if (!std::isfinite(z) || z <= 0.0) continue;
            const IrMaterial* m = &fallback;
            if (use_ids) {
                const int id = (*material_ids)[i];
                if (id >= 0 && static_cast<std::size_t>(id) < materials_by_id.size()) m = &materials_by_id[static_cast<std::size_t>(id)];
            }
            sum += m->ir_reflectance * (power / (z * z) + ambient_gain * ambient_ir);
            ++n;
        }
        if (n == 0) return 0.0;
        return (sum / static_cast<double>(n)) / (0.8 * (1.0 + ambient_gain * 0.1));
    }

    // Mean luma (0..1) of an interleaved 8-bit image.
    inline double MeanLuma(const std::vector<std::uint8_t>& img, int channels)
    {
        if (img.empty() || channels <= 0) return 0.0;
        double s = 0.0;
        const std::size_t px = img.size() / static_cast<std::size_t>(channels);
        for (std::size_t i = 0; i < px; ++i) {
            if (channels >= 3) {
                s += 0.299 * img[i * channels] + 0.587 * img[i * channels + 1] + 0.114 * img[i * channels + 2];
            } else {
                s += img[i * channels];
            }
        }
        return s / (255.0 * static_cast<double>(px));
    }

    // Blur length [px] of a rotation about the vertical axis during the exposure (small angle).
    inline double ExpectedBlurPx(double omega_rad_s, double exposure_s, double fx_px)
    {
        return std::fabs(omega_rad_s) * exposure_s * fx_px;
    }

    namespace detail
    {
        // Depth warped to the camera pose at (row time + extra_dt); see ApplyRollingShutterDepth.
        inline void WarpDepthAt(const std::vector<float>& src, std::vector<float>& out, int width, int height,
                                const PinholeIntrinsics& k, const CameraMotion& motion, double line_time,
                                double extra_dt)
        {
            const float nan_val = std::numeric_limits<float>::quiet_NaN();
            out.assign(src.size(), nan_val);
            auto sample = [&](long u, long v) {
                if (u < 0 || v < 0 || u >= width || v >= height) return nan_val;
                return src[static_cast<std::size_t>(v) * width + u];
            };
            for (int r = 0; r < height; ++r) {
                const double dt = RowDt(r, height, line_time) + extra_dt;
                const auto rot = RotationFromAngularVelocity(motion, dt);
                const double p[3] = {motion.vx * dt, motion.vy * dt, motion.vz * dt};
                for (int x = 0; x < width; ++x) {
                    const double ray[3] = {(x - k.cx) / k.fx, (r - k.cy) / k.fy, 1.0};
                    double d[3];
                    Mul(rot, ray, d);
                    float& o = out[static_cast<std::size_t>(r) * width + x];
                    float z_guess = src[static_cast<std::size_t>(r) * width + x];
                    if (!(std::isfinite(z_guess) && z_guess > 0.0F)) {
                        if (d[2] <= 0.0) continue;
                        z_guess = sample(std::lround(k.fx * d[0] / d[2] + k.cx), std::lround(k.fy * d[1] / d[2] + k.cy));
                        if (!(std::isfinite(z_guess) && z_guess > 0.0F)) continue;
                    }
                    const double q[3] = {d[0] * z_guess + p[0], d[1] * z_guess + p[1], d[2] * z_guess + p[2]};
                    if (q[2] <= 0.0) continue;
                    const long u = std::lround(k.fx * q[0] / q[2] + k.cx);
                    const long v = std::lround(k.fy * q[1] / q[2] + k.cy);
                    const float zs = sample(u, v);
                    if (!(std::isfinite(zs) && zs > 0.0F)) continue;
                    const double pr[3] = {(u - k.cx) / k.fx * zs - p[0], (v - k.cy) / k.fy * zs - p[1], zs - p[2]};
                    double qr[3];
                    MulT(rot, pr, qr);
                    if (qr[2] > 0.0) o = static_cast<float>(qr[2]);
                }
            }
        }

        // Image warped to (row time + extra_dt), accumulated into `acc` (float per channel);
        // `hits` counts samples per pixel. See ApplyRollingShutterImage.
        inline void AccumulateImageAt(const std::vector<std::uint8_t>& src, std::vector<float>& acc,
                                      int width, int height, int channels, const PinholeIntrinsics& k,
                                      const CameraMotion& motion, double line_time, double extra_dt,
                                      const std::vector<float>* ref_depth)
        {
            const bool use_depth = ref_depth && ref_depth->size() == static_cast<std::size_t>(width) * height;
            for (int r = 0; r < height; ++r) {
                const double dt = RowDt(r, height, line_time) + extra_dt;
                const auto rot = RotationFromAngularVelocity(motion, dt);
                const double p[3] = {motion.vx * dt, motion.vy * dt, motion.vz * dt};
                for (int x = 0; x < width; ++x) {
                    const double ray[3] = {(x - k.cx) / k.fx, (r - k.cy) / k.fy, 1.0};
                    double q[3];
                    Mul(rot, ray, q);
                    if (use_depth) {
                        const float z = (*ref_depth)[static_cast<std::size_t>(r) * width + x];
                        if (std::isfinite(z) && z > 0.0F) {
                            for (int i = 0; i < 3; ++i) q[i] = q[i] * z + p[i];
                        }
                    }
                    if (q[2] <= 0.0) continue;   // black (adds 0)
                    const long u = std::lround(k.fx * q[0] / q[2] + k.cx);
                    const long v = std::lround(k.fy * q[1] / q[2] + k.cy);
                    if (u < 0 || v < 0 || u >= width || v >= height) continue;
                    const std::uint8_t* s = &src[(static_cast<std::size_t>(v) * width + u) * channels];
                    float* a = &acc[(static_cast<std::size_t>(r) * width + x) * channels];
                    for (int c = 0; c < channels; ++c) a[c] += s[c];
                }
            }
        }

        inline double SubframeOffset(int k, int n, double exposure_s)
        {
            return ((static_cast<double>(k) + 0.5) / static_cast<double>(n) - 0.5) * exposure_s;
        }
    }

    struct BlurStats
    {
        std::size_t smeared {0};   // depth pixels lost to blur
    };

    // Depth with motion blur (+ rolling shutter when readout_s > 0), in place.
    inline BlurStats ApplyMotionBlurDepth(std::vector<float>& depth, int width, int height,
                                          const PinholeIntrinsics& k, const CameraMotion& motion,
                                          double exposure_s, double readout_s, int subframes,
                                          double fb, double spread_px)
    {
        BlurStats st {};
        if (width <= 0 || height <= 1 || depth.size() != static_cast<std::size_t>(width) * height) return st;
        if (!HasMotion(motion) || subframes < 1) return st;
        const double line_time = readout_s > 0.0 ? readout_s / static_cast<double>(height) : 0.0;
        const std::vector<float> src = depth;
        const std::size_t n = depth.size();
        std::vector<double> sum(n, 0.0), dmin(n, std::numeric_limits<double>::infinity()),
            dmax(n, -std::numeric_limits<double>::infinity());
        std::vector<unsigned char> bad(n, 0);
        std::vector<float> sub;
        for (int s = 0; s < subframes; ++s) {
            detail::WarpDepthAt(src, sub, width, height, k, motion, line_time,
                                detail::SubframeOffset(s, subframes, exposure_s));
            for (std::size_t i = 0; i < n; ++i) {
                const float z = sub[i];
                if (!(std::isfinite(z) && z > 0.0F)) { bad[i] = 1; continue; }
                const double d = fb / z;
                sum[i] += d;
                dmin[i] = std::min(dmin[i], d);
                dmax[i] = std::max(dmax[i], d);
            }
        }
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        for (std::size_t i = 0; i < n; ++i) {
            const bool was_valid = std::isfinite(src[i]) && src[i] > 0.0F;
            if (bad[i] || dmax[i] - dmin[i] > spread_px) {
                if (was_valid && !bad[i]) ++st.smeared;
                depth[i] = nan_val;
                continue;
            }
            depth[i] = static_cast<float>(fb / (sum[i] / subframes));
        }
        return st;
    }

    // Image with motion blur (+ rolling shutter when readout_s > 0), in place.
    inline void ApplyMotionBlurImage(std::vector<std::uint8_t>& img, int width, int height, int channels,
                                     const PinholeIntrinsics& k, const CameraMotion& motion,
                                     double exposure_s, double readout_s, int subframes,
                                     const std::vector<float>* ref_depth = nullptr)
    {
        if (width <= 0 || height <= 1 || channels <= 0) return;
        if (img.size() != static_cast<std::size_t>(width) * height * channels) return;
        if (!HasMotion(motion) || subframes < 1) return;
        const double line_time = readout_s > 0.0 ? readout_s / static_cast<double>(height) : 0.0;
        const std::vector<std::uint8_t> src = img;
        std::vector<float> acc(img.size(), 0.0F);
        for (int s = 0; s < subframes; ++s) {
            detail::AccumulateImageAt(src, acc, width, height, channels, k, motion, line_time,
                                      detail::SubframeOffset(s, subframes, exposure_s), ref_depth);
        }
        for (std::size_t i = 0; i < img.size(); ++i) {
            img[i] = static_cast<std::uint8_t>(std::clamp(std::lround(acc[i] / subframes), 0L, 255L));
        }
    }

    // Brightness scaling (clipped) and gain noise of an 8-bit image.
    inline void ApplyBrightnessAndGainNoise(std::vector<std::uint8_t>& img, double brightness, double noise_sigma,
                                            std::mt19937& rng)
    {
        if (img.empty()) return;
        std::normal_distribution<double> n(0.0, 1.0);
        const bool scale = std::fabs(brightness - 1.0) > 1e-12;
        for (auto& v : img) {
            double x = v;
            if (scale) x *= brightness;
            if (noise_sigma > 0.0) x += noise_sigma * n(rng);
            v = static_cast<std::uint8_t>(std::clamp(std::lround(x), 0L, 255L));
        }
    }
}
