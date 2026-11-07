#pragma once

// Stereo depth error model for RealSense profiles.
//
// Pure math on a z-depth image (float metres, NaN = invalid): no MuJoCo/GL.
// The renderer gives ideal depth; this model turns it into what an active /
// passive stereo camera reports:
//
//   1. range gate     : z < min_z or z > max_z               -> invalid
//   2. left band      : pixel x < disparity(z) = fx*B/z       -> invalid
//                       (left-imager reference: no match in the right image)
//   3. disparity noise: d = fx*B/z, d' = d + N(0, sigma_d)    -> z' = fx*B/d'
//                       => sigma_z(z) ~= z^2 * sigma_d / (fx*B)   (grows with z^2)
//   4. subpixel step  : d' quantised to subpixel_step_px (optional)
//   5. dropout        : random / material-based invalid pixels (optional)
//   6. depth unit     : z' quantised to depth_unit_m (Z16 default 1 mm)
//
// sigma_d is derived from the catalogue accuracy ("2% @ 2 m") at the reference
// width, so a lower stream resolution (smaller fx) gives a larger depth error,
// as on the real camera.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "sensors/camera/realsense/realsense_profile.hpp"

namespace hako::robots::sensor::camera::realsense
{
    // Pinhole focal length in pixels for an image of `width` px and horizontal FOV.
    inline double FocalLengthPx(int width, double hfov_rad)
    {
        return static_cast<double>(width) / (2.0 * std::tan(hfov_rad * 0.5));
    }

    // 1-sigma disparity noise [px] implied by the catalogue accuracy:
    //   sigma_z(z_ref) = ratio * z_ref  and  sigma_z = z^2 * sigma_d / (fx_ref * B)
    //   => sigma_d = ratio * fx_ref * B / z_ref
    inline double DisparitySigmaFromSpec(const RealSenseProfile& p)
    {
        if (p.accuracy_ref_distance_m <= 0.0 || p.baseline_m <= 0.0) return 0.0;
        const double fx_ref = FocalLengthPx(p.accuracy_ref_width, p.DepthHfovRad());
        return p.accuracy_ratio * fx_ref * p.baseline_m / p.accuracy_ref_distance_m;
    }

    struct DepthModelParams
    {
        double fx_px {0.0};             // focal length of the depth stream
        double baseline_m {0.05};
        double min_z_m {0.1};
        double max_z_m {10.0};
        double disparity_sigma_px {0.0};
        double disparity_offset_px {0.0};  // RS-4 thermal drift (added to the disparity)
        double subpixel_step_px {0.0};  // 0 = continuous
        double depth_unit_m {0.001};    // 0 = no quantisation
        bool left_invalid_band {true};
        double invalid_probability {0.0};
        std::vector<int> invalid_material_ids {};
        double material_invalid_probability {1.0};
    };

    // Expected 1-sigma depth error at distance z for the given params.
    inline double ExpectedDepthSigma(const DepthModelParams& m, double z)
    {
        if (m.fx_px <= 0.0 || m.baseline_m <= 0.0) return 0.0;
        return z * z * m.disparity_sigma_px / (m.fx_px * m.baseline_m);
    }

    // Width [px] of the invalid band on the left edge for an object at depth z.
    inline double LeftInvalidBandPx(const DepthModelParams& m, double z)
    {
        if (z <= 0.0) return 0.0;
        return m.fx_px * m.baseline_m / z;
    }

    // Applies the model in place. `material_ids` (same size, -1 = none) may be null.
    // `sigma_scale` (same size) multiplies the disparity sigma per pixel (RS-2: a
    // weak projector pattern gives a noisier match); null = 1 everywhere.
    inline void ApplyDepthModel(
        std::vector<float>& depth,
        int width,
        int height,
        const DepthModelParams& m,
        std::mt19937& rng,
        const std::vector<int>* material_ids = nullptr,
        const std::vector<float>* sigma_scale = nullptr)
    {
        if (width <= 0 || height <= 0) return;
        const std::size_t n = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        if (depth.size() != n) return;

        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        const double fb = m.fx_px * m.baseline_m;
        std::normal_distribution<double> disparity_noise(0.0, 1.0);
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        const bool use_materials = material_ids != nullptr
            && material_ids->size() == n && !m.invalid_material_ids.empty();
        const bool use_scale = sigma_scale != nullptr && sigma_scale->size() == n;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(width)
                    + static_cast<std::size_t>(x);
                const double z = static_cast<double>(depth[i]);
                if (!std::isfinite(z) || z <= 0.0) {
                    depth[i] = nan_val;
                    continue;
                }
                // 1. range gate (on the true depth: the matcher's search range)
                if (z < m.min_z_m || z > m.max_z_m) {
                    depth[i] = nan_val;
                    continue;
                }
                // 2. left invalid band
                if (m.left_invalid_band && fb > 0.0 && static_cast<double>(x) < fb / z) {
                    depth[i] = nan_val;
                    continue;
                }
                // 5. dropout (drawn before noise so the RNG stream is per-pixel stable)
                if (m.invalid_probability > 0.0 && uniform(rng) < m.invalid_probability) {
                    depth[i] = nan_val;
                    continue;
                }
                if (use_materials) {
                    const int mat = (*material_ids)[i];
                    if (std::find(m.invalid_material_ids.begin(), m.invalid_material_ids.end(), mat)
                            != m.invalid_material_ids.end()
                        && uniform(rng) < m.material_invalid_probability) {
                        depth[i] = nan_val;
                        continue;
                    }
                }

                double zn = z;
                if (fb > 0.0 && (m.disparity_sigma_px > 0.0 || m.subpixel_step_px > 0.0
                                 || m.disparity_offset_px != 0.0)) {
                    // 3. disparity noise (+ thermal offset)
                    double d = fb / z + m.disparity_offset_px;
                    if (m.disparity_sigma_px > 0.0) {
                        const double scale = use_scale ? static_cast<double>((*sigma_scale)[i]) : 1.0;
                        d += m.disparity_sigma_px * scale * disparity_noise(rng);
                    }
                    // 4. subpixel quantisation
                    if (m.subpixel_step_px > 0.0) {
                        d = std::round(d / m.subpixel_step_px) * m.subpixel_step_px;
                    }
                    if (d <= 0.0) {
                        depth[i] = nan_val;   // matched "at infinity": no depth
                        continue;
                    }
                    zn = fb / d;
                }
                // 6. depth unit
                if (m.depth_unit_m > 0.0) {
                    zn = std::round(zn / m.depth_unit_m) * m.depth_unit_m;
                }
                depth[i] = (zn > 0.0) ? static_cast<float>(zn) : nan_val;
            }
        }
    }
}
