#pragma once

// Depth quality metrics against the ideal (rendered) depth.
//
// Pure: compares a measured depth image with the ground truth of the same size
// and reports what a point-cloud consumer cares about:
//   missing_rate : truth valid (inside [min_z, max_z]) but measured invalid
//   outlier_rate : measured valid and |error| > k * sigma_z(z_truth)
//                  (flying pixels, mirror images, ...), sigma_z from the stereo model
//   rms_m        : RMS error of the valid, non-outlier pixels
//   rms_all_m    : RMS error of all valid pixels
// Rates are fractions of the truth-valid pixels.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "sensors/camera/realsense/realsense_depth_model.hpp"

namespace hako::robots::sensor::camera::realsense
{
    struct DepthMetrics
    {
        std::size_t truth_valid {0};
        std::size_t measured_valid {0};
        std::size_t missing {0};
        std::size_t outliers {0};
        double missing_rate {0.0};
        double outlier_rate {0.0};
        double rms_m {0.0};
        double rms_all_m {0.0};
    };

    // `outlier_k` sigmas; the threshold never goes below `min_threshold_m`
    // (e.g. the depth unit), so an ideal stream has no outliers.
    inline DepthMetrics ComputeDepthMetrics(
        const std::vector<float>& truth,
        const std::vector<float>& measured,
        const DepthModelParams& m,
        double outlier_k = 3.0,
        double min_threshold_m = 0.002)
    {
        DepthMetrics r {};
        if (truth.size() != measured.size()) return r;
        double se = 0.0;
        double se_all = 0.0;
        std::size_t inliers = 0;
        for (std::size_t i = 0; i < truth.size(); ++i) {
            const double zt = static_cast<double>(truth[i]);
            if (!std::isfinite(zt) || zt < m.min_z_m || zt > m.max_z_m) continue;
            ++r.truth_valid;
            const double zm = static_cast<double>(measured[i]);
            if (!std::isfinite(zm)) {
                ++r.missing;
                continue;
            }
            ++r.measured_valid;
            const double e = zm - zt;
            se_all += e * e;
            const double thr = std::max(min_threshold_m, outlier_k * ExpectedDepthSigma(m, zt));
            if (std::fabs(e) > thr) {
                ++r.outliers;
            } else {
                se += e * e;
                ++inliers;
            }
        }
        if (r.truth_valid > 0) {
            r.missing_rate = static_cast<double>(r.missing) / static_cast<double>(r.truth_valid);
            r.outlier_rate = static_cast<double>(r.outliers) / static_cast<double>(r.truth_valid);
        }
        if (inliers > 0) r.rms_m = std::sqrt(se / static_cast<double>(inliers));
        if (r.measured_valid > 0) r.rms_all_m = std::sqrt(se_all / static_cast<double>(r.measured_valid));
        return r;
    }
}
