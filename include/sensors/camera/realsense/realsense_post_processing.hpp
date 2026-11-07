#pragma once

// librealsense post-processing filters reimplemented on float depth images.
//
// Pure (no MuJoCo/GL/SDK). Order follows the recommended librealsense chain:
//   Decimation -> Threshold -> Depth2Disparity -> Spatial -> Temporal
//   -> Disparity2Depth -> Hole filling
// Defaults follow the librealsense documentation ("Post-processing filters"):
//   decimation magnitude 2 (median for 2-3, mean for 4-8),
//   spatial  iterations 2, alpha 0.5, delta 20, holes_fill 0,
//   temporal alpha 0.4, delta 20, persistency 3 ("valid in 2 of last 4"),
//   hole filling mode 1 (farthest from around).
// Units: in the disparity domain `delta` is in 1/32 px (the D4 subpixel unit,
// so 20 = 0.625 px); in the depth domain it is in millimetres. The filters are
// a behavioural reimplementation, not bit-exact with librealsense
// (e.g. decimation does not pad the output to 4-pixel blocks).
// Invalid depth is NaN on input and output.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace hako::robots::sensor::camera::realsense
{
    struct DecimationConfig
    {
        bool enabled {false};
        int magnitude {2};          // 1-8
    };

    struct ThresholdConfig
    {
        bool enabled {false};
        double min_m {0.1};
        double max_m {4.0};
    };

    struct SpatialConfig
    {
        bool enabled {false};
        int iterations {2};         // 1-5
        double alpha {0.5};         // 0.25-1 (1 = no filtering)
        double delta {20.0};        // 1-50
        int holes_fill {0};         // 0-5 -> none, 2, 4, 8, 16, unlimited px
    };

    struct TemporalConfig
    {
        bool enabled {false};
        double alpha {0.4};         // 0-1 (1 = no filtering)
        double delta {20.0};        // 1-100
        int persistency {3};        // 0-8
    };

    struct HoleFillingConfig
    {
        bool enabled {false};
        int mode {1};               // 0 fill_from_left, 1 farthest_from_around, 2 nearest_from_around
    };

    struct PostProcessingConfig
    {
        bool enabled {false};
        bool disparity_domain {true};   // spatial/temporal on disparity (librealsense default chain)
        DecimationConfig decimation {};
        ThresholdConfig threshold {};
        SpatialConfig spatial {};
        TemporalConfig temporal {};
        HoleFillingConfig hole_filling {};
    };

    inline bool IsValidDepth(float v) { return std::isfinite(v) && v > 0.0F; }

    // --- Decimation ---------------------------------------------------------
    inline void Decimate(std::vector<float>& img, int& width, int& height, int magnitude)
    {
        if (magnitude <= 1 || width <= 0 || height <= 0) return;
        const int n = magnitude;
        const int ow = width / n;
        const int oh = height / n;
        if (ow <= 0 || oh <= 0) return;
        std::vector<float> out(static_cast<std::size_t>(ow) * oh, std::numeric_limits<float>::quiet_NaN());
        std::vector<float> patch;
        patch.reserve(static_cast<std::size_t>(n) * n);
        for (int y = 0; y < oh; ++y) {
            for (int x = 0; x < ow; ++x) {
                patch.clear();
                for (int dy = 0; dy < n; ++dy) {
                    for (int dx = 0; dx < n; ++dx) {
                        const float v = img[static_cast<std::size_t>(y * n + dy) * width + (x * n + dx)];
                        if (IsValidDepth(v)) patch.push_back(v);
                    }
                }
                if (patch.empty()) continue;
                float value = 0.0F;
                if (n <= 3) {
                    const std::size_t mid = (patch.size() - 1) / 2;
                    std::nth_element(patch.begin(), patch.begin() + static_cast<std::ptrdiff_t>(mid), patch.end());
                    value = patch[mid];
                } else {
                    double sum = 0.0;
                    for (float v : patch) sum += v;
                    value = static_cast<float>(sum / static_cast<double>(patch.size()));
                }
                out[static_cast<std::size_t>(y) * ow + x] = value;
            }
        }
        img = std::move(out);
        width = ow;
        height = oh;
    }

    // --- Threshold ----------------------------------------------------------
    inline void Threshold(std::vector<float>& img, double min_m, double max_m)
    {
        for (auto& v : img) {
            if (IsValidDepth(v) && (v < min_m || v > max_m)) v = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // --- Depth <-> disparity (disparity in 1/32 px) -------------------------
    inline void DepthToDisparity(std::vector<float>& img, double fx_px, double baseline_m)
    {
        const double k = 32.0 * fx_px * baseline_m;
        for (auto& v : img) v = IsValidDepth(v) ? static_cast<float>(k / v) : std::numeric_limits<float>::quiet_NaN();
    }

    inline void DisparityToDepth(std::vector<float>& img, double fx_px, double baseline_m)
    {
        const double k = 32.0 * fx_px * baseline_m;
        for (auto& v : img) v = IsValidDepth(v) ? static_cast<float>(k / v) : std::numeric_limits<float>::quiet_NaN();
    }

    // --- Spatial (recursive edge-preserving domain transform) ---------------
    namespace detail
    {
        inline int HolesFillPixels(int index)
        {
            static const std::array<int, 6> table {0, 2, 4, 8, 16, std::numeric_limits<int>::max()};
            return table[static_cast<std::size_t>(std::clamp(index, 0, 5))];
        }

        // One recursive pass along a line given by (start, stride, count).
        inline void SpatialLine(std::vector<float>& img, std::size_t start, std::ptrdiff_t stride, int count,
                                double alpha, double delta, int fill_px)
        {
            float prev = std::numeric_limits<float>::quiet_NaN();
            int hole_run = 0;
            for (int i = 0; i < count; ++i) {
                float& cur = img[static_cast<std::size_t>(static_cast<std::ptrdiff_t>(start) + stride * i)];
                if (IsValidDepth(cur)) {
                    if (IsValidDepth(prev) && std::fabs(cur - prev) < delta) {
                        cur = static_cast<float>(alpha * cur + (1.0 - alpha) * prev);
                    }
                    prev = cur;
                    hole_run = 0;
                } else if (fill_px > 0 && IsValidDepth(prev) && hole_run < fill_px) {
                    cur = prev;
                    ++hole_run;
                } else {
                    ++hole_run;
                }
            }
        }
    }

    // `delta` is in the image's unit (1/32 px or mm); depth images must be passed in mm.
    inline void SpatialFilter(std::vector<float>& img, int width, int height, const SpatialConfig& c)
    {
        if (width <= 0 || height <= 0) return;
        const int fill_px = detail::HolesFillPixels(c.holes_fill);
        for (int it = 0; it < std::max(1, c.iterations); ++it) {
            for (int y = 0; y < height; ++y) {
                const std::size_t row = static_cast<std::size_t>(y) * width;
                detail::SpatialLine(img, row, 1, width, c.alpha, c.delta, fill_px);
                detail::SpatialLine(img, row + width - 1, -1, width, c.alpha, c.delta, fill_px);
            }
            for (int x = 0; x < width; ++x) {
                detail::SpatialLine(img, static_cast<std::size_t>(x), width, height, c.alpha, c.delta, 0);
                detail::SpatialLine(img, static_cast<std::size_t>(height - 1) * width + x, -width, height,
                                    c.alpha, c.delta, 0);
            }
        }
    }

    // --- Temporal (stateful) ------------------------------------------------
    class TemporalFilter
    {
    public:
        void Reset()
        {
            prev_.clear();
            history_.clear();
        }

        // `delta` in the image's unit (see SpatialFilter).
        void Apply(std::vector<float>& img, const TemporalConfig& c)
        {
            if (prev_.size() != img.size()) {
                prev_.assign(img.size(), std::numeric_limits<float>::quiet_NaN());
                history_.assign(img.size(), 0);
            }
            for (std::size_t i = 0; i < img.size(); ++i) {
                float& cur = img[i];
                const bool valid = IsValidDepth(cur);
                history_[i] = static_cast<std::uint8_t>((history_[i] << 1) | (valid ? 1 : 0));
                if (valid) {
                    if (IsValidDepth(prev_[i]) && std::fabs(cur - prev_[i]) < c.delta) {
                        cur = static_cast<float>(c.alpha * cur + (1.0 - c.alpha) * prev_[i]);
                    }
                    prev_[i] = cur;
                } else if (IsValidDepth(prev_[i]) && Persist(history_[i], c.persistency)) {
                    cur = prev_[i];
                }
            }
        }

        // history bit 0 = current frame (invalid when called), bit k = k frames ago.
        static bool Persist(std::uint8_t history, int persistency)
        {
            auto valid_in_last = [&](int n) {
                int count = 0;
                for (int k = 0; k < n; ++k) count += (history >> k) & 1;
                return count;
            };
            switch (persistency) {
            case 1: return valid_in_last(8) >= 8;   // never true for an invalid current frame
            case 2: return valid_in_last(3) >= 2;
            case 3: return valid_in_last(4) >= 2;
            case 4: return valid_in_last(8) >= 2;
            case 5: return valid_in_last(2) >= 1;
            case 6: return valid_in_last(5) >= 1;
            case 7: return valid_in_last(8) >= 1;
            case 8: return true;
            default: return false;
            }
        }

    private:
        std::vector<float> prev_ {};
        std::vector<std::uint8_t> history_ {};
    };

    // --- Hole filling -------------------------------------------------------
    inline void HoleFilling(std::vector<float>& img, int width, int height, int mode)
    {
        if (width <= 0 || height <= 0) return;
        if (mode == 0) {   // fill from left (propagates along the row)
            for (int y = 0; y < height; ++y) {
                float last = std::numeric_limits<float>::quiet_NaN();
                for (int x = 0; x < width; ++x) {
                    float& v = img[static_cast<std::size_t>(y) * width + x];
                    if (IsValidDepth(v)) last = v;
                    else if (IsValidDepth(last)) v = last;
                }
            }
            return;
        }
        const std::vector<float> src = img;
        auto at = [&](int x, int y) {
            if (x < 0 || y < 0 || x >= width || y >= height) return std::numeric_limits<float>::quiet_NaN();
            return src[static_cast<std::size_t>(y) * width + x];
        };
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float& v = img[static_cast<std::size_t>(y) * width + x];
                if (IsValidDepth(v)) continue;
                const std::array<float, 4> n {at(x - 1, y), at(x + 1, y), at(x, y - 1), at(x, y + 1)};
                float best = std::numeric_limits<float>::quiet_NaN();
                for (float c : n) {
                    if (!IsValidDepth(c)) continue;
                    if (!IsValidDepth(best) || (mode == 1 ? c > best : c < best)) best = c;
                }
                v = best;
            }
        }
    }

    // --- Chain --------------------------------------------------------------
    class DepthPostProcessor
    {
    public:
        void Reset() { temporal_.Reset(); }

        // `fx_px` is the focal length of the input (before decimation).
        // Updates width/height when decimation is enabled.
        void Apply(std::vector<float>& depth_m, int& width, int& height,
                   double fx_px, double baseline_m, const PostProcessingConfig& c)
        {
            if (!c.enabled) return;
            if (c.decimation.enabled && c.decimation.magnitude > 1) {
                const int w0 = width;
                Decimate(depth_m, width, height, c.decimation.magnitude);
                fx_px *= static_cast<double>(width) / static_cast<double>(w0);
            }
            if (c.threshold.enabled) Threshold(depth_m, c.threshold.min_m, c.threshold.max_m);

            const bool smooth = c.spatial.enabled || c.temporal.enabled;
            if (smooth) {
                if (c.disparity_domain) {
                    DepthToDisparity(depth_m, fx_px, baseline_m);
                } else {
                    for (auto& v : depth_m) v *= 1000.0F;   // delta in mm
                }
                if (c.spatial.enabled) SpatialFilter(depth_m, width, height, c.spatial);
                if (c.temporal.enabled) temporal_.Apply(depth_m, c.temporal);
                if (c.disparity_domain) {
                    DisparityToDepth(depth_m, fx_px, baseline_m);
                } else {
                    for (auto& v : depth_m) v *= 0.001F;
                }
            }
            if (c.hole_filling.enabled) HoleFilling(depth_m, width, height, c.hole_filling.mode);
        }

    private:
        TemporalFilter temporal_ {};
    };

    // Output size of the chain for an input size (decimation only changes it).
    inline void PostProcessedSize(const PostProcessingConfig& c, int& width, int& height)
    {
        if (c.enabled && c.decimation.enabled && c.decimation.magnitude > 1) {
            width /= c.decimation.magnitude;
            height /= c.decimation.magnitude;
        }
    }
}
