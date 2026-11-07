#pragma once

// Depth-edge effects of a stereo depth camera (RS-1).
//
// Pure math on a z-depth image (float metres, NaN = invalid): no MuJoCo/GL.
// Applied to the rendered (and rolling-shutter distorted) depth, before the
// stereo depth model (realsense_depth_model.hpp):
//
//   1. occlusion shadow: depth is referenced to the left imager. A surface point
//      at column x projects to x_r = x - d(x) (d = fx*B/z) in the right image.
//      If a pixel further right already projects to x_r or further left, the
//      right imager sees that (nearer) surface instead -> no match -> invalid.
//      This gives the dark band on the LEFT of foreground objects with width
//      fx*B*(1/z_fg - 1/z_bg) px, exactly as on D400 cameras. Deterministic.
//   2. flying pixels  : near a depth discontinuity the matching window contains
//      both the foreground and the background, and the matcher reports a depth
//      in between. A discontinuity is a pair of neighbouring pixels whose depths
//      differ by more than jump_ratio * (the nearer one); a smooth steep slope is
//      not one. Pixels whose (2r+1)x(2r+1) window contains such a pair are
//      replaced, with `probability`, by z_lo + u * (z_hi - z_lo), u ~ U(0,1),
//      where z_lo / z_hi are the window's nearest / farthest in-range depths.
//
// The window radius r scales with the stream width (reference 848 px), since the
// matching window has a fixed size in sensor pixels. Defaults are assumptions:
// the D4 ASIC window size is not published.
//
// Cost: O(N) (integral images find the windows that contain a pair); the window
// min / max is computed only for those pixels.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace hako::robots::sensor::camera::realsense
{
    struct EdgeEffectsParams
    {
        double fx_px {0.0};
        double baseline_m {0.05};
        bool occlusion {true};
        bool flying_pixels {true};
        int window_radius_px {4};
        double jump_ratio {0.05};
        double probability {0.5};
        // Only depths inside the matcher's range take part in the flying-pixel
        // mix (background beyond max_z is "nothing", not a far surface).
        double min_z_m {0.0};
        double max_z_m {std::numeric_limits<double>::infinity()};
    };

    struct EdgeEffectsStats
    {
        std::size_t occluded {0};
        std::size_t flying {0};
    };

    // Reference stream width for `window_px` (D435 848 px).
    constexpr int kEdgeWindowRefWidth = 848;

    // Window radius for a stream of `width` px; window_px is given at 848 px.
    inline int FlyingWindowRadiusPx(double window_px_ref, int width)
    {
        const double w = window_px_ref * static_cast<double>(width) / kEdgeWindowRefWidth;
        return std::max(1, static_cast<int>(std::lround(w * 0.5)));
    }

    // Expected occlusion-shadow width [px] left of a foreground edge.
    inline double OcclusionShadowPx(double fx_px, double baseline_m, double z_fg, double z_bg)
    {
        if (z_fg <= 0.0 || z_bg <= 0.0) return 0.0;
        return std::max(0.0, fx_px * baseline_m * (1.0 / z_fg - 1.0 / z_bg));
    }

    // Marks pixels hidden from the right imager (1 = occluded). Right-to-left scan:
    // pixel x is occluded when x - d(x) >= min over x' > x of (x' - d(x')).
    inline void ComputeOcclusionMask(
        const std::vector<float>& depth, int width, int height, double fb,
        std::vector<unsigned char>& mask)
    {
        const std::size_t n = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        mask.assign(n, 0);
        if (fb <= 0.0) return;
        for (int y = 0; y < height; ++y) {
            const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
            double min_xr = std::numeric_limits<double>::infinity();
            for (int x = width - 1; x >= 0; --x) {
                const double z = static_cast<double>(depth[row + static_cast<std::size_t>(x)]);
                if (!std::isfinite(z) || z <= 0.0) continue;
                const double xr = static_cast<double>(x) - fb / z;
                if (xr >= min_xr) {
                    mask[row + static_cast<std::size_t>(x)] = 1;
                } else {
                    min_xr = xr;
                }
            }
        }
    }

    namespace detail
    {
        // Summed-area table (width+1)x(height+1) of a 0/1 mask.
        inline void IntegralImage(const std::vector<unsigned char>& m, int w, int h, std::vector<std::int32_t>& s)
        {
            const std::size_t sw = static_cast<std::size_t>(w) + 1;
            s.assign(sw * (static_cast<std::size_t>(h) + 1), 0);
            for (int y = 0; y < h; ++y) {
                std::int32_t row = 0;
                for (int x = 0; x < w; ++x) {
                    row += m[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)];
                    s[(static_cast<std::size_t>(y) + 1) * sw + static_cast<std::size_t>(x) + 1]
                        = s[static_cast<std::size_t>(y) * sw + static_cast<std::size_t>(x) + 1] + row;
                }
            }
        }

        // Sum of the mask over x in [x0, x1], y in [y0, y1] (clamped; empty -> 0).
        inline std::int32_t BoxSum(const std::vector<std::int32_t>& s, int w, int h, int x0, int y0, int x1, int y1)
        {
            x0 = std::max(x0, 0);
            y0 = std::max(y0, 0);
            x1 = std::min(x1, w - 1);
            y1 = std::min(y1, h - 1);
            if (x0 > x1 || y0 > y1) return 0;
            const std::size_t sw = static_cast<std::size_t>(w) + 1;
            const auto at = [&](int x, int y) { return s[static_cast<std::size_t>(y) * sw + static_cast<std::size_t>(x)]; };
            return at(x1 + 1, y1 + 1) - at(x0, y1 + 1) - at(x1 + 1, y0) + at(x0, y0);
        }
    }

    // Applies occlusion shadows and flying pixels in place.
    inline EdgeEffectsStats ApplyEdgeEffects(
        std::vector<float>& depth, int width, int height,
        const EdgeEffectsParams& p, std::mt19937& rng)
    {
        EdgeEffectsStats st {};
        if (width <= 0 || height <= 0) return st;
        const std::size_t n = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        if (depth.size() != n) return st;
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        const double fb = p.fx_px * p.baseline_m;

        std::vector<unsigned char> occluded;
        if (p.occlusion) {
            ComputeOcclusionMask(depth, width, height, fb, occluded);
        }

        if (p.flying_pixels && p.probability > 0.0 && p.window_radius_px > 0) {
            const auto idx = [width](int x, int y) {
                return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
            };
            const auto in_range = [&](float v) {
                return std::isfinite(v) && v >= p.min_z_m && v <= p.max_z_m;
            };
            const auto jump = [&](float a, float b) {
                if (!in_range(a) || !in_range(b)) return false;
                const double lo = std::min(a, b);
                return static_cast<double>(std::max(a, b)) - lo > p.jump_ratio * lo;
            };
            // Discontinuity pairs: (x, x+1) marked at x, (y, y+1) marked at y.
            std::vector<unsigned char> eh(n, 0);
            std::vector<unsigned char> ev(n, 0);
            bool any = false;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    const float v = depth[idx(x, y)];
                    if (x + 1 < width && jump(v, depth[idx(x + 1, y)])) { eh[idx(x, y)] = 1; any = true; }
                    if (y + 1 < height && jump(v, depth[idx(x, y + 1)])) { ev[idx(x, y)] = 1; any = true; }
                }
            }
            if (any) {
                std::vector<std::int32_t> sh;
                std::vector<std::int32_t> sv;
                detail::IntegralImage(eh, width, height, sh);
                detail::IntegralImage(ev, width, height, sv);
                const int r = p.window_radius_px;
                const std::vector<float> src = depth;   // windows read the unmodified depth
                std::uniform_real_distribution<double> uniform(0.0, 1.0);
                for (int y = 0; y < height; ++y) {
                    for (int x = 0; x < width; ++x) {
                        const std::size_t i = idx(x, y);
                        if (!in_range(src[i])) continue;
                        if (!occluded.empty() && occluded[i]) continue;
                        // Window [x-r, x+r] x [y-r, y+r] contains a whole pair.
                        if (detail::BoxSum(sh, width, height, x - r, y - r, x + r - 1, y + r) == 0
                            && detail::BoxSum(sv, width, height, x - r, y - r, x + r, y + r - 1) == 0) {
                            continue;
                        }
                        if (uniform(rng) >= p.probability) continue;
                        float lo = std::numeric_limits<float>::infinity();
                        float hi = -std::numeric_limits<float>::infinity();
                        for (int yy = std::max(0, y - r); yy <= std::min(height - 1, y + r); ++yy) {
                            for (int xx = std::max(0, x - r); xx <= std::min(width - 1, x + r); ++xx) {
                                const float v = src[idx(xx, yy)];
                                if (!in_range(v)) continue;
                                lo = std::min(lo, v);
                                hi = std::max(hi, v);
                            }
                        }
                        depth[i] = static_cast<float>(lo + uniform(rng) * (hi - lo));
                        ++st.flying;
                    }
                }
            }
        }

        if (!occluded.empty()) {
            for (std::size_t i = 0; i < n; ++i) {
                if (occluded[i] && std::isfinite(depth[i])) {
                    depth[i] = nan_val;
                    ++st.occluded;
                }
            }
        }
        return st;
    }
}
