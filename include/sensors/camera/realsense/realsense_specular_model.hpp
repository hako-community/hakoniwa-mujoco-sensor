#pragma once

// Specular highlights and inter-reflections of shiny surfaces (RS-3).
//
// Pure: no MuJoCo/GL. Ray casting is injected (RayCastFn), so the MuJoCo
// camera passes mj_ray and the tests pass an analytic scene.
//
// For stereo (D400) the classic ToF multipath is not the main issue. On shiny
// metal parts in a bin two things break the depth:
//
//   1. highlight (saturation): the surface normal n is close to the half vector h
//      between the directions to the projector and to the imager, so the
//      projector light is mirrored straight into the imager and saturates it:
//         p_hole = highlight_gain * specular * exp(-(alpha / alpha0)^2) * min(1, P)
//      alpha = angle(n, h), alpha0 = highlight_width_deg, P = laser_power_mw / 150.
//   2. inter-reflection (mirror image): a shiny surface mirrors its neighbour
//      (another part, the bin wall). Stereo matches the mirrored pattern as if it
//      were BEHIND the mirror, at the distance travelled by the light:
//         z' = z * (range + t) / range
//      range = |P| (camera to the surface point), t = reflected ray length to the
//      hit. Probability: p = mirror_gain * specular * rho_hit (clamped to 1).
//      Rays are cast only from pixels with specular >= min_specular, once per
//      ray_stride_px x ray_stride_px block (the block shares the hit distance).
//
// Defaults are ASSUMPTIONS (alpha0 = 8 deg, gains 1, projector at +B/2 on x).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <random>
#include <vector>

#include "sensors/camera/realsense/realsense_texture_model.hpp"

namespace hako::robots::sensor::camera::realsense
{
    // Casts a ray in the depth-camera optical frame (x right, y down, z forward).
    // Returns the hit distance (< 0: no hit within max_dist) and the hit material.
    using RayCastFn = std::function<double(const double origin[3], const double dir[3], double max_dist,
                                           IrMaterial& hit_material)>;

    struct SpecularModelParams
    {
        bool highlight {true};
        double highlight_width_deg {8.0};
        double highlight_gain {1.0};
        double laser_power_mw {150.0};
        double projector_offset_m[3] {0.025, 0.0, 0.0};   // from the left imager (assumed +B/2)
        bool interreflection {true};
        double max_distance_m {0.5};
        int ray_stride_px {4};
        double min_specular {0.5};
        double mirror_gain {1.0};
        std::size_t max_rays {40000};
        // depth-stream intrinsics
        double fx_px {0.0};
        double fy_px {0.0};
        double cx_px {0.0};
        double cy_px {0.0};
    };

    struct SpecularStats
    {
        std::size_t highlight_holes {0};
        std::size_t mirror_pixels {0};
        std::size_t rays {0};
        std::size_t ray_hits {0};   // rays that hit something within max_distance
    };

    namespace detail
    {
        // Unit surface normal from the depth image, oriented toward the camera.
        // Same neighbour rule as SurfaceCosineFast; false when not estimable.
        inline bool SurfaceNormalFast(const float* depth, int w, int h, int x, int y, const RayTable& rt, double n[3])
        {
            const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            const float z0 = depth[i];
            const auto usable = [z0](float v) {
                return std::isfinite(v) && v > 0.0F && std::fabs(v - z0) <= 0.05F * std::min(v, z0);
            };
            const float nan_f = std::numeric_limits<float>::quiet_NaN();
            const float zl = x > 0 ? depth[i - 1] : nan_f;
            const float zr = x + 1 < w ? depth[i + 1] : nan_f;
            const float zu = y > 0 ? depth[i - static_cast<std::size_t>(w)] : nan_f;
            const float zd = y + 1 < h ? depth[i + static_cast<std::size_t>(w)] : nan_f;
            const bool l = usable(zl), r = usable(zr), u = usable(zu), d = usable(zd);
            if (!(l || r) || !(u || d)) return false;
            const double xc = rt.xs[static_cast<std::size_t>(x)];
            const double yc = rt.ys[static_cast<std::size_t>(y)];
            const int xa = l ? x - 1 : x, xb = r ? x + 1 : x;
            const double za = l ? zl : z0, zb = r ? zr : z0;
            const double du[3] = {zb * rt.xs[static_cast<std::size_t>(xb)] - za * rt.xs[static_cast<std::size_t>(xa)],
                                  (zb - za) * yc, zb - za};
            const int ya = u ? y - 1 : y, yb = d ? y + 1 : y;
            const double wa = u ? zu : z0, wb = d ? zd : z0;
            const double dv[3] = {(wb - wa) * xc,
                                  wb * rt.ys[static_cast<std::size_t>(yb)] - wa * rt.ys[static_cast<std::size_t>(ya)],
                                  wb - wa};
            n[0] = du[1] * dv[2] - du[2] * dv[1];
            n[1] = du[2] * dv[0] - du[0] * dv[2];
            n[2] = du[0] * dv[1] - du[1] * dv[0];
            const double nn = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (nn <= 0.0) return false;
            // toward the camera: n . (point) < 0, point = z * (xc, yc, 1)
            const double s = (n[0] * xc + n[1] * yc + n[2]) > 0.0 ? -1.0 / nn : 1.0 / nn;
            for (int k = 0; k < 3; ++k) n[k] *= s;
            return true;
        }

        inline void Normalize(double v[3])
        {
            const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (l > 0.0) for (int k = 0; k < 3; ++k) v[k] /= l;
        }
    }

    // Unit normal toward the camera at (x, y), from the depth image (tests / tools).
    inline bool SurfaceNormal(const std::vector<float>& depth, int w, int h, int x, int y,
                              const TextureModelParams& p, double n[3])
    {
        if (p.fx_px <= 0.0 || p.fy_px <= 0.0) return false;
        const detail::RayTable rt(w, h, p);
        return detail::SurfaceNormalFast(depth.data(), w, h, x, y, rt, n);
    }

    // Highlight hole probability at a surface point (optical frame) with normal n.
    inline double HighlightProbability(const SpecularModelParams& p, const IrMaterial& m,
                                       const double point[3], const double n[3])
    {
        if (!p.highlight || m.specular <= 0.0 || p.highlight_width_deg <= 0.0) return 0.0;
        const double power = std::min(1.0, std::max(0.0, p.laser_power_mw) / 150.0);
        if (power <= 0.0) return 0.0;
        double to_cam[3] = {-point[0], -point[1], -point[2]};
        double to_proj[3] = {p.projector_offset_m[0] - point[0], p.projector_offset_m[1] - point[1],
                             p.projector_offset_m[2] - point[2]};
        detail::Normalize(to_cam);
        detail::Normalize(to_proj);
        double hv[3] = {to_cam[0] + to_proj[0], to_cam[1] + to_proj[1], to_cam[2] + to_proj[2]};
        detail::Normalize(hv);
        const double c = std::clamp(n[0] * hv[0] + n[1] * hv[1] + n[2] * hv[2], -1.0, 1.0);
        // beyond 3.5 widths the probability is below e^-12: skip acos / exp
        const double cut = std::cos(std::min(3.5 * p.highlight_width_deg, 180.0) * 3.14159265358979323846 / 180.0);
        if (c < cut) return 0.0;
        const double alpha = std::acos(c) * 180.0 / 3.14159265358979323846;
        const double a = alpha / p.highlight_width_deg;
        return std::min(1.0, p.highlight_gain * m.specular * std::exp(-a * a) * power);
    }

    // Applies highlights and mirror images in place. `cast` may be empty (no
    // inter-reflection). `material_ids` (-1 = none) may be null -> `fallback`.
    inline SpecularStats ApplySpecularModel(
        std::vector<float>& depth, int w, int h,
        const std::vector<int>* material_ids,
        const std::vector<IrMaterial>& materials_by_id,
        const IrMaterial& fallback,
        const SpecularModelParams& p,
        const RayCastFn& cast,
        std::mt19937& rng)
    {
        SpecularStats st {};
        const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
        if (w <= 0 || h <= 0 || depth.size() != n || p.fx_px <= 0.0 || p.fy_px <= 0.0) return st;
        const bool use_ids = material_ids != nullptr && material_ids->size() == n;
        const std::vector<float> src = depth;
        TextureModelParams tp {};
        tp.fx_px = p.fx_px;
        tp.fy_px = p.fy_px;
        tp.cx_px = p.cx_px;
        tp.cy_px = p.cy_px;
        const detail::RayTable rt(w, h, tp);
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        const int stride = std::max(1, p.ray_stride_px);
        const int bw = (w + stride - 1) / stride;
        const int bh = (h + stride - 1) / stride;
        struct Hit { double t; IrMaterial mat; };
        std::vector<Hit> hits;                               // per cast block: shared reflected-ray result
        std::vector<int> block_hit(static_cast<std::size_t>(bw) * static_cast<std::size_t>(bh), -1);

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
                const double z = static_cast<double>(src[i]);
                if (!std::isfinite(z) || z <= 0.0) continue;
                const IrMaterial* m = &fallback;
                if (use_ids) {
                    const int id = (*material_ids)[i];
                    if (id >= 0 && static_cast<std::size_t>(id) < materials_by_id.size()) m = &materials_by_id[static_cast<std::size_t>(id)];
                }
                if (m->specular <= 0.0 || m->transparent) continue;
                double nrm[3];
                if (!detail::SurfaceNormalFast(src.data(), w, h, x, y, rt, nrm)) continue;
                const double pt[3] = {z * rt.xs[static_cast<std::size_t>(x)], z * rt.ys[static_cast<std::size_t>(y)], z};

                // 1. highlight
                const double ph = HighlightProbability(p, *m, pt, nrm);
                if (ph > 0.0 && uniform(rng) < ph) {
                    depth[i] = nan_val;
                    ++st.highlight_holes;
                    continue;
                }
                // 2. inter-reflection
                if (!p.interreflection || !cast || m->specular < p.min_specular) continue;
                const std::size_t block = static_cast<std::size_t>(y / stride) * static_cast<std::size_t>(bw)
                    + static_cast<std::size_t>(x / stride);
                if (block_hit[block] < 0) {
                    Hit hit {-1.0, fallback};
                    if (st.rays < p.max_rays) {
                        const double range = std::sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
                        const double d[3] = {pt[0] / range, pt[1] / range, pt[2] / range};
                        const double dn = d[0] * nrm[0] + d[1] * nrm[1] + d[2] * nrm[2];
                        double r[3] = {d[0] - 2.0 * dn * nrm[0], d[1] - 2.0 * dn * nrm[1], d[2] - 2.0 * dn * nrm[2]};
                        detail::Normalize(r);
                        const double eps = 1.0e-3;   // leave the surface itself
                        const double o[3] = {pt[0] + eps * r[0], pt[1] + eps * r[1], pt[2] + eps * r[2]};
                        IrMaterial hm = fallback;
                        const double t = cast(o, r, p.max_distance_m, hm);
                        ++st.rays;
                        if (t >= 0.0 && t <= p.max_distance_m) {
                            hit = Hit {t + eps, hm};
                            ++st.ray_hits;
                        }
                    }
                    block_hit[block] = static_cast<int>(hits.size());
                    hits.push_back(hit);
                }
                const Hit& hit = hits[static_cast<std::size_t>(block_hit[block])];
                if (hit.t < 0.0 || hit.mat.transparent) continue;
                const double pm = std::min(1.0, p.mirror_gain * m->specular * hit.mat.ir_reflectance);
                if (uniform(rng) >= pm) continue;
                const double range = std::sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
                depth[i] = static_cast<float>(z * (range + hit.t) / range);
                ++st.mirror_pixels;
            }
        }
        return st;
    }
}
