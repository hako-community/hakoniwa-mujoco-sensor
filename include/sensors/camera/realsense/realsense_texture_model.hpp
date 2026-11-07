#pragma once

// Material table + "can the matcher see a pattern here?" model (RS-2).
//
// Pure (nlohmann only): no MuJoCo/GL. D400 cameras are active stereo: an IR
// projector paints a dot pattern and the left/right IR images are matched. A
// pixel gets no depth when the pattern contrast is too weak:
//   - dark / IR-absorbing material, far surface, grazing incidence,
//   - strong ambient IR (sunlight) washing the pattern out,
//   - projector off on a texture-less surface,
//   - transparent material (the pattern goes through).
//
// Per pixel (normalised units; the indoor white perpendicular wall at
// reference_range_m has projector SNR = 1 without ambient light):
//
//   C   = P * rho * cos(theta) * (R_ref / z)^2        projector pattern
//   N   = 1 + k * A * rho                             ambient raises the noise
//   SNR = C / N + g * A * texture                      + the surface's own texture
//   p_valid = 1 / (1 + (1 / SNR)^gamma)                -> hole with 1 - p_valid
//   sigma_d scale = 1 + c / SNR                        weaker pattern, noisier match
//
//   P = laser_power_mw / 150 (D435 default 150 mW), A = ambient_ir (indoor 0.1,
//   sunlight ~1.0), theta from the depth-image normal.
//
// All material values and gains are ASSUMPTIONS chosen to follow the usual
// tendencies (white wall valid to ~6 m indoors, black parts losing depth beyond
// ~2 m, sunlight washing the projector out while textured surfaces stay valid).
// They are not measured; override them in the JSON.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace hako::robots::sensor::camera::realsense
{
    struct IrMaterial
    {
        double ir_reflectance {0.8};   // diffuse IR (~850 nm) reflectance 0..1
        double specular {0.0};         // mirror-like strength 0..1 (used by RS-3)
        double texture {0.0};          // the surface's own pattern 0..1 (passive stereo)
        bool transparent {false};      // pattern passes through -> no depth
    };

    // Built-in presets (assumed values).
    inline std::map<std::string, IrMaterial> BuiltinIrMaterials()
    {
        return {
            {"machined_aluminum", {0.60, 0.80, 0.10, false}},
            {"ground_steel", {0.50, 0.70, 0.10, false}},
            {"cast_iron", {0.15, 0.10, 0.40, false}},
            {"black_oxide", {0.05, 0.30, 0.10, false}},
            {"black_resin", {0.05, 0.20, 0.00, false}},
            {"white_resin", {0.80, 0.10, 0.00, false}},
            {"cardboard", {0.70, 0.00, 0.50, false}},
            {"concrete", {0.40, 0.00, 0.60, false}},
            {"glass", {0.05, 0.90, 0.00, true}},
        };
    }

    // `*` matches any run of characters (also empty); everything else literally.
    inline bool WildcardMatch(const std::string& pattern, const std::string& name)
    {
        std::size_t p = 0, s = 0, star = std::string::npos, mark = 0;
        while (s < name.size()) {
            if (p < pattern.size() && pattern[p] == '*') {
                star = p++;
                mark = s;
            } else if (p < pattern.size() && pattern[p] == name[s]) {
                ++p;
                ++s;
            } else if (star != std::string::npos) {
                p = star + 1;
                s = ++mark;
            } else {
                return false;
            }
        }
        while (p < pattern.size() && pattern[p] == '*') ++p;
        return p == pattern.size();
    }

    // MuJoCo material name (or pattern) -> IR material.
    struct MaterialTable
    {
        std::map<std::string, IrMaterial> presets {BuiltinIrMaterials()};
        std::map<std::string, IrMaterial> assign {};     // name or pattern with '*'
        IrMaterial default_material {BuiltinIrMaterials().at("white_resin")};

        // Exact name first; otherwise the longest matching pattern (most specific).
        IrMaterial Lookup(const std::string& material_name) const
        {
            if (material_name.empty()) return default_material;
            const auto it = assign.find(material_name);
            if (it != assign.end()) return it->second;
            const IrMaterial* best = nullptr;
            std::size_t best_len = 0;
            for (const auto& [pattern, mat] : assign) {
                if (pattern.find('*') == std::string::npos) continue;
                if (WildcardMatch(pattern, material_name) && (best == nullptr || pattern.size() > best_len)) {
                    best = &mat;
                    best_len = pattern.size();
                }
            }
            return best ? *best : default_material;
        }

        // Index = MuJoCo material id; pixels with id -1 (no material) use the default.
        std::vector<IrMaterial> Resolve(const std::vector<std::string>& material_names) const
        {
            std::vector<IrMaterial> out;
            out.reserve(material_names.size());
            for (const auto& n : material_names) out.push_back(Lookup(n));
            return out;
        }
    };

    namespace detail
    {
        inline bool ReadIrMaterial(const nlohmann::json& j, const std::map<std::string, IrMaterial>& presets,
                                   IrMaterial& out, std::string& error)
        {
            if (j.is_string()) {
                const auto it = presets.find(j.get<std::string>());
                if (it == presets.end()) {
                    error = "unknown material preset '" + j.get<std::string>() + "'";
                    return false;
                }
                out = it->second;
                return true;
            }
            if (!j.is_object()) {
                error = "material must be a preset name or an object";
                return false;
            }
            if (j.contains("base")) {
                if (!ReadIrMaterial(j.at("base"), presets, out, error)) return false;
            }
            if (j.contains("ir_reflectance")) out.ir_reflectance = j.at("ir_reflectance").get<double>();
            if (j.contains("specular")) out.specular = j.at("specular").get<double>();
            if (j.contains("texture")) out.texture = j.at("texture").get<double>();
            if (j.contains("transparent")) out.transparent = j.at("transparent").get<bool>();
            const auto in01 = [](double v) { return v >= 0.0 && v <= 1.0; };
            if (!in01(out.ir_reflectance) || !in01(out.specular) || !in01(out.texture)) {
                error = "material values must be in [0, 1]";
                return false;
            }
            return true;
        }
    }

    // { "presets": { name: {...} }, "assign": { material_name_or_pattern: preset | {...} },
    //   "default": preset | {...} }   Presets extend / override the built-in ones.
    inline bool MaterialTableFromJson(const nlohmann::json& j, MaterialTable& out, std::string& error)
    {
        try {
            if (!j.is_object()) {
                error = "material table must be an object";
                return false;
            }
            MaterialTable t {};
            if (j.contains("presets")) {
                for (const auto& [name, v] : j.at("presets").items()) {
                    IrMaterial m {};
                    if (!detail::ReadIrMaterial(v, t.presets, m, error)) {
                        error = "preset '" + name + "': " + error;
                        return false;
                    }
                    t.presets[name] = m;
                }
            }
            if (j.contains("assign")) {
                for (const auto& [name, v] : j.at("assign").items()) {
                    IrMaterial m {};
                    if (!detail::ReadIrMaterial(v, t.presets, m, error)) {
                        error = "assign '" + name + "': " + error;
                        return false;
                    }
                    t.assign[name] = m;
                }
            }
            if (j.contains("default")) {
                IrMaterial m {};
                if (!detail::ReadIrMaterial(j.at("default"), t.presets, m, error)) {
                    error = "default: " + error;
                    return false;
                }
                t.default_material = m;
            }
            out = t;
            return true;
        } catch (const std::exception& e) {
            error = std::string("material table type error: ") + e.what();
            return false;
        }
    }

    struct TextureModelParams
    {
        double laser_power_mw {150.0};    // 0 = projector off
        double ambient_ir {0.1};          // indoor 0.1, sunlight ~1.0 (assumed scale)
        double reference_range_m {10.0};  // white perpendicular wall: SNR = 1 here (assumed)
        double steepness {6.0};           // gamma of the valid probability (assumed)
        double noise_gain {0.5};          // c: sigma_d scale = 1 + c / SNR (assumed)
        double ambient_noise_gain {10.0}; // k (assumed)
        double passive_gain {20.0};       // g (assumed)
        double max_sigma_scale {10.0};
        // depth-stream intrinsics for the normals
        double fx_px {0.0};
        double fy_px {0.0};
        double cx_px {0.0};
        double cy_px {0.0};
    };

    inline double TextureSnr(const TextureModelParams& p, const IrMaterial& m, double z, double cos_theta)
    {
        if (m.transparent || z <= 0.0) return 0.0;
        const double power = std::max(0.0, p.laser_power_mw) / 150.0;
        const double r = p.reference_range_m / z;
        const double c = power * m.ir_reflectance * std::max(0.0, cos_theta) * r * r;
        const double n = 1.0 + p.ambient_noise_gain * p.ambient_ir * m.ir_reflectance;
        return c / n + p.passive_gain * p.ambient_ir * m.texture;
    }

    inline double ValidProbability(const TextureModelParams& p, double snr)
    {
        if (!(snr > 0.0)) return 0.0;
        const double t = 1.0 / snr;
        const double g = p.steepness;
        double tg;
        if (g == std::floor(g) && g >= 1.0 && g <= 16.0) {   // integer gamma: no pow()
            tg = 1.0;
            for (int k = 0; k < static_cast<int>(g); ++k) tg *= t;
        } else {
            tg = std::pow(t, g);
        }
        return 1.0 / (1.0 + tg);
    }

    // Above this SNR the hole probability is below 1e-8: treated as always valid
    // (no random draw), which keeps the model cheap on well-lit surfaces.
    inline double AlwaysValidSnr(const TextureModelParams& p)
    {
        return std::pow(1.0e8, 1.0 / std::max(p.steepness, 1.0e-3));
    }

    inline double SigmaScale(const TextureModelParams& p, double snr)
    {
        if (!(snr > 0.0)) return p.max_sigma_scale;
        return std::min(p.max_sigma_scale, 1.0 + p.noise_gain / snr);
    }

    // Distance where the valid probability drops to 50 % (SNR = 1), or +inf / 0.
    inline double HalfValidRange(const TextureModelParams& p, const IrMaterial& m, double cos_theta = 1.0)
    {
        const double passive = p.passive_gain * p.ambient_ir * m.texture;
        if (m.transparent) return 0.0;
        if (passive >= 1.0) return std::numeric_limits<double>::infinity();
        const double power = std::max(0.0, p.laser_power_mw) / 150.0;
        const double n = 1.0 + p.ambient_noise_gain * p.ambient_ir * m.ir_reflectance;
        const double k = power * m.ir_reflectance * cos_theta / n;   // C/N = k (R/z)^2
        if (k <= 0.0) return 0.0;
        return p.reference_range_m * std::sqrt(k / (1.0 - passive));
    }

    namespace detail
    {
        // Normalised image coordinates per column / row: (x - cx) / fx, (y - cy) / fy.
        struct RayTable
        {
            std::vector<double> xs;
            std::vector<double> ys;
            RayTable(int w, int h, const TextureModelParams& p) : xs(static_cast<std::size_t>(std::max(w, 0))),
                                                                  ys(static_cast<std::size_t>(std::max(h, 0)))
            {
                for (int x = 0; x < w; ++x) xs[static_cast<std::size_t>(x)] = (x - p.cx_px) / p.fx_px;
                for (int y = 0; y < h; ++y) ys[static_cast<std::size_t>(y)] = (y - p.cy_px) / p.fy_px;
            }
        };

        inline double SurfaceCosineFast(const float* depth, int w, int h, int x, int y, const RayTable& rt)
        {
            const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            const float z0 = depth[i];
            const auto usable = [z0](float v) {
                return std::isfinite(v) && v > 0.0F && std::fabs(v - z0) <= 0.05F * std::min(v, z0);
            };
            const float zl = x > 0 ? depth[i - 1] : std::numeric_limits<float>::quiet_NaN();
            const float zr = x + 1 < w ? depth[i + 1] : std::numeric_limits<float>::quiet_NaN();
            const float zu = y > 0 ? depth[i - static_cast<std::size_t>(w)] : std::numeric_limits<float>::quiet_NaN();
            const float zd = y + 1 < h ? depth[i + static_cast<std::size_t>(w)] : std::numeric_limits<float>::quiet_NaN();
            const bool l = usable(zl), r = usable(zr), u = usable(zu), d = usable(zd);
            if (!(l || r) || !(u || d)) return 1.0;
            const double xc = rt.xs[static_cast<std::size_t>(x)];
            const double yc = rt.ys[static_cast<std::size_t>(y)];
            // horizontal difference (b - a)
            const int xa = l ? x - 1 : x, xb = r ? x + 1 : x;
            const double za = l ? zl : z0, zb = r ? zr : z0;
            const double du0 = zb * rt.xs[static_cast<std::size_t>(xb)] - za * rt.xs[static_cast<std::size_t>(xa)];
            const double du1 = (zb - za) * yc;
            const double du2 = zb - za;
            // vertical difference
            const int ya = u ? y - 1 : y, yb = d ? y + 1 : y;
            const double wa = u ? zu : z0, wb = d ? zd : z0;
            const double dv0 = (wb - wa) * xc;
            const double dv1 = wb * rt.ys[static_cast<std::size_t>(yb)] - wa * rt.ys[static_cast<std::size_t>(ya)];
            const double dv2 = wb - wa;
            const double n0 = du1 * dv2 - du2 * dv1;
            const double n1 = du2 * dv0 - du0 * dv2;
            const double n2 = du0 * dv1 - du1 * dv0;
            const double nn = n0 * n0 + n1 * n1 + n2 * n2;
            if (nn <= 0.0) return 1.0;
            const double vn = xc * xc + yc * yc + 1.0;
            return std::fabs(n0 * xc + n1 * yc + n2) / std::sqrt(nn * vn);
        }
    }

    // |cos| between the surface normal (from the depth image) and the viewing ray.
    // Neighbours across a step (> 5 % depth) or invalid are not used; without a
    // usable neighbour in a direction the surface is taken as facing the camera.
    inline double SurfaceCosine(const std::vector<float>& depth, int w, int h, int x, int y,
                                const TextureModelParams& p)
    {
        if (p.fx_px <= 0.0 || p.fy_px <= 0.0) return 1.0;
        const detail::RayTable rt(w, h, p);
        return detail::SurfaceCosineFast(depth.data(), w, h, x, y, rt);
    }

    // Reference implementation (kept for the tests): same result as SurfaceCosine.
    inline double SurfaceCosineReference(const std::vector<float>& depth, int w, int h, int x, int y,
                                         const TextureModelParams& p)
    {
        const auto at = [&](int xx, int yy) {
            return depth[static_cast<std::size_t>(yy) * static_cast<std::size_t>(w) + static_cast<std::size_t>(xx)];
        };
        const float z0 = at(x, y);
        const auto usable = [&](int xx, int yy) {
            if (xx < 0 || yy < 0 || xx >= w || yy >= h) return false;
            const float v = at(xx, yy);
            return std::isfinite(v) && v > 0.0F && std::fabs(v - z0) <= 0.05F * std::min(v, z0);
        };
        const auto point = [&](int xx, int yy, double out[3]) {
            const double z = at(xx, yy);
            out[0] = z * (xx - p.cx_px) / p.fx_px;
            out[1] = z * (yy - p.cy_px) / p.fy_px;
            out[2] = z;
        };
        const auto diff = [&](int ax, int ay, int bx, int by, double out[3]) {
            double a[3], b[3];
            point(ax, ay, a);
            point(bx, by, b);
            for (int k = 0; k < 3; ++k) out[k] = b[k] - a[k];
        };
        double du[3], dv[3];
        const bool l = usable(x - 1, y), r = usable(x + 1, y), u = usable(x, y - 1), d = usable(x, y + 1);
        if (!(l || r) || !(u || d) || p.fx_px <= 0.0 || p.fy_px <= 0.0) return 1.0;
        diff(l ? x - 1 : x, y, r ? x + 1 : x, y, du);
        diff(x, u ? y - 1 : y, x, d ? y + 1 : y, dv);
        const double n[3] = {du[1] * dv[2] - du[2] * dv[1], du[2] * dv[0] - du[0] * dv[2], du[0] * dv[1] - du[1] * dv[0]};
        const double v[3] = {(x - p.cx_px) / p.fx_px, (y - p.cy_px) / p.fy_px, 1.0};
        const double nn = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        const double vn = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (nn <= 0.0) return 1.0;
        return std::fabs(n[0] * v[0] + n[1] * v[1] + n[2] * v[2]) / (nn * vn);
    }

    struct TextureStats
    {
        std::size_t holes {0};        // weak pattern
        std::size_t transparent {0};  // pattern passed through
    };

    // Turns weak-pattern pixels into holes and returns the per-pixel disparity
    // sigma scale (1 for invalid pixels). `material_ids` (-1 = none) may be null:
    // then every pixel uses `fallback`.
    inline TextureStats ApplyTextureModel(
        std::vector<float>& depth, int w, int h,
        const std::vector<int>* material_ids,
        const std::vector<IrMaterial>& materials_by_id,
        const IrMaterial& fallback,
        const TextureModelParams& p,
        std::mt19937& rng,
        std::vector<float>& sigma_scale)
    {
        TextureStats st {};
        const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
        sigma_scale.assign(n, 1.0F);
        if (w <= 0 || h <= 0 || depth.size() != n) return st;
        const bool use_ids = material_ids != nullptr && material_ids->size() == n;
        const std::vector<float> src = depth;   // normals from the unmodified depth
        const bool have_rays = p.fx_px > 0.0 && p.fy_px > 0.0;
        const detail::RayTable rt(have_rays ? w : 0, have_rays ? h : 0, p);
        const double always_valid = AlwaysValidSnr(p);
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
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
                if (m->transparent) {
                    depth[i] = nan_val;
                    ++st.transparent;
                    continue;
                }
                const double cos_t = have_rays ? detail::SurfaceCosineFast(src.data(), w, h, x, y, rt) : 1.0;
                const double snr = TextureSnr(p, *m, z, cos_t);
                if (snr < always_valid && uniform(rng) >= ValidProbability(p, snr)) {
                    depth[i] = nan_val;
                    ++st.holes;
                    continue;
                }
                sigma_scale[i] = static_cast<float>(SigmaScale(p, snr));
            }
        }
        return st;
    }
}
