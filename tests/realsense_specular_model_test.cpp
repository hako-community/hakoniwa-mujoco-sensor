// Unit test for RS-3: specular highlights and inter-reflection mirror images.
// Backend-free: an analytic ray caster replaces mj_ray.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_specular_model.hpp"

namespace rs = hako::robots::sensor::camera::realsense;

static int g_failures = 0;
static int g_checks = 0;
static const double kPi = 3.14159265358979323846;

static void check(bool cond, const std::string& msg)
{
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::printf("  [FAIL] %s\n", msg.c_str());
    }
}

static void close_to(double a, double b, double tol, const std::string& msg)
{
    ++g_checks;
    const bool ok = std::fabs(a - b) <= tol;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s (got %.5f, want %.5f +- %.5f)\n", ok ? "ok" : "FAIL", msg.c_str(), a, b, tol);
}

static rs::SpecularModelParams Params(int w, int h)
{
    rs::SpecularModelParams p {};
    p.fx_px = rs::FocalLengthPx(w, 87.0 * kPi / 180.0);
    p.fy_px = p.fx_px;
    p.cx_px = w * 0.5;
    p.cy_px = h * 0.5;
    return p;
}

static rs::TextureModelParams Tp(const rs::SpecularModelParams& s)
{
    rs::TextureModelParams t {};
    t.fx_px = s.fx_px;
    t.fy_px = s.fy_px;
    t.cx_px = s.cx_px;
    t.cy_px = s.cy_px;
    return t;
}

// Plane tilted by phi about the vertical axis, z0 on the optical axis:
// sin(phi) X + cos(phi) Z = z0 cos(phi).
static std::vector<float> TiltedPlane(const rs::SpecularModelParams& p, int w, int h, double z0, double phi)
{
    std::vector<float> img(static_cast<std::size_t>(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const double xn = (x - p.cx_px) / p.fx_px;
            img[static_cast<std::size_t>(y) * w + x] =
                static_cast<float>(z0 * std::cos(phi) / (std::sin(phi) * xn + std::cos(phi)));
        }
    return img;
}

static rs::IrMaterial Shiny(double specular, double rho = 0.6)
{
    rs::IrMaterial m {};
    m.ir_reflectance = rho;
    m.specular = specular;
    return m;
}

static std::size_t Holes(const std::vector<float>& img)
{
    std::size_t n = 0;
    for (float v : img) n += std::isfinite(v) ? 0u : 1u;
    return n;
}

static void TestNormal()
{
    std::printf("[surface normal]\n");
    const int w = 200, h = 160;
    const auto p = Params(w, h);
    const double phi = 45.0 * kPi / 180.0;
    const auto img = TiltedPlane(p, w, h, 1.0, phi);
    double n[3];
    check(rs::SurfaceNormal(img, w, h, w / 2, h / 2, Tp(p), n), "normal estimable");
    close_to(n[0], -std::sin(phi), 1e-6, "normal x (toward the camera)");
    close_to(n[1], 0.0, 1e-6, "normal y");
    close_to(n[2], -std::cos(phi), 1e-6, "normal z (toward the camera)");
}

static void TestHighlight()
{
    std::printf("[highlight]\n");
    const int w = 200, h = 160;
    auto p = Params(w, h);
    p.interreflection = false;
    p.projector_offset_m[0] = 0.0;
    const std::vector<float> wall(static_cast<std::size_t>(w) * h, 0.6F);
    const auto mat = Shiny(1.0);

    // expected count from the probability of every pixel
    double sum_p = 0.0, var = 0.0, centre_p = 0.0, edge_p = 0.0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const double z = 0.6;
            const double pt[3] = {z * (x - p.cx_px) / p.fx_px, z * (y - p.cy_px) / p.fy_px, z};
            const double n[3] = {0.0, 0.0, -1.0};
            const double q = rs::HighlightProbability(p, mat, pt, n);
            sum_p += q;
            var += q * (1 - q);
            if (x == w / 2 && y == h / 2) centre_p = q;
            if (x == 5 && y == h / 2) edge_p = q;
        }
    close_to(centre_p, 1.0, 1e-3, "face-on centre: highlight probability ~ specular");
    check(edge_p < 1e-6, "40 deg off the axis: no highlight");

    auto img = wall;
    std::mt19937 rng(1);
    const auto st = rs::ApplySpecularModel(img, w, h, nullptr, {}, mat, p, {}, rng);
    close_to(static_cast<double>(st.highlight_holes), sum_p, 4.0 * std::sqrt(var) + 1.0, "hole count = sum of p");
    check(Holes(img) == st.highlight_holes, "holes are the highlight pixels");
    check(!std::isfinite(img[static_cast<std::size_t>(h / 2) * w + w / 2]), "centre is a hole");

    auto off = p;
    off.laser_power_mw = 0.0;
    auto img2 = wall;
    check(rs::ApplySpecularModel(img2, w, h, nullptr, {}, mat, off, {}, rng).highlight_holes == 0,
          "projector off: no highlight");
    auto img3 = wall;
    check(rs::ApplySpecularModel(img3, w, h, nullptr, {}, Shiny(0.0), p, {}, rng).highlight_holes == 0,
          "matte material: no highlight");
    // A 45 deg tilted plate mirrors the projector away from the camera in the
    // image centre (only rays ~45 deg off the axis, at the image edge, see it face-on).
    auto tilt = TiltedPlane(p, w, h, 0.6, 45.0 * kPi / 180.0);
    rs::ApplySpecularModel(tilt, w, h, nullptr, {}, mat, p, {}, rng);
    std::size_t centre_holes = 0;
    for (int y = 0; y < h; ++y)
        for (int x = w / 4; x < 3 * w / 4; ++x)
            centre_holes += std::isfinite(tilt[static_cast<std::size_t>(y) * w + x]) ? 0u : 1u;
    check(centre_holes == 0, "45 deg plate: no highlight in the image centre");
}

// Mirror wall at optical x = x_wall (ray going -x hits it).
static rs::RayCastFn WallCaster(double x_wall, double rho, int* calls, int* hits = nullptr)
{
    return [x_wall, rho, calls, hits](const double o[3], const double d[3], double max_dist, rs::IrMaterial& hit) {
        if (calls) ++*calls;
        if (d[0] >= -1e-12) return -1.0;
        const double t = (x_wall - o[0]) / d[0];
        if (t < 0.0 || t > max_dist) return -1.0;
        if (hits) ++*hits;
        hit.ir_reflectance = rho;
        hit.transparent = false;
        return t;
    };
}

static void TestMirror()
{
    std::printf("[inter-reflection mirror image]\n");
    const int w = 200, h = 160;
    auto p = Params(w, h);
    p.highlight = false;
    p.ray_stride_px = 1;
    p.max_distance_m = 1.0;
    const double phi = 45.0 * kPi / 180.0;
    const auto plate = TiltedPlane(p, w, h, 1.0, phi);   // reflects the axis ray to -x
    const std::size_t c = static_cast<std::size_t>(h / 2) * w + w / 2;
    {
        auto img = plate;
        std::mt19937 rng(2);
        int calls = 0;
        int hits = 0;
        const auto st = rs::ApplySpecularModel(img, w, h, nullptr, {}, Shiny(1.0), p,
                                               WallCaster(-0.5, 1.0, &calls, &hits), rng);
        close_to(img[c], 1.5, 0.005, "mirror image at z + t (1.0 m + 0.5 m)");
        check(hits > 1000 && st.mirror_pixels == static_cast<std::size_t>(hits) && st.ray_hits == st.mirror_pixels,
              "probability 1: every pixel whose ray hits is mirrored");
        check(st.rays == static_cast<std::size_t>(calls), "ray count");
        // every mirrored pixel lies behind the plate
        bool behind = true;
        for (std::size_t i = 0; i < img.size(); ++i)
            if (std::isfinite(img[i]) && img[i] != plate[i]) behind = behind && img[i] > plate[i];
        check(behind, "mirror images are behind the mirror");
    }
    {
        auto q = p;
        q.ray_stride_px = 4;
        auto img = plate;
        std::mt19937 rng(3);
        int calls = 0;
        const auto st = rs::ApplySpecularModel(img, w, h, nullptr, {}, Shiny(1.0), q, WallCaster(-0.5, 1.0, &calls), rng);
        check(st.rays <= static_cast<std::size_t>((w / 4) * (h / 4)), "stride 4: one ray per 4x4 block");
        close_to(img[c], 1.5, 0.02, "stride 4: centre still at z + t");
    }
    {
        auto img = plate;
        std::mt19937 rng(4);
        int hits = 0;
        const auto st = rs::ApplySpecularModel(img, w, h, nullptr, {}, Shiny(0.8), p,
                                               WallCaster(-0.5, 0.5, nullptr, &hits), rng);
        const double frac = static_cast<double>(st.mirror_pixels) / hits;
        close_to(frac, 0.8 * 0.5, 3.0 * std::sqrt(0.24 / hits) + 0.005, "mirror probability = gain * specular * rho_hit");
    }
    {
        auto q = p;
        q.max_distance_m = 0.3;
        auto img = plate;
        std::mt19937 rng(5);
        rs::ApplySpecularModel(img, w, h, nullptr, {}, Shiny(1.0), q, WallCaster(-0.5, 1.0, nullptr), rng);
        close_to(img[c], 1.0, 1e-6, "wall beyond max_distance: no mirror image at the centre");
    }
    {
        auto img = plate;
        std::mt19937 rng(6);
        int calls = 0;
        rs::ApplySpecularModel(img, w, h, nullptr, {}, Shiny(0.3), p, WallCaster(-0.5, 1.0, &calls), rng);
        check(calls == 0, "specular below min_specular: no rays");
    }
    {
        // material ids: only the shiny half reflects
        auto img = plate;
        std::vector<int> ids(img.size());
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) ids[static_cast<std::size_t>(y) * w + x] = x < w / 2 ? 0 : 1;
        const std::vector<rs::IrMaterial> by_id {Shiny(1.0), Shiny(0.0)};
        std::mt19937 rng(7);
        rs::ApplySpecularModel(img, w, h, &ids, by_id, Shiny(0.0), p, WallCaster(-0.5, 1.0, nullptr), rng);
        bool right_same = true;
        for (int y = 0; y < h; ++y)
            for (int x = w / 2; x < w; ++x) {
                const std::size_t i = static_cast<std::size_t>(y) * w + x;
                right_same = right_same && img[i] == plate[i];
            }
        check(right_same, "matte half untouched");
    }
}

static void TestTiming()
{
    std::printf("[timing 848x480, all pixels shiny, analytic caster]\n");
    const int w = 848, h = 480;
    const auto p = Params(w, h);
    const auto src = TiltedPlane(p, w, h, 0.8, 45.0 * kPi / 180.0);
    std::mt19937 rng(1);
    double total = 0.0;
    std::size_t rays = 0;
    for (int i = 0; i < 5; ++i) {
        auto img = src;
        const auto t0 = std::chrono::steady_clock::now();
        rays = rs::ApplySpecularModel(img, w, h, nullptr, {}, Shiny(0.8), p, WallCaster(-0.5, 0.8, nullptr), rng).rays;
        total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
    std::printf("  ApplySpecularModel: %.2f ms / frame, %zu rays (stride 4), caster excluded\n", total / 5, rays);
    check(total / 5 < 100.0, "specular model (without ray cost) under a frame budget");
}

static void TestConfig()
{
    std::printf("[config]\n");
    rs::RealSenseConfig c {};
    std::string err;
    check(rs::RealSenseConfigFromJson(nlohmann::json {{"model", "D435"}}, c, err), "default config");
    check(!c.specular_model.enabled, "specular model default off");
    auto p = rs::MakeSpecularModelParams(c);
    close_to(p.projector_offset_m[0], 0.025, 1e-12, "projector at +B/2 by default (D435 B = 50 mm)");
    const auto j = nlohmann::json::parse(R"({ "model": "D435",
        "texture_model": { "laser_power_mw": 0 },
        "specular_model": { "enabled": true, "highlight_width_deg": 5, "ray_stride_px": 2,
                            "projector_offset_m": [0.01, 0.0, 0.0], "mirror_gain": 0.7, "seed": 3 } })");
    check(rs::RealSenseConfigFromJson(j, c, err), "specular_model JSON parses: " + err);
    p = rs::MakeSpecularModelParams(c);
    check(p.highlight_width_deg == 5.0 && p.ray_stride_px == 2 && p.mirror_gain == 0.7
              && p.projector_offset_m[0] == 0.01 && p.laser_power_mw == 0.0,
          "specular parameters read (laser power from texture_model)");
    auto bad = j;
    bad["specular_model"]["ray_stride_px"] = 0;
    check(!rs::RealSenseConfigFromJson(bad, c, err), "ray_stride_px 0 rejected");
    bad = j;
    bad["specular_model"]["projector_offset_m"] = {0.01};
    check(!rs::RealSenseConfigFromJson(bad, c, err), "projector_offset_m needs 3 values");
}

int main()
{
    TestNormal();
    TestHighlight();
    TestMirror();
    TestTiming();
    TestConfig();
    std::printf("realsense_specular_model_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
