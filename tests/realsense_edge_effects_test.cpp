// Unit test for RS-1 depth-edge effects (occlusion shadow + flying pixels) and the
// depth quality metrics. Backend-free: no MuJoCo runtime, GL or PDU.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_depth_metrics.hpp"
#include "sensors/camera/realsense/realsense_depth_model.hpp"
#include "sensors/camera/realsense/realsense_edge_effects.hpp"
#include "sensors/camera/realsense/realsense_post_processing.hpp"

namespace rs = hako::robots::sensor::camera::realsense;

static int g_failures = 0;
static int g_checks = 0;
static const float kNaN = std::numeric_limits<float>::quiet_NaN();

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

// D435 at 848 px: fx = 848 / (2 tan 43.5 deg) ~ 446.9 px, B = 50 mm.
static const double kFx = rs::FocalLengthPx(848, 87.0 * 3.14159265358979323846 / 180.0);
static const double kB = 0.05;

// Step image: background z_bg everywhere, foreground z_fg in columns [x0, x1).
static std::vector<float> StepImage(int w, int h, int x0, int x1, float z_fg, float z_bg)
{
    std::vector<float> img(static_cast<std::size_t>(w) * h, z_bg);
    for (int y = 0; y < h; ++y)
        for (int x = x0; x < x1; ++x) img[static_cast<std::size_t>(y) * w + x] = z_fg;
    return img;
}

static rs::EdgeEffectsParams Params(bool occlusion, bool flying, double probability)
{
    rs::EdgeEffectsParams p {};
    p.fx_px = kFx;
    p.baseline_m = kB;
    p.occlusion = occlusion;
    p.flying_pixels = flying;
    p.window_radius_px = 4;
    p.jump_ratio = 0.05;
    p.probability = probability;
    p.min_z_m = 0.1;
    p.max_z_m = 10.0;
    return p;
}

static void TestOcclusion()
{
    std::printf("[occlusion shadow]\n");
    const int w = 300, h = 8;
    const int x0 = 150, x1 = 200;
    auto img = StepImage(w, h, x0, x1, 0.5F, 1.0F);
    std::mt19937 rng(1);
    const auto st = rs::ApplyEdgeEffects(img, w, h, Params(true, false, 0.0), rng);

    const double want = rs::OcclusionShadowPx(kFx, kB, 0.5, 1.0);   // ~22.3 px
    int row_shadow = 0;
    for (int x = x0 - 1; x >= 0 && !std::isfinite(img[3 * w + x]); --x) ++row_shadow;
    close_to(row_shadow, want, 1.0, "shadow width left of the object = fx*B*(1/z_fg - 1/z_bg)");
    check(std::isfinite(img[3 * w + x0 - row_shadow - 1]), "background valid beyond the shadow");
    bool fg_ok = true;
    for (int x = x0; x < x1; ++x) fg_ok = fg_ok && img[3 * w + x] == 0.5F;
    check(fg_ok, "foreground untouched");
    bool right_ok = true;
    for (int x = x1; x < w; ++x) right_ok = right_ok && std::isfinite(img[3 * w + x]);
    check(right_ok, "no shadow on the right side");
    check(st.occluded == static_cast<std::size_t>(row_shadow) * h, "stats count the occluded pixels");

    // A plane slanted away to the right (z grows with x) stays fully visible.
    std::vector<float> slant(static_cast<std::size_t>(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) slant[static_cast<std::size_t>(y) * w + x] = 0.5F + 0.01F * x;
    const auto st2 = rs::ApplyEdgeEffects(slant, w, h, Params(true, false, 0.0), rng);
    check(st2.occluded == 0, "slanted visible plane: no occlusion");

    // Nearer object gives a wider shadow (0.3 m vs 1.0 m).
    auto img3 = StepImage(w, h, x0, x1, 0.3F, 1.0F);
    rs::ApplyEdgeEffects(img3, w, h, Params(true, false, 0.0), rng);
    int s3 = 0;
    for (int x = x0 - 1; x >= 0 && !std::isfinite(img3[3 * w + x]); --x) ++s3;
    close_to(s3, rs::OcclusionShadowPx(kFx, kB, 0.3, 1.0), 1.0, "shadow width at z_fg = 0.3 m");
}

static void TestFlyingPixels()
{
    std::printf("[flying pixels]\n");
    const int w = 300, h = 400;
    const int x0 = 150, x1 = 200;
    const int r = 4;
    {
        auto img = StepImage(w, h, x0, x1, 0.5F, 1.0F);
        const auto orig = img;
        std::mt19937 rng(2);
        const auto st = rs::ApplyEdgeEffects(img, w, h, Params(false, true, 1.0), rng);
        bool inside_ok = true;
        bool outside_ok = true;
        std::size_t changed = 0;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const std::size_t i = static_cast<std::size_t>(y) * w + x;
                const bool near_edge = (x >= x0 - r && x <= x0 + r - 1) || (x >= x1 - r && x <= x1 + r - 1);
                const bool changed_px = img[i] != orig[i];
                if (changed_px) ++changed;
                if (near_edge) inside_ok = inside_ok && img[i] >= 0.5F && img[i] <= 1.0F;
                else outside_ok = outside_ok && !changed_px;
            }
        }
        check(outside_ok, "no flying pixel farther than r from an edge");
        check(inside_ok, "flying pixels lie between foreground and background");
        check(changed == static_cast<std::size_t>(4 * r) * h, "probability 1: every window-spanning pixel flies");
        check(st.flying == changed, "stats count the flying pixels");
    }
    {
        auto img = StepImage(w, h, x0, x1, 0.5F, 1.0F);
        const auto orig = img;
        std::mt19937 rng(3);
        rs::ApplyEdgeEffects(img, w, h, Params(false, true, 0.5), rng);
        std::size_t changed = 0;
        for (std::size_t i = 0; i < img.size(); ++i) changed += img[i] != orig[i] ? 1u : 0u;
        const double n = 4.0 * r * h;
        close_to(changed / n, 0.5, 3.0 * std::sqrt(0.25 / n), "flying fraction = probability");
    }
    {
        // Step below jump_ratio (2 % < 5 %): no flying pixels.
        auto img = StepImage(w, 8, x0, x1, 1.0F, 1.02F);
        std::mt19937 rng(4);
        const auto st = rs::ApplyEdgeEffects(img, w, 8, Params(false, true, 1.0), rng);
        check(st.flying == 0, "small step below jump_ratio: no flying pixels");
    }
    {
        // A smooth steep slope (2 %/px < jump_ratio) is not a discontinuity.
        std::vector<float> slope(static_cast<std::size_t>(w) * 8);
        for (int y = 0; y < 8; ++y)
            for (int x = 0; x < w; ++x) slope[static_cast<std::size_t>(y) * w + x] = 0.5F * std::pow(1.02F, static_cast<float>(x));
        std::mt19937 rng(6);
        const auto st = rs::ApplyEdgeEffects(slope, w, 8, Params(false, true, 1.0), rng);
        check(st.flying == 0, "smooth steep slope: no flying pixels");
    }
    {
        // Background beyond max_z is "nothing": it does not mix into the foreground.
        auto img = StepImage(w, 8, x0, x1, 1.0F, 20.0F);
        std::mt19937 rng(5);
        const auto st = rs::ApplyEdgeEffects(img, w, 8, Params(false, true, 1.0), rng);
        check(st.flying == 0, "no mix with background beyond max_z");
    }
    {
        // Same seed -> same output (deterministic replay).
        auto a = StepImage(w, 32, x0, x1, 0.5F, 1.0F);
        auto b = a;
        std::mt19937 ra(7), rb(7);
        rs::ApplyEdgeEffects(a, w, 32, Params(true, true, 0.5), ra);
        rs::ApplyEdgeEffects(b, w, 32, Params(true, true, 0.5), rb);
        bool same = true;
        for (std::size_t i = 0; i < a.size(); ++i)
            same = same && ((std::isnan(a[i]) && std::isnan(b[i])) || a[i] == b[i]);
        check(same, "same seed -> bit-identical output");
    }
    check(rs::FlyingWindowRadiusPx(7.0, 848) == 4, "window radius 4 px at 848 px");
    check(rs::FlyingWindowRadiusPx(7.0, 424) == 2, "window radius 2 px at 424 px");
    check(rs::FlyingWindowRadiusPx(7.0, 1280) == 5, "window radius 5 px at 1280 px");
    check(rs::FlyingWindowRadiusPx(7.0, 64) == 1, "window radius at least 1 px");
}

static void TestMetrics()
{
    std::printf("[depth metrics]\n");
    rs::DepthModelParams m {};
    m.fx_px = kFx;
    m.baseline_m = kB;
    m.disparity_sigma_px = 0.08;
    m.min_z_m = 0.1;
    m.max_z_m = 10.0;
    // 10 truth pixels at 1 m (sigma_z ~ 3.6 mm -> outlier threshold ~ 10.7 mm),
    // one out of range, one invalid truth.
    std::vector<float> truth(12, 1.0F);
    truth[10] = 20.0F;
    truth[11] = kNaN;
    std::vector<float> meas = truth;
    meas[0] = kNaN;            // missing
    meas[1] = kNaN;            // missing
    meas[2] = 1.10F;           // outlier (flying)
    meas[3] = 1.003F;          // inlier
    const auto r = rs::ComputeDepthMetrics(truth, meas, m);
    check(r.truth_valid == 10, "truth valid count excludes out-of-range / invalid");
    check(r.missing == 2 && r.outliers == 1, "2 missing, 1 outlier");
    close_to(r.missing_rate, 0.2, 1e-12, "missing rate");
    close_to(r.outlier_rate, 0.1, 1e-12, "outlier rate");
    close_to(r.rms_m, std::sqrt(0.003 * 0.003 / 7.0), 1e-6, "inlier RMS");
}

// Flying pixels through the full chain (edges -> stereo noise -> SDK filters).
static void TestChainWithFilters()
{
    std::printf("[chain: edges -> depth model -> post-processing]\n");
    const int w = 848, h = 120;
    const auto truth = StepImage(w, h, 350, 500, 0.6F, 1.0F);
    rs::DepthModelParams m {};
    m.fx_px = kFx;
    m.baseline_m = kB;
    m.disparity_sigma_px = 0.08;
    m.min_z_m = 0.168;
    m.max_z_m = 10.0;

    auto run = [&](bool edges, bool filters) {
        auto img = truth;
        std::mt19937 er(11), dr(12);
        if (edges) rs::ApplyEdgeEffects(img, w, h, Params(true, true, 0.5), er);
        rs::ApplyDepthModel(img, w, h, m, dr);
        if (filters) {
            rs::PostProcessingConfig pc {};
            pc.enabled = true;
            pc.spatial.enabled = true;
            pc.temporal.enabled = false;
            rs::DepthPostProcessor post;
            int ww = w, hh = h;
            post.Apply(img, ww, hh, m.fx_px, m.baseline_m, pc);
        }
        return rs::ComputeDepthMetrics(truth, img, m);
    };
    const auto base = run(false, false);
    const auto edge = run(true, false);
    const auto filt = run(true, true);
    std::printf("  outlier rate: no edges %.4f, edges %.4f, edges+spatial %.4f\n",
                base.outlier_rate, edge.outlier_rate, filt.outlier_rate);
    std::printf("  missing rate: no edges %.4f, edges %.4f, edges+spatial %.4f\n",
                base.missing_rate, edge.missing_rate, filt.missing_rate);
    check(edge.outlier_rate > base.outlier_rate + 0.005, "edges add outliers");
    check(edge.missing_rate > base.missing_rate, "edges add missing pixels (shadow)");
    // The spatial filter pulls part of the flying pixels back toward a side
    // (measured ~ -35 %), but leaves most edges as they are.
    check(filt.outlier_rate < edge.outlier_rate, "spatial filter reduces flying pixels");
    check(filt.outlier_rate > base.outlier_rate, "but does not remove them");
}

static void TestTiming()
{
    std::printf("[timing 848x480]\n");
    const int w = 848, h = 480;
    const auto src = StepImage(w, h, 350, 500, 0.6F, 1.0F);
    std::mt19937 rng(1);
    const int reps = 10;
    double total_ms = 0.0;
    for (int i = 0; i < reps; ++i) {
        auto img = src;
        const auto t0 = std::chrono::steady_clock::now();
        rs::ApplyEdgeEffects(img, w, h, Params(true, true, 0.5), rng);
        total_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
    std::printf("  ApplyEdgeEffects: %.2f ms / frame (occlusion + flying, r = 4)\n", total_ms / reps);
    check(total_ms / reps < 100.0, "edge effects run well under a frame budget");
}

static void TestConfig()
{
    std::printf("[config]\n");
    rs::RealSenseConfig c {};
    std::string err;
    check(rs::RealSenseConfigFromJson(nlohmann::json {{"model", "D435i"}}, c, err), "default config");
    check(!c.edge_effects.enabled, "edge effects default off");
    const auto off = rs::MakeEdgeEffectsParams(c, rs::MakeDepthModelParams(c));
    check(!off.occlusion && !off.flying_pixels, "off -> no occlusion, no flying");

    const auto j = nlohmann::json::parse(R"({
        "model": "D435i",
        "edge_effects": { "enabled": true, "occlusion": false, "seed": 9,
                          "flying_pixels": { "window_px": 9, "jump_ratio": 0.1, "probability": 0.3 } }
    })");
    check(rs::RealSenseConfigFromJson(j, c, err), "edge_effects JSON parses");
    const auto p = rs::MakeEdgeEffectsParams(c, rs::MakeDepthModelParams(c));
    check(!p.occlusion && p.flying_pixels, "occlusion off, flying on");
    check(p.window_radius_px == 5 && p.jump_ratio == 0.1 && p.probability == 0.3 && c.edge_effects.seed == 9,
          "flying parameters read");

    auto bad = j;
    bad["edge_effects"]["flying_pixels"]["probability"] = 1.5;
    check(!rs::RealSenseConfigFromJson(bad, c, err), "probability > 1 rejected");
}

int main()
{
    TestOcclusion();
    TestFlyingPixels();
    TestMetrics();
    TestChainWithFilters();
    TestTiming();
    TestConfig();
    std::printf("realsense_edge_effects_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
