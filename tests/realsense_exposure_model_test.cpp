// Unit test for RS-5: auto / manual exposure, gain noise, IR scene level and
// motion blur of depth and images. Backend-free.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_exposure_model.hpp"

namespace rs = hako::robots::sensor::camera::realsense;

static int g_failures = 0;
static int g_checks = 0;

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

static const double kTref = 8.5e-3;
static const double kTmax = 1.0 / 30.0;

static void TestAutoExposure()
{
    std::printf("[auto / manual exposure]\n");
    auto d = rs::AutoExposure(1.0, kTref, 16, 1e-4, kTmax, 248);
    close_to(d.exposure_s, kTref, 1e-12, "level 1: reference exposure");
    close_to(d.gain, 16, 1e-12, "level 1: reference gain");
    close_to(d.brightness, 1.0, 1e-12, "level 1: correctly exposed");
    d = rs::AutoExposure(4.0, kTref, 16, 1e-4, kTmax, 248);
    close_to(d.exposure_s, kTref / 4, 1e-12, "bright scene: shorter exposure");
    d = rs::AutoExposure(0.2, kTref, 16, 1e-4, kTmax, 248);
    close_to(d.exposure_s, kTmax, 1e-12, "dark scene: exposure capped at 1/fps");
    close_to(d.gain, 16 * (kTref / 0.2) / kTmax, 1e-9, "dark scene: the rest goes to the gain");
    close_to(d.brightness, 1.0, 1e-12, "still correctly exposed");
    d = rs::AutoExposure(0.01, kTref, 16, 1e-4, kTmax, 248);
    close_to(d.gain, 248, 1e-12, "very dark: gain capped");
    close_to(d.brightness, kTmax * 248 * 0.01 / (kTref * 16), 1e-12, "very dark: under-exposed");
    const auto m = rs::ManualExposure(1.0, 2 * kTref, 16, kTref, 16);
    close_to(m.brightness, 2.0, 1e-12, "manual: double exposure -> twice as bright");
    close_to(rs::GainNoiseScale(64, 16), 2.0, 1e-12, "gain x4 -> noise x2");
}

static void TestLevels()
{
    std::printf("[scene levels]\n");
    rs::IrMaterial white {};
    white.ir_reflectance = 0.8;
    const std::vector<float> d1(100, 1.0F);
    close_to(rs::IrSceneLevel(d1, nullptr, {}, white, 150, 0.1), 1.0, 1e-12, "white wall at 1 m indoors = 1");
    const std::vector<float> d2(100, 2.0F);
    close_to(rs::IrSceneLevel(d2, nullptr, {}, white, 150, 0.1), (0.25 + 1.0) / 2.0, 1e-12, "at 2 m: 0.625");
    close_to(rs::IrSceneLevel(d2, nullptr, {}, white, 0, 0.1), 0.5, 1e-12, "projector off: ambient only");
    std::vector<std::uint8_t> grey(300, 128);
    close_to(rs::MeanLuma(grey, 3), 128.0 / 255.0, 1e-12, "mean luma");
    close_to(rs::ExpectedBlurPx(3.0, 0.01, 446.9), 13.407, 1e-9, "blur length = w t fx");
}

static rs::PinholeIntrinsics K(int w, int h)
{
    const double fx = rs::FocalLengthPx(w, 87.0 * 3.14159265358979323846 / 180.0);
    return {fx, fx, w * 0.5, h * 0.5};
}

static void TestDepthBlur()
{
    std::printf("[depth motion blur]\n");
    const int w = 400, h = 40;
    const auto k = K(w, h);
    const double fb = k.fx * 0.05;
    std::vector<float> src(static_cast<std::size_t>(w) * h, 1.0F);
    for (int y = 0; y < h; ++y)
        for (int x = 170; x < 230; ++x) src[static_cast<std::size_t>(y) * w + x] = 0.5F;
    rs::CameraMotion still {};
    rs::CameraMotion yaw {};
    yaw.wy = 3.0;   // about the optical y (vertical) axis
    {
        auto d = src;
        rs::ApplyMotionBlurDepth(d, w, h, k, still, 0.01, 0.0, 4, fb, 1.0);
        check(d == src, "no motion: unchanged");
        auto e = src;
        rs::CameraMotion zero_t = yaw;
        rs::ApplyMotionBlurDepth(e, w, h, k, zero_t, 0.0, 0.0, 1, fb, 1.0);
        check(e == src, "one sub-frame at zero exposure: unchanged (bit-identical)");
    }
    const auto band = [&](const std::vector<float>& d) {
        // away from the image sides (the rotated sub-frames look outside the frame there)
        int n = 0;
        for (int x = 50; x < w - 50; ++x) n += std::isfinite(d[static_cast<std::size_t>(h / 2) * w + x]) ? 0 : 1;
        return n;
    };
    for (double t : {0.004, 0.01}) {
        auto d = src;
        const auto st = rs::ApplyMotionBlurDepth(d, w, h, k, yaw, t, 0.0, 4, fb, 1.0);
        const double span = 0.75 * rs::ExpectedBlurPx(3.0, t, k.fx);   // (N-1)/N of the blur
        char msg[96];
        std::snprintf(msg, sizeof(msg), "exposure %.0f ms: smeared band ~ 2 x (N-1)/N x blur", t * 1e3);
        close_to(band(d), 2.0 * span, 4.0, msg);
        check(st.smeared > 0, "stats count the smeared pixels");
        // the flat parts stay valid and close to the truth
        close_to(d[static_cast<std::size_t>(h / 2) * w + 200], 0.5, 0.002, "inside the box: still ~0.5 m");
        close_to(d[static_cast<std::size_t>(h / 2) * w + 300], 1.0, 0.01, "background: still ~1.0 m");
    }
}

static void TestImageBlur()
{
    std::printf("[image motion blur]\n");
    const int w = 400, h = 20;
    const auto k = K(w, h);
    std::vector<std::uint8_t> img(static_cast<std::size_t>(w) * h, 0);
    for (int y = 0; y < h; ++y)
        for (int x = 198; x < 202; ++x) img[static_cast<std::size_t>(y) * w + x] = 255;
    rs::CameraMotion yaw {};
    yaw.wy = 3.0;
    auto b = img;
    rs::ApplyMotionBlurImage(b, w, h, 1, k, yaw, 0.01, 0.0, 8, nullptr);
    int lit = 0, max_v = 0;
    for (int x = 0; x < w; ++x) {
        const int v = b[static_cast<std::size_t>(h / 2) * w + x];
        lit += v > 0 ? 1 : 0;
        max_v = std::max(max_v, v);
    }
    const double blur = rs::ExpectedBlurPx(3.0, 0.01, k.fx);
    std::printf("  bar 4 px -> lit %d px (blur %.1f px), peak %d\n", lit, blur, max_v);
    close_to(lit, 4.0 + 7.0 / 8.0 * blur, 3.0, "bar smears over (N-1)/N of the blur length");
    check(max_v < 255, "smeared bar is dimmer than the original");
    auto same = img;
    rs::ApplyMotionBlurImage(same, w, h, 1, k, rs::CameraMotion {}, 0.01, 0.0, 8, nullptr);
    check(same == img, "no motion: unchanged");
}

static void TestBrightnessNoise()
{
    std::printf("[brightness / gain noise]\n");
    std::vector<std::uint8_t> img(100000, 100);
    std::mt19937 rng(1);
    auto a = img;
    rs::ApplyBrightnessAndGainNoise(a, 1.0, 0.0, rng);
    check(a == img, "brightness 1, no noise: unchanged");
    auto b = img;
    rs::ApplyBrightnessAndGainNoise(b, 2.0, 0.0, rng);
    check(b[0] == 200, "brightness 2: doubled");
    auto c = img;
    rs::ApplyBrightnessAndGainNoise(c, 3.0, 0.0, rng);
    check(c[0] == 255, "over-exposed: clipped");
    auto d = img;
    rs::ApplyBrightnessAndGainNoise(d, 1.0, 4.0, rng);
    double s = 0, s2 = 0;
    for (auto v : d) { s += v; s2 += static_cast<double>(v) * v; }
    const double mean = s / d.size();
    close_to(std::sqrt(s2 / d.size() - mean * mean), 4.0, 0.1, "gain noise sigma");
}

static void TestTiming()
{
    std::printf("[timing 848x480, 4 sub-frames]\n");
    const int w = 848, h = 480;
    const auto k = K(w, h);
    std::vector<float> src(static_cast<std::size_t>(w) * h, 1.0F);
    rs::CameraMotion yaw {};
    yaw.wy = 3.0;
    double total = 0.0;
    for (int i = 0; i < 5; ++i) {
        auto d = src;
        const auto t0 = std::chrono::steady_clock::now();
        rs::ApplyMotionBlurDepth(d, w, h, k, yaw, 0.01, 0.0, 4, k.fx * 0.05, 1.0);
        total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
    std::printf("  ApplyMotionBlurDepth: %.2f ms / frame\n", total / 5);
    check(total / 5 < 200.0, "depth blur runs");
}

static void TestConfig()
{
    std::printf("[config]\n");
    rs::RealSenseConfig c {};
    std::string err;
    check(rs::RealSenseConfigFromJson(nlohmann::json {{"model", "D435"}}, c, err), "default config");
    check(!c.exposure.enabled, "exposure default off");
    close_to(c.exposure.MaxExposureS(30.0), 1.0 / 30.0, 1e-12, "max exposure 1/fps by default");
    const auto j = nlohmann::json::parse(R"({ "model": "D435",
        "exposure": { "enabled": true, "auto": false, "depth_exposure_us": 20000, "depth_gain": 32,
                      "max_exposure_us": 20000, "rgb_light_level": 0.3, "seed": 4,
                      "motion_blur": { "subframes": 8, "disparity_spread_px": 0.5 } } })");
    check(rs::RealSenseConfigFromJson(j, c, err), "exposure JSON parses: " + err);
    const auto& e = c.exposure;
    check(e.enabled && !e.auto_exposure && e.depth_exposure_us == 20000 && e.depth_gain == 32 && e.rgb_light_level == 0.3
              && e.motion_blur.subframes == 8 && e.motion_blur.disparity_spread_px == 0.5 && e.seed == 4,
          "exposure parameters read");
    close_to(e.MaxExposureS(30.0), 0.02, 1e-12, "explicit max exposure");
    auto bad = j;
    bad["exposure"]["motion_blur"]["subframes"] = 0;
    check(!rs::RealSenseConfigFromJson(bad, c, err), "subframes 0 rejected");
}

int main()
{
    TestAutoExposure();
    TestLevels();
    TestDepthBlur();
    TestImageBlur();
    TestBrightnessNoise();
    TestTiming();
    TestConfig();
    std::printf("realsense_exposure_model_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
