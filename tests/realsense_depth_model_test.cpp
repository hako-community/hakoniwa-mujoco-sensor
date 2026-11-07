// Unit test for the RealSense profile table, stereo depth model and JSON config.
// Backend-free: no MuJoCo runtime, GL or PDU.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_depth_model.hpp"
#include "sensors/camera/realsense/realsense_profile.hpp"

#ifndef HAKO_SENSOR_SOURCE_DIR
#define HAKO_SENSOR_SOURCE_DIR "."
#endif

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
    if (!(std::fabs(a - b) <= tol)) {
        ++g_failures;
        std::printf("  [FAIL] %s (got %.6f, want %.6f +- %.6f)\n", msg.c_str(), a, b, tol);
    }
}

static rs::RealSenseProfile Profile(const char* model)
{
    auto p = rs::FindProfile(model);
    check(p.has_value(), std::string("profile exists: ") + model);
    return p.value_or(rs::RealSenseProfile {});
}

// Sample std-dev of the model output for a flat wall at depth z.
static double MeasuredSigma(const rs::DepthModelParams& m, double z, int width, int samples_rows, double* mean_out)
{
    std::vector<float> img(static_cast<std::size_t>(width) * samples_rows, static_cast<float>(z));
    std::mt19937 rng(123);
    rs::ApplyDepthModel(img, width, samples_rows, m, rng);
    double sum = 0.0;
    double sum2 = 0.0;
    std::size_t n = 0;
    for (float v : img) {
        if (!std::isfinite(v)) continue;
        sum += v;
        sum2 += static_cast<double>(v) * v;
        ++n;
    }
    const double mean = n ? sum / n : 0.0;
    if (mean_out) *mean_out = mean;
    return n ? std::sqrt(std::max(0.0, sum2 / n - mean * mean)) : 0.0;
}

static void TestProfiles()
{
    std::printf("[profiles]\n");
    const auto names = rs::BuiltinModelNames();
    check(names.size() == 14, "14 built-in models");

    // Values from the PDF comparison table (p.19) and spec pages.
    auto d435 = Profile("D435");
    close_to(d435.depth_hfov_deg, 87.0, 1e-9, "D435 depth HFOV 87");
    close_to(d435.accuracy_ratio, 0.02, 1e-9, "D435 2%");
    close_to(d435.accuracy_ref_distance_m, 2.0, 1e-9, "D435 @2m");
    close_to(d435.min_z_m, 0.12, 1e-9, "D435 MinZ 12cm");
    close_to(d435.rgb_hfov_deg, 69.0, 1e-9, "D435 RGB 69");
    check(d435.rgb_shutter == rs::Shutter::Rolling, "D435 RGB rolling");
    check(!d435.has_imu, "D435 no IMU");
    check(Profile("D435i").has_imu, "D435i IMU");
    check(Profile("D435f").ir_filter == "ir_pass", "D435f IR pass");

    auto d421 = Profile("D421");
    check(!d421.has_rgb, "D421 no RGB");
    close_to(d421.depth_hfov_deg, 75.0, 1e-9, "D421 HFOV 75");
    close_to(d421.accuracy_ratio, 0.04, 1e-9, "D421 4%");

    auto d405 = Profile("D405");
    close_to(d405.min_z_m, 0.07, 1e-9, "D405 MinZ 7cm");
    close_to(d405.accuracy_ref_distance_m, 0.20, 1e-9, "D405 @20cm");
    check(!d405.active_projector, "D405 passive stereo");

    auto d415 = Profile("D415");
    close_to(d415.depth_hfov_deg, 65.0, 1e-9, "D415 HFOV 65");
    check(d415.depth_shutter == rs::Shutter::Rolling, "D415 rolling depth");

    auto d436 = Profile("D436");
    close_to(d436.min_z_m, 0.20, 1e-9, "D436 MinZ 20cm (p.38)");
    close_to(d436.rgb_hfov_deg, 90.0, 1e-9, "D436 RGB 90");
    check(d436.rgb_shutter == rs::Shutter::Global && d436.has_imu, "D436 global RGB + IMU");

    auto d455 = Profile("D455");
    close_to(d455.baseline_m, 0.095, 1e-12, "D455 baseline 95mm");
    close_to(d455.accuracy_ref_distance_m, 4.0, 1e-9, "D455 2% @4m");
    check(Profile("D456").ip65 && Profile("D457").ip65, "D456/D457 IP65");
    check(Profile("D457").interface.find("GMSL") != std::string::npos, "D457 GMSL");

    auto d555 = Profile("D555");
    close_to(d555.min_z_m, 0.26, 1e-9, "D555 MinZ 26cm");
    check(d555.ip65, "D555 IP65");

    auto d585 = Profile("D585Pro");
    close_to(d585.depth_hfov_deg, 120.0, 1e-9, "D585 Pro HFOV 120");
    close_to(d585.min_z_m, 0.10, 1e-9, "D585 Pro 10cm");
    check(d585.preliminary, "D585 Pro preliminary");

    check(rs::FindProfile("d435i").has_value(), "case-insensitive lookup");
    check(rs::FindProfile("RealSense D585 Pro").has_value(), "prefix/space lookup");
    check(!rs::FindProfile("D999").has_value(), "unknown model");
}

static void TestNoiseModel()
{
    std::printf("[noise: spec]\n");
    for (const char* model : {"D435", "D455", "D415", "D421", "D405"}) {
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig(model, c);
        c.depth_model.noise_model = "spec";
        c.depth.width = c.profile.accuracy_ref_width;   // spec resolution
        auto m = rs::MakeDepthModelParams(c);
        m.left_invalid_band = false;
        m.depth_unit_m = 0.0;
        const double zref = c.profile.accuracy_ref_distance_m;
        const double want = c.profile.accuracy_ratio * zref;
        close_to(rs::ExpectedDepthSigma(m, zref), want, 1e-9, std::string(model) + " expected sigma at spec distance");
        double mean = 0.0;
        const double got = MeasuredSigma(m, zref, 1000, 200, &mean);
        close_to(got, want, want * 0.05, std::string(model) + " measured sigma at spec distance");
        // 1/d is convex, so z' = fB/(d+n) has a small positive bias ~ z*(sigma_d/d)^2.
        close_to(mean, zref, want * 0.1, std::string(model) + " mean within 10% sigma");
        // quadratic growth: half the distance -> quarter the error
        const double got_half = MeasuredSigma(m, zref * 0.5, 1000, 200, nullptr);
        close_to(got_half / got, 0.25, 0.02, std::string(model) + " sigma ~ z^2");
    }

    // Lower stream resolution (smaller fx) -> larger depth error.
    rs::RealSenseConfig hd {};
    rs::MakeDefaultConfig("D435", hd);
    hd.depth.width = 1280;
    rs::RealSenseConfig vga = hd;
    vga.depth.width = 640;
    close_to(rs::ExpectedDepthSigma(rs::MakeDepthModelParams(vga), 2.0)
                 / rs::ExpectedDepthSigma(rs::MakeDepthModelParams(hd), 2.0),
             2.0, 1e-9, "640 px stream has 2x error of 1280 px");

    // noise_scale
    rs::RealSenseConfig half = hd;
    half.depth_model.noise_scale = 0.5;
    close_to(rs::ExpectedDepthSigma(rs::MakeDepthModelParams(half), 2.0),
             0.5 * rs::ExpectedDepthSigma(rs::MakeDepthModelParams(hd), 2.0), 1e-12, "noise_scale");

    std::printf("[noise: typical (Intel tuning paper)]\n");
    // RMS = z^2 * subpixel / (fx * B), subpixel 0.08 px (default model).
    {
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435", c);   // 848 px, default noise_model "typical"
        check(c.depth_model.noise_model == "typical", "typical is the default");
        auto m = rs::MakeDepthModelParams(c);
        close_to(m.disparity_sigma_px, 0.08, 1e-12, "subpixel 0.08 px");
        const double fx = 848.0 / (2.0 * std::tan(87.0 * rs::kDegToRad / 2.0));
        const double want = 4.0 * 0.08 / (fx * 0.05);
        close_to(rs::ExpectedDepthSigma(m, 2.0), want, 1e-12, "D435 848px RMS @2m = z^2*0.08/(fx*B)");
        check(want < 0.02 * 2.0, "typical RMS is below the 2% catalogue limit");
        m.left_invalid_band = false;
        m.depth_unit_m = 0.0;
        close_to(MeasuredSigma(m, 2.0, 1000, 200, nullptr), want, want * 0.05, "D435 measured typical RMS");
        auto pro = rs::FindProfile("D585Pro");
        check(pro && pro->subpixel_rms_px < 0.08, "D585 Pro assumed better subpixel");
    }

    std::printf("[MinZ: disparity search range]\n");
    {
        // Intel tuning paper: D415 @1280 px MinZ ~43.8 cm (with HFOV 65 / B 55 mm).
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D415", c);
        c.depth.width = 1280;
        close_to(rs::MakeDepthModelParams(c).min_z_m, 0.438, 0.005, "D415 1280px MinZ ~43.8cm");
        c.depth.width = 640;
        close_to(rs::MakeDepthModelParams(c).min_z_m, 0.219, 0.003, "MinZ scales linearly with width");
        // PDF p.43: D555 Min-Z 26 cm at VGA.
        rs::RealSenseConfig d555 {};
        rs::MakeDefaultConfig("D555", d555);
        d555.depth.width = 640;
        d555.depth.height = 480;
        close_to(rs::MakeDepthModelParams(d555).min_z_m, 0.26, 0.01, "D555 VGA MinZ ~26cm (PDF)");
        // disparity shift: nearer MinZ, finite max range
        c.depth.width = 1280;
        c.depth_model.disparity_shift_px = 50.0;
        auto ms = rs::MakeDepthModelParams(c);
        const double fb = rs::FocalLengthPx(1280, c.profile.DepthHfovRad()) * c.profile.baseline_m;
        close_to(ms.min_z_m, fb / 176.0, 1e-9, "shift: MinZ = fB/(126+shift)");
        close_to(ms.max_z_m, fb / 50.0, 1e-9, "shift: MaxZ = fB/shift");
        // fixed mode
        c.depth_model.disparity_shift_px = 0.0;
        c.depth_model.min_z_mode = "fixed";
        close_to(rs::MakeDepthModelParams(c).min_z_m, 0.18, 1e-12, "fixed mode uses catalogue MinZ");
    }
}

static void TestInvalidRules()
{
    std::printf("[invalid]\n");
    rs::RealSenseConfig c {};
    rs::MakeDefaultConfig("D435", c);
    auto m = rs::MakeDepthModelParams(c);   // 848 px
    m.disparity_sigma_px = 0.0;
    const int w = 848;

    // Range gate and NaN passthrough.
    std::vector<float> img = {0.05F, 0.11F, 0.5F, 10.5F, std::numeric_limits<float>::quiet_NaN(), 0.0F};
    m.left_invalid_band = false;
    std::mt19937 rng(1);
    rs::ApplyDepthModel(img, 6, 1, m, rng);
    check(std::isnan(img[0]) && std::isnan(img[1]), "below MinZ -> NaN");
    close_to(img[2], 0.5, 1e-6, "in range kept");
    check(std::isnan(img[3]), "beyond MaxZ -> NaN");
    check(std::isnan(img[4]) && std::isnan(img[5]), "NaN/0 stay invalid");

    // Left invalid band: width = fx*B/z.
    m.left_invalid_band = true;
    const double z = 0.5;
    std::vector<float> row(w, static_cast<float>(z));
    rs::ApplyDepthModel(row, w, 1, m, rng);
    const double band = rs::LeftInvalidBandPx(m, z);
    int invalid = 0;
    for (int x = 0; x < w; ++x) invalid += std::isnan(row[x]) ? 1 : 0;
    close_to(invalid, std::ceil(band), 1.0, "left band width = fx*B/z");
    check(std::isnan(row[0]) && std::isfinite(row[w - 1]), "band is on the left edge");
    close_to(band, (848.0 / (2.0 * std::tan(87.0 * rs::kDegToRad / 2.0))) * 0.05 / 0.5, 1e-9, "band formula");

    // Depth unit quantisation (1 mm).
    m.left_invalid_band = false;
    std::vector<float> q = {1.23456F};
    rs::ApplyDepthModel(q, 1, 1, m, rng);
    close_to(q[0], 1.235, 1e-6, "1 mm depth unit");

    // Material dropout.
    rs::DepthModelParams mm = m;
    mm.invalid_material_ids = {3};
    mm.material_invalid_probability = 1.0;
    std::vector<float> two = {1.0F, 1.0F};
    std::vector<int> mats = {3, 4};
    rs::ApplyDepthModel(two, 2, 1, mm, rng, &mats);
    check(std::isnan(two[0]) && std::isfinite(two[1]), "material-based invalid");

    // Random dropout ratio.
    rs::DepthModelParams dp = m;
    dp.invalid_probability = 0.1;
    std::vector<float> many(100000, 1.0F);
    rs::ApplyDepthModel(many, 1000, 100, dp, rng);
    int dropped = 0;
    for (float v : many) dropped += std::isnan(v) ? 1 : 0;
    close_to(dropped / 100000.0, 0.1, 0.01, "invalid_probability ratio");

    // Determinism with the same seed.
    auto m2 = rs::MakeDepthModelParams(c);
    std::vector<float> a(848 * 4, 2.0F);
    std::vector<float> b = a;
    std::mt19937 r1(42);
    std::mt19937 r2(42);
    rs::ApplyDepthModel(a, 848, 4, m2, r1);
    rs::ApplyDepthModel(b, 848, 4, m2, r2);
    bool same = true;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!(a[i] == b[i] || (std::isnan(a[i]) && std::isnan(b[i])))) same = false;
    }
    check(same, "same seed -> same frame");

    // Disabled model: ideal depth (range gate only).
    rs::RealSenseConfig off = c;
    off.depth_model.enabled = false;
    auto mo = rs::MakeDepthModelParams(off);
    std::vector<float> ideal(848, 1.23456F);
    rs::ApplyDepthModel(ideal, 848, 1, mo, rng);
    close_to(ideal[0], 1.23456, 1e-6, "disabled model keeps ideal depth (left edge)");
}

static void TestJson()
{
    std::printf("[json]\n");
    const std::string dir = std::string(HAKO_SENSOR_SOURCE_DIR) + "/config/realsense/";
    for (const char* f : {"d435i.json", "d455.json", "d405.json", "d421.json", "d555.json", "d415.json"}) {
        rs::RealSenseConfig c {};
        check(rs::LoadRealSenseConfigFromJson(dir + f, c), std::string("load sample ") + f);
    }

    rs::RealSenseConfig c {};
    rs::LoadRealSenseConfigFromJson(dir + "d435i.json", c);
    check(c.profile.model == "D435i", "model D435i");
    check(c.depth.width == 848 && c.depth.height == 480, "depth 848x480");
    check(c.RgbEnabled() && c.rgb.width == 1280, "rgb 1280");
    check(c.rgb_noise.type == "gaussian", "rgb noise");
    check(c.depth_model.seed == 7, "seed");
    check(c.DepthFrameId() == "camera_depth_optical_frame", "depth frame id");
    check(c.ColorFrameId() == "camera_color_optical_frame", "color frame id");

    rs::RealSenseConfig d421 {};
    rs::LoadRealSenseConfigFromJson(dir + "d421.json", d421);
    check(!d421.RgbEnabled(), "D421 rgb disabled");
    check(d421.depth.format == "DEPTH_U16_MM", "D421 U16 format");

    std::string err;
    rs::RealSenseConfig o {};
    check(rs::RealSenseConfigFromJson(
              nlohmann::json::parse(R"({"model":"d455","overrides":{"min_z_m":0.4,"max_z_m":6.0}})"), o, err),
          "overrides parse");
    close_to(o.profile.min_z_m, 0.4, 1e-12, "override min_z");
    close_to(o.profile.max_z_m, 6.0, 1e-12, "override max_z");

    check(!rs::RealSenseConfigFromJson(nlohmann::json::parse(R"({"model":"D999"})"), o, err), "unknown model rejected");
    check(!rs::RealSenseConfigFromJson(nlohmann::json::parse(R"({"depth":{}})"), o, err), "missing model rejected");
    check(!rs::RealSenseConfigFromJson(
              nlohmann::json::parse(R"({"model":"D435","depth":{"format":"RGB"}})"), o, err),
          "bad depth format rejected");
    check(!rs::RealSenseConfigFromJson(
              nlohmann::json::parse(R"({"model":"D435","overrides":{"min_z_m":5,"max_z_m":1}})"), o, err),
          "bad range rejected");
    check(!rs::RealSenseConfigFromJson(
              nlohmann::json::parse(R"({"model":"D435","depth":{"width":"a"}})"), o, err),
          "type error rejected");

    rs::RealSenseConfig p {};
    rs::MakeDefaultConfig("D585Pro", p);
    std::vector<std::string> warnings;
    check(rs::ValidateConfig(p, &warnings).empty(), "D585 Pro valid");
    bool prelim = false;
    for (const auto& w : warnings) prelim = prelim || w.find("preliminary") != std::string::npos;
    check(prelim, "D585 Pro preliminary warning");
    check(p.depth.width == 640 && p.depth.height == 480, "D585 Pro 4:3 default");

    for (const auto& name : rs::BuiltinModelNames()) {
        rs::RealSenseConfig d {};
        check(rs::MakeDefaultConfig(name, d) && rs::ValidateConfig(d).empty(), "default config valid: " + name);
    }
}

int main()
{
    TestProfiles();
    TestNoiseModel();
    TestInvalidRules();
    TestJson();
    std::printf("realsense_depth_model_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
