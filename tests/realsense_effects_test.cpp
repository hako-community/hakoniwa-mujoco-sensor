// Unit test for RealSense post-processing filters, rolling shutter and the IMU
// noise model. Backend-free: no MuJoCo runtime, GL or PDU.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_imu_model.hpp"
#include "sensors/camera/realsense/realsense_post_processing.hpp"
#include "sensors/camera/realsense/realsense_rolling_shutter.hpp"

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
    if (!(std::fabs(a - b) <= tol)) {
        ++g_failures;
        std::printf("  [FAIL] %s (got %.6f, want %.6f +- %.6f)\n", msg.c_str(), a, b, tol);
    }
}

static double StdDev(const std::vector<float>& v)
{
    double s = 0.0, s2 = 0.0;
    std::size_t n = 0;
    for (float x : v) {
        if (!std::isfinite(x)) continue;
        s += x;
        s2 += static_cast<double>(x) * x;
        ++n;
    }
    if (n == 0) return 0.0;
    const double m = s / n;
    return std::sqrt(std::max(0.0, s2 / n - m * m));
}

static void TestDecimationThreshold()
{
    std::printf("[decimation / threshold]\n");
    // 4x2 -> 2x1 with magnitude 2 (median of valid pixels in each 2x2 block)
    std::vector<float> img = {1.0F, 2.0F, 5.0F, kNaN,
                              3.0F, 4.0F, kNaN, kNaN};
    int w = 4, h = 2;
    rs::Decimate(img, w, h, 2);
    check(w == 2 && h == 1 && img.size() == 2, "decimation output size");
    close_to(img[0], 2.0, 1e-6, "2x2 median (lower middle of 1,2,3,4)");
    close_to(img[1], 5.0, 1e-6, "median ignores invalid");
    std::vector<float> all_nan(4, kNaN);
    int w2 = 2, h2 = 2;
    rs::Decimate(all_nan, w2, h2, 2);
    check(std::isnan(all_nan[0]), "all-invalid block stays invalid");
    // magnitude 4 uses the mean
    std::vector<float> m4(16, 1.0F);
    m4[0] = 5.0F;
    int w4 = 4, h4 = 4;
    rs::Decimate(m4, w4, h4, 4);
    close_to(m4[0], 1.25, 1e-6, "magnitude 4 uses mean");

    std::vector<float> t = {0.05F, 1.0F, 5.0F};
    rs::Threshold(t, 0.1, 4.0);
    check(std::isnan(t[0]) && t[1] == 1.0F && std::isnan(t[2]), "threshold");
}

static void TestSpatialTemporal()
{
    std::printf("[spatial / temporal]\n");
    const int w = 200, h = 100;
    const double fx = 400.0, b = 0.05;
    // Noisy flat wall at 2 m, with a step to 1 m on the right half.
    std::mt19937 rng(5);
    std::normal_distribution<double> n(0.0, 0.01);
    std::vector<float> img(static_cast<std::size_t>(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            img[static_cast<std::size_t>(y) * w + x] = static_cast<float>((x < w / 2 ? 2.0 : 1.0) + n(rng));
    auto left_half = [&](const std::vector<float>& v) {
        std::vector<float> out;
        for (int y = 0; y < h; ++y)
            for (int x = 10; x < w / 2 - 10; ++x) out.push_back(v[static_cast<std::size_t>(y) * w + x]);
        return out;
    };
    const double before = StdDev(left_half(img));

    rs::PostProcessingConfig c {};
    c.enabled = true;
    c.spatial.enabled = true;
    rs::DepthPostProcessor pp;
    int ow = w, oh = h;
    pp.Apply(img, ow, oh, fx, b, c);
    const double after = StdDev(left_half(img));
    check(after < before * 0.6, "spatial filter reduces noise on a flat surface");
    // edge preserved: the step 2 m -> 1 m (disparity 10 -> 20 px = 320 units >> delta 20)
    close_to(img[static_cast<std::size_t>(50) * w + w / 2 - 3], 2.0, 0.05, "edge preserved (far side)");
    close_to(img[static_cast<std::size_t>(50) * w + w / 2 + 3], 1.0, 0.05, "edge preserved (near side)");

    // spatial hole filling (holes_fill 1 = 2 px)
    rs::SpatialConfig sc {};
    sc.holes_fill = 1;
    sc.iterations = 1;   // each iteration fills again
    std::vector<float> row = {1.0F, kNaN, kNaN, kNaN, kNaN, 1.0F};
    rs::SpatialFilter(row, 6, 1, sc);
    check(std::isfinite(row[1]) && std::isfinite(row[2]) && std::isfinite(row[3]) && std::isfinite(row[4]),
          "spatial hole fill from both directions (2 px each)");
    std::vector<float> row2 = {1.0F, kNaN, kNaN, kNaN, kNaN, kNaN, kNaN, 1.0F};
    rs::SpatialFilter(row2, 8, 1, sc);
    check(std::isnan(row2[3]) && std::isnan(row2[4]), "spatial hole fill limited to 2 px");

    // temporal persistency 3: valid in 2 of last 4 -> fill
    rs::TemporalFilter tf;
    rs::TemporalConfig tc {};
    std::vector<float> f = {1.0F};
    tf.Apply(f, tc);
    f = {1.0F};
    tf.Apply(f, tc);
    f = {kNaN};
    tf.Apply(f, tc);
    close_to(f[0], 1.0, 1e-6, "temporal persistency fills a hole after 2 valid frames");
    rs::TemporalFilter tf2;
    std::vector<float> g = {1.0F};
    tf2.Apply(g, tc);
    g = {kNaN};
    tf2.Apply(g, tc);
    check(std::isnan(g[0]), "1 valid of last 4 does not persist (index 3)");
    check(rs::TemporalFilter::Persist(0b00000010, 5), "index 5: valid in 1 of last 2");
    check(!rs::TemporalFilter::Persist(0b11111110, 1), "index 1 never fills a hole");
    check(rs::TemporalFilter::Persist(0, 8), "index 8 always persists");
    // temporal smoothing: alpha 0.4
    rs::TemporalFilter tf3;
    std::vector<float> s1 = {100.0F};
    tf3.Apply(s1, tc);
    std::vector<float> s2 = {110.0F};
    tf3.Apply(s2, tc);
    close_to(s2[0], 0.4 * 110.0 + 0.6 * 100.0, 1e-4, "temporal alpha 0.4");
    std::vector<float> s3 = {200.0F};
    tf3.Apply(s3, tc);
    close_to(s3[0], 200.0, 1e-4, "temporal step > delta is not smoothed");
}

static void TestHoleFillingAndChain()
{
    std::printf("[hole filling / chain]\n");
    // 3x3 with centre hole; neighbours 1,2,3,4
    std::vector<float> base = {0.5F, 1.0F, 0.5F,
                               2.0F, kNaN, 3.0F,
                               0.5F, 4.0F, 0.5F};
    auto far = base;
    rs::HoleFilling(far, 3, 3, 1);
    close_to(far[4], 4.0, 1e-6, "farthest_from_around");
    auto nearest = base;
    rs::HoleFilling(nearest, 3, 3, 2);
    close_to(nearest[4], 1.0, 1e-6, "nearest_from_around");
    auto left = base;
    rs::HoleFilling(left, 3, 3, 0);
    close_to(left[4], 2.0, 1e-6, "fill_from_left");

    // chain with decimation changes the size
    rs::PostProcessingConfig c {};
    c.enabled = true;
    c.decimation.enabled = true;
    std::vector<float> img(848 * 480, 1.5F);
    int w = 848, h = 480;
    rs::DepthPostProcessor pp;
    pp.Apply(img, w, h, 400.0, 0.05, c);
    check(w == 424 && h == 240 && img.size() == 424u * 240u, "chain decimation 848x480 -> 424x240");
    int sw = 848, sh = 480;
    rs::PostProcessedSize(c, sw, sh);
    check(sw == 424 && sh == 240, "PostProcessedSize");
    // disabled chain is a no-op
    rs::PostProcessingConfig off {};
    std::vector<float> same = {1.0F, kNaN};
    int w1 = 2, h1 = 1;
    pp.Apply(same, w1, h1, 400.0, 0.05, off);
    check(same[0] == 1.0F && std::isnan(same[1]), "disabled chain is a no-op");
}

static void TestRollingShutter()
{
    std::printf("[rolling shutter]\n");
    const int w = 200, h = 100;
    const rs::PinholeIntrinsics k {100.0, 100.0, 100.0, 50.0};
    // Vertical edge at x = 100: left dark, right bright.
    std::vector<std::uint8_t> img(static_cast<std::size_t>(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) img[static_cast<std::size_t>(y) * w + x] = x < 100 ? 0 : 255;
    rs::CameraMotion yaw {};
    yaw.wy = 10.0;  // rad/s about the optical y (down) axis
    const double readout = 0.02;
    rs::ApplyRollingShutterImage(img, w, h, 1, k, yaw, readout);
    auto edge_col = [&](int row) {
        for (int x = 0; x < w; ++x)
            if (img[static_cast<std::size_t>(row) * w + x] > 127) return x;
        return -1;
    };
    // row r samples the reference at x + fx*w*dt  ->  edge moves to 100 - fx*w*dt
    const double dt_top = (0 - 0.5 * (h - 1)) * readout / h;
    const double dt_bot = ((h - 1) - 0.5 * (h - 1)) * readout / h;
    close_to(edge_col(0), 100.0 - k.fx * 10.0 * dt_top, 1.5, "top row edge shift");
    close_to(edge_col(h - 1), 100.0 - k.fx * 10.0 * dt_bot, 1.5, "bottom row edge shift");
    close_to(edge_col(h / 2), 100.0, 1.5, "middle row (reference time) unchanged");
    check(edge_col(0) > edge_col(h - 1), "edge is slanted (skew)");

    // No motion or zero readout: unchanged.
    std::vector<std::uint8_t> still(static_cast<std::size_t>(w) * h, 7);
    rs::ApplyRollingShutterImage(still, w, h, 1, k, rs::CameraMotion {}, readout);
    check(still[0] == 7, "no motion -> unchanged");

    // Depth: flat wall at 2 m, camera moving forward 1 m/s.
    std::vector<float> depth(static_cast<std::size_t>(w) * h, 2.0F);
    rs::CameraMotion fwd {};
    fwd.vz = 1.0;
    rs::ApplyRollingShutterDepth(depth, w, h, k, fwd, readout);
    close_to(depth[static_cast<std::size_t>(0) * w + 100], 2.0 - 1.0 * dt_top, 1e-4, "top row sees the wall from behind");
    close_to(depth[static_cast<std::size_t>(h - 1) * w + 100], 2.0 - 1.0 * dt_bot, 1e-4, "bottom row closer");

    // Depth with rotation about the optical axis keeps a flat fronto-parallel wall at 2 m.
    std::vector<float> d2(static_cast<std::size_t>(w) * h, 2.0F);
    rs::CameraMotion roll {};
    roll.wz = 3.0;
    rs::ApplyRollingShutterDepth(d2, w, h, k, roll, readout);
    close_to(d2[static_cast<std::size_t>(10) * w + 100], 2.0, 1e-4, "roll keeps wall depth");
}

static void TestImuModel()
{
    std::printf("[imu]\n");
    rs::ImuSpec spec {};   // BMI055 defaults
    rs::ImuModelConfig cfg {};
    cfg.bias_mode = "none";
    cfg.quantize = false;
    cfg.seed = 9;
    rs::ImuNoiseModel m;
    m.Configure(spec, cfg);
    // sigma = density * sqrt(rate)
    close_to(m.AccelSigma(), 150e-6 * rs::kStandardGravity * std::sqrt(250.0), 1e-12, "accel sigma");
    close_to(m.GyroSigma(), 0.014 * rs::kDegToRad * std::sqrt(400.0), 1e-12, "gyro sigma");

    const int n = 20000;
    double s = 0.0, s2 = 0.0;
    for (int i = 0; i < n; ++i) {
        const auto v = m.MeasureGyro({0.0, 0.0, 0.0});
        s += v.x;
        s2 += v.x * v.x;
    }
    const double mean = s / n;
    const double sd = std::sqrt(s2 / n - mean * mean);
    close_to(sd, m.GyroSigma(), m.GyroSigma() * 0.05, "measured gyro sigma");
    close_to(mean, 0.0, m.GyroSigma() * 0.05, "no bias with bias_mode none");

    // quantisation: 12-bit accel over +-4 g
    rs::ImuModelConfig q = cfg;
    q.quantize = true;
    q.noise_scale = 0.0;
    rs::ImuNoiseModel mq;
    mq.Configure(spec, q);
    close_to(mq.AccelLsb(), 4.0 * rs::kStandardGravity / 2048.0, 1e-12, "12-bit accel LSB");
    const auto a = mq.MeasureAccel({0.0, -rs::kStandardGravity, 0.0});
    close_to(a.y / mq.AccelLsb(), std::round(a.y / mq.AccelLsb()), 1e-9, "accel quantised to LSB");
    // saturation at the range
    const auto sat = mq.MeasureGyro({100.0, 0.0, 0.0});
    close_to(sat.x, 2000.0 * rs::kDegToRad, mq.GyroLsb(), "gyro saturates at +-2000 dps");

    // bias: uncalibrated within +-offset, calibrated 10% of it
    rs::ImuModelConfig ub = cfg;
    ub.bias_mode = "uncalibrated";
    rs::ImuNoiseModel mu;
    mu.Configure(spec, ub);
    const double gmax = 1.0 * rs::kDegToRad;
    check(std::fabs(mu.GyroBias().x) <= gmax && std::fabs(mu.GyroBias().y) <= gmax, "uncalibrated gyro bias within +-1 dps");
    rs::ImuModelConfig cb = cfg;
    cb.bias_mode = "calibrated";
    rs::ImuNoiseModel mc;
    mc.Configure(spec, cb);
    check(std::fabs(mc.GyroBias().x) <= 0.1 * gmax + 1e-15, "calibrated gyro bias within 10%");

    // deterministic with seed
    rs::ImuNoiseModel a1, a2;
    a1.Configure(spec, ub);
    a2.Configure(spec, ub);
    check(a1.MeasureAccel({0, 0, 0}).x == a2.MeasureAccel({0, 0, 0}).x, "same seed -> same IMU stream");
}

static void TestConfig()
{
    std::printf("[config]\n");
    std::string err;
    rs::RealSenseConfig c {};
    const auto j = nlohmann::json::parse(R"({
        "model": "D435i",
        "rolling_shutter": {"enabled": true, "rgb_readout_s": 0.03},
        "post_processing": {"enabled": true, "decimation": {"enabled": true, "magnitude": 3},
                            "spatial": {"enabled": true, "alpha": 0.6, "holes_fill": 2},
                            "temporal": {"enabled": true, "persistency": 5},
                            "hole_filling": {"enabled": true, "mode": 2}},
        "imu": {"accel_rate_hz": 62.5, "gyro_rate_hz": 200, "bias_mode": "uncalibrated", "seed": 4},
        "overrides": {"imu": {"chip": "BMI085", "accel_bits": 16}}
    })");
    check(rs::RealSenseConfigFromJson(j, c, err), "parse full config: " + err);
    close_to(c.RgbReadoutS(), 0.03, 1e-12, "rgb readout override");
    close_to(c.DepthReadoutS(), 0.0, 1e-12, "D435 depth is global shutter");
    check(c.post_processing.decimation.magnitude == 3 && c.post_processing.spatial.holes_fill == 2
              && c.post_processing.temporal.persistency == 5 && c.post_processing.hole_filling.mode == 2,
          "post-processing parsed");
    close_to(c.post_processing.spatial.alpha, 0.6, 1e-12, "spatial alpha");
    check(c.ImuEnabled() && c.imu.bias_mode == "uncalibrated" && c.imu.seed == 4, "imu parsed");
    close_to(c.imu.accel_rate_hz, 62.5, 1e-12, "accel rate");
    check(c.profile.imu.chip == "BMI085" && c.profile.imu.accel_bits == 16, "imu spec override");

    rs::RealSenseConfig d415 {};
    rs::MakeDefaultConfig("D415", d415);
    check(d415.DepthReadoutS() > 0.0 && d415.RgbReadoutS() > 0.0, "D415 rolling depth + RGB");
    check(!d415.ImuEnabled(), "D415 has no IMU");
    rs::RealSenseConfig d455 {};
    rs::MakeDefaultConfig("D455", d455);
    check(d455.DepthReadoutS() == 0.0 && d455.RgbReadoutS() == 0.0, "D455 all global shutter");
    check(d455.ImuEnabled(), "D455 IMU enabled by default");
    close_to(d455.imu.gyro_rate_hz, 400.0, 1e-12, "default gyro 400 Hz");

    check(!rs::RealSenseConfigFromJson(nlohmann::json::parse(
              R"({"model":"D435","post_processing":{"temporal":{"persistency":9}}})"), c, err),
          "bad persistency rejected");
    check(!rs::RealSenseConfigFromJson(nlohmann::json::parse(
              R"({"model":"D435i","imu":{"bias_mode":"x"}})"), c, err),
          "bad bias_mode rejected");
    check(!rs::RealSenseConfigFromJson(nlohmann::json::parse(
              R"({"model":"D435","depth_model":{"noise_model":"x"}})"), c, err),
          "bad noise_model rejected");
}

int main()
{
    TestDecimationThreshold();
    TestSpatialTemporal();
    TestHoleFillingAndChain();
    TestRollingShutter();
    TestImuModel();
    TestConfig();
    std::printf("realsense_effects_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
