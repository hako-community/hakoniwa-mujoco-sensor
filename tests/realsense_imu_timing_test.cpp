// Unit test for RS-6: IMU timestamps / drops / lever arm / misalignment and
// camera frame timing. Backend-free.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_imu_timing.hpp"

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
    std::printf("  [%s] %s (got %.6f, want %.6f +- %.6f)\n", ok ? "ok" : "FAIL", msg.c_str(), a, b, tol);
}

static void TestLeverArm()
{
    std::printf("[lever arm]\n");
    const rs::Vec3d r {0.1, 0.0, 0.0};
    const rs::Vec3d w {0.0, 5.0, 0.0};
    const auto a = rs::LeverArmAccel({0, 0, 0}, w, {0, 0, 0}, r);
    close_to(a.x, -2.5, 1e-12, "centripetal -w^2 r toward the axis");
    close_to(a.y, 0.0, 1e-12, "no component along the axis");
    const auto t = rs::LeverArmAccel({0, -9.8, 0}, {0, 0, 0}, {0, 2.0, 0}, r);
    close_to(t.z, -0.2, 1e-12, "tangential alpha x r");
    close_to(t.y, -9.8, 1e-12, "camera acceleration kept");
}

static void TestMisalignment()
{
    std::printf("[misalignment]\n");
    rs::ImuTimingModel none;
    none.Configure(rs::ImuTimingParams {});
    const auto v = none.Misalign({1.0, 2.0, 3.0});
    check(v.x == 1.0 && v.y == 2.0 && v.z == 3.0, "0 deg: identity");
    rs::ImuTimingParams p {};
    p.misalignment_deg = 0.5;
    double sum2 = 0.0;
    const int runs = 400;
    for (int s = 0; s < runs; ++s) {
        p.seed = static_cast<std::uint32_t>(s);
        rs::ImuTimingModel m;
        m.Configure(p);
        const double ang = m.MisalignmentAngleRad();
        sum2 += ang * ang;
        const auto g = m.Misalign({0.0, -9.81, 0.0});
        if (s == 0) {
            close_to(std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z), 9.81, 1e-9, "rotation keeps the norm");
        }
    }
    // |angle|^2 = sum of three N(0, s^2) -> RMS = sqrt(3) s
    close_to(std::sqrt(sum2 / runs), std::sqrt(3.0) * 0.5 * rs::kDegToRad, 0.15 * std::sqrt(3.0) * 0.5 * rs::kDegToRad,
             "RMS misalignment angle = sqrt(3) * sigma");
    p.seed = 3;
    rs::ImuTimingModel a, b;
    a.Configure(p);
    b.Configure(p);
    check(a.MisalignmentAngleRad() == b.MisalignmentAngleRad(), "same seed -> same misalignment");
}

static void TestStampsAndDrops()
{
    std::printf("[timestamps / drops]\n");
    rs::ImuTimingParams p {};
    p.time_offset_s = 0.003;
    rs::ImuTimingModel m;
    m.Configure(p);
    close_to(m.GyroStamp(1.0), 1.003, 1e-12, "fixed offset");
    p.time_offset_s = 0.0;
    p.jitter_s = 1e-3;
    m.Configure(p);
    double s = 0.0, s2 = 0.0, prev = -1.0;
    bool monotonic = true;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const double t = i * 0.0025;   // 400 Hz
        const double st = m.GyroStamp(t);
        monotonic = monotonic && st > prev;
        prev = st;
        s += st - t;
        s2 += (st - t) * (st - t);
    }
    check(monotonic, "jittered stamps stay monotonic");
    close_to(std::sqrt(s2 / n - (s / n) * (s / n)), 1e-3, 5e-5, "jitter sigma");
    p.jitter_s = 0.0;
    p.drop_probability = 0.02;
    m.Configure(p);
    int drops = 0;
    for (int i = 0; i < 50000; ++i) drops += m.Drop() ? 1 : 0;
    close_to(drops / 50000.0, 0.02, 3.0 * std::sqrt(0.02 * 0.98 / 50000.0), "drop rate");

    rs::FrameTimingParams fp {};
    rs::FrameTimingModel f;
    f.Configure(fp);
    check(!f.Enabled() && !f.Drop() && f.Stamp(2.0) == 2.0, "frame timing off: unchanged");
    fp.drop_probability = 1.0;
    f.Configure(fp);
    check(f.Drop(), "frame drop probability 1");
}

static void TestConfig()
{
    std::printf("[config]\n");
    rs::RealSenseConfig c {};
    std::string err;
    check(rs::RealSenseConfigFromJson(nlohmann::json {{"model", "D435i"}}, c, err), "default config");
    check(!c.imu_timing.enabled && !c.frame_timing.enabled, "timing default off");
    const auto j = nlohmann::json::parse(R"({ "model": "D435i",
        "imu_timing": { "enabled": true, "time_offset_s": 0.002, "jitter_s": 0.0001, "drop_probability": 0.001,
                        "position_m": [-0.0055, 0.0051, 0.0117], "misalignment_deg": 0.5, "seed": 8 },
        "frame_timing": { "enabled": true, "jitter_s": 0.001, "drop_probability": 0.01 } })");
    check(rs::RealSenseConfigFromJson(j, c, err), "timing JSON parses: " + err);
    const auto& ip = c.imu_timing.params;
    check(c.imu_timing.enabled && ip.time_offset_s == 0.002 && ip.jitter_s == 0.0001 && ip.drop_probability == 0.001
              && ip.position_m[2] == 0.0117 && ip.misalignment_deg == 0.5 && ip.seed == 8,
          "imu_timing read");
    check(c.frame_timing.enabled && c.frame_timing.params.jitter_s == 0.001
              && c.frame_timing.params.drop_probability == 0.01, "frame_timing read");
    auto bad = j;
    bad["imu_timing"]["position_m"] = {0.0, 1.0};
    check(!rs::RealSenseConfigFromJson(bad, c, err), "position_m needs 3 values");
    bad = j;
    bad["frame_timing"]["drop_probability"] = 2.0;
    check(!rs::RealSenseConfigFromJson(bad, c, err), "drop_probability > 1 rejected");
}

int main()
{
    TestLeverArm();
    TestMisalignment();
    TestStampsAndDrops();
    TestConfig();
    std::printf("realsense_imu_timing_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
