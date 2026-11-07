// Unit test for RS-4: thermal drift (warm-up, ambient schedule, on-chip
// calibration, depth disparity offset, IMU zero-offset drift). Backend-free.

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_depth_model.hpp"
#include "sensors/camera/realsense/realsense_thermal_model.hpp"

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

static void TestWarmup()
{
    std::printf("[warm-up]\n");
    rs::ThermalModel m;
    m.Configure(rs::ThermalParams {});
    m.PowerOn(100.0);
    close_to(m.TemperatureAt(100.0), 25.0, 1e-12, "power-on: ambient");
    close_to(m.TemperatureAt(700.0), 25.0 + 15.0 * (1.0 - std::exp(-1.0)), 1e-9, "after tau: 63 % of the rise");
    close_to(m.TemperatureAt(100.0 + 1e5), 40.0, 1e-6, "steady: ambient + 15 K");
    close_to(m.CalibrationTempC(), 40.0, 1e-12, "default T_cal = warm at 25 C");

    auto p = rs::ThermalParams {};
    p.laser_power_mw = 0.0;
    rs::ThermalModel off;
    off.Configure(p);
    off.PowerOn(0.0);
    close_to(off.TemperatureAt(1e5), 25.0 + 15.0 * 0.6, 1e-6, "projector off: smaller self-heating (60 %)");

    p = rs::ThermalParams {};
    p.start_warm = true;
    rs::ThermalModel warm;
    warm.Configure(p);
    close_to(warm.TemperatureAt(0.0), 40.0, 1e-12, "start_warm: already at the steady temperature");

    rs::ThermalModel lazy;
    lazy.Configure(rs::ThermalParams {});
    close_to(lazy.TemperatureAt(50.0), 25.0, 1e-12, "first query sets the power-on time");
    check(lazy.PoweredOn(), "powered on by the first query");
}

static void TestAmbient()
{
    std::printf("[ambient schedule]\n");
    auto p = rs::ThermalParams {};
    p.ambient_schedule = {{3600.0, 30.0}, {0.0, 15.0}};   // unsorted on purpose
    p.start_warm = true;
    rs::ThermalModel m;
    m.Configure(p);
    close_to(m.AmbientAt(-10.0), 15.0, 1e-12, "before the schedule: first value");
    close_to(m.AmbientAt(1800.0), 22.5, 1e-12, "linear interpolation");
    close_to(m.AmbientAt(9999.0), 30.0, 1e-12, "after the schedule: last value");
    // morning 15 C -> noon 30 C, warm camera, factory T_cal 40 C:
    close_to(m.DisparityOffsetPx(0.0), 0.005 * (30.0 - 40.0), 1e-12, "15 C morning: offset -0.05 px");
    close_to(m.DisparityOffsetPx(3600.0), 0.005 * (45.0 - 40.0), 1e-12, "30 C noon: offset +0.025 px");
}

static void TestCalibration()
{
    std::printf("[on-chip calibration]\n");
    rs::ThermalModel m;
    m.Configure(rs::ThermalParams {});
    m.PowerOn(0.0);
    close_to(m.DisparityOffsetPx(0.0), 0.005 * (25.0 - 40.0), 1e-12, "cold start: offset -0.075 px");
    m.RunOnChipCalibration(0.0);
    close_to(m.DisparityOffsetPx(0.0), 0.0, 1e-12, "calibrated now: offset 0");
    close_to(m.DisparityOffsetPx(1e5), 0.005 * 15.0, 1e-6, "after warming up: offset grows again (+0.075 px)");
    m.RunOnChipCalibration(1e5);
    close_to(m.DisparityOffsetPx(1e5), 0.0, 1e-12, "re-calibrated warm: 0");

    auto p = rs::ThermalParams {};
    p.calibration_temp_c = 30.0;
    rs::ThermalModel c;
    c.Configure(p);
    close_to(c.CalibrationTempC(), 30.0, 1e-12, "explicit calibration temperature");
}

static void TestDepthError()
{
    std::printf("[depth error from the disparity offset]\n");
    rs::DepthModelParams m {};
    m.fx_px = rs::FocalLengthPx(848, 87.0 * 3.14159265358979323846 / 180.0);
    m.baseline_m = 0.05;
    m.min_z_m = 0.1;
    m.max_z_m = 10.0;
    m.left_invalid_band = false;
    m.depth_unit_m = 0.0;
    const double fb = m.fx_px * m.baseline_m;
    for (double z : {0.6, 2.0}) {
        for (double off : {-0.075, 0.075}) {
            m.disparity_offset_px = off;
            std::vector<float> d(16, static_cast<float>(z));
            std::mt19937 rng(1);
            rs::ApplyDepthModel(d, 4, 4, m, rng);
            const double want = fb / (fb / z + off);
            char msg[96];
            std::snprintf(msg, sizeof(msg), "z %.1f m, offset %+.3f px -> %.4f m (error %+.1f mm)", z, off, want,
                          (want - z) * 1e3);
            close_to(d[5], want, 1e-5, msg);
        }
    }
    // error ~ -z^2 * delta / (fx B)
    m.disparity_offset_px = -0.075;
    std::vector<float> d(1, 2.0F);
    std::mt19937 rng(1);
    rs::ApplyDepthModel(d, 1, 1, m, rng);
    close_to(d[0] - 2.0, 4.0 * 0.075 / fb, 0.001, "2 m cold start: ~ +z^2 * 0.075 / (fx B)");

    // offset 0 + no noise: unchanged path (bit-identical to before)
    m.disparity_offset_px = 0.0;
    std::vector<float> e(1, 2.0F);
    rs::ApplyDepthModel(e, 1, 1, m, rng);
    check(e[0] == 2.0F, "offset 0 leaves the ideal depth untouched");
}

static void TestImuDrift()
{
    std::printf("[IMU zero-offset drift]\n");
    rs::ThermalModel m;
    m.Configure(rs::ThermalParams {});
    m.PowerOn(0.0);
    const auto a = m.AccelTco();
    const auto g = m.GyroTco();
    const double amax = 1e-3 * rs::kStandardGravity;
    const double gmax = 0.015 * rs::kDegToRad;
    check(std::fabs(a.x) <= amax && std::fabs(a.y) <= amax && std::fabs(a.z) <= amax, "accel TCO within +-1 mg/K");
    check(std::fabs(g.x) <= gmax && std::fabs(g.y) <= gmax && std::fabs(g.z) <= gmax, "gyro TCO within +-0.015 dps/K");
    const auto da = m.AccelDrift(0.0);
    close_to(da.x, a.x * (25.0 - 40.0), 1e-12, "accel drift = TCO * (T - T_cal)");
    const auto dg = m.GyroDrift(1e5);
    close_to(dg.z, 0.0, 1e-9, "warm (T = T_cal): no gyro drift");
    rs::ThermalModel same;
    same.Configure(rs::ThermalParams {});
    check(same.GyroTco().y == g.y, "same seed -> same TCO");
    auto p = rs::ThermalParams {};
    p.seed = 5;
    rs::ThermalModel other;
    other.Configure(p);
    check(other.GyroTco().y != g.y, "other seed -> other TCO");
}

static void TestConfig()
{
    std::printf("[config]\n");
    rs::RealSenseConfig c {};
    std::string err;
    check(rs::RealSenseConfigFromJson(nlohmann::json {{"model", "D435i"}}, c, err), "default config");
    check(!c.thermal.enabled, "thermal default off");
    const auto j = nlohmann::json::parse(R"({ "model": "D435i",
        "texture_model": { "laser_power_mw": 300 },
        "thermal": { "enabled": true, "ambient_schedule": [[0, 15], [14400, 30]], "time_constant_s": 300,
                     "calibration_temp_c": null, "disparity_coeff_px_per_k": 0.01, "seed": 2 } })");
    check(rs::RealSenseConfigFromJson(j, c, err), "thermal JSON parses: " + err);
    const auto p = rs::MakeThermalParams(c);
    check(c.thermal.enabled && p.ambient_schedule.size() == 2 && p.time_constant_s == 300.0
              && p.disparity_coeff_px_per_k == 0.01 && p.laser_power_mw == 300.0 && !std::isfinite(p.calibration_temp_c),
          "thermal parameters read (laser power from texture_model)");
    auto bad = j;
    bad["thermal"]["ambient_schedule"] = {{0, 15, 3}};
    check(!rs::RealSenseConfigFromJson(bad, c, err), "malformed schedule rejected");
    bad = j;
    bad["thermal"]["laser_share"] = 1.5;
    check(!rs::RealSenseConfigFromJson(bad, c, err), "laser_share > 1 rejected");
}

int main()
{
    TestWarmup();
    TestAmbient();
    TestCalibration();
    TestDepthError();
    TestImuDrift();
    TestConfig();
    std::printf("realsense_thermal_model_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
