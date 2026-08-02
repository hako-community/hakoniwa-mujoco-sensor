// Standalone unit test for the backend-agnostic radar model.
// Verifies pure geometry math and the full Scan pipeline against a mock ray
// caster -- no MuJoCo runtime, no PDU. This is the core value of Strategy C:
// the sensor logic is testable in isolation from any engine.

#include <cmath>
#include <cstdio>
#include <memory>

#include "sensors/radar/radar_math.hpp"
#include "sensors/radar/radar_sensor.hpp"

using hako::robots::types::Vector3;
namespace rmath = hako::robots::sensor::radar::math;
namespace backend = hako::robots::sensor::backend;
namespace radar = hako::robots::sensor::radar;

static int g_failures = 0;
static int g_checks = 0;

static void check(bool cond, const char* msg)
{
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::printf("  [FAIL] %s\n", msg);
    }
}

static void close_to(double a, double b, double tol, const char* msg)
{
    ++g_checks;
    if (std::fabs(a - b) > tol) {
        ++g_failures;
        std::printf("  [FAIL] %s (got %.6f, want %.6f)\n", msg, a, b);
    }
}

// Mock backend: every ray hits a sphere of radius D centred on the sensor,
// with a fixed target velocity. Lets us assert depth/azimuth/elevation/velocity
// deterministically for any random ray direction.
class SphereMockRayCaster : public backend::IRayCaster
{
public:
    SphereMockRayCaster(double d, Vector3 v) : d_(d), v_(v) {}
    backend::RayHit Cast(const Vector3& origin, const Vector3& dir, double /*max*/) override
    {
        backend::RayHit h {};
        h.hit = true;
        h.distance = d_;
        h.point = Vector3(origin.x + dir.x * d_, origin.y + dir.y * d_, origin.z + dir.z * d_);
        h.target_velocity = v_;
        h.target_id = 1;
        return h;
    }
private:
    double d_;
    Vector3 v_;
};

static radar::RadarConfig make_config(double range, double hfov, double vfov, int pps)
{
    radar::RadarConfig c {};
    c.range = range;
    c.horizontal_fov_deg = hfov;
    c.vertical_fov_deg = vfov;
    c.points_per_second = pps;
    c.noise_seed = 42U;
    c.output.update_rate_hz = 10.0;  // -> pps/10 points per scan
    return c;
}

static backend::SensorState forward_state()
{
    backend::SensorState s {};
    s.origin = Vector3(0, 0, 0);
    s.forward = Vector3(1, 0, 0);
    s.left = Vector3(0, 1, 0);
    s.up = Vector3(0, 0, 1);
    s.linear_velocity = Vector3(0, 0, 0);
    return s;
}

int main()
{
    std::printf("== radar pure math ==\n");
    {
        double az, el, dp;
        rmath::ToPolar(Vector3(1, 0, 0), az, el, dp);
        close_to(az, 0.0, 1e-9, "ToPolar forward azimuth");
        close_to(el, 0.0, 1e-9, "ToPolar forward elevation");
        close_to(dp, 1.0, 1e-9, "ToPolar forward depth");

        rmath::ToPolar(Vector3(0, 1, 0), az, el, dp);
        close_to(az, rmath::kPi / 2.0, 1e-9, "ToPolar left azimuth = +pi/2");

        rmath::ToPolar(Vector3(0, 0, 1), az, el, dp);
        close_to(el, rmath::kPi / 2.0, 1e-9, "ToPolar up elevation = +pi/2");

        // RelativeVelocity: target receding along +x -> positive
        close_to(rmath::RelativeVelocity(Vector3(2, 0, 0), Vector3(0, 0, 0), Vector3(1, 0, 0)),
                 2.0, 1e-9, "RelVel receding positive");
        // approaching -> negative
        close_to(rmath::RelativeVelocity(Vector3(-3, 0, 0), Vector3(0, 0, 0), Vector3(1, 0, 0)),
                 -3.0, 1e-9, "RelVel approaching negative");

        // RayDirLocal: spherical sampling. Centre of the window looks straight
        // ahead; the direction is always unit length; the window edges land on
        // exactly +/-FOV/2; and azimuth may now reach a full turn.
        Vector3 dir = rmath::RayDirLocal(0.5, 0.5, 30.0, 10.0);
        close_to(dir.x, 1.0, 1e-9, "RayDirLocal centre looks forward");
        close_to(dir.length(), 1.0, 1e-9, "RayDirLocal unit length");

        Vector3 left_edge = rmath::RayDirLocal(1.0, 0.5, 90.0, 10.0);
        close_to(std::atan2(left_edge.y, left_edge.x), rmath::DegToRad(45.0), 1e-9,
                 "RayDirLocal +az edge = +HFOV/2");
        Vector3 up_edge = rmath::RayDirLocal(0.5, 1.0, 30.0, 60.0);
        close_to(std::asin(up_edge.z), rmath::DegToRad(30.0), 1e-9,
                 "RayDirLocal +el edge = +VFOV/2");

        // 360 deg azimuth is now expressible: the extremes point backwards.
        Vector3 back_l = rmath::RayDirLocal(1.0, 0.5, 360.0, 10.0);
        Vector3 back_r = rmath::RayDirLocal(0.0, 0.5, 360.0, 10.0);
        check(back_l.x < -0.99 && back_r.x < -0.99, "RayDirLocal 360 deg reaches behind");
        close_to(back_l.length(), 1.0, 1e-9, "RayDirLocal 360 deg unit length");

        // Over-wide requests are clamped rather than diverging.
        Vector3 clamped = rmath::RayDirLocal(1.0, 1.0, 720.0, 400.0);
        close_to(clamped.length(), 1.0, 1e-9, "RayDirLocal clamps over-wide FOV");

        // --- selectable ray distributions -------------------------------
        using RD = radar::RayDistribution;
        // Uniform-in-angle vs uniform-in-solid-angle differ off the centre and
        // agree at the window centre and edges.
        Vector3 ua = rmath::RayDirLocal(0.5, 0.75, 30.0, 90.0, RD::UniformAngle);
        Vector3 us = rmath::RayDirLocal(0.5, 0.75, 30.0, 90.0, RD::UniformSolidAngle);
        close_to(ua.length(), 1.0, 1e-9, "UniformAngle unit length");
        close_to(us.length(), 1.0, 1e-9, "UniformSolidAngle unit length");
        check(std::asin(us.z) < std::asin(ua.z) - 1e-6,
              "UniformSolidAngle pulls samples away from the pole");
        close_to(std::asin(rmath::RayDirLocal(0.5, 1.0, 30.0, 90.0, RD::UniformSolidAngle).z),
                 rmath::DegToRad(45.0), 1e-9, "UniformSolidAngle keeps the +el edge");
        // Boresight-weighted: u_r = 0 is exactly the boresight.
        Vector3 bw0 = rmath::RayDirLocal(0.0, 0.3, 60.0, 20.0, RD::BoresightWeighted);
        close_to(bw0.x, 1.0, 1e-9, "BoresightWeighted centre at u_r=0");
        Vector3 bw1 = rmath::RayDirLocal(1.0, 0.0, 60.0, 20.0, RD::BoresightWeighted);
        close_to(std::atan2(bw1.y, bw1.x), rmath::DegToRad(30.0), 1e-9,
                 "BoresightWeighted reaches the +az edge");

        // --- asymmetric windows -----------------------------------------
        radar::RadarConfig wc {};
        wc.horizontal_fov_deg = 60.0; wc.vertical_fov_deg = 20.0;
        rmath::AngularWindow sym = rmath::WindowOf(wc);
        close_to(sym.az_start_rad, rmath::DegToRad(-30.0), 1e-12, "WindowOf symmetric az start");
        close_to(sym.az_end_rad, rmath::DegToRad(30.0), 1e-12, "WindowOf symmetric az end");

        // A rear-facing sector: 150 .. 210 deg. Nothing in it points forward.
        wc.azimuth_start_deg = 150.0; wc.azimuth_end_deg = 210.0;
        rmath::AngularWindow rear = rmath::WindowOf(wc);
        Vector3 r_lo = rmath::RayDirWindow(0.0, 0.5, rear);
        Vector3 r_mid = rmath::RayDirWindow(0.5, 0.5, rear);
        Vector3 r_hi = rmath::RayDirWindow(1.0, 0.5, rear);
        close_to(r_mid.x, -1.0, 1e-9, "rear window centre looks backwards");
        check(r_lo.x < 0.0 && r_hi.x < 0.0, "rear window never looks forward");
        close_to(r_lo.length(), 1.0, 1e-9, "rear window unit length");
        // Elevation still comes from the symmetric VFOV when not overridden.
        close_to(rear.el_end_rad, rmath::DegToRad(10.0), 1e-12,
                 "asymmetric az leaves el on the symmetric FOV");
        // --- distance-dependent detection --------------------------------
        // Disabled by default.
        close_to(rmath::DetectionProbability(50.0, 0.0, 2.0), 1.0, 1e-12,
                 "DetectionProbability disabled when ref range is 0");
        // Certain out to the reference range, then falling as (ref/R)^exp.
        close_to(rmath::DetectionProbability(5.0, 6.0, 2.0), 1.0, 1e-12,
                 "DetectionProbability certain inside the reference range");
        close_to(rmath::DetectionProbability(12.0, 6.0, 2.0), 0.25, 1e-12,
                 "DetectionProbability falls as 1/R^2 at twice the reference");
        close_to(rmath::DetectionProbability(24.0, 6.0, 2.0), 0.0625, 1e-12,
                 "DetectionProbability keeps falling");
        check(rmath::DetectionProbability(24.0, 6.0, 4.0)
              < rmath::DetectionProbability(24.0, 6.0, 2.0),
              "DetectionProbability steeper exponent falls faster");

        // A downward-tilted elevation slice.
        wc.elevation_start_deg = -40.0; wc.elevation_end_deg = -10.0;
        rmath::AngularWindow down = rmath::WindowOf(wc);
        check(rmath::RayDirWindow(0.5, 0.5, down).z < 0.0, "down window points below the horizon");
    }

    std::printf("== radar scan pipeline (mock backend) ==\n");
    {
        const double D = 12.0;
        auto mock = std::make_shared<SphereMockRayCaster>(D, Vector3(0, 0, 0));
        radar::RadarSensor sensor(mock);
        auto cfg = make_config(30.0, 30.0, 10.0, 1500);  // 150 points/scan
        sensor.SetConfig(cfg);

        radar::RadarScanFrame frame;
        sensor.Scan(forward_state(), frame);

        check(frame.detections.size() == 150, "scan produces points_per_second/rate detections");

        const double max_az = rmath::DegToRad(cfg.horizontal_fov_deg * 0.5) + 1e-3;
        const double max_el = rmath::DegToRad(cfg.vertical_fov_deg * 0.5) + 1e-3;
        bool all_depth_ok = true, all_fov_ok = true;
        for (const auto& d : frame.detections) {
            if (std::fabs(d.depth - D) > 1e-3) all_depth_ok = false;
            if (std::fabs(d.azimuth) > max_az || std::fabs(d.altitude) > max_el) all_fov_ok = false;
        }
        check(all_depth_ok, "all detections at sphere depth D");
        check(all_fov_ok, "all detections within configured FOV cone");
    }

    std::printf("== doppler relative velocity via scan ==\n");
    {
        const double D = 10.0;
        // target moving toward sensor along -x (approaching) -> velocity negative
        auto mock = std::make_shared<SphereMockRayCaster>(D, Vector3(-5, 0, 0));
        radar::RadarSensor sensor(mock);
        sensor.SetConfig(make_config(30.0, 6.0, 6.0, 100));  // narrow FOV -> rays near +x
        radar::RadarScanFrame frame;
        sensor.Scan(forward_state(), frame);
        check(!frame.detections.empty(), "narrow-FOV scan has detections");
        bool all_negative = true;
        for (const auto& d : frame.detections) {
            if (d.velocity >= 0.0F) all_negative = false;
        }
        check(all_negative, "approaching target => negative Doppler velocity");
    }

    std::printf("== determinism (seeded) ==\n");
    {
        auto mock = std::make_shared<SphereMockRayCaster>(8.0, Vector3(1, 0, 0));
        radar::RadarSensor s1(mock);
        radar::RadarSensor s2(mock);
        auto cfg = make_config(20.0, 40.0, 20.0, 200);
        s1.SetConfig(cfg);
        s2.SetConfig(cfg);
        radar::RadarScanFrame f1, f2;
        s1.Scan(forward_state(), f1);
        s2.Scan(forward_state(), f2);
        bool same = (f1.detections.size() == f2.detections.size());
        for (size_t i = 0; same && i < f1.detections.size(); ++i) {
            if (f1.detections[i].azimuth != f2.detections[i].azimuth ||
                f1.detections[i].depth != f2.detections[i].depth) {
                same = false;
            }
        }
        check(same, "same seed => identical scan");
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    if (g_failures > 0) {
        std::printf("RESULT: FAIL (%d failures)\n", g_failures);
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
