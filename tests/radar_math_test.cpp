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
        h.target_rcs_m2 = rcs;   // negative (default) = backend has no RCS
        return h;
    }

    // Public so a test can vary only the RCS while holding geometry fixed.
    double rcs {-1.0};

private:
    double d_;
    Vector3 v_;
};

// Mock backend: a target of fixed ANGULAR size sitting at a fixed bearing. A ray
// hits only if it falls inside that patch, so the number of detections measures
// how densely the sampler covers that part of the sky -- which is what changes
// when the elevation window is widened (issue #7).
class PatchMockRayCaster : public backend::IRayCaster
{
public:
    PatchMockRayCaster(double d, double az_deg, double el_deg, double half_deg)
        : d_(d), az_(rmath::DegToRad(az_deg)), el_(rmath::DegToRad(el_deg)),
          half_(rmath::DegToRad(half_deg)) {}

    backend::RayHit Cast(const Vector3& origin, const Vector3& dir, double /*max*/) override
    {
        backend::RayHit h {};
        const double az = std::atan2(dir.y, dir.x);
        const double el = std::atan2(dir.z, std::hypot(dir.x, dir.y));
        if (std::fabs(az - az_) > half_ || std::fabs(el - el_) > half_) {
            return h;   // miss: the ray went past the target
        }
        h.hit = true;
        h.distance = d_;
        h.point = Vector3(origin.x + dir.x * d_, origin.y + dir.y * d_, origin.z + dir.z * d_);
        h.target_id = 1;
        h.target_rcs_m2 = -1.0;
        return h;
    }

private:
    double d_, az_, el_, half_;
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

    // --- radar equation (link budget -> reference range) --------------------
    {
        // Hand-worked reference point. Pt=1 W, G=0 dBi (isotropic, so G=1),
        // lambda=1 m, sigma=1 m^2, Smin=1e-9 W:
        //   Rmax = [ 1 / ((4pi)^3 * 1e-9) ] ^ (1/4)
        const double four_pi_cubed = std::pow(4.0 * rmath::kPi, 3.0);
        const double want = std::pow(1.0 / (four_pi_cubed * 1e-9), 0.25);
        close_to(rmath::RadarEquationRange(1.0, 0.0, 1.0, 1.0, 1e-9), want, 1e-9,
                 "radar equation matches the closed form");

        // Incomplete budgets must return 0 so the caller keeps the direct parameter.
        check(rmath::RadarEquationRange(0.0, 0.0, 1.0, 1.0, 1e-9) == 0.0, "no Pt => 0");
        check(rmath::RadarEquationRange(1.0, 0.0, 0.0, 1.0, 1e-9) == 0.0, "no lambda => 0");
        check(rmath::RadarEquationRange(1.0, 0.0, 1.0, 1.0, 0.0) == 0.0, "no Smin => 0");

        // Gain enters squared: +3 dBi (x2 linear) on both tx and rx is x4 power,
        // and range goes as the fourth root, so exactly x sqrt(2).
        const double g0 = rmath::RadarEquationRange(1.0, 0.0, 1.0, 1.0, 1e-9);
        const double g3 = rmath::RadarEquationRange(1.0, 3.0102999566, 1.0, 1.0, 1e-9);
        close_to(g3 / g0, std::sqrt(2.0), 1e-6, "+3 dBi => range x sqrt(2)");

        // 16x the RCS is 2x the range (fourth root).
        const double s1 = rmath::RadarEquationRange(1.0, 0.0, 1.0, 1.0, 1e-9);
        const double s16 = rmath::RadarEquationRange(1.0, 0.0, 1.0, 16.0, 1e-9);
        close_to(s16 / s1, 2.0, 1e-9, "16x RCS => 2x range");
    }

    // --- per-target RCS scaling --------------------------------------------
    {
        close_to(rmath::ScaleRangeByRcs(10.0, 16.0, 1.0), 20.0, 1e-9,
                 "16x RCS scales reference range x2");
        close_to(rmath::ScaleRangeByRcs(10.0, 1.0, 1.0), 10.0, 1e-9,
                 "RCS == reference leaves range unchanged");
        close_to(rmath::ScaleRangeByRcs(10.0, 1.0, 16.0), 5.0, 1e-9,
                 "target 16x weaker than reference => half the range");
        // Degenerate inputs must not poison the range.
        close_to(rmath::ScaleRangeByRcs(10.0, -1.0, 1.0), 10.0, 1e-9,
                 "unknown RCS leaves range unchanged");
        // Consistency with the equation itself: scaling the reference range by
        // sigma must equal re-evaluating the whole budget at that sigma.
        const double base = rmath::RadarEquationRange(1.0, 6.0, 0.0039, 1.0, 1e-12);
        const double direct = rmath::RadarEquationRange(1.0, 6.0, 0.0039, 7.5, 1e-12);
        close_to(rmath::ScaleRangeByRcs(base, 7.5, 1.0), direct, 1e-9,
                 "scaling agrees with re-evaluating the budget");
    }

    // --- a more reflective target is detected further out -------------------
    {
        // Same geometry, same seed; only the RCS reported by the backend differs.
        // The weak target must not produce more detections than the strong one.
        radar::RadarConfig cfg {};
        cfg.range = 40.0;
        cfg.horizontal_fov_deg = 30.0;
        cfg.vertical_fov_deg = 10.0;
        cfg.points_per_second = 4000;
        cfg.detection_reference_range = 5.0;   // well inside 30 m, so falloff bites
        cfg.detection_falloff_exp = 2.0;
        cfg.reference_rcs_m2 = 1.0;
        cfg.noise_seed = 7;

        auto count_at = [&](double rcs) {
            auto caster = std::make_shared<SphereMockRayCaster>(30.0, Vector3(0, 0, 0));
            caster->rcs = rcs;
            radar::RadarSensor s(caster);
            s.SetConfig(cfg);
            radar::RadarScanFrame f {};
            s.Scan(forward_state(), f);
            return f.detections.size();
        };

        const size_t weak = count_at(0.01);   // radar-absorbent
        const size_t ref = count_at(1.0);     // the reference target
        const size_t strong = count_at(100.0); // corner-reflector-like
        check(weak < ref, "low-RCS target yields fewer detections than the reference");
        check(ref < strong, "high-RCS target yields more detections than the reference");
    }

    // --- elevation coverage vs ray density (#7) -----------------------------
    // Widening the elevation window is not free, and the cost is not the one the
    // issue named. Ground clutter is a property of the SCENE; this is a property
    // of the SAMPLER and applies even to an empty sky: PointsPerScan() does not
    // depend on the window, so the same rays get spread over a larger solid
    // angle and every target inside it collects proportionally fewer of them.
    // A wider radar is therefore a LESS sensitive radar unless the point rate is
    // raised to match. Measured here rather than argued.
    {
        auto count_win = [](double hfov, double el_start, double el_end, int pps,
                            double target_az_deg, double target_el_deg) {
            radar::RadarConfig c {};
            c.range = 30.0;
            c.horizontal_fov_deg = hfov;
            c.vertical_fov_deg = 20.0;          // ignored when the window is set
            if (!(el_start == 0.0 && el_end == 0.0)) {
                c.elevation_start_deg = el_start;
                c.elevation_end_deg = el_end;
            }
            c.points_per_second = pps;
            c.noise_seed = 11U;
            c.output.update_rate_hz = 1.0;      // pps points per scan
            auto caster = std::make_shared<PatchMockRayCaster>(10.0, target_az_deg,
                                                              target_el_deg, 3.0);
            radar::RadarSensor s(caster);
            s.SetConfig(c);
            radar::RadarScanFrame f {};
            s.Scan(forward_state(), f);
            return f.detections.size();
        };
        auto count = [&](double el_start, double el_end, int pps, double target_el_deg) {
            return count_win(60.0, el_start, el_end, pps, 0.0, target_el_deg);
        };

        // Target on the boresight. Baseline = the shipped 20 deg symmetric FOV.
        const size_t base = count(0.0, 0.0, 60000, 0.0);
        // Same point rate, elevation opened to 45 deg (-35..+10): 2.25x the span.
        const size_t wide = count(-35.0, 10.0, 60000, 0.0);
        // Point rate scaled by the same 2.25 -> density restored.
        const size_t wide_comp = count(-35.0, 10.0, 135000, 0.0);

        check(base > 0 && wide > 0, "patch target is hit in both configurations");
        const double dilution = static_cast<double>(base) / static_cast<double>(wide);
        check(dilution > 1.9 && dilution < 2.6,
              "widening elevation 20->45 deg costs ~2.25x the return density");
        const double restored = static_cast<double>(wide_comp) / static_cast<double>(base);
        check(restored > 0.85 && restored < 1.15,
              "scaling points_per_second by the span ratio restores the density");
        std::printf("  [#7] returns on a 6 deg target: vfov20=%zu  el[-35,+10]=%zu  "
                    "el[-35,+10]@2.25x pps=%zu\n", base, wide, wide_comp);

        // And the coverage that motivates the change: traffic 25 deg below the
        // horizon -- an aircraft on final approach seen from above, close in --
        // is simply not there for the symmetric window, at any point rate.
        check(count(0.0, 0.0, 600000, -25.0) == 0,
              "a target 25 deg below the horizon is invisible to the 20 deg FOV");
        check(count(-35.0, 10.0, 60000, -25.0) > 0,
              "the downward-biased window sees it");

        // --- #13: the same law over BOTH axes at once -----------------------
        // Opening azimuth as well multiplies the dilution rather than adding to
        // it: the cost is the ratio of SOLID ANGLES, (150x45)/(60x20) = 5.625.
        // Getting this wrong is how a "better" radar ends up less sensitive than
        // the one it replaces, so the number the manifest has to use is fixed here.
        const size_t both = count_win(150.0, -35.0, 10.0, 60000, 0.0, 0.0);
        const double both_dilution = static_cast<double>(base) / static_cast<double>(both);
        check(both_dilution > 4.8 && both_dilution < 6.5,
              "widening azimuth 60->150 AND elevation 20->45 costs ~5.6x, not ~2.25+2.5");
        const size_t both_comp = count_win(150.0, -35.0, 10.0, 337500, 0.0, 0.0);
        const double both_restored = static_cast<double>(both_comp) / static_cast<double>(base);
        check(both_restored > 0.85 && both_restored < 1.15,
              "scaling pps by the solid-angle ratio restores it on both axes too");
        std::printf("  [#13] wide fit: el+az widened=%zu  (x%.2f dilution)  "
                    "@5.625x pps=%zu\n", both, both_dilution, both_comp);

        // The coverage that motivates #13: traffic 45 deg off the nose -- the
        // bearing to an aircraft converging on a landing point from the side.
        check(count_win(60.0, -35.0, 10.0, 600000, -45.0, -5.0) == 0,
              "a target 45 deg off the nose is invisible to the 60 deg azimuth sector");
        check(count_win(150.0, -35.0, 10.0, 60000, -45.0, -5.0) > 0,
              "the 150 deg sector sees it");
    }

    // --- does a moving-target filter remove ground clutter? (#7) ------------
    // The other cost of looking down is the ground, and the usual answer is
    // "the Doppler filter deals with it". That answer holds only while the
    // aircraft is stationary. Static ground seen from a radar moving at v has a
    // closing rate of v*cos(el) -- indistinguishable from a real target -- so
    // the filter that saved S-7 in a room full of walls does NOT save a moving
    // aircraft from its own ground return. Pinned here because the live
    // measurement can only be taken while hovering.
    {
        auto doppler_of_ground = [](double own_speed, double el_deg) {
            radar::RadarConfig c {};
            c.range = 30.0;
            c.horizontal_fov_deg = 4.0;             // a pencil beam at the patch
            c.elevation_start_deg = el_deg - 2.0;
            c.elevation_end_deg = el_deg + 2.0;
            c.points_per_second = 200;
            c.noise_seed = 5U;
            c.output.update_rate_hz = 1.0;
            auto caster = std::make_shared<PatchMockRayCaster>(6.0, 0.0, el_deg, 5.0);
            radar::RadarSensor s(caster);
            s.SetConfig(c);
            backend::SensorState st = forward_state();
            st.linear_velocity = Vector3(own_speed, 0, 0);   // flying forward
            radar::RadarScanFrame f {};
            s.Scan(st, f);
            double sum = 0.0;
            for (const auto& d : f.detections) sum += d.velocity;
            return f.detections.empty() ? 0.0 : sum / static_cast<double>(f.detections.size());
        };

        const double hovering = doppler_of_ground(0.0, -30.0);
        close_to(hovering, 0.0, 1e-3,
                 "hovering: static ground reads as static (Doppler ~ 0)");

        // 1 m/s forward, ground 30 deg below: -v*cos(30) = -0.866 m/s.
        const double moving = doppler_of_ground(1.0, -30.0);
        close_to(moving, -std::cos(rmath::DegToRad(30.0)), 1e-2,
                 "flying: the same ground closes at v*cos(el)");
        check(std::fabs(moving) > 0.05,
              "which is well above any sane moving-target threshold -- the "
              "Doppler filter does NOT remove ground clutter from a moving aircraft");
        std::printf("  [#7] ground Doppler at el -30 deg: hovering %.3f m/s, "
                    "1 m/s forward %.3f m/s\n", hovering, moving);
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    if (g_failures > 0) {
        std::printf("RESULT: FAIL (%d failures)\n", g_failures);
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
