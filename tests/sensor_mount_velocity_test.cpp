// Issue #12: the sensor's own angular velocity.
//
// Doppler is the radial velocity between the transceiver and the scattering
// point. #11 fixed the target end -- mj_objectVelocity reports the velocity of
// a body's reference point, not of the point the ray struck, so a spinning
// target needs v_O + omega x (hit - xpos). The sensor end had the mirror-image
// hole: the pose PDU reports the velocity of the AIRFRAME origin, but the radar
// is bolted 0.15 m ahead of it. While the drone yaws, that mount swings on a
// lever arm and does not travel at the airframe's velocity.
//
// Feeding the airframe velocity straight through therefore biased every reading
// by the projection of omega x r_mount on the ray -- up to 0.15 m/s at a
// 1 rad/s yaw rate, on top of targets whose own rotation was already exact.
// This test pins both the shared transfer rule and the runtime that applies it.
//
// Backend-free: MakeState is inline and nothing here constructs a SensorRuntime,
// so no MuJoCo runtime and no live PDU channels are involved.

#include <cmath>
#include <cstdio>
#include <memory>

#include "runtime/sensor_runtime.hpp"
#include "sensors/radar/radar_math.hpp"
#include "sensors/radar/radar_sensor.hpp"

using hako::robots::types::Vector3;
namespace backend = hako::robots::sensor::backend;
namespace radar = hako::robots::sensor::radar;
namespace rmath = hako::robots::sensor::radar::math;
namespace rt = hako::robots::runtime;

static int g_fail = 0;
static int g_checks = 0;

static void check(bool cond, const char* msg)
{
    ++g_checks;
    if (!cond) { ++g_fail; std::printf("  [FAIL] %s\n", msg); }
    else { std::printf("  [ ok ] %s\n", msg); }
}

static void close_to(double a, double b, double tol, const char* msg)
{
    ++g_checks;
    if (std::fabs(a - b) > tol) {
        ++g_fail;
        std::printf("  [FAIL] %s (got %.9f want %.9f)\n", msg, a, b);
    } else {
        std::printf("  [ ok ] %s (%.6f)\n", msg, a);
    }
}

static void vec_close(const Vector3& got, const Vector3& want, double tol, const char* msg)
{
    ++g_checks;
    if (std::fabs(got.x - want.x) > tol ||
        std::fabs(got.y - want.y) > tol ||
        std::fabs(got.z - want.z) > tol) {
        ++g_fail;
        std::printf("  [FAIL] %s (got (%.6f,%.6f,%.6f) want (%.6f,%.6f,%.6f))\n",
                    msg, got.x, got.y, got.z, want.x, want.y, want.z);
    } else {
        std::printf("  [ ok ] %s\n", msg);
    }
}

// Every ray hits a motionless surface at 10 m. A static world means whatever
// Doppler comes back is entirely the SENSOR's motion -- which is the quantity
// under test.
class StaticWallCaster : public backend::IRayCaster
{
public:
    backend::RayHit Cast(const Vector3& o, const Vector3& d, double) override
    {
        backend::RayHit h {};
        h.hit = true;
        h.distance = 10.0;
        h.point = Vector3(o.x + d.x * 10.0, o.y + d.y * 10.0, o.z + d.z * 10.0);
        h.target_velocity = Vector3(0, 0, 0);   // the world is not moving
        h.target_id = 1;
        return h;
    }
};

// Narrow forward beam, no noise, no probabilistic dropout: every ray is a
// detection so the velocity assertions are exact rather than statistical.
static radar::RadarConfig pencil_beam()
{
    radar::RadarConfig cfg {};
    cfg.range = 30.0;
    cfg.horizontal_fov_deg = 0.0;   // pencil beam: every ray points along +x
    cfg.vertical_fov_deg = 0.0;
    cfg.points_per_second = 100;
    cfg.noise_seed = 3;
    return cfg;
}

int main()
{
    std::printf("== VelocityAtPoint: the rigid-body transfer rule ==\n");
    {
        // Pure translation: the offset is irrelevant.
        vec_close(backend::VelocityAtPoint(Vector3(1, 2, 3), Vector3(0, 0, 0), Vector3(5, 5, 5)),
                  Vector3(1, 2, 3), 1e-12, "no rotation -> reference velocity unchanged");

        // Pure yaw: a point 2 m along +x on a body spinning at 3 rad/s about +z
        // moves along +y at 6 m/s.
        vec_close(backend::VelocityAtPoint(Vector3(0, 0, 0), Vector3(0, 0, 3), Vector3(2, 0, 0)),
                  Vector3(0, 6, 0), 1e-12, "omega x r for a +x offset under +z spin");

        // An offset parallel to the rotation axis contributes nothing.
        vec_close(backend::VelocityAtPoint(Vector3(0, 0, 0), Vector3(0, 0, 3), Vector3(0, 0, 2)),
                  Vector3(0, 0, 0), 1e-12, "offset along the spin axis adds nothing");

        // Translation and rotation superpose.
        vec_close(backend::VelocityAtPoint(Vector3(1, 0, 0), Vector3(0, 0, 1), Vector3(0, 1, 0)),
                  Vector3(0, 0, 0), 1e-12, "translation and lever arm can cancel exactly");
    }

    std::printf("\n== MakeState: mount offset gains its lever-arm term ==\n");
    {
        rt::Mount m {};
        m.x = 0.15;             // the stock front-radar mount

        // Non-regression: with no rotation the sensor velocity is the base's,
        // byte for byte what every existing manifest produced before #12.
        {
            rt::BasePose base {};
            base.linear_velocity = Vector3(2.0, 0.0, 0.0);
            const auto st = rt::MakeState(base, m);
            vec_close(st.linear_velocity, Vector3(2.0, 0.0, 0.0), 1e-12,
                      "omega = 0 -> unchanged from the pre-#12 behaviour");
            vec_close(st.origin, Vector3(0.15, 0.0, 0.0), 1e-12,
                      "origin still base + rotated mount offset");
        }

        // Yawing in place at 1 rad/s: the mount sweeps sideways at |omega| * 0.15.
        {
            rt::BasePose base {};
            base.angular_velocity = Vector3(0.0, 0.0, 1.0);
            const auto st = rt::MakeState(base, m);
            vec_close(st.linear_velocity, Vector3(0.0, 0.15, 0.0), 1e-12,
                      "yaw rate 1 rad/s, mount 0.15 m fwd -> 0.15 m/s to port");
            vec_close(st.angular_velocity, Vector3(0.0, 0.0, 1.0), 1e-12,
                      "angular velocity is carried into SensorState");
        }

        // The offset is body-frame: at 90 deg of base yaw the same mount points
        // along world +y, so the lever-arm term rotates with it.
        {
            rt::BasePose base {};
            base.yaw_rad = M_PI / 2.0;
            base.angular_velocity = Vector3(0.0, 0.0, 1.0);
            const auto st = rt::MakeState(base, m);
            vec_close(st.origin, Vector3(0.0, 0.15, 0.0), 1e-12,
                      "mount offset rotates into world by base yaw");
            vec_close(st.linear_velocity, Vector3(-0.15, 0.0, 0.0), 1e-12,
                      "lever-arm term rotates with the mount");
        }

        // A purely vertical mount is on the yaw axis and gains nothing.
        {
            rt::Mount up {};
            up.z = 0.15;
            rt::BasePose base {};
            base.angular_velocity = Vector3(0.0, 0.0, 4.0);
            const auto st = rt::MakeState(base, up);
            vec_close(st.linear_velocity, Vector3(0.0, 0.0, 0.0), 1e-12,
                      "mount on the yaw axis is unaffected by yaw rate");
        }

        // Superposition: a translating AND yawing drone gets both terms.
        {
            rt::BasePose base {};
            base.linear_velocity = Vector3(3.0, 0.0, 0.0);
            base.angular_velocity = Vector3(0.0, 0.0, 2.0);
            const auto st = rt::MakeState(base, m);
            vec_close(st.linear_velocity, Vector3(3.0, 0.30, 0.0), 1e-12,
                      "translation + lever arm superpose");
        }
    }

    std::printf("\n== End to end: a yawing drone facing a static wall ==\n");
    {
        // Radar mounted 0.15 m to PORT and aimed along the airframe's +x.
        // Yawing at 1 rad/s swings that mount backwards along -x at 0.15 m/s,
        // so the wall dead ahead appears to RECEDE at 0.15 m/s.
        // Sign convention (radar_math): positive = range increasing.
        rt::Mount side {};
        side.y = 0.15;

        rt::BasePose spinning {};
        spinning.angular_velocity = Vector3(0.0, 0.0, 1.0);
        rt::BasePose still {};

        auto doppler_of = [&](const rt::BasePose& base) {
            auto caster = std::make_shared<StaticWallCaster>();
            radar::RadarSensor s(caster);
            s.SetConfig(pencil_beam());
            radar::RadarScanFrame f {};
            s.Scan(rt::MakeState(base, side), f);
            check(!f.detections.empty(), "the scan produced detections");
            return f.detections.empty() ? 0.0 : static_cast<double>(f.detections[0].velocity);
        };

        close_to(doppler_of(still), 0.0, 1e-6,
                 "a motionless drone reads 0 m/s on a static wall");
        close_to(doppler_of(spinning), 0.15, 1e-6,
                 "yawing at 1 rad/s on a 0.15 m side mount reads 0.15 m/s");

        // This is the whole bug: before #12 both cases above read 0.0, because
        // the airframe origin genuinely was not moving. The wall looked static
        // while the sensor swung past it.
        const double pre_fix = rmath::RelativeVelocity(
            Vector3(0, 0, 0),                 // static wall
            still.linear_velocity,            // airframe velocity, lever arm dropped
            Vector3(1, 0, 0));
        close_to(pre_fix, 0.0, 1e-12,
                 "pre-#12 path reads 0 m/s for the same yawing drone (the defect)");

        // The stock FORWARD mount is the interesting case, and it is not the
        // one above. omega x r_mount is perpendicular to the boresight there, so
        // a target dead ahead correctly reads 0 even at speed -- the term only
        // appears off-axis, peaking at |omega||r| * sin(half-FOV) = 0.075 m/s
        // per rad/s across the 60 deg window the manifests use. Pinned here so a
        // future reader does not mistake the on-axis zero for the bug returning.
        rt::Mount front {};
        front.x = 0.15;
        auto caster = std::make_shared<StaticWallCaster>();
        radar::RadarSensor s(caster);
        s.SetConfig(pencil_beam());   // FOV 0 -> every ray on the boresight
        radar::RadarScanFrame f {};
        s.Scan(rt::MakeState(spinning, front), f);
        check(!f.detections.empty(), "boresight scan produced detections");
        if (!f.detections.empty()) {
            close_to(static_cast<double>(f.detections[0].velocity), 0.0, 1e-6,
                     "forward mount reads 0 m/s dead ahead: lever arm is perpendicular");
        }
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_fail, g_checks);
    if (g_fail > 0) {
        std::printf("RESULT: FAIL (%d failures)\n", g_fail);
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
