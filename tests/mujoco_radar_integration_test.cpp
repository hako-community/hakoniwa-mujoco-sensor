// Real-physics integration test for the MuJoCo radar backend.
//
// Unlike radar_math_test (which mocks the ray caster), this loads an actual
// MuJoCo scene with a MOVING target and drives RadarSensor through
// MujocoRayCaster. It validates the two things only a real engine can confirm:
//   1. mj_ray occlusion -> correct depth to the target.
//   2. mj_objectVelocity -> correct Doppler relative velocity.
//
// Requires linking libmujoco. No PDU / viewer / hakoniwa-core needed.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "sensors/backend/mujoco_ray_caster.hpp"
#include "sensors/radar/radar_sensor.hpp"

using hako::robots::types::Vector3;
namespace backend = hako::robots::sensor::backend;
namespace radar = hako::robots::sensor::radar;

// Radar at origin (z=0.5) looking +x. Target box centred at x=5 on a slide
// joint along x, so we can give it a world velocity via qvel.
// nuser_geom + user on the target carries its radar cross-section: this is the
// contract actors.py writes and MujocoRayCaster reads back.
static const char* kMjcf = R"XML(
<mujoco model="radar_test">
  <option gravity="0 0 0"/>
  <size nuser_geom="1"/>
  <worldbody>
    <geom name="floor" type="plane" size="20 20 0.1" pos="0 0 0"/>
    <body name="target" pos="5 0 0.5">
      <joint name="slide_x" type="slide" axis="1 0 0"/>
      <geom name="target_geom" type="box" size="0.5 0.5 0.5" user="7.5"/>
    </body>
  </worldbody>
</mujoco>
)XML";

static int g_fail = 0;
static int g_checks = 0;
static void close_to(double a, double b, double tol, const char* msg)
{
    ++g_checks;
    if (std::fabs(a - b) > tol) {
        ++g_fail;
        std::printf("  [FAIL] %s (got %.4f want %.4f)\n", msg, a, b);
    } else {
        std::printf("  [ ok ] %s (%.4f)\n", msg, a);
    }
}
static void check(bool c, const char* msg)
{
    ++g_checks;
    if (!c) { ++g_fail; std::printf("  [FAIL] %s\n", msg); }
    else { std::printf("  [ ok ] %s\n", msg); }
}

static backend::SensorState radar_state()
{
    backend::SensorState s {};
    s.origin = Vector3(0, 0, 0.5);
    s.forward = Vector3(1, 0, 0);
    s.left = Vector3(0, 1, 0);
    s.up = Vector3(0, 0, 1);
    s.linear_velocity = Vector3(0, 0, 0);
    return s;
}

static double mean_depth(const radar::RadarScanFrame& f)
{
    double s = 0; for (auto& d : f.detections) s += d.depth; return s / f.detections.size();
}
static double mean_velocity(const radar::RadarScanFrame& f)
{
    double s = 0; for (auto& d : f.detections) s += d.velocity; return s / f.detections.size();
}

int main()
{
    // Write MJCF to a temp file (mj_loadXML needs a path). Must be resolved at
    // run time: a baked-in absolute path makes the test unrunnable anywhere else.
    const std::string path =
        (std::filesystem::temp_directory_path() / "hako_radar_integration_scene.xml").string();
    {
        std::ofstream o(path);
        if (!o) { std::printf("cannot write scene to %s\n", path.c_str()); return 2; }
        o << kMjcf;
    }

    char err[1000] = {0};
    mjModel* m = mj_loadXML(path.c_str(), nullptr, err, sizeof(err));
    if (m == nullptr) { std::printf("mj_loadXML failed: %s\n", err); return 2; }
    mjData* d = mj_makeData(m);

    const int dof = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "slide_x")];

    auto caster = std::make_shared<backend::MujocoRayCaster>(m, d, std::string(""));
    radar::RadarSensor sensor(caster);
    radar::RadarConfig cfg {};
    cfg.range = 20.0;
    cfg.horizontal_fov_deg = 2.0;   // narrow -> rays near +x, all hit target
    cfg.vertical_fov_deg = 2.0;
    cfg.points_per_second = 500;
    cfg.output.update_rate_hz = 10.0;  // 50 pts/scan
    cfg.noise_seed = 7;
    sensor.SetConfig(cfg);

    std::printf("== case A: target approaching at -2 m/s ==\n");
    {
        d->qvel[dof] = -2.0;   // moving toward radar
        mj_forward(m, d);
        radar::RadarScanFrame f;
        sensor.Scan(radar_state(), f);
        check(!f.detections.empty(), "target detected");
        // box front face at x=4.5, radar origin x=0 -> depth ~4.5
        close_to(mean_depth(f), 4.5, 0.15, "mean depth ~ front face (4.5 m)");
        close_to(mean_velocity(f), -2.0, 0.3, "Doppler ~ -2 m/s (approaching)");
    }

    std::printf("== case B: target receding at +3 m/s ==\n");
    {
        d->qvel[dof] = 3.0;
        mj_forward(m, d);
        radar::RadarScanFrame f;
        sensor.Scan(radar_state(), f);
        check(!f.detections.empty(), "target detected");
        close_to(mean_velocity(f), 3.0, 0.4, "Doppler ~ +3 m/s (receding)");
    }

    std::printf("== case D: per-target RCS reaches the sensor ==\n");
    {
        // The value must survive the whole path: MJCF user= -> mjModel.geom_user
        // -> MujocoRayCaster -> RayHit. A mock cannot prove this; only a real
        // model compile can, and silently losing it is the failure mode (MuJoCo
        // drops user data when nuser_geom is not declared).
        d->qvel[dof] = 0.0;
        mj_forward(m, d);
        const backend::RayHit h = caster->Cast(Vector3(0, 0, 0.5), Vector3(1, 0, 0), 20.0);
        check(h.hit, "ray hits the target");
        close_to(h.target_rcs_m2, 7.5, 1e-9, "RCS from geom user= reaches RayHit");

        // The floor carries no user value, so it must read as "unknown" rather
        // than as an RCS of zero.
        const backend::RayHit f = caster->Cast(Vector3(0, 0, 1.0), Vector3(0, 0, -1), 20.0);
        check(f.hit, "ray hits the floor");
        check(f.target_rcs_m2 < 0.0, "un-annotated geom reports unknown RCS");
    }

    std::printf("== case E: Doppler is the velocity of the HIT POINT ==\n");
    {
        // A body spinning about z whose surface is well off the rotation axis.
        // Taking mj_objectVelocity's reference point verbatim gets this wrong:
        // XBODY reports the (stationary) body origin, BODY reports the COM, and
        // neither is the point the ray struck. Only the rigid-body transfer
        // v_P = v_O + omega x (P - O) matches the analytic answer.
        static const char* kSpin = R"XML(
<mujoco model="spin">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="arm" pos="0 0 0">
      <joint name="hz" type="hinge" axis="0 0 1"/>
      <geom name="pad" type="box" size="0.5 0.5 0.5" pos="5 0 0" mass="1"/>
    </body>
  </worldbody>
</mujoco>
)XML";
        const std::string sp =
            (std::filesystem::temp_directory_path() / "hako_radar_spin_scene.xml").string();
        { std::ofstream o(sp); o << kSpin; }
        char e2[1000] = {0};
        mjModel* sm = mj_loadXML(sp.c_str(), nullptr, e2, sizeof(e2));
        check(sm != nullptr, "spin scene loads");
        if (sm != nullptr) {
            mjData* sd = mj_makeData(sm);
            const double omega = 2.0;          // rad/s about +z
            sd->qvel[0] = omega;
            mj_forward(sm, sd);

            backend::MujocoRayCaster c2(sm, sd, std::string(""));
            // Ray along +x hits the near face at x = 4.5, on the y = 0 plane.
            const backend::RayHit h = c2.Cast(Vector3(0, 0, 0), Vector3(1, 0, 0), 20.0);
            check(h.hit, "ray hits the spinning arm");
            // v = omega x r, r = (4.5, 0, 0) -> v = (0, 9.0, 0)
            close_to(h.target_velocity.x, 0.0, 1e-6, "hit-point velocity x");
            close_to(h.target_velocity.y, omega * 4.5, 1e-6, "hit-point velocity y = omega * r");
            close_to(h.target_velocity.z, 0.0, 1e-6, "hit-point velocity z");

            // Radial component along the ray is ~0 here (motion is tangential),
            // which is the physically meaningful statement: a target spinning
            // about an axis through the sensor line-of-sight has no closing rate.
            mj_deleteData(sd);
            mj_deleteModel(sm);
        }
    }

    std::printf("== case C: target stationary ==\n");
    {
        d->qvel[dof] = 0.0;
        mj_forward(m, d);
        radar::RadarScanFrame f;
        sensor.Scan(radar_state(), f);
        close_to(mean_velocity(f), 0.0, 0.15, "Doppler ~ 0 (stationary)");
    }

    mj_deleteData(d);
    mj_deleteModel(m);

    std::printf("\n%d/%d checks passed\n", g_checks - g_fail, g_checks);
    std::printf("RESULT: %s\n", (g_fail == 0) ? "PASS" : "FAIL");
    return (g_fail == 0) ? 0 : 1;
}
