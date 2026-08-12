// M2 demo: A-2 sensing over a simenv-data generated world.
//
// Loads env.xml (produced by hakoniwa-simenv-data) into a kinematic MuJoCo
// world, injects a FIXED sensor pose (no live PDU yet -- that is M3), runs the
// backend-agnostic LidarScanSensor through MujocoRayCaster, and checks the
// measured wall distances against the known simple_room geometry.
//
// This proves: env.xml load + pose injection + LiDAR sensing (A-2), with the
// drone/sensor NOT present in the scanned world.
//
// Build: see build.bash

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "sensors/backend/mujoco_ray_caster.hpp"
#include "sensors/lidar/lidar_scan_sensor.hpp"

using hako::robots::types::Vector3;
namespace backend = hako::robots::sensor::backend;
namespace lidar = hako::robots::sensor::lidar;

static int g_fail = 0;
static int g_checks = 0;

static void close_to(double got, double want, double tol, const char* msg)
{
    ++g_checks;
    if (std::fabs(got - want) > tol) {
        ++g_fail;
        std::printf("  [FAIL] %s (got %.3f want %.3f)\n", msg, got, want);
    } else {
        std::printf("  [ ok ] %s (%.3f m)\n", msg, got);
    }
}

int main(int argc, char** argv)
{
    const std::string xml =
        (argc > 1) ? argv[1]
                   : "../../../hakoniwa-simenv-data/examples/sensor_envs/simple_room/generated/env.xml";

    char err[1000] = {0};
    mjModel* model = mj_loadXML(xml.c_str(), nullptr, err, sizeof(err));
    if (model == nullptr) {
        std::printf("ERROR: mj_loadXML(%s) failed: %s\n", xml.c_str(), err);
        return 2;
    }
    mjData* data = mj_makeData(model);
    mj_forward(model, data);  // kinematic: place geoms, no integration
    std::printf("loaded %s  (ngeom=%ld nbody=%ld)\n", xml.c_str(),
                static_cast<long>(model->ngeom), static_cast<long>(model->nbody));

    // A-2: the sensor is NOT a body in this world; exclude name is empty.
    auto caster = std::make_shared<backend::MujocoRayCaster>(model, data, std::string{});

    lidar::LidarScanSensor sensor(caster);
    lidar::LidarScanConfig cfg{};
    cfg.frame_id = "lidar";
    cfg.angle_min_deg = -180.0;
    cfg.angle_max_deg = 180.0;
    cfg.angle_increment_deg = 1.0;
    cfg.range_min = 0.05;
    cfg.range_max = 20.0;
    sensor.SetConfig(cfg);

    // Fixed pose: room centre at z=1. MuJoCo frame X=North, Y=West, Z=Up.
    backend::SensorState st{};
    st.origin = Vector3(0.0, 0.0, 1.0);
    st.forward = Vector3(1.0, 0.0, 0.0);  // +North
    st.left = Vector3(0.0, 1.0, 0.0);     // +West
    st.up = Vector3(0.0, 0.0, 1.0);
    st.linear_velocity = Vector3(0.0, 0.0, 0.0);

    lidar::LaserScanFrame frame{};
    sensor.Scan(st, frame);

    const int n = static_cast<int>(frame.ranges.size());
    std::printf("scan: %d rays, angle[%.1f, %.1f] deg\n", n,
                frame.angle_min * 180.0 / M_PI, frame.angle_max * 180.0 / M_PI);

    // index for a given degree d: d - angle_min (== d + 180)
    auto at = [&](double deg) -> double {
        int idx = static_cast<int>(std::lround(deg - cfg.angle_min_deg));
        if (idx < 0 || idx >= n) return -1.0;
        return frame.ranges[static_cast<size_t>(idx)];
    };

    // simple_room: walls at +/-4 (N/S, half thickness 0.1 -> 3.9 m),
    // +/-5 (E/W, half thickness 0.1 -> 4.9 m). pillar near +N/+W is off-axis.
    std::printf("checks (fixed pose at room centre):\n");
    close_to(at(0.0), 3.9, 0.05, "forward +North -> wall_north");
    close_to(at(180.0), 3.9, 0.05, "back -North   -> wall_south");
    close_to(at(90.0), 4.9, 0.05, "left +West    -> wall_west");
    close_to(at(-90.0), 4.9, 0.05, "right -West   -> wall_east");

    // Move the sensor 2 m North: distances must shift accordingly, proving the
    // injected pose (not a hard-coded centre) drives the scan.
    st.origin = Vector3(2.0, 0.0, 1.0);
    sensor.Scan(st, frame);
    std::printf("checks (pose shifted +2 m North):\n");
    close_to(at(0.0), 1.9, 0.05, "forward +North -> wall_north (closer)");
    close_to(at(180.0), 5.9, 0.05, "back -North   -> wall_south (farther)");

    mj_deleteData(data);
    mj_deleteModel(model);

    std::printf("\n%s (%d/%d checks passed)\n",
                g_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL",
                g_checks - g_fail, g_checks);
    return g_fail == 0 ? 0 : 1;
}
