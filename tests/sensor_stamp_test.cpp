// Scan timestamps: every range sensor must stamp its frames, and the stamp must
// reach the PDU header.
//
// Why this matters: these payloads are byte-identical when the scene has not
// changed. Without a timestamp a consumer cannot distinguish "nothing moved"
// from "the sensor died", which is exactly the failure a DAA system must not
// miss. radar and lidar3d had stamps; lidar_2d, lidar_scan and ultrasonic did
// not (issue #4).
//
// Backend-free: the only sensor exercised directly is LidarScanSensor, which
// takes an IRayCaster and so needs no MuJoCo. lidar_2d and ultrasonic are bound
// to IWorld, but they use the identical one-line stamping pattern and their PDU
// path is covered here through the converters.

#include <cmath>
#include <cstdio>
#include <memory>

#include "hakoniwa/pdu/converter/common.hpp"
#include "hakoniwa/pdu/converter/sensor_msgs/laser_scan.hpp"
#include "hakoniwa/pdu/converter/sensor_msgs/range.hpp"
#include "sensors/lidar/lidar_scan_sensor.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"

using hako::robots::types::Vector3;
namespace backend = hako::robots::sensor::backend;
namespace lidar = hako::robots::sensor::lidar;
namespace ultra = hako::robots::sensor::ultrasonic;
namespace conv = hako::robots::pdu::converter;

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

class HitCaster : public backend::IRayCaster
{
public:
    backend::RayHit Cast(const Vector3& o, const Vector3& d, double) override
    {
        backend::RayHit h {};
        h.hit = true;
        h.distance = 3.0;
        h.point = Vector3(o.x + d.x * 3.0, o.y + d.y * 3.0, o.z + d.z * 3.0);
        h.target_id = 1;
        return h;
    }
};

static backend::SensorState state_at_origin()
{
    backend::SensorState s {};
    s.origin = Vector3(0, 0, 0);
    s.forward = Vector3(1, 0, 0);
    s.left = Vector3(0, 1, 0);
    s.up = Vector3(0, 0, 1);
    return s;
}

int main()
{
    std::printf("== ToHakoTime splits sec/nanosec correctly ==\n");
    {
        auto t = conv::ToHakoTime(2.15);
        check(t.sec == 2, "2.15 s -> sec = 2");
        // The previous inline conversions truncated, turning 0.15 s into
        // 149999999 ns. Rounding is both correct and consistent across sensors.
        check(t.nanosec == 150000000U, "2.15 s -> nanosec = 150000000 (not truncated)");

        auto z = conv::ToHakoTime(0.0);
        check(z.sec == 0 && z.nanosec == 0U, "0 s -> 0/0");
    }

    std::printf("== LidarScanSensor stamps and advances ==\n");
    {
        lidar::LidarScanConfig cfg {};
        cfg.frame_id = "scan_frame";
        cfg.angle_min_deg = -10.0;
        cfg.angle_max_deg = 10.0;
        cfg.angle_increment_deg = 1.0;
        cfg.range_min = 0.05;
        cfg.range_max = 20.0;
        cfg.scan_frequency_hz = 10;   // -> 0.1 s per scan

        lidar::LidarScanSensor s(std::make_shared<HitCaster>());
        s.SetConfig(cfg);
        const double period = s.GetUpdatePeriodSec();

        lidar::LaserScanFrame f1 {}, f2 {}, f3 {};
        s.Scan(state_at_origin(), f1);
        s.Scan(state_at_origin(), f2);
        s.Scan(state_at_origin(), f3);

        check(f1.stamp_sec > 0.0, "first scan is stamped (not left at 0)");
        close_to(f1.stamp_sec, period, 1e-12, "first stamp = one period");
        close_to(f2.stamp_sec - f1.stamp_sec, period, 1e-12, "stamp advances by one period");
        close_to(f3.stamp_sec - f2.stamp_sec, period, 1e-12, "and again");

        // The scene never changes, so the ranges are identical every time. This
        // is precisely the case the stamp exists to disambiguate.
        check(f1.ranges == f2.ranges, "payload is identical between scans");
        check(f1.stamp_sec != f2.stamp_sec, "...but the stamp still differs");
    }

    std::printf("== LaserScan PDU carries stamp and frame_id ==\n");
    {
        lidar::LaserScanFrame f {};
        f.frame_id = "scan_frame";
        f.stamp_sec = 4.25;
        f.ranges = {1.0F, 2.0F};
        auto pdu = conv::sensor_msgs::ToHakoPdu(f);
        check(pdu.header.stamp.sec == 4, "LaserScan stamp.sec");
        check(pdu.header.stamp.nanosec == 250000000U, "LaserScan stamp.nanosec");
        // frame_id was previously dropped on the floor by this converter.
        check(pdu.header.frame_id == "scan_frame", "LaserScan frame_id reaches the PDU");
    }

    std::printf("== Range PDU carries stamp ==\n");
    {
        ultra::UltrasonicConfig cfg {};
        cfg.frame_id = "ultrasonic_frame";
        ultra::UltrasonicFrame f {};
        f.stamp_sec = 7.5;
        f.range = 1.25;
        auto pdu = conv::sensor_msgs::ToHakoPdu(cfg, f);
        check(pdu.header.stamp.sec == 7, "Range stamp.sec");
        check(pdu.header.stamp.nanosec == 500000000U, "Range stamp.nanosec");
        check(pdu.header.frame_id == "ultrasonic_frame", "Range frame_id");
    }

    std::printf("\n%d/%d checks passed\n", g_checks - g_fail, g_checks);
    std::printf("RESULT: %s\n", g_fail == 0 ? "PASS" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
