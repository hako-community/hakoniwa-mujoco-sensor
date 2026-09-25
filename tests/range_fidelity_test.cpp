#include <cmath>
#include <iostream>
#include <stdexcept>
#include "sensors/lidar/lidar3d_sensor.hpp"
#include "sensors/radar/radar_sensor.hpp"
#include "sensors/radar/radar_config_json.hpp"
#include "sensors/common/capability.hpp"
using namespace hako::robots;
namespace b = sensor::backend;
struct Scene : b::IRayCaster {
    bool present = true;
    double distance = 2;
    b::RayHit Cast(const types::Vector3& origin, const types::Vector3& dir, double) override {
        b::RayHit h;
        h.hit = present; h.distance = distance;
        h.point = types::Vector3(origin.x + distance*dir.x, origin.y + distance*dir.y, origin.z + distance*dir.z);
        h.normal = types::Vector3(-dir.x, -dir.y, -dir.z);
        h.target_velocity = types::Vector3(1.23, 0, 0);
        return h;
    }
};
void require(bool ok) { if (!ok) throw std::runtime_error("range fidelity check failed"); }
int main() {
    auto required = nlohmann::json::parse(R"({"radar_multipath":true})");
    bool capability_rejected = false;
    try { sensor::capability::Require(sensor::capability::Geometric("radar"), required); }
    catch (const std::invalid_argument&) { capability_rejected = true; }
    require(capability_rejected);
    required = nlohmann::json::parse(R"({"doppler":true})");
    sensor::capability::Require(sensor::capability::Geometric("radar"), required);
    auto scene = std::make_shared<Scene>();
    b::SensorState state;
    state.forward = types::Vector3(1,0,0); state.left = types::Vector3(0,1,0); state.up = types::Vector3(0,0,1);
    sensor::lidar::Lidar3DSensor lidar(scene);
    sensor::lidar::Lidar3DConfig lc;
    lc.channels = 2; lc.points_per_second = 80; lc.rotations_per_second = 10;
    lc.channel_elevation_deg = {0, 10}; lc.channel_firing_offset_sec = {0, 0.002};
    lc.empirical_intensity = true; lc.reflectivity = 0.8;
    lidar.SetConfig(lc);
    sensor::lidar::Lidar3DFrame cloud;
    lidar.ScanAt(state, 2.0, cloud);
    require(cloud.points.size() == 8 && cloud.point_time_offset_sec.size() == 8);
    require(std::abs(cloud.point_time_offset_sec[5] - 0.027) < 1e-12);
    require(std::abs(cloud.points[0].intensity - 0.2) < 1e-6);
    scene->distance = 4; lidar.Scan(state, cloud);
    require(std::abs(cloud.points[0].intensity - 0.05) < 1e-6);
    scene->present = false; lidar.Scan(state, cloud); require(cloud.points[0].intensity == 0);
    lidar.Reset(); lidar.Scan(state, cloud); require(cloud.stamp_sec == 0.1);
    bool rejected = false;
    lc.channel_firing_offset_sec[0] = 1;
    try { lidar.SetConfig(lc); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);

    sensor::radar::RadarSensor radar(scene);
    const auto params = nlohmann::json::parse(R"({"points_per_second":100000,
        "update_rate_hz":10,"horizontal_fov_deg":0,"vertical_fov_deg":0,
        "false_alarm_probability":0.2,"clutter_velocity_stddev":0.5,
        "detection_probability_scale":0.7,"doppler_resolution":0.1})");
    sensor::radar::RadarConfig defaults;
    auto rc = sensor::radar::RadarConfigFromJson(params, defaults);
    radar.SetConfig(rc);
    sensor::radar::RadarScanFrame scan;
    radar.Scan(state, scan);
    require(scan.empty_trials == 10000 && scan.target_trials == 0);
    double pfa = double(scan.false_alarms)/scan.empty_trials;
    require(std::abs(pfa - 0.2) < 0.02);
    auto first = scan;
    radar.Reset(); radar.Scan(state, scan);
    require(first.detections.size() == scan.detections.size() && first.header.stamp_sec == scan.header.stamp_sec);
    for (size_t i = 0; i < scan.detections.size(); ++i) {
        require(first.detections[i].depth == scan.detections[i].depth);
        require(first.detections[i].velocity == scan.detections[i].velocity);
    }
    scene->present = true;
    radar.Scan(state, scan);
    double pd = double(scan.target_detections)/scan.target_trials;
    require(std::abs(pd - 0.7) < 0.02 && scan.empty_trials == 0);
    double rmse = std::sqrt(scan.doppler_squared_error_sum/scan.target_detections);
    require(std::abs(rmse - 0.03) < 1e-6);
    rc.detection_probability_scale = 0; radar.SetConfig(rc); radar.Scan(state, scan);
    require(scan.target_detections == 0);
    rc.false_alarm_probability = 2; rejected = false;
    try { radar.SetConfig(rc); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
    std::cout << "Pd=" << pd << " Pfa=" << pfa << " Doppler_RMSE=" << rmse << " PASS\n";
}
