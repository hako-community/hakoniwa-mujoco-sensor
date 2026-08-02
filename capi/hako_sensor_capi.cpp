// C-ABI wrapper over the backend-agnostic sensor models. See hako_sensor_capi.h.
//
// Nothing here touches the sensor model source: it only constructs the existing
// Lidar3DSensor / RadarSensor, injects a CApiRayCaster that forwards to the C
// callback, and marshals POD in/out. The result is libhako_mujoco_sensor_capi.so.

#include "hako_sensor_capi.h"

#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "sensors/backend/ray_caster.hpp"
#include "sensors/lidar/lidar3d_sensor.hpp"
#include "sensors/radar/radar_sensor.hpp"

namespace {

namespace be = hako::robots::sensor::backend;
namespace lidar = hako::robots::sensor::lidar;
namespace radar = hako::robots::sensor::radar;
using hako::robots::types::Vector3;

// Adapts the injected C ray-cast callback to the sensor model's IRayCaster.
// mujoco-sensor therefore never learns who actually performs the cast.
class CApiRayCaster : public be::IRayCaster {
public:
    CApiRayCaster(hako_raycast_fn fn, void* user) : fn_(fn), user_(user) {}

    be::RayHit Cast(const Vector3& origin, const Vector3& dir, double max_distance) override {
        be::RayHit hit{};
        if (fn_ == nullptr) return hit;
        const double o[3] = {origin.x, origin.y, origin.z};
        const double d[3] = {dir.x, dir.y, dir.z};
        double dist = 0.0, p[3] = {0, 0, 0}, n[3] = {0, 0, 0}, tv[3] = {0, 0, 0};
        int tid = -1;
        if (fn_(user_, o, d, max_distance, &dist, p, n, tv, &tid) != 0) {
            hit.hit = true;
            hit.distance = dist;
            hit.point = Vector3(p[0], p[1], p[2]);
            hit.normal = Vector3(n[0], n[1], n[2]);
            hit.target_velocity = Vector3(tv[0], tv[1], tv[2]);
            hit.target_id = tid;
        }
        return hit;
    }

private:
    hako_raycast_fn fn_;
    void* user_;
};

enum class Kind { Lidar3D, Radar };

be::SensorState MakeState(const double s[15]) {
    be::SensorState st{};
    st.origin          = Vector3(s[0], s[1], s[2]);
    st.forward         = Vector3(s[3], s[4], s[5]);
    st.left            = Vector3(s[6], s[7], s[8]);
    st.up              = Vector3(s[9], s[10], s[11]);
    st.linear_velocity = Vector3(s[12], s[13], s[14]);
    return st;
}

}  // namespace

struct hako_sensor_ctx {
    Kind kind{Kind::Lidar3D};
    std::shared_ptr<be::IRayCaster> caster;
    std::unique_ptr<lidar::Lidar3DSensor> lidar_sensor;
    std::unique_ptr<radar::RadarSensor> radar_sensor;
    std::string err;
};

hako_sensor_handle hako_sensor_create(
    const char* type, const char* config_json,
    hako_raycast_fn raycast, void* raycast_user) {
    if (type == nullptr || raycast == nullptr) return nullptr;

    auto* ctx = new (std::nothrow) hako_sensor_ctx();
    if (ctx == nullptr) return nullptr;
    ctx->caster = std::make_shared<CApiRayCaster>(raycast, raycast_user);

    try {
        nlohmann::json p = nlohmann::json::object();
        if (config_json != nullptr && config_json[0] != '\0') {
            p = nlohmann::json::parse(config_json);
        }
        const std::string t = type;
        if (t == "lidar3d" || t == "lidar_3d") {
            ctx->kind = Kind::Lidar3D;
            lidar::Lidar3DConfig c{};
            c.frame_id                = p.value("frame_id", c.frame_id);
            c.channels                = p.value("channels", c.channels);
            c.rotations_per_second    = p.value("rotations_per_second", c.rotations_per_second);
            c.points_per_second       = p.value("points_per_second", c.points_per_second);
            c.max_distance            = p.value("max_distance", c.max_distance);
            c.min_distance            = p.value("min_distance", c.min_distance);
            c.vertical_fov_upper_deg  = p.value("vertical_fov_upper_deg", c.vertical_fov_upper_deg);
            c.vertical_fov_lower_deg  = p.value("vertical_fov_lower_deg", c.vertical_fov_lower_deg);
            c.horizontal_fov_start_deg= p.value("horizontal_fov_start_deg", c.horizontal_fov_start_deg);
            c.horizontal_fov_end_deg  = p.value("horizontal_fov_end_deg", c.horizontal_fov_end_deg);
            ctx->lidar_sensor = std::make_unique<lidar::Lidar3DSensor>(ctx->caster);
            ctx->lidar_sensor->SetConfig(c);
        } else if (t == "radar") {
            ctx->kind = Kind::Radar;
            radar::RadarConfig c{};
            c.frame_id           = p.value("frame_id", c.frame_id);
            c.range              = p.value("range", c.range);
            c.horizontal_fov_deg = p.value("horizontal_fov_deg", c.horizontal_fov_deg);
            c.vertical_fov_deg   = p.value("vertical_fov_deg", c.vertical_fov_deg);
            c.points_per_second  = p.value("points_per_second", c.points_per_second);
            c.noise_seed         = p.value("noise_seed", c.noise_seed);
            ctx->radar_sensor = std::make_unique<radar::RadarSensor>(ctx->caster);
            ctx->radar_sensor->SetConfig(c);
        } else {
            delete ctx;
            return nullptr;
        }
    } catch (const std::exception& e) {
        // config parse failed: keep the handle so the caller can read the error,
        // but it has no sensor -> scan will report the same error.
        ctx->err = std::string("config error: ") + e.what();
        return ctx;
    }
    return ctx;
}

int hako_sensor_scan(hako_sensor_handle h,
    const double state[15], double dt_sec,
    float* out_points_xyzi, int max_points,
    int* out_height, int* out_width, int* out_count) {
    (void)dt_sec;  // direct API: the caller controls cadence, always scan.
    if (h == nullptr) return 1;
    if (state == nullptr || out_points_xyzi == nullptr || max_points < 0) {
        h->err = "invalid arguments";
        return 2;
    }
    if (out_height) *out_height = 0;
    if (out_width)  *out_width = 0;
    if (out_count)  *out_count = 0;

    try {
        const be::SensorState st = MakeState(state);
        if (h->kind == Kind::Lidar3D) {
            if (!h->lidar_sensor) { h->err = "sensor not created"; return 3; }
            lidar::Lidar3DFrame frame{};
            h->lidar_sensor->Scan(st, frame);
            const int total = static_cast<int>(frame.points.size());
            const int n = total < max_points ? total : max_points;
            for (int i = 0; i < n; ++i) {
                const auto& pt = frame.points[static_cast<size_t>(i)];
                float* o = out_points_xyzi + static_cast<size_t>(i) * 4;
                o[0] = pt.x; o[1] = pt.y; o[2] = pt.z; o[3] = pt.intensity;
            }
            if (out_height) *out_height = static_cast<int>(frame.height);
            if (out_width)  *out_width = static_cast<int>(frame.width);
            if (out_count)  *out_count = n;
        } else {
            if (!h->radar_sensor) { h->err = "sensor not created"; return 3; }
            radar::RadarScanFrame frame{};
            h->radar_sensor->Scan(st, frame);
            const int total = static_cast<int>(frame.detections.size());
            const int n = total < max_points ? total : max_points;
            for (int i = 0; i < n; ++i) {
                const auto& d = frame.detections[static_cast<size_t>(i)];
                // polar -> sensor-local cartesian (x fwd, y left, z up); w = Doppler.
                const float cz = std::cos(d.altitude);
                float* o = out_points_xyzi + static_cast<size_t>(i) * 4;
                o[0] = d.depth * cz * std::cos(d.azimuth);
                o[1] = d.depth * cz * std::sin(d.azimuth);
                o[2] = d.depth * std::sin(d.altitude);
                o[3] = d.velocity;
            }
            if (out_height) *out_height = 1;
            if (out_width)  *out_width = total;
            if (out_count)  *out_count = n;
        }
    } catch (const std::exception& e) {
        h->err = std::string("scan error: ") + e.what();
        return 4;
    }
    h->err.clear();
    return 0;
}

void hako_sensor_reset(hako_sensor_handle h) {
    if (h == nullptr) return;
    if (h->lidar_sensor) h->lidar_sensor->Reset();
    if (h->radar_sensor) h->radar_sensor->Reset();
    h->err.clear();
}

void hako_sensor_destroy(hako_sensor_handle h) { delete h; }

const char* hako_sensor_last_error(hako_sensor_handle h) {
    return (h == nullptr) ? "" : h->err.c_str();
}
