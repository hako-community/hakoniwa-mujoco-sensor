#include "sensors/lidar/lidar3d_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace hako::robots::sensor::lidar
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
}

Lidar3DSensor::Lidar3DSensor(std::shared_ptr<backend::IRayCaster> ray_caster)
    : ray_caster_(std::move(ray_caster))
{
}

void Lidar3DSensor::SetConfig(const Lidar3DConfig& config)
{
    config_ = config;
    scheduler_.Reset();
}

const Lidar3DConfig& Lidar3DSensor::GetConfig() const
{
    return config_;
}

int Lidar3DSensor::Height() const
{
    return std::max(1, config_.channels);
}

int Lidar3DSensor::Width() const
{
    // Points per rotation per channel (mirrors Godot Default3DLiDARController).
    if (config_.rotations_per_second <= 0 || config_.channels <= 0) {
        return 0;
    }
    const int points_per_rotation = config_.points_per_second / config_.rotations_per_second;
    return std::max(1, points_per_rotation / config_.channels);
}

void Lidar3DSensor::Reset()
{
    scheduler_.Reset();
}

double Lidar3DSensor::GetUpdatePeriodSec() const
{
    return (config_.rotations_per_second > 0)
        ? 1.0 / static_cast<double>(config_.rotations_per_second)
        : 0.0;
}

bool Lidar3DSensor::ShouldUpdate(double delta_sec)
{
    return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
}

void Lidar3DSensor::Scan(const backend::SensorState& state, Lidar3DFrame& out)
{
    const int n_v = Height();
    const int n_h = Width();
    out.frame_id = config_.frame_id;
    out.stamp_sec = static_cast<double>(++scan_count_) * GetUpdatePeriodSec();
    out.height = static_cast<std::uint32_t>(n_v);
    out.width = static_cast<std::uint32_t>(n_h);
    out.points.clear();
    if (ray_caster_ == nullptr || n_h <= 0) {
        return;
    }
    out.points.reserve(static_cast<size_t>(n_v) * static_cast<size_t>(n_h));

    const double v_lo = config_.vertical_fov_lower_deg;
    const double v_hi = config_.vertical_fov_upper_deg;
    const double h_lo = config_.horizontal_fov_start_deg;
    const double h_hi = config_.horizontal_fov_end_deg;

    const auto& fwd = state.forward;
    const auto& left = state.left;
    const auto& up = state.up;

    for (int iv = 0; iv < n_v; ++iv) {
        const double pitch_deg = (n_v == 1)
            ? 0.5 * (v_lo + v_hi)
            : v_lo + (v_hi - v_lo) * static_cast<double>(iv) / static_cast<double>(n_v - 1);
        const double elev = pitch_deg * kDeg2Rad;
        const double ce = std::cos(elev);
        const double se = std::sin(elev);

        for (int ih = 0; ih < n_h; ++ih) {
            const double yaw_deg = (n_h == 1)
                ? 0.5 * (h_lo + h_hi)
                : h_lo + (h_hi - h_lo) * static_cast<double>(ih) / static_cast<double>(n_h - 1);
            const double az = yaw_deg * kDeg2Rad;
            const double ca = std::cos(az);
            const double sa = std::sin(az);

            // world direction = forward*ce*ca + left*ce*sa + up*se
            const types::Vector3 dir(
                fwd.x * ce * ca + left.x * ce * sa + up.x * se,
                fwd.y * ce * ca + left.y * ce * sa + up.y * se,
                fwd.z * ce * ca + left.z * ce * sa + up.z * se);

            const backend::RayHit hit = ray_caster_->Cast(state.origin, dir, config_.max_distance);

            PointXYZI p {};
            double depth = config_.max_distance;
            float intensity = 0.0F;
            if (hit.hit && hit.distance >= config_.min_distance && hit.distance <= config_.max_distance) {
                depth = hit.distance;
                intensity = 1.0F;
            }
            // sensor-local cartesian (REP-103: x fwd, y left, z up)
            p.x = static_cast<float>(depth * ce * ca);
            p.y = static_cast<float>(depth * ce * sa);
            p.z = static_cast<float>(depth * se);
            p.intensity = intensity;
            out.points.push_back(p);
        }
    }
}

}  // namespace hako::robots::sensor::lidar
