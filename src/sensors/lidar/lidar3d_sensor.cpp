#include "sensors/lidar/lidar3d_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>

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
    if (config.channels <= 0 || config.rotations_per_second <= 0 || config.points_per_second <= 0 ||
        !std::isfinite(config.reflectivity) || config.reflectivity < 0 || config.reflectivity > 1 ||
        !std::isfinite(config.intensity_reference_distance) || config.intensity_reference_distance <= 0)
        throw std::invalid_argument("invalid lidar schedule/intensity config");
    const int columns = std::max(1, config.points_per_second / config.rotations_per_second / config.channels);
    const double column_period = 1.0 / config.rotations_per_second / columns;
    if ((!config.channel_elevation_deg.empty() && config.channel_elevation_deg.size() != static_cast<size_t>(config.channels)) ||
        (!config.channel_firing_offset_sec.empty() && config.channel_firing_offset_sec.size() != static_cast<size_t>(config.channels)))
        throw std::invalid_argument("lidar channel table size mismatch");
    for (double x : config.channel_elevation_deg)
        if (!std::isfinite(x) || x < -90 || x > 90) throw std::invalid_argument("invalid elevation");
    for (double x : config.channel_firing_offset_sec)
        if (!std::isfinite(x) || x < 0 || x >= column_period) throw std::invalid_argument("invalid firing offset");
    config_ = config;
    scan_count_ = 0;
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
    scan_count_ = 0;
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
    ScanAt(state, static_cast<double>(++scan_count_) * GetUpdatePeriodSec(), out);
}

void Lidar3DSensor::ScanAt(const backend::SensorState& state, double scan_end_sec, Lidar3DFrame& out)
{
    if (!std::isfinite(scan_end_sec) || scan_end_sec < 0) throw std::invalid_argument("invalid scan time");
    const int n_v = Height();
    const int n_h = Width();
    out.frame_id = config_.frame_id;
    out.stamp_sec = scan_end_sec;
    out.height = static_cast<std::uint32_t>(n_v);
    out.width = static_cast<std::uint32_t>(n_h);
    out.points.clear();
    out.point_time_offset_sec.clear();
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
        const double pitch_deg = !config_.channel_elevation_deg.empty() ? config_.channel_elevation_deg[iv] : (n_v == 1)
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
                if (config_.empirical_intensity) {
                    // Scalar inverse-square return. Unknown surface normal means
                    // normal incidence; no claim of material/RTX equivalence.
                    const double norm = std::sqrt(hit.normal.x*hit.normal.x + hit.normal.y*hit.normal.y + hit.normal.z*hit.normal.z);
                    const double incidence = norm > 0 ? std::clamp(-(dir.x*hit.normal.x + dir.y*hit.normal.y + dir.z*hit.normal.z)/norm, 0.0, 1.0) : 1.0;
                    const double ratio = config_.intensity_reference_distance / std::max(depth, 1e-9);
                    intensity = static_cast<float>(std::clamp(config_.reflectivity * incidence * ratio * ratio, 0.0, 1.0));
                }
            }
            // sensor-local cartesian (REP-103: x fwd, y left, z up)
            p.x = static_cast<float>(depth * ce * ca);
            p.y = static_cast<float>(depth * ce * sa);
            p.z = static_cast<float>(depth * se);
            p.intensity = intensity;
            out.points.push_back(p);
            const double column_period = GetUpdatePeriodSec() / n_h;
            const double offset = config_.channel_firing_offset_sec.empty()
                ? column_period * iv / n_v : config_.channel_firing_offset_sec[iv];
            out.point_time_offset_sec.push_back(ih * column_period + offset);
        }
    }
}

}  // namespace hako::robots::sensor::lidar
