#include "sensors/lidar/lidar_scan_sensor.hpp"

#include <cmath>
#include <utility>

namespace hako::robots::sensor::lidar
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
}

LidarScanSensor::LidarScanSensor(std::shared_ptr<backend::IRayCaster> ray_caster)
    : ray_caster_(std::move(ray_caster))
{
}

void LidarScanSensor::SetConfig(const LidarScanConfig& config)
{
    config_ = config;
    scheduler_.Reset();
}

const LidarScanConfig& LidarScanSensor::GetConfig() const
{
    return config_;
}

void LidarScanSensor::Reset()
{
    scheduler_.Reset();
}

double LidarScanSensor::GetUpdatePeriodSec() const
{
    return (config_.scan_frequency_hz > 0)
        ? 1.0 / static_cast<double>(config_.scan_frequency_hz)
        : 0.0;
}

bool LidarScanSensor::ShouldUpdate(double delta_sec)
{
    return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
}

void LidarScanSensor::Scan(const backend::SensorState& state, LaserScanFrame& out)
{
    out.frame_id = config_.frame_id;
    out.angle_min = static_cast<float>(config_.angle_min_deg * kDeg2Rad);
    out.angle_max = static_cast<float>(config_.angle_max_deg * kDeg2Rad);
    out.angle_increment = static_cast<float>(config_.angle_increment_deg * kDeg2Rad);
    out.range_min = static_cast<float>(config_.range_min);
    out.range_max = static_cast<float>(config_.range_max);
    out.scan_time = static_cast<float>(GetUpdatePeriodSec());
    out.time_increment = 0.0F;
    out.ranges.clear();
    out.intensities.clear();

    if (ray_caster_ == nullptr || config_.angle_increment_deg <= 0.0) {
        return;
    }

    const auto& fwd = state.forward;
    const auto& left = state.left;

    for (double deg = config_.angle_min_deg; deg <= config_.angle_max_deg + 1.0e-9;
         deg += config_.angle_increment_deg) {
        const double th = deg * kDeg2Rad;
        const double c = std::cos(th);
        const double s = std::sin(th);

        const types::Vector3 dir(
            fwd.x * c + left.x * s,
            fwd.y * c + left.y * s,
            fwd.z * c + left.z * s);

        const backend::RayHit hit = ray_caster_->Cast(state.origin, dir, config_.range_max);

        if (hit.hit && hit.distance >= config_.range_min && hit.distance <= config_.range_max) {
            out.ranges.push_back(static_cast<float>(hit.distance));
            out.intensities.push_back(1.0F);
        } else {
            out.ranges.push_back(static_cast<float>(config_.range_max));
            out.intensities.push_back(0.0F);
        }
    }
}

}  // namespace hako::robots::sensor::lidar
