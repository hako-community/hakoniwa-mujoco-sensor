#include "sensors/radar/radar_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "config/json_config_utils.hpp"
#include "common/json_utils.hpp"
#include "sensors/radar/radar_math.hpp"

namespace hako::robots::sensor::radar
{
namespace
{
noise::NoiseType parse_noise_type(const std::string& value)
{
    if (value == "none" || value == "None") {
        return noise::NoiseType::None;
    }
    if (value == "gaussian_quantized" || value == "GaussianQuantized" ||
        value == "gaussian-quantized" || value == "Gaussian-Quantized") {
        return noise::NoiseType::GaussianQuantized;
    }
    return noise::NoiseType::Gaussian;
}
}  // namespace

RadarSensor::RadarSensor(std::shared_ptr<backend::IRayCaster> ray_caster)
    : ray_caster_(std::move(ray_caster))
{
    RebuildNoisePipeline();
    scheduler_.StartReady(GetUpdatePeriodSec());
}

void RadarSensor::SetConfig(const RadarConfig& config)
{
    config_ = config;
    rng_.seed(config_.noise_seed);
    RebuildNoisePipeline();
    scheduler_.StartReady(GetUpdatePeriodSec());
}

const RadarConfig& RadarSensor::GetConfig() const
{
    return config_;
}

bool RadarSensor::LoadConfig(const std::string& config_path)
{
    common::json root;
    if (!common::load_json_file(config_path, root)) {
        return false;
    }

    RadarConfig cfg {};
    const auto* spec = hako::robots::config::FindObject(root, "spec");
    const auto& s = (spec != nullptr) ? *spec : root;

    cfg.output.name = common::get_json_string(s, "name", "radar");
    cfg.output.pdu_name = "radar_scan";
    cfg.output.update_rate_hz = 10.0;
    cfg.frame_id = common::get_json_string(s, "frame_id", "radar");
    cfg.range = common::get_json_number(s, "Range", cfg.range);
    cfg.horizontal_fov_deg = common::get_json_number(s, "HorizontalFOV", cfg.horizontal_fov_deg);
    cfg.vertical_fov_deg = common::get_json_number(s, "VerticalFOV", cfg.vertical_fov_deg);
    cfg.points_per_second = common::get_json_int(s, "PointsPerSecond", cfg.points_per_second);
    cfg.noise_seed = static_cast<unsigned int>(common::get_json_int(s, "NoiseSeed", static_cast<int>(cfg.noise_seed)));

    if (s.contains("DistanceAccuracy") && s.at("DistanceAccuracy").is_array()) {
        for (const auto& entry : s.at("DistanceAccuracy")) {
            RadarDistanceAccuracy acc {};
            if (entry.contains("Range") && entry.at("Range").is_object()) {
                const auto& r = entry.at("Range");
                acc.range_min = common::get_json_number(r, "Min", 0.0);
                acc.range_max = common::get_json_number(r, "Max", cfg.range);
            } else {
                acc.range_min = 0.0;
                acc.range_max = cfg.range;
            }
            const std::string type = entry.value("type", entry.value("Type", std::string("independent")));
            acc.distance_dependent = (type == "dependent");
            if (acc.distance_dependent && entry.contains("DistanceDependentAccuracy")) {
                const auto& dep = entry.at("DistanceDependentAccuracy");
                acc.percentage = common::get_json_number(dep, "Percentage", 0.0);
                acc.noise_distribution = dep.value("NoiseDistribution", std::string("Gaussian"));
                acc.precision = common::get_json_number(dep, "Precision", 0.0);
            } else if (entry.contains("DistanceIndependentAccuracy")) {
                const auto& indep = entry.at("DistanceIndependentAccuracy");
                acc.stddev = common::get_json_number(indep, "StdDev", 0.0);
                acc.noise_distribution = indep.value("NoiseDistribution", std::string("Gaussian"));
                acc.precision = common::get_json_number(indep, "Precision", 0.0);
            }
            cfg.distance_accuracy.push_back(std::move(acc));
        }
    }

    hako::robots::config::ReadPduConfig(root, cfg.output.pdu_name, cfg.output.update_rate_hz);

    SetConfig(cfg);
    return true;
}

void RadarSensor::RebuildNoisePipeline()
{
    noise_pipeline_.Clear();
    for (const auto& acc : config_.distance_accuracy) {
        noise::RangeNoiseRule rule {};
        rule.range.min = acc.range_min;
        rule.range.max = acc.range_max;
        rule.distance_dependent = acc.distance_dependent;
        rule.percentage = acc.percentage;
        rule.noise.stddev = acc.stddev;
        rule.noise.precision = acc.precision;
        rule.noise.type = parse_noise_type(acc.noise_distribution);
        noise_pipeline_.AddRule(rule);
    }
}

void RadarSensor::Reset()
{
    scheduler_.Reset();
    rng_.seed(config_.noise_seed);
}

double RadarSensor::GetUpdatePeriodSec() const
{
    const double hz = config_.output.update_rate_hz;
    return (hz > 0.0) ? (1.0 / hz) : 0.1;
}

bool RadarSensor::ShouldUpdate(double delta_sec)
{
    return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
}

int RadarSensor::PointsPerScan() const
{
    const double per_scan = static_cast<double>(config_.points_per_second) * GetUpdatePeriodSec();
    return std::max(1, static_cast<int>(std::lround(per_scan)));
}

void RadarSensor::Scan(const backend::SensorState& state, RadarScanFrame& out)
{
    out.detections.clear();
    out.header.frame_id = config_.frame_id;
    out.header.stamp_sec = static_cast<double>(++scan_count_) * GetUpdatePeriodSec();

    if (ray_caster_ == nullptr || config_.range <= 0.0) {
        return;
    }

    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    // Resolved once per scan: the symmetric FOV, or the explicit asymmetric
    // window if the manifest supplied one.
    const math::AngularWindow window = math::WindowOf(config_);

    const int n = PointsPerScan();
    out.detections.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
        // Two independent uniforms -> one direction on the sphere. See
        // RayDirLocal: azimuth and elevation are each spread evenly over the FOV.
        const double u_az = uni01(rng_);
        const double u_el = uni01(rng_);

        const types::Vector3 local_dir =
            math::RayDirWindow(u_az, u_el, window, config_.ray_distribution);
        const types::Vector3 world_dir =
            math::LocalToWorld(local_dir, state.forward, state.left, state.up);

        const backend::RayHit hit = ray_caster_->Cast(state.origin, world_dir, config_.range);
        if (!hit.hit) {
            continue;
        }

        const types::Vector3 rel = hit.point - state.origin;
        const types::Vector3 local_hit = math::WorldToLocal(rel, state.forward, state.left, state.up);

        double azimuth = 0.0;
        double elevation = 0.0;
        double depth = 0.0;
        math::ToPolar(local_hit, azimuth, elevation, depth);

        // Distance-dependent detection. Applied to the TRUE range, before noise,
        // and drawn from the same seeded generator so runs stay reproducible.
        if (math::DetectionProbability(depth, config_.detection_reference_range,
                                       config_.detection_falloff_exp) < uni01(rng_)) {
            continue;
        }

        const double depth_noisy = noise_pipeline_.Apply(depth);
        const double velocity = math::RelativeVelocity(hit.target_velocity, state.linear_velocity, world_dir);

        RadarDetection d {};
        d.velocity = static_cast<float>(velocity);
        d.azimuth = static_cast<float>(azimuth);
        d.altitude = static_cast<float>(elevation);
        d.depth = static_cast<float>(depth_noisy);
        out.detections.push_back(d);
    }
}
}  // namespace hako::robots::sensor::radar
