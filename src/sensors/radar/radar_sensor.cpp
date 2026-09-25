#include "sensors/radar/radar_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>

#include "config/json_config_utils.hpp"
#include "common/json_utils.hpp"
#include "sensors/radar/radar_math.hpp"
#include "sensors/radar/radar_config_json.hpp"

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
    for (double v : {config.detection_probability_scale, config.false_alarm_probability})
        if (!std::isfinite(v) || v < 0 || v > 1) throw std::invalid_argument("invalid radar probability");
    for (double v : {config.clutter_velocity_stddev, config.doppler_stddev, config.doppler_resolution, config.range_resolution})
        if (!std::isfinite(v) || v < 0) throw std::invalid_argument("invalid radar resolution/noise");
    config_ = config;
    scan_count_ = 0;
    effects_rng_.seed(config_.noise_seed ^ 0x9e3779b9U);
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

    cfg = RadarConfigFromJson(s, cfg);
    SetConfig(cfg);
    return true;
}

void RadarSensor::RebuildNoisePipeline()
{
    auto noise_model = std::make_unique<noise::GaussianNoiseModel>();
    noise_model->Reseed(config_.noise_seed ^ 0x85ebca6bU);
    noise_pipeline_ = noise::RangeNoisePipeline(std::move(noise_model));
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
    scan_count_ = 0;
    effects_rng_.seed(config_.noise_seed ^ 0x9e3779b9U);
    rng_.seed(config_.noise_seed);
    RebuildNoisePipeline();
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
    out.target_trials = out.target_detections = out.empty_trials = out.false_alarms = 0;
    out.doppler_squared_error_sum = 0;
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
            ++out.empty_trials;
            if (config_.false_alarm_probability > 0 && uni01(effects_rng_) < config_.false_alarm_probability) {
                RadarDetection clutter {};
                double az, el, unused;
                math::ToPolar(local_dir, az, el, unused);
                clutter.azimuth = static_cast<float>(az);
                clutter.altitude = static_cast<float>(el);
                double range = config_.range * uni01(effects_rng_);
                if (config_.range_resolution > 0) range = std::round(range / config_.range_resolution) * config_.range_resolution;
                clutter.depth = static_cast<float>(std::clamp(range, 0.0, config_.range));
                double velocity = 0;
                if (config_.clutter_velocity_stddev > 0) velocity = std::normal_distribution<double>(0, config_.clutter_velocity_stddev)(effects_rng_);
                if (config_.doppler_resolution > 0) velocity = std::round(velocity / config_.doppler_resolution) * config_.doppler_resolution;
                clutter.velocity = static_cast<float>(velocity);
                out.detections.push_back(clutter);
                ++out.false_alarms;
            }
            continue;
        }
        ++out.target_trials;

        const types::Vector3 rel = hit.point - state.origin;
        const types::Vector3 local_hit = math::WorldToLocal(rel, state.forward, state.left, state.up);

        double azimuth = 0.0;
        double elevation = 0.0;
        double depth = 0.0;
        math::ToPolar(local_hit, azimuth, elevation, depth);

        // Distance-dependent detection. Applied to the TRUE range, before noise,
        // and drawn from the same seeded generator so runs stay reproducible.
        //
        // A target with its own RCS shifts the reference range by (sigma/sigma_ref)^(1/4),
        // straight out of the radar equation: a more reflective surface stays
        // detectable further out. Targets without an RCS keep the reference range,
        // so scenes that do not annotate their geometry behave exactly as before.
        double ref_range = config_.detection_reference_range;
        if (hit.target_rcs_m2 > 0.0) {
            ref_range = math::ScaleRangeByRcs(ref_range, hit.target_rcs_m2,
                                              config_.reference_rcs_m2);
        }
        if (config_.detection_probability_scale * math::DetectionProbability(depth, ref_range,
                                       config_.detection_falloff_exp) < uni01(rng_)) {
            continue;
        }

        double depth_noisy = noise_pipeline_.Apply(depth);
        const double velocity = math::RelativeVelocity(hit.target_velocity, state.linear_velocity, world_dir);
        double measured_velocity = velocity;
        if (config_.doppler_stddev > 0) measured_velocity += std::normal_distribution<double>(0, config_.doppler_stddev)(effects_rng_);
        if (config_.doppler_resolution > 0) measured_velocity = std::round(measured_velocity / config_.doppler_resolution) * config_.doppler_resolution;
        if (config_.range_resolution > 0) depth_noisy = std::clamp(std::round(depth_noisy / config_.range_resolution) * config_.range_resolution, 0.0, config_.range);
        ++out.target_detections;
        out.doppler_squared_error_sum += (measured_velocity - velocity) * (measured_velocity - velocity);

        RadarDetection d {};
        d.velocity = static_cast<float>(measured_velocity);
        d.azimuth = static_cast<float>(azimuth);
        d.altitude = static_cast<float>(elevation);
        d.depth = static_cast<float>(depth_noisy);
        out.detections.push_back(d);
    }
}
}  // namespace hako::robots::sensor::radar
