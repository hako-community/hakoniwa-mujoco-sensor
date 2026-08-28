#pragma once

// Manifest params (JSON) -> RadarConfig, in ONE place.
//
// Why this file exists (2026-08-28, R4): the mapping used to be written out by
// hand in every consumer -- SensorFactory here, and again in
// hakoniwa-mujoco-drone (`include/sensors/radar_params.hpp`). The copies drifted
// exactly as you would expect: the drone-side one silently dropped the radar
// equation (tx_power_w and friends), which zeroes detection_reference_range and
// disables the distance falloff altogether, and NEITHER copy read
// `distance_accuracy`, so every manifest-driven radar reported a NOISELESS
// range. A noiseless range is not a neutral default: the DAA study compares
// "closure straight from Doppler" against "closure from differencing range", and
// the second one is exactly what range noise degrades.
//
// Keeping the mapping here (pure: nlohmann + radar types, no PDU, no MuJoCo)
// means a new key is added once and every consumer gets it.

#include <cmath>
#include <string>

#include <nlohmann/json.hpp>

#include "sensors/radar/radar_math.hpp"
#include "sensors/radar/radar_types.hpp"

namespace hako::robots::sensor::radar
{
    // `params` is the manifest's per-sensor "params" object. Anything absent
    // keeps the value already in `c`, so callers can pre-seed defaults.
    inline RadarConfig RadarConfigFromJson(const nlohmann::json& p, RadarConfig c = RadarConfig{})
    {
        c.frame_id = p.value("frame_id", c.frame_id);
        c.range = p.value("range", c.range);
        c.horizontal_fov_deg = p.value("horizontal_fov_deg", c.horizontal_fov_deg);
        c.vertical_fov_deg = p.value("vertical_fov_deg", c.vertical_fov_deg);
        // Optional ASYMMETRIC window (e.g. a rear sector: 150 .. 210).
        // Absent keys leave NaN, which means "use the symmetric FOV".
        c.azimuth_start_deg = p.value("azimuth_start_deg", c.azimuth_start_deg);
        c.azimuth_end_deg = p.value("azimuth_end_deg", c.azimuth_end_deg);
        c.elevation_start_deg = p.value("elevation_start_deg", c.elevation_start_deg);
        c.elevation_end_deg = p.value("elevation_end_deg", c.elevation_end_deg);
        c.points_per_second = p.value("points_per_second", c.points_per_second);
        c.noise_seed = p.value("noise_seed", c.noise_seed);
        c.output.update_rate_hz = p.value("update_rate_hz", c.output.update_rate_hz);
        // Distance-dependent detection. Keep detection_falloff_exp at 2: the ray
        // sampler already supplies the other factor of 1/R^2.
        c.detection_reference_range =
            p.value("detection_reference_range", c.detection_reference_range);
        c.detection_falloff_exp = p.value("detection_falloff_exp", c.detection_falloff_exp);
        // Radar equation. With a complete link budget the reference range is
        // DERIVED, so sensitivity is expressed physically rather than tuned.
        c.tx_power_w = p.value("tx_power_w", c.tx_power_w);
        c.antenna_gain_dbi = p.value("antenna_gain_dbi", c.antenna_gain_dbi);
        c.wavelength_m = p.value("wavelength_m", c.wavelength_m);
        c.min_detectable_signal_w = p.value("min_detectable_signal_w", c.min_detectable_signal_w);
        c.reference_rcs_m2 = p.value("reference_rcs_m2", c.reference_rcs_m2);
        {
            const double derived = math::RadarEquationRange(
                c.tx_power_w, c.antenna_gain_dbi, c.wavelength_m,
                c.reference_rcs_m2, c.min_detectable_signal_w);
            if (derived > 0.0) c.detection_reference_range = derived;
        }
        {
            // Unknown names fall back to the default rather than failing the load:
            // a manifest written for a newer build still runs here.
            const std::string dist = p.value("ray_distribution", std::string("uniform_angle"));
            if (dist == "uniform_solid_angle") {
                c.ray_distribution = RayDistribution::UniformSolidAngle;
            } else if (dist == "boresight_weighted") {
                c.ray_distribution = RayDistribution::BoresightWeighted;
            } else {
                c.ray_distribution = RayDistribution::UniformAngle;
            }
        }
        // Range noise. Same semantics as the `spec` form's DistanceAccuracy,
        // spelled in the manifest's snake_case.
        if (p.contains("distance_accuracy") && p.at("distance_accuracy").is_array()) {
            c.distance_accuracy.clear();
            for (const auto& e : p.at("distance_accuracy")) {
                RadarDistanceAccuracy acc {};
                acc.range_min = 0.0;
                acc.range_max = c.range;
                if (e.contains("range") && e.at("range").is_object()) {
                    acc.range_min = e.at("range").value("min", 0.0);
                    acc.range_max = e.at("range").value("max", c.range);
                }
                const std::string type = e.value("type", std::string("independent"));
                acc.distance_dependent = (type == "dependent");
                acc.stddev = e.value("stddev", 0.0);
                acc.percentage = e.value("percentage", 0.0);
                acc.precision = e.value("precision", 0.0);
                acc.noise_distribution = e.value("noise_distribution", std::string("Gaussian"));
                c.distance_accuracy.push_back(std::move(acc));
            }
        }
        return c;
    }
}
