#pragma once

#include <limits>
#include <string>
#include <vector>

#include "sensor.hpp"
#include "sensors/noise/noise_config.hpp"

namespace hako::robots::sensor::radar
{
    // One radar return, polar layout identical to CARLA's RadarDetection.
    struct RadarDetection
    {
        float velocity {0.0F};  // m/s, +receding (Doppler)
        float azimuth {0.0F};   // rad, +left
        float altitude {0.0F};  // rad, +up (elevation)
        float depth {0.0F};     // m
    };

    struct RadarScanFrame
    {
        MessageHeader header {};
        std::vector<RadarDetection> detections {};
    };

    // Distance-accuracy rule, reused from the LiDAR/ultrasonic noise spec so a
    // single noise framework covers all range sensors.
    struct RadarDistanceAccuracy
    {
        double range_min {0.0};
        double range_max {0.0};
        bool distance_dependent {false};
        double percentage {0.0};
        double stddev {0.0};
        std::string noise_distribution {"Gaussian"};
        double precision {0.0};
    };

    // How the scan's rays are spread over the field of view. Selectable because
    // the "right" answer depends on what the radar is standing in for.
    enum class RayDistribution
    {
        UniformAngle,        // equal density per degree of az / el (default)
        UniformSolidAngle,   // equal density per steradian -- matters for wide elevation
        BoresightWeighted,   // centre-heavy, as the old planar-projection model was
    };

    struct RadarConfig
    {
        OutputBinding output {};
        std::string frame_id {"radar"};
        double range {30.0};                 // m
        // Symmetric field of view, centred on the boresight.
        double horizontal_fov_deg {30.0};
        double vertical_fov_deg {10.0};
        // Optional ASYMMETRIC window, in sensor-local degrees (azimuth positive
        // to the left, elevation positive up). When set it overrides the
        // symmetric FOV above, which lets a sensor look somewhere other than
        // straight ahead -- a rear-facing sector, or a downward-tilted slice --
        // without needing a mount rotation. NaN means "not specified".
        double azimuth_start_deg {std::numeric_limits<double>::quiet_NaN()};
        double azimuth_end_deg {std::numeric_limits<double>::quiet_NaN()};
        double elevation_start_deg {std::numeric_limits<double>::quiet_NaN()};
        double elevation_end_deg {std::numeric_limits<double>::quiet_NaN()};
        int points_per_second {1500};
        RayDistribution ray_distribution {RayDistribution::UniformAngle};
        // Distance-dependent detection, standing in for the radar equation.
        //
        // A geometric ray caster detects anything its ray touches, anywhere inside
        // `range`, so a 0.5 m airframe and a 3 m helicopter are picked up at the
        // same distance -- target size has no effect and every encounter saturates
        // at the configured range. A real radar does not work that way: received
        // power goes as sigma / R^4, so bigger targets are seen further away.
        //
        // The ray sampler already supplies one factor of (cross-section / R^2):
        // a distant or small target simply intercepts fewer rays. What is missing
        // is the OTHER 1/R^2, and that is what this is. With the default exponent
        // of 2 the expected number of returns falls as 1/R^4, which is the radar
        // equation's dependence.
        //
        // Disabled (<= 0) by default: existing manifests keep their behaviour.
        double detection_reference_range {0.0};   // m; full detection out to here
        double detection_falloff_exp {2.0};       // P = (ref/R)^exp beyond it

        // --- radar equation (optional; derives detection_reference_range) ------
        // Datasheet quantities. When tx_power_w, wavelength_m and
        // min_detectable_signal_w are all > 0 the loader computes
        // detection_reference_range from them (math::RadarEquationRange) instead
        // of taking it verbatim, so sensitivity is expressed in physical terms
        // rather than as a tuned distance.
        double tx_power_w {0.0};                  // Pt
        double antenna_gain_dbi {0.0};            // G, dBi (0 dBi = isotropic)
        double wavelength_m {0.0};                // lambda (77 GHz -> 0.0039 m)
        double min_detectable_signal_w {0.0};     // Smin
        // The RCS the link budget is quoted against. Also the baseline that a
        // per-target RCS is scaled from (math::ScaleRangeByRcs).
        double reference_rcs_m2 {1.0};            // sigma_ref
        unsigned int noise_seed {1U};
        std::vector<RadarDistanceAccuracy> distance_accuracy {};
    };
}
