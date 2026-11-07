#pragma once

// RealSense IMU (Bosch BMI055 / BMI085) measurement model. Pure: no MuJoCo.
//
// measurement = saturate( quantize( truth + bias + white_noise ) )
//   white noise : sigma = noise_density * sqrt(rate)   (discrete-time, per sample)
//   bias        : constant per run, drawn from the zero-offset spec
//                 ("uncalibrated": uniform +-offset, "calibrated": 10% of it,
//                  "none": 0) + optional random walk (gyro bias instability)
//   quantize    : full-scale range / 2^(bits-1) LSB
// Gyro in rad/s, accel in m/s^2 (specific force: +9.81 m/s^2 "up" at rest).
// Output frame = camera optical frame (x right, y down, z forward), so at rest
// with the camera level the accelerometer reads y = -9.81 m/s^2, as the D435i does.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>

#include "sensors/camera/realsense/realsense_profile.hpp"

namespace hako::robots::sensor::camera::realsense
{
    constexpr double kStandardGravity = 9.80665;

    struct ImuModelConfig
    {
        bool enabled {true};
        double accel_rate_hz {250.0};
        double gyro_rate_hz {400.0};
        std::string bias_mode {"calibrated"};   // "none" / "calibrated" / "uncalibrated"
        double noise_scale {1.0};
        bool quantize {true};
        bool saturate {true};
        bool bias_random_walk {false};
        std::uint32_t seed {0};
    };

    struct Vec3d
    {
        double x {0.0}, y {0.0}, z {0.0};
    };

    class ImuNoiseModel
    {
    public:
        void Configure(const ImuSpec& spec, const ImuModelConfig& cfg)
        {
            spec_ = spec;
            cfg_ = cfg;
            rng_.seed(cfg.seed);
            const double offset_scale = cfg.bias_mode == "uncalibrated" ? 1.0
                                      : cfg.bias_mode == "calibrated"   ? 0.1 : 0.0;
            std::uniform_real_distribution<double> u(-1.0, 1.0);
            const double ab = spec.accel_zero_offset_mg * 1e-3 * kStandardGravity * offset_scale;
            const double gb = spec.gyro_zero_offset_dps * kDegToRad * offset_scale;
            accel_bias_ = {ab * u(rng_), ab * u(rng_), ab * u(rng_)};
            gyro_bias_ = {gb * u(rng_), gb * u(rng_), gb * u(rng_)};
        }

        const ImuSpec& Spec() const { return spec_; }
        const ImuModelConfig& Config() const { return cfg_; }
        Vec3d AccelBias() const { return accel_bias_; }
        Vec3d GyroBias() const { return gyro_bias_; }

        double AccelSigma() const
        {
            return cfg_.noise_scale * spec_.accel_noise_density_ug * 1e-6 * kStandardGravity
                * std::sqrt(cfg_.accel_rate_hz);
        }
        double GyroSigma() const
        {
            return cfg_.noise_scale * spec_.gyro_noise_density_dps * kDegToRad * std::sqrt(cfg_.gyro_rate_hz);
        }
        double AccelLsb() const
        {
            return spec_.accel_range_g * kStandardGravity / std::ldexp(1.0, spec_.accel_bits - 1);
        }
        double GyroLsb() const
        {
            return spec_.gyro_range_dps * kDegToRad / std::ldexp(1.0, spec_.gyro_bits - 1);
        }

        Vec3d MeasureAccel(const Vec3d& truth)
        {
            return Measure(truth, accel_bias_, AccelSigma(), AccelLsb(),
                           spec_.accel_range_g * kStandardGravity);
        }

        Vec3d MeasureGyro(const Vec3d& truth)
        {
            if (cfg_.bias_random_walk && cfg_.gyro_rate_hz > 0.0) {
                // Bias instability used as a random-walk rate: sigma per sqrt(s).
                const double rw = spec_.gyro_bias_instability_dph * kDegToRad / 3600.0;
                const double s = rw * std::sqrt(1.0 / cfg_.gyro_rate_hz);
                gyro_bias_.x += s * normal_(rng_);
                gyro_bias_.y += s * normal_(rng_);
                gyro_bias_.z += s * normal_(rng_);
            }
            return Measure(truth, gyro_bias_, GyroSigma(), GyroLsb(), spec_.gyro_range_dps * kDegToRad);
        }

    private:
        double Axis(double truth, double bias, double sigma, double lsb, double range)
        {
            double v = truth + bias + sigma * normal_(rng_);
            if (cfg_.quantize && lsb > 0.0) v = std::round(v / lsb) * lsb;
            if (cfg_.saturate && range > 0.0) v = std::clamp(v, -range, range);
            return v;
        }

        Vec3d Measure(const Vec3d& t, const Vec3d& b, double sigma, double lsb, double range)
        {
            return {Axis(t.x, b.x, sigma, lsb, range), Axis(t.y, b.y, sigma, lsb, range),
                    Axis(t.z, b.z, sigma, lsb, range)};
        }

        ImuSpec spec_ {};
        ImuModelConfig cfg_ {};
        std::mt19937 rng_ {};
        std::normal_distribution<double> normal_ {0.0, 1.0};
        Vec3d accel_bias_ {};
        Vec3d gyro_bias_ {};
    };
}
