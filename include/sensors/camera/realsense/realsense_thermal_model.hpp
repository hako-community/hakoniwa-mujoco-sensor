#pragma once

// Thermal drift of the depth module and the IMU (RS-4). Pure: no MuJoCo.
//
// Temperature (closed form from the power-on time t_on, simulation time):
//   T(t)   = T_amb(t) + dT_ss * (1 - exp(-(t - t_on) / tau))      (start_warm: dT_ss)
//   dT_ss  = steady_rise_k * ((1 - laser_share) + laser_share * P),  P = laser_power_mw / 150
//   T_amb  = ambient_c, or a piecewise-linear schedule [[t_s, deg_c], ...] (clamped ends)
// The camera follows a change of the ambient temperature immediately (hour-scale
// changes such as morning -> noon; the housing lag is not modelled).
//
// Effects, relative to the calibration temperature T_cal:
//   depth : disparity offset  delta_d = beta * (T - T_cal)  [px]
//           -> depth error ~ -z^2 * delta_d / (fx * B): grows with z^2.
//   IMU   : zero-offset drift = tco_axis * (T - T_cal), tco_axis drawn once per run,
//           uniform in [-tco, +tco] per axis (datasheet "TCO +-").
// T_cal defaults to the warm state at 25 C with the nominal projector (factory
// calibration of a warmed-up unit); RunOnChipCalibration() sets it to the
// current temperature (the D400 on-chip self-calibration).
//
// Values are ASSUMPTIONS except the IMU TCO, which are BMI055 datasheet values
// (accel +-1 mg/K, gyro +-0.015 deg/s/K; to be confirmed).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "sensors/camera/realsense/realsense_imu_model.hpp"

namespace hako::robots::sensor::camera::realsense
{
    struct ThermalParams
    {
        double ambient_c {25.0};
        std::vector<std::pair<double, double>> ambient_schedule {};   // (t_s, deg C), sorted by t
        double steady_rise_k {15.0};          // assumed
        double time_constant_s {600.0};       // assumed (10 min)
        double laser_share {0.4};             // assumed share of the projector in the self-heating
        double laser_power_mw {150.0};
        bool start_warm {false};
        double calibration_temp_c {std::numeric_limits<double>::quiet_NaN()};   // NaN = warm at 25 C, nominal laser
        double disparity_coeff_px_per_k {0.005};   // assumed
        double accel_tco_mg_per_k {1.0};           // BMI055 datasheet (to be confirmed)
        double gyro_tco_dps_per_k {0.015};         // BMI055 datasheet (to be confirmed)
        std::uint32_t seed {0};
    };

    class ThermalModel
    {
    public:
        void Configure(const ThermalParams& p)
        {
            p_ = p;
            std::sort(p_.ambient_schedule.begin(), p_.ambient_schedule.end());
            t_on_set_ = false;
            t_cal_ = std::isfinite(p_.calibration_temp_c) ? p_.calibration_temp_c
                                                          : 25.0 + p_.steady_rise_k;   // warm at 25 C, P = 1
            std::mt19937 rng(p_.seed);
            std::uniform_real_distribution<double> u(-1.0, 1.0);
            const double a = p_.accel_tco_mg_per_k * 1e-3 * kStandardGravity;
            const double g = p_.gyro_tco_dps_per_k * kDegToRad;
            accel_tco_ = {a * u(rng), a * u(rng), a * u(rng)};
            gyro_tco_ = {g * u(rng), g * u(rng), g * u(rng)};
        }

        const ThermalParams& Params() const { return p_; }

        // Power-on time; set automatically by the first query when not given.
        void PowerOn(double t)
        {
            t_on_ = t;
            t_on_set_ = true;
        }
        bool PoweredOn() const { return t_on_set_; }

        double AmbientAt(double t) const
        {
            const auto& s = p_.ambient_schedule;
            if (s.empty()) return p_.ambient_c;
            if (t <= s.front().first) return s.front().second;
            if (t >= s.back().first) return s.back().second;
            for (std::size_t i = 1; i < s.size(); ++i) {
                if (t <= s[i].first) {
                    const double span = s[i].first - s[i - 1].first;
                    const double f = span > 0.0 ? (t - s[i - 1].first) / span : 1.0;
                    return s[i - 1].second + f * (s[i].second - s[i - 1].second);
                }
            }
            return s.back().second;
        }

        double SteadyRiseK() const
        {
            const double power = std::max(0.0, p_.laser_power_mw) / 150.0;
            return p_.steady_rise_k * ((1.0 - p_.laser_share) + p_.laser_share * power);
        }

        double TemperatureAt(double t)
        {
            if (!t_on_set_) PowerOn(t);
            const double rise = SteadyRiseK();
            if (p_.start_warm || p_.time_constant_s <= 0.0) return AmbientAt(t) + rise;
            const double dt = std::max(0.0, t - t_on_);
            return AmbientAt(t) + rise * (1.0 - std::exp(-dt / p_.time_constant_s));
        }

        double CalibrationTempC() const { return t_cal_; }
        void SetCalibrationTempC(double c) { t_cal_ = c; }
        // D400 on-chip self-calibration: the current temperature becomes T_cal.
        void RunOnChipCalibration(double t) { t_cal_ = TemperatureAt(t); }

        double DisparityOffsetPx(double t) { return p_.disparity_coeff_px_per_k * (TemperatureAt(t) - t_cal_); }

        Vec3d AccelTco() const { return accel_tco_; }   // m/s^2 per K
        Vec3d GyroTco() const { return gyro_tco_; }     // rad/s per K
        Vec3d AccelDrift(double t)
        {
            const double dk = TemperatureAt(t) - t_cal_;
            return {accel_tco_.x * dk, accel_tco_.y * dk, accel_tco_.z * dk};
        }
        Vec3d GyroDrift(double t)
        {
            const double dk = TemperatureAt(t) - t_cal_;
            return {gyro_tco_.x * dk, gyro_tco_.y * dk, gyro_tco_.z * dk};
        }

    private:
        ThermalParams p_ {};
        bool t_on_set_ {false};
        double t_on_ {0.0};
        double t_cal_ {40.0};
        Vec3d accel_tco_ {};
        Vec3d gyro_tco_ {};
    };
}
