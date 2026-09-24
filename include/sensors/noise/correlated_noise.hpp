#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string_view>

#include "sensors/common/seed_manager.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor::noise
{
    // Three-axis zero-mean Gaussian noise with an explicit covariance matrix.
    // Independent axes are represented by a diagonal matrix; callers must not
    // rely on accidental RNG-state sharing to obtain correlation.
    class CorrelatedGaussian3
    {
    public:
        using Matrix3 = std::array<std::array<double, 3>, 3>;

        CorrelatedGaussian3(const Matrix3& covariance, std::uint64_t experiment_seed,
                            std::string_view sensor_id, std::string_view stream_id = "measurement",
                            std::uint64_t episode_id = 0)
        {
            SetCovariance(covariance);
            const common::SeedManager seeds(experiment_seed);
            rng_.seed(seeds.Derive32(sensor_id, stream_id, "correlated_xyz", episode_id));
        }

        AxisValue Sample()
        {
            const std::array<double, 3> z {normal_(rng_), normal_(rng_), normal_(rng_)};
            return AxisValue {
                lower_[0][0] * z[0],
                lower_[1][0] * z[0] + lower_[1][1] * z[1],
                lower_[2][0] * z[0] + lower_[2][1] * z[1] + lower_[2][2] * z[2],
            };
        }

    private:
        void SetCovariance(const Matrix3& covariance)
        {
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column <= row; ++column) {
                    double value = covariance[row][column];
                    if (row == column) {
                        for (std::size_t k = 0; k < column; ++k) value -= lower_[row][k] * lower_[row][k];
                        if (value < -1e-12) throw std::invalid_argument("covariance must be positive semidefinite");
                        lower_[row][column] = std::sqrt(std::max(0.0, value));
                    } else {
                        for (std::size_t k = 0; k < column; ++k) value -= lower_[row][k] * lower_[column][k];
                        if (std::abs(lower_[column][column]) < 1e-12) {
                            if (std::abs(value) > 1e-12) throw std::invalid_argument("singular covariance is inconsistent");
                            lower_[row][column] = 0.0;
                        } else {
                            lower_[row][column] = value / lower_[column][column];
                        }
                    }
                }
            }
        }

        Matrix3 lower_ {};
        std::mt19937 rng_;
        std::normal_distribution<double> normal_ {0.0, 1.0};
    };
}
