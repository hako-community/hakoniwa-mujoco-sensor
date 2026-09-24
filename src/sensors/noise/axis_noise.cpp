#include "sensors/noise/noise.hpp"
#include "sensors/common/seed_manager.hpp"
#include <cmath>
#include <stdexcept>

namespace hako {
namespace robots {
namespace sensor {
namespace noise {

    std::unique_ptr<INoiseModel> CreateNoiseModel(NoiseType type, double dt_sec)
    {
        switch (type) {
        case NoiseType::None:
            return std::make_unique<PassthroughNoiseModel>();
        case NoiseType::Gaussian:
        case NoiseType::GaussianQuantized:
            return std::make_unique<GaussianNoiseModel>(dt_sec);
        default:
            throw std::invalid_argument("Unknown NoiseType");
        }
    }

    double PassthroughNoiseModel::Apply(double value, const NoiseParams&)
    {
        return value;
    }

    double PassthroughNoiseModel::ApplyRangeRule(double value, const RangeNoiseRule&)
    {
        return value;
    }

    void PassthroughNoiseModel::Reset()
    {
    }

    double GaussianNoiseModel::UpdateDynamicBias(const NoiseParams& params)
    {
        if (params.dynamic_bias_stddev <= 0.0 ||
            params.dynamic_bias_correlation_time <= 0.0) {
            return 0.0;
        }
        const double tau   = params.dynamic_bias_correlation_time;
        const double sigma = params.dynamic_bias_stddev;
        const double decay = std::exp(-dt_ / tau);
        const double noise = sigma * std::sqrt(1.0 - decay * decay) * dist_normal_(rng_);
        current_bias_ = current_bias_ * decay + noise;
        return current_bias_;
    }

    double GaussianNoiseModel::SampleGaussian(double mean, double stddev)
    {
        if (stddev <= 0.0) return mean;
        std::normal_distribution<double> dist(mean, stddev);
        return dist(rng_);
    }

    double GaussianNoiseModel::Quantize(double value, double precision)
    {
        if (precision <= 0.0) return value;
        return std::round(value / precision) * precision;
    }

    AxisNoisePipeline::AxisNoisePipeline(const AxisNoiseParams& params, double dt_sec,
                                         std::uint64_t experiment_seed, std::string sensor_id)
        : params_(params)
        , model_x_(CreateNoiseModel(params.x.type, dt_sec))
        , model_y_(CreateNoiseModel(params.y.type, dt_sec))
        , model_z_(CreateNoiseModel(params.z.type, dt_sec))
    {
        // Each axis has an explicit, domain-separated stream.  The old three
        // default-constructed mt19937 instances generated identical sequences.
        const common::SeedManager seed_manager(experiment_seed);
        model_x_->Reseed(seed_manager.Derive32(sensor_id, "measurement", "x"));
        model_y_->Reseed(seed_manager.Derive32(sensor_id, "measurement", "y"));
        model_z_->Reseed(seed_manager.Derive32(sensor_id, "measurement", "z"));
    }

    AxisValue AxisNoisePipeline::Apply(const AxisValue& value) const
    {
        return AxisValue {
            model_x_->Apply(value.x, params_.x),
            model_y_->Apply(value.y, params_.y),
            model_z_->Apply(value.z, params_.z),
        };
    }

    void AxisNoisePipeline::Reset()
    {
        model_x_->Reset();
        model_y_->Reset();
        model_z_->Reset();
    }

}  // namespace noise
}  // namespace sensor
}  // namespace robots
}  // namespace hako
