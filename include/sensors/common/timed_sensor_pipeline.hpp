#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "sensors/common/seed_manager.hpp"
#include "sensors/common/sensor_contract.hpp"

namespace hako::robots::sensor::common
{
    enum class ResetMode { EpisodeReplay, RuntimeContinue };
    enum class OverflowPolicy { DropOldest, DropNewest };
    enum class DropoutModel { None, Bernoulli, GilbertElliott };
    enum class FaultAction { Drop, Invalidate };

    struct FaultWindow
    {
        contract::TimeNs start_ns {0};
        contract::TimeNs end_ns {0};  // inclusive; end before start disables the entry
        FaultAction action {FaultAction::Drop};
    };

    struct LatencyConfig
    {
        std::int64_t mean_ns {0};
        std::int64_t stddev_ns {0};
        std::int64_t min_ns {0};
        std::int64_t max_ns {std::numeric_limits<std::int64_t>::max()};
    };

    struct DropoutConfig
    {
        DropoutModel model {DropoutModel::None};
        double probability {0.0};  // Bernoulli
        double good_to_bad {0.0};  // Gilbert-Elliott transition probability
        double bad_to_good {1.0};
    };

    struct TimedSensorConfig
    {
        double rate_hz {0.0};
        double phase_sec {0.0};
        LatencyConfig latency {};
        DropoutConfig dropout {};
        std::uint32_t queue_depth {8};
        OverflowPolicy overflow_policy {OverflowPolicy::DropOldest};
        std::int64_t stale_after_ns {0};  // zero disables stale marking
        std::vector<FaultWindow> faults {};
    };

    // Produces all sampling instants due at `now_ns`; it deliberately catches
    // up rather than silently losing samples on a slow physics step.
    class SensorSchedule
    {
    public:
        explicit SensorSchedule(const TimedSensorConfig& config) : config_(config) { Reset(); }

        std::vector<contract::TimeNs> Due(contract::TimeNs now_ns)
        {
            std::vector<contract::TimeNs> result;
            if (period_ns_ <= 0) return result;
            while (next_sample_ns_ <= now_ns) {
                result.push_back(next_sample_ns_);
                next_sample_ns_ += period_ns_;
            }
            return result;
        }

        void Reset()
        {
            period_ns_ = config_.rate_hz > 0.0
                ? static_cast<contract::TimeNs>(std::llround(1'000'000'000.0 / config_.rate_hz)) : 0;
            next_sample_ns_ = static_cast<contract::TimeNs>(std::llround(config_.phase_sec * 1'000'000'000.0));
        }

    private:
        TimedSensorConfig config_;
        contract::TimeNs period_ns_ {0};
        contract::TimeNs next_sample_ns_ {0};
    };

    template <class Payload>
    class TimedSensorPipeline
    {
    public:
        TimedSensorPipeline(TimedSensorConfig config, std::uint64_t experiment_seed,
                            std::string sensor_id, std::uint64_t episode_id = 0)
            : config_(std::move(config)), seed_manager_(experiment_seed), sensor_id_(std::move(sensor_id)),
              episode_id_(episode_id)
        {
            Reseed();
        }

        // Returns false for a dropped acquisition.  Sequence advances for both
        // dropped and queued samples, preserving the loss observation contract.
        bool Enqueue(Payload payload, contract::TimeNs source_time_ns, std::string frame_id,
                     std::string profile_id, std::string calibration_id)
        {
            const std::uint64_t sequence = next_sequence_++;
            const FaultWindow* fault = ActiveFault(source_time_ns);
            if ((fault != nullptr && fault->action == FaultAction::Drop) || ShouldDrop()) {
                ++dropped_count_;
                dropped_since_publish_ = true;
                return false;
            }
            contract::SensorEnvelope<Payload> item {};
            item.payload = std::move(payload);
            item.sequence = sequence;
            item.source_time_ns = source_time_ns;
            item.scheduled_time_ns = source_time_ns + SampleLatencyNs();
            item.sensor_id = sensor_id_;
            item.frame_id = std::move(frame_id);
            item.profile_id = std::move(profile_id);
            item.calibration_id = std::move(calibration_id);
            item.status = (fault != nullptr && fault->action == FaultAction::Invalidate)
                ? contract::SensorStatus::FaultInjected
                : contract::SensorStatus::Valid;

            if (config_.queue_depth == 0) {
                ++dropped_count_;
                dropped_since_publish_ = true;
                return false;
            }
            if (queue_.size() >= config_.queue_depth) {
                ++dropped_count_;
                dropped_since_publish_ = true;
                if (config_.overflow_policy == OverflowPolicy::DropNewest) return false;
                queue_.pop_front();
            }
            // Jitter can make a later acquisition due before an earlier one.
            // Keep the transport queue ordered by delivery time so a long-
            // latency sample does not block an already-due short-latency one.
            const auto insert_at = std::upper_bound(
                queue_.begin(), queue_.end(), item.scheduled_time_ns,
                [](contract::TimeNs scheduled_time_ns, const auto& queued) {
                    return scheduled_time_ns < queued.scheduled_time_ns;
                });
            queue_.insert(insert_at, std::move(item));
            return true;
        }

        std::vector<contract::SensorEnvelope<Payload>> PublishDue(contract::TimeNs now_ns)
        {
            std::vector<contract::SensorEnvelope<Payload>> result;
            while (!queue_.empty() && queue_.front().scheduled_time_ns <= now_ns) {
                auto item = std::move(queue_.front());
                queue_.pop_front();
                item.publish_time_ns = now_ns;
                if (config_.stale_after_ns > 0 && now_ns - item.source_time_ns > config_.stale_after_ns) {
                    item.status = item.status | contract::SensorStatus::Stale;
                }
                if (dropped_since_publish_) {
                    item.status = item.status | contract::SensorStatus::DroppedPrevious;
                    dropped_since_publish_ = false;
                }
                result.push_back(std::move(item));
            }
            return result;
        }

        void Reset(ResetMode mode, std::uint64_t episode_id = 0)
        {
            if (mode == ResetMode::RuntimeContinue) return;
            queue_.clear();
            next_sequence_ = 0;
            dropped_count_ = 0;
            dropped_since_publish_ = false;
            ge_bad_state_ = false;
            episode_id_ = episode_id;
            Reseed();
        }

        std::uint64_t dropped_count() const { return dropped_count_; }
        std::uint32_t queue_depth() const { return static_cast<std::uint32_t>(queue_.size()); }

    private:
        void Reseed()
        {
            latency_rng_.seed(seed_manager_.Derive32(sensor_id_, "latency", "", episode_id_));
            dropout_rng_.seed(seed_manager_.Derive32(sensor_id_, "dropout", "", episode_id_));
        }

        bool ShouldDrop()
        {
            const auto probability = [this](double p) {
                return std::bernoulli_distribution(std::clamp(p, 0.0, 1.0))(dropout_rng_);
            };
            if (config_.dropout.model == DropoutModel::Bernoulli) return probability(config_.dropout.probability);
            if (config_.dropout.model == DropoutModel::GilbertElliott) {
                if (ge_bad_state_) ge_bad_state_ = !probability(config_.dropout.bad_to_good);
                else ge_bad_state_ = probability(config_.dropout.good_to_bad);
                return ge_bad_state_;
            }
            return false;
        }

        const FaultWindow* ActiveFault(contract::TimeNs source_time_ns) const
        {
            for (const auto& fault : config_.faults) {
                if (fault.end_ns >= fault.start_ns && source_time_ns >= fault.start_ns && source_time_ns <= fault.end_ns) {
                    return &fault;
                }
            }
            return nullptr;
        }

        contract::TimeNs SampleLatencyNs()
        {
            double value = static_cast<double>(config_.latency.mean_ns);
            if (config_.latency.stddev_ns > 0) {
                value += std::normal_distribution<double>(0.0, static_cast<double>(config_.latency.stddev_ns))(latency_rng_);
            }
            value = std::clamp(value, static_cast<double>(config_.latency.min_ns),
                               static_cast<double>(config_.latency.max_ns));
            return static_cast<contract::TimeNs>(std::llround(value));
        }

        TimedSensorConfig config_;
        SeedManager seed_manager_;
        std::string sensor_id_;
        std::uint64_t episode_id_ {0};
        std::uint64_t next_sequence_ {0};
        std::uint64_t dropped_count_ {0};
        bool dropped_since_publish_ {false};
        bool ge_bad_state_ {false};
        std::mt19937 latency_rng_;
        std::mt19937 dropout_rng_;
        std::deque<contract::SensorEnvelope<Payload>> queue_;
    };
}
