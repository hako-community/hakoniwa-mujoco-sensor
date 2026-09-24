#include <cassert>
#include <cmath>
#include <vector>

#include "sensors/common/seed_manager.hpp"
#include "sensors/common/measurement_limits.hpp"
#include "sensors/common/timed_sensor_pipeline.hpp"
#include "sensors/noise/correlated_noise.hpp"
#include "sensors/noise/noise.hpp"

using namespace hako::robots::sensor;

int main()
{
    // Seed derivation is stable and domain-separated, independent of OS/STL hash.
    common::SeedManager seeds(1234);
    assert(seeds.Derive("imu", "measurement", "x", 2) == seeds.Derive("imu", "measurement", "x", 2));
    assert(seeds.Derive("imu", "measurement", "x", 2) != seeds.Derive("imu", "measurement", "y", 2));

    noise::NoiseParams gaussian {};
    gaussian.type = noise::NoiseType::Gaussian;
    gaussian.stddev = 1.0;
    noise::GaussianNoiseModel x;
    noise::GaussianNoiseModel y;
    x.Reseed(seeds.Derive32("imu", "measurement", "x"));
    y.Reseed(seeds.Derive32("imu", "measurement", "y"));
    assert(x.Apply(0.0, gaussian) != y.Apply(0.0, gaussian));

    // Explicit covariance, not shared RNG state, creates deliberate correlation.
    const noise::CorrelatedGaussian3::Matrix3 covariance {{{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}}};
    noise::CorrelatedGaussian3 correlated(covariance, 1234, "imu");
    const auto correlated_sample = correlated.Sample();
    assert(std::abs(correlated_sample.x - correlated_sample.y) < 1e-12);
    assert(std::abs(correlated_sample.y - correlated_sample.z) < 1e-12);

    const auto limited = common::ClampMeasurement(12.0, -10.0, 10.0);
    assert(limited.value == 10.0 && limited.saturated);

    common::TimedSensorConfig cfg {};
    cfg.latency.mean_ns = 10;
    cfg.latency.min_ns = 0;
    cfg.latency.max_ns = 100;
    cfg.stale_after_ns = 5;
    common::TimedSensorPipeline<int> pipeline(cfg, 9, "camera/front");
    assert(pipeline.Enqueue(10, 0, "camera", "truth", "none"));
    assert(pipeline.PublishDue(9).empty());
    const auto published = pipeline.PublishDue(10);
    assert(published.size() == 1 && published[0].sequence == 0);
    assert(contract::HasStatus(published[0].status, contract::SensorStatus::Stale));

    // Queue overflow is a dropped acquisition and is reported on next publish.
    common::TimedSensorConfig overflow_cfg {};
    overflow_cfg.queue_depth = 1;
    overflow_cfg.latency.mean_ns = 10;
    common::TimedSensorPipeline<int> overflow(overflow_cfg, 9, "imu");
    assert(overflow.Enqueue(1, 0, "imu", "truth", "none"));
    assert(overflow.Enqueue(2, 1, "imu", "truth", "none"));
    const auto overflowed = overflow.PublishDue(11);
    assert(overflowed.size() == 1 && overflowed[0].sequence == 1);
    assert(contract::HasStatus(overflowed[0].status, contract::SensorStatus::DroppedPrevious));

    common::TimedSensorConfig fault_cfg {};
    fault_cfg.faults.push_back(common::FaultWindow {10, 20, common::FaultAction::Invalidate});
    common::TimedSensorPipeline<int> faults(fault_cfg, 9, "contact");
    assert(faults.Enqueue(1, 10, "foot", "fault", "none"));
    const auto faulted = faults.PublishDue(10);
    assert(faulted.size() == 1);
    assert(contract::HasStatus(faulted[0].status, contract::SensorStatus::FaultInjected));
    assert(!contract::HasStatus(faulted[0].status, contract::SensorStatus::Valid));

    common::TimedSensorConfig schedule_cfg {};
    schedule_cfg.rate_hz = 10.0;
    common::SensorSchedule schedule(schedule_cfg);
    const auto due = schedule.Due(250'000'000);
    assert(due.size() == 3 && due[0] == 0 && due[2] == 200'000'000);

    // Episode replay restores sequence/RNG/queue; runtime continue leaves queued data untouched.
    common::TimedSensorConfig replay_cfg {};
    replay_cfg.latency.mean_ns = 5;
    common::TimedSensorPipeline<int> replay(replay_cfg, 77, "radar");
    replay.Enqueue(1, 0, "radar", "truth", "none");
    const auto first = replay.PublishDue(5);
    replay.Reset(common::ResetMode::EpisodeReplay, 0);
    replay.Enqueue(1, 0, "radar", "truth", "none");
    const auto second = replay.PublishDue(5);
    assert(first[0].sequence == second[0].sequence && first[0].scheduled_time_ns == second[0].scheduled_time_ns);
    return 0;
}
