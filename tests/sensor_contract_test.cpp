#include <cassert>
#include <cstdint>
#include <string>

#include "sensors/common/sensor_contract.hpp"

int main()
{
    using namespace hako::robots::sensor::contract;

    const SensorStatus status = SensorStatus::Valid | SensorStatus::Saturated;
    assert(HasStatus(status, SensorStatus::Valid));
    assert(HasStatus(status, SensorStatus::Saturated));
    assert(!HasStatus(status, SensorStatus::Stale));

    SensorEnvelope<int> envelope {};
    envelope.payload = 42;
    envelope.sensor_id = "imu/head";
    envelope.sequence = 7;
    envelope.source_time_ns = 0;  // simulation start is a valid source time
    envelope.scheduled_time_ns = 1'000'000;
    envelope.publish_time_ns = 1'500'000;
    envelope.status = status;
    envelope.profile_id = "truth";
    envelope.calibration_id = "none";

    const SensorHealth health = MakeSensorHealth(envelope, 3, 2);
    assert(health.sensor_id == envelope.sensor_id);
    assert(health.sequence == 7);
    assert(health.source_time_ns == 0);
    assert(health.publish_time_ns == 1'500'000);
    assert(health.dropped_count == 3);
    assert(health.queue_depth == 2);
    assert(health.profile_id == "truth");
    int delivered = 0;
    assert(DeliverEnvelope(envelope, [&delivered](int payload) {
        delivered = payload;
        return true;
    }));
    assert(delivered == 42);
    return 0;
}
