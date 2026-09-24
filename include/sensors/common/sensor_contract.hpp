#pragma once

// Phase S0 observation contract.
//
// This header is deliberately transport- and PDU-independent.  Sensor models
// use SensorEnvelope internally; existing payload PDUs stay unchanged and a
// future adapter emits SensorHealth on a sidecar channel.

#include <cstdint>
#include <string>
#include <utility>

namespace hako::robots::sensor::contract
{
    using TimeNs = std::int64_t;

    enum class SensorStatus : std::uint32_t
    {
        None                = 0U,
        Valid               = 1U << 0U,
        Stale               = 1U << 1U,
        DroppedPrevious     = 1U << 2U,
        Saturated           = 1U << 3U,
        CalibrationExpired  = 1U << 4U,
        BackendDegraded     = 1U << 5U,
        FaultInjected       = 1U << 6U,
    };

    constexpr SensorStatus operator|(SensorStatus lhs, SensorStatus rhs)
    {
        return static_cast<SensorStatus>(
            static_cast<std::uint32_t>(lhs) | static_cast<std::uint32_t>(rhs));
    }

    constexpr SensorStatus operator&(SensorStatus lhs, SensorStatus rhs)
    {
        return static_cast<SensorStatus>(
            static_cast<std::uint32_t>(lhs) & static_cast<std::uint32_t>(rhs));
    }

    constexpr bool HasStatus(SensorStatus value, SensorStatus flag)
    {
        return (value & flag) != SensorStatus::None;
    }

    // All three times are simulation-clock times in nanoseconds.  A value of
    // zero is valid at simulation start; no field uses a sentinel timestamp.
    template <class Payload>
    struct SensorEnvelope
    {
        Payload payload {};
        std::uint64_t sequence {0};
        TimeNs source_time_ns {0};
        TimeNs scheduled_time_ns {0};
        TimeNs publish_time_ns {0};
        SensorStatus status {SensorStatus::Valid};
        std::string sensor_id {};
        std::string frame_id {};
        std::string profile_id {};
        std::string calibration_id {};
    };

    // Transport-neutral representation of the future hako_msgs/SensorHealth
    // sidecar.  Join it with its payload using sensor_id + sequence +
    // source_time_ns.  dropped_count is cumulative since the selected reset.
    struct SensorHealth
    {
        std::string sensor_id {};
        std::uint64_t sequence {0};
        TimeNs source_time_ns {0};
        TimeNs scheduled_time_ns {0};
        TimeNs publish_time_ns {0};
        SensorStatus status {SensorStatus::Valid};
        std::uint64_t dropped_count {0};
        std::uint32_t queue_depth {0};
        std::string profile_id {};
        std::string calibration_id {};
    };

    template <class Payload>
    inline SensorHealth MakeSensorHealth(const SensorEnvelope<Payload>& envelope,
                                         std::uint64_t dropped_count,
                                         std::uint32_t queue_depth)
    {
        return SensorHealth {
            envelope.sensor_id,
            envelope.sequence,
            envelope.source_time_ns,
            envelope.scheduled_time_ns,
            envelope.publish_time_ns,
            envelope.status,
            dropped_count,
            queue_depth,
            envelope.profile_id,
            envelope.calibration_id,
        };
    }

    // Keep metadata and payload coupled until the transport boundary.  The
    // current Hakoniwa PDUs intentionally retain their wire layout; adapters
    // receive only the payload while SensorHealth is emitted from the same
    // envelope by the caller.
    template <class Payload, class Sender>
    inline bool DeliverEnvelope(const SensorEnvelope<Payload>& envelope, Sender&& sender)
    {
        return std::forward<Sender>(sender)(envelope.payload);
    }
}
