#pragma once

#include <cstdint>
#include <string_view>

namespace hako::robots::sensor::common
{
    // Stable seed derivation for independent sensor stochastic streams.  This
    // intentionally does not use std::hash, whose result is not portable.
    class SeedManager
    {
    public:
        explicit SeedManager(std::uint64_t experiment_seed = 0)
            : experiment_seed_(experiment_seed) {}

        std::uint64_t Derive(std::string_view sensor_id,
                             std::string_view stream_id,
                             std::string_view axis_or_plane = "",
                             std::uint64_t episode_id = 0) const
        {
            std::uint64_t state = Mix(experiment_seed_);
            state = Mix(state ^ Hash(sensor_id));
            state = Mix(state ^ Hash(stream_id));
            state = Mix(state ^ Hash(axis_or_plane));
            return Mix(state ^ episode_id);
        }

        std::uint32_t Derive32(std::string_view sensor_id,
                               std::string_view stream_id,
                               std::string_view axis_or_plane = "",
                               std::uint64_t episode_id = 0) const
        {
            return static_cast<std::uint32_t>(Derive(sensor_id, stream_id, axis_or_plane, episode_id));
        }

    private:
        static constexpr std::uint64_t Hash(std::string_view text)
        {
            // FNV-1a, followed by SplitMix64 below.  Both are specified here
            // so the same config yields the same stream on every platform.
            std::uint64_t result = 14695981039346656037ULL;
            for (const char ch : text) {
                result ^= static_cast<unsigned char>(ch);
                result *= 1099511628211ULL;
            }
            return result;
        }

        static constexpr std::uint64_t Mix(std::uint64_t value)
        {
            value += 0x9e3779b97f4a7c15ULL;
            value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
            value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
            return value ^ (value >> 31U);
        }

        std::uint64_t experiment_seed_;
    };
}
