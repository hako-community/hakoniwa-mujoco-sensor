#pragma once

#include "sensors/common/sensor_contract.hpp"
#include <array>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

// Application-owned payload carried by the existing std_msgs/UInt8MultiArray.
// See docs/sensor_health_transport.md for the byte contract. No custom registry type.
namespace hako::robots::sensor::health_wire {
using contract::SensorHealth;
inline constexpr std::size_t HeaderSize = 66;
inline constexpr std::size_t MaxTextBytes = 127;
inline constexpr std::size_t MaxSize = HeaderSize + 3 * MaxTextBytes;

inline void CheckText(const std::string& text) {
    if (text.size() > MaxTextBytes || text.find('\0') != std::string::npos)
        throw std::invalid_argument("SensorHealth string exceeds 127 bytes or contains NUL");
    // Reject malformed UTF-8, overlong encodings, surrogates and > U+10FFFF.
    for (std::size_t i = 0; i < text.size();) {
        auto c = static_cast<unsigned char>(text[i++]);
        if (c < 0x80) continue;
        unsigned n = c >= 0xC2 && c <= 0xDF ? 1 : c >= 0xE0 && c <= 0xEF ? 2 :
                     c >= 0xF0 && c <= 0xF4 ? 3 : 0;
        if (!n || i + n > text.size()) throw std::invalid_argument("SensorHealth UTF-8");
        unsigned code = c & ((1U << (6 - n)) - 1);
        for (unsigned j = 0; j < n; ++j) {
            auto next = static_cast<unsigned char>(text[i++]);
            if ((next & 0xC0) != 0x80) throw std::invalid_argument("SensorHealth UTF-8");
            code = (code << 6) | (next & 0x3F);
        }
        if ((n == 1 && code < 0x80) || (n == 2 && code < 0x800) ||
            (n == 3 && code < 0x10000) || code > 0x10FFFF ||
            (code >= 0xD800 && code <= 0xDFFF))
            throw std::invalid_argument("SensorHealth UTF-8");
    }
}

inline void Validate(const SensorHealth& h) {
    CheckText(h.sensor_id); CheckText(h.profile_id); CheckText(h.calibration_id);
    if (h.sensor_id.empty() || h.source_time_ns < 0 ||
        h.scheduled_time_ns < h.source_time_ns || h.publish_time_ns < h.source_time_ns ||
        (static_cast<std::uint32_t>(h.status) & ~127U))
        throw std::invalid_argument("SensorHealth fields");
}

inline std::vector<std::uint8_t> Encode(const SensorHealth& h) {
    Validate(h);
    std::vector<std::uint8_t> bytes {'H', 'S', 'H', '1'};
    auto put = [&](std::uint64_t v, unsigned width) {
        for (unsigned i = 0; i < width; ++i) bytes.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    };
    put(1, 2); put(0, 2);
    put(HeaderSize + h.sensor_id.size() + h.profile_id.size() + h.calibration_id.size(), 4);
    put(h.sequence, 8); put(h.source_time_ns, 8); put(h.scheduled_time_ns, 8);
    put(h.publish_time_ns, 8); put(static_cast<std::uint32_t>(h.status), 4);
    put(h.dropped_count, 8); put(h.queue_depth, 4);
    for (const auto* s : {&h.sensor_id, &h.profile_id, &h.calibration_id}) put(s->size(), 2);
    for (const auto* s : {&h.sensor_id, &h.profile_id, &h.calibration_id})
        bytes.insert(bytes.end(), s->begin(), s->end());
    return bytes;
}

inline SensorHealth Decode(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < HeaderSize || bytes.size() > MaxSize ||
        bytes[0] != 'H' || bytes[1] != 'S' || bytes[2] != 'H' || bytes[3] != '1')
        throw std::invalid_argument("SensorHealth header");
    std::size_t at = 4;
    auto get = [&](unsigned width) {
        std::uint64_t value = 0;
        for (unsigned i = 0; i < width; ++i) value |= std::uint64_t(bytes[at++]) << (8 * i);
        return value;
    };
    if (get(2) != 1 || get(2) != 0 || get(4) != bytes.size())
        throw std::invalid_argument("SensorHealth version/reserved/length");
    SensorHealth h;
    h.sequence = get(8);
    auto time = [&]() {
        auto n = get(8);
        if (n > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
            throw std::invalid_argument("SensorHealth negative time");
        return static_cast<std::int64_t>(n);
    };
    h.source_time_ns = time(); h.scheduled_time_ns = time(); h.publish_time_ns = time();
    h.status = static_cast<contract::SensorStatus>(get(4));
    h.dropped_count = get(8); h.queue_depth = static_cast<std::uint32_t>(get(4));
    std::array<std::size_t, 3> lengths {};
    for (auto& length : lengths) length = static_cast<std::size_t>(get(2));
    if (HeaderSize + lengths[0] + lengths[1] + lengths[2] != bytes.size())
        throw std::invalid_argument("SensorHealth text lengths");
    unsigned index = 0;
    for (auto* text : {&h.sensor_id, &h.profile_id, &h.calibration_id}) {
        auto size = lengths[index++];
        text->assign(reinterpret_cast<const char*>(bytes.data() + at), size);
        at += size;
    }
    Validate(h);
    return h;
}
} // namespace hako::robots::sensor::health_wire
