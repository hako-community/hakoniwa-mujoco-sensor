#pragma once
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>

namespace hako::robots::sensor::capability {
// Conservative geometric backend descriptor. Per-point times are internal to
// Lidar3DFrame; the legacy XYZI PDU does not transport them.
inline nlohmann::json Geometric(const std::string& type, bool legacy_pdu = true) {
    nlohmann::json caps = nlohmann::json::object();
    caps["multiple_returns"] = false;
    caps["radar_multipath"] = false;
    caps["motion_distortion"] = false;
    caps["material_reflectivity"] = "unsupported";
    caps["per_point_timestamp"] = type == "lidar_3d" && !legacy_pdu;
    caps["doppler"] = type == "radar";
    return caps;
}

inline void Require(const nlohmann::json& capabilities, const nlohmann::json& required) {
    if (!required.is_object()) throw std::invalid_argument("required_capabilities must be an object");
    for (auto it = required.begin(); it != required.end(); ++it) {
        if (!(it.value().is_boolean() || it.value().is_string()))
            throw std::invalid_argument("capability must be boolean or named fidelity");
        if (!capabilities.contains(it.key()) || capabilities.at(it.key()) != it.value())
            throw std::invalid_argument("backend capability insufficient: " + it.key());
    }
}
}
