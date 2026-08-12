#pragma once

// Converts a backend-agnostic RadarScanFrame into a PointCloud2 PDU.
//
// We reuse the already-generated sensor_msgs/PointCloud2 type (same channel
// machinery as the LiDAR pipeline) instead of introducing a new RadarScan PDU
// type, so no registry/codegen changes are needed. Each detection becomes a
// 16-byte point: x,y,z (sensor-local cartesian, ROS REP-103: x fwd, y left,
// z up) + velocity (Doppler, m/s) in place of LiDAR's intensity field.
//
// A native sensor_msgs/RadarScan PDU type (polar layout) can later replace this
// converter without touching the sensor model -- the separation is the point.

#include <cmath>
#include <cstring>
#include <vector>

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_PointCloud2.hpp"
#include "sensors/radar/radar_types.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    inline HakoCpp_PointCloud2 ToHakoPointCloud2(
        const hako::robots::sensor::radar::RadarScanFrame& frame)
    {
        constexpr uint32_t kPointStep = 16;  // 4 x float32
        constexpr uint8_t kFloat32 = 7;      // sensor_msgs::PointField::FLOAT32

        HakoCpp_PointCloud2 out {};
        out.header.frame_id = frame.header.frame_id;
        // Carry the scan timestamp through: it is what lets a consumer notice
        // that the sensor has stopped producing frames. Shared helper so all
        // five sensors round the sec/nanosec split identically.
        out.header.stamp = ToHakoTime(frame.header.stamp_sec);
        out.is_bigendian = false;
        out.is_dense = true;
        out.height = 1;
        out.width = static_cast<uint32_t>(frame.detections.size());
        out.point_step = kPointStep;
        out.row_step = kPointStep * out.width;

        const char* names[4] = {"x", "y", "z", "velocity"};
        for (uint32_t i = 0; i < 4; ++i) {
            HakoCpp_PointField f {};
            f.name = names[i];
            f.offset = i * 4U;
            f.datatype = kFloat32;
            f.count = 1U;
            out.fields.push_back(f);
        }

        out.data.resize(static_cast<size_t>(out.width) * kPointStep);
        size_t off = 0;
        for (const auto& d : frame.detections) {
            const float cz = std::cos(d.altitude);
            const float x = d.depth * cz * std::cos(d.azimuth);  // forward
            const float y = d.depth * cz * std::sin(d.azimuth);  // left
            const float z = d.depth * std::sin(d.altitude);      // up
            const float v = d.velocity;
            std::memcpy(&out.data[off + 0], &x, 4);
            std::memcpy(&out.data[off + 4], &y, 4);
            std::memcpy(&out.data[off + 8], &z, 4);
            std::memcpy(&out.data[off + 12], &v, 4);
            off += kPointStep;
        }
        return out;
    }
}
