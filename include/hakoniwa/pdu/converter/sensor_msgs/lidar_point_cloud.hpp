#pragma once

// Converts a 3D LiDAR Lidar3DFrame into a sensor_msgs/PointCloud2 PDU.
//
// Layout matches the existing Godot Default3DLiDARController output: an
// organized cloud (height x width) of 16-byte points x,y,z,intensity (float32).
// Reuses the already-generated PointCloud2 type/channel machinery (same as the
// radar pipeline), so no registry/codegen changes are needed.

#include <cstring>

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_PointCloud2.hpp"
#include "sensors/lidar/lidar3d_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    inline HakoCpp_PointCloud2 ToHakoPointCloud2(
        const hako::robots::sensor::lidar::Lidar3DFrame& frame)
    {
        constexpr uint32_t kPointStep = 16;  // 4 x float32
        constexpr uint8_t kFloat32 = 7;      // sensor_msgs::PointField::FLOAT32

        HakoCpp_PointCloud2 out {};
        out.header.frame_id = frame.frame_id;
        // Carry the scan timestamp through. Shared helper so all five sensors
        // round the sec/nanosec split identically.
        out.header.stamp = ToHakoTime(frame.stamp_sec);
        out.is_bigendian = false;
        out.is_dense = true;
        out.height = frame.height;
        out.width = frame.width;
        out.point_step = kPointStep;
        out.row_step = kPointStep * frame.width;

        const char* names[4] = {"x", "y", "z", "intensity"};
        for (uint32_t i = 0; i < 4; ++i) {
            HakoCpp_PointField f {};
            f.name = names[i];
            f.offset = i * 4U;
            f.datatype = kFloat32;
            f.count = 1U;
            out.fields.push_back(f);
        }

        out.data.resize(frame.points.size() * kPointStep);
        size_t off = 0;
        for (const auto& p : frame.points) {
            std::memcpy(&out.data[off + 0], &p.x, 4);
            std::memcpy(&out.data[off + 4], &p.y, 4);
            std::memcpy(&out.data[off + 8], &p.z, 4);
            std::memcpy(&out.data[off + 12], &p.intensity, 4);
            off += kPointStep;
        }
        return out;
    }
}
