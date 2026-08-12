#pragma once

#include "hakoniwa/pdu/converter/common.hpp"
#include "sensor_msgs/pdu_cpptype_LaserScan.hpp"
#include "sensors/lidar/lidar_2d_sensor.hpp"

namespace hako::robots::pdu::converter::sensor_msgs
{
    inline HakoCpp_LaserScan ToHakoPdu(
        const hako::robots::sensor::lidar::LaserScanFrame& frame)
    {
        HakoCpp_LaserScan out {};
        // Header was left empty here: the frame_id the sensor resolved never
        // reached the PDU, and there was no timestamp at all.
        out.header.frame_id = frame.frame_id;
        out.header.stamp = ToHakoTime(frame.stamp_sec);
        out.angle_min = frame.angle_min;
        out.angle_max = frame.angle_max;
        out.angle_increment = frame.angle_increment;
        out.time_increment = frame.time_increment;
        out.scan_time = frame.scan_time;
        out.range_min = frame.range_min;
        out.range_max = frame.range_max;
        out.ranges = frame.ranges;
        out.intensities = frame.intensities;
        return out;
    }
}
