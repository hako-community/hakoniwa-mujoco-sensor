#pragma once

#include "hakoniwa/pdu/converter/sensor_msgs/radar_point_cloud.hpp"
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "sensor_msgs/pdu_cpptype_PointCloud2.hpp"
#include "sensor_msgs/pdu_cpptype_conv_PointCloud2.hpp"
#include "sensors/radar/radar_types.hpp"

namespace hako::robots::pdu::adapter::sensor_msgs
{
    // Publishes a radar scan as a PointCloud2 PDU (x,y,z,velocity per point).
    class RadarPointCloudPduAdapter
    {
    public:
        RadarPointCloudPduAdapter(
            hakoniwa::pdu::Endpoint& endpoint,
            const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key)
        {
        }

        bool send(const hako::robots::sensor::radar::RadarScanFrame& frame)
        {
            // Single-writer per PduKey by convention.
            auto pdu = hako::robots::pdu::converter::sensor_msgs::ToHakoPointCloud2(frame);
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_PointCloud2& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_PointCloud2,
            hako::pdu::msgs::sensor_msgs::PointCloud2> endpoint_;
    };
}
