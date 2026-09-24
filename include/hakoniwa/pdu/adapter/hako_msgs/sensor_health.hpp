#pragma once

#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "hako_msgs/pdu_cpptype_SensorHealth.hpp"
#include "hako_msgs/pdu_cpptype_conv_SensorHealth.hpp"
#include "sensors/common/sensor_contract.hpp"

namespace hako::robots::pdu::adapter::hako_msgs
{
    class SensorHealthPduAdapter
    {
    public:
        SensorHealthPduAdapter(hakoniwa::pdu::Endpoint& endpoint,
                               const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key) {}

        bool send(const hako::robots::sensor::contract::SensorHealth& health)
        {
            HakoCpp_SensorHealth pdu {};
            pdu.sensor_id = health.sensor_id;
            pdu.sequence = health.sequence;
            pdu.source_time_ns = health.source_time_ns;
            pdu.scheduled_time_ns = health.scheduled_time_ns;
            pdu.publish_time_ns = health.publish_time_ns;
            pdu.status = static_cast<std::uint32_t>(health.status);
            pdu.dropped_count = health.dropped_count;
            pdu.queue_depth = health.queue_depth;
            pdu.profile_id = health.profile_id;
            pdu.calibration_id = health.calibration_id;
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_SensorHealth,
            hako::pdu::msgs::hako_msgs::SensorHealth> endpoint_;
    };
}
