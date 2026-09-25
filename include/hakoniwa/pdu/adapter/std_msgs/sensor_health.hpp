#pragma once

#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "std_msgs/pdu_cpptype_UInt8MultiArray.hpp"
#include "std_msgs/pdu_cpptype_conv_UInt8MultiArray.hpp"
#include "sensors/common/sensor_health_wire.hpp"
#include "sensors/common/sensor_contract.hpp"

namespace hako::robots::pdu::adapter::std_msgs
{
    class SensorHealthPduAdapter
    {
    public:
        SensorHealthPduAdapter(hakoniwa::pdu::Endpoint& endpoint,
                               const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key) {}

        bool send(const hako::robots::sensor::contract::SensorHealth& health)
        {
            try {
                HakoCpp_UInt8MultiArray pdu {};
                pdu.data = hako::robots::sensor::health_wire::Encode(health);
                return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
            } catch (const std::invalid_argument&) {
                return false;
            }
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_UInt8MultiArray,
            hako::pdu::msgs::std_msgs::UInt8MultiArray> endpoint_;
    };
}
