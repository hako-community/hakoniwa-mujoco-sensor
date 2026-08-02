#pragma once
// =============================================================================
// HakoBatteryStatusPduAdapter ― hako_msgs/HakoBatteryStatus 送信
//   { full_voltage, curr_voltage, curr_temp, status, cycles }
//   箱庭の共通 PDU 型を使う（drone 系と揃える）。
// =============================================================================
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "hako_msgs/pdu_cpptype_HakoBatteryStatus.hpp"
#include "hako_msgs/pdu_cpptype_conv_HakoBatteryStatus.hpp"
#include "sensors/battery/battery_model.hpp"

namespace hako::robots::pdu::adapter::hako_msgs
{
    class HakoBatteryStatusPduAdapter
    {
    public:
        HakoBatteryStatusPduAdapter(hakoniwa::pdu::Endpoint& endpoint,
                                    const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key) {}

        bool send(const hako::robots::sensor::BatteryState& s)
        {
            HakoCpp_HakoBatteryStatus pdu {};
            pdu.full_voltage = s.full_voltage;
            pdu.curr_voltage = s.curr_voltage;
            pdu.curr_temp = s.temperature;
            pdu.status = s.status;
            pdu.cycles = s.cycles;
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_HakoBatteryStatus& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_HakoBatteryStatus,
            hako::pdu::msgs::hako_msgs::HakoBatteryStatus> endpoint_;
    };
}
