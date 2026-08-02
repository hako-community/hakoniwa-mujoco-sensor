#pragma once
// =============================================================================
// WrenchStampedPduAdapter ― geometry_msgs/WrenchStamped 送信（S4, 薄ラッパー）
//   手首 力覚（ch22 wrist_wrench）を force[3]/torque[3]＋frame_id＋timestamp で送る。
//   HakoCpp_WrenchStamped = { Header header; Wrench{force,torque} }。adapter 内で直接埋める。
//   PDU 基盤（TypedEndpoint）は submodule 流用（plan §0）。
// =============================================================================
#include <array>
#include <string>

#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "hakoniwa/pdu/converter/common.hpp"
#include "geometry_msgs/pdu_cpptype_WrenchStamped.hpp"
#include "geometry_msgs/pdu_cpptype_conv_WrenchStamped.hpp"

namespace hako::robots::pdu::adapter::geometry_msgs
{
    class WrenchStampedPduAdapter
    {
    public:
        WrenchStampedPduAdapter(hakoniwa::pdu::Endpoint& endpoint,
                                const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key) {}

        bool send(const std::array<double, 3>& force,
                  const std::array<double, 3>& torque,
                  const std::string& frame_id, double timestamp)
        {
            HakoCpp_WrenchStamped pdu {};
            pdu.header.stamp = hako::robots::pdu::converter::ToHakoTime(timestamp);
            pdu.header.frame_id = frame_id;
            pdu.wrench.force.x = force[0];
            pdu.wrench.force.y = force[1];
            pdu.wrench.force.z = force[2];
            pdu.wrench.torque.x = torque[0];
            pdu.wrench.torque.y = torque[1];
            pdu.wrench.torque.z = torque[2];
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_WrenchStamped& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<
            HakoCpp_WrenchStamped, hako::pdu::msgs::geometry_msgs::WrenchStamped> endpoint_;
    };
}
