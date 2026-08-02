#pragma once
// =============================================================================
// BoolPduAdapter ― std_msgs/Bool 送受信（S4, 薄ラッパー）
//   グリッパ接触 ON/OFF（ch21 gripper_contact）を Bool で送る。
//   HakoCpp_Bool は { bool data } のみ。converter は不要で adapter 内で直接埋める。
//   PDU 基盤（TypedEndpoint）は submodule 流用（plan §0）。Bool=8 は S1 で確認済。
// =============================================================================
#include "hakoniwa/pdu/endpoint.hpp"
#include "hakoniwa/pdu/type_endpoint.hpp"
#include "std_msgs/pdu_cpptype_Bool.hpp"
#include "std_msgs/pdu_cpptype_conv_Bool.hpp"

namespace hako::robots::pdu::adapter::std_msgs
{
    class BoolPduAdapter
    {
    public:
        BoolPduAdapter(hakoniwa::pdu::Endpoint& endpoint, const hakoniwa::pdu::PduKey& key)
            : endpoint_(endpoint, key) {}

        bool send(bool value)
        {
            HakoCpp_Bool pdu {};
            pdu.data = value;
            return endpoint_.send(pdu) == HAKO_PDU_ERR_OK;
        }

        bool recv(HakoCpp_Bool& out)
        {
            return endpoint_.recv(out) == HAKO_PDU_ERR_OK;
        }

    private:
        hakoniwa::pdu::TypedEndpoint<HakoCpp_Bool, hako::pdu::msgs::std_msgs::Bool> endpoint_;
    };
}
