#include "sensors/common/sensor_health_wire.hpp"
#include "std_msgs/pdu_cpptype_conv_UInt8MultiArray.hpp"
#include "pdu_convertor.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>

namespace wire = hako::robots::sensor::health_wire;
int main(int argc, char** argv) {
    try {
        if (argc < 2) return 2;
        const std::string mode(argv[1]);
        std::vector<std::uint8_t> bytes;
        if (argc == 3) {
            std::string hex(argv[2]);
            if (hex.size() % 2) return 2;
            for (std::size_t i = 0; i < hex.size(); i += 2)
                bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
        }
        using Converter = hako::pdu::PduConvertor<HakoCpp_UInt8MultiArray,
                          hako::pdu::msgs::std_msgs::UInt8MultiArray>;
        Converter conv;
        if (mode == "encode") {
            wire::SensorHealth h {"joint_state", 9007199254740993ULL, 0, 1000000,
                2000000, static_cast<hako::robots::sensor::contract::SensorStatus>(5),
                2, 3, "sim.truth", "校正"};
            bytes = wire::Encode(h);
        } else if (mode == "decode") {
            bytes = wire::Encode(wire::Decode(bytes));
        } else if (mode == "wrap") {
            HakoCpp_UInt8MultiArray msg {};
            msg.data = wire::Encode(wire::Decode(bytes));
            bytes.assign(1024, 0);
            int written = conv.cpp2pdu(msg, reinterpret_cast<char*>(bytes.data()), 1024);
            if (written <= 0 || written > 1024) return 1;
            bytes.resize(written);
        } else if (mode == "unwrap") {
            HakoCpp_UInt8MultiArray msg {};
            if (bytes.size() < 24 || !conv.pdu2cpp(reinterpret_cast<char*>(bytes.data()), msg)) return 1;
            if (!msg.layout.dim.empty() || msg.layout.data_offset != 0) return 1;
            bytes = wire::Encode(wire::Decode(msg.data));
        } else return 2;
        for (auto byte : bytes)
            std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(byte);
        std::cout << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
