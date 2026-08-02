// B-1: manifest-driven A-2 sensor runtime demo.
//
//   env.xml + manifest(JSON) -> SensorRuntime creates the SELECTED sensors
//   pos PDU (Twist) -> BasePose -> Step() drives each sensor -> per-sensor PDU
//
// Each published PDU is written to <out_dir>/<pdu_name>.bin (transport-agnostic
// sink; the same bytes would go to SHM/endpoint in integration).
//
// Usage: sensor_runtime_demo <env.xml> <manifest.json> <pos_twist.bin> <out_dir>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "runtime/sensor_runtime.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"

namespace rt = hako::robots::runtime;
using hako::robots::types::Vector3;

static std::vector<char> read_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::printf("usage: %s <env.xml> <manifest.json> <pos_twist.bin> <out_dir>\n", argv[0]);
        return 2;
    }
    const std::string env_xml = argv[1];
    const std::string manifest = argv[2];
    const std::string pos_bin = argv[3];
    const std::string out_dir = argv[4];

    rt::SensorRuntime runtime(env_xml);
    if (!runtime.ok()) { std::printf("ERROR: %s\n", runtime.last_error().c_str()); return 2; }
    if (!runtime.LoadManifest(manifest)) {
        std::printf("ERROR manifest: %s\n", runtime.last_error().c_str());
        return 2;
    }
    std::printf("runtime: env=%s, %zu sensor(s) selected:\n", env_xml.c_str(), runtime.component_count());
    for (const auto& c : runtime.components()) {
        std::printf("  - id=%s type=%s pdu=%s\n", c->id().c_str(), c->type().c_str(), c->pdu_name().c_str());
    }

    // pos PDU (Twist) -> BasePose (ROS == MuJoCo N,W,Up for envsim worlds)
    std::vector<char> pos_buf = read_file(pos_bin);
    if (pos_buf.empty()) { std::printf("ERROR: empty pos file\n"); return 2; }
    HakoCpp_Twist twist{};
    hako::pdu::msgs::geometry_msgs::Twist twist_conv;
    if (!twist_conv.pdu2cpp(pos_buf.data(), twist)) { std::printf("ERROR: decode Twist\n"); return 2; }
    rt::BasePose base {};
    base.origin = Vector3(twist.linear.x, twist.linear.y, twist.linear.z);
    base.yaw_rad = twist.angular.z;

    int published = 0;
    auto sink = [&](const std::string& pdu_name, const char* data, int len) {
        const std::string path = out_dir + "/" + pdu_name + ".bin";
        std::ofstream out(path, std::ios::binary);
        out.write(data, len);
        std::printf("  published %-14s -> %s (%d bytes)\n", pdu_name.c_str(), path.c_str(), len);
        ++published;
    };

    // dt large enough that every sensor's scheduler fires once.
    runtime.Step(base, 1.0, sink);
    std::printf("done: %d PDU(s) published\n", published);
    return published > 0 ? 0 : 1;
}
