// M4: sensing-mode switch demo.
//
// Reads a scene config {env, manifest}, resolves Pattern A vs B from env.xml
// presence, and:
//   A: runs the A-2 SensorRuntime over env.xml and publishes the sensor PDUs
//      (Godot would set ExternalSensing=true and only visualize).
//   B: delegates to Godot self-sensing (no PDU emitted here; Godot's
//      Default3DLiDARController publishes the same lidar_points).
//
// Usage: sensing_switch_demo <scene.json> <pos_twist.bin> <out_dir>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "runtime/sensing_mode.hpp"
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
    if (argc < 4) {
        std::printf("usage: %s <scene.json> <pos_twist.bin> <out_dir>\n", argv[0]);
        return 2;
    }
    const std::string scene_path = argv[1];
    const std::string pos_bin = argv[2];
    const std::string out_dir = argv[3];

    std::ifstream sf(scene_path);
    if (!sf) { std::printf("ERROR: cannot open scene %s\n", scene_path.c_str()); return 2; }
    nlohmann::json scene;
    try { sf >> scene; } catch (const std::exception& e) { std::printf("ERROR scene json: %s\n", e.what()); return 2; }

    const std::string env = scene.value("env", "");
    const std::string manifest = scene.value("manifest", "");

    const rt::SensingDecision d = rt::ResolveSensingMode(env);
    std::printf("scene=%s env='%s'\n", scene.value("name", scene_path).c_str(), env.c_str());
    std::printf("decision: mode=%s use_mujoco_runtime=%s godot_external_sensing=%s\n  reason: %s\n",
                rt::ToString(d.mode),
                d.use_mujoco_runtime ? "true" : "false",
                d.godot_external_sensing ? "true" : "false",
                d.reason.c_str());

    if (!d.use_mujoco_runtime) {
        std::printf("PATTERN_B: delegate to Godot self-sensing (Godot publishes lidar_points).\n");
        return 0;
    }

    // Pattern A: run the A-2 runtime and publish.
    rt::SensorRuntime runtime(env);
    if (!runtime.ok()) { std::printf("ERROR: %s\n", runtime.last_error().c_str()); return 2; }
    if (!runtime.LoadManifest(manifest)) { std::printf("ERROR manifest: %s\n", runtime.last_error().c_str()); return 2; }

    std::vector<char> pos_buf = read_file(pos_bin);
    if (pos_buf.empty()) { std::printf("ERROR: empty pos file\n"); return 2; }
    HakoCpp_Twist twist{};
    hako::pdu::msgs::geometry_msgs::Twist twist_conv;
    if (!twist_conv.pdu2cpp(pos_buf.data(), twist)) { std::printf("ERROR: decode Twist\n"); return 2; }
    rt::BasePose base{};
    base.origin = Vector3(twist.linear.x, twist.linear.y, twist.linear.z);
    base.yaw_rad = twist.angular.z;

    int published = 0;
    auto sink = [&](const std::string& pdu_name, const char* data, int len) {
        const std::string path = out_dir + "/" + pdu_name + ".bin";
        std::ofstream out(path, std::ios::binary);
        out.write(data, len);
        std::printf("PATTERN_A: published %-14s -> %s (%d bytes)\n", pdu_name.c_str(), path.c_str(), len);
        ++published;
    };
    runtime.Step(base, 1.0, sink);
    std::printf("PATTERN_A: %d PDU(s) published by mujoco-sensor\n", published);
    return 0;
}
