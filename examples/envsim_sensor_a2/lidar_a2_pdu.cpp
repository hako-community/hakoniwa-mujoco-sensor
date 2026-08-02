// M3: A-2 LiDAR -> real hakoniwa LaserScan PDU binary (cross-language contract).
//
// Pipeline proven here (no conductor; the PDU binary is byte-identical to what
// the SHM/endpoint transport carries -- that wiring rides on the proven radar
// SHM path and is folded into integration M4/M5):
//
//   pos PDU (Twist binary, in) -> SensorState
//     -> LidarScanSensor.Scan over env.xml (mj_ray)
//     -> LaserScanFrame -> HakoCpp_LaserScan (ToHakoPdu)
//     -> LaserScan PDU binary (out, decoded by Python hakoniwa_pdu)
//
// Usage: lidar_a2_pdu <env.xml> <pos_twist.bin> <out_laser_scan.bin>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "sensors/backend/mujoco_ray_caster.hpp"
#include "sensors/lidar/lidar_scan_sensor.hpp"

// hakoniwa PDU converters (header-only): domain frame -> HakoCpp -> binary
#include "hakoniwa/pdu/converter/sensor_msgs/laser_scan.hpp"
#include "sensor_msgs/pdu_cpptype_conv_LaserScan.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"

using hako::robots::types::Vector3;
namespace backend = hako::robots::sensor::backend;
namespace lidar = hako::robots::sensor::lidar;

static std::vector<char> read_file(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::printf("usage: %s <env.xml> <pos_twist.bin> <out_laser_scan.bin>\n", argv[0]);
        return 2;
    }
    const std::string env_xml = argv[1];
    const std::string pos_bin = argv[2];
    const std::string out_bin = argv[3];

    // --- env world (kinematic) ---
    char err[1000] = {0};
    mjModel* model = mj_loadXML(env_xml.c_str(), nullptr, err, sizeof(err));
    if (model == nullptr) { std::printf("ERROR mj_loadXML: %s\n", err); return 2; }
    mjData* data = mj_makeData(model);
    mj_forward(model, data);
    auto caster = std::make_shared<backend::MujocoRayCaster>(model, data, std::string{});

    // --- pos PDU (Twist) -> SensorState ---
    // pos is ROS frame; for the envsim world ROS (x,y,z) == MuJoCo (N, W, Up).
    std::vector<char> pos_buf = read_file(pos_bin);
    if (pos_buf.empty()) { std::printf("ERROR: empty pos file %s\n", pos_bin.c_str()); return 2; }
    HakoCpp_Twist twist{};
    hako::pdu::msgs::geometry_msgs::Twist twist_conv;
    if (!twist_conv.pdu2cpp(pos_buf.data(), twist)) {
        std::printf("ERROR: failed to decode Twist pos PDU\n");
        return 2;
    }
    const double yaw = twist.angular.z;  // rotation about up
    backend::SensorState st{};
    st.origin = Vector3(twist.linear.x, twist.linear.y, twist.linear.z);
    st.forward = Vector3(std::cos(yaw), std::sin(yaw), 0.0);
    st.left = Vector3(-std::sin(yaw), std::cos(yaw), 0.0);
    st.up = Vector3(0.0, 0.0, 1.0);
    st.linear_velocity = Vector3(0.0, 0.0, 0.0);
    std::printf("pos: origin=(%.2f,%.2f,%.2f) yaw=%.1f deg\n",
                st.origin.x, st.origin.y, st.origin.z, yaw * 180.0 / M_PI);

    // --- sense ---
    lidar::LidarScanSensor sensor(caster);
    lidar::LidarScanConfig cfg{};
    cfg.frame_id = "lidar";
    cfg.angle_min_deg = -180.0; cfg.angle_max_deg = 180.0; cfg.angle_increment_deg = 1.0;
    cfg.range_min = 0.05; cfg.range_max = 20.0;
    sensor.SetConfig(cfg);
    lidar::LaserScanFrame frame{};
    sensor.Scan(st, frame);

    // --- LaserScanFrame -> HakoCpp -> PDU binary ---
    HakoCpp_LaserScan cpp = hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(frame);
    std::vector<char> buffer(1 << 16, 0);
    hako::pdu::msgs::sensor_msgs::LaserScan ls_conv;
    const int pdu_size = ls_conv.cpp2pdu(cpp, buffer.data(), static_cast<int>(buffer.size()));
    if (pdu_size < 0) { std::printf("ERROR: cpp2pdu failed\n"); return 2; }

    std::ofstream out(out_bin, std::ios::binary);
    out.write(buffer.data(), pdu_size);
    out.close();
    std::printf("wrote %s (%d bytes, %zu ranges)\n", out_bin.c_str(), pdu_size, frame.ranges.size());

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
