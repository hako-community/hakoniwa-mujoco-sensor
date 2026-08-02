// A-1: A-2 3D LiDAR -> sensor_msgs/PointCloud2 PDU (cross-language contract).
//
//   pos PDU (Twist, in) -> SensorState
//     -> Lidar3DSensor.Scan over env.xml (mj_ray, organized height x width)
//     -> Lidar3DFrame -> HakoCpp_PointCloud2 (ToHakoPointCloud2)
//     -> PointCloud2 PDU binary (out, decoded by Python hakoniwa_pdu)
//
// Same layout (x,y,z,intensity float32) as the Godot Default3DLiDARController.
//
// Usage: lidar3d_a2_pdu <env.xml> <pos_twist.bin> <out_point_cloud2.bin>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "sensors/backend/mujoco_ray_caster.hpp"
#include "sensors/lidar/lidar3d_sensor.hpp"

#include "hakoniwa/pdu/converter/sensor_msgs/lidar_point_cloud.hpp"
#include "sensor_msgs/pdu_cpptype_conv_PointCloud2.hpp"
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
        std::printf("usage: %s <env.xml> <pos_twist.bin> <out_point_cloud2.bin>\n", argv[0]);
        return 2;
    }
    char err[1000] = {0};
    mjModel* model = mj_loadXML(argv[1], nullptr, err, sizeof(err));
    if (model == nullptr) { std::printf("ERROR mj_loadXML: %s\n", err); return 2; }
    mjData* data = mj_makeData(model);
    mj_forward(model, data);
    auto caster = std::make_shared<backend::MujocoRayCaster>(model, data, std::string{});

    std::vector<char> pos_buf = read_file(argv[2]);
    if (pos_buf.empty()) { std::printf("ERROR: empty pos file\n"); return 2; }
    HakoCpp_Twist twist{};
    hako::pdu::msgs::geometry_msgs::Twist twist_conv;
    if (!twist_conv.pdu2cpp(pos_buf.data(), twist)) { std::printf("ERROR: decode Twist\n"); return 2; }
    const double yaw = twist.angular.z;
    backend::SensorState st{};
    st.origin = Vector3(twist.linear.x, twist.linear.y, twist.linear.z);
    st.forward = Vector3(std::cos(yaw), std::sin(yaw), 0.0);
    st.left = Vector3(-std::sin(yaw), std::cos(yaw), 0.0);
    st.up = Vector3(0.0, 0.0, 1.0);
    st.linear_velocity = Vector3(0.0, 0.0, 0.0);

    lidar::Lidar3DSensor sensor(caster);
    lidar::Lidar3DConfig cfg{};
    cfg.frame_id = "front_lidar_frame";
    cfg.channels = 17;
    cfg.rotations_per_second = 10;
    cfg.points_per_second = 61370;  // -> width = 61370/10/17 = 361
    cfg.max_distance = 20.0;
    cfg.min_distance = 0.05;
    cfg.vertical_fov_lower_deg = -40.0;
    cfg.vertical_fov_upper_deg = 40.0;
    cfg.horizontal_fov_start_deg = -180.0;
    cfg.horizontal_fov_end_deg = 180.0;
    sensor.SetConfig(cfg);

    lidar::Lidar3DFrame frame{};
    sensor.Scan(st, frame);
    std::printf("pos=(%.2f,%.2f,%.2f) yaw=%.0f  cloud %ux%u (%zu points)\n",
                st.origin.x, st.origin.y, st.origin.z, yaw * 180.0 / M_PI,
                frame.height, frame.width, frame.points.size());

    HakoCpp_PointCloud2 cpp = hako::robots::pdu::converter::sensor_msgs::ToHakoPointCloud2(frame);
    std::vector<char> buffer(1 << 20, 0);  // 1 MB
    hako::pdu::msgs::sensor_msgs::PointCloud2 pc_conv;
    const int pdu_size = pc_conv.cpp2pdu(cpp, buffer.data(), static_cast<int>(buffer.size()));
    if (pdu_size < 0) { std::printf("ERROR: cpp2pdu PointCloud2\n"); return 2; }

    std::ofstream out(argv[3], std::ios::binary);
    out.write(buffer.data(), pdu_size);
    out.close();
    std::printf("wrote %s (%d bytes)\n", argv[3], pdu_size);

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
