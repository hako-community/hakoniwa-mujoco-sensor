// M6: A-2 sensor SHM bridge (external participant).
//
//   Attaches to the running hakoniwa master SHM as an EXTERNAL client (no
//   conductor / not a SYNC asset, same access path the Python clients use),
//   then loops:
//     read  Drone/pos (Twist, ch) -> BasePose
//     run   the A-2 SensorRuntime over env.xml (mujoco-sensor real sensing)
//     write Drone/lidar_points (PointCloud2, ch) back to SHM
//
//   This is the A-2 "detection" producer for the M6 end-to-end demo: the
//   physics drone (drone-core) provides pos and consumes move commands; this
//   bridge senses the env.xml obstacle world from the drone pose and publishes
//   lidar_points that the avoidance controller reads.
//
// Usage: m6_sensor_bridge <env.xml> <manifest.json>
//          [robot=Drone] [pos_ch=1] [pos_size=72]
//          [lidar_ch=16] [lidar_chan_size=177424] [hz=20]

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "runtime/sensor_runtime.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"

extern "C" {
#include "hako_asset.h"
}

namespace rt = hako::robots::runtime;
using hako::robots::types::Vector3;

static std::atomic_bool g_run{true};
static void on_sig(int) { g_run = false; }

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::printf("usage: %s <env.xml> <manifest.json> [robot=Drone] [pos_ch=1] "
                    "[pos_size=72] [lidar_ch=16] [lidar_chan_size=177424] [hz=20]\n", argv[0]);
        return 2;
    }
    const std::string env_xml = argv[1];
    const std::string manifest = argv[2];
    const std::string robot = (argc > 3) ? argv[3] : "Drone";
    const int    pos_ch          = (argc > 4) ? std::atoi(argv[4]) : 1;
    const size_t pos_size        = (argc > 5) ? std::strtoul(argv[5], nullptr, 10) : 72u;
    const int    lidar_ch        = (argc > 6) ? std::atoi(argv[6]) : 16;
    const size_t lidar_chan_size = (argc > 7) ? std::strtoul(argv[7], nullptr, 10) : 177424u;
    const int    hz              = (argc > 8) ? std::atoi(argv[8]) : 20;

    std::signal(SIGINT, on_sig);
    std::signal(SIGTERM, on_sig);

    rt::SensorRuntime runtime(env_xml);
    if (!runtime.ok()) { std::printf("ERROR env: %s\n", runtime.last_error().c_str()); return 2; }
    if (!runtime.LoadManifest(manifest)) {
        std::printf("ERROR manifest: %s\n", runtime.last_error().c_str()); return 2;
    }
    std::printf("[m6_sensor] env=%s manifest=%s, %zu sensor(s)\n",
                env_xml.c_str(), manifest.c_str(), runtime.component_count());

    if (hako_initialize_for_external() != 0) {
        std::printf("[m6_sensor] ERROR: hako_initialize_for_external() failed "
                    "(is the master/drone service running?)\n");
        return 3;
    }
    std::printf("[m6_sensor] attached to SHM (external). robot=%s pos_ch=%d lidar_ch=%d hz=%d\n",
                robot.c_str(), pos_ch, lidar_ch, hz);

    std::vector<char> posbuf(pos_size);
    std::vector<char> lidarbuf(lidar_chan_size);
    hako::pdu::msgs::geometry_msgs::Twist twist_conv;
    const auto period = std::chrono::microseconds(1000000 / (hz > 0 ? hz : 1));

    long reads = 0, frames = 0;
    while (g_run) {
        const auto t0 = std::chrono::steady_clock::now();
        if (hako_asset_pdu_read(robot.c_str(), pos_ch, posbuf.data(), posbuf.size()) == 0) {
            HakoCpp_Twist tw{};
            if (twist_conv.pdu2cpp(posbuf.data(), tw)) {
                ++reads;
                rt::BasePose base{};
                base.origin = Vector3(tw.linear.x, tw.linear.y, tw.linear.z);
                base.yaw_rad = tw.angular.z;

                bool wrote = false;
                auto sink = [&](const std::string& name, const char* data, int len) {
                    if (name != "lidar_points") return;
                    if (static_cast<size_t>(len) > lidar_chan_size) return;
                    std::memset(lidarbuf.data(), 0, lidar_chan_size);
                    std::memcpy(lidarbuf.data(), data, static_cast<size_t>(len));
                    if (hako_asset_pdu_write(robot.c_str(), lidar_ch,
                                             lidarbuf.data(), lidar_chan_size) == 0) {
                        wrote = true;
                    }
                };
                // dt large enough to fire the scheduler every loop (publish at loop rate).
                runtime.Step(base, 1.0, sink);
                if (wrote) {
                    ++frames;
                    if (frames % 40 == 1) {
                        std::printf("[m6_sensor] frame#%ld pos=(%.2f,%.2f,%.2f) -> lidar_points\n",
                                    frames, tw.linear.x, tw.linear.y, tw.linear.z);
                    }
                }
            }
        }
        std::this_thread::sleep_until(t0 + period);
    }
    std::printf("[m6_sensor] stop. reads=%ld frames=%ld\n", reads, frames);
    return 0;
}
