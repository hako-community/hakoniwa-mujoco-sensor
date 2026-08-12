// Multi-sensor A-2 SHM bridge (external participant).
//
// Same idea as m6_sensor_bridge, but publishes BOTH lidar and radar so the
// Godot Pattern A path can be exercised end-to-end for each. Attaches to the
// running hakoniwa master SHM as an EXTERNAL client (no conductor), loops:
//     read  Drone/pos (Twist) -> BasePose
//     run   the A-2 SensorRuntime over env.xml
//     write each due sensor's PointCloud2 to its SHM channel by pdu_name:
//         "lidar_points" -> lidar_ch (ch16)
//         "radar_scan"/"radar_points" -> radar_ch (ch19)
//
// Extra sensors (a second radar, say) map their pdu_name onto further channels
// through A2_PDU_MAP, e.g. A2_PDU_MAP="radar_points_rear=21". Unset means the
// built-in mapping above and nothing else -- a manifest carrying a sensor whose
// pdu_name has no channel is simply not published, so adding sensors never
// disturbs a single-sensor setup.
//
// Usage: sensor_bridge_multi <env.xml> <manifest.json>
//          [robot=Drone] [pos_ch=1] [pos_size=72]
//          [lidar_ch=16] [radar_ch=19] [chan_size=177424] [hz=20]

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <map>
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
                    "[pos_size=72] [lidar_ch=16] [radar_ch=19] [chan_size=177424] [hz=20]\n",
                    argv[0]);
        return 2;
    }
    const std::string env_xml   = argv[1];
    const std::string manifest  = argv[2];
    const std::string robot     = (argc > 3) ? argv[3] : "Drone";
    const int    pos_ch         = (argc > 4) ? std::atoi(argv[4]) : 1;
    const size_t pos_size       = (argc > 5) ? std::strtoul(argv[5], nullptr, 10) : 72u;
    const int    lidar_ch       = (argc > 6) ? std::atoi(argv[6]) : 16;
    const int    radar_ch       = (argc > 7) ? std::atoi(argv[7]) : 19;
    const size_t chan_size      = (argc > 8) ? std::strtoul(argv[8], nullptr, 10) : 177424u;
    const int    hz             = (argc > 9) ? std::atoi(argv[9]) : 20;
    // A-1 actor injection: a free-joint body in env.xml driven from outside.
    //   actor_robot = "demo"  -> scripted head-on motion (single-drone verification)
    //   actor_robot = <robot> -> driven by that robot's pos PDU (Stage B: 2 drones)
    const std::string actor_body  = (argc > 10) ? argv[10] : "";
    const std::string actor_robot = (argc > 11) ? argv[11] : "demo";
    const int         actor_ch    = (argc > 12) ? std::atoi(argv[12]) : 1;

    std::signal(SIGINT, on_sig);
    std::signal(SIGTERM, on_sig);

    rt::SensorRuntime runtime(env_xml);
    if (!runtime.ok()) { std::printf("ERROR env: %s\n", runtime.last_error().c_str()); return 2; }
    if (!runtime.LoadManifest(manifest)) {
        std::printf("ERROR manifest: %s\n", runtime.last_error().c_str()); return 2;
    }
    std::printf("[multi] env=%s manifest=%s, %zu sensor(s)\n",
                env_xml.c_str(), manifest.c_str(), runtime.component_count());

    if (hako_initialize_for_external() != 0) {
        std::printf("[multi] ERROR: hako_initialize_for_external() failed "
                    "(is the master/drone service running?)\n");
        return 3;
    }
    std::printf("[multi] attached to SHM (external). robot=%s lidar_ch=%d radar_ch=%d hz=%d\n",
                robot.c_str(), lidar_ch, radar_ch, hz);

    // runtime pdu_name -> SHM channel id
    std::map<std::string, int> name2ch = {
        {"lidar_points", lidar_ch},
        {"radar_scan",   radar_ch},   // manifest radar pdu_name
        {"radar_points", radar_ch},
    };
    // A2_PDU_MAP="name=ch[,name=ch...]" adds or overrides entries.
    if (const char* extra = std::getenv("A2_PDU_MAP")) {
        std::string spec(extra);
        size_t pos = 0;
        while (pos < spec.size()) {
            const size_t comma = spec.find(',', pos);
            const std::string item = spec.substr(pos, comma == std::string::npos
                                                     ? std::string::npos : comma - pos);
            const size_t eq = item.find('=');
            if (eq != std::string::npos && eq > 0) {
                const std::string key = item.substr(0, eq);
                const int ch = std::atoi(item.substr(eq + 1).c_str());
                name2ch[key] = ch;
                std::printf("[multi] pdu map: %s -> ch%d\n", key.c_str(), ch);
            }
            if (comma == std::string::npos) break;
            pos = comma + 1;
        }
    }

    if (!actor_body.empty()) {
        if (runtime.HasActor(actor_body)) {
            std::printf("[multi] actor '%s' driven by %s\n", actor_body.c_str(), actor_robot.c_str());
        } else {
            std::printf("[multi] WARN: no free-joint body '%s' in env.xml -> actor disabled\n",
                        actor_body.c_str());
        }
    }

    std::vector<char> posbuf(pos_size);
    std::vector<char> actorbuf(pos_size);
    Vector3 actor_prev{};
    bool actor_have_prev = false;
    Vector3 actor_vel{};
    const auto t_start = std::chrono::steady_clock::now();
    std::vector<char> outbuf(chan_size);
    hako::pdu::msgs::geometry_msgs::Twist twist_conv;
    const auto period = std::chrono::microseconds(1000000 / (hz > 0 ? hz : 1));

    long reads = 0;
    std::map<std::string, long> frames;
    // A-1: the drone's own velocity, by finite difference on Drone/pos. Needed so the
    // radar reports a real relative (Doppler) velocity instead of a constant 0 m/s.
    Vector3 prev_pos{};
    auto prev_t = std::chrono::steady_clock::now();
    bool have_prev = false;
    Vector3 self_vel{};
    const double vel_alpha = 0.3;   // low-pass: pos comes in quantized, raw diff is noisy
    // #12: the yaw RATE, same finite difference on Drone/pos.angular.z. A radar
    // mounted 0.15 m ahead of the body origin does not move at the body's
    // velocity while the drone yaws -- it swings on that lever arm, and the
    // runtime needs omega to add the term (see runtime::MakeState).
    double prev_yaw = 0.0;
    double yaw_rate = 0.0;

    while (g_run) {
        const auto t0 = std::chrono::steady_clock::now();
        if (hako_asset_pdu_read(robot.c_str(), pos_ch, posbuf.data(), posbuf.size()) == 0) {
            HakoCpp_Twist tw{};
            if (twist_conv.pdu2cpp(posbuf.data(), tw)) {
                ++reads;
                rt::BasePose base{};
                base.origin = Vector3(tw.linear.x, tw.linear.y, tw.linear.z);
                base.yaw_rad = tw.angular.z;

                const double dt_s = std::chrono::duration<double>(t0 - prev_t).count();
                if (have_prev && dt_s > 1e-4) {
                    const Vector3 raw((base.origin.x - prev_pos.x) / dt_s,
                                      (base.origin.y - prev_pos.y) / dt_s,
                                      (base.origin.z - prev_pos.z) / dt_s);
                    self_vel = Vector3(self_vel.x + vel_alpha * (raw.x - self_vel.x),
                                       self_vel.y + vel_alpha * (raw.y - self_vel.y),
                                       self_vel.z + vel_alpha * (raw.z - self_vel.z));
                    // Yaw is an angle: differencing it raw makes a drone crossing
                    // +/-pi look like it spun at hundreds of rad/s for one frame.
                    // Wrap the DIFFERENCE into (-pi, pi] first.
                    double dyaw = base.yaw_rad - prev_yaw;
                    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                    const double raw_rate = dyaw / dt_s;
                    yaw_rate += vel_alpha * (raw_rate - yaw_rate);
                }
                prev_pos = base.origin;
                prev_yaw = base.yaw_rad;
                prev_t = t0;
                have_prev = true;
                base.linear_velocity = self_vel;
                base.angular_velocity = Vector3(0.0, 0.0, yaw_rate);

                if (!actor_body.empty() && runtime.HasActor(actor_body)) {
                    Vector3 apos{}, avel{};
                    bool ok = false;
                    if (actor_robot == "demo") {
                        // triangle wave along +x (MuJoCo north): 6.0 m -> 1.5 m -> 6.0 m
                        const double t = std::chrono::duration<double>(t0 - t_start).count();
                        // stays INSIDE the room (simple_room's north wall face is at x = 3.9)
                        const double speed = 1.0, span = 2.4, period = 2.0 * span;
                        const double ph = std::fmod(t, period);
                        const double x = (ph < span) ? (3.4 - speed * ph)
                                                     : (3.4 - speed * (period - ph));
                        const double vx = (ph < span) ? -speed : speed;
                        apos = Vector3(x, 0.0, 0.6);
                        avel = Vector3(vx, 0.0, 0.0);
                        ok = true;
                    } else if (hako_asset_pdu_read(actor_robot.c_str(), actor_ch,
                                                   actorbuf.data(), actorbuf.size()) == 0) {
                        HakoCpp_Twist atw{};
                        if (twist_conv.pdu2cpp(actorbuf.data(), atw)) {
                            apos = Vector3(atw.linear.x, atw.linear.y, atw.linear.z);
                            if (actor_have_prev && dt_s > 1e-4) {
                                const Vector3 raw((apos.x - actor_prev.x) / dt_s,
                                                  (apos.y - actor_prev.y) / dt_s,
                                                  (apos.z - actor_prev.z) / dt_s);
                                actor_vel = Vector3(actor_vel.x + vel_alpha * (raw.x - actor_vel.x),
                                                    actor_vel.y + vel_alpha * (raw.y - actor_vel.y),
                                                    actor_vel.z + vel_alpha * (raw.z - actor_vel.z));
                            }
                            actor_prev = apos;
                            actor_have_prev = true;
                            avel = actor_vel;
                            ok = true;
                        }
                    }
                    if (ok) runtime.SetActor(actor_body, apos, 0.0, avel);
                }

                auto sink = [&](const std::string& name, const char* data, int len) {
                    auto it = name2ch.find(name);
                    if (it == name2ch.end()) return;
                    if (static_cast<size_t>(len) > chan_size) return;
                    std::memset(outbuf.data(), 0, chan_size);
                    std::memcpy(outbuf.data(), data, static_cast<size_t>(len));
                    if (hako_asset_pdu_write(robot.c_str(), it->second,
                                             outbuf.data(), chan_size) == 0) {
                        long& fc = frames[name];
                        ++fc;
                        if (fc % 40 == 1) {
                            std::printf("[multi] frame#%ld %s pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) "
                                        "yawrate=%.2f -> ch%d\n",
                                        fc, name.c_str(), tw.linear.x, tw.linear.y, tw.linear.z,
                                        self_vel.x, self_vel.y, self_vel.z, yaw_rate, it->second);
                        }
                    }
                };
                runtime.Step(base, 1.0, sink);
            }
        }
        std::this_thread::sleep_until(t0 + period);
    }
    std::printf("[multi] stop. reads=%ld", reads);
    for (const auto& kv : frames) std::printf(" %s=%ld", kv.first.c_str(), kv.second);
    std::printf("\n");
    return 0;
}
