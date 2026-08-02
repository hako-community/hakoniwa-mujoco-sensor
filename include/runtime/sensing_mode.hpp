#pragma once

// M4: sensing-mode switch.
//
// The drone's sensor detection runs in one of two paths, chosen by whether an
// environment geometry XML (from hakoniwa-envsim-sensor) exists for the scene:
//
//   Pattern A (env.xml present): hakoniwa-mujoco-sensor senses (mj_ray over
//     env.xml) and publishes the sensor PDUs. Godot is a pure visualizer and
//     DISABLES its own ray casting (ExternalSensing = true).
//
//   Pattern B (no env.xml): geometry exists only in the Godot scene, so Godot
//     keeps doing its own ray casting (Default3DLiDARController) and publishes
//     the sensor PDUs itself (ExternalSensing = false).
//
// Both paths emit the SAME PDU (e.g. lidar_points / PointCloud2), so every
// downstream consumer is path-agnostic.

#include <filesystem>
#include <string>

namespace hako::robots::runtime
{
    enum class SensingMode
    {
        MujocoA2,   // Pattern A: sensing in hakoniwa-mujoco-sensor
        GodotSelf,  // Pattern B: sensing in Godot (Default3DLiDARController)
    };

    struct SensingDecision
    {
        SensingMode mode {SensingMode::GodotSelf};
        bool use_mujoco_runtime {false};      // run the A-2 SensorRuntime?
        bool godot_external_sensing {false};  // tell Godot to disable self ray casting?
        std::string reason {};
    };

    // Decide the path from the presence of an environment XML for the scene.
    inline SensingDecision ResolveSensingMode(const std::string& env_xml)
    {
        SensingDecision d {};
        std::error_code ec;
        const bool present =
            !env_xml.empty() && std::filesystem::exists(env_xml, ec) && !ec;
        if (present) {
            d.mode = SensingMode::MujocoA2;
            d.use_mujoco_runtime = true;
            d.godot_external_sensing = true;
            d.reason = "env.xml present -> mujoco-sensor senses, Godot visualizes only";
        } else {
            d.mode = SensingMode::GodotSelf;
            d.use_mujoco_runtime = false;
            d.godot_external_sensing = false;
            d.reason = env_xml.empty()
                ? "no env configured -> Godot self-sensing"
                : "env.xml not found -> Godot self-sensing";
        }
        return d;
    }

    inline const char* ToString(SensingMode m)
    {
        return m == SensingMode::MujocoA2 ? "MUJOCO_A2(PatternA)" : "GODOT_SELF(PatternB)";
    }
}
