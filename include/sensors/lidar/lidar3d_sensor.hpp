#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "sensor.hpp"
#include "sensors/backend/ray_caster.hpp"
#include "common/update_scheduler.hpp"

namespace hako::robots::sensor::lidar
{
    // One LiDAR return, sensor-local cartesian (ROS REP-103: x fwd, y left, z up).
    struct PointXYZI
    {
        float x {0.0F};
        float y {0.0F};
        float z {0.0F};
        float intensity {0.0F};
    };

    // Organized point cloud (height = vertical channels, width = points/rotation).
    struct Lidar3DFrame
    {
        std::string frame_id {"lidar"};
        // Scan timestamp. Without it a consumer cannot tell a live sensor from
        // one that has stopped: a scan of empty sky is byte-identical to the
        // previous one, so "the payload stopped changing" would fire on a
        // perfectly healthy LiDAR. Same fix the radar already carries
        // (ISO 15964 4.2 d: fault detection).
        double stamp_sec {0.0};
        std::uint32_t height {0};
        std::uint32_t width {0};
        std::vector<PointXYZI> points {};  // row-major: channel (pitch) outer, yaw inner
    };

    // Config mirrors the Godot Default3DLiDARController parameters so the A-2
    // sensor reproduces the same scan pattern as the existing Godot 3D LiDAR.
    struct Lidar3DConfig
    {
        std::string frame_id {"front_lidar_frame"};
        int channels {16};                 // vertical beams (height)
        int rotations_per_second {10};
        int points_per_second {10000};
        double max_distance {10.0};
        double min_distance {0.05};
        double vertical_fov_upper_deg {-15.0};
        double vertical_fov_lower_deg {-25.0};
        double horizontal_fov_start_deg {-20.0};
        double horizontal_fov_end_deg {20.0};
    };

    // Backend-independent (Strategy C) 3D LiDAR. Ray cast injected via
    // IRayCaster, pose via SensorState -> works in the A-2 design (sensor not in
    // the scanned world). Output is an organized PointCloud2-ready frame.
    class Lidar3DSensor : public ISensor
    {
    public:
        explicit Lidar3DSensor(std::shared_ptr<backend::IRayCaster> ray_caster);

        void SetConfig(const Lidar3DConfig& config);
        const Lidar3DConfig& GetConfig() const;

        // Grid geometry derived from config (height x width).
        int Height() const;
        int Width() const;

        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;

        void Scan(const backend::SensorState& state, Lidar3DFrame& out);

    private:
        std::shared_ptr<backend::IRayCaster> ray_caster_;
        Lidar3DConfig config_ {};
        common::UpdateScheduler scheduler_ {};
        unsigned long scan_count_ {0UL};
    };
}
