#pragma once

#include <memory>
#include <string>

#include "sensor.hpp"
#include "sensors/backend/ray_caster.hpp"
#include "common/update_scheduler.hpp"
#include "sensors/lidar/lidar_2d_sensor.hpp"  // reuse LaserScanFrame

namespace hako::robots::sensor::lidar
{
    // Configuration for the backend-agnostic (Strategy C) 2D LiDAR.
    struct LidarScanConfig
    {
        std::string frame_id {"lidar"};
        double angle_min_deg {-180.0};
        double angle_max_deg {180.0};
        double angle_increment_deg {1.0};
        double range_min {0.05};
        double range_max {20.0};
        int scan_frequency_hz {10};
    };

    // Backend-independent 2D LiDAR. The ray cast is injected via IRayCaster and
    // the pose is supplied per-scan via SensorState, so the same model runs on
    // MuJoCo (mj_ray) or any other backend -- and, crucially for the A-2 design,
    // the sensor does NOT need to live inside the world being scanned.
    //
    // The scan plane is spanned by the sensor's forward/left axes; angle 0 points
    // along +forward, positive angles rotate toward +left.
    class LidarScanSensor : public ISensor
    {
    public:
        explicit LidarScanSensor(std::shared_ptr<backend::IRayCaster> ray_caster);

        void SetConfig(const LidarScanConfig& config);
        const LidarScanConfig& GetConfig() const;

        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;

        void Scan(const backend::SensorState& state, LaserScanFrame& out);

    private:
        std::shared_ptr<backend::IRayCaster> ray_caster_;
        LidarScanConfig config_ {};
        common::UpdateScheduler scheduler_ {};
        unsigned long scan_count_ {0UL};   // drives LaserScanFrame::stamp_sec
    };
}
