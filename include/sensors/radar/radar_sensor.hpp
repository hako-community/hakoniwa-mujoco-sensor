#pragma once

#include <memory>
#include <random>
#include <string>

#include "sensor.hpp"
#include "sensors/backend/ray_caster.hpp"
#include "common/update_scheduler.hpp"
#include "sensors/noise/noise.hpp"
#include "sensors/radar/radar_types.hpp"

namespace hako::robots::sensor::radar
{
    class IRadarSensor : public ISensor
    {
    public:
        virtual ~IRadarSensor() = default;
        virtual bool LoadConfig(const std::string& config_path) = 0;
        virtual void SetConfig(const RadarConfig& config) = 0;
        virtual const RadarConfig& GetConfig() const = 0;
        // Produce one scan frame given the sensor pose/motion (backend-agnostic).
        virtual void Scan(const backend::SensorState& state, RadarScanFrame& out) = 0;
    };

    // Backend-independent radar sensor model.
    // The ray cast (and therefore the physics/render engine) is injected via
    // IRayCaster, so the very same model runs on MuJoCo (mj_ray) or any other
    // backend that can report a hit point + target velocity.
    class RadarSensor : public IRadarSensor
    {
    public:
        explicit RadarSensor(std::shared_ptr<backend::IRayCaster> ray_caster);

        bool LoadConfig(const std::string& config_path) override;
        void SetConfig(const RadarConfig& config) override;
        const RadarConfig& GetConfig() const override;

        void Reset() override;
        double GetUpdatePeriodSec() const override;
        bool ShouldUpdate(double delta_sec) override;

        void Scan(const backend::SensorState& state, RadarScanFrame& out) override;

    private:
        // Monotonic scan counter -> the timestamp on each frame. Without it a
        // scan that returns nothing is byte-identical to the previous one, and a
        // consumer cannot tell a live sensor from one that has stopped
        // (ISO 15964 4.2 d: fault detection).
        unsigned long scan_count_ {0UL};
        void RebuildNoisePipeline();
        int PointsPerScan() const;

        std::shared_ptr<backend::IRayCaster> ray_caster_;
        RadarConfig config_ {};
        common::UpdateScheduler scheduler_ {};
        noise::RangeNoisePipeline noise_pipeline_;
        std::mt19937 rng_ {1U};
    };
}
