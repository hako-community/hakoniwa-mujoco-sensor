#pragma once

// RealSense IMU (D435i / D435if / D436 / D455 family / D555 / D585 Pro) on MuJoCo.
//
// Truth is taken at the MJCF camera used for depth (the IMU is rigidly mounted
// in the camera housing; the small lever arm to the IMU chip is ignored):
//   gyro  : mj_objectVelocity(mjOBJ_CAMERA, local)         angular part
//   accel : mj_objectAcceleration(mjOBJ_CAMERA, local)     linear part
//           (MuJoCo's specific force: includes -gravity, like the MJCF accelerometer)
// converted to the optical frame (x right, y down, z forward) and passed through
// ImuNoiseModel. Accel and gyro run at their own rates (62.5/250 Hz, 200/400 Hz);
// samples falling in one physics step share that step's state, so the physics
// timestep should be <= the gyro period.
//
// Does not need GL: built into the main library (links libmujoco at the consumer).

#include <memory>
#include <string>
#include <vector>

#include "physics.hpp"
#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_imu_model.hpp"
#include "sensors/imu/imu_sensor.hpp"

namespace hako::robots::sensor::camera::realsense
{
    struct MotionSample
    {
        double timestamp {0.0};
        Vec3d value {};          // accel m/s^2 or gyro rad/s, optical frame
    };

    class RealSenseImuSensor
    {
    public:
        RealSenseImuSensor(std::shared_ptr<hako::robots::physics::IWorld> world, const std::string& camera_name);

        bool LoadConfig(const std::string& path);
        bool LoadConfig(const RealSenseConfig& config);
        const RealSenseConfig& GetConfig() const { return config_; }
        bool Enabled() const { return config_.ImuEnabled(); }
        std::string FrameId() const { return config_.frame_id + "_imu_optical_frame"; }

        void Reset();

        // Advances the internal clocks by delta_sec and appends the samples due.
        void Update(double delta_sec, std::vector<MotionSample>& accel_out, std::vector<MotionSample>& gyro_out);

        // Latest gyro sample combined with the latest accel sample
        // (librealsense "unite_imu_method: copy"), for sensor_msgs/Imu.
        // Orientation is identity: the RealSense IMU does not output attitude.
        bool GetLatest(hako::robots::sensor::ImuFrame& out) const;

        // Noise-free values at the current state (optical frame).
        bool TruthAccel(Vec3d& out) const;
        bool TruthGyro(Vec3d& out) const;

        const ImuNoiseModel& NoiseModel() const { return model_; }

        // RS-4 thermal zero-offset drift (thermal.enabled): added to the truth
        // before the noise model. The IMU keeps its own ThermalModel (same config
        // -> same temperature curve as the camera when both start together).
        ThermalModel& GetThermalModel() { return thermal_; }
        // IMU calibration at sim_time: the current temperature becomes T_cal.
        void RunImuCalibration(double sim_time) { thermal_.RunOnChipCalibration(sim_time); }

    private:
        int CameraId() const;

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        std::string camera_name_;
        RealSenseConfig config_ {};
        ImuNoiseModel model_ {};
        ThermalModel thermal_ {};
        double accel_clock_ {0.0};
        double gyro_clock_ {0.0};
        bool has_accel_ {false};
        bool has_gyro_ {false};
        MotionSample last_accel_ {};
        MotionSample last_gyro_ {};
    };
}
