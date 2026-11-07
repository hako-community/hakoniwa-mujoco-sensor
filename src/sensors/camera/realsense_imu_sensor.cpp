#include "sensors/camera/realsense/realsense_imu_sensor.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

#include <mujoco/mujoco.h>

namespace hako::robots::sensor::camera::realsense
{
namespace
{
// MuJoCo camera frame (x right, y up, z backward) -> optical (x right, y down, z forward).
Vec3d ToOptical(const mjtNum* v)
{
    return {static_cast<double>(v[0]), -static_cast<double>(v[1]), -static_cast<double>(v[2])};
}
}

RealSenseImuSensor::RealSenseImuSensor(std::shared_ptr<hako::robots::physics::IWorld> world,
                                       const std::string& camera_name)
    : world_(std::move(world)), camera_name_(camera_name)
{
    if (!world_) {
        throw std::invalid_argument("World is null");
    }
}

bool RealSenseImuSensor::LoadConfig(const std::string& path)
{
    RealSenseConfig config {};
    if (!LoadRealSenseConfigFromJson(path, config)) {
        return false;
    }
    return LoadConfig(config);
}

bool RealSenseImuSensor::LoadConfig(const RealSenseConfig& config)
{
    const std::string reason = ValidateConfig(config);
    if (!reason.empty()) {
        std::cerr << "Invalid RealSense IMU config (" << config.profile.model << "): " << reason << std::endl;
        return false;
    }
    if (CameraId() < 0) {
        std::cerr << "RealSense IMU: camera '" << camera_name_ << "' not found in the model" << std::endl;
        return false;
    }
    config_ = config;
    model_.Configure(config_.profile.imu, config_.imu);
    thermal_.Configure(MakeThermalParams(config_));
    timing_.Configure(config_.imu_timing.params);
    dropped_accel_ = 0;
    dropped_gyro_ = 0;
    Reset();
    return true;
}

void RealSenseImuSensor::Reset()
{
    accel_clock_ = 0.0;
    gyro_clock_ = 0.0;
    has_accel_ = false;
    has_gyro_ = false;
}

int RealSenseImuSensor::CameraId() const
{
    const mjModel* m = world_->getModel();
    return m ? mj_name2id(m, mjOBJ_CAMERA, camera_name_.c_str()) : -1;
}

bool RealSenseImuSensor::TruthGyro(Vec3d& out) const
{
    const mjModel* m = world_->getModel();
    const mjData* d = world_->getData();
    const int id = CameraId();
    if (!m || !d || id < 0) return false;
    mjtNum res[6];
    mj_objectVelocity(m, d, mjOBJ_CAMERA, id, res, 1);
    out = ToOptical(res);
    return true;
}

bool RealSenseImuSensor::TruthAccel(Vec3d& out) const
{
    const mjModel* m = world_->getModel();
    mjData* d = world_->getData();
    const int id = CameraId();
    if (!m || !d || id < 0) return false;
    // cacc is filled by mj_step only when the model needs it (e.g. an accelerometer
    // sensor); recompute it for the current state.
    mj_rnePostConstraint(m, d);
    mjtNum res[6];
    mj_objectAcceleration(m, d, mjOBJ_CAMERA, id, res, 1);
    out = ToOptical(res + 3);
    return true;
}

bool RealSenseImuSensor::TruthAngularAccel(Vec3d& out) const
{
    const mjModel* m = world_->getModel();
    mjData* d = world_->getData();
    const int id = CameraId();
    if (!m || !d || id < 0) return false;
    mj_rnePostConstraint(m, d);
    mjtNum res[6];
    mj_objectAcceleration(m, d, mjOBJ_CAMERA, id, res, 1);
    out = ToOptical(res);
    return true;
}

void RealSenseImuSensor::Update(double delta_sec, std::vector<MotionSample>& accel_out,
                                std::vector<MotionSample>& gyro_out)
{
    if (!Enabled() || delta_sec <= 0.0) return;
    const mjData* d = world_->getData();
    const double now = d ? static_cast<double>(d->time) : 0.0;

    const double accel_period = 1.0 / config_.imu.accel_rate_hz;
    const double gyro_period = 1.0 / config_.imu.gyro_rate_hz;
    accel_clock_ += delta_sec;
    gyro_clock_ += delta_sec;

    const bool thermal = config_.thermal.enabled;
    const bool timing = config_.imu_timing.enabled;
    const auto plus = [](const Vec3d& a, const Vec3d& b) { return Vec3d {a.x + b.x, a.y + b.y, a.z + b.z}; };
    Vec3d truth {};
    if (accel_clock_ >= accel_period && TruthAccel(truth)) {
        if (timing && timing_.HasLeverArm()) {
            Vec3d w {};
            Vec3d alpha {};
            if (TruthGyro(w) && TruthAngularAccel(alpha)) truth = LeverArmAccel(truth, w, alpha, timing_.Position());
        }
        if (timing) truth = timing_.Misalign(truth);
        if (thermal) truth = plus(truth, thermal_.AccelDrift(now));
        while (accel_clock_ >= accel_period) {
            accel_clock_ -= accel_period;
            if (timing && timing_.Drop()) {
                ++dropped_accel_;
                continue;
            }
            const double t = now - accel_clock_;
            last_accel_ = {timing ? timing_.AccelStamp(t) : t, model_.MeasureAccel(truth)};
            accel_out.push_back(last_accel_);
            has_accel_ = true;
        }
    }
    if (gyro_clock_ >= gyro_period && TruthGyro(truth)) {
        if (timing) truth = timing_.Misalign(truth);
        if (thermal) truth = plus(truth, thermal_.GyroDrift(now));
        while (gyro_clock_ >= gyro_period) {
            gyro_clock_ -= gyro_period;
            if (timing && timing_.Drop()) {
                ++dropped_gyro_;
                continue;
            }
            const double t = now - gyro_clock_;
            last_gyro_ = {timing ? timing_.GyroStamp(t) : t, model_.MeasureGyro(truth)};
            gyro_out.push_back(last_gyro_);
            has_gyro_ = true;
        }
    }
}

bool RealSenseImuSensor::GetLatest(hako::robots::sensor::ImuFrame& out) const
{
    if (!has_accel_ || !has_gyro_) return false;
    out.header.stamp_sec = last_gyro_.timestamp;
    out.header.frame_id = FrameId();
    out.orientation = {};
    out.angular_velocity = {last_gyro_.value.x, last_gyro_.value.y, last_gyro_.value.z};
    out.linear_acceleration = {last_accel_.value.x, last_accel_.value.y, last_accel_.value.z};
    return true;
}

}
