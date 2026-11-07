#pragma once

// IMU / camera timing and IMU mounting errors (RS-6). Pure: no MuJoCo.
//
//   timestamps : IMU sample time += time_offset_s + N(0, jitter_s), kept monotonic
//                per stream (VIO / SLAM are sensitive to a few ms of offset).
//   drops      : IMU samples / camera frames are lost with drop_probability.
//   lever arm  : the IMU chip sits at r (optical frame, from the depth camera
//                origin). On a rotating body it feels
//                  a_imu = a_cam + alpha x r + w x (w x r)
//                (gyro is the same everywhere on a rigid body).
//   misalignment: the IMU axes are rotated by a small fixed rotation R_mis
//                (each axis angle ~ N(0, misalignment_deg), drawn once per run).
//
// Defaults are OFF (0). Typical values to try (ASSUMPTIONS): jitter 0.1 ms with
// hardware timestamps / 1 ms with system time, misalignment 0.5 deg, lever arm a
// few cm (read the real unit's depth -> IMU extrinsics with rs-enumerate-devices -c).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "sensors/camera/realsense/realsense_imu_model.hpp"
#include "sensors/camera/realsense/realsense_rolling_shutter.hpp"

namespace hako::robots::sensor::camera::realsense
{
    struct ImuTimingParams
    {
        double time_offset_s {0.0};
        double jitter_s {0.0};
        double drop_probability {0.0};
        double position_m[3] {0.0, 0.0, 0.0};   // IMU chip in the depth optical frame
        double misalignment_deg {0.0};
        std::uint32_t seed {0};
    };

    struct FrameTimingParams
    {
        double jitter_s {0.0};
        double drop_probability {0.0};
        std::uint32_t seed {0};
    };

    inline Vec3d Cross(const Vec3d& a, const Vec3d& b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }

    // Specific force at the IMU chip from the one at the camera origin.
    inline Vec3d LeverArmAccel(const Vec3d& a_cam, const Vec3d& w, const Vec3d& alpha, const Vec3d& r)
    {
        const Vec3d t = Cross(alpha, r);
        const Vec3d c = Cross(w, Cross(w, r));
        return {a_cam.x + t.x + c.x, a_cam.y + t.y + c.y, a_cam.z + t.z + c.z};
    }

    class ImuTimingModel
    {
    public:
        void Configure(const ImuTimingParams& p)
        {
            p_ = p;
            rng_.seed(p.seed);
            const double s = p.misalignment_deg * kDegToRad;
            CameraMotion m {};
            if (s > 0.0) {
                std::normal_distribution<double> n(0.0, s);
                m.wx = n(rng_);
                m.wy = n(rng_);
                m.wz = n(rng_);
            }
            rot_ = detail::RotationFromAngularVelocity(m, 1.0);
            last_accel_t_ = -1e300;
            last_gyro_t_ = -1e300;
        }

        const ImuTimingParams& Params() const { return p_; }
        bool HasLeverArm() const { return p_.position_m[0] != 0.0 || p_.position_m[1] != 0.0 || p_.position_m[2] != 0.0; }
        Vec3d Position() const { return {p_.position_m[0], p_.position_m[1], p_.position_m[2]}; }

        // Misalignment rotation applied to a measured vector (IMU axes vs camera axes).
        Vec3d Misalign(const Vec3d& v) const
        {
            const double in[3] = {v.x, v.y, v.z};
            double out[3];
            detail::Mul(rot_, in, out);
            return {out[0], out[1], out[2]};
        }
        double MisalignmentAngleRad() const
        {
            const double tr = rot_.m[0][0] + rot_.m[1][1] + rot_.m[2][2];
            return std::acos(std::clamp((tr - 1.0) * 0.5, -1.0, 1.0));
        }

        bool Drop()
        {
            if (p_.drop_probability <= 0.0) return false;
            return uniform_(rng_) < p_.drop_probability;
        }

        double AccelStamp(double t) { return Stamp(t, last_accel_t_); }
        double GyroStamp(double t) { return Stamp(t, last_gyro_t_); }

    private:
        double Stamp(double t, double& last)
        {
            double s = t + p_.time_offset_s;
            if (p_.jitter_s > 0.0) s += p_.jitter_s * normal_(rng_);
            s = std::max(s, last + 1e-7);   // monotonic per stream
            last = s;
            return s;
        }

        ImuTimingParams p_ {};
        detail::Mat3 rot_ {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        std::mt19937 rng_ {};
        std::normal_distribution<double> normal_ {0.0, 1.0};
        std::uniform_real_distribution<double> uniform_ {0.0, 1.0};
        double last_accel_t_ {-1e300};
        double last_gyro_t_ {-1e300};
    };

    class FrameTimingModel
    {
    public:
        void Configure(const FrameTimingParams& p)
        {
            p_ = p;
            rng_.seed(p.seed);
            last_ = -1e300;
        }
        bool Enabled() const { return p_.jitter_s > 0.0 || p_.drop_probability > 0.0; }
        bool Drop() { return p_.drop_probability > 0.0 && uniform_(rng_) < p_.drop_probability; }
        double Stamp(double t)
        {
            double s = t;
            if (p_.jitter_s > 0.0) s += p_.jitter_s * normal_(rng_);
            s = std::max(s, last_ + 1e-7);
            last_ = s;
            return s;
        }

    private:
        FrameTimingParams p_ {};
        std::mt19937 rng_ {};
        std::normal_distribution<double> normal_ {0.0, 1.0};
        std::uniform_real_distribution<double> uniform_ {0.0, 1.0};
        double last_ {-1e300};
    };
}
