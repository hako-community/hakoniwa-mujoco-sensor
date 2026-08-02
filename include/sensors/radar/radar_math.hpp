#pragma once

// Pure, backend-independent radar geometry math.
//
// These helpers carry no MuJoCo / Godot / PDU dependency, so they are unit
// testable in isolation. Ray casting plus Doppler relative velocity follows
// CARLA's ray-cast radar; the ray DIRECTION is drawn on the sphere rather than
// on CARLA's projection plane (see RayDirLocal). Everything is expressed in the
// ROS REP-103 sensor-local frame: x = forward, y = left, z = up.

#include <cmath>

#include "primitive_types.hpp"
#include "sensors/radar/radar_types.hpp"

namespace hako::robots::sensor::radar::math
{
    using hako::robots::types::Vector3;

    constexpr double kPi = 3.14159265358979323846;

    inline double Dot(const Vector3& a, const Vector3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline double DegToRad(double deg) { return deg * kPi / 180.0; }

    // Widest angles the FOV can express. A full turn in azimuth, a full sweep
    // pole to pole in elevation.
    constexpr double kMaxHFovDeg = 360.0;
    constexpr double kMaxVFovDeg = 180.0;

    // Direction from an (azimuth, elevation) pair in the sensor-local frame.
    inline Vector3 DirFromAzEl(double az_rad, double el_rad)
    {
        const double ce = std::cos(el_rad);
        return Vector3(ce * std::cos(az_rad),   // x: forward
                       ce * std::sin(az_rad),   // y: left
                       std::sin(el_rad));       // z: up
    }

    // --- ray distributions ---------------------------------------------------
    // Which one is right depends on what the radar is standing in for, so the
    // choice is a configuration item (RadarConfig::ray_distribution) rather than
    // something baked into the sampler.

    // Equal density per degree of azimuth and of elevation. What an az/el raster
    // scan produces, and the default.
    inline Vector3 RayUniformAngle(double u_az, double u_el, double h_rad, double v_rad)
    {
        return DirFromAzEl((u_az - 0.5) * h_rad, (u_el - 0.5) * v_rad);
    }

    // Equal density per steradian. Uniform-in-angle over-samples near the poles,
    // which starts to matter once the elevation FOV is wide; drawing sin(el)
    // uniformly instead spreads the rays evenly over the spherical patch.
    inline Vector3 RayUniformSolidAngle(double u_az, double u_el, double h_rad, double v_rad)
    {
        const double s_lo = std::sin(-0.5 * v_rad);
        const double s_hi = std::sin(0.5 * v_rad);
        const double s = s_lo + (s_hi - s_lo) * u_el;
        return DirFromAzEl((u_az - 0.5) * h_rad,
                           std::asin(std::min(std::max(s, -1.0), 1.0)));
    }

    // Centre-heavy, reproducing the character of the old planar-projection model
    // (a uniform radius on a disk gives density ~ 1/r, so the boresight is
    // sampled far more densely than the edges). Expressed in ANGLE space, so
    // unlike the original it does not diverge past 180 deg. Useful when the
    // point of the run is a long-range detection straight ahead.
    inline Vector3 RayBoresightWeighted(double u_r, double u_theta, double h_rad, double v_rad)
    {
        const double theta = 2.0 * kPi * u_theta;
        return DirFromAzEl(0.5 * h_rad * u_r * std::cos(theta),
                           0.5 * v_rad * u_r * std::sin(theta));
    }

    // Direction (unit, sensor-local frame) of one ray inside the FOV.
    //   u1, u2 : two independent uniforms on [0,1)
    //
    // The direction is drawn on the SPHERE, from an azimuth and an elevation.
    // This replaces the earlier CARLA-style construction, which placed the ray
    // endpoint on a disk spanning +/- tan(FOV/2)*range on a plane at x = range.
    // Two consequences of that model made it the limiting factor for detect-and
    // -avoid work, and both disappear here:
    //
    //   * tan(FOV/2) diverges at 180 deg, so the FOV could not exceed it -- and
    //     in practice not much beyond 150 deg. A 90 deg crossing encounter sits
    //     at a near-constant +/-45 deg bearing, so it stayed outside a forward
    //     sensor's view for the whole encounter. Azimuth can now reach 360 deg.
    //   * the disk sampling was centre-heavy and could not be turned off. It is
    //     now one selectable distribution among three, not the only behaviour.
    inline Vector3 RayDirLocal(double u1, double u2, double hfov_deg, double vfov_deg,
                               RayDistribution dist = RayDistribution::UniformAngle)
    {
        const double h = DegToRad(std::min(std::max(hfov_deg, 0.0), kMaxHFovDeg));
        const double v = DegToRad(std::min(std::max(vfov_deg, 0.0), kMaxVFovDeg));
        switch (dist) {
            case RayDistribution::UniformSolidAngle:  return RayUniformSolidAngle(u1, u2, h, v);
            case RayDistribution::BoresightWeighted:  return RayBoresightWeighted(u1, u2, h, v);
            case RayDistribution::UniformAngle:
            default:                                  return RayUniformAngle(u1, u2, h, v);
        }
    }

    // Probability that a ray which geometrically hit at `range_m` is actually
    // reported. See RadarConfig::detection_reference_range for why this exists.
    // A reference range of 0 (or less) disables the model -> always 1.
    inline double DetectionProbability(double range_m, double ref_range_m, double falloff_exp)
    {
        if (ref_range_m <= 0.0) return 1.0;
        if (range_m <= ref_range_m) return 1.0;
        const double p = std::pow(ref_range_m / range_m, falloff_exp);
        return (p < 0.0) ? 0.0 : ((p > 1.0) ? 1.0 : p);
    }

    // An arbitrary angular window, not necessarily centred on the boresight.
    struct AngularWindow
    {
        double az_start_rad {0.0};
        double az_end_rad {0.0};
        double el_start_rad {0.0};
        double el_end_rad {0.0};

        double az_span() const { return az_end_rad - az_start_rad; }
        double el_span() const { return el_end_rad - el_start_rad; }
        double az_centre() const { return 0.5 * (az_start_rad + az_end_rad); }
        double el_centre() const { return 0.5 * (el_start_rad + el_end_rad); }
    };

    // Resolve the window a config asks for. An explicit azimuth/elevation range
    // wins; otherwise the symmetric FOV is centred on the boresight. Keeping
    // both forms means every existing manifest keeps working unchanged.
    inline AngularWindow WindowOf(const RadarConfig& c)
    {
        AngularWindow w {};
        if (std::isnan(c.azimuth_start_deg) || std::isnan(c.azimuth_end_deg)) {
            const double h = DegToRad(std::min(std::max(c.horizontal_fov_deg, 0.0), kMaxHFovDeg));
            w.az_start_rad = -0.5 * h;
            w.az_end_rad = 0.5 * h;
        } else {
            w.az_start_rad = DegToRad(c.azimuth_start_deg);
            w.az_end_rad = DegToRad(c.azimuth_end_deg);
        }
        if (std::isnan(c.elevation_start_deg) || std::isnan(c.elevation_end_deg)) {
            const double v = DegToRad(std::min(std::max(c.vertical_fov_deg, 0.0), kMaxVFovDeg));
            w.el_start_rad = -0.5 * v;
            w.el_end_rad = 0.5 * v;
        } else {
            w.el_start_rad = DegToRad(c.elevation_start_deg);
            w.el_end_rad = DegToRad(c.elevation_end_deg);
        }
        return w;
    }

    // Ray direction inside an arbitrary window. The distributions are defined
    // relative to the window's own centre, so "boresight-weighted" means
    // weighted towards the middle of whatever sector the sensor is looking at.
    inline Vector3 RayDirWindow(double u1, double u2, const AngularWindow& w,
                                RayDistribution dist = RayDistribution::UniformAngle)
    {
        switch (dist) {
            case RayDistribution::UniformSolidAngle: {
                const double s_lo = std::sin(w.el_start_rad);
                const double s_hi = std::sin(w.el_end_rad);
                const double s = s_lo + (s_hi - s_lo) * u2;
                return DirFromAzEl(w.az_start_rad + w.az_span() * u1,
                                   std::asin(std::min(std::max(s, -1.0), 1.0)));
            }
            case RayDistribution::BoresightWeighted: {
                const double theta = 2.0 * kPi * u2;
                return DirFromAzEl(w.az_centre() + 0.5 * w.az_span() * u1 * std::cos(theta),
                                   w.el_centre() + 0.5 * w.el_span() * u1 * std::sin(theta));
            }
            case RayDistribution::UniformAngle:
            default:
                return DirFromAzEl(w.az_start_rad + w.az_span() * u1,
                                   w.el_start_rad + w.el_span() * u2);
        }
    }

    // Transform a sensor-local direction/vector into the world frame given the
    // sensor's orthonormal basis (unit world vectors).
    inline Vector3 LocalToWorld(
        const Vector3& local, const Vector3& fwd, const Vector3& left, const Vector3& up)
    {
        return Vector3(
            local.x * fwd.x + local.y * left.x + local.z * up.x,
            local.x * fwd.y + local.y * left.y + local.z * up.y,
            local.x * fwd.z + local.y * left.z + local.z * up.z);
    }

    // Project a world-frame vector onto the sensor basis -> sensor-local vector.
    inline Vector3 WorldToLocal(
        const Vector3& world_vec, const Vector3& fwd, const Vector3& left, const Vector3& up)
    {
        return Vector3(Dot(world_vec, fwd), Dot(world_vec, left), Dot(world_vec, up));
    }

    // Convert a sensor-local hit vector to radar polar coordinates.
    //   azimuth   : atan2(left, forward)   (left positive)
    //   elevation : atan2(up, horizontal)
    //   depth     : range to hit (m)
    inline void ToPolar(const Vector3& local, double& azimuth, double& elevation, double& depth)
    {
        depth = local.length();
        azimuth = std::atan2(local.y, local.x);
        elevation = std::atan2(local.z, std::hypot(local.x, local.y));
    }

    // Doppler relative velocity along the ray (world frame).
    // Convention: positive => target receding (range increasing).
    inline double RelativeVelocity(
        const Vector3& target_velocity,
        const Vector3& sensor_velocity,
        const Vector3& dir_unit_world)
    {
        const Vector3 dv(
            target_velocity.x - sensor_velocity.x,
            target_velocity.y - sensor_velocity.y,
            target_velocity.z - sensor_velocity.z);
        return Dot(dv, dir_unit_world);
    }
}
