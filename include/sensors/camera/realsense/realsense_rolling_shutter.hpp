#pragma once

// Rolling-shutter distortion from camera ego-motion (pure, no MuJoCo/GL).
//
// A rolling-shutter sensor exposes row r at t_r = t_mid + (r - (H-1)/2) * line_time.
// The renderer gives one global-shutter image at t_mid (the reference). For each
// output pixel of row r we take the camera pose at t_r relative to the reference,
//     R_r = exp([w] * dt),  p_r = v * dt        (w, v in the camera optical frame)
// cast the pixel ray from that pose and sample the reference image where the ray
// lands. With depth available (depth image itself, or an aligned depth) the
// translation is included (one fixed-point step using the reference depth);
// otherwise only rotation is applied, which is the dominant term for handheld /
// robot-mounted cameras.
//
// Optical frame: x right, y down, z forward (ROS *_optical_frame).
// Nearest-neighbour sampling; samples outside the reference become invalid
// (NaN for depth, 0 for colour).

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace hako::robots::sensor::camera::realsense
{
    struct PinholeIntrinsics
    {
        double fx {0.0};
        double fy {0.0};
        double cx {0.0};
        double cy {0.0};
    };

    struct CameraMotion
    {
        double wx {0.0}, wy {0.0}, wz {0.0};   // angular velocity [rad/s], optical frame
        double vx {0.0}, vy {0.0}, vz {0.0};   // linear velocity [m/s], optical frame
    };

    namespace detail
    {
        struct Mat3
        {
            double m[3][3];
        };

        // Rodrigues: exp([w] * dt)
        inline Mat3 RotationFromAngularVelocity(const CameraMotion& mo, double dt)
        {
            const double ax = mo.wx * dt, ay = mo.wy * dt, az = mo.wz * dt;
            const double th = std::sqrt(ax * ax + ay * ay + az * az);
            Mat3 r {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
            if (th < 1e-12) return r;
            const double kx = ax / th, ky = ay / th, kz = az / th;
            const double c = std::cos(th), s = std::sin(th), t = 1.0 - c;
            r.m[0][0] = c + kx * kx * t;      r.m[0][1] = kx * ky * t - kz * s; r.m[0][2] = kx * kz * t + ky * s;
            r.m[1][0] = ky * kx * t + kz * s; r.m[1][1] = c + ky * ky * t;      r.m[1][2] = ky * kz * t - kx * s;
            r.m[2][0] = kz * kx * t - ky * s; r.m[2][1] = kz * ky * t + kx * s; r.m[2][2] = c + kz * kz * t;
            return r;
        }

        inline void Mul(const Mat3& r, const double in[3], double out[3])
        {
            for (int i = 0; i < 3; ++i) out[i] = r.m[i][0] * in[0] + r.m[i][1] * in[1] + r.m[i][2] * in[2];
        }

        inline void MulT(const Mat3& r, const double in[3], double out[3])
        {
            for (int i = 0; i < 3; ++i) out[i] = r.m[0][i] * in[0] + r.m[1][i] * in[1] + r.m[2][i] * in[2];
        }

        inline double RowDt(int row, int height, double line_time)
        {
            return (static_cast<double>(row) - 0.5 * static_cast<double>(height - 1)) * line_time;
        }
    }

    inline bool HasMotion(const CameraMotion& m)
    {
        return m.wx != 0.0 || m.wy != 0.0 || m.wz != 0.0 || m.vx != 0.0 || m.vy != 0.0 || m.vz != 0.0;
    }

    // Colour / mono image (interleaved uint8). `ref_depth` (same size, metres,
    // NaN invalid) is optional and enables the translation term.
    inline void ApplyRollingShutterImage(
        std::vector<std::uint8_t>& img, int width, int height, int channels,
        const PinholeIntrinsics& k, const CameraMotion& motion, double readout_s,
        const std::vector<float>* ref_depth = nullptr)
    {
        if (width <= 0 || height <= 1 || channels <= 0 || readout_s <= 0.0 || !HasMotion(motion)) return;
        if (img.size() != static_cast<std::size_t>(width) * height * channels) return;
        const bool use_depth = ref_depth && ref_depth->size() == static_cast<std::size_t>(width) * height;
        const double line_time = readout_s / static_cast<double>(height);
        const std::vector<std::uint8_t> src = img;
        for (int r = 0; r < height; ++r) {
            const double dt = detail::RowDt(r, height, line_time);
            const auto rot = detail::RotationFromAngularVelocity(motion, dt);
            const double p[3] = {motion.vx * dt, motion.vy * dt, motion.vz * dt};
            for (int x = 0; x < width; ++x) {
                const double ray[3] = {(x - k.cx) / k.fx, (r - k.cy) / k.fy, 1.0};
                double q[3];
                detail::Mul(rot, ray, q);
                if (use_depth) {
                    const float z = (*ref_depth)[static_cast<std::size_t>(r) * width + x];
                    if (std::isfinite(z) && z > 0.0F) {
                        for (int i = 0; i < 3; ++i) q[i] = q[i] * z + p[i];
                    }
                }
                std::uint8_t* dst = &img[(static_cast<std::size_t>(r) * width + x) * channels];
                if (q[2] <= 0.0) {
                    for (int c = 0; c < channels; ++c) dst[c] = 0;
                    continue;
                }
                const long u = std::lround(k.fx * q[0] / q[2] + k.cx);
                const long v = std::lround(k.fy * q[1] / q[2] + k.cy);
                if (u < 0 || v < 0 || u >= width || v >= height) {
                    for (int c = 0; c < channels; ++c) dst[c] = 0;
                    continue;
                }
                const std::uint8_t* s = &src[(static_cast<std::size_t>(v) * width + u) * channels];
                for (int c = 0; c < channels; ++c) dst[c] = s[c];
            }
        }
    }

    // Depth image (z-depth metres, NaN invalid). Translation is always included.
    inline void ApplyRollingShutterDepth(
        std::vector<float>& depth, int width, int height,
        const PinholeIntrinsics& k, const CameraMotion& motion, double readout_s)
    {
        if (width <= 0 || height <= 1 || readout_s <= 0.0 || !HasMotion(motion)) return;
        if (depth.size() != static_cast<std::size_t>(width) * height) return;
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        const double line_time = readout_s / static_cast<double>(height);
        const std::vector<float> src = depth;
        auto sample = [&](long u, long v) {
            if (u < 0 || v < 0 || u >= width || v >= height) return nan_val;
            return src[static_cast<std::size_t>(v) * width + u];
        };
        for (int r = 0; r < height; ++r) {
            const double dt = detail::RowDt(r, height, line_time);
            const auto rot = detail::RotationFromAngularVelocity(motion, dt);
            const double p[3] = {motion.vx * dt, motion.vy * dt, motion.vz * dt};
            for (int x = 0; x < width; ++x) {
                const double ray[3] = {(x - k.cx) / k.fx, (r - k.cy) / k.fy, 1.0};
                double d[3];
                detail::Mul(rot, ray, d);
                float& out = depth[static_cast<std::size_t>(r) * width + x];
                // First guess of the row-time depth: the reference depth at this pixel.
                float z_guess = src[static_cast<std::size_t>(r) * width + x];
                if (!(std::isfinite(z_guess) && z_guess > 0.0F)) {
                    // No depth here: rotation-only lookup.
                    if (d[2] <= 0.0) { out = nan_val; continue; }
                    z_guess = sample(std::lround(k.fx * d[0] / d[2] + k.cx), std::lround(k.fy * d[1] / d[2] + k.cy));
                    if (!(std::isfinite(z_guess) && z_guess > 0.0F)) { out = nan_val; continue; }
                }
                const double q[3] = {d[0] * z_guess + p[0], d[1] * z_guess + p[1], d[2] * z_guess + p[2]};
                if (q[2] <= 0.0) { out = nan_val; continue; }
                const long u = std::lround(k.fx * q[0] / q[2] + k.cx);
                const long v = std::lround(k.fy * q[1] / q[2] + k.cy);
                const float zs = sample(u, v);
                if (!(std::isfinite(zs) && zs > 0.0F)) { out = nan_val; continue; }
                // Point seen in the reference, expressed in the row-time camera.
                const double pr[3] = {(u - k.cx) / k.fx * zs - p[0], (v - k.cy) / k.fy * zs - p[1], zs - p[2]};
                double qr[3];
                detail::MulT(rot, pr, qr);
                out = qr[2] > 0.0 ? static_cast<float>(qr[2]) : nan_val;
            }
        }
    }
}
