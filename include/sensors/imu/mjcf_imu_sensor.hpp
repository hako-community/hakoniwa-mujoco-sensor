#pragma once
// =============================================================================
// MjcfImuSensor ― MJCF の <framequat>/<gyro>/<accelerometer> を直読みする IMU
//
// 【なぜ ImuSensor と別に必要か】
// 同ディレクトリの `ImuSensor` は
//     linear_acceleration = (body_vel - prev_body_vel) / dt
// と **body 速度の差分**で加速度を作る。これは物理の加速度であって
// **加速度計の出力ではない**（重力の反力を含まない）。実測では
//     ImuSensor          → 直立静止時 acc ≈ (0,0,0)
//     MJCF accelerometer → 直立静止時 acc ≈ (0,0,+9.81)
// となり、実機の IMU と突き合わせる用途では使えない。
//
// 本クラスは MuJoCo の `sensordata` を直接読むので、
//   - `framequat`     : site 姿勢（w,x,y,z）
//   - `gyro`          : site 座標系の角速度
//   - `accelerometer` : site 座標系の比力（**重力込み**）
// がそのまま出る。上流シミュレータ（limxsdk 系）と同じ値の出どころになる。
//
// 【出所】hakoniwa-humanoid の H5（2026-08-02）で
//   mujoco_plant/sensors/imu/mjcf_imu_source.hpp として実装したものを、
//   H7-5 で本ライブラリへ**昇格**し、ノイズ適用（noise::AxisNoisePipeline）を追加した。
//
// 【設定】`ImuSensor` と同じ JSON を使う（frame_id / update_rate_hz / noise）。
//   `mjcf_binding.runtime_source: "mjcf_sensor"` を目印にするとよい。
//   MJCF センサは**型で引く**（framequat / gyro / accelerometer が各 1 個である前提）。
// =============================================================================
#include <memory>
#include <string>
#include <utility>

#include <mujoco/mujoco.h>

#include "physics.hpp"
#include "sensors/imu/imu_sensor.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor
{
    class MjcfImuSensor
    {
    public:
        explicit MjcfImuSensor(std::shared_ptr<hako::robots::physics::IWorld> world)
            : world_(std::move(world))
            , sensor_(std::make_unique<ImuSensor>(world_))
            , gyro_noise_(noise::AxisNoiseParams {})
            , acc_noise_(noise::AxisNoiseParams {})
        {
        }

        // 設定パースは ImuSensor に委譲（frame_id / update_rate / noise を読む）。
        bool LoadConfig(const std::string& config_path, std::string* error_message = nullptr)
        {
            auto fail = [&](const std::string& m) {
                if (error_message != nullptr) { *error_message = m; }
                return false;
            };
            if (!sensor_->LoadConfig(config_path)) {
                return fail("IMU 設定のロードに失敗: " + config_path);
            }
            const mjModel* model = world_->getModel();
            quat_adr_ = FindSensorAdr(model, mjSENS_FRAMEQUAT, 4);
            gyro_adr_ = FindSensorAdr(model, mjSENS_GYRO, 3);
            acc_adr_ = FindSensorAdr(model, mjSENS_ACCELEROMETER, 3);
            if (quat_adr_ < 0 || gyro_adr_ < 0 || acc_adr_ < 0) {
                return fail("MJCF に framequat / gyro / accelerometer が揃っていません");
            }
            RebuildNoise();
            return true;
        }

        const ImuConfig& GetConfig() const { return sensor_->GetConfig(); }
        double GetUpdatePeriodSec() const { return sensor_->GetUpdatePeriodSec(); }
        bool ShouldUpdate(double delta_sec) { return sensor_->ShouldUpdate(delta_sec); }
        bool HasNoise() const { return has_noise_; }

        void Build(ImuFrame& out)
        {
            const mjtNum* sd = world_->getData()->sensordata;
            out.header.frame_id = GetConfig().frame_id;
            // MJCF framequat は (w,x,y,z)。Quaternion は w/x/y/z のフィールド名を持つので
            // PDU 変換（converter/sensor_msgs/imu.hpp）が名前で写し、
            // sensor_msgs/Imu の (x,y,z,w) 並びは自動的に吸収される。
            out.orientation.w = sd[quat_adr_ + 0];
            out.orientation.x = sd[quat_adr_ + 1];
            out.orientation.y = sd[quat_adr_ + 2];
            out.orientation.z = sd[quat_adr_ + 3];

            noise::AxisValue g {sd[gyro_adr_ + 0], sd[gyro_adr_ + 1], sd[gyro_adr_ + 2]};
            noise::AxisValue a {sd[acc_adr_ + 0], sd[acc_adr_ + 1], sd[acc_adr_ + 2]};
            if (has_noise_) {
                g = gyro_noise_.Apply(g);
                a = acc_noise_.Apply(a);
            }
            out.angular_velocity.x = g.x;
            out.angular_velocity.y = g.y;
            out.angular_velocity.z = g.z;
            out.linear_acceleration.x = a.x;
            out.linear_acceleration.y = a.y;
            out.linear_acceleration.z = a.z;
        }

        void Reset()
        {
            sensor_->Reset();
            gyro_noise_.Reset();
            acc_noise_.Reset();
        }

    private:
        static int FindSensorAdr(const mjModel* model, int type, int dim)
        {
            for (int i = 0; i < model->nsensor; ++i) {
                if (model->sensor_type[i] == type && model->sensor_dim[i] == dim) {
                    return model->sensor_adr[i];
                }
            }
            return -1;
        }

        static noise::NoiseType ParseType(const std::string& v)
        {
            if (v == "gaussian") { return noise::NoiseType::Gaussian; }
            if (v == "gaussian_quantized") { return noise::NoiseType::GaussianQuantized; }
            return noise::NoiseType::None;
        }

        static noise::NoiseParams ToParams(const noise::NoiseModelConfig& c)
        {
            noise::NoiseParams p {};
            p.type = ParseType(c.type);
            p.mean = c.mean;
            p.stddev = c.stddev;
            p.bias_mean = c.bias_mean;
            p.bias_stddev = c.bias_stddev;
            p.dynamic_bias_stddev = c.dynamic_bias_stddev;
            p.dynamic_bias_correlation_time = c.dynamic_bias_correlation_time;
            p.precision = c.precision;
            return p;
        }

        static noise::AxisNoiseParams ToAxis(const noise::AxisNoiseConfig& c, bool& any)
        {
            noise::AxisNoiseParams p {};
            p.x = ToParams(c.x);
            p.y = ToParams(c.y);
            p.z = ToParams(c.z);
            any = any || p.x.type != noise::NoiseType::None
                      || p.y.type != noise::NoiseType::None
                      || p.z.type != noise::NoiseType::None;
            return p;
        }

        void RebuildNoise()
        {
            const auto& n = GetConfig().noise;
            has_noise_ = false;
            const double dt = GetUpdatePeriodSec();
            gyro_noise_ = noise::AxisNoisePipeline(ToAxis(n.angular_velocity, has_noise_), dt);
            acc_noise_ = noise::AxisNoisePipeline(ToAxis(n.linear_acceleration, has_noise_), dt);
        }

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        std::unique_ptr<ImuSensor> sensor_;
        int quat_adr_ {-1};
        int gyro_adr_ {-1};
        int acc_adr_ {-1};
        bool has_noise_ {false};
        noise::AxisNoisePipeline gyro_noise_;
        noise::AxisNoisePipeline acc_noise_;
    };
}
