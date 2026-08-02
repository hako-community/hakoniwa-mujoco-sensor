#pragma once
// =============================================================================
// ForceTorqueSensor ― MuJoCo ネイティブ force/torque センサの薄ラッパー
//
//   mjSENS_FORCE / mjSENS_TORQUE の sensordata を読み、3D 力・トルク（site 座標系）を返す。
//   物理（反力計算）は MuJoCo に任せ、本クラスは読み出し＋ノイズ付与のみ。
//
//   ノイズ: noise::GaussianNoiseModel を流用し、白色 stddev に加え
//   **静的バイアス bias_stddev / 動的バイアス（Gauss-Markov）** を扱う
//   ＝バイアス安定性/アラン分散を表現できる。従来の force_stddev/torque_stddev
//   （白色のみ）も後方互換で受ける。
//
// 【出所】hakoniwa-armpi の mujoco_plant/sensors/force_torque/force_torque_sensor.hpp
//         （S4 実装 + S6 ノイズ拡張・動作実績あり）を 2026-08-02（hakoniwa-humanoid H7）
//         に本ライブラリへ**昇格**した。変更点は namespace と include パスのみ。
// =============================================================================

#include <array>
#include <fstream>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include "physics.hpp"
#include "common/update_scheduler.hpp"
#include "sensors/noise/noise.hpp"

namespace hako::robots::sensor
{
    namespace nz = hako::robots::sensor::noise;

    struct ForceTorqueConfig
    {
        std::string frame_id {"end_effector_link"};
        std::string force_sensor {"wrist_force"};
        std::string torque_sensor {"wrist_torque"};
        double update_rate_hz {30.0};
        double force_stddev {0.0};    // 後方互換: 白色 stddev（0=無ノイズ）
        double torque_stddev {0.0};
        nz::NoiseParams force_noise {};   // 白色＋静的/動的バイアス（force 3軸共通）
        nz::NoiseParams torque_noise {};  // 同上（torque 3軸共通）
        bool noisy {false};               // いずれかのノイズ項が有効
    };

    class ForceTorqueSensor
    {
    public:
        explicit ForceTorqueSensor(std::shared_ptr<hako::robots::physics::IWorld> world)
            : world_(std::move(world)) {}

        bool LoadConfig(const std::string& path)
        {
            std::ifstream ifs(path);
            if (!ifs.is_open()) { return false; }
            nlohmann::json root;
            try { ifs >> root; } catch (...) { return false; }
            const auto& spec = root.contains("spec") ? root.at("spec") : root;
            config_ = ForceTorqueConfig {};
            auto str = [&](const char* k, const std::string& d) {
                return (spec.contains(k) && spec.at(k).is_string())
                    ? spec.at(k).get<std::string>() : d;
            };
            config_.frame_id = str("frame_id", config_.frame_id);
            config_.force_sensor = str("force_sensor", config_.force_sensor);
            config_.torque_sensor = str("torque_sensor", config_.torque_sensor);
            if (spec.contains("update_rate") && spec.at("update_rate").is_number()) {
                config_.update_rate_hz = spec.at("update_rate").get<double>();
            }
            if (spec.contains("noise") && spec.at("noise").is_object()) {
                const auto& n = spec.at("noise");
                // (a) 後方互換: スカラ force_stddev / torque_stddev（白色のみ）。
                config_.force_noise = ParseScalar(n, "force_stddev");
                config_.torque_noise = ParseScalar(n, "torque_stddev");
                // (b) S6: リッチ指定 force:{...} / torque:{...}（バイアス安定性含む）。
                if (n.contains("force") && n.at("force").is_object()) {
                    config_.force_noise = ParseNoiseObject(n.at("force"));
                }
                if (n.contains("torque") && n.at("torque").is_object()) {
                    config_.torque_noise = ParseNoiseObject(n.at("torque"));
                }
            }
            config_.force_stddev = config_.force_noise.stddev;
            config_.torque_stddev = config_.torque_noise.stddev;
            config_.noisy = HasNoise(config_.force_noise) || HasNoise(config_.torque_noise);

            // 動的バイアスの相関時間に合わせ dt=更新周期でモデルを構築（per-axis 独立状態）。
            const double dt = GetUpdatePeriodSec();
            for (auto& m : force_models_)  { m = nz::CreateNoiseModel(config_.force_noise.type, dt); }
            for (auto& m : torque_models_) { m = nz::CreateNoiseModel(config_.torque_noise.type, dt); }

            ResolveAddresses();
            scheduler_.StartReady(GetUpdatePeriodSec());
            return force_adr_ >= 0 && torque_adr_ >= 0;
        }

        const ForceTorqueConfig& GetConfig() const { return config_; }
        double GetUpdatePeriodSec() const
        {
            return (config_.update_rate_hz > 0.0) ? (1.0 / config_.update_rate_hz) : 0.1;
        }
        bool ShouldUpdate(double delta_sec)
        {
            return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
        }

        // force[3], torque[3]（site 座標系）を取得。noisy なら白色＋バイアスを付与。
        void Build(std::array<double, 3>& force, std::array<double, 3>& torque)
        {
            if (force_adr_ < 0 || torque_adr_ < 0) { ResolveAddresses(); }
            const mjData* data = world_->getData();
            for (int i = 0; i < 3; ++i) {
                force[i]  = data->sensordata[force_adr_ + i];
                torque[i] = data->sensordata[torque_adr_ + i];
            }
            if (config_.force_noise.type != nz::NoiseType::None) {
                for (int i = 0; i < 3; ++i) {
                    force[i] = force_models_[i]->Apply(force[i], config_.force_noise);
                }
            }
            if (config_.torque_noise.type != nz::NoiseType::None) {
                for (int i = 0; i < 3; ++i) {
                    torque[i] = torque_models_[i]->Apply(torque[i], config_.torque_noise);
                }
            }
        }

    private:
        static bool HasNoise(const nz::NoiseParams& p)
        {
            return p.stddev > 0.0 || p.bias_stddev > 0.0 || p.dynamic_bias_stddev > 0.0
                || p.bias_mean != 0.0 || p.mean != 0.0;
        }

        // スカラ "<key>"（白色 stddev のみ）→ NoiseParams。
        static nz::NoiseParams ParseScalar(const nlohmann::json& n, const char* key)
        {
            nz::NoiseParams p {};
            if (n.contains(key) && n.at(key).is_number()) {
                p.stddev = n.at(key).get<double>();
                if (p.stddev > 0.0) { p.type = nz::NoiseType::Gaussian; }
            }
            return p;
        }

        // オブジェクト {stddev,bias_mean,bias_stddev,dynamic_bias_stddev,
        // dynamic_bias_correlation_time,mean} → NoiseParams（バイアス安定性含む）。
        static nz::NoiseParams ParseNoiseObject(const nlohmann::json& j)
        {
            nz::NoiseParams p {};
            auto num = [&](const char* k, double d) {
                return (j.contains(k) && j.at(k).is_number()) ? j.at(k).get<double>() : d;
            };
            p.mean = num("mean", 0.0);
            p.stddev = num("stddev", 0.0);
            p.bias_mean = num("bias_mean", 0.0);
            p.bias_stddev = num("bias_stddev", 0.0);
            p.dynamic_bias_stddev = num("dynamic_bias_stddev", 0.0);
            p.dynamic_bias_correlation_time = num("dynamic_bias_correlation_time", 0.0);
            p.type = HasNoise(p) ? nz::NoiseType::Gaussian : nz::NoiseType::None;
            return p;
        }

        void ResolveAddresses()
        {
            const mjModel* model = world_->getModel();
            const int fid = mj_name2id(model, mjOBJ_SENSOR, config_.force_sensor.c_str());
            const int tid = mj_name2id(model, mjOBJ_SENSOR, config_.torque_sensor.c_str());
            force_adr_ = (fid >= 0) ? model->sensor_adr[fid] : -1;
            torque_adr_ = (tid >= 0) ? model->sensor_adr[tid] : -1;
        }

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        ForceTorqueConfig config_ {};
        int force_adr_ {-1};
        int torque_adr_ {-1};
        std::array<std::unique_ptr<nz::INoiseModel>, 3> force_models_ {};
        std::array<std::unique_ptr<nz::INoiseModel>, 3> torque_models_ {};
        hako::robots::common::UpdateScheduler scheduler_ {};
    };
}
