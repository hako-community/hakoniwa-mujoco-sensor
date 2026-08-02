#pragma once
// =============================================================================
// ContactSensor ― MuJoCo ネイティブ touch センサ（mjSENS_TOUCH）の薄ラッパー
//
//   複数の touch センサの法線力を合算し、しきい値で接触 ON/OFF（Bool）を判定する。
//   物理（接触力計算）は MuJoCo に任せ、本クラスは sensordata 読み出し＋判定のみ。
//   read の定石: mj_name2id(SENSOR) → model->sensor_adr → data->sensordata[adr]。
//
// 【出所】hakoniwa-armpi の mujoco_plant/sensors/contact/contact_sensor.hpp（S4 で実装・動作実績あり）
//         を 2026-08-02（hakoniwa-humanoid H7）に本ライブラリへ**昇格**した。
//         変更点は namespace（armpi::sensor → sensor）と include パス（M1 のレイアウト）のみ。
//
// 【重要な使い方】mjSENS_TOUCH は「接触 geom の一方が site の所属 body」でないと拾えない。
//   足裏なら足の collision geom を持つ body（*_ankle_roll_link）に site を置くこと。
// =============================================================================

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include "physics.hpp"
#include "common/update_scheduler.hpp"

namespace hako::robots::sensor
{
    struct ContactConfig
    {
        std::vector<std::string> touch_sensors {};  // MJCF touch センサ名（指先ごと）
        double threshold_n {0.5};                    // 接触判定の力しきい値 [N]
        double update_rate_hz {30.0};
    };

    class ContactSensor
    {
    public:
        explicit ContactSensor(std::shared_ptr<hako::robots::physics::IWorld> world)
            : world_(std::move(world)) {}

        // config JSON（spec.touch_sensors / spec.threshold_n / spec.update_rate）を読む。
        bool LoadConfig(const std::string& path)
        {
            std::ifstream ifs(path);
            if (!ifs.is_open()) { return false; }
            nlohmann::json root;
            try { ifs >> root; } catch (...) { return false; }
            const auto& spec = root.contains("spec") ? root.at("spec") : root;
            config_ = ContactConfig {};
            if (spec.contains("touch_sensors") && spec.at("touch_sensors").is_array()) {
                for (const auto& s : spec.at("touch_sensors")) {
                    config_.touch_sensors.push_back(s.get<std::string>());
                }
            }
            if (spec.contains("threshold_n") && spec.at("threshold_n").is_number()) {
                config_.threshold_n = spec.at("threshold_n").get<double>();
            }
            if (spec.contains("update_rate") && spec.at("update_rate").is_number()) {
                config_.update_rate_hz = spec.at("update_rate").get<double>();
            }
            ResolveAddresses();
            scheduler_.StartReady(GetUpdatePeriodSec());
            return !config_.touch_sensors.empty();
        }

        const ContactConfig& GetConfig() const { return config_; }
        double GetUpdatePeriodSec() const
        {
            return (config_.update_rate_hz > 0.0) ? (1.0 / config_.update_rate_hz) : 0.1;
        }
        bool ShouldUpdate(double delta_sec)
        {
            return scheduler_.ShouldUpdate(delta_sec, GetUpdatePeriodSec());
        }

        // 全 touch センサの法線力を合算 → out_force。閾値超で out_contact=true。
        void Build(bool& out_contact, double& out_force)
        {
            if (addr_.empty()) { ResolveAddresses(); }
            const mjData* data = world_->getData();
            double total = 0.0;
            for (const int a : addr_) {
                if (a >= 0) { total += data->sensordata[a]; }
            }
            out_force = total;
            out_contact = (total > config_.threshold_n);
        }

    private:
        void ResolveAddresses()
        {
            addr_.clear();
            const mjModel* model = world_->getModel();
            for (const auto& name : config_.touch_sensors) {
                const int id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
                addr_.push_back(id >= 0 ? model->sensor_adr[id] : -1);
            }
        }

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        ContactConfig config_ {};
        std::vector<int> addr_ {};  // touch_sensors と同順の sensordata アドレス
        hako::robots::common::UpdateScheduler scheduler_ {};
    };
}
