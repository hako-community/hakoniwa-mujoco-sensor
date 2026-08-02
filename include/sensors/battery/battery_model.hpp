#pragma once
// =============================================================================
// BatteryModel ― 電流積算 + 放電曲線によるバッテリ状態モデル
//
// 【設計の出所】hakoniwa-drone-pro の `src/aircraft/impl/battery/battery_dynamics.hpp`
//   の**設計を参考**にした（放電曲線テーブル + 電流積算 + 温度係数の構造）。
//   ライセンスは LicenseRef-hakoniwalab-NC で本ライブラリ（別ライセンス）とは異なるため、
//   **コードは持ち込まず、考え方だけを取り入れて独自に実装**している。
//   - 参考にした点: ①時間ベースの単純減算ではなく**電流ベースの容量積算**
//                   ②容量→端子電圧の**放電曲線テーブルを外部化して補間**
//   - 変えた点: 電流源をロータ電流ではなく **関節トルク由来**にした（脚型ロボット向け）
//               温度係数は初版では扱わない（構造だけ残す）
//
// 【物理シミュレーションではない】発熱・内部抵抗の詳細は扱わない。
//   「残量 80% でも高負荷時に電圧が落ちる」を再現するための実用モデル。
//
// 消費電流の近似:
//     I = I_idle + Σ_i k_i * |τ_i * ω_i| / V
//   （機械出力を電圧で割って電流に換算し、効率 k で割り増しする）
// =============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace hako::robots::sensor
{
    struct BatteryDischargePoint
    {
        double remaining_ratio {0.0};   // 残容量 [0..1]
        double voltage {0.0};           // 端子電圧 [V]
    };

    struct BatteryConfig
    {
        double capacity_mah {9500.0};       // 定格容量 [mAh]
        double nominal_voltage {48.0};      // 公称電圧 [V]
        double full_voltage {54.6};         // 満充電電圧 [V]
        double cutoff_voltage {42.0};       // 遮断電圧 [V]
        double idle_current_a {1.5};        // 待機電流 [A]
        double efficiency {0.75};           // 電気→機械の総合効率
        double initial_ratio {1.0};         // 開始時の残量 [0..1]
        double update_rate_hz {1.0};
        std::vector<BatteryDischargePoint> curve {};   // 残量→電圧（降順/昇順どちらでも）
    };

    struct BatteryState
    {
        double full_voltage {0.0};
        double curr_voltage {0.0};
        double temperature {25.0};
        double remaining_ratio {1.0};
        double current_a {0.0};
        std::uint32_t status {0};       // 0=OK / 1=LOW(<20%) / 2=CRITICAL(<5%)
        std::uint32_t cycles {0};
    };

    class BatteryModel
    {
    public:
        void Configure(const BatteryConfig& config)
        {
            config_ = config;
            if (config_.curve.empty()) {
                // 既定の放電曲線（リチウムイオンの典型形。満充電付近と終端で急峻）
                config_.curve = {
                    {1.00, config_.full_voltage},
                    {0.90, config_.nominal_voltage + 4.2},
                    {0.70, config_.nominal_voltage + 2.4},
                    {0.40, config_.nominal_voltage + 0.6},
                    {0.20, config_.nominal_voltage - 1.2},
                    {0.10, config_.nominal_voltage - 3.0},
                    {0.00, config_.cutoff_voltage},
                };
            }
            std::sort(config_.curve.begin(), config_.curve.end(),
                      [](const auto& a, const auto& b) {
                          return a.remaining_ratio < b.remaining_ratio;
                      });
            consumed_mah_ = config_.capacity_mah * (1.0 - config_.initial_ratio);
        }

        const BatteryConfig& GetConfig() const { return config_; }

        // dt 秒ぶん進める。joint_torque / joint_velocity は同じ長さの配列。
        void Update(double dt_sec,
                    const std::vector<double>& joint_torque,
                    const std::vector<double>& joint_velocity)
        {
            const double v = CurrentVoltage();
            double mech_w = 0.0;
            const std::size_t n = std::min(joint_torque.size(), joint_velocity.size());
            for (std::size_t i = 0; i < n; ++i) {
                mech_w += std::abs(joint_torque[i] * joint_velocity[i]);
            }
            const double eff = (config_.efficiency > 0.0) ? config_.efficiency : 1.0;
            current_a_ = config_.idle_current_a + (v > 0.0 ? (mech_w / eff) / v : 0.0);
            consumed_mah_ += current_a_ * 1000.0 * (dt_sec / 3600.0);
            consumed_mah_ = std::clamp(consumed_mah_, 0.0, config_.capacity_mah);
        }

        BatteryState GetState() const
        {
            BatteryState s {};
            s.full_voltage = config_.full_voltage;
            s.remaining_ratio = Remaining();
            s.curr_voltage = CurrentVoltage();
            s.current_a = current_a_;
            s.temperature = 25.0;   // 温度は初版では扱わない（構造だけ残す）
            s.status = (s.remaining_ratio < 0.05) ? 2u
                     : (s.remaining_ratio < 0.20) ? 1u : 0u;
            s.cycles = 0u;
            return s;
        }

        double Remaining() const
        {
            if (config_.capacity_mah <= 0.0) { return 0.0; }
            return std::clamp(1.0 - consumed_mah_ / config_.capacity_mah, 0.0, 1.0);
        }

        // 残量 → 端子電圧（放電曲線の線形補間）。負荷が大きいほど電圧が下がる
        // （内部抵抗の代わりに電流に比例した降下を載せる簡易表現）。
        double CurrentVoltage() const
        {
            const double r = Remaining();
            const auto& c = config_.curve;
            double v = c.empty() ? config_.nominal_voltage : c.back().voltage;
            if (!c.empty()) {
                if (r <= c.front().remaining_ratio) {
                    v = c.front().voltage;
                } else if (r >= c.back().remaining_ratio) {
                    v = c.back().voltage;
                } else {
                    for (std::size_t i = 1; i < c.size(); ++i) {
                        if (r <= c[i].remaining_ratio) {
                            const double t = (r - c[i - 1].remaining_ratio)
                                / std::max(c[i].remaining_ratio - c[i - 1].remaining_ratio, 1e-9);
                            v = c[i - 1].voltage + t * (c[i].voltage - c[i - 1].voltage);
                            break;
                        }
                    }
                }
            }
            // 負荷による電圧降下（内部抵抗 ≒ 0.02Ω 相当の簡易表現）
            return std::max(v - current_a_ * 0.02, 0.0);
        }

    private:
        BatteryConfig config_ {};
        double consumed_mah_ {0.0};
        double current_a_ {0.0};
    };
}
