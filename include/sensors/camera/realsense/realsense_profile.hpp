#pragma once

// RealSense D400 / D500 series camera profiles.
//
// Pure data: no MuJoCo, GL, PDU or JSON dependency, so the table can be used by
// unit tests, tools and other backends.
//
// Source: "RealSense Overview (ja) 2026-09" (devai_sensor/RealSense-Overview-ja-2026-09.pdf),
//   p.19 D400 comparison table, p.25-26 D585 Pro, p.29 D421, p.38 D436, p.39 D455, p.43 D555.
// Values that the PDF does not state are marked "assumed" in the comment of each
// entry (mainly the stereo baseline of some models and the max range). Every value
// can be overridden from the JSON config (see realsense_config.hpp).
//
// Public RealSense / Bosch information used in addition to the PDF ("web"):
//   - Intel "Tuning depth cameras for best performance": depth RMS error
//     = z^2 * subpixel / (fx * B); subpixel RMS of a well-calibrated unit < 0.1 px,
//     0.08 px used for the published curves, ~0.05 px on well textured targets.
//     MinZ scales linearly with the X resolution (D415 1280 px: ~43.8 cm,
//     D435 848 px: ~16.8 cm) -> disparity search range fx*B/MinZ = 126 px.
//   - Intel "Self-calibration for D400": D415 baseline ~55 mm, D435 ~50 mm.
//   - D435i / D455 IMU: Bosch BMI055 (later BMI085, PCN 118035-00), accel
//     62.5/250 Hz, gyro 200/400 Hz, gyro +-2000 dps, accel 12-bit (BMI055) /
//     16-bit (BMI085); typical noise density accel 150 ug/sqrt(Hz), gyro
//     0.014 dps/sqrt(Hz) (Bosch Sensortec community).

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace hako::robots::sensor::camera::realsense
{
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    enum class Shutter
    {
        Global,
        Rolling,
    };

    inline const char* ToString(Shutter s)
    {
        return s == Shutter::Global ? "global" : "rolling";
    }

    // Inertial measurement unit of the camera (D435i, D435if, D436, D455 family, D555, D585 Pro).
    struct ImuSpec
    {
        std::string chip {"BMI055"};
        std::vector<double> accel_rates_hz {62.5, 250.0};
        std::vector<double> gyro_rates_hz {200.0, 400.0};
        double accel_range_g {4.0};                 // assumed (configurable)
        double gyro_range_dps {2000.0};
        int accel_bits {12};
        int gyro_bits {16};
        double accel_noise_density_ug {150.0};      // ug/sqrt(Hz), typical (Bosch)
        double gyro_noise_density_dps {0.014};      // dps/sqrt(Hz), typical (Bosch)
        double accel_zero_offset_mg {70.0};         // +- uncalibrated zero-g offset (BMI055 datasheet)
        double gyro_zero_offset_dps {1.0};          // +- uncalibrated zero-rate offset (BMI055 datasheet)
        double gyro_bias_instability_dph {2.0};     // deg/h (BMI085 closed-loop gyro, same as BMI055)
    };

    struct RealSenseProfile
    {
        std::string model {};          // e.g. "D435i"
        std::string family {};         // e.g. "D435" (comparison-table column)

        // --- Depth (active / passive stereo, reported from the left imager) ---
        double depth_hfov_deg {87.0};  // horizontal FOV at HD (vertical follows aspect)
        double depth_vfov_deg {58.0};  // catalogue value, informative only
        Shutter depth_shutter {Shutter::Global};
        int depth_max_width {1280};
        int depth_max_height {720};
        double depth_max_fps {90.0};   // at reduced resolution
        double baseline_m {0.050};     // stereo baseline
        bool active_projector {true};  // false = passive stereo (D405)

        // Depth accuracy "ratio @ distance" (e.g. 2% @ 2 m), measured at the
        // reference width (max depth width). Used as 1-sigma depth error.
        double accuracy_ratio {0.02};
        double accuracy_ref_distance_m {2.0};
        int accuracy_ref_width {1280};

        double min_z_m {0.12};         // Min-Z catalogue value (used with min_z_mode "fixed")
        double max_z_m {10.0};         // usable max range (beyond: invalid)

        // "Typical" noise model (web: Intel tuning white paper).
        double subpixel_rms_px {0.08};      // disparity RMS of a well-calibrated unit
        double max_disparity_px {126.0};    // D4 disparity search range -> MinZ = fx*B/126
        double rolling_readout_s {0.0};     // rolling-shutter frame readout, 0 = global

        // --- Color ---
        bool has_rgb {true};
        double rgb_hfov_deg {69.0};
        double rgb_vfov_deg {42.0};
        Shutter rgb_shutter {Shutter::Rolling};
        int rgb_default_width {1280};
        int rgb_default_height {720};
        double rgb_max_fps {30.0};

        double rgb_rolling_readout_s {0.0}; // rolling RGB readout time, 0 = global

        // --- Others ---
        bool has_imu {false};
        ImuSpec imu {};
        std::string ir_filter {"none"};  // "none" / "ir_cut" / "ir_pass" / "band_pass"
        bool ip65 {false};
        std::string interface {"USB-C"};
        bool preliminary {false};        // true = values partly assumed (pre-release)
        std::string note {};

        double DepthHfovRad() const { return depth_hfov_deg * kDegToRad; }
        double RgbHfovRad() const { return rgb_hfov_deg * kDegToRad; }
    };

    namespace detail
    {
        inline RealSenseProfile D435Base()
        {
            RealSenseProfile p {};
            p.family = "D435";
            p.depth_hfov_deg = 87.0;
            p.depth_vfov_deg = 58.0;
            p.depth_shutter = Shutter::Global;
            p.depth_max_fps = 90.0;
            p.baseline_m = 0.050;           // web: Intel self-calibration paper (~50 mm)
            p.accuracy_ratio = 0.02;        // PDF p.19: 2% @ 2 m
            p.accuracy_ref_distance_m = 2.0;
            p.min_z_m = 0.12;               // PDF p.19: ~12 cm
            p.max_z_m = 10.0;               // assumed
            p.has_rgb = true;
            p.rgb_hfov_deg = 69.0;          // PDF p.19: 2MP 69x42 rolling
            p.rgb_vfov_deg = 42.0;
            p.rgb_shutter = Shutter::Rolling;
            p.rgb_rolling_readout_s = 0.025; // assumed (not published)
            p.rgb_default_width = 1280;     // 2MP sensor; HD stream as sim default
            p.rgb_default_height = 720;
            p.rgb_max_fps = 30.0;
            return p;
        }

        inline RealSenseProfile D455Base()
        {
            RealSenseProfile p {};
            p.family = "D455";
            p.depth_hfov_deg = 87.0;
            p.depth_vfov_deg = 58.0;
            p.depth_shutter = Shutter::Global;
            p.depth_max_fps = 90.0;
            p.baseline_m = 0.095;           // PDF p.39: 95 mm
            p.accuracy_ratio = 0.02;        // PDF p.19 / p.39: 2% @ 4 m
            p.accuracy_ref_distance_m = 4.0;
            p.min_z_m = 0.20;               // PDF p.19: ~20 cm
            p.max_z_m = 10.0;               // assumed
            p.has_rgb = true;
            p.rgb_hfov_deg = 90.0;          // PDF p.19: 1MP 90x65 global
            p.rgb_vfov_deg = 65.0;
            p.rgb_shutter = Shutter::Global;
            p.rgb_default_width = 1280;
            p.rgb_default_height = 800;
            p.rgb_max_fps = 30.0;
            p.has_imu = true;
            return p;
        }

        inline std::vector<RealSenseProfile> BuildProfiles()
        {
            std::vector<RealSenseProfile> v;

            {   // D421: entry-level module, no RGB (PDF p.19, p.28-29)
                RealSenseProfile p {};
                p.model = "D421";
                p.family = "D421";
                p.depth_hfov_deg = 75.0;
                p.depth_vfov_deg = 50.0;
                p.depth_shutter = Shutter::Global;
                p.depth_max_fps = 90.0;
                p.baseline_m = 0.050;       // PDF p.29
                p.accuracy_ratio = 0.04;    // 4% @ 2 m
                p.accuracy_ref_distance_m = 2.0;
                p.min_z_m = 0.14;           // ~14 cm
                p.max_z_m = 4.0;            // PDF: "0.2 m to over 3 m" -> 4 m assumed
                p.has_rgb = false;
                p.ir_filter = "band_pass";
                p.interface = "USB-C";
                p.note = "Entry level module, indoor/outdoor, 0-35 C";
                v.push_back(p);
            }
            {   // D405: short range, passive stereo, RGB via the left imager (PDF p.19, p.30)
                RealSenseProfile p {};
                p.model = "D405";
                p.family = "D405";
                p.depth_hfov_deg = 87.0;
                p.depth_vfov_deg = 58.0;
                p.depth_shutter = Shutter::Global;
                p.depth_max_fps = 90.0;
                p.baseline_m = 0.018;       // assumed (datasheet value; not in PDF)
                p.active_projector = false;
                p.accuracy_ratio = 0.014;   // 1.4% @ 20 cm
                p.accuracy_ref_distance_m = 0.20;
                p.min_z_m = 0.07;           // Min-Z 7 cm (sub-mm)
                p.max_z_m = 0.50;           // PDF p.21: Short Range < 50 cm
                p.has_rgb = true;
                p.rgb_hfov_deg = 87.0;      // 1MP 87x58 global (left imager)
                p.rgb_vfov_deg = 58.0;
                p.rgb_shutter = Shutter::Global;
                p.rgb_default_width = 1280;
                p.rgb_default_height = 720;
                p.rgb_max_fps = 30.0;
                p.ir_filter = "ir_cut";
                p.note = "Pick & place, defect inspection, robot arm (D401 module: same optics)";
                v.push_back(p);
            }
            {   // D415: mid range, rolling shutter, standard FOV (PDF p.19, p.32)
                RealSenseProfile p {};
                p.model = "D415";
                p.family = "D415";
                p.depth_hfov_deg = 65.0;
                p.depth_vfov_deg = 40.0;
                p.depth_shutter = Shutter::Rolling;
                p.rolling_readout_s = 0.025; // assumed (not published)
                p.depth_max_fps = 90.0;
                p.baseline_m = 0.055;       // web: Intel self-calibration paper (~55 mm)
                p.accuracy_ratio = 0.02;    // 2% @ 2 m
                p.accuracy_ref_distance_m = 2.0;
                p.min_z_m = 0.18;           // ~18 cm
                p.max_z_m = 10.0;           // assumed
                p.has_rgb = true;
                p.rgb_hfov_deg = 69.0;      // 2MP 69x42 rolling
                p.rgb_vfov_deg = 42.0;
                p.rgb_shutter = Shutter::Rolling;
                p.rgb_rolling_readout_s = 0.025; // assumed (not published)
                p.rgb_max_fps = 30.0;
                p.note = "Scanning, interaction (D410/D415 module: same optics)";
                v.push_back(p);
            }
            {
                auto p = D435Base();
                p.model = "D435";
                p.note = "Robotics, mid/long range collision avoidance (D430 module: no RGB)";
                v.push_back(p);
            }
            {
                auto p = D435Base();
                p.model = "D435i";
                p.has_imu = true;
                p.note = "D435 + IMU";
                v.push_back(p);
            }
            {
                auto p = D435Base();
                p.model = "D435f";
                p.ir_filter = "ir_pass";
                p.note = "D435 + IR pass filter";
                v.push_back(p);
            }
            {
                auto p = D435Base();
                p.model = "D435if";
                p.has_imu = true;
                p.ir_filter = "ir_pass";
                p.note = "D435 + IMU + IR pass filter";
                v.push_back(p);
            }
            {   // D436: D435 depth + 1MP global RGB with matched FOV + IMU (PDF p.23, p.37-38)
                auto p = D435Base();
                p.model = "D436";
                p.family = "D436";
                p.min_z_m = 0.20;           // PDF p.38: Min-Z 20 cm
                p.rgb_hfov_deg = 90.0;      // 1MP 90x65 global
                p.rgb_vfov_deg = 65.0;
                p.rgb_shutter = Shutter::Global;
                p.rgb_rolling_readout_s = 0.0;
                p.rgb_default_width = 1280; // up to 1280x800 @ 60 fps
                p.rgb_default_height = 800;
                p.rgb_max_fps = 60.0;
                p.has_imu = true;
                p.note = "Color & depth matching, fast moving robots; 0-50 C";
                v.push_back(p);
            }
            {
                auto p = D455Base();
                p.model = "D455";
                p.note = "Long range, matched depth/RGB FOV (D450 module: same optics)";
                v.push_back(p);
            }
            {
                auto p = D455Base();
                p.model = "D455f";
                p.ir_filter = "ir_pass";
                p.note = "D455 + IR pass filter";
                v.push_back(p);
            }
            {
                auto p = D455Base();
                p.model = "D456";
                p.ip65 = true;
                p.note = "D455 based, IP65, USB";
                v.push_back(p);
            }
            {
                auto p = D455Base();
                p.model = "D457";
                p.ip65 = true;
                p.interface = "GMSL2/FAKRA";
                p.note = "D455 based, IP65, GMSL/FAKRA";
                v.push_back(p);
            }
            {   // D555: Vision SoC V5, PoE (PDF p.19, p.42-43)
                auto p = D455Base();
                p.model = "D555";
                p.family = "D555";
                p.min_z_m = 0.26;           // PDF p.43: 26 cm (VGA)
                p.rgb_default_width = 1280; // up to 1280x800 @ 60 fps
                p.rgb_default_height = 800;
                p.rgb_max_fps = 60.0;
                p.ip65 = true;
                p.interface = "PoE(RJ45)/USB-C";
                p.note = "Vision SoC V5, PoE, IP65, -20 to 50 C, DDS";
                v.push_back(p);
            }
            {   // D585 Pro: Gen5 SoC, announced (PDF p.25-26). Partly assumed.
                RealSenseProfile p {};
                p.model = "D585Pro";
                p.family = "D585Pro";
                p.depth_hfov_deg = 120.0;   // PDF p.25: 120 x 100
                p.depth_vfov_deg = 100.0;
                p.depth_shutter = Shutter::Global;
                p.depth_max_width = 1280;   // PDF p.26: 60 fps @ 1280x960
                p.depth_max_height = 960;
                p.depth_max_fps = 60.0;
                p.baseline_m = 0.095;       // assumed (not published)
                p.accuracy_ratio = 0.01;    // assumed: "2x depth quality" vs D455 (2% @ 4 m)
                p.subpixel_rms_px = 0.04;   // assumed: "2x depth quality" (PDF p.25)
                p.accuracy_ref_distance_m = 4.0;
                p.accuracy_ref_width = 1280;
                p.min_z_m = 0.10;           // PDF p.25: 10 cm - 10 m+
                p.max_z_m = 10.0;
                p.has_rgb = true;
                p.rgb_hfov_deg = 120.0;     // assumed: matched to depth
                p.rgb_vfov_deg = 100.0;
                p.rgb_shutter = Shutter::Global;
                p.rgb_default_width = 1280;
                p.rgb_default_height = 960;
                p.rgb_max_fps = 30.0;       // PDF p.26: dual RGB mode 30 fps RGB + 30 fps depth
                p.has_imu = true;           // PDF p.26: vSLAM (wide FOV + GS + IMU)
                p.ip65 = true;              // PDF p.26: IP65 standard
                p.interface = "GMSL2/USB-C";
                p.preliminary = true;
                p.note = "Pre-release; baseline, accuracy and RGB FOV are assumed";
                v.push_back(p);
            }
            return v;
        }
    }

    inline const std::vector<RealSenseProfile>& BuiltinProfiles()
    {
        static const std::vector<RealSenseProfile> profiles = detail::BuildProfiles();
        return profiles;
    }

    inline std::string NormalizeModelName(const std::string& name)
    {
        std::string out;
        for (char c : name) {
            if (c == ' ' || c == '-' || c == '_') continue;
            out.push_back(static_cast<char>((c >= 'a' && c <= 'z') ? c - 'a' + 'A' : c));
        }
        return out;
    }

    // Case/space-insensitive lookup ("d435i", "D585 Pro" ...). Optional "RealSense " prefix.
    inline std::optional<RealSenseProfile> FindProfile(const std::string& model)
    {
        std::string key = NormalizeModelName(model);
        const std::string prefix = "REALSENSE";
        if (key.rfind(prefix, 0) == 0) key = key.substr(prefix.size());
        for (const auto& p : BuiltinProfiles()) {
            if (NormalizeModelName(p.model) == key) return p;
        }
        return std::nullopt;
    }

    inline std::vector<std::string> BuiltinModelNames()
    {
        std::vector<std::string> names;
        for (const auto& p : BuiltinProfiles()) names.push_back(p.model);
        return names;
    }
}
