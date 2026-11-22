#pragma once

// FANUC iRVision 2D カメラ（ロボットの手首に付ける 2D ビジョンセンサー）の機種プロファイル。
//
// 既存の CameraSensor（MuJoCo 描画、L8 / R8G8B8 出力、内部パラメータ fx・fy・cx・cy）を
// そのまま使い、ここでは「カタログの値から、描画に渡す設定値を決める」部分だけを持つ。
// MuJoCo・PDU には依存しない（nlohmann/json だけ）。
//
// 情報源と確かさ（docs/physical_ai/irvision_2d_camera.md に詳しく書く）:
//   - 公式（FANUC iRVision の仕様表）: グレースケール 1280×1024 / カラー 640×512、
//     レンズ 8・12・16・25 mm、LED 照明 赤・白・なし、外形 80×131.8×74 mm、0.6 kg。
//   - 推定: 撮像素子は 1/1.8 型（販売店の記載）。画素ピッチは 1/1.8 型の公称の横幅 7.2 mm を
//     1280 画素で割った 5.625 µm を既定にする（実機の型番が分かったら差し替える）。
//   - 推定: カラー版は同じ光学サイズを 640×512 で使う（画素ピッチ 2 倍）として扱う。
//   - 不明: フレームレート。既定の更新周期は 10 Hz とし、設定で変えられるようにする。

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace hako::robots::sensor::camera::irvision
{

enum class ImageType
{
    Grayscale,
    Color,
};

enum class LedColor
{
    None,
    Red,
    White,
};

// カメラ本体の外形。光軸は長さ（length）の向き。前面（窓）が width × height。
struct Housing
{
    double width_mm {80.0};
    double height_mm {74.0};
    double length_mm {131.8};
    double mass_kg {0.6};
};

struct IrvisionProfile
{
    std::string model {"FANUC iRVision 2D"};
    ImageType image_type {ImageType::Grayscale};
    double focal_length_mm {8.0};
    double pixel_pitch_um {5.625};
    int width_px {1280};
    int height_px {1024};
    LedColor led {LedColor::Red};
    double update_rate_hz {10.0};
    double clip_near_m {0.03};
    double clip_far_m {3.0};
    Housing housing {};
};

constexpr double kPi = 3.14159265358979323846;

inline bool IsCatalogLens(double focal_length_mm)
{
    for (double f : {8.0, 12.0, 16.0, 25.0}) {
        if (std::fabs(focal_length_mm - f) < 1e-9) return true;
    }
    return false;
}

// カタログの組み合わせ（画像の種類 × レンズ × LED）から作る。
inline IrvisionProfile MakeProfile(ImageType image_type, double focal_length_mm, LedColor led)
{
    if (!IsCatalogLens(focal_length_mm)) {
        throw std::invalid_argument("iRVision 2D lens must be 8, 12, 16 or 25 mm");
    }
    IrvisionProfile p {};
    p.image_type = image_type;
    p.focal_length_mm = focal_length_mm;
    p.led = led;
    if (image_type == ImageType::Color) {
        p.width_px = 640;
        p.height_px = 512;
        p.pixel_pitch_um = 11.25;  // 同じ光学サイズを 640×512 で使う（推定）
    }
    return p;
}

inline double SensorWidthMm(const IrvisionProfile& p) { return p.width_px * p.pixel_pitch_um * 1e-3; }
inline double SensorHeightMm(const IrvisionProfile& p) { return p.height_px * p.pixel_pitch_um * 1e-3; }

// ピンホールの焦点距離（画素単位）。正方画素なので fx = fy。
inline double FocalLengthPx(const IrvisionProfile& p) { return p.focal_length_mm / (p.pixel_pitch_um * 1e-3); }

inline double HorizontalFovRad(const IrvisionProfile& p)
{
    return 2.0 * std::atan(SensorWidthMm(p) / (2.0 * p.focal_length_mm));
}

inline double VerticalFovRad(const IrvisionProfile& p)
{
    return 2.0 * std::atan(SensorHeightMm(p) / (2.0 * p.focal_length_mm));
}

// 光軸に垂直な平面までの距離 distance_m のとき、写る範囲（横, 縦）[m]。
inline std::pair<double, double> FieldOfViewAt(const IrvisionProfile& p, double distance_m)
{
    return {distance_m * SensorWidthMm(p) / p.focal_length_mm, distance_m * SensorHeightMm(p) / p.focal_length_mm};
}

// LED の色（MuJoCo の light の diffuse、USD の light の color に使う 0〜1 の RGB）。
// 赤は 625 nm 付近の LED を sRGB で近似した値。
inline std::array<double, 3> LedRgb(LedColor led)
{
    switch (led) {
        case LedColor::Red: return {1.0, 0.08, 0.02};
        case LedColor::White: return {1.0, 1.0, 1.0};
        case LedColor::None: break;
    }
    return {0.0, 0.0, 0.0};
}

inline const char* ToString(ImageType t) { return t == ImageType::Color ? "color" : "grayscale"; }

inline const char* ToString(LedColor led)
{
    switch (led) {
        case LedColor::Red: return "red";
        case LedColor::White: return "white";
        case LedColor::None: break;
    }
    return "none";
}

// 既存の CameraSensor が読む JSON（camera_config_loader の "spec" 形式）を作る。
// "irvision" の項目は CameraSensor は読まない（MJCF・USD を作る側と、確認ツールが使う）。
inline nlohmann::json ToCameraSpecJson(const IrvisionProfile& p, const std::string& frame_id,
                                       const std::string& camera_name)
{
    const double f_px = FocalLengthPx(p);
    nlohmann::json spec = {
        {"frame_id", frame_id},
        {"update_rate", p.update_rate_hz},
        {"horizontal_fov", HorizontalFovRad(p)},
        {"image", {{"width", p.width_px}, {"height", p.height_px},
                   {"format", p.image_type == ImageType::Color ? "R8G8B8" : "L8"}}},
        {"clip", {{"near", p.clip_near_m}, {"far", p.clip_far_m}}},
        {"intrinsics", {{"fx", f_px}, {"fy", f_px}, {"cx", p.width_px / 2.0}, {"cy", p.height_px / 2.0}}},
    };
    const auto led_rgb = LedRgb(p.led);
    nlohmann::json irvision = {
        {"model", p.model},
        {"image_type", ToString(p.image_type)},
        {"focal_length_mm", p.focal_length_mm},
        {"pixel_pitch_um", p.pixel_pitch_um},
        {"sensor_size_mm", {SensorWidthMm(p), SensorHeightMm(p)}},
        {"vertical_fov", VerticalFovRad(p)},
        {"led", {{"color", ToString(p.led)}, {"rgb", led_rgb}}},
        {"housing_mm", {{"width", p.housing.width_mm}, {"height", p.housing.height_mm},
                        {"length", p.housing.length_mm}}},
        {"mass_kg", p.housing.mass_kg},
    };
    return {{"spec", spec}, {"mjcf_binding", {{"camera_name", camera_name}}}, {"irvision", irvision}};
}

}  // namespace hako::robots::sensor::camera::irvision
