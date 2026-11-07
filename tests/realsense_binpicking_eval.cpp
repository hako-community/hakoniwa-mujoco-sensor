// RS-7: evaluation of the RealSense model on the bin-picking sample scene.
//
// Loads examples/realsense_binpicking/binpicking_scene.xml, lets the 30 work
// pieces settle (3 s), and compares each effect of the RealSense model against
// the ideal depth inside the bin (missing rate, outlier rate, RMS, capture time):
//   D435i at 0.72 m above the pile: stereo error only, + each effect alone,
//   the full preset (config/realsense/d435i_binpicking.json), the preset with the
//   camera moving (hand-eye at 1 rad/s);
//   D405 at 0.37 m: the full preset (config/realsense/d405_binpicking.json).
// The table is printed and written to realsense_binpicking_eval.md (current dir),
// with images realsense_binpicking_{rgb,depth_ideal,depth_full,error}.png.
// Needs HAKO_SENSOR_BUILD_CAMERA=ON, libmujoco and an OpenGL context.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "physics.hpp"
#include "sensors/camera/image_frame_writer.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "sensors/camera/realsense/realsense_camera_sensor.hpp"
#include "sensors/camera/realsense/realsense_depth_metrics.hpp"

namespace cam = hako::robots::sensor::camera;
namespace rs = hako::robots::sensor::camera::realsense;

namespace
{
class FileWorld : public hako::robots::physics::IWorld
{
public:
    void loadModel(const std::string& path) override
    {
        char err[1000] = {0};
        model = mj_loadXML(path.c_str(), nullptr, err, sizeof(err));
        if (!model) {
            std::printf("mj_loadXML failed: %s\n", err);
            return;
        }
        data = mj_makeData(model);
        mj_forward(model, data);
    }
    void advanceTimeStep() override { mj_step(model, data); }
    std::shared_ptr<hako::robots::physics::IRigidBody> getRigidBody(const std::string&) override { return nullptr; }
    std::shared_ptr<hako::robots::actuator::ITorqueActuator> getTorqueActuator(const std::string&) override
    {
        return nullptr;
    }
};

int g_failures = 0;
int g_checks = 0;

void check(bool cond, const std::string& msg)
{
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::printf("  [FAIL] %s\n", msg.c_str());
    }
}

// Ideal depth outside the ROI -> NaN (metrics only inside the bin).
std::vector<float> MaskRoi(const cam::DepthFrame& f, int u0, int u1, int v0, int v1)
{
    std::vector<float> out(f.data.size(), std::numeric_limits<float>::quiet_NaN());
    for (int v = std::max(0, v0); v <= std::min(f.height - 1, v1); ++v)
        for (int u = std::max(0, u0); u <= std::min(f.width - 1, u1); ++u)
            out[static_cast<std::size_t>(v) * f.width + u] = f.data[static_cast<std::size_t>(v) * f.width + u];
    return out;
}

struct Row
{
    std::string name;
    rs::DepthMetrics m;
    double ms {0.0};
    std::string note;
};

// Depth -> colour (near = warm, far = cool), invalid = black.
cam::ImageFrame Colorize(const std::vector<float>& d, int w, int h, double zmin, double zmax)
{
    cam::ImageFrame f {};
    f.width = w;
    f.height = h;
    f.channels = 3;
    f.format = "R8G8B8";
    f.data.assign(static_cast<std::size_t>(w) * h * 3, 0);
    for (std::size_t i = 0; i < d.size(); ++i) {
        if (!std::isfinite(d[i])) continue;
        const double t = std::clamp((d[i] - zmin) / (zmax - zmin), 0.0, 1.0);
        const double r = std::clamp(1.5 - std::fabs(4.0 * t - 1.0), 0.0, 1.0);   // jet-like
        const double g = std::clamp(1.5 - std::fabs(4.0 * t - 2.0), 0.0, 1.0);
        const double b = std::clamp(1.5 - std::fabs(4.0 * t - 3.0), 0.0, 1.0);
        f.data[i * 3] = static_cast<std::uint8_t>(255 * r);
        f.data[i * 3 + 1] = static_cast<std::uint8_t>(255 * g);
        f.data[i * 3 + 2] = static_cast<std::uint8_t>(255 * b);
    }
    return f;
}

// Error map: grey = within 3 sigma, magenta = outlier, black = missing, dark = no truth.
cam::ImageFrame ErrorMap(const std::vector<float>& truth_all, const std::vector<float>& meas, int w, int h,
                         const rs::DepthModelParams& m)
{
    cam::ImageFrame f {};
    f.width = w;
    f.height = h;
    f.channels = 3;
    f.format = "R8G8B8";
    f.data.assign(static_cast<std::size_t>(w) * h * 3, 40);
    for (std::size_t i = 0; i < meas.size(); ++i) {
        std::uint8_t* p = &f.data[i * 3];
        const double zt = truth_all[i];
        if (!std::isfinite(zt)) continue;
        const double zm = meas[i];
        if (!std::isfinite(zm)) {
            p[0] = p[1] = p[2] = 0;
            continue;
        }
        const double thr = std::max(0.002, 3.0 * rs::ExpectedDepthSigma(m, zt));
        if (std::fabs(zm - zt) > thr) {
            p[0] = 255; p[1] = 0; p[2] = 255;
        } else {
            p[0] = p[1] = p[2] = 170;
        }
    }
    return f;
}

void TurnOffEffects(rs::RealSenseConfig& c)
{
    c.edge_effects.enabled = false;
    c.texture_model.enabled = false;
    c.specular_model.enabled = false;
    c.thermal.enabled = false;
    c.exposure.enabled = false;
    c.frame_timing.enabled = false;
}
}

int main()
{
    const std::string src = HAKO_SENSOR_SOURCE_DIR;
    auto world = std::make_shared<FileWorld>();
    world->loadModel(src + "/examples/realsense_binpicking/binpicking_scene.xml");
    if (!world->getModel()) return 1;
    for (int i = 0; i < 3000; ++i) world->advanceTimeStep();   // 3 s: the pile settles
    std::printf("[scene] settled at t = %.2f s, %d bodies\n", world->getData()->time, static_cast<int>(world->getModel()->nbody));
    auto renderer = std::make_shared<cam::MujocoCameraRenderer>(world);

    std::vector<std::string> md;
    const auto table = [&](const std::string& title, const std::vector<Row>& rows) {
        md.push_back("### " + title);
        md.push_back("");
        md.push_back("| 条件 | 欠損率 | 外れ値率 | RMS（外れ値を除く） | 1 回の撮影 | 備考 |");
        md.push_back("|---|---:|---:|---:|---:|---|");
        for (const auto& r : rows) {
            char line[512];
            std::snprintf(line, sizeof(line), "| %s | %.2f%% | %.2f%% | %.2f mm | %.0f ms | %s |", r.name.c_str(),
                          100.0 * r.m.missing_rate, 100.0 * r.m.outlier_rate, 1e3 * r.m.rms_m, r.ms, r.note.c_str());
            md.push_back(line);
        }
        md.push_back("");
    };

    // ---------------- D435i above the bin ----------------
    rs::RealSenseConfig preset {};
    check(rs::LoadRealSenseConfigFromJson(src + "/config/realsense/d435i_binpicking.json", preset),
          "load d435i_binpicking.json");
    rs::RealSenseConfig ideal_c {};
    rs::MakeDefaultConfig("D435i", ideal_c);
    ideal_c.depth = preset.depth;
    ideal_c.depth_model.enabled = false;
    rs::RealSenseCameraSensor ideal(renderer, "rs_bin");
    check(ideal.LoadConfig(ideal_c), "LoadConfig (ideal)");
    cam::DepthFrame truth_f;
    ideal.CaptureDepth(truth_f);
    // bin interior (0.38 x 0.28 m) projected at the pile height (camera 0.72 m above z = 0.03)
    const auto& kp = ideal.GetDepthModelParams();
    const double cx = truth_f.width * 0.5, cy = truth_f.height * 0.5, zr = 0.72;
    const int u0 = static_cast<int>(cx + kp.fx_px * (-0.19 - 0.05) / zr), u1 = static_cast<int>(cx + kp.fx_px * (0.19 - 0.05) / zr);
    const int v0 = static_cast<int>(cy - kp.fx_px * 0.14 / zr), v1 = static_cast<int>(cy + kp.fx_px * 0.14 / zr);
    const auto truth = MaskRoi(truth_f, u0, u1, v0, v1);
    std::printf("[D435i] ROI u %d..%d, v %d..%d\n", u0, u1, v0, v1);
    // the fixture sample shaft (x = 0.42, y = 0, z = 0.154 top) must be in view
    const int us = static_cast<int>(cx + kp.fx_px * (0.42 - 0.05) / (0.75 - 0.154));
    const float zs = truth_f.data[static_cast<std::size_t>(truth_f.height / 2) * truth_f.width + us];
    std::printf("[D435i] sample shaft on the fixture at u %d: depth %.3f m (want ~%.3f)\n", us, zs, 0.75 - 0.154);
    check(std::isfinite(zs) && std::fabs(zs - (0.75 - 0.154)) < 0.01, "fixture sample shaft visible");

    rs::RealSenseConfig base = preset;
    TurnOffEffects(base);
    struct Variant
    {
        std::string name;
        std::function<void(rs::RealSenseConfig&)> edit;
        bool moving;
    };
    const std::vector<Variant> variants {
        {"ステレオ誤差のみ（typical）", [](rs::RealSenseConfig&) {}, false},
        {"＋ 飛び点・影（RS-1）", [](rs::RealSenseConfig& c) { c.edge_effects.enabled = true; }, false},
        {"＋ 材質の穴（RS-2）", [](rs::RealSenseConfig& c) { c.texture_model.enabled = true; }, false},
        {"＋ 白飛び・鏡像（RS-3）", [](rs::RealSenseConfig& c) { c.specular_model.enabled = true; }, false},
        {"＋ 温度：20 °C で冷えた起動（RS-4）", [](rs::RealSenseConfig& c) { c.thermal.enabled = true; }, false},
        {"＋ 露光・ゲイン（RS-5、静止）", [](rs::RealSenseConfig& c) { c.exposure.enabled = true; }, false},
    };
    std::vector<Row> rows;
    rs::DepthMetrics m_base {};
    for (const auto& v : variants) {
        rs::RealSenseConfig c = base;
        v.edit(c);
        rs::RealSenseCameraSensor s(renderer, "rs_bin");
        s.SetMaterialSource(world);
        check(s.LoadConfig(c), "LoadConfig: " + v.name);
        cam::DepthFrame f;
        const auto t0 = std::chrono::steady_clock::now();
        s.CaptureDepth(f);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        Row r {v.name, rs::ComputeDepthMetrics(truth, f.data, s.GetDepthModelParams()), ms, ""};
        if (c.specular_model.enabled) {
            const auto& st = s.GetLastSpecularStats();
            r.note = "白飛び " + std::to_string(st.highlight_holes) + " px、鏡像 " + std::to_string(st.mirror_pixels)
                + " px、光線 " + std::to_string(st.rays);
        }
        if (c.texture_model.enabled) r.note = "穴 " + std::to_string(s.GetLastTextureStats().holes) + " px";
        if (c.thermal.enabled) {
            char b[96];
            std::snprintf(b, sizeof(b), "%.1f °C、視差 %+.3f px", s.GetLastTemperatureC(), s.GetLastDisparityOffsetPx());
            r.note = b;
        }
        if (c.exposure.enabled) {
            char b[96];
            std::snprintf(b, sizeof(b), "露光 %.2f ms、ゲイン %.1f", 1e3 * s.GetLastDepthExposure().exposure_s,
                          s.GetLastDepthExposure().gain);
            r.note = b;
        }
        if (rows.empty()) m_base = r.m;
        rows.push_back(r);
    }
    // full preset, static and moving
    rs::DepthMetrics m_full {};
    rs::SpecularStats full_spec {};
    for (int moving = 0; moving < 2; ++moving) {
        rs::RealSenseCameraSensor s(renderer, "rs_bin");
        s.SetMaterialSource(world);
        check(s.LoadConfig(preset), "LoadConfig: preset");
        if (moving) {
            rs::CameraMotion mo {};
            mo.wx = 1.0;   // hand-eye camera swinging at 1 rad/s
            s.SetCameraMotion(mo);
        }
        cam::DepthFrame f;
        const auto t0 = std::chrono::steady_clock::now();
        s.CaptureDepth(f);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        char b[128];
        std::snprintf(b, sizeof(b), "露光 %.2f ms、白飛び %zu px、鏡像 %zu px", 1e3 * s.GetLastDepthExposure().exposure_s,
                      s.GetLastSpecularStats().highlight_holes, s.GetLastSpecularStats().mirror_pixels);
        Row r {moving ? "全部（プリセット）＋ カメラが 1 rad/s で振れる" : "全部（d435i_binpicking.json）",
               rs::ComputeDepthMetrics(truth, f.data, s.GetDepthModelParams()), ms, b};
        if (!moving) {
            m_full = r.m;
            full_spec = s.GetLastSpecularStats();
        }
        rows.push_back(r);
    }
    table("D435i（箱の上 0.72 m、848×480）", rows);
    {
        // images of the full preset (static)
        rs::RealSenseCameraSensor s(renderer, "rs_bin");
        s.SetMaterialSource(world);
        s.LoadConfig(preset);
        cam::ImageFrame rgb;
        cam::DepthFrame f;
        s.Capture(rgb, f);
        const bool ok = cam::WriteImageFrameToPng(rgb, "realsense_binpicking_rgb.png")
            && cam::WriteImageFrameToPng(Colorize(truth_f.data, f.width, f.height, 0.66, 0.745), "realsense_binpicking_depth_ideal.png")
            && cam::WriteImageFrameToPng(Colorize(f.data, f.width, f.height, 0.66, 0.745), "realsense_binpicking_depth_full.png")
            && cam::WriteImageFrameToPng(ErrorMap(truth_f.data, f.data, f.width, f.height, s.GetDepthModelParams()),
                                         "realsense_binpicking_error.png");
        check(ok, "images written");
    }
    check(m_full.missing_rate > m_base.missing_rate && m_full.outlier_rate > m_base.outlier_rate,
          "full preset: more missing pixels and outliers than the stereo error alone");
    check(full_spec.highlight_holes + full_spec.mirror_pixels > 0, "shiny parts produce highlights / mirror images");

    // ---------------- D405 close to the pile ----------------
    {
        rs::RealSenseConfig p405 {};
        check(rs::LoadRealSenseConfigFromJson(src + "/config/realsense/d405_binpicking.json", p405),
              "load d405_binpicking.json");
        rs::RealSenseConfig ic {};
        rs::MakeDefaultConfig("D405", ic);
        ic.depth = p405.depth;
        ic.depth_model.enabled = false;
        rs::RealSenseCameraSensor id(renderer, "rs_near");
        check(id.LoadConfig(ic), "LoadConfig (D405 ideal)");
        cam::DepthFrame tf;
        id.CaptureDepth(tf);
        const auto& k5 = id.GetDepthModelParams();
        const double c5x = tf.width * 0.5, c5y = tf.height * 0.5, z5 = 0.37;
        const auto t5 = MaskRoi(tf, static_cast<int>(c5x - k5.fx_px * 0.12 / z5), static_cast<int>(c5x + k5.fx_px * 0.12 / z5),
                                static_cast<int>(c5y - k5.fx_px * 0.09 / z5), static_cast<int>(c5y + k5.fx_px * 0.09 / z5));
        std::vector<Row> r5;
        rs::RealSenseConfig b5 = p405;
        TurnOffEffects(b5);
        for (int full = 0; full < 2; ++full) {
            rs::RealSenseCameraSensor s(renderer, "rs_near");
            s.SetMaterialSource(world);
            check(s.LoadConfig(full ? p405 : b5), "LoadConfig: D405");
            cam::DepthFrame f;
            const auto t0 = std::chrono::steady_clock::now();
            s.CaptureDepth(f);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            r5.push_back({full ? "全部（d405_binpicking.json）" : "ステレオ誤差のみ（typical）",
                          rs::ComputeDepthMetrics(t5, f.data, s.GetDepthModelParams()), ms,
                          full ? "プロジェクタなし（パッシブ）、穴 " + std::to_string(s.GetLastTextureStats().holes) + " px" : ""});
        }
        table("D405（山の上 約 0.37 m、848×480、中央 0.24 × 0.18 m）", r5);
        check(r5[1].m.missing_rate > r5[0].m.missing_rate, "D405 preset loses depth on texture-less parts");
    }

    for (const auto& l : md) std::printf("%s\n", l.c_str());
    if (FILE* fp = std::fopen("realsense_binpicking_eval.md", "wb")) {
        for (const auto& l : md) std::fprintf(fp, "%s\n", l.c_str());
        std::fclose(fp);
    }
    std::printf("realsense_binpicking_eval: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
