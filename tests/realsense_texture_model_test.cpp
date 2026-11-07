// Unit test for RS-2: material table and weak-pattern holes (texture model).
// Backend-free: no MuJoCo runtime, GL or PDU.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_depth_model.hpp"
#include "sensors/camera/realsense/realsense_texture_model.hpp"

namespace rs = hako::robots::sensor::camera::realsense;

static int g_failures = 0;
static int g_checks = 0;
static const double kPi = 3.14159265358979323846;

static void check(bool cond, const std::string& msg)
{
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::printf("  [FAIL] %s\n", msg.c_str());
    }
}

static void close_to(double a, double b, double tol, const std::string& msg)
{
    ++g_checks;
    const bool ok = std::fabs(a - b) <= tol;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s (got %.5f, want %.5f +- %.5f)\n", ok ? "ok" : "FAIL", msg.c_str(), a, b, tol);
}

static rs::TextureModelParams Params(int w, int h)
{
    rs::TextureModelParams p {};
    p.fx_px = rs::FocalLengthPx(w, 87.0 * kPi / 180.0);
    p.fy_px = p.fx_px;
    p.cx_px = w * 0.5;
    p.cy_px = h * 0.5;
    return p;
}

// Hole fraction of a flat perpendicular wall at z made of one material. A narrow
// FOV (large fx) keeps every viewing ray perpendicular to the wall (cos ~ 1).
static double HoleRate(const rs::TextureModelParams& p, const rs::IrMaterial& m, float z, int w = 200, int h = 200)
{
    std::vector<float> img(static_cast<std::size_t>(w) * h, z);
    std::mt19937 rng(42);
    std::vector<float> scale;
    auto q = p;
    q.fx_px = q.fy_px = 5000.0;
    q.cx_px = w * 0.5;
    q.cy_px = h * 0.5;
    rs::ApplyTextureModel(img, w, h, nullptr, {}, m, q, rng, scale);
    std::size_t holes = 0;
    for (float v : img) holes += std::isfinite(v) ? 0u : 1u;
    return static_cast<double>(holes) / img.size();
}

// Expected hole fraction on a perpendicular wall (cos ~ 1 near the centre; small FOV patch).
static double ExpectedHole(const rs::TextureModelParams& p, const rs::IrMaterial& m, double z)
{
    return 1.0 - rs::ValidProbability(p, rs::TextureSnr(p, m, z, 1.0));
}

static void TestTable()
{
    std::printf("[material table]\n");
    const auto j = nlohmann::json::parse(R"({
        "presets": { "blasted_aluminum": { "base": "machined_aluminum", "specular": 0.3 } },
        "assign": { "work_black*": "black_oxide", "work_*": "machined_aluminum",
                    "work_special": { "ir_reflectance": 0.33 }, "bin_*": "blasted_aluminum" },
        "default": "cardboard"
    })");
    rs::MaterialTable t {};
    std::string err;
    check(rs::MaterialTableFromJson(j, t, err), "table parses: " + err);
    const auto builtin = rs::BuiltinIrMaterials();
    check(t.Lookup("work_black_01").ir_reflectance == builtin.at("black_oxide").ir_reflectance,
          "longest pattern wins (work_black* over work_*)");
    check(t.Lookup("work_alu").ir_reflectance == builtin.at("machined_aluminum").ir_reflectance, "pattern work_*");
    check(t.Lookup("work_special").ir_reflectance == 0.33, "exact name wins over patterns");
    check(t.Lookup("bin_a").specular == 0.3 && t.Lookup("bin_a").ir_reflectance == 0.6, "preset with base");
    check(t.Lookup("other").ir_reflectance == builtin.at("cardboard").ir_reflectance, "default for unmatched");
    check(t.Lookup("").ir_reflectance == builtin.at("cardboard").ir_reflectance, "default for unnamed");
    const auto by_id = t.Resolve({"work_alu", "", "bin_x"});
    check(by_id.size() == 3 && by_id[2].specular == 0.3, "resolve by material id");
    check(rs::WildcardMatch("a*b*c", "aXXbYc") && !rs::WildcardMatch("a*b", "aXc") && rs::WildcardMatch("*", ""),
          "wildcard matcher");

    rs::MaterialTable bad {};
    check(!rs::MaterialTableFromJson(nlohmann::json::parse(R"({"assign":{"x":"no_such"}})"), bad, err),
          "unknown preset rejected");
    check(!rs::MaterialTableFromJson(nlohmann::json::parse(R"({"assign":{"x":{"ir_reflectance":1.5}}})"), bad, err),
          "value out of [0,1] rejected");
}

static void TestMaterialsAndDistance()
{
    std::printf("[materials and distance]\n");
    const auto p = Params(200, 200);
    const auto b = rs::BuiltinIrMaterials();
    const auto& white = b.at("white_resin");
    const auto& black = b.at("black_oxide");
    std::printf("  50%% range (indoor, perpendicular): white %.2f m, aluminum %.2f m, cast iron %.2f m, black %.2f m\n",
                rs::HalfValidRange(p, white), rs::HalfValidRange(p, b.at("machined_aluminum")),
                rs::HalfValidRange(p, b.at("cast_iron")), rs::HalfValidRange(p, black));
    check(HoleRate(p, white, 3.0F) < 0.01, "white wall at 3 m: almost no holes");
    check(HoleRate(p, white, 0.6F) == 0.0, "white at 0.6 m (bin picking): no holes");
    check(HoleRate(p, black, 0.6F) < 0.001, "black at 0.6 m: valid");
    const double n = 200.0 * 200.0;
    for (float z : {1.8F, 2.2F, 2.8F}) {
        const double want = ExpectedHole(p, black, z);
        close_to(HoleRate(p, black, z), want, 3.0 * std::sqrt(want * (1 - want) / n) + 0.01,
                 "black hole rate at " + std::to_string(z).substr(0, 3) + " m follows the model");
    }
    check(HoleRate(p, black, 1.8F) < HoleRate(p, black, 2.2F) && HoleRate(p, black, 2.2F) < HoleRate(p, black, 2.8F),
          "hole rate grows with distance");
    check(HoleRate(p, black, 2.5F) > HoleRate(p, white, 2.5F) + 0.3, "black loses much more than white");

    rs::IrMaterial glass = b.at("glass");
    check(HoleRate(p, glass, 0.6F) == 1.0, "transparent: all holes");
}

static void TestProjectorAndAmbient()
{
    std::printf("[projector off / sunlight]\n");
    const auto b = rs::BuiltinIrMaterials();
    auto p = Params(200, 200);
    p.laser_power_mw = 0.0;
    check(HoleRate(p, b.at("white_resin"), 1.0F) == 1.0, "projector off: texture-less white wall -> all holes");
    const double card = HoleRate(p, b.at("cardboard"), 1.0F);
    const double want = ExpectedHole(p, b.at("cardboard"), 1.0);
    close_to(card, want, 0.02, "projector off: textured cardboard keeps part of the depth");
    check(card < 0.9, "textured surface is not lost entirely");

    auto sun = Params(200, 200);
    sun.ambient_ir = 1.0;
    const auto indoor = Params(200, 200);
    check(rs::HalfValidRange(sun, b.at("white_resin")) < 0.5 * rs::HalfValidRange(indoor, b.at("white_resin")),
          "sunlight washes out the projector (shorter range on white)");
    check(HoleRate(sun, b.at("concrete"), 6.0F) < 0.01, "sunlight: textured concrete stays valid far away");
}

// Plane tilted by phi about the vertical axis, at z0 on the optical axis.
static std::vector<float> TiltedPlane(const rs::TextureModelParams& p, int w, int h, double z0, double phi)
{
    std::vector<float> img(static_cast<std::size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const double xn = (x - p.cx_px) / p.fx_px;
            img[static_cast<std::size_t>(y) * w + x] =
                static_cast<float>(z0 * std::cos(phi) / (std::sin(phi) * xn + std::cos(phi)));
        }
    }
    return img;
}

static void TestIncidence()
{
    std::printf("[incidence angle]\n");
    const int w = 200, h = 200;
    const auto p = Params(w, h);
    for (double deg : {0.0, 45.0, 70.0}) {
        const double phi = deg * kPi / 180.0;
        const auto img = TiltedPlane(p, w, h, 1.5, phi);
        close_to(rs::SurfaceCosine(img, w, h, w / 2, h / 2, p), std::cos(phi), 0.01,
                 "normal from depth: cos at " + std::to_string(static_cast<int>(deg)) + " deg");
        double max_diff = 0.0;
        for (int y = 1; y < h - 1; y += 7)
            for (int x = 1; x < w - 1; x += 7)
                max_diff = std::max(max_diff, std::fabs(rs::SurfaceCosine(img, w, h, x, y, p)
                                                        - rs::SurfaceCosineReference(img, w, h, x, y, p)));
        check(max_diff < 1e-9, "fast normal == reference normal");
    }
    const auto& black = rs::BuiltinIrMaterials().at("black_oxide");
    auto flat = TiltedPlane(p, w, h, 2.0, 0.0);
    auto tilt = TiltedPlane(p, w, h, 2.0, 70.0 * kPi / 180.0);
    std::mt19937 r1(1), r2(1);
    std::vector<float> s1, s2;
    rs::ApplyTextureModel(flat, w, h, nullptr, {}, black, p, r1, s1);
    rs::ApplyTextureModel(tilt, w, h, nullptr, {}, black, p, r2, s2);
    // Compare the central 40x40 patch (both at ~2 m there; the tilted plane's
    // depth changes strongly toward the image sides).
    const auto centre_holes = [&](const std::vector<float>& img) {
        std::size_t holes = 0;
        for (int y = h / 2 - 20; y < h / 2 + 20; ++y)
            for (int x = w / 2 - 20; x < w / 2 + 20; ++x)
                holes += std::isfinite(img[static_cast<std::size_t>(y) * w + x]) ? 0u : 1u;
        return holes / 1600.0;
    };
    const double hf = centre_holes(flat);
    const double ht = centre_holes(tilt);
    std::printf("  black at 2 m, centre holes: perpendicular %.3f, tilted 70 deg %.3f\n", hf, ht);
    check(ht > hf + 0.4, "steep surface loses more depth than a perpendicular one");
}

static void TestMaterialIdsAndSigma()
{
    std::printf("[material ids / sigma scale]\n");
    const int w = 100, h = 50;
    const auto p = Params(w, h);
    const auto b = rs::BuiltinIrMaterials();
    std::vector<float> img(static_cast<std::size_t>(w) * h, 2.5F);
    std::vector<int> ids(img.size(), -1);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) ids[static_cast<std::size_t>(y) * w + x] = x < 50 ? 0 : 1;
    const std::vector<rs::IrMaterial> by_id {b.at("black_oxide"), b.at("white_resin")};
    std::mt19937 rng(3);
    std::vector<float> scale;
    rs::ApplyTextureModel(img, w, h, &ids, by_id, b.at("white_resin"), p, rng, scale);
    std::size_t left = 0, right = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            (x < 50 ? left : right) += std::isfinite(img[static_cast<std::size_t>(y) * w + x]) ? 0u : 1u;
    check(left > 10 * (right + 1), "material id selects black (left) vs white (right)");

    close_to(rs::SigmaScale(p, 1.0), 1.5, 1e-12, "sigma scale 1.5 at SNR 1");
    close_to(rs::SigmaScale(p, 50.0), 1.01, 1e-12, "sigma scale ~1 at high SNR");

    // sigma scale enters the depth model: RMS grows by the scale.
    rs::DepthModelParams m {};
    m.fx_px = p.fx_px;
    m.baseline_m = 0.05;
    m.disparity_sigma_px = 0.08;
    m.min_z_m = 0.1;
    m.max_z_m = 10.0;
    m.left_invalid_band = false;
    m.depth_unit_m = 0.0;
    const auto rms = [&](const std::vector<float>* sc) {
        std::vector<float> d(10000, 1.0F);
        std::mt19937 r(5);
        rs::ApplyDepthModel(d, 100, 100, m, r, nullptr, sc);
        double s2 = 0.0;
        for (float v : d) s2 += (v - 1.0) * (v - 1.0);
        return std::sqrt(s2 / d.size());
    };
    const std::vector<float> two(10000, 2.0F);
    // ~2 (z = fB/d is slightly non-linear in the disparity noise)
    close_to(rms(&two) / rms(nullptr), 2.0, 0.05, "sigma scale 2 doubles the depth RMS");
}

static void TestTiming()
{
    std::printf("[timing 848x480]\n");
    const int w = 848, h = 480;
    auto p = Params(w, h);
    const auto src = TiltedPlane(p, w, h, 1.0, 0.5);
    std::vector<int> ids(src.size(), 0);
    const std::vector<rs::IrMaterial> by_id {rs::BuiltinIrMaterials().at("machined_aluminum")};
    std::mt19937 rng(1);
    std::vector<float> scale;
    double total_ms = 0.0;
    const int reps = 10;
    for (int i = 0; i < reps; ++i) {
        auto img = src;
        const auto t0 = std::chrono::steady_clock::now();
        rs::ApplyTextureModel(img, w, h, &ids, by_id, by_id[0], p, rng, scale);
        total_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
    std::printf("  ApplyTextureModel: %.2f ms / frame\n", total_ms / reps);
    check(total_ms / reps < 100.0, "texture model runs well under a frame budget");
}

static void TestConfig()
{
    std::printf("[config]\n");
    rs::RealSenseConfig c {};
    std::string err;
    check(rs::RealSenseConfigFromJson(nlohmann::json {{"model", "D435i"}}, c, err), "default config");
    check(!c.texture_model.enabled, "texture model default off");

    const auto j = nlohmann::json::parse(R"({
        "model": "D435i",
        "texture_model": { "enabled": true, "laser_power_mw": 0, "ambient_ir": 1.0, "seed": 4,
                           "materials": { "assign": { "work_*": "black_oxide" }, "default": "cardboard" } }
    })");
    check(rs::RealSenseConfigFromJson(j, c, err), "inline material table: " + err);
    const auto p = rs::MakeTextureModelParams(c);
    check(c.texture_model.enabled && p.laser_power_mw == 0.0 && p.ambient_ir == 1.0 && c.texture_model.seed == 4,
          "texture parameters read");
    check(c.texture_model.materials.Lookup("work_1").ir_reflectance == 0.05, "inline assign");
    check(std::fabs(p.cx_px - 424.0) < 1e-12 && p.fx_px > 400.0, "intrinsics from the depth stream");

    auto file = j;
    file["texture_model"]["materials"] = "materials.json";
    check(rs::RealSenseConfigFromJson(file, c, err, std::string(HAKO_SENSOR_SOURCE_DIR) + "/config/realsense"),
          "material table file relative to the config: " + err);
    check(c.texture_model.materials.Lookup("work_black_3").ir_reflectance == 0.05
              && c.texture_model.materials.Lookup("bin_1").texture == 0.5,
          "config/realsense/materials.json assignments");
    file["texture_model"]["materials"] = "no_such_file.json";
    check(!rs::RealSenseConfigFromJson(file, c, err, "."), "missing material file rejected");
    auto bad = j;
    bad["texture_model"]["reference_range_m"] = 0.0;
    check(!rs::RealSenseConfigFromJson(bad, c, err), "reference_range_m <= 0 rejected");
}

int main()
{
    TestTable();
    TestMaterialsAndDistance();
    TestProjectorAndAmbient();
    TestIncidence();
    TestMaterialIdsAndSigma();
    TestTiming();
    TestConfig();
    std::printf("realsense_texture_model_test: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
