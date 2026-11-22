// Unit test for the FANUC iRVision 2D camera profile. Backend-free (no MuJoCo, no GL).
//   1) catalog optics -> FOV / focal length in pixels / field of view at a distance
//   2) catalog validation (lens 8 / 12 / 16 / 25 mm only, color variant)
//   3) config/irvision/irvision_2d_gray_8mm.json matches MakeProfile(Grayscale, 8, Red)
//   4) config/irvision/irvision_2d.xml (MJCF) matches the same optics and housing

#include <cmath>
#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "sensors/camera/irvision/irvision_profile.hpp"

namespace irv = hako::robots::sensor::camera::irvision;

#ifndef IRVISION_CONFIG_DIR
#error "IRVISION_CONFIG_DIR must point to config/irvision"
#endif

static int g_failures = 0;
static int g_checks = 0;

static void check(bool cond, const std::string& msg)
{
    ++g_checks;
    if (!cond) ++g_failures;
    std::printf("  [%s] %s\n", cond ? "ok" : "FAIL", msg.c_str());
}

static void close_to(double a, double b, double tol, const std::string& msg)
{
    ++g_checks;
    const bool ok = std::fabs(a - b) <= tol;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s (got %.6f, want %.6f +- %.6f)\n", ok ? "ok" : "FAIL", msg.c_str(), a, b, tol);
}

static std::string ReadText(const std::string& path)
{
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static double Deg(double rad) { return rad * 180.0 / irv::kPi; }

static void TestOptics()
{
    std::printf("[1] optics of grayscale / 8 mm\n");
    const auto p = irv::MakeProfile(irv::ImageType::Grayscale, 8.0, irv::LedColor::Red);
    close_to(irv::SensorWidthMm(p), 7.2, 1e-9, "sensor width = 1280 px x 5.625 um");
    close_to(irv::SensorHeightMm(p), 5.76, 1e-9, "sensor height keeps the 5:4 aspect");
    close_to(irv::FocalLengthPx(p), 1422.2222222, 1e-6, "focal length in pixels");
    close_to(Deg(irv::HorizontalFovRad(p)), 48.4555, 1e-3, "horizontal FOV [deg]");
    close_to(Deg(irv::VerticalFovRad(p)), 39.5978, 1e-3, "vertical FOV [deg]");
    const auto [w, h] = irv::FieldOfViewAt(p, 0.3);
    close_to(w, 0.270, 1e-9, "field of view width at 300 mm [m]");
    close_to(h, 0.216, 1e-9, "field of view height at 300 mm [m]");
    // longer lens -> narrower view, same sensor
    const auto p25 = irv::MakeProfile(irv::ImageType::Grayscale, 25.0, irv::LedColor::None);
    check(irv::HorizontalFovRad(p25) < irv::HorizontalFovRad(p), "25 mm is narrower than 8 mm");
    close_to(irv::FieldOfViewAt(p25, 0.3).first, 0.3 * 7.2 / 25.0, 1e-12, "25 mm field of view width at 300 mm");
}

static void TestCatalog()
{
    std::printf("[2] catalog\n");
    for (double f : {8.0, 12.0, 16.0, 25.0}) {
        check(irv::IsCatalogLens(f), "lens " + std::to_string(static_cast<int>(f)) + " mm is in the catalog");
    }
    bool threw = false;
    try {
        (void)irv::MakeProfile(irv::ImageType::Grayscale, 6.0, irv::LedColor::Red);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    check(threw, "lens 6 mm is rejected");
    const auto c = irv::MakeProfile(irv::ImageType::Color, 8.0, irv::LedColor::White);
    check(c.width_px == 640 && c.height_px == 512, "color is 640 x 512");
    close_to(irv::HorizontalFovRad(c), irv::HorizontalFovRad(irv::MakeProfile(irv::ImageType::Grayscale, 8.0,
                                                                              irv::LedColor::Red)),
             1e-12, "color keeps the same optical size (same FOV)");
    const auto spec = irv::ToCameraSpecJson(c, "f", "c");
    check(spec["spec"]["image"]["format"] == "R8G8B8", "color outputs R8G8B8");
    close_to(irv::LedRgb(irv::LedColor::None)[0], 0.0, 0.0, "no LED is black");
}

static void TestJsonConfig()
{
    std::printf("[3] config/irvision/irvision_2d_gray_8mm.json\n");
    const std::string path = std::string(IRVISION_CONFIG_DIR) + "/irvision_2d_gray_8mm.json";
    nlohmann::json file;
    try {
        file = nlohmann::json::parse(ReadText(path));
    } catch (const std::exception& e) {
        check(false, std::string("parse ") + path + ": " + e.what());
        return;
    }
    const auto p = irv::MakeProfile(irv::ImageType::Grayscale, 8.0, irv::LedColor::Red);
    const auto want = irv::ToCameraSpecJson(p, "irvision_optical", "irvision_cam");
    const auto& s = file["spec"];
    const auto& w = want["spec"];
    check(s["frame_id"] == w["frame_id"], "frame_id");
    check(file["mjcf_binding"]["camera_name"] == want["mjcf_binding"]["camera_name"], "mjcf_binding.camera_name");
    check(s["image"] == w["image"], "image (1280 x 1024, L8)");
    close_to(s["horizontal_fov"].get<double>(), w["horizontal_fov"].get<double>(), 1e-12, "horizontal_fov");
    close_to(s["update_rate"].get<double>(), w["update_rate"].get<double>(), 0.0, "update_rate");
    for (const char* k : {"fx", "fy", "cx", "cy"}) {
        close_to(s["intrinsics"][k].get<double>(), w["intrinsics"][k].get<double>(), 1e-9, std::string("intrinsics.") + k);
    }
    close_to(s["clip"]["near"].get<double>(), w["clip"]["near"].get<double>(), 0.0, "clip.near");
    close_to(s["clip"]["far"].get<double>(), w["clip"]["far"].get<double>(), 0.0, "clip.far");
    const auto& fi = file["irvision"];
    const auto& wi = want["irvision"];
    close_to(fi["vertical_fov"].get<double>(), wi["vertical_fov"].get<double>(), 1e-12, "irvision.vertical_fov");
    check(fi["led"]["color"] == "red", "irvision.led.color = red");
    for (int i = 0; i < 3; ++i) {
        close_to(fi["led"]["rgb"][i].get<double>(), wi["led"]["rgb"][i].get<double>(), 1e-12, "irvision.led.rgb");
    }
    check(fi["housing_mm"] == wi["housing_mm"], "irvision.housing_mm (80 x 74 x 131.8)");
    close_to(fi["mass_kg"].get<double>(), 0.6, 0.0, "irvision.mass_kg");
}

static double Attr(const std::string& xml, const std::string& pattern, int group = 1)
{
    std::smatch m;
    if (!std::regex_search(xml, m, std::regex(pattern))) return std::nan("");
    return std::stod(m[group].str());
}

static void TestMjcf()
{
    std::printf("[4] config/irvision/irvision_2d.xml\n");
    const std::string xml = ReadText(std::string(IRVISION_CONFIG_DIR) + "/irvision_2d.xml");
    check(!xml.empty(), "MJCF is readable");
    const auto p = irv::MakeProfile(irv::ImageType::Grayscale, 8.0, irv::LedColor::Red);
    // <camera name="irvision_cam" ... focal="fx fy" sensorsize="w h" resolution="W H"/>
    const std::string cam = R"re(<camera name="irvision_cam"[^>]*?)re";
    close_to(Attr(xml, cam + R"re(focal="([0-9.]+) )re"), p.focal_length_mm * 1e-3, 1e-12, "camera focal [m]");
    close_to(Attr(xml, cam + R"re(sensorsize="([0-9.]+) )re"), irv::SensorWidthMm(p) * 1e-3, 1e-12, "camera sensorsize x [m]");
    close_to(Attr(xml, cam + R"re(sensorsize="[0-9.]+ ([0-9.]+)")re"), irv::SensorHeightMm(p) * 1e-3, 1e-12,
             "camera sensorsize y [m]");
    close_to(Attr(xml, cam + R"re(resolution="([0-9]+) )re"), p.width_px, 0.0, "camera resolution x");
    close_to(Attr(xml, cam + R"re(resolution="[0-9]+ ([0-9]+)")re"), p.height_px, 0.0, "camera resolution y");
    // housing box half sizes: length (x) is 131.8 mm minus the 2 mm front layer, width 80, height 74
    const std::string body = R"re(<geom name="irv_body"[^>]*?size="([0-9.]+) ([0-9.]+) ([0-9.]+)")re";
    close_to(2 * Attr(xml, body, 1) + 0.002, p.housing.length_mm * 1e-3, 1e-9, "housing length incl. front layer [m]");
    close_to(2 * Attr(xml, body, 2), p.housing.width_mm * 1e-3, 1e-9, "housing width [m]");
    close_to(2 * Attr(xml, body, 3), p.housing.height_mm * 1e-3, 1e-9, "housing height [m]");
    close_to(Attr(xml, R"re(<inertial[^>]*?mass="([0-9.]+)")re"), p.housing.mass_kg, 0.0, "mass [kg]");
    close_to(Attr(xml, R"re(offwidth="([0-9]+)")re"), p.width_px, 0.0, "offscreen buffer width >= image width");
    close_to(Attr(xml, R"re(offheight="([0-9]+)")re"), p.height_px, 0.0, "offscreen buffer height >= image height");
    const auto rgb = irv::LedRgb(p.led);
    close_to(Attr(xml, R"re(<light name="irvision_led"[^>]*?diffuse="([0-9.]+) )re"), rgb[0], 1e-12, "LED diffuse R");
    close_to(Attr(xml, R"re(<light name="irvision_led"[^>]*?diffuse="[0-9.]+ ([0-9.]+) )re"), rgb[1], 1e-12, "LED diffuse G");
}

int main()
{
    TestOptics();
    TestCatalog();
    TestJsonConfig();
    TestMjcf();
    std::printf("\n%d / %d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
