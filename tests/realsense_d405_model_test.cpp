// Unit test: the D405 wrist-camera model (config/realsense/d405_wrist_model.json and d405_model.xml)
// agrees with the built-in RealSense profile FindProfile("D405"). Backend-free (no MuJoCo, no GL).

#include <cmath>
#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_profile.hpp"

namespace rs = hako::robots::sensor::camera::realsense;

#ifndef REALSENSE_CONFIG_DIR
#error "REALSENSE_CONFIG_DIR must point to config/realsense"
#endif

static int g_failures = 0;
static int g_checks = 0;

static void close_to(double a, double b, double tol, const std::string& msg)
{
    ++g_checks;
    const bool ok = std::fabs(a - b) <= tol;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s (got %.6f, want %.6f +- %.6f)\n", ok ? "ok" : "FAIL", msg.c_str(), a, b, tol);
}

static void check(bool cond, const std::string& msg)
{
    ++g_checks;
    if (!cond) ++g_failures;
    std::printf("  [%s] %s\n", cond ? "ok" : "FAIL", msg.c_str());
}

static std::string ReadText(const std::string& path)
{
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static double Attr(const std::string& xml, const std::string& pattern, int group = 1)
{
    std::smatch m;
    if (!std::regex_search(xml, m, std::regex(pattern))) return std::nan("");
    return std::stod(m[group].str());
}

int main()
{
    constexpr double kPi = 3.14159265358979323846;
    const auto prof = rs::FindProfile("D405");
    check(prof.has_value(), "FindProfile(D405)");
    if (!prof) return 1;
    const auto& p = *prof;

    std::printf("[1] d405_wrist_model.json vs profile\n");
    const auto j = nlohmann::json::parse(ReadText(std::string(REALSENSE_CONFIG_DIR) + "/d405_wrist_model.json"));
    const auto& m = j["realsense_model"];
    const auto& d = m["depth"];
    close_to(m["imagers"]["baseline_mm"].get<double>() / 1000, p.baseline_m, 1e-12, "baseline");
    close_to(m["imagers"]["left_offset_mm"].get<double>() * 2, m["imagers"]["baseline_mm"].get<double>(), 1e-12,
             "left imager at half the baseline");
    close_to(d["hfov_deg"].get<double>(), p.depth_hfov_deg, 1e-12, "depth hfov");
    close_to(d["vfov_deg_catalogue"].get<double>(), p.depth_vfov_deg, 1e-12, "depth vfov (catalogue)");
    close_to(d["min_z_m"].get<double>(), p.min_z_m, 1e-12, "min z");
    close_to(d["max_z_m"].get<double>(), p.max_z_m, 1e-12, "max z");
    close_to(d["subpixel_rms_px"].get<double>(), p.subpixel_rms_px, 1e-12, "subpixel rms");
    check(d["active_projector"].get<bool>() == p.active_projector, "passive stereo");
    close_to(d["width"].get<double>(), p.depth_max_width, 0.0, "depth width");
    close_to(d["height"].get<double>(), p.depth_max_height, 0.0, "depth height");
    const double fx = p.depth_max_width / (2 * std::tan(p.depth_hfov_deg * kPi / 360));
    close_to(d["fx_px"].get<double>(), fx, 1e-9, "fx = width / (2 tan(hfov/2))");
    close_to(d["max_disparity_px"].get<double>(), fx * p.baseline_m / p.min_z_m, 1e-9, "max disparity = fx B / MinZ");

    std::printf("[2] d405_model.xml vs json\n");
    const std::string xml = ReadText(std::string(REALSENSE_CONFIG_DIR) + "/d405_model.xml");
    const double vfov = 2 * std::atan(std::tan(p.depth_hfov_deg * kPi / 360) * p.depth_max_height / p.depth_max_width) * 180 / kPi;
    close_to(Attr(xml, R"re(<camera name="d405_left"[^>]*?fovy="([0-9.]+)")re"), vfov, 1e-6, "camera fovy (from hfov and aspect)");
    close_to(Attr(xml, R"re(<camera name="d405_left"[^>]*?resolution="([0-9]+) )re"), p.depth_max_width, 0.0, "resolution x");
    close_to(Attr(xml, R"re(<camera name="d405_left" pos="[0-9.\-]+ ([0-9.\-]+) )re"), p.baseline_m / 2, 1e-12, "camera at the left imager");
    const std::string body = R"re(<geom name="d405_body"[^>]*?size="([0-9.]+) ([0-9.]+) ([0-9.]+)")re";
    const auto& h = m["housing_mm"];
    close_to(2 * Attr(xml, body, 1) + 0.001, h["depth"].get<double>() / 1000, 1e-9, "housing depth incl. glass layer");
    close_to(2 * Attr(xml, body, 2), h["width"].get<double>() / 1000, 1e-9, "housing width");
    close_to(2 * Attr(xml, body, 3), h["height"].get<double>() / 1000, 1e-9, "housing height");
    close_to(Attr(xml, R"re(<inertial[^>]*?mass="([0-9.]+)")re"), m["mass_kg"].get<double>(), 0.0, "mass");
    close_to(Attr(xml, R"re(offwidth="([0-9]+)")re"), p.depth_max_width, 0.0, "offscreen width");
    close_to(Attr(xml, R"re(offheight="([0-9]+)")re"), p.depth_max_height, 0.0, "offscreen height");

    std::printf("\n%d / %d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
