// realsense_depth_offline - apply the RealSense depth error model to an ideal depth image
// rendered by another simulator (e.g. Isaac Sim's distance_to_image_plane).
//
// The same header-only math as RealSenseCameraSensor::RenderDepth, in the same order, for a
// camera at rest (no rolling shutter / motion blur):
//   exposure level -> edge effects (occlusion shadow, flying pixels) -> texture model
//   (weak-pattern holes, sigma scale) -> specular highlights (no inter-reflection: no ray
//   caster outside MuJoCo) -> thermal offset -> depth model (range gate, left band,
//   disparity noise, subpixel step, depth unit) -> SDK post processing.
// So a frame from Isaac Sim gets the errors of the hakoniwa model (the single source of
// truth) without porting it to Python.
//
// Usage:
//   realsense_depth_offline --config d405_wrist.json --depth ideal.npy --out measured.npy
//       [--materials ids.npy --material-names names.json] [--time 0] [--frame 0]
//       [--stats stats.json]
//   ideal.npy : float32 (H, W), z-depth in metres, NaN / <= 0 / inf = no surface.
//               (H, W) must equal the config depth stream size.
//   ids.npy   : int32 (H, W), index into names.json (a JSON list of material names), -1 = none.
//               Names are resolved with the config's texture_model material table.
//   --frame   : offsets every random seed (frame k of a sequence gets its own noise).
//   measured.npy: float32 (H', W') metres, NaN = invalid (H', W' after decimation).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_depth_model.hpp"
#include "sensors/camera/realsense/realsense_edge_effects.hpp"
#include "sensors/camera/realsense/realsense_exposure_model.hpp"
#include "sensors/camera/realsense/realsense_post_processing.hpp"
#include "sensors/camera/realsense/realsense_specular_model.hpp"
#include "sensors/camera/realsense/realsense_texture_model.hpp"
#include "sensors/camera/realsense/realsense_thermal_model.hpp"

namespace rs = hako::robots::sensor::camera::realsense;

namespace
{
// --- minimal .npy (version 1/2, little endian, C order, 2-D) ---------------------------------
struct Npy
{
    std::string descr;
    int rows {0};
    int cols {0};
    std::vector<char> data;
};

Npy ReadNpy(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open " + path);
    char magic[6];
    f.read(magic, 6);
    if (std::memcmp(magic, "\x93NUMPY", 6) != 0) throw std::runtime_error(path + ": not a .npy file");
    unsigned char ver[2];
    f.read(reinterpret_cast<char*>(ver), 2);
    std::uint32_t hlen = 0;
    if (ver[0] == 1) {
        unsigned char b[2];
        f.read(reinterpret_cast<char*>(b), 2);
        hlen = b[0] | (b[1] << 8);
    } else {
        unsigned char b[4];
        f.read(reinterpret_cast<char*>(b), 4);
        hlen = b[0] | (b[1] << 8) | (b[2] << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
    }
    std::string header(hlen, '\0');
    f.read(header.data(), hlen);
    auto field = [&](const std::string& key) {
        const auto k = header.find("'" + key + "'");
        if (k == std::string::npos) throw std::runtime_error(path + ": header without " + key);
        return header.substr(header.find(':', k) + 1);
    };
    Npy n;
    const std::string d = field("descr");
    n.descr = d.substr(d.find('\'') + 1, d.find('\'', d.find('\'') + 1) - d.find('\'') - 1);
    if (field("fortran_order").find("False") == std::string::npos)
        throw std::runtime_error(path + ": fortran order is not supported");
    const std::string s = field("shape");
    if (std::sscanf(s.c_str(), " (%d, %d)", &n.rows, &n.cols) != 2)
        throw std::runtime_error(path + ": shape must be 2-D");
    const std::size_t bytes = static_cast<std::size_t>(n.rows) * n.cols * 4;
    if (n.descr != "<f4" && n.descr != "<i4") throw std::runtime_error(path + ": dtype must be <f4 or <i4");
    n.data.resize(bytes);
    f.read(n.data.data(), static_cast<std::streamsize>(bytes));
    if (static_cast<std::size_t>(f.gcount()) != bytes) throw std::runtime_error(path + ": truncated data");
    return n;
}

void WriteNpyF32(const std::string& path, const std::vector<float>& v, int rows, int cols)
{
    std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (" + std::to_string(rows) + ", "
        + std::to_string(cols) + "), }";
    while ((10 + header.size() + 1) % 64 != 0) header += ' ';
    header += '\n';
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write " + path);
    f.write("\x93NUMPY\x01\x00", 8);
    const std::uint16_t hl = static_cast<std::uint16_t>(header.size());
    const char b[2] = {static_cast<char>(hl & 0xff), static_cast<char>(hl >> 8)};
    f.write(b, 2);
    f.write(header.data(), static_cast<std::streamsize>(header.size()));
    f.write(reinterpret_cast<const char*>(v.data()), static_cast<std::streamsize>(v.size() * sizeof(float)));
}

std::size_t CountValid(const std::vector<float>& d)
{
    std::size_t n = 0;
    for (float v : d) n += (std::isfinite(v) && v > 0.0F) ? 1 : 0;
    return n;
}

int Usage()
{
    std::cerr << "usage: realsense_depth_offline --config <json> --depth <ideal.npy> --out <measured.npy>\n"
                 "         [--materials <ids.npy> --material-names <names.json>] [--time <s>] [--frame <k>]\n"
                 "         [--stats <stats.json>]\n";
    return 2;
}
}  // namespace

int main(int argc, char** argv)
{
    std::string config_path, depth_path, out_path, ids_path, names_path, stats_path;
    double t = 0.0;
    std::uint32_t frame = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(a + " needs a value");
            return argv[++i];
        };
        if (a == "--config") config_path = next();
        else if (a == "--depth") depth_path = next();
        else if (a == "--out") out_path = next();
        else if (a == "--materials") ids_path = next();
        else if (a == "--material-names") names_path = next();
        else if (a == "--stats") stats_path = next();
        else if (a == "--time") t = std::stod(next());
        else if (a == "--frame") frame = static_cast<std::uint32_t>(std::stoul(next()));
        else return Usage();
    }
    if (config_path.empty() || depth_path.empty() || out_path.empty()) return Usage();

    try {
        rs::RealSenseConfig c {};
        if (!rs::LoadRealSenseConfigFromJson(config_path, c)) {
            std::cerr << "cannot load RealSense config " << config_path << "\n";
            return 1;
        }
        std::vector<std::string> warnings;
        const std::string reason = rs::ValidateConfig(c, &warnings);
        if (!reason.empty()) {
            std::cerr << "invalid RealSense config: " << reason << "\n";
            return 1;
        }
        for (const auto& w : warnings) std::cerr << "warning: " << w << "\n";

        const Npy dn = ReadNpy(depth_path);
        if (dn.descr != "<f4") throw std::runtime_error("depth must be float32");
        const int W = dn.cols, H = dn.rows;
        if (W != c.depth.width || H != c.depth.height) {
            std::cerr << "depth is " << W << "x" << H << " but the config stream is " << c.depth.width << "x"
                      << c.depth.height << " (render the ideal depth at the stream size)\n";
            return 1;
        }
        std::vector<float> depth(static_cast<std::size_t>(W) * H);
        std::memcpy(depth.data(), dn.data.data(), depth.size() * sizeof(float));
        const float nan_val = std::numeric_limits<float>::quiet_NaN();
        for (float& v : depth)
            if (!std::isfinite(v) || v <= 0.0F) v = nan_val;
        const std::size_t valid_in = CountValid(depth);

        std::vector<int> ids;
        std::vector<rs::IrMaterial> mats_by_id;
        if (!ids_path.empty()) {
            const Npy in = ReadNpy(ids_path);
            if (in.descr != "<i4" || in.rows != H || in.cols != W)
                throw std::runtime_error("materials must be int32 with the depth's shape");
            ids.resize(depth.size());
            std::memcpy(ids.data(), in.data.data(), ids.size() * sizeof(int));
            std::vector<std::string> names;
            if (!names_path.empty()) {
                std::ifstream nf(names_path);
                names = nlohmann::json::parse(nf).get<std::vector<std::string>>();
            }
            mats_by_id = c.texture_model.materials.Resolve(names);
        }
        const std::vector<int>* ids_ptr = ids.empty() ? nullptr : &ids;
        const rs::IrMaterial fallback = c.texture_model.materials.default_material;

        const rs::DepthModelParams depth_params = rs::MakeDepthModelParams(c);
        const rs::EdgeEffectsParams edge_params = rs::MakeEdgeEffectsParams(c, depth_params);
        const rs::TextureModelParams texture_params = rs::MakeTextureModelParams(c);
        const rs::SpecularModelParams specular_params = rs::MakeSpecularModelParams(c);
        std::mt19937 depth_rng(c.depth_model.seed + frame);
        std::mt19937 edge_rng(c.edge_effects.seed + frame);
        std::mt19937 texture_rng(c.texture_model.seed + frame);
        std::mt19937 specular_rng(c.specular_model.seed + frame);

        nlohmann::json stats;
        stats["model"] = c.profile.model;
        stats["size"] = {W, H};
        stats["fx_px"] = depth_params.fx_px;
        stats["baseline_m"] = depth_params.baseline_m;
        stats["min_z_m"] = depth_params.min_z_m;
        stats["max_z_m"] = depth_params.max_z_m;
        stats["disparity_sigma_px"] = depth_params.disparity_sigma_px;
        stats["valid_in"] = valid_in;

        // 1. exposure (camera at rest: brightness / gain only)
        const auto& ex = c.exposure;
        rs::ExposureDecision exposure {};
        if (ex.enabled) {
            const double level = rs::IrSceneLevel(depth, ids_ptr, mats_by_id, fallback, c.texture_model.laser_power_mw,
                                                  c.texture_model.ambient_ir, ex.ambient_gain);
            const double t_ref = ex.reference_depth_exposure_us * 1e-6;
            exposure = ex.auto_exposure
                ? rs::AutoExposure(level, t_ref, ex.reference_gain, ex.min_exposure_us * 1e-6,
                                   ex.MaxExposureS(c.depth.fps), ex.max_gain)
                : rs::ManualExposure(level, ex.depth_exposure_us * 1e-6, ex.depth_gain, t_ref, ex.reference_gain);
            stats["exposure"] = {{"exposure_s", exposure.exposure_s}, {"gain", exposure.gain},
                                 {"brightness", exposure.brightness}};
        }
        // 2. edge effects
        if (edge_params.occlusion || edge_params.flying_pixels) {
            const auto st = rs::ApplyEdgeEffects(depth, W, H, edge_params, edge_rng);
            stats["edge"] = {{"occluded", st.occluded}, {"flying", st.flying}};
        }
        // 3. texture model
        std::vector<float> sigma_scale;
        const std::vector<float>* sigma_ptr = nullptr;
        if (c.texture_model.enabled) {
            const auto st = rs::ApplyTextureModel(depth, W, H, ids_ptr, mats_by_id, fallback, texture_params,
                                                  texture_rng, sigma_scale);
            sigma_ptr = &sigma_scale;
            stats["texture"] = {{"holes", st.holes}, {"transparent", st.transparent}};
        }
        // 4. specular (highlights only: inter-reflection needs the scene's ray caster)
        if (c.specular_model.enabled) {
            const auto st = rs::ApplySpecularModel(depth, W, H, ids_ptr, mats_by_id, fallback, specular_params,
                                                   rs::RayCastFn {}, specular_rng);
            stats["specular"] = {{"highlight_holes", st.highlight_holes}, {"mirror_pixels", st.mirror_pixels}};
        }
        // 5. thermal drift + exposure noise -> 6. depth model
        rs::DepthModelParams params = depth_params;
        if (c.thermal.enabled) {
            rs::ThermalModel thermal;
            thermal.Configure(rs::MakeThermalParams(c));
            params.disparity_offset_px = thermal.DisparityOffsetPx(t);
            stats["thermal"] = {{"time_s", t}, {"temperature_c", thermal.TemperatureAt(t)},
                                {"disparity_offset_px", params.disparity_offset_px}};
        }
        if (ex.enabled) {
            params.disparity_sigma_px *= rs::GainNoiseScale(exposure.gain, ex.reference_gain)
                / std::sqrt(std::clamp(exposure.brightness, 1e-3, 1.0));
        }
        stats["disparity_sigma_px_applied"] = params.disparity_sigma_px;
        rs::ApplyDepthModel(depth, W, H, params, depth_rng, ids_ptr, sigma_ptr);
        // 7. SDK post processing (single frame: the temporal filter has no history)
        int w = W, h = H;
        rs::DepthPostProcessor post;
        post.Apply(depth, w, h, depth_params.fx_px, depth_params.baseline_m, c.post_processing);

        stats["out_size"] = {w, h};
        stats["valid_out"] = CountValid(depth);
        WriteNpyF32(out_path, depth, h, w);
        if (!stats_path.empty()) std::ofstream(stats_path) << stats.dump(2) << "\n";
        std::cout << stats.dump() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
