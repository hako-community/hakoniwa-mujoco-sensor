// Render smoke test for RealSenseCameraSensor / RealSenseImuSensor on the real
// MuJoCo renderer. Needs HAKO_SENSOR_BUILD_CAMERA=ON, a linkable libmujoco and an
// OpenGL context (a hidden GLFW window), so it is built only when both are available.
//
// Scene: a flat wall perpendicular to the camera axis at 2 m and a thin pole on the
// right; the cameras sit on a rig that can yaw about the vertical axis. Checks:
//   - depth is z-depth around 2 m with the typical RMS z^2*0.08/(fx*B),
//   - the left invalid band has width fx*B/z,
//   - RGB is rendered with its own FOV/resolution, and D421 has no RGB,
//   - decimation changes the output size and the derived DepthCameraConfig,
//   - rolling shutter (D415) skews the pole while the rig yaws,
//   - the IMU reads gravity and the yaw rate in the optical frame,
//   - the generic DepthCameraSensor still works next to it (existing API),
//   - RS-1: a box at 0.6 m in front of a wall at 1.0 m (camera looking -x) casts an
//     occlusion shadow of fx*B*(1/0.6 - 1/1.0) px on its left and gets flying
//     pixels on its edges; with edge effects off the output is bit-identical,
//   - RS-2: three panels at 2.45 m (camera looking -y) with MuJoCo materials mapped
//     by config/realsense/materials.json: black oxide loses most depth, aluminum and
//     cardboard stay valid; with the projector off the texture-less aluminum is lost
//     and the textured cardboard keeps part; texture model off is bit-identical,
//   - RS-3 (separate scene): a face-on aluminum plate gets a highlight hole at the
//     centre; a 45 deg aluminum plate mirrors the side wall, whose image appears
//     behind the plate at z * (range + t) / range; specular model off is bit-identical,
//   - RS-4: a cold start makes the 2 m wall read ~1.3 cm farther (disparity offset
//     -0.075 px), the warm camera reads true, the on-chip calibration removes the
//     cold offset, and the IMU at rest reads its thermal zero-offset drift,
//   - RS-5: the box scene with the camera yawing at 3 rad/s: depth loses a band of
//     ~(N-1)/N * w*t*fx px at each box edge (wider for a longer exposure), the RGB
//     edges smear, auto exposure lengthens the exposure / raises the gain in a dark
//     scene, and exposure off is bit-identical (depth and RGB),
//   - RS-6: on the rig yawing at 5 rad/s an IMU chip 5 cm off the axis reads the
//     centripetal -w^2 r, IMU stamps carry the time offset, misalignment tilts
//     gravity, and camera frames can be dropped / jittered.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include "physics.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "sensors/camera/realsense/realsense_camera_sensor.hpp"
#include "sensors/camera/realsense/realsense_depth_metrics.hpp"
#include "sensors/camera/realsense/realsense_imu_sensor.hpp"

namespace cam = hako::robots::sensor::camera;
namespace rs = hako::robots::sensor::camera::realsense;

namespace
{
const char* kScene = R"(
<mujoco model="realsense_smoke">
  <visual><global offwidth="1920" offheight="1080"/></visual>
  <asset>
    <material name="wall_mat" rgba="0.8 0.7 0.6 1"/>
    <material name="pole_mat" rgba="0.1 0.1 0.1 1"/>
    <material name="work_black_plate" rgba="0.05 0.05 0.05 1"/>
    <material name="work_alu_plate" rgba="0.7 0.7 0.75 1"/>
    <material name="bin_card" rgba="0.6 0.45 0.3 1"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <body name="rig" pos="0 0 1">
      <joint name="yaw" type="hinge" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <camera name="rs_depth" pos="0 0 0" xyaxes="0 -1 0 0 0 1"/>
      <camera name="rs_color" pos="0 -0.015 0" xyaxes="0 -1 0 0 0 1"/>
    </body>
    <geom name="wall" type="box" pos="2.05 0 1" size="0.05 20 20" material="wall_mat"/>
    <geom name="pole" type="box" pos="1.9 -1.0 1" size="0.02 0.02 10" material="pole_mat"/>
    <camera name="rs_edge" pos="0 0 1" xyaxes="0 1 0 0 0 1"/>
    <geom name="rear_wall" type="box" pos="-1.05 0 1" size="0.05 20 20" material="wall_mat"/>
    <geom name="edge_box" type="box" pos="-0.65 0 1" size="0.05 0.1 0.1" material="pole_mat"/>
    <camera name="rs_mat" pos="0 0 1" xyaxes="-1 0 0 0 0 1"/>
    <geom name="backdrop" type="box" pos="0 -3.05 1" size="20 0.05 20" material="wall_mat"/>
    <geom name="panel_black" type="box" pos="0.45 -2.5 1" size="0.2 0.05 0.3" material="work_black_plate"/>
    <geom name="panel_alu" type="box" pos="0 -2.5 1" size="0.2 0.05 0.3" material="work_alu_plate"/>
    <geom name="panel_card" type="box" pos="-0.45 -2.5 1" size="0.2 0.05 0.3" material="bin_card"/>
  </worldbody>
</mujoco>
)";

// RS-3: shiny plates. Camera at the origin looking +x (image right = world -y).
const char* kSpecScene = R"(
<mujoco model="realsense_specular">
  <visual><global offwidth="1920" offheight="1080"/></visual>
  <asset>
    <material name="wall_mat" rgba="0.8 0.7 0.6 1"/>
    <material name="work_alu_face" rgba="0.7 0.7 0.75 1"/>
    <material name="work_alu_mirror" rgba="0.7 0.7 0.75 1"/>
    <material name="work_alu_shaft" rgba="0.7 0.7 0.75 1"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <camera name="rs_spec" pos="0 0 1" xyaxes="0 -1 0 0 0 1"/>
    <geom name="backdrop" type="box" pos="2.05 0 1" size="0.05 3 3" material="wall_mat"/>
    <geom name="face_plate" type="box" pos="0.6 0.25 1" euler="0 0 22.62" size="0.005 0.12 0.12" material="work_alu_face"/>
    <geom name="mirror_plate" type="box" pos="0.7 -0.3 1" euler="0 0 45" size="0.005 0.12 0.1" material="work_alu_mirror"/>
    <geom name="side_wall" type="box" pos="0.8 -0.65 1" size="0.4 0.05 0.5" material="wall_mat"/>
    <geom name="shaft" type="cylinder" pos="0.8 0 1" size="0.04 0.15" material="work_alu_shaft"/>
  </worldbody>
</mujoco>
)";

class XmlWorld : public hako::robots::physics::IWorld
{
public:
    void loadModel(const std::string& xml) override
    {
        char err[1000] = {0};
        mjSpec* spec = mj_parseXMLString(xml.c_str(), nullptr, err, sizeof(err));
        if (!spec) {
            std::printf("mj_parseXMLString failed: %s\n", err);
            return;
        }
        model = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        if (!model) {
            std::printf("mj_compile failed\n");
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

void close_to(double a, double b, double tol, const std::string& msg)
{
    ++g_checks;
    const bool ok = std::fabs(a - b) <= tol;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s (got %.5f, want %.5f +- %.5f)\n", ok ? "ok" : "FAIL", msg.c_str(), a, b, tol);
}

struct Stats
{
    double mean {0.0};
    double sigma {0.0};
    std::size_t valid {0};
};

Stats ColumnStats(const cam::DepthFrame& f, int x0, int x1)
{
    double s = 0.0;
    double s2 = 0.0;
    std::size_t n = 0;
    for (int y = 0; y < f.height; ++y) {
        for (int x = x0; x < x1; ++x) {
            const float v = f.data[static_cast<std::size_t>(y) * f.width + x];
            if (!std::isfinite(v)) continue;
            s += v;
            s2 += static_cast<double>(v) * v;
            ++n;
        }
    }
    Stats st {};
    st.valid = n;
    if (n) {
        st.mean = s / n;
        st.sigma = std::sqrt(std::max(0.0, s2 / n - st.mean * st.mean));
    }
    return st;
}

int FirstValidColumn(const cam::DepthFrame& f, int row)
{
    for (int x = 0; x < f.width; ++x) {
        if (std::isfinite(f.data[static_cast<std::size_t>(row) * f.width + x])) return x;
    }
    return -1;
}

// Centre column of the pole in a row: pixels clearly nearer than the wall 16 px
// to both sides (the wall's z-depth itself varies when the camera is rotated).
double PoleColumn(const cam::DepthFrame& f, int row)
{
    double sum = 0.0;
    int n = 0;
    const auto at = [&](int x) { return f.data[static_cast<std::size_t>(row) * f.width + x]; };
    for (int x = f.width / 2; x < f.width - 16; ++x) {
        const float v = at(x);
        const float l = at(x - 16);
        const float r = at(x + 16);
        if (std::isfinite(v) && std::isfinite(l) && std::isfinite(r) && v < std::min(l, r) - 0.05F) {
            sum += x;
            ++n;
        }
    }
    return n ? sum / n : -1.0;
}

void SetYawRate(XmlWorld& world, double rate)
{
    world.getData()->qvel[0] = rate;
    mj_forward(world.getModel(), world.getData());
}
}

int main()
{
    auto world = std::make_shared<XmlWorld>();
    world->loadModel(kScene);
    if (!world->getModel()) return 1;
    auto renderer = std::make_shared<cam::MujocoCameraRenderer>(world);
    const double z = 2.0;

    std::printf("[D435i: separate RGB camera, 848x480 depth / 1280x720 RGB, typical noise]\n");
    {
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435i", c);
        c.depth_model.seed = 3;
        rs::RealSenseCameraSensor sensor(renderer, "rs_depth", "rs_color");
        check(sensor.LoadConfig(c), "LoadConfig");
        cam::ImageFrame rgb;
        cam::DepthFrame depth;
        sensor.Capture(rgb, depth);
        check(depth.width == 848 && depth.height == 480, "depth size 848x480");
        check(depth.frame_id == "camera_depth_optical_frame", "depth frame id");
        check(rgb.width == 1280 && rgb.height == 720 && rgb.data.size() == 1280u * 720u * 3u, "rgb 1280x720");
        check(rgb.frame_id == "camera_color_optical_frame", "rgb frame id");

        const auto& m = sensor.GetDepthModelParams();
        const Stats st = ColumnStats(depth, 200, 600);
        close_to(st.mean, z, 0.005, "wall mean depth");
        close_to(st.sigma, rs::ExpectedDepthSigma(m, z), 0.1 * rs::ExpectedDepthSigma(m, z),
                 "depth RMS = z^2*0.08/(fx*B) at 848 px");
        const double band = rs::LeftInvalidBandPx(m, z);
        close_to(FirstValidColumn(depth, 240), std::ceil(band), 1.0, "left invalid band = fx*B/z");
    }

    std::printf("[D435: shared render, RGB/depth same FOV override, model disabled]\n");
    {
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435", c);
        c.profile.rgb_hfov_deg = c.profile.depth_hfov_deg;
        c.rgb.width = c.depth.width;
        c.rgb.height = c.depth.height;
        c.depth_model.enabled = false;
        rs::RealSenseCameraSensor sensor(renderer, "rs_depth");
        check(sensor.LoadConfig(c), "LoadConfig");
        cam::ImageFrame rgb;
        cam::DepthFrame depth;
        sensor.Capture(rgb, depth);
        const Stats st = ColumnStats(depth, 0, 600);
        close_to(st.mean, z, 0.005, "ideal depth mean");
        check(st.sigma < 0.005, "ideal depth has no stereo noise");
        check(st.valid == static_cast<std::size_t>(600) * depth.height, "no left band when disabled");
        check(rgb.width == depth.width && !rgb.data.empty(), "shared-render RGB");
    }

    std::printf("[D435: post-processing with decimation]\n");
    {
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435", c);
        c.post_processing.enabled = true;
        c.post_processing.decimation.enabled = true;
        c.post_processing.spatial.enabled = true;
        c.post_processing.temporal.enabled = true;
        c.post_processing.hole_filling.enabled = true;
        c.post_processing.spatial.holes_fill = 5;   // unlimited: also closes the left band
        rs::RealSenseCameraSensor sensor(renderer, "rs_depth");
        check(sensor.LoadConfig(c), "LoadConfig");
        cam::DepthFrame depth;
        for (int i = 0; i < 3; ++i) sensor.CaptureDepth(depth);
        check(depth.width == 424 && depth.height == 240, "decimated output 424x240");
        check(sensor.GetDepthCameraConfig().image.width == 424
                  && sensor.GetDepthCameraConfig().image.height == 240, "DepthCameraConfig follows decimation");
        const Stats st = ColumnStats(depth, 100, 300);
        close_to(st.mean, z, 0.005, "filtered mean depth");
        check(st.sigma < rs::ExpectedDepthSigma(sensor.GetDepthModelParams(), z) * 0.7, "filters reduce noise");
        check(FirstValidColumn(depth, 120) == 0, "hole filling closes the left band");
    }

    std::printf("[D421: no RGB; D405: out of range]\n");
    {
        rs::RealSenseCameraSensor d421(renderer, "rs_depth");
        check(d421.LoadConfig(std::string(HAKO_SENSOR_SOURCE_DIR) + "/config/realsense/d421.json"), "load d421.json");
        cam::ImageFrame rgb;
        cam::DepthFrame depth;
        d421.Capture(rgb, depth);
        check(rgb.data.empty() && rgb.width == 0, "D421 RGB empty");
        check(ColumnStats(depth, 300, 548).valid > 0, "D421 depth valid at 2 m");

        // D405 max range is 0.5 m: a wall at 2 m is out of range.
        rs::RealSenseCameraSensor d405(renderer, "rs_depth");
        check(d405.LoadConfig(std::string(HAKO_SENSOR_SOURCE_DIR) + "/config/realsense/d405.json"), "load d405.json");
        d405.Capture(rgb, depth);
        check(ColumnStats(depth, 0, depth.width).valid == 0, "D405: 2 m is beyond its range -> all invalid");
    }

    std::printf("[existing DepthCameraSensor alongside]\n");
    {
        cam::DepthCameraSensor generic(renderer, "rs_depth");
        cam::DepthCameraConfig dc {};
        check(generic.LoadConfig(dc), "generic LoadConfig");
        cam::DepthFrame depth;
        generic.Capture(depth);
        close_to(ColumnStats(depth, 0, depth.width / 2).mean, z, 0.005, "generic depth camera unchanged");
    }

    std::printf("[D415 rolling shutter while the rig yaws at 5 rad/s]\n");
    {
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D415", c);
        c.depth_model.enabled = false;   // isolate the geometry
        rs::RealSenseCameraSensor sensor(renderer, "rs_depth");
        check(sensor.LoadConfig(c), "LoadConfig");
        sensor.SetMotionSource(world);
        cam::DepthFrame still;
        sensor.CaptureDepth(still);
        const double col_still_top = PoleColumn(still, 0);
        const double col_still_bot = PoleColumn(still, still.height - 1);
        check(col_still_top > 0 && std::fabs(col_still_top - col_still_bot) < 1.0, "static: pole is vertical");

        SetYawRate(*world, 5.0);
        cam::DepthFrame moving;
        sensor.CaptureDepth(moving);
        // Rows near the top/bottom edge look outside the rendered frame while rotating
        // (they become invalid, as nothing was rendered there); compare rows inside.
        const double fx = rs::FocalLengthPx(still.width, c.profile.DepthHfovRad());
        const double cx = still.width * 0.5;
        const double xn = (PoleColumn(still, still.height / 2) - cx) / fx;
        const auto expected_col = [&](int row) {
            // optical yaw rate = -5 rad/s; row r sees the reference rotated by theta(r)
            const double dt = (row - 0.5 * (still.height - 1)) * c.DepthReadoutS() / still.height;
            const double theta = -5.0 * dt;
            return cx + fx * std::tan(std::atan(xn) - theta);
        };
        const int r_top = 60;
        const int r_bot = still.height - 61;
        std::printf("  pole column still %.1f, moving row%d %.1f (want %.1f), row%d %.1f (want %.1f)\n",
                    col_still_top, r_top, PoleColumn(moving, r_top), expected_col(r_top), r_bot,
                    PoleColumn(moving, r_bot), expected_col(r_bot));
        close_to(PoleColumn(moving, r_top), expected_col(r_top), 2.0, "pole column (upper row)");
        close_to(PoleColumn(moving, r_bot), expected_col(r_bot), 2.0, "pole column (lower row)");
        check(std::fabs(PoleColumn(moving, r_top) - PoleColumn(moving, r_bot)) > 30.0, "pole is skewed");
        close_to(PoleColumn(moving, moving.height / 2), PoleColumn(still, still.height / 2), 2.0,
                 "middle row unchanged");
    }

    std::printf("[D435i IMU on the yawing rig]\n");
    {
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435i", c);
        c.imu.bias_mode = "none";
        rs::RealSenseImuSensor imu(world, "rs_depth");
        check(imu.LoadConfig(c), "IMU LoadConfig");
        std::vector<rs::MotionSample> accel;
        std::vector<rs::MotionSample> gyro;
        imu.Update(0.0101, accel, gyro);
        check(accel.size() == 2 && gyro.size() == 4, "10 ms -> 2 accel (250 Hz) + 4 gyro (400 Hz)");
        rs::Vec3d a {};
        rs::Vec3d w {};
        check(imu.TruthAccel(a) && imu.TruthGyro(w), "truth available");
        const double g = std::fabs(world->getModel()->opt.gravity[2]);
        close_to(a.y, -g, 1e-6, "accel truth: y = -g (optical y down)");
        close_to(w.y, -5.0, 1e-9, "gyro truth: yaw +5 rad/s about up -> optical y = -5");
        const double gs = imu.NoiseModel().GyroSigma();
        const double as = imu.NoiseModel().AccelSigma();
        close_to(gyro.back().value.y, -5.0, 6.0 * gs, "gyro sample");
        close_to(accel.back().value.y, -g, 6.0 * as + imu.NoiseModel().AccelLsb(), "accel sample");
        hako::robots::sensor::ImuFrame frame {};
        check(imu.GetLatest(frame) && frame.header.frame_id == "camera_imu_optical_frame", "combined ImuFrame");
        SetYawRate(*world, 0.0);

        rs::RealSenseConfig d415 {};
        rs::MakeDefaultConfig("D415", d415);
        rs::RealSenseImuSensor none(world, "rs_depth");
        check(none.LoadConfig(d415) && !none.Enabled(), "D415 has no IMU");
    }

    std::printf("[RS-1 D435 edge effects: box at 0.6 m, wall at 1.0 m]\n");
    {
        const int row = 240;
        const auto at = [](const cam::DepthFrame& f, int y, int x) {
            return f.data[static_cast<std::size_t>(y) * f.width + x];
        };
        const auto box_left = [&](const cam::DepthFrame& f) {
            for (int x = 0; x < f.width; ++x) {
                const float v = at(f, row, x);
                if (std::isfinite(v) && v < 0.8F) return x;
            }
            return -1;
        };
        const auto box_right = [&](const cam::DepthFrame& f) {
            for (int x = f.width - 1; x >= 0; --x) {
                const float v = at(f, row, x);
                if (std::isfinite(v) && v < 0.8F) return x;
            }
            return -1;
        };

        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435", c);
        c.depth_model.enabled = false;   // isolate the geometry
        c.edge_effects.enabled = true;
        c.edge_effects.flying_pixels.enabled = false;
        rs::RealSenseCameraSensor sensor(renderer, "rs_edge");
        check(sensor.LoadConfig(c), "LoadConfig (occlusion only)");
        cam::DepthFrame depth;
        sensor.CaptureDepth(depth);
        const int xl = box_left(depth);
        const int xr = box_right(depth);
        int shadow = 0;
        for (int x = xl - 1; x >= 0 && !std::isfinite(at(depth, row, x)); --x) ++shadow;
        const auto& m = sensor.GetDepthModelParams();
        const double want = rs::OcclusionShadowPx(m.fx_px, m.baseline_m, 0.6, 1.0);
        std::printf("  box columns %d..%d, shadow %d px\n", xl, xr, shadow);
        check(xl > 0 && xr > xl, "box found");
        close_to(shadow, want, 1.5, "occlusion shadow = fx*B*(1/0.6 - 1/1.0)");
        bool right_ok = true;
        for (int x = xr + 1; x < depth.width; ++x) right_ok = right_ok && std::isfinite(at(depth, row, x));
        check(right_ok, "no shadow right of the box");
        check(sensor.GetLastEdgeEffectsStats().occluded > 0, "stats report occluded pixels");

        c.edge_effects.flying_pixels.enabled = true;
        c.edge_effects.flying_pixels.probability = 1.0;
        check(sensor.LoadConfig(c), "LoadConfig (occlusion + flying)");
        sensor.CaptureDepth(depth);
        const int r = sensor.GetEdgeEffectsParams().window_radius_px;
        int flying = 0;
        bool near_edges = true;
        for (int x = 0; x < depth.width; ++x) {
            const float v = at(depth, row, x);
            if (!std::isfinite(v) || v <= 0.62F || v >= 0.98F) continue;
            ++flying;
            near_edges = near_edges && (std::abs(x - xl) <= r + 1 || std::abs(x - xr) <= r + 1);
        }
        std::printf("  flying pixels in row %d: %d (r = %d)\n", row, flying, r);
        check(flying > 0, "flying pixels appear on the box edges");
        check(near_edges, "flying pixels only within r of the edges");

        // Edge effects off: bit-identical to a config without them (other params changed).
        rs::RealSenseConfig a {};
        rs::MakeDefaultConfig("D435", a);
        a.depth_model.seed = 5;
        rs::RealSenseConfig b = a;
        b.edge_effects.enabled = false;
        b.edge_effects.flying_pixels.probability = 0.9;
        b.edge_effects.seed = 77;
        rs::RealSenseCameraSensor sa(renderer, "rs_edge");
        rs::RealSenseCameraSensor sb(renderer, "rs_edge");
        check(sa.LoadConfig(a) && sb.LoadConfig(b), "LoadConfig (regression pair)");
        cam::DepthFrame da;
        cam::DepthFrame db;
        sa.CaptureDepth(da);
        sb.CaptureDepth(db);
        bool same = da.data.size() == db.data.size();
        for (std::size_t i = 0; same && i < da.data.size(); ++i) {
            same = (std::isnan(da.data[i]) && std::isnan(db.data[i])) || da.data[i] == db.data[i];
        }
        check(same, "edge effects off -> bit-identical depth");

        // Metrics of the full chain (typical noise + edges) against the ideal depth.
        rs::RealSenseConfig ideal_c {};
        rs::MakeDefaultConfig("D435", ideal_c);
        ideal_c.depth_model.enabled = false;
        rs::RealSenseCameraSensor ideal(renderer, "rs_edge");
        check(ideal.LoadConfig(ideal_c), "LoadConfig (ideal)");
        cam::DepthFrame truth;
        ideal.CaptureDepth(truth);
        rs::RealSenseConfig full = a;
        full.edge_effects.enabled = true;
        rs::RealSenseCameraSensor sf(renderer, "rs_edge");
        check(sf.LoadConfig(full), "LoadConfig (full)");
        cam::DepthFrame df;
        sf.CaptureDepth(df);
        const auto mb = rs::ComputeDepthMetrics(truth.data, da.data, sa.GetDepthModelParams());
        const auto mf = rs::ComputeDepthMetrics(truth.data, df.data, sf.GetDepthModelParams());
        std::printf("  metrics without edges: missing %.4f outlier %.5f rms %.2f mm\n",
                    mb.missing_rate, mb.outlier_rate, mb.rms_m * 1e3);
        std::printf("  metrics with edges   : missing %.4f outlier %.5f rms %.2f mm\n",
                    mf.missing_rate, mf.outlier_rate, mf.rms_m * 1e3);
        check(mf.missing_rate > mb.missing_rate && mf.outlier_rate > mb.outlier_rate,
              "edges add missing pixels and outliers");
    }

    std::printf("[RS-2 D435 texture model: black / aluminum / cardboard panels at 2.45 m]\n");
    {
        // Panel columns (camera right = world -x): black x=+0.45, aluminum 0, cardboard -0.45.
        const int cols[3][2] = {{313, 370}, {395, 452}, {477, 534}};
        const char* names[3] = {"black oxide", "aluminum", "cardboard"};
        const auto hole_rate = [&](const cam::DepthFrame& f, int k) {
            std::size_t n = 0, holes = 0;
            for (int y = 210; y <= 270; ++y) {
                for (int x = cols[k][0]; x <= cols[k][1]; ++x) {
                    ++n;
                    holes += std::isfinite(f.data[static_cast<std::size_t>(y) * f.width + x]) ? 0u : 1u;
                }
            }
            return static_cast<double>(holes) / static_cast<double>(n);
        };
        const auto j = nlohmann::json::parse(R"({ "model": "D435",
            "texture_model": { "enabled": true, "materials": "materials.json", "seed": 3 } })");
        rs::RealSenseConfig c {};
        std::string err;
        check(rs::RealSenseConfigFromJson(j, c, err, std::string(HAKO_SENSOR_SOURCE_DIR) + "/config/realsense"),
              "config with materials.json: " + err);
        c.depth_model.seed = 5;
        rs::RealSenseCameraSensor sensor(renderer, "rs_mat");
        sensor.SetMaterialSource(world);
        check(sensor.LoadConfig(c), "LoadConfig (texture model)");
        cam::DepthFrame depth;
        sensor.CaptureDepth(depth);
        double hr[3];
        for (int k = 0; k < 3; ++k) hr[k] = hole_rate(depth, k);
        std::printf("  projector on : holes %s %.3f, %s %.3f, %s %.3f\n", names[0], hr[0], names[1], hr[1], names[2], hr[2]);
        const auto& mats = sensor.GetResolvedMaterials();
        const int black_id = mj_name2id(world->getModel(), mjOBJ_MATERIAL, "work_black_plate");
        check(black_id >= 0 && static_cast<std::size_t>(black_id) < mats.size()
                  && mats[static_cast<std::size_t>(black_id)].ir_reflectance == 0.05,
              "work_black_plate -> black_oxide via work_black*");
        // Model value on the optical axis (the patch is within ~15 deg of it).
        const auto& tp = sensor.GetTextureModelParams();
        const double want_black = 1.0 - rs::ValidProbability(
            tp, rs::TextureSnr(tp, mats[static_cast<std::size_t>(black_id)], 2.45, 1.0));
        close_to(hr[0], want_black, 0.08, "black oxide at 2.45 m: hole rate follows the model");
        check(hr[0] > 0.3, "black oxide loses a large part of the depth");
        check(hr[1] < 0.02 && hr[2] < 0.02, "aluminum and cardboard stay valid");
        check(sensor.GetLastTextureStats().holes > 0, "stats report holes");

        c.texture_model.laser_power_mw = 0.0;
        check(sensor.LoadConfig(c), "LoadConfig (projector off)");
        sensor.CaptureDepth(depth);
        for (int k = 0; k < 3; ++k) hr[k] = hole_rate(depth, k);
        std::printf("  projector off: holes %s %.3f, %s %.3f, %s %.3f\n", names[0], hr[0], names[1], hr[1], names[2], hr[2]);
        check(hr[1] > 0.9, "projector off: texture-less aluminum is lost");
        check(hr[2] > 0.3 && hr[2] < 0.7, "projector off: textured cardboard keeps about half");

        // Texture model off (table still loaded): bit-identical to the plain config.
        rs::RealSenseConfig plain {};
        rs::MakeDefaultConfig("D435", plain);
        plain.depth_model.seed = 5;
        rs::RealSenseConfig off = c;
        off.texture_model.enabled = false;
        rs::RealSenseCameraSensor sp(renderer, "rs_mat");
        rs::RealSenseCameraSensor so(renderer, "rs_mat");
        so.SetMaterialSource(world);
        check(sp.LoadConfig(plain) && so.LoadConfig(off), "LoadConfig (regression pair)");
        cam::DepthFrame dp;
        cam::DepthFrame dof;
        sp.CaptureDepth(dp);
        so.CaptureDepth(dof);
        bool same = dp.data.size() == dof.data.size();
        for (std::size_t i = 0; same && i < dp.data.size(); ++i) {
            same = (std::isnan(dp.data[i]) && std::isnan(dof.data[i])) || dp.data[i] == dof.data[i];
        }
        check(same, "texture model off -> bit-identical depth");
    }

    std::printf("[RS-3 D435 specular model: face-on and 45 deg aluminum plates]\n");
    {
        auto sworld = std::make_shared<XmlWorld>();
        sworld->loadModel(kSpecScene);
        check(sworld->getModel() != nullptr, "specular scene loads");
        auto srenderer = std::make_shared<cam::MujocoCameraRenderer>(sworld);
        const auto patch = [](const cam::DepthFrame& f, int u0, int u1, int v0, int v1, std::vector<float>& vals) {
            std::size_t n = 0, holes = 0;
            vals.clear();
            for (int y = v0; y <= v1; ++y) {
                for (int x = u0; x <= u1; ++x) {
                    const float v = f.data[static_cast<std::size_t>(y) * f.width + x];
                    ++n;
                    if (std::isfinite(v)) vals.push_back(v); else ++holes;
                }
            }
            return static_cast<double>(holes) / static_cast<double>(n);
        };
        const auto j = nlohmann::json::parse(R"({ "model": "D435",
            "texture_model": { "materials": "materials.json" },
            "specular_model": { "enabled": true, "seed": 9 } })");
        rs::RealSenseConfig c {};
        std::string err;
        check(rs::RealSenseConfigFromJson(j, c, err, std::string(HAKO_SENSOR_SOURCE_DIR) + "/config/realsense"),
              "config with specular_model: " + err);
        c.depth_model.seed = 5;
        rs::RealSenseCameraSensor sensor(srenderer, "rs_spec");
        sensor.SetMaterialSource(sworld);
        check(sensor.LoadConfig(c), "LoadConfig (specular)");
        cam::DepthFrame depth;
        const auto t0 = std::chrono::steady_clock::now();
        sensor.CaptureDepth(depth);
        const double ms_on = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const auto st = sensor.GetLastSpecularStats();

        std::vector<float> vals;
        // Face-on plate centre at u ~ 238 (y = +0.25 -> image left), v = 240.
        const double centre = patch(depth, 228, 248, 230, 250, vals);
        // ~9.5 deg off the plate normal; the projector at +B/2 shifts the mirror
        // direction ~1.1 deg to the right -> alpha ~ 8.4 deg -> p ~ 0.8 * e^-1.1 ~ 0.27
        const double side = patch(depth, 315, 325, 230, 250, vals);
        std::printf("  face-on plate: highlight holes centre %.3f, ~8 deg off %.3f\n", centre, side);
        check(centre > 0.6, "face-on shiny plate: highlight hole at the centre");
        check(side < centre - 0.3, "highlight fades away from the mirror direction");

        // Vertical aluminum shaft (r = 4 cm at 0.76 m, centre column 424): a narrow
        // highlight band where the surface faces the camera, none on the flanks.
        const double band = patch(depth, 421, 427, 225, 255, vals);
        const double flank = patch(depth, 436, 442, 225, 255, vals);
        const double flank_l = patch(depth, 406, 412, 225, 255, vals);
        std::printf("  shaft: highlight holes centre band %.3f, flanks %.3f / %.3f\n", band, flank_l, flank);
        // The rendered cylinder is a polygon prism (stepwise normals), so the band is
        // somewhat weaker than the smooth-cylinder value (~0.68).
        check(band > 0.3 && band > std::max(flank, flank_l) + 0.3, "shiny cylinder: highlight band along the axis");
        check(flank < 0.15 && flank_l < 0.15, "cylinder flanks keep their depth");

        // 45 deg plate centre at u ~ 615 (y = -0.3 -> image right).
        patch(depth, 603, 627, 230, 250, vals);
        std::size_t mirrored = 0;
        std::vector<float> mir;
        for (float v : vals) if (v > 0.85F) { ++mirrored; mir.push_back(v); }
        std::sort(mir.begin(), mir.end());
        const double frac = vals.empty() ? 0.0 : static_cast<double>(mirrored) / vals.size();
        const double med = mir.empty() ? 0.0 : mir[mir.size() / 2];
        std::printf("  45 deg plate: mirrored %.3f of valid pixels, median depth %.3f m (plate ~0.70 m), rays %zu (hits %zu), mirrored px %zu, %.1f ms\n",
                    frac, med, st.rays, st.ray_hits, st.mirror_pixels, ms_on);
        // p = mirror_gain * specular(0.8) * rho_wall(white resin 0.8) = 0.64
        close_to(frac, 0.64, 0.12, "mirror image fraction = gain * specular * rho");
        close_to(med, 1.0, 0.05, "mirror image at z * (range + t) / range ~ 1.0 m");
        check(st.rays > 0 && st.mirror_pixels > 0 && st.highlight_holes > 0, "stats");

        // Specular model off: bit-identical to the plain config.
        rs::RealSenseConfig plain {};
        rs::MakeDefaultConfig("D435", plain);
        plain.depth_model.seed = 5;
        rs::RealSenseConfig off = c;
        off.specular_model.enabled = false;
        rs::RealSenseCameraSensor sp(srenderer, "rs_spec");
        rs::RealSenseCameraSensor so(srenderer, "rs_spec");
        so.SetMaterialSource(sworld);
        check(sp.LoadConfig(plain) && so.LoadConfig(off), "LoadConfig (regression pair)");
        cam::DepthFrame dp;
        cam::DepthFrame dof;
        sp.CaptureDepth(dp);
        const auto t1 = std::chrono::steady_clock::now();
        so.CaptureDepth(dof);
        const double ms_off = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();
        std::printf("  capture time: specular on %.1f ms, off %.1f ms\n", ms_on, ms_off);
        bool same = dp.data.size() == dof.data.size();
        for (std::size_t i = 0; same && i < dp.data.size(); ++i) {
            same = (std::isnan(dp.data[i]) && std::isnan(dof.data[i])) || dp.data[i] == dof.data[i];
        }
        check(same, "specular model off -> bit-identical depth");
    }

    std::printf("[RS-4 D435i thermal drift on the 2 m wall]\n");
    {
        mjData* d = world->getData();
        SetYawRate(*world, 0.0);
        d->time = 0.0;
        mj_forward(world->getModel(), d);
        rs::RealSenseConfig plain {};
        rs::MakeDefaultConfig("D435i", plain);
        plain.depth_model.seed = 21;
        rs::RealSenseConfig hot = plain;
        hot.thermal.enabled = true;
        const auto mean_diff = [&](rs::RealSenseCameraSensor& a, rs::RealSenseCameraSensor& b) {
            cam::DepthFrame fa;
            cam::DepthFrame fb;
            a.CaptureDepth(fa);
            b.CaptureDepth(fb);
            return ColumnStats(fb, 200, 600).mean - ColumnStats(fa, 200, 600).mean;
        };
        rs::RealSenseCameraSensor ref(renderer, "rs_depth");
        rs::RealSenseCameraSensor th(renderer, "rs_depth");
        check(ref.LoadConfig(plain) && th.LoadConfig(hot), "LoadConfig (thermal)");
        const double fb = th.GetDepthModelParams().fx_px * th.GetDepthModelParams().baseline_m;
        const double want_cold = fb / (fb / 2.0 - 0.075) - 2.0;
        const double cold = mean_diff(ref, th);
        std::printf("  cold start: T %.2f C, offset %+.4f px, wall mean %+.2f mm\n", th.GetLastTemperatureC(),
                    th.GetLastDisparityOffsetPx(), cold * 1e3);
        close_to(th.GetLastTemperatureC(), 25.0, 1e-9, "cold start temperature = ambient");
        close_to(cold, want_cold, 0.0015, "cold start: wall reads farther by fB/(fB/z - 0.075) - z");

        d->time = 3600.0;   // one hour later: warm (tau = 10 min)
        const double warm = mean_diff(ref, th);
        std::printf("  after 1 h : T %.2f C, offset %+.4f px, wall mean %+.2f mm\n", th.GetLastTemperatureC(),
                    th.GetLastDisparityOffsetPx(), warm * 1e3);
        close_to(warm, 0.0, 0.0015, "warm camera reads true (factory calibration is warm)");

        d->time = 0.0;
        rs::RealSenseCameraSensor ref2(renderer, "rs_depth");
        rs::RealSenseCameraSensor cal(renderer, "rs_depth");
        check(ref2.LoadConfig(plain) && cal.LoadConfig(hot), "LoadConfig (calibrated)");
        cal.RunOnChipCalibration(0.0);
        close_to(mean_diff(ref2, cal), 0.0, 0.0015, "on-chip calibration at the cold start removes the offset");

        // IMU at rest, noise / bias / quantisation off: reads the thermal drift only.
        rs::RealSenseConfig ic = hot;
        ic.imu.bias_mode = "none";
        ic.imu.noise_scale = 0.0;
        ic.imu.quantize = false;
        ic.imu.saturate = false;
        rs::RealSenseImuSensor imu(world, "rs_depth");
        check(imu.LoadConfig(ic), "IMU LoadConfig (thermal)");
        std::vector<rs::MotionSample> accel;
        std::vector<rs::MotionSample> gyro;
        imu.Update(0.0101, accel, gyro);
        const auto tco = imu.GetThermalModel().GyroTco();
        check(!gyro.empty(), "gyro samples");
        close_to(gyro.back().value.x, tco.x * (25.0 - 40.0), 1e-9, "IMU gyro x = TCO * (T - T_cal) at rest");
        imu.RunImuCalibration(0.0);
        accel.clear();
        gyro.clear();
        imu.Update(0.0101, accel, gyro);
        close_to(gyro.back().value.x, 0.0, 1e-9, "IMU calibration removes the drift");
        d->time = 0.0;
    }

    std::printf("[RS-5 D435 exposure / motion blur: box scene, camera yaw 3 rad/s]\n");
    {
        rs::CameraMotion yaw {};
        yaw.wy = 3.0;   // optical y = vertical axis
        const auto band = [](const cam::DepthFrame& f) {
            int n = 0;
            for (int x = 300; x < 560; ++x) n += std::isfinite(f.data[240 * static_cast<std::size_t>(f.width) + x]) ? 0 : 1;
            return n;
        };
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435", c);
        c.depth_model.enabled = false;
        c.exposure.enabled = true;
        c.exposure.auto_exposure = false;
        rs::RealSenseCameraSensor sensor(renderer, "rs_edge");
        cam::DepthFrame depth;
        check(sensor.LoadConfig(c), "LoadConfig (exposure)");
        sensor.CaptureDepth(depth);
        const int still = band(depth);   // left occlusion-free: depth model off -> only the box itself
        sensor.SetCameraMotion(yaw);
        int bands[2] = {0, 0};
        const double times[2] = {0.002, 0.010};
        for (int i = 0; i < 2; ++i) {
            c.exposure.depth_exposure_us = times[i] * 1e6;
            check(sensor.LoadConfig(c), "LoadConfig (manual exposure)");
            sensor.CaptureDepth(depth);
            bands[i] = band(depth) - still;
        }
        const double fx = sensor.GetDepthModelParams().fx_px;
        const double want2 = 2.0 * 0.75 * rs::ExpectedBlurPx(3.0, 0.002, fx);
        const double want10 = 2.0 * 0.75 * rs::ExpectedBlurPx(3.0, 0.010, fx);
        std::printf("  depth holes from blur: 2 ms %d px (want %.1f), 10 ms %d px (want %.1f)\n", bands[0], want2,
                    bands[1], want10);
        close_to(bands[0], want2, 3.0, "2 ms exposure: narrow smear band");
        close_to(bands[1], want10, 4.0, "10 ms exposure: band ~ 2 x (N-1)/N x w t fx");
        check(sensor.GetLastBlurStats().smeared > 0, "blur stats");

        // RGB (separate 1280x720 render, rolling): edges smear with the exposure.
        const auto transitions = [](const cam::ImageFrame& f) {
            const int row = f.height / 2;
            int lo = 255, hi = 0;
            std::vector<int> luma(static_cast<std::size_t>(f.width));
            for (int x = 0; x < f.width; ++x) {
                const std::uint8_t* p = &f.data[(static_cast<std::size_t>(row) * f.width + x) * 3];
                luma[static_cast<std::size_t>(x)] = (299 * p[0] + 587 * p[1] + 114 * p[2]) / 1000;
                lo = std::min(lo, luma[static_cast<std::size_t>(x)]);
                hi = std::max(hi, luma[static_cast<std::size_t>(x)]);
            }
            int n = 0;
            const int a = lo + (hi - lo) / 5, b = hi - (hi - lo) / 5;
            for (int v : luma) n += (v > a && v < b) ? 1 : 0;
            return n;
        };
        rs::RealSenseConfig cr {};
        rs::MakeDefaultConfig("D435", cr);
        cr.exposure.enabled = true;
        cr.exposure.auto_exposure = false;
        cr.exposure.rgb_gain_noise_dn = 0.0;
        cr.exposure.motion_blur.subframes = 8;
        rs::RealSenseCameraSensor rgb_sensor(renderer, "rs_edge");
        check(rgb_sensor.LoadConfig(cr), "LoadConfig (RGB exposure)");
        cam::ImageFrame rgb;
        rgb_sensor.Capture(rgb, depth);
        const int t_still = transitions(rgb);
        rgb_sensor.SetCameraMotion(yaw);
        rgb_sensor.Capture(rgb, depth);
        const int t_blur = transitions(rgb);
        const double fx_rgb = rgb_sensor.GetRgbCameraConfig().intrinsics.fx;
        std::printf("  RGB transition pixels in the middle row: still %d, blurred %d (blur %.1f px)\n", t_still, t_blur,
                    rs::ExpectedBlurPx(3.0, 0.010, fx_rgb));
        check(t_blur > t_still + 15, "RGB edges smear under motion");

        // Auto exposure: dark RGB scene -> exposure capped at 1/fps, gain up.
        cr.exposure.auto_exposure = true;
        cr.exposure.rgb_light_level = 0.2;
        check(rgb_sensor.LoadConfig(cr), "LoadConfig (auto, dark)");
        rgb_sensor.Capture(rgb, depth);
        const auto& re = rgb_sensor.GetLastRgbExposure();
        std::printf("  auto exposure dark RGB: %.1f ms, gain %.1f, brightness %.2f; depth %.2f ms\n",
                    re.exposure_s * 1e3, re.gain, re.brightness, rgb_sensor.GetLastDepthExposure().exposure_s * 1e3);
        close_to(re.exposure_s, 1.0 / 30.0, 1e-9, "dark scene: exposure 1/fps");
        close_to(re.gain, 16.0 * (0.010 / 0.2) / (1.0 / 30.0), 1e-6, "dark scene: gain raised");
        check(rgb_sensor.GetLastDepthExposure().exposure_s < 0.0085,
              "depth auto exposure: box at 0.6 m is bright -> shorter than 8.5 ms");

        // Exposure off (motion set): bit-identical to the plain config, depth and RGB.
        rs::RealSenseConfig plain {};
        rs::MakeDefaultConfig("D435", plain);
        rs::RealSenseConfig off = cr;
        off.exposure.enabled = false;
        rs::RealSenseCameraSensor sp(renderer, "rs_edge");
        rs::RealSenseCameraSensor so(renderer, "rs_edge");
        check(sp.LoadConfig(plain) && so.LoadConfig(off), "LoadConfig (regression pair)");
        sp.SetCameraMotion(yaw);
        so.SetCameraMotion(yaw);
        cam::ImageFrame rp;
        cam::ImageFrame ro;
        cam::DepthFrame dp;
        cam::DepthFrame dof;
        sp.Capture(rp, dp);
        so.Capture(ro, dof);
        bool same = dp.data.size() == dof.data.size() && rp.data == ro.data;
        for (std::size_t i = 0; same && i < dp.data.size(); ++i) {
            same = (std::isnan(dp.data[i]) && std::isnan(dof.data[i])) || dp.data[i] == dof.data[i];
        }
        check(same, "exposure off -> bit-identical depth and RGB");
    }

    std::printf("[RS-6 D435i IMU timing / lever arm / misalignment, frame timing]\n");
    {
        mjData* d = world->getData();
        d->time = 0.0;
        SetYawRate(*world, 5.0);
        rs::RealSenseConfig c {};
        rs::MakeDefaultConfig("D435i", c);
        c.imu.bias_mode = "none";
        c.imu.noise_scale = 0.0;
        c.imu.quantize = false;
        c.imu.saturate = false;
        c.imu_timing.enabled = true;
        c.imu_timing.params.position_m[0] = 0.05;   // 5 cm to the right of the depth origin
        c.imu_timing.params.time_offset_s = 0.003;
        rs::RealSenseImuSensor imu(world, "rs_depth");
        check(imu.LoadConfig(c), "IMU LoadConfig (timing)");
        std::vector<rs::MotionSample> accel;
        std::vector<rs::MotionSample> gyro;
        imu.Update(0.0101, accel, gyro);
        check(!accel.empty() && !gyro.empty(), "samples");
        std::printf("  accel at the chip: (%.4f %.4f %.4f), gyro y %.4f\n", accel.back().value.x, accel.back().value.y,
                    accel.back().value.z, gyro.back().value.y);
        close_to(accel.back().value.x, -25.0 * 0.05, 1e-6, "lever arm: centripetal -w^2 r = -1.25 m/s^2");
        close_to(gyro.back().value.y, -5.0, 1e-9, "gyro unchanged by the lever arm");
        close_to(gyro.back().timestamp, 0.0 - 0.0001 + 0.003, 0.0026, "stamps carry the +3 ms offset");
        check(gyro.back().timestamp > 0.0, "offset stamp after the sample time");

        c.imu_timing.params.position_m[0] = 0.0;
        c.imu_timing.params.misalignment_deg = 1.0;
        c.imu_timing.params.seed = 2;
        SetYawRate(*world, 0.0);
        rs::RealSenseImuSensor tilt(world, "rs_depth");
        check(tilt.LoadConfig(c), "IMU LoadConfig (misalignment)");
        accel.clear();
        gyro.clear();
        tilt.Update(0.0101, accel, gyro);
        const auto& g = accel.back().value;
        const double horiz = std::sqrt(g.x * g.x + g.z * g.z);
        const double ang = std::atan2(horiz, -g.y);
        std::printf("  misaligned gravity: (%.4f %.4f %.4f), tilt %.3f deg (rotation %.3f deg)\n", g.x, g.y, g.z,
                    ang / rs::kDegToRad, tilt.GetTimingModel().MisalignmentAngleRad() / rs::kDegToRad);
        check(ang > 0.0 && ang <= tilt.GetTimingModel().MisalignmentAngleRad() + 1e-9,
              "gravity tilts by at most the misalignment angle");

        // camera frames
        rs::RealSenseConfig fc {};
        rs::MakeDefaultConfig("D435", fc);
        fc.frame_timing.enabled = true;
        fc.frame_timing.params.drop_probability = 1.0;
        rs::RealSenseCameraSensor drop(renderer, "rs_depth");
        check(drop.LoadConfig(fc), "LoadConfig (frame drop)");
        cam::DepthFrame df;
        cam::ImageFrame rf;
        drop.Capture(rf, df);
        check(drop.LastFrameDropped() && df.data.empty() && rf.data.empty() && drop.DroppedFrames() == 1,
              "dropped frame: depth and RGB cleared");
        fc.frame_timing.params.drop_probability = 0.0;
        fc.frame_timing.params.jitter_s = 0.002;
        d->time = 1.0;
        rs::RealSenseCameraSensor jit(renderer, "rs_depth");
        check(jit.LoadConfig(fc), "LoadConfig (frame jitter)");
        jit.Capture(rf, df);
        std::printf("  jittered frame stamp: depth %.5f, rgb %.5f (sim 1.0)\n", df.timestamp, rf.timestamp);
        check(!jit.LastFrameDropped() && df.timestamp != 1.0 && std::fabs(df.timestamp - 1.0) < 0.012
                  && rf.timestamp == df.timestamp,
              "frame stamp jittered, depth and RGB share it");
        d->time = 0.0;
    }

    std::printf("realsense_render_smoke: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
