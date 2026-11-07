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
//   - the generic DepthCameraSensor still works next to it (existing API).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "physics.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/mujoco_camera_renderer.hpp"
#include "sensors/camera/realsense/realsense_camera_sensor.hpp"
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

    std::printf("realsense_render_smoke: %d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
