#pragma once

// RealSense depth camera on the MuJoCo renderer (HAKO_SENSOR_BUILD_CAMERA=ON).
//
// Added alongside the generic cameras: CameraSensor / DepthCameraSensor /
// RgbdCameraSensor / StereoCameraSensor are unchanged.
//
// Pipeline per capture:
//   render (MuJoCo)
//   -> rolling-shutter distortion  (rolling streams only, needs SetMotionSource)
//   -> depth-edge effects          (realsense_edge_effects.hpp, optional: occlusion
//                                   shadow + flying pixels)
//   -> stereo depth model          (realsense_depth_model.hpp)
//   -> post-processing             (librealsense-style filters, optional)
//
// Differences from RgbdCameraSensor:
//   - depth and RGB streams may have different FOV and resolution (as on the
//     real camera, e.g. D435 depth 87 deg / RGB 69 deg); they are rendered
//     separately when they differ.
//   - RGB may come from another MJCF camera (the RGB imager sits beside the
//     left depth imager on the real device).
//   - models without RGB (D421) return an empty RGB frame.
//
// The derived CameraConfig / DepthCameraConfig (output size after decimation)
// can be passed to the existing CameraInfo / Image PDU converters unchanged.
// The IMU is a separate sensor: RealSenseImuSensor (realsense_imu_sensor.hpp).

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "physics.hpp"
#include "sensors/camera/camera_sensor.hpp"
#include "sensors/camera/realsense/realsense_config.hpp"
#include "sensors/camera/realsense/realsense_post_processing.hpp"
#include "sensors/camera/realsense/realsense_rolling_shutter.hpp"

namespace hako::robots::sensor::camera::realsense
{
    class RealSenseCameraSensor : public CameraSensorBase
    {
    public:
        // rgb_camera_name empty = render RGB from the depth camera.
        RealSenseCameraSensor(
            std::shared_ptr<MujocoCameraRenderer> renderer,
            const std::string& depth_camera_name,
            const std::string& rgb_camera_name = "");
        ~RealSenseCameraSensor() override;

        bool LoadConfig(const std::string& path);
        bool LoadConfig(const RealSenseConfig& config);
        const RealSenseConfig& GetConfig() const { return config_; }

        // Camera ego-motion for rolling shutter: velocities of the MJCF cameras are
        // read from this world at capture time. Without it (or with
        // SetCameraMotion) rolling shutter has no effect.
        void SetMotionSource(std::shared_ptr<hako::robots::physics::IWorld> world);
        // Fixed motion (optical frame) instead of the world, e.g. for tests.
        void SetCameraMotion(const CameraMotion& motion);

        void Reset() override;

        // Generic camera configs derived from the profile (for PDU converters).
        const CameraConfig& GetRgbCameraConfig() const { return rgb_config_; }
        const DepthCameraConfig& GetDepthCameraConfig() const { return depth_config_; }
        const DepthModelParams& GetDepthModelParams() const { return depth_params_; }
        const EdgeEffectsParams& GetEdgeEffectsParams() const { return edge_params_; }
        // Counts of the last depth capture (zero when edge effects are off).
        const EdgeEffectsStats& GetLastEdgeEffectsStats() const { return last_edge_stats_; }
        bool HasRgb() const { return config_.RgbEnabled(); }

        // RGB is cleared when the model has no RGB or the stream is disabled.
        void Capture(ImageFrame& rgb_out, DepthFrame& depth_out);
        void CaptureDepth(DepthFrame& depth_out);

    private:
        bool RenderDepth(DepthFrame& depth_out, ImageFrame* rgb_out);
        bool RenderRgb(ImageFrame& rgb_out);
        bool SharedRender() const;
        CameraMotion MotionOf(const std::string& camera_name) const;
        void ProcessRgb(ImageFrame& rgb, const CameraMotion& motion, const std::vector<float>* ref_depth);

        std::shared_ptr<MujocoCameraRenderer> renderer_;
        std::string depth_camera_name_;
        std::string rgb_camera_name_;
        std::shared_ptr<hako::robots::physics::IWorld> motion_world_;
        bool fixed_motion_set_ {false};
        CameraMotion fixed_motion_ {};
        RealSenseConfig config_ {};
        CameraConfig rgb_config_ {};
        DepthCameraConfig depth_render_config_ {};  // render resolution
        DepthCameraConfig depth_config_ {};         // output (after decimation)
        DepthModelParams depth_params_ {};
        EdgeEffectsParams edge_params_ {};
        EdgeEffectsStats last_edge_stats_ {};
        DepthPostProcessor post_ {};
        std::mt19937 depth_rng_;
        std::mt19937 edge_rng_;   // separate stream: enabling edges keeps the depth noise sequence
        std::mt19937 rgb_noise_rng_;
    };
}
