#pragma once

// Manifest-driven, backend-agnostic A-2 sensor runtime (B-1).
//
// Loads a simenv-data env.xml into a kinematic MuJoCo world, reads a sensor
// manifest, creates the SELECTED sensors via a type->creator factory, and on
// each Step() drives every sensor from the drone pose (BasePose) and emits its
// PDU binary through a transport-agnostic sink.
//
// Header-only on purpose: it pulls the hakoniwa PDU registry converters
// (cpp2pdu) which the consumer provides on its include path -- the static
// library stays registry-agnostic, exactly like the pdu/adapter headers.

#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include "sensors/backend/mujoco_ray_caster.hpp"
#include "sensors/backend/ray_caster.hpp"
#include "sensors/lidar/lidar3d_sensor.hpp"
#include "sensors/lidar/lidar_scan_sensor.hpp"
#include "sensors/radar/radar_sensor.hpp"
#include "sensors/radar/radar_config_json.hpp"   // manifest params -> RadarConfig (1 か所)
#include "sensors/radar/radar_math.hpp"   // RadarEquationRange (link budget -> ref range)

// converters (frame -> HakoCpp) and registry serializers (cpp2pdu)
#include "hakoniwa/pdu/converter/sensor_msgs/lidar_point_cloud.hpp"
#include "hakoniwa/pdu/converter/sensor_msgs/radar_point_cloud.hpp"
#include "hakoniwa/pdu/converter/sensor_msgs/laser_scan.hpp"
#include "sensor_msgs/pdu_cpptype_conv_PointCloud2.hpp"
#include "sensor_msgs/pdu_cpptype_conv_LaserScan.hpp"

namespace hako::robots::runtime
{
    namespace types = hako::robots::types;
    namespace backend = hako::robots::sensor::backend;

    // Drone pose driving all sensors (world/MuJoCo frame; yaw about up/Z).
    struct BasePose
    {
        types::Vector3 origin {};
        double yaw_rad {0.0};
        // A-1: the sensor's own world velocity. Without it every Doppler reading is
        // the target's velocity alone, so a moving drone sees a static wall as 0 m/s.
        // This is the velocity of the BODY origin; MakeState adds the mount's
        // lever-arm term before handing it to a sensor.
        types::Vector3 linear_velocity {};
        // The airframe's world angular velocity, rad/s. For a drone whose pose
        // arrives as (position, yaw) this is (0, 0, yaw_rate). Zero is a valid
        // value and reproduces the pre-#12 behaviour exactly.
        types::Vector3 angular_velocity {};
    };

    // Per-sensor mounting relative to the drone body.
    struct Mount
    {
        double x {0.0};
        double y {0.0};
        double z {0.0};
        double yaw_deg {0.0};
    };

    inline backend::SensorState MakeState(const BasePose& base, const Mount& m)
    {
        const double by = base.yaw_rad;
        const double cb = std::cos(by);
        const double sb = std::sin(by);
        // mount offset is body-frame; rotate into world by base yaw
        backend::SensorState st {};
        const types::Vector3 r_mount(
            cb * m.x - sb * m.y,
            sb * m.x + cb * m.y,
            m.z);
        st.origin = base.origin + r_mount;
        const double yaw = by + m.yaw_deg * M_PI / 180.0;
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        st.forward = types::Vector3(c, s, 0.0);
        st.left = types::Vector3(-s, c, 0.0);
        st.up = types::Vector3(0.0, 0.0, 1.0);
        // #12: the mount rides a lever arm. The pose PDU reports the velocity of
        // the airframe origin, but the transceiver sits at r_mount from it, so a
        // rotating drone moves the sensor at v_body + omega x r_mount. Dropping
        // that term biases Doppler by the RADIAL part of omega x r_mount, which
        // depends on where in the FOV the ray points: the stock front radar
        // (x=0.15, 60 deg azimuth window) is worst at the sector edge, measuring
        // 0.075 m/s of phantom velocity per rad/s of yaw and 0 dead ahead, since
        // the lever-arm velocity is perpendicular to the boresight. A mount
        // looking to the side takes the full |omega||r_mount|. This is the same
        // rigid-body rule #11 applied at the target end -- other end, same fix.
        st.angular_velocity = base.angular_velocity;
        st.linear_velocity = backend::VelocityAtPoint(
            base.linear_velocity, base.angular_velocity, r_mount);
        return st;
    }

    // A configured, selectable sensor that can scan from a pose and serialize a PDU.
    class ISensorComponent
    {
    public:
        virtual ~ISensorComponent() = default;
        virtual const std::string& id() const = 0;
        virtual const std::string& type() const = 0;
        virtual const std::string& pdu_name() const = 0;
        virtual const std::string& pdu_robot() const = 0;  // PDU channel robot/owner
        virtual bool ShouldUpdate(double dt) = 0;
        // scan + serialize into buf; returns pdu byte size, or -1 on error.
        virtual int Publish(const BasePose& base, char* buf, int len) = 0;
    };

    namespace detail
    {
        struct ComponentBase
        {
            std::string id_;
            std::string type_;
            std::string pdu_name_;
            std::string pdu_robot_;
            Mount mount_ {};
        };

        class Lidar3DComponent : public ISensorComponent, ComponentBase
        {
        public:
            Lidar3DComponent(std::shared_ptr<backend::IRayCaster> caster,
                             const sensor::lidar::Lidar3DConfig& cfg,
                             std::string id, std::string pdu_name, std::string pdu_robot, Mount mount)
                : sensor_(std::move(caster))
            {
                id_ = std::move(id); type_ = "lidar_3d"; pdu_name_ = std::move(pdu_name);
                pdu_robot_ = std::move(pdu_robot); mount_ = mount;
                sensor_.SetConfig(cfg);
            }
            const std::string& id() const override { return id_; }
            const std::string& type() const override { return type_; }
            const std::string& pdu_name() const override { return pdu_name_; }
            const std::string& pdu_robot() const override { return pdu_robot_; }
            bool ShouldUpdate(double dt) override { return sensor_.ShouldUpdate(dt); }
            int Publish(const BasePose& base, char* buf, int len) override
            {
                sensor::lidar::Lidar3DFrame frame {};
                sensor_.Scan(MakeState(base, mount_), frame);
                auto cpp = pdu::converter::sensor_msgs::ToHakoPointCloud2(frame);
                hako::pdu::msgs::sensor_msgs::PointCloud2 conv;
                return conv.cpp2pdu(cpp, buf, len);
            }
        private:
            sensor::lidar::Lidar3DSensor sensor_;
        };

        class Lidar2DComponent : public ISensorComponent, ComponentBase
        {
        public:
            Lidar2DComponent(std::shared_ptr<backend::IRayCaster> caster,
                             const sensor::lidar::LidarScanConfig& cfg,
                             std::string id, std::string pdu_name, std::string pdu_robot, Mount mount)
                : sensor_(std::move(caster))
            {
                id_ = std::move(id); type_ = "lidar_2d"; pdu_name_ = std::move(pdu_name);
                pdu_robot_ = std::move(pdu_robot); mount_ = mount;
                sensor_.SetConfig(cfg);
            }
            const std::string& id() const override { return id_; }
            const std::string& type() const override { return type_; }
            const std::string& pdu_name() const override { return pdu_name_; }
            const std::string& pdu_robot() const override { return pdu_robot_; }
            bool ShouldUpdate(double dt) override { return sensor_.ShouldUpdate(dt); }
            int Publish(const BasePose& base, char* buf, int len) override
            {
                sensor::lidar::LaserScanFrame frame {};
                sensor_.Scan(MakeState(base, mount_), frame);
                auto cpp = pdu::converter::sensor_msgs::ToHakoPdu(frame);
                hako::pdu::msgs::sensor_msgs::LaserScan conv;
                return conv.cpp2pdu(cpp, buf, len);
            }
        private:
            sensor::lidar::LidarScanSensor sensor_;
        };

        class RadarComponent : public ISensorComponent, ComponentBase
        {
        public:
            RadarComponent(std::shared_ptr<backend::IRayCaster> caster,
                           const sensor::radar::RadarConfig& cfg,
                           std::string id, std::string pdu_name, std::string pdu_robot, Mount mount)
                : sensor_(std::move(caster))
            {
                id_ = std::move(id); type_ = "radar"; pdu_name_ = std::move(pdu_name);
                pdu_robot_ = std::move(pdu_robot); mount_ = mount;
                sensor_.SetConfig(cfg);
            }
            const std::string& id() const override { return id_; }
            const std::string& type() const override { return type_; }
            const std::string& pdu_name() const override { return pdu_name_; }
            const std::string& pdu_robot() const override { return pdu_robot_; }
            bool ShouldUpdate(double dt) override { return sensor_.ShouldUpdate(dt); }
            int Publish(const BasePose& base, char* buf, int len) override
            {
                sensor::radar::RadarScanFrame frame {};
                sensor_.Scan(MakeState(base, mount_), frame);
                auto cpp = pdu::converter::sensor_msgs::ToHakoPointCloud2(frame);
                hako::pdu::msgs::sensor_msgs::PointCloud2 conv;
                return conv.cpp2pdu(cpp, buf, len);
            }
        private:
            sensor::radar::RadarSensor sensor_;
        };

        inline Mount ParseMount(const nlohmann::json& j)
        {
            Mount m {};
            if (j.contains("mount")) {
                const auto& mj = j.at("mount");
                m.x = mj.value("x", 0.0);
                m.y = mj.value("y", 0.0);
                m.z = mj.value("z", 0.0);
                m.yaw_deg = mj.value("yaw_deg", 0.0);
            }
            return m;
        }
    }  // namespace detail

    // type -> component creator. New sensor types are added with one entry.
    class SensorFactory
    {
    public:
        static std::unique_ptr<ISensorComponent> Create(
            const std::string& type,
            const nlohmann::json& comp,
            std::shared_ptr<backend::IRayCaster> caster,
            const std::string& default_robot = "")
        {
            const std::string id = comp.value("id", type);
            const std::string pdu_name = comp.value("pdu_name", id);
            const std::string pdu_robot = comp.value("pdu_robot", default_robot);
            const Mount mount = detail::ParseMount(comp);
            const nlohmann::json p = comp.value("params", nlohmann::json::object());

            if (type == "lidar_3d") {
                sensor::lidar::Lidar3DConfig c {};
                c.frame_id = p.value("frame_id", c.frame_id);
                c.channels = p.value("channels", c.channels);
                c.rotations_per_second = p.value("rotations_per_second", c.rotations_per_second);
                c.points_per_second = p.value("points_per_second", c.points_per_second);
                c.max_distance = p.value("max_distance", c.max_distance);
                c.min_distance = p.value("min_distance", c.min_distance);
                c.vertical_fov_upper_deg = p.value("vertical_fov_upper_deg", c.vertical_fov_upper_deg);
                c.vertical_fov_lower_deg = p.value("vertical_fov_lower_deg", c.vertical_fov_lower_deg);
                c.horizontal_fov_start_deg = p.value("horizontal_fov_start_deg", c.horizontal_fov_start_deg);
                c.horizontal_fov_end_deg = p.value("horizontal_fov_end_deg", c.horizontal_fov_end_deg);
                return std::make_unique<detail::Lidar3DComponent>(std::move(caster), c, id, pdu_name, pdu_robot, mount);
            }
            if (type == "lidar_2d") {
                sensor::lidar::LidarScanConfig c {};
                c.frame_id = p.value("frame_id", c.frame_id);
                c.angle_min_deg = p.value("angle_min_deg", c.angle_min_deg);
                c.angle_max_deg = p.value("angle_max_deg", c.angle_max_deg);
                c.angle_increment_deg = p.value("angle_increment_deg", c.angle_increment_deg);
                c.range_min = p.value("range_min", c.range_min);
                c.range_max = p.value("range_max", c.range_max);
                c.scan_frequency_hz = p.value("scan_frequency_hz", c.scan_frequency_hz);
                return std::make_unique<detail::Lidar2DComponent>(std::move(caster), c, id, pdu_name, pdu_robot, mount);
            }
            if (type == "radar") {
                // ★ 写しは `sensors/radar/radar_config_json.hpp` の 1 か所に集約した
                //   （2026-08-28）。ここで並べ直さないこと —— 消費側が 2 つある以上、
                //   並べ直した瞬間に片方だけが新しいキーを読む状態に戻る。
                const sensor::radar::RadarConfig c = sensor::radar::RadarConfigFromJson(p);
                return std::make_unique<detail::RadarComponent>(std::move(caster), c, id, pdu_name, pdu_robot, mount);
            }
            return nullptr;  // unknown type -> skipped by runtime
        }

        static bool Known(const std::string& type)
        {
            return type == "lidar_3d" || type == "lidar_2d" || type == "radar";
        }
    };

    // Sink receives each published PDU (transport-agnostic: file / SHM / endpoint).
    using PublishSink = std::function<void(const std::string& pdu_name, const char* data, int len)>;

    class SensorRuntime
    {
    public:
        explicit SensorRuntime(const std::string& env_xml)
        {
            char err[1000] = {0};
            model_ = mj_loadXML(env_xml.c_str(), nullptr, err, sizeof(err));
            if (model_ == nullptr) {
                last_error_ = std::string("mj_loadXML: ") + err;
                return;
            }
            data_ = mj_makeData(model_);
            mj_forward(model_, data_);
            caster_ = std::make_shared<backend::MujocoRayCaster>(model_, data_, std::string{});
        }

        ~SensorRuntime()
        {
            if (data_) mj_deleteData(data_);
            if (model_) mj_deleteModel(model_);
        }

        bool ok() const { return model_ != nullptr && data_ != nullptr; }
        const std::string& last_error() const { return last_error_; }
        size_t component_count() const { return components_.size(); }

        // Parse a manifest JSON (object with "components":[...]) and create the
        // selected sensors. Unknown/non-sensor entries are skipped.
        bool LoadManifest(const std::string& manifest_path)
        {
            std::ifstream f(manifest_path);
            if (!f) { last_error_ = "cannot open manifest: " + manifest_path; return false; }
            nlohmann::json j;
            try { f >> j; } catch (const std::exception& e) { last_error_ = e.what(); return false; }
            if (!j.contains("components")) { last_error_ = "manifest has no 'components'"; return false; }

            for (const auto& comp : j.at("components")) {
                const std::string kind = comp.value("kind", "sensor");
                const std::string type = comp.value("type", "");
                if (kind != "sensor" || !SensorFactory::Known(type)) {
                    continue;  // not a sensor we provide -> skip
                }
                auto c = SensorFactory::Create(type, comp, caster_);
                if (c) components_.push_back(std::move(c));
            }
            return true;
        }

        // --- A-1: dynamic actors ------------------------------------------
        // A body declared in env.xml with a FREE joint can be driven from outside:
        // we write its qpos/qvel every step and re-run kinematics. A mocap body will
        // NOT do -- mj_objectVelocity returns zero for it, so the radar's Doppler
        // would stay 0, which is the whole reason the actor exists.
        bool HasActor(const std::string& body_name) const
        {
            return ActorAdr(body_name).first >= 0;
        }

        bool SetActor(const std::string& body_name,
                      const types::Vector3& pos,
                      double yaw_rad,
                      const types::Vector3& linear_velocity)
        {
            const auto adr = ActorAdr(body_name);
            if (adr.first < 0) {
                last_error_ = "no free-joint body named '" + body_name + "' in env.xml";
                return false;
            }
            const int q = adr.first;   // qpos: x y z qw qx qy qz
            const int d = adr.second;  // qvel: vx vy vz wx wy wz
            data_->qpos[q + 0] = pos.x;
            data_->qpos[q + 1] = pos.y;
            data_->qpos[q + 2] = pos.z;
            data_->qpos[q + 3] = std::cos(yaw_rad * 0.5);
            data_->qpos[q + 4] = 0.0;
            data_->qpos[q + 5] = 0.0;
            data_->qpos[q + 6] = std::sin(yaw_rad * 0.5);
            data_->qvel[d + 0] = linear_velocity.x;
            data_->qvel[d + 1] = linear_velocity.y;
            data_->qvel[d + 2] = linear_velocity.z;
            data_->qvel[d + 3] = 0.0;
            data_->qvel[d + 4] = 0.0;
            data_->qvel[d + 5] = 0.0;
            actors_dirty_ = true;
            return true;
        }

        // Drive all selected sensors from the drone pose and emit due PDUs.
        void Step(const BasePose& base, double dt, const PublishSink& sink)
        {
            if (actors_dirty_) {          // actor poses changed -> refresh kinematics
                mj_forward(model_, data_);
                actors_dirty_ = false;
            }
            for (auto& c : components_) {
                if (!c->ShouldUpdate(dt)) continue;
                const int n = c->Publish(base, buffer_.data(), static_cast<int>(buffer_.size()));
                if (n > 0) sink(c->pdu_name(), buffer_.data(), n);
            }
        }

        const std::vector<std::unique_ptr<ISensorComponent>>& components() const { return components_; }

    private:
        // (qposadr, dofadr) of the body's free joint, or (-1,-1)
        std::pair<int, int> ActorAdr(const std::string& body_name) const
        {
            if (model_ == nullptr) return {-1, -1};
            const int bid = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
            if (bid < 0) return {-1, -1};
            const int jadr = model_->body_jntadr[bid];
            const int jnum = model_->body_jntnum[bid];
            for (int j = jadr; j < jadr + jnum; ++j) {
                if (model_->jnt_type[j] == mjJNT_FREE) {
                    return {model_->jnt_qposadr[j], model_->jnt_dofadr[j]};
                }
            }
            return {-1, -1};
        }

        mjModel* model_ {nullptr};
        mjData* data_ {nullptr};
        std::shared_ptr<backend::IRayCaster> caster_;
        std::vector<std::unique_ptr<ISensorComponent>> components_;
        std::vector<char> buffer_ = std::vector<char>(1 << 20, 0);  // 1 MB scratch
        std::string last_error_;
        bool actors_dirty_ {false};
    };
}
