#include "sensors/backend/mujoco_ray_caster.hpp"

#include <algorithm>
#include <utility>

#include <mujoco/mujoco.h>

namespace hako::robots::sensor::backend
{

MujocoRayCaster::MujocoRayCaster(
    std::shared_ptr<hako::robots::physics::IWorld> world,
    std::string exclude_body_name)
    : world_(std::move(world))
    , exclude_body_name_(std::move(exclude_body_name))
{
}

MujocoRayCaster::MujocoRayCaster(
    mjModel* model,
    mjData* data,
    std::string exclude_body_name)
    : raw_model_(model)
    , raw_data_(data)
    , exclude_body_name_(std::move(exclude_body_name))
{
}

mjModel* MujocoRayCaster::model() const
{
    return (world_ != nullptr) ? world_->getModel() : raw_model_;
}

mjData* MujocoRayCaster::data() const
{
    return (world_ != nullptr) ? world_->getData() : raw_data_;
}

RayHit MujocoRayCaster::Cast(
    const hako::robots::types::Vector3& origin,
    const hako::robots::types::Vector3& dir,
    double max_distance)
{
    RayHit result {};

    mjModel* model = this->model();
    mjData* data = this->data();
    if (model == nullptr || data == nullptr) {
        return result;
    }

    const int exclude_body_id =
        exclude_body_name_.empty() ? -1 : mj_name2id(model, mjOBJ_BODY, exclude_body_name_.c_str());

    mjtNum o[3] = {origin.x, origin.y, origin.z};
    mjtNum d[3] = {dir.x, dir.y, dir.z};
    const mjtNum epsilon = 1.0e-4;
    mjtNum traveled = 0.0;

    // Advance past self-geometry hits (mirrors LiDAR2DSensor::CastRay).
    for (int attempt = 0; attempt < 16; ++attempt) {
        int geomid = -1;
        mjtNum normal[3] = {0.0, 0.0, 0.0};
        const mjtNum hit_dist = mj_ray(model, data, o, d, nullptr, 1, -1, &geomid, normal);
        if (hit_dist < 0.0 || geomid < 0) {
            return result;  // no hit
        }

        const double true_dist = static_cast<double>(traveled + hit_dist);
        if (!self_geom_.IsSelfGeom(model, exclude_body_id, geomid)) {
            if (true_dist > max_distance) {
                return result;
            }
            result.hit = true;
            result.distance = true_dist;
            result.target_id = geomid;
            result.point = hako::robots::types::Vector3(
                o[0] + d[0] * hit_dist, o[1] + d[1] * hit_dist, o[2] + d[2] * hit_dist);
            result.normal = hako::robots::types::Vector3(normal[0], normal[1], normal[2]);

            // Per-target RCS, carried in the geom's first user slot. Requires the
            // MJCF to declare <size nuser_geom="1"/> and the geom to set
            // user="<m^2>"; anything else leaves it negative and the sensor uses
            // its reference value. MuJoCo zero-fills user data, so 0 also means
            // "not set" -- an RCS of exactly zero would be an invisible target,
            // which is not something a scene author expresses this way.
            if (model->nuser_geom > 0) {
                const mjtNum u = model->geom_user[geomid * model->nuser_geom];
                if (u > 0.0) {
                    result.target_rcs_m2 = static_cast<double>(u);
                }
            }

            // Doppler: world velocity OF THE HIT POINT.
            //
            // mj_objectVelocity reports the velocity of a reference point, not of
            // an arbitrary point on the body:
            //   mjOBJ_BODY  -> the inertial frame origin (xipos, the COM)
            //   mjOBJ_XBODY -> the body frame origin     (xpos)
            // For a rotating target those differ from the point the ray actually
            // struck, and Doppler is by definition the radial velocity of the
            // SCATTERING point. Measured on a body spinning at 2 rad/s with the
            // geom 3 m off the rotation axis, taking the reference point verbatim
            // is off by 6 m/s -- and which of the two is closer depends purely on
            // where the COM happens to sit, so neither is "the right one".
            //
            // Rigid-body transfer fixes it exactly:  v_P = v_O + omega x (P - O).
            const int body_id = model->geom_bodyid[geomid];
            if (body_id >= 0) {
                mjtNum vel6[6] = {0, 0, 0, 0, 0, 0};  // [angular(3), linear(3)], world frame
                mj_objectVelocity(model, data, mjOBJ_XBODY, body_id, vel6, /*flg_local=*/0);
                const hako::robots::types::Vector3 omega(vel6[0], vel6[1], vel6[2]);
                const hako::robots::types::Vector3 v_xpos(vel6[3], vel6[4], vel6[5]);
                const hako::robots::types::Vector3 r(
                    result.point.x - data->xpos[3 * body_id + 0],
                    result.point.y - data->xpos[3 * body_id + 1],
                    result.point.z - data->xpos[3 * body_id + 2]);
                result.target_velocity = VelocityAtPoint(v_xpos, omega, r);
            }
            return result;
        }

        // Self hit: step forward and re-cast.
        const mjtNum step = hit_dist + epsilon;
        traveled += step;
        if (traveled >= static_cast<mjtNum>(max_distance)) {
            return result;
        }
        o[0] += d[0] * step;
        o[1] += d[1] * step;
        o[2] += d[2] * step;
    }

    return result;
}
}  // namespace hako::robots::sensor::backend
