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

            // Doppler: world linear velocity of the hit body.
            const int body_id = model->geom_bodyid[geomid];
            if (body_id >= 0) {
                mjtNum vel6[6] = {0, 0, 0, 0, 0, 0};  // [angular(3), linear(3)]
                mj_objectVelocity(model, data, mjOBJ_BODY, body_id, vel6, /*flg_local=*/0);
                result.target_velocity = hako::robots::types::Vector3(vel6[3], vel6[4], vel6[5]);
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
