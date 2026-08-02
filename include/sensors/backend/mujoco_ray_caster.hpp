#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "physics.hpp"
#include "sensor.hpp"
#include "sensors/backend/ray_caster.hpp"

namespace hako::robots::sensor::backend
{
    // MuJoCo implementation of IRayCaster.
    //
    // Uses mj_ray for occlusion and mj_objectVelocity to read the hit body's
    // world linear velocity, so Doppler relative velocity is obtained directly
    // from MuJoCo state (no position-difference tracking needed -- the key
    // advantage of the C++/MuJoCo backend for radar, per the strategy doc).
    class MujocoRayCaster : public IRayCaster
    {
    public:
        // Engine-integrated form: pose/state comes from a hakoniwa IWorld.
        MujocoRayCaster(
            std::shared_ptr<hako::robots::physics::IWorld> world,
            std::string exclude_body_name);

        // Raw form: directly bound to MuJoCo model/data (no IWorld dependency).
        // Useful for tests and for any integration that already owns mjModel*/mjData*.
        MujocoRayCaster(
            mjModel* model,
            mjData* data,
            std::string exclude_body_name);

        RayHit Cast(
            const hako::robots::types::Vector3& origin,
            const hako::robots::types::Vector3& dir,
            double max_distance) override;

    private:
        mjModel* model() const;
        mjData* data() const;

        // Reuse ISensor's body-tree self-exclusion logic.
        class SelfGeomHelper : public ISensor
        {
        public:
            void Reset() override {}
            double GetUpdatePeriodSec() const override { return 0.0; }
            bool ShouldUpdate(double) override { return false; }
        };

        std::shared_ptr<hako::robots::physics::IWorld> world_;
        mjModel* raw_model_ {nullptr};
        mjData* raw_data_ {nullptr};
        std::string exclude_body_name_;
        SelfGeomHelper self_geom_ {};
    };
}
