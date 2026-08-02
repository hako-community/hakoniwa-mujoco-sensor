#pragma once

// Strategy-C backend abstraction.
//
// Range/ray based sensors (LiDAR, Ultrasonic, Radar) must not depend directly
// on a concrete physics/render engine. They depend only on this IRayCaster
// interface. A MuJoCo backend implements it with mj_ray + mj_objectVelocity;
// a Godot backend would implement the equivalent with IntersectRay on the C#
// side. The same sensor model therefore runs on either backend.

#include "primitive_types.hpp"

namespace hako::robots::sensor::backend
{
    // Result of a single ray cast, expressed in the world frame.
    struct RayHit
    {
        bool hit {false};
        double distance {0.0};                       // metres from origin to hit point
        hako::robots::types::Vector3 point {};       // world hit position
        hako::robots::types::Vector3 normal {};      // world surface normal
        hako::robots::types::Vector3 target_velocity {}; // world linear velocity of hit body (Doppler)
        int target_id {-1};                          // backend-specific body/geom id (-1 = none)
    };

    // Pose + motion state of the sensor at scan time, world frame.
    // Provided by the integration layer (robot/avatar), keeping the sensor
    // model independent from how the pose is obtained.
    struct SensorState
    {
        hako::robots::types::Vector3 origin {};   // world position of sensor origin
        hako::robots::types::Vector3 forward {};  // unit, sensor +x (look direction)
        hako::robots::types::Vector3 left {};     // unit, sensor +y
        hako::robots::types::Vector3 up {};       // unit, sensor +z
        hako::robots::types::Vector3 linear_velocity {}; // world linear velocity of sensor
    };

    class IRayCaster
    {
    public:
        virtual ~IRayCaster() = default;

        // Cast a ray from origin along dir (unit, world frame) up to max_distance metres.
        virtual RayHit Cast(
            const hako::robots::types::Vector3& origin,
            const hako::robots::types::Vector3& dir,
            double max_distance) = 0;
    };
}
