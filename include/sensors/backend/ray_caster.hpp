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
        // Radar cross-section of the hit surface, m^2. Negative = the backend has
        // no RCS for this target, so the sensor falls back to its reference value.
        // Kept here rather than looked up by the sensor so the sensor model stays
        // backend-agnostic: MuJoCo reads it from the geom, Godot would read it
        // from node metadata, and neither leaks into the model.
        double target_rcs_m2 {-1.0};
    };

    // Rigid-body velocity transfer: the world velocity of a point P that sits
    // r metres from a reference point O, where O moves at v_ref and the body
    // spins at omega (world frame, rad/s).
    //
    //     v_P = v_O + omega x r
    //
    // Both ends of the Doppler equation need this, and getting either one wrong
    // shows up as a plain velocity error in the radar output:
    //   target side -- the ray's hit point vs the body origin mj_objectVelocity
    //                  reports (see MujocoRayCaster::Cast)
    //   sensor side -- the mount point vs the airframe origin the pose PDU
    //                  reports (see runtime::MakeState)
    // Kept here, next to the two structs it relates, so the rule has one
    // definition instead of one per caller.
    inline hako::robots::types::Vector3 VelocityAtPoint(
        const hako::robots::types::Vector3& v_ref,
        const hako::robots::types::Vector3& omega,
        const hako::robots::types::Vector3& r)
    {
        return hako::robots::types::Vector3(
            v_ref.x + (omega.y * r.z - omega.z * r.y),
            v_ref.y + (omega.z * r.x - omega.x * r.z),
            v_ref.z + (omega.x * r.y - omega.y * r.x));
    }

    // Pose + motion state of the sensor at scan time, world frame.
    // Provided by the integration layer (robot/avatar), keeping the sensor
    // model independent from how the pose is obtained.
    struct SensorState
    {
        hako::robots::types::Vector3 origin {};   // world position of sensor origin
        hako::robots::types::Vector3 forward {};  // unit, sensor +x (look direction)
        hako::robots::types::Vector3 left {};     // unit, sensor +y
        hako::robots::types::Vector3 up {};       // unit, sensor +z
        // World linear velocity OF THE SENSOR ORIGIN -- not of the vehicle body.
        // The two differ whenever the sensor is mounted off the body origin and
        // the vehicle rotates: the mount rides a lever arm and moves at
        // v_body + omega x r_mount. Doppler is the radial velocity of the
        // transceiver, which sits at the origin, so this is the term the model
        // needs. The integration layer composes it with VelocityAtPoint().
        hako::robots::types::Vector3 linear_velocity {};
        // World angular velocity of the sensor, rad/s. The sensor is rigid with
        // its mount, so this is the vehicle's angular velocity. Carried so a
        // consumer holding an offset from the sensor origin can run the same
        // transfer again without having to reach back to the vehicle state.
        hako::robots::types::Vector3 angular_velocity {};
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
