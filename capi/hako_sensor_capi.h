#ifndef HAKO_SENSOR_CAPI_H
#define HAKO_SENSOR_CAPI_H

/*
 * hako_sensor_capi -- stable C-ABI over the backend-agnostic sensor models.
 *
 * This is the Phase 2 (4th-slide) deliverable: a general C interface that lets
 * ANY consumer (Godot, a test harness, another engine) drive the mujoco-sensor
 * LiDAR/Radar models in-process, WITHOUT core-pro / PDU / SHM.
 *
 * The library never knows who the consumer is: world ray casts are *injected*
 * through a C function pointer (hako_raycast_fn). A Godot sample implements it
 * with PhysicsDirectSpaceState3D.IntersectRay; a unit test implements it with a
 * constant hit. The sensor model (include/src) is untouched -- this header only
 * wraps Lidar3DSensor / RadarSensor behind an opaque handle + POD arguments so
 * the ABI stays stable across C++ ABI changes.
 *
 * Frames: everything is world-frame for the ray-cast callback; the returned
 * point cloud is sensor-local cartesian (ROS REP-103: x forward, y left, z up),
 * 4 floats/point [x, y, z, w] where w = intensity (LiDAR) or radial velocity /
 * Doppler in m/s (Radar). This matches the PointCloud2 layout the PDU path uses,
 * so a consumer can share one visualizer for both paths.
 */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hako_sensor_ctx* hako_sensor_handle;

/*
 * World-frame ray cast callback. Return 1 on hit, 0 on miss.
 * On hit, fill out_dist (metres), out_point[3] (world hit position),
 * out_normal[3] (world surface normal), out_target_vel[3] (world linear
 * velocity of the hit body, for Doppler; zero if unknown) and *out_target_id
 * (backend body/geom id, -1 if none). out_* may be written even on miss.
 */
typedef int (*hako_raycast_fn)(
    void* user,
    const double origin[3], const double dir[3], double max_dist,
    double* out_dist, double out_point[3], double out_normal[3],
    double out_target_vel[3], int* out_target_id);

/*
 * Create a sensor.
 *   type        : "lidar3d" | "radar"
 *   config_json : JSON object of the sensor params (same field names as the A-2
 *                 manifest "params": e.g. channels/rotations_per_second/... for
 *                 lidar3d; range/horizontal_fov_deg/... for radar). "" or "{}"
 *                 uses model defaults.
 *   raycast     : world ray-cast callback (required; NULL -> create fails)
 *   raycast_user: opaque pointer passed back to the callback.
 * Returns NULL on failure.
 */
hako_sensor_handle hako_sensor_create(
    const char* type,
    const char* config_json,
    hako_raycast_fn raycast, void* raycast_user);

/*
 * Run one scan.
 *   state[15]       : origin[3], forward[3], left[3], up[3], linear_velocity[3]
 *                     (all world frame; forward/left/up are unit basis vectors).
 *   dt_sec          : elapsed time since previous scan (drives the scheduler).
 *   out_points_xyzi : caller buffer, 4 floats per point (x,y,z,w).
 *   max_points      : capacity of out_points_xyzi in POINTS (not floats).
 *   out_height/out_width/out_count : grid height, width and written point count.
 * Returns 0 on success, non-zero on error (see hako_sensor_last_error).
 * A due-but-empty scan is success with *out_count == 0.
 */
int hako_sensor_scan(hako_sensor_handle h,
    const double state[15], double dt_sec,
    float* out_points_xyzi, int max_points,
    int* out_height, int* out_width, int* out_count);

void  hako_sensor_reset(hako_sensor_handle h);
void  hako_sensor_destroy(hako_sensor_handle h);

/* Last error string for the handle (never NULL; "" when no error). */
const char* hako_sensor_last_error(hako_sensor_handle h);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* HAKO_SENSOR_CAPI_H */
