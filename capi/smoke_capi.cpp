// Backend-free smoke test for the C-ABI: a dummy ray caster reports a constant
// hit 5 m ahead, so both lidar3d and radar must return a non-empty point cloud.
// No MuJoCo, no PDU, no Godot -- proves the API + injection wiring in isolation.

#include <cstdio>
#include <cmath>
#include <vector>

#include "hako_sensor_capi.h"

// Constant hit at 5 m along the ray; used for every cast.
static int dummy_raycast(void* /*user*/,
                         const double origin[3], const double dir[3], double max_dist,
                         double* out_dist, double out_point[3], double out_normal[3],
                         double out_target_vel[3], int* out_target_id) {
    const double dist = 5.0;
    if (dist > max_dist) return 0;
    *out_dist = dist;
    for (int i = 0; i < 3; ++i) {
        out_point[i] = origin[i] + dir[i] * dist;
        out_normal[i] = -dir[i];
        out_target_vel[i] = 0.0;
    }
    *out_target_id = 1;
    return 1;
}

static int scan_one(const char* type, const char* cfg) {
    hako_sensor_handle h = hako_sensor_create(type, cfg, dummy_raycast, nullptr);
    if (h == nullptr) { std::printf("[smoke] create(%s) FAILED\n", type); return 1; }

    // identity pose: origin 0, forward +x, left +y, up +z, zero velocity.
    const double state[15] = {0,0,0, 1,0,0, 0,1,0, 0,0,1, 0,0,0};
    std::vector<float> pts(200000 * 4);
    int hgt = 0, wid = 0, cnt = 0;
    const int rc = hako_sensor_scan(h, state, 0.1, pts.data(), 200000, &hgt, &wid, &cnt);
    if (rc != 0) {
        std::printf("[smoke] scan(%s) rc=%d err=%s\n", type, rc, hako_sensor_last_error(h));
        hako_sensor_destroy(h);
        return 1;
    }
    std::printf("[smoke] %-7s height=%d width=%d count=%d  first=(%.2f,%.2f,%.2f w=%.2f)\n",
                type, hgt, wid, cnt,
                cnt > 0 ? pts[0] : 0.f, cnt > 0 ? pts[1] : 0.f,
                cnt > 0 ? pts[2] : 0.f, cnt > 0 ? pts[3] : 0.f);
    hako_sensor_destroy(h);
    return cnt > 0 ? 0 : 1;
}

int main() {
    int rc = 0;
    rc |= scan_one("lidar3d",
        "{\"channels\":15,\"points_per_second\":54150,\"max_distance\":20.0,"
        "\"vertical_fov_lower_deg\":-15.0,\"vertical_fov_upper_deg\":15.0,"
        "\"horizontal_fov_start_deg\":-180.0,\"horizontal_fov_end_deg\":180.0}");
    rc |= scan_one("radar",
        "{\"range\":20.0,\"horizontal_fov_deg\":60.0,\"vertical_fov_deg\":20.0,"
        "\"points_per_second\":1500}");
    std::printf("[smoke] RESULT: %s\n", rc == 0 ? "PASS" : "FAIL");
    return rc;
}
