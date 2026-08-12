#!/usr/bin/env python3
"""M6 end-to-end controller: detection -> avoidance steering.

Drives the REAL drone (hakosim takeoff/moveToPosition) and consumes the A-2
mujoco-sensor's lidar_points (published to SHM by m6_sensor_bridge.py). When a
forward obstacle is detected within D_safe, the drone steers laterally around it
and then continues to the goal.

lidar_points is read via hakopy.pdu_read directly (NOT hakosim.getLidarData,
whose PduManager declared-reader path returns empty for externally-published
channels -- same issue noted in the radar live-execution work).

Usage: python m6_avoid.py <config_pdudef.json>
Verification (printed as RESULT): (1) a detection event occurred,
(2) min approach to the pillar >= collision radius, (3) goal reached.
"""
import sys, time, math, struct
import hakopy
import hakoniwa_pdu.apps.drone.hakosim as hakosim
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

LIDAR_CH, LIDAR_SIZE = 16, 177424
# Scenario (env-frame, matches m6/env.xml): pillar box centre (2,0), half 0.4.
H        = 0.7     # flight height
GOAL_X   = 3.2     # goal ahead, beyond the pillar
PILLAR   = (2.0, 0.0)
D_SAFE   = 1.1     # detect/avoid threshold (m, forward)
LANE_HALF= 0.5     # front sector half-width in y (m)
Y_AVOID  = 1.3     # lateral offset to steer around (m, +y = left)
GOAL_EPS = 0.30
COLL_R   = 0.6     # min allowed approach to pillar centre (box half 0.4 + margin)
STEP_FWD = 0.5
MAXSTEP  = 60


def lidar_front():
    """Return (d_front, npts) from the A-2 lidar_points (drone/sensor frame:
    x fwd, y left, z up). None if unreadable."""
    raw = hakopy.pdu_read("Drone", LIDAR_CH, LIDAR_SIZE)
    if not raw:
        return None, 0
    try:
        pc = pdu_to_py_PointCloud2(bytes(raw))
    except Exception:
        return None, 0
    data = bytes(pc.data); step = pc.point_step
    if step <= 0 or not data:
        return None, 0
    best, n = None, 0
    for i in range(len(data) // step):
        x, y, z, inten = struct.unpack_from("<ffff", data, i * step)
        if inten <= 0:
            continue
        if x > 0.15 and abs(y) < LANE_HALF and abs(z) < 0.5:
            d = math.hypot(x, y); n += 1
            if best is None or d < best:
                best = d
    return best, n


def main():
    c = hakosim.MultirotorClient(sys.argv[1], "Drone")
    c.confirmConnection(); c.enableApiControl(True); c.armDisarm(True)
    print("[m6_avoid] takeoff", flush=True)
    c.takeoff(H); time.sleep(1.0)

    state = "GO"
    detected = False
    reached = False
    min_front = math.inf
    min_pillar = math.inf

    for k in range(MAXSTEP):
        p = c.simGetVehiclePose().position
        px, py = p.x_val, p.y_val
        min_pillar = min(min_pillar, math.hypot(px - PILLAR[0], py - PILLAR[1]))
        if math.hypot(px - GOAL_X, py - 0.0) < GOAL_EPS:
            reached = True
            break
        d_front, n = lidar_front()
        if d_front is not None:
            min_front = min(min_front, d_front)

        if state == "GO":
            if d_front is not None and d_front < D_SAFE and px < PILLAR[0]:
                detected = True
                state = "AVOID"
            else:
                tgt = (min(px + STEP_FWD, GOAL_X), 0.0, H)
        if state == "AVOID":
            if px > PILLAR[0] + 0.5:          # cleared the pillar in x
                state = "GO"
                tgt = (min(px + STEP_FWD, GOAL_X), 0.0, H)
            else:
                tgt = (px + 0.3, Y_AVOID, H)  # sidestep while edging forward

        df = f"{d_front:.2f}" if d_front is not None else "--"
        print(f"[{k:02d}] pos=({px:.2f},{py:.2f}) state={state} d_front={df} "
              f"pts={n} -> tgt=({tgt[0]:.2f},{tgt[1]:.2f})", flush=True)
        c.moveToPosition(tgt[0], tgt[1], tgt[2], 1.0, timeout_sec=1.0)

    p = c.simGetVehiclePose().position
    print(f"[m6_avoid] end pos=({p.x_val:.2f},{p.y_val:.2f},{p.z_val:.2f})", flush=True)
    c1 = detected
    c2 = min_pillar >= COLL_R
    c3 = reached
    print(f"[m6_avoid] detection_event={c1} min_front={min_front:.2f} "
          f"min_pillar_dist={min_pillar:.2f}(>= {COLL_R}? {c2}) goal_reached={c3}", flush=True)
    ok = c1 and c2 and c3
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(detect={c1}, clearance={c2}, goal={c3})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
