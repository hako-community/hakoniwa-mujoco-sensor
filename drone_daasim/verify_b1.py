#!/usr/bin/env python3
"""B-1 verification: do the two drones detect EACH OTHER with the radar?

Both drones take off and fly toward one another. For each drone we read its own
radar_points channel directly with hakopy (the PduManager declared-reader path
does not surface an external publisher's writes) and report the nearest return
plus its Doppler velocity.

Success criteria:
  * both drones publish radar returns
  * the nearest range shrinks as they close in
  * the Doppler of the nearest return is negative (approaching)

Usage: $PYENV_PY drone_daasim/verify_b1.py [config_pdudef.json]
"""
import os
import struct
import sys
import time

import hakopy
import hakoniwa_pdu.apps.drone.hakosim as hakosim
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "config2", "webavatar-2-radar.json")
RADAR_CH = 19
RADAR_SIZE = 177424
POINT_STEP = 16


def radar_summary(robot):
    """(count, nearest_range_m, doppler_of_nearest) from robot's radar_points."""
    raw = hakopy.pdu_read(robot, RADAR_CH, RADAR_SIZE)
    if not raw:
        return None
    try:
        pc = pdu_to_py_PointCloud2(bytes(raw))
    except Exception as e:
        print("   decode error:", e)
        return None
    data = bytes(pc.data)
    step = pc.point_step or POINT_STEP
    n = min(int(pc.width), len(data) // step) if pc.width else len(data) // step
    best_r, best_v, count = None, 0.0, 0
    for i in range(n):
        x, y, z, v = struct.unpack_from("<ffff", data, i * step)
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        r = (x * x + y * y + z * z) ** 0.5
        count += 1
        if best_r is None or r < best_r:
            best_r, best_v = r, v
    return count, best_r, best_v


def main():
    a = hakosim.MultirotorClient(CFG, "Drone")
    a.confirmConnection(); a.enableApiControl(True); a.armDisarm(True)
    b = hakosim.MultirotorClient(CFG, "Drone1")
    b.confirmConnection(); b.enableApiControl(True); b.armDisarm(True)

    print("takeoff both ...")
    a.takeoff(0.6)
    b.takeoff(0.6)

    print("closing in (Drone: -2 -> -0.6, Drone1: +2 -> +0.6) ...")
    for label, x_a, x_b in (("apart", -2.5, 2.5), ("close", -0.7, 0.7)):
        a.moveToPosition(x_a, 0.0, 0.6, 1.0)
        b.moveToPosition(x_b, 0.0, 0.6, 1.0)
        time.sleep(0.5)
        for robot in ("Drone", "Drone1"):
            s = radar_summary(robot)
            if s is None:
                print(f"  [{label}] {robot}: read failed")
                continue
            count, rng, vel = s
            rng_s = f"{rng:.2f} m" if rng is not None else "--"
            print(f"  [{label}] {robot}: {count:4d} pts  nearest {rng_s}  doppler {vel:+.2f} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
