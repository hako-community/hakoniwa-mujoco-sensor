#!/usr/bin/env python3
"""B-1 probe: isolate the OTHER drone from walls in each drone's radar.

Both drones face each other along x, ~1.4 m apart, so the other drone sits in
the forward cone (azimuth~0, elevation~0) of each radar. Walls are off-axis or
farther. We read each radar, keep only forward-cone points, and report them.

No takeoff/move here -- assumes two_drone_run.sh is already up and the drones
are hovering at x=+-0.7, z=0.6.
"""
import math
import struct
import sys
import os

import hakopy
import hakoniwa_pdu.apps.drone.hakosim as hakosim
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "config2", "webavatar-2-radar.json")
RADAR_CH, RADAR_SIZE, POINT_STEP = 19, 177424, 16
CONE_DEG = 15.0  # half-angle of the forward cone we treat as "straight ahead"


def read_points(robot):
    raw = hakopy.pdu_read(robot, RADAR_CH, RADAR_SIZE)
    if not raw:
        return None
    pc = pdu_to_py_PointCloud2(bytes(raw))
    data = bytes(pc.data)
    step = pc.point_step or POINT_STEP
    n = min(int(pc.width), len(data) // step) if pc.width else len(data) // step
    pts = []
    for i in range(n):
        x, y, z, v = struct.unpack_from("<ffff", data, i * step)
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        pts.append((x, y, z, v))
    return pts


def classify(robot):
    pts = read_points(robot)
    if pts is None:
        print(f"  {robot}: read failed")
        return
    tan_cone = math.tan(math.radians(CONE_DEG))
    fwd = []
    for (x, y, z, v) in pts:
        if x <= 0.05:
            continue  # behind / sideways
        if abs(y) <= x * tan_cone and abs(z) <= x * tan_cone:
            r = (x * x + y * y + z * z) ** 0.5
            fwd.append((r, v, x, y, z))
    fwd.sort()
    print(f"  {robot}: total {len(pts):4d} pts | forward-cone(<{CONE_DEG:.0f}deg) {len(fwd):3d} pts")
    for (r, v, x, y, z) in fwd[:8]:
        print(f"      r={r:5.2f}m doppler={v:+.2f} xyz=({x:+.2f},{y:+.2f},{z:+.2f})")


def main():
    a = hakosim.MultirotorClient(CFG, "Drone")
    a.confirmConnection()
    print(f"forward-cone radar returns (other drone should appear ~1.15-1.4 m ahead):")
    for _ in range(int(sys.argv[1]) if len(sys.argv) > 1 else 3):
        for robot in ("Drone", "Drone1"):
            classify(robot)
        print("  ---")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
