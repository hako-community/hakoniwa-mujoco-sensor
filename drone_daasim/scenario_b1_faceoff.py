#!/usr/bin/env python3
"""B-1 mutual detection, done right: make the two drones FACE each other.

Earlier finding: the flight controller yaws both drones to 0 deg (facing +x)
after takeoff regardless of their spawned heading, so they end up facing the
SAME way -- only the rear drone sees the front one. Here we command yaw so each
drone faces the other, then confirm both radars return the other drone in their
forward cone. Finally we let Drone approach the hovering Drone1 and sample
Drone1's radar from a background thread to show an approaching (negative) Doppler.

Run AFTER: bash drone_daasim/two_drone_run.sh   (single session -- commands must be
sent by the same clients that took off; a second external client cannot write).
"""
import math
import os
import struct
import threading
import time

import hakopy
import hakoniwa_pdu.apps.drone.hakosim as hakosim
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "config2", "webavatar-2-radar.json")
RADAR_CH, RADAR_SIZE = 19, 177424


def cone(robot, half_deg=15.0):
    """Forward-cone returns: (count, nearest_range, doppler_of_nearest)."""
    raw = hakopy.pdu_read(robot, RADAR_CH, RADAR_SIZE)
    if not raw:
        return 0, None, 0.0
    pc = pdu_to_py_PointCloud2(bytes(raw))
    data = bytes(pc.data)
    step = pc.point_step or 16
    n = min(int(pc.width), len(data) // step) if pc.width else len(data) // step
    t = math.tan(math.radians(half_deg))
    fwd = []
    for i in range(n):
        x, y, z, v = struct.unpack_from("<ffff", data, i * step)
        if x <= 0.05:
            continue
        if abs(y) <= x * t and abs(z) <= x * t:
            fwd.append(((x * x + y * y + z * z) ** 0.5, v))
    fwd.sort()
    if not fwd:
        return 0, None, 0.0
    return len(fwd), fwd[0][0], fwd[0][1]


def report(tag):
    for robot in ("Drone", "Drone1"):
        c, r, d = cone(robot)
        rs = f"{r:.2f} m" if r is not None else "--"
        print(f"  [{tag}] {robot}: forward-cone {c:3d} pts  nearest {rs}  doppler {d:+.2f} m/s")


def main():
    a = hakosim.MultirotorClient(CFG, "Drone")
    a.confirmConnection(); a.enableApiControl(True); a.armDisarm(True)
    b = hakosim.MultirotorClient(CFG, "Drone1")
    b.confirmConnection(); b.enableApiControl(True); b.armDisarm(True)

    print("takeoff both ...")
    a.takeoff(0.6); b.takeoff(0.6)

    # Face each other and hold 3 m apart (Drone faces +x, Drone1 faces -x).
    print("face-off: Drone yaw=0 (+x), Drone1 yaw=180 (-x), 3 m apart ...")
    a.moveToPosition(-1.5, 0.0, 0.6, 1.0, yaw_deg=0)
    b.moveToPosition(1.5, 0.0, 0.6, 1.0, yaw_deg=180)
    time.sleep(1.2)
    report("faced")

    # Doppler: Drone1 hovers facing Drone; Drone flies toward it while a
    # background thread samples Drone1's radar (shm reads are fine off-thread).
    print("approach: Drone flies -1.5 -> +0.5 (toward hovering Drone1 @ +1.5) ...")
    samples = []
    stop = threading.Event()

    def sampler():
        while not stop.is_set():
            c, r, d = cone("Drone1")
            if r is not None:
                samples.append((r, d))
            time.sleep(0.12)

    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    a.moveToPosition(0.5, 0.0, 0.6, 0.5, yaw_deg=0)   # slow approach
    stop.set(); th.join(timeout=1.0)

    if samples:
        nearest = min(samples, key=lambda s: s[0])
        most_neg = min(samples, key=lambda s: s[1])
        print(f"  Drone1 during approach: {len(samples)} samples")
        print(f"    closest actor return : {nearest[0]:.2f} m  (doppler {nearest[1]:+.2f})")
        print(f"    most-negative doppler: {most_neg[1]:+.2f} m/s  (at {most_neg[0]:.2f} m)")
    else:
        print("  Drone1: no forward-cone samples during approach")
    report("after")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
