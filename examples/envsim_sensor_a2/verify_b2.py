#!/usr/bin/env python3
"""
B-2 check: the formal drone sensor manifest drives multi-sensor selection.

  1. validate config/a2/drone-a2-sensors.json against the schema
  2. run the runtime with the drone manifest + a drone pose
  3. confirm every selected sensor publishes its PDU
  4. confirm per-sensor MOUNT is applied (front_lidar offset 0.15 m -> its
     forward beam is ~0.15 m closer to wall than the centre safety_scan)

Run: python3 verify_b2.py
"""

import math
import os
import struct
import subprocess
import sys
import tempfile

from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_LaserScan import pdu_to_py_LaserScan

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "sensor_runtime_demo")
MANIFEST = os.path.normpath(os.path.join(HERE, "../../config/a2/drone-a2-sensors.json"))
ENV_XML = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-simenv-data/examples/sensor_envs/simple_room/generated/env.xml"))
POS = (0.0, 0.0, 1.0)   # drone at room centre
fails = []


def check(cond, msg):
    print(("  [ ok ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def laser_ranges(sc):
    r = sc.ranges
    if isinstance(r, (bytes, bytearray)):
        return list(struct.unpack(f"<{len(r) // 4}f", r))
    return list(r)


def pc2(pc, iv, ih):
    data = bytes(pc.data) if isinstance(pc.data, (bytes, bytearray)) else bytes(bytearray(pc.data))
    x, y, z, w = struct.unpack_from("<4f", data, (iv * pc.width + ih) * pc.point_step)
    return math.sqrt(x * x + y * y + z * z)


def main() -> int:
    # 1. schema validation
    rc = subprocess.run([sys.executable, os.path.join(HERE, "validate_manifest.py"), MANIFEST],
                        capture_output=True, text=True)
    print(rc.stdout.strip() or rc.stderr.strip())
    check(rc.returncode == 0, "drone manifest passes schema validation")

    if not os.path.exists(DEMO):
        print(f"ERROR: build first (bash build.bash). missing {DEMO}")
        return 2

    with tempfile.TemporaryDirectory() as td:
        pos_bin = os.path.join(td, "pos.bin")
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = POS
        t.angular.x = t.angular.y = t.angular.z = 0.0
        with open(pos_bin, "wb") as f:
            f.write(bytes(py_to_pdu_Twist(t)))

        r = subprocess.run([DEMO, ENV_XML, MANIFEST, pos_bin, td], capture_output=True, text=True)
        print(r.stdout.strip())
        if r.returncode != 0:
            print(r.stderr.strip()); print("RESULT: FAIL (runtime error)"); return 1

        # 3. all selected sensors published
        for name in ("lidar_points", "scan", "radar_scan"):
            check(os.path.exists(os.path.join(td, name + ".bin")), f"published '{name}'")

        with open(os.path.join(td, "lidar_points.bin"), "rb") as f:
            lp = pdu_to_py_PointCloud2(f.read())
        with open(os.path.join(td, "scan.bin"), "rb") as f:
            sc = pdu_to_py_LaserScan(f.read())

    # front_lidar: channels=15 (incl. pitch 0 at iv=7), width=361 (yaw 0 at ih=180)
    check(lp.height == 15 and lp.width == 361, f"front_lidar 15x361 (got {lp.height}x{lp.width})")
    d_front = pc2(lp, 7, 180)        # forward horizontal beam, mounted +0.15 m forward
    d_scan = laser_ranges(sc)[180]   # centre 2D scan forward

    # 4. mount applied: front lidar is ~0.15 m closer to wall_north than centre scan
    check(abs(d_scan - 3.9) < 0.05, f"centre scan forward = wall_north 3.9m (got {d_scan:.3f})")
    check(abs(d_front - 3.75) < 0.05, f"front_lidar (+0.15m) forward = 3.75m (got {d_front:.3f})")
    check(abs((d_scan - d_front) - 0.15) < 0.02,
          f"mount offset applied: scan-front = {d_scan - d_front:.3f} ~ 0.15m")

    print()
    print("RESULT: PASS" if not fails else f"RESULT: FAIL ({len(fails)})")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
