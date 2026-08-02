#!/usr/bin/env python3
"""
B-1 check: manifest-driven A-2 sensor runtime.

  Python: build a `pos` Twist PDU
  C++:    sensor_runtime_demo reads drone-sensors.json, creates the SELECTED
          sensors, senses env.xml, writes one PDU per sensor
  Python: decode each PDU (lidar_points/scan/radar_scan) and sanity-check

Proves: sensors are *selected via manifest* and each emits its PDU.

Run: python3 verify_runtime.py
"""

import math
import os
import struct
import subprocess
import tempfile

from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_LaserScan import pdu_to_py_LaserScan

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "sensor_runtime_demo")
MANIFEST = os.path.join(HERE, "drone-sensors.json")
ENV_XML = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-envsim-sensor/examples/simple_room/generated/env.xml"))
POS = (0.0, 0.0, 1.0)
fails = []


def check(cond, msg):
    print(("  [ ok ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def pc2_points(pc):
    data = bytes(pc.data) if isinstance(pc.data, (bytes, bytearray)) else bytes(bytearray(pc.data))
    n = len(data) // pc.point_step
    return [struct.unpack_from("<4f", data, i * pc.point_step) for i in range(n)]


def main() -> int:
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

        # 3 sensors selected from the manifest -> 3 PDU files
        for name in ("lidar_points", "scan", "radar_scan"):
            check(os.path.exists(os.path.join(td, name + ".bin")), f"manifest sensor published '{name}'")

        # lidar_points (3D PointCloud2)
        with open(os.path.join(td, "lidar_points.bin"), "rb") as f:
            pc = pdu_to_py_PointCloud2(f.read())
        pts = pc2_points(pc)
        check(pc.height == 17 and pc.width == 361, f"lidar_points organized 17x361 (got {pc.height}x{pc.width})")
        fwd = pts[8 * pc.width + 180]
        dfwd = math.sqrt(fwd[0] ** 2 + fwd[1] ** 2 + fwd[2] ** 2)
        check(abs(dfwd - 3.9) < 0.05, f"lidar_points forward -> wall_north 3.9m (got {dfwd:.3f})")

        # scan (2D LaserScan)
        with open(os.path.join(td, "scan.bin"), "rb") as f:
            sc = pdu_to_py_LaserScan(f.read())
        ranges = sc.ranges
        if isinstance(ranges, (bytes, bytearray)):
            ranges = list(struct.unpack(f"<{len(ranges) // 4}f", ranges))
        check(len(ranges) == 361, f"scan has 361 ranges (got {len(ranges)})")
        check(abs(ranges[180] - 3.9) < 0.05, f"scan forward -> wall_north 3.9m (got {ranges[180]:.3f})")

        # radar_scan (PointCloud2, x,y,z,velocity)
        with open(os.path.join(td, "radar_scan.bin"), "rb") as f:
            rad = pdu_to_py_PointCloud2(f.read())
        rpts = pc2_points(rad)
        depths = [math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) for p in rpts]
        check(len(rpts) > 0, f"radar_scan has detections (got {len(rpts)})")
        check(all(0.0 < d <= 20.01 for d in depths), "radar_scan depths within range")

    print()
    print("RESULT: PASS" if not fails else f"RESULT: FAIL ({len(fails)})")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
