#!/usr/bin/env python3
"""
M3 cross-language PDU contract check.

  Python: build a `pos` Twist PDU (drone position)        [py_to_pdu_Twist]
  C++:    lidar_a2_pdu reads pos -> senses env.xml -> writes LaserScan PDU
  Python: decode the LaserScan PDU and verify wall ranges [pdu_to_py_LaserScan]

This proves the A-2 LiDAR output round-trips through the real hakoniwa PDU
binary (the same payload the SHM/endpoint transport carries) into a Python
consumer.

Run: python3 verify_pdu.py
"""

import math
import os
import struct
import subprocess
import sys
import tempfile

from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_LaserScan import pdu_to_py_LaserScan

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "lidar_a2_pdu")
ENV_XML = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-simenv-data/examples/sensor_envs/simple_room/generated/env.xml"))

# Drone pose: room centre shifted +2 m North (MuJoCo/ROS x=North), yaw 0.
POS = (2.0, 0.0, 1.0)
YAW = 0.0
TOL = 0.05

fails = []


def check(cond, msg):
    print(("  [ ok ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def main() -> int:
    if not os.path.exists(DEMO):
        print(f"ERROR: build first (bash build.bash). missing {DEMO}")
        return 2

    with tempfile.TemporaryDirectory() as td:
        pos_bin = os.path.join(td, "pos.bin")
        out_bin = os.path.join(td, "laser_scan.bin")

        # 1) Python -> pos Twist PDU
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = POS
        t.angular.x = t.angular.y = 0.0
        t.angular.z = YAW
        raw = py_to_pdu_Twist(t)
        with open(pos_bin, "wb") as f:
            f.write(bytes(raw))

        # 2) C++ A-2 sensing -> LaserScan PDU
        r = subprocess.run([DEMO, ENV_XML, pos_bin, out_bin],
                           capture_output=True, text=True)
        print(r.stdout.strip())
        if r.returncode != 0:
            print(r.stderr.strip())
            print("RESULT: FAIL (publisher error)")
            return 1

        # 3) Python decode LaserScan
        with open(out_bin, "rb") as f:
            data = f.read()
        scan = pdu_to_py_LaserScan(data)

    # hakoniwa_pdu returns primitive arrays as raw bytes; decode to float32.
    ranges = scan.ranges
    if isinstance(ranges, (bytes, bytearray)):
        ranges = list(struct.unpack(f"<{len(ranges) // 4}f", ranges))
    else:
        ranges = list(ranges)

    n = len(ranges)
    amin_deg = scan.angle_min * 180.0 / math.pi
    print(f"decoded LaserScan: {n} ranges, angle_min={amin_deg:.1f} deg, "
          f"range_max={scan.range_max:.1f}")

    def at(deg):
        idx = int(round(deg - amin_deg))
        return ranges[idx] if 0 <= idx < n else -1.0

    check(n == 361, f"range count == 361 (got {n})")
    check(abs(at(0.0) - 1.9) < TOL, f"forward +North -> wall_north 1.9m (got {at(0.0):.3f})")
    check(abs(at(180.0) - 5.9) < TOL, f"back -North -> wall_south 5.9m (got {at(180.0):.3f})")
    check(abs(at(90.0) - 4.9) < TOL, f"left +West -> wall_west 4.9m (got {at(90.0):.3f})")
    check(abs(at(-90.0) - 4.9) < TOL, f"right -West -> wall_east 4.9m (got {at(-90.0):.3f})")

    print()
    if fails:
        print(f"RESULT: FAIL ({len(fails)} check(s) failed)")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
