#!/usr/bin/env python3
"""
A-1 cross-language check for the 3D LiDAR (PointCloud2).

  Python: build a `pos` Twist PDU
  C++:    lidar3d_a2_pdu senses env.xml -> writes PointCloud2 PDU
  Python: decode PointCloud2, parse x,y,z,intensity, verify geometry

Config is fixed in lidar3d_a2_pdu.cpp: channels=17 (v -40..40), width=361
(h -180..180), pose at room centre (0,0,1).

Run: python3 verify_pdu_3d.py
"""

import math
import os
import struct
import subprocess
import tempfile

from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "lidar3d_a2_pdu")
ENV_XML = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-envsim-sensor/examples/simple_room/generated/env.xml"))

N_V, N_H = 17, 361          # channels, width (must match lidar3d_a2_pdu.cpp)
POS = (0.0, 0.0, 1.0)
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
        out_bin = os.path.join(td, "pc2.bin")
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = POS
        t.angular.x = t.angular.y = t.angular.z = 0.0
        with open(pos_bin, "wb") as f:
            f.write(bytes(py_to_pdu_Twist(t)))

        r = subprocess.run([DEMO, ENV_XML, pos_bin, out_bin], capture_output=True, text=True)
        print(r.stdout.strip())
        if r.returncode != 0:
            print(r.stderr.strip()); print("RESULT: FAIL (publisher error)"); return 1

        with open(out_bin, "rb") as f:
            pc = pdu_to_py_PointCloud2(f.read())

    data = bytes(pc.data) if isinstance(pc.data, (bytes, bytearray)) else bytes(bytearray(pc.data))
    npts = len(data) // pc.point_step
    print(f"decoded PointCloud2: {pc.height}x{pc.width}, point_step={pc.point_step}, "
          f"{npts} points, fields={[f.name for f in pc.fields]}")

    def point(iv, ih):
        idx = iv * pc.width + ih
        x, y, z, inten = struct.unpack_from("<4f", data, idx * pc.point_step)
        return x, y, z, inten, math.sqrt(x * x + y * y + z * z)

    check(pc.height == N_V and pc.width == N_H, f"organized {pc.height}x{pc.width} == {N_V}x{N_H}")
    check(npts == N_V * N_H, f"point count == {N_V * N_H} (got {npts})")
    check([f.name for f in pc.fields] == ["x", "y", "z", "intensity"], "fields = x,y,z,intensity")

    # forward horizontal beam: iv=8 (pitch 0), ih=180 (yaw 0) -> wall_north 3.9 m, z~0
    x, y, z, inten, d = point(8, 180)
    check(abs(d - 3.9) < 0.05 and abs(z) < 0.02, f"forward horiz -> wall_north 3.9m,z0 (d={d:.3f} z={z:.3f})")

    # left horizontal beam: iv=8, ih=270 (yaw 90) -> wall_west 4.9 m
    x, y, z, inten, d = point(8, 270)
    check(abs(d - 4.9) < 0.05, f"left horiz -> wall_west 4.9m (d={d:.3f})")

    # down-forward beam: iv=0 (pitch -40), ih=180 -> floor top (z=0.1): depth=0.9/sin40=1.40, z=-0.9
    x, y, z, inten, d = point(0, 180)
    check(abs(d - 1.40) < 0.06 and abs(z + 0.9) < 0.05,
          f"down-40deg -> floor (d={d:.3f} z={z:.3f}, expect ~1.40,-0.90)")

    # up beam: iv=16 (pitch +40), ih=180 -> no ceiling -> max range, intensity 0
    x, y, z, inten, d = point(16, 180)
    check(inten == 0.0 and d > 19.0, f"up+40deg -> no hit (intensity={inten}, d={d:.2f})")

    print()
    print("RESULT: PASS" if not fails else f"RESULT: FAIL ({len(fails)})")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
