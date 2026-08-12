#!/usr/bin/env python3
"""
M5 visualization harness: render the A-path lidar_points over env.tscn in Godot
and save a PNG (demonstrates "Godot visualizes the lidar_points").

  A-path (lidar3d_a2_pdu) -> lidar_points -> decode hit points
    -> world (Godot) coords -> godot_viz/viz.gd renders env.tscn + cloud -> PNG

Needs a display (DISPLAY). Run: python3 render_viz.py [out.png]
"""

import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile

from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
A_DEMO = os.path.join(HERE, "lidar3d_a2_pdu")
GODOT = "/usr/local/bin/Godot_v4.6.3-stable_linux.x86_64"
VIZ_PROJ = os.path.join(HERE, "godot_viz")
ENV_XML = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-simenv-data/examples/sensor_envs/simple_room/generated/env.xml"))
ENV_TSCN = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-simenv-data/examples/sensor_envs/simple_room/generated/env.tscn"))
POS = (0.0, 0.0, 1.0)


def main() -> int:
    out_png = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(HERE, "lidar_viz.png")
    if not os.path.exists(A_DEMO):
        print(f"ERROR: build first. missing {A_DEMO}"); return 2

    with tempfile.TemporaryDirectory() as td:
        # A-path -> lidar_points
        pos_bin = os.path.join(td, "pos.bin")
        t = Twist(); t.linear.x, t.linear.y, t.linear.z = POS
        t.angular.x = t.angular.y = t.angular.z = 0.0
        open(pos_bin, "wb").write(bytes(py_to_pdu_Twist(t)))
        lp = os.path.join(td, "lidar_points.bin")
        r = subprocess.run([A_DEMO, ENV_XML, pos_bin, lp], capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stderr); return 1
        with open(lp, "rb") as f:
            pc = pdu_to_py_PointCloud2(f.read())
        data = bytes(pc.data) if isinstance(pc.data, (bytes, bytearray)) else bytes(bytearray(pc.data))

        # decode hit points (intensity>0), sensor-local REP-103 -> Godot world
        # pose MuJoCo (mx,my,mz); world MuJoCo = pose + (px,py,pz);
        # Godot world = (X=-myW, Y=mzUp, Z=mxN) = (-(py), mz+pz, px)  [pose=(0,0,1)]
        pts = []
        for i in range(pc.height * pc.width):
            x, y, z, inten = struct.unpack_from("<4f", data, i * pc.point_step)
            if inten <= 0.0:
                continue
            wm = (POS[0] + x, POS[1] + y, POS[2] + z)  # MuJoCo world (N,W,Up)
            g = (-wm[1], wm[2], wm[0])                 # -> Godot (E,Up,N)
            pts.append(g)
        pts_path = os.path.join(td, "points.csv")
        with open(pts_path, "w") as f:
            for g in pts:
                f.write(f"{g[0]:.4f} {g[1]:.4f} {g[2]:.4f}\n")
        print(f"A-path hit points: {len(pts)}")

        shutil.copy(ENV_TSCN, os.path.join(VIZ_PROJ, "env.tscn"))
        cmd = [GODOT, "--path", VIZ_PROJ, "--rendering-method", "gl_compatibility",
               "++", "--points", pts_path, "--out", out_png]
        env = dict(os.environ)
        env.setdefault("DISPLAY", ":1")
        r = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=120)
        for l in r.stdout.splitlines():
            if "VIZ" in l:
                print("  " + l)
        if r.returncode != 0:
            print("godot stderr tail:", r.stderr.strip()[-400:])

    if os.path.exists(out_png) and os.path.getsize(out_png) > 1000:
        print(f"RESULT: PASS  screenshot={out_png} ({os.path.getsize(out_png)} bytes)")
        return 0
    print("RESULT: FAIL (no screenshot; rendering may be unavailable on this display)")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
