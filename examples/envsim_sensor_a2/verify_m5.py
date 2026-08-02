#!/usr/bin/env python3
"""
M5 check: A/B numeric reconciliation with REAL Godot, headless.

  A-path: hakoniwa-mujoco-sensor (mj_ray over env.xml)  -> lidar_points
  B-path: Godot intersect_ray over env.tscn (Pattern B)  -> ranges
          (godot_bpath/bpath.gd, the same mechanism as Default3DLiDARController)

Both see the SAME geometry (env.xml <-> env.tscn from a single OBB source), so
their per-beam ranges must agree. This is the numeric A/B reconciliation that
M4 deferred to M5, and it exercises the actual Godot ray casting.

Run: python3 verify_m5.py
"""

import math
import os
import shutil
import struct
import subprocess
import tempfile

from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
A_DEMO = os.path.join(HERE, "lidar3d_a2_pdu")
GODOT = "/usr/local/bin/Godot_v4.6.3-stable_linux.x86_64"
BPATH_PROJ = os.path.join(HERE, "godot_bpath")
ENV_XML = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-envsim-sensor/examples/simple_room/generated/env.xml"))
ENV_TSCN = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-envsim-sensor/examples/simple_room/generated/env.tscn"))
N_V, N_H = 17, 361
POS = (0.0, 0.0, 1.0)
TOL = 0.05
fails = []


def check(cond, msg):
    print(("  [ ok ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def a_path_ranges(td):
    pos_bin = os.path.join(td, "pos.bin")
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z = POS
    t.angular.x = t.angular.y = t.angular.z = 0.0
    with open(pos_bin, "wb") as f:
        f.write(bytes(py_to_pdu_Twist(t)))
    out = os.path.join(td, "lidar_points.bin")
    r = subprocess.run([A_DEMO, ENV_XML, pos_bin, out], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("A-path failed: " + r.stderr)
    with open(out, "rb") as f:
        pc = pdu_to_py_PointCloud2(f.read())
    data = bytes(pc.data) if isinstance(pc.data, (bytes, bytearray)) else bytes(bytearray(pc.data))
    rngs = []
    for i in range(pc.height * pc.width):
        x, y, z, _ = struct.unpack_from("<4f", data, i * pc.point_step)
        rngs.append(math.sqrt(x * x + y * y + z * z))
    return rngs, pc.height, pc.width


def b_path_ranges(td):
    shutil.copy(ENV_TSCN, os.path.join(BPATH_PROJ, "env.tscn"))
    out = os.path.join(td, "bpath_ranges.csv")
    cmd = [GODOT, "--headless", "--path", BPATH_PROJ, "++",
           "--pose", str(POS[0]), str(POS[1]), str(POS[2]), "0", "--out", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    line = [l for l in r.stdout.splitlines() if "BPATH" in l]
    print("  " + (line[0] if line else r.stdout.strip()[-200:]))
    with open(out) as f:
        return [float(x) for x in f if x.strip()]


def main() -> int:
    if not os.path.exists(A_DEMO):
        print(f"ERROR: build first (bash build.bash). missing {A_DEMO}")
        return 2
    if not os.path.exists(GODOT):
        print(f"ERROR: Godot not found at {GODOT}")
        return 2

    with tempfile.TemporaryDirectory() as td:
        a, h, w = a_path_ranges(td)
        b = b_path_ranges(td)

    check(h == N_V and w == N_H, f"A-path grid {h}x{w} == {N_V}x{N_H}")
    check(len(a) == len(b) == N_V * N_H, f"beam counts equal ({len(a)} vs {len(b)})")

    n = min(len(a), len(b))
    diffs = [abs(a[i] - b[i]) for i in range(n)]
    diffs_sorted = sorted(diffs)
    maxd = max(diffs)
    median = diffs_sorted[n // 2]
    over = sum(1 for d in diffs if d > TOL)
    frac = over / n
    print(f"  A/B per-beam range diff: median={median:.4f}m max={maxd:.4f}m, "
          f">{TOL}m: {over}/{n} ({frac*100:.2f}%)")

    # cardinal beams (iv=8 pitch 0): forward/back/left/right
    def at(iv, ih):
        return a[iv * N_H + ih], b[iv * N_H + ih]
    for label, ih, exp in [("forward(+N)", 180, 3.9), ("back", 0, 3.9),
                           ("left(+W)", 270, 4.9), ("right", 90, 4.9)]:
        av, bv = at(8, ih)
        check(abs(av - exp) < TOL and abs(bv - exp) < TOL and abs(av - bv) < TOL,
              f"{label}: A={av:.3f} B={bv:.3f} (~{exp})")

    check(median < 0.01, f"median diff < 0.01m (got {median:.4f})")
    check(frac < 0.02, f"<2% beams disagree >{TOL}m (got {frac*100:.2f}%)")

    print()
    print("RESULT: PASS" if not fails else f"RESULT: FAIL ({len(fails)})")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
