#!/usr/bin/env python3
"""
M4 check: env-XML-presence sensing-mode switch + same-PDU contract.

  A) scene WITH env.xml  -> Pattern A: mujoco-sensor publishes lidar_points,
                            and tells Godot to disable self-sensing.
  B) scene WITHOUT env.xml -> Pattern B: no PDU here; Godot self-senses.

Same-PDU: the Pattern-A lidar_points must satisfy the Godot lidar_points
contract (sensor_msgs/PointCloud2, fields x,y,z,intensity, organized
height x width, point_step 16) so a consumer is path-agnostic.

Run: python3 verify_m4.py
"""

import json
import os
import subprocess
import tempfile

from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "sensing_switch_demo")
CFG = os.path.normpath(os.path.join(HERE, "../../config/a2"))
ENV_XML = os.path.normpath(os.path.join(
    HERE, "../../../hakoniwa-envsim-sensor/examples/simple_room/generated/env.xml"))
MANIFEST = os.path.join(CFG, "drone-a2-sensors.json")
fails = []


def check(cond, msg):
    print(("  [ ok ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def run(scene, td):
    pos = os.path.join(td, "pos.bin")
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z = 0.0, 0.0, 1.0
    t.angular.x = t.angular.y = t.angular.z = 0.0
    with open(pos, "wb") as f:
        f.write(bytes(py_to_pdu_Twist(t)))
    sp = os.path.join(td, "scene.json")
    with open(sp, "w") as f:
        json.dump(scene, f)
    r = subprocess.run([DEMO, sp, pos, td], capture_output=True, text=True)
    print(r.stdout.strip())
    return r


def main() -> int:
    if not os.path.exists(DEMO):
        print(f"ERROR: build first (bash build.bash). missing {DEMO}")
        return 2

    # --- Pattern A: env.xml present ---
    print("[Pattern A] scene WITH env.xml")
    with tempfile.TemporaryDirectory() as td:
        r = run({"name": "with_env", "env": ENV_XML, "manifest": MANIFEST}, td)
        check("MUJOCO_A2" in r.stdout, "A: mode resolved MUJOCO_A2")
        check("godot_external_sensing=true" in r.stdout, "A: Godot external_sensing=true (self ray cast OFF)")
        lp = os.path.join(td, "lidar_points.bin")
        check(os.path.exists(lp), "A: mujoco-sensor published lidar_points")
        if os.path.exists(lp):
            with open(lp, "rb") as f:
                pc = pdu_to_py_PointCloud2(f.read())
            names = [f.name for f in pc.fields]
            # Godot lidar_points contract
            check(names == ["x", "y", "z", "intensity"], f"A: fields x,y,z,intensity (got {names})")
            check(pc.point_step == 16, f"A: point_step 16 (got {pc.point_step})")
            check(pc.height > 1 and pc.width > 1, f"A: organized {pc.height}x{pc.width}")

    # --- Pattern B: no env.xml ---
    print("[Pattern B] scene WITHOUT env.xml")
    with tempfile.TemporaryDirectory() as td:
        r = run({"name": "no_env", "env": "", "manifest": MANIFEST}, td)
        check("GODOT_SELF" in r.stdout, "B: mode resolved GODOT_SELF")
        check("godot_external_sensing=false" in r.stdout, "B: Godot self-sensing ON")
        check(not os.path.exists(os.path.join(td, "lidar_points.bin")),
              "B: mujoco-sensor did NOT publish (Godot publishes lidar_points)")

    print()
    print("Note: both paths emit the same lidar_points (PointCloud2 x,y,z,intensity);")
    print("Pattern B is the existing Godot Default3DLiDARController (not run headless here).")
    print("RESULT: PASS" if not fails else f"RESULT: FAIL ({len(fails)})")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
