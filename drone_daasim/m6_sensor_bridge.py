#!/usr/bin/env python3
"""M6 A-2 sensor bridge (external, Python/hakopy).

Attaches to the running hakoniwa master SHM as an external client (the same
path radar_receiver.py / takeoff.py use), then loops:
  read  Drone/pos (Twist) from SHM
  sense the env.xml obstacle world from that pose with the REAL A-2
        mujoco-sensor 3D LiDAR (examples/.../lidar3d_a2_pdu)
  write Drone/lidar_points (PointCloud2) back to SHM

This is the A-2 "detection" producer for the M6 end-to-end demo. The raw pos
Twist (x,y,z) is already in the env.xml MuJoCo frame (N,W,Up) -- hakosim's
simGetVehiclePose returns it unconverted -- so no frame transform is needed.

Env vars: HAKO_BINARY_PATH must point at the hakoniwa offset dir (set by env.sh).
Usage: python m6_sensor_bridge.py <env.xml> <lidar3d_a2_pdu_bin> [hz=5]
"""
import os
import sys
import time
import tempfile
import subprocess

import hakopy
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist, pdu_to_py_Twist

ROBOT = "Drone"
POS_CH, POS_SIZE = 1, 72
LIDAR_CH, LIDAR_SIZE = 16, 177424


def main() -> int:
    if len(sys.argv) < 3:
        print(f"usage: {sys.argv[0]} <env.xml> <lidar3d_a2_pdu_bin> [hz=5]")
        return 2
    env_xml = sys.argv[1]
    demo = sys.argv[2]
    hz = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    period = 1.0 / hz if hz > 0 else 0.2

    for p in (env_xml, demo):
        if not os.path.exists(p):
            print(f"[m6_sensor][ERROR] missing: {p}")
            return 2

    if not hakopy.init_for_external():
        print("[m6_sensor][ERROR] hakopy.init_for_external() failed "
              "(is the drone service / master running?)", flush=True)
        return 3
    print(f"[m6_sensor] attached to SHM (external). env={env_xml} hz={hz}", flush=True)

    tmpd = tempfile.mkdtemp(prefix="m6_sensor_")
    pos_bin = os.path.join(tmpd, "pos.bin")
    out_bin = os.path.join(tmpd, "pc2.bin")

    frames, reads = 0, 0
    try:
        while True:
            t0 = time.time()
            raw = hakopy.pdu_read(ROBOT, POS_CH, POS_SIZE)
            if raw and len(raw) > 0:
                try:
                    tw = pdu_to_py_Twist(raw)
                except Exception:
                    tw = None
                if tw is not None:
                    reads += 1
                    # rebuild a clean pos.bin in the format lidar3d_a2_pdu expects
                    t2 = Twist()
                    t2.linear.x, t2.linear.y, t2.linear.z = tw.linear.x, tw.linear.y, tw.linear.z
                    t2.angular.z = tw.angular.z
                    with open(pos_bin, "wb") as f:
                        f.write(bytes(py_to_pdu_Twist(t2)))
                    r = subprocess.run([demo, env_xml, pos_bin, out_bin],
                                       capture_output=True, text=True)
                    if r.returncode == 0:
                        with open(out_bin, "rb") as f:
                            lb = f.read()
                        if len(lb) <= LIDAR_SIZE:
                            buf = bytearray(LIDAR_SIZE)  # zero-padded to channel size
                            buf[:len(lb)] = lb
                            hakopy.pdu_write(ROBOT, LIDAR_CH, buf, len(buf))
                            frames += 1
                            if frames % 20 == 1:
                                print(f"[m6_sensor] frame#{frames} "
                                      f"pos=({tw.linear.x:.2f},{tw.linear.y:.2f},{tw.linear.z:.2f}) "
                                      f"-> lidar_points", flush=True)
                    else:
                        if frames == 0:
                            print(f"[m6_sensor][WARN] sensing failed: {r.stdout.strip()} {r.stderr.strip()[:160]}", flush=True)
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    print(f"[m6_sensor] stop. reads={reads} frames={frames}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
