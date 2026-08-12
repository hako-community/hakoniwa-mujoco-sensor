#!/usr/bin/env python3
"""U1: mujoco-sensor 3D LiDAR published as a REGISTERED hakoniwa SYNC asset.

Unlike the M6 external bridge (hakopy.init_for_external + pdu_write, whose writes
do NOT surface to PduManager readers), this registers as a real SYNC asset via
hakopy.asset_register, so its Drone/lidar_points write is visible to other
PduManager-based assets (Godot, hakosim getLidarData) -- exactly like drone-core's
pos is visible to Godot. Sensor body stays in mujoco-sensor (lidar3d_a2_pdu does
the real C++ sensing over env.xml); this asset only orchestrates pos->sense->publish.

Usage: python lidar_sensor_asset.py <pdu_config.json> <env.xml> <lidar3d_a2_pdu> [delta_msec=1] [sense_every=20]
"""
import os, sys, tempfile, subprocess
import hakopy
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import py_to_pdu_Twist, pdu_to_py_Twist

ASSET_NAME = "LidarSensor"
POS_CH, POS_SIZE = 1, 72
LIDAR_CH, LIDAR_SIZE = 16, 177424

config_path = env_xml = demo = ""
delta_time_usec = 1000
sense_every = 20
_step = 0
_tmpd = None
_frames = 0


def my_on_initialize(context):
    return 0

def my_on_reset(context):
    return 0

def my_on_simulation_step(context):
    global _step, _frames
    _step += 1
    if (_step % sense_every) != 0:
        return 0
    raw = hakopy.pdu_read("Drone", POS_CH, POS_SIZE)
    if not raw:
        return 0
    try:
        tw = pdu_to_py_Twist(bytes(raw))
    except Exception:
        return 0
    t2 = Twist()
    t2.linear.x, t2.linear.y, t2.linear.z = tw.linear.x, tw.linear.y, tw.linear.z
    t2.angular.z = tw.angular.z
    pos_bin = os.path.join(_tmpd, "pos.bin")
    out_bin = os.path.join(_tmpd, "pc2.bin")
    with open(pos_bin, "wb") as f:
        f.write(bytes(py_to_pdu_Twist(t2)))
    r = subprocess.run([demo, env_xml, pos_bin, out_bin], capture_output=True, text=True)
    if r.returncode == 0:
        with open(out_bin, "rb") as f:
            lb = f.read()
        if len(lb) <= LIDAR_SIZE:
            buf = bytearray(LIDAR_SIZE)
            buf[:len(lb)] = lb
            hakopy.pdu_write("Drone", LIDAR_CH, buf, len(buf))
            _frames += 1
            if _frames % 20 == 1:
                print(f"[lidar_asset] frame#{_frames} pos=({tw.linear.x:.2f},{tw.linear.y:.2f},{tw.linear.z:.2f}) -> lidar_points", flush=True)
    return 0


my_callback = {
    "on_initialize": my_on_initialize,
    "on_simulation_step": my_on_simulation_step,
    "on_manual_timing_control": None,
    "on_reset": my_on_reset,
}


def main():
    global config_path, env_xml, demo, delta_time_usec, sense_every, _tmpd
    if len(sys.argv) < 4:
        print(f"usage: {sys.argv[0]} <pdu_config.json> <env.xml> <lidar3d_a2_pdu> [delta_msec=1] [sense_every=20]")
        return 2
    config_path = sys.argv[1]
    env_xml = sys.argv[2]
    demo = sys.argv[3]
    delta_time_usec = (int(sys.argv[4]) if len(sys.argv) > 4 else 1) * 1000
    sense_every = int(sys.argv[5]) if len(sys.argv) > 5 else 20
    _tmpd = tempfile.mkdtemp(prefix="lidar_asset_")

    ret = hakopy.asset_register(ASSET_NAME, config_path, my_callback, delta_time_usec, hakopy.HAKO_ASSET_MODEL_PLANT)
    if not ret:
        print("[lidar_asset] ERROR: asset_register failed", flush=True)
        return 1
    print(f"[lidar_asset] registered SYNC asset '{ASSET_NAME}' (delta={delta_time_usec}us, sense_every={sense_every})", flush=True)
    hakopy.start()
    print("[lidar_asset] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
