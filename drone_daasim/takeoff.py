#!/usr/bin/env python3
"""手順4(A): 最小 hakosim クライアントで離陸→移動し、pos の変化を表示。

前提: native drone service 起動済み ＋ conductor start 済み。
使い方: $PYENV_PY drone_daasim/takeoff.py [config_path]
  config_path 省略時は DRONE_CORE/config/pdudef/webavatar.json。
注意: Godot を SYNC アセットとして登録した状態では、sim が歩進しないと
      pos が読めず "MetaData not found" になる（＝Start 同期の調査対象）。
"""
import os
import sys
import time

import hakoniwa_pdu.apps.drone.hakosim as hakosim

# drone_daasim lives inside hakoniwa-mujoco-sensor, so ".." is that repo, not
# drone-core -- the sibling one directory further up. The old default resolved to
# hakoniwa-mujoco-sensor/config/pdudef/webavatar.json, which does not exist, so a
# bare `python takeoff.py` died with FileNotFoundError. Left over from the
# 2026-07-04 move of localsim/ into this repo, which fixed env.sh but not this.
_HERE = os.path.dirname(os.path.abspath(__file__))
DRONE_CORE = os.environ.get(
    "DRONE_CORE", os.path.join(os.path.dirname(os.path.dirname(_HERE)), "hakoniwa-drone-core"))
CFG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(DRONE_CORE, "config/pdudef/webavatar.json")


def main() -> int:
    c = hakosim.MultirotorClient(CFG, "Drone")
    c.confirmConnection()
    c.enableApiControl(True)
    c.armDisarm(True)

    def pos(retries=80):
        for _ in range(retries):
            try:
                p = c.simGetVehiclePose().position
                return (round(p.x_val, 3), round(p.y_val, 3), round(p.z_val, 3))
            except Exception:
                c.pdu_manager.run_nowait()
                time.sleep(0.1)
        return None

    print("pos before takeoff:", pos())
    c.takeoff(0.5)
    print("pos after takeoff :", pos())
    c.moveToPosition(1.0, 0.0, 0.5, 2.0)
    print("pos after move    :", pos())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
