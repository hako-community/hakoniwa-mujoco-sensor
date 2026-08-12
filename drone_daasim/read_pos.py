#!/usr/bin/env python3
"""手順4(B): 外部リーダで Drone/pos(ch1, Twist) を直接読む（SYNC に影響しない）。

native + conductor start が動いていれば pos が読める。
Godot 登録時に sim が止まっているかの切り分けに有用。
使い方: $PYENV_PY drone_daasim/read_pos.py [count]
"""
import sys
import time

import hakopy
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import pdu_to_py_Twist

POS_CH = 1   # webavatar.json の Drone/pos (geometry_msgs/Twist, 72B)
POS_SZ = 72
COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 10


def main() -> int:
    if not hakopy.init_for_external():
        print("ERROR: init_for_external failed")
        return 1
    got = 0
    for i in range(COUNT * 50):
        raw = hakopy.pdu_read("Drone", POS_CH, POS_SZ)
        if raw:
            try:
                t = pdu_to_py_Twist(bytearray(raw))
                print(f"[{i}] Drone pos = ({t.linear.x:.3f}, {t.linear.y:.3f}, {t.linear.z:.3f})")
                got += 1
                if got >= COUNT:
                    break
            except Exception as e:
                print(f"[{i}] decode error: {e}")
        time.sleep(0.1)
    if got == 0:
        print("pos が読めませんでした（sim 未歩進 or pos 未配信の可能性）")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
