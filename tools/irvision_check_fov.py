#!/usr/bin/env python3
"""iRVision 2D カメラの MJCF を MuJoCo で実際に描画し、画角が設定 JSON と合うかを確かめる。

光軸の正面 distance [m] に、大きさの分かった白い板（黒い背景の前）を置いて撮り、
画像に写った板の画素幅から「その距離で写る範囲」を逆算して、JSON から計算した値と比べる。
グレースケールへの変換は CameraSensor の L8 と同じ式（0.299 R + 0.587 G + 0.114 B）。

使い方:
    MUJOCO_GL=egl python tools/irvision_check_fov.py [--config config/irvision/irvision_2d_gray_8mm.json]
                                                     [--distance 0.3] [--out fov.png]
必要: mujoco（3.9.0。hakoniwa-mujoco-runtime の mujoco.lock と同じ版）、numpy。--out には Pillow。
終了コード: 0 = 合格（横・縦とも許容差以内）、1 = 不合格。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]


def build_scene(camera_xml: Path, distance: float, board_w: float, board_h: float) -> mujoco.MjModel:
    world = mujoco.MjSpec()
    world.visual.global_.offwidth = 1280  # attach ではカメラ側の visual は引き継がれない
    world.visual.global_.offheight = 1024
    world.visual.headlight.ambient = [0.0, 0.0, 0.0]
    world.visual.headlight.diffuse = [0.0, 0.0, 0.0]
    body = world.worldbody
    # 撮る面を正面から均一に照らす平行光（LED の色の影響を避ける）
    body.add_light(name="fill", pos=[0, 0, 0], dir=[1, 0, 0], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                   diffuse=[1, 1, 1], ambient=[0.3, 0.3, 0.3])
    # 黒い背景（板の 0.05 m 奥）と白い板
    body.add_geom(name="background", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[distance + 0.05, 0, 0],
                  size=[0.001, 2.0, 2.0], rgba=[0, 0, 0, 1])
    body.add_geom(name="board", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[distance, 0, 0],
                  size=[0.0005, board_w / 2, board_h / 2], rgba=[1, 1, 1, 1])
    # カメラを原点に置く（本体の座標 = 世界の座標。光軸 +x）
    cam_spec = mujoco.MjSpec.from_file(str(camera_xml))
    for light in cam_spec.lights:  # 赤色 LED は画角の確認では消す
        light.active = False
    frame = body.add_frame(pos=[0, 0, 0])
    frame.attach_body(cam_spec.body("irvision_2d"), "cam_", "")
    return world.compile()


def measure(gray: np.ndarray, threshold: int = 128) -> tuple[int, int]:
    """白い板の横・縦の画素数（中央の行・列で数える）。"""
    h, w = gray.shape
    row = gray[h // 2, :] >= threshold
    col = gray[:, w // 2] >= threshold
    return int(row.sum()), int(col.sum())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--config", default=str(REPO / "config/irvision/irvision_2d_gray_8mm.json"))
    ap.add_argument("--distance", type=float, default=0.3, help="光学中心から板までの距離 [m]")
    ap.add_argument("--tolerance", type=float, default=0.02, help="許容差（相対）")
    ap.add_argument("--out", help="撮った画像（L8）を PNG で保存する")
    args = ap.parse_args()

    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    spec, irv = cfg["spec"], cfg["irvision"]
    width, height = spec["image"]["width"], spec["image"]["height"]
    hfov, vfov = spec["horizontal_fov"], irv["vertical_fov"]
    want_w = 2 * args.distance * math.tan(hfov / 2)
    want_h = 2 * args.distance * math.tan(vfov / 2)
    board_w, board_h = 0.75 * want_w, 0.75 * want_h  # 画面の 3/4 を占める板

    camera_xml = Path(args.config).with_name(irv["mjcf"])
    model = build_scene(camera_xml, args.distance, board_w, board_h)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        renderer.update_scene(data, camera="cam_irvision_cam")
        rgb = renderer.render()
    gray = (0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]).astype(np.uint8)

    px_w, px_h = measure(gray)
    got_w = board_w * width / px_w if px_w else float("inf")
    got_h = board_h * height / px_h if px_h else float("inf")
    err_w, err_h = abs(got_w - want_w) / want_w, abs(got_h - want_h) / want_h
    ok = err_w <= args.tolerance and err_h <= args.tolerance

    print(f"config        : {args.config}")
    print(f"image         : {width} x {height}  (camera fovy in model = {model.cam_fovy[0]:.3f} deg)")
    print(f"distance      : {args.distance:.3f} m, board {board_w*1000:.1f} x {board_h*1000:.1f} mm")
    print(f"board in image: {px_w} x {px_h} px")
    print(f"field of view : got {got_w*1000:.1f} x {got_h*1000:.1f} mm, want {want_w*1000:.1f} x {want_h*1000:.1f} mm")
    print(f"error         : {err_w*100:.2f} % x {err_h*100:.2f} %  (tolerance {args.tolerance*100:.1f} %)")
    print("RESULT        : " + ("PASS" if ok else "FAIL"))

    if args.out:
        from PIL import Image
        Image.fromarray(gray, mode="L").save(args.out)
        print(f"saved         : {args.out}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
