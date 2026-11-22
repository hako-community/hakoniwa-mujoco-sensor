#!/usr/bin/env python3
"""realsense_depth_offline の回帰試験（MuJoCo・GL 不要。numpy だけ使う）。

    python3 tests/realsense_depth_offline_test.py <realsense_depth_offline の実行ファイル>

config/realsense/d405_wrist.json（手首の D405）で、理想の深度の平面・段差に誤差を掛けて確かめる:
  1. 平面 0.15 / 0.25 / 0.40 m：RMS が理論値 z²σ/(fx·B)（深度の単位の丸めを足したもの）の ±20%、平均のずれが小さい
  2. 左の無効帯の幅 = fx·B/z（±1 画素）
  3. 範囲外（MinZ 7 cm より近い・0.5 m より遠い）は無効
  4. 段差（手前 0.20 m・奥 0.30 m）で、手前の物の左の奥の面に遮蔽の影（無効）が出る（左の撮像が基準）
  5. 同じ乱数の種なら同じ結果、--frame を変えると違う結果
  6. 大きさが設定と合わない深度画像は断る
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
CONFIG = REPO / "config" / "realsense" / "d405_wrist.json"
W, H = 1280, 720
FX = W / (2 * math.tan(math.radians(87.0) / 2))
B = 0.018
SIGMA_D = 0.08
UNIT = 0.0001

checks = 0
failures = 0


def check(cond: bool, msg: str) -> None:
    global checks, failures
    checks += 1
    failures += 0 if cond else 1
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")


def run(tool: str, tmp: Path, depth: np.ndarray, *extra: str, material: str = "work_cast_disc"):
    np.save(tmp / "in.npy", depth.astype(np.float32))
    np.save(tmp / "ids.npy", np.zeros(depth.shape, np.int32))
    (tmp / "names.json").write_text(json.dumps([material]))
    r = subprocess.run([tool, "--config", str(CONFIG), "--depth", str(tmp / "in.npy"), "--out", str(tmp / "out.npy"),
                        "--materials", str(tmp / "ids.npy"), "--material-names", str(tmp / "names.json"), *extra],
                       capture_output=True, text=True)
    if r.returncode != 0:
        return None, r
    return np.load(tmp / "out.npy"), json.loads(r.stdout)


def main() -> int:
    tool = sys.argv[1]
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        print("[1-2] planes")
        for z in (0.15, 0.25, 0.40):
            out, st = run(tool, tmp, np.full((H, W), z))
            v = np.isfinite(out)
            theory = math.sqrt((z * z * SIGMA_D / (FX * B)) ** 2 + UNIT ** 2 / 12)
            e = out[v] - z
            rms = float(np.sqrt(np.mean(e ** 2)))
            # 模様の弱い画素は σ が大きくなる（texture model の sigma scale）ので、理論値は下限として ±20% を見る
            check(0.8 * theory <= rms <= 1.2 * theory * max(1.0, st["disparity_sigma_px_applied"] / SIGMA_D) * 1.15,
                  f"z={z}: rms {rms * 1e3:.3f} mm vs theory {theory * 1e3:.3f} mm")
            check(abs(float(np.mean(e))) < 0.2 * theory + UNIT, f"z={z}: mean {np.mean(e) * 1e3:+.4f} mm")
            first = int(np.argmax(v.any(axis=0)))
            check(abs(first - FX * B / z) <= 1.0, f"z={z}: left band {first} px vs fx*B/z {FX * B / z:.1f} px")
            check(v[:, first + 5:].mean() > 0.85, f"z={z}: valid ratio right of the band {v[:, first + 5:].mean():.3f}")

        print("[3] range gate")
        for z in (0.05, 0.6):
            out, _ = run(tool, tmp, np.full((H, W), z))
            check(not np.isfinite(out).any(), f"z={z}: all invalid")

        print("[4] step edge (occlusion shadow left of the near object)")
        step = np.full((H, W), 0.30)
        step[:, 640:] = 0.20                      # 右半分が手前
        out, st = run(tool, tmp, step)
        shadow = FX * B * (1 / 0.20 - 1 / 0.30)   # 右の撮像からは見えない奥の面の幅
        band = ~np.isfinite(out[:, 640 - int(shadow) + 2:640])
        clear = np.isfinite(out[:, 300:640 - int(shadow) - 8]).mean()
        check(band.mean() > 0.9, f"shadow {shadow:.1f} px left of the step is invalid ({band.mean():.3f})")
        check(clear > 0.85, f"far plane away from the shadow stays valid ({clear:.3f})")
        check(st["edge"]["occluded"] > 0, f"edge stats occluded {st['edge']['occluded']}")

        print("[5] determinism")
        a, _ = run(tool, tmp, np.full((H, W), 0.25))
        b, _ = run(tool, tmp, np.full((H, W), 0.25))
        c, _ = run(tool, tmp, np.full((H, W), 0.25), "--frame", "1")
        check(np.array_equal(a, b, equal_nan=True), "same seed -> same frame")
        check(not np.array_equal(a, c, equal_nan=True), "--frame 1 -> different noise")

        print("[6] size mismatch is rejected")
        out, r = run(tool, tmp, np.full((480, 640), 0.25))
        check(out is None and "stream" in r.stderr, "640x480 rejected")

    print(f"\n{checks - failures} / {checks} checks passed")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
