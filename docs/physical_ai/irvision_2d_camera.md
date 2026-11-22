# FANUC iRVision 2D カメラ（ロボット手首の 2D ビジョンセンサー）

協働ロボット（FANUC CRX）の手首に付ける iRVision 2D カメラを、既存の `CameraSensor` の上に
**機種プロファイル＋設定 JSON＋MJCF のカメラモデル**として追加した。
新しい描画処理は書いていない（グレースケール `L8`・内部パラメータ fx/fy/cx/cy は既存の機能）。
既存のファイルは `CMakeLists.txt` にテストを登録した以外は変更していない。

Isaac Sim のアセットには同等のカメラが無いため、ここを正本とし、USD は image2sim-framework で
この設定から作る（`image2sim-framework` の `config/camera_templates/irvision_2d.yaml`）。

## 構成

| ファイル | 内容 | 依存 |
|---|---|---|
| `include/sensors/camera/irvision/irvision_profile.hpp` | 機種プロファイル（カタログの組み合わせ、画角・焦点距離の画素換算・写る範囲、LED の色、`CameraSensor` 用 JSON の生成） | nlohmann のみ（純粋） |
| `config/irvision/irvision_2d_gray_8mm.json` | 設定（グレースケール・8 mm・赤色 LED）。`LoadCameraConfigFromJson` / `LoadCameraProfileConfigFromJson` でそのまま読める | |
| `config/irvision/irvision_2d.xml` | MJCF のカメラモデル（本体・窓・レンズ・LED・撮像・赤色スポットライト） | MuJoCo 3.9.0 |
| `tests/irvision_profile_test.cpp` | 光学の計算、カタログの検査、JSON・MJCF がプロファイルと一致するか（50 項目） | 常にビルド |
| `tools/irvision_check_fov.py` | MuJoCo で実際に描画し、写る範囲を JSON の値と比べる（ctest の `irvision_fov_render_test`） | Python の mujoco・numpy。EGL |

## 仕様と情報源

| 項目 | 値 | 確かさ | 情報源 |
|---|---|---|---|
| 画像 | グレースケール 1280 × 1024（カラーは 640 × 512） | 公式 | FANUC iRVision の仕様表 |
| レンズ | 8 / 12 / 16 / 25 mm（既定 8 mm） | 公式 | 同上。実機のラベルは 8 mm に丸 |
| LED 照明 | 赤・白・なし（既定 赤） | 公式 | 同上。実機の写真で赤く光る |
| 外形・質量 | 80 × 131.8 × 74 mm、0.6 kg | 公式 | 同上 |
| 撮像素子 | 1/1.8 型 | 推定 | 販売店の記載（Kowa SC130EF2） |
| 画素ピッチ | 5.625 µm（1/1.8 型の公称の横幅 7.2 mm ÷ 1280） | 推定 | 型番が分かったら差し替える |
| 撮像の大きさ | 7.2 × 5.76 mm | 推定 | 上の 2 つから |
| 画角（8 mm） | 横 48.46°、縦 39.60° | 計算 | |
| 焦点距離（画素） | fx = fy = 1422.2 px、cx = 640、cy = 512 | 計算 | |
| 写る範囲（300 mm 先） | 270 × 216 mm | 計算 | |
| 更新周期 | 10 Hz | 仮 | フレームレートは公開されていない |
| 描画の範囲 | 0.03〜3.0 m | 仮 | |

カラー版は、同じ光学サイズを 640 × 512 で使う（画素ピッチ 11.25 µm）として扱う（推定）。

## 座標

`irvision_2d` の本体の座標（MJCF）:

| 軸 | 向き |
|---|---|
| 原点 | 前面の窓の中心（＝光学中心。`irvision_optical` サイトとカメラ `irvision_cam` を置く） |
| +x | 光軸の向き（前）。本体は x = −131.8〜0 mm に伸びる |
| +y | 左（前面の窓は y 方向 80 mm） |
| +z | 上（前面の窓は z 方向 74 mm） |

MuJoCo のカメラは自分の −z を見るので、`xyaxes="0 -1 0 0 0 1"`（画像の右 = −y、上 = +z）で +x を向かせている。
光学は `focal`・`sensorsize`・`resolution` で与えており、`fovy` は使わない。

## 使い方

ロボットのセルへは、mjSpec の attach で取り込む（hakoniwa-cobot の `gen_cell.py` がハンドを付けるのと同じ方法）。

```python
cam = mujoco.MjSpec.from_file("config/irvision/irvision_2d.xml")
mount = robot_spec.body("hand").add_frame(pos=[...], quat=[...])  # 取り付け位置はセルごとに決める
mount.attach_body(cam.body("irvision_2d"), "wrist_", "")
robot_spec.visual.global_.offwidth = 1280   # ★ 下の注意を参照
robot_spec.visual.global_.offheight = 1024
```

C++ では、既存の `CameraSensor` に `config/irvision/irvision_2d_gray_8mm.json` を読ませる
（`camera_name` は attach の接頭辞を付けた名前、上の例なら `wrist_irvision_cam`）。

> ★ **画面外バッファの大きさ**：MuJoCo の画面外描画は、モデルの `visual/global` の `offwidth`・`offheight`
> （既定 640 × 480）を超える画像を描けない。カメラの MJCF には 1280 × 1024 を書いてあるが、
> **attach ではこの設定は取り込む側へ引き継がれない**ので、セルの MJCF でも 1280 × 1024 以上にすること。

## 確認

```bash
cmake -S . -B build -DMUJOCO_ROOT=<mujoco-3.9.0> -DPython3_EXECUTABLE=<mujoco を入れた python>
cmake --build build -j
ctest --test-dir build -R irvision --output-on-failure
```

| テスト | 内容 | 結果（2026-11-22、Linux・MuJoCo 3.9.0） |
|---|---|---|
| `irvision_profile_test` | 光学の計算・カタログ・JSON・MJCF の一致 | 50/50 合格 |
| `irvision_fov_render_test` | 300 mm 先に白い板を置いて描画し、写る範囲を逆算 | 269.4 × 215.4 mm（設定 270 × 216 mm、差 0.2%・0.3%） |

`irvision_fov_render_test` は、ctest の Python で `import mujoco` ができるときだけ登録する。
既存の設定 JSON の読み込み（`LoadCameraConfigFromJson`・`LoadCameraProfileConfigFromJson`）でも、
同じ値と `camera_name = irvision_cam` が読めることを確かめた。

## 未対応・今後

- レンズのゆがみ（歪曲収差）は入れていない（8 mm の産業用レンズは小さいとみて、ピンホールで近似）。
- 露光・ゲイン・ノイズは既存の `noise`（ガウス）だけ。iRVision の露光制御は再現していない。
- 取り付け位置（手首のどこに、どの向きで付くか）はロボット側の設定で決める。
  CRX-25iA（Mazak Ez LOADER 30）の手首での位置は、動画からの推定値を devai_camera で管理している。
