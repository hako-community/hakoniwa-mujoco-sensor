# RealSense デプスカメラ（RealSenseCameraSensor / RealSenseImuSensor）

`RealSense-Overview-ja-2026-09.pdf`（devai_sensor）の D400 / D500 シリーズを、
MuJoCo 描画の上に**機種プロファイル＋ステレオ深度誤差モデル**として追加した。
あわせて、ローリングシャッター、librealsense 相当の後処理フィルター、IMU 出力を持つ。
既存の `CameraSensor` / `DepthCameraSensor` / `RgbdCameraSensor` / `StereoCameraSensor`
と `ImuSensor` / `MjcfImuSensor` は**変更していない**（新規ファイルの追加だけ）。

## 構成

| ファイル | 内容 | 依存 |
|---|---|---|
| `include/sensors/camera/realsense/realsense_profile.hpp` | 機種表（14 機種、IMU 仕様を含む） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_depth_model.hpp` | ステレオ深度誤差モデル | なし（純粋） |
| `include/sensors/camera/realsense/realsense_rolling_shutter.hpp` | ローリングシャッター歪み | なし（純粋） |
| `include/sensors/camera/realsense/realsense_edge_effects.hpp` | 深度の境界：隠れ領域（影）と飛び点（RS-1、既定 OFF） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_depth_metrics.hpp` | 真値との比較（欠損率・外れ値率・RMS） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_texture_model.hpp` | 材質表と、模様が見えない所の穴（RS-2、既定 OFF） | nlohmann のみ |
| `config/realsense/materials.json` | 材質表のサンプル（MuJoCo の material 名 → IR の性質） | |
| `include/sensors/camera/realsense/realsense_specular_model.hpp` | 光沢面の白飛びと相互反射の鏡像（RS-3、既定 OFF） | なし（光線は外から渡す） |
| `include/sensors/camera/realsense/realsense_thermal_model.hpp` | 温度ドリフト：暖機・外気温・自己校正、深度と IMU（RS-4、既定 OFF） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_exposure_model.hpp` | 露光・ゲイン（自動／手動）とモーションブラー（RS-5、既定 OFF） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_imu_timing.hpp` | IMU の時刻ずれ・揺らぎ・欠落・取り付け位置と角度、フレームの揺らぎ・欠落（RS-6、既定 OFF） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_post_processing.hpp` | 後処理フィルター（間引き・閾値・空間・時間・穴埋め） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_imu_model.hpp` | IMU 誤差モデル（BMI055 / BMI085） | なし（純粋） |
| `include/sensors/camera/realsense/realsense_config.hpp` | 設定構造体と JSON 読み込み・検証 | nlohmann のみ |
| `realsense_camera_sensor.hpp` / `src/sensors/camera/realsense_camera_sensor.cpp` | カメラ本体（MuJoCo 描画） | `HAKO_SENSOR_BUILD_CAMERA=ON` |
| `realsense_imu_sensor.hpp` / `src/sensors/camera/realsense_imu_sensor.cpp` | IMU 本体（MuJoCo から真値） | MuJoCo のみ（GL 不要、本体ライブラリに含む） |
| `config/realsense/*.json` | 設定サンプル（D435i / D455 / D405 / D421 / D555 / D415） | |
| `tests/realsense_depth_model_test.cpp` | 機種表・深度誤差・JSON の単体テスト | 常にビルド |
| `tests/realsense_effects_test.cpp` | 後処理・ローリングシャッター・IMU 誤差の単体テスト | 常にビルド |
| `tests/realsense_edge_effects_test.cpp` | 隠れ領域・飛び点・評価指標の単体テスト | 常にビルド |
| `tests/realsense_texture_model_test.cpp` | 材質表・穴・入射角・ノイズ倍率の単体テスト | 常にビルド |
| `tests/realsense_specular_model_test.cpp` | 白飛び・鏡像の単体テスト（解析的な光線で代用） | 常にビルド |
| `tests/realsense_thermal_model_test.cpp` | 温度・視差のずれ・自己校正・IMU のゼロ点の単体テスト | 常にビルド |
| `tests/realsense_exposure_model_test.cpp` | 自動露光・ゲイン雑音・明るさ・ブラーの単体テスト | 常にビルド |
| `tests/realsense_imu_timing_test.cpp` | 取り付け位置・角度、時刻の揺らぎ・欠落の単体テスト | 常にビルド |
| `tests/realsense_render_smoke.cpp` | 実描画のスモークテスト（壁・ポール・回転台） | カメラ ON + libmujoco + OpenGL |

## 情報源

- **PDF**（RealSense Overview ja 2026-09）: 各機種の FOV、シャッター、精度、Min-Z、RGB、IMU の有無、IP65。
- **Intel の技術資料（Web）**:
  - "Tuning depth cameras for best performance": 深度 RMS 誤差 = z²·subpixel/(fx·B)。
    よく校正された機体の subpixel RMS は 0.1 px 未満で、資料のグラフは 0.08 px を使用
    （テクスチャが良ければ 0.05 px 近く）。Min-Z は X 解像度に比例
    （D415 1280 px で約 43.8 cm、D435 848 px で約 16.8 cm）→ 視差の探索幅 126 px に相当。
  - "Self-calibration for D400": D415 の基線長 約 55 mm、D435 約 50 mm。
  - "Post-processing filters": フィルターの順序と既定値。
  - D435i / D455 の IMU: Bosch BMI055（後に BMI085 へ変更、PCN 118035-00）。
    加速度 62.5/250 Hz、ジャイロ 200/400 Hz、ジャイロ ±2000 dps、加速度 12 bit（BMI055）。
- **Bosch（Web）**: 雑音密度の代表値 加速度 150 µg/√Hz、ジャイロ 0.014 dps/√Hz
  （Bosch Sensortec コミュニティの回答。正式なバイアス値は NDA）。

## 機種表

| 機種 | Depth FOV / シャッター | 精度（カタログ） | Min-Z（カタログ） | 基線長 | RGB | IMU |
|---|---|---|---|---|---|---|
| D421 | 75°×50° G | 4% @ 2 m | 14 cm | 50 mm | なし | − |
| D405 | 87°×58° G | 1.4% @ 20 cm | 7 cm | 18 mm※ | 1MP 87°×58° G | − |
| D415 | 65°×40° **R** | 2% @ 2 m | 18 cm | 55 mm（Web） | 2MP 69°×42° **R** | − |
| D435 / i / f / if | 87°×58° G | 2% @ 2 m | 12 cm | 50 mm（Web） | 2MP 69°×42° **R** | i, if |
| D436 | 87°×58° G | 2% @ 2 m | 20 cm | 50 mm | 1MP 90°×65° G | ○ |
| D455 / f / D456 / D457 | 87°×58° G | 2% @ 4 m | 20 cm | 95 mm | 1MP 90°×65° G | ○ |
| D555 | 87°×58° G | 2% @ 4 m | 26 cm (VGA) | 95 mm※ | 1MP 90°×65° G | ○ |
| D585Pro | 120°×100° G | **仮定** | 10 cm | **仮定** 95 mm | **仮定** | ○ |

G = グローバル、R = ローリング（ローリングの読み出し時間 25 ms は仮定値）。
※ は PDF・Intel 資料に記載がない値（公開データシートの値）。D4xx の最大距離 10 m も設定値。
D585 Pro は発表段階で `preliminary`（読み込み時に警告）。subpixel 0.04 px（「品質 2 倍」から）も仮定。
どの値も JSON の `overrides` で上書きできる。

## 深度誤差モデル

描画で得た理想の z 深度（m）に対して、画素ごとに次を順に適用する。無効値は `NaN`
（`DEPTH_U16_MM` で PDU 化すると 0）。

1. **距離の範囲**: z < Min-Z または z > 最大距離 → 無効
2. **左端の無効帯**: x < fx·B/z の画素 → 無効（左イメージャ基準のため、右画像に対応点がない）
3. **視差ノイズ**: d = fx·B/z に N(0, σd) を加え、z' = fx·B/d' に戻す。
   誤差は σz ≈ z²·σd/(fx·B) で、距離の 2 乗で増える
4. **サブピクセル量子化**（任意）: `subpixel_step_px` 刻み
5. **欠損**（任意）: ランダム欠損、材質指定の欠損（MuJoCo の material id）
6. **深度単位**: `depth_unit_m`（Z16 の既定 1 mm）

### ノイズの大きさ（`noise_model`）

| 値 | σd | 意味 |
|---|---|---|
| `typical`（既定） | `subpixel_rms_px`（0.08 px） | Intel 資料の典型値。実機の平面フィット RMS に近い |
| `spec` | ratio·fx_ref·B/z_ref | カタログ精度（2% @ 2 m 等）を 1σ とみなす。上限寄り（悲観的） |

例: D435 を 848×480 で使うと、2 m で typical ≈ **1.4 cm**、spec ≈ 6.0 cm。
カタログ精度は工場出荷の合否基準なので、通常の個体は typical に近い。

### Min-Z（`min_z_mode`）

- `disparity`（既定）: Min-Z = fx·B/(126 + disparity_shift)。解像度に比例して変わる（Intel 資料）。
  例: D415 1280 px → 43.8 cm、D555 640 px → 25.4 cm（PDF の 26 cm と一致）。
- `fixed`: カタログ値（`overrides.min_z_m` を書くと自動でこちら）。
- `disparity_shift_px` を上げると近くが見え、代わりに最大距離 fx·B/shift ができる（実機と同じ）。

## ローリングシャッター

ローリングの機種（D415 の深度と RGB、D435 系の RGB）で、カメラが動いていると行ごとに
撮影時刻がずれて像が歪む。行 r の時刻 t_r = t_mid + (r − (H−1)/2)·読み出し時間/H の
カメラ姿勢から光線を飛ばし、基準画像（t_mid の描画）を引く。

- 深度: 回転＋並進（基準深度で 1 回近似）。画像: 回転のみ（同じカメラで RGB と深度を
  一括描画したときは並進も入る）。
- カメラの速度は `SetMotionSource(world)` で MJCF カメラから読む（`mj_objectVelocity`）。
  設定しないと効果なし。テスト用に `SetCameraMotion()` で固定値も渡せる。
- 回転中は画面端の行が描画範囲の外を見に行くので、その画素は無効（深度 NaN、画像 0）。
- 読み出し時間は `rolling_shutter.depth_readout_s` / `rgb_readout_s` で変更できる。

## 深度の境界：隠れ領域と飛び点（`edge_effects`、既定 OFF）

ローリングシャッターの後、ステレオ深度誤差の前にかける（描画した深度の幾何で決まるため）。
乱数は深度誤差と別の系列（`edge_effects.seed`）なので、入れても深度ノイズの系列は変わらない。
`enabled: false` では出力が入れる前とビット一致する。

1. **隠れ領域（影）**: 深度は左イメージャ基準。列 x の点は右画像で x − d(x)（d = fx·B/z）に写る。
   右側の画素がすでに同じかより左に写っていれば、右イメージャからは手前の面に隠れて見えない → 無効。
   手前の物体の**左側**に幅 fx·B·(1/z_手前 − 1/z_奥) px の影ができる（D435 848 px で 0.6 m の箱と 1.0 m の壁なら約 15 px）。
   乱数を使わない幾何の判定。
2. **飛び点**: 隣り合う画素の深度差が `jump_ratio` × 近いほうの深度を超える所を段差とし、
   (2r+1)×(2r+1) の照合窓に段差を含む画素を、確率 `probability` で窓内の最も近い深度と最も遠い深度の間の一様乱数にする。
   段差のない急な斜面は対象にならない。範囲外（最大距離より遠い）の深度は混ぜない。
   r は解像度に比例（`window_px` は 848 px 基準。848 px で r = 4、424 px で r = 2）。

| 設定 | 既定 | 種類 |
|---|---|---|
| `occlusion` | true | 幾何（仮定なし） |
| `flying_pixels.window_px` | 7（848 px 基準） | **仮定値**（D4 の照合窓の大きさは非公開） |
| `flying_pixels.jump_ratio` | 0.05 | **仮定値** |
| `flying_pixels.probability` | 0.5 | **仮定値** |

処理時間は 848×480 で約 8 ms（MSVC Release、この PC）。段差の検出は積分画像で O(N)、窓の最小・最大は段差の近くだけ計算する。

評価の道具 `ComputeDepthMetrics(truth, measured, params)` は、真値（描画値）に対する欠損率、
外れ値率（|誤差| > 3σz、下限 2 mm）、外れ値を除いた RMS を返す。
実描画（0.6 m の箱と 1.0 m の壁、D435 typical）では、境界の効果で欠損率 2.71% → 3.22%、外れ値率 0.35% → 0.83%。
単体テストの合成画像では、後処理の空間フィルターで飛び点の外れ値率が 1.03% → 0.66% に減る（消えはしない）。

## 材質表と模様の見えない所の穴（`texture_model`、既定 OFF）

D400 はアクティブステレオで、IR プロジェクタの点の模様を左右の IR 画像で照合する。
模様の濃淡が見えない画素は深度が出ない。隠れ領域・飛び点の後、ステレオ深度誤差の前にかける。

```
C   = P · ρ · cosθ · (R_ref / z)²       プロジェクタの模様（P = laser_power_mw / 150）
N   = 1 + k · A · ρ                     環境光（A = ambient_ir）で雑音が増える
SNR = C / N + g · A · texture            ＋ 物体自身の模様
有効の確率 = 1 / (1 + (1/SNR)^γ)         → それ以外は穴
視差ノイズの倍率 = 1 + c / SNR            模様が弱いほど照合の誤差も増える（ApplyDepthModel に渡す）
```

- ρ・texture・transparent は**材質表**から引く。画素の材質は、描画のセグメンテーション（MuJoCo の material id）で決まる
  （描画は増えない）。material 名は `SetMaterialSource(world)`（なければ `SetMotionSource` の world）の model から取る。
- θ は深度画像の法線と視線のなす角。段差（5% 超）や無効の隣は使わない。
- 透明（`transparent`）は穴にする（奥の面を透かして見せるのは未対応）。
- SNR が十分高い画素（穴の確率 1e-8 未満、γ = 6 で SNR > 21.5）は乱数を引かずに有効とする。

### 材質表（`config/realsense/materials.json`）

```json
{
  "presets": { "blasted_aluminum": { "base": "machined_aluminum", "specular": 0.3, "texture": 0.3 } },
  "assign":  { "work_black*": "black_oxide", "work_*": "machined_aluminum", "bin_*": "cardboard" },
  "default": "white_resin"
}
```

- `assign` のキーは MuJoCo の material 名。`*` は任意の文字列。名前が完全に一致するものを優先し、なければ最も長いパターン。
- material の無い geom（rgba だけ）と、どれにも当たらない material は `default`。geom 名での割り当ては未対応。
- 設定の `"materials"` はファイル名（設定ファイルからの相対パス）か、表そのもの（オブジェクト）。

組み込みの材質（**すべて仮定値**。850 nm 付近の一般的な傾向から決めた値で、実測ではない）:

| 材質 | ρ（IR 反射率） | specular | texture | 屋内・正対で穴が 50% になる距離 |
|---|---:|---:|---:|---:|
| machined_aluminum | 0.60 | 0.80 | 0.10 | 6.9 m |
| ground_steel | 0.50 | 0.70 | 0.10 | 6.5 m |
| cast_iron | 0.15 | 0.10 | 0.40 | 8.1 m（自身の模様が効く） |
| black_oxide | 0.05 | 0.30 | 0.10 | **2.4 m** |
| black_resin | 0.05 | 0.20 | 0.00 | **2.2 m** |
| white_resin | 0.80 | 0.10 | 0.00 | 6.7 m |
| cardboard | 0.70 | 0.00 | 0.50 | 穴にならない（自身の模様で SNR ≥ 1） |
| concrete | 0.40 | 0.00 | 0.60 | 穴にならない（同上） |
| glass | 0.05 | 0.90 | 0.00（透明） | 0 m（常に穴） |

`specular` は RS-3（鏡面・相互反射）で使う。

### 設定

| 項目 | 既定 | 種類 |
|---|---|---|
| `laser_power_mw` | 150（0 でプロジェクタ OFF） | D435 の既定値 |
| `ambient_ir` | 0.1（屋内） | 日光は 1.0 程度（**仮定**） |
| `reference_range_m` | 10 | **仮定**（白い正対面で SNR = 1 になる距離。プロジェクタだけ・環境光なし） |
| `steepness`（γ） | 6 | **仮定** |
| `noise_gain`（c） | 0.5 | **仮定** |
| `ambient_noise_gain`（k） | 10 | **仮定** |
| `passive_gain`（g） | 20 | **仮定** |

処理時間は 848×480 で約 9 ms（MSVC Release、この PC）。

### 分かっている制約

- ローリングシャッターで深度を歪めたとき、材質 id は歪める前の画素のまま（ずれは行の読み出し中の動きの分だけ）。
- 屋外の値（日光で模様が消える度合い）と、材質ごとの値は実測で合わせていない。

## 光沢面の白飛びと相互反射（`specular_model`、既定 OFF）

ステレオ方式では ToF のようなマルチパスは主な問題ではない。バラ積みの金属ワークで深度を壊すのは次の 2 つで、
これに絞って入れた。材質の `specular` は材質表（`texture_model.materials`）から、
プロジェクタの出力は `texture_model.laser_power_mw` から取る（`texture_model` 自体は OFF でもよい）。
材質の穴（RS-2）の後、ステレオ深度誤差の前にかける。

1. **白飛び**: 面の法線 n が、プロジェクタ方向とイメージャ方向の中間ベクトル h に近いと、
   プロジェクタの光がそのまま返ってイメージャが飽和する → 穴。
   `p = highlight_gain · specular · exp(−(α/α0)²) · min(1, P)`（α = ∠(n, h)、α0 = `highlight_width_deg`）。
   正対した光沢板の中央に丸い穴、円柱では帯状の穴になる。
2. **相互反射（鏡像）**: 光沢面は隣の物体（別のワーク・箱の壁）を映す。ステレオは映った模様を
   **鏡の奥**にある物体として照合するので、深度は光の通った道のりになる:
   `z' = z · (range + t) / range`（range はカメラから面まで、t は反射光線が当たるまで）。
   確率 `p = mirror_gain · specular · ρ_当たった物体`。
   - 反射光線は MuJoCo の `mj_ray` で飛ばす（`SetMaterialSource` か `SetMotionSource` の world）。
     `specular ≥ min_specular` の画素だけ、`ray_stride_px` 四方のブロックに 1 本（ブロック内は当たった距離を共有）。
   - 深度画像から戻した表面の点は描画の精度で鏡の内側に 0.1〜0.5 mm 入ることがあるので、
     1 cm 未満の当たりは鏡自身とみなして先へ進めて飛ばし直す（最大 3 回、進めた距離は足す）。

| 設定 | 既定 | 種類 |
|---|---|---|
| `highlight_width_deg`（α0） | 8 | **仮定値** |
| `highlight_gain` / `mirror_gain` | 1.0 / 1.0 | **仮定値** |
| `projector_offset_m` | (+B/2, 0, 0)（左イメージャ基準、光学座標） | 公開図面からの概略（**仮定値**） |
| `max_distance_m` | 0.5 | 箱の大きさに合わせた設定値 |
| `ray_stride_px` | 4 | 速度と細かさの釣り合い |
| `min_specular` | 0.5 | アルミ 0.8・研削鋼 0.7 は対象、黒染め 0.3 は対象外 |
| `max_rays` | 40000 | 1 フレームの上限 |

処理時間（848×480、全画素が光沢の合成画像、光線の計算を除く）は約 61 ms。
ただし計測時は PC の CPU 使用率が 100%（別のプロセス）で、他のモデルも普段の約 3 倍に出ていたので、普段は 20 ms 前後と見込む（要再計測）。
実描画（2 枚の板）では、光線 2867 本で 1 回の撮影が約 80 ms 長くなった（同じ負荷の下で）。

### 分かっている制約

- ブロック内の画素は同じ反射距離を使うので、ブロックが別々の面にまたがると鏡像の位置がずれる。
- 鏡像は 1 回反射まで。鏡像の中の物体自身の材質（黒いので見えない、など）は確率に入れているが、鏡像の上の穴・飛び点は作らない。
- 白飛びはプロジェクタの光だけ。照明の映り込みは入れていない。

## 温度ドリフト（`thermal`、既定 OFF）

起動直後からカメラ内部が暖まり（暖機）、外気温（工場の朝と昼など）でも温度が変わる。
温度でレンズと基板が伸び縮みし、左右の画像の相対位置（視差の零点）がずれる。IMU のゼロ点も温度で動く。
時刻はシミュレーション時刻（描画の `data->time`、IMU は `Update` 時の `data->time`）。

```
温度    T(t) = T_外気(t) + ΔT_定常 · (1 − exp(−(t − t_起動)/τ))     （start_warm なら ΔT_定常）
        ΔT_定常 = steady_rise_k · ((1 − laser_share) + laser_share · P)   P = laser_power_mw / 150
深度    視差に β · (T − T_校正) [px] を足す → 深度の誤差 ≈ −z² · β · ΔT / (fx · B)（距離の 2 乗で増える）
IMU     ゼロ点 += TCO_軸 · (T − T_校正)   TCO_軸 は起動ごとに [−TCO, +TCO] の一様乱数（軸ごと）
```

- 外気温は `ambient_c`（一定）か、`ambient_schedule: [[t_s, °C], ...]`（直線補間、端は一定）。外気温の変化には遅れなく追従する（時間単位の変化を想定）。
- 起動時刻は最初の撮影（IMU は最初の `Update`）の時刻。`GetThermalModel().PowerOn(t)` で指定もできる。
- 校正温度 `calibration_temp_c` の既定は「25 °C で暖まった状態」（25 + ΔT_定常、プロジェクタ既定出力）。工場で暖機後に校正した個体を想定。
- 自己校正: `RealSenseCameraSensor::RunOnChipCalibration()`（直前の撮影時刻）／`RunOnChipCalibration(t)`、
  IMU は `RealSenseImuSensor::RunImuCalibration(t)`。その時点の温度が校正温度になる。
- カメラと IMU はそれぞれ温度モデルを持つ（同じ設定・同じ時刻に起動すれば同じ温度の曲線）。

| 設定 | 既定 | 種類 |
|---|---|---|
| `steady_rise_k`（ΔT_定常） | 15 K | **仮定値** |
| `time_constant_s`（τ） | 600 s（10 分） | **仮定値** |
| `laser_share` | 0.4（自己発熱のうちプロジェクタ分） | **仮定値** |
| `disparity_coeff_px_per_k`（β） | 0.005 px/K | **仮定値**（D435 848 px で 2 m・15 K なら約 1.3 cm） |
| `accel_tco_mg_per_k` | 1.0 mg/K | BMI055 データシートの値（**要確認**） |
| `gyro_tco_dps_per_k` | 0.015 °/s/K | BMI055 データシートの値（**要確認**） |

例（D435 848 px、既定値）: 冷えた状態で起動すると（25 °C、校正 40 °C、−0.075 px）、0.6 m で +1.2 mm、2 m で +13.5 mm 遠く読む。
1 時間後（暖機済み）はほぼ 0。朝 15 °C → 昼 30 °C の工場では、暖機後でも −0.05 px（朝）〜 +0.025 px（昼）。

## 露光・ゲインとモーションブラー（`exposure`、既定 OFF）

### 露光・ゲイン

描画は「明るさ 1 の場面を基準の露光（深度 8.5 ms、RGB 10 ms）・ゲイン 16 で撮った」画像とみなす。

```
必要な露光の積    E = t_基準 · g_基準 / L          （L: 場面の明るさ）
自動露光          t = clamp(E / g_基準, t_最小, t_最大)   t_最大 = 1/fps（max_exposure_us で変更）
                  g = clamp(E / t, g_基準, g_最大)       露光が上限に達したらゲインで補う
明るさ            b = t · g · L / (t_基準 · g_基準)       1 = 適正、< 1 暗い、> 1 明るすぎ
ゲインの雑音      √(g / g_基準) 倍（深度の視差ノイズと RGB の雑音）
```

- **深度（IR）の明るさ** L_IR = 有効画素の平均 ρ·(P/z² + ambient_gain·ambient_ir) / (0.8·(1 + 0.1·ambient_gain))。
  屋内・白い壁・1 m・既定のプロジェクタで 1。ρ は材質表、P と ambient_ir は `texture_model` から取る。
  暗いと（b < 1）視差ノイズを 1/√b 倍にする。
- **RGB の明るさ**は `rgb_light_level`（1 = 描画が適正な明るさ）。b ≠ 1 なら画素値を b 倍して 0〜255 で切る。
- 手動（`"auto": false`）では `depth_exposure_us`／`depth_gain`、`rgb_exposure_us`／`rgb_gain` をそのまま使う。
- 撮影ごとの結果は `GetLastDepthExposure()`／`GetLastRgbExposure()`（露光時間・ゲイン・明るさ）。

### モーションブラー

ローリングシャッターと同じく、描画は 1 回だけで、カメラの動き（`SetMotionSource`／`SetCameraMotion`）から
露光中の N 時刻（s_k = ((k + 0.5)/N − 0.5)·t）の姿勢に合わせて画像をずらす。ローリングの機種は行ごとの時刻も同時に入れる。

- RGB: N 枚の平均。
- 深度: 模様が流れると照合できないので、N 枚の視差のばらつきが `disparity_spread_px` を超えた画素は穴、それ以外は視差の平均。
  段差の両側に幅 約 (N−1)/N·ω·t·fx px の穴の帯ができる。
- **動いている物体はぶれない**（分かるのはカメラ自身の動きだけ）。
- ずらした先が画面の外になる画素は、深度は穴、RGB は黒として平均する（画面の両端が暗くなる）。

| 設定 | 既定 | 種類 |
|---|---|---|
| 基準の露光（`reference_depth_exposure_us`／`reference_rgb_exposure_us`） | 8.5 ms／10 ms | **仮定値**（librealsense の既定に近い値） |
| `reference_gain`／`max_gain` | 16／248 | **仮定値** |
| `min_exposure_us`／`max_exposure_us` | 100 µs／1/fps | 設定値 |
| `ambient_gain` | 10 | **仮定値**（日光 ambient_ir 1.0 が 1 m のプロジェクタの 10 倍） |
| `rgb_gain_noise_dn` | 1.0 DN（基準ゲイン） | **仮定値** |
| `motion_blur.subframes` | 4 | 速度と細かさの釣り合い |
| `motion_blur.disparity_spread_px` | 1.0 px | **仮定値** |

## バラ積みの見本場面と評価（RS-7）

| ファイル | 内容 |
|---|---|
| `tools/gen_binpicking_scene.py` | 場面の生成（固定の種）。段ボールの箱にワーク 30 個（アルミの軸 12・黒染めのブロック 10・鋳物 8）、加工機の治具と見本荷姿、下向きのカメラ `rs_bin`（床から 0.75 m）と `rs_near`（0.40 m） |
| `examples/realsense_binpicking/binpicking_scene.xml` | 生成した MJCF（ワークは自由物体。撮る前に物理で 3 秒落ち着かせる） |
| `config/realsense/d435i_binpicking.json` | 全部入りのプリセット（D435i、冷えた起動 20 °C、自動露光、IMU の揺らぎ・取り付け角度） |
| `config/realsense/d405_binpicking.json` | 近距離のプリセット（D405、実機どおりプロジェクタなし、照明 ambient_ir 0.5 は仮定） |
| `tests/realsense_binpicking_eval.cpp` | 効果を 1 つずつ・全部入れたときの欠損率・外れ値率・RMS・撮影時間を箱の中で比べ、表（`realsense_binpicking_eval.md`）と図（RGB・理想の深度・全部入りの深度・誤差の図の PNG）を書き出す |

結果（2026-11-07、D435i、箱の上 0.72 m）: 外れ値率は ステレオ誤差のみ 0.47% → 飛び点・影 5.89%、白飛び・鏡像 4.75%、
冷えた起動 5.66%（約 2 mm の一様なずれ）、全部入り 14.51%。欠損率は全部入り 2.79%、カメラが 1 rad/s で振れると 7.12%。
D405（0.37 m、パッシブ）は全部入りで欠損 18.93%。

## 後処理フィルター（librealsense 相当、既定 OFF）

順序: 間引き → 閾値 → 視差へ変換 → 空間 → 時間 → 深度へ戻す → 穴埋め。

| フィルター | 主な設定（既定値） |
|---|---|
| decimation | magnitude 2（2〜3 は中央値、4〜8 は平均）。出力解像度が 1/n になり、`GetDepthCameraConfig()` も追従 |
| threshold | min 0.1 m, max 4.0 m |
| spatial | iterations 2, alpha 0.5, delta 20, holes_fill 0（0〜5: なし/2/4/8/16/無制限 px） |
| temporal | alpha 0.4, delta 20, persistency 3（2/4 フレームで有効なら穴を前値で埋める） |
| hole_filling | mode 1（0 左から、1 周囲の最も遠い値、2 最も近い値） |

delta の単位は、視差領域では 1/32 px（D4 のサブピクセル単位。20 = 0.625 px）、
`disparity_domain: false` では mm。librealsense と動作は同じ考え方だが、ビット単位の一致ではない
（間引き後の 4 px 揃えはしない）。時間フィルターは状態を持つので、`Reset()` で初期化される。

## IMU（RealSenseImuSensor）

```
測定値 = 飽和( 量子化( 真値 + バイアス + 白色雑音 ) )
```

- 真値は深度用の MJCF カメラ位置で取得（ジャイロ: `mj_objectVelocity`、加速度: `mj_objectAcceleration`。
  加速度は比力で重力を含む）。光学座標（x 右、y 下、z 前）で出力するので、水平に置くと加速度は
  y ≈ −9.81 m/s²（D435i 実機と同じ向き）。
- 白色雑音 σ = 雑音密度·√(レート)。量子化は ±レンジ / 2^(bits−1)（加速度 ±4 g は仮定値）。
- バイアス `bias_mode`: `uncalibrated`（データシートのゼロ点ずれ ±70 mg、±1 dps の範囲で一様）、
  `calibrated`（その 10%、既定）、`none`。`bias_random_walk` でジャイロのバイアス不安定性（2°/h）を足せる。
- 加速度とジャイロは別レートで出す（`Update()` が該当サンプルを返す）。`GetLatest()` は最新の
  ジャイロと加速度をまとめた `ImuFrame` を返すので、既存の `sensor_msgs/Imu` 変換に渡せる
  （姿勢は出さないので単位四元数）。物理の刻み幅はジャイロ周期以下にすること。
- チップは既定 BMI055。新しい個体（BMI085）は `overrides.imu` で `chip` と `accel_bits: 16` を指定する。
- IMU チップとカメラ原点の位置ずれ（数 cm）は、既定では無視する（`imu_timing.position_m` で入れられる。次の節）。

### IMU とフレームの時刻・取り付け（`imu_timing`／`frame_timing`、既定 OFF）

VIO・SLAM は IMU とカメラの時刻関係に敏感なので、それを崩す要素を入れられるようにした。

| 項目 | 内容 |
|---|---|
| 時刻ずれ `time_offset_s` | IMU のタイムスタンプに一定のずれを足す |
| 揺らぎ `jitter_s` | IMU のタイムスタンプに N(0, σ)。ストリームごとに単調増加を保つ |
| 欠落 `drop_probability` | IMU のサンプルが確率で抜ける（`DroppedAccel()`／`DroppedGyro()` で数を取れる） |
| 取り付け位置 `position_m` | チップの位置 r（深度の光学座標、深度原点から）。加速度に α×r + ω×(ω×r) を足す。ジャイロは剛体のどこでも同じ |
| 取り付け角度 `misalignment_deg` | 各軸 N(0, σ) の小さな回転を起動ごとに決め、加速度・ジャイロに掛ける |
| フレームの揺らぎ `frame_timing.jitter_s` | 深度と RGB のタイムスタンプに同じずれを足す（単調増加を保つ） |
| フレームの欠落 `frame_timing.drop_probability` | `Capture` の結果を空にする（`LastFrameDropped()`／`DroppedFrames()`） |

試すときの目安（**仮定値**）: 揺らぎはハードウェアのタイムスタンプで 0.1 ms、PC の時刻で 1 ms、取り付け角度 0.5°。
取り付け位置は実機の値を `rs-enumerate-devices -c` の深度 → IMU の外部パラメータで確かめて入れる（機種表には持っていない）。
処理の順は、取り付け位置 → 取り付け角度 → 温度のゼロ点 → 雑音・量子化。

## 設定（JSON）

全項目の例は `config/realsense/d435i.json`。書かなかった項目は機種の既定値になり、
`model` だけでも動く。機種名は大文字・小文字、空白、`-`、`RealSense ` の接頭辞を無視して照合する。

- `overrides`: `depth_hfov_deg`, `baseline_m`, `accuracy_ratio`, `accuracy_ref_distance_m`,
  `accuracy_ref_width`, `min_z_m`, `max_z_m`, `rgb_hfov_deg`, `has_rgb`, `has_imu`,
  `subpixel_rms_px`, `max_disparity_px`, `rolling_readout_s`, `rgb_rolling_readout_s`,
  `imu`（`chip`, `accel_range_g`, `gyro_range_dps`, `accel_bits`, `gyro_bits`,
  `accel_noise_density_ug`, `gyro_noise_density_dps`, `accel_zero_offset_mg`,
  `gyro_zero_offset_dps`, `gyro_bias_instability_dph`）
- `frame_id` は ROS の realsense2_camera と同じ形で `<frame_id>_depth_optical_frame`、
  `<frame_id>_color_optical_frame`、`<frame_id>_imu_optical_frame` になる。
- カタログ外の指定（解像度・fps・IMU レート）は通すが警告を出す。

## 使い方（C++）

```cpp
#include "sensors/camera/realsense/realsense_camera_sensor.hpp"
#include "sensors/camera/realsense/realsense_imu_sensor.hpp"
namespace rs = hako::robots::sensor::camera::realsense;

auto renderer = std::make_shared<camera::MujocoCameraRenderer>(world);
rs::RealSenseCameraSensor cam(renderer, "rs_depth", "rs_color");  // 第3引数省略で同じカメラ
cam.LoadConfig("config/realsense/d435i.json");
cam.SetMotionSource(world);                                       // ローリングシャッター用

rs::RealSenseImuSensor imu(world, "rs_depth");
imu.LoadConfig("config/realsense/d435i.json");

// 毎ステップ
std::vector<rs::MotionSample> accel, gyro;
imu.Update(dt, accel, gyro);
hako::robots::sensor::ImuFrame imu_frame;
if (imu.GetLatest(imu_frame)) { /* ToHakoPdu(imu_frame) */ }

if (cam.ShouldUpdate(dt)) {
    camera::ImageFrame rgb;
    camera::DepthFrame depth;
    cam.Capture(rgb, depth);          // RGB なしの機種では rgb は空
    // ToHakoPdu(cam.GetDepthCameraConfig(), depth.timestamp, camera_info);  // 間引き後のサイズ
    // ToHakoPdu(cam.GetRgbCameraConfig(),   rgb.timestamp,   camera_info);
}
```

## モデルに入れていないもの

動いている物体のブラー。
温度ドリフトは `thermal`、露光・ゲインとカメラの動きによるブラーは `exposure`、
IMU とカメラの時刻・取り付けは `imu_timing`／`frame_timing` で入れた。
（深度の境界の飛び点と隠れ領域は `edge_effects`、模様の見えない所の穴は `texture_model`、
光沢面の白飛びと相互反射は `specular_model` で入れた。ToF 型のマルチパスは扱わない。）

## テスト

```bash
ctest --test-dir build -R "realsense_(depth_model|effects|edge_effects|texture_model|specular_model|thermal_model|exposure_model|imu_timing)_test" --output-on-failure
# 実描画（-DHAKO_SENSOR_BUILD_CAMERA=ON と MUJOCO_LIBRARY が必要）
ctest --test-dir build -R realsense_render_smoke --output-on-failure
```

2026-11-06 に Windows（MSVC 2022、vcpkg の MuJoCo）で確認:
単体テスト 2 本とも合格、実描画 41/41。主な実測:
D435i 848 px・2 m の壁で RMS 1.432 cm（式の値 1.432 cm）、左の無効帯 12 px（11.2 px）、
D415 を 5 rad/s で回転させたときのポールの位置 735.5 / 814.5 px（式の値 735.2 / 814.7 px）、
IMU のジャイロ真値 −5.000 rad/s・加速度真値 −9.81 m/s²（光学座標の y）。

2026-11-07（RS-1 追加後）: 単体テスト 3 本（132／58／37）、実描画 54/54 すべて合格。
主な実測: 0.6 m の箱の左の影 14 px（式の値 14.9 px）、合成画像の影 22 px（22.3 px）・52 px（52.1 px）、
飛び点は箱の縁から r 以内にだけ出る。

2026-11-07（RS-2 追加後）: 単体テスト 4 本（132／58／37／47）、実描画 66/66 すべて合格。
主な実測（2.45 m の 3 枚の板、D435）: 穴の割合 黒染め 0.525（モデルの値 0.510）、アルミ 0.000、段ボール 0.000。
プロジェクタ OFF では 黒染め 1.000、アルミ 1.000、段ボール 0.499。
合成画像: 黒染めの穴は 1.8 / 2.2 / 2.8 m で 0.043 / 0.264 / 0.786（モデル 0.044 / 0.266 / 0.783）、
2 m で正対 0.115 → 70° 傾けると 0.764。

2026-11-07（RS-3 追加後）: 単体テスト 5 本（132／58／37／47／30）、実描画 78/78 すべて合格。
主な実測（D435、アルミ）: 正対した板の白飛び 中央 0.776（モデル約 0.79）、中央から約 8° で 0.225（モデル約 0.27）。
縦の円柱（半径 4 cm、0.76 m）は軸に沿った帯だけが白飛び（帯 0.479、側面 0.000。描画の円柱は多角柱なので、滑らかな円柱の値 約 0.68 より低い）。
45° の板では側面の壁の鏡像が 0.59〜0.63 の画素に出て（モデル 0.64）、深度の中央値 0.998 m（式の値 約 1.00 m、板そのものは約 0.70 m）。
解析シーン: 45° の鏡で 1.0 m の面に 0.5 m 先の壁が映り、鏡像は 1.500 m。

2026-11-07（RS-4 追加後）: 単体テスト 6 本（132／58／37／47／30／36）、実描画 88/88 すべて合格。
主な実測（D435i、2 m の壁）: 冷えた起動で壁の平均が +13.52 mm（式の値 +13.52 mm）、1 時間後 +0.03 mm、
起動時に自己校正すると 0.00 mm。静止した IMU のジャイロは TCO·(25 − 40) を読み、校正後は 0。

処理時間の再計測（2026-11-07）は、別のプロセスで CPU 使用率が 97〜100% のままで普段の値は取れていない。
同じ条件の比較では、飛び点・影 18〜21 ms、材質の穴 19〜25 ms、鏡面（光線を除く）51〜62 ms で、
鏡面は飛び点・影の約 2.7 倍（空いているときの飛び点・影 7.6 ms から、鏡面は普段 20 ms 前後と見込む）。

2026-11-07（RS-5 追加後）: 単体テスト 7 本（132／58／37／47／30／36／41）、実描画 102/102 すべて合格。
主な実測（D435、0.6 m の箱と 1.0 m の壁、カメラが 3 rad/s で回転）: 深度の穴の帯 露光 2 ms で 4 px（式 4.0 px）、
10 ms で 20 px（式 20.1 px）。RGB（1280×720）の中央の行の中間調の画素 静止 2 → 回転 67。
暗い RGB（明るさ 0.2）の自動露光は 33.3 ms・ゲイン 24.0、深度は箱が近く明るいので 8.11 ms。
深度のブラー（848×480、4 枚）は 70.6 ms／フレーム（CPU が混んでいる状態。飛び点・影が 17 ms に出る条件）。

2026-11-07（RS-6 追加後）: 単体テスト 8 本（132／58／37／47／30／36／41／21）、実描画 114/114 すべて合格。
主な実測（D435i、5 rad/s で回る台）: 軸から 5 cm のチップの加速度 x = −1.2500 m/s²（−ω²r）、ジャイロは −5.0000 rad/s のまま。
取り付け角度 1°（σ）の個体で重力が 0.807° 傾く（回転角 0.807°）。フレームの揺らぎ 2 ms で時刻 1.00061 s（深度と RGB で同じ）。
実描画のテストは `main` に多数のセンサを置くので、MSVC ではスタックを 16 MB にしている（ライブラリ側の変更ではない）。
