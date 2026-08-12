# hakoniwa-mujoco-sensor

箱庭（[Hakoniwa](https://github.com/toppers/hakoniwa)）向けの **MuJoCo センサーライブラリ**。
`hakoniwa-mujoco-robots` から **センサー実装を分離・集約**した、バックエンド非依存
（Strategy C）のセンサーライブラリ。レイ/レンジ系センサー（LiDAR / Ultrasonic /
**Radar(新規)**）は具体的な物理・描画エンジンに直接依存せず、`IRayCaster`
抽象越しに動作する。MuJoCo バックエンドと Godot バックエンドのいずれでも同一の
センサーモデルを再利用できる。

## レイヤ構造

```
ISensor (sensor.hpp)
  └ センサーモデル (RadarSensor / LiDAR2DSensor / UltrasonicSensor ...)
        ├ backend::IRayCaster        ← レイキャストを抽象化（エンジン非依存）
        │     └ MujocoRayCaster      （mj_ray + mj_objectVelocity）
        │     └ (Godot側) IntersectRay 実装  ※C#側で同interfaceを実装
        ├ noise::RangeNoisePipeline  ← 共有ノイズ
        └ common::UpdateScheduler    ← 共有スケジューラ
  → ドメインFrame (RadarScanFrame ...)
        └ pdu::adapter (RadarPointCloudPduAdapter) → PDU(SHM)
```

センサーモデルは **PDU を知らない**（Frame を返すだけ）。PDU 化は `pdu/adapter`
層が担当。これにより 1 つのモデルを複数バックエンド・複数 PDU 表現で再利用できる。

**利用側の例**: [`hakoniwa-humanoid`](https://github.com/hako-community/hakoniwa-humanoid)
（31-DoF ヒューマノイドの PLANT）が本ライブラリを submodule として link し、
IMU / odometry / tf / joint_state / 足裏接触 / 力覚 / RGBD カメラ / バッテリを PDU へ流している。

**新しいセンサーはこのリポジトリに実装する**（下位の
[`hakoniwa-mujoco-runtime`](https://github.com/hako-community/hakoniwa-mujoco-runtime)
は物理バックエンドと箱庭アセット基盤のみを持つ）。

## ディレクトリ

| パス | 内容 |
|---|---|
| `include/sensor.hpp` | `ISensor` 基底（`IsSelfGeom` 含む） |
| `include/sensors/backend/` | **`IRayCaster`（Strategy C 抽象）** と `MujocoRayCaster` |
| `include/sensors/radar/` | **Radar: `radar_types` / `radar_math`（純粋） / `radar_sensor`** |
| `include/sensors/{lidar,ultrasonic,imu,odometry,tf,joint_state}/` | 既存センサー（mujoco-robots から集約） |
| `include/sensors/noise/`, `common/` | 共有ノイズ・スケジューラ |
| `include/hakoniwa/pdu/{converter,adapter}/` | Frame ⇔ PDU（`RadarPointCloudPduAdapter` 追加） |
| `src/sensors/...` | 上記の実装 |
| `tests/radar_math_test.cpp` | Radar モデルの単体テスト（バックエンド非依存、MuJoCo/PDU 不要） |
| `config/radar-sample.json` | Radar 設定サンプル |
| **`capi/`** | **C-ABI 成果物**（`hako_sensor_capi.{h,cpp}` + smoke + build.bash）→ `libhako_mujoco_sensor_capi.so` |
| **`examples/godot/`** | C-ABI を使う **Godot サンプル**（成果物ではない）。README 参照 |
| **`drone_daasim/`** | **ドローン DAA（Detect And Avoid）シミュレーション一式**。本ライブラリの利用側デモ。下記参照 |

## ビルド

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

- 静的ライブラリ `libhako_mujoco_sensor.a` を生成。
- ライブラリの**コンパイル**には MuJoCo ヘッダのみ必要（`libmujoco` のリンクは
  最終実行ファイル側の責務）。PDU アダプタはヘッダオンリーで、ロボット統合コードが
  include したときのみコンパイルされる。
- 主要オプション: `MUJOCO_INCLUDE_DIR`, `HAKONIWA_INSTALL_PREFIX`,
  `NLOHMANN_JSON_INCLUDE_DIR`, `HAKO_SENSOR_BUILD_TESTS`。

## Radar の使い方（C++ / MuJoCo バックエンド）

```cpp
auto caster = std::make_shared<backend::MujocoRayCaster>(world, "drone_body");
radar::RadarSensor radar(caster);
radar.LoadConfig("config/radar-sample.json");

// 毎ステップ:
if (radar.ShouldUpdate(dt)) {
    backend::SensorState st = /* origin / forward,left,up / linear_velocity をworldから構築 */;
    radar::RadarScanFrame frame;
    radar.Scan(st, frame);
    // frame -> PDU
    adapter::sensor_msgs::RadarPointCloudPduAdapter pub(endpoint, key);
    pub.send(frame);
}
```

Doppler 相対速度は `MujocoRayCaster` が `mj_objectVelocity` で対象 body の
世界速度を直接取得して算出する（Godot の位置差分トラッキングが不要）。

## Godot バックンド（Strategy C の第2実装）

Godot 側では C# で `IRayCaster` 相当（`IntersectRay` で hit 点・対象速度を返す）を
実装し、同じ `RadarSensor` ロジック（FOV分布・polar変換・相対速度・ノイズ仕様）を
共通仕様として共有する。詳細は `devai/radar_hakoniwa_porting_plan_20260627.md` 参照。

## C-ABI（`capi/`）— プロセス内直呼びの API 層（Phase 2 成果物）

core-pro / PDU / MuJoCo プロセス無しで、任意のコンシューマ（Godot・テスト・別エンジン）が
LiDAR/Radar モデルを **プロセス内で直接駆動**するための安定 C-ABI。world レイキャストは
C 関数ポインタで**注入**するため、この層は**コンシューマを一切知らない**（Godot 非依存）。
本体（`include/src`）は無改造で、`capi/` をラッパとして上に足すだけ。

```c
hako_sensor_handle h = hako_sensor_create("lidar3d"|"radar", config_json, raycast, user);
hako_sensor_scan(h, state /*[15]*/, dt, out_xyzi, max_points, &h, &w, &count);
hako_sensor_reset(h); hako_sensor_destroy(h);
```
- `state[15]` = origin/forward/left/up/linear_velocity（各3, world 系）。
- 出力は 4float/点（x,y,z,w）: LiDAR は intensity、Radar は Doppler 相対速度を w に格納
  （PDU 経路の PointCloud2 と同一 16B レイアウト）。
- `.so` は **libmujoco を必要としない**（本体センサ .cpp は mujoco を直接呼ばず、レイキャストは注入コールバックの責務）。

ビルド:
```bash
cd capi && bash build.bash   # -> libhako_mujoco_sensor_capi.so (+ smoke_capi)
./smoke_capi                 # backend-free smoke（lidar3d/radar が点群を返す）
```
（CMake でも `hako_mujoco_sensor_capi` SHARED ターゲット + `ctest -R hako_sensor_capi_smoke`。）

Godot からの使用例は `examples/godot/`（**サンプル止まり**、godot-drone にはコミットしない）を参照。
設計/検証の詳細は `devai/arch/phase2_api_layer_plan_20260701.md` と
`devai/phase2_api_layer_impl_report_20260704.md`。


---

## 提供センサー（2026-08 時点）

| センサー | 主な出力 / PDU 型 | 備考 |
|---|---|---|
| `joint_state` | `sensor_msgs/JointState` | MJCF の関節束を name/position/velocity で束ねる |
| `imu`（`ImuSensor` / `MjcfImuSensor`） | `sensor_msgs/Imu` | **`MjcfImuSensor` は MJCF の `framequat`/`gyro`/`accelerometer` 直読み**。`ImuSensor` は加速度を body 速度差分で作るため重力が入らない点に注意 |
| `odometry` | `nav_msgs/Odometry` | |
| `tf` | `tf2_msgs/TFMessage` | |
| `contact` | `std_msgs/Bool` + 接触力 | MJCF の `touch` センサ |
| `force_torque` | `geometry_msgs/WrenchStamped` | |
| `camera`（RGBD） | `sensor_msgs/{Image,CameraInfo}` | `HAKO_SENSOR_BUILD_CAMERA=ON` のときのみ（GLFW/OpenGL 依存） |
| `battery` | `hako_msgs/HakoBatteryStatus` | 関節トルク由来の電流積算 + 放電曲線 |
| `lidar` / `ultrasonic` / `radar` | 距離・点群 | `IRayCaster` 抽象越し（MuJoCo / Godot 両対応） |

---

## drone_daasim — ドローン DAA シミュレーション

`drone_daasim/` は本ライブラリの**利用側デモ**。レーダー / LiDAR で相手機を検知し、
**衝突回避（DAA: Detect And Avoid）** まで行う end-to-end シミュレーション一式。

ライブラリ本体（`include/` `src/`）とは独立していて、**本体のビルドには一切不要**。
逆に本ライブラリが「実機の制御ループに入れて成立するか」を確認する回帰テストでもある。

> 旧名 `localsim/`。2026-08-11 に改称し追跡対象にした（以前は `.gitignore` 対象で、
> リポジトリの再クローン時に丸ごと消失する事故があったため）。

### 構成

```
[1] drone-core 物理（master + conductor）  ... 2 機分の pos を配信、move 指令を受ける
[2] sensor_bridge_multi × 2（本ライブラリ） ... 各機の pos を読み、相手機を free joint の
                                              ACTOR として自分の運動学 MuJoCo 世界に注入し、
                                              radar/lidar を PDU 配信
[3] シナリオ（Python）                     ... 検知を読んで回避操舵、判定を出力
[4] Godot（任意）                          ... 可視化
```

**物理世界に障害物は無い**。相手機の存在を知っているのは [2] のセンサーだけなので、
「知覚 → 判断 → 駆動」が本物の閉ループになっている。

### 前提

| 依存 | 入手先 |
|---|---|
| MuJoCo 3.9.0（header + lib の**一致ペア**） | `<workspace>/.cache/deps/mujoco_bin-src`、または `MUJOCO_LIB_DIR` で指定 |
| `hakoniwa-drone-core` | 兄弟ディレクトリ。`lnx/linux-main_hako_drone_service` がビルド済みであること |
| `hakoniwa-simenv-data` | 兄弟ディレクトリ。環境ジオメトリ（`examples/sensor_envs/`）の供給元 |
| 箱庭 core | `/usr/local/hakoniwa`（`bin/hako-cmd`, `share/hakoniwa/offset`） |
| Python | pyenv 3.12.3（`hakopy`, `hakoniwa-pdu`） |
| Godot mono + .NET8（可視化時のみ） | `GODOT_MONO`, `DOTNET_ROOT` |

パスは全て `drone_daasim/env.sh` が解決し、同名の環境変数で上書きできる。

```bash
# センサーブリッジをビルド（初回のみ）
bash examples/envsim_sensor_a2/multi_build.bash
```

### 使い方

```bash
# 1) スタックを起動（headless）。第1引数はセンシング世界の選択
bash drone_daasim/two_drone_run.sh noground

# 2) シナリオを実行
python3 drone_daasim/two_drone_avoid.py      # S-1 正面衝突回避（RESULT: PASS を出力）

# 3) 後片付け（必須）
bash drone_daasim/cleanup.sh
```

**env モード**（`two_drone_run.sh` / `two_drone_viz_run.sh` 共通の第1／第5引数）:

| モード | センシング世界 | 用途 |
|---|---|---|
| `noground`（既定） | 地面なし | レーダーが相手機のみを返す。回避デモが最も見やすい |
| `ground` | 地面あり | 実機同様の静止クラッタが乗る（0.0 m/s の返りは全て地面） |
| `room` | simple_room（床+4壁+柱） | 静止物 + 動体の混在（S-7） |
| `crewed` | 有人機サイズの目標 | S-6 |

> 既定が `noground` なのは、**壁を消すとレーダーが地面反射で埋まる**ため。実測で
> `DETECTED: 4 object(s)` が全て 0.0 m/s の地面クラスタになり、肝心の相手機が埋没した。
> Godot は地面を**描いたまま**、センシング世界からだけ外している
> （`hakoniwa-simenv-data` の `actors.py --drop`）。

### シナリオ

| スクリプト | 内容 | 根拠 |
|---|---|---|
| `two_drone_avoid.py` | **S-1 正面衝突** — 両機が自機の右へ回避 | 施行規則 §182 |
| `scenario_s2_converging.py` | S-2 90° 交差 | §181 / §186 |
| `scenario_s3_overtaking.py` | S-3 追越 | §185 / §186 |
| `scenario_s4_vertical.py` | S-4 垂直回避（上昇/降下） | ISO 15964 3.9 |
| `scenario_s5_landing.py` | S-5 着陸機優先 | §183 / §184 |
| `scenario_s6_crewed.py` | S-6 有人機との遭遇（UAS が必ず譲る） | NEDO DRESS/JRC 実証の縮尺再現 |
| `scenario_s7_clutter.py` | S-7 静止物 + 動体の同時検知 | ISO 15964 4.6 |
| `scenario_s8_failsafe.py` | S-8 センサー故障時のフェイルセーフ | ISO 15964 6.2.5, 4.2 d)-f) |
| `scenario_b1_faceoff.py` | 相互検知 + 接近 Doppler の確認（回避なし） | — |

判定は ISO 21384-3 の 6 ステップ（探知 → 認識 → 回避機動 → 結果確認 → 復帰 → 飛行）で出力し、
最後に `RESULT: PASS` / `FAIL` を返す。

### 可視化（Godot）

```bash
# 2機・衝突回避シーン（two_drone_avoid.tscn）
bash drone_daasim/two_drone_viz_run.sh window radar oblique 1.0 noground
python3 drone_daasim/two_drone_avoid.py      # 別シェルで重ねて実行

# 1機・レーダースキャンシーン（sensor_viz.tscn。壁あり・検知3Dタグあり）
bash drone_daasim/sensor_viz_run.sh window radar top
```

実行時キー: `L`=LiDAR / `R`=Radar / `N`=なし、`C`=カメラ切替、`+`/`-`=ズーム。

センサー構成はマニフェストで差し替えられる（`config/a2/`）。機体ごとに別構成も可能:

```bash
# 前方60° + 後方セクターの混成で追越を見る
A2_MANIFEST=$PWD/config/a2/drone-a2-sensors.json \
A2_MANIFEST2=$PWD/config/a2/drone-a2-sensors-rear.json \
  bash drone_daasim/two_drone_viz_run.sh window radar top 0.45 noground
python3 drone_daasim/scenario_s3_overtaking.py
```

### 注意点

- **起動・停止の順序が重要**。`hako-cmd` は master が生きていないと SHM セマフォで
  **永久にブロック**し、SIGTERM も無視する。必ず `cleanup.sh` を使う
  （`hako-cmd stop` → プロセス kill → `reset` → mmap 削除の順）。
- **pdudef のチャネル構成を変えたら** `/var/lib/hakoniwa/mmap/*.bin` の削除が必須
  （`reset` では消えない。古いサイズの mmap が残ると `data_size mismatch` になる）。
- シナリオは**必ずクリーン起動の直後に**実行する。SIGTERM で殺したクライアントは
  アセット解除されず、master の lockstep が固まる。
- `demo/*.mp4`（デモ録画）は追跡していない。`demo_record.sh` で再生成できる。

詳細は [`drone_daasim/README.md`](./drone_daasim/README.md)。

---

## 由来と第三者成果物

本リポジトリは `hakoniwa-mujoco-robots` の再編（2026-08-02）で生まれた。取り込み元と扱いは次のとおり。

| 対象 | 扱い |
|---|---|
| `hakoniwa-mujoco-robots` | センサー実装の移設元（同一著者） |
| `hakoniwa-armpi` | `sensors/contact/` と `sensors/force_torque/` を昇格（同一著者・変更は namespace と include パスのみ） |
| `hakoniwa-humanoid` | `MjcfImuSensor`（MJCF センサ直読み + ノイズ）を昇格 |
| `hakoniwa-drone-pro`（`LicenseRef-hakoniwalab-NC`） | `battery_model.hpp` は**設計のみ参考**（電流ベースの容量積算 / 放電曲線の外部化）。**コードは 1 行も持ち込んでいない**。電流源はロータ電流ではなく関節トルク由来に置き換えている |
| [nlohmann/json](https://github.com/nlohmann/json)（MIT） | ヘッダを include するのみ（同梱しない） |
| [MuJoCo](https://github.com/google-deepmind/mujoco)（Apache-2.0） | ヘッダを include するのみ（同梱しない） |

`drone_daasim/` はドローン DAA デモ一式（本ライブラリの**利用側**）。詳細は下記
「[drone_daasim](#drone_daasim--ドローン-daa-シミュレーション)」節を参照。

---

## ライセンス

**[PolyForm Noncommercial License 1.0.0](./LICENSE)**（licensor: **hakoniwa community**）
— 利用は非営利目的に限ります。
商用利用については別途ご相談ください。

上表の第三者成果物はそれぞれの元ライセンスに従います（本ライセンスは上書きしません）。
