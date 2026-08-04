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

`localsim/` はドローン/DAA 由来のローカル実験用資材で、本ライブラリとは無関係のため
**リポジトリには含めていない**（`.gitignore` 対象）。

---

## ライセンス

**[PolyForm Noncommercial License 1.0.0](./LICENSE)** — 利用は非営利目的に限ります。
商用利用については別途ご相談ください。

上表の第三者成果物はそれぞれの元ライセンスに従います（本ライセンスは上書きしません）。
