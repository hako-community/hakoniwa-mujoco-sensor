# hakoniwa-mujoco-sensor

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
