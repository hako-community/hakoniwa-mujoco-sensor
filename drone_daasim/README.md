# drone_daasim — ドローン DAA（Detect And Avoid）シミュレーション一式

> ★★ **2026-08-21（P1・案 B）: DAA の「判断と採点」は
> [`hakoniwa-drone-companion`](../../hakoniwa-drone-companion) へ移した。**
> 決定 §10-2「**sensor はセンシング専業、Companion は判断**」による。
> ここに残っているのは **センサ配線と箱庭スタックの起動手順**である。
>
> | 移したもの（→ `hakoniwa-drone-companion/scenarios/`） | ここに残したもの |
> |---|---|
> | `daa_metrics.py`（Well Clear・tau・採点） | ★ `daa_common.py`（**radar_fit / scan / PDU / fly_to** が本体。`radar_fit_test.py` と `probe_elevation.py` も使う） |
> | `scenario_b1_faceoff.py`, `scenario_s2`〜`s8` | `env.sh` / `*_run.sh` / `cleanup.sh` / `takeoff.py` / `read_pos.py` |
> | `two_drone_avoid.py`, `m6_avoid.py`, `verify_b1.py`, `probe_b1_cone.py` | `lidar_sensor_asset.py`, `m6_sensor_bridge.py`, `radar_fit_test.py`, `probe_elevation.py` |
>
> ★ **`env.sh` が `SCENARIOS_DIR` と `PYTHONPATH` を用意する**ので、ランナー `.sh` の
> 使い方は変わらない。Companion リポジトリが無い場所にあるなら `COMPANION_REPO` か
> `SCENARIOS_DIR` で指定する（無いと `env.sh` が WARN を出す）。
>
> ★ **DAA の正は C++**（`hakoniwa-drone-companion/include/core/`）。`daa_metrics.py` は
> **従**であり、食い違ったら C++ が正しい。両者は ctest `daa_conformance` で突き合わせている。
>
> ★★ **P1-b 完了（2026-08-22）**: `daa_common.py` にあった `classify_encounter` /
> `role_from_bearing` と役割の定数は、**DAA の規則**であって C++
> `core/rules_of_the_air.hpp` と同じものの 2 個目の実装だったので、
> `hakoniwa-drone-companion/scenarios/daa_rules.py` へ移した。
> 使う側は `import daa_rules` に変える（`env.sh` の `PYTHONPATH` はそのままで通る）。
>
> ★ **`SpeedTracker` はここに残した**。あれは規則ではなく**自機状態の推定**で、
> C++ の対応物も `rules_of_the_air.hpp` ではなく `companion_app.cpp` のインライン EMA である。
> 何より `probe_elevation.py`（センサ側）が使っているので、移すと
> **mujoco-sensor が companion に依存する**ことになり依存の向きが逆になる。
>
> ★★ **移しただけでは負債は減らない**（重複の置き場所が変わるだけ）ので、
> 移設先に ctest `rules_conformance` を付けた。C++（正）と Python（従）に同じ入力を食わせ、
> **誰が譲るかの符号**（相手が右 = az<0 → こちらが譲る）まで毎回突き合わせる。

> 旧名 `localsim`。2026-08-11 に `drone_daasim/` へ改称し、**git 管理下に入れた**
> （以前は `.gitignore` 対象で、リポジトリの再クローン時に消失する事故があったため）。

WebServer 非依存の **local-SHM** 経路で drone-core を起動し、`hakoniwa-mujoco-sensor` の
A-2 センサ（LiDAR/Radar）を配信して Godot avatar で可視化・調査するためのスクリプト一式。
すべて env 非破壊（pip/apt/sudo なし）。作成: 2026-06-28。

> **配置（2026-07-04 移設）**: 本 `drone_daasim/` は **hakoniwa-mujoco-sensor（このリポジトリ）内**に
> 置く（mujoco-sensor 導入用ツールのため。drone-core は非改変）。実行は mujoco-sensor 直下から
> `bash drone_daasim/<script>.sh`。他リポジトリ（drone-core の native, godot-drone）は sibling
> （`.../hakoniwa/`）として参照し、`env.sh` の `DRONE_CORE`/`GODOT_DRONE`/`SENSOR_REPO`/`SIMENV_DATA`
> で上書き可能。
>
> **リポジトリ再編（2026-08）への追随**:
> - `hakoniwa-envsim-sensor` は**廃止**され `hakoniwa-simenv-data` に統合。環境ジオメトリの
>   パスも `examples/<name>/` → `examples/sensor_envs/<name>/` に移動した。`env.sh` は
>   これを `SENSOR_ENVS` として解決する（旧 `ENVSIM_REPO` は後方互換で残置）。
> - `hakoniwa-mujoco-robots` は `hakoniwa-mujoco-sensor`（センサ）と
>   `hakoniwa-mujoco-runtime`（MuJoCo バックエンド + 箱庭アセット基盤）の 2 層に再編。
>   libmujoco の探索先も旧 robots の build ディレクトリから移動している（下記「前提」参照）。

## 現状サマリ（2026-06-28 時点の検証結果）

| 経路 | 結果 |
|---|---|
| native(物理) + conductor + hakosim（Godot なし） | ✅ 離陸・移動まで動作（pos (0,0,0)→takeoff後(0,0,0.484)→move後(0.985,0,0.499)） |
| native + **Godot avatar** + conductor | ⚠️ Godot 登録は成功（Plugins lib 整合後）だが **sim が歩進しない**（native `advanceTimeStep`=0 / Godot `EventStart`=0、pos は `MetaData not found`） |

→ **調査対象 = Godot を 2 つ目の SYNC アセットにすると Start が届かず conductor が同期待ちで停止する**点。

## ★原因判明（2026-06-29 / 上記サマリの仮説を訂正）

`HakoAssetImpl.cs` に計装ログ(`[hako-dbg]`)を追加して再ビルド・観測した結果、**昨日の「Godot に Start が届かない／EventStart=0」は誤り**だった。実際は:

- Godot は headless でも `_PhysicsProcess`→`Execute()` が正常に回る（`Execute#1..#66000`）。
- **Start イベントは届いている**（`PollEvent ev=HakoSimAssetEvent_Start`→`StartCallback BEGIN/END (asset_start_feedback sent)`）。
- state も `Stopped→Runnable→Running` と正常遷移。
- しかし `Running` 後も **`wtime=0 atime=0` のまま**＝world 時刻が 0 から進まない。

**真の原因 = 時刻同期のブートストラップ・デッドロック（lockstep / max_delay=0 想定）:**
1. drone 設定は `simulation.lockstep=true`, `timeStep=0.001`(=1000us, Godot 側 delta=1000 と一致)。lockstep では conductor の `max_delay≈0`。
2. conductor の `time_begins_to_move()`(core `hako_time.cpp`) は「どのアセットも world から `max_delay` 以上遅れていない」時だけ world を delta 進める。max_delay=0 だと **アセットが world より進んだ時刻を報告しないと world が進まない**。
3. native 物理アセットは「先にステップして world+delta を報告」する楽観型なので world を進められる（＝Godot 無し正常系が動く理由）。
4. 一方 **Godot の `HakoAssetImpl.Execute()` は `if (next_asset_time_usec <= world_time)` ゲートで「world が先に進んでから」しかステップしない**。world=0 では `1000<=0` が偽で**一度もステップできず**、heartbeat で常に time=0 を報告し続ける。
5. → max_delay=0 では Godot(time=0) が world=0 を pin。native は待ち、Godot も待ち→ **wtime=0 で相互停止**。pos PDU が書かれず takeoff.py が `MetaData not found`。

= native(楽観ステップ) と Godot(world 追従ステップ) の**ブートストラップ非対称性**が lockstep 下で噛み合わない、が核心。

## ★★解決（2026-06-29）— 真の原因と修正（上記「lockstep」説は撤回）

動作実績のある **C/C++ Godot 版**（`/data/buildman/ArmPi_Ultra/work_sensor/robot/hakoniwa-armpi/godot/armpi-viewer/addons/hakoniwa/scripts/hakoniwa_simulation_node.gd`）と突き合わせて**真因が確定**した：

- 動作版 `_tick_internal()` は **`is_pdu_sync_mode()` を最優先**で判定し、sync 時に **`notify_write_pdu_done()` を必ず送る**（START 時=`_handle_start_event` でも送る）。
- 問題の C# `HakoAssetImpl.Execute()` は **global の `asset_is_simulation_mode()`(常にTrue) を先**に判定するため sync 分岐に到達せず、**`asset_notify_write_pdu_done()` が一度も呼ばれない**。conductor が PDU 同期バリアを解放できず world が 0 から進まない（`wtime=0`）＝ takeoff の `MetaData not found`。

**修正（適用済み・検証OK）= F1**: `HakoAssetImpl.cs` を動作版に合わせて:
1. `Execute()` で `asset_is_pdu_sync_mode()` を `asset_is_simulation_mode()` より**先に**判定し、その分岐で PDU受信(EventTick)＋`asset_notify_write_pdu_done()` を呼ぶ。
2. `StartCallback()` に `asset_notify_write_pdu_done()` を追加（start_feedback の前）。

→ 再ビルド後の実測: `wtime` が 0→7000→1.6e6→3.6e6 と進行、takeoff 成功
（pos (0,0,0)→takeoff後(0,0,**0.478**)→move後(**0.981**,0,0.499)）。**Godot あり 2 アセット SYNC で離陸・移動が通った。**

**前提となる別修正（PDUサイズ整合）**: `hakoniwa-godot-drone/webavatar.json` の `pdu_size` を master(drone-core `config/pdudef/webavatar.json`) に一致させる（120↔256, 56↔64 がズレていた）。不一致だと PDU 受信が壊れる。加えて `fix_custom_json.sh apply` で custom.json のチャネル(pos=ch1)も整合。

**注意（共通ソース）**: `HakoAssetImpl.cs` は Win/Linux 共通。Windows で同 C# が動いていた理由は未解明（別ビルド or service が別 pdudef で asset を sync_mode にしない等の可能性）。F1 は動作版C/C++を踏襲する変更だが、Windows へ反映する際は要確認。

---
（旧・撤回済みの修正候補メモ: F2=conductor の max_delay>0 化／F3=Godot を PDU sync 専用アセット化。F1 で解決したため不要。）

## 前提（このPCで確認済み）
- Python: `~/.pyenv/versions/3.12.3/bin/python`（hakoniwa-pdu 1.3.5, hakopy, mujoco 等）
- .NET8: `~/.dotnet`（Godot mono の C# ビルド/実行に必要）
- Godot mono: `/usr/local/bin/Godot_v4.6.3-stable_mono_linux_x86_64/...`
- 箱庭 core: `/usr/local/hakoniwa`（`bin/hako-cmd`, `share/hakoniwa/offset`）
- libmujoco.so.3.9.0: `<workspace>/.cache/deps/mujoco_bin-src/lib`、無ければ
  `~/.pyenv/versions/3.12.3/lib/python3.12/site-packages/mujoco`
  （旧 `hakoniwa-mujoco-robots/src/cmake-build/_deps/...` は同リポの廃止に伴い消滅）
- センサー環境ジオメトリ: `hakoniwa-simenv-data/examples/sensor_envs/{simple_room,open_field}/`
- Plugins: `hakoniwa-godot-drone/Plugins/Linux/x86_64/{libhako_service_c.so,libshakoc.so}`（core 整合版に差替済み）
- Xvfb `DISPLAY=:1`（GPU 無し → Godot は `gl_compatibility`）

設定はすべて `env.sh` で上書き可（`MUJOCO_LIB_DIR`, `GODOT_MONO`, `PYENV_PY` 等）。

## ファイル
| スクリプト | 役割 |
|---|---|
| `env.sh` | 共通設定（各スクリプトが source）。`_check_env` で前提チェック |
| `build_godot.sh` | Godot C# ソリューションをビルド（初回/Scripts 変更後） |
| `fix_custom_json.sh apply\|restore\|show` | Godot `custom.json` を master(webavatar) にチャネル整合/復元 |
| `start_drone.sh` | 手順1: native 物理（**repoルートで起動必須**） |
| `start_godot.sh [headless\|window]` | 手順2: Godot avatar（cwd=godot-drone） |
| `conductor.sh start\|stop\|status\|reset` | 手順3: 箱庭 conductor |
| `takeoff.py` | 手順4A: hakosim で離陸→移動し pos 変化を表示 |
| `read_pos.py [count]` | 手順4B: 外部リーダで Drone/pos を直読（SYNC 非干渉） |
| `diag.sh` | 診断（advanceTimeStep / EventStart / register / status） |
| `cleanup.sh` | stop + kill + reset（native/Godot/`sensor_bridge_multi` を kill） |
| `run_sequence.sh [headless\|window]` | 上記を一括実行（bg 起動→start→diag） |
| `a2_viz_run.sh [window\|headless]` | **A-2 センサ可視化デモ（LiDAR+Radar 常設配信）**。native→Godot(外部センシング+env再構築)→start→`sensor_bridge_multi` を順序保証で起動 |

## 使い方

### A) まず正常系（Godot なし）で疎通確認
```bash
# 端末1
bash drone_daasim/start_drone.sh            # WAIT START まで出れば OK（起動したまま）
# 端末2
bash drone_daasim/conductor.sh start
$HOME/.pyenv/versions/3.12.3/bin/python drone_daasim/takeoff.py   # pos が 0→0.48 と変化すれば疎通OK
# 後片付け
bash drone_daasim/cleanup.sh
```

### B) 調査対象（Godot avatar あり）— 一括
```bash
bash drone_daasim/fix_custom_json.sh apply   # custom.json を webavatar 整合（pos=ch1）
bash drone_daasim/run_sequence.sh headless   # cleanup→native→Godot→start→diag
# diag で:
#   native advanceTimeStep 回数 = 0
#   Godot EventStart(推定)      = 0   ← ここが止まっている兆候
$HOME/.pyenv/versions/3.12.3/bin/python drone_daasim/read_pos.py   # pos 読めない＝sim 未歩進
bash drone_daasim/cleanup.sh
```
`window` 指定で非headless（GL描画）も試せる: `bash drone_daasim/run_sequence.sh window`

### B-2) A-2 センサ可視化デモ（LiDAR + Radar を常設で配信）
`hakoniwa-mujoco-sensor` の A-2 センサ（3D LiDAR + Radar）を **毎回のデモ起動で自動配信**し、
Godot に PDU 経由で可視化する。センシング対象（simple_room の env.xml）と表示ワールド
（同 OBB を EnvRoomBuilder で再構築）が一致するので、点群を壁に対して確認できる。
```bash
bash drone_daasim/a2_viz_run.sh window     # 既定=window(GL描画)。CI/無GUIは headless
#   起動: cleanup → native → Godot(HAKO_EXTERNAL_SENSING=1 + HAKO_ENV_OBB=simple_room.obb.json)
#         → hako-cmd start → sensor_bridge_multi
#   配信: Drone/lidar_points(ch16) + Drone/radar_scan→radar_points(ch19)
#   ログ: logs/{drone,godot,a2_bridge}.log
$HOME/.pyenv/versions/3.12.3/bin/python drone_daasim/takeoff.py   # 機体を動かして追従を見る
bash drone_daasim/cleanup.sh
```
上書き可能な環境変数: `A2_ENV`(env.xml) / `A2_OBB`(OBB json) / `A2_MANIFEST`(センサ選択) /
`A2_BRIDGE`(bridge バイナリ) / `A2_SENSOR_HZ`(既定20)。
前提: `sensor_bridge_multi` ビルド済み（`hakoniwa-mujoco-sensor/examples/envsim_sensor_a2/build.bash`）。

### C) 個別に手動起動（端末を分けて観察）
```bash
# 端末1: native
bash drone_daasim/start_drone.sh
# 端末2: Godot（登録ログ "OK: Register on Hakoniwa: GodotAsset" を確認）
bash drone_daasim/start_godot.sh headless
# 端末3: conductor + 診断
bash drone_daasim/conductor.sh start
bash drone_daasim/diag.sh
```

## 調査の着眼点（Start 同期が進まない問題）
1. **native は SYNC で待っているか** … `logs/drone.log` が `SYNC MODE: true` で止まり `advanceTimeStep` が 0 のまま。
2. **Godot は Start を受けているか** … `logs/godot.log` に `EventStart`/`Event Start` が出ない（`Event Initialize` までは出る）。
   - 該当コード: `hakoniwa-godot-drone/Scripts/hakoniwa-sim/sim/HakoAsset.cs`
     - `_PhysicsProcess` → `hakoAsset.Execute()`（isReady 後、1ms 毎）
     - `Scripts/hakoniwa-sim/sim/core/impl/HakoAssetImpl.cs` の `Execute()`/`PollEvent()`/`StartCallback()`（`asset_start_feedback`）
   - **ログ追加の例**（C#。要 `build_godot.sh` 再ビルド）:
     ```csharp
     // HakoAssetImpl.PollEvent() 冒頭
     GD.Print($"[hako] event={ev} t={HakoCppWrapper.asset_get_worldtime?...}");
     // StartCallback() 冒頭/末尾
     GD.Print("[hako] StartCallback BEGIN/END + start_feedback");
     ```
   - `_PhysicsProcess` が呼ばれているか（`accumulatedDelta`/`Execute` 回数）も print すると、
     「Execute は回っているが event=Start が来ない」のか「_PhysicsProcess 自体が回っていない」のか切り分く。
3. **conductor 側の期待アセット数 / タイミング** … native と Godot の登録順、`hako-cmd start` のタイミング。
   `run_sequence.sh` は Godot 登録 OK を待ってから start している。
4. **delta_time 整合** … Godot は `delta_time=1000us`(1ms)。native/conductor の delta と歩進の噛み合わせ。
5. **ライブラリ版差の残り** … `Plugins/Linux/x86_64/libhako_service_c.so` と `/usr/local/hakoniwa` の core/conductor が
   完全に同一ビルドか（登録は通ったが Start プロトコルで差がないか）。

## レーダー構成（radar fit）— DAA シナリオが「何本のレーダーで飛んでいるか」

シナリオ側（`daa_common.scan_best`）は**その機体が積んでいる全レーダー**を読む。
どのレーダーを積んでいるかは決め打ちではなく、ブリッジ自身が配線を解決するのと
**同じ 2 ファイル**から導出する。

| 情報 | 出どころ |
|---|---|
| 何を積んでいるか / どこを向いているか（方位窓・マウント yaw） | A-2 マニフェスト（`config/a2/drone-a2-sensors*.json`） |
| どのチャネルに publish されるか | pdudef（`config2/webavatar-2-radar*.json`） |

`two_drone_run.sh` / `two_drone_viz_run.sh` は起動時に実際に使った組み合わせを
`logs/stack.json` に書く。**シナリオはランチャとは別のシェルで起動されるのが普通**
（`demo_all.sh` がまさにそう）なので、環境変数ではなくこのファイルで受け渡す。

```bash
# 前方 60° 1本（従来どおり）
bash two_drone_run.sh noground && python scenario_s3_overtaking.py
#   -> [fit] Drone1: 1 radar(s): front_radar ch19 az [-30,+30] | 60 deg of 360 covered
#      追い越される側は相手を最後まで見つけられない（後方死角）

# 前方 60° + 後方 150..210° の 2本
A2_DUAL_RADAR=1 A2_MANIFEST=../config/a2/drone-a2-sensors-dual.json \
  bash two_drone_run.sh noground && S3_GAP=3.0 python scenario_s3_overtaking.py
#   -> [fit] Drone1: 2 radar(s): front_radar ch19 + rear_radar ch21 | 120 deg of 360
#      追い越される側が rear_radar で 2.89 m に検知（シナリオ側は無改造）
```

### 仰角の覆域（#7）

fit は方位だけでなく**仰角の窓**も持つ。既定の `vertical_fov_deg: 20.0` は
ボアサイト中心の ±10° であり、距離 r で追える高度差は **r·tan(10°) ≒ 0.18r** しかない。
最終進入中の機体は「近くて下」——まさにこの窓から外れる位置に来る。

`config/a2/drone-a2-sensors-approach.json` は仰角を **-35..+10°（45°）** の
**非対称**な窓に開く。水平線の下に必要な範囲は上より遥かに広いためである。

```bash
# S-5 を既定の 20° 窓で
bash two_drone_run.sh noground && python scenario_s5_landing.py

# 仰角を開いた fit で（シナリオ側は無改造）
A2_MANIFEST=../config/a2/drone-a2-sensors-approach.json \
  bash two_drone_run.sh noground && python scenario_s5_landing.py
```

**窓を広げる代償は 2 つあり、どちらも実測してある**（詳細＝`devai/radar_issue7_*.md`）。

1. **レイ密度の希釈**: `PointsPerScan()` は窓の広さに依存しない。窓を 2.25 倍にすると
   同じレイが 2.25 倍の立体角に散り、**同じ目標に当たる本数が 1/2.25 になる**
   （実測 1751 → 799）。approach マニフェストが `points_per_second` を
   1500 → 3375 にしているのはこのため（実測 1779 ＝ 復元）。
2. **地面クラッタ**: 高度 1.2 m・地面ありの世界で、返り 49 → **237 本/frame**、
   最近傍の地面反射 6.33 m → **1.91 m**。ホバリング中は全て静止（Doppler≈0）なので
   移動目標フィルタで落ちるが、**飛行中は落ちない**——1 m/s で前進すると
   30° 下の地面は 0.87 m/s で近づくため。地面のある世界ではここを承知で使うこと
   （`two_drone_run.sh noground` ＝ S-5 の既定は地面が無いので代償ゼロ）。

### 方位の覆域（#13）

狭い進入幾何では**仰角より方位が先に律速する**。`S5_START=4.0` の S-5 では
初期 4 tick の未検知が全て `azimuth -52..-41 deg outside [-30,+30]` だった。

`config/a2/drone-a2-sensors-approach-wide.json` は #7 の仰角窓に加えて方位を
**150°** に開く。`points_per_second` は**立体角比 (150×45)/(60×20) = 5.625 倍**
（1500 → 8438）。**軸を 2 本広げると代償は足し算ではなく掛け算**になる。

| fit | 窓 | 初探知（`S5_START=4.0`） | 結果 |
|---|---|---|---|
| 既定 | 60×20 | 2.73 m（1回のみ・分類不能） | **FAIL** |
| approach (#7) | 60×45 | 1.99 m | PASS |
| approach-wide (#13) | 150×45 | **6.13 m（初回 tick）** | PASS |

**シナリオ側の角度フィルタが fit より狭いと、広げた覆域を自分で捨てる。**
S-5 の `AZ_HALF=25` がまさにそれで、150° レーダーに対して 40〜50° の交通を
**このファイルが**捨てていた（`AZ_HALF = None` に修正済み）。
`dc.coverage_gap(..., az_half, el_half)` に自前の窓を渡すと、
「**シナリオ自身の窓**で外れた」のか「レーダーの窓で外れた」のかを区別して返す。

### 見えなかった理由の切り分けと計測

`dc.coverage_gap()` は方位・仰角・距離・自前の窓のどれで外れたかを名前で返す。
S-5 は毎 tick `MISS[...]` として記録する。
`python probe_elevation.py [robot] [secs]` は仰角バンド別の返り本数と静止率を出す
読み取り専用プローブ。

マニフェストを書いたら**必ず**次で検算する。窓を広げて `points_per_second` を
上げ忘れた場合に、必要な値を計算して警告する:

```bash
python examples/envsim_sensor_a2/validate_manifest.py config/a2/drone-a2-sensors-*.json
#  front_radar: window 150x45 deg, 844 pts/scan -> 0.1250 pts/deg^2 (1.00x the baseline fit)
#  [WARN] ... is 5.6x THINNER than the baseline -- it will detect less, at every range.
#         raise points_per_second to ~8438 ...
```

上書き用の環境変数（通常は不要）:

- `A2_MANIFEST_<robot>` — 機体ごとのマニフェスト（`A2_MANIFEST` は両機共通）
- `A2_PDUDEF` — チャネル定義（`sensor_bridge_multi` と同じ変数）
- `A2_RADAR_CHANNELS="19,21"` — マニフェストも pdudef も無視してチャネル直指定

**マニフェストに載っていても pdudef にチャネルが無いレーダーは fit から外れる**
（ブリッジが publish しないものを読みに行かないため）。単一レーダー構成では
`scan_best()` は従来の `scan()` と完全に同一の結果を返す。

## 注意
- ⚠️ **起動・片付けの順序（重要・ハマりどころ）**: `hako-cmd stop/status/reset` は
  **マスタ(=native物理サービス)が生きていることが前提**で、マスタ不在で呼ぶと
  応答待ちで**無限ハングする**（cleanup.sh が先頭の `hako-cmd stop` で固まる事象を確認）。
  - 正しい順序: **1) start_drone.sh（`WAIT START` を待つ）→ 2) hako-cmd start →
    3) takeoff.py → 4) 片付けは native が生きているうちに hako-cmd stop → プロセス kill → reset**。
  - native を先に kill すると以後の hako-cmd は全てハングする。`cleanup.sh` は各 hako-cmd を
    `timeout 5` でガード済み（固まっても5秒で抜ける）。`conductor.sh status` も同じ理由で
    マスタ不在だとハングするので注意。
  - 検証実績(2026-06-29, Godotなし正常系): pos (0,0,0)→takeoff後(0,0,**0.467**)→move後(**0.973**,0,0.498) で離陸・移動が通る。
- `start_drone.sh` は **必ず repoルート cwd**（`drone_config` の `paramFilePath: config/controller/param-api-mixer.txt` が cwd 相対）。`cd lnx` から起動すると `Failed to load controller param_file_path` で落ちる。
- `custom.json` は元々 master と pos/motor チャネルがスワップ（pos=ch0）。`fix_custom_json.sh apply` で整合（pos=ch1）。調査後 `restore` で戻せる（backup=`custom.json.bak`）。
- 関連ドキュメント: `devai/drone_core_buildfree_run_20260628.md`, `devai/godot_drone_linux_run_20260628.md`
</content>
