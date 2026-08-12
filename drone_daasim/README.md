# drone_daasim — ドローン DAA（Detect And Avoid）シミュレーション一式

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
