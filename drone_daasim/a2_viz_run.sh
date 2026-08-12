#!/usr/bin/env bash
# A-2 sensor visualization demo launcher (standing LiDAR + Radar delivery).
#
# Wires the multi-sensor bridge into the *standard* A-2 GUI demo, so both the
# LiDAR (lidar_points/ch16) and Radar (radar_scan->ch19 = Godot radar_points)
# point clouds are delivered to Godot over PDU on every launch. This is the
# permanent form of Phase 1 remaining item #3.
#
# Bring-up order (see [[hako-cmd-master-ordering]]):
#   cleanup -> native physics (master+conductor) -> WAIT START
#           -> Godot avatar (HAKO_EXTERNAL_SENSING=1 + HAKO_ENV_OBB) -> Register
#           -> hako-cmd start -> PDU CREATED
#           -> sensor_bridge_multi (external client, attaches AFTER start)
#
# Scene alignment: the bridge scans simple_room's generated env.xml while Godot
# rebuilds the SAME room from simple_room.obb.json (EnvRoomBuilder). Sensing
# world == displayed world, so point clouds land on the visible walls/pillar.
#
#   Usage: bash a2_viz_run.sh [window|headless]     (default: window)
#          Stop: bash cleanup.sh   (kills native/Godot/bridge + SHM reset)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
MODE="${1:-window}"

# --- paths (override via env) ---------------------------------------------
SENSOR_REPO="${SENSOR_REPO:-$(cd "$DRONE_CORE/../hakoniwa-mujoco-sensor" && pwd)}"
# 環境ジオメトリは env.sh の SENSOR_ENVS（hakoniwa-simenv-data/examples/sensor_envs）で解決する。
# 旧 hakoniwa-envsim-sensor は 2026-08 に廃止。
A2_ENV="${A2_ENV:-$SENSOR_ENVS/simple_room/generated/env.xml}"
A2_OBB="${A2_OBB:-$SENSOR_ENVS/simple_room/simple_room.obb.json}"
A2_MANIFEST="${A2_MANIFEST:-$SENSOR_REPO/config/a2/drone-a2-sensors.json}"
BRIDGE="${A2_BRIDGE:-$SENSOR_REPO/examples/envsim_sensor_a2/sensor_bridge_multi}"
SENSOR_HZ="${A2_SENSOR_HZ:-20}"

for f in "$A2_ENV" "$A2_OBB" "$A2_MANIFEST" "$BRIDGE"; do
  [ -e "$f" ] || { echo "MISSING: $f"; exit 2; }
done
[ -x "$BRIDGE" ] || { echo "not executable: $BRIDGE (build via examples/envsim_sensor_a2/build.bash)"; exit 2; }

# --- 0) clean slate --------------------------------------------------------
_say "== cleanup =="; timeout 20 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true

# --- 1) native physics -----------------------------------------------------
cd "$DRONE_CORE"
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/drone.log"
nohup ./lnx/linux-main_hako_drone_service "$DRONE_CONF" "$PDUDEF" > "$LOG_DIR/drone.log" 2>&1 &
DRONE_PID=$!; _say "native pid=$DRONE_PID (log: logs/drone.log)"
_say "== WAIT START 待ち =="
for _ in $(seq 1 60); do grep -q "WAIT START" "$LOG_DIR/drone.log" 2>/dev/null && break; sleep 0.5; done
grep -q "WAIT START" "$LOG_DIR/drone.log" \
  && _say "native: WAIT START OK" || { _say "native が WAIT START 未到達"; tail -5 "$LOG_DIR/drone.log"; exit 1; }

# --- 2) Godot avatar (external sensing + env geometry) ---------------------
cd "$GODOT_DRONE"
export PATH="$DOTNET_ROOT:$PATH"
export LD_LIBRARY_PATH="$GODOT_DRONE/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/godot.log"
GARGS=(--path "$GODOT_DRONE")
[ "$MODE" = headless ] && GARGS+=(--headless) || GARGS+=(--rendering-method gl_compatibility)
HAKO_EXTERNAL_SENSING=1 HAKO_ENV_OBB="$A2_OBB" \
  nohup "$GODOT_MONO" "${GARGS[@]}" > "$LOG_DIR/godot.log" 2>&1 &
GODOT_PID=$!; _say "godot pid=$GODOT_PID mode=$MODE (external sensing + env OBB)"
_say "== Godot 登録待ち =="
for _ in $(seq 1 80); do
  grep -q "OK: Register on Hakoniwa" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register OK"; break; }
  grep -qE "Can not register|Can not declare pdu" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register FAIL"; tail -8 "$LOG_DIR/godot.log"; break; }
  kill -0 "$GODOT_PID" 2>/dev/null || { _say "Godot exited"; tail -8 "$LOG_DIR/godot.log"; break; }
  sleep 0.5
done
grep -q "EnvRoom" "$LOG_DIR/godot.log" 2>/dev/null && _say "Godot: $(grep -m1 EnvRoom "$LOG_DIR/godot.log")"

# --- 3) conductor start ----------------------------------------------------
_say "== conductor start =="; "$HAKO_CMD" start >/dev/null 2>&1
until grep -q "PDU CREATED" "$LOG_DIR/drone.log" 2>/dev/null; do sleep 0.2; done
sleep 0.5; _say "PDU CREATED"

# --- 4) sensor_bridge_multi (external, AFTER start) ------------------------
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
export HAKO_BINARY_PATH="$HAKO_BINARY_PATH"
: > "$LOG_DIR/a2_bridge.log"
# stdbuf -oL: line-buffer the bridge's stdout so the readiness grep below is not
# racing C's default 4KB block buffering (which delays/interleaves log writes).
nohup stdbuf -oL -eL "$BRIDGE" "$A2_ENV" "$A2_MANIFEST" Drone 1 72 16 19 177424 "$SENSOR_HZ" \
  > "$LOG_DIR/a2_bridge.log" 2>&1 &
BRIDGE_PID=$!; _say "sensor_bridge_multi pid=$BRIDGE_PID (hz=$SENSOR_HZ, lidar->ch16 radar->ch19)"
_say "== 初回配信待ち =="
got_l=0; got_r=0
for _ in $(seq 1 60); do
  grep -q -- "-> ch16" "$LOG_DIR/a2_bridge.log" 2>/dev/null && got_l=1
  grep -q -- "-> ch19" "$LOG_DIR/a2_bridge.log" 2>/dev/null && got_r=1
  { [ "$got_l" = 1 ] && [ "$got_r" = 1 ]; } && break
  kill -0 "$BRIDGE_PID" 2>/dev/null || { _say "bridge exited"; tail -8 "$LOG_DIR/a2_bridge.log"; break; }
  sleep 0.5
done
[ "$got_l" = 1 ] && _say "LiDAR publishing OK (ch16)"  || _say "WARN: LiDAR not publishing (see logs/a2_bridge.log)"
[ "$got_r" = 1 ] && _say "Radar publishing OK (ch19)"  || _say "WARN: Radar not publishing (see logs/a2_bridge.log)"

echo
_say "起動完了。native=$DRONE_PID godot=$GODOT_PID bridge=$BRIDGE_PID"
_say "  機体を動かす : $PYENV_PY $HERE/takeoff.py"
_say "  ブリッジlog  : tail -f $LOG_DIR/a2_bridge.log"
_say "  Godot log    : tail -f $LOG_DIR/godot.log"
_say "  片付け       : bash $HERE/cleanup.sh"
