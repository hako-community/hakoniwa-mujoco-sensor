#!/usr/bin/env bash
# Sensor visualization demo launcher (§18 Stage V) -- uses the DEDICATED scene
# `Scenes/sensor_viz.tscn`. `drone_1.tscn` is never touched or launched here.
#
# Difference from a2_viz_run.sh:
#   - Godot is started with an explicit scene path (res://Scenes/sensor_viz.tscn)
#   - HAKO_SENSOR_MANIFEST : the A-2 manifest is the single source of truth for
#                            the drawn FOV / Range / mount (no hard-coded values)
#   - HAKO_VIZ_MODE        : radar | lidar | none  (L / R / N keys at runtime)
#                            LiDAR と Radar は同時に出さない（片方ずつ）
#   - HAKO_VIZ_CAM         : scene | top | oblique  (C key cycles at runtime)
#
#   Usage: bash sensor_viz_run.sh [window|headless] [radar|lidar|none] [scene|top|oblique] [zoom]
#          Stop: bash cleanup.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
MODE="${1:-window}"
VIZ="${2:-${HAKO_VIZ_MODE:-radar}}"
CAM="${3:-${HAKO_VIZ_CAM:-top}}"
CAMZOOM="${4:-${HAKO_VIZ_CAM_ZOOM:-1.0}}"

SENSOR_REPO="${SENSOR_REPO:-$(cd "$HERE/.." && pwd)}"
# 環境ジオメトリは env.sh の SENSOR_ENVS（hakoniwa-simenv-data/examples/sensor_envs）で解決する。
# 旧 hakoniwa-envsim-sensor は 2026-08 に廃止。
A2_ENV="${A2_ENV:-$SENSOR_ENVS/simple_room/generated/env_actors.xml}"
A2_OBB="${A2_OBB:-$SENSOR_ENVS/simple_room/simple_room.obb.json}"
A2_MANIFEST="${A2_MANIFEST:-$SENSOR_REPO/config/a2/drone-a2-sensors.json}"
BRIDGE="${A2_BRIDGE:-$SENSOR_REPO/examples/envsim_sensor_a2/sensor_bridge_multi}"
SENSOR_HZ="${A2_SENSOR_HZ:-20}"
# A-1: free-joint actor injected into the sensing world.
#   A2_ACTOR_ROBOT=demo   -> scripted head-on motion (1機での検証用)
#   A2_ACTOR_ROBOT=Drone2 -> 2機目の pos で駆動（Stage B）
ACTOR_BODY="${A2_ACTOR_BODY:-actor_drone2}"
ACTOR_ROBOT="${A2_ACTOR_ROBOT:-demo}"
SCENE="${HAKO_VIZ_SCENE:-res://Scenes/sensor_viz.tscn}"

for f in "$A2_ENV" "$A2_OBB" "$A2_MANIFEST" "$BRIDGE"; do
  [ -e "$f" ] || { echo "MISSING: $f"; exit 2; }
done
[ -x "$BRIDGE" ] || { echo "not executable: $BRIDGE"; exit 2; }
[ -e "$GODOT_DRONE/Scenes/sensor_viz.tscn" ] || { echo "MISSING: $GODOT_DRONE/Scenes/sensor_viz.tscn"; exit 2; }

# --- 0) clean slate --------------------------------------------------------
_say "== cleanup =="; timeout 20 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true

# --- 1) native physics -----------------------------------------------------
cd "$DRONE_CORE"
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/drone.log"
nohup ./lnx/linux-main_hako_drone_service "$DRONE_CONF" "$PDUDEF" > "$LOG_DIR/drone.log" 2>&1 &
DRONE_PID=$!; _say "native pid=$DRONE_PID"
for _ in $(seq 1 60); do grep -q "WAIT START" "$LOG_DIR/drone.log" 2>/dev/null && break; sleep 0.5; done
grep -q "WAIT START" "$LOG_DIR/drone.log" \
  && _say "native: WAIT START OK" || { _say "native が WAIT START 未到達"; tail -5 "$LOG_DIR/drone.log"; exit 1; }

# --- 2) Godot: sensor_viz scene --------------------------------------------
cd "$GODOT_DRONE"
export PATH="$DOTNET_ROOT:$PATH"
export LD_LIBRARY_PATH="$GODOT_DRONE/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/godot.log"
GARGS=(--path "$GODOT_DRONE")
[ "$MODE" = headless ] && GARGS+=(--headless) || GARGS+=(--rendering-method gl_compatibility)
[ "${HAKO_VIZ_FULLSCREEN:-0}" = "1" ] && GARGS+=(--fullscreen)
GARGS+=("$SCENE")
HAKO_EXTERNAL_SENSING=1 HAKO_ENV_OBB="$A2_OBB" \
HAKO_SENSOR_MANIFEST="$A2_MANIFEST" HAKO_VIZ_MODE="$VIZ" HAKO_VIZ_CAM="$CAM" HAKO_VIZ_CAM_ZOOM="$CAMZOOM" \
  nohup "$GODOT_MONO" "${GARGS[@]}" > "$LOG_DIR/godot.log" 2>&1 &
GODOT_PID=$!; _say "godot pid=$GODOT_PID scene=$SCENE viz_mode=$VIZ cam=$CAM"
for _ in $(seq 1 80); do
  grep -q "OK: Register on Hakoniwa" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register OK"; break; }
  grep -qE "Can not register|Can not declare pdu" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register FAIL"; tail -8 "$LOG_DIR/godot.log"; break; }
  kill -0 "$GODOT_PID" 2>/dev/null || { _say "Godot exited"; tail -12 "$LOG_DIR/godot.log"; break; }
  sleep 0.5
done
grep -q "\[SensorViz\]" "$LOG_DIR/godot.log" 2>/dev/null && _say "$(grep -m1 '\[SensorViz\]' "$LOG_DIR/godot.log")"

# --- 3) conductor start ----------------------------------------------------
_say "== conductor start =="; "$HAKO_CMD" start >/dev/null 2>&1
until grep -q "PDU CREATED" "$LOG_DIR/drone.log" 2>/dev/null; do sleep 0.2; done
sleep 0.5; _say "PDU CREATED"

# --- 4) sensor_bridge_multi (external, AFTER start) ------------------------
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
export HAKO_BINARY_PATH="$HAKO_BINARY_PATH"
: > "$LOG_DIR/a2_bridge.log"
nohup stdbuf -oL -eL "$BRIDGE" "$A2_ENV" "$A2_MANIFEST" Drone 1 72 16 19 177424 "$SENSOR_HZ" \
  "$ACTOR_BODY" "$ACTOR_ROBOT" 1 \
  > "$LOG_DIR/a2_bridge.log" 2>&1 &
BRIDGE_PID=$!; _say "sensor_bridge_multi pid=$BRIDGE_PID"
got_l=0; got_r=0
for _ in $(seq 1 60); do
  grep -q -- "-> ch16" "$LOG_DIR/a2_bridge.log" 2>/dev/null && got_l=1
  grep -q -- "-> ch19" "$LOG_DIR/a2_bridge.log" 2>/dev/null && got_r=1
  { [ "$got_l" = 1 ] && [ "$got_r" = 1 ]; } && break
  kill -0 "$BRIDGE_PID" 2>/dev/null || { _say "bridge exited"; tail -8 "$LOG_DIR/a2_bridge.log"; break; }
  sleep 0.5
done
[ "$got_l" = 1 ] && _say "LiDAR publishing OK (ch16)"  || _say "WARN: LiDAR not publishing"
[ "$got_r" = 1 ] && _say "Radar publishing OK (ch19)"  || _say "WARN: Radar not publishing"

echo
_say "起動完了。native=$DRONE_PID godot=$GODOT_PID bridge=$BRIDGE_PID"
_say "  表示切替 : L=LiDARのみ / R=Radarのみ / N=なし   C=カメラ切替(scene→top→oblique)"
_say "  機体を動かす : $PYENV_PY $HERE/takeoff.py $DRONE_CORE/config/pdudef/webavatar.json"
_say "  片付け   : bash $HERE/cleanup.sh"
