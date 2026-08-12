#!/usr/bin/env bash
# 2-drone COLLISION AVOIDANCE display -- dedicated scene `Scenes/two_drone_avoid.tscn`.
#
# Combines two_drone_run.sh (2-drone native + 2 bridges, each injecting the OTHER
# drone as a free-joint actor) with a Godot scene built for this demo only.
# Godot runs as ONE asset that reads BOTH Drone and Drone1 (the scene's
# TwoDroneBootstrap has AlwaysEnabled=true and clones the avatar; HAKO_PDU_CONFIG
# points at a 2-robot channel def).
#
# How this differs from sensor_viz_run.sh (the 1-drone radar SCAN scene):
#   - scene : two_drone_avoid.tscn        (sensor_viz.tscn is 1 drone, never cloned)
#   - env   : open_field, NO WALLS        (Godot draws a 40x40 ground slab; the
#             sensing world has no ground at all, so the radar reports only the
#             other drone). The slab is the placeholder for the real map tiles
#             (Cesium) planned as the next step.
#   - tags  : the floating 3D detection labels are off (ShowDetectionLabels=false);
#             ranges/Doppler are still read from the HUD.
#
#   Usage: bash two_drone_viz_run.sh [window|headless] [radar|lidar|none] \
#                [scene|top|oblique] [zoom] [noground|ground|room|crewed]
#          noground (default) : the sensing world has no ground -> the radar reports
#                               ONLY the other aircraft (clean avoidance demo)
#          ground             : the ground slab is sensed too -> realistic static
#                               clutter (every return at 0.0 m/s is the ground)
#          room               : the walled simple_room -> static + dynamic objects (S-7)
#          crewed             : helicopter-sized target in Drone's sensing world (S-6)
#          Sensor fit comes from the manifests, and each aircraft may have its own:
#            A2_MANIFEST   (Drone)   A2_MANIFEST2 (Drone1, defaults to A2_MANIFEST)
#          Both are handed to Godot as HAKO_SENSOR_MANIFEST / HAKO_SENSOR_MANIFEST2 so
#          the drawn window matches what each aircraft actually carries.
#          Move : $PYENV_PY drone_daasim/scenario_b1_faceoff.py     (face-off + approach)
#          Stop : bash cleanup.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
MODE="${1:-window}"
VIZ="${2:-radar}"
CAM="${3:-oblique}"
CAMZOOM="${4:-1.0}"
GROUND="${5:-${A2_GROUND:-noground}}"

SENSOR_REPO="${SENSOR_REPO:-$(cd "$HERE/.." && pwd)}"
# 環境ジオメトリは env.sh の SENSOR_ENVS（hakoniwa-simenv-data/examples/sensor_envs）で解決する。
# 旧 hakoniwa-envsim-sensor は 2026-08 に廃止。
# Sensing world. Godot always draws the ground slab from A2_OBB; what changes here
# is whether the SENSOR world contains it (ground reflections on/off).
case "$GROUND" in
  ground)   _ENV_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors.xml" ;;
  noground) _ENV_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors_nofloor.xml" ;;
  room)     _ENV_DEFAULT="$SENSOR_ENVS/simple_room/generated/env_actors.xml"
            _OBB_DEFAULT="$SENSOR_ENVS/simple_room/simple_room.obb.json" ;;
  crewed)   _ENV_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors_heli.xml"
            _ENV2_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors_nofloor.xml" ;;
  *) echo "5th arg must be noground | ground | room | crewed (got: $GROUND)"; exit 2 ;;
esac
A2_ENV="${A2_ENV:-$_ENV_DEFAULT}"
# The two bridges may sense different worlds (see two_drone_run.sh).
A2_ENV2="${A2_ENV2:-${_ENV2_DEFAULT:-$A2_ENV}}"
A2_OBB="${A2_OBB:-${_OBB_DEFAULT:-$SENSOR_ENVS/open_field/open_field.obb.json}}"
A2_MANIFEST="${A2_MANIFEST:-$SENSOR_REPO/config/a2/drone-a2-sensors.json}"
A2_MANIFEST2="${A2_MANIFEST2:-$A2_MANIFEST}"
BRIDGE="${A2_BRIDGE:-$SENSOR_REPO/examples/envsim_sensor_a2/sensor_bridge_multi}"
SENSOR_HZ="${A2_SENSOR_HZ:-20}"
# Dual-radar option. OFF by default: the standard configs are untouched, so the
# single-radar path runs on exactly the same files it always did. A2_DUAL_RADAR=1
# swaps in the pdudef/channel-config that carry the extra channel and tells the
# bridges where to publish the second radar.
if [ "${A2_DUAL_RADAR:-0}" = "1" ]; then
  PDUDEF2="${PDUDEF2:-$HERE/config2/webavatar-2-radar2.json}"
  PDU_CONFIG2="${PDU_CONFIG2:-$HERE/config2/avatar-drone-2-radar2.json}"
  export A2_PDU_MAP="${A2_PDU_MAP:-radar_points_rear=21}"
fi
CONF2="${CONF2:-$HERE/config2/api-2}"
PDUDEF2="${PDUDEF2:-$HERE/config2/webavatar-2-radar.json}"
PDU_CONFIG2="${PDU_CONFIG2:-$HERE/config2/avatar-drone-2.json}"
ACTOR_BODY="${A2_ACTOR_BODY:-actor_drone2}"
SCENE="${HAKO_VIZ_SCENE:-res://Scenes/two_drone_avoid.tscn}"

for f in "$A2_ENV" "$A2_ENV2" "$A2_OBB" "$A2_MANIFEST" "$A2_MANIFEST2" "$BRIDGE" "$CONF2" "$PDUDEF2" "$PDU_CONFIG2"; do
  [ -e "$f" ] || { echo "MISSING: $f"; exit 2; }
done
[ -e "$GODOT_DRONE/Scenes/two_drone_avoid.tscn" ] || { echo "MISSING two_drone_avoid.tscn"; exit 2; }

# --- 0) clean slate. pdudef changes channel layout -> drop mmap images. -----
_say "== cleanup =="; timeout 20 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true
rm -f /var/lib/hakoniwa/mmap/*.bin 2>/dev/null || sudo rm -f /var/lib/hakoniwa/mmap/*.bin 2>/dev/null || true

# --- 1) native physics (2-drone config) ------------------------------------
cd "$DRONE_CORE"
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/drone.log"
nohup ./lnx/linux-main_hako_drone_service "$CONF2" "$PDUDEF2" > "$LOG_DIR/drone.log" 2>&1 &
DRONE_PID=$!; _say "native pid=$DRONE_PID conf=$CONF2"
for _ in $(seq 1 60); do grep -q "WAIT START" "$LOG_DIR/drone.log" 2>/dev/null && break; sleep 0.5; done
grep -q "WAIT START" "$LOG_DIR/drone.log" \
  && _say "native: WAIT START OK" || { _say "native が WAIT START 未到達"; tail -8 "$LOG_DIR/drone.log"; exit 1; }

# --- 2) Godot: two_drone_avoid scene (TWO drones, walls-free env) ----------
cd "$GODOT_DRONE"
export PATH="$DOTNET_ROOT:$PATH"
export LD_LIBRARY_PATH="$GODOT_DRONE/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/godot.log"
GARGS=(--path "$GODOT_DRONE")
[ "$MODE" = headless ] && GARGS+=(--headless) || GARGS+=(--rendering-method gl_compatibility)
[ "${HAKO_VIZ_FULLSCREEN:-0}" = "1" ] && GARGS+=(--fullscreen)
GARGS+=("$SCENE")
HAKO_EXTERNAL_SENSING=1 HAKO_ENV_OBB="$A2_OBB" HAKO_PDU_CONFIG="$PDU_CONFIG2" \
HAKO_SENSOR_MANIFEST="$A2_MANIFEST" HAKO_SENSOR_MANIFEST2="$A2_MANIFEST2" HAKO_VIZ_MODE="$VIZ" HAKO_VIZ_CAM="$CAM" HAKO_VIZ_CAM_ZOOM="$CAMZOOM" \
  nohup "$GODOT_MONO" "${GARGS[@]}" > "$LOG_DIR/godot.log" 2>&1 &
GODOT_PID=$!; _say "godot pid=$GODOT_PID scene=$SCENE viz=$VIZ cam=$CAM (env=$GROUND, manifest=$(basename "$A2_MANIFEST")/$(basename "$A2_MANIFEST2"))"
for _ in $(seq 1 80); do
  grep -q "OK: Register on Hakoniwa" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register OK"; break; }
  grep -qE "Can not register|Can not declare pdu|Invalid HakoAssets" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register FAIL"; tail -12 "$LOG_DIR/godot.log"; break; }
  kill -0 "$GODOT_PID" 2>/dev/null || { _say "Godot exited"; tail -15 "$LOG_DIR/godot.log"; break; }
  sleep 0.5
done
grep -q "\[TwoDrone\]" "$LOG_DIR/godot.log" 2>/dev/null && _say "$(grep -m1 '\[TwoDrone\]' "$LOG_DIR/godot.log")"
grep -c "\[SensorViz\] initialized" "$LOG_DIR/godot.log" 2>/dev/null | grep -q '^2$' && _say "SensorViz: 2 rigs initialized"

# --- 3) conductor start ----------------------------------------------------
_say "== conductor start =="; "$HAKO_CMD" start >/dev/null 2>&1
until grep -q "PDU CREATED" "$LOG_DIR/drone.log" 2>/dev/null; do sleep 0.2; done
sleep 0.5; _say "PDU CREATED"

# --- 4) two bridges, each injecting the OTHER drone as its actor -----------
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
export HAKO_BINARY_PATH="$HAKO_BINARY_PATH"
: > "$LOG_DIR/b1_drone.log"; : > "$LOG_DIR/b1_drone1.log"
nohup stdbuf -oL -eL "$BRIDGE" "$A2_ENV" "$A2_MANIFEST" Drone  1 72 16 19 177424 "$SENSOR_HZ" \
  "$ACTOR_BODY" Drone1 1 > "$LOG_DIR/b1_drone.log" 2>&1 &
B1=$!
nohup stdbuf -oL -eL "$BRIDGE" "$A2_ENV2" "$A2_MANIFEST2" Drone1 1 72 16 19 177424 "$SENSOR_HZ" \
  "$ACTOR_BODY" Drone  1 > "$LOG_DIR/b1_drone1.log" 2>&1 &
B2=$!
echo "$B1" > "$LOG_DIR/bridge_Drone.pid"; echo "$B2" > "$LOG_DIR/bridge_Drone1.pid"
_say "bridges: Drone pid=$B1 / Drone1 pid=$B2"
ok1=0; ok2=0
for _ in $(seq 1 60); do
  grep -q -- "-> ch19" "$LOG_DIR/b1_drone.log"  2>/dev/null && ok1=1
  grep -q -- "-> ch19" "$LOG_DIR/b1_drone1.log" 2>/dev/null && ok2=1
  { [ "$ok1" = 1 ] && [ "$ok2" = 1 ]; } && break
  sleep 0.5
done
[ "$ok1" = 1 ] && _say "Drone  radar publishing OK (ch19)" || { _say "WARN: Drone radar 配信なし";  tail -5 "$LOG_DIR/b1_drone.log"; }
[ "$ok2" = 1 ] && _say "Drone1 radar publishing OK (ch19)" || { _say "WARN: Drone1 radar 配信なし"; tail -5 "$LOG_DIR/b1_drone1.log"; }

echo
_say "起動完了。native=$DRONE_PID godot=$GODOT_PID bridge=$B1/$B2"
_say "  衝突回避 : $PYENV_PY $HERE/two_drone_avoid.py   (2機正面→相互検知→各機右へ回避→ゴール)"
_say "  対面+接近 : $PYENV_PY $HERE/scenario_b1_faceoff.py"
_say "  表示切替  : L=LiDAR / R=Radar / N=なし   C=カメラ切替   +/- ズーム"
_say "  片付け    : bash $HERE/cleanup.sh"
