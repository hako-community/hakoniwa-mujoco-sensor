#!/usr/bin/env bash
# B-1: two-drone bring-up with MUTUAL radar detection.
#
# Topology (no Godot -- this verifies the PDU/physics layer):
#   native drone service (2 drones: Drone @ x=-2 facing +x, Drone1 @ x=+2 facing -x)
#     + bridge#1: robot=Drone   , actor_drone2 driven by Drone1's pos  -> Drone/radar_points
#     + bridge#2: robot=Drone1  , actor_drone2 driven by Drone's  pos  -> Drone1/radar_points
#
# Each bridge owns an independent SensorRuntime (its own kinematic MuJoCo world),
# so the two drones see each other by injecting the OTHER one as a free-joint actor.
#
#   Usage: bash two_drone_run.sh [noground|ground|room|crewed]
#          noground (default) : open field, sensing world has NO geometry at all
#                               -> the radar reports only the other aircraft.
#          ground             : open field, the ground slab is sensed too.
#          room               : the walled simple_room (static + dynamic objects,
#                               ISO 15964 4.6) -- used by scenario S-7.
#          crewed             : S-6. Drone's sensing world carries a HELICOPTER-sized
#                               target (2.8 x 2.8 x 1.7 m) while Drone1's still sees a
#                               small UA, so only the aircraft under test gets the big
#                               radar target. See scenario_s6_crewed.py for the sizing.
#          Stop:  bash cleanup.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"

SENSOR_REPO="${SENSOR_REPO:-$(cd "$HERE/.." && pwd)}"
# 環境ジオメトリは env.sh の SENSOR_ENVS（hakoniwa-simenv-data/examples/sensor_envs）で解決する。
# 旧 hakoniwa-envsim-sensor は 2026-08 に廃止。
ENVMODE="${1:-noground}"
case "$ENVMODE" in
  noground) _ENV_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors_nofloor.xml" ;;
  ground)   _ENV_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors.xml" ;;
  room)     _ENV_DEFAULT="$SENSOR_ENVS/simple_room/generated/env_actors.xml" ;;
  crewed)   _ENV_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors_heli.xml"
            _ENV2_DEFAULT="$SENSOR_ENVS/open_field/generated/env_actors_nofloor.xml" ;;
  *) echo "arg must be noground | ground | room (got: $ENVMODE)"; exit 2 ;;
esac
A2_ENV="${A2_ENV:-$_ENV_DEFAULT}"
# The two bridges may sense DIFFERENT worlds: each one owns an independent
# kinematic MuJoCo world, so the target injected into Drone's world does not have
# to be the same object as the one injected into Drone1's.
A2_ENV2="${A2_ENV2:-${_ENV2_DEFAULT:-$A2_ENV}}"
A2_MANIFEST="${A2_MANIFEST:-$SENSOR_REPO/config/a2/drone-a2-sensors.json}"
# Each aircraft may carry a DIFFERENT sensor fit -- e.g. one with a forward
# radar and one with a rear sector (ISO 15964 8: mixing sensors).
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
fi
CONF2="${CONF2:-$HERE/config2/api-2}"
PDUDEF2="${PDUDEF2:-$HERE/config2/webavatar-2-radar.json}"
# #5: the bridges read the channel layout (org_name -> channel_id/pdu_size) out
# of the very pdudef the master is started with, so a sensor is wired up by being
# declared once, here. This used to need A2_PDU_MAP="radar_points_rear=21"
# alongside -- the same channel number written a second time, with nothing to
# catch a disagreement. A2_PDU_MAP still overrides if you need it.
export A2_PDUDEF="$PDUDEF2"
ACTOR_BODY="${A2_ACTOR_BODY:-actor_drone2}"

for f in "$A2_ENV" "$A2_ENV2" "$A2_MANIFEST" "$A2_MANIFEST2" "$BRIDGE" "$CONF2" "$PDUDEF2"; do
  [ -e "$f" ] || { echo "MISSING: $f"; exit 2; }
done

# --- 0) clean slate. The pdudef changes channel layout, so the mmap images MUST
#        be dropped or the master will keep the old 1-drone layout.
_say "== cleanup =="; timeout 20 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true
rm -f /var/lib/hakoniwa/mmap/*.bin 2>/dev/null || sudo rm -f /var/lib/hakoniwa/mmap/*.bin 2>/dev/null || true

# --- 1) native physics with the 2-drone config ------------------------------
cd "$DRONE_CORE"
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/drone.log"
nohup ./lnx/linux-main_hako_drone_service "$CONF2" "$PDUDEF2" > "$LOG_DIR/drone.log" 2>&1 &
DRONE_PID=$!; _say "native pid=$DRONE_PID conf=$CONF2"
for _ in $(seq 1 60); do grep -q "WAIT START" "$LOG_DIR/drone.log" 2>/dev/null && break; sleep 0.5; done
grep -q "WAIT START" "$LOG_DIR/drone.log" \
  && _say "native: WAIT START OK" || { _say "native が WAIT START 未到達"; tail -12 "$LOG_DIR/drone.log"; exit 1; }
grep -cE "Drone1|Drone" "$LOG_DIR/drone.log" >/dev/null && _say "$(grep -m2 -E 'drone_config|Drone1' "$LOG_DIR/drone.log" | tail -1)"

# --- 2) conductor start ------------------------------------------------------
_say "== conductor start =="; "$HAKO_CMD" start >/dev/null 2>&1
until grep -q "PDU CREATED" "$LOG_DIR/drone.log" 2>/dev/null; do sleep 0.2; done
sleep 0.5; _say "PDU CREATED"

# --- 3) two bridges, each injecting the OTHER drone as its actor -------------
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
export HAKO_BINARY_PATH="$HAKO_BINARY_PATH"
: > "$LOG_DIR/b1_drone.log"; : > "$LOG_DIR/b1_drone1.log"
nohup stdbuf -oL -eL "$BRIDGE" "$A2_ENV" "$A2_MANIFEST" Drone  1 72 16 19 177424 "$SENSOR_HZ" \
  "$ACTOR_BODY" Drone1 1 > "$LOG_DIR/b1_drone.log" 2>&1 &
B1=$!
nohup stdbuf -oL -eL "$BRIDGE" "$A2_ENV2" "$A2_MANIFEST2" Drone1 1 72 16 19 177424 "$SENSOR_HZ" \
  "$ACTOR_BODY" Drone  1 > "$LOG_DIR/b1_drone1.log" 2>&1 &
B2=$!
# PID files so a scenario can inject a sensor fault into ONE bridge (S-8).
echo "$B1" > "$LOG_DIR/bridge_Drone.pid"; echo "$B2" > "$LOG_DIR/bridge_Drone1.pid"

# #6: record what each aircraft was actually launched with, so a scenario can
# discover its RADAR FIT -- the manifest says what is fitted and where it looks,
# the pdudef says which channel each radar publishes on. A scenario is normally
# started from a different shell than this launcher (demo_all.sh does exactly
# that), so exported variables would not reach it; a file that outlives the
# launcher does. Env vars still override, for a one-off run.
cat > "$LOG_DIR/stack.json" <<EOF
{
  "pdudef": "$PDUDEF2",
  "manifests": { "Drone": "$A2_MANIFEST", "Drone1": "$A2_MANIFEST2" }
}
EOF
_say "bridges: Drone pid=$B1 / Drone1 pid=$B2 (env=$ENVMODE: $(basename "$A2_ENV") / $(basename "$A2_ENV2"), manifest=$(basename "$A2_MANIFEST")/$(basename "$A2_MANIFEST2"))"

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
_say "起動完了。native=$DRONE_PID bridge(Drone)=$B1 bridge(Drone1)=$B2"
_say "  回避   : $PYENV_PY $HERE/two_drone_avoid.py   (2機正面衝突回避: 相互検知→各機右へ回避→ゴール, RESULT:PASS)"
_say "  検証   : $PYENV_PY $HERE/scenario_b1_faceoff.py   (2機を対面させ相互検知＋接近Doppler)"
_say "  (旧)   : $PYENV_PY $HERE/verify_b1.py   ※yaw未指定だと両機+x向きで後方機しか相手を見ない"
_say "  片付け : bash $HERE/cleanup.sh"
