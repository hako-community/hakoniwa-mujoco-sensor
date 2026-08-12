#!/usr/bin/env bash
# M6 launcher: clean end-to-end orchestration for the detection->avoidance demo.
#   cleanup -> native drone physics -> hako-cmd start -> A-2 sensor bridge ->
#   control client (passed as args; default = m6_smoke2 read check).
# Order matters (see [[hako-cmd-master-ordering]]):
#   - bridge attaches AFTER hako-cmd start (PDU data must exist or pdu_read segfaults)
#   - cleanup runs while native is alive
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"

SENSOR_REPO="${SENSOR_REPO:-$(cd "$DRONE_CORE/../hakoniwa-mujoco-sensor" && pwd)}"
ENVXML="${M6_ENV:-$SENSOR_REPO/examples/envsim_sensor_a2/m6/env.xml}"
DEMO="${M6_DEMO:-$SENSOR_REPO/examples/envsim_sensor_a2/lidar3d_a2_pdu}"
SENSOR_HZ="${M6_SENSOR_HZ:-3}"
CFG="$DRONE_CORE/$PDUDEF"
BRIDGE_PID=""; DRONE_PID=""

cleanup() {
  _say "M6 cleanup"
  [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null || true
  timeout 20 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# 0) clean slate
timeout 20 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true

# 1) native physics (repo root cwd)
cd "$DRONE_CORE"
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
./lnx/linux-main_hako_drone_service "$DRONE_CONF" "$PDUDEF" > "$LOG_DIR/drone.log" 2>&1 &
DRONE_PID=$!
_say "native pid=$DRONE_PID, waiting WAIT START..."
until grep -q "WAIT START" "$LOG_DIR/drone.log" 2>/dev/null; do sleep 0.2; done

# 2) conductor start, wait for PDU creation
"$HAKO_CMD" start >/dev/null 2>&1
_say "conductor started, waiting PDU CREATED..."
until grep -q "PDU CREATED" "$LOG_DIR/drone.log" 2>/dev/null; do sleep 0.2; done
sleep 0.5

# 3) A-2 sensor bridge (external, AFTER start)
HAKO_BINARY_PATH="$HAKO_BINARY_PATH" "$PYENV_PY" "$HERE/m6_sensor_bridge.py" \
    "$ENVXML" "$DEMO" "$SENSOR_HZ" > "$LOG_DIR/m6_bridge.log" 2>&1 &
BRIDGE_PID=$!
_say "sensor bridge pid=$BRIDGE_PID (hz=$SENSOR_HZ), waiting first publish..."
for _ in $(seq 1 50); do grep -q "lidar_points" "$LOG_DIR/m6_bridge.log" 2>/dev/null && break; sleep 0.2; done
grep -q "lidar_points" "$LOG_DIR/m6_bridge.log" 2>/dev/null \
  && _say "sensor publishing OK" || _say "WARN: sensor not publishing (see $LOG_DIR/m6_bridge.log)"

# 4) control client
if [ "$#" -gt 0 ]; then
  _say "control: $*"
  "$PYENV_PY" "$@" "$CFG"
else
  _say "control: m6_smoke2 (read check)"
  "$PYENV_PY" /tmp/claude-1000/-data-buildman-drone/16ab1ac4-3cbd-44fe-adcc-04637820f859/scratchpad/m6_smoke2.py "$CFG"
fi
rc=$?
_say "control exited rc=$rc"
exit $rc
