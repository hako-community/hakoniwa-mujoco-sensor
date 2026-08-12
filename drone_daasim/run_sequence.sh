#!/usr/bin/env bash
# 一括再現: cleanup → native(bg) → WAIT START → Godot(bg) → 登録待ち → conductor start → diag。
# プロセスは起動したまま残す（調査用）。終了は drone_daasim/cleanup.sh。
#   引数: headless（既定）| window   （Godot の起動モード）
#   例:   bash drone_daasim/run_sequence.sh window
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
MODE="${1:-headless}"

_say "== cleanup =="; bash "$DAASIM_DIR/cleanup.sh" >/dev/null 2>&1

# 1) native drone（repoルートで）
cd "$DRONE_CORE"
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/drone.log"
nohup ./lnx/linux-main_hako_drone_service "$DRONE_CONF" "$PDUDEF" > "$LOG_DIR/drone.log" 2>&1 &
DRONE_PID=$!; _say "native pid=$DRONE_PID (log: logs/drone.log)"

_say "== WAIT START 待ち =="
for _ in $(seq 1 60); do grep -q "WAIT START" "$LOG_DIR/drone.log" 2>/dev/null && break; sleep 0.5; done
grep -q "WAIT START" "$LOG_DIR/drone.log" && _say "native: WAIT START OK" || { _say "native が WAIT START に到達せず"; tail -5 "$LOG_DIR/drone.log"; }

# 2) Godot avatar（cwd=GODOT_DRONE で ./custom.json を読む）
cd "$GODOT_DRONE"
export PATH="$DOTNET_ROOT:$PATH"
export LD_LIBRARY_PATH="$GODOT_DRONE/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"
: > "$LOG_DIR/godot.log"
GARGS=(--path "$GODOT_DRONE"); [ "$MODE" = headless ] && GARGS+=(--headless) || GARGS+=(--rendering-method gl_compatibility)
nohup "$GODOT_MONO" "${GARGS[@]}" > "$LOG_DIR/godot.log" 2>&1 &
GODOT_PID=$!; _say "godot pid=$GODOT_PID mode=$MODE (log: logs/godot.log)"

_say "== Godot 登録待ち =="
for _ in $(seq 1 80); do
  grep -q "OK: Register on Hakoniwa" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register OK"; break; }
  grep -qE "Can not register|Can not declare pdu" "$LOG_DIR/godot.log" 2>/dev/null && { _say "Godot: Register FAIL"; break; }
  kill -0 "$GODOT_PID" 2>/dev/null || { _say "Godot exited"; break; }
  sleep 0.5
done

# 3) conductor start
_say "== conductor start =="; "$HAKO_CMD" start >/dev/null 2>&1; sleep 2

# 4) diag
echo; bash "$DAASIM_DIR/diag.sh"
echo
_say "プロセスは起動中。次:"
_say "  pos変化テスト : $PYENV_PY $DAASIM_DIR/takeoff.py"
_say "  pos外部読取   : $PYENV_PY $DAASIM_DIR/read_pos.py"
_say "  片付け        : bash $DAASIM_DIR/cleanup.sh"
