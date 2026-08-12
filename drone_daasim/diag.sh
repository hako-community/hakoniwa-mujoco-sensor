#!/usr/bin/env bash
# 診断: ログとステータスから「sim が歩進しているか / Godot が Start を受けたか」を要約。
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

echo "===== hako-cmd status ====="
"$HAKO_CMD" status 2>&1 | grep -iE "status=" || echo "  (status 取得不可: master 未起動?)"

echo "===== native drone (logs/drone.log) ====="
if [ -f "$LOG_DIR/drone.log" ]; then
  echo "  advanceTimeStep 回数 : $(grep -c 'advanceTimeStep' "$LOG_DIR/drone.log")"
  echo "  最後の行            : $(tail -1 "$LOG_DIR/drone.log")"
  grep -qE "WAIT START" "$LOG_DIR/drone.log" && echo "  WAIT START          : yes"
  grep -qE "start simulation" "$LOG_DIR/drone.log" && echo "  start simulation    : yes"
else echo "  (no drone.log)"; fi

echo "===== Godot avatar (logs/godot.log) ====="
if [ -f "$LOG_DIR/godot.log" ]; then
  grep -qE "OK: Register on Hakoniwa" "$LOG_DIR/godot.log" && echo "  Register            : OK" || echo "  Register            : NG/未"
  grep -qE "Can not register" "$LOG_DIR/godot.log" && echo "  ! Can not register   : 検出"
  grep -qE "Can not declare pdu" "$LOG_DIR/godot.log" && echo "  ! Can not declare pdu: 検出"
  echo "  EventInitialize     : $(grep -c 'Event Initialize' "$LOG_DIR/godot.log")"
  echo "  EventStart(推定)    : $(grep -cE 'EventStart|Event Start' "$LOG_DIR/godot.log")"
else echo "  (no godot.log)"; fi

echo "===== プロセス ====="
ps -eo pid,args 2>/dev/null | grep -E "linux-main_hako_drone_service|Godot_v4.6.3-stable_mono" | grep -v grep | cut -c1-100 || echo "  (none)"
