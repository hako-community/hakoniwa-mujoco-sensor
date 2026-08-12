#!/usr/bin/env bash
# 後片付け: conductor 停止 → native/Godot プロセス kill → SHM reset。
# ★順序依存に注意: hako-cmd stop/status/reset は「マスタ(native物理サービス)が
#   生きている」前提で動く。マスタ不在で呼ぶと応答待ちで無限ハングする。
#   → 必ず stop を「native を kill する前」に実行し、各 hako-cmd は timeout でガードする。
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
# まだ native が生きているうちに stop（マスタ不在だとハングするので timeout 必須）。
# ★hako-cmd は SIGTERM を無視してブロックに居座るため、timeout は -k で KILL まで送る
#   （-k なしの timeout だと TERM が効かず timeout 自身も子を待ち続けてハングする）。
timeout -k 2 5 "$HAKO_CMD" stop  >/dev/null 2>&1 || true
pkill -f "linux-main_hako_drone_service" 2>/dev/null || true
pkill -f "Godot_v4.6.3-stable_mono"      2>/dev/null || true
pkill -f "sensor_bridge_multi"           2>/dev/null || true
# reset も同様に。native を kill した後はマスタ不在で必ずハングするので -k で強制終了させる
timeout -k 2 5 "$HAKO_CMD" reset >/dev/null 2>&1 || true
sleep 0.3 2>/dev/null || true
if ps -eo pid,args 2>/dev/null | grep -E "linux-main_hako_drone_service|Godot_v4.6.3-stable_mono" | grep -qv grep; then
  echo "[daasim] まだ残っています:"; ps -eo pid,args | grep -E "linux-main_hako_drone_service|Godot_v4.6.3-stable_mono" | grep -v grep | cut -c1-90
else
  echo "[daasim] clean"
fi
