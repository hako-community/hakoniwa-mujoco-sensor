#!/usr/bin/env bash
# 手順1: ネイティブ drone 物理サービスを起動（フォアグラウンド）。
# ★必ず DRONE_CORE ルートから実行する（drone_config の paramFilePath が cwd 相対のため）。
# 出力は logs/drone.log にも保存。WAIT START が出れば conductor 待ち。
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
_check_env || echo "[daasim] 上記 WARN を確認してください（続行します）"

cd "$DRONE_CORE"
export LD_LIBRARY_PATH="$MUJOCO_LIB_DIR:${LD_LIBRARY_PATH:-}"
_say "native service 起動: ./lnx/linux-main_hako_drone_service $DRONE_CONF $PDUDEF"
_say "log: $LOG_DIR/drone.log   （別端末で start_godot.sh → conductor.sh start）"
exec ./lnx/linux-main_hako_drone_service "$DRONE_CONF" "$PDUDEF" 2>&1 | tee "$LOG_DIR/drone.log"
