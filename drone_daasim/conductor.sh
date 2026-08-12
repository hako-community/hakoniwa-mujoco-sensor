#!/usr/bin/env bash
# 手順3: 箱庭 conductor 操作。 引数: start | stop | status | reset
#   start でシミュレーションのステップ開始（全アセットが SYNC で歩進）。
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
CMD="${1:-status}"
_say "hako-cmd $CMD"
"$HAKO_CMD" "$CMD"
