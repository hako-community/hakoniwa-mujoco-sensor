#!/usr/bin/env bash
# 手順2: Godot avatar（C#/mono）を起動（フォアグラウンド）。
#   引数: headless（既定）| window
#   ★cwd=GODOT_DRONE で実行（HakoAsset が "./custom.json" を cwd 相対で読むため）。
#   ★custom.json は master(webavatar) とチャネル整合が必要（pos=ch1）。fix_custom_json.sh 参照。
# 出力は logs/godot.log にも保存。"OK: Register on Hakoniwa: GodotAsset" が出れば登録成功。
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
MODE="${1:-headless}"

cd "$GODOT_DRONE"
export PATH="$DOTNET_ROOT:$PATH"
export LD_LIBRARY_PATH="$GODOT_DRONE/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"

args=(--path "$GODOT_DRONE")
if [ "$MODE" = "headless" ]; then
  args+=(--headless)
else
  args+=(--rendering-method gl_compatibility)
fi
_say "Godot 起動($MODE): $GODOT_MONO ${args[*]}"
_say "cwd=$PWD  custom.json pos ch=$($PYENV_PY -c "import json;d=json.load(open('custom.json'));print([p['channel_id'] for r in d['robots'] if r['name']=='Drone' for p in r['shm_pdu_readers'] if p['org_name']=='pos'][0])" 2>/dev/null)"
_say "log: $LOG_DIR/godot.log"
exec "$GODOT_MONO" "${args[@]}" 2>&1 | tee "$LOG_DIR/godot.log"
