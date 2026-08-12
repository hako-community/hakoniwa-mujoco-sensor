#!/usr/bin/env bash
# Godot の C# ソリューションをビルド（dotnet 必要）。初回や Scripts 変更後に実行。
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$GODOT_DRONE"
export PATH="$DOTNET_ROOT:$PATH"
export LD_LIBRARY_PATH="$GODOT_DRONE/Plugins/Linux/x86_64:${LD_LIBRARY_PATH:-}"
_say "build solutions (headless)"
DISPLAY="$DISPLAY" "$GODOT_MONO" --headless --path "$GODOT_DRONE" --build-solutions --quit
echo "exit=$?  (dll: .godot/mono/temp/bin/Debug/hakoniwa_1.dll)"
