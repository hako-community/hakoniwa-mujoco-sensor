#!/usr/bin/env bash
# 共通設定。各スクリプトが `source` する。必要なら環境変数で上書き可。
# 例: MUJOCO_LIB_DIR=/path/to/lib bash drone_daasim/start_drone.sh
set -u

_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DAASIM_DIR="$_HERE"
# This drone_daasim lives inside hakoniwa-mujoco-sensor (its home). The other repos
# are siblings under the same parent dir (.../hakoniwa/). All overridable via env.
export SENSOR_REPO="${SENSOR_REPO:-$(cd "$_HERE/.." && pwd)}"          # hakoniwa-mujoco-sensor (home)
_HAKO_ROOT="$(cd "$_HERE/../.." && pwd)"                                # .../hakoniwa
export DRONE_CORE="${DRONE_CORE:-$_HAKO_ROOT/hakoniwa-drone-core}"
export GODOT_DRONE="${GODOT_DRONE:-$_HAKO_ROOT/hakoniwa-godot-drone}"
# 2026-08 の 2 層化で、sensor が依存する下位レイヤ（primitive_types.hpp 等）。
export RUNTIME_REPO="${RUNTIME_REPO:-$_HAKO_ROOT/hakoniwa-mujoco-runtime}"

# センサー用環境ジオメトリ（env.xml / env.tscn / *.obb.json）。
# 2026-08 に hakoniwa-envsim-sensor は廃止され hakoniwa-simenv-data に統合された。
# ディレクトリも examples/<name>/ から examples/sensor_envs/<name>/ に移動している。
# 旧リポジトリ本体は 2026-08-12 に削除済み（フォールバック先はもう存在しない）。
export SIMENV_DATA="${SIMENV_DATA:-$_HAKO_ROOT/hakoniwa-simenv-data}"
export SENSOR_ENVS="${SENSOR_ENVS:-$SIMENV_DATA/examples/sensor_envs}"
if [ ! -d "$SENSOR_ENVS" ]; then
  echo "WARN: センサー用環境データが見つかりません: $SENSOR_ENVS" >&2
  echo "      hakoniwa-simenv-data を clone するか SENSOR_ENVS を明示してください。" >&2
fi
# 旧名。まだ ENVSIM_REPO を参照する手元スクリプト向けの後方互換。
export ENVSIM_REPO="${ENVSIM_REPO:-$SIMENV_DATA}"

export LOG_DIR="${LOG_DIR:-$_HERE/logs}"; mkdir -p "$LOG_DIR"

# Python（既存検証環境＝pyenv 3.12.3。hakoniwa-pdu 1.3.5 等が入っている）
export PYENV_PY="${PYENV_PY:-$HOME/.pyenv/versions/3.12.3/bin/python}"

# .NET SDK（~/.dotnet にユーザインストール済み。Godot mono のビルド/実行に必要）
export DOTNET_ROOT="${DOTNET_ROOT:-$HOME/.dotnet}"

# Godot mono バイナリ（C# プロジェクト用）
export GODOT_MONO="${GODOT_MONO:-/usr/local/bin/Godot_v4.6.3-stable_mono_linux_x86_64/Godot_v4.6.3-stable_mono_linux.x86_64}"

# 箱庭コマンド / offset（/usr/local/hakoniwa に core 導入済み）
export HAKO_CMD="${HAKO_CMD:-/usr/local/hakoniwa/bin/hako-cmd}"
export HAKO_BINARY_PATH="${HAKO_BINARY_PATH:-/usr/local/hakoniwa/share/hakoniwa/offset}"

# libmujoco.so.3.9.0 を含むディレクトリ（native drone service が動的リンク）
if [ -z "${MUJOCO_LIB_DIR:-}" ]; then
  # 旧 hakoniwa-mujoco-robots/src/cmake-build/_deps/... は同リポの廃止に伴い消滅。
  # 現在はワークスペース直下の FetchContent キャッシュか pyenv の mujoco を使う。
  for d in \
    "$(cd "$_HAKO_ROOT/.." && pwd)/.cache/deps/mujoco_bin-src/lib" \
    "$HOME/.pyenv/versions/3.12.3/lib/python3.12/site-packages/mujoco"; do
    if [ -e "$d/libmujoco.so.3.9.0" ]; then MUJOCO_LIB_DIR="$d"; break; fi
  done
fi
export MUJOCO_LIB_DIR="${MUJOCO_LIB_DIR:-}"

# 仮想ディスプレイ（このPCは Xvfb の :1。GPU 無しのため Godot は gl_compatibility）
export DISPLAY="${DISPLAY:-:1}"

# 共有 pdudef / drone 設定（DRONE_CORE からの相対）
export PDUDEF="${PDUDEF:-config/pdudef/webavatar.json}"
export DRONE_CONF="${DRONE_CONF:-config/drone/api-1}"

_say() { printf "\033[1;34m[daasim] %s\033[0m\n" "$*"; }

_check_env() {
  local ok=1
  [ -x "$PYENV_PY" ] || { echo "WARN: PYENV_PY not found: $PYENV_PY"; ok=0; }
  [ -x "$GODOT_MONO" ] || { echo "WARN: GODOT_MONO not found: $GODOT_MONO"; ok=0; }
  [ -x "$HAKO_CMD" ] || { echo "WARN: HAKO_CMD not found: $HAKO_CMD"; ok=0; }
  [ -d "$HAKO_BINARY_PATH" ] || { echo "WARN: HAKO_BINARY_PATH not found: $HAKO_BINARY_PATH"; ok=0; }
  [ -n "$MUJOCO_LIB_DIR" ] && [ -e "$MUJOCO_LIB_DIR/libmujoco.so.3.9.0" ] || { echo "WARN: libmujoco.so.3.9.0 not found (set MUJOCO_LIB_DIR)"; ok=0; }
  [ -x "$DRONE_CORE/lnx/linux-main_hako_drone_service" ] || { echo "WARN: native service missing or not +x: chmod +x $DRONE_CORE/lnx/linux-*"; ok=0; }
  return $((1-ok))
}
