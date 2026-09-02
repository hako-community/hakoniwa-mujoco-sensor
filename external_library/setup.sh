#!/usr/bin/env bash
# external_library 配下に外部依存を用意する。
#
#   external_library/setup.sh
#
# ★★★★ 2026-09-02: **他リポジトリの木を一切指さない。**
#   それまでは configure 時に外を探しに行っていた:
#     * MuJoCo   -> `../../.cache/deps/mujoco_bin-src` ＝ **ドローンの木の外**にある
#                   別プロジェクト（ヒューマノイド）のキャッシュへの symlink
#     * nlohmann -> `../nlohmann-json` ＝ **hakoniwa-drone-pro/thirdparty への symlink**
#                   （drone-pro は PRIVATE で、しかも「使わない」と決めたリポジトリ）
#   ★★★ **このリポジトリは単体でビルドできなければ意味がない。**
#
#   * nlohmann/json は **git submodule**（版が git に残る）
#   * MuJoCo は **バイナリ**なので `mujoco.lock` の版と SHA256 で固定して取得する
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"

echo "=== external_library の用意 ==="

# --- 1. submodule（nolman = nlohmann/json）---
if [[ -d "${ROOT}/.git" ]]; then
    echo "  [submodule] nolman を同期する"
    git -C "${ROOT}" submodule update --init --recursive external_library/nolman
else
    echo "  [WARN] git 管理下ではないので submodule を同期できない" >&2
fi

# --- 2. MuJoCo（バイナリリリース。版と SHA256 を lock で固定）---
LOCK="${HERE}/mujoco.lock"
lock_get() { sed -n "s/^[[:space:]]*$1[[:space:]]*=[[:space:]]*//p" "${LOCK}" | head -1; }

# ★★★★ プラットフォームを判定して lock の項目を選ぶ（2026-09-02）。
#   ★★★ Windows（Git Bash / MSYS）で Linux 版を掴むと **`mujoco.dll` が無い**まま進み、
#     「DLL not found」という分かりにくい形で落ちる。ここで取り違えないこと。
case "$(uname -s)" in
    Linux)                    _os=linux ;;
    MINGW*|MSYS*|CYGWIN*)     _os=windows ;;
    Darwin)
        echo "  [ERROR] macOS は公式が .dmg でしか配っていないため自動取得しない。" >&2
        echo "          手で ${HERE}/mujoco/ に 3.9.0 を置いてください。" >&2
        exit 1 ;;
    *) echo "  [ERROR] 未知の OS: $(uname -s)" >&2; exit 1 ;;
esac
case "$(uname -m)" in
    x86_64|amd64)             _arch=x86_64 ;;
    aarch64|arm64)            _arch=aarch64 ;;
    *) echo "  [ERROR] 未知の CPU: $(uname -m)" >&2; exit 1 ;;
esac
_key="${_os}_${_arch}"

MJ_VERSION="$(lock_get version)"
MJ_URL="$(lock_get "${_key}\.url")"
MJ_SHA="$(lock_get "${_key}\.sha256")"
MJ_STRIP="$(lock_get "${_key}\.strip")"
if [[ -z "${MJ_URL}" ]]; then
    echo "  [ERROR] mujoco.lock に ${_key} の項目が無い" >&2
    exit 1
fi

want_num="$(awk -F. '{printf "%d%03d%03d", $1, $2, $3}' <<<"${MJ_VERSION}")"
have=""
if [[ -f "${HERE}/mujoco/include/mujoco/mujoco.h" ]]; then
    have="$(grep -m1 mjVERSION_HEADER "${HERE}/mujoco/include/mujoco/mujoco.h" | awk '{print $3}')"
fi

if [[ "${have}" == "${want_num}" ]]; then
    echo "  [skip] mujoco ${MJ_VERSION} は既にある（${_key}）"
else
    if [[ -e "${HERE}/mujoco" ]]; then
        echo "  [ERROR] external_library/mujoco が lock (${MJ_VERSION}) と違う。" >&2
        echo "          消してから再実行してください: rm -rf ${HERE}/mujoco" >&2
        exit 1
    fi
    tmp="$(mktemp -d)"
    trap 'rm -rf "${tmp}"' EXIT
    _ext="tar.gz"; [[ "${MJ_URL}" == *.zip ]] && _ext="zip"
    echo "  [fetch] mujoco ${MJ_VERSION} (${_key}) <- ${MJ_URL}"
    curl -fL --retry 3 -o "${tmp}/mujoco.${_ext}" "${MJ_URL}"
    echo "  [verify] sha256"
    echo "${MJ_SHA}  ${tmp}/mujoco.${_ext}" | sha256sum -c - \
        || { echo "  [ERROR] SHA256 が lock と一致しない。取得物を信用しない" >&2; exit 1; }
    mkdir -p "${tmp}/x"
    if [[ "${_ext}" == "zip" ]]; then
        # ★★★ 公式の Windows zip は**パス区切りが円記号**なので unzip が警告を出し、
        #   **警告だけでも終了コード 1 を返す**（`set -e` がここで止まる。実際に踏んだ）。
        #   → 0 と 1 は成功として扱い、**中身があるかで判定する**。
        # ★ Git for Windows には unzip が無いことがある。tar(bsdtar) は zip を読める。
        if command -v unzip >/dev/null 2>&1; then
            unzip -q "${tmp}/mujoco.zip" -d "${tmp}/x" || [[ $? -le 1 ]]
        else
            tar -xf "${tmp}/mujoco.zip" -C "${tmp}/x"
        fi
    else
        tar xzf "${tmp}/mujoco.tar.gz" -C "${tmp}/x"
    fi
    # ★★ zip が読み取り専用の属性を持っていると、あとで消せない／上書きできない。
    chmod -R u+w "${tmp}/x" 2>/dev/null || true
    if [[ ! -f "${tmp}/x/include/mujoco/mujoco.h" && ! -f "${tmp}/x/${MJ_STRIP}/include/mujoco/mujoco.h" ]]; then
        echo "  [ERROR] 展開したが include/mujoco/mujoco.h が無い（書庫の構造が想定と違う）" >&2
        exit 1
    fi
    if [[ -n "${MJ_STRIP}" ]]; then
        mv "${tmp}/x/${MJ_STRIP}" "${HERE}/mujoco"
    else
        mv "${tmp}/x" "${HERE}/mujoco"          # ★ Windows の zip は包みが無い
    fi
    echo "  [ok] external_library/mujoco"
fi

echo
echo "=== 確認 ==="
for d in nolman mujoco; do
    printf "  %-24s %s\n" "${d}" "$([[ -e "${HERE}/${d}" ]] && echo あり || echo '(無し)')"
done
grep -m1 mjVERSION_HEADER "${HERE}/mujoco/include/mujoco/mujoco.h" | sed 's/^/  MuJoCo /'

# --- 3. ★★★★ 他リポジトリへ逃げていないことを最後に断言する ---
leaks="$(find "${HERE}" -maxdepth 2 -type l -printf '%p -> %l\n' 2>/dev/null \
         | grep -E 'hakoniwa-drone-pro|work_humanoid|ArmPi_Ultra' || true)"
if [[ -n "${leaks}" ]]; then
    echo "  [ERROR] 他リポジトリの木を指している:" >&2
    echo "${leaks}" >&2
    exit 1
fi
echo
echo "  ★ 外部リポジトリ（drone-pro・ヒューマノイド）への参照は無い"
