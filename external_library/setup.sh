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
MJ_VERSION="$(lock_get version)"
MJ_URL="$(lock_get url)"
MJ_SHA="$(lock_get sha256)"
MJ_STRIP="$(lock_get strip)"
want_num="$(awk -F. '{printf "%d%03d%03d", $1, $2, $3}' <<<"${MJ_VERSION}")"

have=""
if [[ -f "${HERE}/mujoco/include/mujoco/mujoco.h" ]]; then
    have="$(grep -m1 mjVERSION_HEADER "${HERE}/mujoco/include/mujoco/mujoco.h" | awk '{print $3}')"
fi

if [[ "${have}" == "${want_num}" ]]; then
    echo "  [skip] mujoco ${MJ_VERSION} は既にある"
else
    if [[ -e "${HERE}/mujoco" ]]; then
        echo "  [ERROR] external_library/mujoco が lock (${MJ_VERSION}) と違う。" >&2
        echo "          消してから再実行してください: rm -rf ${HERE}/mujoco" >&2
        exit 1
    fi
    tmp="$(mktemp -d)"; trap 'rm -rf "${tmp}"' EXIT
    echo "  [fetch] mujoco ${MJ_VERSION} <- ${MJ_URL}"
    curl -fL --retry 3 -o "${tmp}/mujoco.tar.gz" "${MJ_URL}"
    echo "  [verify] sha256"
    echo "${MJ_SHA}  ${tmp}/mujoco.tar.gz" | sha256sum -c - \
        || { echo "  [ERROR] SHA256 が lock と一致しない。取得物を信用しない" >&2; exit 1; }
    tar xzf "${tmp}/mujoco.tar.gz" -C "${tmp}"
    mv "${tmp}/${MJ_STRIP}" "${HERE}/mujoco"
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
