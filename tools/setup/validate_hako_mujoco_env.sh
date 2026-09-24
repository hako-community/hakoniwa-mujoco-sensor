#!/usr/bin/env sh
# Validates only; it neither downloads dependencies nor changes the shell.
set -eu

ok=1
need() {
    name="$1"
    eval "value=\${$name:-}"
    if [ -z "$value" ]; then
        echo "WARN: $name is not set" >&2
        ok=0
    else
        printf '%s=%s\n' "$name" "$value"
    fi
}

need HAKO_CORE_INC_PATH
need HAKO_CORE_LIB_PATH
need HAKO_DRONE_INC_PATH
need HAKO_DRONE_LIB_PATH
need HAKO_BINARY_PATH
need MUJOCO_ROOT

if [ -n "${MUJOCO_ROOT:-}" ]; then
    [ -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ] || { echo "WARN: mujoco.h not found under MUJOCO_ROOT" >&2; ok=0; }
    case "$(uname -s)" in
        Darwin) candidate="$MUJOCO_ROOT/lib/libmujoco.dylib" ;;
        *) candidate="$MUJOCO_ROOT/lib/libmujoco.so" ;;
    esac
    [ -e "$candidate" ] || { echo "WARN: MuJoCo library not found: $candidate" >&2; ok=0; }
fi

[ "$ok" -eq 1 ] || { echo 'Build environment is incomplete. Set variables for installed Hakoniwa and matching MuJoCo.' >&2; exit 1; }
echo 'Hakoniwa/MuJoCo Unix build environment: valid'
