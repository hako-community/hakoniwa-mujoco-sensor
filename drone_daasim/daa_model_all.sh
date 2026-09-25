#!/usr/bin/env bash
# daa_model_all.sh — ★★★★ 模型縮尺（1/250）の DAA 8 本を 1 本ずつ通しで回す回帰台（2026-09-14）。
#
# ★★★★ 位置づけ（ユーザ確定 2026-09-14）:
#   * **正は実寸**（Origin-01＋内蔵 FC ＝ `hakoniwa-drone-companion/tools/s2_full_run.bash`）。
#   * 模型縮尺のプラントは **hakoniwa-drone-core の `linux-main_hako_drone_service`**（半幅 0.25 m の小型機）で、
#     内蔵 FC ではない。**検証範囲の外の旧土俵**である。
#   * それでも回すのは、**シナリオの Python（daa_common / daa_metrics / 各シナリオ）が実寸と共通**だから。
#     共通の Python を変えたら、これで「旧土俵を壊していない」ことを見る（見張り）。
#
# 8 本と土俵（各シナリオの `Run AFTER` のとおり）:
#   S-1 two_drone_avoid        noground
#   S-2 scenario_s2_converging noground ＋ 広角マニフェスト（方位 ±75°）
#   S-3 scenario_s3_overtaking noground
#   S-4 scenario_s4_vertical   noground
#   S-5 scenario_s5_landing    noground
#   S-6 scenario_s6_crewed     crewed
#   S-7 scenario_s7_clutter    room
#   S-8 scenario_s8_failsafe   noground（ブリッジに SIGSTOP）
#
#   bash drone_daasim/daa_model_all.sh            # 8 本
#   ONLY="s2 s7" bash drone_daasim/daa_model_all.sh
#
# ★★★ 箱庭の実験は同時に 2 つ走らせない（[[hakoniwa-core-landmines]]）。1 本ごとに cleanup する。
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
# ★★★★ 縮尺は模型に固定する（呼ぶ側の環境に DAA_SCALE=full が残っていると実寸の値で走る）
unset DAA_SCALE S2_SCALE S3_SCALE S4_SCALE S5_SCALE S6_SCALE S7_SCALE S8_SCALE
CFG="$(cd "$HERE/.." && pwd)/config/drone_sensors"
OUT="${OUT_DIR:-$HERE/logs/model_all}"
TO="${SCENARIO_TIMEOUT:-900}"
mkdir -p "$OUT"

# 名前  スクリプト  土俵  マニフェスト（空なら既定）
TABLE="
s1 two_drone_avoid.py        noground -
s2 scenario_s2_converging.py noground drone-sensors-wide.json
s3 scenario_s3_overtaking.py noground -
s4 scenario_s4_vertical.py   noground -
s5 scenario_s5_landing.py    noground -
s6 scenario_s6_crewed.py     crewed   -
s7 scenario_s7_clutter.py    room     -
s8 scenario_s8_failsafe.py   noground -
"
ONLY="${ONLY:-s1 s2 s3 s4 s5 s6 s7 s8}"

declare -a SUMMARY
while read -r name script env manifest; do
  [ -z "$name" ] && continue
  case " $ONLY " in *" $name "*) ;; *) continue ;; esac
  log="$OUT/${name}.log"
  _say "==== ${name}（${script} ／ ${env} ／ ${manifest}）===="
  if [ "$manifest" = "-" ]; then unset A2_MANIFEST; else export A2_MANIFEST="$CFG/$manifest"; fi
  if ! bash "$HERE/two_drone_run.sh" "$env" > "$OUT/${name}_stack.log" 2>&1; then
    _say "  ✕ スタックが上がらない（$OUT/${name}_stack.log）"
    SUMMARY+=("${name}  STACK-FAIL")
    timeout 20 bash "$HERE/cleanup.sh" > /dev/null 2>&1
    continue
  fi
  ( cd "$SCENARIOS_DIR" && timeout -k 5 "$TO" "$PYENV_PY" "$script" ) > "$log" 2>&1
  rc=$?
  res="$(grep -m1 '^RESULT:' "$log" || true)"
  [ -z "$res" ] && res="RESULT 行なし（rc=${rc}）"
  _say "  ${res}"
  SUMMARY+=("${name}  ${res}")
  timeout 20 bash "$HERE/cleanup.sh" > /dev/null 2>&1
  sleep 2
done <<< "$TABLE"
unset A2_MANIFEST

echo
_say "==== まとめ（ログ: $OUT）===="
for s in "${SUMMARY[@]}"; do echo "  $s"; done
fails=$(printf '%s\n' "${SUMMARY[@]}" | grep -vc 'RESULT: PASS')
exit $(( fails > 0 ))
