#!/usr/bin/env bash
# Record the whole radar demo set.
#
# Five clips, chosen so that each one answers a question the previous one raises:
#   1  what the radar sees at all           (single aircraft, walls, detection tags)
#   2  the baseline avoidance               (one 60 deg forward radar)
#   3  where that baseline breaks           (90 deg crossing -> never detected)
#   4  what fixes it                        (360 deg azimuth, same encounter)
#   5  the cheaper fix for one blind sector (two radars: forward + rear)
#
#   Usage: bash demo_all.sh [output-dir] [seconds-per-clip]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"
REPO="$(cd "$HERE/.." && pwd)"
OUT="${1:-$HERE/demo}"
SECS="${2:-75}"
CFG="$REPO/config/a2"
mkdir -p "$OUT"

rec() { bash "$HERE/demo_record.sh" "$@"; }

# --- 1) what the radar sees ------------------------------------------------
rec "$OUT/01_radar_scan.mp4" \
  "1. Radar scan - one aircraft, walled room" \
  "sensor_viz.tscn: wireframe = sensor FOV, points = returns, tags = detected objects (textfile captions handle : and =)" \
  "bash $HERE/sensor_viz_run.sh window radar oblique 1.0" \
  "$PYENV_PY $HERE/takeoff.py $DRONE_CORE/config/pdudef/webavatar.json" \
  "$SECS"

# --- 2) baseline avoidance --------------------------------------------------
rec "$OUT/02_avoid_single_radar.mp4" \
  "2. Avoidance - single forward radar (60 deg)" \
  "S-1 head-on. Rule 182: both alter course to their own right. Well Clear 1.25 m" \
  "bash $HERE/two_drone_viz_run.sh window radar oblique 1.0 noground" \
  "$PYENV_PY $HERE/two_drone_avoid.py" \
  "$SECS"

# --- 3) where the forward sector breaks -------------------------------------
rec "$OUT/03_crossing_60deg_fails.mp4" \
  "3. LIMIT of a 60 deg sector - 90 deg crossing" \
  "On a collision course the bearing stays near 45 deg: outside the FOV, never detected" \
  "bash $HERE/two_drone_viz_run.sh window radar top 0.5 noground" \
  "S2_START=4.0 $PYENV_PY $HERE/scenario_s2_converging.py" \
  "$SECS"

# --- 4) the same encounter with 360 deg azimuth -----------------------------
rec "$OUT/04_crossing_360deg.mp4" \
  "4. Same crossing with 360 deg azimuth" \
  "Rule 181: the aircraft that sees the other on its right gives way; the other holds course" \
  "A2_MANIFEST=$CFG/drone-a2-sensors-360.json bash $HERE/two_drone_viz_run.sh window radar top 0.5 noground" \
  "S2_START=4.0 $PYENV_PY $HERE/scenario_s2_converging.py" \
  "$SECS"

# --- 5) two radars on one aircraft ------------------------------------------
rec "$OUT/05_dual_radar_overtaking.mp4" \
  "5. Two radars per aircraft - forward 60 deg + rear sector" \
  "S-3 overtaking. The rear sector (az 150..210) covers the blind spot behind each aircraft" \
  "A2_DUAL_RADAR=1 A2_MANIFEST=$CFG/drone-a2-sensors-dual.json bash $HERE/two_drone_viz_run.sh window radar top 0.45 noground" \
  "S3_GAP=3.0 $PYENV_PY $HERE/scenario_s3_overtaking.py" \
  "$SECS"

echo
_say "demo set written to $OUT"
ls -la "$OUT"/*.mp4
