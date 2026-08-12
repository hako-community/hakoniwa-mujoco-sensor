#!/usr/bin/env bash
# Record one demo video: bring the stack up, film the Godot window while a
# scenario runs, burn a caption, tear everything down.
#
# The caption matters as much as the footage. These clips exist to show what a
# particular radar configuration can and cannot do, and a viewer cannot tell a
# 60 deg sector from a 360 deg one by looking at a wireframe alone.
#
#   Usage: bash demo_record.sh <out.mp4> <title> <subtitle> <launcher-cmd> <scenario-cmd> [seconds]
#
# `launcher-cmd` and `scenario-cmd` are single strings run with bash -c, so the
# caller can put env assignments in front of them.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/env.sh"

OUT="$1"; TITLE="$2"; SUBTITLE="$3"; LAUNCH="$4"; SCENARIO="$5"; SECS="${6:-70}"
FONT="${DEMO_FONT:-/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf}"
RAW="${OUT%.mp4}.raw.mp4"
mkdir -p "$(dirname "$OUT")"

_say "== demo: $TITLE =="
timeout 60 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true

# Full-screen Godot: the visualisation is the subject of these clips, so it gets
# the whole frame rather than a window on a desktop.
export HAKO_VIZ_FULLSCREEN="${HAKO_VIZ_FULLSCREEN:-1}"
# Push Godot's own HUD below the caption bar burned in at the end.
export HAKO_VIZ_HUD_Y="${HAKO_VIZ_HUD_Y:-112}"
_say "launch: $LAUNCH"
timeout 300 bash -c "$LAUNCH" >/dev/null 2>&1
sleep 2

# Locate the Godot window and its absolute position on the X display.
WID=""
for _ in $(seq 1 20); do
  WID=$(DISPLAY="$DISPLAY" xwininfo -root -tree 2>/dev/null \
        | grep -o '0x[0-9a-f]* "hakoniwa_1 (DEBUG)"' | head -1 | cut -d' ' -f1)
  [ -n "$WID" ] && break
  sleep 1
done
[ -n "$WID" ] || { echo "ERROR: Godot window not found"; exit 2; }
GEO=$(DISPLAY="$DISPLAY" xwininfo -id "$WID")
X=$(echo "$GEO" | awk '/Absolute upper-left X/ {print $4}')
Y=$(echo "$GEO" | awk '/Absolute upper-left Y/ {print $4}')
W=$(echo "$GEO" | awk '/^  Width:/ {print $2}')
H=$(echo "$GEO" | awk '/^  Height:/ {print $2}')
# x11grab wants even dimensions for yuv420p.
W=$((W - W % 2)); H=$((H - H % 2))
_say "window $WID at ${X},${Y} ${W}x${H}"

_say "recording ${SECS}s -> $RAW"
DISPLAY="$DISPLAY" ffmpeg -y -loglevel error -f x11grab -framerate 15 \
  -video_size "${W}x${H}" -i "${DISPLAY}+${X},${Y}" -t "$SECS" \
  -c:v libx264 -preset veryfast -pix_fmt yuv420p "$RAW" &
REC=$!
sleep 1

_say "scenario: $SCENARIO"
timeout "$((SECS - 5))" bash -c "$SCENARIO" > "$LOG_DIR/demo_scenario.log" 2>&1 || true
tail -3 "$LOG_DIR/demo_scenario.log" | sed 's/^/    /'

wait "$REC" 2>/dev/null || true

# Caption: a title bar across the top, subtitle under it. Drawn after the fact so
# the recording itself stays a faithful capture of the window.
#
# The text goes through textfile= rather than text=. Inside a filter description
# ':' separates options and ',' separates filters, so a caption containing either
# silently breaks the whole filter chain and no output is produced -- which is
# exactly what happened to the first clip, whose subtitle used "FOV = ...".
# textfile= sidesteps escaping entirely.
_say "captioning -> $OUT"
TITLE_F="$(mktemp)"; SUB_F="$(mktemp)"
printf '%s' "$TITLE" > "$TITLE_F"
printf '%s' "$SUBTITLE" > "$SUB_F"
ffmpeg -y -loglevel error -i "$RAW" -vf "\
drawbox=x=0:y=0:w=iw:h=104:color=black@0.78:t=fill,\
drawtext=fontfile=${FONT}:textfile=${TITLE_F}:x=24:y=16:fontsize=34:fontcolor=white,\
drawtext=fontfile=${FONT}:textfile=${SUB_F}:x=24:y=62:fontsize=22:fontcolor=0x9fe8ff" \
  -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT"
rm -f "$RAW" "$TITLE_F" "$SUB_F"

timeout 60 bash "$HERE/cleanup.sh" >/dev/null 2>&1 || true
_say "done: $(du -h "$OUT" | cut -f1)  $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$OUT" 2>/dev/null)s"
