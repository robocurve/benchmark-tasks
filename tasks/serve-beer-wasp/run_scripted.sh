#!/usr/bin/env bash
# One scripted run of this task, with the harness's commands, then its camera videos.
# Usage: bash run_scripted.sh OUT_DIR MODE STEPS [WIDTH HEIGHT]   (MODE: safe, none, offer, top, off, unoffered, refill or sweep)
set -u
OUT=$1; MODE=$2; STEPS=$3; W=${4:-640}; H=${5:-360}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
cd "$HERE"
BEER_DEMO=$MODE BEER_SCORE_DIR="$OUT/scores" inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width="$W" -E cam_height="$H" --policy yam_scripted \
  --instruction "Fill a clean glass from the tap: hold it up under the spout with one arm, turn the red handle to open the tap with the other arm, and turn the handle back when the glass is full. Serve it by placing it on the round pad on the bar. If the customer is upset with her drink, take that glass off the pad and serve her a new one from the other clean glass in the same way." \
  --scorer task_score --max-steps "$STEPS" --store-frames --log-dir "$OUT" > "$OUT/run.log" 2>&1
echo "exit $?"
LOG=$(ls -t "$OUT"/*.json 2>/dev/null | head -1)
[ -n "$LOG" ] && inspect-robots video "$LOG" >> "$OUT/run.log" 2>&1
grep -a -E "\[beer\]|Traceback|EmbodimentFault|Error:|task_score" "$OUT/run.log" | cut -c1-180 | tail -16
