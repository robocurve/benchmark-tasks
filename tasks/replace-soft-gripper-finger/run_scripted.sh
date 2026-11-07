#!/usr/bin/env bash
# One scripted run of this task with the walkthrough's command, from this folder, in the Arena environment.
# Usage: run_scripted.sh new|used OUT_DIR [WIDTH HEIGHT]
set -uo pipefail
MODE=${1:?new or used}
OUT=${2:?out dir}
W=${3:-640}
H=${4:-360}
case "$MODE" in
  new) STEPS=1700 ;;
  used) STEPS=1700 ;;
  *) echo "unknown mode $MODE" >&2; exit 2 ;;
esac
mkdir -p "$OUT"
cd "$(dirname "$0")"
export FINGER_DEMO="$MODE" LIFT_SCORE_DIR="$OUT/scores"
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width="$W" -E cam_height="$H" -P hold_s=3 --scorer task_score --max-steps "$STEPS" \
  --instruction "One soft finger on this spare gripper is worn. Its three screws are already out, in the white block. Replace the finger with a new one, put the worn finger in the red bin, put the fork plate back on the new finger and drop the three screws into their holes." \
  --store-frames --log-dir "$OUT" > "$OUT/run.log" 2>&1
echo "exit $?"
grep -a -E "task_score|Traceback" "$OUT/run.log" | cut -c1-200
