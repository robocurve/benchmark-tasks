#!/usr/bin/env bash
# One Dream-1 run of this task with the standard command, stopped if the model's session is lost mid-run.
# Run on an otherwise idle machine, from this folder, in the Arena environment (source <ARENA_DIR>/yam.env).
#
# Usage: run_dream.sh OUT_DIR [ROWS_PER_REQUEST] [MAX_STEPS]     (defaults: 20 rows, 2,400 steps)
#
# The gateway drops a session left idle for about 8 s of wall time, and the adapter then quietly starts a new episode, which makes the
# rest of the run invalid. The session can also expire during the adapter's 32-step warm-up, before the first action. Nothing has
# played by then, so that first expiry is allowed, using the simulated time in $BEER_STEP_FILE (task.py writes it).
set -uo pipefail
OUT=${1:?usage: run_dream.sh OUT_DIR [ROWS_PER_REQUEST] [MAX_STEPS]}
ROWS=${2:-20}
STEPS=${3:-2400}
INSTRUCTION="Fill a clean glass from the tap: hold it up under the spout with one arm, turn the red handle to open the tap with the other arm, and turn the handle back when the glass is full. Serve it by placing it on the round pad on the bar. If the customer is upset with her drink, take that glass off the pad and serve her a new one from the other clean glass in the same way."
WARMUP_END=1.75  # the first call comes at step 32 of a 20 Hz run (1.6 s)
if ps -u "$USER" -o cmd= | grep -q -E "^([^ ]*python[^ ]* )?[^ ]*inspect-robots run"; then  # a wrapper that holds such a command, waiting to start it, is not a run
  echo "another inspect-robots run is on this machine: not starting" >&2
  exit 1
fi
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
cd "$HERE"
export BEER_SCORE_DIR="$OUT/scores" BEER_STEP_FILE="$OUT/step"
date +%T
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 -P rows_per_request="$ROWS" --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps "$STEPS" --store-frames --log-dir "$OUT" > "$OUT/run.log" 2>&1 &
PID=$!
seen=0
while kill -0 "$PID" 2>/dev/null; do
  n=$(grep -a -c -E "Dream: .*(expired|serve_died|replaced|SessionExpired)" "$OUT/run.log")
  if [ "$n" -gt "$seen" ]; then
    t=$(cat "$OUT/step" 2>/dev/null || echo unknown)
    echo "session expiry $n at simulated t = $t s"
    if [ "$n" -gt 1 ] || [ "$t" = unknown ] || awk "BEGIN{exit !($t > $WARMUP_END)}"; then
      echo "SESSION LOST: run stopped, not a valid rollout"
      kill "$PID"
      break
    fi
    seen=$n
  fi
  sleep 0.5
done
wait "$PID"
echo "exit $?"
date +%T
grep -a -E "\[beer\]|task_score|Traceback|BAD_REQUEST|EmbodimentFault" "$OUT/run.log" | cut -c1-240
echo "Dream: lines in the log: $(grep -a -c -E 'Dream: ' "$OUT/run.log")"
