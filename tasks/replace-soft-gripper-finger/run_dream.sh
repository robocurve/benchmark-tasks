#!/usr/bin/env bash
# One Dream-1 run of a task file with the walkthrough's command, stopped if the model's session is lost mid-run.
# Run on an otherwise idle machine, from the task's folder, in the Arena environment (source <ARENA_DIR>/yam.env).
#
# Usage: run_dream.sh OUT_DIR "INSTRUCTION" [ROWS_PER_REQUEST] [MAX_STEPS]     (defaults: 20 rows, 2,400 steps)
#
# The gateway drops a session left idle for about 8 s of wall time, and the adapter then quietly starts a new episode, which makes the
# rest of the run invalid. The session can also expire during the adapter's 32-step warm-up, before the first action. Nothing has
# played by then, so that first expiry is allowed if the task's helper writes the simulated time to $LIFT_STEP_FILE (as fingerkit.py does);
# without that file, the watcher cannot tell and stops at any expiry.
set -uo pipefail
OUT=${1:?usage: run_dream.sh OUT_DIR "INSTRUCTION" [ROWS_PER_REQUEST] [MAX_STEPS]}
INSTRUCTION=${2:?instruction, word for word as in task.json}
ROWS=${3:-20}
STEPS=${4:-2400}
WARMUP_END=1.75  # the first call comes at step 32 of a 20 Hz run (1.6 s)
if ps -u "$USER" -o cmd= | grep -q "[i]nspect-robots run"; then
  echo "another inspect-robots run is on this machine: not starting" >&2
  exit 1
fi
mkdir -p "$OUT"
export LIFT_SCORE_DIR="$OUT/scores" LIFT_STEP_FILE="$OUT/step"
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
grep -a -E "task_score|Traceback" "$OUT/run.log" | cut -c1-200
echo "Dream: lines in the log: $(grep -a -c -E 'Dream: ' "$OUT/run.log")"
