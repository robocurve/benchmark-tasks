#!/usr/bin/env bash
# One Dream-1 run with the walkthrough's command, stopped if the model's session is lost mid-run.
# Usage (from this folder, in the Arena environment): bash run_dream.sh OUT_DIR
#
# The gateway drops a session left idle for about 8 s of wall time, and the adapter then quietly starts a new episode. So:
# 20 rows per call instead of the default 48, and a watcher. The session can also expire during the adapter's 32-step
# warm-up, before the first action; nothing has played by then, so that first expiry is allowed.
OUT=${1:?usage: run_dream.sh OUT_DIR}
mkdir -p "$OUT"
export LIFT_SCORE_DIR="$OUT/scores" LIFT_STEP_FILE="$OUT/step"
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 -P rows_per_request=20 \
  --instruction "Put the blender in the lift and press the green button to send it up." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir "$OUT" > "$OUT/run.log" 2>&1 &
PID=$!
seen=0
while kill -0 "$PID" 2>/dev/null; do
  n=$(grep -a -c -E "Dream: .*(expired|serve_died|replaced|SessionExpired)" "$OUT/run.log")
  if [ "$n" -gt "$seen" ]; then
    t=$(cat "$OUT/step" 2>/dev/null || echo 0)
    if [ "$n" -gt 1 ] || awk "BEGIN{exit !($t > 1.75)}"; then
      echo "session lost at simulated t = $t s: run stopped, not a valid rollout"
      kill "$PID"
      break
    fi
    seen=$n
  fi
  sleep 0.5
done
wait "$PID"
grep -a -E "\[service_lift\]|task_score" "$OUT/run.log"
