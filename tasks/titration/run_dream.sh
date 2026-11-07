#!/usr/bin/env bash
# One Dream-1 run of this task with the standard command, stopped if the model's session is lost mid-run.
# Run on an otherwise idle machine, from this folder, in the Arena environment (source <ARENA_DIR>/yam.env).
#
# Usage: run_dream.sh OUT_DIR [SEED] [ROWS_PER_REQUEST]     (defaults: seed 1, 20 rows)
#
# The gateway drops a session left idle for about 8 s of wall time, and the adapter then quietly starts a new episode, which makes the
# rest of the run invalid. The session can also expire during the adapter's 32-step warm-up, before the first action. Nothing has
# played by then, so that first expiry is allowed, using the simulated time in the task's score record, written every second.
set -uo pipefail
OUT=${1:?usage: run_dream.sh OUT_DIR [SEED] [ROWS_PER_REQUEST]}
SEED=${2:-1}
ROWS=${3:-20}
HERE=$(cd "$(dirname "$0")" && pwd)
INSTRUCTION=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['instruction'])" "$HERE/task.json")
STEPS=$(python3 -c "import json,re,sys; print(re.search(r'[\d,]+', json.load(open(sys.argv[1]))['run_limit']).group().replace(',',''))" "$HERE/task.json")
if ps -u "$USER" -o cmd= | grep -q -E "^([^ ]*python[^ ]* )?[^ ]*inspect-robots run"; then
  echo "another inspect-robots run is on this machine: not starting" >&2
  exit 1
fi
mkdir -p "$OUT"
cd "$HERE" || exit 1
export TITRATION_SCORE_DIR="$OUT/scores"
date +%T
echo "steps $STEPS, seed $SEED, rows $ROWS"
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 -P rows_per_request="$ROWS" --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps "$STEPS" --seed "$SEED" --store-frames --log-dir "$OUT" > "$OUT/run.log" 2>&1 &
PID=$!
seen=0
while kill -0 "$PID" 2>/dev/null; do
  n=$(grep -a -c -E "Dream: .*(expired|serve_died|replaced|SessionExpired)" "$OUT/run.log")
  if [ "$n" -gt "$seen" ]; then
    t=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['t'])" "$OUT"/scores/run_seed*.json 2>/dev/null || echo 0)
    echo "session expiry $n, record at simulated t = $t s"
    if [ "$n" -gt 1 ] || awk "BEGIN{exit !($t > 1.0)}"; then
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
grep -a -E "\[titration\]|task_score|Traceback|BAD_REQUEST|EmbodimentFault" "$OUT/run.log" | cut -c1-240
echo "Dream: lines in the log: $(grep -a -c -E 'Dream: ' "$OUT/run.log")"
