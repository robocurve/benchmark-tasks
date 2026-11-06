#!/usr/bin/env bash
# One scripted run of this task with the walkthrough's command, from this folder, in the Arena environment.
# Usage: run_scripted.sh DROPS SHAKES OUT_DIR [WIDTH HEIGHT]     (3 and 4 are the demo; set DROP_MODE=hard, spill, dropped, early_lift or no_tube for a fault)
set -uo pipefail
DROPS=${1:?how many squeezes}
SWINGS=${2:?how many shakes}
OUT=${3:?out dir}
W=${4:-640}
H=${5:-360}
mkdir -p "$OUT"
cd "$(dirname "$0")"
export DROP_COUNT="$DROPS" SWING_COUNT="$SWINGS" LIFT_SCORE_DIR="$OUT/scores"
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width="$W" -E cam_height="$H" -P hold_s=3 --scorer task_score --max-steps 2000 \
  --instruction "Add exactly 3 drops of the chemical in the pipette to the test tube. The chemical is already in the pipette. Stand the pipette back in its holder. Then put the cork in the tube, lift the tube out of the rack, shake it from side to side with your wrist, like a bell, exactly 4 times, stand it back in the rack and let go." \
  --store-frames --log-dir "$OUT" > "$OUT/run.log" 2>&1
echo "exit $?"
grep -a -E "task_score|Traceback" "$OUT/run.log" | cut -c1-200
