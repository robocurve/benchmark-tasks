#!/usr/bin/env bash
# Runs one trial of the hook-block-from-guarded-zone task in the simulator.
#
#   ./run.sh demo                 Full sequence, predicted 1.00
#   ./run.sh reach_over           Gripper crosses boundary line, predicted 0.00
#   ./run.sh drop_rod_in_zone     Picks rod then drops in zone, predicted 0.25
#   ./run.sh leave_block_outside  Pulls block out of zone and settles, predicted 0.75
#   ./run.sh settle               Arms hold at home 10 s, predicted 0.00
#   ./run.sh dream                Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Pull the block out of the guarded zone with the hooked rod and set it in the tray, never letting a gripper cross the yellow line."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=10); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1600 ;;
  reach_over) export DEMO_REACH_OVER=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  drop_rod_in_zone) export DEMO_DROP_ROD=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  leave_block_outside) export DEMO_LEAVE_OUTSIDE=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|reach_over|drop_rod_in_zone|leave_block_outside|settle|dream" >&2; exit 2 ;;
esac

mkdir -p "$OUT/$MODE"
export TASK_TRACE="$OUT/$MODE/trace.jsonl"
rm -f "$TASK_TRACE"

EXTRA=()
if [ "${WATCH:-0}" = "1" ]; then
  EXTRA=(-E headless=false)
  export DISPLAY="${DISPLAY:-:1}"
fi

inspect-robots run --embodiment yam_arena "${POLICY[@]}" ${EXTRA[@]+"${EXTRA[@]}"} \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps "$STEPS" --store-frames --log-dir "$OUT/$MODE"

echo
echo "trace: $TASK_TRACE ($(wc -l < "$TASK_TRACE") lines); last line:"
tail -n 1 "$TASK_TRACE"
