#!/usr/bin/env bash
# Runs one trial of the sliding-tile-sort-three-tiles task in the simulator.
#
#   ./run.sh demo            Full 5-move push sequence, predicted 1.00
#   ./run.sh wrong_order     Stops after move 4 (2 in goal), predicted 0.50
#   ./run.sh lift_tile       Lifts a tile out of frame, predicted 0.00
#   ./run.sh settle          Both arms idle at home, predicted 0.00
#   ./run.sh dream           Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Slide the three tiles in the frame into the order shown on the card using the empty cell, without lifting a tile out of the frame."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=10); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  wrong_order) export DEMO_WRONG_ORDER=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  lift_tile) export DEMO_LIFT_TILE=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|wrong_order|lift_tile|settle|dream" >&2; exit 2 ;;
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

