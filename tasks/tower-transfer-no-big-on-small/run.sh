#!/usr/bin/env bash
# Runs one trial of the tower-transfer-no-big-on-small task in the simulator.
#
#   ./run.sh demo                 Correct 7-move run, predicted 1.00
#   ./run.sh big_on_small         Medium placed on small block, predicted 0.00
#   ./run.sh leave_one            Stops after 6 moves, predicted 0.75
#   ./run.sh wrong_target         Tower moved to middle mark, predicted 0.50
#   ./run.sh settle               Arms hold at home 8 s, predicted 0.00
#   ./run.sh dream                Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Move the three-block tower from the left mark to the right mark, keeping the biggest at the bottom, and never set a bigger block on a smaller one."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1600 ;;
  big_on_small) export DEMO_BIG_ON_SMALL=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  leave_one) export DEMO_LEAVE_ONE=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  wrong_target) export DEMO_WRONG_TARGET=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|big_on_small|leave_one|wrong_target|settle|dream" >&2; exit 2 ;;
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

