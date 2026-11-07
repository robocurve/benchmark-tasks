#!/usr/bin/env bash
# Runs one trial of the stack-crates-largest-first task in the simulator.
#
#   ./run.sh demo                 Correct run, predicted 1.00
#   ./run.sh wrong_order          Medium crate first, large on top
#   ./run.sh offset               Small crate placed 25 mm off
#   ./run.sh leave_small          Small crate never placed
#   ./run.sh knock                Stack pushed after building
#   ./run.sh settle               Arms hold at home 8 s, predicted 0.00
#   ./run.sh dream                Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Stack the three crates on the pallet mark with the biggest at the bottom and the smallest on top, without letting the stack lean or fall."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  wrong_order) export DEMO_WRONG_ORDER=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  offset) export DEMO_OFFSET=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  leave_small) export DEMO_LEAVE_SMALL=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  knock) export DEMO_KNOCK=1; POLICY=(--policy yam_scripted); STEPS=1800 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|wrong_order|offset|leave_small|knock|settle|dream" >&2; exit 2 ;;
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
