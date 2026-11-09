#!/usr/bin/env bash
# Runs one trial of the interlocked-drawer-transfer task in the simulator.
#
#   ./run.sh demo             Correct carry transfer, predicted 1.00
#   ./run.sh park             Correct staging pad transfer, predicted 1.00
#   ./run.sh both_open        Both drawers opened at same time, predicted 0.25 (TIP fault)
#   ./run.sh leave_open       Right drawer left open at end, predicted 0.75
#   ./run.sh wrong_drawer     Block returned to left drawer, predicted 0.50
#   ./run.sh block_dropped    Block dropped off table, predicted 0.00
#   ./run.sh settle           Arms hold at home 8 s, predicted 0.00
#   ./run.sh dream            Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Move the white block from the left drawer to the right drawer, never letting both drawers be open at the same time."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=2200 ;;
  park) export DEMO_PARK=1; POLICY=(--policy yam_scripted); STEPS=2200 ;;
  both_open) export DEMO_BOTH_OPEN=1; POLICY=(--policy yam_scripted); STEPS=1400 ;;
  leave_open) export DEMO_LEAVE_OPEN=1; POLICY=(--policy yam_scripted); STEPS=2200 ;;
  wrong_drawer) export DEMO_WRONG_DRAWER=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  block_dropped) export DEMO_BLOCK_DROPPED=1; POLICY=(--policy yam_scripted); STEPS=1400 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|park|both_open|leave_open|wrong_drawer|block_dropped|settle|dream" >&2; exit 2 ;;
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
