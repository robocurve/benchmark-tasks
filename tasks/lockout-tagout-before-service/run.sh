#!/usr/bin/env bash
# Runs one trial of the lockout-tagout-before-service task in the simulator.
#
#   ./run.sh demo             Correct run, predicted 1.00
#   ./run.sh cover_first      Cover opened before the breaker is off
#   ./run.sh no_pin           Lock pin never inserted
#   ./run.sh no_test          TEST never pressed
#   ./run.sh no_hold          Cover not held open
#   ./run.sh order_swapped    Steps done in the wrong order
#   ./run.sh settle           Parts rest, arms hold at home 8 s, predicted 0.00
#   ./run.sh dream            Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Switch the breaker off, lock it with the red pin, press TEST, then hold the cover open and press the blue service button: never open the cover while the breaker is on."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=2200 ;;
  cover_first) export DEMO_COVER_FIRST=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  no_pin) export DEMO_NO_PIN=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  no_test) export DEMO_NO_TEST=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  no_hold) export DEMO_NO_HOLD=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  order_swapped) export DEMO_ORDER_SWAPPED=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|cover_first|no_pin|no_test|no_hold|order_swapped|settle|dream" >&2; exit 2 ;;
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
