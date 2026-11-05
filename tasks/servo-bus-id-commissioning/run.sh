#!/usr/bin/env bash
# Runs one trial of the servo bus commissioning task in the simulator.
#
#   ./run.sh demo         the scripted demo (both arms), reaches 1.0
#   ./run.sh settle       parts rest for a few seconds, checks stability (scores 0.0)
#   ./run.sh all_plugged  presses ASSIGN while all 3 are plugged in (scores 0.25)
#   ./run.sh wrong_order  commissions in reverse order without reset (scores 0.25)
#   ./run.sh recover      wrong order, resets, then commissions in correct order (scores 1.0)
#   ./run.sh dream        Dream-1 baseline policy
#
# Output goes to $OUT/<mode> (default ~/yam-eval): log, frames, and trace.jsonl.
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Three new servo modules (red, green, blue) are plugged into the bus and the bus status lamp is red. Press the white ASSIGN button on the front of the controller box (numbers the single un-numbered servo on the bus; yellow RESET button behind it clears numbers) until red is 1, green is 2, blue is 3 (the 3 x 3 readout board lights each servo's number in its colour row, columns 1 to 3), all three are plugged in, and the lamp turns green."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  all_plugged) export DEMO_ALL_PLUGGED=1; POLICY=(--policy yam_scripted); STEPS=300 ;;
  wrong_order) export DEMO_WRONG_ORDER=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  recover) export DEMO_RECOVER=1; POLICY=(--policy yam_scripted); STEPS=2700 ;;
  dream) POLICY=(--policy dream); STEPS=3000 ;;
  *) echo "usage: $0 demo|settle|all_plugged|wrong_order|recover|dream" >&2; exit 2 ;;
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
