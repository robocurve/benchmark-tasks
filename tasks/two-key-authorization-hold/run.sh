#!/usr/bin/env bash
# Runs one trial of the two-key-authorization-hold task in the simulator.
#
#   ./run.sh demo             Correct run, predicted 1.00
#   ./run.sh sequential       Keys held one after the other, never together
#   ./run.sh one_key          Only one guard opened and one key pressed
#   ./run.sh release_early    Both keys released before 3.0 s twice
#   ./run.sh no_confirm       Keys held until ARMED but CONFIRM never pressed
#   ./run.sh settle           Parts rest, arms hold at home 8 s, predicted 0.00
#   ./run.sh dream            Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Slide both red guards aside, then press and hold both key buttons together until the ARMED lamp turns green, and press the white CONFIRM button before the lamp turns off."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1200 ;;
  sequential) export DEMO_SEQUENTIAL=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  one_key) export DEMO_ONE_KEY=1; POLICY=(--policy yam_scripted); STEPS=800 ;;
  release_early) export DEMO_RELEASE_EARLY=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  no_confirm) export DEMO_NO_CONFIRM=1; POLICY=(--policy yam_scripted); STEPS=1000 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|sequential|one_key|release_early|no_confirm|settle|dream" >&2; exit 2 ;;
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
