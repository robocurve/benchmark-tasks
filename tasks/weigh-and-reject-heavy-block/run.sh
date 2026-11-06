#!/usr/bin/env bash
# Runs one trial of the weigh-and-reject-heavy-block task in the simulator.
#
#   ./run.sh demo             Correct run, predicted 1.00
#   ./run.sh no_weigh         Heavy block rejected without weighing
#   ./run.sh wrong_pick       Light block put in the reject tray
#   ./run.sh weigh_too_short  Blocks lifted before the scale settles
#   ./run.sh settle           Parts rest, arms hold at home 8 s, predicted 0.00
#   ./run.sh dream            Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Weigh each grey block on the scale, then put only the heavy one into the red reject tray, and leave the other two on the green shelf."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  no_weigh) export DEMO_NO_WEIGH=1; POLICY=(--policy yam_scripted); STEPS=600 ;;
  wrong_pick) export DEMO_WRONG_PICK=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  weigh_too_short) export DEMO_WEIGH_TOO_SHORT=1; POLICY=(--policy yam_scripted); STEPS=600 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|no_weigh|wrong_pick|weigh_too_short|settle|dream" >&2; exit 2 ;;
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
