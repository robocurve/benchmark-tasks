#!/usr/bin/env bash
# Runs one trial of the domino-chain-set-and-tip task in the simulator.
#
#   ./run.sh demo            Correct bimanual placement and toppling, predicted 1.00
#   ./run.sh gap_too_wide    Domino 3 placed with gap too wide, predicted 0.75
#   ./run.sh tip_too_early   First domino tipped before second arm places, predicted 0.25
#   ./run.sh no_tip          Four dominoes placed standing on marks without tip, predicted 0.50
#   ./run.sh settle          Arms hold at home 8 s, predicted 0.00
#   ./run.sh dream           Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Stand the four dominoes on end on the white marks 30 mm apart, then tip the first one so every domino falls."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1600 ;;
  gap_too_wide) export DEMO_GAP_TOO_WIDE=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  tip_too_early) export DEMO_TIP_TOO_EARLY=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  no_tip) export DEMO_NO_TIP=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|gap_too_wide|tip_too_early|no_tip|settle|dream" >&2; exit 2 ;;
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
