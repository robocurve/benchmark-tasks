#!/usr/bin/env bash
# Runs one trial of the bridge-plate-across-trench task in the simulator.
#
#   ./run.sh demo                  Correct bridge placement + push, predicted 1.00
#   ./run.sh no_bridge_push        Push block into trench without bridge, predicted 0.00
#   ./run.sh bridge_short_overlap  Bridge placed with short overlap tips into trench, predicted 0.00
#   ./run.sh skip_target           Push block onto bridge only, predicted 0.50
#   ./run.sh settle                Arms hold at home 8 s, predicted 0.00
#   ./run.sh dream                 Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Lay the bridge plate across the trench, then push the block over it onto the far platform without letting the plate or the block fall into the trench."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  no_bridge_push) export DEMO_NO_BRIDGE_PUSH=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  bridge_short_overlap) export DEMO_BRIDGE_SHORT_OVERLAP=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  skip_target) export DEMO_SKIP_TARGET=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|no_bridge_push|bridge_short_overlap|skip_target|settle|dream" >&2; exit 2 ;;
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
