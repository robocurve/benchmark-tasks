#!/usr/bin/env bash
# Runs one trial of the clear-lane-then-push-block-to-dock task in the simulator.
#
#   ./run.sh demo          Clear loose blocks and push cargo to dock, predicted 1.00
#   ./run.sh push_first    Pushes cargo into blockage without clearing lane, predicted 0.00
#   ./run.sh hit_post      Veers into fragile red post during push, predicted 0.25
#   ./run.sh stop_halfway  Clears lane but pushes cargo only halfway, predicted 0.50
#   ./run.sh settle        Both arms idle at home, predicted 0.00
#   ./run.sh dream         Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Move the two loose blocks out of the lane, then push the cargo block along the lane to the dock without touching the red post."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=10); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  push_first) export DEMO_PUSH_FIRST=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  hit_post) export DEMO_HIT_POST=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  stop_halfway) export DEMO_STOP_HALFWAY=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  dream) export DEMO_DREAM=1; POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|push_first|hit_post|stop_halfway|settle|dream" >&2; exit 2 ;;
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
