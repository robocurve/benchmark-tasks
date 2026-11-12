#!/usr/bin/env bash
# Runs one trial of the spring-drawer-hold-and-retrieve task in the simulator.
#
#   ./run.sh demo             Correct run, predicted 1.00
#   ./run.sh no_hold          Drawer not held open
#   ./run.sh pinch            Block pinched through the closing drawer
#   ./run.sh leave_open       Drawer left open at the end
#   ./run.sh settle           Parts rest, arms hold at home 8 s, predicted 0.00
#   ./run.sh dream            Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Take the white block out of the cabinet and put it on the green shelf, without letting the drawer close on anything, then leave the drawer closed."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=2200 ;;
  no_hold) export DEMO_NO_HOLD=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  pinch) export DEMO_PINCH=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  leave_open) export DEMO_LEAVE_OPEN=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|no_hold|pinch|leave_open|settle|dream" >&2; exit 2 ;;
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
