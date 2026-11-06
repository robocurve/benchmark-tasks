#!/usr/bin/env bash
# Runs one trial of the backlash dial setpoint task in the simulator.
#
#   ./run.sh demo         the scripted bimanual demo (overshoot & reverse on right dial), predicted 1.00
#   ./run.sh naive        single push to nominal target without reversal (backlash uncompensated), predicted 0.50
#   ./run.sh no_lock      both dials positioned in band but neither locked, predicted 0.25
#   ./run.sh settle       parts rest at start, arms hold at home for 8 s, predicted 0.00
#   ./run.sh dream        Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Adjust both sliding knobs until the white marker lamps on their indicator bars light up, then press the yellow LOCK button beside each rail."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=900 ;;
  naive) export DEMO_NAIVE=1; POLICY=(--policy yam_scripted); STEPS=750 ;;
  no_lock) export DEMO_NO_LOCK=1; POLICY=(--policy yam_scripted); STEPS=650 ;;
  dream) POLICY=(--policy dream); STEPS=2400 ;;
  *) echo "usage: $0 demo|naive|no_lock|settle|dream" >&2; exit 2 ;;
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
