#!/usr/bin/env bash
# Runs one trial of the thermal soak task in the simulator.
#
#   ./run.sh demo         the scripted bimanual demo (learns lag on sample 1, hits 60-70 band on sample 2), predicted 1.0
#   ./run.sh naive        lifts sample 2 when bar shows 6 discs (core short of 60-70 band), predicted 0.75
#   ./run.sh overshoot    leaves sample 2 on plate until core > 90 C (trips FIRE zero-gate), predicted 0.0
#   ./run.sh no_sample1   skips sample 1 and heats sample 2 directly using study value, predicted 0.75
#   ./run.sh settle       parts rest for a few seconds, checks stability, predicted 0.0
#   ./run.sh dream        Dream-1 baseline policy, predicted 0.0
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Press the red HEAT button on the front left to heat the plate to its 110 deg C setpoint, soak the orange sample so its core ends between 60 and 70 deg C, then leave it on the cool pad and press the white OFF button on the front right. While a sample rests on the cool pad, the status lamp indicates its core temperature band (amber < 60, green 60-70, red > 70 deg C) and the bar shows surface temperature; the blue expendable sample may be used to gauge the thermal lag."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_HOLD_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=2000 ;;
  naive) export DEMO_NAIVE=1; POLICY=(--policy yam_scripted); STEPS=2000 ;;
  overshoot) export DEMO_OVERSHOOT=1; POLICY=(--policy yam_scripted); STEPS=2500 ;;
  no_sample1) export DEMO_NO_SAMPLE1=1; POLICY=(--policy yam_scripted); STEPS=1600 ;;
  dream) POLICY=(--policy dream); STEPS=3000 ;;
  *) echo "usage: $0 demo|naive|overshoot|no_sample1|settle|dream" >&2; exit 2 ;;
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
