#!/usr/bin/env bash
# Runs one trial of the coupled knob peak tuning task in the simulator.
#
#   ./run.sh demo         the scripted bimanual demo (5 readings along the ridge), predicted 1.0
#   ./run.sh recover      as demo, with a deliberate 8 mm overshoot of the left knob pulled back (backward push), predicted 1.0
#   ./run.sh settle       parts rest for a few seconds, checks stability (predicted 0.0)
#   ./run.sh axis_search  scripted single-axis search, 8 readings then a last move (predicted 0.75, variant 0)
#   ./run.sh no_read      moves directly to the variant-0 peak without readings (predicted 0.50)
#   ./run.sh memorize     3 readings and the hard-coded variant-0 peak (1.0 on variant 0; run with VARIANT=1 or 2 to see it fail)
#   ./run.sh dream        Dream-1 baseline policy
#
# VARIANT=0|1|2 selects the hidden peak (README.md, default 0). Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Press the white READ button between the two tracks to show the red needle for the current positions of the orange (left) and teal (right) sliders; use at most eight READ presses, including at least three at slider positions 10 mm or more apart. Then set both sliders so the needle stands on the green band at the end of the scale, and let go."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1000 ;;
  recover) export DEMO_RECOVER=1; POLICY=(--policy yam_scripted); STEPS=1100 ;;
  axis_search) export DEMO_AXIS_SEARCH=1; POLICY=(--policy yam_scripted); STEPS=1250 ;;
  no_read) export DEMO_NO_READ=1; POLICY=(--policy yam_scripted); STEPS=400 ;;
  memorize) export DEMO_MEMORIZE=1; POLICY=(--policy yam_scripted); STEPS=650 ;;
  dream) POLICY=(--policy dream); STEPS=2400 ;;
  *) echo "usage: $0 demo|recover|settle|axis_search|no_read|memorize|dream" >&2; exit 2 ;;
esac

export TASK_VARIANT="${VARIANT:-0}"
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
