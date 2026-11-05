#!/usr/bin/env bash
# Runs one trial of the star-pattern cold-plate bolt torque task in the simulator.
#
#   ./run.sh demo         the scripted demo (both arms in star order), reaches 1.0
#   ./run.sh settle       parts rest for a few seconds, checks stability (scores 0.0)
#   ./run.sh adjacent     presses neighbouring corners in a row (latches WARP, scores 0.0)
#   ./run.sh one_arm      left arm presses only its 2 bolts twice (latches WARP by LEVEL, scores 0.0)
#   ./run.sh no_star      right stage levels but wrong within-pass order (latches WARP, scores 0.0)
#   ./run.sh dream        Dream-1 baseline policy
#
# Output goes to $OUT/<mode> (default ~/yam-eval): log, frames, and trace.jsonl.
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Tighten the cold plate's four white bolt buttons in three passes, pressing each bolt once per pass with diagonal corners pressed consecutively and no bolt more than one stage ahead. The large status lamp turns green when all four bolts reach stage 3 without plate warp."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle)   export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo)     POLICY=(--policy yam_scripted); STEPS=1800 ;;
  adjacent) export DEMO_ADJACENT=1; POLICY=(--policy yam_scripted); STEPS=500 ;;
  one_arm)  export DEMO_ONE_ARM=1; POLICY=(--policy yam_scripted); STEPS=800 ;;
  no_star)  export DEMO_NO_STAR=1; POLICY=(--policy yam_scripted); STEPS=1800 ;;
  dream)    POLICY=(--policy dream); STEPS=2400 ;;
  *) echo "usage: $0 demo|settle|adjacent|one_arm|no_star|dream" >&2; exit 2 ;;
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
