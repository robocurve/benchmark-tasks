#!/usr/bin/env bash
# Runs one trial of the compress-and-lock-spring-cartridge task in the simulator.
#
#   ./run.sh demo                 Correct run, predicted 1.00
#   ./run.sh order_bar_first_arm  Bar arm starts first, still correct
#   ./run.sh one_arm              Single arm tries both
#   ./run.sh release_early        Plunger released before the bar is across
#   ./run.sh bar_first            Bar pushed before compressing
#   ./run.sh no_release           Plunger held to the end
#   ./run.sh settle               Arms hold at home 8 s, predicted 0.00
#   ./run.sh dream                Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Compress the spring plunger all the way in with one hand and slide the lock bar across with the other, then let go so the cartridge stays compressed."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  order_bar_first_arm) export DEMO_ORDER_BAR_FIRST_ARM=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  one_arm) export DEMO_ONE_ARM=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  release_early) export DEMO_RELEASE_EARLY=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  bar_first) export DEMO_BAR_FIRST=1; POLICY=(--policy yam_scripted); STEPS=1000 ;;
  no_release) export DEMO_NO_RELEASE=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|order_bar_first_arm|one_arm|release_early|bar_first|no_release|settle|dream" >&2; exit 2 ;;
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
