#!/usr/bin/env bash
# Runs one trial of the hot-swap-failed-drive-sled task in the simulator.
#
#   ./run.sh demo             Correct run, predicted 1.00
#   ./run.sh no_latch         Sled pulled without holding the latch
#   ./run.sh wrong_bay        Wrong bay swapped
#   ./run.sh wrong_spare      Wrong spare inserted
#   ./run.sh release_early    Latch released early
#   ./run.sh settle           Parts rest, arms hold at home 8 s, predicted 0.00
#   ./run.sh dream            Dream-1 baseline policy, predicted 0.00
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="One bay has a red lamp. Replace its drive sled with the spare that matches the bay's key tab, and put the removed sled on the discard pad, without moving the other two sleds."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=2200 ;;
  no_latch) export DEMO_NO_LATCH=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  wrong_bay) export DEMO_WRONG_BAY=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  wrong_spare) export DEMO_WRONG_SPARE=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  release_early) export DEMO_RELEASE_EARLY=1; POLICY=(--policy yam_scripted); STEPS=1200 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|no_latch|wrong_bay|wrong_spare|release_early|settle|dream" >&2; exit 2 ;;
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
