#!/bin/bash
# Run one trial of the angle-grinder task with inspect-robots, from this folder, in a shell with the yam_arena
# environment active (on the GPU box: `source /data/IsaacLab-Arena/yam.env`).
#
#   ./run.sh dream          Dream-1, 2,400 steps (expected score 0)
#   ./run.sh demo           the scripted full-score demo (expected 1.0)
#   ./run.sh settle         the grinder held, nothing else done (expected 0)
#   ./run.sh no_unlock      tries to turn the guard with the lever shut (expected 0)
#   ./run.sh too_little     guard turned two thirds of the way, lever shut (expected 0.50)
#   ./run.sh wrong_way      guard turned 60 degrees the wrong way (measured 0.25: the lever ends raised)
#   ./run.sh forget_lock    guard turned right, lever left open (expected 0.50)
#   ./run.sh touch_disc     everything right after a fingertip is pressed on the disc (expected 0.25)
#   TASK_VARIANT=B ./run.sh demo      the mirrored second layout (the layout of the setup image is A)
#   OUT=~/yam-eval ./run.sh demo      where the logs and frames go (default ~/yam-eval)
# Every mode is a real `inspect-robots run`; the scripted ones play the task's demo() with --policy yam_scripted.
set -euo pipefail
MODE="${1:-dream}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Make the angle grinder safe for the operator standing at the marked spot."
command -v inspect-robots >/dev/null 2>&1 || source "${ARENA_ENV:-/data/IsaacLab-Arena/yam.env}" >/dev/null 2>&1
cd "$(dirname "$0")"
VARIANT="${TASK_VARIANT:-A}"
export TASK_VARIANT="$VARIANT"
case "$MODE" in
  dream) POLICY=(--policy dream) ;;
  demo|settle|no_unlock|too_little|wrong_way|forget_lock|touch_disc)
    export DEMO_MODE="$MODE"; POLICY=(--policy yam_scripted -P hold_s=3) ;;
  *) echo "unknown mode $MODE: dream demo settle no_unlock too_little wrong_way forget_lock touch_disc" >&2; exit 2 ;;
esac
mkdir -p "$OUT/$MODE-$VARIANT"
exec inspect-robots run --embodiment yam_arena "${POLICY[@]}" \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir "$OUT/$MODE-$VARIANT"
