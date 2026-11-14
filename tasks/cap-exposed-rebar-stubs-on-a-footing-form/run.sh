#!/usr/bin/env bash
# Runs one trial of cap-exposed-rebar-stubs-on-a-footing-form in the yam_arena simulator.
#   ./run.sh dream     the Dream-1 policy, 2,400 steps (expected 0.0)
#   ./run.sh demo      the scripted demo (python demo.py), expected 1.0
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
MODE="${1:-dream}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Make the rebar stubs safe for the crew to walk the formwork."
command -v inspect-robots >/dev/null 2>&1 || source "${ARENA_ENV:-/data/IsaacLab-Arena/yam.env}" >/dev/null 2>&1
cd "$(dirname "$0")"
case "$MODE" in
  dream) exec inspect-robots run --embodiment yam_arena --policy dream \
    -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir "$OUT/$MODE" ;;
  demo|one_cap|two_caps|off_target|stay|settle)
    exec python demo.py task.py "$OUT/$MODE" --mode "$MODE" ;;
  *) echo "unknown mode $MODE"; exit 2 ;;
esac
