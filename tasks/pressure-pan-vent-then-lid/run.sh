#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /data/IsaacLab-Arena/yam.env
mode="${1:-demo}"

case "$mode" in
  settle)
    export SETTLE_ONLY=1
    policy=yam_scripted
    steps=200
    ;;
  demo)
    policy=yam_scripted
    steps=1500
    ;;
  ablation)
    export ABLATION=1
    policy=yam_scripted
    steps=1200
    ;;
  dream)
    policy=dream
    steps=2400
    ;;
  *)
    echo "Unknown mode: $mode (use settle, demo, ablation, dream)" >&2
    exit 2
    ;;
esac

out="$PWD/evidence/$mode"
mkdir -p "$out"
export TASK_TRACE="$out/trace.jsonl"
: > "$TASK_TRACE"

inspect-robots run --embodiment yam_arena --policy "$policy" \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The beans are done. Take the lid off the pressure pan and set it flat on the grey trivet, then let go and keep both fingers at least 3 cm from the lid and the pan." \
  --scorer task_score --max-steps "$steps" --store-frames --log-dir "$out"
