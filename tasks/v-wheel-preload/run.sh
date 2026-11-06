#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /data/IsaacLab-Arena/yam.env
mode="${1:-demo}"
case "$mode" in
  settle) export SETTLE_ONLY=1; policy=yam_scripted; steps=160;;
  demo) policy=yam_scripted; steps=700;;
  ablation) export SKIP_SIDE=1; policy=yam_scripted; steps=700;;
  dream) policy=dream; steps=2400;;
  *) exit 2;;
esac
out="$PWD/evidence/$mode"
mkdir -p "$out"
export TASK_TRACE="$out/trace.jsonl"
: > "$TASK_TRACE"
inspect-robots run --embodiment yam_arena --policy "$policy" \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the blue preload tab and the orange lock tab toward the rail. Hold the green side-load cap with the carriage untouched, then release it. Push the carriage along the rail from its green start marker to its amber end marker." \
  --scorer task_score --max-steps "$steps" --store-frames --log-dir "$out"
