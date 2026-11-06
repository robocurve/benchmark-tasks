#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /data/IsaacLab-Arena/yam.env
mode="${1:-demo}"
case "$mode" in
  demo) policy=yam_scripted; steps=720;;
  dream) policy=dream; steps=2400;;
  *) exit 2;;
esac
out="$PWD/evidence/$mode"
mkdir -p "$out"
export TASK_TRACE="$out/trace.jsonl"
: > "$TASK_TRACE"
inspect-robots run --embodiment yam_arena --policy "$policy" \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the blue GAP tab and orange LOCK tab toward the rail to their end marks. Move the carriage to the far end and back. Hold the green CAL button for half a second and release it." \
  --scorer task_score --max-steps "$steps" --store-frames --log-dir "$out"
