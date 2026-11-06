#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /data/IsaacLab-Arena/yam.env
mode="${1:-demo}"
case "$mode" in
  settle) export SETTLE_ONLY=1; policy=yam_scripted; steps=160;;
  demo) policy=yam_scripted; steps=760;;
  ablation) export SKIP_APERTURE=1; policy=yam_scripted; steps=760;;
  dream) policy=dream; steps=2400;;
  *) exit 2;;
esac
out="$PWD/evidence/$mode"
mkdir -p "$out"
export TASK_TRACE="$out/trace.jsonl"
: > "$TASK_TRACE"
inspect-robots run --embodiment yam_arena --policy "$policy" \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Slide the two blue focus and aperture tabs toward the rail, then slide the orange lens lock. Press the green TEST cap and clear both arms from the chart carriage and camera." \
  --scorer task_score --max-steps "$steps" --store-frames --log-dir "$out"
