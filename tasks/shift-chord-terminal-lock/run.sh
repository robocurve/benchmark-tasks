#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /data/IsaacLab-Arena/yam.env
mode="${1:-demo}"
unset SETTLE_ONLY ABLATION
case "$mode" in
  settle) export SETTLE_ONLY=1; policy=yam_scripted; steps=200 ;;
  demo) policy=yam_scripted; steps=1200 ;;
  ablation) export ABLATION=1; export ABLATION_ARM="${ABLATION_ARM:-left}"; policy=yam_scripted; steps=1200 ;;
  dream) policy=dream; steps=1200 ;;
  *) echo "Use settle, demo, ablation or dream" >&2; exit 2 ;;
esac
out="$PWD/evidence/$mode"
mkdir -p "$out"
export TASK_TRACE="$out/trace.jsonl"
: > "$TASK_TRACE"
: > "$TASK_TRACE.hands.jsonl"
inspect-robots run --embodiment yam_arena --policy "$policy" \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Enter the code on the card and leave the terminal secure. Let go and keep both fingers at least 3 cm from the panel." \
  --scorer task_score --max-steps "$steps" --store-frames --log-dir "$out"
