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
    export ABLATION=idle
    policy=yam_scripted
    steps=900
    ;;
  noguard)
    export ABLATION=noguard
    policy=yam_scripted
    steps=900
    ;;
  posecheck)
    export POSE_CHECK=1
    policy=yam_scripted
    steps=640
    ;;
  dream)
    policy=dream
    steps=1500
    ;;
  *)
    echo "Unknown mode: $mode (use settle, demo, ablation, noguard, posecheck, dream)" >&2
    exit 2
    ;;
esac

out="$PWD/evidence/$mode"
mkdir -p "$out"
export TASK_TRACE="$out/trace.jsonl"
: > "$TASK_TRACE"

inspect-robots run --embodiment yam_arena --policy "$policy" \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the fry basket out of its outer basket: slide the button guard forward, hold the release button on the handle while lifting by the grip, set the fry basket flat on the mat, and let go of everything; touch nothing hot." \
  --scorer task_score --max-steps "$steps" --store-frames --log-dir "$out"
