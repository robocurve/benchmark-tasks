#!/bin/bash
# Dev only: one simulator trial of the scripted demo. dev/run.sh <label> [max_steps]; variants and tuning via PLASMA_* env vars.
set -u
cd "$(dirname "$0")/.."
source ~/IsaacLab-Arena/yam.env
[[ -e /lib/aarch64-linux-gnu/libgomp.so.1 ]] && export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 PYTORCH_JIT=0  # aarch64 hosts only
LOCK=${SIM_LOCK:-/tmp/plasma-sim.lock}
OUT=~/yam-eval/plasma-$1
mkdir -p "$OUT"
flock "$LOCK" inspect-robots run --embodiment yam_plasma -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=${CAM_W:-320} -E cam_height=${CAM_H:-176} --policy yam_scripted --no-prompt \
  --instruction "Cut the overhanging strip off the plate along the white line with the plasma torch, guiding the torch along the straightedge, then put the torch and the straightedge back where they were and move both hands away." \
  --scorer task_score --max-steps "${2:-2400}" --seed 0 --store-frames --log-dir "$OUT" > "$OUT/stdout.log" 2>&1
echo "exit $?" >> "$OUT/stdout.log"
grep -E "task_score|Traceback|Error" "$OUT/stdout.log" | tail -5
