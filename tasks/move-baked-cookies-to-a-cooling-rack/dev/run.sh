#!/usr/bin/env bash
# Dev runner (created 11_15_26): dev/run.sh <label> <max-steps> [policy]; env COOKIE_* switches pass through. One simulator at a time.
set -euo pipefail
cd "$(dirname "$0")/.."
source ~/IsaacLab-Arena/yam.env
export PYTORCH_JIT=0
[[ $(uname -m) == aarch64 ]] && export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
LOCK=${SIM_LOCK:-$HOME/sim.lock}
INSTR="Move the four cookies from the sheet pan onto the cooling rack with the spatula, without touching or breaking them, then put the spatula back on its rest."
flock "$LOCK" inspect-robots run --embodiment yam_cookie -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=${CAM_W:-320} -E cam_height=${CAM_H:-176} --policy "${3:-yam_scripted}" --no-prompt --instruction "$INSTR" \
  --scorer task_score --max-steps "$2" --seed 0 --store-frames --log-dir ~/yam-eval/cookie-rack-"$1"
