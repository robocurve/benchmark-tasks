#!/usr/bin/env bash
set -euo pipefail
if ! command -v inspect-robots >/dev/null; then
  source "$HOME/IsaacLab-Arena/yam.env"
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
task_dir=$(cd "$(dirname "$0")" && pwd)
run_tag=${1:-demo}
inspect-robots run --embodiment yam_retrieval --policy yam_retrieval_scripted \
  -E task="$task_dir/task.py" -E object_jitter=0 -E device=cpu \
  -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction 'Use the spatula to pull the blue part out from under the grey guard. Put the part in the green tray, return the spatula to its two supports, and release both objects.' \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames \
  --log-dir "${RETRIEVAL_LOG_ROOT:-$HOME/yam-eval/tool-retrieval}/$run_tag"
