#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")" && pwd)"
mode="${1:?choose pilot, demo or dream}"
source "${YAM_ENV:-/data/IsaacLab-Arena/yam.env}"
export PYTHONPATH="$root/runtime:$root/policy:$root:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
case "$mode" in
  pilot) policy=(--policy yam_scripted); steps=900; width=320; height=180 ;;
  demo) policy=(--policy yam_scripted); steps=2400; width=1280; height=720 ;;
  dream) policy=(--policy dream_current -P "server_url=http://127.0.0.1:${DREAM_BRIDGE_PORT:-8241}" -P rows_per_request=8); steps=2400; width=1280; height=720 ;;
  *) echo "Choose pilot, demo or dream" >&2; exit 2 ;;
esac
dir="$root/runs/$mode/$(date -u +%Y%m%dT%H%M%S%N)"
mkdir -p "$dir"
cd "$dir"
export YAM_TELEMETRY="$dir/trajectory.jsonl"
python "$root/evidence.py" snapshot "$dir/sources.json"
inspect-robots run --embodiment snack_yam "${policy[@]}" -E "task=$root/task.py" \
  -E object_jitter=0 -E control_hz=20 -E cam_width=$width -E cam_height=$height \
  --instruction "Take one chip out of the open packet and put it on the plate. Leave the other two chips inside the packet and both hands clear." \
  --seed 0 --epochs 1 --scorer task_score --max-steps "$steps" --store-frames --log-dir "$dir"

