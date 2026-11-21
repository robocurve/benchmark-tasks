#!/usr/bin/env bash
set -euo pipefail
steps=${1:?Expected control step limit}
out=${2:?Expected new output directory}
if [[ ! "$steps" =~ ^[1-9][0-9]*$ ]] || (( steps > 1200 )); then
  exit 2
fi
mkdir -- "$out"
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export TOWEL_TRACE="$out/trace.jsonl"
sha256sum "$task_dir"/*.py "$task_dir"/task.json > "$out/source.sha256"
timeout -k 15 1200 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$task_dir/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps "$steps" \
  --store-frames --log-dir "$out/inspect" --scorer task_score \
  --instruction 'Start the new roll in the open paper-towel dispenser. Feed a straight, unbroken tail through the outlet and leave the lever released.'
