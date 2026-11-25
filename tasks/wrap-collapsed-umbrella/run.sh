#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
policy="${1:-yam_scripted}"
case "$policy" in
  yam_scripted|dream) ;;
  *) printf 'Usage: %s [yam_scripted|dream]\n' "$0" >&2; exit 2 ;;
esac
output="${2:-$PWD/runs/$policy}"
instruction=$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')
exec inspect-robots run --embodiment yam_arena --policy "$policy" \
  -E "task=$PWD/umbrella_task.py" -E object_jitter=0 -E device=cuda:0 -E control_hz=20 \
  -E cam_width=480 -E cam_height=272 \
  --instruction "$instruction" \
  --scorer task_score --max-steps 1800 --store-frames --log-dir "$output"
