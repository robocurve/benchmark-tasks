#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-demo}"
case "$MODE" in
  dream) POLICY=dream ;;
  demo|settle|no_board) POLICY=yam_scripted; export DEMO_MODE="$MODE" ;;
  *) echo "Unknown mode: $MODE" >&2; exit 2 ;;
esac
if [[ -f /data/IsaacLab-Arena/yam.env ]]; then source /data/IsaacLab-Arena/yam.env; fi
OUT="${OUT:-$HOME/yam-eval/$(basename "$HERE")/$MODE}"
mkdir -p "$OUT"
INSTRUCTION="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["instruction"])' "$HERE/task.json")"
inspect-robots run --embodiment yam_arena --policy "$POLICY" \
  -E "task=$HERE/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps "${STEPS:-2500}" --store-frames --log-dir "$OUT"
