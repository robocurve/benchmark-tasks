#!/usr/bin/env bash
# One run on the Brev instance, detached-friendly: bash dev/run.sh <label> <max_steps> [extra inspect-robots args...]
# e.g. bash dev/run.sh demo-l3 3600 --policy inspection_mirror_scripted -E leak=3   (FRAMES=1 stores frames and builds the video)
set -euo pipefail
source /data/IsaacLab-Arena/yam.env >/dev/null 2>&1 || true
here=$(cd "$(dirname "$0")/.." && pwd)
if [[ ${2:-} == dream ]]; then  # Dream-1: bash dev/run.sh <name> dream <seed> <steps>  -> ~/yam-eval/inspection-mirror-<name>/
  label=$1; seed=$3; steps=$4; shift 4
  out=$HOME/yam-eval/inspection-mirror-$label; rm -rf "$out"
  set -- --policy dream -P server_url=http://127.0.0.1:8211 --seed "$seed" "$@"
else
  label=$1; steps=$2; shift 2
  out=${RUNS_DIR:-${HOME}/inspection-mirror-runs}/$label-$(date -u +%m%d-%H%M%S)  # set RUNS_DIR to a roomier disk if needed
fi
export TMPDIR=${TMPDIR_OVERRIDE:-${HOME}/inspection-mirror-tmp}; mkdir -p "$TMPDIR"
export PYTHONPATH="$here${PYTHONPATH:+:$PYTHONPATH}"
cd "$here"
INSTR=$(python3 -c 'import json; print(json.load(open("task.json"))["instruction"])')
mkdir -p "$out"
inspect-robots run --embodiment yam_inspection_mirror -E object_jitter=0 -E control_hz=20 -E cam_width=320 \
  -E cam_height=176 -E telemetry="$out/telemetry.jsonl" --instruction "$INSTR" --scorer task_score \
  --max-steps "$steps" ${FRAMES:+--store-frames} --log-dir "$out" "$@" 2>&1 | tee "$out/stdout.log" "$out/run.out"
[[ -n ${FRAMES:-} ]] && for log in "$out"/*.json; do inspect-robots video "$log" || true; done
echo "OUT=$out"
