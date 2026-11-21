#!/bin/bash
# One simulated trial on the GPU machine. Usage: bash dev/run.sh <log name> <policy> <seed> <max steps> [ENV=VAL ...]
# Logs go to ~/yam-eval/canning-jar-lid-press-<log name>/ (run.out, debug.log, the inspect-robots JSON). It refuses
# to overwrite a finished run of the same name, so no run is started twice. FRAMES=1: store frames (for a video).
set -u
source /data/IsaacLab-Arena/yam.env
export PATH=/data/IsaacLab-Arena/.venv/bin:$HOME/.local/bin:$PATH
TASK=$(cd "$(dirname "$0")/.." && pwd)
name=$1 policy=$2 seed=$3 steps=$4; shift 4
dir=$HOME/yam-eval/canning-jar-lid-press-$name
if [ -e "$dir/run.out" ]; then echo "refusing: $dir already holds a run"; exit 2; fi
mkdir -p "$dir"
# the GPU is shared: wait until 8 GB of its memory is free
while [ "$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)" -lt 8000 ]; do
  echo "$(date -u +%H:%M:%S) waiting for 8 GB of free GPU memory"; sleep 20
done
cd "$TASK" || exit 1
sha256sum "$TASK"/*.py > "$dir/sha256.txt"
extra=()
if [ "${FRAMES:-0}" = "1" ]; then extra+=(--store-frames); fi
if [ "$policy" = "yam_scripted" ]; then extra+=(-P hold_s=${HOLD_S:-3}); fi
if [ "$policy" = "dream" ]; then extra+=(-P server_url=http://127.0.0.1:8211); fi
echo "=== $(date -u +%H:%M:%S) canning-jar-lid-press-$name ($policy, seed $seed, $steps steps) $*" | tee "$dir/started.txt"
printf 'n\n\n' | env "$@" JARS_DEBUG="$dir/debug.log" inspect-robots run --embodiment yam_arena \
  -E task=canning_jars.py -E object_jitter=0 -E control_hz=20 -E cam_width=${CAM_W:-640} -E cam_height=${CAM_H:-360} \
  --instruction "These three jars of tomato sauce were canned yesterday and have cooled overnight. Put them away for the winter. The wooden shelf is the pantry shelf." \
  --scorer task_score --max-steps "$steps" --policy "$policy" "${extra[@]}" --seed "$seed" \
  --log-dir "$dir" > "$dir/run.out" 2>&1
echo "exit $? score $(grep -h 'task_score:' "$dir/run.out" | tail -1 | awk '{print $2}')" | tee "$dir/finished.txt"
grep -h "jardemo:\|Error\|error" "$dir/run.out" | tail -6
