#!/bin/bash
# One simulated trial on the GPU machine. Usage: bash dev/run.sh <log name> <policy> <seed> <max steps> [ENV=VAL ...]
# Logs go to ~/yam-eval/cuffs-<log name>/ (run.out, debug.log, the inspect-robots JSON). It refuses to overwrite a
# finished run of the same name, so no run is started twice.
set -u
source /data/IsaacLab-Arena/yam.env
export PATH=/data/IsaacLab-Arena/.venv/bin:$HOME/.local/bin:$PATH
TASK=$(cd "$(dirname "$0")/.." && pwd)
name=$1 policy=$2 seed=$3 steps=$4; shift 4
dir=$HOME/yam-eval/cuffs-$name
if [ -e "$dir/run.out" ]; then echo "refusing: $dir already holds a run"; exit 2; fi
mkdir -p "$dir"
# the GPU is shared: wait until 12 GB of its memory is free
while [ "$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)" -lt 12000 ]; do
  echo "$(date -u +%H:%M:%S) waiting for 12 GB of free GPU memory"; sleep 20
done
cd "$TASK" || exit 1
sha256sum "$TASK"/*.py > "$dir/sha256.txt"
extra=()
if [ "${FRAMES:-1}" = "1" ]; then extra+=(--store-frames); fi  # FRAMES=0: no stored frames (no video needed)
if [ "$policy" = "yam_scripted" ]; then extra+=(-P hold_s=${HOLD_S:-3}); fi
if [ "$policy" = "dream" ]; then extra+=(-P server_url=http://127.0.0.1:8211); fi
echo "=== $(date -u +%H:%M:%S) cuffs-$name ($policy, seed $seed, $steps steps) $*" | tee "$dir/started.txt"
printf 'n\n\n' | env "$@" CUFF_DEBUG="$dir/debug.log" inspect-robots run --embodiment yam_arena \
  -E task=wrist_form_cuffs.py -E object_jitter=0 -E control_hz=20 -E cam_width=${CAM_W:-640} -E cam_height=${CAM_H:-360} \
  --instruction "Put one handcuff round each wrist form. Close each cuff so a 6 mm rod still fits between cuff and form but a 12 mm rod does not. Double-lock both cuffs. Then move both grippers clear." \
  --scorer task_score --max-steps "$steps" --policy "$policy" "${extra[@]}" --seed "$seed" \
  --log-dir "$dir" > "$dir/run.out" 2>&1
echo "exit $? score $(grep -h 'task_score:' "$dir/run.out" | tail -1 | awk '{print $2}')" | tee "$dir/finished.txt"
grep -h "CUFF PROBE\|cuffdemo:" "$dir/run.out" | tail -5
