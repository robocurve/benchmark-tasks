#!/bin/bash
# One simulated trial on the GPU machine. Usage: bash dev/run.sh <log name> <policy> <seed> <max steps> [ENV=VAL ...]
# Logs go to ~/yam-eval/golf-ball-on-tee-<log name>/ (run.out, debug.log, the inspect-robots JSON). It refuses to
# overwrite a finished run of the same name, so no run is started twice.
set -u
source /data/IsaacLab-Arena/yam.env
export PATH=/data/IsaacLab-Arena/.venv/bin:$HOME/.local/bin:$PATH
TASK=$(cd "$(dirname "$0")/.." && pwd)
name=$1 policy=$2 seed=$3 steps=$4; shift 4
dir=$HOME/yam-eval/golf-ball-on-tee-$name
if [ -e "$dir/run.out" ]; then echo "refusing: $dir already holds a run"; exit 2; fi
mkdir -p "$dir"
# the GPU is shared: start only while less than 36,000 MiB of its memory is in use (one simulation uses about 5 GB)
while [ "$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)" -ge 36000 ]; do
  echo "$(date -u +%H:%M:%S) waiting: GPU memory in use is 36,000 MiB or more"; sleep 20
done
cd "$TASK" || exit 1
sha256sum "$TASK"/*.py > "$dir/sha256.txt"
extra=()
if [ "${FRAMES:-0}" = "1" ]; then extra+=(--store-frames); fi  # FRAMES=1 only for the runs that become videos
if [ "$policy" = "yam_scripted" ]; then extra+=(-P hold_s=${HOLD_S:-1}); fi
if [ "$policy" = "dream" ]; then extra+=(-P server_url=http://127.0.0.1:8211); fi
echo "=== $(date -u +%H:%M:%S) golf-ball-on-tee-$name ($policy, seed $seed, $steps steps) $*" | tee "$dir/started.txt"
printf 'n\n\n' | env "$@" GOLF_DEBUG="$dir/debug.log" inspect-robots run --embodiment yam_arena \
  -E task=golf_tee.py -E object_jitter=0 -E control_hz=20 -E cam_width=${CAM_W:-640} -E cam_height=${CAM_H:-360} \
  --instruction "Push the tee into the rubber holder, then balance the golf ball on the tee." \
  --scorer task_score --max-steps "$steps" --policy "$policy" "${extra[@]}" --seed "$seed" \
  --log-dir "$dir" > "$dir/run.out" 2>&1
echo "exit $? score $(grep -h 'task_score:' "$dir/run.out" | tail -1 | awk '{print $2}')" | tee "$dir/finished.txt"
grep -h "golfdemo:" "$dir/run.out" | tail -3
grep -h "score report" "$dir/debug.log" 2>/dev/null | tail -1 | cut -c1-700
