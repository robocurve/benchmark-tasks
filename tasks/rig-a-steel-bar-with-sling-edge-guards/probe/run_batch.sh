#!/usr/bin/env bash
# Repeats and controls for check 2 on one USD (default sling_probe.usda). Four Isaac Sim processes at a time.
# Usage: bash run_batch.sh <tag> [usd]
set -u
TAG=${1:-final}
USD=${2:-sling_probe.usda}
cd "$(dirname "$0")"
source ~/IsaacLab-Arena/yam.env
run() {  # name, args...
  local name=$1; shift
  python run_probe.py --usd "$USD" --out "out/$TAG/$name" "$@" > "out/$TAG/$name.log" 2>&1
}
mkdir -p "out/$TAG"
md5sum "$USD" sling_model.py run_probe.py build_probe_usd.py > "out/$TAG/inputs.md5"
run guards-a --case guards &
run guards-b --case guards &
run guards-swing+3 --case guards --swing 3 &
run guards-swing-3 --case guards --swing -3 &
wait
run guards-slow --case guards --speed 0.015 &
run guards-fast --case guards --speed 0.045 &
run noguards-a --case noguards &
run noguards-swing+3 --case noguards --swing 3 &
wait
run noguards-swing-3 --case noguards --swing -3 &
run bottomless-a --case bottomless &
run guards-weakmag --case guards --mag-f0 2.0 &
wait
echo BATCH-DONE
