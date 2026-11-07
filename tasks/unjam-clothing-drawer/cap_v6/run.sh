#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODE=${1:-cooperate}
QUALITY=${2:-preview}
if [[ "$QUALITY" != preview && "$QUALITY" != capture ]]; then
  printf 'Quality must be preview or capture\n' >&2
  exit 2
fi
if [[ "$MODE" != reach && "$MODE" != cooperate ]]; then
  printf 'Usage: bash run.sh [reach|cooperate]\n' >&2
  exit 2
fi
python "$HERE/../install_scene_hook.py" --check
python "$HERE/../install_hq_cameras.py" --check
OUT="$HOME/yam-eval/drawer-cap-v6/$(date -u +%Y%m%dT%H%M%S)-$MODE-$$"
mkdir -p "$(dirname "$OUT")"
printf 'Output: %s\n' "$OUT"
OPENBLAS_NUM_THREADS=1 timeout --signal=INT --kill-after=30s 20m \
 python -u "$HERE/run_episode.py" --mode "$MODE" --quality "$QUALITY" --output "$OUT" 2>&1 | tee "$OUT.console.txt"
