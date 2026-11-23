#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
phase="${1:-help}"
tag="${2:-bedside-review}"
case "$phase" in
  setup)
    python3 -m venv .venv
    .venv/bin/python -m pip install -r runtime/requirements.txt
    exit ;;
  station-*)
    arena="${YAM_ARENA_DIR:-$HOME/research/robocurve/IsaacLab-Arena-YAM}"
    if [[ -f "$arena/yam.env" ]]; then set +u; source "$arena/yam.env"; set -u; fi
    if [[ -e "$arena/.venv/bin/activate" ]]; then source "$arena/.venv/bin/activate"; fi
    py="$(command -v python3)"
    if ! "$py" -c 'import isaaclab,yam_arena,inspect_robots' 2>/dev/null; then echo "Activate the HPC01 IsaacLab-Arena-YAM environment or set YAM_ARENA_DIR." >&2; exit 2; fi
    if ! "$py" -c 'import mujoco; assert mujoco.__version__=="3.3.7"' 2>/dev/null; then
      deps="$PWD/runtime/station-deps"
      export PYTHONPATH="$deps${PYTHONPATH:+:$PYTHONPATH}"
      if ! "$py" -c 'import mujoco; assert mujoco.__version__=="3.3.7"' 2>/dev/null; then
        taskpy="$PWD/.venv/bin/python"
        if [[ ! -x "$taskpy" ]]; then
          echo "Run bash reproduce.sh setup to create the task environment." >&2
          exit 2
        fi
        exec "$taskpy" -I runtime/station_geometry.py "$py" "${phase#station-}" "$tag"
      fi
    fi
    exec "$py" -m isaac.qualify "${phase#station-}" "$tag" ;;
  *) echo "Usage: bash reproduce.sh setup | station-framing | station-native | station-dream | station-all | station-qualify [RUN_TAG]" >&2; exit 2 ;;
esac
