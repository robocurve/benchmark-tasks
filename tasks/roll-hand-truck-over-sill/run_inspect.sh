#!/usr/bin/env bash
set -euo pipefail
: "${ISAAC_PYTHON:?Set ISAAC_PYTHON to the installed IsaacLab environment Python}"
: "${INSPECT_OVERLAY:?Set INSPECT_OVERLAY to the isolated Inspect packages}"
: "${ARENA_ROOT:?Set ARENA_ROOT to the pinned Arena checkout}"
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PYTHONPATH="$INSPECT_OVERLAY:$ARENA_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
if [[ $(uname -m) == aarch64 ]]; then
  export LD_PRELOAD="/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
fi
exec timeout -k 15 2400 "$ISAAC_PYTHON" "$task_dir/run_inspect.py" "$@"
