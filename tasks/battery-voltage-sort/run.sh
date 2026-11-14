#!/usr/bin/env bash
# All task code, launch commands and outputs live in this submission folder.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bash tasks/battery-voltage-sort/run.sh MODE [extra arguments]

  check     Check the GPU, installed packages and Inspect Robots registration.
  hold      Load this task and hold the ready pose for 100 actions.
  demo      Run and record this task's local reference demonstration.
  gui       Run and record that demonstration in an Isaac Sim window.
  view      View this task's initial scene until its window is closed.
  start     Start that viewer independently of this terminal; print its PID/log.
  dream     Run the configured hosted Dream-1 policy on this task.
  dream-start Start that model trial independently of this terminal.
  video LOG Render the camera videos from an Inspect Robots JSON log.

Override paths with YAM_SIM_ENV and YAM_SIM_OUTPUT.
EOF
}

mode=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  check|hold|demo|gui|view|start|dream|dream-start|video) ;;
  help|-h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sim_env=${YAM_SIM_ENV:-${VIRTUAL_ENV:-${CONDA_PREFIX:-/data/conda/envs/yam-arena}}}
output_root=${YAM_SIM_OUTPUT:-$task_dir/output}
python=$sim_env/bin/python
inspect=$sim_env/bin/inspect-robots
[[ -x "$python" && -x "$inspect" ]] || {
  printf 'Simulator environment is missing: %s\n' "$sim_env" >&2
  exit 1
}

# ROS on this host injects Python 3.10 packages and system libraries. Isaac Sim
# uses its own Python 3.12 environment and bundled native libraries.
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PATH="$sim_env/bin:$PATH"
export CONDA_PREFIX="$sim_env"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1

if [[ "$mode" == video ]]; then
  [[ $# -eq 1 ]] || { usage >&2; exit 2; }
  exec "$inspect" video "$1"
fi

if [[ "$mode" != check && "${OMNI_KIT_ACCEPT_EULA:-}" != YES && "${ACCEPT_EULA:-}" != Y ]]; then
  printf 'Activate your existing YAM Conda environment, which carries your NVIDIA license settings, before launching.\n' >&2
  exit 1
fi

mkdir -p "$output_root"
run_dir=$(mktemp -d "$output_root/$mode.XXXXXX")
printf 'Run output: %s\n' "$run_dir"

run_logged() {
  "$@" 2>&1 | tee "$run_dir/console.log"
}

case "$mode" in
  check)
    nvidia-smi --query-gpu=name,driver_version,memory.total,memory.free --format=csv
    run_logged "$python" - <<'PY'
import sys
from importlib import metadata

import torch

print("Python:", sys.executable, sys.version.split()[0])
for name in ("isaacsim", "isaaclab", "isaaclab-arena", "yam-arena", "inspect-robots", "torch"):
    print(name + ":", metadata.version(name))
assert torch.cuda.is_available(), "PyTorch cannot access CUDA"
print("CUDA:", torch.version.cuda, "GPU:", torch.cuda.get_device_name(0))
assert torch.ones(4, device="cuda").mul(2).sum().item() == 8
print("CUDA computation: PASS")
PY
    "$inspect" list embodiments
    "$inspect" list policies
    ;;
  demo|gui)
    graphics=--headless
    if [[ "$mode" == gui ]]; then graphics=--no-headless; fi
    run_logged "$python" "$task_dir/run.py" --policy demo "$graphics" --output "$run_dir/trial" "$@"
    ;;
  hold)
    run_logged "$python" "$task_dir/run.py" --policy hold --max-steps 100 --no-record --output "$run_dir/trial" "$@"
    ;;
  view)
    run_logged "$python" "$task_dir/run.py" --policy view --no-headless --no-record --output "$run_dir/trial" "$@"
    ;;
  dream)
    instruction=$("$python" -c 'import sys; sys.path.insert(0, sys.argv[1]); import spec; print(spec.INSTRUCTION)' "$task_dir")
    limit=$("$python" -c 'import sys; sys.path.insert(0, sys.argv[1]); import spec; print(spec.MAX_STEPS)' "$task_dir")
    run_logged "$inspect" run --embodiment yam_arena --policy dream \
      -P model=dreamlabs/dream-1-beta -P record=false \
      -E task="$task_dir/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "$instruction" --scorer task_score --max-steps "$limit" \
      --seed 0 --epochs 1 --store-frames --no-prompt --grader none --fail-on-error 1 --log-dir "$run_dir" "$@"
    ;;
  start|dream-start)
    child_mode=view
    if [[ "$mode" == dream-start ]]; then child_mode=dream; fi
    "$python" - "$task_dir/run.sh" "$run_dir" "$child_mode" "$@" <<'PY'
import subprocess
import sys
from pathlib import Path

script, output, mode = sys.argv[1], Path(sys.argv[2]), sys.argv[3]
log = output / "launcher.log"
with log.open("w") as stream:
    process = subprocess.Popen(
        ["bash", script, mode, *sys.argv[4:]],
        stdin=subprocess.DEVNULL,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
(output / ("viewer.pid" if mode == "view" else "model.pid")).write_text(str(process.pid) + "\n")
label = "Viewer" if mode == "view" else "Model evaluation"
print(label + " launcher PID:", process.pid)
print(label + " startup log:", log)
if mode == "view":
    print("Close the Isaac Lab window to stop the viewer.")
PY
    ;;
esac
