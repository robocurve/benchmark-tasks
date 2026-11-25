#!/usr/bin/env bash
# Evidence run on a frozen source snapshot.
#   run_frozen.sh <run-name> <command run inside the snapshot's isaac/ dir, as one string>
# Copies the task folder's {isaac,src,task.json} into $OUT/<run>/source (read-only afterwards), writes
# source-sha256.txt and source-combined-sha256.txt, then runs the command in the Isaac container (one GPU) with the
# snapshot as the working tree and the run directory as the output. Log: $OUT/<run>/run.log.
# Environment (host-specific; nothing host-specific is stored in this folder):
#   ISAAC_LAUNCHER    container launcher script (Isaac Sim 6.1 + Isaac Lab-Arena podman wrapper, --gpu)
#   GPU_SLOT_WRAPPER  optional GPU slot allocator script that wraps the launcher (sets the GPU)
#   ARENA_VENV_BIN    bin/ directory of the Isaac Lab-Arena Python venv as seen inside the container
#   TUBE_OUT          host output root, mounted at the same path in the container (default: <task folder>/runs)
#   TUBE_SETUP, TUBE_REPLAY  passed into the container when set (named start setup; trace to replay open-loop)
set -euo pipefail
run=$1; shift
cmd=$1
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
OUT=${TUBE_OUT:-$here/runs}
E=${ISAAC_LAUNCHER:?set ISAAC_LAUNCHER to the Isaac container launcher script}
V=${ARENA_VENV_BIN:?set ARENA_VENV_BIN to the Arena venv bin directory}
dir=$OUT/$run
[[ -e $dir ]] && { echo "exists: $dir" >&2; exit 1; }
mkdir -p "$dir/source"
cp -r "$here/isaac" "$here/src" "$here/task.json" "$dir/source/"
find "$dir/source" -name __pycache__ -prune -exec rm -rf {} +
(cd "$dir/source" && find . -type f | LC_ALL=C sort | xargs sha256sum) > "$dir/source-sha256.txt"
sha256sum "$dir/source-sha256.txt" | cut -d' ' -f1 > "$dir/source-combined-sha256.txt"
chmod -R a-w "$dir/source"
echo "$cmd" > "$dir/command.txt"
envs=()
for v in TUBE_SETUP TUBE_REPLAY TUBE_MIRROR; do [[ -n ${!v:-} ]] && envs+=(-e "$v=${!v}"); done
echo "${envs[*]:-}" > "$dir/env.txt"
export RC_CONTAINER_TIMEOUT=${RC_CONTAINER_TIMEOUT:-${TUBE_TIMEOUT:-5400}}  # read by container launchers that support it
launch=("$E")
[[ -n ${GPU_SLOT_WRAPPER:-} ]] && launch=("$GPU_SLOT_WRAPPER" bash "$E")
timeout -k 5 "${TUBE_TIMEOUT:-14400}" "${launch[@]}" --gpu -- \
  -e PATH="$V:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" -e TMPDIR="$dir/tmp" "${envs[@]}" \
  -- bash -c "mkdir -p '$dir/tmp' && cd '$dir/source/isaac' && $cmd" > "$dir/run.log" 2>&1
