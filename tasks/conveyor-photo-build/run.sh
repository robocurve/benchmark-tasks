#!/usr/bin/env bash
# One scored trial with the exact instruction from task.json: the scripted demo (or a scripted baseline) or Dream-1.
#   bash tasks/conveyor-photo-build/run.sh /path/to/python yam_scripted /path/to/logs [demo|no_staging|hold_on_belt|keep_extra]
#   bash tasks/conveyor-photo-build/run.sh /path/to/python dream /path/to/logs
set -euo pipefail
cg_python=${1:?Usage: run.sh /path/to/python yam_scripted-or-dream /path/to/logs [plan]}
cg_policy=${2:?Policy is required: yam_scripted or dream}
cg_logs=${3:?Output directory is required}
cg_plan=${4:-demo}
case "$cg_policy" in yam_scripted|dream) ;; *) echo 'Expected yam_scripted or dream' >&2; exit 2 ;; esac
cg_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cg_cli="$(dirname "$cg_python")/inspect-robots"
cg_policy_args=()
if [[ "$cg_policy" == dream ]]; then
  # All 48 rows of each chunk play before Dream is asked again (its default). Dream's gateway drops a session
  # left idle for about 8 s; this scene steps in about 0.04 s, but on a machine where 48 steps take longer than
  # that, play fewer rows per request with CONVEYOR_DREAM_ROWS (e.g. 16).
  cg_policy_args=(-P model=dreamlabs/dream-1-beta -P "rows_per_request=${CONVEYOR_DREAM_ROWS:-48}")
fi
cg_instruction=$("$cg_python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["instruction"])' "$cg_dir/task.json")
# The task is several Python files; the revision hashes all of them, in name order.
cg_revision=$("$cg_python" - "$cg_dir" <<'PY'
import hashlib, sys
from pathlib import Path
h = hashlib.sha256()
for p in sorted(Path(sys.argv[1]).glob('*.py')):
    h.update(p.name.encode() + b'\0' + p.read_bytes())
print(h.hexdigest()[:12])
PY
)
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CONVEYOR_PLAN="$cg_plan"
echo "plan=$cg_plan revision=$cg_revision"
# -E device=cpu: the belt's surface velocity runs only in PhysX's CPU pipeline; the cameras still render on the GPU.
"$cg_cli" run --embodiment yam_arena --policy "$cg_policy" "${cg_policy_args[@]}" \
  -E "task=$cg_dir/task.py" -E device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$cg_instruction" \
  --scorer task_score --grader none --no-prompt --max-steps 2400 --seed 0 \
  --environment-revision "$cg_revision" --store-frames --log-dir "$cg_logs"
