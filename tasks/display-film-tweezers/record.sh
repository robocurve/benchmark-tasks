#!/usr/bin/env bash
# Standard Inspect Robots recording, with the exact instruction from task.json.
set -euo pipefail
film_python=${1:?Usage: record.sh /path/to/python yam_scripted-or-dream /path/to/logs}
film_policy=${2:?Policy is required}
film_logs=${3:?Output directory is required}
case "$film_policy" in yam_scripted|dream) ;; *) echo 'Expected yam_scripted or dream' >&2; exit 2 ;; esac
film_policy_args=()
if [[ "$film_policy" == dream ]]; then
  # Dream's gateway drops a session left idle for about 8 s. A 48-row chunk takes about 6 s on an
  # L40S but about 14 s at 0.3 s/step on slower GPUs; set FILM_DREAM_ROWS (e.g. 16) there.
  film_policy_args=(-P model=dreamlabs/dream-1-beta -P "rows_per_request=${FILM_DREAM_ROWS:-48}")
fi
film_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
film_cli="$(dirname "$film_python")/inspect-robots"
film_instruction=$("$film_python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["instruction"])' "$film_dir/task.json")
film_revision=$("$film_python" -c 'import sys; sys.path.insert(0,sys.argv[1]); from probe import source_hash; print(source_hash())' "$film_dir")
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$film_cli" run --embodiment yam_arena --policy "$film_policy" "${film_policy_args[@]}" \
  -E "task=$film_dir/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$film_instruction" \
  --scorer task_score --grader none --no-prompt --max-steps 2400 --seed 0 \
  --environment-revision "$film_revision" --store-frames --log-dir "$film_logs"
