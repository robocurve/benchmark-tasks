#!/usr/bin/env bash
# Run each "label steps args..." line (e.g. dev/all_runs.txt) of a queue file one at a time, each only once at most one other sim holds the GPU.
# Each run gets its own stdin ("n", then an empty note), so a run that ends early never reads the queue's next lines.
q=$1
here=$(cd "$(dirname "$0")/.." && pwd)
while IFS= read -r line <&3; do
  [[ -z $line || $line == \#* ]] && continue
  while [[ $(nvidia-smi --query-compute-apps=pid --format=csv,noheader | wc -l) -gt 1 ]]; do sleep 20; done
  echo "=== $(date -u +%H:%M:%S) start: $line"
  # shellcheck disable=SC2086
  printf 'n\n\n' | bash "$here/dev/run.sh" $line 2>&1 | grep -E "\[inspect|OUT=|task_score|Error|error|Traceback|score" | grep -v "not put down" | tail -400
  echo "=== $(date -u +%H:%M:%S) done: $line"
done 3< "$q"
