#!/usr/bin/env bash
# Sequential GPU checks; outputs stay outside the submission source tree by default.
set -euo pipefail
film_python=${1:?Usage: validate.sh /path/to/python /path/to/output}
film_out=${2:?Usage: validate.sh /path/to/python /path/to/output}
film_task_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$film_out"
film_out=$(mktemp -d "$film_out/run-XXXXXXXX")
printf 'Validation artifacts: %s\n' "$film_out"
"$film_python" -m pytest "$film_task_dir/tests" -q > "$film_out/tests.log" 2>&1
for film_mode in hold direct decoy press shear single demo; do
  film_steps=1400
  film_epochs=1
  film_extra=()
  case "$film_mode" in
    hold) film_steps=200 ;;
    direct) film_steps=380 ;;
    decoy) film_steps=800 ;;
    press|shear) film_steps=400 ;;
    demo) film_epochs=3; film_extra=(--measure-load) ;;
  esac
  "$film_python" "$film_task_dir/probe.py" --mode "$film_mode" --steps "$film_steps" \
    --epochs "$film_epochs" "${film_extra[@]}" --out "$film_out/$film_mode" > "$film_out/$film_mode.log" 2>&1
  if [[ "$film_mode" == direct ]]; then
    for film_case in "$film_out/$film_mode"/*/; do
      "$film_python" "$film_case/source/audit_scene.py" "$film_case" > "$film_case/audit.log"
    done
    continue
  fi
  for ((film_epoch=0; film_epoch<film_epochs; film_epoch++)); do
    "$film_python" "$film_out/$film_mode/source/audit_scene.py" "$film_out/$film_mode" \
      --epoch "$film_epoch" > "$film_out/$film_mode/$film_epoch-audit.log"
  done
  "$film_python" - "$film_out/$film_mode" "$film_mode" "$film_epochs" <<'PY'
import json, sys
from pathlib import Path
root, mode, epochs = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])
for epoch in range(epochs):
    p = root / f'{epoch}-summary.json'
    assert p.exists(), f'Simulator did not finish: {p}'
    s = json.loads(p.read_text())
    assert s['source_unchanged'], 'Source changed during validation; rerun.'
    if mode in ('hold', 'press', 'shear', 'decoy'):
        assert s['score'] == 0 and s['final_info']['peeled_strips'] == 0, s
    if mode == 'hold':
        assert not s['final_info']['invalid_trial'], s
    if mode == 'single':
        assert s['score'] == 0, 'Single-arm baseline bypasses substrate stabilization: ' + str(s)
    if mode == 'demo':
        assert s['success'] and s['score'] == 1, s
    print(json.dumps(s), flush=True)
PY
done
