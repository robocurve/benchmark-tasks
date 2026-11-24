#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
mkdir -p results
out="$(mktemp -d results/cushion-ablation.XXXXXX)"
echo "Writing cushion ablation to $out"
for condition in stock filter; do
  mkdir "$out/$condition"
  for mode in reference fixed_heel_on_foam idle; do
    python3 -m validation.cushion.ablate "$mode" "$condition" "$out/$condition" \
      >"$out/$condition/$mode.log" 2>&1 || { cat "$out/$condition/$mode.log" >&2; exit 1; }
  done
done
python3 -m validation.cushion.compare "$out"
