#!/bin/bash
# dev/checks.sh [NAME:VARIANT:STEPS ...]: the scorer checks, one yam_pair run each, in turn (dev/run.sh), then a
# table of the end scores in dev/checks.txt. With no arguments, all of them.
D=$(cd "$(dirname "$0")/.." && pwd)
cd "$D" || exit 1
RUNS=("$@")
[ ${#RUNS[@]} -eq 0 ] && RUNS=(c_nothing:nothing:300 c_alone:alone:900 c_drop:drop:700 c_unslid:unslid:800 c_set_down:set_down:600
                               c_let_go_b:let_go_b:800 c_one_end_hung:one_end_hung:800 c_untightened:untightened:800
                               c_one_end:one_end:1500 c_one_screw:one_screw:1500 c_slip:slip:1500
                               c_still_held:still_held:1500 c_forearm:forearm:1700 c_driver_touch:driver_touch:1700)
for r in "${RUNS[@]}"; do
  IFS=: read -r name variant steps <<<"$r"
  rm -rf dev/logs/$name
  CLP_TORQUE_LOG=dev/runs/${name}_torque.npz TAILN=3 dev/run.sh "$name" "$steps" "$variant" > /dev/null
  gzip -c "dev/$name.log" > "dev/runs/$name.log.gz"
done
python3 dev/summary.py "${RUNS[@]}" | tee -a dev/checks.txt
