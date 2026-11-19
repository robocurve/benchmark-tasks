#!/bin/bash
# Short runs that only record the first frames of seeded starts (to calibrate the camera estimate).
cd "$(dirname "$0")/.."
for s in 0 1 2 3; do HOLD_S=0.5 bash dev/run.sh look-s$s yam_scripted $s 40; done
