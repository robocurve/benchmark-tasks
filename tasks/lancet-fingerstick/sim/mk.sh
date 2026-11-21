#!/bin/bash
# mk.sh: sim copies of the task in sim/arm/: lancet_fingerstick.py as is, plus traced copies (t00 = no delay, t05 = 0.05 s
# start delay; t00b/t00c and t05b/t05c are copies for parallel runs) with log_snippet.py appended, and the traced stems
# with stems.py appended (o00 one arm, j00 one-arm jab, x00 one-arm 30 mm jab in one control step, w00 one-arm press
# near the apex driven toward the knuckle, f00 a press in the stand after the twist, k00 the lancet held in the
# container, c00 the tab twisted the other way, a00 the left closed on air above
# the ring plus a one-step jab, g00 the left's grip without its lift, y00 the demo's hold and a 60 mm one-step jab).
# Rerun after any task edit.
set -eu
cd "$(dirname "$0")"
rm -rf arm && mkdir arm && cp ../lancet_fingerstick.py ../mechanism.py ../task.json arm/
grep -q 'def demo(delay=0.0):' arm/lancet_fingerstick.py || { echo "demo() signature changed"; exit 3; }
for s in 00 05; do
  sed "s/def demo(delay=0.0):/def demo(delay=0.$s):/" arm/lancet_fingerstick.py > arm/t$s.py
  cat log_snippet.py >> arm/t$s.py
done
for c in b c; do cp arm/t00.py arm/t00$c.py; cp arm/t05.py arm/t05$c.py; done
for k in o j x w f k c a g y; do
  cat arm/lancet_fingerstick.py log_snippet.py stems.py | sed "s/^_STEM = \"none\"$/_STEM = \"$k\"/" > arm/${k}00.py
  grep -q "^_STEM = \"$k\"$" arm/${k}00.py || { echo "${k}00 stem not set"; exit 3; }
done
echo MK OK
