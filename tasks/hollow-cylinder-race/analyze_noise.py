"""Summarise the noise study (probe_race.txt log): run-to-run noise of identical cylinders, lane bias, gaps between
kinds, and the false-positive and miss rates of the scorer's threshold.

    python analyze_noise.py noise_probe.log [gap_min_s]
"""

import json
import sys

import numpy as np

rows = [json.loads(line[6:]) for line in open(sys.argv[1]) if line.startswith("PROBE ")]
gap_min = float(sys.argv[2]) if len(sys.argv) > 2 else 0.020
same, cross, missing = [], {}, 0
times = {"thin": [], "thick": [], "solid": []}
for r in rows:
    c = r["cross"] or {}
    if len(c) != 2:
        missing += 1
        continue
    t0, t1 = c["red"], c["green"]          # red ran in lane 0, green in lane 1
    times[r["lane0"]].append(t0)
    times[r["lane1"]].append(t1)
    if r["lane0"] == r["lane1"]:
        same.append((r["lane0"], t0 - t1))
    else:
        key = tuple(sorted((r["lane0"], r["lane1"])))
        slow = max((r["lane0"], t0), (r["lane1"], t1), key=lambda kv: {"solid": 0, "thick": 1, "thin": 2}[kv[0]])
        fast = min((r["lane0"], t0), (r["lane1"], t1), key=lambda kv: {"solid": 0, "thick": 1, "thin": 2}[kv[0]])
        cross.setdefault(key, []).append(slow[1] - fast[1])

print(f"races: {len(rows)}, with both finish times: {len(rows) - missing}, missing a time: {missing}")
for k, v in times.items():
    if v:
        print(f"  {k:5s}: n {len(v):2d}  mean {np.mean(v)*1000:7.1f} ms  sd {np.std(v, ddof=1)*1000:5.1f} ms  "
              f"range {min(v)*1000:.1f}-{max(v)*1000:.1f} ms")
g = np.array([d for _, d in same])
print(f"identical pairs (lane 0 minus lane 1): n {len(g)}  mean {g.mean()*1000:+.1f} ms (lane bias)  "
      f"sd {g.std(ddof=1)*1000:.1f} ms  max |gap| {np.abs(g).max()*1000:.1f} ms")
fp = int((np.abs(g) > gap_min).sum())
print(f"threshold {gap_min*1000:.0f} ms: false positives {fp}/{len(g)} identical races "
      f"(a gap above the threshold between identical cylinders)")
print(f"threshold / largest identical gap = {gap_min / np.abs(g).max():.1f}; threshold / sd = {gap_min / g.std(ddof=1):.1f}")
for key, v in sorted(cross.items()):
    v = np.array(v)
    print(f"  {key[0]} vs {key[1]}: n {len(v)}  slower-kind lag mean {v.mean()*1000:.1f} ms  min {v.min()*1000:.1f} ms  "
          f"wrong order {(v < 0).sum()}  below threshold {(v <= gap_min).sum()}")
