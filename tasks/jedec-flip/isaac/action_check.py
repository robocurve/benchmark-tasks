"""Count the official run's executed actions that differ from the commanded ones.

    python action_check.py <official log dir>/policy_log.json [--out action_check.json]

The Inspect Robots CLI clamps every action to the URDF joint bounds (gripper 0..1) and limits the per-step change to
0.05 x the joint range (relative to the previous executed action). This re-applies both guards to the commanded
actions that official_run.py logs and counts the steps (and entries) the guards would change. Target: 0.
Needs yam_arena (the embodiment's joint bounds) on the path.
"""

import json
import sys

import numpy as np
from yam_arena import ik


def main():
    path = sys.argv[1]
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else None
    acts = np.asarray(json.load(open(path))["actions"], float)
    lo1, hi1 = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
    lo, hi = np.r_[lo1, lo1], np.r_[hi1, hi1]
    dmax = 0.05 * (hi - lo)
    prev = None
    n_bounds = n_delta = steps = 0
    worst = 0.0
    for a in acts:
        e = np.clip(a, lo, hi)
        nb = int(np.sum(np.abs(e - a) > 1e-9))
        if prev is not None:
            e2 = np.clip(e, prev - dmax, prev + dmax)
            nd = int(np.sum(np.abs(e2 - e) > 1e-9))
            e = e2
        else:
            nd = 0
        n_bounds += nb
        n_delta += nd
        steps += int(nb + nd > 0)
        worst = max(worst, float(np.max(np.abs(e - a))))
        prev = e
    res = dict(
        actions=len(acts),
        steps_changed=steps,
        entries_clamped_bounds=n_bounds,
        entries_clamped_delta=n_delta,
        max_abs_change=worst,
    )
    print(json.dumps(res))
    if out:
        open(out, "w").write(json.dumps(res, indent=1) + "\n")


if __name__ == "__main__":
    main()
