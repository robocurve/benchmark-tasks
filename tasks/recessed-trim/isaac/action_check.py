"""Executed-vs-commanded action check of an official run (reads JSON only).

    python action_check.py <official log dir> [--out action_check.json]

Compares the actions the policy commanded (``policy_actions.jsonl``, written by official_run.py every step, rounded to
1e-6) with the
actions the Inspect Robots CLI executed (``actions/<run>/scene-0-e0.jsonl``, after its joint-bound clamp and per-step
delta guard), step by step, and counts the steps where any entry differs by more than 1e-5. Target: 0. It also
re-applies both guards to the commanded actions (needs yam_arena for the joint bounds) as a second count.
"""

import glob
import json
import os
import sys

import numpy as np


def main():
    d = sys.argv[1]
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else None
    pa = os.path.join(d, "policy_actions.jsonl")
    if os.path.exists(pa):  # every step (policy_log.json is written every 20 steps)
        cmd = np.asarray([json.loads(line)["action"] for line in open(pa)], float)
    else:
        cmd = np.asarray(json.load(open(os.path.join(d, "policy_log.json")))["actions"], float)
    f = sorted(glob.glob(os.path.join(d, "actions", "*", "scene-0-e0.jsonl")))[0]
    rows = [json.loads(line) for line in open(f)]
    exe = {r["t"]: np.asarray(r["action"], float) for r in rows if "t" in r}
    n = min(len(cmd), len(exe))
    diffs = [float(np.max(np.abs(exe[t] - cmd[t]))) for t in range(n) if t in exe]
    changed = int(sum(1 for x in diffs if x > 1e-5))
    res = dict(commanded=len(cmd), executed=len(exe), compared=len(diffs), steps_changed=changed,
               max_abs_diff=max(diffs) if diffs else None)
    try:
        from yam_arena import ik

        lo1, hi1 = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        lo, hi = np.r_[lo1, lo1], np.r_[hi1, hi1]
        dmax = 0.05 * (hi - lo)
        prev, guard = None, 0
        for a in cmd:
            e = np.clip(a, lo, hi)
            if prev is not None:
                e = np.clip(e, prev - dmax, prev + dmax)
            guard += int(np.max(np.abs(e - a)) > 1e-9)
            prev = e
        res["steps_guard_would_change"] = guard
    except ImportError:
        pass
    print(json.dumps(res))
    if out:
        open(out, "w").write(json.dumps(res, indent=1) + "\n")


if __name__ == "__main__":
    main()
