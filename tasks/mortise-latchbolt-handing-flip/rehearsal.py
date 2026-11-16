"""MuJoCo physics rehearsal of the scripted demo with real finger contact (no welds), scored by the task's own score().

Run: MJ_CHECK=<path to mj_check.py> python rehearsal.py [every_n_steps]
mj_check builds the station, both YAM arms (convex collision hulls, official gains), this package's lock case and
latchbolt colliders, and plays demo() through the official scripted policy. This prints the latchbolt joints and the
score every n control steps, then the final result.
"""
import math
import os
import pathlib
import runpy
import sys

TASK = pathlib.Path(__file__).resolve().parent / "mortise_latchbolt_handing_flip.py"
EVERY = int(sys.argv[1]) if len(sys.argv) > 1 else 20

mj = runpy.run_path(os.environ["MJ_CHECK"], run_name="mj_check")
_step = mj["Sim"].step
_n = [0]


def step(self, action):
    info = _step(self, action)
    _n[0] += 1
    if _n[0] % EVERY == 0:
        lb = info["states"]["latchbolt"]
        print(f"t={self.t:5.2f}s bolt={1000 * lb.joints.get('bolt', 0.0):7.2f} mm "
              f"turn={math.degrees(lb.joints.get('turn', 0.0)):7.2f} deg score={info.get('score_now')}", flush=True)
    return info


mj["Sim"].step = step
res = mj["rollout"](str(TASK), mode="demo")
print({k: res.get(k) for k in ("score", "success", "steps", "ik_worst_mm", "contacts", "error")})
