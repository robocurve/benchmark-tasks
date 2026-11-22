"""Scorer check for space-the-rice-pans-on-the-cooler-shelf, without the simulator: set pan poses are fed to task.py's
scorer for 25 steps (no gripper near either pan) and the score and report are printed. It covers the rules the scripted
demo could not reach: pans set down less than 5 cm apart (0.5), touching (0.25) and stacked (0), plus a pan still held
and a scene still moving.

    python scorer_check.py task.py
"""
import importlib.util
import pathlib
import sys

import numpy as np

path = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "task.py").resolve()
sys.path.insert(0, str(path.parent))
spec = importlib.util.spec_from_file_location("task", path)
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
import kitchenkit as kk  # noqa: E402  (the module task.py imported)

I = np.array([0.0, 0.0, 0.0, 1.0])  # no rotation (x, y, z, w)
X, Z = t.SHELF_XY[0], t.SHELF_TOP + 0.0005


def run(case, ya, yb, zb=Z, moving=False, held=False):
    kk.STATE.step = -1
    t._reset()
    for k in range(25):
        kk.STATE.step = k
        d = 0.003 * k if moving else 0.0
        kk.STATE.poses = {"pan_a": (np.array([X, ya - d, Z]), I), "pan_b": (np.array([X, yb + d, zb]), I)}
        # a 'held' pan: finger pad points on pan_b's flange at its near end
        kk.STATE.fingers = {"right_robot": np.array([[X, yb - t.PAN_OUT[1] / 2, zb + t.FLANGE_Z]] * 4)} if held else {}
        score = t.score(None)
    r = t.report()
    print(f"{case:<34} score {score:.2f}  gap {r['gap']}  placed {r['placed']}  stacked {r['stacked']}  still {r['still']}")
    return score


expect = {
    "full demo targets (9.6 cm apart)": (run("full demo targets (9.6 cm apart)", -t.SLOT_Y, t.SLOT_Y), 1.0),
    "2.4 cm apart": (run("2.4 cm apart", -0.174, 0.174), 0.5),
    "0.5 cm apart (touching)": (run("0.5 cm apart (touching)", -0.165, 0.164), 0.25),
    "stacked in the middle": (run("stacked in the middle", 0.0, 0.0, zb=Z + t.PAN_H + 0.0005), 0.0),
    "second pan still held": (run("second pan still held", -t.SLOT_Y, t.SLOT_Y, held=True), 0.0),
    "pans still moving": (run("pans still moving", -t.SLOT_Y, t.SLOT_Y, moving=True), 0.0),
}
bad = [k for k, (got, want) in expect.items() if abs(got - want) > 1e-9]
print("all as expected" if not bad else f"UNEXPECTED: {bad}")
