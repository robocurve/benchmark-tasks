"""Offline check of the scorer on hand-built end states (no simulator). Run with the harness's Python, from this folder:

    python check_score.py

Each case feeds the same state for 60 control steps (3 s) and prints the score at the end, against the score the
task's text gives.
"""

from __future__ import annotations

import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("TASK_DEMO", "success")

import task as T  # noqa: E402

UP = (0.0, 0.0, 0.0, 1.0)
FAR = [(0.25, 0.33, T.TOP + 0.21), (0.0, -0.305, T.TOP + 0.26)]


def state(pos, quat=UP):
    return SimpleNamespace(pos=tuple(pos), quat_xyzw=tuple(quat))


def run(cell=T.CELL_HOME, cap=T.CAP_HOME, ep=T.EP0, filt=T.FILT0, force=10.0, caught=False, tcps=FAR, steps=60) -> float:
    T._reset()
    for _ in range(steps):
        T._R.update(force=force, caught=caught)
        T.update(state(cell), state(cap), ep, UP, filt, tcps)
    return T._score()


CASES = [
    ("everything done, grippers clear", {}, 1.0),
    ("thumbscrew not tightened (2 N)", dict(force=2.0), 0.25),
    ("tightened to 8 N", dict(force=8.0), 1.0),
    ("finder left open", dict(cap=T.CAP_START), 0.5),
    ("cell only 1.5 cm on, tightened", dict(cell=(T.TX, T.CELL_HOME[1] + 0.015, T.CELL_HOME[2])), 0.0),
    ("eyepiece left out, on the table", dict(ep=(0.40, -0.10, T.TOP + 0.04)), 0.5),
    ("eyepiece 4 mm proud of the holder", dict(ep=(T.EP0[0], T.EP0[1], T.EP0[2] + 0.004)), 0.5),
    ("a gripper on the eyepiece", dict(tcps=[FAR[0], (T.EP0[0], T.EP0[1], T.EP0[2] + 0.016)]), 0.5),
    ("SUN filter screwed into the eyepiece, the rest done", dict(caught=True), 0.0),
    ("SUN filter lying in the holder", dict(filt=(T.EP0[0], T.EP0[1], T.TOP + 0.115)), 0.0),
    ("SUN filter on the eyecup, not screwed in", dict(filt=(T.EP0[0], T.EP0[1], T.EP0[2] + 0.044)), 1.0),
    ("nothing done", dict(cell=T.CELL_START, cap=T.CAP_START, force=0.0), 0.0),
    ("cap only", dict(cell=T.CELL_START, force=0.0), 0.0),
]

if __name__ == "__main__":
    bad = 0
    for name, kw, want in CASES:
        got = run(**kw)
        ok = abs(got - want) < 1e-9
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {got:4.2f} (text: {want:4.2f})  {name}")
    sys.exit(1 if bad else 0)
