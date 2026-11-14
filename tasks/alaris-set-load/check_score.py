"""The score lines on hand-built states, with no simulator: python3 check_score.py. Runs task.py's own scorer (`fit_seated`, `clamp_seated` and the
`Trial` class), read from its source, against stand-ins for the simulator's state."""

import ast
import json
import math
import os
import pathlib
import sys

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()


class Part:
    """A part's pose: what task.py reads from mechkit's state."""

    def __init__(self, pos, yaw_deg=0.0, tilt_deg=0.0):
        self.pos = np.asarray(pos, float)
        y, t = math.radians(yaw_deg), math.radians(tilt_deg)
        rz = np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])
        self.R = rz @ ry

    def rotation(self):
        return self.R

    def tilt_deg(self):
        return float(np.degrees(np.arccos(np.clip(self.R[2, 2], -1.0, 1.0))))

    def to_world(self, pts):
        return np.atleast_2d(pts) @ self.R.T + self.pos


class Rope:
    def __init__(self, a, b, gap=0.0):
        self.joint_gaps = np.full(5, 0.0) if np.isscalar(gap) and gap == 0.0 else (np.array([gap, 0, 0, 0, 0], float) if np.isscalar(gap) else np.asarray(gap, float))
        self.max_joint_gap = float(self.joint_gaps.max())
        self._pts = np.linspace(a, b, 7)

    def polyline(self):
        return self._pts


class World:
    """The simulator's state at one step: parts, segment, both hands' fingertips."""

    def __init__(self):
        self.parts, self.rope, self.tips, self.pushed = {}, None, None, []


W = World()


class FakeMechkit:
    @staticmethod
    def state(env, name):
        return W.parts[name]

    @staticmethod
    def fingertips(env):
        return W.tips

    @staticmethod
    def push(env, name, force, at=None, body=None):
        W.pushed.append((name, force))

    on_step = on_reset = staticmethod(lambda fn: fn)


class FakeCordkit:
    @staticmethod
    def rope_state(env, name):
        return W.rope


ns = {"np": np, "math": math, "json": json, "os": os, "m": FakeMechkit, "cordkit": FakeCordkit, "TABLE_TOP": {"station": 0.7475}}
skip = {"OBJECTS", "TRIAL"}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign) and not any(isinstance(n, ast.Name) and n.id in skip for t in node.targets for n in ast.walk(t)):
        seg = ast.get_source_segment(src, node)
        if "m.Look" in seg or "LOOKS" in seg:
            continue
        exec(seg, ns)
    if isinstance(node, ast.FunctionDef) and node.name in ("_yaw", "fit_seated", "clamp_seated", "stretch"):
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
    if isinstance(node, ast.ClassDef) and node.name == "Trial":
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
os.environ["ALARIS_SCORE_DIR"] = "/tmp/alaris_check_scores"
TOP, XF, Z_UP, Z_LOW, X_SEAT, SEATED_X, HOLD = (ns[k] for k in ("TOP", "XF", "Z_UP", "Z_LOW", "X_SEAT", "SEATED_X", "HOLD_STEPS"))
SEAT = np.array([X_SEAT, 0.0, TOP + Z_UP])
HOME = np.array([SEATED_X, 0.0, TOP + Z_LOW])
HANG = np.array([0.28, -0.14, TOP + ns["CLAMP_Z0"]])
HOLDER = np.array([0.28, -0.14, TOP + ns["Z_HOLD"]])
FAR = np.full((24, 3), 5.0)  # both hands far away


def state(fit=HOLDER, clamp=HANG, fit_yaw=0.0, clamp_yaw=0.0, gap=0.0, tips=FAR):
    W.parts = {"fitment": Part(fit, fit_yaw), "sclamp": Part(clamp, clamp_yaw)}
    W.rope = Rope(np.asarray(fit) - [0, 0, 0.01], np.asarray(clamp) + [0, 0, 0.01], gap)
    W.tips = tips


def run(steps):
    """steps: a list of (count, state kwargs). Returns the score and full after the last step."""
    t = ns["Trial"]()
    for count, kw in steps:
        for _ in range(count):
            state(**kw)
            t.step(None)
    return t.score, t.full


near_joint = np.full((24, 3), 5.0)
near_joint[0] = (SEAT + HOME) / 2 + np.array([-0.03, 0.0, 0.0])  # one fingertip 3 cm from the middle of the segment
CASES = {
    "loaded, hands clear, held 2 s": ([(5, {}), (5, {"fit": SEAT}), (HOLD, {"fit": SEAT, "clamp": HOME})], (1.0, True)),
    "loaded, hands clear, 1 step short of 2 s": ([(5, {"fit": SEAT}), (HOLD - 1, {"fit": SEAT, "clamp": HOME})], (0.6, False)),
    "loaded, a fingertip 3 cm from the segment": ([(5, {"fit": SEAT}), (HOLD + 5, {"fit": SEAT, "clamp": HOME, "tips": near_joint})], (0.6, False)),
    "clamp home before the fitment": ([(5, {"clamp": HOME}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.2, False)),
    "fitment lifted 4 mm after seating, then back": ([(5, {"fit": SEAT}), (2, {"fit": SEAT + [0, 0, 0.004]}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.2, False)),
    "clamp spun a full turn, then pressed home square": ([(5, {"fit": SEAT})] + [(1, {"fit": SEAT, "clamp_yaw": y}) for y in (90.0, 180.0, 270.0)] + [(HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.2, False)),
    "clamp home but turned 35 degrees": ([(5, {"fit": SEAT}), (HOLD + 5, {"fit": SEAT, "clamp": HOME, "clamp_yaw": 35.0})], (0.2, False)),
    "clamp turned 35 degrees while hanging, then home square": ([(5, {"fit": SEAT}), (5, {"fit": SEAT, "clamp_yaw": 35.0}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (1.0, True)),
    "segment stretched 3 mm after seating": ([(5, {"fit": SEAT}), (2, {"fit": SEAT, "gap": 0.003}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.2, False)),
    "clamp pressed only 8 mm short of home": ([(5, {"fit": SEAT}), (HOLD + 5, {"fit": SEAT, "clamp": HOME - [0.008, 0, 0]})], (0.2, False)),
    "fitment seated, clamp hanging": ([(HOLD + 5, {"fit": SEAT})], (0.2, False)),
    "fitment 3 mm off the slot's end": ([(HOLD + 5, {"fit": SEAT - [0.003, 0, 0], "clamp": HOME})], (0.0, False)),
    "clamp dropped below the table, then everything loaded": ([(2, {"clamp": HANG - [0, 0, 0.2]}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.0, False)),
    "clamp pressed onto the pins before the fitment, home after it": ([(5, {"clamp": HOME - [0.003, 0, 0]}), (5, {"fit": SEAT, "clamp": HOME - [0.003, 0, 0]}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.2, False)),
    "segment opened 1.9 mm at three joints in a row (5.7 mm over 100 mm)": ([(5, {"fit": SEAT}), (2, {"fit": SEAT, "gap": [0.0019, 0.0019, 0.0019, 0, 0]}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.2, False)),
    "segment stretched before the fitment was seated": ([(2, {"gap": 0.003}), (5, {"fit": SEAT}), (HOLD + 5, {"fit": SEAT, "clamp": HOME})], (0.2, False)),
    "clamp 3 mm short of the back stop": ([(5, {"fit": SEAT}), (HOLD + 5, {"fit": SEAT, "clamp": HOME - [0.003, 0, 0]})], (0.2, False)),
    "nothing done": ([(HOLD + 5, {})], (0.0, False)),
}
bad = 0
for name, (steps, want) in CASES.items():
    got = run(steps)
    good = (round(got[0], 3), got[1]) == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: score {got[0]}, full {got[1]} (want {want[0]}, {want[1]})")
W.pushed = []  # the recess's catch: on when the clamp is home, off again once it is pulled back out
run([(5, {"fit": SEAT}), (5, {"fit": SEAT, "clamp": HOME}), (5, {"fit": SEAT, "clamp": HOME - [0.006, 0, 0]})])
good = [p[1][0] for p in W.pushed] == [ns["CATCH_N"], 0.0]
bad += not good
print(f"{'ok ' if good else 'BAD'} the catch holds a clamp that is home and lets go of one pulled out: pushes {[p[1][0] for p in W.pushed]}")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
