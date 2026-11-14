"""The score on hand-built states, with no simulator: python3 check_score.py. Runs task.py's own scorer (`seat_world`, `seated`, `moment` and the
`Trial` class), read from its source, against stand-ins for the simulator's state: the walls' hinge angles, the modules' poses and both hands'
fingertips."""

import ast
import json
import math
import os
import pathlib
import sys

import numpy as np

path = pathlib.Path(__file__).with_name("task.py")
src = path.read_text()
sys.path.insert(0, str(path.parent))
import gatekit as gk  # noqa: E402


class Part:
    def __init__(self, pos, tilt_deg=0.0, joints=None):
        self.pos = np.asarray(pos, float)
        t = math.radians(tilt_deg)
        self.R = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])
        self.joints = joints or {}

    def rotation(self):
        return self.R

    def tilt_deg(self):
        return float(np.degrees(np.arccos(np.clip(self.R[2, 2], -1.0, 1.0))))

    def joint(self, link):
        return self.joints[f"{link}_joint"]

    def deg(self, link):
        return math.degrees(self.joint(link))

    def to_world(self, pts):
        return np.atleast_2d(pts) @ self.R.T + self.pos

    def to_local(self, pts):
        return (np.atleast_2d(pts) - self.pos) @ self.R


class W:
    parts: dict = {}
    tips = None


class FakeMechkit:
    @staticmethod
    def state(env, name):
        return W.parts[name]

    @staticmethod
    def fingertips(env):
        return W.tips

    @staticmethod
    def lock(env, part, link, *a, **k):
        pass

    on_step = on_reset = staticmethod(lambda fn: fn)


ns = {"np": np, "math": math, "json": json, "os": os, "pathlib": pathlib, "m": FakeMechkit, "gk": gk, "TABLE_TOP": {"station": 0.7475}, "__file__": str(path)}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign):
        seg = ast.get_source_segment(src, node)
        if any(t in seg for t in ("m.Look", "OBJECTS", "TRIAL")):
            continue
        exec(seg, ns)
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in ("seat_world", "seated", "moment", "Trial"):
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
os.environ["FRAME_SCORE_DIR"] = "/tmp/frame_check_scores"
TOP, OPEN, HOLD, MODULES = ns["TOP"], ns["OPEN"], ns["HOLD_STEPS"], ns["MODULES"]
FAR = np.full((24, 3), 5.0)
SEATS = [ns["seat_world"](None, k) for k in range(4)]
TRAY = [np.array([t[0], t[1], TOP + ns["TRAY_H"]]) for t in ns["TRAY"]]


def frame(shut):
    a = math.radians(OPEN if shut else 0.0)
    return Part((ns["F"][0], ns["F"][1], TOP + ns["STAND_H"]), joints={"wall_l_joint": a, "wall_r_joint": -a})


def run(steps):
    t = ns["Trial"]()
    for count, kw in steps:
        for _ in range(count):
            mods = kw.get("mods", TRAY)
            W.parts = {"frame": frame(kw.get("shut", False)), **{n: Part(p) for n, p in zip(MODULES, mods)}}
            W.tips = kw.get("tips", FAR)
            t.step(None)
    return round(t.score, 3), t.full


four = list(SEATS)
three = SEATS[:3] + [TRAY[3]]
high = SEATS[:3] + [SEATS[3] + np.array([0.0, 0.0, 0.003])]
off = SEATS[:3] + [SEATS[3] + np.array([0.0015, 0.0, 0.0])]
fell = SEATS[:3] + [SEATS[3] - np.array([0.0, 0.0, 0.02])]
rolled = SEATS[:3] + [np.array([SEATS[3][0], 0.03, TOP + 0.005])]  # fallen through, then rolled onto the table 3 cm across: no longer near its position
pinch = FAR.copy()
pinch[0] = SEATS[0] + np.array([0.0, 0.0145, 0.03])  # a fingertip 2 mm off the module's face, near its top
near_end = FAR.copy()
near_end[0] = np.array([ns["F"][0] + 0.045, ns["WALL_IN"] + 0.010, TOP + ns["STAND_H"] + 0.010])  # a fingertip 1 cm off a wall's far end
CASES = {
    "all four set, frame shut and clicked, hands clear 1 s": ([(5, dict(mods=four)), (HOLD + 2, dict(mods=four, shut=True))], (1.0, True)),
    "all four, shut, a fingertip 1 cm off a wall's far end": ([(5, dict(mods=four)), (HOLD + 2, dict(mods=four, shut=True, tips=near_end))], (0.6, False)),
    "all four set on the open frame, not closed": ([(HOLD + 5, dict(mods=four))], (0.3, False)),
    "three set on the open frame, the fourth still in the tray": ([(HOLD + 5, dict(mods=three))], (0.225, False)),
    "two set on the open frame": ([(HOLD + 5, dict(mods=SEATS[:2] + TRAY[2:]))], (0.15, False)),
    "one set on the open frame": ([(HOLD + 5, dict(mods=SEATS[:1] + TRAY[1:]))], (0.075, False)),
    "one set on the open frame, its foot 1.4 mm over the open edges (2.4 mm over its shut seat)": ([(HOLD + 5, dict(mods=[SEATS[0] + np.array([0.0, 0.0, 0.0024])] + TRAY[1:]))], (0.075, False)),
    "one set on the open frame, its foot 2 mm over the open edges": ([(HOLD + 5, dict(mods=[SEATS[0] + np.array([0.0, 0.0, 0.003])] + TRAY[1:]))], (0.0, False)),
    "two set and shut, the other two positions empty": ([(5, dict(mods=SEATS[:2] + TRAY[2:])), (HOLD + 2, dict(mods=SEATS[:2] + TRAY[2:], shut=True))], (0.15, False)),
    "one set 2 mm off along the frame, open": ([(HOLD + 5, dict(mods=[SEATS[0] + np.array([0.002, 0.0, 0.0])] + TRAY[1:]))], (0.0, False)),
    "three set and shut, the fourth position empty": ([(5, dict(mods=three)), (HOLD + 2, dict(mods=three, shut=True))], (0.6, False)),
    "shut on a module 3 mm high (forced)": ([(5, dict(mods=four)), (HOLD + 2, dict(mods=high, shut=True))], (0.0, False)),
    "shut with one module 1.5 mm off along the frame, the other three seated": ([(5, dict(mods=four)), (HOLD + 2, dict(mods=off, shut=True))], (0.225, False)),
    "three seated and shut, the fourth fallen through, lying under its position": ([(5, dict(mods=fell)), (HOLD + 2, dict(mods=fell, shut=True))], (0.225, False)),
    "three seated and shut, the fourth fell through and rolled clear of the frame": ([(5, dict(mods=fell)), (HOLD + 2, dict(mods=rolled, shut=True))], (0.225, False)),
    "one set on the open frame, still pinched by the hand": ([(HOLD + 5, dict(mods=SEATS[:1] + TRAY[1:], tips=pinch))], (0.0, False)),
    "frame squeezed shut empty": ([(HOLD + 5, dict(shut=True))], (0.0, False)),
    "a module dropped off the table, the rest done": ([(2, dict(mods=SEATS[:3] + [TRAY[3] - np.array([0.0, 0.0, 0.1])])), (HOLD + 2, dict(mods=four, shut=True))], (0.0, False)),
    "nothing done": ([(HOLD + 5, {})], (0.0, False)),
}
bad = 0
for name, (steps, want) in CASES.items():
    got = run(steps)
    good = got == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: score {got[0]}, full {got[1]} (want {want[0]}, {want[1]})")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
