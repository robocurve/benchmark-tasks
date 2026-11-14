"""The score on hand-built states, with no simulator: python3 check_score.py. Runs task.py's own scorer (the kinematics, `presses`, `flange`,
`arm_surface` and the `Trial` class), read from its source, against stand-ins for the simulator's state: the FR3's joint and button positions and
both YAMs' fingertips."""

import ast
import math
import os
import pathlib
import sys

import numpy as np

path = pathlib.Path(__file__).with_name("task.py")
src = path.read_text()
sys.path.insert(0, str(path.parent))
import gatekit as gk  # noqa: E402


class Arm:
    def __init__(self, joints):
        self.joints = joints

    def joint(self, link):
        return self.joints[f"{link}_joint"]


class W:
    arm = None
    tips = None


class FakeMechkit:
    @staticmethod
    def state(env, name):
        return W.arm

    @staticmethod
    def fingertips(env):
        return W.tips

    lock = unlock = staticmethod(lambda *a, **k: None)
    on_step = on_reset = staticmethod(lambda fn: fn)


ns = {"np": np, "math": math, "os": os, "pathlib": pathlib, "m": FakeMechkit, "gk": gk, "TABLE_TOP": {"station": 0.7475}, "__file__": str(path),
      "HERE": path.parent}
skip = ("m.Look", "_fr3()", "m.fixed", "_sp =", "STAND =", "OBJECTS", "Trial()", "HERE =")
FUNCS = ("_rpy", "_rz", "fk", "_axis_name", "_link7_world", "to_station", "flange", "q_now", "presses", "_read_stl", "_samples", "_box_samples", "arm_surface",
         "teach_point")
for node in ast.parse(src).body:
    if isinstance(node, ast.FunctionDef) and node.name in FUNCS:
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign):
        seg = ast.get_source_segment(src, node)
        if any(t in seg for t in skip):
            continue
        exec(seg, ns)
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in ("_rpy", "_rz", "fk", "_axis_name", "_link7_world", "to_station", "flange", "q_now",
                                                                           "presses", "_read_stl", "_samples", "_box_samples", "arm_surface", "teach_point",
                                                                           "Trial"):
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
Q_START, Q_SERVICE, Q_REF, SIGNS, HOLD = ns["Q_START"], ns["Q_SERVICE"], ns["Q_REF"], ns["SIGNS"], ns["HOLD_STEPS"]
FLANK, FRONT_LEFT, axis_name = ns["FLANK"], ns["FRONT_LEFT"], ns["_axis_name"]
FAR = np.full((24, 3), 5.0)


def joints(q, enable=0.0, guide=0.0, teach=0.0):
    j = {f"link{i + 1}_joint": float(SIGNS[i] * (q[i] - Q_REF[i])) for i in range(7)}
    for nm, travel in (("enable", enable), ("guide", guide), ("teach", teach)):
        _, sg = axis_name(ns["TEACH_N"] if nm == "teach" else (FRONT_LEFT if nm == "enable" else -FRONT_LEFT) * FLANK)
        j[f"{nm}_joint"] = -sg * travel
    return j


def near_flange(q, d=0.03):
    t = FAR.copy()
    t[0] = ns["flange"](q)[0] + np.array([0.0, 0.0, d])
    return t


def run(steps):
    t = ns["Trial"]()
    t.posed = True
    for count, kw in steps:
        for _ in range(count):
            W.arm = Arm(joints(kw.get("q", Q_START), kw.get("en", 0.0), kw.get("gd", 0.0), kw.get("teach", 0.0)))
            W.tips = kw.get("tips", FAR)
            t.step(None)
    return round(t.score, 3), t.full


def guide(to=Q_SERVICE, n=30, en=0.0025, gd=0.0025, frm=Q_START):
    return [(1, dict(q=frm + (to - frm) * k / n, en=en, gd=gd)) for k in range(n + 1)]


BAND = 0.0025


def touching(q, where):
    """Fingertips with one point 1 cm off the arm: below its lowest point (a finger of the Hand), or over Teach's cap on the Pilot-Disc."""
    t = FAR.copy()
    pts = ns["arm_surface"](q)
    if where == "low":
        t[0] = pts[np.argmin(pts[:, 2])] - np.array([0.0, 0.0, 0.01])
    else:
        tp, tn = ns["teach_point"](q)
        t[0] = tp + 0.01 * tn
    return t


Q_LOW = Q_SERVICE.copy()
Q_LOW[1] += 0.45  # leaning on: the Hand well below the bench's top
start = [(5, dict(q=Q_START))]
done = start + guide() + [(3, dict(q=Q_SERVICE))]
taught = done + [(3, dict(q=Q_SERVICE, teach=0.003, tips=near_flange(Q_SERVICE, 0.2)))]
off3 = Q_SERVICE.copy()
off3[1] += 0.04  # a few centimetres off the pose
CASES = {
    "guided in band to the pose, let out, Teach pressed, hands clear 1 s": (taught + [(HOLD + 2, dict(q=Q_SERVICE))], (1.0, True)),
    "taught, a fingertip still 3 cm over the hand": (taught + [(HOLD + 2, dict(q=Q_SERVICE, tips=near_flange(Q_SERVICE)))], (0.75, False)),
    "at the pose and let out, never taught": (done + [(HOLD + 2, dict(q=Q_SERVICE))], (0.75, False)),
    "at the pose, buttons still held (guiding)": (start + guide() + [(HOLD + 2, dict(q=Q_SERVICE, en=BAND, gd=BAND))], (0.5, False)),
    "let out about 4 cm off the pose, Teach pressed": (start + guide(to=off3) + [(3, dict(q=off3)), (3, dict(q=off3, teach=0.003)), (HOLD + 2, dict(q=off3))],
                                                       (0.5, False)),
    "guided 12 cm the wrong way, let out": (start + guide(to=Q_START - 0.8 * (Q_SERVICE - Q_START)) + [(HOLD, dict(q=Q_START - 0.8 * (Q_SERVICE - Q_START)))],
                                           (0.25, False)),
    "squeezed through the band (full press): the arm held, dragged anyway": (start + guide(en=0.005) + [(HOLD, dict(q=Q_SERVICE))], (0.0, False)),
    "pushed to the pose with no buttons, Teach pressed": (start + guide(en=0.0, gd=0.0) + [(3, dict(q=Q_SERVICE, teach=0.003)), (HOLD + 2, dict(q=Q_SERVICE))],
                                                         (0.0, False)),
    "Enabling held, Guiding not pressed": (start + guide(gd=0.0) + [(HOLD, dict(q=Q_SERVICE))], (0.0, False)),
    "Teach pressed at the start pose only": (start + [(3, dict(q=Q_START, teach=0.003)), (HOLD, dict(q=Q_START))], (0.0, False)),
    "taught, a fingertip resting 1 cm off the Hand's finger": (taught + [(HOLD + 2, dict(q=Q_SERVICE, tips=touching(Q_SERVICE, "low")))], (0.75, False)),
    "taught, a fingertip left 1 cm off Teach": (taught + [(HOLD + 2, dict(q=Q_SERVICE, tips=touching(Q_SERVICE, "teach")))], (0.75, False)),
    "let out with the Guiding Button still in (the arm held), Teach pressed": (start + guide() + [(3, dict(q=Q_SERVICE, en=0.0, gd=BAND)),
                                                                                                  (3, dict(q=Q_SERVICE, gd=BAND, teach=0.003)),
                                                                                                  (HOLD + 2, dict(q=Q_SERVICE, gd=BAND))], (0.5, False)),
    "the Hand driven down onto the bench": (start + guide(to=Q_LOW) + [(HOLD, dict(q=Q_LOW))], (0.0, False)),
    "nothing done": ([(HOLD + 5, dict(q=Q_START))], (0.0, False)),
}
bad = 0
for name, (steps, want) in CASES.items():
    got = run(steps)
    good = got == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: score {got[0]}, full {got[1]} (want {want[0]}, {want[1]})")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
