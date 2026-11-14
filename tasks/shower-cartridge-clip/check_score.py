"""The score on hand-built states, with no simulator: python3 check_score.py. Runs task.py's own scorer (`legs`, `state_now`, the `Trial` class,
`_full` and `score`), read from its source, against stand-ins for the simulator's state: the cartridge's travel and turn, the clip's pose and both
hands' fingertips."""

import ast
import math
import os
import pathlib
import sys

import numpy as np

path = pathlib.Path(__file__).with_name("task.py")
src = path.read_text()


class Part:
    def __init__(self, pos, yaw_deg=0.0, tilt_deg=0.0, joints=None):
        self.pos = np.asarray(pos, float)
        y, t = math.radians(yaw_deg), math.radians(tilt_deg)
        rz = np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])
        self.R = rz @ rx
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
    def push(env, name, force, at=None, body=None):
        pass

    on_step = on_reset = staticmethod(lambda fn: fn)


ns = {"np": np, "math": math, "os": os, "pathlib": pathlib, "m": FakeMechkit, "TABLE_TOP": {"station": 0.7475}, "__file__": str(path)}
names = ("legs", "depth", "cart_pos", "state_now", "Trial", "_full", "score", "success")
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign):
        seg = ast.get_source_segment(src, node)
        if any(t in seg for t in ("m.Look", "m.mechanism", "m.fixed", "m.free", "m.slider", "m.Link", "_valve()", "_clip()", "_cartridge(", "Orings()", "Trial()", "VALVE", "HOLDER", "CLIP =", "OBJECTS")):
            continue
        exec(seg, ns)
    if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names:
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
ns["TRIAL"] = ns["Trial"]()
TOP, PROUD, HOLD = ns["TOP"], ns["PROUD"], ns["HOLD_STEPS"]
SLOT, VY, LUG_BOT, LEG_L, ZV, XV = (ns[k] for k in ("SLOT", "VY", "LUG_BOT", "LEG_L", "ZV", "XV"))
SX = (SLOT[0] + SLOT[1]) / 2
FAR = np.full((24, 3), 5.0)
SEATED = np.array([SX, VY, TOP + LUG_BOT + LEG_L])  # the clip's crossbar with both legs on their stops
IN_HOLDER = np.array([0.20, 0.10, TOP + 0.010 + LEG_L + 0.001])
ON_EARS = np.array([SX, VY, TOP + ZV + 0.008 + LEG_L])  # legs in their slots, stopped on the ears of a cartridge that is not home


def st(out_mm=0.0, clip=IN_HOLDER, yaw=0.0, tilt=0.0, tips=FAR):
    """out_mm: how far the ears stand out of home (0 flush)."""
    W.parts = {"valve": Part((0.0, VY, TOP), joints={"slide_joint": PROUD - out_mm / 1000.0, "roll_joint": 0.0}), "clip": Part(clip, yaw, tilt)}
    W.tips = tips
    return {}


def run(steps):
    t = ns["Trial"]()
    ns["TRIAL"] = t
    for count, kw in steps:
        for _ in range(count):
            st(**kw)
            t.step(None)
    return round(ns["score"](None), 3), bool(t.full)


near = FAR.copy()
near[0] = SEATED + np.array([0.0, 0.0, 0.02])  # a fingertip 2 cm over the crossbar
CASES = {
    "flush, clip on both stops, hands clear 1 s": ([(5, dict(out_mm=0.0, clip=SEATED))] + [(HOLD + 2, dict(out_mm=0.3, clip=SEATED))], (1.0, True)),
    "flush, clip on its stops, a hand 2 cm off": ([(HOLD + 5, dict(out_mm=0.0, clip=SEATED, tips=near))], (0.5, False)),
    "clip at the slots' height but 20 mm beside them, not pushed": ([(HOLD + 5, dict(out_mm=PROUD * 1000, clip=SEATED + np.array([0.0, 0.02, 0.01])))], (0.0, False)),
    "clip at the slots, turned 30 degrees, lowered": ([(HOLD + 5, dict(out_mm=PROUD * 1000, clip=SEATED + np.array([0.0, 0.0, 0.01]), yaw=30.0))], (0.0, False)),
    "flush, clip on its stops but turned 8 degrees": ([(HOLD + 5, dict(out_mm=0.0, clip=SEATED, yaw=8.0))], (0.25, False)),
    "not pushed, clip in its slots stopped on the ears": ([(HOLD + 5, dict(out_mm=PROUD * 1000, clip=ON_EARS))], (0.25, False)),
    "pushed flush, let go (1 mm out), clip on the ears": ([(5, dict(out_mm=0.0)), (HOLD + 5, dict(out_mm=1.0, clip=ON_EARS))], (0.25, False)),
    "flush, clip started in its slots": ([(HOLD + 5, dict(out_mm=0.2, clip=ON_EARS))], (0.5, False)),
    "clip dropped on the table, then seated with all else done": ([(3, dict(out_mm=0.0, clip=np.array([0.25, 0.1, TOP + 0.003]))), (HOLD + 5, dict(out_mm=0.0, clip=SEATED))], (0.0, False)),
    "flush, clip's tips 10 mm under their stops (not in the slots)": ([(HOLD + 5, dict(out_mm=0.0, clip=SEATED - np.array([0.0, 0.0, 0.010])))], (0.25, False)),
    "nothing done": ([(HOLD + 5, dict(out_mm=PROUD * 1000))], (0.0, False)),
}
bad = 0
for name, (steps, want) in CASES.items():
    got = run(steps)
    good = got == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: score {got[0]}, full {got[1]} (want {want[0]}, {want[1]})")
print("all good" if not bad else f"{bad} FAILED")
sys.exit(1 if bad else 0)
