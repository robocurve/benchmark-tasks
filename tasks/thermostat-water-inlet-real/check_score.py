"""The score lines on hand-built states, with no simulator: python3 check_score.py. Runs task.py's own scorer (`measure`, `seated`, `flush`,
`on_studs`, the `Tracker` class and `score`), read from its source, against stand-ins for the simulator's state, one control step at a time."""

import ast
import contextlib
import io
import json
import math
import os
import pathlib
import types

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()
tree = ast.parse(src)
KEEP_FUNCS = {"measure", "seated", "flush", "on_studs", "_tier", "score", "success"}
TOP = 0.75
ns = {"math": math, "np": np, "json": json, "os": os, "TOP": TOP}
for node in tree.body:
    names = [e for t in getattr(node, "targets", []) for e in (t.elts if isinstance(t, ast.Tuple) else [t])]
    if isinstance(node, ast.Assign) and all(isinstance(e, ast.Name) and e.id.isupper() for e in names):
        try:
            exec(compile(ast.Module([node], []), "task.py", "exec"), ns)
        except Exception:
            pass  # constants that need the simulator (TOP, grips, looks) are not used by the scorer
    elif isinstance(node, (ast.FunctionDef, ast.ClassDef)) and (node.name in KEEP_FUNCS or node.name == "Tracker"):
        exec(compile(ast.Module([node], []), "task.py", "exec"), ns)
ns["TOP"] = TOP

GAPS = {"th": 0.2, "in": 0.2}  # what the gripper-to-part distances read this step, in m
ns["gripper_points"] = lambda env: None
ns["thermostat_gap"] = lambda pts, th: GAPS["th"]
ns["inlet_gap"] = lambda pts, inl: GAPS["in"]


def rot(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


class Part:
    def __init__(self, pos, R=np.eye(3)):
        self.pos, self.R = np.asarray(pos, float), np.asarray(R, float)

    def rotation(self):
        return self.R


STATE = {}
ns["m"] = types.SimpleNamespace(state=lambda env, name: STATE[name])
env = types.SimpleNamespace(unwrapped=types.SimpleNamespace(step_dt=0.05))
XF, ZC = ns["XF"], ns["ZC"]
BORE = np.array([XF, 0.0, TOP + ZC])

TH_HOME = Part(BORE + [0.003, 0.0, 0.0])
TH_BENCH = Part([0.300, -0.120, TOP + 0.05], rot([0, 1, 0], 90))
IN_AWAY = Part([0.30, 0.15, TOP + 0.10])
IN_FLUSH = Part(BORE + [0.0002, 0.0, 0.0])
IN_HANGING = Part(BORE + [-0.0072, 0.0, -0.001], rot([0, 1, 0], -11.0))  # pivoted on its lower edge until the holes bind, as in the demo
IN_OFF = Part([0.30, 0.10, TOP + 0.04])


def run(phases):
    """phases: (steps, thermostat Part, inlet Part, thermostat-to-hand m, inlet-to-hand m)."""
    T = ns["Tracker"]()
    for n, th, inl, gth, gin in phases:
        STATE["thermostat"], STATE["inlet"] = th, inl
        GAPS["th"], GAPS["in"] = gth, gin
        with contextlib.redirect_stdout(io.StringIO()):  # the tracker's own progress lines
            for _ in range(n):
                T.step(env)
    ns["TRACK"] = T
    return ns["score"](None), T.last


ALONE = (30, TH_HOME, IN_AWAY, 0.15, 0.0)  # thermostat seated and let go, the inlet still in the hand away from the housing
PUSHED = (30, TH_HOME, IN_FLUSH, 0.03, 0.0)  # inlet held flush over it
HANGS = (30, TH_HOME, IN_HANGING, 0.15, 0.08)  # let go, the inlet resting on the studs

CASES = [
    ("full", [ALONE, PUSHED, HANGS], 1.0),
    ("none: nothing moves", [(60, TH_BENCH, IN_OFF, 0.3, 0.3)], 0.0),
    ("inlet closed over an empty housing, thermostat on the bench", [(30, TH_BENCH, IN_AWAY, 0.2, 0.0), (30, TH_BENCH, IN_FLUSH, 0.2, 0.0), (30, TH_BENCH, IN_HANGING, 0.2, 0.1)], 0.0),
    ("thermostat in backwards (pellet out), then covered", [(30, Part(BORE + [0.003, 0, 0], rot([0, 0, 1], 180)), IN_AWAY, 0.15, 0.0),
                                                          (30, Part(BORE + [0.003, 0, 0], rot([0, 0, 1], 180)), IN_FLUSH, 0.15, 0.0),
                                                          (30, Part(BORE + [0.003, 0, 0], rot([0, 0, 1], 180)), IN_HANGING, 0.15, 0.1)], 0.0),
    ("thermostat cocked 6 degrees, inlet forced over it", [(30, Part(BORE + [0.003, 0, 0], rot([0, 1, 0], 6)), IN_AWAY, 0.15, 0.0),
                                                           (30, Part(BORE + [0.003, 0, 0], rot([0, 1, 0], 6)), IN_FLUSH, 0.15, 0.0),
                                                           (30, Part(BORE + [0.003, 0, 0], rot([0, 1, 0], 6)), IN_HANGING, 0.15, 0.1)], 0.0),
    ("thermostat held in by the hand until the inlet comes", [(30, TH_HOME, IN_AWAY, 0.0, 0.0), PUSHED, HANGS], 0.0),
    ("valve at the side (90 degrees)", [(30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 90)), IN_AWAY, 0.15, 0.0),
                                        (30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 90)), IN_FLUSH, 0.03, 0.0),
                                        (30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 90)), IN_HANGING, 0.15, 0.1)], 0.3),
    ("valve 9 degrees off top: within tolerance", [(30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 9)), IN_AWAY, 0.15, 0.0),
                                                   (30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 9)), IN_FLUSH, 0.03, 0.0),
                                                   (30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 9)), IN_HANGING, 0.15, 0.1)], 1.0),
    ("inlet held flush by the gripper at the end", [ALONE, PUSHED, (30, TH_HOME, IN_FLUSH, 0.15, 0.0)], 0.6),
    ("inlet hung on the studs but never pushed flush", [ALONE, (30, TH_HOME, IN_HANGING, 0.15, 0.0), HANGS], 0.6),
    ("inlet pushed flush, then dropped off the studs", [ALONE, PUSHED, (30, TH_HOME, IN_OFF, 0.15, 0.2)], 0.6),
    ("inlet flush for only 0.5 s", [ALONE, (10, TH_HOME, IN_FLUSH, 0.03, 0.0), HANGS], 0.6),
    ("seated, then knocked half out (void)", [ALONE, PUSHED, (30, Part(BORE + [-0.006, 0, 0]), IN_HANGING, 0.15, 0.1)], 0.0),
    ("seated and let go only", [ALONE], 0.6),
    ("seated, valve at the side, let go only", [(30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 90)), IN_AWAY, 0.15, 0.0)], 0.3),
    ("seated with the valve up, let go, then turned to the side in its seat", [ALONE, (30, Part(BORE + [0.003, 0, 0], rot([1, 0, 0], 90)), IN_AWAY, 0.15, 0.0)], 0.3),
    ("full, but the hand only 5 mm from the inlet at the end", [ALONE, PUSHED, (30, TH_HOME, IN_HANGING, 0.15, 0.005)], 0.6),
]

bad = 0
for name, phases, want in CASES:
    got, last = run(phases)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.1f} (want {want:.1f})  {name}")
hang = ns["measure"](TH_HOME, IN_HANGING, None)
print("hanging inlet as measured:", json.dumps({k: hang[k] for k in ("face_gap_mm", "stud_mm")}))
raise SystemExit(1 if bad else 0)
