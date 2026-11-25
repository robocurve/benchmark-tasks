"""The scoring table on hand-built records, no simulator: python check_score.py"""
import importlib.util
import pathlib
import sys
import types

# task.py imports the simulator's modules at import time; the scoring table (_tier) needs none of them
for name in ("demokit", "mechkit", "yam_arena", "yam_arena.task_file"):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
mk = sys.modules["mechkit"]
mk.Look = lambda **kw: kw
mk.rgb = lambda *a: a
mk.ENV_PATH = "/World/envs/env_0"
mk.set_visible = lambda *a, **kw: None
mk.Mesh = object
for fn in ("box", "cylinder", "col_box", "hinge", "slider", "mechanism", "on_step", "on_reset"):
    setattr(mk, fn, lambda *a, **kw: None)
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)

base = {"pierced": False, "torn": False, "gun_fell": False, "fastened": False, "out": False, "clear": False, "through": False, "lifted_mm": 0.0}
cases = [
    ("the full solution: fastened through the label alone, needle out, label let go", dict(fastened=True, out=True, clear=True, through=True, lifted_mm=52.0), 1.0),
    ("fastened, but the needle still in the label", dict(fastened=True, out=False, clear=True, through=True, lifted_mm=52.0), 0.5),
    ("fastened and out, but the label still held up", dict(fastened=True, out=True, clear=False, through=True, lifted_mm=52.0), 0.5),
    ("through the label, never fired", dict(through=True, out=True, clear=True, lifted_mm=52.0), 0.5),
    ("label lifted 2 cm, nothing more", dict(lifted_mm=21.0), 0.25),
    ("label lifted under 2 cm (its curl alone)", dict(lifted_mm=8.3), 0.0),
    ("fired with the needle through the label, but the shirt pierced on the way", dict(fastened=True, out=True, clear=True, through=True, lifted_mm=52.0, pierced=True), 0.0),
    ("label torn past 110 degrees", dict(through=True, lifted_mm=60.0, torn=True), 0.0),
    ("gun knocked off the table", dict(lifted_mm=52.0, gun_fell=True), 0.0),
]
bad = 0
# the firing gate on hand-built needle poses: the label hinged at the origin along x, lifted 40 degrees (seam height 0)
import math
import numpy as np
a = math.radians(40.0)
Rl = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])
hinge = np.zeros(3)
nrm = Rl[:, 2]  # the label's face normal
def at(u):
    return Rl @ np.array([0.0, u, 0.0])
gate = [
    ("needle through the label's face 38 mm out (2.4 cm clear)", at(0.038) - 0.004 * nrm, at(0.038) + 0.021 * nrm, True),
    ("the same needle entering from the label's underside", at(0.038) + 0.004 * nrm, at(0.038) - 0.021 * nrm, True),
    ("needle through the label 20 mm out (1.3 cm clear: too low)", at(0.020) - 0.004 * nrm, at(0.020) + 0.021 * nrm, False),
    ("needle slipped under the label from its free edge, tip under it", at(0.040) - 0.004 * nrm, at(0.065) - 0.004 * nrm, False),
    ("needle passing beside the label (outside its 30 mm width)", at(0.038) + np.array([0.02, 0.0, 0.0]) - 0.004 * nrm, at(0.038) + np.array([0.02, 0.0, 0.0]) + 0.021 * nrm, False),
    ("needle under the label, not through it (both ends below its face)", at(0.038) - 0.004 * nrm, at(0.038) - 0.010 * nrm - Rl @ np.array([0.0, 0.02, 0.0]), False),
]
for name, tip, base_pt, want in gate:
    got, clear = task.needle_through(tip, base_pt, hinge, Rl, 0.0)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} gate {got!s:5} (want {want!s:5}, label {clear * 1000:.0f} mm clear)  {name}")
# the needle's entry on hand-built paths, step by step (the needle has no collider, so a sideways slide into the label must not count)
ex = Rl[:, 0]  # across the label's width
def path(points):
    """points: (tip, base) per step; returns (through at the end, torn at any step)."""
    entered, prev, torn = False, None, False
    for tip, base_pt in points:
        entered, prev, ripped = task.track_entry(entered, prev, tip, base_pt, hinge, Rl)
        torn = torn or ripped
    got, _ = task.needle_through(points[-1][0], points[-1][1], hinge, Rl, 0.0)
    return bool(got and entered), torn
def needle(u, x, h):
    tip = at(u) + x * ex + h * nrm
    return tip, tip + 0.025 * nrm
pierce = [needle(0.038, 0.0, h) for h in (0.008, 0.004, 0.001, -0.002, -0.004)]
entry = [
    ("pushed through the label's face 38 mm out", pierce, (True, False)),
    ("slid in sideways from beside the label, straddling its plane (x 20 mm to 0)", [needle(0.038, x, -0.004) for x in (0.020, 0.016, 0.014, 0.008, 0.0)], (False, False)),
    ("slid in from beyond the free edge, straddling its plane (u 66 mm to 38)", [needle(u, 0.0, -0.004) for u in (0.066, 0.061, 0.059, 0.050, 0.038)], (False, False)),
    ("pushed through, then dragged sideways out through the label's edge", pierce + [needle(0.038, x, -0.004) for x in (0.008, 0.014, 0.016)], (False, True)),
    ("pushed through, then drawn back out", pierce + [needle(0.038, 0.0, h) for h in (0.001, 0.006)], (False, False)),
]
for name, pts, want in entry:
    got = path(pts)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} entry (through, torn) {got} (want {want})  {name}")
# the let-go check on hand-built fingertips (12 per hand, left then right), the label's middle at 30 mm out
mid = at(0.030)
away = np.tile(mid + np.array([0.0, -0.12, 0.10]), (24, 1))
pinned = away.copy()
pinned[18] = mid + 0.003 * nrm  # one right fingertip on the label
lgo = [
    ("both hands 12 cm off, label let down to 20 degrees", away, math.radians(20.0), True),
    ("left hand off, right hand pinning the label down at 20 degrees", pinned, math.radians(20.0), False),
    ("both hands off, label still standing at 40 degrees", away, math.radians(40.0), False),
]
for name, tips, ang, want in lgo:
    got = task.let_go(tips, mid, ang)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} let go {got!s:5} (want {want!s:5})  {name}")
for name, over, want in cases:
    got = task._tier({**base, **over})
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {name}")
sys.exit(1 if bad else 0)
