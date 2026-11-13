"""The blow gate, the contact rule, the holding test and the scoring table on hand-built cases, no simulator: python check_score.py"""
import importlib.util
import math
import pathlib
import sys
import types

import numpy as np

# task.py imports the simulator's modules at import time; the gates (rim_blow, contact_blow, gripping) and the table (_tier) need none of them
for name in ("demokit", "mechkit", "yam_arena", "yam_arena.task_file"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
mk = sys.modules["mechkit"]
mk.Look = lambda **kw: kw
mk.rgb = lambda *a: a
for fn in ("Mesh", "box", "lathe", "col_box", "col_cylinder", "col_hull", "slider", "mechanism", "free", "fixed", "on_step", "on_reset"):
    setattr(mk, fn, lambda *a, **kw: None)
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)

bad = 0
gap = np.array([0.0, -1.0, 0.0])  # the open side, as in the scene: the small tin leans toward +y
top = task.H1


def at_rim(deg, dz=-0.004, out=0.0):
    """A fingertip on the rim, `deg` degrees round from the open side, `dz` from the tin's top, `out` beyond its outer radius."""
    a = math.radians(deg)
    rhat = np.array([math.sin(a), -math.cos(a), 0.0])  # 0 degrees: -y
    return (task.R1_TOP + out) * rhat + np.array([0.0, 0.0, top + dz]), rhat


def opens(deg, speed, dz=-0.004, out=0.0):
    q, rhat = at_rim(deg, dz, out)
    blow = task.rim_blow(q, -speed * rhat, gap)
    if blow is None:
        return "not at the rim"
    v_in, ang = blow
    if v_in >= task.V_DENT:
        return "dent"
    return "opens" if v_in >= task.V_OPEN and ang <= 45.0 else "holds"


gate = [
    ("a blow on the open side at 0.29 m/s, 15 degrees round (the demo's)", (15.0, 0.29), "opens"),
    ("the same blow 50 degrees round", (50.0, 0.29), "holds"),
    ("the same blow on the tight side, 180 degrees round", (180.0, 0.29), "holds"),
    ("a tap on the open side at 0.10 m/s, too soft", (0.0, 0.10), "holds"),
    ("a blow on the open side at 1.2 m/s", (0.0, 1.2), "dent"),
    ("a blow on the open side 3 cm below the rim (on the big tin's side, not the seam)", (0.0, 0.29, -0.03), "not at the rim"),
    ("a fingertip 6 mm above the big tin's rim, on the small tin", (0.0, 0.29, 0.006), "not at the rim"),
    ("a fingertip moving with the shaker (carried or rocked by a hand on it): no speed relative to the tin", (0.0, 0.0), "holds"),
]
for name, args, want in gate:
    got = opens(*args)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} gate {got:15} (want {want:15})  {name}")

# a blow needs contact: the fast fingertip reaches the wall (a finger point within 5 mm) that step or the next; (relative speed, angle, gap to the wall) per step
def lands(seq):
    pending, out = None, None
    for v, ang, gap in seq:
        landed, pending = task.contact_blow(pending, (v, ang), gap)
        out = out or landed
    return out is not None


contact = [
    ("the demo's blow, at the wall in the same step", [(0.29, 15.0, 0.0)], True),
    ("fast 8 mm out, at the wall the next step", [(0.29, 15.0, 0.008), (0.05, 15.0, 0.003)], True),
    ("a pulled punch: fast, then stopped 8 mm out", [(0.29, 15.0, 0.008), (0.0, 15.0, 0.008), (0.0, 15.0, 0.008)], False),
    ("the shaker jerked toward a fingertip hovering 8 mm off (its surface 4 mm off)", [(0.30, 15.0, 0.008), (0.20, 15.0, 0.008), (0.0, 15.0, 0.008)], False),
    ("fast, 2 steps from the wall", [(0.29, 15.0, 0.012), (0.29, 15.0, 0.008), (0.02, 15.0, 0.003)], True),
    ("a slow touch at the wall", [(0.10, 15.0, 0.008), (0.05, 15.0, 0.0)], False),
]
for name, seq, want in contact:
    got = lands(seq)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} lands {got!s:5} (want {want!s:5})  {name}")

# the same rule on fingertip points, measured to the wall's real surface (above the rim: to its top edge)
def lands_at(seq):
    pending, out = None, None
    for v, ang, q in seq:
        landed, pending = task.contact_blow(pending, (v, ang), task.wall_gap(q))
        out = out or landed
    return out is not None


rim = task.R1_TOP
swipes = [
    ("the demo's blow: fast 11 mm out at the rim, then at the wall", [(0.29, 4.0, (0.0, -(rim + 0.011), top - 0.004)), (0.05, 4.0, (0.0, -(rim + 0.002), top - 0.004))], True),
    ("a rising swipe that misses: fast 11.5 mm out at the rim, then 14 mm above it over the gap", [(0.17, 0.0, (0.0, -(rim + 0.0115), top + 0.0015)), (0.05, 0.0, (0.0, -(rim - 0.0035), top + 0.014))], False),
    ("the same, ending 20 mm above the rim", [(0.17, 0.0, (0.0, -(rim + 0.0115), top + 0.0015)), (0.05, 0.0, (0.0, -(rim - 0.006), top + 0.020))], False),
    ("fast at the rim, then onto the rim's top edge", [(0.29, 0.0, (0.0, -(rim + 0.009), top + 0.001)), (0.05, 0.0, (0.0, -(rim + 0.001), top + 0.004))], True),
]
for name, seq, want in swipes:
    got = lands_at(seq)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} lands {got!s:5} (want {want!s:5})  {name}")

# a hand holding a tin is not striking it: 12 fingertip points, 6 on each finger (tip and 3 cm up it, at 3 points across), in the tin's frame
def hand(half_open, z, y0=0.0):
    pts = []
    for sx in (-1.0, 1.0):
        for dx in (-0.018, 0.0, 0.018):
            for dy in (0.017, -0.013):
                pts.append((sx * (half_open + 0.0045), y0 + dy, z + dx))
    return np.array(pts)


r_small = task.r2(0.05)
grips = [
    ("fingers either side of the small tin, 2 cm above the seam", hand(r_small, 0.05), task.r2, task.H2, True),
    ("fingers either side of the big tin, 9 cm up", hand(task.r1(0.09), 0.09), task.r1, task.H1, True),
    ("a closed hand at the big tin's rim (fingers together, as in a blow)", hand(0.0, task.H1 - 0.004, y0=-task.R1_TOP - 0.01), task.r1, task.H1, False),
    ("an open hand 4 cm off the small tin's side", hand(r_small, 0.05, y0=-0.08), task.r2, task.H2, False),
]
for name, pts, rad, h, want in grips:
    got = task.gripping(pts, rad, 0.0, h)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} holding {got!s:5} (want {want!s:5})  {name}")

base = {"dent": False, "spill": False, "fell": False, "broken": False, "out": False, "big_set": False, "small_set": False}
cases = [
    ("opened, small tin out, both tins standing on the mat, hands clear", dict(broken=True, out=True, big_set=True, small_set=True), 1.0),
    ("opened, small tin out, the big tin set down, the small tin still in the hand", dict(broken=True, out=True, big_set=True), 0.75),
    ("opened, small tin out, the big tin still held", dict(broken=True, out=True), 0.5),
    ("seal broken by a strike, nothing more", dict(broken=True), 0.25),
    ("small tin out without the seal breaking (cannot happen in the scene: both latches hold it in)", dict(out=True, big_set=True, small_set=True), 0.0),
    ("opened and both tins set, but a blow over 1.0 m/s dented the tin", dict(broken=True, out=True, big_set=True, small_set=True, dent=True), 0.0),
    ("opened, then the big tin tipped past 45 degrees (the drink spilled)", dict(broken=True, out=True, spill=True), 0.0),
    ("a tin fell off the table", dict(broken=True, fell=True), 0.0),
    ("nothing done", dict(), 0.0),
]
for name, over, want in cases:
    got = task._tier({**base, **over})
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {name}")
sys.exit(1 if bad else 0)
