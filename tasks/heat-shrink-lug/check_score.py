"""The scorer on hand-built states, with no simulator: the tiers, the 2 s hold, the grasp and the stand gate.

    python check_score.py

Exits non-zero if any case gives another value than the scoring text.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import types

import numpy as np

for name in ("demokit", "mechkit", "yam_arena", "yam_arena.task_file"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
mk = sys.modules["mechkit"]
mk.Look = lambda **kw: kw
mk.rgb = lambda *a: a
mk.ENV_PATH = "/W"
mk.set_visible = lambda *a, **kw: None
for fn in ("box", "cylinder", "col_box", "col_hull", "fixed", "free", "on_step", "on_reset"):
    setattr(mk, fn, lambda *a, **kw: None)
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

bad = 0


def check(label, got, want):
    global bad
    ok = got == want if isinstance(want, bool) else abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {label}: {got} (want {want})")


# the tiers
base = dict(n_rec=8, charred=False, ends=True, on_stand=True, off=True, clear=True, fell=False, split=False, hand=0.0)
for label, change, want in (("all done", {}, 1.0), ("never switched off", dict(off=False), 0.9), ("a hand within 5 cm", dict(clear=False), 0.9),
                            ("gun not in its stand", dict(on_stand=False), 0.9), ("one station charred", dict(charred=True), 0.8),
                            ("7 of 8, both ends", dict(n_rec=7), 0.9375), ("7 of 8, one end", dict(n_rec=7, ends=False), 0.7375),
                            ("4 of 8, one end", dict(n_rec=4, ends=False), 0.55), ("3 of 8", dict(n_rec=3), 0.1875), ("1 of 8", dict(n_rec=1), 0.0625), ("none", dict(n_rec=0, ends=False), 0.0), ("split", dict(split=True), 0.0),
                            ("fell", dict(fell=True), 0.0), ("fingers' dose 3.01", dict(hand=3.01), 0.0), ("fingers' dose 3.0", dict(hand=3.0), 1.0)):
    check(f"tier, {label}", t._moment(dict(base, **change)), want)

# the hold: at most 0.95 until the 1.0 state has held 40 steps, then success
tr = t.TRIAL  # the one success() reads
tr.reset()
t.now = lambda env: dict(base)
for _ in range(39):
    tr.step(None)
check("after 39 steps at 1.0", tr.score, 0.95)
tr.step(None)
check("after 40 steps at 1.0", tr.score, 1.0)
check("success after 40 steps", t.success(None), True)


class Pose:
    """A gun pose: the nozzle's tip at `pos`, turned by `R`."""

    def __init__(self, pos, R=None):
        self.pos, self.R = np.asarray(pos, float), t._gun_R() if R is None else R

    def to_world(self, P):
        return self.pos + np.asarray(P) @ self.R.T

    def to_local(self, P):
        return (np.asarray(P) - self.pos) @ self.R

    def rotation(self):
        return self.R


# the stand gate: the gun is in its stand while any of it lies on the stand's blocks
rest = t.NOZZLE_REST + np.array([0.0, 0.0, t.TOP])
for label, where, want in (("at rest", rest, True), ("slid 3.5 cm along its cradles", rest + [0, 0.035, 0], True),
                           ("slid 5 cm along its cradles", rest + [0, 0.05, 0], True), ("lifted 5 mm", rest + [0, 0, 0.005], True),
                           ("lifted 1.5 cm", rest + [0, 0, 0.015], False), ("12 cm forward, clear of every block", rest + [0.12, 0, 0], False),
                           ("held at the work line", np.array([0.30, -0.02, 0.16 + t.TOP]), False)):
    check(f"on its stand, {label}", t.on_its_stand(Pose(where)), want)


# the split: any point of the nozzle (tip, rim, side, the barrel's front rim) within 3 mm of the sleeve's surface, or inside it
import math  # noqa: E402

cable = Pose(np.array([t.X0, t.CY, 0.16]), np.eye(3))  # the cable level along x, at the work line
x_mid = t.X0 + t.sec_x(3)


def gun_at(dist, tilt_deg):
    """The nozzle's tip `dist` from the cable's axis on its -y side, the jet towards the axis and tilted `tilt_deg` up about x."""
    a = math.radians(tilt_deg)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])
    return Pose(np.array([x_mid, t.CY - dist, 0.16]), Rx @ t._gun_R())


none_done, all_done = [False] * t.N_SEC, [True] * t.N_SEC
for label, g, rec, want in (("the demo's working pose: 50 mm, tilted 13 degrees", gun_at(0.050, 13.0), none_done, False),
                            ("25 mm off the sleeve's axis, square", gun_at(0.025, 0.0), none_done, False),
                            ("the tip 12 mm from the axis, square (on the sleeve)", gun_at(0.012, 0.0), none_done, True),
                            ("the tip 14 mm from the axis, tilted 45 degrees (the rim in the sleeve)", gun_at(0.014, 45.0), none_done, True),
                            ("the tip 16 mm from the axis, square, all recovered (6 mm off the shrunk surface)", gun_at(0.016, 0.0), all_done, False),
                            ("the tip 9.5 mm from the axis, square, all recovered (2 mm off it)", gun_at(0.0095, 0.0), all_done, True)):
    check(f"split, {label}", t.nozzle_on_sleeve(cable, g, rec), want)


# the grasp: one hand's two fingers either side of the grip
def tips(half, y=-t.GOFF, z=-0.09):
    P = np.full((24, 3), 9.0)
    P[12:18] = (+half, y, z)
    P[18:24] = (-half, y, z)
    return P


g = Pose(np.zeros(3), np.eye(3))
for half, want in ((0.0045, False), (0.0125, True), (0.0185, True), (0.023, False), (0.04, False)):
    check(f"grasped, fingertip points at +-{1000 * half:.1f} mm across the 28 mm grip", t._grasped(g, tips(half)), want)
P = np.full((24, 3), 9.0)
P[0], P[12] = (0.02, -t.GOFF, -0.09), (-0.02, -t.GOFF, -0.09)
check("grasped, one finger of each hand", t._grasped(g, P), False)

print("all cases as the scoring text" if not bad else f"{bad} cases differ")
sys.exit(1 if bad else 0)
