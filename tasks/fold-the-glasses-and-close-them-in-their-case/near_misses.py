"""Near-successes, unintended strategies and fixed habits, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: spec}, expected score, expected success). Specs are ready states from each part's own
kinematics (GLASSES.state(...), CASE.state(...)).
"""

import math

import numpy as np

from task import CASE, CX, CY, FLOOR_Z, FOLDED, GLASSES, GX, GY, IN_X, IN_Y, TOP

F = math.pi / 2  # fully folded
SHUT = -math.pi / 2


def _qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _qy(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def glasses(right=F, left=F, x=GX, y=GY, z=TOP, quat=None, vel=None, rest=False):
    """The glasses posed at (x, y, z); rest: then moved up or down so their lowest point is at z."""
    j = {"right_temple": right, "left_temple": left}
    s = GLASSES.state(joints=j, pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)
    if rest:
        s = GLASSES.state(joints=j, pos=(x, y, z + z - s.low[2]), quat_xyzw=quat, lin_vel=vel)
    return s


def in_case(right=F, left=F, dx=0.0, dy=0.0, dz=0.0, quat=None, vel=None):
    """Folded glasses standing upright in the case, the front 15 mm from its near inner wall, centred along it."""
    return glasses(right, left, x=CX - IN_X / 2 + 0.015 + dx, y=CY + dy, z=FLOOR_Z + dz, quat=quat, vel=vel)


def lying_in_case():
    """Folded glasses lying lenses down in the case (front turned up to face the sky)."""
    return glasses(x=CX - 0.020, y=CY, z=FLOOR_Z, quat=_qy(90), rest=True)


def lid(q):
    return CASE.state(joints={"lid": q})


SHUT_CASE = {"case": lid(SHUT)}
MOVING = (0.0, 0.0, -0.05)

CASES = [
    ("folded, in the case, lid shut", {"glasses": in_case(), **SHUT_CASE}, 1.0, True),
    ("folded, in the case, lid 3 deg from shut", {"glasses": in_case(), "case": lid(SHUT + math.radians(3.0))},
     1.0, True),
    ("folded, in the case, lid 4 deg open", {"glasses": in_case(), "case": lid(SHUT + math.radians(4.0))},
     0.5, False),
    ("folded, in the case, lid still open", {"glasses": in_case()}, 0.5, False),
    ("in the case, lid shut, still moving", {"glasses": in_case(vel=MOVING), **SHUT_CASE}, 0.0, False),
    ("temples at the 15 deg limit: their tips stick out past the case wall", {"glasses": in_case(FOLDED, FOLDED)},
     0.25, False),
    ("temples at the 15 deg limit, standing on the table", {"glasses": glasses(FOLDED, FOLDED)}, 0.25, False),
    ("left temple 16 deg short of folded, in the case",
     {"glasses": in_case(F, FOLDED - math.radians(1.0)), **SHUT_CASE}, 0.0, False),
    ("folded glasses lying lenses down in the case, lid shut", {"glasses": lying_in_case(), **SHUT_CASE}, 1.0, True),
    ("in the case but held 3 mm above its floor", {"glasses": in_case(dz=0.003), **SHUT_CASE}, 0.0, False),
    ("on the case floor, turned 20 deg (an end over the wall)",
     {"glasses": in_case(quat=_qz(20), dy=0.0), **SHUT_CASE}, 0.25, False),
    ("folded, standing on the table", {"glasses": glasses()}, 0.25, False),
    ("folded, held 1.5 mm above the table", {"glasses": glasses(z=TOP + 0.0015)}, 0.0, False),
    ("folded, in the case, held 1.5 mm above its floor, lid shut", {"glasses": in_case(dz=0.0015), **SHUT_CASE},
     0.0, False),
    ("folded, in the case, 1 mm above its floor (settling), lid shut", {"glasses": in_case(dz=0.001), **SHUT_CASE},
     1.0, True),
    ("folded, leaning 19 deg on the table", {"glasses": glasses(quat=_qx(19), rest=True)}, 0.25, False),
    ("folded, leaning 21 deg on the table", {"glasses": glasses(quat=_qx(21), rest=True)}, 0.0, False),
    ("knocked over: folded, lying lenses down on the table", {"glasses": glasses(z=TOP, quat=_qy(90), rest=True)},
     0.0, False),
    ("then picked up and closed in the case: still 0 (knocked over earlier)", {"glasses": in_case(), **SHUT_CASE},
     0.0, False),
    ("scene reset", {}, 0.0, False),
    ("folded, standing on the table, lid shut on the empty case", {"glasses": glasses(), **SHUT_CASE}, 0.25, False),
    ("folded, held 3 mm above the table", {"glasses": glasses(z=TOP + 0.003)}, 0.0, False),
    ("habit: only one temple folded (the nearer, right one)", {"glasses": glasses(F, 0.0)}, 0.0, False),
    ("habit: only the left temple folded", {"glasses": glasses(0.0, F)}, 0.0, False),
    ("habit: lid shut first, glasses untouched", {**SHUT_CASE}, 0.0, False),
    ("habit: open glasses set across the case, lid open", {"glasses": glasses(0.0, 0.0, x=CX - IN_X / 2 + 0.01,
                                                                              y=CY, z=TOP + 0.048)}, 0.0, False),
    ("open glasses knocked flat on the table", {"glasses": glasses(0.0, 0.0, z=TOP, quat=_qy(90), rest=True)}, 0.0,
     False),
    ("scene reset again", {}, 0.0, False),
    ("after the reset, folded and closed in the case", {"glasses": in_case(), **SHUT_CASE}, 1.0, True),
    ("folded, knocked off the table", {"glasses": glasses(x=0.3, y=0.9, z=TOP - 0.3)}, 0.0, False),
]


def _run() -> int:
    import task

    base = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(base) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
