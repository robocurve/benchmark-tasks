"""Near-successes, fixed habits and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). Every object is a parts.py Part, so each
case gives its state from Part.state(pos=..., quat_xyzw=...): pos is the part's origin (the bolt's shank end, the
gasket's underside centre, the upper flange's underside centre). Offline there is no robot, so the release check
passes; _run() below also checks the release rule with a gripper placed near a part.
"""

import math
import os

import task
from task import BOLTS, CX, CY, GAP, HOLES, LOW_TOP, RING, RING_START_X, SEATED_Z, SPACERS, TOP, UP_Z0, UPPER

os.environ[task.OFFLINE_FLAG] = "1"  # the task credits these offline cases as let go


def tilt_q(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def ring(dx=0.0, dy=0.0, dz=0.0, tilt=0.0, vel=None):
    return RING.state(pos=(CX + dx, CY + dy, TOP + LOW_TOP + dz), quat_xyzw=tilt_q(tilt), lin_vel=vel)


RING_OUT_FRONT = RING.state()  # never moved: still on the block in front


def bolt(i, hole, dx=0.0, dz=0.0, tilt=0.0):
    hx, hy = HOLES[hole]
    return BOLTS[i].state(pos=(CX + hx + dx, CY + hy, TOP + SEATED_Z + dz), quat_xyzw=tilt_q(tilt))


def four(**kw):
    return {"bolt_1": bolt(0, "far_left"), "bolt_2": bolt(1, "near_left"), "bolt_3": bolt(2, "far_right"),
            "bolt_4": bolt(3, "near_right"), **kw}


def far_only(**kw):
    return {"bolt_1": bolt(0, "far_left"), "bolt_3": bolt(2, "far_right"), **kw}


def near_only(**kw):
    return {"bolt_2": bolt(1, "near_left"), "bolt_4": bolt(3, "near_right"), **kw}


def upper(dx=0.0, dz=0.0):
    return UPPER.state(pos=(CX + dx, CY, TOP + UP_Z0 + dz))


def four_shifted(dx, **kw):
    """The upper flange slid dx along x on its spacers, every bolt in its hole moved with it."""
    return {"upper_flange": upper(dx=dx), "bolt_1": bolt(0, "far_left", dx=dx), "bolt_2": bolt(1, "near_left", dx=dx),
            "bolt_3": bolt(2, "far_right", dx=dx), "bolt_4": bolt(3, "near_right", dx=dx), **kw}


# Spacers out and the joint closed onto the gasket: the upper flange 4 mm above the lower one, the bolts with it.
CLOSE = -(GAP - 0.004)
CLOSED = {
    "upper_flange": upper(dz=CLOSE),
    "spacer_1": SPACERS[0].state(pos=(0.20, 0.10, TOP)), "spacer_2": SPACERS[1].state(pos=(0.20, -0.10, TOP)),
    **{f"bolt_{i + 1}": bolt(i, h, dz=CLOSE) for i, h in enumerate(("far_left", "near_left", "far_right", "near_right"))},
}

CASES = [
    ("gasket centred, four bolts in", four(gasket=ring()), 1.0, True),
    ("gasket 4.0 mm off the axis", four(gasket=ring(dx=-0.004)), 1.0, True),
    ("gasket 4.1 mm off the axis: no gasket, so no bolt credit", four(gasket=ring(dx=-0.0029, dy=0.0029)), 0.0, False),
    ("gasket 1.5 mm up (riding on something)", four(gasket=ring(dz=0.0015)), 1.0, True),
    ("gasket 2 mm up", four(gasket=ring(dz=0.002)), 0.0, False),
    ("gasket tilted 2.9 deg", four(gasket=ring(tilt=2.9)), 1.0, True),
    ("gasket tilted 3.1 deg", four(gasket=ring(tilt=3.1)), 0.0, False),
    ("gasket still moving", four(gasket=ring(vel=(0.02, 0.0, 0.0))), 0.0, False),
    # Partial credit: 0.2 for the centred gasket, 0.2 more per bolt, in any order.
    ("gasket centred, no bolts", {"gasket": ring()}, 0.2, False),
    ("gasket centred against the two far bolts, near holes empty", far_only(gasket=ring(dx=0.001)), 0.6, False),
    ("gasket centred, both far bolts and one near bolt", {**far_only(gasket=ring()), "bolt_2": bolt(1, "near_left")},
     0.8, False),
    ("gasket centred, both near bolts and one far bolt (the reviewer's case)",
     {**near_only(gasket=ring()), "bolt_1": bolt(0, "far_left")}, 0.8, False),
    ("gasket centred with only the near bolts in", near_only(gasket=ring()), 0.6, False),
    ("gasket centred, one far bolt only", {"gasket": ring(), "bolt_3": bolt(2, "far_right")}, 0.4, False),
    # Fixed habits and wrong outcomes.
    ("habit: all four bolts first, gasket left on its block", four(gasket=RING_OUT_FRONT), 0.0, False),
    ("habit: nearest holes first, gasket stopped against them short of the axis",
     near_only(gasket=ring(dx=-0.040)), 0.0, False),
    ("three bolts in, gasket left on its block", {**far_only(), "bolt_2": bolt(1, "near_left")}, 0.0, False),
    ("gasket pushed too far, 10 mm past the axis, four bolts in", four(gasket=ring(dx=0.010)), 0.0, False),
    # The loose upper flange: bolts are judged in its holes, wherever it is.
    ("upper flange slid 2 mm on its spacers, bolts in its holes", four_shifted(0.002, gasket=ring()), 1.0, True),
    ("upper flange slid 4 mm, bolts left 4 mm off its holes", {**four(gasket=ring()), "upper_flange": upper(dx=0.004)},
     0.2, False),
    ("spacers out, joint closed onto the centred gasket, four bolts in", {**CLOSED, "gasket": ring()}, 1.0, True),
    # Bolt limits.
    ("a bolt leaning 3 deg in its hole (head 2.6 mm off centre)",
     four(gasket=ring(), bolt_2=bolt(1, "near_left", tilt=3.0)), 1.0, True),
    ("a near bolt tilted 9 deg: not in", four(gasket=ring(), bolt_2=bolt(1, "near_left", tilt=9.0)), 0.8, False),
    ("a bolt head 2 mm above the flange", four(gasket=ring(), bolt_4=bolt(3, "near_right", dz=0.002)), 1.0, True),
    ("a near bolt head 2.5 mm up: not in",
     four(gasket=ring(), bolt_4=bolt(3, "near_right", dz=0.0025)), 0.8, False),
    ("a bolt 3 mm off its hole centre", four(gasket=ring(), bolt_1=bolt(0, "far_left", dx=0.003)), 1.0, True),
    ("a bolt 3.5 mm off its hole centre: not in", four(gasket=ring(), bolt_1=bolt(0, "far_left", dx=0.0035)), 0.8,
     False),
    ("gasket left where it starts, nothing done", {"gasket": RING.state(pos=(CX + RING_START_X, CY, TOP + LOW_TOP))},
     0.0, False),
]


def _release_cases(start) -> int:
    """The release rule, which check.py cannot reach offline: a gripper within 5 mm, or no measurement yet."""
    bad = 0
    full = {**start, **four(gasket=ring())}
    names = ["gasket", *task.BOLT_NAMES]
    checks = [
        ("gripper 4.9 mm from a near bolt: that bolt not counted", {n: 0.05 for n in names} | {"bolt_4": 0.0049}, 0.8),
        ("gripper 5.0 mm from a near bolt: counted", {n: 0.05 for n in names} | {"bolt_4": 0.005}, 1.0),
        ("gripper still on the gasket: nothing counted", {n: 0.05 for n in names} | {"gasket": 0.0}, 0.0),
    ]
    for label, gaps, want in checks:
        task._GAP.clear()
        task._GAP.update(gaps)
        got = 1.0 if task.success(full) else float(task.score(full))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._GAP.clear()
    os.environ[task.OFFLINE_FLAG] = "0"  # in the simulator, before the first step has measured the grippers
    got = float(task.score(full))
    os.environ[task.OFFLINE_FLAG] = "1"
    flag = "ok  " if got == 0.0 else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag} {got:4.2f}  no gripper measurement yet: fails closed")
    return bad


def _run() -> int:
    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = {**start, **changes}
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad + _release_cases(start)


if __name__ == "__main__":
    raise SystemExit(_run())
