"""Edge cases for task.py's scorer, fed set poses and contact forces step by step (no physics, no simulator start-up).

    python scorer_check.py        # in the Isaac Lab-Arena environment, from this folder; prints one line per case

The plate and the food stay at their starting poses; the towel pad lies flat on the rim (its underside on the rim's
surface) wherever a case puts it, pressing with the force given. Each case checks the score and the reason.
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("task_under_check", HERE / "task.py")
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
kk = sys.modules["kitchenkit"]

PLATE = np.array(t.PLATE_POS, float)
Q0 = np.array([0.0, 0.0, 0.0, 1.0])
FAR_AWAY = {a: np.tile([0.0, s * 0.30, 1.20], (24, 1)) for a, s in (("left_robot", 1), ("right_robot", -1))}
TANGENT = {"far": (0.7071, 0.7071), "right": (1.0, 0.0), "near": (0.0, 1.0)}


def start() -> None:
    t._reset()
    kk.STATE.clear()


def step(xy, force=1.5, contacts=None, fingers=None, lift=0.0) -> float:
    """One control step: the towel's underside centre at plate-frame (x, y) on the rim surface (+ lift)."""
    towel = PLATE + [xy[0], xy[1], t.PLATE_T + lift]
    kk.STATE.step += 1
    kk.STATE.poses = {"plate": (PLATE.copy(), Q0), "towel": (towel, Q0), "food": (np.array(t.FOOD_POS, float), Q0)}
    kk.STATE.contacts = {("towel", "plate"): force, **(contacts or {})}
    kk.STATE.peaks = dict(kk.STATE.contacts)
    kk.STATE.fingers = fingers or FAR_AWAY
    return t.score(None)


def stroke(k, offset=0.0, amp=0.02, step_len=0.005, passes=3, lateral=0.0, force=1.5):
    """Rub smudge k: the pad's centre runs along the rim from -amp to +amp (around offset) and back, passes times."""
    sx, sy = t.SMUDGES[k]
    u = np.array(TANGENT[k])
    n = np.array([sx, sy]) / math.hypot(sx, sy)
    path = np.arange(-amp, amp + 1e-9, step_len)
    for i in range(passes):
        for s in (path if i % 2 == 0 else path[::-1]):
            step(np.array([sx, sy]) + (offset + s) * u + lateral * n, force=force)


def clean_all():
    for k in t.SMUDGES:
        stroke(k)


def food_contact(who, newtons):
    return {("food", t.FOOD_WATCH[who]): newtons}


CASES = []


def case(name, expect):
    def deco(fn):
        CASES.append((name, expect, fn))
        return fn
    return deco


@case("whole smudges wiped (three passes of 4 cm strokes over each, 1.5 N)", 1.0)
def _full():
    clean_all()


@case("one smudge left", 0.67)
def _two():
    stroke("near")
    stroke("right")


@case("short strokes over one side of each smudge (the previous rule's loophole)", 0.0)
def _partial():
    for k in t.SMUDGES:
        stroke(k, offset=0.015, amp=0.005, step_len=0.0025, passes=8)


@case("the same strokes pressed with 1.0 N (below the 1.2 N threshold)", 0.0)
def _soft():
    for k in t.SMUDGES:
        stroke(k, force=1.0)


@case("towel pressed on each smudge and nudged 1.5 mm (a dab, not a wipe)", 0.0)
def _dab():
    for k, (sx, sy) in t.SMUDGES.items():
        for i in range(4):
            step(np.array([sx, sy]) + [0.0005 * i, 0.0])


@case("strokes 2 mm above the rim (no pressure)", 0.0)
def _light():
    for k in t.SMUDGES:
        sx, sy = t.SMUDGES[k]
        for s in np.arange(-0.02, 0.0201, 0.005):
            step(np.array([sx, sy]) + s * np.array(TANGENT[k]), force=0.0, lift=0.002)


@case("whole smudges wiped, then the right gripper body brushes the food (0.05 N)", 0.0)
def _body():
    clean_all()
    step((-0.2, 0.0), force=0.0, contacts=food_contact("right gripper body", 0.05))


@case("whole smudges wiped, then the left finger tip_right brushes the food (0.05 N)", 0.0)
def _left_finger():
    clean_all()
    step((-0.2, 0.0), force=0.0, contacts=food_contact("left finger tip_right", 0.05))


@case("whole smudges wiped, then the towel's side 8 mm from the food's edge (no contact)", 0.0)
def _towel_near():
    clean_all()
    step((-(t.FOOD_D / 2 + 0.008 + t.TOWEL[0] / 2), 0.0), force=0.0, lift=0.002)


@case("whole smudges wiped, then a finger pad 7.5 mm from the food's side (no contact)", 0.0)
def _pad_near():
    clean_all()
    f = {"left_robot": FAR_AWAY["left_robot"],
         "right_robot": np.tile(PLATE + [-(t.FOOD_D / 2 + 0.0075), 0.0, t.PLATE_T + 0.015], (24, 1))}
    step((-0.2, 0.0), force=0.0, fingers=f)


@case("whole smudges wiped, a contact of 0.01 N on the food (below the 0.02 N threshold)", 1.0)
def _tiny():
    clean_all()
    step((-0.2, 0.0), force=0.0, contacts=food_contact("towel", 0.01))


def main() -> int:
    bad = 0
    for name, expect, fn in CASES:
        start()
        fn()
        got = t.score(None)
        r = t.report()
        old = r["old_rules_not_scored"]
        ok = abs(got - expect) < 1e-9
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {got:.2f} (want {expect:.2f})  {name}: coverage "
              f"{ {k: round(v, 2) for k, v in r['coverage'].items()} }, food {r['food_at']}; previous rules: smudges "
              f"{old['would_clean']}, food check {'fired' if old['food_box_hit'] else 'silent'}")
    print("all cases pass" if not bad else f"{bad} case(s) failed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
