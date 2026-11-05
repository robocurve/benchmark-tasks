"""Near-successes and unintended strategies, scored offline.

CASES (checked by factory/check.py and by running this file): (label, {object: (centre, size or None, velocity or
None)}, expected score, expected success), every object unrotated. ROTATED_CASES (checked by running this file):
(label, {object: (centre, quat_xyzw)}, expected score, expected success), with the bounding box worked out from the
rotation. Objects not listed stay where they started. The cases run in order and the scorer keeps trial history
(whether any of the four other bars has moved more than 10 mm or turned more than 5 degrees), cleared only when
every bar is back within 0.5 mm of its start after a jump of more than 20 mm between two calls (a reset). Cases that
set the history are followed by a reset case, as a new trial would be.
"""

import math

from task import (CROSS, CROSS_YS, PAD_SIZE, PAD_TOP, PAD_XY, PILE_X, RED_SIZE, RED_TOP, RED_Y, SIDE_TOP,
                  SIDE_XS, TOP)

MOVING = (0.0, 0.0, -0.05)
RZ = RED_SIZE[2] / 2


def cross_settled(dx=0.0, dy=0.0):
    """Both crossing bars lying on the side bars (dropped 4 mm from where the red bar held them)."""
    return {n: ((PILE_X + dx, y + dy, SIDE_TOP + CROSS[2] / 2), None, None) for n, y in CROSS_YS.items()}


def red_at(x, y, z_bottom, vel=None):
    return {"red_bar": ((x, y, z_bottom + RZ), None, vel)}


ON_PAD = red_at(PAD_XY[0], PAD_XY[1], PAD_TOP)
DONE = {**cross_settled(), **ON_PAD}
DRAWN = 0.290  # how far the demo draws it

LIMIT = 0.010 - 1e-9  # just inside a stated limit
OVER = 1e-5  # just past one


def cross_up(h):
    return {n: ((PILE_X, y, SIDE_TOP + h + CROSS[2] / 2), None, None) for n, y in CROSS_YS.items()}


PAD_EDGE_X = PAD_XY[0] + PAD_SIZE[0] / 2 + 0.001 - RED_SIZE[0] / 2  # red's far side on the pad's edge + 1 mm slack

CASES = [
    ("red on the pad, crossing bars down on the side bars", dict(DONE), 1.0, True),
    ("red on the pad 15 mm off centre along it, still wholly on the pad",
     {**cross_settled(), **red_at(PAD_XY[0], PAD_XY[1] + 0.015, PAD_TOP)}, 1.0, True),
    ("red's side exactly 1 mm past the pad's edge (the allowance)",
     {**cross_settled(), **red_at(PAD_EDGE_X, PAD_XY[1], PAD_TOP)}, 1.0, True),
    ("red's side 1.01 mm past the pad's edge: lying on the pad, clear of the pile",
     {**cross_settled(), **red_at(PAD_EDGE_X + OVER, PAD_XY[1], PAD_TOP)}, 0.5, False),
    ("red overhanging the pad's long edge by 5 mm, lying on the pad",
     {**cross_settled(), **red_at(PAD_XY[0] + PAD_SIZE[0] / 2 - RED_SIZE[0] / 2 + 0.005, PAD_XY[1], PAD_TOP)},
     0.5, False),
    ("red exactly 1 mm above the pad (the tolerance)",
     {**cross_settled(), **red_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.001)}, 1.0, True),
    ("red held 1.01 mm above the pad", {**cross_settled(), **red_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.001 + OVER)},
     0.0, False),
    ("red held 2.5 mm above the pad", {**cross_settled(), **red_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.0025)},
     0.0, False),
    ("red held 10 mm above the pad", {**cross_settled(), **red_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.010)},
     0.0, False),
    ("red on the pad, still moving", {**cross_settled(), **red_at(PAD_XY[0], PAD_XY[1], PAD_TOP, MOVING)},
     0.0, False),
    ("red on the pad, crossing bars still held up at their starting height", dict(ON_PAD), 0.0, False),
    ("red on the pad, crossing bars exactly 1 mm above the side bars", {**ON_PAD, **cross_up(0.001)}, 1.0, True),
    ("red on the pad, crossing bars held 1.01 mm above the side bars", {**ON_PAD, **cross_up(0.001 + OVER)},
     0.0, False),
    ("red on the pad, crossing bars held 1.5 mm above the side bars", {**ON_PAD, **cross_up(0.0015)}, 0.0, False),
    ("red on the pad, crossing bars held 3 mm above the side bars", {**ON_PAD, **cross_up(0.003)}, 0.0, False),
    ("red on the pad, crossing bars settled 8 mm off (pulled along a little)",
     {**ON_PAD, **cross_settled(dy=0.008)}, 1.0, True),
    ("red on the pad, near side bar moved just under 10 mm",
     {**DONE, "bar_near": ((SIDE_XS["bar_near"] - LIMIT, 0.0, TOP + 0.015), None, None)}, 1.0, True),
    ("red drawn out 290 mm and left on the table, clear of the pile",
     {**cross_settled(), **red_at(PILE_X, RED_Y + DRAWN, TOP)}, 0.5, False),
    ("red drawn out and laid on the table on the robot's right, clear of the pile",
     {**cross_settled(), **red_at(0.20, -0.25, TOP)}, 0.5, False),
    ("red drawn 200 mm: out from under the crossing bars, still between the side bars",
     {**cross_settled(), **red_at(PILE_X, RED_Y + 0.200, TOP)}, 0.25, False),
    ("red drawn 150 mm: crossing bars still on it", red_at(PILE_X, RED_Y + 0.150, TOP), 0.0, False),
    ("red left lying on top of the crossing bars",
     {**cross_settled(), **red_at(PILE_X, 0.0, SIDE_TOP + CROSS[2])}, 0.0, False),
    ("nothing moved (a new trial)", {}, 0.0, False),
    # Cases that set the trial history, each followed by a reset (see the docstring).
    ("red drawn 150 mm and the crossing bars carried 150 mm with it",
     {**red_at(PILE_X, RED_Y + 0.150, TOP),
      **{n: ((PILE_X, y + 0.150, RED_TOP + CROSS[2] / 2), None, None) for n, y in CROSS_YS.items()}}, 0.0, False),
    ("reset: everything back at the start in one jump", {}, 0.0, False),
    ("red on the pad, near side bar moved 10.01 mm", {**DONE, "bar_near": ((SIDE_XS["bar_near"] - 0.010 - OVER, 0.0,
                                                                            TOP + 0.015), None, None)}, 0.0, False),
    ("then the side bar put back: still 0", dict(DONE), 0.0, False),
    ("reset", {}, 0.0, False),
    ("by hand: crossing bars lifted 25 mm off the red bar", cross_up(RED_TOP - SIDE_TOP + 0.025), 0.0, False),
    ("by hand: lowered to 1 mm above the red bar", cross_up(RED_TOP - SIDE_TOP + 0.001), 0.0, False),
    ("by hand: set back exactly where they started (no jump, so no reset)", {}, 0.0, False),
    ("then red drawn and put on the pad, crossing bars down on the side bars: still 0", dict(DONE), 0.0, False),
    ("reset", {}, 0.0, False),
    ("red on the pad, far side bar pushed 12 mm",
     {**DONE, "bar_far": ((SIDE_XS["bar_far"] + 0.012, 0.0, TOP + 0.015), None, None)}, 0.0, False),
    ("far side bar then pushed back, red on the pad: still 0", dict(DONE), 0.0, False),
    ("a crossing bar knocked onto the table", {**DONE, "cross_left": ((0.40, 0.10, TOP + CROSS[2] / 2), None, None)},
     0.0, False),
    ("reset", {}, 0.0, False),
]


def quat_z(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


def quat_x(deg):
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def quat_y(deg):
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


I = (0.0, 0.0, 0.0, 1.0)
CROSS_DOWN = {n: ((PILE_X, y, SIDE_TOP + CROSS[2] / 2), I) for n, y in CROSS_YS.items()}
RED_PAD = {"red_bar": ((PAD_XY[0], PAD_XY[1], PAD_TOP + RZ), I)}

ROTATED_CASES = [
    ("red on the pad yawed 3 degrees, footprint still inside the pad",
     {**CROSS_DOWN, "red_bar": ((PAD_XY[0], PAD_XY[1], PAD_TOP + RZ), quat_z(3.0))}, 1.0, True),
    ("red rolled onto its side on the pad (30 mm tall)",
     {**CROSS_DOWN, "red_bar": ((PAD_XY[0], PAD_XY[1], PAD_TOP + RED_SIZE[0] / 2), quat_y(90.0))}, 1.0, True),
    ("red standing on its end on the pad",
     {**CROSS_DOWN, "red_bar": ((PAD_XY[0], PAD_XY[1], PAD_TOP + RED_SIZE[1] / 2), quat_x(90.0))}, 0.0, False),
    ("red leaning on the pad, one end raised (tilted 5 degrees along its length)",
     {**CROSS_DOWN, "red_bar": ((PAD_XY[0], PAD_XY[1], PAD_TOP + 0.030), quat_x(5.0))}, 0.0, False),
    ("red on the pad, a crossing bar turned 4.99 degrees", {**CROSS_DOWN, **RED_PAD,
     "cross_left": ((PILE_X, CROSS_YS["cross_left"], SIDE_TOP + CROSS[2] / 2), quat_z(4.99))}, 1.0, True),
    ("red on the pad, a crossing bar turned 5.01 degrees", {**CROSS_DOWN, **RED_PAD,
     "cross_left": ((PILE_X, CROSS_YS["cross_left"], SIDE_TOP + CROSS[2] / 2), quat_z(5.01))}, 0.0, False),
    ("then turned back: still 0", {**CROSS_DOWN, **RED_PAD}, 0.0, False),
    ("reset", {}, 0.0, False),
    ("red on the pad, a side bar tipped over onto its side", {**CROSS_DOWN, **RED_PAD,
     "bar_far": ((SIDE_XS["bar_far"] + 0.01, 0.0, TOP + 0.015), quat_y(90.0))}, 0.0, False),
    ("reset", {}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def rot(q):
        x, y, z, w = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    def state(pos, size, vel=None, quat=I):
        pos = np.asarray(pos, float)
        half = np.abs(rot(quat)) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, np.asarray(quat, float), v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    task.score(start)  # a new trial: clears the history
    cases = [(label, {n: (p, None, v) for n, (p, _, v) in ch.items()}, {}, want, ws) for label, ch, want, ws in CASES]
    cases += [(label, {}, ch, want, ws) for label, ch, want, ws in ROTATED_CASES]
    bad = 0
    for label, plain, rotated, want, want_success in cases:
        st = dict(start)
        for name, (pos, sz, vel) in plain.items():
            st[name] = state(pos, sizes[name] if sz is None else sz, vel)
        for name, (pos, quat) in rotated.items():
            st[name] = state(pos, sizes[name], None, quat)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
