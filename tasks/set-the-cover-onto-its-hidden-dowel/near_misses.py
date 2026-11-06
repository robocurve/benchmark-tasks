"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success).
"""

import math

import numpy as np

from task import COVER, COVER_PART, COVER_START, DOWEL_AT, DOWEL_L, H_TOP, HOLE_XY, TOP

COCK = math.atan2(DOWEL_L, COVER[0] / 2 - DOWEL_AT[0])  # 4.36 degrees geometric (dowel axis to far edge); 4.58 measured in the simulator


def _q(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(a * math.sin(angle / 2)), math.cos(angle / 2))


def cover(x, y, z=H_TOP, quat=(0.0, 0.0, 0.0, 1.0), vel=None):
    return {"cover": COVER_PART.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)}


def cocked(cx, cy):
    """Resting tilted on its dowel tip and its far bottom edge (4.36 degrees geometric), centre at (cx, cy)."""
    return cover(cx, cy, H_TOP + (COVER[0] / 2) * math.sin(COCK), _q((0, 1, 0), COCK))


def seated_turned(yaw, off=(0.0, 0.0)):
    """Flat, with the dowel at the hole centre plus `off`, turned `yaw` about the dowel."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = DOWEL_AT
    x = HOLE_XY[0] + off[0] - (c * dx - s * dy)
    y = HOLE_XY[1] + off[1] - (s * dx + c * dy)
    return cover(x, y, H_TOP, _q((0, 0, 1), yaw))


SX = HOLE_XY[0] - DOWEL_AT[0]  # cover centre x when seated
Y = HOLE_XY[1]

CASES = [
    ("seated: flat, dowel in the hole", seated_turned(0.0), 1.0, True),
    ("seated but still moving", cover(SX, Y, vel=(0.05, 0.0, 0.0)), 0.0, False),
    ("seated, turned 20 degrees about the dowel (one dowel does not set the turn)", seated_turned(math.radians(20)),
     1.0, True),
    ("dowel 1.5 mm off the hole centre in x (against the hole wall)", seated_turned(0.0, (0.0015 - 1e-7, 0.0)), 1.0,
     True),
    ("dowel 1.5 mm off in y", seated_turned(0.0, (0.0, -0.0015 + 1e-7)), 1.0, True),
    ("dowel 2 mm off: not in the hole", seated_turned(0.0, (0.002, 0.0)), 0.0, False),
    ("cocked on its dowel at the start position", cocked(*COVER_START), 0.0, False),
    ("cocked, centred over the hole (dowel 40 mm short of it)", cocked(HOLE_XY[0], Y), 0.0, False),
    ("cocked just short of the hole", cocked(SX - 0.010, Y), 0.0, False),
    ("cocked after a straight push along x (dowel 16 mm to the side)", cocked(SX + 0.022, Y - 0.016), 0.0, False),
    ("flat, centred side to side on the housing, dowel 33 mm off the hole", cover(SX, -0.010), 0.0, False),
    ("flat, centred on the housing", cover(0.330, -0.010), 0.0, False),
    ("cocked after a straight push along y (dowel 17 mm short)", cocked(SX - 0.017, Y + 0.019), 0.0, False),
    ("tilted 1.0 degree across, dowel in hole", cover(SX, Y, H_TOP, _q((1, 0, 0), math.radians(1.0))), 1.0, True),
    ("tilted 1.4 degrees across: a corner 1.6 mm up", cover(SX, Y, H_TOP, _q((1, 0, 0), math.radians(1.4))), 0.0,
     False),
    ("1.4 mm above seated (within the corner tolerance)", cover(SX, Y, H_TOP + 0.0014), 1.0, True),
    ("2 mm above seated", cover(SX, Y, H_TOP + 0.002), 0.0, False),
    ("flat on the housing, dowel hanging over its near edge", cover(0.175, Y), 0.0, False),
    ("flat, dowel over the housing's side edge", cover(SX, 0.110, H_TOP, _q((0, 0, 1), -math.pi / 2)),
     0.0, False),
    ("upside down on the housing, dowel up", cover(SX, Y, H_TOP + COVER[2], _q((1, 0, 0), math.pi)), 0.0, False),
    ("pushed off onto the table", cover(0.10, Y, TOP + DOWEL_L, _q((0, 1, 0), COCK)), 0.0, False),
]


def _run() -> int:
    import task

    bad = 0
    start = {o.name: (o.state() if hasattr(o, "state") else None) for o in task.OBJECTS}
    for label, changes, want, want_success in CASES:
        st = dict(start) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
