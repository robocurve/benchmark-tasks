"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None)}, expected score, expected success).
The scorer reads only the rod's world-aligned outline (lowest and highest points, extent seen from above), so a
turned or tilted rod is given here as the outline it would have.
"""

import math

from task import FLOOR_Z, ROD, TOP, TRAY_X, TRAY_Y, WALL_H, X0, X1, Y0, Y1

L, W, H = ROD


def turned(deg):
    """Outline of the rod lying flat, turned deg from the tray's long side."""
    a = math.radians(deg)
    return (L * math.cos(a) + W * math.sin(a), L * math.sin(a) + W * math.cos(a), H)


def flat_in_tray(deg, dx=0.0, dy=0.0, lift=0.0):
    return ((TRAY_X + dx, TRAY_Y + dy, FLOOR_Z + lift + H / 2), turned(deg), None)


def propped(deg_tilt, x_low_end):
    """Lying along x, its low end on the floor at x_low_end, tilted up over the back wall's top."""
    t = math.radians(deg_tilt)
    sx, sz = L * math.cos(t) + H * math.sin(t), L * math.sin(t) + H * math.cos(t)
    return ((x_low_end + sx / 2, TRAY_Y, FLOOR_Z + sz / 2), (sx, W, sz), None)


MOVING = (0.0, 0.0, -0.05)
ACROSS_TOPS = FLOOR_Z + WALL_H + H / 2  # resting across two wall tops

CASES = [
    ("flat on the diagonal, 34 degrees", {"rod": flat_in_tray(34)}, 1.0, True),
    ("flat at 31 degrees (near the low end of the range)", {"rod": flat_in_tray(31)}, 1.0, True),
    ("flat at 37 degrees (near the high end)", {"rod": flat_in_tray(37)}, 1.0, True),
    ("flat at -34 degrees (the other diagonal)", {"rod": flat_in_tray(-34)}, 1.0, True),
    ("flat at 34 degrees, held 2 mm above the floor", {"rod": flat_in_tray(34, lift=0.002)}, 0.0, False),
    ("flat at 34 degrees, 1 mm above the floor (contact)", {"rod": flat_in_tray(34, lift=0.001)}, 1.0, True),
    ("flat at 34 degrees but 4 mm above the floor (held)", {"rod": flat_in_tray(34, lift=0.004)}, 0.0, False),
    ("flat on the diagonal but still moving", {"rod": (flat_in_tray(34)[0], turned(34), MOVING)}, 0.0, False),
    ("one end raised 6 mm (tilted ~1.6 degrees)",
     {"rod": ((TRAY_X, TRAY_Y, FLOOR_Z + (H + 0.006) / 2), (turned(34)[0], turned(34)[1], H + 0.006), None)},
     0.0, False),
    ("one end raised 4 mm (top at 24 mm)",
     {"rod": ((TRAY_X, TRAY_Y, FLOOR_Z + (H + 0.004) / 2), (turned(34)[0], turned(34)[1], H + 0.004), None)},
     1.0, True),
    ("outline 2 mm past a wall face (cannot fit: overlaps a wall)",
     {"rod": flat_in_tray(34, dy=(Y1 - TRAY_Y) - turned(34)[1] / 2 + 0.002)}, 0.0, False),
    ("propped: low end on the floor at the front wall, resting on the back wall's top",
     {"rod": propped(11.5, X0)}, 0.3, False),
    ("propped but still sliding", {"rod": (propped(11.5, X0)[0], propped(11.5, X0)[1], MOVING)}, 0.0, False),
    ("across both end walls' tops, along x", {"rod": ((TRAY_X, TRAY_Y, ACROSS_TOPS), None, None)}, 0.0, False),
    ("across the tray on the diagonal but resting on two wall tops (turned 25 degrees)",
     {"rod": ((TRAY_X, TRAY_Y, ACROSS_TOPS), turned(25), None)}, 0.0, False),
    ("stood on end in the tray", {"rod": ((TRAY_X, TRAY_Y, FLOOR_Z + L / 2), (W, W, L), None)}, 0.0, False),
    ("leaning steeply in a corner (top 150 mm up)",
     {"rod": ((X0 + 0.08, TRAY_Y, FLOOR_Z + 0.075), (0.170, W, 0.150), None)}, 0.0, False),
    ("on the table in front of the tray", {"rod": ((X0 - 0.06, 0.0, TOP + H / 2), (W, L, H), None)}, 0.0, False),
    ("flat on the table beside the tray, turned 34 degrees",
     {"rod": ((TRAY_X, 0.20, TOP + H / 2), turned(34), None)}, 0.0, False),
    ("one end on the floor but midpoint over the side wall",
     {"rod": ((TRAY_X, Y1 + 0.01, FLOOR_Z + 0.025), (W, L * 0.9, 0.05), None)}, 0.0, False),
    ("on top of the tray's back wall", {"rod": ((X1 + 0.005, 0.0, FLOOR_Z + WALL_H + H / 2), (W, L, H), None)},
     0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None):
        pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, size, vel) in changes.items():
            st[name] = state(pos, sizes[name] if size is None else size, vel)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
