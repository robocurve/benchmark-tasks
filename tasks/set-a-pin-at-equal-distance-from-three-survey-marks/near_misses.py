"""Near-successes and unintended strategies, scored offline.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Give a quaternion for every tilted or turned case.
"""

import numpy as np

from task import CENTRE, MARK_H, PIN_H, PIN_R, PIN_START, TOP, _M, spread

Z = TOP + PIN_H / 2
MOVING = (0.0, 0.0, -0.05)


def at(xy, dz=0.0):
    return (float(xy[0]), float(xy[1]), Z + dz)


def at_spread(target, angle_deg):
    """The point along a direction from the centre where the spread first reaches ``target`` (m), by bisection."""
    u = np.array([np.cos(np.radians(angle_deg)), np.sin(np.radians(angle_deg))])
    lo, hi = 0.0, 0.1
    for _ in range(80):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if spread(CENTRE + mid * u) < target else (lo, mid)
    return CENTRE + lo * u, u


def tilt(deg, axis=(1.0, 0.0, 0.0)):
    h = np.radians(deg) / 2
    return (*(np.sin(h) * np.asarray(axis)), np.cos(h))


def tilted(deg):
    """A pin tilted ``deg`` and resting on its rim at the point: its lowest corner on the table."""
    t = np.radians(deg)
    dz = PIN_H / 2 * np.cos(t) + PIN_R * np.sin(t) - PIN_H / 2
    return (at(CENTRE, dz), None, None, tilt(deg))


def limit_cases(target, inside, outside, angle):
    p, u = at_spread(target, angle)
    return [
        (f"spread at {target * 1000:.0f} mm exactly ({angle} deg from the centre)", {"pin": (at(p), None, None)},
         inside, inside == 1.0),
        (f"spread just over {target * 1000:.0f} mm ({angle} deg)", {"pin": (at(p + 0.0003 * u), None, None)},
         outside, False),
    ]


_sides = {"ab": (_M[0] + _M[1]) / 2, "bc": (_M[1] + _M[2]) / 2, "ac": (_M[0] + _M[2]) / 2}
_a, _b, _c = (np.linalg.norm(_M[(i + 1) % 3] - _M[(i + 2) % 3]) for i in range(3))
_incentre = (_a * _M[0] + _b * _M[1] + _c * _M[2]) / (_a + _b + _c)

CASES = [
    ("pin at the equidistant point", {"pin": (at(CENTRE), None, None)}, 1.0, True),
    ("pin 3 mm off to the robot's left", {"pin": (at(CENTRE + (0, 0.003)), None, None)}, 1.0, True),
    ("pin standing on its other end", {"pin": (at(CENTRE), None, None, (1.0, 0.0, 0.0, 0.0))}, 1.0, True),
    ("pin tilted 4 deg", {"pin": tilted(4)}, 1.0, True),
    ("pin tilted 5 deg exactly, resting on its rim", {"pin": tilted(5)}, 1.0, True),
    ("pin tilted 6 deg", {"pin": tilted(6)}, 0.0, False),
    ("pin lying on its side at the point", {"pin": ((*CENTRE, TOP + 0.015), None, None, tilt(90))}, 0.0, False),
    ("pin still moving", {"pin": (at(CENTRE), None, MOVING)}, 0.0, False),
    ("pin held 2 mm above the table", {"pin": (at(CENTRE, 0.002), None, None)}, 0.0, False),
    ("pin bottom 1 mm above the table (limit)", {"pin": (at(CENTRE, 0.001), None, None)}, 1.0, True),
    ("pin balanced on top of a mark", {"pin": ((*_M[1], TOP + MARK_H + PIN_H / 2), None, None)}, 0.0, False),
    *limit_cases(0.010, 1.0, 0.5, 0),
    *limit_cases(0.010, 1.0, 0.5, 90),
    *limit_cases(0.018, 0.5, 0.25, 0),
    *limit_cases(0.018, 0.5, 0.25, 225),
    *limit_cases(0.028, 0.25, 0.0, 0),
    *limit_cases(0.028, 0.25, 0.0, 135),
    ("pin left at its start", {"pin": (at(PIN_START), None, None)}, 0.0, False),
    ("pin at the centroid of the three marks", {"pin": (at(_M.mean(0)), None, None)}, 0.0, False),
    ("pin at the incentre of the three marks", {"pin": (at(_incentre), None, None)}, 0.0, False),
    ("pin at the midpoint between the two outer marks", {"pin": (at(_sides["ac"]), None, None)}, 0.0, False),
    ("pin at the midpoint of marks a and b", {"pin": (at(_sides["ab"]), None, None)}, 0.0, False),
    ("pin beside the middle mark", {"pin": (at(_M[1] + (0, 0.03)), None, None)}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        x, y, z, w = q
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        half = np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    def size(o):
        return np.array([2 * o.radius, 2 * o.radius, o.height])

    sizes = {o.name: size(o) for o in task.OBJECTS}
    start = {o.name: state(o.pos, sizes[o.name]) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, sz, vel, *quat) in changes.items():
            st[name] = state(pos, sizes[name] if sz is None else sz, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}  (spread {spread(st['pin'].pos) * 1000:.1f} mm)")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
