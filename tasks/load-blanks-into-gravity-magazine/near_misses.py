"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). With a quaternion the object is rotated and its bounding box is the rotated one, as in the simulator.
Objects not listed stay where they start (on the tray pad).
"""

import math

import numpy as np

from task import BLANK, MAG_XY, RIM, TOP

MX, MY = MAG_XY
H = BLANK[2]


def level(k, dx=0.0, dy=0.0, dz=0.0):
    """Centre of a blank lying flat as the k-th in the stack (0 = on the table)."""
    return (MX + dx, MY + dy, TOP + H / 2 + k * H + dz)


def stack(n):
    return {f"blank_{i + 1}": (level(i), None, None) for i in range(n)}


MOVING = (0.0, 0.0, -0.3)


def quat(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return (*(a * math.sin(h)), math.cos(h))


def _half_height(q):
    x, y, z, w = q
    row = np.array([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)])
    return float(np.abs(row) @ (np.asarray(BLANK) / 2))


def rotated(q, bottom=TOP, dx=0.0, dy=0.0):
    """A blank rotated by q, centred over the channel, its lowest point at `bottom`."""
    return ((MX + dx, MY + dy, bottom + _half_height(q)), None, None, q)

CASES = [
    ("all four stacked flat in the magazine", stack(4), 1.0, True),
    ("three in", stack(3), 0.75, False),
    ("two in", stack(2), 0.5, False),
    ("one in", stack(1), 0.25, False),
    ("four in, the top one still falling", {**stack(3), "blank_4": (level(3, dz=0.02), None, MOVING)}, 0.75, False),
    ("four in, against the channel walls (centres 2.5 mm off)",
     {f"blank_{i + 1}": (level(i, dx=0.0025 * (-1) ** i, dy=-0.0025), None, None) for i in range(4)}, 1.0, True),
    ("one blank jammed across the top, its top 15 mm above the rim",
     {**stack(3), "blank_4": ((MX, MY, RIM + 0.015 - H / 2), None, None)}, 0.75, False),
    ("one blank standing on edge in the channel", {**stack(1), "blank_2": rotated(quat((1, 0, 0), 90), bottom=TOP + H)},
     0.25, False),
    ("blank resting on the magazine's far wall, outside the channel",
     {**stack(3), "blank_4": ((MX + 0.03, MY, RIM + H / 2), None, None)}, 0.75, False),
    # Rotations (real quaternions).
    ("one blank turned 7 deg about the vertical, flat on the floor", {"blank_1": rotated(quat((0, 0, 1), 7))}, 0.25, False),
    ("one blank tilted 4.9 deg about an edge direction", {"blank_1": rotated(quat((1, 0, 0), 4.9))}, 0.25, False),
    ("one blank tilted 4.9 deg about its diagonal", {"blank_1": rotated(quat((1, 1, 0), 4.9))}, 0.25, False),
    ("one blank tilted 6 deg (leaning, not flat)", {"blank_1": rotated(quat((1, 0, 0), 6))}, 0.0, False),
    ("one blank upside down, flat on the floor", {"blank_1": rotated(quat((1, 0, 0), 180))}, 0.25, False),
    ("one blank turned 45 deg lying across the rim", {"blank_1": rotated(quat((0, 0, 1), 45), bottom=RIM)}, 0.0, False),
    # Support: every counted blank rests on the floor or on a counted blank.
    ("four in, the top one 5 mm above the one below it, at rest (not supported)",
     {**stack(3), "blank_4": (level(3, dz=0.005), None, None)}, 0.75, False),
    ("four in, the top one 1.5 mm above the one below it (contact)",
     {**stack(3), "blank_4": (level(3, dz=0.0015), None, None)}, 1.0, True),
    ("bottom blank missing: three stacked 15 mm off the floor (not supported)",
     {f"blank_{i + 1}": (level(i + 1), None, None) for i in range(3)}, 0.0, False),
    ("all four stacked on the table beside the magazine",
     {f"blank_{i + 1}": ((MX - 0.08, MY, TOP + H / 2 + i * H), None, None) for i in range(4)}, 0.0, False),
    ("one blank on the table under the magazine's sight slot, outside", {"blank_1": ((MX - 0.04, MY, TOP + H / 2), None, None)},
     0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, q=None):
        """As the simulator reports it: with q (xyzw) the bounding box is the rotated object's world-aligned box."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if q is None else np.asarray(q, float) / np.linalg.norm(q)
        x, y, z, w = q
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        half = np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, size, vel, *q) in changes.items():
            st[name] = state(pos, sizes[name] if size is None else size, vel, q[0] if q else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
