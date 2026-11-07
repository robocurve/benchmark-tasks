"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None)}, expected score, expected success).
Play is how far the crate can slide front to back. With the crate at the far wall the gap is 62.0 mm and the bay is
0.5 mm longer than crate plus gap, so the 60 mm block leaves 2.5 mm of play, the 54 mm block 8.5 mm, the 44 mm
block 18.5 mm. Play is computed from each block's own thickness, so it takes only these values (plus 62.5 mm with no
block); the 3 mm and 10 mm limits sit well between them.
"""

import numpy as np

from task import BAY_CY, BLOCK_H, BLOCK_W, CRATE, CRATE_X, FAR_IN, NEAR_IN, THICK, TOP, WALL_H

Z = TOP + BLOCK_H / 2


def in_gap(t, against="near", lift=0.0):
    """A block of thickness t standing in the near gap, against the near wall or the crate."""
    x = NEAR_IN + t / 2 if against == "near" else CRATE_X - CRATE[0] / 2 - t / 2
    return (x, BAY_CY, Z + lift)


MOVING = (0.0, 0.0, -0.05)


def _q(lean_deg=0.0, turn_deg=0.0, roll_deg=0.0):
    """xyzw quaternion: roll about x, then lean about y (toward the crate), then turn about the vertical."""
    def axis(a, deg):
        h = np.radians(deg) / 2
        v = np.zeros(3)
        v["xyz".index(a)] = np.sin(h)
        return np.r_[v, np.cos(h)]

    def mul(a, b):
        (x1, y1, z1, w1), (x2, y2, z2, w2) = a, b
        return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])

    return tuple(mul(axis("z", turn_deg), mul(axis("y", lean_deg), axis("x", roll_deg))))


def rotated(name, lean=0.0, turn=0.0, roll=0.0):
    """Block `name` in the near gap, rotated, its widened bounding box on the floor against the near wall."""
    q = np.asarray(_q(lean, turn, roll))
    x, y, z, w = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    half = np.abs(r) @ (np.array((THICK[name], BLOCK_W, BLOCK_H)) / 2)
    return ((NEAR_IN + half[0], BAY_CY, TOP + half[2]), None, None, tuple(q))
CRATE_AT_NEAR = (NEAR_IN + CRATE[0] / 2, BAY_CY, TOP + CRATE[2] / 2)

CASES = [
    ("60 mm block in the gap, against the near wall: 2.5 mm play", {"block_c": (in_gap(THICK["block_c"]), None, None)},
     1.0, True),
    ("60 mm block in the gap, against the crate", {"block_c": (in_gap(THICK["block_c"], "crate"), None, None)}, 1.0, True),
    ("54 mm block in the gap: 8.5 mm play", {"block_a": (in_gap(THICK["block_a"]), None, None)}, 0.5, False),
    ("44 mm block in the gap: 18.5 mm play", {"block_d": (in_gap(THICK["block_d"]), None, None)}, 0.0, False),
    ("60 mm block in the gap but still moving", {"block_c": (in_gap(THICK["block_c"]), None, MOVING)}, 0.0, False),
    ("60 mm block held 1 mm above the floor (counts: stands within 1.5 mm)",
     {"block_c": (in_gap(THICK["block_c"], lift=0.001), None, None)}, 1.0, True),
    ("60 mm block held 3 mm above the floor in the gap", {"block_c": (in_gap(THICK["block_c"], lift=0.003), None, None)},
     0.0, False),
    ("60 mm block standing on the crate's top", {"block_c": ((CRATE_X, BAY_CY, TOP + CRATE[2] + BLOCK_H / 2), None, None)},
     0.0, False),
    ("66 mm block resting across the gap on the near wall and crate",
     {"block_b": ((NEAR_IN + 0.031, BAY_CY, TOP + WALL_H + BLOCK_H / 2), None, None)}, 0.0, False),
    ("60 mm block on the table outside the bay, in front of the near wall",
     {"block_c": ((NEAR_IN - 0.05, BAY_CY, Z), None, None)}, 0.0, False),
    ("crate pushed to the near wall, 60 mm block in the far gap",
     {"crate": (CRATE_AT_NEAR, None, None), "block_c": ((FAR_IN - THICK["block_c"] / 2, BAY_CY, Z), None, None)}, 1.0, True),
    ("crate pushed to the near wall, nothing in the far gap", {"crate": (CRATE_AT_NEAR, None, None)}, 0.0, False),
    ("60 mm block in the gap, crate still sliding",
     {"block_c": (in_gap(THICK["block_c"]), None, None), "crate": ((CRATE_X, BAY_CY, TOP + CRATE[2] / 2), None, MOVING)},
     0.0, False),
    ("crate lifted onto the bay walls, 60 mm block in the gap",
     {"crate": ((CRATE_X, BAY_CY, TOP + WALL_H + CRATE[2] / 2), None, None),
      "block_c": (in_gap(THICK["block_c"]), None, None)}, 0.0, False),
    # Rotated blocks: the simulator's bounding box widens, but play uses each block's own thickness.
    ("60 mm block turned 1.5 deg (as the demo leaves it)", {"block_c": rotated("block_c", turn=1.5)}, 1.0, True),
    ("54 mm block turned 4.9 deg (box 59.8 mm front to back): still 8.5 mm play",
     {"block_a": rotated("block_a", turn=4.9)}, 0.5, False),
    ("54 mm block leaning 2.9 deg and turned 1.4 deg", {"block_a": rotated("block_a", lean=2.9, turn=1.4)}, 0.5, False),
    ("44 mm block leaning 2.9 deg and turned 3.9 deg", {"block_d": rotated("block_d", lean=2.9, turn=3.9)}, 0.0, False),
    ("60 mm block leaning 4 deg (does not count)", {"block_c": rotated("block_c", lean=4.0)}, 0.0, False),
    ("60 mm block turned 6 deg (does not count)", {"block_c": rotated("block_c", turn=6.0)}, 0.0, False),
    ("60 mm block lying on its 60 x 75 mm face, thickness still front to back", {"block_c": rotated("block_c", roll=90.0)},
     1.0, True),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        """As the simulator reports it: a rotated object's bounding box is the world-aligned box around it."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        half = np.abs(task._rot(q)) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            pos, size, vel = spec[:3]
            st[name] = state(pos, sizes[name] if size is None else size, vel, spec[3] if len(spec) > 3 else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
