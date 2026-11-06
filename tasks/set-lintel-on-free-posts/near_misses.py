"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Cases run in order and share the task's record of how far each post has moved and turned, as the steps
of one trial would. RESET puts every object back at its start in one jump (a scene reset), which clears that
record; the cases after it start a new trial.
"""

import numpy as np

from task import LINTEL, POST, POST_TOP, POST_X, POST_Y, TOP

PL, PR = POST_Y["left"], POST_Y["right"]
POST_Z = TOP + POST[2] / 2
ON_TOP = POST_TOP + LINTEL[2] / 2
MOVING = (0.0, 0.0, -0.05)
LINTEL_HOME = (0.18, -0.20, TOP + LINTEL[2] / 2)


def q(axis, deg):
    a = np.radians(deg) / 2
    return tuple(np.r_[np.sin(a) * np.asarray(axis, float), np.cos(a)])


def lintel(dy=0.0, dx=0.0, dz=0.0, quat=None, vel=None):
    return ((POST_X + dx, dy, ON_TOP + dz), None, vel, quat)


def post(side, dx=0.0, dy=0.0, dz=0.0, quat=None):
    return ((POST_X + dx, POST_Y[side] + dy, POST_Z + dz), None, None, quat)


def standing_back(t):
    """The right post on its way from lying on its side (t=0, toppled outward) to standing on its mark (t=1)."""
    lying = np.array((POST_X, PR - 0.055, TOP + POST[0] / 2))
    up = np.array((POST_X, PR, POST_Z))
    return (tuple(lying + t * (up - lying)), None, None, q((1, 0, 0), 90 * (1 - t)))


RESET = ("reset: everything back at its start (new trial)", {}, 0.0, False)
UNTOUCHED = (LINTEL_HOME, None, None)

CASES = [
    ("lintel centred on both posts", {"lintel": lintel()}, 1.0, True),
    ("overhangs differ by 9.8 mm (4.9 mm off along)", {"lintel": lintel(dy=0.0049)}, 1.0, True),
    ("overhangs differ by 12 mm", {"lintel": lintel(dy=0.006)}, 0.7, False),
    ("9.9 mm off sideways", {"lintel": lintel(dx=0.0099)}, 1.0, True),
    ("12 mm off sideways", {"lintel": lintel(dx=-0.012)}, 0.7, False),
    ("30 mm off along: still on both posts", {"lintel": lintel(dy=-0.030)}, 0.7, False),
    ("41 mm off along: one post's top centre not under it", {"lintel": lintel(dy=0.041)}, 0.0, False),
    ("21 mm off sideways: post top centres not under it", {"lintel": lintel(dx=0.021)}, 0.0, False),
    ("centred but still moving", {"lintel": lintel(vel=MOVING)}, 0.0, False),
    ("held still 1.5 mm above the post tops", {"lintel": lintel(dz=0.0015)}, 0.0, False),
    ("held still 0.9 mm above the post tops (within the 1 mm seat test)", {"lintel": lintel(dz=0.0009)}, 1.0, True),
    ("lintel on the table between the posts", {"lintel": ((POST_X, 0.0, TOP + LINTEL[2] / 2), None, None)}, 0.0, False),
    ("lintel turned 8 degrees about the vertical: post top centres 9.7 mm off its centre line",
     {"lintel": lintel(quat=q((0, 0, 1), 8))}, 1.0, True),
    ("lintel turned 10 degrees: post top centres 12.2 mm off its centre line",
     {"lintel": lintel(quat=q((0, 0, 1), 10))}, 0.7, False),
    ("lintel turned 20 degrees: post top centres 24 mm off its centre line, not under it",
     {"lintel": lintel(quat=q((0, 0, 1), 20))}, 0.0, False),
    ("lintel upside down (other 40 x 220 face down), centred", {"lintel": lintel(quat=q((1, 0, 0), 180))}, 1.0, True),
    ("lintel on its edge (30 x 220 face down)",
     {"lintel": ((POST_X, 0.0, POST_TOP + LINTEL[0] / 2), None, None, q((0, 1, 0), 90))}, 0.0, False),
    ("lintel tilted 6 degrees along its length", {"lintel": lintel(quat=q((1, 0, 0), 6))}, 0.0, False),
    ("left post leaning 2.5 degrees, lintel centred",
     {"lintel": lintel(), "post_left": post("left", quat=q((1, 0, 0), 2.5))}, 1.0, True),
    ("centred, left post nudged 9.9 mm sideways", {"lintel": lintel(), "post_left": post("left", dx=0.0099)}, 1.0, True),
    ("centred, both posts slid 9 mm outward", {"lintel": lintel(), "post_left": post("left", dy=0.009),
                                               "post_right": post("right", dy=-0.009)}, 1.0, True),
    RESET,
    ("left post leaning 4 degrees, lintel centred",
     {"lintel": lintel(), "post_left": post("left", quat=q((1, 0, 0), 4))}, 0.0, False),
    RESET,
    ("left post turned 45 degrees in place, lintel centred",
     {"lintel": lintel(), "post_left": post("left", quat=q((0, 0, 1), 45))}, 0.0, False),
    RESET,
    ("left post turned 12 degrees in place, lintel centred",
     {"lintel": lintel(), "post_left": post("left", quat=q((0, 0, 1), 12))}, 1.0, True),
    RESET,
    ("centred, left post shifted 15 mm outward", {"lintel": lintel(dy=0.0075), "post_left": post("left", dy=0.015)},
     0.5, False),
    ("then the post pushed back to its start: the record keeps 15 mm", {"lintel": lintel()}, 0.5, False),
    RESET,
    ("posts pushed 30 mm together", {"lintel": lintel(), "post_left": post("left", dy=-0.015),
                                     "post_right": post("right", dy=0.015)}, 0.5, False),
    RESET,
    ("posts pushed 70 mm together", {"lintel": lintel(), "post_left": post("left", dy=-0.035),
                                     "post_right": post("right", dy=0.035)}, 0.0, False),
    RESET,
    ("posts swapped (identical, so the end photo looks right)",
     {"lintel": lintel(), "post_left": post("left", dy=-0.14), "post_right": post("right", dy=0.14)}, 0.0, False),
    RESET,
    ("a post lifted 50 mm", {"lintel": UNTOUCHED, "post_right": post("right", dz=0.05)}, 0.0, False),
    ("then the post put back exactly and the lintel set centred: still 0", {"lintel": lintel()}, 0.0, False),
    RESET,
    # Topple a post, stand it back on its mark step by step before touching the lintel, then set the lintel.
    ("right post toppled outward", {"post_right": standing_back(0.0)}, 0.0, False),
    ("being stood back up (1/4)", {"post_right": standing_back(0.25)}, 0.0, False),
    ("being stood back up (1/2)", {"post_right": standing_back(0.5)}, 0.0, False),
    ("being stood back up (3/4)", {"post_right": standing_back(0.75)}, 0.0, False),
    ("stood back within 0.4 mm of its mark, at rest, lintel untouched: the record stays",
     {"post_right": ((POST_X + 0.0004, PR, POST_Z), None, None)}, 0.0, False),
    ("then the lintel set centred: still 0", {"lintel": lintel(), "post_right": post("right", dx=0.0004)}, 0.0, False),
    RESET,
    ("right post toppled, stood back within 1.5 mm in one jump, then the lintel set: 0",
     {"post_right": standing_back(0.0)}, 0.0, False),
    ("  (stood back 1.5 mm off its mark: not a reset)", {"post_right": post("right", dx=0.0015)}, 0.0, False),
    ("  (lintel set)", {"lintel": lintel(), "post_right": post("right", dx=0.0015)}, 0.0, False),
    RESET,
    ("a post still moving at the end", {"lintel": lintel(), "post_right": ((POST_X, PR, POST_Z), None, MOVING)},
     0.0, False),
    RESET,
    ("both posts laid on their sides, lintel across them",
     {"post_left": ((POST_X, PL, TOP + POST[0] / 2), None, None, q((1, 0, 0), 90)),
      "post_right": ((POST_X, PR, TOP + POST[0] / 2), None, None, q((1, 0, 0), 90)),
      "lintel": ((POST_X, 0.0, TOP + POST[0] + LINTEL[2] / 2), None, None)}, 0.0, False),
    RESET,
    ("success again after a reset", {"lintel": lintel()}, 1.0, True),
]


def _run() -> int:
    """Score every case in order with task.py's own score() and success(), building states as factory/check.py
    does; print and count mismatches."""
    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        """As the simulator reports it: with a quaternion, the world-aligned box of the rotated object."""
        pos = np.asarray(pos, float)
        qq = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        half = np.abs(task._rot(ObjectState(pos, qq, pos, pos, pos, pos))) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, qq, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    bad = 0
    task.score(start)
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            pos, size, vel, *quat = spec
            st[name] = state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
