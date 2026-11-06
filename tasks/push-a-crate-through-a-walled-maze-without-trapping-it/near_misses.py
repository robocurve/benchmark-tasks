"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). The scorer keeps trial history (whether the crate was ever lifted more than 20 mm or entered the dead
end), cleared only when the crate jumps back to its start pose (a real reset), so those cases come last, each block
starting from a reset.
"""

import numpy as np

from task import CRATE, TOP, XB0, XB1, XE, Y_L, Y_R, YE

Z = TOP + CRATE[2] / 2
MID_X = (XB0 + XB1) / 2  # 0.37
HOME_Y = YE - CRATE[1] / 2  # against the side corridor's end wall: 0.08


def c(x, y, dz=0.0):
    return {"crate": ((x, y, Z + dz), None, None)}


def yaw_q(deg):
    a = np.radians(deg) / 2
    return (0.0, 0.0, float(np.sin(a)), float(np.cos(a)))


def roll_q(deg):
    a = np.radians(deg) / 2
    return (float(np.sin(a)), 0.0, 0.0, float(np.cos(a)))


CASES = [
    ("against the side corridor's end wall, centred", c(MID_X, HOME_Y), 1.0, True),
    ("on the goal, 15 mm toward the near wall", c(MID_X - 0.015, HOME_Y), 1.0, True),
    ("on the goal, 15 mm toward the far wall", c(MID_X + 0.015, HOME_Y), 1.0, True),
    ("against the side corridor's far wall", c(XB1 - CRATE[0] / 2 - 0.0005, HOME_Y), 1.0, True),
    ("near corners exactly on the goal line (y = -5 mm)", c(MID_X, -0.005 + CRATE[1] / 2), 1.0, True),
    ("near corners 0.1 mm short of the goal line", c(MID_X, -0.005 + CRATE[1] / 2 - 0.0001), 0.5, False),
    ("3 mm short of the goal", c(MID_X, -0.005 + CRATE[1] / 2 - 0.003), 0.5, False),
    ("on the goal, turned 10 degrees (corners still on it)",
     {"crate": ((MID_X, HOME_Y - 0.009, Z), None, None, yaw_q(10))}, 1.0, True),
    ("turned 10 degrees, near edge 1 mm inside the goal line: a corner out",
     {"crate": ((MID_X, -0.005 + CRATE[1] / 2 + 0.001, Z), None, None, yaw_q(10))}, 0.5, False),
    ("on the goal, turned 3 degrees", {"crate": ((MID_X, HOME_Y - 0.003, Z), None, None, yaw_q(3))}, 1.0, True),
    ("on the goal but still sliding", {"crate": ((MID_X, HOME_Y, Z), None, (0.0, 0.03, 0.0))}, 0.0, False),
    ("half into the side corridor (centre on the left wall line)", c(MID_X, Y_L), 0.5, False),
    ("centre 0.1 mm short of the left wall line", c(MID_X, Y_L - 0.0001), 0.25, False),
    ("at the bend, lined up (centre 370 mm out)", c(MID_X, -0.13), 0.25, False),
    ("at the bend, centre exactly 350 mm out", c(XB0 + CRATE[0] / 2, -0.13), 0.25, False),
    ("at the bend, centre 349.9 mm out", c(XB0 + CRATE[0] / 2 - 0.0001, -0.13), 0.0, False),
    ("at the bend, centre exactly 390 mm out", c(XB1 - CRATE[0] / 2, -0.13), 0.25, False),
    ("at the bend, centre 390.1 mm out: misses the opening", c(XB1 - CRATE[0] / 2 + 0.0001, -0.13), 0.0, False),
    ("start position", c(0.20, -0.13), 0.0, False),
    ("pushed part way along the straight corridor", c(0.28, -0.13), 0.0, False),
    ("pushed out of the open end onto the table", c(0.06, -0.13), 0.0, False),
    ("tipped onto its side in the side corridor", {"crate": ((MID_X, HOME_Y, TOP + CRATE[1] / 2), None, None,
                                                             roll_q(90))}, 0.0, False),
    ("propped at 8 degrees on the goal", {"crate": ((MID_X, HOME_Y, Z), None, None, roll_q(8))}, 0.0, False),
    ("lifted 15 mm in the side corridor: not lifted by the rule", c(MID_X, 0.03, dz=0.015), 0.0, False),
    ("then set down in the goal: 1.0 (never above 20 mm)", c(MID_X, HOME_Y), 1.0, True),
    # Trial-history cases last. Each block starts from a real reset (the crate jumps back to its start).
    ("centre exactly 440 mm out: not yet in the dead end", c(XB1, -0.13), 0.0, False),
    ("then into the goal: 1.0", c(MID_X, HOME_Y), 1.0, True),
    ("centre 441 mm out: in the dead end", c(XB1 + 0.001, -0.13), 0.0, False),
    ("then pulled back and into the goal: still 0, it entered the dead end", c(MID_X, HOME_Y), 0.0, False),
    ("reset: the crate jumps back to its start", c(0.20, -0.13), 0.0, False),
    ("in the dead end against its wall", c(XE - CRATE[0] / 2, -0.13), 0.0, False),
    ("reset", c(0.20, -0.13), 0.0, False),
    ("in the dead end, pressed into its corner", c(XE - CRATE[0] / 2, Y_R + CRATE[1] / 2), 0.0, False),
    ("reset", c(0.20, -0.13), 0.0, False),
    ("lifted 30 mm at its start", c(0.20, -0.13, dz=0.030), 0.0, False),
    ("lowered to 15 mm above its start", c(0.20, -0.13, dz=0.015), 0.0, False),
    ("set back down exactly at its start (a 15 mm move, not a reset)", c(0.20, -0.13), 0.0, False),
    ("then pushed into the goal: still 0, it was lifted", c(MID_X, HOME_Y), 0.0, False),
    ("reset", c(0.20, -0.13), 0.0, False),
    ("lifted 30 mm over the walls toward the goal", c(MID_X, 0.0, dz=0.03), 0.0, False),
    ("then set down on the goal: still 0, it was lifted", c(MID_X, HOME_Y), 0.0, False),
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

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    task.score(start)  # a new trial: clears the history
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, size, vel, *quat) in changes.items():
            st[name] = state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
