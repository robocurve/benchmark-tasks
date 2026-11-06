"""Near-successes and unintended strategies, scored offline by factory/check.py. Cases run in order against one copy of
the task module, so the whole-trial tilt record carries from one case to the next (a case that tips the chest zeroes
every later case until a scene reset puts everything back at its start)."""

import math

import numpy as np

from task import CHEST, PAD, PAD_SIZE, TIN, TIN_SIZE, TOP, X0, Y0

PAD_TOP = TOP + PAD_SIZE[2]
TIN_H = TIN_SIZE[2] / 2


def tilt(deg, axis=(0.0, 1.0, 0.0)):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    s = math.sin(math.radians(deg) / 2)
    return (a[0] * s, a[1] * s, a[2] * s, math.cos(math.radians(deg) / 2))


def tin_on_pad(dx=0.0, dy=0.0, up=0.0, vel=None):
    return ((PAD.pos[0] + dx, PAD.pos[1] + dy, PAD_TOP + TIN_H + up), None, vel)


def chest(drawer=0.0, dx=0.0, dy=0.0, deg=0.0):
    pos = (X0 + dx, Y0 + dy, TOP)
    return CHEST.state(joints={"drawer": drawer}, pos=pos, quat_xyzw=tilt(deg) if deg else None)


def tin_tilted(x, y, surface, deg, axis=(0.0, 1.0, 0.0)):
    """The tin turned deg about axis, its lowest corner or edge resting on the surface at height surface."""
    q = tilt(deg, axis)
    x_, y_, z_, w_ = q
    r = np.array([
        [1 - 2 * (y_ * y_ + z_ * z_), 2 * (x_ * y_ - z_ * w_), 2 * (x_ * z_ + y_ * w_)],
        [2 * (x_ * y_ + z_ * w_), 1 - 2 * (x_ * x_ + z_ * z_), 2 * (y_ * z_ - x_ * w_)],
        [2 * (x_ * z_ - y_ * w_), 2 * (y_ * z_ + x_ * w_), 1 - 2 * (x_ * x_ + y_ * y_)],
    ])
    half_z = (np.abs(r) @ (np.asarray(TIN_SIZE) / 2))[2]
    return ((x, y, surface + half_z), None, None, q)


TIN_ON_TABLE = ((0.22, -0.16, TOP + TIN_H), None, None)
DONE = {"red_tin": tin_on_pad(), "chest": chest()}

CASES = [
    ("start: tin in the closed drawer", {"chest": chest()}, 0.0, False),
    ("tin on the pad, drawer closed", DONE, 1.0, True),
    ("tin on the pad, drawer open exactly 1 cm", {"red_tin": tin_on_pad(), "chest": chest(0.01)}, 1.0, True),
    ("tin on the pad, drawer open 1.01 cm", {"red_tin": tin_on_pad(), "chest": chest(0.0101)}, 0.0, False),
    ("tin on the pad, drawer fully open", {"red_tin": tin_on_pad(), "chest": chest(0.13)}, 0.0, False),
    ("tin on the table off the pad, drawer closed", {"red_tin": TIN_ON_TABLE, "chest": chest()}, 0.0, False),
    ("tin on the table off the pad, drawer open", {"red_tin": TIN_ON_TABLE, "chest": chest(0.13)}, 0.0, False),
    ("tin tilted 30 degrees over the pad's edge, an edge resting on the pad",
     {"red_tin": tin_tilted(PAD.pos[0] + 0.045, PAD.pos[1], PAD_TOP, 30.0), "chest": chest()}, 0.0, False),
    ("tin leaning 20 degrees against the chest, an edge on the table",
     {"red_tin": tin_tilted(0.205, 0.0, TOP, 20.0), "chest": chest()}, 0.0, False),
    ("tin lying on its side, flat on the pad", {"red_tin": tin_tilted(PAD.pos[0], PAD.pos[1], PAD_TOP, 90.0,
                                                                     (1.0, 0.0, 0.0)), "chest": chest()}, 1.0, True),
    ("tin on the pad, tilted 4.9 degrees", {"red_tin": tin_tilted(PAD.pos[0], PAD.pos[1], PAD_TOP, 4.9),
                                             "chest": chest()}, 1.0, True),
    ("tin on the pad, tilted 5.1 degrees", {"red_tin": tin_tilted(PAD.pos[0], PAD.pos[1], PAD_TOP, 5.1),
                                             "chest": chest()}, 0.0, False),
    ("tin on the table, tilted 5.1 degrees", {"red_tin": tin_tilted(0.22, -0.16, TOP, 5.1), "chest": chest()}, 0.0,
     False),
    ("tin held 1.5 mm above the pad", {"red_tin": tin_on_pad(up=0.0015), "chest": chest()}, 0.0, False),
    ("tin on the pad, 1 mm up (within tolerance)", {"red_tin": tin_on_pad(up=0.001), "chest": chest()}, 1.0, True),
    ("tin on the pad but still moving", {"red_tin": tin_on_pad(vel=(0.0, 0.0, -0.05)), "chest": chest()}, 0.0, False),
    ("tin centre exactly on the pad's edge", {"red_tin": tin_on_pad(dx=0.05), "chest": chest()}, 1.0, True),
    ("tin centre 1 mm past the pad's edge", {"red_tin": tin_on_pad(dx=0.051), "chest": chest()}, 0.0, False),
    ("tin held 1.5 mm above the table", {"red_tin": ((0.22, -0.16, TOP + TIN_H + 0.0015), None, None)}, 0.0, False),
    ("tin on the chest's top panel", {"red_tin": ((X0, Y0, TOP + 0.165 + TIN_H), None, None)}, 0.0, False),
    ("drawer fully open, tin still in it", {"red_tin": ((TIN.pos[0] - 0.13, Y0, TIN.pos[2]), None, None),
                                            "chest": chest(0.13)}, 0.0, False),
    ("done, chest slid exactly 3 cm", {"red_tin": tin_on_pad(), "chest": chest(dx=-0.03)}, 1.0, True),
    ("done, chest slid 3.1 cm", {"red_tin": tin_on_pad(), "chest": chest(dx=-0.031)}, 0.0, False),
    ("tin on the table, chest slid 3.1 cm", {"red_tin": TIN_ON_TABLE, "chest": chest(dx=-0.031)}, 0.0, False),
    ("done, chest leaning 3 degrees now", {"red_tin": tin_on_pad(), "chest": chest(deg=3.0)}, 1.0, True),
    ("done, chest leaning 4 degrees now", {"red_tin": tin_on_pad(), "chest": chest(deg=4.0)}, 0.0, False),
    ("chest rocked 9.9 degrees during the trial", {"chest": chest(0.13, deg=-9.9)}, 0.0, False),
    ("then done: a 9.9 degree rock is allowed", DONE, 1.0, True),
    ("chest tipped forward 30 degrees onto the open drawer",
     {"red_tin": ((X0 - 0.15, Y0, TOP + 0.10), None, None, tilt(30.0)), "chest": chest(0.13, deg=-30.0)}, 0.0, False),
    ("then stood back up, tin on the pad, drawer shut", DONE, 0.0, False),
    ("then tin on the table: still 0 after the tip", {"red_tin": TIN_ON_TABLE, "chest": chest()}, 0.0, False),
    ("scene reset: everything back at its start", {}, 0.0, False),
    ("after the reset, done again", DONE, 1.0, True),
    ("chest tipped 10.1 degrees backwards", {"chest": chest(deg=10.1)}, 0.0, False),
    ("then done: still 0", DONE, 0.0, False),
]

def _run() -> int:
    """Score every case in order with task.py's own score() and success(); print and count mismatches."""
    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        """As the simulator reports it: a rotated object's world-aligned bounding box is wider than the object."""
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

    start = {"chest": CHEST.state(), "red_tin": state(TIN.pos, TIN.size), "green_pad": state(PAD.pos, PAD.size)}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            if isinstance(spec, ObjectState):
                st[name] = spec
                continue
            pos, size, vel, *quat = spec
            st[name] = state(pos, TIN.size if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
