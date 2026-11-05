"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Give a quaternion for every tilted or turned case. Cases run in order and score() keeps a trial history
(any item leaving the counter zeroes the trial until every item is back at its start), so the cases where an item
leaves the counter come last, followed by a reset.
"""

import numpy as np

from task import BOTTLE, CT, KNIFE, MUG, PAN, SPRAY, TOP


def qz(a):
    return (0.0, 0.0, float(np.sin(a / 2)), float(np.cos(a / 2)))


def qy(a):
    return (0.0, float(np.sin(a / 2)), 0.0, float(np.cos(a / 2)))


def qx(a):
    return (float(np.sin(a / 2)), 0.0, 0.0, float(np.cos(a / 2)))


def on(x, y, h):
    """Centre of an item of height h standing on the counter at (x, y)."""
    return (x, y, CT + h / 2)


SAFE = {
    "mug": (on(0.25, 0.195, MUG.height), None, None),
    "knife": (on(0.175, 0.13, KNIFE.size[2]), None, None, qz(-np.pi / 2)),
    "medicine_bottle": (on(0.25, 0.07, BOTTLE.height), None, None),
    "spray_box": (on(0.24, -0.07, SPRAY.size[2]), None, None),
    "pan": (on(0.23, -0.19, PAN.height), None, None),
}


def but(**changes):
    return {**SAFE, **changes}


MOVING = (0.05, 0.0, 0.0)
MUG_R, PAN_R = MUG.radius, PAN.radius

LEAN = np.radians(25)  # knife with one end on the counter, the other propped on something 70 mm along it
LYING = qx(np.pi / 2)  # a cylinder on its side, axis left to right

CASES = [
    ("all five back", SAFE, 1.0, True),
    ("mug's far side exactly at the 15 cm line", but(mug=(on(0.30 - MUG_R, 0.195, MUG.height), None, None)), 1.0, True),
    ("mug's far side 1 mm past the line", but(mug=(on(0.301 - MUG_R, 0.195, MUG.height), None, None)), 0.8, False),
    ("pan's centre back but its rim 5 mm past the line", but(pan=(on(0.305 - PAN_R, -0.19, PAN.height), None, None)),
     0.8, False),
    ("pan turned 30 degrees, rim 15 mm inside the line",
     but(pan=(on(0.285 - PAN_R, -0.19, PAN.height), None, None, qz(np.pi / 6))), 1.0, True),
    ("mug turned 45 degrees, rim 15 mm inside", but(mug=(on(0.285 - MUG_R, 0.195, MUG.height), None, None, qz(np.pi / 4))),
     1.0, True),
    ("spray box turned 45 degrees, corner 25 mm inside", but(spray_box=(on(0.24, -0.07, SPRAY.size[2]), None, None,
                                                                     qz(np.pi / 4))), 1.0, True),
    ("spray box turned 45 degrees, corner 2 mm past the line",
     but(spray_box=(on(0.302 - 0.025 * np.sqrt(2), -0.07, SPRAY.size[2]), None, None, qz(np.pi / 4))), 0.8, False),
    ("knife left lying front to back across the line",
     but(knife=(on(0.25, 0.13, KNIFE.size[2]), None, None)), 0.8, False),
    ("knife leaning 25 degrees, one end on the counter, the other propped up",
     but(knife=((0.225, 0.13, CT + 0.07 * np.sin(LEAN) + 0.012 * np.cos(LEAN)), None, None, qy(LEAN))), 0.8, False),
    ("knife resting 1 mm proud (contact noise)",
     but(knife=(on(0.175, 0.13, KNIFE.size[2] + 0.002), None, None, qz(-np.pi / 2))), 1.0, True),
    ("knife still in the gripper 2.5 mm above the counter",
     but(knife=(on(0.175, 0.13, KNIFE.size[2] + 0.005), None, None, qz(-np.pi / 2))), 0.8, False),
    ("bottle held still 2 mm above the counter", but(medicine_bottle=(on(0.25, 0.07, BOTTLE.height + 0.004), None, None)),
     0.8, False),
    ("mug still sliding", but(mug=(on(0.25, 0.195, MUG.height), None, MOVING)), 0.8, False),
    ("three back, two untouched", but(spray_box=(SPRAY.pos, None, None), pan=(PAN.pos, None, None)), 0.6, False),
    ("nothing moved but the knife", {"knife": SAFE["knife"]}, 0.2, False),
    ("bottle stood on the spray box (stacked)",
     but(medicine_bottle=(on(0.24, -0.07, SPRAY.size[2] + BOTTLE.height), None, None)), 0.8, False),
    ("mug overhanging the near edge, still on the counter",
     but(mug=(on(0.17, 0.195, MUG.height), None, None)), 0.8, False),
    ("mug tilting over the near edge, lowest point 5 mm below the top",
     but(mug=((0.16, 0.195, CT - 0.005 + MUG.height / 2 * np.cos(np.radians(25)) + MUG_R * np.sin(np.radians(25))),
              None, None, qy(np.radians(-25)))), 0.8, False),
    ("pan overhanging the left-right side, still on the counter",
     but(pan=(on(0.23, -0.225, PAN.height), None, None)), 0.8, False),
    ("bottle lying on its side, back from the line",
     but(medicine_bottle=((0.25, 0.07, CT + BOTTLE.radius), None, None, LYING)), 1.0, True),
    ("bottle on its side, turned front to back, its end past the line",
     but(medicine_bottle=((0.27, 0.07, CT + BOTTLE.radius), None, None, qy(np.pi / 2))), 0.8, False),
    ("mug lying on its side, back from the line", but(mug=((0.25, 0.195, CT + MUG_R), None, None, LYING)), 1.0, True),
    ("bottle tipped 10 degrees, leaning on something", but(medicine_bottle=(on(0.25, 0.07, BOTTLE.height), None, None,
                                                                           qx(np.radians(10)))), 0.8, False),
    # From here on an item leaves the counter, and the trial history keeps the score at 0.
    ("mug pushed off the near edge onto the table", but(mug=((0.11, 0.195, TOP + MUG.height / 2), None, None)), 0.0, False),
    ("then the mug put back: all five back, but the trial already failed", SAFE, 0.0, False),
    ("bottle knocked off the far edge (toddler side)",
     but(medicine_bottle=((0.48, 0.03, TOP + BOTTLE.height / 2), None, None)), 0.0, False),
    ("pan dragged off the side onto the table", but(pan=((0.23, -0.29, TOP + PAN.height / 2), None, None)), 0.0, False),
    ("bottle teetering on the far edge, centre past it", but(medicine_bottle=(on(0.451, 0.03, BOTTLE.height), None, None)),
     0.0, False),
    ("every item back at its start: a new trial", {}, 0.0, False),
    ("all five back in the new trial", SAFE, 1.0, True),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import task
    from yam_arena.task_file import Box, ObjectState

    def state(pos, size, vel=None, quat=None):
        """As the simulator reports it: the bounding box is the world-aligned box of the rotated object."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        st0 = ObjectState(pos, q, np.zeros(3), np.zeros(3), pos, pos)
        half = np.abs(task._rot(st0)) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size if isinstance(o, Box) else (2 * o.radius, 2 * o.radius, o.height) for o in task.OBJECTS}
    start = {o.name: state(o.pos, sizes[o.name]) for o in task.OBJECTS}
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
