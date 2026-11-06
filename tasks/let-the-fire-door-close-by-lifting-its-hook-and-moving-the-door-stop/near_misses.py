"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready state}, expected score,
expected success). The room's state (door and hook angles) comes from its own kinematics.
"""

import math

from task import DOOR, HINGE, ROOM, SHUT_Q, STOP, STOP_XY, TOP

D = math.radians
Z = TOP + STOP[2] / 2


def room(door=0.0, hook=0.0):
    return {"room": ROOM.state(joints={"door": door, "hook": hook})}


def stop(x, y, z=Z, vel=None, quat=None):
    return {"door_stop": ((x, y, z), None, vel, quat)} if quat else {"door_stop": ((x, y, z), None, vel)}


SHUT = room(SHUT_Q)
AWAY = stop(0.180, 0.200)  # well clear of the swing area (the demo puts it at (0.17, -0.06))
BEHIND_WALL_X = HINGE[0] + DOOR[1] / 2 + STOP[0] / 2  # block's near face on the shut door's line
YAW30 = (0.0, 0.0, math.sin(D(15)), math.cos(D(15)))
ON_SIDE = (math.sin(D(45)), 0.0, 0.0, math.cos(D(45)))

CASES = [
    ("hook lifted, door stop still in place", room(0.0, 0.75), 0.0, False),
    ("hook hanging down, door stop in place", room(0.0, -math.pi / 2), 0.0, False),
    ("door stop slid 2 cm along the door, still against it", stop(STOP_XY[0] - 0.02, STOP_XY[1]), 0.0, False),
    ("door stop pulled 3 cm toward the robot, still in the swing area", stop(STOP_XY[0] - 0.03, STOP_XY[1] - 0.03),
     0.0, False),
    ("door stop taken away, door still hooked open", AWAY, 0.0, False),
    ("door stop away, hook lifted but door not moved", {**AWAY, **room(0.0, 0.75)}, 0.0, False),
    ("door stop away, door swung half shut", {**AWAY, **room(D(-45))}, 0.0, False),
    ("door stop away, door shut", {**AWAY, **SHUT}, 1.0, True),
    ("door shut, hook hanging down", {**AWAY, **room(SHUT_Q, -math.pi / 2)}, 1.0, True),
    ("door exactly 2 degrees short of shut", {**AWAY, **room(SHUT_Q + D(2))}, 1.0, True),
    ("door 2.1 degrees short of shut", {**AWAY, **room(SHUT_Q + D(2.1))}, 0.0, False),
    ("door shut, door stop still being carried", {**SHUT, **stop(0.18, 0.20, Z + 0.05, (0.1, 0.0, 0.0))}, 0.0, False),
    ("door shut, door stop held 2 mm above the table", {**SHUT, **stop(0.18, 0.20, Z + 0.002)}, 0.0, False),
    ("door shut, door stop 1 mm above the table", {**SHUT, **stop(0.18, 0.20, Z + 0.001)}, 1.0, True),
    ("door shut, door stop turned 30 degrees", {**SHUT, **stop(0.18, 0.20, quat=YAW30)}, 1.0, True),
    ("door shut, door stop on its side", {**SHUT, **stop(0.18, 0.20, TOP + STOP[1] / 2, quat=ON_SIDE)}, 1.0, True),
    ("door shut, door stop on top of the side wall", {**SHUT, **stop(0.32, 0.105, TOP + 0.060 + STOP[2] / 2)}, 0.0,
     False),
    ("door shut, door stop behind the wall, exactly 5 mm off the door line",
     {**SHUT, **stop(BEHIND_WALL_X + 0.005, -0.02)}, 1.0, True),
    ("door shut, door stop behind the wall, 4 mm off the door line",
     {**SHUT, **stop(BEHIND_WALL_X + 0.004, -0.02)}, 0.0, False),
    ("door shut, door stop just outside the arc (6 mm)",
     {**SHUT, **stop(HINGE[0] - 0.165 - 0.006 - STOP[0] / 2, HINGE[1] - 0.03)}, 1.0, True),
    ("door shut, door stop corner 4.5 mm left of the arc, in the 5 mm band past the open door line",
     {**SHUT, **stop(HINGE[0] - math.sqrt(0.165 ** 2 - 0.0025 ** 2) - 0.0045 - STOP[0] / 2,
                     HINGE[1] + 0.0025 + STOP[1] / 2)}, 0.0, False),
    ("door shut, same corner 5.5 mm left of the arc",
     {**SHUT, **stop(HINGE[0] - math.sqrt(0.165 ** 2 - 0.0025 ** 2) - 0.0055 - STOP[0] / 2,
                     HINGE[1] + 0.0025 + STOP[1] / 2)}, 1.0, True),
    ("door shut, door stop 3 mm outside the arc",
     {**SHUT, **stop(HINGE[0] - 0.165 - 0.003 - STOP[0] / 2, HINGE[1] - 0.03)}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own success() then score()."""
    import numpy as np

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

    start = {o.name: (o.state() if hasattr(o, "state") else state(o.pos, o.size)) for o in task.OBJECTS}
    sizes = {o.name: o.size for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            if isinstance(spec, ObjectState):
                st[name] = spec
                continue
            pos, size, vel, *quat = spec
            st[name] = state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        raw = float(task.score(st))
        got = 1.0 if ok else raw
        good = (got, ok) == (want, want_success) and (not ok or raw == 1.0)
        bad += not good
        print(f"{'ok  ' if good else 'FAIL'} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
