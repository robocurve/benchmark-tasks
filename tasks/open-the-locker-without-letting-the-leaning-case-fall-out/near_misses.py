"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). The cases run in order on one copy of the task module, so the drop and tipping cases come last. Offline
there is no robot, so the release check passes; _run() below also checks the release rule with a gripper on the case.
"""

import math
import os

import task
from task import CASE, LEAN0, LOCKER, MAT, MAT_TOP, MAT_XY, TOP, XD, _lean_pose

H2 = CASE[2] / 2


def tipped(deg):
    """Quaternion of the case tipped ``deg`` degrees forward (top toward the robot)."""
    a = math.radians(deg)
    return (0.0, -math.sin(a / 2), 0.0, math.cos(a / 2))


def lift(deg):
    """Centre height above a surface for the case tipped ``deg`` with its lowest corner on it."""
    a = math.radians(deg)
    return CASE[0] / 2 * math.sin(a) + H2 * math.cos(a)


def on_mat(dx=0.0, dy=0.0, up=0.0, deg=0.0):
    return (MAT_XY[0] + dx, MAT_XY[1] + dy, MAT_TOP + lift(deg) + up)


os.environ[task.OFFLINE_FLAG] = "1"  # offline there is no robot: each case stands for a state with the grippers away

MOVING = (0.0, 0.0, -0.05)
lean_c, lean_q = _lean_pose(LEAN0)
fall_c, fall_q = _lean_pose(math.radians(30.0))  # leaning 30 degrees as the door swings away from it
edge_dx = MAT[0] / 2 - CASE[0] / 2  # case's front face flush with the mat's near edge

CASES = [
    ("standing on the middle of the mat", {"blue_case": (on_mat(), None, None)}, 1.0, True),
    ("standing on the mat, front face on the mat's edge", {"blue_case": (on_mat(dx=-edge_dx), None, None)}, 1.0,
     True),
    ("standing on the mat, leaning 4 degrees",
     {"blue_case": (on_mat(deg=4.0), None, None, tipped(4.0))}, 1.0, True),
    ("overhanging the mat's front edge by 5 mm",
     {"blue_case": (on_mat(dx=-edge_dx - 0.005), None, None)}, 0.5, False),
    ("standing on the table beside the mat",
     {"blue_case": ((MAT_XY[0], MAT_XY[1] + 0.12, TOP + H2), None, None)}, 0.5, False),
    ("standing on the mat but still moving", {"blue_case": (on_mat(), None, MOVING)}, 0.0, False),
    ("held 1 cm above the mat", {"blue_case": (on_mat(up=0.010), None, None)}, 0.0, False),
    ("leaning 8 degrees on the mat", {"blue_case": (on_mat(deg=8.0), None, None, tipped(8.0))}, 0.0, False),
    ("as it starts, leaning on the shut door", {"blue_case": (tuple(lean_c), None, None, tuple(lean_q))}, 0.0,
     False),
    ("pushed back upright inside the locker", {"blue_case": ((XD + 0.0475, -0.025, TOP + 0.010 + H2), None, None)},
     0.0, False),
    ("leaning 44 degrees on the door (not yet tipped)",
     {"blue_case": ((XD - 0.03, -0.025, TOP + 0.010 + lift(44.0)), None, None, tipped(44.0))}, 0.0, False),
    ("then stood on the mat: a 44-degree lean is not tipping", {"blue_case": (on_mat(), None, None)}, 1.0, True),
    ("set down gently: lowered at 0.3 m/s, 2 mm above the mat",
     {"blue_case": (on_mat(up=0.002), None, (0.0, 0.0, -0.3))}, 0.0, False),
    ("then standing on the mat after being set down gently", {"blue_case": (on_mat(), None, None)}, 1.0, True),
    ("lowered at exactly 0.45 m/s, 3 mm above the mat (the limit, not a drop)",
     {"blue_case": (on_mat(up=0.003), None, (0.0, 0.0, -0.45))}, 0.0, False),
    ("then standing on the mat: moving down at 0.45 m/s is not dropping", {"blue_case": (on_mat(), None, None)}, 1.0,
     True),
    # Drops (judged over the whole trial; from here on the record is kept, so these cases come after the clean ones).
    ("dropped from 5 cm: falling at 0.99 m/s just above the mat",
     {"blue_case": (on_mat(up=0.004), None, (0.0, 0.0, -0.99))}, 0.0, False),
    ("then upright in place on the mat (the drop still counts)", {"blue_case": (on_mat(), None, None)}, 0.0, False),
    ("falls out of the locker as the door opens: leaning 30 degrees, falling at 0.8 m/s",
     {"blue_case": (tuple(fall_c), None, (-0.4, 0.0, -0.8), tuple(fall_q)),
      "locker": LOCKER.state(joints={"door": 1.2})}, 0.0, False),
    ("then standing upright on the table outside the locker (the fall still counts)",
     {"blue_case": ((MAT_XY[0], MAT_XY[1] + 0.12, TOP + H2), None, None)}, 0.0, False),
    # Habits (each tips the case past 45 degrees, so each scores 0; they come last because the record is kept):
    # opening the door in one go, or slowly all the way before reaching in, or with one arm and letting it go.
    ("habit, door opened wide first: case tipped 60 degrees out over the locker's front edge",
     {"blue_case": ((XD - 0.02, -0.005, TOP + 0.010 + lift(60.0)), None, None, tipped(60.0))}, 0.0, False),
    ("door opened to 90 degrees and held, the case lying across the opening",
     {"blue_case": ((XD - 0.01, -0.005, TOP + 0.010 + lift(80.0)), None, None, tipped(80.0)),
      "locker": LOCKER.state(joints={"door": 1.57})}, 0.0, False),
    ("tipped: laid on its face on the table",
     {"blue_case": ((XD - 0.10, -0.025, TOP + lift(90.0)), None, None, tipped(90.0))}, 0.0, False),
    ("habit, tipped out onto the mat: lying on its face on the mat",
     {"blue_case": ((MAT_XY[0], MAT_XY[1], MAT_TOP + lift(90.0)), None, None, tipped(90.0))}, 0.0, False),
    ("habit, laid on its back on the mat",
     {"blue_case": ((MAT_XY[0], MAT_XY[1], MAT_TOP + lift(90.0)), None, None, tipped(-90.0))}, 0.0, False),
    ("tipped, then stood on the mat (the tipping still counts)", {"blue_case": (on_mat(), None, None)}, 0.0, False),
]


def _run() -> int:
    import sys

    sys.path.insert(0, ".")
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
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad + _release_cases(start, state)


def _release_cases(start, state) -> int:
    """The release rule, which check.py cannot reach offline: a gripper within 5 mm of the case, or no measurement."""
    import task

    standing = {**start, "blue_case": state(on_mat(), CASE)}
    checks = [
        ("standing on the mat, still gripped (0 mm)", 0.0, {"blue_case": 0.0}),
        ("standing on the mat, a finger 4.9 mm from the case", 0.0, {"blue_case": 0.0049}),
        ("standing on the mat, every gripper link 5.0 mm away (limit)", 1.0, {"blue_case": 0.005}),
    ]
    bad = 0
    for label, want, gaps in checks:
        task._clear()
        task._GAP.clear()
        task._GAP.update(gaps)
        got = 1.0 if task.success(standing) else float(task.score(standing))
        flag = "ok  " if got == want else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    task._GAP.clear()
    os.environ[task.OFFLINE_FLAG] = "0"  # in the simulator, before the first step has measured the grippers
    got = float(task.score(standing))
    os.environ[task.OFFLINE_FLAG] = "1"
    flag = "ok  " if got == 0.0 else "FAIL"
    bad += flag == "FAIL"
    print(f"{flag} {got:4.2f}  standing on the mat, no gripper measurement yet: fails closed")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
