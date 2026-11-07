"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). With a quaternion the object is turned and its bounding box widened, as the simulator reports it. Objects
not listed stay where they started. The scorer keeps one piece of trial history (whether any carton has been
lifted or the bottom carton moved too far), cleared only by a scene reset (every carton back within 0.5 mm of its
start after a jump of more than 20 mm in one step), so each case that sets it is followed by a "new trial" case.
"""

import math

from task import BASE_X, BASE_Y, CARTON, START_X, START_Z, TOP

H = CARTON[2]
MOVING = (0.05, 0.0, 0.0)
NEW = ("every carton back at its start (a new trial)", {}, 0.0, False)


def about(axis, deg):
    h = math.radians(deg) / 2
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = math.sin(h)
    return (*v, math.cos(h))


def at(n, x, dy=0.0, dz=0.0, vel=None, quat=None, base=BASE_Y):
    return {n: ((x, base + dy, START_Z[n] + dz), None, vel, quat)}


def stack(x2, x3, x1=BASE_X, y1=BASE_Y, **kw3):
    """Bottom carton at x1, y1; the others at x2, x3 (centres), each resting on the one below."""
    return {**at("carton_1", x1, base=y1), **at("carton_2", x2, base=y1), **at("carton_3", x3, base=y1, **kw3)}


def rocked(deg, pivot_x=BASE_X + CARTON[0] / 2):
    """The squared stack tipped as one body by deg degrees away from the robot about the bottom carton's far edge."""
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    out = {}
    for n in ("carton_1", "carton_2", "carton_3"):
        dx, dz = BASE_X - pivot_x, START_Z[n] - TOP
        out[n] = ((pivot_x + dx * c + dz * s_, BASE_Y, TOP - dx * s_ + dz * c), None, None, about("y", deg))
    return out


def tilted_on(n, deg, x=BASE_X):
    """Carton n tilted deg degrees about y, its lowest edge on the carton below (or the table)."""
    h = (CARTON[0] * math.sin(math.radians(deg)) + H * math.cos(math.radians(deg))) / 2
    return {n: ((x, BASE_Y, START_Z[n] - H / 2 + h), None, None, about("y", deg))}


X = BASE_X
CASES = [
    ("both upper cartons flush with the bottom one", stack(X, X), 1.0, True),
    ("top carton 4.5 mm toward the robot, middle flush", stack(X, X - 0.0045), 1.0, True),
    ("top carton 4.5 mm away from the robot", stack(X, X + 0.0045), 1.0, True),
    ("top carton 6 mm toward the robot", stack(X, X - 0.006), 0.5, False),
    ("middle carton 6 mm sideways, top flush with it", stack(X, X, dy=0.0) | at("carton_2", X, dy=0.006)
     | at("carton_3", X, dy=0.006), 0.5, False),
    ("middle flush, top still 30 mm back (as at the start)", stack(X, X - 0.030), 0.5, False),
    ("middle still 18 mm back, top flush with the middle one", stack(X - 0.018, X - 0.018), 0.5, False),
    ("both upper cartons pushed 10 mm too far: middle overhangs, top flush with it", stack(X + 0.010, X + 0.010), 0.5, False),
    ("top carton turned 5 degrees: corners 3.3 mm over", stack(X, X, quat=about("z", 5)), 1.0, True),
    ("top carton turned 10 degrees: corners 6.3 mm over", stack(X, X, quat=about("z", 10)), 0.5, False),
    ("top carton tilted 5 degrees (one edge propped)", stack(X, X, quat=about("y", 5)), 0.0, False),
    ("squared but the top carton still sliding", stack(X, X, vel=MOVING), 0.0, False),
    ("squared, the bottom carton pushed 15 mm along", stack(X + 0.015, X + 0.015, x1=X + 0.015), 1.0, True),
    ("squared, the whole stack shoved 25 mm along", stack(X + 0.025, X + 0.025, x1=X + 0.025), 0.0, False),
    NEW,
    ("top carton knocked off onto the table, middle flush",
     {**stack(X, X), "carton_3": ((X - 0.15, BASE_Y, TOP + H / 2), None, None)}, 0.0, False),
    ("the stack toppled: all three on the table",
     {f"carton_{i}": ((X - 0.10 * i, BASE_Y, TOP + H / 2), None, None) for i in (1, 2, 3)}, 0.0, False),
    NEW,
    ("top carton held 5 mm above the middle one, flush", stack(X, X, dz=0.005), 0.0, False),
    ("top carton held 1.4 mm above the middle one, flush", stack(X, X, dz=0.0014), 0.0, False),
    ("top carton 0.9 mm above the middle one (contact noise), flush", stack(X, X, dz=0.0009), 1.0, True),
    ("top carton overhanging by exactly 5 mm", stack(X, X - 0.005), 1.0, True),
    ("top carton tilted exactly 3 degrees, edge down on the middle one", {**stack(X, X), **tilted_on("carton_3", 3)},
     1.0, True),
    ("top carton tilted 3.5 degrees", {**stack(X, X), **tilted_on("carton_3", 3.5)}, 0.0, False),
    ("squared, the bottom carton exactly 20 mm along", stack(X + 0.020, X + 0.020, x1=X + 0.020), 1.0, True),
    ("mid-push: the squared stack rocked 8 degrees as one about the bottom far edge (nothing lifted)", rocked(8),
     0.0, False),
    ("... then settles back, squared: rocking is not lifting", stack(X, X), 1.0, True),
    NEW,
    # Cases that set the trial history, each followed by a reset.
    ("top carton lifted 10 mm, then ...", stack(START_X["carton_2"], START_X["carton_3"], dz=0.010), 0.0, False),
    ("... set down flush: still 0 for this trial", stack(X, X), 0.0, False),
    NEW,
    ("taken apart and rebuilt squarely in the same place (middle carton lifted)",
     {**stack(X, X), "carton_2": ((X, BASE_Y, START_Z["carton_2"] + 0.03), None, None)}, 0.0, False),
    ("... and set down", stack(X, X), 0.0, False),
    NEW,
    ("bottom carton dragged 25 mm, then ...", stack(START_X["carton_2"] + 0.025, START_X["carton_3"] + 0.025,
                                                    x1=X + 0.025), 0.0, False),
    ("... pushed back to 15 mm and squared: still 0 for this trial", stack(X + 0.015, X + 0.015, x1=X + 0.015),
     0.0, False),
    NEW,
    ("top carton fell to the table, then ...", {"carton_3": ((X - 0.15, BASE_Y, TOP + H / 2), None, None)}, 0.0, False),
    ("... picked up and set flush on the stack (restacked): 0", stack(X, X), 0.0, False),
    NEW,
    ("top carton lifted 10 mm, then ...", stack(START_X["carton_2"], START_X["carton_3"], dz=0.010), 0.0, False),
    ("... set back exactly at its start by hand (no scene reset)", {}, 0.0, False),
    ("... then squared: still 0, the hand rebuild did not reset the trial", stack(X, X), 0.0, False),
    NEW,
    ("squared after the reset", stack(X, X), 1.0, True),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, quat=None):
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float)
        x, y, z, w = q
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        pos, half = np.asarray(pos, float), np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
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
