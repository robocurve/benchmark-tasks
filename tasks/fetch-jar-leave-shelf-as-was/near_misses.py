"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None)}, expected score, expected success).
Objects not listed stay where they started. The cases run in order and the scorer keeps one piece of trial
history (whether anything has been knocked over), so the knocked-over cases come last: once one has run, every
later case stays at 0 until the items are back at their starting poses, as in a real trial.
"""

from task import (BACK_X, BACK_Y, BOTTLE_H, BOTTLE_HOMES, FRONT_X, FRONT_Y, JAR_H, JAR_HOMES, MAT_TOP, MAT_XY, R,
                  SHELF_TOP, TOP)

MOVING = (0.0, 0.0, -0.05)
BM = BOTTLE_HOMES["bottle_middle"]


def on_mat(dx=0.0, dy=0.0, lift=0.0):
    return (MAT_XY[0] + dx, MAT_XY[1] + dy, MAT_TOP + JAR_H / 2 + lift)


def bottle_on_shelf(x, y):
    return (x, y, SHELF_TOP + BOTTLE_H / 2)


def jar_on_shelf(x, y):
    return (x, y, SHELF_TOP + JAR_H / 2)


BLUE_OK = {"blue_jar": (on_mat(), None, None)}

CASES = [
    ("blue jar on the mat, shelf as it was", dict(BLUE_OK), 1.0, True),
    ("blue jar 15 mm off the mat's centre, outline still inside", {"blue_jar": (on_mat(0.015, -0.015), None, None)},
     1.0, True),
    ("blue jar overhanging the mat edge by 5 mm", {"blue_jar": (on_mat(0.025), None, None)}, 0.0, False),
    ("blue jar held 10 mm above the mat", {"blue_jar": (on_mat(lift=0.010), None, None)}, 0.0, False),
    ("blue jar on the mat, still moving", {"blue_jar": (on_mat(), None, MOVING)}, 0.0, False),
    ("blue jar still in its spot on the shelf", {}, 0.0, False),
    ("blue jar on the table beside the mat", {"blue_jar": ((MAT_XY[0], MAT_XY[1] - 0.12, TOP + JAR_H / 2), None,
                                                           None)}, 0.0, False),
    ("middle bottle put back 9 mm off its spot",
     {**BLUE_OK, "bottle_middle": (bottle_on_shelf(BM[0] - 0.009, BM[1]), None, None)}, 1.0, True),
    ("middle bottle put back 12 mm off its spot",
     {**BLUE_OK, "bottle_middle": (bottle_on_shelf(BM[0] - 0.012, BM[1]), None, None)}, 0.5, False),
    ("middle bottle put back 19 mm off its spot",
     {**BLUE_OK, "bottle_middle": (bottle_on_shelf(BM[0] - 0.019, BM[1]), None, None)}, 0.5, False),
    ("middle bottle only slid 25 mm forward and never put back (the lift-over attempt)",
     {**BLUE_OK, "bottle_middle": (bottle_on_shelf(BM[0] - 0.025, BM[1]), None, None)}, 0.0, False),
    ("middle bottle pushed 35 mm sideways toward its neighbour, never put back",
     {**BLUE_OK, "bottle_middle": (bottle_on_shelf(BM[0], BM[1] + 0.035), None, None)}, 0.0, False),
    ("middle bottle stood in the blue jar's old spot",
     {**BLUE_OK, "bottle_middle": (bottle_on_shelf(BACK_X, BACK_Y[1]), None, None)}, 0.0, False),
    ("middle and left bottles swapped (identical bottles, shelf looks the same)",
     {**BLUE_OK, "bottle_middle": (bottle_on_shelf(FRONT_X, FRONT_Y[0]), None, None),
      "bottle_left": (bottle_on_shelf(FRONT_X, FRONT_Y[1]), None, None)}, 1.0, True),
    ("the two white jars swapped",
     {**BLUE_OK, "white_jar_left": (jar_on_shelf(*JAR_HOMES["white_jar_right"]), None, None),
      "white_jar_right": (jar_on_shelf(*JAR_HOMES["white_jar_left"]), None, None)}, 1.0, True),
    ("middle bottle left standing on the table", {**BLUE_OK, "bottle_middle": ((0.18, 0.27, TOP + BOTTLE_H / 2),
                                                                                None, None)}, 0.0, False),
    ("middle bottle hovering 10 mm above its spot (still held)",
     {**BLUE_OK, "bottle_middle": ((BM[0], BM[1], SHELF_TOP + BOTTLE_H / 2 + 0.010), None, None)}, 0.0, False),
    ("shelf cleared: every bottle and white jar on the table, blue jar on the mat",
     {**BLUE_OK, **{n: ((0.12 + 0.07 * i, -0.20, TOP + h / 2), None, None)
                    for i, (n, h) in enumerate([("white_jar_left", JAR_H), ("white_jar_right", JAR_H),
                                                ("bottle_left", BOTTLE_H), ("bottle_middle", BOTTLE_H),
                                                ("bottle_right", BOTTLE_H)])}}, 0.0, False),
    # Knocked-over cases last (trial history, see the docstring).
    ("blue jar lying on its side on the mat",
     {"blue_jar": ((MAT_XY[0], MAT_XY[1], MAT_TOP + R), (JAR_H, 2 * R, 2 * R), None)}, 0.0, False),
    ("after something was knocked over: everything then put right, still 0", dict(BLUE_OK), 0.0, False),
    ("middle bottle lying on the shelf, blue jar on the mat",
     {**BLUE_OK, "bottle_middle": ((BM[0] + 0.02, BM[1], SHELF_TOP + R), (BOTTLE_H, 2 * R, 2 * R), None)},
     0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import Cylinder, ObjectState

    def state(pos, size, vel=None):
        pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), v, np.zeros(3), pos - half, pos + half)

    def size(o):
        return (2 * o.radius, 2 * o.radius, o.height) if isinstance(o, Cylinder) else o.size

    sizes = {o.name: size(o) for o in task.OBJECTS}
    start = {o.name: state(o.pos, size(o)) for o in task.OBJECTS}
    task.score(start)  # a new trial: clears the history
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, sz, vel) in changes.items():
            st[name] = state(pos, sizes[name] if sz is None else sz, vel)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
