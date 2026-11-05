"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). With a quaternion the object is turned and its bounding box widened, as the simulator reports it.
Objects not listed stay where they started. The cases run in order and the scorer keeps one piece of trial
history (whether either lower plate has moved more than 3 mm or tilted), cleared only by a scene reset (every plate
back within 0.5 mm of its start after a jump of more than 20 mm in one step), so each case that sets it is followed
by a "new trial" case that jumps back to the start, as the simulator's reset does.
"""

import math

from task import (LANE_Y1, PAD_SIZE, PAD_TOP, PAD_XY, PILE_X, PILE_X0, PILE_X1, PILE_Y, PLATE, PLATE_Z, TOP, UNDER_TOP)

L, W, T = PLATE
MOVING = (0.0, 0.0, -0.05)


def about(axis, deg):
    """quat_xyzw for a turn of deg degrees about the x, y or z axis."""
    h = math.radians(deg) / 2
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = math.sin(h)
    return (*v, math.cos(h))


def half_height(q):
    """Half the bounding-box height of a plate turned by q (as check.py and the simulator compute it)."""
    x, y, z, w = q
    r = [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
    return (abs(r[0]) * L + abs(r[1]) * W + abs(r[2]) * T) / 2


def top_at(x, y, bottom, vel=None, size=None, quat=None):
    zc = bottom + (T / 2 if quat is None else half_height(quat))
    return {"plate_top": ((x, y, zc), size, vel, quat)}


ON_PAD = top_at(PAD_XY[0], PAD_XY[1], PAD_TOP)
NEW = ("every plate back at its start (a new trial)", {}, 0.0, False)

CASES = [
    ("top plate flat on the pad, centred", dict(ON_PAD), 1.0, True),
    ("top plate on the pad 19 mm off centre along x, still wholly on it",
     top_at(PAD_XY[0] + (PAD_SIZE[0] - L) / 2 - 0.001, PAD_XY[1], PAD_TOP), 1.0, True),
    ("top plate turned across the pad (80 x 160 footprint does not fit its 120 mm side)",
     top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, size=(W, L, T)), 0.5, False),
    ("top plate overhanging the pad's far edge by 5 mm (lying on pad and table)",
     top_at(PAD_XY[0] + (PAD_SIZE[0] - L) / 2 + 0.005, PAD_XY[1], PAD_TOP), 0.5, False),
    ("top plate on the pad, still moving", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, MOVING), 0.0, False),
    ("top plate held 10 mm above the pad", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.010), 0.0, False),
    ("top plate leaning on the pad, tilted 15 degrees", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("y", 15)),
     0.0, False),
    ("top plate on the pad, tilted 2 degrees (counts as flat)",
     top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("y", 2)), 1.0, True),
    ("top plate on the pad, tilted 4 degrees (one end propped up)",
     top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("y", 4)), 0.0, False),
    ("top plate flat on the pad, turned 10 degrees: corners still inside",
     top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("z", 10)), 1.0, True),
    ("top plate flat on the pad, turned 25 degrees: corners over the pad's edge",
     top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("z", 25)), 0.5, False),
    ("top plate lying upside down on the pad", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("x", 180)),
     1.0, True),
    ("top plate standing on its long edge on the pad", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("x", 90)),
     0.0, False),
    ("top plate pushed off behind the cradle (toward the robot), flat on the table in its lane",
     top_at(PILE_X0 - L / 2 - 0.02, PILE_Y, TOP), 0.0, False),
    ("top plate pushed off the far end, flat on the table 30 mm clear, in the lane",
     top_at(PILE_X1 + L / 2 + 0.03, PILE_Y, TOP), 0.0, False),
    ("top plate off the far end, leaning 10 degrees on the pile's end", top_at(PILE_X1 + L / 2 - 0.005, PILE_Y, TOP,
                                                                              quat=about("y", 10)), 0.0, False),
    ("top plate flat on the table behind the cradle, moved out of its lane",
     top_at(PILE_X0 - L / 2 - 0.02, LANE_Y1 + W / 2 + 0.001, TOP), 0.5, False),
    ("top plate slid exactly 40 mm along the pile", top_at(PILE_X + 0.040, PILE_Y, UNDER_TOP), 0.25, False),
    ("top plate slid exactly 40 mm toward the robot", top_at(PILE_X - 0.040, PILE_Y, UNDER_TOP), 0.25, False),
    ("top plate slid 39 mm, turned 2 degrees: corner past 40 mm", top_at(PILE_X + 0.039, PILE_Y, UNDER_TOP,
                                                                        quat=about("z", 2)), 0.25, False),
    ("top plate on the pad, exactly 1 mm past the pad's far edge (the slack)",
     top_at(PAD_XY[0] + (PAD_SIZE[0] - L) / 2 + 0.001, PAD_XY[1], PAD_TOP), 1.0, True),
    ("top plate on the pad, tilted exactly 3 degrees", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP, quat=about("y", 3)),
     1.0, True),
    ("top plate held 1.4 mm above the pad, still", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.0014), 0.0, False),
    ("top plate 0.9 mm above the pad (contact noise)", top_at(PAD_XY[0], PAD_XY[1], PAD_TOP + 0.0009), 1.0, True),
    ("top plate on the pad, middle plate moved exactly 3 mm",
     {**ON_PAD, "plate_middle": ((PILE_X + 0.003, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 1.0, True),
    ("top plate on the pad, middle plate tilted exactly 3 degrees",
     {**ON_PAD, "plate_middle": ((PILE_X, PILE_Y, PLATE_Z["plate_middle"]), None, None, about("x", 3))}, 1.0, True),
    ("top plate flat on the table on the robot's right", top_at(0.25, -0.15, TOP), 0.5, False),
    ("top plate on the table, 2 mm of it still under the pile's end",
     top_at(PILE_X1 + L / 2 - 0.002, PILE_Y, TOP), 0.0, False),
    ("top plate slid 60 mm along the pile (the demo's pre-grasp)", top_at(PILE_X + 0.060, PILE_Y, UNDER_TOP),
     0.25, False),
    ("top plate slid 41 mm toward the robot", top_at(PILE_X - 0.041, PILE_Y, UNDER_TOP), 0.25, False),
    ("top plate slid 35 mm: not enough to grip", top_at(PILE_X + 0.035, PILE_Y, UNDER_TOP), 0.0, False),
    ("top plate lying on the rails", top_at(PILE_X, PILE_Y, TOP + 3 * T + 0.004), 0.0, False),
    ("top plate knocked off the table", top_at(0.25, 0.14, TOP - 0.40), 0.0, False),
    ("top plate on the pad, middle plate nudged 2 mm",
     {**ON_PAD, "plate_middle": ((PILE_X + 0.002, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 1.0, True),
    NEW,
    # Cases that set the trial history, each followed by a reset.
    ("top plate on the pad, middle plate dragged 5 mm",
     {**ON_PAD, "plate_middle": ((PILE_X + 0.005, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 0.0, False),
    NEW,
    ("both upper plates lifted together; the top one on the pad, the middle one lying on it",
     {**ON_PAD, "plate_middle": ((PAD_XY[0], PAD_XY[1], PAD_TOP + T * 1.5), None, None)}, 0.0, False),
    NEW,
    ("middle plate on the pad, blue top plate set back on the bottom one (a swap; the colours show it)",
     {"plate_middle": ((PAD_XY[0], PAD_XY[1], PAD_TOP + T / 2), None, None),
      **top_at(PILE_X, PILE_Y, TOP + T)}, 0.0, False),
    NEW,
    ("whole pile pushed 25 mm along the cradle",
     {n: ((PILE_X + 0.025, PILE_Y, z), None, None) for n, z in PLATE_Z.items()}, 0.0, False),
    NEW,
    ("middle plate pulled out 30 mm, then ...",
     {"plate_middle": ((PILE_X + 0.030, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 0.0, False),
    ("... pushed back by hand to 5 mm ...",
     {"plate_middle": ((PILE_X + 0.005, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 0.0, False),
    ("... and to 0.3 mm: the pile rebuilt by hand, no scene reset",
     {"plate_middle": ((PILE_X + 0.0003, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 0.0, False),
    ("... then the top plate on the pad: still 0 for this trial", dict(ON_PAD), 0.0, False),
    NEW,
    ("middle plate pulled out 30 mm and pushed back; top plate then put on the pad",
     {"plate_middle": ((PILE_X + 0.030, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 0.0, False),
    ("... the middle plate back within 1 mm and the top plate on the pad: still 0 for this trial",
     {**ON_PAD, "plate_middle": ((PILE_X + 0.001, PILE_Y, PLATE_Z["plate_middle"]), None, None)}, 0.0, False),
    NEW,
    ("bottom plate tilted 4 degrees in place, top plate on the pad",
     {**ON_PAD, "plate_bottom": ((PILE_X, PILE_Y, PLATE_Z["plate_bottom"]), None, None, about("y", 4))}, 0.0, False),
    NEW,
    ("top plate on the pad after the reset", dict(ON_PAD), 1.0, True),
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
