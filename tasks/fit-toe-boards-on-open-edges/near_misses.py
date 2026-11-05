"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). With a quaternion the object is rotated and its bounding box is the rotated one, as in the simulator.
Objects not listed stay where they start. score() keeps trial history (whether a ball has left the platform), so
the ball cases come last: once one has run, every later case scores 0 until the layout is back at its start.
"""

import math

import numpy as np

from task import (BALL_R, BALL_XY, BOARD_H, BOARD_T, FRONT_SIZE, PLAT_TOP, PLAT_X0, PLAT_Y0, PLAT_Y1, SIDE_L, SIDE_SIZE,
                  TARGET, TOP)

Z = PLAT_TOP + BOARD_H / 2  # centre height of a board standing on the platform


def at(xy, dz=0.0):
    return (xy[0], xy[1], Z + dz)


A, B, S = TARGET["board_front_a"], TARGET["board_front_b"], TARGET["board_side"]
SET = {"board_front_a": (at(A), None, None), "board_front_b": (at(B), None, None), "board_side": (at(S), None, None)}


def with_(**changes):
    d = dict(SET)
    for name, v in changes.items():
        if v is None:
            d.pop(name)
        else:
            d[name] = v
    return d


MOVING = (0.0, 0.0, -0.05)


def quat(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return (*(a * math.sin(h)), math.cos(h))


def _matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def resting(xy, size, q, dz=0.0):
    """A board rotated by q, its lowest point on the platform top (plus dz): (centre, None, None, q)."""
    half_h = float(np.abs(_matrix(q))[2] @ (np.asarray(size) / 2))
    return ((xy[0], xy[1], PLAT_TOP + half_h + dz), None, None, q)

CASES = [
    ("all three boards set, joints 0.5-3 mm", SET, 1.0, True),
    # Joint between the near boards: 6 mm is allowed, 7 mm is not.
    ("middle joint 6 mm", with_(board_front_b=(at((B[0], B[1] - 0.003)), None, None)), 1.0, True),
    ("middle joint 7.5 mm (board a moved 4.5 mm left, its end 4 mm past the left edge)",
     with_(board_front_a=(at((A[0], A[1] + 0.0045)), None, None)), 0.87, False),
    # Corner: the side board's near end 6 mm and 7 mm behind board a.
    ("corner gap 6 mm", with_(board_side=(at((S[0] + 0.0045, S[1])), None, None)), 1.0, True),
    ("corner gap 7 mm", with_(board_side=(at((S[0] + 0.0055, S[1])), None, None)), 0.87, False),
    # Flush: outer face 4 mm inside the edge counts, 5 mm does not (the board then covers nothing).
    ("board b outer face 4 mm inside the near edge", with_(board_front_b=(at((B[0] + 0.0035, B[1])), None, None)),
     1.0, True),
    ("board b outer face 5 mm inside the near edge", with_(board_front_b=(at((B[0] + 0.0045, B[1])), None, None)),
     0.64, False),
    ("side board overhanging the left edge by 4 mm", with_(board_side=(at((S[0], S[1] + 0.004)), None, None)), 1.0, True),
    ("side board 10 mm in from the left edge", with_(board_side=(at((S[0], S[1] - 0.010)), None, None)), 0.50, False),
    # Partial credit: covered length / 440 mm x 0.9.
    ("only the two near boards set", with_(board_side=None), 0.50, False),
    ("only the side board set", with_(board_front_a=None, board_front_b=None), 0.37, False),
    ("only board a set (with its 15 mm at the corner)", with_(board_front_b=None, board_side=None), 0.27, False),
    # Not standing, not supported, not at rest.
    ("board a lying flat in place, on its face",
     with_(board_front_a=resting((A[0] + 0.0125, A[1]), FRONT_SIZE, quat((0, 1, 0), 90))), 0.61, False),
    ("board a held 1.5 mm above the platform", with_(board_front_a=(at(A, 0.0015), None, None)), 0.61, False),
    ("board a held 2.5 mm above the platform", with_(board_front_a=(at(A, 0.0025), None, None)), 0.61, False),
    ("board a resting 0.9 mm into the platform (contact depth)", with_(board_front_a=(at(A, -0.0009), None, None)), 1.0, True),
    ("board a held 10 mm above the platform", with_(board_front_a=(at(A, 0.010), None, None)), 0.61, False),
    ("all set but board a still moving", with_(board_front_a=(at(A), None, MOVING)), 0.61, False),
    ("boards stood on the table against the platform's near face",
     {"board_front_a": ((PLAT_X0 - BOARD_T / 2, A[1], TOP + BOARD_H / 2), None, None),
      "board_front_b": ((PLAT_X0 - BOARD_T / 2, B[1], TOP + BOARD_H / 2), None, None)}, 0.0, False),
    ("boards set 20 mm in from both edges, a fence inside the platform",
     {"board_front_a": (at((A[0] + 0.020, A[1] - 0.02)), None, None),
      "board_front_b": (at((B[0] + 0.020, B[1])), None, None),
      "board_side": (at((S[0] + 0.02, S[1] - 0.020)), None, None)}, 0.0, False),
    ("side board alone, turned to lie along the near edge (it covers 182 mm)",
     {"board_side": resting((PLAT_X0 + 0.0075, PLAT_Y0 + 0.001 + SIDE_L / 2), SIDE_SIZE, quat((0, 0, 1), 90))},
     0.37, False),
    # Real rotations: turned over, tilted, turned about the vertical.
    ("board a turned over (180 deg about its long axis), still upright",
     with_(board_front_a=resting(A, FRONT_SIZE, quat((0, 1, 0), 180))), 1.0, True),
    ("side board turned over (180 deg about its long axis)",
     with_(board_side=resting(S, SIDE_SIZE, quat((1, 0, 0), 180))), 1.0, True),
    ("board a with the other long face out (turned 180 deg about the vertical)",
     with_(board_front_a=resting(A, FRONT_SIZE, quat((0, 0, 1), 180))), 1.0, True),
    ("board a leaning 5 deg outward", with_(board_front_a=resting(A, FRONT_SIZE, quat((0, 1, 0), -5))), 1.0, True),
    ("board a leaning 6 deg outward", with_(board_front_a=resting(A, FRONT_SIZE, quat((0, 1, 0), -6))), 0.61, False),
    ("board b turned 2 deg about the vertical", with_(board_front_b=resting(B, FRONT_SIZE, quat((0, 0, 1), 2))),
     1.0, True),
    ("board b turned 4 deg about the vertical (one end 5 mm in from the edge)",
     with_(board_front_b=resting(B, FRONT_SIZE, quat((0, 0, 1), 4))), 0.64, False),
    ("side board standing on its end", with_(board_side=resting(S, SIDE_SIZE, quat((0, 1, 0), 90))), 0.50, False),
    # Balls: a ball held in the air scores 0 now but not for the rest of the trial.
    ("all set, ball 1 held 20 mm up over the platform",
     with_(ball_1=((*BALL_XY["ball_1"], PLAT_TOP + BALL_R + 0.02), None, None)), 0.0, False),
    ("all set, ball 2 rolled against board b", with_(ball_2=((PLAT_X0 + BOARD_T + 0.001 + BALL_R, -0.06, PLAT_TOP + BALL_R),
                                                           None, None)), 1.0, True),
    # From here on a ball has left the platform during the trial.
    ("ball 3 fell off onto the table", with_(ball_3=((0.30, PLAT_Y1 + 0.05, TOP + BALL_R), None, None)), 0.0, False),
    ("ball 3 put back on the platform afterwards, all boards set", SET, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def state(pos, size, vel=None, q=None):
        """As the simulator reports it: with q (xyzw) the bounding box is the rotated object's world-aligned box."""
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if q is None else np.asarray(q, float) / np.linalg.norm(q)
        x, y, z, w = q
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        half = np.abs(r) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    def size_of(o):
        return o.size if hasattr(o, "size") else np.full(3, 2 * o.radius)

    sizes = {o.name: size_of(o) for o in task.OBJECTS}
    start = {o.name: state(o.pos, size_of(o)) for o in task.OBJECTS}
    task.score(start)  # a new trial: clears the history
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, size, vel, *q) in changes.items():
            st[name] = state(pos, sizes[name] if size is None else size, vel, q[0] if q else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
