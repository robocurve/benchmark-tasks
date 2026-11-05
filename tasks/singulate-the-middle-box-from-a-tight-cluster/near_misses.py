"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Objects not listed stay at their start. Cases run in order and the task keeps a whole-trial record of
brown cubes that left the mat. It clears only when every cube is back within 0.5 mm of its start, at rest, after a
jump of over 50 mm since the previous call (an episode reset), so a "new trial" case must follow a case in which some
cube is far from its start.
"""

import math

from task import CENTRE, CUBE, CUBES, MAT_TOP, MAT_X, MAT_Y, PAD_TOP, PAD_XY, PITCH, TOP

H = CUBE / 2
PX, PY = PAD_XY
CX, CY = CENTRE
MOVING = (0.0, 0.0, -0.05)


def yaw(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


def tilt_x(deg):
    return (math.sin(math.radians(deg) / 2), 0.0, 0.0, math.cos(math.radians(deg) / 2))


def on_pad(dx=0.0, dy=0.0, up=0.0):
    return (PX + dx, PY + dy, PAD_TOP + H + up)


def on_mat(x, y):
    return (x, y, MAT_TOP + H)


def start(name):
    return tuple(CUBES[name].pos)


def row_pushed(d):
    """The middle row (left cube, blue cube, right cube) pushed d metres toward the robot's right."""
    return {n: (on_mat(CX, CUBES[n].pos[1] - d), None, None) for n in ("box_ml", "blue_box", "box_mr")}


BLUE_ON_PAD = {"blue_box": (on_pad(), None, None)}


def tilt_for_height(h):
    """The tilt about x (degrees) at which a cube's highest point is h above its lowest."""
    return math.degrees(math.asin(h / CUBE / math.sqrt(2))) - 45.0


def tilted_on_mat(name, h):
    x, y, _ = CUBES[name].pos
    return ((x, y, MAT_TOP + h / 2), None, None, tilt_x(tilt_for_height(h)))


def gap_x(d):
    """The cubes in front of and behind the blue one moved back so each gap to it is d."""
    back = d - (PITCH - CUBE)
    return {"box_nc": (on_mat(CX - PITCH - back, CY), None, None), "box_fc": (on_mat(CX + PITCH + back, CY), None, None)}


FR = CUBES["box_fr"].pos
TILT_H = H * (math.cos(math.radians(10)) + math.sin(math.radians(10)))  # half height of a cube tilted 10 degrees

CASES = [
    ("blue cube centred on the pad", {**BLUE_ON_PAD}, 1.0, True),
    ("blue cube on the pad, 14 mm off centre (inside the edges)", {"blue_box": (on_pad(0.014, -0.014), None, None)},
     1.0, True),
    ("blue cube on the pad, turned 45 degrees (corners 4.6 mm inside the edges)",
     {"blue_box": (on_pad(), None, None, yaw(45))}, 1.0, True),
    ("blue cube on the pad, 20 mm over its edge", {"blue_box": (on_pad(0.035), None, None)}, 0.5, False),
    ("blue cube held still 6 mm above the pad", {"blue_box": (on_pad(up=0.006), None, None)}, 0.0, False),
    ("blue cube on the pad, still moving", {"blue_box": (on_pad(), None, MOVING)}, 0.0, False),
    ("blue cube leaning 10 degrees on the pad",
     {"blue_box": ((PX, PY, PAD_TOP + TILT_H), None, None, tilt_x(10))}, 0.0, False),
    ("blue cube on the table off the mat", {"blue_box": ((0.10, 0.05, TOP + H), None, None)}, 0.5, False),
    ("blue cube on the table but held 5 mm up", {"blue_box": ((0.10, 0.05, TOP + H + 0.005), None, None)}, 0.0, False),
    ("middle row pushed 115 mm: blue cube on the mat with room on its near and far faces", row_pushed(0.115),
     0.25, False),
    ("middle row pushed 60 mm: no finger room", row_pushed(0.060), 0.0, False),
    ("intact cluster with simulator-noise rotations (1e-5 rad): no finger room",
     {n: (tuple(CUBES[n].pos), None, None, (3e-6, -2e-6, 4e-6, 1.0)) for n in CUBES}, 0.0, False),
    ("blue cube with finger room but held 2 mm above the mat",
     {"blue_box": (on_mat(0.18, 0.0)[:2] + (MAT_TOP + H + 0.002,), None, None)}, 0.0, False),
    ("blue cube alone on an empty part of the mat, turned 20 degrees",
     {"blue_box": (on_mat(0.18, 0.0), None, None, yaw(20))}, 0.25, False),
    ("cubes in front of and behind the blue one moved 10 mm away: not enough room",
     {"box_nc": (on_mat(CX - PITCH - 0.010, CY), None, None), "box_fc": (on_mat(CX + PITCH + 0.010, CY), None, None)},
     0.0, False),
    ("cubes in front of and behind the blue one moved 16 mm away: finger room",
     {"box_nc": (on_mat(CX - PITCH - 0.016, CY), None, None), "box_fc": (on_mat(CX + PITCH + 0.016, CY), None, None)},
     0.25, False),
    ("only the cube in front moved 30 mm away: room on one face only",
     {"box_nc": (on_mat(CX - PITCH - 0.030, CY), None, None)}, 0.0, False),
    ("blue cube stacked on top of a brown cube",
     {"blue_box": ((CX, CY + PITCH, MAT_TOP + CUBE + H), None, None)}, 0.0, False),
    ("blue cube on the pad, a brown cube moved to the mat's edge (centre still over the mat)",
     {**BLUE_ON_PAD, "box_fl": (on_mat(MAT_X[1] - 0.005, 0.20), None, None)}, 1.0, True),
    ("blue cube on the pad, a brown cube rolled onto another face, flat on the mat",
     {**BLUE_ON_PAD, "box_nl": (start("box_nl"), None, None, tilt_x(90))}, 1.0, True),
    ("blue cube on the pad, a brown cube stacked on another",
     {**BLUE_ON_PAD, "box_nc": ((CX - PITCH, CY - PITCH, MAT_TOP + CUBE + H), None, None)}, 0.0, False),
    ("blue cube on the pad, a brown cube leaning 10 degrees",
     {**BLUE_ON_PAD, "box_nr": ((CX - PITCH, CY - PITCH, MAT_TOP + TILT_H), None, None, tilt_x(10))}, 0.0, False),
    ("blue cube on the pad, a brown cube still sliding", {**BLUE_ON_PAD, "box_fc": (start("box_fc"), None, MOVING)},
     0.0, False),
    # Exact limits pass; just past them fails.
    ("blue cube on the table, bottom exactly 1 mm up", {"blue_box": ((0.10, 0.05, TOP + H + 0.001), None, None)},
     0.5, False),
    ("blue cube on the table, bottom 1.1 mm up", {"blue_box": ((0.10, 0.05, TOP + H + 0.0011), None, None)},
     0.0, False),
    ("blue on the pad, a brown cube's bottom exactly 3 mm above the mat",
     {**BLUE_ON_PAD, "box_nl": ((*CUBES["box_nl"].pos[:2], MAT_TOP + H + 0.003), None, None)}, 1.0, True),
    ("blue on the pad, a brown cube's bottom 3.1 mm above the mat",
     {**BLUE_ON_PAD, "box_nl": ((*CUBES["box_nl"].pos[:2], MAT_TOP + H + 0.0031), None, None)}, 0.0, False),
    ("blue on the pad, a brown cube tilted to exactly 53 mm high", {**BLUE_ON_PAD, "box_nl": tilted_on_mat("box_nl", 0.053)},
     1.0, True),
    ("blue on the pad, a brown cube tilted to 53.2 mm high", {**BLUE_ON_PAD, "box_nl": tilted_on_mat("box_nl", 0.0532)},
     0.0, False),
    ("blue cube tilted to exactly 53 mm high on the pad",
     {"blue_box": ((PX, PY, PAD_TOP + 0.0265), None, None, tilt_x(tilt_for_height(0.053)))}, 1.0, True),
    ("blue cube's edge exactly on the pad's edge", {"blue_box": (on_pad(0.015), None, None)}, 1.0, True),
    ("blue cube 0.1 mm over the pad's edge", {"blue_box": (on_pad(0.0151), None, None)}, 0.5, False),
    ("blue cube on the mat with exactly 15 mm gaps front and back", gap_x(0.015), 0.25, False),
    ("blue cube on the mat with 14.9 mm gaps front and back", gap_x(0.0149), 0.0, False),
    ("blue cube held exactly 1 mm above the mat with 15 mm gaps",
     {**gap_x(0.015), "blue_box": ((CX, CY, MAT_TOP + H + 0.001), None, None)}, 0.25, False),
    ("blue cube held 1.1 mm above the mat with 15 mm gaps",
     {**gap_x(0.015), "blue_box": ((CX, CY, MAT_TOP + H + 0.0011), None, None)}, 0.0, False),
    # Whole-trial record: a brown cube that leaves the mat zeroes the trial even if it is put back.
    ("new trial: starting layout", {}, 0.0, False),
    ("blue cube on the pad, a brown cube pushed off the mat onto the table",
     {**BLUE_ON_PAD, "box_fr": ((MAT_X[1] + 0.03, 0.12, TOP + H), None, None)}, 0.0, False),
    ("same trial: that brown cube put back in place", {**BLUE_ON_PAD}, 0.0, False),
    ("new trial: starting layout", {}, 0.0, False),
    # Exploit: push a cube off, carry it back exactly to its start, then take the blue cube. Still 0.
    ("exploit: a brown cube pushed off the mat", {"box_fr": ((MAT_X[1] + 0.03, FR[1], TOP + H), None, None)}, 0.0, False),
    ("exploit: carried back to 30 mm from its place", {"box_fr": ((FR[0] + 0.03, FR[1], FR[2]), None, None)}, 0.0, False),
    ("exploit: 10 mm from its place", {"box_fr": ((FR[0] + 0.01, FR[1], FR[2]), None, None)}, 0.0, False),
    ("exploit: back exactly at its start, at rest", {}, 0.0, False),
    ("exploit: then the blue cube on the pad", {**BLUE_ON_PAD}, 0.0, False),
    ("new trial: starting layout", {}, 0.0, False),
    ("blue and a brown cube both carried to the pad",
     {"blue_box": (on_pad(-0.0, 0.0), None, None), "box_ml": ((PX, PY, PAD_TOP + CUBE + H), None, None)}, 0.0, False),
    ("new trial: starting layout", {}, 0.0, False),
    ("brown cube lifted and set back down elsewhere on the mat, blue on the pad",
     {**BLUE_ON_PAD, "box_ml": (on_mat(0.16, MAT_Y[0] + 0.05), None, None)}, 1.0, True),
    ("new trial: starting layout", {}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import ObjectState

    def rot(q):
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def state(pos, size, vel=None, quat=None):
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        half = np.abs(rot(q)) @ (np.asarray(size, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: o.size for o in task.OBJECTS}
    start_st = {o.name: state(o.pos, o.size) for o in task.OBJECTS}
    task.score(start_st)
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start_st)
        for name, (pos, size, vel, *quat) in changes.items():
            st[name] = state(pos, sizes[name] if size is None else size, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        raw = float(task.score(st))
        got = 1.0 if ok else raw
        flag = "ok  " if (got, ok) == (want, want_success) and (not ok or raw == 1.0) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
