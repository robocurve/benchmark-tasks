"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Give a quaternion for every tilted or turned case.

The cases run in order, one control step each, and task.py keeps history between them as it does over a trial:
the wafer counts as at rest only once its pose has stayed put for 10 steps, and the drop record lasts until a scene
reset. So each end state is held for 10 cases (``hold``); only the last of them is at rest.
"""

import math

import numpy as np

from task import (FRONT_MARGIN, FRONT_X, H, LEDGE_TOP, POST, R, REST_STEPS, SEAT_X, STOP_GAP, STOP_X, TOP, WAFER_START,
                  WALL_H, YC, post_gaps)

Z_ON = LEDGE_TOP + H / 2  # centre height lying on the ledges


def on(x, y=YC, dz=0.0):
    return (x, y, Z_ON + dz)


def tilt_x(deg):
    """Turned about the slot's long axis (x): one edge down."""
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def tilted_on(x, deg):
    """Tilted deg about x with its lowest edge exactly on the ledge tops."""
    a = math.radians(deg)
    return (x, YC, LEDGE_TOP + (H / 2) * math.cos(a) + R * math.sin(a))


def x_at_gap(gap):
    """Centre x on the slot centre line where the larger post gap equals gap (bisection)."""
    from yam_arena.task_file import ObjectState
    lo, hi = FRONT_X, SEAT_X
    for _ in range(200):
        m = (lo + hi) / 2
        p = np.array(on(m))
        g = max(post_gaps(ObjectState(p, np.array((0, 0, 0, 1.0)), np.zeros(3), np.zeros(3), p, p)))
        lo, hi = (m, hi) if g > gap else (lo, m)
    return hi


def hold(label, spec, want, want_success, steps=REST_STEPS, near_last=False):
    """The same end state for `steps` control steps: not yet at rest until the last one, unless it is within
    0.5 mm and 0.5 degrees of the previous end state (near_last), when it is still from the first step."""
    if near_last:
        return [(f"{label} (step {k + 1}, still: within 0.5 mm of the last state)", {"wafer": spec}, want, want_success)
                for k in range(steps)]
    out = [(f"{label} (step {k + 1}, not yet still)", {"wafer": spec}, 0.0, False) for k in range(steps - 1)]
    return out + [(f"{label} (still for {steps} steps)", {"wafer": spec}, want, want_success)]


FALLING = (0.0, 0.0, -0.8)
FLIP = (1.0, 0.0, 0.0, 0.0)  # upside down: looks the same as face up
SIM_STILL = (0.0, -0.0, -0.024)  # the velocity the simulator reports for a wafer lying still on the ledges

CASES = [
    *hold("start on the pins", (WAFER_START, None, None), 0.0, False),
    *hold("seated at the stop, centred", (on(SEAT_X), None, None), 1.0, True),
    *hold("seated, simulator reporting 2.4 cm/s", (on(SEAT_X), None, SIM_STILL), 1.0, True, near_last=True),
    *hold("seated, upside down", (on(SEAT_X), None, None, FLIP), 1.0, True),
    *hold("gap exactly 2 mm at the posts", (on(x_at_gap(STOP_GAP)), None, None), 1.0, True),
    *hold("gap 2.05 mm", (on(x_at_gap(STOP_GAP + 0.00005)), None, None), 0.5, False,
          near_last=True),
    *hold("3 mm short of the stop (2.3 mm gap)", (on(SEAT_X - 0.003), None, None), 0.5, False,
          near_last=True),
    *hold("on both ledges, middle of the slot", (on(0.32), None, None), 0.5, False),
    *hold("touching the left post, 5 mm toward the left wall", (on(SEAT_X - 0.004, YC + 0.005), None, None), 0.5, False),
    *hold("centre exactly 20 mm inside the ledges' near ends", (on(FRONT_X + FRONT_MARGIN), None, None), 0.5, False),
    *hold("centre 19.9 mm inside the ledges' near ends", (on(FRONT_X + FRONT_MARGIN - 0.0001), None, None), 0.0, False),
    *hold("underside exactly 1 mm above the ledges at the stop", (on(SEAT_X, dz=0.001), None, None), 1.0, True),
    *hold("underside 1.05 mm above the ledges (held)", (on(SEAT_X, dz=0.00105), None, None), 0.0, False),
    *hold("held 3 mm above the ledges at the stop", (on(SEAT_X, dz=0.003), None, None), 0.0, False),
    *hold("tilted exactly 2 degrees, edge on the ledges", (tilted_on(0.32, 2.0), None, None, tilt_x(2.0)), 0.5, False),
    *hold("tilted 2.05 degrees", (tilted_on(0.32, 2.05), None, None, tilt_x(2.05)), 0.0, False),
    *hold("cross-slotted: one edge on a ledge, tilted 10 degrees", (tilted_on(SEAT_X, 10), None, None, tilt_x(10)), 0.0,
          False),
    # Lowered onto the ledges 0.2 mm per step (4 mm/s): its pose keeps changing, so it is never at rest.
    *[(f"lowered slowly onto the seat, step {k + 1}", {"wafer": (on(SEAT_X, dz=0.002 - 0.0002 * k), None, None)}, 0.0,
       False) for k in range(10)],
    ("sliding 0.6 mm per step at the stop", {"wafer": (on(SEAT_X - 0.0006), None, None)}, 0.0, False),
    *hold("set down on the table beside the pins", ((0.20, 0.30, TOP + H / 2), None, None), 0.0, False),
    *hold("lying across the tops of the side walls", ((0.32, YC, TOP + WALL_H + H / 2), None, None), 0.0, False),
    *hold("lying on the stop posts", ((STOP_X + 0.005, YC, TOP + POST[2] + H / 2), None, None), 0.0, False),
    *hold("on the table inside the slot, below the ledges", ((0.32, YC, TOP + H / 2), None, None), 0.0, False),
    # History: a drop zeroes the trial even if the wafer then lies seated.
    ("exactly 0.5 m/s above the ledges: not a drop", {"wafer": (on(0.32, dz=0.01), None, (0.0, 0.0, -0.5))}, 0.0, False),
    *hold("seated after moving at 0.5 m/s", (on(SEAT_X), None, None), 1.0, True),
    ("falling onto the ledges at 0.8 m/s", {"wafer": (on(SEAT_X, dz=0.01), None, FALLING)}, 0.0, False),
    *hold("seated and at rest after that drop", (on(SEAT_X), None, None), 0.0, False),
    # The robot cannot clear the record: putting the wafer back on its pins by hand is not a scene reset.
    ("carried back toward the pins", {"wafer": ((WAFER_START[0] + 0.01, WAFER_START[1], WAFER_START[2]), None, None)},
     0.0, False),
    *hold("set back exactly on its pins by the robot", (WAFER_START, None, None), 0.0, False),
    *hold("seated again after that", (on(SEAT_X), None, None), 0.0, False),
    # A scene reset: the wafer jumps back onto its pins in one step.
    ("new trial: reset onto the pins", {"wafer": (WAFER_START, None, None)}, 0.0, False),
    *hold("seated in the new trial", (on(SEAT_X), None, None), 1.0, True),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import numpy as np

    import task
    from yam_arena.task_file import Box, Cylinder, ObjectState

    def size(o):
        return np.asarray(o.size, float) if isinstance(o, Box) else np.array([2 * o.radius, 2 * o.radius, o.height])

    def state(pos, sz, vel=None, quat=None):
        pos = np.asarray(pos, float)
        q = np.array([0.0, 0.0, 0.0, 1.0]) if quat is None else np.asarray(quat, float) / np.linalg.norm(quat)
        x, y, z, w = q
        r = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        half = np.abs(r) @ (np.asarray(sz, float) / 2)
        v = np.zeros(3) if vel is None else np.asarray(vel, float)
        return ObjectState(pos, q, v, np.zeros(3), pos - half, pos + half)

    sizes = {o.name: size(o) for o in task.OBJECTS}
    start = {o.name: state(o.pos, sizes[o.name]) for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, (pos, sz, vel, *quat) in changes.items():
            st[name] = state(pos, sizes[name] if sz is None else sz, vel, quat[0] if quat else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        if flag == "FAIL" or "step 1," not in label and "not yet" not in label and "step" not in label[-12:]:
            print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
