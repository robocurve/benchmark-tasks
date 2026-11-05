"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw])}, expected score, expected
success). Objects not listed stay where they started. The cases run in order, and the scorer keeps trial history
(whether the stack has been broken, and whether it has been lifted as one), so a case that lifts or breaks the stack
affects the cases after it, as in a real trial. The record clears only on a scene reset: every box back within 0.5 mm
and 1 degree of its start, at rest, reached by a jump of more than 20 mm in one step. RESET below is such a reset: the
stack seen 50 mm up, then back at its start in the next step.
"""

import numpy as np

from task import SHELF_TOP, SHELF_X0, SHELF_X1, SHELF_XY, SIZES, STACK_XY, TOP

MOVING = (0.0, 0.0, -0.05)
H = {n: s[2] for n, s in SIZES.items()}


def yaw_q(deg):
    a = np.radians(deg) / 2
    return (0.0, 0.0, float(np.sin(a)), float(np.cos(a)))


def tilt_q(deg):
    a = np.radians(deg) / 2
    return (float(np.sin(a)), 0.0, 0.0, float(np.cos(a)))


def stack(x, y, base_z, dm=(0.0, 0.0), dt=(0.0, 0.0), lift_m=0.0, lift_t=0.0, vel=None, q=None):
    """The three boxes stacked with the bottom box's base at base_z; dm/dt shift the middle/top box sideways
    (relative to the box below), lift_m/lift_t raise them off the box below."""
    zb = base_z + H["bottom_box"] / 2
    zm = base_z + H["bottom_box"] + lift_m + H["middle_box"] / 2
    zt = base_z + H["bottom_box"] + lift_m + H["middle_box"] + lift_t + H["top_box"] / 2
    mx, my = x + dm[0], y + dm[1]
    tx, ty = mx + dt[0], my + dt[1]
    out = {"bottom_box": ((x, y, zb), None, vel), "middle_box": ((mx, my, zm), None, vel),
           "top_box": ((tx, ty, zt), None, vel)}
    if q is not None:
        out = {n: (*v, q) for n, v in out.items()}
    return out


SX, SY = SHELF_XY
X0, Y0 = STACK_XY
LIFTED = stack(X0, Y0, TOP + 0.080)  # carried, 80 mm up
ON_SHELF = stack(SX, SY, SHELF_TOP)
MIDWAY = stack(X0 + 0.07, Y0, TOP)  # set down on the table 70 mm out
RESET = [("(scene reset: stack seen 50 mm up)", stack(X0, Y0, TOP + 0.050), 0.0, False),
         ("(scene reset: back at the start in one step)", {}, 0.0, False)]

CASES = [
    ("stack on the shelf, centred", ON_SHELF, 1.0, True),
    ("stack on the shelf, still falling", stack(SX, SY, SHELF_TOP, vel=MOVING), 0.0, False),
    ("stack held 3 mm above the shelf", stack(SX, SY, SHELF_TOP + 0.003), 0.0, False),
    ("stack held 1.5 mm above the shelf", stack(SX, SY, SHELF_TOP + 0.0015), 0.0, False),
    ("stack on the shelf, 15 mm toward the robot (base still wholly on the shelf)",
     stack(SX - 0.015, SY, SHELF_TOP), 1.0, True),
    ("stack on the shelf, turned 10 degrees, base wholly on it", stack(SX, SY, SHELF_TOP, q=yaw_q(10)), 1.0, True),
    ("stack on the shelf, turned 20 degrees: corners past the front edge",
     stack(SX, SY, SHELF_TOP, q=yaw_q(20)), 0.5, False),
    ("bottom box overhanging the front edge by 10 mm", stack(SHELF_X0 + 0.050, SY, SHELF_TOP), 0.5, False),
    ("bottom box overhanging the far edge by 30 mm", stack(SHELF_X1 - 0.030, SY, SHELF_TOP), 0.5, False),
    ("middle box 9 mm off centre", stack(SX, SY, SHELF_TOP, dm=(0.009, 0.0)), 1.0, True),
    ("middle box 12 mm off centre", stack(SX, SY, SHELF_TOP, dm=(0.012, 0.0)), 0.5, False),
    ("top box 20 mm off centre", stack(SX, SY, SHELF_TOP, dt=(0.0, 0.020)), 0.5, False),
    ("top box resting 2 mm high (held)", stack(SX, SY, SHELF_TOP, lift_t=0.002), 0.0, False),
    ("only the bottom box on the shelf, others left on the table",
     {"bottom_box": ((SX, SY, SHELF_TOP + H["bottom_box"] / 2), None, None),
      "middle_box": ((0.20, 0.15, TOP + H["middle_box"] / 2), None, None),
      "top_box": ((0.20, -0.15, TOP + H["top_box"] / 2), None, None)}, 0.0, False),
    *RESET,
    ("stack slid 70 mm out along the table, never lifted", MIDWAY, 0.0, False),
    *RESET,
    # Lifted as one, then set down on the table.
    ("stack lifted 80 mm as one", LIFTED, 0.0, False),
    ("then set down intact on the table 70 mm out", MIDWAY, 0.25, False),
    ("then set down intact 70 mm out, top box 20 mm off centre",
     stack(X0 + 0.07, Y0, TOP, dt=(0.020, 0.0)), 0.25, False),
    ("then set down 20 mm from the start: not moved far enough", stack(X0 + 0.02, Y0, TOP), 0.0, False),
    ("then on the shelf", ON_SHELF, 1.0, True),
    *RESET,
    ("set down 70 mm out without a lift in this trial", MIDWAY, 0.0, False),
    *RESET,
    # Breaking the stack zeroes the trial, even when it is rebuilt on the shelf.
    ("top box lifted 10 mm off the middle box", stack(X0, Y0, TOP, lift_t=0.010), 0.0, False),
    ("then the stack rebuilt on the shelf: still 0", ON_SHELF, 0.0, False),
    *RESET,
    ("boxes carried one at a time: top box to the shelf first",
     {"top_box": ((SX, SY, SHELF_TOP + H["top_box"] / 2), None, None)}, 0.0, False),
    ("then all three stacked on the shelf: still 0", ON_SHELF, 0.0, False),
    *RESET,
    ("middle and top lifted together 6 mm off the bottom box", stack(X0, Y0, TOP, lift_m=0.006), 0.0, False),
    ("then stack on the shelf: still 0", ON_SHELF, 0.0, False),
    *RESET,
    ("middle box slid 30 mm on the bottom box during the carry",
     stack(X0, Y0, TOP + 0.08, dm=(0.030, 0.0)), 0.0, False),
    ("then pushed back and put on the shelf: still 0", ON_SHELF, 0.0, False),
    *RESET,
    ("whole stack tipped 20 degrees while carried", stack(X0, Y0, TOP + 0.08, q=tilt_q(20)), 0.0, False),
    ("then on the shelf: still 0", ON_SHELF, 0.0, False),
    *RESET,
    ("stack tilted 10 degrees while carried (not broken)", stack(X0, Y0, TOP + 0.08, q=tilt_q(10)), 0.0, False),
    ("then on the shelf", ON_SHELF, 1.0, True),
    *RESET,
    # Hand-rebuilding the start does not clear the record (only a scene reset does).
    ("top box lifted 10 mm off", stack(X0, Y0, TOP, lift_t=0.010), 0.0, False),
    ("then put back exactly at its start", {}, 0.0, False),
    ("then the stack carried to the shelf: still 0", ON_SHELF, 0.0, False),
    *RESET,
    ("top box lifted 10 mm off", stack(X0, Y0, TOP, lift_t=0.010), 0.0, False),
    ("then the stack rebuilt at the start turned 30 degrees", stack(X0, Y0, TOP, q=yaw_q(30)), 0.0, False),
    ("then on the shelf: still 0", ON_SHELF, 0.0, False),
    *RESET,
    # Exact limits and states the scorer cannot tell from released ones.
    ("still squeezed but resting fully on the shelf (counts, as the text says)", ON_SHELF, 1.0, True),
    *RESET,
    ("held 0.9 mm above the shelf (within 1 mm: counts)", stack(SX, SY, SHELF_TOP + 0.0009), 1.0, True),
    *RESET,
    ("middle box exactly 10 mm off centre", stack(SX, SY, SHELF_TOP, dm=(0.010, 0.0)), 1.0, True),
    *RESET,
    ("middle box exactly 25 mm off centre", stack(SX, SY, SHELF_TOP, dm=(0.025, 0.0)), 0.5, False),
    *RESET,
    ("middle box 26 mm off centre (broken)", stack(SX, SY, SHELF_TOP, dm=(0.026, 0.0)), 0.0, False),
    *RESET,
    ("top box upside down on the middle box, on the shelf (flat either way up)",
     {**ON_SHELF, "top_box": (ON_SHELF["top_box"][0], None, None, tilt_q(180))}, 1.0, True),
    *RESET,
    ("bottom box exactly at the shelf's front edge", stack(SHELF_X0 + 0.060, SY, SHELF_TOP), 1.0, True),
    *RESET,
    ("stack lifted, set down on the table, regripped and put on the shelf (allowed)", LIFTED, 0.0, False),
    ("(set down on the table 70 mm out)", MIDWAY, 0.25, False),
    ("(lifted again)", stack(X0 + 0.07, Y0, TOP + 0.08), 0.0, False),
    ("(on the shelf)", ON_SHELF, 1.0, True),
    *RESET,
    ("bottom box set down on its side on the shelf (others knocked off)",
     {"bottom_box": ((SX, SY, SHELF_TOP + 0.06), None, None, tilt_q(90))}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import sys
    from pathlib import Path

    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here.parents[3] / "factory"))
    import check  # noqa: E402  (the factory checker's state(), which widens rotated boxes like the simulator)
    import task

    sizes = {o.name: np.asarray(o.size, float) for o in task.OBJECTS}
    start = {o.name: check.state(o.pos, o.size) for o in task.OBJECTS}
    task.score(start)  # a new trial: clears the history
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start)
        for name, spec in changes.items():
            pos, sz, vel, *rest = spec
            st[name] = check.state(pos, sizes[name] if sz is None else sz, vel, rest[0] if rest else None)
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
