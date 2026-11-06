"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: ready Part state}, expected score, expected success). The cutter's state comes from its
own kinematics (body pose and blade position), so turned and tilted cases are exact.
"""

import math

import numpy as np

from task import BLADE_OUT, CUTTER, FLOOR_Z, TOP, TRAY_IN, TRAY_XY

TX, TY = TRAY_XY


def quat(axis, deg):
    a = math.radians(deg) / 2
    v = np.asarray(axis, float) / np.linalg.norm(axis) * math.sin(a)
    return (*v, math.cos(a))


def cutter(blade_in, x=TX, y=TY, z=FLOOR_Z, q=None, vel=None):
    """The cutter with ``blade_in`` metres of the blade slid back (0: fully out, 0.030: fully in)."""
    return CUTTER.state(joints={"blade": blade_in}, pos=(x, y, z), quat_xyzw=q, lin_vel=vel)


IN = BLADE_OUT  # blade fully retracted
OUT_X = TX - 0.012  # with the blade out the cutter is 140 mm long; this keeps its tip inside the 160 mm tray

CASES = [
    ("in the tray, blade fully in", {"box_cutter": cutter(IN)}, 1.0, True),
    ("in the tray, 2 mm of blade showing", {"box_cutter": cutter(BLADE_OUT - 0.002)}, 1.0, True),
    ("in the tray, 2.1 mm of blade showing", {"box_cutter": cutter(BLADE_OUT - 0.0021, x=OUT_X)}, 0.0, False),
    ("in the tray, blade fully out", {"box_cutter": cutter(0.0, x=OUT_X)}, 0.0, False),
    ("in the tray, blade half out", {"box_cutter": cutter(0.015, x=OUT_X)}, 0.0, False),
    ("in the tray turned 20 deg, blade in", {"box_cutter": cutter(IN, q=quat((0, 0, 1), 20))}, 1.0, True),
    ("in the tray turned 180 deg, blade in", {"box_cutter": cutter(IN, q=quat((0, 0, 1), 180))}, 1.0, True),
    ("in the tray, its lowest point exactly 1 mm above the floor", {"box_cutter": cutter(IN, z=FLOOR_Z + 0.001)},
     1.0, True),
    ("held 1.5 mm above the tray floor, blade in", {"box_cutter": cutter(IN, z=FLOOR_Z + 0.0015)}, 0.0, False),
    ("held 10 mm above the tray, blade in", {"box_cutter": cutter(IN, z=FLOOR_Z + 0.010)}, 0.0, False),
    ("in the tray but still moving", {"box_cutter": cutter(IN, vel=(0.0, 0.0, -0.05))}, 0.0, False),
    ("blade out, tip over the tray's front wall", {"box_cutter": cutter(0.0)}, 0.0, False),
    ("blade in, handle end over the back wall",
     {"box_cutter": cutter(IN, x=TX - TRAY_IN[0] / 2 + 0.050)}, 0.0, False),
    ("lying on its side in the tray", {"box_cutter": cutter(IN, z=FLOOR_Z + 0.0125, q=quat((1, 0, 0), 90))}, 0.0,
     False),
    ("upside down in the tray", {"box_cutter": cutter(IN, z=FLOOR_Z + 0.032, q=quat((1, 0, 0), 180))}, 0.0, False),
    ("tilted 15 deg, one end propped on a wall", {"box_cutter": cutter(IN, z=FLOOR_Z + 0.010,
                                                                       q=quat((0, 1, 0), 15))}, 0.0, False),
    ("blade retracted, left where it started", {"box_cutter": cutter(IN, x=0.200, y=-0.200, z=TOP)}, 0.0, False),
    ("blade retracted, cutter on the table beside the tray", {"box_cutter": cutter(IN, x=TX, y=TY - 0.10, z=TOP)},
     0.0, False),
    ("2 mm of blade showing, on the table", {"box_cutter": cutter(BLADE_OUT - 0.002, x=0.2, y=-0.2, z=TOP)}, 0.0,
     False),
    ("2.1 mm of blade showing, on the table", {"box_cutter": cutter(BLADE_OUT - 0.0021, x=0.2, y=-0.2, z=TOP)}, 0.0,
     False),
    ("blade retracted, held 1.5 mm above the table", {"box_cutter": cutter(IN, x=0.2, y=-0.2, z=TOP + 0.0015)},
     0.0, False),
    ("blade retracted, on its side on the table",
     {"box_cutter": cutter(IN, x=0.2, y=-0.2, z=TOP + 0.0125, q=quat((1, 0, 0), 90))}, 0.0, False),
    ("blade retracted, moving on the table", {"box_cutter": cutter(IN, x=0.2, y=-0.2, z=TOP, vel=(0.05, 0, 0))},
     0.0, False),
    ("blade retracted, propped on a tray wall at 15 deg",
     {"box_cutter": cutter(IN, x=TX - TRAY_IN[0] / 2 - 0.03, y=TY, z=TOP + 0.012, q=quat((0, 1, 0), -15))},
     0.0, False),
    ("untouched: blade out on the table", {"box_cutter": cutter(0.0, x=0.200, y=-0.200, z=TOP)}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import task

    start = {o.name: o.state() for o in task.OBJECTS}
    bad = 0
    for label, changes, want, want_success in CASES:
        st = dict(start) | changes
        ok = bool(task.success(st))
        got = 1.0 if ok else float(task.score(st))
        flag = "ok  " if (got, ok) == (want, want_success) else "FAIL"
        bad += flag == "FAIL"
        print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
