"""Near-successes and unintended strategies, scored offline by factory/check.py. A part's state comes from its own
kinematics (Part.state); the hand is a loose two-block part whose origin is the bottom of the palm's near face."""

import math

from task import HAND, HAND_Y, MAT, MAT_XY, PALM, PALM_X0, RECESS_X, ROBOT, TOP

MAT_TOP = TOP + MAT[2]
CX = 0.035  # the hand's outline centre is 35 mm nearer the robot than its origin


def yaw(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def tilt_x(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def hand(x=MAT_XY[0] + CX, y=MAT_XY[1], z=MAT_TOP, quat=None, vel=None):
    return HAND.state(pos=(x, y, z), quat_xyzw=quat, lin_vel=vel)


def arm(deg, brake=0.0):
    return ROBOT.state(joints={"arm": -math.radians(deg), "brake": brake})


ON_TABLE = dict(x=0.20, y=-0.05, z=TOP)

CASES = [
    ("brake button held down, arm not moved", {"robot": arm(0, 0.02)}, 0.0, False),
    ("arm turned 19 deg, hand still in the recess", {"robot": arm(19)}, 0.0, False),
    ("arm turned exactly 20 deg, hand still in the recess", {"robot": arm(20)}, 0.0, False),
    ("arm turned 50 deg, hand left in the recess", {"robot": arm(50)}, 0.0, False),
    ("arm turned 50 deg, hand on the table", {"robot": arm(50), "hand": hand(**ON_TABLE)}, 0.5, False),
    ("arm turned 50 deg, hand held 5 cm above the mat", {"robot": arm(50), "hand": hand(z=MAT_TOP + 0.05)}, 0.0,
     False),
    ("arm turned 50 deg, hand fell to the floor", {"robot": arm(50), "hand": hand(x=0.20, y=-0.50, z=TOP - 0.3)},
     0.0, False),
    ("hand flat on the mat, centred", {"robot": arm(50), "hand": hand()}, 1.0, True),
    ("hand flat on the mat, turned 90 deg", {"robot": arm(50), "hand": hand(quat=yaw(90))}, 1.0, True),
    ("hand on the mat, arm swung back to the fixture afterwards", {"robot": arm(0), "hand": hand()}, 1.0, True),
    ("hand 1 mm above the mat (limit)", {"robot": arm(50), "hand": hand(z=MAT_TOP + 0.001)}, 1.0, True),
    ("hand 1.5 mm above the mat (still held)", {"robot": arm(50), "hand": hand(z=MAT_TOP + 0.0015)}, 0.0, False),
    ("hand on the mat but still moving", {"robot": arm(50), "hand": hand(vel=(0.0, 0.0, -0.05))}, 0.0, False),
    ("hand on the mat leaning 15 deg", {"robot": arm(50), "hand": hand(z=MAT_TOP + 0.03, quat=tilt_x(15))}, 0.0, False),
    ("hand on its side on the mat", {"robot": arm(50), "hand": hand(z=MAT_TOP + 0.035, quat=tilt_x(90))}, 0.5, False),
    ("hand centre exactly on the mat's far edge", {"robot": arm(50), "hand": hand(x=MAT_XY[0] + MAT[0] / 2 + CX)},
     1.0, True),
    ("hand centre 5 mm past the mat's far edge",
     {"robot": arm(50), "hand": hand(x=MAT_XY[0] + MAT[0] / 2 + CX + 0.005)}, 0.0, False),
    ("hand freed onto the table beside the mat, arm back at the fixture", {"robot": arm(0), "hand": hand(**ON_TABLE)},
     0.5, False),
    ("arm at 15 deg, hand freed and resting on the table", {"robot": arm(15), "hand": hand(**ON_TABLE)}, 0.5, False),
    ("arm at 15 deg, hand pulled 10 mm toward the robot, palm still in the recess",
     {"robot": arm(15), "hand": hand(x=PALM_X0 - 0.010, y=HAND_Y, z=TOP)}, 0.0, False),
    ("arm at 15 deg, palm back face exactly at the recess front (x 33.0)",
     {"robot": arm(15), "hand": hand(x=RECESS_X - PALM[0], y=HAND_Y, z=TOP)}, 0.0, False),
    ("arm at 15 deg, palm back face 1 mm nearer the robot than the recess front",
     {"robot": arm(15), "hand": hand(x=RECESS_X - PALM[0] - 0.001, y=HAND_Y, z=TOP)}, 0.5, False),
    ("arm at 15 deg, hand out of the recess but held 2 cm up",
     {"robot": arm(15), "hand": hand(x=0.20, y=-0.05, z=TOP + 0.02)}, 0.0, False),
    ("arm at 15 deg, hand out of the recess on the table but still sliding",
     {"robot": arm(15), "hand": hand(x=0.20, y=-0.05, z=TOP, vel=(-0.05, 0.0, 0.0))}, 0.0, False),
    ("arm at 15 deg, hand out of the recess, tilted 15 deg, 3 cm over the mat",
     {"robot": arm(15), "hand": hand(z=MAT_TOP + 0.03, quat=tilt_x(15))}, 0.0, False),
    ("arm at 0, hand fell to the floor", {"robot": arm(0), "hand": hand(x=0.20, y=-0.50, z=TOP - 0.3)}, 0.0, False),
]


def _run() -> int:
    """Score every case with task.py's own score() and success(); print and count mismatches."""
    import task

    start = {o.name: o.state() for o in task.OBJECTS if hasattr(o, "state")}
    start["mat"] = None
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
