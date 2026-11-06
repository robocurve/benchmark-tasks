"""Near-successes and unintended strategies, scored offline.

Each case: (label, {object: state}, expected score, expected success); the saucepan is a ready state
(Part.state()). Cases run in order on one copy of the task module, so the whole-trial record (handle out on the
burner area, fallen, tipped) carries over; the cases that trip it come last, in groups that start from a fresh record
when run here (the factory checker runs CASES only).
"""

import math

from task import BURNER, CTR_TOP, PAN, PAN_START, RING_T, TOP

BX, BY = BURNER
ON_RING = CTR_TOP + RING_T


def pan(x=BX, y=BY, z=ON_RING, yaw_deg=-90.0, tilt=0.0, vel=None):
    a, b = math.radians(yaw_deg) / 2, math.radians(tilt) / 2
    qz = (0.0, 0.0, math.sin(a), math.cos(a))
    if tilt:  # tilt about world x after the yaw: q = qx * qz
        qx = (math.sin(b), 0.0, 0.0, math.cos(b))
        x1, y1, z1, w1 = qx
        x2, y2, z2, w2 = qz
        q = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
    else:
        q = qz
    return {"saucepan": PAN.state(pos=(x, y, z), quat_xyzw=q, lin_vel=vel)}


CASES = [
    ("on the burner, handle to the robot's right", pan(), 1.0, True),
    ("on the burner, handle to the robot's left", pan(yaw_deg=90.0), 1.0, True),
    ("on the burner, handle turned 70 deg from the room side", pan(yaw_deg=-70.0), 1.0, True),
    ("on the burner, handle 60 deg: end 8 mm inside the far edge", pan(yaw_deg=-60.0), 0.0, False),
    ("20 mm off the burner centre (limit)", pan(x=BX + 0.020), 1.0, True),
    ("21 mm off centre", pan(x=BX + 0.021), 0.0, False),
    ("not moved: still at its start", {"saucepan": PAN.state()}, 0.0, False),
    ("turned in at its start but not on the burner", pan(x=PAN_START[0], y=PAN_START[1], z=CTR_TOP, yaw_deg=90.0),
     0.0, False),
    ("at its start turned so the handle is out past the near-right corner (off the burner area: no hazard)",
     pan(x=PAN_START[0], y=PAN_START[1], z=CTR_TOP, yaw_deg=-90.0), 0.0, False),
    ("handle out 60 mm off centre (outside the burner area: no hazard)", pan(x=BX - 0.06, yaw_deg=0.0), 0.0, False),
    ("handle out, pan lifted 10 mm over the burner (base not down: no hazard)", pan(z=ON_RING + 0.010, yaw_deg=0.0),
     0.0, False),
    ("on the burner, tilted 5 deg (limit)", pan(tilt=5.0), 1.0, True),
    ("on the burner, tilted 8 deg", pan(tilt=8.0), 0.0, False),
    ("on the burner, still sliding", pan(vel=(0.03, 0.0, 0.0)), 0.0, False),
    ("held 3 mm above the burner", pan(z=ON_RING + 0.003), 0.0, False),
    ("on the counter top beside the ring (0 mm)", pan(z=CTR_TOP), 1.0, True),
    ("... after all of the above, still on the burner turned in", pan(), 1.0, True),
    # whole-trial record from here on
    ("sliding across the burner with the handle out (moving)", pan(yaw_deg=0.0, vel=(0.03, 0.0, 0.0)), 0.0, False),
    ("... then turned in and set on the burner: still 0", pan(), 0.0, False),
]
GROUPS = [
    CASES,
    [("on the burner, handle straight out to the room (65 mm past the edge)", pan(yaw_deg=0.0), 0.0, False),
     ("... turned in: still 0", pan(), 0.0, False)],
    [("on the burner, handle pointing at the robot (end 5 mm past the near edge)", pan(yaw_deg=180.0), 0.0, False),
     ("... turned in on the burner: still 0", pan(), 0.0, False)],
    [("tipped over on the counter", pan(x=0.25, y=-0.05, tilt=40.0), 0.0, False),
     ("... stood on the burner: still 0", pan(), 0.0, False)],
    [("knocked to the table", pan(x=0.30, y=-0.30, z=TOP), 0.0, False),
     ("... put back on the burner: still 0", pan(), 0.0, False)],
]


def _run() -> int:
    import task

    start = {"counter": task.COUNTER.state(), "step_stool": task.STOOL.state()}
    bad = 0
    for group in GROUPS:
        task._clear(None)
        for label, changes, want, want_ok in group:
            st = start | changes
            ok = bool(task.success(st))
            got = 1.0 if ok else float(task.score(st))
            flag = "ok  " if (got, ok) == (want, want_ok) else "FAIL"
            bad += flag == "FAIL"
            print(f"{flag} {got:4.2f}  {label}")
    return bad


if __name__ == "__main__":
    raise SystemExit(_run())
