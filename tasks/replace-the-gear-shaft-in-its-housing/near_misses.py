"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready ObjectState}, expected score, expected success). Objects not listed stay at their
start. Gripper clearance is measured only in the simulator; offline every case counts as released.
"""

import numpy as np

from task import AZ, DRIFT, DRIFT_R, GEAR, GX, GZ, HX0, HX1, HY, NEAR_END, NEW, NOSE_TIP, OLD, OLD_L, R, TOP

MOVING = (0.0, 0.0, -0.05)


def _q(axis, deg):
    a = np.radians(deg) / 2
    v = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(v * np.sin(a)), np.cos(a))


def new_at(tip_x, y=HY, z=AZ, quat=None, vel=None):
    """The new shaft along the bore with its lead end at tip_x."""
    return NEW.state(pos=(tip_x - NOSE_TIP, y, z), quat_xyzw=quat, lin_vel=vel)


def old_at(front_x, y=HY, z=AZ, quat=None):
    return OLD.state(pos=(front_x + OLD_L / 2, y, z), quat_xyzw=quat)


ON_RAIL = old_at(HX1 + 0.006)  # pushed out onto the rail behind the housing
HOME = new_at(HX1 + 0.004)
GEAR_DROPPED = GEAR.state(pos=(GX, HY, GZ - 0.010))

CASES = [
    ("swapped: new shaft 4 mm past the rear face, old shaft on the rail behind", {"new_shaft": HOME, "old_shaft": ON_RAIL},
     1.0, True),
    ("lead end exactly 2 mm short of the rear face", {"new_shaft": new_at(HX1 - 0.002), "old_shaft": ON_RAIL},
     1.0, True),
    ("lead end 2.5 mm short of the rear face", {"new_shaft": new_at(HX1 - 0.0025), "old_shaft": ON_RAIL}, 0.0, False),
    ("pushed until the near end is exactly flush with the front face",
     {"new_shaft": new_at(HX0 + NEAR_END + NOSE_TIP), "old_shaft": ON_RAIL}, 1.0, True),
    ("pushed 1 mm past flush: near end inside the front bore",
     {"new_shaft": new_at(HX0 + NEAR_END + NOSE_TIP + 0.001), "old_shaft": ON_RAIL}, 0.0, False),
    ("part way: new shaft 45 mm in, old shaft sticking out the back",
     {"new_shaft": new_at(HX0 + 0.045), "old_shaft": old_at(HX0 + 0.045)}, 0.0, False),
    ("old shaft hanging: its front end still 3 mm inside the rear wall",
     {"new_shaft": new_at(HX1 - 0.003), "old_shaft": old_at(HX1 - 0.003, quat=_q((0, 1, 0), 20))}, 0.0, False),
    ("old shaft just clear of the rear face, still in line", {"new_shaft": HOME, "old_shaft": old_at(HX1 + 0.0005)},
     1.0, True),
    ("gear dropped 10 mm, both shafts where a swap leaves them",
     {"new_shaft": HOME, "old_shaft": ON_RAIL, "gear": GEAR_DROPPED}, 0.0, False),
    ("gear exactly 2 mm low", {"new_shaft": HOME, "old_shaft": ON_RAIL, "gear": GEAR.state(pos=(GX, HY, GZ - 0.002))},
     1.0, True),
    ("gear 2.5 mm low", {"new_shaft": HOME, "old_shaft": ON_RAIL, "gear": GEAR.state(pos=(GX, HY, GZ - 0.0025))},
     0.0, False),
    ("gear pitched 10 degrees (one end dropped)", {"new_shaft": HOME, "old_shaft": ON_RAIL,
                                                  "gear": GEAR.state(quat_xyzw=_q((0, 1, 0), 10))}, 0.0, False),
    ("gear swung 12 degrees sideways on the shaft", {"new_shaft": HOME, "old_shaft": ON_RAIL,
                                                     "gear": GEAR.state(quat_xyzw=_q((1, 0, 0), 12))}, 1.0, True),
    ("old shaft pushed out, new shaft never inserted (still on its rest)", {"old_shaft": ON_RAIL}, 0.0, False),
    ("old shaft pushed out with the drift, drift left through the bore, gear dropped onto it",
     {"old_shaft": ON_RAIL, "drift": DRIFT.state(pos=(HX0 + 0.075, HY, AZ - R + DRIFT_R)),
      "gear": GEAR.state(pos=(GX, HY, GZ - 0.004))}, 0.0, False),
    ("new shaft lying on top of the housing", {"new_shaft": new_at(HX1, z=TOP + 0.08 + R), "old_shaft": ON_RAIL},
     0.0, False),
    ("new shaft through, tilted 5 degrees", {"new_shaft": new_at(HX1 + 0.004, quat=_q((0, 1, 0), 5)),
                                             "old_shaft": ON_RAIL}, 0.0, False),
    ("swapped but the new shaft still moving", {"new_shaft": new_at(HX1 + 0.004, vel=MOVING), "old_shaft": ON_RAIL},
     0.0, False),
    ("old shaft pulled out to the front and laid on the table, new shaft through",
     {"new_shaft": HOME, "old_shaft": OLD.state(pos=(HX0 - 0.08, HY + 0.06, TOP + R))}, 1.0, True),
]
