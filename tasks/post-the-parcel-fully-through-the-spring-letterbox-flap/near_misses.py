"""Near-successes and unintended strategies, scored offline by factory/check.py.

Each case: (label, {object: (centre, size or None, velocity or None[, quat_xyzw]) or a ready state}, expected score,
expected success).
"""

import math

from task import DOOR_H, FLAP_SHUT, HALL_D, LETTERBOX, PACKET_SIZE, SLOT_Z0, TOP, X_IN, X_OUT, YC

L, W, H = PACKET_SIZE


def qy(deg):
    """Turned about the world y axis (front end down for positive angles)."""
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def flat_at(low_x, bottom):
    """Lying flat along x with its trailing end at low_x and its underside at height `bottom` above the table."""
    return (low_x + L / 2, YC, TOP + bottom + H / 2)


def leaning(deg, low_x):
    """Tilted `deg` about y, with its bounding box's near side at low_x and its lowest point on the table."""
    a = math.radians(deg)
    hx = (L * math.cos(a) + H * math.sin(a)) / 2
    hz = (L * math.sin(a) + H * math.cos(a)) / 2
    return (low_x + hx, YC, TOP + hz)


def flap(q):
    return {"letterbox": LETTERBOX.state(joints={"flap": q})}


MOVING = (0.0, 0.0, -0.05)
IN_HALL = flat_at(X_IN + 0.04, 0.0)

CASES = [
    ("inside on the box floor, flap shut", {"packet": (IN_HALL, None, None)}, 1.0, True),
    ("inside, trailing end exactly at the inner face", {"packet": (flat_at(X_IN, 0.0), None, None)}, 1.0, True),
    ("on the box floor, 1 mm still inside the slot line", {"packet": (flat_at(X_IN - 0.001, 0.0), None, None)},
     0.0, False),
    ("inside, leaning 60 deg against the door's inner face",
     {"packet": (leaning(60, X_IN + 0.0005), None, None, qy(60))}, 1.0, True),
    ("inside against the back wall", {"packet": (flat_at(X_IN + HALL_D - L, 0.0), None, None)}, 1.0, True),
    ("inside, flap at the shut limit (5 mm)", {"packet": (IN_HALL, None, None), **flap(FLAP_SHUT)}, 1.0, True),
    ("inside, flap 6 mm open", {"packet": (IN_HALL, None, None), **flap(math.asin(0.006 / 0.066))}, 0.0, False),
    ("inside, flap held open 0.6 rad", {"packet": (IN_HALL, None, None), **flap(0.6)}, 0.0, False),
    ("inside but still sliding", {"packet": (IN_HALL, None, MOVING)}, 0.0, False),
    ("lying on the box roof", {"packet": (flat_at(X_IN + 0.04, DOOR_H + 0.010), None, None)}, 0.0, False),
    ("lying on top of the door panel", {"packet": (flat_at(X_OUT - 0.05, DOOR_H), None, None)}, 0.0, False),
    ("fell behind the box box", {"packet": (flat_at(X_IN + HALL_D + 0.02, 0.0), None, None)}, 0.0, False),
    ("stuck flat in the slot, 70 mm in (no partial credit)", {"packet": (flat_at(X_OUT + 0.07 - L, SLOT_Z0), None, None)}, 0.0, False),
    ("stuck flat in the slot, flap resting on it",
     {"packet": (flat_at(X_OUT + 0.07 - L, SLOT_Z0), None, None), **flap(1.15)}, 0.0, False),
    ("in the slot, 1 mm above its floor (limit)", {"packet": (flat_at(X_OUT + 0.07 - L, SLOT_Z0 + 0.001), None, None)},
     0.0, False),
    ("in the slot, held 3 mm above its floor", {"packet": (flat_at(X_OUT + 0.07 - L, SLOT_Z0 + 0.003), None, None)},
     0.0, False),
    ("in the slot but still moving", {"packet": (flat_at(X_OUT + 0.07 - L, SLOT_Z0), None, MOVING)}, 0.0, False),
    ("only 9 mm into the slot", {"packet": (flat_at(X_OUT + 0.009 - L, SLOT_Z0), None, None)}, 0.0, False),
    ("exactly 10 mm into the slot", {"packet": (flat_at(X_OUT + 0.010 - L, SLOT_Z0), None, None)}, 0.0, False),
    ("hanging tilted 20 deg out of the slot", {"packet": ((X_OUT + 0.0, YC, TOP + SLOT_Z0), None, None, qy(-20))},
     0.0, False),
    ("on the table in front of the door", {"packet": (flat_at(X_OUT - L - 0.01, 0.0), None, None)}, 0.0, False),
    ("standing on end against the door face", {"packet": ((X_OUT - H / 2 - 0.001, YC, TOP + L / 2), None, None, qy(90))},
     0.0, False),
]
