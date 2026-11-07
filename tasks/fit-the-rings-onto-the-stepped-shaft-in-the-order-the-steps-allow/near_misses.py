"""Near-successes and unintended strategies, scored offline by the factory checker.

Each case: (label, {object: ready state}, expected score, expected success). Ring states come from the part's own
kinematics (Part.state), as the simulator reports them.
"""

import numpy as np

from task import (BLUE_RING, ORANGE_RING, PLATE_TOP, POST_XY, RING_H, SHOULDER, TOP)

H = RING_H / 2
PX, PY = POST_XY
MOVING = (0.0, 0.0, -0.05)


def q(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    s = np.sin(np.radians(deg) / 2)
    return (*(a * s), np.cos(np.radians(deg) / 2))


def blue(z=PLATE_TOP, dx=0.0, dy=0.0, quat=None, vel=None):
    """Blue with its underside at z (when flat)."""
    return BLUE_RING.state(pos=(PX + dx, PY + dy, z + H), quat_xyzw=quat, lin_vel=vel)


def orange(z=SHOULDER, dx=0.0, dy=0.0, quat=None, vel=None):
    return ORANGE_RING.state(pos=(PX + dx, PY + dy, z + H), quat_xyzw=quat, lin_vel=vel)


def tilted(state_fn, deg, z, **kw):
    """A ring tilted about y by deg, lowest point at z."""
    s = state_fn(z=z, quat=q((0, 1, 0), deg), **kw)
    return state_fn(z=z + (z - float(s.low[2])), quat=q((0, 1, 0), deg), **kw)


ORANGE_ON_TABLE = ORANGE_RING.state(pos=(0.25, 0.12, TOP + H))
BLUE_ON_TABLE = BLUE_RING.state(pos=(0.25, -0.12, TOP + H))

CASES = [
    ("both seated", {"blue_ring": blue(), "orange_ring": orange()}, 1.0, True),
    ("both seated, orange turned 45 degrees", {"blue_ring": blue(), "orange_ring": orange(quat=q((0, 0, 1), 45))},
     1.0, True),
    ("blue exactly 1 mm up", {"blue_ring": blue(PLATE_TOP + 0.001), "orange_ring": orange()}, 1.0, True),
    ("blue 1.2 mm up (held above the plate)", {"blue_ring": blue(PLATE_TOP + 0.0012), "orange_ring": orange()},
     0.0, False),
    ("orange exactly 1.5 mm above the shoulder", {"blue_ring": blue(), "orange_ring": orange(SHOULDER + 0.0015)},
     1.0, True),
    ("orange 2 mm above the shoulder (held)", {"blue_ring": blue(), "orange_ring": orange(SHOULDER + 0.002)},
     0.0, False),
    ("blue seated, orange held in the air 3 cm above the shaft top",
     {"blue_ring": blue(), "orange_ring": orange(SHOULDER + 0.07)}, 0.0, False),
    ("blue seated, orange still at its start (one ring fitted)", {"blue_ring": blue()}, 0.0, False),
    ("blue seated, orange moved elsewhere on the table", {"blue_ring": blue(), "orange_ring": ORANGE_ON_TABLE},
     0.0, False),
    ("blue seated, orange dropped on the floor",
     {"blue_ring": blue(), "orange_ring": ORANGE_RING.state(pos=(0.25, 0.12, TOP - 0.3))}, 0.0, False),
    ("blue seated, orange lying cocked on the shaft top",
     {"blue_ring": blue(), "orange_ring": tilted(orange, 20, SHOULDER + 0.02)}, 0.0, False),
    ("habit 'left arm first' (orange first): orange on the shoulder, blue stuck on top of it",
     {"orange_ring": orange(), "blue_ring": blue(SHOULDER + RING_H)}, 0.0, False),
    ("habit 'left arm first', blue stuck on orange and turned 30 degrees",
     {"orange_ring": orange(), "blue_ring": blue(SHOULDER + RING_H, quat=q((0, 0, 1), 30))}, 0.0, False),
    ("orange seated, blue still at its start", {"orange_ring": orange()}, 0.0, False),
    ("orange seated, blue on the table", {"orange_ring": orange(), "blue_ring": BLUE_ON_TABLE}, 0.0, False),
    ("blue tilted 3 degrees on the plate", {"blue_ring": tilted(blue, 3.0, PLATE_TOP), "orange_ring": orange()},
     1.0, True),
    ("blue tilted 4 degrees (cocked on the plate)", {"blue_ring": tilted(blue, 4.0, PLATE_TOP),
                                                     "orange_ring": orange()}, 0.0, False),
    ("orange tilted 4 degrees on the shoulder", {"blue_ring": blue(), "orange_ring": tilted(orange, 4.0, SHOULDER)},
     0.0, False),
    ("blue pushed 5.6 mm along the diagonal (bore against the shaft)",
     {"blue_ring": blue(dx=0.004, dy=0.004), "orange_ring": orange()}, 1.0, True),
    ("blue on the plate level but 7 mm off the axis (not around the shaft)",
     {"blue_ring": blue(dx=0.007), "orange_ring": orange()}, 0.0, False),
    ("both seated but blue still moving", {"blue_ring": blue(vel=MOVING), "orange_ring": orange()}, 0.0, False),
    ("blue lying on the plate edge beside the shaft, tilted",
     {"blue_ring": tilted(blue, 12, PLATE_TOP, dx=0.045)}, 0.0, False),
    ("both rings on the table", {"blue_ring": BLUE_ON_TABLE, "orange_ring": ORANGE_ON_TABLE}, 0.0, False),
]
