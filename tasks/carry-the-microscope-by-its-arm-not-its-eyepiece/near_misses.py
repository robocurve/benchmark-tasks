"""Near-successes and unintended strategies, scored offline by the factory checker (factory/check.py).

Each case: (label, {object: spec}, expected score, expected success). The microscope's spec is a ready state from its
own kinematics (SCOPE.state(...)). Cases run in order and share the whole-trial record, so a fault stays until a
scene reset (an empty case after the microscope was moved).
"""

import math

from task import BASE, BASE_CX, EYE_STOP, MARK_TOP, SCOPE, SCOPE_XY, SCOPE_YAW, SHELF_XY, TOP

MX, MY = SHELF_XY


def _mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


YAW = (0.0, 0.0, math.sin(SCOPE_YAW / 2), math.cos(SCOPE_YAW / 2))


def rot_y(deg):
    """Tipped deg about the microscope's own y axis (across it), on top of its start heading."""
    a = math.radians(deg) / 2
    return _mul(YAW, (0.0, math.sin(a), 0.0, math.cos(a)))


def rot_z(deg):
    a = math.radians(deg) / 2
    return _mul(YAW, (0.0, 0.0, math.sin(a), math.cos(a)))


def on_mark(dx=0.0, dy=0.0, dz=0.0, eye=0.0, quat=None, vel=None):
    """The microscope with its base centre (dx, dy) from the mark's centre and its underside dz above the mark."""
    q = YAW if quat is None else quat
    x, y, z, w = q
    # The base's centre is BASE_CX along the part's own x from its origin.
    bx, by = (1 - 2 * (y * y + z * z)) * BASE_CX, 2 * (x * y + z * w) * BASE_CX
    return {"microscope": SCOPE.state(joints={"eyepiece": eye}, pos=(MX - bx + dx, MY - by + dy, MARK_TOP + dz),
                                      quat_xyzw=q, lin_vel=vel)}


def carried(eye=0.0, z=0.05, quat=None, vz=0.0):
    return {"microscope": SCOPE.state(joints={"eyepiece": eye}, pos=(0.27, 0.05, TOP + z),
                                      quat_xyzw=YAW if quat is None else quat, lin_vel=(0.0, -0.1, vz))}


# Tilting about y by a lifts one end of the base by BASE[0] sin(a): keep the low corner on the mark.
def tilted_on_mark(deg):
    lift = BASE[0] / 2 * math.sin(math.radians(deg))
    return on_mark(dz=lift, quat=rot_y(deg))


SCENE_RESET = {}

CASES = [
    ("on the mark, centred, eyepiece seated", on_mark(), 1.0, True),
    ("1 mm above the mark", on_mark(dz=0.001), 1.0, True),
    ("held 3 mm above the mark", on_mark(dz=0.003), 0.0, False),
    ("still moving on the mark", on_mark(vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("20 mm off centre", on_mark(dx=0.020), 1.0, True),
    ("21 mm off centre", on_mark(dy=0.021), 0.0, False),
    ("turned 90 degrees on the mark", on_mark(quat=rot_z(90)), 1.0, True),
    ("tilted exactly 3 degrees on the mark edge", tilted_on_mark(3.0), 1.0, True),
    ("tilted 4 degrees on the mark edge", tilted_on_mark(4.0), 0.0, False),
    ("eyepiece 1 mm up at the end", on_mark(eye=0.001), 1.0, True),
    ("carried by the arm, eyepiece 3 mm up exactly", carried(eye=0.003), 0.0, False),
    ("then set on the mark: 1.0", on_mark(), 1.0, True),
    ("carried tilted exactly 15 degrees", carried(quat=rot_y(15)), 0.0, False),
    ("then set on the mark: still 1.0", on_mark(), 1.0, True),
    ("left on the table at its start", {}, 0.0, False),
    ("pushed along the table to the shelf's foot",
     {"microscope": SCOPE.state(pos=(MX - 0.15, MY + 0.10, TOP))}, 0.0, False),
    ("set on the shelf beside the mark", on_mark(dy=0.075), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    # Faults from here on; each is followed by a scene reset.
    ("lifted by the eyepiece: tube at its stop, scope hanging 8 mm up", carried(eye=EYE_STOP, z=0.008), 0.0, False),
    ("then set on the mark with the eyepiece pushed back in: still 0", on_mark(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("after the reset, delivered: 1.0", on_mark(), 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("eyepiece pulled 4 mm up while carried", carried(eye=0.004), 0.0, False),
    ("then delivered: 0", on_mark(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("tipped 16 degrees while carried", carried(quat=rot_y(16)), 0.0, False),
    ("then delivered: 0", on_mark(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("dropped: falling at 0.7 m/s", carried(z=0.03, vz=-0.7), 0.0, False),
    ("then lying flat on the mark: 0", on_mark(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("released 15 mm up: landing at 0.55 m/s", carried(z=0.06, vz=-0.55), 0.0, False),
    ("then on the mark: 1.0", on_mark(), 1.0, True),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("centre-of-object grasp from above: lands on the eyepiece, which slides out as the arm lifts",
     {"microscope": SCOPE.state(joints={"eyepiece": 0.012}, quat_xyzw=YAW)}, 0.0, False),
    ("then the eyepiece drops back and the scope is carried to the mark by its arm: still 0", on_mark(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("highest-point grasp: the eyepiece pulled to its stop, the scope dragged up the shelf's side",
     {"microscope": SCOPE.state(joints={"eyepiece": EYE_STOP}, pos=(0.27, 0.06, TOP + 0.02), quat_xyzw=rot_y(10))},
     0.0, False),
    ("then on the mark: 0", on_mark(), 0.0, False),
    ("scene reset", SCENE_RESET, 0.0, False),
    ("knocked over on the table",
     {"microscope": SCOPE.state(pos=(SCOPE_XY[0], SCOPE_XY[1], TOP + 0.04), quat_xyzw=rot_y(90))}, 0.0, False),
]
