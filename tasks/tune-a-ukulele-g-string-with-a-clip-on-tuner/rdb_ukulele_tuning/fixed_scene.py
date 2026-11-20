"""Fixed scene: a soprano ukulele standing on its side in playing position, held in two padded foam V-blocks and a
strap, a clip-on tuner on its headstock tip, on the calibrated dual-arm YAM station. Tabletop frame: x forward from
the line joining the arm bases, y to the robot's left, z up from the table top; metres. Importable without NumPy:
the scorer's tests need only the standard library.

The ukulele's own frame (``layout``: x across the strings toward the G string, y along the strings, z out of the
back toward the strings) is turned a quarter turn about y: its x points up, its z toward the robot. So the soundboard
faces the robot, the neck runs along y with the headstock on the robot's right, the G string is the top string and
the G and C strings' buttons stick straight up from the headstock's upper edge.
"""

import math
from pathlib import Path

from . import layout as L

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only Python
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("ukulele.usda")
CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 2400
ARMS = ("left", "right")
BASES = {arm: (_BASES[arm][0], _BASES[arm][1], _BASES[arm][2] - TOP) for arm in ARMS}    # tabletop frame

# Placement, chosen for reach (each hand works pointing down, in front of its own base): the G button's grip point
# at (282, -163, 150) mm for the right hand, the G string over the soundhole at (253, 97, 118) mm for the left.
SOUNDBOARD_X = .262                             # the soundboard's plane (and the headstock's face)
NUT_Y = -.128                                   # the nut
CENTRE_Z = .1005                                # the ukulele's centre line (the neck's axis) above the table
ORIGIN = (SOUNDBOARD_X + L.TOP_Z, NUT_Y, CENTRE_Z)          # where the instrument frame's origin is on the table
ROTATION_WXYZ = (math.sqrt(.5), 0., -math.sqrt(.5), 0.)    # a quarter turn about y: instrument x up, z toward -x

INSTRUMENT = "Ukulele"                          # the fixed articulation's prim (its root link is welded to the world)
LAMP_SCOPE = "Lamps"                            # the tuner's lamps, shown and hidden as the display changes
ROOT_LINK = "instrument"
BUTTON_LINKS = {name: f"button_{name}" for name in L.STRINGS}
BUTTON_JOINTS = {name: f"peg_{name}" for name in L.STRINGS}
PLUCK_LINK, PLUCK_JOINT = "pluck", "pluck_slide"
NEEDLE_LINK, NEEDLE_JOINT = "needle", "needle_pivot"
LINKS = (ROOT_LINK, *BUTTON_LINKS.values(), PLUCK_LINK, NEEDLE_LINK)
JOINTS = (*BUTTON_JOINTS.values(), PLUCK_JOINT, NEEDLE_JOINT)
LINK_MASSES = (L.INSTRUMENT_MASS, *(L.BUTTON_MASS for _ in L.STRINGS), L.PLUCK_MASS, L.NEEDLE_MASS)

INSTRUCTION = ("The ukulele's G string has gone flat. Tune it to G4 with the clip-on tuner. Then let go of the tuning "
               "button, pluck the G string once more to check it, and move both hands away from the ukulele.")


def direction_to_station(v):
    """A direction of the instrument frame in the tabletop frame."""
    return (-float(v[2]), float(v[1]), float(v[0]))


def direction_to_instrument(v):
    return (float(v[2]), float(v[1]), -float(v[0]))


def to_station(point):
    """A point of the instrument frame in the tabletop frame."""
    d = direction_to_station(point)
    return tuple(ORIGIN[i] + d[i] for i in range(3))


def to_instrument(point):
    """A point of the tabletop frame in the instrument frame."""
    return direction_to_instrument(tuple(float(point[i]) - ORIGIN[i] for i in range(3)))


def side_camera():
    """Review camera: high between the arms and in front of the ukulele, looking down at the soundboard, the
    headstock's upper edge and the tuner's screen. Not a policy input."""
    import numpy as np
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.02, NUT_Y + .07, TOP + .55))
    look = np.array((SOUNDBOARD_X + .01, NUT_Y + .06, TOP + CENTRE_Z - .01))
    back = eye - look
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(18.))
