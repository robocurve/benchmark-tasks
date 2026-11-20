"""Fixed scene: one Etch A Sketch lying screen up on the calibrated dual-arm YAM station."""

from pathlib import Path

import numpy as np

from . import layout

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("etch_a_sketch.usda")
CONTROL_HZ = 20
MAX_STEPS = 3000
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

# The toy lies flat, screen up, its knobs on the side nearest the robot: the toy's X axis (the player's right)
# points along the station's -y, its Y axis (up the screen) along +x.
TOY_XY = (.380, 0.)
TOY_YAW = float(-np.pi / 2)
_C, _S = float(np.cos(TOY_YAW)), float(np.sin(TOY_YAW))
TOY_ROTATION = np.array(((_C, -_S, 0.), (_S, _C, 0.), (0., 0., 1.)))
TOY_START = (*TOY_XY, 0.)

KNOB_OF = {"left": "left_knob", "right": "right_knob"}       # each arm takes the knob on its own side
KNOB_TOY = {"left_knob": (-layout.KNOB_X, layout.KNOB_Y), "right_knob": (layout.KNOB_X, layout.KNOB_Y)}
LINKS = ("frame", "left_knob", "right_knob")
JOINTS = ("left_knob", "right_knob")


def to_station(point_toy):
    """A point in the toy frame, in the tabletop frame, with the toy on its marks."""
    p = np.zeros(3)
    p[:len(point_toy)] = point_toy
    return np.asarray(TOY_START) + TOY_ROTATION @ p


def knob_station(name):
    """Centre of a knob's top face, tabletop frame."""
    x, y = KNOB_TOY[name]
    return to_station((x, y, layout.KNOB_TOP))


INSTRUCTION = (
    "Draw a right-angled triangle on the Etch A Sketch, starting at the dot near the lower left of the screen: "
    "a line 80 mm to the right, then 60 mm straight up, then a straight slanted line back down to the dot, "
    "turning both knobs at the same time for the slanted line. Do not lift, tilt or shake the toy. Then let go "
    "of both knobs and move both arms away."
)


def side_camera():
    """Review camera: in front of and above the toy, between the arms, looking down at the screen and both
    knobs. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((TOY_XY[0] - .20, TOY_XY[1], TOP + .46))
    back = eye - (TOY_XY[0] + .01, TOY_XY[1], TOP + .02)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(24.))
