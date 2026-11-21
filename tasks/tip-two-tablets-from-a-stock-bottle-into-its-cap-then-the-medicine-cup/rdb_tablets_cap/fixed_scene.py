"""Fixed scene: a closed stock bottle of tablets, its cap and a medicine cup on the dual-arm YAM station."""

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

USD = Path(__file__).with_name("tablets.usda")
CONTROL_HZ = 20
MAX_STEPS = 4800
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

# Tabletop frame, metres: x forward from the line through the arm bases, y to the robot's left.
BOTTLE_XY = (.360, -.140)        # stands upright, capped
CAP_XY = (.330, .050)            # clear landing place for the removed cap
CUP_XY = (.400, .165)            # stands upright, empty
TABLETS = tuple(f"tablet_{i:02d}" for i in range(layout.N_TABLETS))
BODIES = ("bottle", "cap", "cup", *TABLETS)
MASSES = (layout.BOTTLE_MASS, layout.CAP_MASS, layout.CUP_MASS, *([layout.TABLET_MASS] * layout.N_TABLETS))
START = {"bottle": (*BOTTLE_XY, 0.), "cap": (*BOTTLE_XY, layout.CLOSED_Z), "cup": (*CUP_XY, 0.)}
START.update({name: tuple(np.asarray(START["bottle"]) + layout.tablet_start(i)) for i, name in enumerate(TABLETS)})

INSTRUCTION = (
    "Steady the bottle, push its safety cap down, turn it counterclockwise and lift it off. Without touching a tablet, hold the bottle's cap in one hand and, with the other, "
    "tip two tablets from the bottle into it; tip any extra from the cap back into the bottle, then tip the two from "
    "the cap into the medicine cup. Leave the bottle standing and let go of everything."
)


def side_camera():
    """Review camera: in front of the robot's left shoulder, looking down at the cap, the cup and the bottle. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.12, .12, TOP + .38))
    back = eye - (.38, .02, TOP + .04)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(22.))
