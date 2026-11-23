"""Fixed scene: one storage drawer with a pillow in it on the calibrated dual-arm YAM station."""

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

USD = Path(__file__).with_name("drawer.usda")
CONTROL_HZ = 20
MAX_STEPS = 1800
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table
ACTORS = layout.ACTORS
START = layout.start_poses()                    # tabletop frame
WORLD_START = {name: (x, y, z + TOP) for name, (x, y, z) in START.items()}

INSTRUCTION = (
    "The pillow in the open drawer stands higher than the drawer's opening. Shut the drawer completely with the "
    "whole pillow inside: hold the pillow down while you push the drawer in, and get your hand out as it closes. "
    "Then let go and move both arms away from the drawer."
)


def side_camera():
    """Review camera: in front of the unit on the left, above, looking down at the open drawer and the edge the
    pillow catches on. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.02, .50, TOP + .42))
    back = eye - (layout.FRAME_FRONT_X - .04, 0., TOP + .10)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(22.))
