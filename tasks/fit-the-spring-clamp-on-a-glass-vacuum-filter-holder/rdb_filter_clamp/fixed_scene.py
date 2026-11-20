"""Fixed scene: a 47 mm glass vacuum filter holder on its flask, its loose funnel and its spring clamp, on the
calibrated dual-arm YAM station."""

from pathlib import Path

import numpy as np

from . import layout as L

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("filter_clamp.usda")
CONTROL_HZ = 20
MAX_STEPS = 2400
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

# Tabletop frame: x forward from the line through the arm bases, y to the robot's left, z up from the table.
HOLDER_START = np.array((*L.AXIS, 0.))                       # holder frame origin (axis at table height)
FUNNEL_START = np.array((*L.FUNNEL_START, 0.))               # funnel frame origin (its ground bottom face)
CLAMP_START = np.array((*L.CLAMP_START, L.CLAMP_REST_Z))     # clamp frame origin (ring centre)
JOINT = np.array((*L.AXIS, L.JOINT_Z))                       # centre of the joint between the flanges
ACTORS = ("holder", "funnel")
CLAMP_BODIES = ("knuckle", "jawL", "jawR")
CLAMP_JOINTS = ("hingeL", "hingeR")                          # opening = +hingeL = -hingeR

INSTRUCTION = (
    "Seat the glass funnel over the membrane on the fritted base. Hold it steady while fitting the spring clamp "
    "around both flanges. Take the other hand clear. Grasp only the upper funnel and lift the entire dry assembly "
    "until the bottom of the flask clears the bench by at least 20 mm for one second. Lower it upright onto its "
    "mark, release it and withdraw both hands."
)


def side_camera():
    """Review camera beyond the table's far side, to the right, looking back at the holder: the whole flask,
    the joint and the seated funnel's top, the funnel on its wipe and the clamp in frame. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.90, -.25, TOP + .50))
    back = eye - (.29, -.03, TOP + .20)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(15.))
