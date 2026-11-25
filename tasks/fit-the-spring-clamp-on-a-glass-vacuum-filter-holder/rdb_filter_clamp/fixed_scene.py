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
    "Grip the glass funnel by its upper cylinder and lower it straight onto the membrane on the frit, "
    "the porous disc that supports the membrane. Hold the funnel steady. With the other gripper, squeeze the "
    "spring clamp handles, slide the jaws over both glass rims (flanges) and let them close. Move that gripper "
    "clear. With the flask on its mark, lift the assembly straight up by the upper cylinder alone. Stop lifting "
    "if the flanges separate. Keep the lowest edge of the flask foot at or below the gauge top. Hold that edge "
    "between the top of the blue stripe and the gauge top for 1 continuous second. Set the flask down upright "
    "on its mark, release the funnel, then move both grippers clear and keep them clear for 0.5 s."
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
