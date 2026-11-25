"""Fixed scene: a collapsed limp tote bag, a carton and a bulky pack on the calibrated dual-arm YAM station."""

from pathlib import Path

import numpy as np

from . import bag_model as m

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only NumPy
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

TASK_ID = "stand-a-limp-bag-up-and-load-a-two-handed-item"
SCORER = "bag_load_score"
USD = Path(__file__).with_name("bag.usda")
CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 2400
ARMS = ("left", "right")
BASES = {arm: np.asarray(_BASES[arm]) - (0., 0., TOP) for arm in ARMS}        # tabletop frame: z = 0 on the table

INSTRUCTION = (
    "Put the box of crackers and the pack of toilet rolls into the bag so that both rest on the bag's base and "
    "the bag stands upright on its own. Then lift the bag by its two handles, set it back down, let go and move "
    "both arms away."
)


def side_camera():
    """Review camera: beyond the bag and to the robot's left, above rim height, looking back at the bag and the
    arms behind it. Not a policy input."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((m.BAG_XY[0] + .60, m.BAG_XY[1] + .52, TOP + .58))
    back = eye - (m.BAG_XY[0] - .04, m.BAG_XY[1] - .03, TOP + .11)
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(20.))
