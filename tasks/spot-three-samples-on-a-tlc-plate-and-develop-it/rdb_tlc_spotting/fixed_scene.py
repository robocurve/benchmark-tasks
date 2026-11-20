"""Fixed scene: a glass-backed TLC plate on a narrow bar, three empty capillary spotters in a rack, three vials of
dyed water, and a rectangular glass TLC developing tank with 5 mm of water and its lid, on the calibrated dual-arm
YAM station.
Tabletop frame: x forward from the line joining the arm bases, y to the robot's left, z up from the table top;
metres. Importable without NumPy: the scorer's tests need only the standard library.
"""

from pathlib import Path

from . import layout as L

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only Python
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("tlc.usda")
CONTROL_HZ = L.CONTROL_HZ
PHYSICS_HZ = 240
MAX_STEPS = 4800
ARMS = ("left", "right")
BASES = {arm: (_BASES[arm][0], _BASES[arm][1], _BASES[arm][2] - TOP) for arm in ARMS}    # tabletop frame

SPOTTERS = ("spotter_1", "spotter_2", "spotter_3")
ACTORS = ("plate", *SPOTTERS, "tank", "lid")
IDENTITY_XYZW = (0., 0., 0., 1.)
START = {                                       # (position, quaternion xyzw) of each loose body, tabletop frame
    "plate": ((L.PLATE_X, L.PLATE_Y, L.PLATE_Z), IDENTITY_XYZW),
    **{name: ((L.RACK_X, L.HOLE_Y[k], L.RACK[2] - L.HOLE_DEPTH + L.CAP_LEN / 2), IDENTITY_XYZW)
       for k, name in enumerate(SPOTTERS)},
    "tank": ((L.TANK_X, L.TANK_Y, 0.), IDENTITY_XYZW),                   # its frame: the base's middle
    "lid": ((L.LID_X, L.LID_Y, L.LID[2] / 2), IDENTITY_XYZW),            # its frame: its middle
}

INSTRUCTION = ("Counting from the robot's left, the first spotter goes with the first vial and the first mark on the "
               "baseline, the second with the second, the third with the third. Fill each spotter by dipping its tip "
               "in its vial, touch it once, briefly and lightly, to its mark, and return it to the rack. Then stand the "
               "plate in the developing tank, spotted face out, bottom edge in the solvent and baseline above it, "
               "leaning back against the back wall, and put the lid on.")


def side_camera():
    """Review camera: in front of the table on the right, high, looking back at the plate, the rack, the vials and
    the tank. Not a policy input."""
    import numpy as np
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.60, -.10, TOP + .34))
    look = np.array((.235, -.02, TOP + .03))
    back = eye - look
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(16.))
