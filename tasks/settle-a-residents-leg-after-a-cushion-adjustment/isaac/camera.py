"""Task-specific overhead view. Wrist calibrations and arm bases use the station defaults."""

from dataclasses import replace

import numpy as np
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.rig import Camera, Lens

EYE = (1.32, -1.02, 1.16)
LOOK = (0.54, -0.24, 0.84)


def configure(rig):
    z = np.asarray(EYE) - LOOK
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    q = matrix_to_quat_xyzw(np.column_stack((x, y, z)))
    cam = Camera(EYE, tuple(q), "opengl", Lens(585, 585, 320, 180, 640, 360))
    return replace(rig, top_cam=cam)


def floor_stand(top, factory):
    """Anchor the overhead mast to the floor behind the near edge of the table."""
    from yam_arena import station

    layout = factory(top)
    x, y, z = top.pos
    foot_z = station.FLOOR_Z + 0.01
    bottom = station.FLOOR_Z + 0.02
    layout["stand_foot"] = (
        "box",
        (0.24, 0.22, 0.02),
        (x - 0.07, y, foot_z),
        (0.0, 0.0, 0.0, 1.0),
    )
    layout["stand_mast"] = (
        "capsule",
        (0.012, z - bottom),
        (x - 0.07, y, (z + bottom) / 2),
        (0.0, 0.0, 0.0, 1.0),
    )
    return layout
