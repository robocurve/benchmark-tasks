"""Reference sequence using two ordinary parallel-jaw grippers."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def pinch(lateral, approach):
    z = np.asarray(approach, dtype=float)
    z /= np.linalg.norm(z)
    x = np.asarray(lateral) - np.dot(lateral, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


GLASS_PINCH = pinch([0., -1., 0.], [1., -.68, -.18])
CLOTH_ROT = rotation("Z", np.deg2rad(C.CLOTH_YAW))
CLOTH_PINCH = CLOTH_ROT @ pinch([0., -1., 0.], [.80, .09, -1.])
GLASS_GRIP = .020
CLOTH_GRIP = .050


def glass(pos, angle=0., grip=GLASS_GRIP):
    return pos + C.GRASP, rotation("Z", np.deg2rad(angle)) @ GLASS_PINCH, grip


def cloth_pick(height, grip=CLOTH_GRIP):
    return C.CLOTH_START + CLOTH_ROT @ C.CLOTH_GRASP + [0., 0., height], CLOTH_PINCH, grip


def cloth_wipe(distance, height=.122, grip=CLOTH_GRIP):
    r = np.interp(height, C.PROFILE_Z, C.PROFILE_R)
    center = C.GLASS_WORK + CLOTH_ROT @ [-r - distance, 0., height]
    return center + CLOTH_ROT @ (C.CLOTH_GRASP - C.PAD_FACE), CLOTH_PINCH, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.8, "Approach the upper stem and the folded microfiber",
          left=glass(C.GLASS_START + [-.090, 0., .021], grip=.70),
          right=cloth_pick(.100, .72))
    p.add(1.3, "Align each gripper",
          left=glass(C.GLASS_START, grip=.64), right=cloth_pick(0., .66))
    p.add(.68, "Grip the stem near the bowl and pinch the cloth",
          left=glass(C.GLASS_START), right=cloth_pick(0.))
    p.add(1.35, "Lift the glass and the cloth",
          left=glass(C.GLASS_START + [0., 0., .058]), right=cloth_pick(.140))
    p.add(2.25, "Present the first side of the bowl",
          left=glass(C.GLASS_WORK, -8.), right=cloth_wipe(.036))
    gap = .026 if mode == "air-pass" else -.0040
    p.add(1.12, "Press the folded microfiber against the near bowl wall",
          right=cloth_wipe(gap))
    angle = -1. if mode == "short-pass" else 13.
    end_height = .136 if mode == "short-pass" else .174
    p.add(3.85, "Turn the glass under the cloth with a continuous polishing stroke",
          left=glass(C.GLASS_WORK, angle), right=cloth_wipe(gap, end_height))
    p.add(1.12, "Withdraw the cloth from the bowl", right=cloth_wipe(.044, end_height))
    p.add(2.5, "Return to the two bench positions",
          left=glass(C.GLASS_START + [0., 0., .036]), right=cloth_pick(.083))
    p.add(.8, "Level the held glass above its resting point",
          left=glass(C.GLASS_START + [0., 0., .036]), right=cloth_pick(.051))
    p.add(1.40, "Set the glass upright and lower the cloth",
          left=glass(C.GLASS_START + [0., 0., -.0005]), right=cloth_pick(.003))
    p.add(.70, "Rest the glass on its foot")
    p.add(.85, "Release the glass and cloth",
          left=glass(C.GLASS_START + [0., 0., -.0005], grip=.76), right=cloth_pick(.003, .72))
    p.add(1.2, "Withdraw both open grippers",
          left=glass(C.GLASS_START + [-.10, .036, .007], grip=.76), right=cloth_pick(.093, .72))
    return p.moves
