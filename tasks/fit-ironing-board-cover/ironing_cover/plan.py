"""Move both sewn ends past the deck edges, then tuck and release."""

import numpy as np
from yam_arena.constants import DOWN_XYZW
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from .motion import Program, rotation

FINGERS_ALONG_BOARD = rotation("Z", np.pi) @ quat_xyzw_to_matrix(DOWN_XYZW)


def hem(sign, y, z, grip=1., x=0., tilt=30.):
    orientation = rotation("Y", -np.deg2rad(tilt)) @ FINGERS_ALONG_BOARD
    return np.array([C.BOARD_X + x, sign * y, C.BOARD_TOP + z]), orientation, grip


def brace(sign, height):
    """Press the padded top with closed finger pads on the opposite half."""
    return hem(sign, .155, height, grip=.18, x=-.036, tilt=0.)


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.6, "approach the nose seam and counterhold", left=hem(1,.308,.095), right=brace(-1,.075))
    p.add(1.5, "lower the left fingers and brace the cover", left=hem(1,.308,.019,.46), right=brace(-1,-.010))
    p.add(.75, "pinch the nose elastic", left=hem(1,.308,.012,.035))
    p.add(1.35, "stretch nose out against the counterhold", left=hem(1,.407,.017,.035))
    p.add(.85, "lower the nose seam past the deck", left=hem(1,.408,-.029,.035))
    p.add(1.1, "angle nose fingertips below the rim", left=hem(1,.416,-.031,.035,tilt=77.))
    p.add(1.1, "guide nose elastic inward", left=hem(1,.339,-.039,.035,tilt=77.))
    p.add(.45, "release nose elastic underneath", left=hem(1,.337,-.043,.7,tilt=77.))
    p.add(.85, "lower and withdraw nose fingertips", left=hem(1,.408,-.055,.85,-.029,77.))
    p.add(.9, "clear nose and lift toward center", left=hem(1,.426,.102,1.,-.066,30.))
    p.add(1.15, "move left to counterhold the fitted side", left=brace(1,.075))
    p.add(.75, "hand over the center counterhold", left=brace(1,-.010))
    if mode != "lifted":
        p.add(.68, "lift the right brace vertically off the pad", right=brace(-1,.090))
        p.add(1.3, "approach the unfitted heel", right=hem(-1,.308,.085))
        p.add(1.15, "lower onto heel elastic", right=hem(-1,.308,.019,.46))
        p.add(.70, "pinch heel elastic", right=hem(-1,.308,.012,.035))
        p.add(1.25, "stretch heel out against the counterhold", right=hem(-1,.427,.017,.035))
        p.add(.85, "lower the heel seam past the deck", right=hem(-1,.427,-.029,.035))
        p.add(1.1, "angle heel fingertips below the rim", right=hem(-1,.429,-.031,.035,tilt=77.))
        p.add(1.1, "guide heel elastic inward", right=hem(-1,.339,-.039,.035,tilt=77.))
        p.add(.45, "release heel elastic underneath", right=hem(-1,.337,-.043,.7,tilt=77.))
        p.add(.85, "lower and withdraw heel fingertips", right=hem(-1,.408,-.055,.85,-.029,77.))
    p.add(1.05, "lift the working hand clear of the fitted hem",
          right=brace(-1,.080) if mode == "lifted" else hem(-1,.426,.104,1.,-.066,30.))
    p.add(1.25, "lift the counterhold vertically off the pad", left=brace(1,.080))
    return p.moves
