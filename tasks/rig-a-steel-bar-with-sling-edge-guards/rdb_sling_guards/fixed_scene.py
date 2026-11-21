"""Fixed scene: a 4 x 4 x 12 in steel square on two 2x4 blocks, fixed to the table of the calibrated dual-arm YAM
station; a 3 ft two-ply web sling with steel triangle fittings laid flat on the table under the bar's middle; two trays
with two printed magnetic edge guards each; a 1,000 kg eye double hook with two latches hanging on a chain hoist's
hook over the bar's middle.

Tabletop frame: x forward from the line joining the arm bases, y to the robot's left, z up from the table top; metres.
Importable without NumPy: the scorer's tests need only Python.
"""

import math
from pathlib import Path

from . import layout as L

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the grader's tests need only Python
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

USD = Path(__file__).with_name("rigging.usda")
CONTROL_HZ = 20
PHYSICS_HZ = 240
MAX_STEPS = 6000
ARMS = ("left", "right")
BASES = {arm: (_BASES[arm][0], _BASES[arm][1], _BASES[arm][2] - TOP) for arm in ARMS}    # tabletop frame

# prim names in rigging.usda
ROOT = "Rigging"
BAR = "Bar"                                     # kinematic body: the bar
SLING = "Sling"                                 # floating articulation: fitA, link_00 .. link_NN, fitB
GUARDS = "Guards"                               # free bodies guard_1 .. guard_4
RIG = "HookRig"                                 # fixed-base articulation: beam, hoist, gimbal, hook, two latches
FITS = ("fitA", "fitB")                         # fitA at the left (+y) end of the sling, fitB at the right
LINKS = tuple(f"link_{i:02d}" for i in range(L.N_LINKS))
GUARD_NAMES = ("guard_1", "guard_2", "guard_3", "guard_4")
RIG_JOINTS = ("lift", "swing_x", "swing_y", "latchL_hinge", "latchR_hinge")
SLING_ROOT = LINKS[L.N_LINKS // 2]

# -- where things start -------------------------------------------------------------------------------------------
# The sling lies flat on the table along y at x = XS, its middle under the bar; each triangle fitting lies turned back
# over the end of its own leg (the web's loop turns freely round the fitting's base bar), its apex toward the bar.
WEB_Z = L.WEB_T / 2 + .0004                     # the web's centre plane on the table
BASE_Z = L.FIT_BAR_R + .0005                    # the fittings' base bars rest on the table
APEX_REST_Z = WEB_Z + L.WEB_T / 2 + L.FIT_BAR_R + .0005     # the apex bars rest on the web


def _frame(d):
    """Body rotation (columns x, y, z) with local x = world x and local y along ``d`` (in the y-z plane)."""
    d = (0., d[0], d[1])
    n = math.sqrt(d[1] ** 2 + d[2] ** 2)
    y = (0., d[1] / n, d[2] / n)
    x = (1., 0., 0.)
    z = (0., -y[2], y[1])                       # x cross y
    return (x, y, z)


def sling_start():
    """[(name, (x, y, z), rotation as column vectors x, y, z)] for every body of the sling. Every body's local y runs
    along the web from fitting A (left) toward B; fitting A's apex is at local y = -FIT_H, B's at +FIT_H."""
    ll = L.LINK_L
    dz = BASE_Z - WEB_Z
    dy = math.sqrt(ll * ll - dz * dz)
    y0 = L.N_LINKS * ll / 2
    H = [(y0, BASE_Z), (y0 - dy, WEB_Z)]
    for _ in range(L.N_LINKS - 2):
        H.append((H[-1][0] - ll, WEB_Z))
    H.append((H[-1][0] - dy, BASE_Z))
    eps = math.asin((APEX_REST_Z - BASE_Z) / L.FIT_H)
    out = [("fitA", (L.XS, H[0][0], H[0][1]), _frame((math.cos(eps), -math.sin(eps))))]     # apex toward -y, up
    for i in range(L.N_LINKS):
        (ya, za), (yb, zb) = H[i], H[i + 1]
        out.append((LINKS[i], (L.XS, (ya + yb) / 2, (za + zb) / 2), _frame((yb - ya, zb - za))))
    out.append(("fitB", (L.XS, H[-1][0], H[-1][1]), _frame((math.cos(eps), math.sin(eps)))))     # apex toward +y, up
    return out


def sling_start_bends():
    """The start bend (rad) of hinge_00 .. hinge_NN, body i to body i + 1 about +x (written at reset: a floating
    articulation starts with straight hinges)."""
    bodies = sling_start()
    out = []
    for (_, _, R0), (_, _, R1) in zip(bodies[:-1], bodies[1:]):
        d0, d1 = R0[1], R1[1]
        out.append(math.atan2(d0[1] * d1[2] - d0[2] * d1[1], d0[1] * d1[1] + d0[2] * d1[2]))
    return out


def tray_slots():
    """Where each guard starts: name -> (tray side, edge it is shaped for, inside-corner point). A top-edge guard stands
    as a tent on the tray's inner side (its corner and tabs up, the two flanges 45 degrees down either side, resting on
    their free edges); a bottom-edge guard sits on a block on the outer side as it will sit on that side's bottom edge
    (one flange flat on the block, the other standing on the outside, its corner at the block's outer edge so that the
    tabs hang clear of the floor)."""
    slots = {}
    for side, s in (("left", 1.), ("right", -1.)):
        x, y = L.TRAY_XY[side]
        floor = L.TRAY_SIZE[2]
        top_name, bottom_name = ("guard_1", "guard_2") if side == "left" else ("guard_3", "guard_4")
        slots[top_name] = (side, "TL" if side == "left" else "TR",
                           (x, y + s * L.TENT_Y, floor + L.GUARD_FL / math.sqrt(2) + .0003))
        slots[bottom_name] = (side, "BL" if side == "left" else "BR",
                              (x, y + s * L.BLOCK_CORNER_Y, floor + L.GUARD_BLOCK_H + L.GUARD_T + .0003))
    return slots


def tray_block(side):
    """The block under the bottom-edge guard: (centre, half extents) in the tabletop frame."""
    x, y = L.TRAY_XY[side]
    s = 1. if side == "left" else -1.
    outer = y + s * (L.BLOCK_CORNER_Y - .001)
    inner = outer - s * L.GUARD_BLOCK[1]
    floor = L.TRAY_SIZE[2]
    return ((x, (outer + inner) / 2, floor + L.GUARD_BLOCK_H / 2), (L.GUARD_BLOCK[0] / 2, L.GUARD_BLOCK[1] / 2, L.GUARD_BLOCK_H / 2))


HOOK_O = (L.XS, 0., L.HOOK_Z + L.BOWL_R_IN)      # the hook frame's origin at rest (midway between the bowl centres)
PIVOT = (L.XS, 0., HOOK_O[2] + L.EYE_TOP + L.PEND_L)   # the pendulum's pivot: the top of the hoist's load chain
INSTRUCTION = ("The steel bar is sitting on its blocks with the web sling laid under it. Hang both of the sling's "
               "triangle fittings on the crane hook so the bar is ready to lift, then let go and move clear.")


def side_camera():
    """Review camera: in front of the bar and to the robot's right, above it, looking at the bar's middle: the right
    side face with its two edges at the sling, the top face, the double hook and the right tray show. Not a policy
    input."""
    import numpy as np
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.60, -.45, TOP + .36))
    look = np.array((L.XS, 0., TOP + .15))
    back = eye - look
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(18.))
