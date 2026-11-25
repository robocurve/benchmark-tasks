"""A travel slow cooker with a locking lid (Crock-Pot Cook & Carry, 6 quart, manual dial), built from the kit's parts
as one self-contained module, so a sibling task can copy it.

What it models. One free-standing articulation (MeshMechanism, self-collision on) on its four feet:
  - base: the heating base (an oval stainless body), the stoneware's rim, the glass lid with its rim and loop knob
    (the lid is part of the base: in this model it never comes off), the two side handles, a clip hook on each side
    above the handle, a latch base on each end of the lid's rim, and four index marks round the dial;
  - lever_l / lever_r ("Latch" in the maker's guide): a plastic lever pivoted at the outer end of each latch base on
    an axis along x; closed, it lies flat on the latch base pointing in toward the lid's middle; opened, it is
    flipped up and round so that it stands up and then points out over the handle;
  - wire_l / wire_r ("Wire Clip"): a U-shaped steel wire hinged on its lever just past the top of the lever's pivot
    (3 mm in from it and 7 mm above it when the lever is closed: an over-centre draw latch). Its crossbar sits in the
    clip hook ("Clip Hook"): a block beside the body with a nose pointing outward and a short lip turned down at the
    nose's end; the crossbar is under the nose, inside the lip, 1.5 mm below the nose (that play stands for the give
    of the lid's gasket, which a rigid model does not have). Flipping the lever up carries the wire's hinge out over
    the handle and down, so the crossbar drops below the lip and the wire swings out of the hook under its own
    weight as the lever comes upright (the guide hooks and unhooks the wire with the lever standing up: "Flip the
    latch up and around to loosen the Wire Clip. Clear the Wire Clip from the Clip Hook.");
  - dial: the heat-setting knob low on the front (a 45 mm round knob with a raised bar), turning clockwise as seen
    from the front (positive about +x) from OFF (bar upright) through LOW, HIGH and WARM, 45 degrees apart.
Frame: origin on the table under the middle of the footprint; x from the front (dial side, toward the robot) to the
back, y along the cooker's length (handles at +-y), z up. Metres; comments in millimetres.

Tuned numbers (all ESTIMATES unless the README gives a source; one-minute checks there):
  overall 415 long over the handles, 250 high over the lid knob, 5.22 kg (the published review figures);
  lever pivot 0.08 N m static friction (about 2.6 N at 31 mm from the pivot) standing in for the wire's
  over-centre pull; dial 0.06 N m static friction standing in for its switch detents; wire hinge damping 1e-4 N m s/rad;
  lever 20 g, wire 7 g, dial 25 g; frictions: feet 0.6/0.5, plastic 0.5/0.4, steel wire 0.4/0.3;
  hook lip 4 mm deep, wire 3 mm thick: the crossbar leaves the hook when the lever is about 80 degrees up.
"""
import math

import numpy as np

from yam_joints import Hinge, Link
from yam_mesh import Mesh, MeshMechanism
from yam_parts import box, cyl

MM = 0.001
STEEL_BODY = (0.74, 0.75, 0.77)
BLACK = (0.07, 0.07, 0.08)
STONE = (0.20, 0.13, 0.09)
RIM_GREY = (0.46, 0.47, 0.49)
GLASS = (0.66, 0.74, 0.78)
WIRE = (0.62, 0.63, 0.66)
WHITE = (0.95, 0.95, 0.93)

MASS = 5.22  # kg, the review's overall weight (stoneware 2.9 kg of it)
# body and lid (estimates within the published 415 x 300 x 250 mm)
BODY_RX, BODY_RY, BODY_Z0, BODY_Z1 = 0.1275, 0.160, 0.010, 0.1855
STONE_RX, STONE_RY, STONE_Z1 = 0.1235, 0.156, 0.1925
LIDRIM_RX, LIDRIM_RY, LIDRIM_Z1 = 0.121, 0.154, 0.199
DOME_RX, DOME_RY, DOME_TOP = 0.095, 0.122, 0.224
KNOB_TOP = 0.250
HANDLE_OUT = 0.2075  # handle's outer face: 415 mm over the handles

# latch (per side s = +1 left, -1 right; values for s = +1)
LATCH_BASE_Y = (0.124, 0.163)
LATCH_BASE_Z = (0.199, 0.208)
LATCH_BASE_HX = 0.006
LEVER_Y = (0.124, 0.172)
LEVER_Z = (0.2085, 0.2165)
LEVER_HX = 0.010
PIVOT = (0.169, 0.2125)  # lever pivot (y, z)
WIRE_TOP = (0.166, 0.2195)  # wire hinge on the lever, lever closed: 3 mm in from and 7 mm above the pivot
WIRE_PLAY = 0.0015  # the crossbar's play under the hook's nose, lever closed (stands for the gasket's give)
WIRE_R = 0.0015
WIRE_HX = 0.027  # legs 54 mm apart
HOOK_ROOF = (0.156, 0.1735, 0.181, 0.185)  # the hook's nose: y0, y1, z0, z1
HOOK_LIP = (0.170, 0.1735, 0.177, 0.185)  # the lip turned down at the nose's end, 4 mm below the nose
CROSS_Z = HOOK_ROOF[2] - WIRE_R - WIRE_PLAY  # 0.178: crossbar height, lever closed
WIRE_LEN = WIRE_TOP[1] - CROSS_Z  # 0.0415
HOOK_HX = 0.010
LEVER_STATIC, LEVER_DYNAMIC = 0.08, 0.06  # N m
LEVER_OPEN_DEG = 185.0
LEVER_MASS, WIRE_MASS = 0.020, 0.007

# dial
DIAL_Z = 0.055
DIAL_FACE_X = -BODY_RX  # the front wall at y = 0
KNOB_R, KNOB_X = 0.0225, (-0.1290, -0.1470)
BAR_X, BAR_HY, BAR_HZ = (-0.1470, -0.1590), 0.005, 0.020
DIAL_AXIS_X = 0.5 * (KNOB_X[0] + BAR_X[1])
DIAL_STATIC, DIAL_DYNAMIC = 0.06, 0.05  # N m
DIAL_MASS = 0.025
SETTINGS = {"OFF": 0.0, "LOW": 45.0, "HIGH": 90.0, "WARM": 135.0}  # degrees clockwise seen from the front
MARK_R = 0.032


def _ellipse(rx, ry, n=48):
    a = np.arange(n) * 2 * math.pi / n
    return np.column_stack((rx * np.cos(a), ry * np.sin(a)))


def _ellipse_proxies(rx, ry, z0, z1):
    """Boxes enclosing an oval slab (offline geometry only)."""
    out, cuts = [], (-1.0, -0.8, -0.45, 0.45, 0.8, 1.0)
    for lo, hi in zip(cuts, cuts[1:]):
        inner = min(abs(lo), abs(hi)) if lo * hi > 0 else 0.0
        hx = rx * math.sqrt(max(0.0, 1.0 - inner * inner))
        out.append(box((0.0, (lo + hi) / 2 * ry, (z0 + z1) / 2), (2 * hx, (hi - lo) * ry, z1 - z0)))
    return tuple(out)


def oval_slab(name, rx, ry, z0, z1, color):
    """An oval prism from z0 to z1 (convex collider)."""
    from yam_mesh import _extrude, _outline
    p, f = _extrude(_outline(_ellipse(rx, ry)), z1 - z0)
    p = p + np.array([0.0, 0.0, (z0 + z1) / 2])
    return Mesh(name, p, f, collision="convex", color=color, proxy=_ellipse_proxies(rx, ry, z0, z1))


def dome(name, rx, ry, z0, z1, color, rings=6, n=40):
    """The glass lid's dome: a half-ellipsoid cap over an oval of rx, ry at z0 rising to z1 (convex collider)."""
    pts = []
    for k in range(rings):
        t = k / rings
        s = math.cos(t * math.pi / 2)
        for x, y in _ellipse(rx * s, ry * s, n):
            pts.append((x, y, z0 + (z1 - z0) * math.sin(t * math.pi / 2)))
    pts.append((0.0, 0.0, z1))
    bottom = len(pts)
    pts.append((0.0, 0.0, z0))
    faces = []
    for k in range(rings - 1):
        for i in range(n):
            a, b = k * n + i, k * n + (i + 1) % n
            c, d = (k + 1) * n + (i + 1) % n, (k + 1) * n + i
            faces += [(a, b, c), (a, c, d)]
    top = (rings - 1) * n
    for i in range(n):
        faces.append((top + i, top + (i + 1) % n, bottom - 1))
        faces.append((i, bottom, (i + 1) % n))
    p = np.asarray(pts, float)
    f = np.asarray(faces, int)
    from yam_mesh import signed_volume
    if signed_volume((p, f)) < 0:
        f = f[:, ::-1]
    proxy = (box((0.0, 0.0, (z0 + z1) / 2), (2 * rx * 0.85, 2 * ry * 0.85, z1 - z0)),
             box((0.0, 0.0, z0 + (z1 - z0) * 0.25), (2 * rx, 2 * ry * 0.95, (z1 - z0) * 0.5)))
    return Mesh(name, p, f, collision="convex", color=color, proxy=proxy)


def bx(x0, x1, y0, y1, z0, z1, color=None):
    lo, hi = np.array([x0, y0, z0]), np.array([x1, y1, z1])
    return box(tuple((lo + hi) / 2), tuple(hi - lo), color)


def _side_shapes(s):
    """Handle, clip hook and latch base on side s (+1 left, -1 right)."""
    def ys(a, b):
        return (min(s * a, s * b), max(s * a, s * b))
    out = []
    for (y0, y1), (z0, z1), hx in ((ys(0.150, HANDLE_OUT - 0.001), (0.150, 0.158), 0.045),  # top arm
                                    (ys(0.196, HANDLE_OUT), (0.100, 0.158), 0.050),  # grip
                                    (ys(0.150, HANDLE_OUT - 0.001), (0.100, 0.108), 0.045)):  # bottom arm
        out.append(bx(-hx, hx, y0, y1, z0, z1, BLACK))
    y0, y1 = ys(HOOK_ROOF[0], HOOK_ROOF[1])
    out.append(bx(-HOOK_HX, HOOK_HX, y0, y1, HOOK_ROOF[2], HOOK_ROOF[3], BLACK))
    y0, y1 = ys(HOOK_LIP[0], HOOK_LIP[1])
    out.append(bx(-HOOK_HX, HOOK_HX, y0, y1, HOOK_LIP[2], HOOK_LIP[3], BLACK))
    y0, y1 = ys(*LATCH_BASE_Y)
    out.append(bx(-LATCH_BASE_HX, LATCH_BASE_HX, y0, y1, LATCH_BASE_Z[0], LATCH_BASE_Z[1], BLACK))
    return out


def base_shapes():
    s = [cyl((fx, fy, BODY_Z0 / 2), 0.012, BODY_Z0, "z", BLACK) for fx in (-0.075, 0.075) for fy in (-0.100, 0.100)]
    s += _side_shapes(1) + _side_shapes(-1)
    # lid knob: two posts and a loop bar along y
    for py in (-0.028, 0.028):
        s.append(bx(-0.008, 0.008, py - 0.005, py + 0.005, DOME_TOP - 0.003, KNOB_TOP - 0.008, BLACK))
    s.append(bx(-0.008, 0.008, -0.036, 0.036, KNOB_TOP - 0.009, KNOB_TOP, BLACK))
    # index marks round the dial (OFF, LOW, HIGH, WARM), white, 1 mm proud of the front wall
    for deg in SETTINGS.values():
        a = math.radians(deg)
        y, z = -MARK_R * math.sin(a), DIAL_Z + MARK_R * math.cos(a)
        xw = -BODY_RX * math.sqrt(max(0.0, 1 - (y / BODY_RY) ** 2))
        s.append(box((xw - 0.0005, y, z), (0.003, 0.004, 0.004), WHITE))
    return s


def base_meshes():
    return (oval_slab("body", BODY_RX, BODY_RY, BODY_Z0, BODY_Z1, STEEL_BODY),
            oval_slab("stoneware_rim", STONE_RX, STONE_RY, BODY_Z1, STONE_Z1, STONE),
            oval_slab("lid_rim", LIDRIM_RX, LIDRIM_RY, STONE_Z1, LIDRIM_Z1, RIM_GREY),
            dome("lid_glass", DOME_RX, DOME_RY, LIDRIM_Z1, DOME_TOP, GLASS))


def lever_link(s):
    y0, y1 = sorted((s * LEVER_Y[0], s * LEVER_Y[1]))
    shapes = (bx(-LEVER_HX, LEVER_HX, y0, y1, LEVER_Z[0], LEVER_Z[1], BLACK),)
    lo, hi = (-LEVER_OPEN_DEG, 0.0) if s > 0 else (0.0, LEVER_OPEN_DEG)
    joint = Hinge("x", (0.0, s * PIVOT[0], PIVOT[1]), lo, hi, damping=0.002, armature=1e-5,
                  static_friction=LEVER_STATIC, dynamic_friction=LEVER_DYNAMIC)
    return Link("lever_l" if s > 0 else "lever_r", shapes, joint, LEVER_MASS, color=BLACK, friction=(0.5, 0.4))


def wire_link(s):
    y, zt = s * WIRE_TOP[0], WIRE_TOP[1]
    shapes = [cyl((0.0, y, CROSS_Z), WIRE_R, 2 * WIRE_HX + 2 * WIRE_R, "x", WIRE)]
    for sx in (-1, 1):
        shapes.append(cyl((sx * WIRE_HX, y, (CROSS_Z + zt) / 2), WIRE_R, WIRE_LEN, "z", WIRE))
        shapes.append(cyl((sx * (WIRE_HX + LEVER_HX + 0.0025) / 2, y, zt), WIRE_R, WIRE_HX - LEVER_HX - 0.0025, "x",
                          WIRE))
    lo, hi = (-40.0, 220.0) if s > 0 else (-220.0, 40.0)
    joint = Hinge("x", (0.0, y, zt), lo, hi, damping=1e-4, armature=1e-5)
    return Link("wire_l" if s > 0 else "wire_r", tuple(shapes), joint, WIRE_MASS, parent="lever_l" if s > 0 else "lever_r",
                color=WIRE, friction=(0.4, 0.3))


def dial_link():
    shapes = (cyl(((KNOB_X[0] + KNOB_X[1]) / 2, 0.0, DIAL_Z), KNOB_R, KNOB_X[0] - KNOB_X[1], "x", BLACK),
              bx(BAR_X[1], BAR_X[0], -BAR_HY, BAR_HY, DIAL_Z - BAR_HZ, DIAL_Z + BAR_HZ, BLACK),
              bx(BAR_X[1] - 0.0005, BAR_X[1], -0.003, 0.003, DIAL_Z + 0.011, DIAL_Z + 0.018, WHITE))
    joint = Hinge("x", (DIAL_AXIS_X, 0.0, DIAL_Z), 0.0, 140.0, damping=0.003, armature=2e-5,
                  static_friction=DIAL_STATIC, dynamic_friction=DIAL_DYNAMIC)
    return Link("dial", shapes, joint, DIAL_MASS, color=BLACK, friction=(0.7, 0.6))


def cooker(name, pos, yaw=0.0):
    links = (lever_link(1), wire_link(1), lever_link(-1), wire_link(-1), dial_link())
    return MeshMechanism(name, pos, base_shapes(), links, mass=MASS, color=STEEL_BODY, yaw=yaw, friction=(0.6, 0.5),
                         contact_offset=0.001, self_collide=True, meshes={"base": base_meshes()})


# ---------------------------------------------------------------- reading the state (cooker frame)
SIDES = {"l": 1, "r": -1}


def lever_open_deg(state, side):
    """How far the lever has been flipped up and round (0 closed, 180 pointing out over the handle)."""
    s = SIDES[side]
    return -s * math.degrees(state.joints["lever_" + side])


def wire_cross(state, side):
    """The wire's crossbar centre in the cooker frame."""
    s = SIDES[side]
    p = state.link_to_world("wire_" + side, (0.0, s * WIRE_TOP[0], CROSS_Z))
    return np.asarray(state.to_local(np.asarray(p, float)), float)


def hooked(state, side):
    """The wire's crossbar is in the clip hook: under the hook's nose, inboard of the middle of the lip, and not
    below the lip's bottom edge (a crossbar hanging outside the lip or below it is out of the hook)."""
    s = SIDES[side]
    c = wire_cross(state, side)
    return bool(0.150 < s * c[1] < 0.5 * (HOOK_LIP[0] + HOOK_LIP[1]) and c[2] > HOOK_LIP[2] - WIRE_R
                and abs(c[0]) < WIRE_HX + 0.01)


def dial_deg(state):
    return math.degrees(state.joints["dial"])
