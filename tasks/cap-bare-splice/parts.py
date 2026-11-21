"""The shapes of every part, built from layout.py's numbers with mechkit's boxes and cylinders. Pure Python + NumPy.

Each builder returns shapes in an assembly's base frame: world positions minus the assembly's ``origin`` (every
assembly here has yaw 0, except the loose nuts, whose frame is the nut frame of capping.py).
"""

from __future__ import annotations

import math

import numpy as np

import layout as L
import mechkit as mk

MM = L.MM


# -- helpers -----------------------------------------------------------------------------------------------------------
def frame_x_to(u) -> np.ndarray:
    """A rotation whose x axis is the unit vector ``u`` (its y axis horizontal where it can be)."""
    u = L.unit(u)
    helper = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.95 else np.array([0.0, 1.0, 0.0])
    v = np.cross(helper, u)
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    return np.column_stack([u, v, w])


def _q(R) -> tuple:
    return tuple(float(v) for v in mk.matrix_to_quat(np.asarray(R, float)))


def rod(a, b, radius, color) -> mk.Shape:
    """A cylinder from point a to point b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return mk.cylinder(tuple((a + b) / 2), radius, float(np.linalg.norm(b - a)), axis="X", color=color,
                       quat=_q(frame_x_to(b - a)))


def moved(shape, R, t) -> mk.Shape:
    """``shape`` (in some frame) placed by rotation R and translation t."""
    R = np.asarray(R, float)
    c = tuple(np.asarray(t, float) + R @ np.asarray(shape.center, float))
    q = _q(R @ shape.rotation())
    if shape.kind == "box":
        return mk.box(c, shape.size, shape.color, quat=q)
    return mk.cylinder(c, shape.radius, shape.height, axis=shape.axis, color=shape.color, quat=q)


def shifted(shapes, origin):
    o = np.asarray(origin, float)
    return [moved(s, np.eye(3), -o) for s in shapes]


def yz_plate(center, size_yz, thickness, holes=(), color=None):
    """A flat plate standing in the y-z plane (thickness along x), with square holes (dy, dz, sy, sz)."""
    flat = mk.plate_with_holes((0.0, 0.0, 0.0), size_yz, holes, thickness, color)
    out = []
    for s in flat:  # (x, y, z) of the flat plate -> (y, z, x)
        cx, cy, _ = s.center
        sx, sy, sz = s.size
        out.append(mk.box((center[0], center[1] + cx, center[2] + cy), (sz, sx, sy), color))
    return out


# -- the backboard, the box, the cables (one fixed assembly, "jbox") ------------------------------------------------------
def jbox_static():
    """World shapes of everything fixed: backboard and foot, box, screws, cable sheaths, grounds."""
    s = []
    fx, t = L.BOARD_FACE_X, L.BOARD_T
    y0, y1 = L.BOARD_Y
    s.append(mk.box((fx + t / 2, (y0 + y1) / 2, L.TOP + L.BOARD_H / 2), (t, y1 - y0, L.BOARD_H), L.PLYWOOD))
    d, h = L.FOOT
    s.append(mk.box((fx + t + d / 2, (y0 + y1) / 2, L.TOP + h / 2), (d, y1 - y0, h), L.STUD_WOOD))
    # the box: back and four walls
    W, D, T = L.BOX_W, L.BOX_D, L.BOX_T
    y, z = L.BOX_Y, L.BOX_Z
    s.append(mk.box((fx - T / 2, y, z), (T, W, W), L.STEEL))
    xm = L.RIM_X + (D - T) / 2
    for sign in (1, -1):
        s.append(mk.box((xm, y, z + sign * (W / 2 - T / 2)), (D - T, W, T), L.STEEL))  # top, bottom
        s.append(mk.box((xm, y + sign * (W / 2 - T / 2), z), (D - T, T, W - 2 * T), L.STEEL))  # sides
    # the two screw ears (inside the corners, at the rim) and the cover screws standing out of them
    for sy, sz in ((1, 1), (-1, -1)):
        cy, cz = y + sy * L.SCREW_OFF, z + sz * L.SCREW_OFF
        ear_c = (L.RIM_X + 2.0 * MM, y + sy * (L.INNER_HALF - L.EAR / 2), z + sz * (L.INNER_HALF - L.EAR / 2))
        s.append(mk.box(ear_c, (4.0 * MM, L.EAR, L.EAR), L.STEEL))
        shank = L.RIM_X - L.COVER_GAP - L.COVER_T - 0.6 * MM  # the screw's head stands just proud of the cover
        s.append(rod((shank, cy, cz), (L.RIM_X + 3.0 * MM, cy, cz), L.SCREW_SHANK_R, L.SCREW_ZINC))
        s.append(rod((shank - L.SCREW_HEAD_T, cy, cz), (shank, cy, cz), L.SCREW_HEAD_R, L.SCREW_ZINC))
    # the green ground screw on the back, inside
    gx, gy, gz = L.GROUND_SCREW
    s.append(rod((gx - 2.5 * MM, gy, gz), (gx + 1.0 * MM, gy, gz), 3.2 * MM, L.GROUND_GREEN))
    # cable sheaths: along the board from above and below, and a short stub inside each knockout
    sx = fx - L.SHEATH_T / 2
    top_out, bot_out = z + W / 2, z - W / 2
    s.append(mk.box((sx, y, (top_out + L.TOP + L.BOARD_H) / 2), (L.SHEATH_T, L.SHEATH_W, L.TOP + L.BOARD_H - top_out),
                    L.SHEATH))
    s.append(mk.box((sx, y, (bot_out + L.TOP) / 2 + 0.005), (L.SHEATH_T, L.SHEATH_W, bot_out - L.TOP - 0.01), L.SHEATH))
    sx_in = L.BACK_IN_X - L.SHEATH_T / 2
    for sign in (1, -1):
        zc = z + sign * (L.INNER_HALF - L.SHEATH_IN / 2)
        s.append(mk.box((sx_in, y, zc), (L.SHEATH_T, L.SHEATH_W, L.SHEATH_IN), L.SHEATH))
    # bare grounds from each sheath stub to the green screw
    gxr = L.BACK_IN_X - 1.0 * MM
    for sign in (1, -1):
        start = (gxr, y, z + sign * (L.INNER_HALF - L.SHEATH_IN))
        bend = (gxr, y, gz)
        s.append(rod(start, bend, L.GROUND_R, L.COPPER))
    s.append(rod((gxr, y + L.GROUND_R, gz), (gxr, gy, gz), L.GROUND_R, L.COPPER))
    return s


def cover_shapes():
    """World shapes of the cover as drawn: hanging open, turned 180 deg about the pivot screw (up and left)."""
    p = np.asarray(L.PIVOT, float)
    centre_closed = np.array([L.COVER_X, L.BOX_Y, L.BOX_Z])
    centre = 2 * p - centre_closed  # the 180 deg turn about the pivot's axis (x): y and z mirrored through it
    centre[0] = L.COVER_X
    hole = L.COVER_SCREW_HOLE
    # slots round the two screws, in the cover's own y, z (the 180 deg turn maps the closed (+o, +o) to (-o, -o))
    o = L.SCREW_OFF
    holes = [(-o, -o, hole, hole), (o, o, hole, hole)]
    return yz_plate(tuple(centre), (L.COVER_W, L.COVER_W), L.COVER_T, holes, L.STEEL)


# -- splices: a pair of conductors twisted together, as a chain of hinged links --------------------------------------
def splice_parts(name: str):
    """(base shapes, links) for one splice, in world coordinates; links are [(anchor, axis, shapes)]."""
    ins = L.BLACK_INS if name == "black" else L.WHITE_INS
    root = np.asarray(L.SPLICE_ROOT[name], float)
    u = L.splice_dir(name)
    R = frame_x_to(u)
    up = R[:, 2]  # across the pair, near vertical
    # the base: each cable's conductor, from its sheath stub down (or up) to the root
    base = []
    for sign in (1, -1):
        start = np.array([root[0], root[1], L.BOX_Z + sign * (L.INNER_HALF - L.SHEATH_IN)])
        base.append(rod(start, root + np.array([0.0, 0.0, sign * L.PAIR_R]), L.PAIR_R, ins))
    links = []
    for i in range(L.LINKS):
        a = L.link_start(name, i)
        b = L.link_start(name, i + 1)
        shapes = []
        if i < L.TWISTED_FROM:
            for sign in (1, -1):
                off = sign * (L.PAIR_R + 0.1 * MM) * up
                shapes.append(rod(a + off, b + off, L.PAIR_R, ins))
        elif i < L.LINKS - 1:
            shapes.append(rod(a, b, L.TWIST_R, ins))
        else:
            mid = b - L.COPPER_LEN * u
            shapes.append(rod(a, mid, L.TWIST_R, ins))
            shapes.append(rod(mid, b, L.COPPER_R, L.COPPER))
            if name == "white":
                shapes += [moved(s, R, L.white_nut_mouth()) for s in nut_shapes()]
        axis = "Z" if i % 2 == 0 else "Y"
        links.append((a, axis, shapes))
    return base, links


# -- the winged wire nut (nut frame: mouth at the origin, axis +x) ----------------------------------------------------------
def _hex_tube(x0: float, x1: float, apothem: float, wall: float, color):
    out = []
    side = 2.0 * (apothem + wall) * math.tan(math.radians(30.0))
    for k in range(6):
        phi = math.radians(60.0 * k + 30.0)  # flats facing the nut frame's +-z (square to jaws closing along z)
        n = np.array([0.0, math.cos(phi), math.sin(phi)])
        c = np.array([(x0 + x1) / 2, 0.0, 0.0]) + n * (apothem + wall / 2)
        out.append(mk.box(tuple(c), (x1 - x0, side, wall), color, ypr=(0.0, 0.0, phi - math.pi / 2)))
    return out


def nut_shapes(color=L.NUT_YELLOW, upright: bool = False):
    """The nut in its nut frame; ``upright``: in an upright nut's body frame (L.NUT_UPRIGHT_R)."""
    if upright:
        return [moved(x, L.NUT_UPRIGHT_R, np.zeros(3)) for x in nut_shapes(color)]
    s = []
    s += _hex_tube(0.0, L.NUT_MOUTH_LEN, L.NUT_MOUTH_IN, L.NUT_WALL, color)  # the flared skirt
    s += _hex_tube(L.NUT_MOUTH_LEN, L.NUT_BORE, L.NUT_IN, L.NUT_WALL, color)
    # the step between the skirt and the bore: a ring of six thin boxes
    s += _hex_tube(L.NUT_MOUTH_LEN - 0.6 * MM, L.NUT_MOUTH_LEN, L.NUT_IN, L.NUT_MOUTH_IN - L.NUT_IN, color)
    r, h = L.NUT_CAP
    s.append(mk.cylinder((L.NUT_BORE + h / 2, 0.0, 0.0), r, h, axis="X", color=color))
    wl, wr, wt = L.NUT_WING
    rin = L.NUT_IN + L.NUT_WALL
    for sign in (1, -1):
        s.append(mk.box((L.NUT_WING_X, sign * (rin + wr / 2), 0.0), (wl, wr, wt), color))
    return s
