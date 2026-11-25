"""The scene's parts as bodykit bodies: the silicone mat, the hearing aid and its battery door (a hinge), the old and new
size 13 zinc-air cells, the new cell's pull tab (a bond that breaks under a pull), the dial pack, the battery magnet in
its holder and the used-battery cup. Lengths in metres; the comments give millimetres.

The aid lies on its side on the mat, its long axis along the world's y, the battery door at its right (-y) end. Its own
frame, in which its parts are written: x across its depth (13 mm), y along its length (36 mm, door at -y), z through its
thickness (11 mm), origin at the middle of its underside; `Q` turns that frame into the world's (here the identity).
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena.task_file import TABLE_TOP

import bodykit as bk
from bodykit import Body, BoxShape, CylShape, MeshShape

MM = 0.001
TOP = TABLE_TOP["station"]
MAT_T = 3 * MM
Z0 = TOP + MAT_T  # the mat's top

# -- layout (world) ------------------------------------------------------------------------------------------------
MAT_C, MAT_SIZE = (0.29, -0.04), (0.26, 0.30)
AID_XY = (0.36, 0.04)
PACK_XY = (0.25, 0.0)
CUP_XY = (0.26, -0.08)
HOLDER_XY = (0.20, -0.16)
Q = np.eye(3)  # aid frame -> world

# -- hearing aid ---------------------------------------------------------------------------------------------------
AID_D, AID_L, AID_T = 13 * MM, 36 * MM, 11 * MM
CRADLE_C = np.array([0.0, -11.0 * MM])  # the cradle's axis, aid frame, door shut (0.9 mm clear of the end plate)
CRADLE_R = 4.8 * MM  # inner radius of the cradle's ring (9.6 mm across: about 0.85 mm of play round the cell)
RING_T = 0.6 * MM
RING_Z = (3.2 * MM, 6.2 * MM)  # the cradle's wall: the cell stands 1.7 mm proud of it
LIP_R = 3.2 * MM  # the lip ring under the cell: a 6.4 mm hole
LIP_Z = (2.6 * MM, 3.2 * MM)
SHELL_TOP = 8.3 * MM  # underside of the aid's shell over the door's cavity
PIVOT = np.array([6.5 * MM, -18 * MM])  # the door's pin, aid frame
DOOR_OPEN_MAX = 95.0  # degrees

# -- cells (IEC PR48, size 13) -------------------------------------------------------------------------------------
CELL_R = 3.95 * MM
CELL_BODY_H = 4.7 * MM
CAP_R, CAP_H = 2.8 * MM, 0.7 * MM
CELL_MASS = 0.003  # a size 13 cell is 0.8 g; rounded up for the solver (fidelity note)
SEAT_Z = LIP_Z[1] + CELL_BODY_H / 2  # a seated cell's origin (middle of its can), aid frame

# -- dial pack -----------------------------------------------------------------------------------------------------
PACK_R, PACK_BASE, PACK_H = 24 * MM, 3 * MM, 5 * MM
POCKET = 9.4 * MM  # square pocket's inner width (collider); drawn round
POCKET_REL = np.array([0.0, 0.0])  # the open pocket at the pack's centre line, nearest the robot's view
# -- tab: an orange film disc on the + face and a 6 x 14 mm tail folded up at the cell's +y edge
TAB_W, TAB_TAIL = 6 * MM, 14 * MM
TAB_BREAK_N, TAB_BREAK_NM = 0.4, 0.003

# -- battery magnet ------------------------------------------------------------------------------------------------
MAG_R, MAG_H = 2.5 * MM, 12 * MM  # the magnet at the end of a slim 5 mm neck
SHAFT_R, SHAFT_H = 4 * MM, 69 * MM
STICK_L = MAG_H + SHAFT_H  # tip face to the shaft's top (the brush stands 12 mm above)
HOLDER = (40 * MM, 40 * MM, 25 * MM)
HOLDER_HOLE, HOLDER_FLOOR = 9 * MM, 5 * MM

# -- cup: 30 mL medicine cup -------------------------------------------------------------------------------------
CUP_R, CUP_H = 20 * MM, 30 * MM

BEIGE = (0.80, 0.70, 0.60)
DOOR_C = (0.74, 0.64, 0.55)
STEEL = (0.78, 0.79, 0.80)
DARK = (0.18, 0.18, 0.19)
ORANGE = (0.95, 0.45, 0.08)
WHITE = (0.93, 0.93, 0.92)


def ring(r_in: float, t: float, z0: float, z1: float, color, n: int = 12, centre=(0.0, 0.0), collide: bool = True) -> list:
    """A ring of n boxes, inner radius r_in, wall t, from z0 to z1, around `centre` (x, y)."""
    out = []
    r = r_in + t / 2
    w = 2 * (r_in + t) * math.tan(math.pi / n) + 0.0002
    for k in range(n):
        a = 2 * math.pi * k / n
        out.append(BoxShape((centre[0] + r * math.cos(a), centre[1] + r * math.sin(a), (z0 + z1) / 2), (t, w, z1 - z0), color,
                            yaw=a, collide=collide))
    return out


def q3(p) -> np.ndarray:
    p = np.asarray(p, float)
    return Q @ (p if p.size == 3 else np.r_[p, 0.0])


def aid_world(p_aid) -> np.ndarray:
    """A point in the aid's start frame, in the world."""
    return np.array([AID_XY[0], AID_XY[1], Z0]) + q3(p_aid)


def turn(shapes) -> list:
    """Shapes written in the aid's frame, turned into the world's axes (Q), about the body's origin."""
    out = []
    ang = math.atan2(Q[1, 0], Q[0, 0])  # Q's turn about z
    for sh in shapes:
        c = tuple(q3(sh.centre).tolist())
        if isinstance(sh, MeshShape):
            pts = tuple(tuple((Q @ np.asarray(p, float)).tolist()) for p in sh.points)
            from dataclasses import replace

            out.append(replace(sh, centre=c, points=pts))
        elif isinstance(sh, BoxShape) and sh.quat_xyzw is None:
            out.append(BoxShape(c, sh.size, sh.color, yaw=sh.yaw + ang, collide=sh.collide, friction=sh.friction))
        elif isinstance(sh, BoxShape):
            qz = (0.0, 0.0, math.sin(ang / 2), math.cos(ang / 2))
            out.append(BoxShape(c, sh.size, sh.color, collide=sh.collide, quat_xyzw=_qmul(qz, sh.quat_xyzw), friction=sh.friction))
        else:
            from dataclasses import replace

            out.append(replace(sh, centre=c))
    return out


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def mat() -> Body:
    return Body.build("mat", (MAT_C[0], MAT_C[1], TOP), 1.0, [BoxShape((0, 0, MAT_T / 2), (MAT_SIZE[0], MAT_SIZE[1], MAT_T), (0.42, 0.44, 0.47))],
                      kinematic=True, friction=(1.0, 0.9))


def rounded_slab(x0, x1, y0, y1, z0, z1, r_lo, r_hi, bevel, color, n=6) -> MeshShape:
    """A drawn slab with rounded corners (radius r_lo at the y0 end, r_hi at the y1 end) and bevelled top and bottom edges."""
    def outline(inset):
        pts = []
        for cx, cy, r, a0 in ((x1 - r_hi, y1 - r_hi, r_hi, 0.0), (x0 + r_hi, y1 - r_hi, r_hi, 90.0), (x0 + r_lo, y0 + r_lo, r_lo, 180.0),
                              (x1 - r_lo, y0 + r_lo, r_lo, 270.0)):
            rr = max(r - inset, 1e-5)
            for k in range(n + 1):
                a = math.radians(a0 + 90.0 * k / n)
                pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
        return pts

    layers = [(z0, bevel), (z0 + bevel, 0.0), (z1 - bevel, 0.0), (z1, bevel)]
    rings = [[(x, y, z) for x, y in outline(ins)] for z, ins in layers]
    m = len(rings[0])
    pts = [p for ring in rings for p in ring]
    faces = []
    for L in range(3):
        for k in range(m):
            a, b = L * m + k, L * m + (k + 1) % m
            faces += [(a, b, b + m), (a, b + m, a + m)]
    cb = len(pts)
    pts.append(((x0 + x1) / 2, (y0 + y1) / 2, z0))
    ct = len(pts)
    pts.append(((x0 + x1) / 2, (y0 + y1) / 2, z1))
    for k in range(m):
        faces.append((cb, (k + 1) % m, k))
        faces.append((ct, 3 * m + k, 3 * m + (k + 1) % m))
    return MeshShape(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), tuple(pts), tuple(faces), color, smooth=False, roughness=0.45)


def aid() -> Body:
    d, t = AID_D, AID_T
    y_cav0, y_cav1 = -17 * MM, -5.6 * MM
    cav_c, cav_l = (y_cav0 + y_cav1) / 2, y_cav1 - y_cav0
    y_end = AID_L / 2
    shapes = [
        BoxShape((0, (y_cav1 + y_end) / 2, t / 2), (d, y_end - y_cav1, t), None),  # collider; drawn as the rounded shell below
        rounded_slab(-d / 2, d / 2, y_cav1, y_end, 0.0, t, 0.3 * MM, 5.5 * MM, 1.2 * MM, BEIGE),
        BoxShape((0, cav_c, (SHELL_TOP + t) / 2), (d, cav_l, t - SHELL_TOP), BEIGE),
        BoxShape((0, cav_c, 1.0 * MM), (d, cav_l, 2.0 * MM), BEIGE),
        # the program button and the microphone ports on the upper side (drawn only)
        CylShape((-2.5 * MM, 9 * MM, t + 0.2 * MM), 1.6 * MM, 0.5 * MM, (0.45, 0.42, 0.40), collide=False),
        CylShape((2.5 * MM, 12 * MM, t + 0.05 * MM), 0.5 * MM, 0.15 * MM, DARK, collide=False),
        CylShape((2.5 * MM, 15 * MM, t + 0.05 * MM), 0.5 * MM, 0.15 * MM, DARK, collide=False),
    ]
    hook = [(0, y_end, t / 2), (0, y_end + 4 * MM, t / 2), (2 * MM, y_end + 9 * MM, t / 2), (6 * MM, y_end + 12 * MM, t / 2),
            (11 * MM, y_end + 12 * MM, t / 2), (14 * MM, y_end + 8 * MM, t / 2)]
    for a, b in zip(hook[:-1], hook[1:]):  # the ear hook, a brown tube curling round at the top end (drawn only)
        shapes.append(bk.segment(np.array(a), np.array(b), 3.5 * MM, (0.62, 0.48, 0.36), collide=False))
    return Body.build("aid", aid_world((0, 0, 0)), 0.006, turn(shapes), friction=(0.9, 0.8), contact_offset=0.001, solver_iterations=32)


def door(start_deg: float = 0.0) -> Body:
    """The battery door: its origin at the pin (aid frame PIVOT), turning about +z to open (the cradle swings out past the
    aid's -y end)."""
    c = CRADLE_C - PIVOT  # the cradle's axis from the pin
    shapes = ring(CRADLE_R, RING_T, *RING_Z, DOOR_C, centre=c)
    shapes += ring(LIP_R, CRADLE_R + RING_T - LIP_R, *LIP_Z, DOOR_C, centre=c)
    shapes += [
        BoxShape((-AID_D / 2, 0.5 * MM, 5.65 * MM), (AID_D, 1.0 * MM, 10.7 * MM), DOOR_C),  # the outer end plate
        BoxShape((-10.5 * MM, 0.75 * MM, 12.75 * MM), (5 * MM, 1.5 * MM, 3.5 * MM), DOOR_C),  # the door's pull tab at the free corner
        # the + mark on the end plate's top
        BoxShape((-4 * MM, 0.5 * MM, 11.02 * MM), (2.4 * MM, 0.5 * MM, 0.05 * MM), WHITE, collide=False),
        BoxShape((-4 * MM, 0.5 * MM, 11.02 * MM), (0.5 * MM, 0.9 * MM, 0.05 * MM), WHITE, collide=False),
    ]
    a = math.radians(start_deg) / 2
    return Body.build("door", aid_world((*PIVOT, 0.0)), 0.002, turn(shapes), friction=(0.6, 0.5), contact_offset=0.001, solver_iterations=32,
                      start_xyzw=(0.0, 0.0, math.sin(a), math.cos(a)) if start_deg else None)


CRADLE_REL = q3(np.r_[CRADLE_C - PIVOT, SEAT_Z])  # a seated cell's origin in the door's frame (world axes at the start)


def door_hinge():
    return bk.hinge("door_pin", "aid", "door", tuple(aid_world((*PIVOT, 0.0))), axis="Z", limits_deg=(0.0, DOOR_OPEN_MAX),
                    spring=0.0026, damping=0.0001, target_deg=0.0)


def cell(name: str, pos, flip: bool = False) -> Body:
    """A size 13 zinc-air cell, its origin at the middle of its can; + face (flat, one air hole) at +z, the - cap below.
    `flip` starts it turned over (+ face down)."""
    s = 1.0
    shapes = [
        CylShape((0, 0, 0), CELL_R, CELL_BODY_H, STEEL),
        CylShape((0, 0, -s * (CELL_BODY_H / 2 + CAP_H / 2)), CAP_R, CAP_H, (0.62, 0.63, 0.65)),
        CylShape((0, 0, s * (CELL_BODY_H / 2 + 0.01 * MM)), 0.35 * MM, 0.02 * MM, DARK, collide=False),  # the air hole
        CylShape((0, 0, s * (CELL_BODY_H / 2 - 0.5 * MM)), CELL_R + 0.02 * MM, 0.6 * MM, (0.66, 0.67, 0.69), collide=False),  # the crimp
    ]
    return Body.build(name, pos, CELL_MASS, shapes, friction=(0.4, 0.3), contact_offset=0.001, solver_iterations=32, damping=(0.0, 2.0),
                      start_xyzw=(1.0, 0.0, 0.0, 0.0) if flip else None)


def tab(name: str, cell_pos) -> Body:
    """The new cell's pull tab: an orange film disc on its + face, its 6 x 14 mm tail folded straight up at the cell's +y
    edge. Origin at the disc's centre. Collider: the disc as a 5 x 5 x 0.3 mm plate, the tail 0.8 mm thick."""
    top = np.asarray(cell_pos, float) + np.array([0, 0, CELL_BODY_H / 2 + 0.15 * MM])
    shapes = [
        CylShape((0, 0, 0), 3.4 * MM, 0.15 * MM, ORANGE, collide=False),
        BoxShape((0, 0, 0), (5 * MM, 5 * MM, 0.3 * MM), None),
        BoxShape((0, CELL_R + 0.4 * MM, TAB_TAIL / 2), (TAB_W, 0.8 * MM, TAB_TAIL), ORANGE),
        BoxShape((0, CELL_R - 0.4 * MM, 0.4 * MM), (TAB_W, 1.2 * MM, 0.8 * MM), ORANGE),
    ]
    return Body.build(name, tuple(top), 0.0005, shapes, friction=(0.9, 0.8), contact_offset=0.001, solver_iterations=32)


def tab_bond(cell_name: str, tab_name: str, at):
    return bk.bond("tab_bond", cell_name, tab_name, tuple(at), break_force=TAB_BREAK_N, break_torque=TAB_BREAK_NM)


def pack() -> Body:
    """The cells' dial pack lying flat, turned to its open pocket (the robot's cell); the other cells are behind its card."""
    px, py = POCKET_REL
    w, s = POCKET, 30 * MM
    hz = (PACK_BASE + PACK_H) / 2
    h = PACK_H - PACK_BASE
    shapes = [
        CylShape((0, 0, PACK_BASE / 2), PACK_R, PACK_BASE, (0.85, 0.87, 0.90)),
        # the pocket's walls (collider), and the dial's top drawn round it
        BoxShape((px, py + (w + s) / 2, hz), (s + w, s, h), None),
        BoxShape((px, py - (w + s) / 2, hz), (s + w, s, h), None),
        BoxShape((px + (w + s) / 2, py, hz), (s, w, h), None),
        BoxShape((px - (w + s) / 2, py, hz), (s, w, h), None),
        CylShape((0, 0, hz), PACK_R, h, (0.20, 0.33, 0.62), collide=False),
        CylShape((px, py, PACK_H + 0.05 * MM), 4.3 * MM, 0.12 * MM, (0.05, 0.05, 0.06), collide=False),
    ]
    for k in range(1, 8):  # the other cells' places under the card, marked round the dial
        a = 2 * math.pi * k / 8 - math.pi / 2
        shapes.append(CylShape((14 * MM * math.cos(a), 14 * MM * math.sin(a) + 0, PACK_H + 0.05 * MM), 3.0 * MM, 0.1 * MM, ORANGE, collide=False))
    return Body.build("pack", (*PACK_XY, Z0), 0.02, shapes, kinematic=True, friction=(0.6, 0.5))


def pocket_world() -> np.ndarray:
    return np.array([PACK_XY[0] + POCKET_REL[0], PACK_XY[1] + POCKET_REL[1], Z0 + PACK_BASE + CAP_H + CELL_BODY_H / 2])


def holder() -> Body:
    hx, hy, hz = HOLDER
    g = HOLDER_HOLE
    wall = (hx - g) / 2
    zc, zh = HOLDER_FLOOR + (hz - HOLDER_FLOOR) / 2, hz - HOLDER_FLOOR
    color = (0.30, 0.32, 0.36)
    shapes = [
        BoxShape((0, 0, HOLDER_FLOOR / 2), (hx, hy, HOLDER_FLOOR), color),
        BoxShape(((g + wall) / 2, 0, zc), (wall, hy, zh), color),
        BoxShape((-(g + wall) / 2, 0, zc), (wall, hy, zh), color),
        BoxShape((0, (g + wall) / 2, zc), (g, wall, zh), color),
        BoxShape((0, -(g + wall) / 2, zc), (g, wall, zh), color),
    ]
    return Body.build("holder", (*HOLDER_XY, Z0), 0.1, shapes, kinematic=True, friction=(0.5, 0.4))


def magnet() -> Body:
    """The battery magnet: origin at the magnet's face (its tip), shaft up +z, a brush at the top. Its 8 mm grip is
    flat-sided (square), as on many battery magnets, so a gripper holds it without it swinging."""
    shapes = [
        CylShape((0, 0, MAG_H / 2), MAG_R, MAG_H, STEEL, friction=(0.5, 0.45)),
        BoxShape((0, 0, MAG_H + SHAFT_H / 2), (2 * SHAFT_R, 2 * SHAFT_R, SHAFT_H), (0.10, 0.25, 0.55)),  # a flat-sided pen grip
        CylShape((0, 0, STICK_L + 6 * MM), 2.5 * MM, 12 * MM, DARK, collide=False),
    ]
    return Body.build("magnet", (*HOLDER_XY, Z0 + HOLDER_FLOOR + 0.0002), 0.008, shapes, friction=(1.0, 0.9), contact_offset=0.001,
                      solver_iterations=32)


def cup() -> Body:
    shapes = [CylShape((0, 0, 0.5 * MM), CUP_R, 1.0 * MM, WHITE)]
    shapes += ring(CUP_R, 1.0 * MM, 1.0 * MM, CUP_H, WHITE, n=16)
    return Body.build("cup", (*CUP_XY, Z0), 0.004, shapes, kinematic=True, friction=(0.5, 0.4))


GENTLE = ("old_cell", "new_cell", "tab")  # small parts: a stiff arm pressing one into a wall must not fling it


def _gentle(stage, prim_path: str, spec) -> None:
    if spec.name in GENTLE:
        from pxr import PhysxSchema

        prim = stage.GetPrimAtPath(f"{prim_path}/body")
        PhysxSchema.PhysxRigidBodyAPI(prim).GetMaxDepenetrationVelocityAttr().Set(0.05)


if _gentle not in bk.EXTRA_AUTHORS:
    bk.EXTRA_AUTHORS.append(_gentle)


def bodies(flip: bool = False, door_open: float = 0.0) -> list:
    """`door_open` (degrees): development only, the door starting open with the old cell in its cradle."""
    seat = aid_world((*CRADLE_C, SEAT_Z))
    if door_open:
        a = math.radians(door_open)
        rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
        piv = aid_world((*PIVOT, 0.0))
        seat = piv + rz @ (seat - piv)
    new = pocket_world() - (np.array([0, 0, CAP_H]) if flip else 0.0)
    objs = [mat(), aid(), door(door_open), cell("old_cell", tuple(seat)), pack(), cell("new_cell", tuple(new), flip=flip)]
    door_hinge()
    if not flip:
        objs.append(tab("tab", new))
        tab_bond("new_cell", "tab", objs[-1].pos)
    else:  # a scorer check: the tab already off, lying in the cup
        objs.append(tab("tab", (CUP_XY[0], CUP_XY[1], Z0 + 1.0 * MM - CELL_BODY_H / 2)))
    objs += [cup(), holder(), magnet()]
    return objs
