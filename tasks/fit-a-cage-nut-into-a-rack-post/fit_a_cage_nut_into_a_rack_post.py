"""On the station table: a 9U network cabinet (DIGITUS DN-19 09U-6/6: 600 mm wide, 600 mm deep, 509 mm tall, sheet steel
in light grey RAL 7035, two pairs of galvanized 19-inch profile rails with square holes for cage nuts), standing on the
bench with its glass front door taken off and its open front facing the robot, as a technician faces a rack. Its right
front rail (as seen from the front) is the one worked on: its mounting flange faces the robot, with real 9.53 mm square
holes; one hole is marked with a red paint-pen square. The cabinet's side panel stands just outside that rail and its
top, bottom and back panels close it, so the back (inside) face of the flange can only be reached through the cabinet's
front opening, round the flange's inner edge; the opposite front rail stands 465 mm away (hole centres) and the two rear
rails 400 mm behind. A 1U cantilever shelf (StarTech CABSHELFV1U) is screwed into U1, and an M6 cage nut lies face down
on it. A small flat-blade screwdriver lies on the bench to the right of the cabinet.

Task (instruction): fit the cage nut into the marked hole on the cabinet's front rail. The instruction does not say how.
The procedure the task checks is the rack makers': from inside the rack, lips facing out, "Hook one lip of the cage nut
in the square rail hole" (Oracle, SPARC M8/M7 Servers Installation Guide, "Insert Cage Nuts Into Rail Holes"), then,
with a flat-blade screwdriver, "Hold the cage nut in one hand, and compress the cage-nut clip with a flat-blade
screwdriver. With the clip compressed, push the edge of the cage nut fully into the hole from the inside of the
rack-mounting flange. Release the screwdriver pressure on the clip to lock the cage nut into place." (IBM, "Installing
the cage nuts in a 7014-S25 or 7014-S11 rack"). Turned home with nothing on its far clip, the clip's end swings out past
the hole's edge and catches behind the flange: the nut jams half in and the hooked clip is levered back out.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- cabinet: fixed. Front edge at x 0.180, back at 0.780; y -0.0675 (right side panel) to 0.5325; 0 to 509 mm above the
  table. Worked rail (front right): mounting flange x 0.205 (its front face, toward the robot) to 0.207, y -0.0175 to
  0.0075 (its inner edge, the 450 mm opening), web 30 mm deep along its -y edge, 40 to 465 mm above the table. Holes:
  9.53 mm square, three per U (U = 44.45 mm, U1 starting 54.5 mm above the table; holes 6.35, 22.225 and 38.1 mm above
  each U line), centred on y 0; U lines and numbers 1-9 on its front face. The marked hole is U3's middle hole, its
  middle 165.6 mm above the table. Opposite front rail: holes centred on y 0.465. Rear rails: back faces at x 0.605.
- shelf: fixed, in U1: 1U front plate on the front rails, floor 435 mm wide (y 0.015 to 0.450) and 250 mm deep (x 0.205
  to 0.455), its top 56.0 mm above the table, sides 43.2 mm tall.
- cage nut (yellow-zinc plated), loose, 5.5 g, face down on the shelf, lips toward +-y, its face centre at (0.280,
  0.144): an M6 square nut (10 x 10 x 4.8 mm, 5 mm bore) in a spring-steel cage (10.1 x 11.2 x 6.0 mm) with two lips
  on opposite sides. Each lip is a 5 mm wide strip folded over (inner leg 4.75 mm long, outer leg flaring out to
  13.72 mm across both lips, its end 2.3 mm in front of the cage's face) on a sprung hinge at its root (0.02 N m/rad,
  from 50 deg in to 10 deg out): pressed in it passes the 9.53 mm hole, let go it springs out over the flange's front
  face.
- screwdriver, loose, 40 g, lying along x with its tip at (0.190, -0.135), pointing away from the robot: tip 0.6 x
  3.5 mm, 60 mm shank, fluted handle 25 mm across.

Score: 0 for the rest of the run once the nut has fallen (once lifted, back down on the shelf, the cabinet's floor or
the bench with no gripper near), a lip has sprung out at another hole, or a lip has been forced 30 deg open (its clip
bent). Otherwise 0.5 once the nut sits in the marked hole by itself with at least one lip sprung out over the front
face, both grippers' centres 3 cm or more from it and the screwdriver out of the hole. Full score: seated (both lips
sprung out over the front face around the marked hole, the nut flat against the flange's back face and centred in the
hole), released like that, still for 1 s.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, allow_matrix_grips, as_numpy, on_reset, on_step  # noqa: E402
from bodykit import _quat_from_matrix, plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The file keeps the best stage reached itself and a broken rule latches 0, so the score is read at the end of the run.
SCORE_OVER = "end"

# -- the worked rail (world frame, m; heights above the table) --------------------------------------------------------
X0 = 0.205  # front face of the rail's mounting flange (toward the robot)
FL_T = 0.002  # flange thickness (steel)
XB = X0 + FL_T  # its back face, inside the cabinet: the cage nut sits against it
HOLE = 0.00953  # EIA-310 square hole (0.375 in)
HH = HOLE / 2
HY = 0.0  # the rail's hole column
IN_EDGE = 0.0075  # flange edge on the opening side, from the hole centre (EIA-310: 450 mm opening, 465 mm hole centres)
OUT_EDGE = 0.0175  # flange edge on the outer side, where the L-rail's web runs back into the cabinet
WEB_D = 0.030  # how far the web runs back from the flange's front face
U = 0.04445
N_U = 9
CAB_H = 0.509  # the cabinet's height
Z_U0 = (CAB_H - N_U * U) / 2  # bottom of U1 above the table (52.5 mm)
Z_RAIL0, Z_RAIL1 = 0.040, 0.465  # the rail's bottom and top
HOLE_Z = [Z_U0 + n * U + d for n in range(N_U) for d in (0.00635, 0.022225, 0.0381)]
MARK_U, MARK_K = 3, 1  # the marked hole: U3, middle hole
MARK = (MARK_U - 1) * 3 + MARK_K
HZ = HOLE_Z[MARK]
# -- the cabinet (DIGITUS DN-19 09U-6/6, front door off) ---------------------------------------------------------------
HOLE_PITCH_Y = 0.465  # EIA-310: hole centres across the rack
CAB_W, CAB_D = 0.600, 0.600
SETBACK = 0.025  # the front rails' front face behind the cabinet's front edge
XF = X0 - SETBACK  # the cabinet's front edge
Y_C = HY + HOLE_PITCH_Y / 2  # the cabinet's middle (y)
Y_R, Y_L = Y_C - CAB_W / 2, Y_C + CAB_W / 2  # its right and left sides
MOUNT_D = 0.400  # front rails' front face to rear rails' back face (equipment mounting depth 320-450 mm)
SHEET_T = 0.0015  # side, top, bottom and back panels (sheet steel)
RETURN_W, LIP_H = 0.020, 0.035  # the side panels' front returns; the top and bottom panels' front lips
SHELF_W, SHELF_D, SHELF_H, SHELF_T = 0.435, 0.250, 0.0432, 0.0015  # the 1U shelf: inside width, depth, height, steel
SHELF_TOP = Z_U0 + SHELF_T  # its floor's top face above the table
STEEL, DARKSTEEL, PANEL, SHELF_C, PAINT, RED, BLACK = (0.66, 0.68, 0.70), (0.30, 0.31, 0.32), (0.80, 0.81, 0.79), (
    0.10, 0.10, 0.11), (0.06, 0.06, 0.07), (0.85, 0.08, 0.06), (0.04, 0.04, 0.05)  # galvanized rails, RAL 7035 panels
RAIL_O = (X0, HY, TOP)


def _box(x0, x1, y0, y1, z0, z1, color, origin, collide=True) -> BoxShape:
    """A box from world corners (x, y; heights above the table) in the frame of a body whose origin is ``origin``."""
    return BoxShape(((x0 + x1) / 2 - origin[0], (y0 + y1) / 2 - origin[1], (z0 + z1) / 2 + TOP - origin[2]),
                    (x1 - x0, y1 - y0, z1 - z0), color, collide=collide)


_SEG = {"1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgecd", "7": "abc", "8": "abcdefg",
        "9": "abcdfg"}


def _digit(ch, x, yc, zc, o, h=0.0045, w=0.0028, t=0.0006) -> list:
    """A 7-segment digit painted on the flange's front face at x (visual), read from the front (from -x): segments a
    (top) .. g (middle)."""
    out = []
    for s in _SEG[ch]:
        if s in "adg":
            z = {"a": zc + h / 2, "g": zc, "d": zc - h / 2}[s]
            out.append(_box(x - 0.00015, x + 0.00015, yc - w / 2, yc + w / 2, z - t / 2, z + t / 2, PAINT, o, False))
        else:
            right = s in "bc"  # seen from the front (looking +x), the viewer's right is -y
            y = yc - w / 2 if right else yc + w / 2
            z0, z1 = (zc, zc + h / 2) if s in "bf" else (zc - h / 2, zc)
            out.append(_box(x - 0.00015, x + 0.00015, y - t / 2, y + t / 2, z0, z1, PAINT, o, False))
    return out


def _rail_boxes(hy: float, x_front: float, facing: float, inner: float, holes: bool) -> list:
    """An L-shaped 19-inch rail as world boxes (x0, x1, y0, y1, z0, z1, colour, collide): its mounting flange at
    ``x_front`` (the face equipment mounts on; ``facing`` -1: faces -x, +1: faces +x), its hole column at ``hy``, its
    inner (opening) edge on the ``inner`` side (+1: +y), its web running into the cabinet along the outer edge. With
    ``holes`` the holes are cut through; otherwise they are painted on (out of reach)."""
    xa, xb = sorted((x_front, x_front - facing * FL_T))
    y_in, y_out = hy + inner * IN_EDGE, hy - inner * OUT_EDGE
    out = []
    lo, hi = sorted((y_out, hy - inner * HH))
    out.append((xa, xb, lo, hi, Z_RAIL0, Z_RAIL1, STEEL, True))
    lo, hi = sorted((hy + inner * HH, y_in))
    out.append((xa, xb, lo, hi, Z_RAIL0, Z_RAIL1, STEEL, True))
    if holes:
        edges = [Z_RAIL0] + [v for z in HOLE_Z for v in (z - HH, z + HH)] + [Z_RAIL1]
        for z0, z1 in zip(edges[0::2], edges[1::2]):
            out.append((xa, xb, hy - HH, hy + HH, z0, z1, STEEL, True))
    else:
        out.append((xa, xb, hy - HH, hy + HH, Z_RAIL0, Z_RAIL1, STEEL, True))
        xp = x_front + facing * 0.0002
        for z in HOLE_Z:
            out.append((xp - 0.0001, xp + 0.0001, hy - HH, hy + HH, z - HH, z + HH, BLACK, False))
    wx = sorted((x_front, x_front - facing * WEB_D))
    wy = sorted((y_out, y_out - inner * FL_T))
    out.append((wx[0], wx[1], wy[0], wy[1], Z_RAIL0, Z_RAIL1, STEEL, True))
    return out


def _cabinet_boxes() -> list:
    """The cabinet as world boxes: four L-rails, side, top, bottom and back panels with the front returns and lips."""
    xb = XF + CAB_D
    out = _rail_boxes(HY, X0, -1.0, +1.0, True)
    out += _rail_boxes(HY + HOLE_PITCH_Y, X0, -1.0, -1.0, False)
    xr = X0 + MOUNT_D  # the rear rails' back face (equipment's rear brackets mount on it)
    out += _rail_boxes(HY, xr, +1.0, +1.0, False)
    out += _rail_boxes(HY + HOLE_PITCH_Y, xr, +1.0, -1.0, False)
    for y0, y1, r0, r1 in ((Y_R, Y_R + SHEET_T, Y_R + SHEET_T, Y_R + SHEET_T + RETURN_W),
                           (Y_L - SHEET_T, Y_L, Y_L - SHEET_T - RETURN_W, Y_L - SHEET_T)):
        out.append((XF, xb, y0, y1, 0.0, CAB_H, PANEL, True))  # side panel
        out.append((XF, XF + SHEET_T, r0, r1, 0.0, CAB_H, PANEL, True))  # its front return
    out.append((XF, xb, Y_R, Y_L, 0.0, SHEET_T, PANEL, True))  # bottom panel
    out.append((XF, XF + SHEET_T, Y_R, Y_L, 0.0, LIP_H, PANEL, True))  # its front lip
    out.append((XF, xb, Y_R, Y_L, CAB_H - SHEET_T, CAB_H, PANEL, True))  # top panel
    out.append((XF, XF + SHEET_T, Y_R, Y_L, CAB_H - LIP_H, CAB_H, PANEL, True))  # its front lip
    out.append((xb - SHEET_T, xb, Y_R, Y_L, 0.0, CAB_H, PANEL, True))  # back panel
    # the rails are held top and bottom by depth members along the side panels
    for z0 in (Z_RAIL0 - 0.012, Z_RAIL1):
        for y0, y1 in ((Y_R + SHEET_T, Y_R + SHEET_T + 0.015), (Y_L - SHEET_T - 0.015, Y_L - SHEET_T)):
            out.append((X0, xr, y0, y1, z0, z0 + 0.012, DARKSTEEL, True))
    # a 1U cantilever shelf (StarTech CABSHELFV1U) screwed into U1 of both front rails:
    # a 1U front plate with ears on the rails' front faces, a floor 435 mm wide and 250 mm deep, sides 1U tall
    z0, z1 = Z_U0, Z_U0 + SHELF_H
    out.append((X0 - SHELF_T, X0, Y_C - 0.2415, Y_C + 0.2415, z0, z1, SHELF_C, True))  # front plate and ears
    out.append((X0, X0 + SHELF_D, Y_C - SHELF_W / 2, Y_C + SHELF_W / 2, z0, z0 + SHELF_T, SHELF_C, True))  # floor
    for y0 in (Y_C - SHELF_W / 2, Y_C + SHELF_W / 2 - SHELF_T):
        out.append((X0, X0 + SHELF_D, y0, y0 + SHELF_T, z0, z1, SHELF_C, True))  # sides
    out.append((X0 + SHELF_D - SHELF_T, X0 + SHELF_D, Y_C - SHELF_W / 2, Y_C + SHELF_W / 2, z0, z0 + 0.012, SHELF_C,
                True))  # rear lip
    for k in range(9):  # vent slots in the floor (paint)
        xs = X0 + 0.030 + k * 0.024
        out.append((xs, xs + 0.008, Y_C - 0.15, Y_C + 0.15, z0 + SHELF_T, z0 + SHELF_T + 0.0002, DARKSTEEL, False))
    for hy in (HY, HY + HOLE_PITCH_Y):  # its screws in U1's bottom and top holes
        for dz in (0.00635, 0.0381):
            out.append((X0 - SHELF_T - 0.003, X0 - SHELF_T, hy - 0.006, hy + 0.006, z0 + dz - 0.006, z0 + dz + 0.006,
                        DARKSTEEL, False))
    return out


def _rail_paint() -> list:
    """Visual paint on the worked rail's front face: U lines and numbers on the outer strip, and the marked hole."""
    o = RAIL_O
    out = []
    xf = X0 - 0.00025
    y_out = HY - OUT_EDGE
    y0, y1, yc = y_out + 0.0015, HY - HH - 0.0015, HY - (OUT_EDGE + HH) / 2
    for n in range(N_U + 1):
        z = Z_U0 + n * U
        out.append(_box(xf - 0.00015, xf + 0.00015, y0, y1, z - 0.0003, z + 0.0003, PAINT, o, False))
    for n in range(N_U):
        out += _digit(str(n + 1), xf, yc, Z_U0 + (n + 0.5) * U, o)
    for xp in (X0 - 0.0003, XB + 0.0003):  # the marked hole: a red paint-pen square round it on both faces
        r0, r1 = HH + 0.0006, HH + 0.0022
        for (ya, yb, za, zb) in ((HY - r1, HY + r1, HZ + r0, HZ + r1), (HY - r1, HY + r1, HZ - r1, HZ - r0),
                                 (HY - r1, HY - r0, HZ - r0, HZ + r0), (HY + r0, HY + r1, HZ - r0, HZ + r0)):
            out.append(_box(xp - 0.0001, xp + 0.0001, ya, yb, za, zb, RED, o, False))
    out.append(_box(xf - 0.0001, xf + 0.0001, y_out + 0.0015, HY - HH - 0.0025, HZ - 0.0015, HZ + 0.0015, RED, o,
                    False))
    return out


CAB_BOXES = _cabinet_boxes()
RAIL = Body.build("rack", pos=RAIL_O, mass=21.0, color=STEEL, fixed=True, friction=(0.45, 0.4), contact_offset=0.001,
                  shapes=[_box(*b[:6], b[6], RAIL_O, b[7]) for b in CAB_BOXES] + _rail_paint())

# -- the cage nut (its frame: a along its axis, out of the face that goes against the flange; w toward a lip; s = a x w)
NUT_S, NUT_T = 0.0100, 0.0048  # M6 square nut: 10 mm across flats, 4.8 mm thick
SHEET = 0.0006  # spring-steel cage
CAGE_W, CAGE_S = 0.0101, NUT_S + 2 * SHEET  # the cage: 10.1 mm toward the lips, 11.2 mm across
CAGE_D = NUT_T + 2 * SHEET  # back plate + nut + front frame
BORE = 0.0050  # M6 minor diameter (the passage); thread major 6 mm (visual)
LIP_W_IN = 0.0039  # the lips' inner legs: outer face 3.9 mm from the axis (7.8 mm neck inside the 9.53 mm hole)
LIP_WIDTH = 0.0050  # each lip is 5 mm wide (along s)
LIP_FOLD = (0.00475, -0.00015, 0.00045)  # fold centre (a, w from the root) and radius
LIP_P1 = (0.00475, 0.00015)  # outer leg centreline from the fold ...
LIP_END = (0.0026, 0.00266)  # ... to its end (end radius 0.3 mm): outer face 6.86 mm from the axis (13.72 mm across),
# the end's back 2.3 mm in front of the cage's face (the clamp gap: panels 1.83-2.64 mm)
LIP_KG, NUT_KG = 0.0003, 0.0049  # 5.5 g in all
LIP_K, LIP_C = 0.02, 0.0003  # a lip's spring about its root: N m/rad, N m s/rad
LIP_OPEN, LIP_CLOSE = 10.0, 50.0  # degrees the lip may open / close from relaxed
NUT_CO, NUT_IT = 0.0005, 48  # contact offset (m) and solver iterations of the nut and its lips
ZINC, NUTC, BOREC = (0.96, 0.74, 0.10), (0.80, 0.58, 0.08), (0.12, 0.12, 0.13)  # yellow-zinc plated nut and cage
NUT_REST = 0.0052  # face down on the shelf, resting on its lips' folds: its face this high above the shelf's floor
NUT_START = (0.280, 0.144, TOP + SHELF_TOP + NUT_REST + 0.0016)  # spawned 1.6 mm above where it settles
NUT_MID0 = SHELF_TOP + NUT_REST + CAGE_D / 2  # its middle's height above the table, at rest on the shelf


def _cage_shapes() -> list:
    """The nut in its cage, in the frame of the middle of the cage's face (a, w, s): a block 10.1 (w) x 11.2 (s) x 6.0
    (a) mm with the M6 bore through it (5 mm square passage), the nut's face and the threaded bore (visual)."""
    hw, hs, p = CAGE_W / 2, CAGE_S / 2, BORE / 2
    a0, a1 = -CAGE_D, 0.0
    ac, ad = (a0 + a1) / 2, a1 - a0
    out = [BoxShape((ac, 0.0, (p + hs) / 2), (ad, CAGE_W, hs - p), ZINC),
           BoxShape((ac, 0.0, -(p + hs) / 2), (ad, CAGE_W, hs - p), ZINC),
           BoxShape((ac, (p + hw) / 2, 0.0), (ad, hw - p, 2 * p), ZINC),
           BoxShape((ac, -(p + hw) / 2, 0.0), (ad, hw - p, 2 * p), ZINC)]
    out.append(BoxShape((0.0001, 0.0, 0.0), (0.0002, 2 * (LIP_W_IN - SHEET), 0.0060), NUTC, collide=False))
    out.append(CylShape((-CAGE_D / 2 + 0.0002, 0.0, 0.0), 0.0030, CAGE_D + 0.0004, BOREC, collide=False, axis="X"))
    return out


def _lip_shapes(sign: float) -> list:
    """A lip in its root's frame (root on the cage's face at w = +-3.9 mm): inner leg, fold, outer leg, rounded end."""
    s = sign
    fa, fw, fr = LIP_FOLD
    out = [BoxShape((fa / 2, -s * SHEET / 2, 0.0), (fa, SHEET, LIP_WIDTH), ZINC),
           CylShape((fa, s * fw, 0.0), fr, LIP_WIDTH, ZINC, axis="Z")]
    p1 = (LIP_P1[0], s * LIP_P1[1], 0.0)
    p2 = (LIP_END[0], s * LIP_END[1], 0.0)
    d = np.subtract(p2, p1)
    out.append(plate(p1, p2, LIP_WIDTH, SHEET, (d[1], -d[0], 0.0), ZINC))
    out.append(CylShape(p2, SHEET / 2, LIP_WIDTH, ZINC, axis="Z"))
    return out


def _cage_nut(face_centre, A, W) -> list:
    """The cage nut as three bodies: the nut in its cage, and two lips (a: +w, b: -w) on sprung hinges at their roots.
    ``face_centre`` is the middle of the cage's face, ``A`` its axis and ``W`` the direction of lip a (world)."""
    A, W = np.asarray(A, float), np.asarray(W, float)
    S = np.cross(A, W)
    q = tuple(float(v) for v in _quat_from_matrix(np.column_stack([A, W, S])))
    c = np.asarray(face_centre, float)
    out = [Body.build("cage_nut", pos=tuple(float(v) for v in c), mass=NUT_KG, color=ZINC, init_quat=q,
                      friction=(0.5, 0.45), contact_offset=NUT_CO, solver_iterations=NUT_IT, shapes=_cage_shapes())]
    i = int(np.argmax(np.abs(S)))
    ax, sgn = "XYZ"[i], float(np.sign(S[i]))  # the hinges turn about S (a world axis at the start)
    for sign, lab in ((1.0, "a"), (-1.0, "b")):
        root = c + sign * LIP_W_IN * W
        lo, hi = sorted((sgn * sign * -LIP_CLOSE, sgn * sign * LIP_OPEN))
        out.append(Body.build(f"lip_{lab}", pos=tuple(float(v) for v in root), mass=LIP_KG, color=ZINC,
                              init_quat=q, friction=(0.5, 0.45), contact_offset=NUT_CO, solver_iterations=NUT_IT,
                              shapes=_lip_shapes(sign),
                              joint=Joint(anchor=tuple(float(v) for v in root), kind="revolute", axis=ax,
                                          parent="cage_nut", limits=(lo, hi), stiffness=LIP_K, damping=LIP_C,
                                          target=0.0)))
    return out


# -- the flat-blade screwdriver (its frame: x from the blade's tip toward the handle, y across the blade, z along it) -
SD_TIP_T, SD_TIP_W, SD_TIP_L = 0.0006, 0.0035, 0.009  # tip 0.6 x 3.5 mm (ISO 2380-1), its thin part 9 mm long
SD_WEDGE_T, SD_WEDGE_L = 0.0012, 0.004  # then 1.2 mm thick for 4 mm, then the round shank
SD_SHANK_R, SD_SHANK_END = 0.00175, 0.060  # 3.5 mm shank, 60 mm from the tip
SD_HANDLE = (0.066, 0.161, 0.025)  # fluted handle: from, to, 25 mm across
SD_KG = 0.040
SD_START = (0.190, -0.135)  # the tip's (x, y) where it lies on the bench, pointing away from the robot (+x)
HANDLE_C, BLADE_C, SHANKC = (0.85, 0.55, 0.10), (0.60, 0.61, 0.63), (0.70, 0.71, 0.73)


def _screwdriver_shapes() -> list:
    out = [BoxShape((SD_TIP_L / 2, 0.0, 0.0), (SD_TIP_L, SD_TIP_T, SD_TIP_W), BLADE_C),
           BoxShape((SD_TIP_L + SD_WEDGE_L / 2, 0.0, 0.0), (SD_WEDGE_L, SD_WEDGE_T, SD_TIP_W), BLADE_C)]
    x0 = SD_TIP_L + SD_WEDGE_L
    out.append(CylShape(((x0 + SD_SHANK_END) / 2, 0.0, 0.0), SD_SHANK_R, SD_SHANK_END - x0, SHANKC, axis="X"))
    out.append(CylShape(((SD_SHANK_END + SD_HANDLE[0]) / 2, 0.0, 0.0), 0.0055, SD_HANDLE[0] - SD_SHANK_END, SHANKC,
                        axis="X"))
    h0, h1, d = SD_HANDLE
    for k in range(4):  # four flutes round the handle: flats at +-y (where it is gripped) and +-z (it rests on one)
        a = k * math.pi / 4
        out.append(BoxShape(((h0 + h1) / 2, 0.0, 0.0), (h1 - h0, d, 0.58 * d), HANDLE_C,
                            quat_xyzw=(math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))))
    return out


SCREWDRIVER = Body.build("screwdriver", pos=(SD_START[0], SD_START[1], TOP + SD_HANDLE[2] / 2 + 0.001), mass=SD_KG,
                         color=HANDLE_C, yaw=math.pi, friction=(0.4, 0.35), contact_offset=0.001,
                         solver_iterations=32, shapes=_screwdriver_shapes())

OBJECTS = [RAIL, *_cage_nut(NUT_START, (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)), SCREWDRIVER]

# -- scoring ----------------------------------------------------------------------------------------------------------
LIFTED_Z, BENCH_Z = 0.03, 0.02  # the nut lifted: its middle 3 cm above where it lay; fallen: back below 2 cm above it
# (on the shelf, the cabinet's floor or the bench) with no gripper near
HELD_R = 0.06  # a gripper this close to the nut's middle may be holding it (it has not fallen then)
CLEAR_R = 0.03  # released: both grippers' centres at least this far from the nut's middle (fingertips 1.7 cm)
SD_OUT = 0.010  # the screwdriver's tip at least this far in front of the flange (or CLEAR_R from the nut's middle)
FLAT_DEG = 8.0  # seated: the nut's axis within 8 deg of the flange's normal ...
FACE_BACK, FACE_FWD = 0.0012, 0.0005  # ... its face from 1.2 mm behind to 0.5 mm into the flange's back face ...
CENTRE_Y, CENTRE_Z = 0.0015, 0.0035  # ... its middle within 1.5 mm (across) and 3.5 mm (up/down) of the hole's middle
OUT_PAST = 0.0002  # a lip is out when its end's middle is in front of the front face and 0.2 mm past the hole's edge
BENT_DEG = 30.0  # a lip forced 30 deg open (20 deg past its stop) has been bent: the nut is ruined
STILL_STEPS, STILL_MM = 20, 0.5
_DEBUG = os.environ.get("CAGE_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("CAGE_DEBUG_EVERY", "10"))
_EP: dict = {}
_TCP: dict = {}
_ARM_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(step=0, tick=0, counted=-1, best=0.0, failed="", poses={}, lifted=False, seated=False, one_out=False,
               free=False, near=1.0, recent=[], outs=(None, None), angles=(0.0, 0.0), stage=0.0)
    _TCP.clear()


_reset()


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("CAGE_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _pose(base, name, origin):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _lip_angle(pn, pl, sign) -> float:
    """A lip's opening angle (deg) relative to the nut: its a-axis turned toward its own side (+) or in (-)."""
    a_l = pn[1].T @ pl[1][:, 0]
    return math.degrees(math.atan2(sign * a_l[1], a_l[0]))


def _lip_end(pl, sign) -> np.ndarray:
    """World position of a lip's rounded end (its middle)."""
    return pl[0] + pl[1] @ np.array([LIP_END[0], sign * LIP_END[1], 0.0])


def _out_at(e, sign):
    """The hole (index into HOLE_Z) over whose edge this lip's end (world) has sprung out, or None: the end is in front
    of the flange's front face (toward the robot) and past that hole's edge on the lip's own side."""
    if e[0] > X0 or sign * (e[1] - HY) < HH + OUT_PAST:
        return None
    for k, z in enumerate(HOLE_Z):
        if abs(e[2] - TOP - z) < HH:
            return k
    return None


def _track_inner(env) -> None:
    """Each gripper's centre (TCP) from its joints; the nut's, its lips' and the screwdriver's poses; whether the nut
    is seated, hangs by one lip or is free of both grippers and the screwdriver; the rules it may break."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        _TCP[side] = np.asarray(ik.tcp_pose(q[_ARM_IDS[side]], station.BASES[side])[0], float)
    origin = as_numpy(base.scene.env_origins[0])
    P = {k: _pose(base, k, origin) for k in ("cage_nut", "lip_a", "lip_b", "screwdriver")}
    pn, rn = P["cage_nut"]
    A = rn[:, 0]
    mid = pn - (CAGE_D / 2) * A
    outs, angles, ends = [], [], []
    for lab, sign in (("a", 1.0), ("b", -1.0)):
        e = _lip_end(P[f"lip_{lab}"], sign)
        ends.append(e)
        outs.append(_out_at(e, sign))
        angles.append(_lip_angle(P["cage_nut"], P[f"lip_{lab}"], sign))
    near = min(float(np.linalg.norm(v - mid)) for v in _TCP.values())
    sp = P["screwdriver"][0]
    sd_clear = sp[0] <= X0 - SD_OUT or float(np.linalg.norm(sp - mid)) >= CLEAR_R
    z = mid[2] - TOP
    if z > NUT_MID0 + LIFTED_Z:
        _EP["lifted"] = True
    if not _EP["failed"]:
        if _EP["lifted"] and z < NUT_MID0 + BENCH_Z and near > HELD_R:
            _EP["failed"] = "the nut fell"
        elif any(o is not None and o != MARK for o in outs):
            _EP["failed"] = "a lip sprang out at another hole"
        elif max(angles) > BENT_DEG:
            _EP["failed"] = "a lip was forced open: the nut's clip is bent"
    flat = -A[0] >= math.cos(math.radians(FLAT_DEG))
    face_ok = XB - FACE_FWD <= pn[0] <= XB + FACE_BACK
    centred = abs(mid[1] - HY) <= CENTRE_Y and abs(z - HZ) <= CENTRE_Z
    seated = outs[0] == MARK and outs[1] == MARK and flat and face_ok and centred
    one_out = (outs[0] == MARK) != (outs[1] == MARK)
    free = near >= CLEAR_R and sd_clear
    stage = 0.5 if free and (seated or one_out) else 0.0
    if not _EP["failed"]:
        _EP["best"] = max(_EP["best"], stage)
    _EP.update(poses=P, seated=seated, one_out=one_out, free=free, near=near, outs=tuple(outs), angles=tuple(angles),
               ends=ends, stage=stage, mid=mid)


def _tally() -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"] and "mid" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append(np.asarray(_EP["mid"], float).copy())
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(np.abs(p - recent[-1]).max() * 1000 < STILL_MM for p in recent)
    return dict(still=still, done=bool(_EP["seated"] and _EP["free"] and still and not _EP["failed"]))


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY or "mid" not in _EP:
        return
    pn, rn = _EP["poses"]["cage_nut"]
    ax = rn[:, 0]
    ends = ""
    for (lab, sign), e in zip((("A", 1.0), ("B", -1.0)), _EP["ends"]):
        ends += (f" end{lab}(front+{(X0 - e[0]) * 1000:.2f} past+{(sign * (e[1] - HY) - HH) * 1000:.2f} "
                 f"dz{(e[2] - TOP - HZ) * 1000:.1f})")
    print(f"DBG step={_EP['step']} nut face=({pn[0]:.4f},{pn[1]:.4f},{pn[2] - TOP:.4f}) "
          f"axis=({ax[0]:.3f},{ax[1]:.3f},{ax[2]:.3f}) lipA={_EP['angles'][0]:.1f} lipB={_EP['angles'][1]:.1f}{ends}",
          flush=True)
    sp, sr = _EP["poses"]["screwdriver"]
    print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.4f},{v[1]:.4f},{v[2] - TOP:.4f})" for k, v in _TCP.items())
          + f" | sd tip=({sp[0]:.4f},{sp[1]:.4f},{sp[2] - TOP:.4f})", flush=True)
    print(f"DBG   stage={_EP['stage']} best={_EP['best']} seated={_EP['seated']} one_out={_EP['one_out']} "
          f"free={_EP['free']} near={_EP['near']:.3f} outs={_EP['outs']} still={t['still']} "
          f"failed={_EP['failed'] or '-'}", flush=True)


def score(objects) -> float:
    """The best stage reached (0 for the rest of the run once the nut has fallen, a lip has sprung out at another hole
    or been forced open): 0.5 the nut sits in the marked hole by itself, at least one lip sprung out over the front
    face, nothing touching it. Full score (1.0) is success()."""
    t = _tally()
    if _DEBUG:
        _debug(t)
    return 0.0 if _EP["failed"] else _EP["best"]


def success(objects) -> bool:
    """Seated in the marked hole (both lips sprung out over the front face, the nut flat against the flange's back
    face and centred in the hole), both grippers' centres 3 cm or more from it, the screwdriver out of the hole, still
    for 1 s, no rule broken."""
    return _tally()["done"]


# -- scripted demo ----------------------------------------------------------------------------------------------------
# As the rack makers' guides say: the left gripper reaches into the cabinet through its front opening, takes the cage
# nut from the shelf by its two plain sides, turns it lips-first toward the rail and brings it, round the flange's inner
# edge, to the back of the marked hole; there, tilted 45 deg, it pushes the nut's +y lip through the hole and slides it
# over the hole's +y edge (hooked). The right gripper brings the screwdriver from the front and slides its blade into
# the hole along the hole's -y side, 6.8 mm past the flange, where it keeps the far lip in. The left turns the nut flat
# about its hooked lip and presses it home: the far lip, kept in line by the blade and then pressed in by the hole's
# edge, passes the hole. The right draws the blade out and the far lip springs out over the front face; the left lets
# go. For the video, the right then puts the screwdriver back on the bench and brings its wrist camera 7 cm from the
# marked hole (the closing view of the seated nut); the left, which waited 1.3 cm behind the nut, draws away and the run
# ends once the nut is scored. CAGE_ONE=1 runs it the wrong way: the blade stays out and the left hand turns the nut
# home alone.

WRONG_ONE = os.environ.get("CAGE_ONE") == "1"
OPEN_C = -0.030
PINCH, PINCH_LIGHT = -0.0028, -0.0048  # fingers 2.8 mm inside each face of the 11.2 mm cage; 0.8 mm while turning it
# The left gripper holds the nut by its two plain sides (11.2 mm apart), its fingers closing across them. In the nut's
# frame its centre (TCP) sits GRIP_A behind the face and GRIP_W toward lip a, and it points along -w tilted GRIP_DELTA
# toward the face, so that the tapering fingers stay behind the face (behind the flange) all along the flange.
GRIP_A, GRIP_W, GRIP_DELTA = -0.0130, 0.0060, math.radians(17.0)
GRIP_SIGN = -1.0
HOOK_PHI = math.radians(45.0)  # the nut turned this far (its +y side toward the rail) to hook its +y lip
HOOK_PAST = 0.0015  # the hooked lip's end this far past the hole's +y edge ...
HOOK_FRONT = 0.0008  # ... and its middle this far in front of the front face (its back 0.5 mm clear)
ENTRY_IN = 0.0020  # on the way in, that end this far inside the +y edge, ...
PRE_BACK = 0.0030  # ... first this far behind the flange's back face, ...
ENTRY_FRONT = 0.0028  # ... then pushed through the hole to this far in front of the front face
PULL_SNUG = 0.0004  # while the nut is turned flat, the hooked lip's end is pulled this far against the front face ...
PRESS_HOOK = 0.0010  # ... and the nut pressed this far toward the hooked side
T_TURN = 8.0
SEAT = np.array([XB - 0.0025, HY + 0.0004, TOP + HZ])  # pressed home: its face aimed 2.5 mm into the flange (the arm
# gives way about 2 mm under the push)


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


DOWN_SD = grip_rotation(0.0)  # right arm straight down, fingers closing along y (across the screwdriver's handle)


def _nut_frame(phi: float):
    """The nut's axis A and lip-a direction W (world) facing the rail, turned ``phi`` about z (+y side forward)."""
    return (np.array([-math.cos(phi), -math.sin(phi), 0.0]), np.array([-math.sin(phi), math.cos(phi), 0.0]))


def _grip(face_centre, A, W):
    """(TCP, rotation) of the left gripper holding the nut whose face centre, axis and lip-a direction are given."""
    A, W = np.asarray(A, float), np.asarray(W, float)
    S = np.cross(A, W)
    d = math.cos(GRIP_DELTA) * -W + math.sin(GRIP_DELTA) * A
    xg = GRIP_SIGN * S
    rot = np.column_stack([xg, np.cross(d, xg), d])
    return np.asarray(face_centre, float) + GRIP_A * A + GRIP_W * W, rot


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    for sd in itertools.product((-0.4, 0.0, 0.4), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2),
                                (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _lip_end_offset(A, W):
    """Lip a's end (its middle) from the face centre, for a relaxed lip."""
    return LIP_END[0] * A + (LIP_W_IN + LIP_END[1]) * W


def _hook_centres():
    """The nut's axis and lip-a direction, turned HOOK_PHI with its +y side toward the rail, and three face centres
    (world), from where lip a's end is: pre (the end just behind the hole, inside its +y edge), entry (the end pushed
    through the hole, in front of the front face) and hooked (the end slid past the hole's +y edge, over the front
    face)."""
    A, W = _nut_frame(HOOK_PHI)
    end = _lip_end_offset(A, W)
    y_in = HY + HH - ENTRY_IN
    pre = np.array([XB + PRE_BACK, y_in, TOP + HZ]) - end
    entry = np.array([X0 - ENTRY_FRONT, y_in, TOP + HZ]) - end
    hooked = np.array([X0 - HOOK_FRONT, HY + HH + HOOK_PAST, TOP + HZ]) - end
    return A, W, pre, entry, hooked


T_HOOKED = 14.7  # the left gripper has the nut hooked by its +y lip at this time (s): checked in _left()
T_SD_IN = T_HOOKED + 0.5  # the screwdriver goes in ...
T_PUSH = T_SD_IN + 3.2  # the nut is turned home ...
T_SD_OUT = T_PUSH + 7.4  # the screwdriver comes out, near the end of the turn ...
T_LET_GO = T_SD_OUT + 1.5  # the left gripper lets go
FACE0 = np.array([NUT_START[0], NUT_START[1], TOP + SHELF_TOP + NUT_REST])  # the nut's face centre, at rest
STAGE = np.array([XB + 0.030, HY + 0.050, TOP + HZ])  # the nut's face centre on its way to the hole, inside the cabinet


def _left() -> list:
    # take the nut from the shelf, lying face down, by its two plain sides: straight down onto it, fingers open
    A0, W0 = np.array([0.0, 0.0, -1.0]), np.array([0.0, 1.0, 0.0])
    grab, rot0 = _grip(FACE0, A0, W0)
    out = _lead_in("left", grab + [0.0, 0.0, 0.04], rot0, OPEN_C, "above the cage nut, fingers either side")
    out += [_w(1.5, grab, OPEN_C, "fingers down either side of the nut", rot0),
            _w(0.4, grab, OPEN_C, "settle", rot0),
            _w(0.8, grab, PINCH, "take the nut", rot0),
            _w(0.6, grab, PINCH, "settle", rot0)]
    # lift it, turn it lips-first toward the rail (its face toward -x) and bring it behind the marked hole
    A, W, pre, entry, hooked = _hook_centres()
    for t, c, (Ak, Wk), lab in ((1.0, FACE0 + [0.0, 0.0, 0.07], (A0, W0), "lift it"),
                                (2.5, STAGE, (A, W), "turn it lips-first toward the rail, behind the flange"),
                                (1.5, pre, (A, W), "behind the marked hole, tilted 45 deg"),
                                (0.5, pre, (A, W), "settle"),
                                (1.5, entry, (A, W), "the +y lip pushed through the hole"),
                                (0.4, entry, (A, W), "settle"),
                                (1.0, hooked, (A, W), "slide it toward +y: the lip's end over the front face")):
        p, r = _grip(c, Ak, Wk)
        out.append(_w(t, p, PINCH, lab, r))
    assert abs(sum(w.duration for w in out) - T_HOOKED) < 1e-6, sum(w.duration for w in out)
    p, r = _grip(hooked, A, W)
    out += [_w(T_PUSH - T_HOOKED, p, PINCH, "hold it hooked while the blade goes in", r)]
    # turn it flat about the hooked lip's end (pulled snug onto the front face and pressed toward that side)
    end = hooked + _lip_end_offset(A, W) + [HOOK_FRONT + PULL_SNUG, PRESS_HOOK, 0.0]
    n = 8
    for k in range(1, n + 1):
        Ak, Wk = _nut_frame(HOOK_PHI * (1.0 - k / n))
        p, r = _grip(end - _lip_end_offset(Ak, Wk), Ak, Wk)
        out.append(_w(T_TURN / n, p, PINCH_LIGHT, "turn it flat about the hooked lip, pushing it in", r))
    Af, Wf = _nut_frame(0.0)
    seat_tcp, rot_f = _grip(SEAT, Af, Wf)
    near = seat_tcp + [LEFT_WAIT_BACK, 0.0, 0.0]
    away = seat_tcp + [0.015, 0.035, 0.0]
    park = seat_tcp + [0.030, 0.035, 0.030]
    out += [_w(0.6, seat_tcp, PINCH, "press it home, flat against the flange", rot_f),
            _w(T_LET_GO - T_PUSH - T_TURN - 0.6, seat_tcp, PINCH, "hold it there", rot_f),
            _w(0.8, seat_tcp, OPEN_C, "let go", rot_f)]
    if not WRONG_ONE:  # wait, fingers open, 1.3 cm behind the nut, while the right wrist camera comes to look at it
        out += [_w(1.0, near, OPEN_C, "draw back 1.3 cm behind the nut, fingers open", rot_f),
                _w(T_LEFT_AWAY - T_LET_GO - 1.8, near, OPEN_C, "wait there while the right camera looks at the nut",
                   rot_f)]
    out += [_w(1.0, away, OPEN_C, "back off from the flange, into the opening", rot_f),
            _w(1.5, park, OPEN_C, "clear of the nut", rot_f),
            _w(3.0, park, OPEN_C, "stay clear", rot_f)]
    return out


PINCH_SD = -(SD_HANDLE[2] / 2 - 0.003)
SD_GRIP_L = 0.075  # the right gripper takes the handle here (from the tip) ...
SD_AXIS_BELOW = 0.006  # ... with the screwdriver's axis this far beyond its centre, along its pointing direction
SD_TILT = math.radians(30.0)  # the right gripper points down and 30 deg forward, so its wrist stays out of the cabinet
ROT_SD = _ry(-SD_TILT) @ DOWN_SD
SD_LINE_Y = HY - 0.0020  # the blade enters the hole 2 mm to the -y side of its middle ...
SD_WALL_Y = HY - HH + SD_TIP_T / 2 - 0.0025  # ... then is pressed against that side (aimed 2.5 mm into it)
SD_DEPTH = 0.0068  # the tip this far behind the flange's back face
SD_AXIS_Z = 0.0125  # the screwdriver's axis above the bench where it lies (on a flute of its handle)
# The closing view (for the video only): the right wrist camera (Dream's calibrated D405 mount) looks straight at the
# marked hole from VIEW_D away, from VIEW_PSI to the right and VIEW_ALPHA above, the image turned VIEW_PHI (it comes in
# along that line from VIEW_D_FAR); the left waits, fingers open, 1.3 cm behind the nut, for VIEW_HOLD s, then draws
# away; once both grippers are clear the run is scored and ends (about 2 s later), the camera still on the nut.
VIEW_D, VIEW_D_FAR = 0.070, 0.16
VIEW_PSI, VIEW_ALPHA, VIEW_PHI = math.radians(40.0), math.radians(50.0), math.radians(30.0)
VIEW_HOLD = 2.0
LEFT_WAIT_BACK = 0.013
T_VIEW = T_SD_OUT + 13.5  # the right wrist camera is at the closing view
T_LEFT_AWAY = T_VIEW + VIEW_HOLD  # the left draws away from the nut


def _view_pose(d: float = VIEW_D):
    """(TCP, rotation) of the right gripper whose wrist camera looks straight at the marked hole from ``d`` away."""
    from yam_arena import station
    from yam_arena.constants import TCP_OFFSET
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.wrist_camera("right")  # pose in the gripper frame, OpenGL convention (looks along -z, +y up)
    f = np.array([math.cos(VIEW_ALPHA) * math.cos(VIEW_PSI), math.cos(VIEW_ALPHA) * math.sin(VIEW_PSI),
                  -math.sin(VIEW_ALPHA)])
    up = np.array([0.0, 0.0, 1.0]) - f[2] * f
    up /= np.linalg.norm(up)
    up = math.cos(VIEW_PHI) * up + math.sin(VIEW_PHI) * np.cross(f, up)
    rot = np.column_stack([np.cross(f, up), up, -f]) @ quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw, float)).T
    c = np.array([X0, HY, TOP + HZ]) - d * f
    return c - rot @ np.asarray(cam.pos, float) + rot @ np.asarray(TCP_OFFSET, float), rot


def _sd_tcp(tip) -> np.ndarray:
    """The right gripper's centre when the screwdriver's tip is at ``tip`` (world), the blade pointing +x."""
    return np.asarray(tip, float) - [SD_GRIP_L, 0.0, 0.0] - SD_AXIS_BELOW * ROT_SD[:, 2]


def _right() -> list:
    rot = ROT_SD
    grab = _sd_tcp([SD_START[0], SD_START[1], TOP + SD_AXIS_Z])
    lift = grab + [0.0, 0.0, 0.10]
    front = _sd_tcp([X0 - 0.010, SD_LINE_Y, TOP + HZ])
    entered = front + [0.010 + 0.003, 0.0, 0.0]  # tip 3 mm into the hole
    at_wall = np.array([entered[0], entered[1] + SD_WALL_Y - SD_LINE_Y, entered[2]])
    inside = at_wall + [FL_T + SD_DEPTH - 0.001, 0.0, 0.0]
    away = _sd_tcp([X0 - 0.045, HY, TOP + HZ + 0.025])  # back off, the blade 4.5 cm in front of the hole
    out = _lead_in("right", grab + [0.0, 0.0, 0.06], rot, OPEN_C, "above the screwdriver's handle")
    out += [_w(1.2, grab, OPEN_C, "fingers either side of the handle", rot),
            _w(0.4, grab, OPEN_C, "settle", rot),
            _w(0.8, grab, PINCH_SD, "take the screwdriver", rot),
            _w(0.6, grab, PINCH_SD, "settle", rot),
            _w(1.5, lift, PINCH_SD, "lift it", rot),
            _w(2.0, front + [-0.03, 0.0, 0.012], PINCH_SD, "in front of the marked hole", rot),
            _w(1.5, front, PINCH_SD, "blade level with the hole, near its -y side", rot)]
    t = sum(w.duration for w in out)
    if WRONG_ONE:
        return out + [_w(T_SD_OUT - t, front, PINCH_SD, "the blade stays out of the hole", rot),
                      _w(1.5, away, PINCH_SD, "move away", rot),
                      _w(6.0, away, PINCH_SD, "stay clear", rot)]
    out += [_w(T_SD_IN - t, front, PINCH_SD, "wait for the nut to be hooked", rot),
            _w(1.2, entered, PINCH_SD, "blade tip into the hole", rot),
            _w(0.5, at_wall, PINCH_SD, "against the hole's -y side", rot),
            _w(1.4, inside, PINCH_SD, "blade in along that side, past the far lip", rot),
            _w(T_SD_OUT - T_SD_IN - 3.1, inside, PINCH_SD, "keep the far lip in line while the nut is turned home",
               rot),
            _w(1.2, front, PINCH_SD, "draw the blade out: the far lip springs out over the front face", rot),
            _w(1.5, away, PINCH_SD, "back off in front of the hole", rot),
            _w(2.5, lift, PINCH_SD, "back over where the screwdriver lay", rot),
            _w(1.5, grab + [0.0, 0.0, 0.004], PINCH_SD, "lower it onto the bench", rot),
            _w(0.8, grab + [0.0, 0.0, 0.004], OPEN_C, "put it down", rot),
            _w(0.3, grab + [0.0, 0.0, 0.004], OPEN_C, "settle", rot),
            _w(1.0, grab + [0.0, 0.0, 0.06], OPEN_C, "rise", rot)]
    # the closing view of the seated nut: the wrist camera VIEW_D from the marked hole, held until the run ends (once
    # the left has drawn away and the nut is scored); the overhead camera still sees the hole past the right wrist
    view, rv = _view_pose()
    far, _ = _view_pose(VIEW_D_FAR)
    out += [_w(2.0, far + [-0.03, -0.04, 0.03], 0.0, "turn the wrist camera toward the marked hole", rv),
            _w(1.2, far, 0.0, "in front of the cabinet, camera toward the hole", rv),
            _w(1.5, view, 0.0, "camera close to the hole: look at the seated nut", rv)]
    assert abs(sum(w.duration for w in out) - T_VIEW) < 1e-6, (sum(w.duration for w in out), T_VIEW)
    return out + [_w(12.0, view, 0.0, "hold the view of the seated nut", rv)]


def demo():
    return {"left": _left(), "right": _right()}
