"""A fast-food fry station. On a stainless work counter stands a countertop fry dump station: a stainless base
holding a full-size food pan 2 in deep, a back panel and a heat hood over the pan (two calrod heaters and a lamp). A
heap of fries lies against the pan's back wall; the station's fry scoop lies empty in the pan's front:
a stainless flat-bottomed scoop that narrows to a straight spout at its funnel end, its black cool-touch handle on
the right. An empty small paper fry carton (red) stands on the counter in front of the station. The fries are the real trial's clean replica
fries: hard-maple sticks of a 3/8 in straight-cut fry's size, every edge bevelled 2 mm.

Task (the instruction gives the goal and its constraints only): with the scoop, fill the carton with a portion of fries,
put the scoop back in the pan and leave the carton standing on the counter, arms back home. The way it is done: hold
the carton by one hand, take the scoop by its handle with the other, drive it along the pan's floor into the heap so the
fries ride up onto it, draw the load back out, lift it, tip it into the carton through the funnel end, then lay the
scoop back in the pan.

Rules: the carton may not go into the pan (it is not the scoop); a fry the robot touches may not be served; only fries
the scoop carried up out of the pan count. Scored by the number of fries in the carton (18-28, a small order), the
fries spilled, the carton standing, the scoop back and the arms home.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()


def _quat_from_matrix(m) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix, by Shepperd's method (bodykit's own picks wrong signs near 180 deg)."""
    m = np.asarray(m, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        k = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * k, (m[2, 1] - m[1, 2]) / k, (m[0, 2] - m[2, 0]) / k, (m[1, 0] - m[0, 1]) / k
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        k = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / k, 0.25 * k, (m[0, 1] + m[1, 0]) / k, (m[0, 2] + m[2, 0]) / k
    elif m[1, 1] > m[2, 2]:
        k = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / k, (m[0, 1] + m[1, 0]) / k, 0.25 * k, (m[1, 2] + m[2, 1]) / k
    else:
        k = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / k, (m[0, 2] + m[2, 0]) / k, (m[1, 2] + m[2, 1]) / k, 0.25 * k
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


bodykit._quat_from_matrix = _quat_from_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the work counter: a stainless top on the table ----------------------------------------------------------------------
CT_T = 0.010
CT = TOP + CT_T  # the counter's surface
CT_X0, CT_X1, CT_Y0, CT_Y1 = 0.100, 0.895, -0.62, 0.62

# -- the dump station (Avantco FFDS-1: 13 in wide, 22.5 in deep, 19.5 in tall, base 2 7/8 in, full-size pan 2 in deep) --
X0, YS = 0.215, 0.155  # its front face and its centre line
ST_W, ST_D, ST_H, ST_BASE = 0.330, 0.572, 0.495, 0.073
PAN_IN_W, PAN_IN_L, PAN_DEPTH, PAN_WALL = 0.300, 0.505, 0.051, 0.006  # the pan's inside at the top, its depth
PAN_X0 = X0 + 0.034  # the pan's inside front wall
PAN_FLOOR = CT + ST_BASE - PAN_DEPTH  # the pan's floor (top face)
HOOD_X0, HOOD_Z0, HOOD_Z1 = X0 + 0.210, CT + 0.430, CT + ST_H  # the heat hood: front edge, underside, top
BACK_T = 0.012

# -- the fry scoop (flat bottom; 9 in long, 8 in wide at the open end, 3 3/4 in at the funnel end, where a straight,
# parallel-sided spout 5 cm long runs out from the flared body) --------------------------------------------------------
SC_L, SC_W0, SC_W1, SC_H = 0.229, 0.095, 0.203, 0.057
SC_SPOUT = 0.050  # the straight spout at the funnel end
SC_T = float(os.environ.get("FRIES_SC_T", "0.001"))  # the sheet: its lip thin enough to slide under a fry's bevel
SC_WALL_T = float(os.environ.get("FRIES_SC_WALL_T", "0.001"))  # the side walls' sheet (rolled top edges drawn)
SC_MASS = 0.20
H_ROOT_X, H_LEN, H_RISE_DEG, H_SEC = 0.035, 0.120, 20.0, (0.015, 0.022)  # handle: where on the right wall, length,
# rise, section (along the scoop's x, and across)
SC_X = PAN_X0 + 0.004  # the scoop's funnel end, at the start (lying in the pan's front)
SC_Y = YS

# -- the carton (small fry carton: 4.50 x 2.25 x 4.50 in, paperboard, about 7.7 g) ------------------------------------
CA_TOP_W, CA_TOP_D, CA_BOT_W, CA_BOT_D = 0.114, 0.057, 0.080, 0.034
CA_H_BACK, CA_H_FRONT, CA_T, CA_MASS = 0.114, 0.082, 0.003, 0.0077
CA_XY = (X0 - float(os.environ.get("FRIES_CA_GAP", "0.090")), YS)  # where it stands at the start

# -- the fries: the real trial's replica fries, hard-maple sticks cut from 3/8 in square dowel (a 3/8 in straight-cut
# fry's size), every edge bevelled 2 mm at 45 degrees; hard maple about 705 kg/m3 at 12% moisture, so about 4.3 g for a
# 75 mm stick (a fast-food fry weighs 5 g) ----------------------------------------------------------------------------
FRY_S = float(os.environ.get("FRIES_S", "0.009525"))
FRY_CH = float(os.environ.get("FRIES_CH", "0.002"))  # the long edges and the ends' edges eased: 45-degree chamfers
FRY_LENS = (0.055, 0.082, 0.064, 0.095, 0.071, 0.088, 0.060, 0.078, 0.091, 0.068, 0.075, 0.085)
FRY_RHO = float(os.environ.get("FRIES_RHO", "700"))  # the stand-in's density (kg/m3): mass from the hull's volume
# friction (PhysX averages the two bodies' values): 0.5 stick on stick (oak on oak 0.54-0.62 static, clean wood on wood
# 0.25-0.5), 0.55 on the pan and 0.5 on the scoop (wood on clean metal 0.2-0.6; Engineering ToolBox)
FRY_MU = float(os.environ.get("FRIES_FRY_MU", "0.50"))
STATION_MU = float(os.environ.get("FRIES_PAN_MU", "0.60"))  # the pan's brushed steel
SCOOP_MU = float(os.environ.get("FRIES_SCOOP_MU", "0.50"))  # the scoop's polished steel
HEAP_GAP = float(os.environ.get("FRIES_HEAP_GAP", "0.050"))  # between the scoop's open end and the heap (spawned)
HEAP_X0, HEAP_X1 = SC_X + SC_L + HEAP_GAP, PAN_X0 + PAN_IN_L - 0.006  # the heap fills the pan behind the scoop
HEAP_Y0, HEAP_Y1 = YS - PAN_IN_W / 2 + 0.012, YS + PAN_IN_W / 2 - 0.012
HEAP_PER_LAYER = tuple(int(v) for v in os.environ.get("FRIES_LAYERS", "44,40,36,32,28,22,16,10,6").split(","))
HEAP_SHRINK = (0.004, 0.010)  # each layer starts this much further back (x) and in from the sides (y): a mound

STEEL, STEEL_D, STEEL_L, BLACK, BLACK_L = (0.70, 0.71, 0.73), (0.52, 0.53, 0.55), (0.82, 0.83, 0.85), \
    (0.05, 0.05, 0.06), (0.16, 0.16, 0.17)
RED, RED_D, YELLOW, WHITE = (0.78, 0.07, 0.06), (0.60, 0.05, 0.04), (0.98, 0.78, 0.10), (0.96, 0.95, 0.92)
HEAT, LAMP, GREEN = (0.85, 0.30, 0.10), (1.0, 0.93, 0.70), (0.15, 0.85, 0.25)
SC_STEEL, SC_EDGE, PAN_IN = (0.88, 0.89, 0.91), (0.40, 0.41, 0.44), (0.38, 0.39, 0.41)  # polished scoop, worn pan
FRY_COLS = ((0.93, 0.74, 0.33), (0.88, 0.66, 0.26), (0.95, 0.80, 0.42))


# -- the bodies --------------------------------------------------------------------------------------------------------

def _counter() -> Body:
    lx, ly = CT_X1 - CT_X0, CT_Y1 - CT_Y0
    shapes = [BoxShape((0.0, 0.0, CT_T / 2), (lx, ly, CT_T), STEEL)]
    for k in range(10):  # brushed-steel streaks
        shapes.append(BoxShape((0.0, -ly / 2 + 0.06 + k * 0.12, CT_T + 0.0002), (lx - 0.02, 0.002, 0.0003), STEEL_L,
                               collide=False))
    return Body.build("counter", pos=((CT_X0 + CT_X1) / 2, (CT_Y0 + CT_Y1) / 2, TOP), mass=20.0, kinematic=True,
                      color=STEEL, shapes=shapes, friction=(0.35, 0.3))


def _station() -> Body:
    """The dump station, fixed. Origin: on the counter, under the middle of its front face."""
    w, d, b = ST_W, ST_D, ST_BASE
    fz = PAN_FLOOR - CT  # the pan floor's height in the station frame
    px0, px1 = PAN_X0 - X0, PAN_X0 - X0 + PAN_IN_L
    hw = PAN_IN_W / 2
    t = PAN_WALL
    shapes = [
        BoxShape((d / 2, 0.0, fz / 2), (d, w, fz), STEEL),  # the base under the pan
        # the base's top deck round the pan, with the pan's flange
        BoxShape((px0 / 2, 0.0, (fz + b) / 2), (px0, w, b - fz), STEEL),  # front
        BoxShape(((px1 + d) / 2, 0.0, (fz + b) / 2), (d - px1, w, b - fz), STEEL),  # back
    ]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape(((px0 + px1) / 2, s * (hw + (w / 2 - hw) / 2), (fz + b) / 2), (px1 - px0, w / 2 - hw, b - fz),
                               STEEL))
        # the pan's flange (a raised rim) and its rolled edge, drawn
        shapes.append(BoxShape(((px0 + px1) / 2, s * (hw + 0.007), b + 0.001), (px1 - px0 + 0.028, 0.014, 0.002), STEEL_L,
                               collide=False))
    for x in (px0 - 0.007, px1 + 0.007):
        shapes.append(BoxShape((x, 0.0, b + 0.001), (0.014, 2 * hw + 0.028, 0.002), STEEL_L, collide=False))
    # the pan's floor (a slightly darker, worn steel) and its walls' inner faces
    shapes.append(BoxShape(((px0 + px1) / 2, 0.0, fz - 0.0004), (px1 - px0, 2 * hw, 0.001), PAN_IN, collide=False))
    for s in (-1.0, 1.0):  # the pan's inner walls, drawn darker
        shapes.append(BoxShape(((px0 + px1) / 2, s * (hw + 0.0004), (fz + b) / 2), (px1 - px0, 0.001, b - fz), STEEL_D,
                               collide=False))
    for x in (px0 - 0.0004, px1 + 0.0004):
        shapes.append(BoxShape((x, 0.0, (fz + b) / 2), (0.001, 2 * hw, b - fz), STEEL_D, collide=False))
    # the control panel on the base's front (7 3/4 x 2 1/4 in): heat and light switches, a pilot light
    shapes += [BoxShape((-0.0015, 0.0, b / 2), (0.003, 0.197, 0.057), BLACK_L, collide=False),
               CylShape((-0.007, 0.045, b / 2), 0.006, 0.010, BLACK, collide=False, axis="X"),
               CylShape((-0.007, -0.045, b / 2), 0.006, 0.010, BLACK, collide=False, axis="X"),
               CylShape((-0.004, 0.0, b / 2 + 0.012), 0.004, 0.004, GREEN, collide=False, axis="X")]
    # the back panel and the heat hood over the pan
    hz0, hz1 = HOOD_Z0 - CT, HOOD_Z1 - CT
    shapes.append(BoxShape((d - BACK_T / 2, 0.0, (b + hz0) / 2), (BACK_T, w, hz0 - b), STEEL))
    hx0 = HOOD_X0 - X0
    shapes.append(BoxShape(((hx0 + d) / 2, 0.0, (hz0 + hz1) / 2), (d - hx0, w, hz1 - hz0), STEEL))
    shapes.append(BoxShape((hx0 + 0.004, 0.0, (hz0 + hz1) / 2), (0.008, w + 0.002, hz1 - hz0 + 0.002), STEEL_D,
                           collide=False))  # the hood's front lip
    for s in (-1.0, 1.0):  # two calrod elements and their reflector, under the hood
        shapes.append(CylShape(((hx0 + d) / 2, s * 0.075, hz0 - 0.008), 0.0045, d - hx0 - 0.06, HEAT, collide=False,
                               axis="X"))
    shapes.append(BoxShape(((hx0 + d) / 2, 0.0, hz0 - 0.001), (d - hx0 - 0.03, w - 0.03, 0.002), STEEL_L,
                           collide=False))
    shapes.append(CylShape((hx0 + 0.09, 0.0, hz0 - 0.006), 0.022, 0.010, LAMP, collide=False))  # the halogen lamp
    shapes.append(CylShape((hx0 + 0.09, 0.0, hz0 - 0.002), 0.030, 0.004, STEEL_D, collide=False))
    for k in range(6):  # vents on the hood's top
        shapes.append(BoxShape((hx0 + 0.08 + 0.05 * k, 0.0, hz1 + 0.0005), (0.006, w - 0.08, 0.001), STEEL_D,
                               collide=False))
    for sx in (0.02, d - 0.02):  # rubber feet
        for sy in (-1.0, 1.0):
            shapes.append(CylShape((sx, sy * (w / 2 - 0.02), 0.0005), 0.008, 0.001, BLACK, collide=False))
    return Body.build("station", pos=(X0, YS, CT), mass=9.0, kinematic=True, color=STEEL, shapes=shapes,
                      friction=(STATION_MU, STATION_MU - 0.05))


def _sc_width(x: float) -> float:
    """The scoop's width inside at ``x`` from its funnel end: the straight spout, then the flare to the open end."""
    w = SC_W0 + (SC_W1 - SC_W0) * (np.asarray(x, float) - SC_SPOUT) / (SC_L - SC_SPOUT)
    w = np.where(np.asarray(x, float) <= SC_SPOUT, SC_W0, w)
    return float(w) if w.ndim == 0 else w


def _scoop_shapes() -> tuple[list, int]:
    """The scoop in its own frame: origin under the middle of the funnel end, x toward the open (wide) end. Returns the
    shapes and the index of the first handle shape (the handle has its own, grippier material)."""
    shapes = []
    n = 16
    for i in range(n):  # the flat bottom, in strips (each as wide as its middle; the walls hide the steps)
        xa, xb = SC_L * i / n, SC_L * (i + 1) / n
        shapes.append(BoxShape(((xa + xb) / 2, 0.0, SC_T / 2), (xb - xa + 0.0004, _sc_width((xa + xb) / 2) - 0.001,
                                                                 SC_T), SC_STEEL))
    for s in (-1.0, 1.0):  # the side walls: straight along the spout, then along the flare
        for xa, xb in ((0.0, SC_SPOUT), (SC_SPOUT, SC_L)):
            p1 = (xa, s * (_sc_width(xa) / 2), SC_H / 2)
            p2 = (xb, s * (_sc_width(xb) / 2), SC_H / 2)
            shapes.append(plate(p1, p2, SC_H, SC_WALL_T, (0.0, s, 0.0), SC_STEEL))
            q1 = (xa, s * (_sc_width(xa) / 2 + 0.001), SC_H - 0.0015)
            q2 = (xb, s * (_sc_width(xb) / 2 + 0.001), SC_H - 0.0015)
            shapes.append(plate(q1, q2, 0.004, 0.004, (0.0, s, 0.0), SC_EDGE, collide=False))  # rolled top edge
    # the handle: a steel strap off the right wall, then the black cool-touch grip, rising toward its end
    y0 = -(_sc_width(H_ROOT_X) / 2)
    first_handle = len(shapes)
    shapes.append(BoxShape((H_ROOT_X, y0 - 0.012, SC_H - 0.008), (0.018, 0.026, 0.003), STEEL))
    a = math.radians(H_RISE_DEG)
    d = np.array([0.0, -math.cos(a), math.sin(a)])
    g0 = np.array([H_ROOT_X, y0 - 0.020, SC_H - 0.004])
    g1 = g0 + d * H_LEN
    shapes.append(plate(tuple(g0.tolist()), tuple(g1.tolist()), H_SEC[0], H_SEC[1], (0.0, 0.0, 1.0), BLACK))
    shapes.append(CylShape(tuple((g1 + d * 0.004).tolist()), 0.004, 0.003, BLACK_L, collide=False, axis="Y"))
    return shapes, first_handle


_SCOOP_SHAPES, HANDLE_FIRST = _scoop_shapes()


def _scoop() -> Body:
    return Body.build("scoop", pos=(SC_X, SC_Y, PAN_FLOOR + 0.0005), mass=SC_MASS, color=SC_STEEL, shapes=_SCOOP_SHAPES,
                      friction=(SCOOP_MU, SCOOP_MU - 0.05), com=(0.116, -0.020, 0.028), solver_iterations=32)


def _carton_shapes() -> list:
    """The carton in its own frame: origin under the middle of its base, x toward its low (scooped) front, which faces
    the station; its taller back faces the robot."""
    hb, ht = CA_BOT_D / 2, CA_TOP_D / 2
    wb, wt = CA_BOT_W / 2, CA_TOP_W / 2
    shapes = [BoxShape((0.0, 0.0, CA_T / 2), (CA_BOT_D + CA_T, CA_BOT_W + CA_T, CA_T), RED)]
    # the back (taller, -x) and the front (+x): their slopes outward, in strips
    for sx, h in ((-1.0, CA_H_BACK), (1.0, CA_H_FRONT)):
        b = np.array([sx * hb, 0.0, CA_T])
        t = np.array([sx * (hb + (ht - hb) * h / CA_H_BACK), 0.0, h])
        for k in range(3):
            p, q = b + (t - b) * k / 3, b + (t - b) * (k + 1) / 3
            w = 2 * (wb + (wt - wb) * (k + 1) / 3 * h / CA_H_BACK)
            shapes.append(plate(tuple(p.tolist()), tuple(q.tolist()), w + CA_T, CA_T, (sx, 0.0, 0.0), RED))
    # the sides (gussets): full depth up to the front's height, then a top edge rising to the back's height
    def half_w(z):
        return wb + (wt - wb) * (z - CA_T) / (CA_H_BACK - CA_T)

    def half_d(z):
        return hb + (ht - hb) * (z - CA_T) / (CA_H_BACK - CA_T)

    xf_top = half_d(CA_H_FRONT)  # the front's top edge
    for sy in (-1.0, 1.0):
        for k in range(3):
            z0, z1 = CA_T + (CA_H_FRONT - CA_T) * k / 3, CA_T + (CA_H_FRONT - CA_T) * (k + 1) / 3
            shapes.append(plate((0.0, sy * half_w(z0), z0), (0.0, sy * half_w(z1), z1), 2 * half_d(z1) + CA_T, CA_T,
                                (0.0, sy, 0.0), RED))
        n_up = 4
        for k in range(n_up):
            z0 = CA_H_FRONT + (CA_H_BACK - CA_H_FRONT) * k / n_up
            z1 = CA_H_FRONT + (CA_H_BACK - CA_H_FRONT) * (k + 1) / n_up
            zm = (z0 + z1) / 2
            x_back = -half_d(zm)
            x_edge = xf_top + (-half_d(CA_H_BACK) - xf_top) * (zm - CA_H_FRONT) / (CA_H_BACK - CA_H_FRONT)
            xc, depth = (x_back + x_edge) / 2, x_edge - x_back
            shapes.append(plate((xc, sy * half_w(z0), z0), (xc, sy * half_w(z1), z1), depth + CA_T, CA_T,
                                (0.0, sy, 0.0), RED))
    # the print on the back: a yellow band and a white panel (drawn, not collided)
    zb = CA_H_BACK * 0.55
    xb = -(hb + (ht - hb) * zb / CA_H_BACK) - CA_T / 2 - 0.0004
    shapes.append(BoxShape((xb, 0.0, zb), (0.0006, 2 * (wb + (wt - wb) * zb / CA_H_BACK) - 0.006, 0.016), YELLOW,
                           collide=False))
    shapes.append(BoxShape((xb, 0.0, zb + 0.022), (0.0006, 0.040, 0.014), WHITE, collide=False))
    return shapes


def _carton() -> Body:
    return Body.build("carton", pos=(CA_XY[0], CA_XY[1], CT + 0.0005), mass=CA_MASS, color=RED,
                      shapes=_carton_shapes(), friction=(1.0, 0.9), com=(0.0, 0.0, 0.040), solver_iterations=32)


def _rect_corners(cx, cy, length, width, yaw) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    u, v = np.array([c, s]) * length / 2, np.array([-s, c]) * width / 2
    p = np.array([cx, cy])
    return np.array([p + u + v, p + u - v, p - u - v, p - u + v])


def _rects_overlap(a: np.ndarray, b: np.ndarray) -> bool:
    """Separating-axis test for two convex quadrilaterals (corner arrays)."""
    for poly in (a, b):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


_FRY_LOOK: dict = {}  # each fry's length and colour, for its hull and look


def _fry_ring(x: float, k: float) -> list:
    """The fry's cross-section at ``x`` along it: a 9 mm square with its corners chamfered (an octagon), scaled by k."""
    h, c = FRY_S / 2, FRY_CH
    octa = ((h, h - c), (h - c, h), (c - h, h), (-h, h - c), (-h, c - h), (c - h, -h), (h - c, -h), (h, c - h))
    return [(x, y * k, z * k) for y, z in octa]


def _fry_points(length: float) -> list:
    """A fry's hull: the chamfered square section along its length, and its two ends chamfered as well."""
    h, c = FRY_S / 2, FRY_CH
    k = (h - c) / h
    return (_fry_ring(-length / 2, k) + _fry_ring(-length / 2 + c, 1.0) + _fry_ring(length / 2 - c, 1.0)
            + _fry_ring(length / 2, k))


def _fry_volume(length: float) -> float:
    """The hull's volume (m3): the chamfered section along the middle, a frustum at each end."""
    a = FRY_S ** 2 - 2.0 * FRY_CH ** 2
    k = (FRY_S / 2 - FRY_CH) / (FRY_S / 2)
    return a * (length - 2 * FRY_CH) + 2 * FRY_CH / 3.0 * a * (1.0 + k + k * k)


def _fry_mass(length: float) -> float:
    return FRY_RHO * _fry_volume(length)


def _author_fry(stage, prim_path: str, spec) -> None:
    """The fry's collider and look: one convex hull (a chamfered stick), friction as the body's own material."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, Vt

    from yam_arena.station import linear_rgb

    length, col = _FRY_LOOK[spec.name]
    pts = _fry_points(length)
    faces, counts = [], []
    for r in range(3):  # the sides between the four rings
        for j in range(8):
            a, b = r * 8 + j, r * 8 + (j + 1) % 8
            faces += [a, b, b + 8, a + 8]
            counts.append(4)
    faces += list(range(7, -1, -1)) + list(range(24, 32))
    counts += [8, 8]
    path = f"{prim_path}/body/hull"
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    look = f"{prim_path}/Looks/fry"
    surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.55)
    surface.func(look, surface)
    bind_visual_material(path, look, stage=stage)
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    coll = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    coll.CreateContactOffsetAttr(float(spec.contact_offset))
    coll.CreateRestOffsetAttr(0.0)
    bind_physics_material(path, f"{prim_path}/Looks/physics", stage=stage)


def _heap() -> list[Body]:
    """The fries as dumped from a fryer basket: a jumbled mound from the pan's back wall to just behind the empty scoop.
    They are spawned lying flat at random angles in layers (no two touching), each layer a little smaller, and settle
    onto each other while the trial is set up. The pattern is fixed (seeded): the same heap every trial."""
    out = []
    rng = np.random.default_rng(20261116)
    k_len = 0
    margin = FRY_S + 0.004  # inflated width for the no-touch test

    def add(cx, cy, z, yaw, length):
        i = len(out)
        col = FRY_COLS[i % 3]
        out.append(Body.build(f"fry_{i:03d}", pos=(cx, cy, z), mass=_fry_mass(length), yaw=yaw, color=col,
                              shapes=[BoxShape((0.0, 0.0, 0.0), (length, FRY_S, FRY_S), None, collide=False)],
                              friction=(FRY_MU, FRY_MU - 0.05), contact_offset=0.0015))
        _FRY_LOOK[f"fry_{i:03d}"] = (length, col)

    for layer, target in enumerate(HEAP_PER_LAYER):
        x0, x1 = HEAP_X0 + layer * HEAP_SHRINK[0], HEAP_X1
        y0, y1 = HEAP_Y0 + layer * HEAP_SHRINK[1], HEAP_Y1 - layer * HEAP_SHRINK[1]
        z = PAN_FLOOR + 0.0015 + FRY_S / 2 + layer * (FRY_S + 0.0040)
        placed: list[np.ndarray] = []
        tries = 0
        while len(placed) < target and tries < 20000:
            tries += 1
            length = FRY_LENS[k_len % len(FRY_LENS)]
            cx, cy = float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1))
            yaw = float(rng.uniform(0.0, math.pi))
            corners = _rect_corners(cx, cy, length, FRY_S, yaw)
            if (corners[:, 0].min() < max(x0 - 0.02, HEAP_X0) or corners[:, 0].max() > PAN_X0 + PAN_IN_L - 0.006
                    or corners[:, 1].min() < YS - PAN_IN_W / 2 + 0.006 or corners[:, 1].max() > YS + PAN_IN_W / 2 - 0.006):
                continue
            fat = _rect_corners(cx, cy, length + 0.004, margin, yaw)
            if any(_rects_overlap(fat, q) for q in placed):
                continue
            placed.append(fat)
            k_len += 1
            add(cx, cy, z, yaw, length)
    return out


FRIES = _heap()
OBJECTS = [_counter(), _station(), _scoop(), _carton()] + FRIES
N_FRIES = len(FRIES)


# -- the scoop's handle gets its own material (black phenolic, grippy), after the body is authored --------------------
_author_plain = bodykit._author_sliders


_PATHS: dict = {}


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_plain(stage, prim_path, spec)
    _PATHS[spec.name] = f"{prim_path}/body"
    if spec.name.startswith("fry_"):  # a rigid-contact view on a loose body needs its contacts reported
        from pxr import PhysxSchema

        _author_fry(stage, prim_path, spec)
        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/body"))
        api.CreateThresholdAttr(0.0)
    if spec.name != "scoop":
        return
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material

    path = f"{prim_path}/Looks/handle_physics"
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=1.1, dynamic_friction=1.0)
    mat.func(path, mat)
    for i in range(HANDLE_FIRST, len(spec.shapes)):
        if spec.shapes[i].collide:
            bind_physics_material(f"{prim_path}/body/s{i}", path, stage=stage)


bodykit._author_sliders = _author_sliders


# -- geometry the scorer and the demo share ------------------------------------------------------------------------------

def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _ca_half_w(z: float) -> float:
    """The carton's half width (its walls' middle) at height z above its base."""
    return CA_BOT_W / 2 + (CA_TOP_W - CA_BOT_W) / 2 * (min(max(z, CA_T), CA_H_BACK) - CA_T) / (CA_H_BACK - CA_T)


def _ca_half_d(z: float) -> float:
    return CA_BOT_D / 2 + (CA_TOP_D - CA_BOT_D) / 2 * (min(max(z, CA_T), CA_H_BACK) - CA_T) / (CA_H_BACK - CA_T)


H_RISE = math.radians(H_RISE_DEG)
H_D = np.array([0.0, -math.cos(H_RISE), math.sin(H_RISE)])  # along the handle, toward its end (scoop frame)
H_G0 = np.array([H_ROOT_X, -_sc_width(H_ROOT_X) / 2 - 0.020, SC_H - 0.004])  # the handle's root, on its centre line


def _handle_point(s: float) -> np.ndarray:
    return H_G0 + H_D * s


RIM = CT + ST_BASE  # the pan's rim (and the base's top deck)

# -- scoring --------------------------------------------------------------------------------------------------------
# the portion, counted in fries: a small order is 110 g, 22 fries at 5 g a fry (USDA FNDDS 71401030); 18-28 fries are
# 90-140 g of real fries
PORTION_MIN, PORTION_MAX = 18, 28
PARTIAL_MIN = 6
SPILL_MAX = 4
STAND_TILT, STAND_GAP = 15.0, 0.005
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0
FRY_MASS = np.array([f.mass for f in FRIES])

_V = {"bodies": None, "contacts": None, "failed": "", "links": [], "dt": 1.0 / 240.0}


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy())
    return as_numpy(a)


def _views():
    """Batched PhysX views, made once: every fry's pose, and the fries' contacts with every link of both arms."""
    if _V["bodies"] is None and not _V["failed"] and FRIES[0].name in _PATHS:
        try:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths

            sv = PhysxManager.get_physics_sim_view()
            paths = [_PATHS[f.name] for f in FRIES]
            env = paths[0].rsplit("/", 2)[0]
            links = [f"{env}/{arm}{p}" for arm in ("LeftArm", "RightArm") for p in asset_link_paths().values()]
            _V["links"] = links
            _V["bodies"] = sv.create_rigid_body_view(paths)
            _V["contacts"] = sv.create_rigid_contact_view(paths, filter_patterns=[links] * len(paths),
                                                          max_contact_data_count=4096)
        except Exception as e:  # noqa: BLE001
            _V["failed"] = repr(e)[:300]
    return _V["bodies"]


def _fresh() -> dict:
    return {"steps": 0, "t0": None, "violation": "", "touched": np.zeros(len(FRIES), bool),
            "carried": np.zeros(len(FRIES), bool), "excluded": None, "s": {}, "tcps": None, "grip0": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("FRIES_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, bodykit.rotation(p[3:7])


def _tilt(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def _grippers(base, origin) -> dict:
    """Each arm's TCP (position, rotation) and its finger links' positions."""
    out = {}
    for arm, side in (("left_robot", "left"), ("right_robot", "right")):
        robot = base.scene[arm]
        names = list(robot.body_names)
        poses = as_numpy(robot.data.body_link_pose_w)[0]

        def find(key):
            return next(j for j, n in enumerate(names) if n == key or n.endswith("/" + key))

        g = poses[find("gripper")]
        rg = bodykit.rotation(g[3:7])
        tcp = g[:3] - origin + rg @ np.array(TCP_OFFSET)
        tips = [poses[find(k)][:3] - origin for k in ("tip_left", "tip_right")]
        out[side] = (tcp, rg, tips)
    return out


def _carton_points(pc, rc) -> np.ndarray:
    pts = []
    for z, hw, hd in ((0.0, CA_BOT_W / 2, CA_BOT_D / 2), (CA_H_FRONT, _ca_half_w(CA_H_FRONT), _ca_half_d(CA_H_FRONT)),
                      (CA_H_BACK, CA_TOP_W / 2, CA_TOP_D / 2)):
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                pts.append(pc + rc @ np.array([sx * hd, sy * hw, z]))
    return np.array(pts)


def _in_carton(q: np.ndarray) -> bool:
    """A fry's centre (carton frame) inside the carton's walls, at most 2 cm above its back rim (mounded fries)."""
    z = float(q[2])
    if not (CA_T * 0.5 <= z <= CA_H_BACK + 0.020):
        return False
    return abs(float(q[1])) <= _ca_half_w(z) + 0.001 and abs(float(q[0])) <= _ca_half_d(z) + 0.001


def _in_pan(p: np.ndarray) -> bool:
    return (PAN_X0 - 0.004 <= p[0] <= PAN_X0 + PAN_IN_L + 0.004 and abs(p[1] - YS) <= PAN_IN_W / 2 + 0.004
            and p[2] <= RIM + 0.10)


def _on_scoop(fp: np.ndarray, ps: np.ndarray, rs: np.ndarray) -> np.ndarray:
    """Which fries (centres, world) lie in the scoop's bowl: between its walls, from its funnel end to its open end, at
    most 3 cm above its walls' top edges (a heaped scoopful)."""
    q = (fp - ps) @ rs
    x = np.clip(q[:, 0], 0.0, SC_L)
    return ((q[:, 0] >= 0.0) & (q[:, 0] <= SC_L) & (np.abs(q[:, 1]) <= _sc_width(x) / 2) & (q[:, 2] >= -0.005)
            & (q[:, 2] <= SC_H + 0.030))


_SC_XS = sorted({float(v) for v in np.linspace(0.0, SC_L, 8)} | {SC_SPOUT})
_SCOOP_PTS = np.array([[x, s * _sc_width(x) / 2, z] for x in _SC_XS for s in (-1.0, 1.0)
                       for z in (0.0, SC_H / 2, SC_H)] + [[0.0, y, 0.0] for y in np.linspace(-SC_W0 / 2, SC_W0 / 2, 5)]
                      + [list(_handle_point(t)) for t in np.linspace(0.0, H_LEN, 5)])


def _hand_points(tcp, rot, tips) -> np.ndarray:
    """Points along both fingers of a gripper (from 13 mm past the TCP to 80 mm behind it), for clearances."""
    xa, ap = rot[:, 0], rot[:, 2]
    pts = [tcp]
    for tip in tips:
        off = float(np.dot(np.asarray(tip) - tcp, xa))
        for u in (-0.013, 0.0, 0.02, 0.04, 0.06, 0.08):
            pts.append(tcp + xa * off - ap * u)
    return np.array(pts)


@on_step
def _follow(env) -> None:
    import time

    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    if _EP["t0"] is None:
        _EP["t0"] = time.time()
    view = _views()
    if view is not None:
        fp = _np(view.get_transforms())[:, :3] - origin
    else:
        fp = np.array([_pose(base, origin, f.name)[0] for f in FRIES])
    # only the scoop may handle the fries: a fry any robot link touches is "handled" (it may not be served)
    if _V["contacts"] is not None:
        try:
            m = _np(_V["contacts"].get_contact_force_matrix(dt=_V["dt"])).reshape(len(FRIES), -1, 3)
            hit = np.linalg.norm(m, axis=2).max(axis=1) > 0.05
            if (hit & ~_EP["touched"]).any() and _DEBUG["on"]:
                k = int(np.argmax(hit & ~_EP["touched"]))
                j = int(np.argmax(np.linalg.norm(m[k], axis=1)))
                print(f"[fries] {'/'.join(_V['links'][j].rsplit('/', 3)[-3:])} touched {FRIES[k].name}", file=sys.stderr)
            _EP["touched"] |= hit
        except Exception as e:  # noqa: BLE001
            _V["failed"] = _V["failed"] or repr(e)[:300]
    pc, rc = _pose(base, origin, "carton")
    ps, rs = _pose(base, origin, "scoop")
    # the carton may not go into the fries (the scoop is the station's only utensil)
    cpts = _carton_points(pc, rc)
    dipped = [q for q in cpts if PAN_X0 <= q[0] <= PAN_X0 + PAN_IN_L and abs(q[1] - YS) <= PAN_IN_W / 2
              and q[2] < RIM - 0.002]
    if dipped and not _EP["violation"]:
        _EP["violation"] = "the carton went into the pan"
    # the end state
    in_carton = np.array([_in_carton(rc.T @ (p - pc)) for p in fp])
    in_pan = np.array([_in_pan(p) for p in fp])
    on_scoop = _on_scoop(fp, ps, rs)
    if _EP["excluded"] is None:  # nothing that starts in the scoop or the carton can ever count (there is none)
        _EP["excluded"] = on_scoop | in_carton
    # a fry is served only if the scoop carried it up out of the pan: in the scoop's bowl, above the pan's rim
    _EP["carried"] |= on_scoop & (fp[:, 2] > RIM + 0.005) & ~_EP["excluded"]
    handled = _EP["touched"]
    served = in_carton & _EP["carried"] & ~handled
    # a fry is wasted (spilled) outside both the pan and the carton, if the robot touched it, or if it is in the carton
    # without having been carried there by the scoop
    spilled = int(((~in_carton & ~in_pan) | (handled & ~in_carton) | (in_carton & ~served & ~handled)).sum())
    grams = float(FRY_MASS[served].sum())
    standing = (_tilt(rc) <= STAND_TILT and abs(float(pc[2]) - CT) <= STAND_GAP and not
                (X0 - 0.002 <= pc[0] <= X0 + ST_D and abs(pc[1] - YS) <= ST_W / 2))
    lip = ps + rs @ np.array([SC_L, 0.0, 0.0])
    scoop_back = (_in_pan(ps) and _in_pan(lip) and _tilt(rs) <= 12.0 and float(ps[2]) <= PAN_FLOOR + 0.03)
    grips = _grippers(base, origin)
    _EP["tcps"] = [grips["left"][0], grips["right"][0]]
    _EP["s"] = {"grams": grams, "n_in": int(served.sum()), "spilled": spilled, "standing": standing,
                "handled_in": int((handled & in_carton).sum()),
                "scoop_back": scoop_back, "home": _arms_home()}
    if _DEBUG["on"] and _EP["steps"] in (1, 40):  # the settled heap: its extent and height profile (for the setup text)
        top = fp[:, 2] + FRY_S / 2 - PAN_FLOOR
        bands = np.linspace(PAN_X0, PAN_X0 + PAN_IN_L, 11)
        prof = [f"{int(round(1000 * top[(fp[:, 0] >= a) & (fp[:, 0] < b)].max())) if ((fp[:, 0] >= a) & (fp[:, 0] < b)).any() else 0}"
                for a, b in zip(bands[:-1], bands[1:])]
        print(f"[fries] heap n={len(fp)} mass={FRY_MASS.sum() * 1000:.0f}g x={fp[:, 0].min():.3f}..{fp[:, 0].max():.3f} "
              f"y={fp[:, 1].min():.3f}..{fp[:, 1].max():.3f} top_mm(by 5 cm from the pan's front)={','.join(prof)} "
              f"in_pan={int(np.sum([_in_pan(p) for p in fp]))}", file=sys.stderr, flush=True)
    if _DEBUG["on"] and _EP["steps"] % 40 == 0:  # where the wasted fries lie (not on the scoop)
        lost = np.where(((~in_carton & ~in_pan) | (in_carton & ~served)) & ~on_scoop)[0]
        print(f"[fries] t={_EP['steps'] / 20:5.2f}s lost=" + " ".join(
            f"{FRIES[k].name[4:]}({1000 * fp[k, 0]:.0f},{1000 * fp[k, 1]:.0f},{1000 * (fp[k, 2] - CT):.0f})" for k in lost),
            file=sys.stderr, flush=True)
    if _DEBUG["on"] and _EP["steps"] % 5 == 0:
        mm = lambda v: np.round(np.asarray(v) * 1000).astype(int).tolist()  # noqa: E731
        tl, rl, tips_l = grips["left"]
        tr, rr, tips_r = grips["right"]
        # the right grip on the handle: how far the scoop has turned and slid in the fingers since it was taken
        rel_r, rel_p = rs.T @ rr, rs.T @ (tr - ps)
        held = float(np.linalg.norm(tr - (ps + rs @ _handle_point(0.05)))) < 0.03
        if held and _EP["grip0"] is None:
            _EP["grip0"] = (rel_r, rel_p)
        slip = ""
        if held and _EP["grip0"] is not None:
            r0, p0 = _EP["grip0"]
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(r0.T @ rel_r)) - 1.0) / 2.0))))
            slip = f"slip={ang:4.1f}deg/{float(np.linalg.norm(rel_p - p0)) * 1000:4.1f}mm "
        # how close the left hand comes to the scoop and to the right hand
        lpts = _hand_points(tl, rl, tips_l)
        spts = np.array([ps + rs @ q for q in _SCOOP_PTS])
        d_scoop = float(np.min(np.linalg.norm(lpts[:, None, :] - spts[None, :, :], axis=2)))
        d_hands = float(np.min(np.linalg.norm(lpts[:, None, :] - _hand_points(tr, rr, tips_r)[None, :, :], axis=2)))
        ry = math.degrees(math.atan2(float(rs[0, 2]), float(rs[2, 2])))
        print(f"[fries] t={_EP['steps'] / 20:5.2f}s wall={time.time() - _EP['t0']:5.1f} on_scoop={int(on_scoop.sum())}/{FRY_MASS[on_scoop].sum() * 1000:.0f}g "
              f"carried={int(_EP['carried'].sum())} in_carton={int(in_carton.sum())} served={int(served.sum())}/{grams * 1000:.0f}g spilled={spilled} touched={int(_EP['touched'].sum())} "
              f"handled_in={int((handled & in_carton).sum())} "
              f"viol='{_EP['violation']}' carton={mm(pc - [CA_XY[0], CA_XY[1], CT])} c_tilt={_tilt(rc):4.1f} "
              f"stand={standing} scoop={mm(ps - [SC_X, SC_Y, PAN_FLOOR])} s_pitch={ry:5.1f} s_tilt={_tilt(rs):4.1f} "
              f"s_yaw={math.degrees(math.atan2(float(rs[1, 0]), float(rs[0, 0]))):5.1f} "
              f"back={scoop_back} {slip}L-scoop={d_scoop * 1000:.0f}mm L-R={d_hands * 1000:.0f}mm "
              f"TL={mm(tl - [0, 0, TOP])} TR={mm(tr - [0, 0, TOP])} home={_EP['s']['home']} score={score(None)} "
              f"vf='{_V['failed'][:80]}'", file=sys.stderr, flush=True)


def score(objects) -> float:
    """0.0 if the carton went into the pan (latched), or if a fry a robot link has touched is in the carton at the end.
    Only fries the scoop carried up out of the pan count as served. At the end: 1.0 with 18-28 served fries in the
    carton, the carton standing on the counter, at most 4 fries spilled (outside both the pan and the carton, touched by
    the robot, or in the carton without having been carried there on the scoop), the scoop back in the pan and both arms
    home; 0.5 with 18-28 served fries in the standing carton; 0.25 with at least 6 in the standing carton; else 0."""
    s = _EP["s"]
    if not s or _EP["violation"] or s["handled_in"]:
        return 0.0
    if s["standing"] and PORTION_MIN <= s["n_in"] <= PORTION_MAX:
        if s["spilled"] <= SPILL_MAX and s["scoop_back"] and s["home"]:
            return 1.0
        return 0.5
    if s["standing"] and s["n_in"] >= PARTIAL_MIN:
        return 0.25
    return 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the carton, the scoop and every fry at rest: the trial ends there."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ["carton", "scoop"] + [f.name for f in FRIES])


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand pinches the empty carton low at its left end, across its depth, its wrist leaned out to the left, and
# holds it standing on the counter. The right takes the scoop's handle from its end, fingers above and below it. It tips
# the scoop lip-down onto the pan's floor and pushes it along the floor into the heap: the heap packs against the pan's
# back wall and the fries ride up over the scoop's thin lip. It levels the scoop, draws it back along the floor, tips the
# lip up and lifts the scoopful out over the pan's front rim, brings the funnel end over the carton's low front and tips
# the scoop up until the fries slide out through the funnel into the carton. The right lays the scoop back in the pan,
# lets go and goes home; the left lets go of the standing carton and goes home.

GRIP_S = float(os.environ.get("FRIES_GRIP_S", "0.05"))  # where along the handle the right hand holds it
HANDLE_OPEN = -0.019  # fingers open round the 22 mm handle
CA_LEAN = float(os.environ.get("FRIES_CA_LEAN", "-55"))  # the carton grip, leaned out to the left (deg)
CA_GRIP_Y, CA_GRIP_Z = 0.040, 0.045  # the carton grip: toward its left end, low (carton frame)
CA_OPEN = -0.04695
CA_CLOSE = float(os.environ.get("FRIES_CA_CLOSE", f"{-(0.017 + 0.0115 * (0.045 - 0.003) / 0.111 + 0.0015 - 0.003):.4f}"))
CA_PRESS = 0.002
DIG_DEG, PUSH, PRESS = float(os.environ.get("FRIES_DIG_DEG", "8")), float(os.environ.get("FRIES_PUSH", "0.30")), 0.003
PUSH_S = float(os.environ.get("FRIES_PUSH_S", "6.0"))  # how long the push into the heap takes
LIFT_DEG = float(os.environ.get("FRIES_LIFT_DEG", "-8"))  # the lip raised through the heap, the funnel end down
SCOOP_ONLY = bool(os.environ.get("FRIES_SCOOP_ONLY"))  # test: stop after the first scoopful is lifted out
PULL = float(os.environ.get("FRIES_PULL", "0.0"))  # drawn back this much while the lip comes up
DRAG = float(os.environ.get("FRIES_DRAG", "0.08"))  # the full scoop drawn back along the floor, out of the heap
DRAG_S = float(os.environ.get("FRIES_DRAG_S", "3.0"))
SCOOP_YS = tuple(float(v) for v in os.environ.get("FRIES_SCOOP_YS", "0.0").split(","))  # one per scoopful
CARRY_DEG = LIFT_DEG  # carried lip-up
POUR_DX, POUR_DZ = float(os.environ.get("FRIES_POUR_DX", "0.010")), float(os.environ.get("FRIES_POUR_DZ", "0.015"))
BACK_DX = float(os.environ.get("FRIES_BACK_DX", "0.025"))
CARRY_S = float(os.environ.get("FRIES_CARRY_S", "3.5"))  # carrying the scoopful from the pan to the carton  # where the scoop is laid back, behind its start
POUR_DY = float(os.environ.get("FRIES_POUR_DY", "-0.006"))
POUR_DEGS = tuple(float(v) for v in os.environ.get("FRIES_POUR_DEGS", "-25,-40,-55,-65").split(","))
SHAKE_DEG = float(os.environ.get("FRIES_SHAKE", "8"))
N_SHAKES = int(os.environ.get("FRIES_N_SHAKES", "3"))
RETREAT = float(os.environ.get("FRIES_RETREAT", "0.0"))  # the tipped scoop drawn back toward the station
LIFT_OFF = float(os.environ.get("FRIES_LIFT_OFF", "0.060"))  # then lifted straight up before it is levelled
LEAD_S, LEAD_STEPS = 3.0, 15


def _r_handle() -> np.ndarray:
    """The right TCP's rotation on the handle with the scoop level: pointing along the handle toward the scoop (+y,
    20 degrees down), fingers closing across the handle (above and below it)."""
    xc, zc = np.array([0.0, 0.0, -1.0]), np.array([0.0, 1.0, 0.0])
    return _rx(-H_RISE_DEG) @ np.column_stack([xc, np.cross(zc, xc), zc])


def _r_carton() -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return _rx(CA_LEAN) @ np.asarray(grip_rotation(math.pi / 2))  # closing across the carton's depth (world x)


SEEDS8 = [(-0.7, 1.0, 0.8, -0.5, 0.0, 0.0), (0.0, 1.0, 0.8, -0.5, 0.0, 0.0), (0.7, 1.0, 0.8, -0.5, 0.0, 0.0),
          (0.0, 1.8, 1.6, -1.2, 0.0, 0.0), (-0.7, 1.8, 1.6, -0.5, 0.5, 1.0), (0.7, 1.8, 1.6, -0.5, -0.5, -1.0),
          (0.0, 1.0, 1.6, -1.2, 0.5, -1.0), (0.0, 1.8, 0.8, -0.5, -0.5, 1.0)]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    seeds = SEEDS8 + list(itertools.product((-0.7, 0.0, 0.7), (1.0, 1.8), (0.8, 1.6), (-1.2, -0.5), (-0.5, 0.0, 0.5),
                                            (-1.0, 0.0, 1.0)))
    for k, seed in enumerate(seeds):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
            if room > math.radians(15.0):
                break
        if best is not None and k >= len(SEEDS8):
            break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(secs / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _scoop_tcp(origin, rot, s=None):
    """The right TCP for the scoop at (origin, rot), held at ``s`` along its handle."""
    g = _handle_point(GRIP_S if s is None else s)
    return np.asarray(origin, float) + rot @ g, rot @ _r_handle()


def _carton_tcp(origin, rot):
    return np.asarray(origin, float) + rot @ np.array([0.0, CA_GRIP_Y, CA_GRIP_Z]), rot @ _r_carton()


def demo():
    if os.environ.get("FRIES_CARTON_SCOOP"):
        return _carton_scoop_plan()
    plan = _demo_plan()
    if os.environ.get("FRIES_ONE_HAND"):  # the check that the second hand is needed: the left never moves
        from yam_arena import ik, station
        from yam_arena.constants import READY

        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        plan["left"] = [Waypoint(sum(w.duration for w in plan["right"]), tuple(float(v) for v in p), FINGER_OPEN,
                                 "one-hand check: the left stays home", r)]
    return plan


def _carton_scoop_plan():
    """Adversarial check: one hand takes fries from the station with the carton itself, dipping it into the pan's front
    where the fries lie (the scorer must give 0). The right stays home."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    rl = _r_carton()
    c0 = np.array([CA_XY[0], CA_XY[1], CT])
    pl, _ = _carton_tcp(c0, np.eye(3))
    back = -rl[:, 2]
    steps = []

    def add(t, pos, rot, finger, label):
        steps.append(Waypoint(t, tuple(float(v) for v in pos), finger, label, rot))

    q = _good_ik("left", pl + back * 0.06 + [0.0, 0.0, 0.01], rl)
    steps += _joint_path("left", np.array(READY, float), q, "beside the carton", CA_OPEN)
    add(1.5, pl, rl, CA_OPEN, "round its left end")
    add(0.8, pl, rl, CA_CLOSE, "hold it")
    add(2.0, pl + [0.0, 0.0, 0.12], rl, CA_CLOSE, "lift it")
    r_dig = _ry(15.0) @ rl
    dig = np.array([0.440, 0.130, RIM + 0.005])  # the heap's front edge, as far as the carton grip reaches
    add(2.5, dig + [-0.04, 0.0, 0.035], r_dig, CA_CLOSE, "over the pan, mouth tipped forward")
    add(2.0, dig, r_dig, CA_CLOSE, "into the fries")
    add(2.0, dig + [0.01, 0.0, -0.003], r_dig, CA_CLOSE, "scoop them with the carton")
    add(1.5, dig + [-0.03, 0.0, 0.04], _ry(15.0) @ rl, CA_CLOSE, "lift it out, mouth up")
    add(3.0, pl + [0.0, 0.0, 0.06], rl, CA_CLOSE, "back over the counter")
    add(1.5, pl + [0.0, 0.0, -CA_PRESS], rl, CA_CLOSE, "set it down")
    add(0.8, pl, rl, CA_OPEN, "let go")
    add(0.8, pl + [0.0, 0.035, 0.0], rl, CA_OPEN, "off its end")
    add(1.0, pl + [0.0, 0.035, 0.0] + back * 0.05, rl, CA_OPEN, "back away")
    steps += _joint_path("left", _good_ik("left", pl + [0.0, 0.035, 0.0] + back * 0.05, rl), np.array(READY, float),
                         "home", CA_OPEN)
    p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    steps.append(Waypoint(3.0, tuple(float(v) for v in p), CA_OPEN, "hold", r))
    pr, rh = ik.tcp_pose(np.array(READY, float), station.BASES["right"])
    return {"left": steps, "right": [Waypoint(sum(w.duration for w in steps), tuple(float(v) for v in pr), FINGER_OPEN,
                                              "the right stays home", rh)]}


def _demo_plan():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}

    def put(arm, t, pos, rot, finger, label, n=1):
        p0, _f, r0 = cur[arm]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            a = j / n
            r = rot if n == 1 else _slerp(r0, rot, a)
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * a), finger, label, r))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        if t <= 1e-6:
            return
        if not plan[arm]:
            from yam_arena import ik, station

            p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), FINGER_OPEN, label, r))
            return
        last = plan[arm][-1]
        plan[arm].append(Waypoint(t, last.pos, last.finger, label, last.yaw))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def sync(label="wait for the other hand"):
        t = max(dur("left"), dur("right"))
        for arm in ("left", "right"):
            wait(arm, t - dur(arm), label)

    def from_home(arm, pos, rot, finger, label):
        plan[arm] += _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)
        cur[arm] = (np.asarray(pos, float), finger, rot)

    def to_home(arm, label="back to where it started"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), label, f)

    def scoop_at(t, origin, rot, finger, label, n=1, s=None):
        p, r = _scoop_tcp(origin, rot, s)
        put("right", t, p, r, finger, label, n)

    def scoop_turn(t, pivot_local, origin0, rot0, rot1, finger, label, n=6):
        """Turn the held scoop from rot0 to rot1 about a point fixed in the scoop (``pivot_local``)."""
        piv = np.asarray(origin0, float) + rot0 @ np.asarray(pivot_local, float)
        for j in range(1, n + 1):
            r = _slerp(rot0, rot1, j / n)
            o = piv - r @ np.asarray(pivot_local, float)
            scoop_at(t / n, o, r, finger, label)
        return piv - rot1 @ np.asarray(pivot_local, float)

    L, R = "left", "right"
    # -- 1. the left takes the carton low at its left end (wrist out to the left); the right takes the scoop's handle -
    c0 = np.array([CA_XY[0], CA_XY[1], CT])
    pl, rl = _carton_tcp(c0, np.eye(3))
    back_l = -rl[:, 2]  # back along the left's approach
    from_home(L, pl + back_l * 0.06 + [0.0, 0.0, 0.01], rl, CA_OPEN, "beside the carton's left end")
    put(L, 1.5, pl, rl, CA_OPEN, "fingers either side of the carton's left end, low")
    put(L, 0.8, pl + [0.0, 0.0, -CA_PRESS], rl, CA_CLOSE, "hold it gently, standing on the counter")
    s0 = np.array([SC_X, SC_Y, PAN_FLOOR])
    p_end, r_h = _scoop_tcp(s0, np.eye(3), s=H_LEN + 0.03)
    from_home(R, p_end, r_h, HANDLE_OPEN, "in line with the handle, beyond its end")
    scoop_at(1.5, s0, np.eye(3), HANDLE_OPEN, "fingers over and under the handle, along it to its root")
    scoop_at(0.8, s0, np.eye(3), FINGER_CLOSED, "grip the handle")
    lip_local = np.array([SC_L, 0.0, 0.0])
    pour = np.array([c0[0] + POUR_DX, c0[1] + POUR_DY, CT + CA_H_FRONT + POUR_DZ])
    o, r_cur = s0, np.eye(3)
    for k_scoop, dy in enumerate(SCOOP_YS):
        start = s0 + [0.0, dy, 0.0]
        if k_scoop:  # -- back into the pan's front for another scoopful ------------------------------------------
            scoop_at(2.0, start + [0.0, 0.0, RIM + 0.02 - PAN_FLOOR], np.eye(3), FINGER_CLOSED,
                     "back over the pan's front", n=2)
            scoop_at(1.0, start + [0.0, 0.0, 0.002], np.eye(3), FINGER_CLOSED, "down onto the pan's floor")
        # -- 2. scoop: lip down, along the floor into the heap, level it, raise the lip, lift the scoopful out -----
        o = scoop_turn(1.2, lip_local, start, np.eye(3), _ry(DIG_DEG), FINGER_CLOSED, "tip it lip-down onto the floor")
        o = o + [0.0, 0.0, -PRESS]
        scoop_at(0.4, o, _ry(DIG_DEG), FINGER_CLOSED, "press the lip to the floor")
        o = o + [PUSH, 0.0, 0.0]
        scoop_at(PUSH_S, o, _ry(DIG_DEG), FINGER_CLOSED, "push it along the floor into the heap", n=max(3, int(PUSH_S)))
        o = scoop_turn(1.0, lip_local, o, _ry(DIG_DEG), np.eye(3), FINGER_CLOSED, "level it, lip still on the floor")
        if DRAG > 0:
            o = o + [-DRAG, 0.0, 0.0]
            scoop_at(DRAG_S, o, np.eye(3), FINGER_CLOSED, "draw the full scoop back out of the heap, along the floor",
                     n=max(2, int(DRAG_S)))
        o = scoop_turn(1.2, np.zeros(3), o + [-PULL, 0.0, 0.0], np.eye(3), _ry(LIFT_DEG), FINGER_CLOSED,
                       "draw it back a little, raising the lip under the fries")
        scoop_at(1.0, o + [0.0, 0.0, 0.015], _ry(LIFT_DEG), FINGER_CLOSED, "lift the scoopful gently")
        o = o + [0.0, 0.0, RIM + 0.025 - o[2]]
        scoop_at(1.5, o, _ry(CARRY_DEG), FINGER_CLOSED, "lift it clear of the pan's rim")
        if SCOOP_ONLY:
            scoop_at(3.0, o, _ry(CARRY_DEG), FINGER_CLOSED, "hold the scoopful up")
            plan["left"] = []
            wait(L, dur(R), "the left stays home (scoop test)")
            return plan
        # -- 3. the funnel end over the carton's mouth; tip the scoop up until the fries slide out ----------------
        if k_scoop == 0:
            sync()
        scoop_at(CARRY_S, pour + [0.0, 0.0, 0.04], _ry(CARRY_DEG), FINGER_CLOSED, "bring the funnel end over the carton",
                 n=3)
        scoop_at(1.0, pour, _ry(CARRY_DEG), FINGER_CLOSED, "lower it over the mouth")
        o, r_prev = pour, _ry(CARRY_DEG)
        for k, deg in enumerate(POUR_DEGS):
            o = scoop_turn(1.6, np.zeros(3), o, r_prev, _ry(deg), FINGER_CLOSED,
                           "tip the scoop up: the fries slide out through the funnel")
            r_prev = _ry(deg)
            scoop_at(0.8, o, r_prev, FINGER_CLOSED, "let them fall")
        for _ in range(N_SHAKES):  # a little shake frees the fries jammed across the funnel
            o = scoop_turn(0.35, np.zeros(3), o, r_prev, _ry(POUR_DEGS[-1] + SHAKE_DEG), FINGER_CLOSED, "shake")
            o = scoop_turn(0.35, np.zeros(3), o, _ry(POUR_DEGS[-1] + SHAKE_DEG), r_prev, FINGER_CLOSED, "shake")
        scoop_at(0.8, o, r_prev, FINGER_CLOSED, "let the last fall")
        o = o + [RETREAT, 0.0, LIFT_OFF]
        scoop_at(1.2, o, r_prev, FINGER_CLOSED, "lift the funnel end off the fries it rests on")
        scoop_at(0.5, o, r_prev, FINGER_CLOSED, "they drop into the carton")
        o = scoop_turn(1.5, np.zeros(3), o, r_prev, np.eye(3), FINGER_CLOSED, "level the scoop")
        o = o + [0.0, 0.0, 0.04]
        scoop_at(1.0, o, np.eye(3), FINGER_CLOSED, "lift it")
    sync()
    # -- 4. the scoop back in the pan; the carton let go, standing; both home -------------------------------------
    s_back = s0 + [BACK_DX, 0.0, 0.0]  # well inside the pan's front: the scoop turns a little in the grip while pushing
    scoop_at(2.5, s_back + [0.0, 0.0, RIM + 0.04 - PAN_FLOOR], np.eye(3), FINGER_CLOSED, "back over the pan's front", n=2)
    scoop_at(1.5, s_back + [0.0, 0.0, 0.002], np.eye(3), FINGER_CLOSED, "lay it in the pan")
    scoop_at(0.6, s_back + [0.0, 0.0, 0.002], np.eye(3), HANDLE_OPEN, "let go")
    scoop_at(1.2, s_back + [0.0, 0.0, 0.002], np.eye(3), HANDLE_OPEN, "out along the handle", s=H_LEN + 0.03)
    to_home(R)
    put(L, 0.8, pl, rl, CA_OPEN, "let go of the carton")
    wait(L, 0.4, "fingers open")
    put(L, 0.8, pl + [0.0, 0.035, 0.0], rl, CA_OPEN, "slide the fingers off the carton's end")
    put(L, 1.0, pl + [0.0, 0.035, 0.0] + back_l * 0.05, rl, CA_OPEN, "back away")
    to_home(L)
    sync("home")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, last.finger, "hold still: the trial ends once the full score has held 1 s",
                                  last.yaw))
    return plan


def _slerp(r0, r1, a):
    from yam_arena import ik

    return ik.slerp_matrix(np.asarray(r0, float), np.asarray(r1, float), a)
