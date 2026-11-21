"""Shut a full kebab box: one hand folds the lettuce leaf hanging over the front rim back inside and holds it there while the other swings the lid down;
the tuck hand slips out just before the lid's front wall arrives, and the lid hand presses the lid's front until its two tabs click under the base's flange.

A moulded bagasse 9 x 6 in clamshell (230 x 158 mm footprint, 46 mm base, 80 mm shut) stands on the counter, its hinge on the left edge (the back, +x in the box's frame), the lid
sprung open on its hinge (rest angle 85 degrees, about 73 sagged). The hinge springs the lid open (about 0.1 N up at the front edge when shut), so the lid stays shut only
when its tabs are caught. Each of the two tabs is a 20 mm wide strip hanging from the lid's front wall, sprung (a bend in the bagasse) and carrying a 2.5 mm barb on
its inner face; it passes down through a notch in the base's front flange, the barb's ramp rides over the notch's inner edge, and the barb's ledge
catches under the flange. In the base: about 350 g of doner kebab as rigid free bodies (nine meat portions, three lettuce pieces,
three tomato slices, two onion rings, two chilli peppers) and one lettuce leaf in two parts: its base lies on the heap under a meat strip, its tip is
hinged to it 10 mm inside the front rim and sprung (the leaf's own stiffness), sticking 23 mm out past the front wall (about 10 mm past the flange). Folded back in and let go, the tip
springs back out over the rim, where the lid's front traps it and holds the lid about 2 mm proud, so the barbs cannot reach under the flange.
Code: the scorer (`Trial.step`, `judge`). The hinge, tab, notch and leaf are bodies, joints and collisions; the code stands in for nothing in the act.

Debug:  python demokit.py task.py OUT --no-record --mode full,none,one_hand
Final:  python demokit.py task.py OUT
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Close the kebab box for takeaway. Tuck any salad hanging over the rim back inside, close the lid, and push its tabs into their slots so they lock."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
HOLD_STEPS = 40  # 2 s at 20 Hz
ENV = os.environ.get

# -- the box (BioJX 9 x 6 in bagasse clamshell: 230 x 158 mm, 46 mm base, 80 mm shut, 30 g) in its own frame: origin at the footprint's centre on the counter, front -x, hinge +x
CX, CY = float(ENV("KB_X", "0.32")), float(ENV("KB_Y", "0.0"))
YAW = 90.0  # the box is turned so that its front (-x in its frame) faces the right arm (-y) and its hinge lies along the left side
FX0, FX1, FY = -0.079, 0.079, 0.115  # footprint (flange outer edge): front, back (the hinge line), half width
WX0, WX1, WY = -0.067, 0.071, 0.107  # wall outer faces: front (12 mm flange in front of it), back, sides
T = 0.002  # wall, floor and flange thickness
TZ = 0.046  # flange top: the rim
LH = 0.034  # lid height: 80 mm shut
IX0, IX1, IY = WX0 + T, WX1 - T, WY - T  # inside the walls
LID_OPEN = float(ENV("KB_OPEN", "85.0"))  # the lid's hinge spring rest angle (joint zero, its stop); its weight sags it to about 73 degrees
LID_K = float(ENV("KB_LIDK", "0.015"))  # N m/rad, about the rest angle: 0.026 N m open at shut against 0.011 N m of the lid's weight
TAB_C = (-0.050, 0.050)  # the two tabs' centres along the front, as in the maker's photos (about a quarter of the way in from each end)
NOTCH_Y, NOTCH_X = 0.011, -0.071  # each notch in the base's front flange: half width, inner edge
TAB_Y, TAB_X0, TAB_X1, TAB_Z0, TAB_ZH = 0.010, -0.0735, -0.0720, 0.038, 0.074  # the tab strip (lid shut): half width, outer and inner face, bottom, hinge height
BARB_P, BARB_Z, BARB_H = 0.0025, 0.0435, 0.004  # barb: how far it stands in from the strip, its ledge's height, the ramp's height below it
TAB_K = float(ENV("KB_TABK", "1.5"))  # N m/rad, each tab: about 2.5 N across its barb to bend the strip 3 degrees out

# the leaf (one lettuce leaf in two parts): base on the heap, tip hinged to it 10 mm inside the front rim
LEAF_Y = float(ENV("KB_LEAFY", "0.0"))  # the leaf hangs over the front between the two tabs
LEAF_W = 0.022  # half width
PIV_X, PIV_Z = -0.055, 0.046  # the tip's hinge line
TIP_L = 0.035
TIP_UP = 20.0  # degrees the tip's rest pose rises from level; its weight sags it about 9 degrees
TIP_K = float(ENV("KB_TIPK", "0.003"))  # N m/rad

LOOKS = {"bagasse": m.Look(m.rgb(0.88, 0.85, 0.77), metallic=0.0, roughness=0.95), "meat": m.Look(m.rgb(0.47, 0.27, 0.14), metallic=0.0, roughness=0.75),
         "meat2": m.Look(m.rgb(0.36, 0.19, 0.09), metallic=0.0, roughness=0.75), "lettuce": m.Look(m.rgb(0.50, 0.74, 0.28), metallic=0.0, roughness=0.6),
         "leaf": m.Look(m.rgb(0.40, 0.68, 0.22), metallic=0.0, roughness=0.6), "tomato": m.Look(m.rgb(0.82, 0.16, 0.10), metallic=0.0, roughness=0.45),
         "onion": m.Look(m.rgb(0.92, 0.88, 0.86), metallic=0.0, roughness=0.5), "chilli": m.Look(m.rgb(0.62, 0.70, 0.20), metallic=0.0, roughness=0.4)}


def _rot(x, z, x0, z0, deg):
    """(x, z) turned `deg` about the line along y through (x0, z0), right-hand rule about +y: positive lifts points on the -x side."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return x0 + (x - x0) * c + (z - z0) * s, z0 - (x - x0) * s + (z - z0) * c


def _bx(x0, x1, y0, y1, z0, z1):
    return m.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def _about_y(mesh, deg, x0, z0):
    return m.Mesh(mesh.v - [x0, 0, z0], mesh.f).turned(deg, "y").moved([x0, 0, z0])


def _col_turned(name, b, turn):
    deg, x0, z0 = turn
    xa, xb, ya, yb, za, zb = b
    cx, cz = _rot((xa + xb) / 2, (za + zb) / 2, x0, z0, deg)
    return f"""
    def Cube "{name}" {m._COL_HEAD}
        double size = 1
        double3 xformOp:translate = ({cx:.6f}, {(ya + yb) / 2:.6f}, {cz:.6f})
        float xformOp:rotateY = {deg:.4f}
        float3 xformOp:scale = ({xb - xa:.6f}, {yb - ya:.6f}, {zb - za:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateY", "xformOp:scale"]
    }}"""


def _boxes_part(prefix, boxes, look, turn=None):
    parts, cols = [], []
    for i, b in enumerate(boxes):
        mesh = _bx(*b)
        if turn:
            mesh = _about_y(mesh, *turn)
        parts.append((f"{prefix}{i}", mesh, look))
        cols.append(_col_turned(f"{prefix}c{i}", b, turn) if turn else m.col_box(f"{prefix}c{i}", (b[1] - b[0], b[3] - b[2], b[5] - b[4]), ((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2)))
    return parts, cols


BASE = [(WX0, WX1, -WY, WY, 0.0, T),  # floor
        (WX0, WX0 + T, -WY, WY, 0.0, TZ), (WX1 - T, WX1, -WY, WY, 0.0, TZ), (WX0, WX1, -WY, -WY + T, 0.0, TZ), (WX0, WX1, WY - T, WY, 0.0, TZ),  # walls
        (FX0, WX0, -FY, TAB_C[0] - NOTCH_Y, TZ - T, TZ), (FX0, WX0, TAB_C[0] + NOTCH_Y, TAB_C[1] - NOTCH_Y, TZ - T, TZ), (FX0, WX0, TAB_C[1] + NOTCH_Y, FY, TZ - T, TZ),  # front flange, notched at each tab
        (NOTCH_X, WX0, TAB_C[0] - NOTCH_Y, TAB_C[0] + NOTCH_Y, TZ - T, TZ), (NOTCH_X, WX0, TAB_C[1] - NOTCH_Y, TAB_C[1] + NOTCH_Y, TZ - T, TZ),
        (WX1, FX1, -FY, FY, TZ - T, TZ), (FX0, FX1, -FY, -WY, TZ - T, TZ), (FX0, FX1, WY, FY, TZ - T, TZ)]  # back and side flanges

LID = [(WX0, WX1, -WY, WY, TZ + LH - T, TZ + LH),  # roof
       (WX0, WX0 + T, -WY, WY, TZ, TZ + LH - T), (WX1 - T, WX1, -WY, WY, TZ, TZ + LH - T), (WX0, WX1, -WY, -WY + T, TZ, TZ + LH - T), (WX0, WX1, WY - T, WY, TZ, TZ + LH - T),
       (FX0, WX0, -FY, TAB_C[0] - 0.012, TZ, TZ + T), (FX0, WX0, TAB_C[0] + 0.012, TAB_C[1] - 0.012, TZ, TZ + T), (FX0, WX0, TAB_C[1] + 0.012, FY, TZ, TZ + T),  # front flange, cut where the tabs hang
       (WX1, FX1, -FY, FY, TZ, TZ + T), (FX0, FX1, -FY, -WY, TZ, TZ + T), (FX0, FX1, WY, FY, TZ, TZ + T)]
LID_TURN = (LID_OPEN, FX1, TZ)


def _barb(yc):
    """The barb on a tab strip's inner face (lid shut): a ledge at BARB_Z, a ramp below it down to the strip."""
    p = m.prism([(BARB_Z, TAB_X1), (BARB_Z, TAB_X1 + BARB_P), (BARB_Z - BARB_H, TAB_X1)], yc - TAB_Y + 0.001, yc + TAB_Y - 0.001)  # outline (z, x), extruded along y
    return m.Mesh(np.c_[p.v[:, 1], p.v[:, 2], p.v[:, 0]], p.f)


def _box_part():
    base_parts, base_cols = _boxes_part("base", BASE, "bagasse")
    lid_parts, lid_cols = _boxes_part("lid", LID, "bagasse", LID_TURN)
    lid = m.hinge("lid", "Y", (FX1, 0.0, TZ), -LID_OPEN, 0.0, lid_parts, lid_cols, mass=0.014, spring=LID_K, damping=0.004, friction=0.0005, gravity=True, inertia=(1.2e-4, 3e-5, 1.2e-4))
    tabs = []
    for k, yc in enumerate(TAB_C):
        strip = (TAB_X0, TAB_X1, yc - TAB_Y, yc + TAB_Y, TAB_Z0, TAB_ZH + 0.002)
        bridge = (TAB_X0, WX0, yc - TAB_Y, yc + TAB_Y, TAB_ZH, TAB_ZH + 0.002)
        tab_parts, tab_cols = _boxes_part(f"tab{k}s", [strip], "bagasse", LID_TURN)
        tab_parts.append((f"bridge{k}", _about_y(_bx(*bridge), *LID_TURN), "bagasse"))
        barb = _about_y(_barb(yc), *LID_TURN)
        tab_parts.append((f"barb{k}", barb, "bagasse"))
        tab_cols.append(m.col_hull(f"barb{k}c", barb))
        hx, hz = _rot((TAB_X0 + TAB_X1) / 2, TAB_ZH, *LID_TURN[1:], LID_OPEN)
        tabs.append(m.hinge(f"tab{k}", "Y", (hx, yc, hz), -1.0, 12.0, tab_parts, tab_cols, mass=0.002, spring=TAB_K, damping=0.002, friction=0.0, inertia=(3e-6, 3e-6, 3e-6), parent="lid"))
    return m.mechanism("box", (CX, CY, TOP), [lid, *tabs], LOOKS, frame_parts=base_parts, frame_colliders=base_cols, friction=(0.5, 0.45), solver_iters=64, self_collisions=True, free_mass=0.016, yaw_deg=YAW,
                       frame_inertia=(7e-5, 3.5e-5, 1e-4))


ROOT = (PIV_X, PIV_X + 0.070, LEAF_Y - LEAF_W, LEAF_Y + LEAF_W, PIV_Z - T, PIV_Z)
TIP = (PIV_X - TIP_L, PIV_X, LEAF_Y - LEAF_W + 0.003, LEAF_Y + LEAF_W - 0.003, PIV_Z, PIV_Z + T)


def _sheet(b, seed, turn=None):
    """The leaf's look: an irregular wavy-edged sheet inside the box b (x0, x1, y0, y1, z0, z1); its collider stays the box b."""
    r = np.random.default_rng(seed)
    x0, x1, y0, y1, z0, z1 = b
    cx, cy, a, c = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
    ph = r.uniform(0, 2 * np.pi, 3)
    pts = []
    for t in np.linspace(0, 2 * np.pi, 40, endpoint=False):  # superellipse outline with a ruffled rim, never outside the box
        k = 0.82 + 0.06 * math.sin(5 * t + ph[0]) + 0.05 * math.sin(9 * t + ph[1]) + 0.03 * math.sin(14 * t + ph[2])
        e = 0.35
        px = a * k * np.sign(math.cos(t)) * abs(math.cos(t)) ** e
        py = c * k * np.sign(math.sin(t)) * abs(math.sin(t)) ** e
        pts.append((cx + min(max(px, -a), a), cy + min(max(py, -c), c)))
    mesh = m.prism(pts, z0, z1)
    return _about_y(mesh, *turn) if turn else mesh


def _leaf_part():
    root_parts, root_cols = _boxes_part("root", [ROOT], "leaf")
    tip_parts, tip_cols = _boxes_part("tip", [TIP], "leaf", (TIP_UP, PIV_X, PIV_Z))
    root_parts[0] = ("root0", _sheet(ROOT, 1), "leaf")
    tip_parts[0] = ("tip0", _sheet(TIP, 2, (TIP_UP, PIV_X, PIV_Z)), "leaf")
    tip = m.hinge("tip", "Y", (PIV_X, LEAF_Y, PIV_Z), -25.0, 150.0, tip_parts, tip_cols, mass=0.003, spring=TIP_K, damping=0.0005, friction=0.0, gravity=True, inertia=(1e-6, 1.5e-6, 1e-6))
    return m.mechanism("leaf", (CX, CY, TOP), [tip], LOOKS, frame_parts=root_parts, frame_colliders=root_cols, friction=(0.9, 0.8), solver_iters=32, free_mass=0.004, yaw_deg=YAW, frame_inertia=(1.5e-6, 2e-6, 3e-6))


# -- the food: rigid free bodies, each mesh built in the box's frame and the body placed at the box's origin ------------------------------------------------------------------------------

RNG = np.random.default_rng(7)


def _meat(cx, cy, z0, sx, sy, sz, n=5):
    """A portion of shaved doner meat: a heap of thin strips, one rigid body (its collider their hull)."""
    out = []
    for k in range(n):
        ly, wx = sy * RNG.uniform(0.75, 1.0), sx / n * 1.4
        x = cx - sx / 2 + (k + 0.5) * sx / n + RNG.uniform(-0.002, 0.002)
        h = sz * RNG.uniform(0.55, 1.0)
        b = m.box((wx, ly, h), (0, 0, 0)).turned(RNG.uniform(-3, 3), "z")
        out.append(b.moved([x, cy + RNG.uniform(-0.004, 0.004), z0 + sz - h / 2 - 0.0005]))
    return out


def _flat(cx, cy, z0, r, h, sides=9, jag=0.25):
    ang = np.sort(RNG.uniform(0, 2 * np.pi, sides))
    rr = r * (1 - jag * RNG.uniform(0, 1, sides))
    return m.prism([(cx + a * math.cos(t), cy + a * math.sin(t)) for a, t in zip(rr, ang)], z0, z0 + h)


def _ring(cx, cy, z0, r, w):
    pts = [(cx + r * math.cos(t), cy + r * math.sin(t), z0 + w / 2) for t in np.linspace(0, 2 * np.pi, 17)]
    return m.tube(pts, w / 2, sides=8)


def _chilli(cx, cy, z0, yaw):
    s = np.linspace(-0.03, 0.03, 7)
    pts = [(cx + a * math.cos(math.radians(yaw)) - 0.004 * (a / 0.03) ** 2 * math.sin(math.radians(yaw)), cy + a * math.sin(math.radians(yaw)) + 0.004 * (a / 0.03) ** 2 * math.cos(math.radians(yaw)), z0 + 0.006) for a in s]
    return m.tube(pts, 0.006, sides=10)


FOOD = []  # (name, meshes, look, mass)
for i, (x, y) in enumerate([(-0.033, -0.052), (0.035, -0.052), (-0.033, 0.052), (0.035, 0.052)]):
    FOOD.append((f"meat{i}", _meat(x, y, 0.0025, 0.054, 0.090, 0.016), "meat" if i % 2 else "meat2", 0.035))
for i, (x, y) in enumerate([(-0.028, -0.062), (0.028, -0.005), (-0.025, 0.060)]):
    FOOD.append((f"meat{4 + i}", _meat(x, y, 0.019, 0.052, 0.064, 0.014, n=4), "meat", 0.020))
FOOD.append(("lettuce0", [_flat(-0.030, LEAF_Y, 0.034, 0.030, 0.0095, jag=0.12)], "lettuce", 0.010))
FOOD.append(("lettuce1", [_flat(0.030, 0.050, 0.034, 0.028, 0.008)], "lettuce", 0.008))
FOOD.append(("lettuce2", [_flat(0.035, -0.055, 0.034, 0.026, 0.008)], "lettuce", 0.008))
for i, (x, y) in enumerate([(-0.035, 0.055), (-0.035, -0.060), (0.040, -0.005)]):
    FOOD.append((f"tomato{i}", [m.cylinder(0.022, 0.006, (x, y, 0.0365), segments=20)], "tomato", 0.012))
FOOD.append(("anchor", _meat(0.0, LEAF_Y, PIV_Z + 0.0005, 0.022, 0.060, 0.012, n=2), "meat", 0.030))
FOOD.append(("behind", _meat(0.0425, LEAF_Y, PIV_Z - 0.0025, 0.042, 0.044, 0.012, n=3), "meat2", 0.030))  # behind the leaf's base, up to the back wall: the full box leaves the leaf no room to slide in
FOOD.append(("onion0", [_ring(-0.035, 0.055, 0.0405, 0.018, 0.005)], "onion", 0.005))
FOOD.append(("onion1", [_ring(0.040, 0.060, 0.0425, 0.020, 0.005)], "onion", 0.005))
FOOD.append(("chilli0", [_chilli(0.030, -0.048, 0.0425, 0)], "chilli", 0.006))
FOOD.append(("chilli1", [_chilli(-0.025, -0.075, 0.0435, 0)], "chilli", 0.006))

FOOD_PTS = {n: np.vstack([mm.v for mm in meshes]) for n, meshes, _, _ in FOOD}  # in the body's own frame (the box's frame at spawn)


def _food(name, meshes, look, mass):
    joined = m.join(*meshes)
    return m.free(name, (CX, CY, TOP), [(f"p{k}", mm, look) for k, mm in enumerate(meshes)], [m.col_hull("hull", joined)], LOOKS, mass=mass, friction=(0.7, 0.6), torsional_patch=None, solver_iters=16, yaw_deg=YAW)


OBJECTS = [_box_part(), _leaf_part()] + [_food(*f) for f in FOOD]

# -- geometry the scorer reads ------------------------------------------------------------------------------------------------------------------------------------------------------------


def lid_open_deg(q_deg: float) -> float:
    return q_deg + LID_OPEN  # 0 shut, 85 at rest


def front_gap(lid_open: float) -> float:
    """How far the lid's front edge stands above the base's flange, m."""
    return (FX1 - FX0) * math.sin(math.radians(max(lid_open, 0.0)))


def _grid(x0, x1, y0, y1, z0, z1, n=(1, 1, 1)):
    return np.array([(x, y, z) for x in np.linspace(x0, x1, n[0]) for y in np.linspace(y0, y1, n[1]) for z in np.linspace(z0, z1, n[2])])


BASE_PTS = np.vstack([_grid(FX0, FX1, -FY, FY, TZ, TZ, (12, 16, 1)), _grid(WX0, WX0, -WY, WY, 0, TZ, (1, 16, 4)), _grid(WX1, WX1, -WY, WY, 0, TZ, (1, 16, 4)),
                      _grid(WX0, WX1, -WY, -WY, 0, TZ, (10, 1, 4)), _grid(WX0, WX1, WY, WY, 0, TZ, (10, 1, 4))])
LID_PTS = np.vstack([_grid(WX0, WX1, -WY, WY, TZ + LH, TZ + LH, (10, 15, 1)), _grid(WX0, WX0, -WY, WY, TZ, TZ + LH, (1, 15, 3)), _grid(FX0, FX0, -FY, FY, TZ, TZ, (1, 16, 1)),
                     _grid(WX0, WX1, -WY, -WY, TZ, TZ + LH, (10, 1, 3)), _grid(WX0, WX1, WY, WY, TZ, TZ + LH, (10, 1, 3)), _grid(TAB_X0, TAB_X0, TAB_C[0] - TAB_Y, TAB_C[0] + TAB_Y, TAB_Z0, TAB_ZH, (1, 3, 4)), _grid(TAB_X0, TAB_X0, TAB_C[1] - TAB_Y, TAB_C[1] + TAB_Y, TAB_Z0, TAB_ZH, (1, 3, 4))])


def box_points(lid_open: float) -> np.ndarray:
    """Sample points (box frame, about 15 mm apart) over the base and over the lid at its current angle."""
    lx, lz = _rot(LID_PTS[:, 0], LID_PTS[:, 2], FX1, TZ, lid_open)
    return np.vstack([BASE_PTS, np.c_[lx, LID_PTS[:, 1], lz]])


def inside(local_pts: np.ndarray) -> bool:
    """Every point of a piece lies inside the base's walls (1 mm slack) and below the shut lid's roof: none crosses the seam or lies outside the box."""
    p = local_pts
    return bool((p[:, 0] >= IX0 - 0.001).all() and (p[:, 0] <= IX1 + 0.001).all() and (np.abs(p[:, 1]) <= IY + 0.001).all() and (p[:, 2] >= -0.002).all() and (p[:, 2] <= TZ + LH).all())


def judge(lid_open: float, food_in: bool, caught: bool, flat: bool, released: bool, dead: bool, best: float) -> float:
    """0 if the box tipped or left the counter. 1.0 at the end: both tabs caught, all food inside, box flat, both grippers 3 cm clear and still for 2 s.
    Otherwise the highest substep reached: 0.7 lid down (front edge within 10 mm of the flange) with all food inside, a clean seam; 0.3 lid down with food in the seam or outside."""
    if dead:
        return 0.0
    if released and caught and food_in and flat and lid_open <= 2.0:
        return 1.0
    return min(best, 0.7)


def reached(lid_open: float, food_in: bool) -> float:
    if front_gap(lid_open) > 0.010:
        return 0.0
    return 0.7 if food_in else 0.3


# -- the scorer ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------


def hand_points(env) -> np.ndarray:
    """Points over both whole grippers (station frame): from 6 cm behind the gripper link's origin (wrist side) to the fingertips, 10 cm across, 6 cm wide, about 1.5 cm apart:
    fingers, pads, palm and housing."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        g = np.array([(sx, dy, dz) for sx in np.linspace(-0.05, 0.05, 7) for dy in np.linspace(-0.03, 0.03, 5) for dz in np.linspace(-0.06, 0.147, 14)])
        out.append(pose[:3] - origin + g @ R.T)
    return np.vstack(out)


def _links(env, asset_name):
    """World poses (station frame) of an articulation's bodies, by name: (R, p)."""
    base = env.unwrapped
    a = base.scene[asset_name]
    origin = m._np(base.scene.env_origins[0])
    poses = m._np(a.data.body_link_pose_w)[0]
    return {n: (m.quat_matrix(poses[k, 3:7]), poses[k, :3] - origin) for k, n in enumerate(a.body_names)}


BARB_TIP = np.array([[*_rot(TAB_X1 + BARB_P, BARB_Z, *LID_TURN[1:], LID_OPEN)]])
BARB_TIP = np.array([[BARB_TIP[0, 0], 0.0, BARB_TIP[0, 1]]])
LEAF_PTS = {"frame": _bx(*ROOT).v, "tip": _about_y(_bx(*TIP), TIP_UP, PIV_X, PIV_Z).v}


class Trial:
    def reset(self, env=None) -> None:
        self.n, self.held, self.dead, self.score, self.best, self.full = 0, 0, False, 0.0, 0.0, False
        self.last, self.prev, self.why = {}, None, ""

    def step(self, env) -> None:
        st = m.state(env, "box")
        lid_open = lid_open_deg(st.deg("lid"))
        tab_q = max(abs(st.deg("tab0")), abs(st.deg("tab1")))
        # food: every piece's points in the box's frame
        food_in, outs = True, []
        for name, *_ in FOOD:
            fs = m.state(env, name)
            if not inside(st.to_local(fs.to_world(FOOD_PTS[name]))):
                food_in = False
                outs.append(name)
        for n, (R, p) in _links(env, "leaf").items():
            if not inside(st.to_local(LEAF_PTS[n] @ R.T + p)):
                food_in = False
                outs.append(f"leaf_{n}")
        links = _links(env, "box")
        barbs = [st.to_local(BARB_TIP @ links[f"tab{k}"][0].T + links[f"tab{k}"][1])[0] for k in range(2)]
        caught = bool(lid_open <= 2.0 and all(b[0] >= NOTCH_X + 0.0005 and b[2] <= TZ - T + 0.0003 for b in barbs))  # both tabs' barbs past the notch edge and under the flange
        barb = barbs[0] if barbs[0][2] > barbs[1][2] else barbs[1]
        tilt = st.tilt_deg()
        if tilt > 30.0 or st.pos[2] < TOP - 0.05:
            self.dead, self.why = True, self.why or f"box tipped {tilt:.0f} deg or off the counter"
        flat = tilt < 5.0
        self.best = max(self.best, reached(lid_open, food_in))
        # hands clear: whole grippers against the box's surfaces, near points only
        dist = 0.0  # only read once the box is latched clean: nothing else can reach 1.0
        if caught and food_in and flat:
            hp = st.to_local(np.vstack([m.fingertips(env), hand_points(env)]))
            hp = hp[(hp[:, 0] > FX0 - 0.04) & (hp[:, 0] < FX1 + 0.20) & (np.abs(hp[:, 1]) < FY + 0.04) & (hp[:, 2] < 0.26)]
            bp = box_points(lid_open)
            dist = float(np.min(np.linalg.norm(hp[:, None, :] - bp[None, :, :], axis=2))) if len(hp) else 1.0
        now = np.array([lid_open, tab_q, *st.pos])
        moving = self.prev is not None and (abs(now[0] - self.prev[0]) > 0.05 or abs(now[1] - self.prev[1]) > 0.1 or np.linalg.norm(now[2:] - self.prev[2:]) > 0.0005)
        self.prev = now
        self.held = self.held + 1 if (dist >= 0.03 and not moving) else 0
        released = self.held >= HOLD_STEPS
        self.score = judge(lid_open, food_in, caught, flat, released, self.dead, self.best)
        self.full = bool(self.score >= 1.0)
        self.n += 1
        self.last = dict(lid=round(lid_open, 1), gap_mm=round(front_gap(lid_open) * 1000, 1), tab=round(tab_q, 1), barb_mm=np.round(barb * 1000, 1).tolist(), caught=caught, food_in=food_in,
                         out=outs[:4], tilt=round(tilt, 1), dist_mm=round(dist * 1000, 1), held=self.held, best=self.best, dead=self.dead, why=self.why, score=self.score)
        if self.n % 20 == 0 and ENV("KB_TRACE"):
            print(f"[kb] step {self.n}: {self.last}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """Both tabs caught under the flange, every piece of food inside the box, the box flat, both grippers 3 cm clear and all still for 2 s, the box never tipped past 30 degrees."""
    return TRIAL.full


def score(objects) -> float:
    """1.0 latched with all food inside, released and still 2 s; else the highest substep reached: 0.7 lid down with all food inside, 0.3 lid down with food in the seam. 0 if the box tipped or left the counter."""
    return TRIAL.score


# -- the demo -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

RLID = float(ENV("KB_RLID", "0.12"))  # where the left hand pushes the lid: distance from the hinge
LY = float(ENV("KB_LY", "0.045"))  # and across
STAND = float(ENV("KB_STAND", "0.016"))  # from the lid's outer face to the TCP
LEAD_DEG = float(ENV("KB_LEADDEG", "5.0"))  # the pad leads the lid by this much of its swing
LEAD = float(ENV("KB_LEAD", "10.0"))  # the lid is held this far open while the tuck hand slips out
TUCK_Z = float(ENV("KB_TUCKZ", "0.057"))  # the tuck hand's TCP height over the counter
TUCK_IN = float(ENV("KB_TUCKIN", "-0.020"))  # where the tuck fingers' front edge goes (box frame x)
TIP_UNDER = float(ENV("KB_TIPUNDER", "0.050"))  # the fingers' top under the leaf's tip, before the lift
TIP_LIFT = float(ENV("KB_TIPLIFT", "0.082"))  # and after it
TUCK_TILT = float(ENV("KB_TUCKTILT", "80"))
TUCK_ROLL = float(ENV("KB_TUCKROLL", "90"))
FINGER_LOW = float(ENV("KB_FLOW", "0.003"))  # the tuck fingers' lowest point over the rim  # how far in the tuck hand's TCP goes (box frame x)
PRESS_N = float(ENV("KB_PRESSN", "16.0"))


def demo_script(ctx):
    """Modes: full (right hand folds the leaf's tip in and holds it, left hand swings the lid down to 10 degrees, right slips out, left shuts the lid and presses the tabs home).
    none: the arms wait. one_hand: the left hand shuts the lid and presses with the leaf left over the rim. tuck_release: the right hand folds the leaf in and leaves before the lid comes.
    push_release: the right hand pushes the leaf's tip straight in along the rim and leaves before the lid comes. no_press: as full, without the press on the tabs.
    hold_in: as full, but the right hand stays on the tucked tip while the left shuts and presses the lid onto it, and leaves only at the end."""
    mode = ctx.mode
    if mode not in ("full", "none", "one_hand", "tuck_release", "push_release", "no_press", "hold_in"):
        raise ValueError(f"unknown mode {mode!r}")
    L, R = ctx.left, ctx.right
    if mode == "none":
        yield from L.wait(3.0)
        return
    cp = ctx.pos("box")
    c, s = math.cos(math.radians(YAW)), math.sin(math.radians(YAW))

    def W(x, y, z):  # a point in the box's frame (as laid out at reset), in the station frame
        return cp + np.array([c * x - s * y, s * x + c * y, z])

    gl = d.down(0)  # left: fingers closing along y, so a finger's flat side meets the lid's face
    gr = d._as_grip(d.tilted(TUCK_TILT, YAW).rot @ d.axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(TUCK_ROLL)))  # right: nearly level, pointing along the box's +x, the two fingers stacked one above the other

    def lid_deg():
        return lid_open_deg(ctx.deg("box", "lid"))

    def pad(phi):
        x, z = _rot(FX1 - RLID, TZ + LH + STAND, FX1, TZ, phi)
        return W(x, LY, z)

    def lid_down_to(stop, t=8.0):
        def target():
            return pad(max(lid_deg() - LEAD_DEG, stop - 1.0))
        yield from L.follow(target, t=t, speed=0.08, until=lambda: lid_deg() <= stop, label=f"lid down to {stop}")

    def fingers_r():  # the right fingers' lowest and highest points and their furthest reach along the box's +x, in the box's frame
        tt = m.fingertips(ctx.emb._env)[12:] - cp
        return float(tt[:, 2].min()), float(tt[:, 2].max()), float((c * tt[:, 0] + s * tt[:, 1]).max())

    def put(front, low, t=1.0, label="right"):
        """Move the right fingers so that their front edge is at box x `front` and their lowest point at height `low`, read from the fingertip points and corrected twice."""
        tcp = R.at() - cp
        x_t, z_t = c * tcp[0] + s * tcp[1], tcp[2]
        for k in range(3):
            lo, hi, ahead = fingers_r()
            x_t, z_t = x_t + (front - ahead), z_t + (low - lo)
            yield from R.move(W(x_t, LEAF_Y, z_t), t=t if k == 0 else 0.5, label=label)
        return fingers_r()

    def tuck():
        yield from R.move(W(-0.19, LEAF_Y, 0.09), grip=gr, finger=d.FINGER_CLOSED, t=2.5, label="right in front of the leaf")
        lo, hi, ahead = fingers_r()
        h = hi - lo
        yield from R.move(R.at() + np.array([0.0, 0.0, (TIP_UNDER - h) - lo]), t=1.0, label="right down, in front of the box")
        lo, hi, ahead = yield from put(FX0 - 0.003, TIP_UNDER - h, label="right under the leaf's tip")
        print(f"[kb] right under the tip: fingers {1000 * lo:.0f} to {1000 * hi:.0f} mm high, front at x {1000 * ahead:.0f} mm; tip {ctx.deg('leaf', 'tip'):.0f}", flush=True)
        tcp = R.at() - cp
        dx, dz = ahead - (c * tcp[0] + s * tcp[1]), hi - tcp[2]  # the fingers' front edge and top from the TCP
        r = TIP_L - 0.006
        for phi in (20, 35, 50, 65, 80):  # the fingers' front top edge follows the tip's arc, 6 mm in from its end
            fx, fz = PIV_X - r * math.cos(math.radians(phi)) + 0.002, PIV_Z + r * math.sin(math.radians(phi)) + 0.002
            yield from R.move(W(fx - dx, LEAF_Y, fz - dz), t=0.6, label="lift the tip")
        print(f"[kb] lifted: tip {ctx.deg('leaf', 'tip'):.0f}", flush=True)
        yield from put(TUCK_IN, TZ + FINGER_LOW, t=3.0, label="fold the tip in")
        lo, hi, ahead = fingers_r()
        print(f"[kb] tucked {TRIAL.last} tip {ctx.deg('leaf', 'tip'):.0f} fingers {1000 * lo:.0f} to {1000 * hi:.0f}, front {1000 * ahead:.0f}", flush=True)

    def right_out():
        yield from R.move(R.at() + W(-0.10, 0.0, 0.0) - cp, t=1.2, label="right slips out")
        yield from R.move(R.at() + np.array([-0.06, -0.04, 0.10]), t=1.0, label="right away")

    if mode in ("full", "tuck_release", "no_press", "hold_in"):
        yield from tuck()
    if mode == "push_release":  # straight in at rim height, as if sliding the leaf back in
        yield from R.move(W(-0.17, LEAF_Y, 0.09), grip=gr, finger=d.FINGER_CLOSED, t=2.5, label="right in front of the leaf")
        lo, hi, ahead = fingers_r()
        yield from R.move(R.at() + np.array([0.0, 0.0, (TZ + 0.004) - lo]), t=1.0, label="right down")
        yield from put(FX0 - 0.02, TZ + 0.004, label="right to the tip")
        yield from put(TUCK_IN, TZ + 0.004, t=3.0, label="push the tip in")
        print(f"[kb] pushed in {TRIAL.last} tip {ctx.deg('leaf', 'tip'):.0f}", flush=True)
    if mode in ("tuck_release", "push_release"):
        yield from right_out()
        yield from R.wait(1.5)
    # left hand: behind the open lid, then the lid down
    yield from L.move(pad(lid_deg()) + W(0.05, 0.0, 0.05) - cp, grip=gl, finger=d.FINGER_CLOSED, t=2.5, label="left behind the lid")
    yield from L.move(pad(lid_deg() + 3.0), t=1.0, label="left to the lid")
    yield from lid_down_to(LEAD)
    print(f"[kb] lid at lead {TRIAL.last}", flush=True)
    if mode in ("full", "no_press"):
        yield from ctx.together(right_out(), lid_down_to(0.3, t=4.0))  # the lid follows the tuck hand down as it slips out, so its front wall stays over the tip
    else:
        yield from lid_down_to(0.3, t=4.0)
    print(f"[kb] lid shut {TRIAL.last}", flush=True)
    # slide the pad along the roof to the front, between the tabs, keeping the lid down
    yield from L.move(W(WX0 + 0.014, 0.0, TZ + LH + STAND - 0.003), t=1.5, label="left to the front, between the tabs")
    if mode != "no_press":
        res = yield from L.press((0.0, 0.0, -1.0), PRESS_N, 0.015, speed=0.02, label="press the tabs home")
    print(f"[kb] pressed {TRIAL.last}", flush=True)
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="left up")
    yield from L.move(L.at() + np.array([-0.08, 0.10, 0.0]), t=1.0, label="left away")
    if mode == "hold_in":
        yield from right_out()
    yield from L.wait(3.0)
    print(f"[kb] end {TRIAL.last}", flush=True)
