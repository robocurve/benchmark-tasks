"""A lab bench with a Formlabs Form 4 resin 3D printer on it, its orange cover swung open and its build platform
taken off. A print has failed: the grey cured raft it left behind is stuck to the clear film at the bottom of the
printer's resin tank. In front of the printer stand the two scrapers from the printer's Finish Kit, each leaning in
its own rest with its blade toward the printer: a white plastic tank scraper and a stainless metal scraper. To the
printer's left stands an empty polypropylene waste tray.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
D = math.radians

PRINTER_GREY = (0.22, 0.22, 0.24)
SCREEN = (0.10, 0.12, 0.16)
COVER = (0.95, 0.47, 0.12)
TANK_BLACK = (0.09, 0.09, 0.10)
FILM = (0.30, 0.32, 0.35)
RESIN = (0.60, 0.61, 0.63)
WHITE_PP = (0.93, 0.93, 0.90)
STEEL = (0.74, 0.75, 0.77)
BLACK = (0.08, 0.08, 0.09)
TRAY = (0.86, 0.86, 0.82)
WOOD = (0.66, 0.50, 0.32)


def _qy(a: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


# -- the printer: Formlabs Form 4 ----------------------------------------------------------------------------------------
# Maker's tech specs: 39.8 W x 36.7 D x 55.4 H cm, 18.3 kg; build volume 20.0 x 12.5 x 21.0 cm. The printer stands on the
# bench, front toward the robot, and does not move (fixed). Modelled as its base (top deck 200 mm up: estimate), the
# rear Z tower, the build platform's carriage arm at the top of its travel (platform taken off) and the orange cover
# swung up over the back. Positions in this file are world metres: x forward, y to the robot's left, z up.
PR_W, PR_D, PR_H = 0.398, 0.367, 0.554
PX0 = 0.210  # front face
PY = 0.060  # centre, left-right
DECK = 0.200  # top of the base, where the tank seats (estimate)
TOWER_D = 0.100


def _printer_shapes():
    """Part frame: origin on the bench under the middle of the front face."""
    d, w = PR_D, PR_W
    return [
        parts.box((d, w, DECK), at=(d / 2, 0.0, DECK / 2), color=PRINTER_GREY),
        parts.box((TOWER_D, w, PR_H - DECK), at=(d - TOWER_D / 2, 0.0, (PR_H + DECK) / 2), color=PRINTER_GREY),
        # build platform carriage arm, raised to its top, no platform on it
        parts.box((0.177, 0.120, 0.025), at=(d - TOWER_D - 0.0885, 0.0, 0.5025), color=PRINTER_GREY),
        # touchscreen on the base's front, right side
        parts.box((0.004, 0.090, 0.060), at=(-0.002, -0.125, 0.150), color=SCREEN),
        # the flip-top cover, swung up and back over the tower
        parts.box((0.020, w, 0.290), at=(d - TOWER_D + 0.040, 0.0, PR_H + 0.145), color=COVER),
    ]


PRINTER = parts.Part("form_4", pos=(PX0, PY, TOP), shapes=_printer_shapes(), friction=(0.5, 0.4))

# -- the resin tank: Formlabs Form 4 Resin Tank (RT-F4-01), seated and latched in the printer ---------------------------
# The film at the tank's bottom is 5 mm above the deck; the resin well is 250 x 150 mm and its walls stand 32 mm above
# the film (estimates; the maker's printed area is 200 x 125 mm). The tank has been drained for the trial. Fixed.
FILM_UP = DECK + 0.005  # film's top surface above the bench
FILM_Z = TOP + FILM_UP
WALL_H = 0.032  # wall tops above the film
RIM_Z = FILM_Z + WALL_H
TX0, TX1 = PX0 + 0.055, PX0 + 0.205  # well, front and back inner faces
TY0, TY1 = PY - 0.125, PY + 0.125  # well, right and left inner faces
WALL = 0.025


def _tank_shapes():
    """Part frame: origin at the middle of the well's floor (the film's top surface)."""
    lx, ly = TX1 - TX0, TY1 - TY0
    h = WALL_H + 0.005
    zc = WALL_H - h / 2
    film_fr = (0.30, 0.25)  # cured resin and polypropylene on the clear film (estimate)
    return [
        parts.box((lx, ly, 0.005), at=(0.0, 0.0, -0.0025), color=FILM, friction=film_fr),
        parts.box((WALL, ly + 2 * WALL, h), at=(-lx / 2 - WALL / 2, 0.0, zc), color=TANK_BLACK),
        parts.box((WALL, ly + 2 * WALL, h), at=(lx / 2 + WALL / 2, 0.0, zc), color=TANK_BLACK),
        parts.box((lx, WALL, h), at=(0.0, -ly / 2 - WALL / 2, zc), color=TANK_BLACK),
        parts.box((lx, WALL, h), at=(0.0, ly / 2 + WALL / 2, zc), color=TANK_BLACK),
    ]


TANK_C = ((TX0 + TX1) / 2, (TY0 + TY1) / 2)
TANK = parts.Part("resin_tank", pos=(*TANK_C, FILM_Z), shapes=_tank_shapes(), friction=(0.5, 0.4),
                  contact_offset=0.002)

# -- the failed print: its cured raft, stuck to the film -----------------------------------------------------------------
# A cured resin slab standing in for the raft a failed print leaves on the film: 50 x 60 mm, 8 mm thick, its film-side
# edge chamfered 45 degrees for 3 mm all round (the coupon is printed that way, so a blade has a defined way in), cured
# Grey resin, 27 g (STL in fixture/). Part frame: origin at the middle of its underside. Its long side runs left-right;
# it lies 12 mm in front of the tank's back wall.
RAFT_X, RAFT_Y = 0.050, 0.060
FOOT_IN = FOOT_H = 0.003  # the chamfer: 3 mm in, 3 mm up
RAFT_H = 0.008
RAFT_MASS = 0.027
RAFT_AT = (TX1 - 0.012 - RAFT_X / 2, 0.120)


def _raft_shapes():
    hx, hy, b = RAFT_X / 2, RAFT_Y / 2, FOOT_IN
    pts = [(-hx + b, -hy + b, 0.0), (hx - b, -hy + b, 0.0), (hx - b, hy - b, 0.0), (-hx + b, hy - b, 0.0),
           (-hx, -hy, FOOT_H), (hx, -hy, FOOT_H), (hx, hy, FOOT_H), (-hx, hy, FOOT_H),
           (-hx, -hy, RAFT_H), (hx, -hy, RAFT_H), (hx, hy, RAFT_H), (-hx, hy, RAFT_H)]
    faces = [(0, 3, 2, 1), (8, 9, 10, 11), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7),
             (4, 5, 9, 8), (5, 6, 10, 9), (6, 7, 11, 10), (7, 4, 8, 11)]
    return [parts.hull(pts, faces, color=RESIN)]


RAFT = parts.Part("cured_resin", pos=(*RAFT_AT, FILM_Z), mass=RAFT_MASS, shapes=_raft_shapes(),
                  friction=(0.35, 0.30), contact_offset=0.002)

# The raft is stuck to the film by a breakable bond (a rigid joint the simulator releases for good once the load it
# carries passes a limit). The joint sits at the middle of the raft, 5.5 mm above the film, so a push on its side
# loads it mainly in shear, while an edge lifted by a blade under it loads it in bending, as a peel does. Estimates for
# a weak bond to a non-stick film: shear 60 N (about 0.025 MPa over the coupon's 24 cm2 bonded face), about 3 N to peel
# one edge.
BOND_FORCE = 60.0  # N
BOND_TORQUE = 0.04  # N m: about 3 N on a blade under one edge
BOND_AT = (0.0, 0.0, 0.0055)  # raft frame: the middle of its upright sides

_author_plain = parts._author


def _author_with_bond(part, root_path, translation, orientation):
    root = _author_plain(part, root_path, translation, orientation)
    if part.name == RAFT.name:
        _bond(root_path)
    return root


parts._author = _author_with_bond


def _bond(root_path: str) -> None:
    """A breakable fixed joint from the world to the raft (authored before the simulation starts)."""
    from isaaclab.sim.utils import get_current_stage
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    stage = get_current_stage()
    pose = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(root_path)).RemoveScaleShear()
    anchor = pose.Transform(Gf.Vec3d(*BOND_AT))
    parent = root_path.rsplit("/", 1)[0]
    j = UsdPhysics.FixedJoint.Define(stage, f"{parent}/resin_bond")
    j.CreateBody1Rel().SetTargets([Sdf.Path(root_path)])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in anchor)))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(pose.ExtractRotationQuat()))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(*BOND_AT))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    j.CreateBreakForceAttr().Set(float(BOND_FORCE))
    j.CreateBreakTorqueAttr().Set(float(BOND_TORQUE))


# -- the scrapers: Formlabs Form 4 Finish Kit ----------------------------------------------------------------------------
# Part frame of each: origin at the middle of the blade's front edge on its underside, x from the edge toward the
# handle's end, z out of the blade's top face, so the tool's underside is the plane z = 0.
# Plastic tank scraper (white polypropylene, 130 mm long, blade 55 mm wide, 2.5 mm thick, its front 12 mm tapering to a
# 0.8 mm edge; handle 58 x 22 x 14 mm; 25 g: estimates from the maker's photographs).
PS_W, PS_T, PS_TAPER, PS_EDGE = 0.055, 0.0025, 0.012, 0.0008
PS_BLADE = 0.055
PS_HANDLE = (0.072, 0.130, 0.022, 0.014)  # from x, to x, width, thickness
PS_MASS = 0.025
GRIP_AT = 0.110  # where the fingers take the handle (tool x)
# Metal scraper (stainless blade 40 mm wide, 1.2 mm thick, 60 mm long with a 0.5 mm edge; black handle 90 x 26 x 16 mm;
# 165 mm overall, 68 g: estimates).
MS_W, MS_T, MS_TAPER, MS_EDGE, MS_BLADE = 0.040, 0.0012, 0.006, 0.0005, 0.060
MS_HANDLE = (0.075, 0.165, 0.026, 0.016)
MS_MASS = 0.068


def _wedge(x0, x1, w, t0, t1, color, friction=None):
    """A blade's tapered front: thickness t0 at x0 (the edge) growing to t1 at x1, w wide; a convex piece."""
    hw = w / 2
    pts = [(x0, -hw, 0.0), (x1, -hw, 0.0), (x1, hw, 0.0), (x0, hw, 0.0),
           (x0, -hw, t0), (x1, -hw, t1), (x1, hw, t1), (x0, hw, t0)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return parts.hull(pts, faces, color=color, friction=friction)


def _plastic_shapes():
    fr = (0.30, 0.25)
    hx0, hx1, hw, ht = PS_HANDLE
    return [
        _wedge(0.0, PS_TAPER, PS_W, PS_EDGE, PS_T, WHITE_PP, fr),
        parts.box((PS_BLADE - PS_TAPER, PS_W, PS_T), at=((PS_BLADE + PS_TAPER) / 2, 0.0, PS_T / 2), color=WHITE_PP,
                  friction=fr),
        parts.box((hx0 - PS_BLADE, 0.030, 0.004), at=((hx0 + PS_BLADE) / 2, 0.0, 0.002), color=WHITE_PP),
        parts.box((hx1 - hx0, hw, ht), at=((hx0 + hx1) / 2, 0.0, ht / 2), color=WHITE_PP),
    ]


def _metal_shapes():
    hx0, hx1, hw, ht = MS_HANDLE
    return [
        _wedge(0.0, MS_TAPER, MS_W, MS_EDGE, MS_T, STEEL, (0.25, 0.20)),
        parts.box((MS_BLADE - MS_TAPER, MS_W, MS_T), at=((MS_BLADE + MS_TAPER) / 2, 0.0, MS_T / 2), color=STEEL),
        parts.box((hx0 - MS_BLADE, 0.022, 0.010), at=((hx0 + MS_BLADE) / 2, 0.0, 0.005), color=STEEL),
        parts.box((hx1 - hx0, hw, ht), at=((hx0 + hx1) / 2, 0.0, ht / 2), color=BLACK),
    ]


# Each scraper leans in its rest at 45 degrees, blade toward the printer (+x), its edge 10 mm above the bench against
# the rest's lip, handle rising toward the robot.
REST_PITCH = D(45.0)
EDGE_UP = 0.010
PLASTIC_EDGE = (0.180, 0.120)
METAL_EDGE = (0.180, 0.235)


def tool_rotation(pitch: float) -> np.ndarray:
    """A scraper pointing +x (its edge forward) with its blade at ``pitch`` to the bench, handle rising toward -x."""
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[-c, 0.0, s], [0.0, -1.0, 0.0], [s, 0.0, c]])


REST_Q = tuple(float(v) for v in parts._matrix_to_quat(tool_rotation(REST_PITCH)))
PLASTIC = parts.Part("plastic_scraper", pos=(*PLASTIC_EDGE, TOP + EDGE_UP), quat=REST_Q, mass=PS_MASS,
                     shapes=_plastic_shapes(), friction=(0.9, 0.8), contact_offset=0.002)
METAL = parts.Part("metal_scraper", pos=(*METAL_EDGE, TOP + EDGE_UP), quat=REST_Q, mass=MS_MASS,
                   shapes=_metal_shapes(), friction=(0.9, 0.8), contact_offset=0.002)


def _rest_shapes(length):
    """A scraper rest (printed PLA, drawing in fixture/): a 10 mm rib under the tool's underside at 45 degrees on two
    posts, and a lip in front of the edge. Part frame: origin on the bench under the tool's edge."""
    rib_t = 0.010
    c, s = math.cos(REST_PITCH), math.sin(REST_PITCH)
    mid = length / 2
    # the rib's top face is the tool's underside plane
    cx = -mid * c - (rib_t / 2) * s
    cz = EDGE_UP + mid * s - (rib_t / 2) * c
    q = _qy(REST_PITCH)
    out = [parts.box((length, 0.010, rib_t), at=(cx, 0.0, cz), color=WOOD, quat=q)]
    for frac in (0.25, 0.80):
        px = -frac * length * c + 0.004
        top = EDGE_UP + frac * length * s - 0.010
        out.append(parts.box((0.010, 0.010, top), at=(px, 0.0, top / 2), color=WOOD))
    out.append(parts.box((0.006, 0.064, EDGE_UP + 0.004), at=(0.0045, 0.0, (EDGE_UP + 0.004) / 2), color=WOOD))
    return out


PLASTIC_REST = parts.Part("plastic_scraper_rest", pos=(*PLASTIC_EDGE, TOP), shapes=_rest_shapes(0.128),
                          friction=(0.6, 0.5), contact_offset=0.002)
METAL_REST = parts.Part("metal_scraper_rest", pos=(*METAL_EDGE, TOP), shapes=_rest_shapes(0.160),
                        friction=(0.6, 0.5), contact_offset=0.002)

# -- the waste tray: Thermo Scientific Nalgene 6902-1000 autoclavable tray, polypropylene, 260 x 159 x 64 mm, 2 L ---------
# Walls modelled upright, 3 mm thick, floor 3 mm (the real walls taper slightly). Fixed (taped to the bench).
TR_L, TR_W, TR_H, TR_T = 0.260, 0.159, 0.064, 0.003
TRAY_C = (0.335, 0.400)


def _tray_shapes():
    lx, ly = TR_L, TR_W
    return [
        parts.box((lx, ly, TR_T), at=(0.0, 0.0, TR_T / 2), color=TRAY),
        parts.box((TR_T, ly, TR_H), at=(-lx / 2 + TR_T / 2, 0.0, TR_H / 2), color=TRAY),
        parts.box((TR_T, ly, TR_H), at=(lx / 2 - TR_T / 2, 0.0, TR_H / 2), color=TRAY),
        parts.box((lx, TR_T, TR_H), at=(0.0, -ly / 2 + TR_T / 2, TR_H / 2), color=TRAY),
        parts.box((lx, TR_T, TR_H), at=(0.0, ly / 2 - TR_T / 2, TR_H / 2), color=TRAY),
    ]


WASTE_TRAY = parts.Part("waste_tray", pos=(*TRAY_C, TOP), shapes=_tray_shapes(), friction=(0.5, 0.4),
                        contact_offset=0.002)

OBJECTS = [PRINTER, TANK, RAFT, PLASTIC, METAL, PLASTIC_REST, METAL_REST, WASTE_TRAY]
DEMO_TOUCHES = ("plastic_scraper_rest",)


# -- geometry for scoring ------------------------------------------------------------------------------------------------
def frame(st):
    return np.asarray(st.pos, float), _rot(st.quat_xyzw)


def box_gap(pts, centre, rot, half) -> float:
    """Closest distance from any point to an oriented box (0 inside), m."""
    pts = np.asarray(pts, float).reshape(-1, 3)
    if len(pts) == 0:
        return 1.0
    loc = (pts - centre) @ rot
    q = np.abs(loc) - half
    return float((np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)).min())


RAFT_HALF = np.array([RAFT_X / 2, RAFT_Y / 2, RAFT_H / 2])


def raft_box(st):
    p, r = frame(st)
    return p + r @ np.array([0.0, 0.0, RAFT_H / 2]), r, RAFT_HALF


def raft_gap(pts, st) -> float:
    return box_gap(pts, *raft_box(st))


def raft_moved(st):
    """(mm the raft's middle has moved from where it was stuck, degrees it has turned)."""
    p, r = frame(st)
    d = float(np.linalg.norm(p - np.array([*RAFT_AT, FILM_Z])))
    turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))))
    return d, turn


def raft_samples(st) -> np.ndarray:
    """Corners and a 5 x 5 grid on the underside of the raft (world)."""
    p, r = frame(st)
    loc = [(x, y, z) for x in (-RAFT_X / 2, RAFT_X / 2) for y in (-RAFT_Y / 2, RAFT_Y / 2) for z in (FOOT_H, RAFT_H)]
    loc += [(x, y, 0.0) for x in np.linspace(-RAFT_X / 2 + FOOT_IN, RAFT_X / 2 - FOOT_IN, 5)
            for y in np.linspace(-RAFT_Y / 2 + FOOT_IN, RAFT_Y / 2 - FOOT_IN, 5)]
    return p + np.asarray(loc, float) @ r.T


def _tool_boxes(shapes):
    out = []
    for s in shapes:
        if s.kind == "hull":
            pts = np.asarray(s.points, float)
            out.append(((pts.min(axis=0) + pts.max(axis=0)) / 2, (pts.max(axis=0) - pts.min(axis=0)) / 2))
        else:
            out.append((np.asarray(s.at, float), np.asarray(s.size, float) / 2))
    return out


PLASTIC_BOXES = _tool_boxes(PLASTIC.shapes)
METAL_BOXES = _tool_boxes(METAL.shapes)


def tool_gap(pts, st, boxes) -> float:
    p, r = frame(st)
    return min(box_gap(pts, p + r @ c, r, h) for c, h in boxes)


def in_well(p, z_top=None) -> bool:
    """Over the tank's resin well (inside its walls seen from above) and below its rim (or ``z_top``)."""
    top = RIM_Z if z_top is None else z_top
    return bool(TX0 - EPS <= p[0] <= TX1 + EPS and TY0 - EPS <= p[1] <= TY1 + EPS and p[2] < top + EPS)


def raft_in_tank(st) -> bool:
    c, _, _ = raft_box(st)
    return in_well(c, RIM_Z + 0.010)


TRAY_IN = (TRAY_C[0] - TR_L / 2 + TR_T, TRAY_C[0] + TR_L / 2 - TR_T, TRAY_C[1] - TR_W / 2 + TR_T,
           TRAY_C[1] + TR_W / 2 - TR_T)


def raft_in_tray(st) -> bool:
    """The raft's middle inside the tray's walls seen from above and below its rim."""
    c, _, _ = raft_box(st)
    x0, x1, y0, y1 = TRAY_IN
    return bool(x0 - EPS <= c[0] <= x1 + EPS and y0 - EPS <= c[1] <= y1 + EPS and c[2] < TOP + TR_H + EPS)


PS_CORNERS = np.array([[0.0, -PS_W / 2, 0.0], [0.0, PS_W / 2, 0.0]])
MS_BLADE_PTS = np.array([[x, y, 0.0] for x in (0.0, MS_BLADE / 2, MS_BLADE) for y in (-MS_W / 2, 0.0, MS_W / 2)])
CORNER_TILT = D(10.0)  # edge tilted this much with one corner on the film: digging with a corner
STAB = D(50.0)  # blade this steep to the film with its edge on it: digging in
ON_FILM = 0.0015  # m: an edge point this close to the film touches it


def plastic_edge(st):
    """(the edge's two corners, its middle) in the world, and the blade's angle to the bench (rad)."""
    p, r = frame(st)
    corners = p + PS_CORNERS @ r.T
    return corners, p, math.acos(max(-1.0, min(1.0, abs(float(r[2, 2])))))


def corner_dig(st) -> bool:
    corners, _, _ = plastic_edge(st)
    h = corners[:, 2] - FILM_Z
    inside = any(in_well(c, RIM_Z) for c in corners)
    return bool(inside and h.min() <= ON_FILM + EPS and (h.max() - h.min()) >= PS_W * math.sin(CORNER_TILT) - EPS)


def stab(st) -> bool:
    """The blade steeper than 50 degrees with any point of its edge (either corner or the middle) on the film."""
    corners, mid, ang = plastic_edge(st)
    pts = np.vstack([corners, mid])
    low = pts[np.argmin(pts[:, 2])]
    return bool(in_well(low) and low[2] - FILM_Z <= ON_FILM + EPS and ang > STAB + EPS)


def metal_in_tank(st) -> bool:
    p, r = frame(st)
    return any(in_well(q) for q in p + MS_BLADE_PTS @ r.T)


def scraper_home(st) -> bool:
    """Back in its rest: its edge within 15 mm of where it started and the tool turned no more than 10 degrees."""
    p, r = frame(st)
    r0 = tool_rotation(REST_PITCH)
    d = float(np.linalg.norm(p - np.array([*PLASTIC_EDGE, TOP + EDGE_UP])))
    turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(r0.T @ r) - 1.0) / 2.0))))
    return bool(d <= 0.015 + EPS and turn <= 10.0 + EPS)


def raft_on_scraper(st_raft, st_tool) -> bool:
    return tool_gap(raft_samples(st_raft), st_tool, PLASTIC_BOXES) < 0.003 - EPS


# -- the trial record ------------------------------------------------------------------------------------------------------
TOUCH = 0.002  # a hand point within 2 mm of the raft or the film touches it
CLEAR = 0.030  # let go: every sampled hand point 30 mm or more from the plastic scraper
FREE_MM, FREE_DEG = 0.003, 3.0  # the raft has come free once it has moved or turned this much
HOLD_S = 1.0
STILL_M = 0.001
_rec = parts.__dict__.setdefault("resin_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", zero=False, why="", freed=False, freed_t=None, snap=None, still=None,
                raft_snap=None, raft_still=None, free=None, t=None, gap=None)


if not _rec:
    reset_record()


def observe(objects, hands, t) -> None:
    """One control step. ``hands``: the sampled points of both grippers (world, (N, 3)), or None if the arms could not
    be read, which costs the trial its credit."""
    _rec["steps"] += 1
    try:
        raft, tool, metal = objects[RAFT.name], objects[PLASTIC.name], objects[METAL.name]
        raft_p, tool_p = np.asarray(raft.pos, float), np.asarray(tool.pos, float)
        snap = np.concatenate([raft_p, tool_p, frame(tool)[1][:, 0], frame(raft)[1][:, 2]])
        if not np.all(np.isfinite(snap)) or not np.all(np.isfinite(np.asarray(metal.pos, float))):
            raise ValueError("scene state unknown")
        if hands is None or len(hands) == 0 or not np.all(np.isfinite(hands)):
            raise ValueError("arm state unknown")
        if not _rec["zero"]:
            film_touch = hands[(hands[:, 0] >= TX0) & (hands[:, 0] <= TX1) & (hands[:, 1] >= TY0) &
                               (hands[:, 1] <= TY1) & (hands[:, 2] <= FILM_Z + TOUCH + EPS)]
            if raft_in_tank(raft) and raft_gap(hands, raft) < TOUCH - EPS:
                _rec.update(zero=True, why="a gripper touched the cured resin in the tank")
            elif len(film_touch):
                _rec.update(zero=True, why="a gripper touched the tank film")
            elif metal_in_tank(metal):
                _rec.update(zero=True, why="the metal scraper went into the tank")
            elif corner_dig(tool):
                _rec.update(zero=True, why="the plastic scraper dug in with a corner")
            elif stab(tool):
                _rec.update(zero=True, why="the plastic scraper's edge was driven into the film steeply")
        d, turn = raft_moved(raft)
        if not _rec["freed"] and (d > FREE_MM + EPS or turn > FREE_DEG + EPS):
            _rec.update(freed=True, freed_t=float(t))
        if _rec["snap"] is None or float(np.abs(snap - _rec["snap"]).max()) > STILL_M + EPS:
            _rec["snap"], _rec["still"] = snap, float(t)
        if _rec["raft_snap"] is None or float(np.abs(snap[:3] - _rec["raft_snap"]).max()) > STILL_M + EPS:
            _rec["raft_snap"], _rec["raft_still"] = snap[:3], float(t)
        gap = tool_gap(hands, tool, PLASTIC_BOXES)
        _rec["free"] = (float(t) if _rec["free"] is None else _rec["free"]) if gap >= CLEAR - EPS else None
        _rec.update(gap=gap, t=float(t))
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), free=None)


def _held_still(since) -> bool:
    return since is not None and _rec["t"] is not None and _rec["t"] - since >= HOLD_S - EPS


# -- scoring -------------------------------------------------------------------------------------------------------------
def score(objects):
    """0 for the whole trial if a gripper touched the cured resin while it was in the tank or touched the tank film, the
    metal scraper went into the tank, or the plastic scraper dug at the film with a corner (edge tilted 10 degrees or
    more with a corner on it) or with its edge on the film at over 50 degrees (and if the scene or the arms could not be
    read at any step). Otherwise: 1.0 with the raft in the waste tray, the plastic scraper back in its rest, every hand
    point 30 mm or more from it and everything still for 1 s; 0.75 with the raft in the tray and still for 1 s, not
    carried on a scraper still in hand; 0.5 the same with the raft out of the tank but not in the tray; 0.25 once the
    raft has come free of the film."""
    try:
        if not _rec["known"] or _rec["zero"] or _rec["steps"] == 0:
            return 0.0
        raft, tool = objects[RAFT.name], objects[PLASTIC.name]
        let_go = _rec["free"] is not None and _held_still(_rec["free"])
        off = let_go or not raft_on_scraper(raft, tool)  # not carried on a scraper that is still in hand
        raft_still = _held_still(_rec["raft_still"])
        if raft_in_tray(raft) and off and raft_still:
            all_still = _held_still(_rec["still"])
            if scraper_home(tool) and all_still and let_go:
                return 1.0
            return 0.75
        if not raft_in_tank(raft) and not raft_in_tray(raft) and off and raft_still:
            return 0.5
        return 0.25 if _rec["freed"] else 0.0
    except (KeyError, ValueError, TypeError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        c, _, _ = raft_box(objects[RAFT.name])
        return float(math.hypot(c[0] - TRAY_C[0], c[1] - TRAY_C[1]))
    except Exception:  # noqa: BLE001
        return 1.0


# -- the arms as point clouds: hull faces sampled every 3 to 5 mm -------------------------------------------------------
HAND_LINKS = ("link5", "gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            ii, jj = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing="ij")
            keep = (ii + jj) <= m
            s_, t_ = ii[keep][:, None] / m, jj[keep][:, None] / m
            pts.append(c + (a - c) * s_ + (b - c) * t_)
    pts = np.concatenate(pts)
    _, idx = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(idx)]])


def link_points(name, step=0.003) -> np.ndarray:
    key = (name, step)
    if key not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[key] = np.concatenate([_hull_face_points(h, step) for h in hs])
    return _FACES[key]


STEP = {"link5": 0.005, "gripper": 0.005, "tip_left": 0.003, "tip_right": 0.003}


def hand_points(link_poses: dict) -> np.ndarray:
    return np.concatenate([link_points(name, STEP[name]) @ m[:3, :3].T + m[:3, 3]
                           for (_, name), m in link_poses.items() if name in HAND_LINKS])


def _arm_poses(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out = {}
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                out[(arm, n)] = m
    if len(out) != 2 * len(HAND_LINKS) or not all(np.isfinite(m).all() for m in out.values()):
        return None
    return out


@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["last_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001
        objects = {}
    pts = None
    try:
        poses = _arm_poses(sim)
        if poses is not None:
            pts = hand_points(poses)
    except Exception:  # noqa: BLE001
        pts = None
    observe(objects, pts, float(sim.t))
    log = (str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("resin.jsonl")) if os.environ.get("TRACE_OUT")
           else os.environ.get("RESIN_LOG"))
    if log and objects and _rec["steps"] % 2 == 0:
        try:
            raft, tool = objects[RAFT.name], objects[PLASTIC.name]
            corners, mid, ang = plastic_edge(tool)
            d, turn = raft_moved(raft)
            row = dict(t=round(float(sim.t), 3), score=score(objects), zero=_rec["zero"], why=_rec["why"],
                       freed=_rec["freed"], known=_rec["known"], reason=_rec["reason"],
                       raft=np.round(raft.pos, 4).tolist(), raft_mm=round(d * 1000, 2), raft_deg=round(turn, 2),
                       in_tank=raft_in_tank(raft), in_tray=raft_in_tray(raft), on_tool=raft_on_scraper(raft, tool),
                       edge=np.round(mid, 4).tolist(), edge_h=np.round((corners[:, 2] - FILM_Z) * 1000, 2).tolist(),
                       pitch=round(math.degrees(ang), 1), home=scraper_home(tool),
                       gap=None if _rec["gap"] is None else round(_rec["gap"], 4),
                       metal=np.round(objects[METAL.name].pos, 4).tolist())
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            a = base.scene["left_robot"]
            poses = parts._np(a.data.body_link_pose_w)[0]
            row["grip"] = np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist()
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the fixed layout --------------------------------------------------------------------------------
# The left arm takes the plastic scraper by its handle, straight down, as it leans in its rest (blade 45 degrees down).
# Leaning the hand 20 degrees away brings the blade to 25 degrees. It sets the edge on the film 10 mm in front of the
# raft and slides it forward along the film: the edge goes in under the raft's proud edge, the blade's slope lifts that
# edge and the raft comes free; the blade keeps going and pushes the raft to the tank's back wall, where the blade
# slides in under it. Pivoting about the edge, the hand leans further until the blade is at 8 degrees, lifts the raft
# out level, carries it over the tank's left wall to the waste tray and tips it off the edge into the tray. Last it lays
# the scraper back in its rest, lets go and goes back to rest. The right arm stays at rest.
# RESIN_DEMO=metal does the same with the metal scraper (scores 0); RESIN_DEMO=corner tilts the edge 12 degrees
# (scores 0); RESIN_DEMO=leave frees the raft and puts the scraper back without lifting the raft out (0.25);
# RESIN_DEMO=shove drives the blade along about 6 mm above the film, into the raft's upright side, then backs out.
SQUEEZE_P = -(PS_HANDLE[2] / 2 - 0.006)  # 6 mm past each face: a firm grip, so the tool does not turn in it
SQUEEZE_M = -(MS_HANDLE[2] / 2 - 0.006)
SAG = 0.003  # the arm carries a light tool about this much below its target; the demo aims high
PRESS = 0.001  # and presses the edge this much into the film while it slides


def _w(d, p, finger, label, rot):
    return Waypoint(d, tuple(float(v) for v in p), finger, label, tilt._TiltedYaw(0.0, rot))


def grip_point(edge, pitch, grip_x=GRIP_AT, mid_t=PS_HANDLE[3] / 2, roll=0.0) -> np.ndarray:
    """Where the TCP must be for the tool's edge middle to be at ``edge`` with the blade at ``pitch``."""
    r = tool_rotation(pitch)
    if roll:
        c, s = math.cos(roll), math.sin(roll)
        r = r @ np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return np.asarray(edge, float) + r @ np.array([grip_x, 0.0, mid_t]) + np.array([0.0, 0.0, SAG])


def lean(pitch, roll=0.0):
    """The hand's rotation that holds a tool taken at the rest's 45 degrees with its blade at ``pitch``."""
    r = tilt.rotation(0.0, math.degrees(REST_PITCH - pitch), 0.0)
    if roll:
        c, s = math.cos(roll), math.sin(roll)
        r = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]) @ r
    return r


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def demo():
    mode = os.environ.get("RESIN_DEMO", "")
    metal = mode == "metal"
    tool_xy = METAL_EDGE if metal else PLASTIC_EDGE
    grip_x = 0.120 if metal else GRIP_AT
    mid_t = MS_HANDLE[3] / 2 if metal else PS_HANDLE[3] / 2
    squeeze = SQUEEZE_M if metal else SQUEEZE_P
    roll = D(12.0) if mode == "corner" else 0.0

    def w(d, edge, pitch, finger, label, rl=0.0):
        return _w(d, grip_point(edge, pitch, grip_x, mid_t, rl), finger, label, lean(pitch, rl))

    y = RAFT_AT[1]
    rest_edge = np.array([*tool_xy, TOP + EDGE_UP])
    up = np.array([0.0, 0.0, 0.10])
    x_lip = RAFT_AT[0] - RAFT_X / 2
    carry, work, flat, tip = D(10.0), D(25.0), D(8.0), D(40.0)
    shove = mode == "shove"  # the edge aimed 8.5 mm up (about 6 mm once the arm sags): it meets the raft's upright side
    start = np.array([x_lip - 0.010, y, FILM_Z + (0.0085 if shove else -PRESS)])
    end_x = TX1 - 0.010
    front = np.array([PX0 - 0.005, y, TOP + 0.215])  # the edge in front of the printer, above its deck
    over_wall = np.array([TX0 - 0.020, y, RIM_Z + 0.015])  # the edge over the tank's front wall
    plan = [
        w(2.5, rest_edge + [0.0, 0.0, 0.08], REST_PITCH, FINGER_OPEN, "above the scraper's handle"),
        w(1.5, rest_edge, REST_PITCH, FINGER_OPEN, "fingers down round the handle"),
        w(0.8, rest_edge, REST_PITCH, squeeze, "close on the handle"),
        w(1.5, rest_edge + up, REST_PITCH, squeeze, "lift it out of its rest"),
        w(2.0, front, carry, squeeze, "blade up in front of the printer"),
        w(1.5, over_wall, carry, squeeze, "edge over the tank's front wall"),
        w(2.0, start + [0.0, 0.0, 0.042], carry, squeeze, "over the film in front of the raft"),
        w(1.5, start + [0.0, 0.0, 0.008], work, squeeze, "blade to 25 degrees, edge just above the film", roll),
        w(0.8, start, work, squeeze, "edge on the film", roll),
        w(5.0, [end_x, y, start[2]], work, squeeze, "slide the edge in under the raft", roll),
    ]
    under = np.array([end_x, y, FILM_Z - 0.001])
    if mode in ("leave", "shove"):
        plan += [
            w(1.5, under + [-0.030, 0.0, 0.042], carry, squeeze, "back up off the film"),
            w(1.5, over_wall, carry, squeeze, "back over the tank's front wall"),
            w(1.5, front, carry, squeeze, "back in front of the printer"),
            w(2.0, rest_edge + up, REST_PITCH, squeeze, "back over the rest"),
        ]
    else:
        lifted = under + [0.0, 0.0, 0.062]
        over_tray = np.array([TRAY_C[0] - 0.030, TRAY_C[1], TOP + TR_H + 0.020])
        plan += [
            w(1.5, under, flat, squeeze, "pivot on the edge: blade to 8 degrees"),
            w(2.0, lifted, flat, squeeze, "lift the raft out level"),
            w(3.0, [over_tray[0], over_tray[1], lifted[2]], flat, squeeze, "carry it over to the tray"),
            w(1.5, over_tray, flat, squeeze, "lower it into the tray"),
            w(2.0, over_tray, tip, squeeze, "tip it off the edge"),
            w(1.0, over_tray, tip, squeeze, "let it slide off"),
            w(1.5, over_tray + [0.0, 0.0, 0.05], REST_PITCH, squeeze, "lift the scraper out of the tray"),
            w(2.0, [rest_edge[0], over_tray[1], rest_edge[2] + 0.075], REST_PITCH, squeeze, "in front of the printer"),
            w(2.5, rest_edge + [0.0, 0.0, 0.075], REST_PITCH, squeeze, "back over the rest"),
        ]
    plan += [
        w(1.5, rest_edge + [0.0, 0.0, 0.004], REST_PITCH, squeeze, "lay the scraper in its rest"),
        w(0.8, rest_edge + [0.0, 0.0, 0.004], REST_PITCH, FINGER_OPEN, "let go"),
        w(1.2, rest_edge + [0.0, 0.0, 0.10], REST_PITCH, FINGER_OPEN, "lift the fingers clear"),
    ]
    lrest, lyaw = _rest_pose("left")
    plan.append(Waypoint(2.0, lrest, FINGER_OPEN, "back to the rest pose", lyaw))
    plan.append(Waypoint(3.0, lrest, FINGER_OPEN, "wait", lyaw))
    rrest, ryaw = _rest_pose("right")
    t_left = sum(wp.duration for wp in plan)
    plans = {"left": plan, "right": [Waypoint(t_left, rrest, FINGER_OPEN, "stay at rest", ryaw)]}
    for arm, pl in plans.items():
        r, yw = _rest_pose(arm)
        f = pl[0].finger
        pl[:0] = [Waypoint(2.0, r, f, "to the rest pose", yw), Waypoint(0.5, r, f, "settle at the rest pose", yw)]
    return plans
