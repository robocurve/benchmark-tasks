"""A kitchen counter (the station table). On a white cutting board lies one frozen salmon portion in its clear vacuum
film, which is sealed across both ends and carries a small printed label; a pair of spring-loaded kitchen shears lies
open on the same board, blades pointing away from the arms. On the counter beside the board stand an empty
rectangular glass food dish and a round red scrap bowl. Where the dish and the bowl stand changes between trials.
Nobody is in the scene.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
D = math.radians
SUCCESS_HOLD_S = 1.0

WHITE = (0.93, 0.93, 0.90)
FILM = (0.62, 0.74, 0.86)
GLINT = (0.86, 0.93, 0.99)
SEAL = (0.97, 0.97, 0.97)
SEAL_BAND = (0.50, 0.62, 0.78)
SALMON = (0.96, 0.52, 0.40)
FAT = (0.99, 0.80, 0.70)
STEEL = (0.72, 0.74, 0.76)
GRIP = (0.10, 0.10, 0.11)
GLASS = (0.78, 0.88, 0.90)
RED = (0.80, 0.12, 0.12)


def _qz(a: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# -- the cutting board: Winco CBWT-1218, white polyethylene, 12 x 18 x 1/2 in -------------------------------------
BOARD_L, BOARD_W, BOARD_T = 0.457, 0.305, 0.0127  # L along x
BOARD_X0 = 0.170  # its near edge (toward the arms)
BOARD = parts.Part("board", pos=(BOARD_X0 + BOARD_L / 2, 0.120, TOP), mass=1.6, friction=(0.5, 0.4),
                   shapes=[parts.box((BOARD_L, BOARD_W, BOARD_T), at=(0, 0, BOARD_T / 2), color=WHITE)])
BOARD_TOP = TOP + BOARD_T

# -- the frozen portion and its vacuum film -------------------------------------------------------------------------
# Fish: one 4 oz (113 g) skinless portion, taken as a rigid 105 x 62 x 18 mm block (estimate).
FISH = (0.105, 0.062, 0.018)
FISH_MASS = 0.113
# Film: a clear tube sealed across both ends. In the simulation its body is a rigid clear sleeve 2 to 3 mm bigger than
# the fish; each end is a panel (with the flat film beyond it) hinged on its lower edge and held shut by the seal.
WALL = 0.0025
L_IN, W_IN, H_IN = FISH[0] + 0.004, FISH[1] + 0.004, FISH[2] + 0.003  # 109 x 66 x 21 mm inside
W_OUT, H_OUT = W_IN + 2 * WALL, H_IN + 2 * WALL  # 71 x 26 mm
END_X = L_IN / 2 + WALL  # each end panel's outer face, 57 mm from the middle
LOOSE = 0.028  # flat, unwelded film between the end panel and the seal band
BAND = 0.008  # the welded seal band at each end
FLAT_W = 0.090  # the flat end's width: half the tube's 180 mm girth (the fish's own girth is 160 mm)
K_SEAL = 1.0  # N m/rad: the seal holding an end panel shut (a stiff spring at its stop)


def _end_shapes(s: int):
    """End panel at local x = s * END_X (s = +1 or -1), with its flat film and seal band (seen, not collided)."""
    x0 = s * (L_IN / 2 + WALL / 2)
    xl = s * (END_X + LOOSE / 2)
    xb = s * (END_X + LOOSE + BAND / 2)
    return (
        parts.box((WALL, W_OUT, H_OUT), at=(x0, 0, H_OUT / 2), color=FILM, visible=False),
        parts.box((0.0015, W_OUT, 0.0015), at=(x0, 0, H_OUT - 0.00075), color=FILM, collide=False),
        parts.box((LOOSE, FLAT_W, 0.0012), at=(xl, 0, 0.0006), color=FILM, collide=False),
        parts.box((BAND, FLAT_W, 0.0016), at=(xb, 0, 0.0008), color=SEAL_BAND, collide=False),
    )


POUCH_SHAPES = [
    parts.box((L_IN, W_OUT, WALL), at=(0, 0, WALL / 2), color=FILM, visible=False),  # lower film
    parts.box((L_IN, W_OUT, WALL), at=(0, 0, H_OUT - WALL / 2), color=FILM, visible=False),  # upper film
    *[parts.box((L_IN, WALL, H_IN), at=(0, s * (W_OUT - WALL) / 2, WALL + H_IN / 2), color=FILM, visible=False)
      for s in (1, -1)],
    *[parts.box((L_IN, 0.0015, 0.0015), at=(0, s * (W_OUT / 2 - 0.00075), H_OUT - 0.00075), color=FILM, collide=False)
      for s in (1, -1)],  # the film's two upper edges, so the clear sleeve shows
    *[parts.box((L_IN, 0.0015, 0.0015), at=(0, s * (W_OUT / 2 - 0.00075), 0.00075), color=FILM, collide=False)
      for s in (1, -1)],
    parts.box((0.045, 0.034, 0.0006), at=(0.012, 0, H_OUT + 0.0003), color=SEAL, collide=False),  # printed label
    parts.box((0.045, 0.008, 0.0004), at=(0.012, 0.010, H_OUT + 0.0008), color=(0.80, 0.10, 0.10), collide=False),
    parts.box((0.030, 0.004, 0.0004), at=(0.012, -0.004, H_OUT + 0.0008), color=(0.15, 0.30, 0.55), collide=False),
    parts.box((0.022, 0.003, 0.0004), at=(0.012, -0.010, H_OUT + 0.0008), color=(0.15, 0.30, 0.55), collide=False),
    # glints of the clear film across the top and down the sides, so the wrapped fish reads as wrapped
    *[parts.box((0.0015, W_OUT, 0.0006), at=(x, 0, H_OUT + 0.0003), color=GLINT, collide=False)
      for x in (-0.045, -0.030, 0.034, 0.044)],
    *[parts.box((L_IN, 0.0006, 0.004), at=(0, s * (W_OUT / 2 + 0.0003), H_OUT - 0.006), color=GLINT, collide=False)
      for s in (1, -1)],
]
END_JOINTS = (
    parts.Hinge("end_px", shapes=_end_shapes(1), pivot=(L_IN / 2, 0, 0), axis=(0, 1, 0), lower=0.0, upper=2.6,
                mass=0.012, spring=K_SEAL, damping=0.002, max_effort=1.0),
    parts.Hinge("end_nx", shapes=_end_shapes(-1), pivot=(-L_IN / 2, 0, 0), axis=(0, -1, 0), lower=0.0, upper=2.6,
                mass=0.012, spring=K_SEAL, damping=0.002, max_effort=1.0),
)
POUCH_XY = (0.265, 0.020)  # its near flat end 2 mm in from the board's near edge
POUCH = parts.Part("pouch", pos=(*POUCH_XY, BOARD_TOP), mass=0.03, friction=(0.5, 0.4), contact_offset=0.003,
                   shapes=POUCH_SHAPES, joints=END_JOINTS)
FISH_PART = parts.Part("salmon", pos=(*POUCH_XY, BOARD_TOP + WALL + FISH[2] / 2), mass=FISH_MASS,
                       friction=(0.2, 0.15), contact_offset=0.003,
                       shapes=[parts.box(FISH, color=SALMON),
                               parts.box((FISH[0] - 0.01, 0.004, 0.001), at=(0, 0.012, FISH[2] / 2 + 0.0004),
                                         color=FAT, collide=False),
                               parts.box((FISH[0] - 0.01, 0.004, 0.001), at=(0, -0.012, FISH[2] / 2 + 0.0004),
                                         color=FAT, collide=False)])

# -- the shears: OXO Good Grips poultry shears 1072292 (241 mm, 102 mm blades, spring-loaded), lying open ---------
# Shears frame: origin at the pivot, halfway up the 19 mm handles; x along the blades (their bisector), z up. Each
# half is a pivot boss, a shank and the front of its rubber grip (collided), the rest of its handle and its blade
# (seen, not collided). The steel blades have no collider, so each half's centre of mass is set where the real one is
# estimated to be: 28 mm behind the pivot. The hand takes the grips just behind that, about 33 mm behind the pivot:
# open they are 63 mm across there, shut 44 mm (side by side).
HALF = 0.30  # rad: each half 17 degrees off the bisector when open (blade tips 60 mm apart)
SHUT = 2 * HALF  # the jaw's angle when the blades are closed
BLADE, HANDLE_T = 0.102, 0.019
GRIP_D, GRIP_RAD, GRIP_L, GRIP_W = 0.035, 0.010, 0.024, 0.020  # grip centre 35 mm back along its handle, 10 mm out
COM_D = 0.028
K_SHEARS, PRELOAD = 0.6, 0.4  # N m/rad; the spring pushes the shears open with 0.24 N m at the open stop
HOLD_JAW = 0.25  # the jaw angle at which the hand holds the shears (grips squeezed 5 mm each side)
_u1 = np.array([math.cos(HALF), math.sin(HALF), 0.0])
_u2 = np.array([math.cos(HALF), -math.sin(HALF), 0.0])
_n1 = np.array([math.sin(HALF), -math.cos(HALF), 0.0])  # each half's outward side
_n2 = np.array([math.sin(HALF), math.cos(HALF), 0.0])


def _half(u, n, ang, dz, grip_yaw):
    grip = -u * GRIP_D + n * GRIP_RAD
    return (
        parts.cylinder(0.012, HANDLE_T, at=(0, 0, 0), color=STEEL),  # the pivot boss
        parts.box((0.016, 0.014, HANDLE_T), at=tuple(-u * 0.016), color=GRIP, quat=_qz(ang)),  # shank
        parts.box((GRIP_L, GRIP_W, HANDLE_T), at=tuple(grip), color=GRIP, quat=_qz(grip_yaw),
                  friction=(1.4, 1.2)),  # rubber grip, its outer face square to the fingers when they hold it
        parts.box((0.095, 0.018, 0.016), at=tuple(-u * 0.095 + n * GRIP_RAD), color=GRIP, quat=_qz(ang),
                  collide=False),  # the rest of the handle
        parts.box((BLADE, 0.012, 0.002), at=tuple(u * BLADE / 2 + [0, 0, dz]), color=STEEL, quat=_qz(ang),
                  collide=False),  # blade: seen, not collided; cuts follow the rule in observe()
    )


def _grip_corners(jaw: float) -> tuple[np.ndarray, np.ndarray]:
    """Corners of the two grips (plan view) in the frame of a hand holding them symmetrically at jaw angle ``jaw``:
    x along the line midway between the halves (toward the blades), y across."""
    out = []
    for u, n, gy, sgn in ((_u1, _n1, HOLD_JAW / 2, -1), (_u2, _n2, -HOLD_JAW / 2, 1)):
        c = -u * GRIP_D + n * GRIP_RAD
        rg = np.array([[math.cos(gy), -math.sin(gy), 0], [math.sin(gy), math.cos(gy), 0], [0, 0, 1]])
        pts = np.array([c + rg @ np.array([sx * GRIP_L / 2, sy * GRIP_W / 2, 0]) for sx in (-1, 1) for sy in (-1, 1)])
        a = sgn * jaw / 2
        r = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        out.append(pts @ r.T)
    return out[0], out[1]


def grip_span(jaw: float) -> float:
    """Outside width (m) across the two grips at jaw angle ``jaw``."""
    a, b = _grip_corners(jaw)
    return float(b[:, 1].max() - a[:, 1].min())


def grip_reach(jaw: float) -> float:
    """Distance (m) from the pivot back to the middle of the two grips, at jaw angle ``jaw``."""
    a, b = _grip_corners(jaw)
    return float(-(a[:, 0].mean() + b[:, 0].mean()) / 2)


SHEARS_AT = (0.250, 0.170)  # the pivot; the blades point away from the arms, the handles toward them
SHEARS = parts.Part(
    "shears", pos=(*SHEARS_AT, BOARD_TOP + HANDLE_T / 2), mass=0.11, friction=(0.9, 0.8), contact_offset=0.004,
    shapes=_half(_u1, _n1, HALF, 0.0012, HOLD_JAW / 2),
    joints=(parts.Hinge("jaw", shapes=_half(_u2, _n2, -HALF, -0.0012, -HOLD_JAW / 2), pivot=(0, 0, 0), axis=(0, 0, 1),
                        lower=0.0, upper=SHUT, mass=0.09, spring=K_SHEARS, damping=0.02, max_effort=3.0,
                        target=-PRELOAD),),
    com=(("base", tuple(-_u1 * COM_D)), ("jaw", tuple(-_u2 * COM_D))))

# -- the dish: Pyrex Simply Store 3-cup rectangle (no lid), 184 x 140 x 44 mm --------------------------------------
DISH_L, DISH_W, DISH_H, DISH_T = 0.184, 0.140, 0.044, 0.005
DISH_IN = (DISH_L - 2 * DISH_T, DISH_W - 2 * DISH_T)
DISH_SHAPES = [
    parts.box((DISH_L, DISH_W, DISH_T), at=(0, 0, DISH_T / 2), color=GLASS),
    *[parts.box((DISH_T, DISH_W, DISH_H - DISH_T), at=(s * (DISH_L - DISH_T) / 2, 0, (DISH_H + DISH_T) / 2),
                color=GLASS) for s in (1, -1)],
    *[parts.box((DISH_L - 2 * DISH_T, DISH_T, DISH_H - DISH_T), at=(0, s * (DISH_W - DISH_T) / 2, (DISH_H + DISH_T) / 2),
                color=GLASS) for s in (1, -1)],
]
DISH_XY = (0.260, -0.165)
DISH = parts.Part("dish", pos=(*DISH_XY, TOP), yaw=math.pi / 2, mass=0.45, friction=(0.4, 0.3), shapes=DISH_SHAPES)

# -- the scrap bowl: Rachael Ray 4 qt melamine garbage bowl, about 260 mm across and 146 mm tall ------------------
BOWL_R_TOP, BOWL_R_FOOT, BOWL_H, BOWL_N = 0.130, 0.075, 0.146, 16
_lean = math.atan2(BOWL_R_TOP - BOWL_R_FOOT, BOWL_H)


def _bowl_shapes():
    out = [parts.cylinder(BOWL_R_FOOT + 0.004, 0.006, at=(0, 0, 0.003), color=RED)]
    r_mid = (BOWL_R_TOP + BOWL_R_FOOT) / 2
    slant = math.hypot(BOWL_H, BOWL_R_TOP - BOWL_R_FOOT)
    width = 2 * BOWL_R_TOP * math.tan(math.pi / BOWL_N) + 0.003
    for k in range(BOWL_N):
        phi = 2 * math.pi * k / BOWL_N
        # turn about z by phi, then lean the wall's top outward about the wall's own tangent axis
        qz = np.array(_qz(phi))
        qy = np.array([0.0, math.sin(_lean / 2), 0.0, math.cos(_lean / 2)])  # top outward
        x1, y1, z1, w1 = qz
        x2, y2, z2, w2 = qy
        q = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
        out.append(parts.box((0.005, width, slant), at=(r_mid * math.cos(phi), r_mid * math.sin(phi), BOWL_H / 2),
                             color=RED, quat=q))
    return out


BOWL_XY = (0.200, -0.440)
BOWL = parts.Part("bowl", pos=(*BOWL_XY, TOP), mass=0.55, friction=(0.6, 0.5), shapes=_bowl_shapes())

OBJECTS = [BOARD, POUCH, FISH_PART, SHEARS, DISH, BOWL]


# Lot A: the dish just right of the board's near corner, the bowl beyond it. Lot B: the dish 40 mm further from the
# arms and 5 mm further right, the bowl 70 mm further out and 10 mm further right. Only the dish and the bowl move between lots (the jointed
# film and shears always start where they are declared).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"dish": tuple(DISH.pos), "bowl": tuple(BOWL.pos)}, side=-1),
    lots.Lot("B", place={"dish": (0.300, -0.170, TOP), "bowl": (0.270, -0.450, TOP)}, side=-1),
)

# -- geometry used by the scorer --------------------------------------------------------------------------------------


def _frame(st):
    return np.asarray(st.pos, float), _rot(st.quat_xyzw)


def _link(st, name):
    if name == "base":
        return _frame(st)
    p, q = st.links[name]
    return np.asarray(p, float), _rot(q)


def _part_boxes(part, st, collide_only=True):
    """World (centre, rotation, size) of a part's shapes in state ``st``."""
    poses = {"base": _frame(st)}
    for j in part.joints:
        if j.name in st.links:
            poses[j.name] = _link(st, j.name)
    out = []
    for name, origin, shapes in part.links:
        if name not in poses:
            continue
        p, r = poses[name]
        for s in shapes:
            if collide_only and not s.collide:
                continue
            out.append((p + r @ (np.asarray(s.at) - origin), r @ s.R, 2 * s.half))
    return out


_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) / 2


def _corners(boxes) -> np.ndarray:
    return np.concatenate([c + (_CORNERS * s) @ r.T for c, r, s in boxes] + [np.array([c for c, _, _ in boxes])])


def _box_gap(pts, centre, rot, size) -> float:
    """Smallest signed distance (m, negative inside) from any point to one oriented box."""
    loc = (np.asarray(pts, float) - centre) @ rot
    q = np.abs(loc) - np.asarray(size, float) / 2
    out = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    return float(out.min())


def fish_corners(objects) -> np.ndarray:
    return _corners(_part_boxes(FISH_PART, objects["salmon"]))


def bag_points(objects) -> np.ndarray:
    """Corners and centres of every piece of film, the flat seals included."""
    return _corners(_part_boxes(POUCH, objects["pouch"], collide_only=False))


def in_dish(objects, pts) -> np.ndarray:
    """Which points are inside the dish: within its inner walls and below its rim."""
    o, r = _frame(objects["dish"])
    loc = (np.asarray(pts, float) - o) @ r
    return ((np.abs(loc[:, 0]) <= DISH_IN[0] / 2 + EPS) & (np.abs(loc[:, 1]) <= DISH_IN[1] / 2 + EPS)
            & (loc[:, 2] <= DISH_H + EPS) & (loc[:, 2] >= -0.010))


def tilt_deg(st) -> float:
    return math.degrees(math.acos(float(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


def fish_in_dish(objects) -> bool:
    """The whole fish inside the dish's inner walls, lying on its floor (lowest point within 4 mm of the floor)."""
    pts = fish_corners(objects)
    o, r = _frame(objects["dish"])
    loc = (pts - o) @ r
    inside = bool(np.all(np.abs(loc[:, 0]) <= DISH_IN[0] / 2 + EPS) and np.all(np.abs(loc[:, 1]) <= DISH_IN[1] / 2 + EPS))
    low = float(loc[:, 2].min())
    return inside and DISH_T - 0.002 <= low <= DISH_T + 0.004 + EPS and tilt_deg(objects["dish"]) <= 10.0 + EPS


def _over(objects, name, xy, margin=0.0) -> bool:
    """Is world point xy over the named flat object's outline (board or dish)?"""
    st = objects[name]
    o, r = _frame(st)
    loc = (np.array([xy[0], xy[1], o[2]]) - o) @ r
    half = (BOARD_L / 2, BOARD_W / 2) if name == "board" else (DISH_L / 2, DISH_W / 2)
    return abs(loc[0]) <= half[0] + margin and abs(loc[1]) <= half[1] + margin


def _in_bowl_xy(objects, p, r_max) -> bool:
    o = np.asarray(objects["bowl"].pos, float)
    return bool(math.hypot(p[0] - o[0], p[1] - o[1]) <= r_max + EPS)


def bag_in_bowl(objects) -> bool:
    """Every piece of film inside the scrap bowl: each film panel's centre within 110 mm of the bowl's axis and at
    least 10 mm below its rim."""
    o = np.asarray(objects["bowl"].pos, float)
    for c, _, _ in _part_boxes(POUCH, objects["pouch"]):
        if not (_in_bowl_xy(objects, c, 0.110) and c[2] <= o[2] + BOWL_H - 0.010 + EPS):
            return False
    return True


def shears_down(objects) -> bool:
    """The shears lie on the board or the counter, outside the dish and the bowl: their lowest collided point within
    5 mm of the board top or the counter, lying flat (within 15 degrees)."""
    st = objects["shears"]
    pts = _corners(_part_boxes(SHEARS, st))
    low = float(pts[:, 2].min())
    piv = np.asarray(st.pos, float)
    on_board = _over(objects, "board", piv[:2]) and abs(low - (objects["board"].pos[2] + BOARD_T)) <= 0.005 + EPS
    on_counter = (not _over(objects, "board", piv[:2])) and abs(low - TOP) <= 0.005 + EPS
    away = not _over(objects, "dish", piv[:2], 0.02) and not _in_bowl_xy(objects, piv, BOWL_R_TOP + 0.02)
    return bool((on_board or on_counter) and away and tilt_deg(st) <= 15.0 + EPS)


# -- the cut rule ---------------------------------------------------------------------------------------------------
# The blades have no collider. When the jaw closes (from at least 9 degrees open to within 2 degrees of shut, within
# 2 s), the film that lay between the open blades is cut: where the film's flat end crosses the blades' plane inside
# the wedge the blades swept as the hand closed them (from the pivot out to the 102 mm blade length, as wide as they
# were last open), the two planes crossing at 20 degrees or more. Film from 4 mm inside an end panel to the start of
# that end's seal band (its 28 mm of unwelded film) is cut there; cuts at one end add up, and once they cover 80 mm or
# more of the 90 mm flat end (enough for the fish's 160 mm girth to pass) that end opens: its panel is free to swing.
# Cuts only in the seal band open nothing. A cut all the way across (to within 1 mm of both sides) cuts the strip off;
# it falls straight down from there, so over the dish that is film in the dish. If any part of the fish lies in the
# swept wedge the fish is cut, and closing the jaw on the fish at any time also cuts it; a blade tip pushed 3 mm into
# the fish pierces it: each is 0 for the trial. A closing during which a blade passes more than 1 mm into the board or
# the counter cuts nothing.
OPEN_AT = SHUT - D(9.0)
SHUT_AT = SHUT - D(2.0)
OPEN_SPAN = 0.080
MIN_CROSS = D(20.0)
_perp1 = np.array([-_u1[1], _u1[0], 0.0])


def blade_tips(objects) -> np.ndarray:
    st = objects["shears"]
    o, r = _frame(st)
    th = float(st.joints.get("jaw", 0.0))
    rz = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
    return np.array([o + r @ (_u1 * BLADE), o + r @ (rz @ (_u2 * BLADE))])


def _wedge(objects, a0, a1, base_frame=False, nr=34, na=9) -> np.ndarray:
    """World points over the region of the blades' plane between angles a0..a1 (rad) and radii 0..102 mm. Angles are
    measured from the closed blade line (u1) when shut, or from the base frame's x axis with ``base_frame``."""
    o, r = _frame(objects["shears"])
    rr = np.linspace(0.004, BLADE, nr)
    aa = np.linspace(a0, a1, na)
    pts = []
    for a in aa:
        d = np.array([math.cos(a), math.sin(a), 0.0]) if base_frame else _u1 * math.cos(a) + _perp1 * math.sin(a)
        pts.append(rr[:, None] * d)
    return o + np.concatenate(pts) @ r.T


def _in_fish(objects, pts, margin) -> bool:
    fo, fr = _frame(objects["salmon"])
    loc = (np.asarray(pts, float) - fo) @ fr
    return bool(np.any(np.all(np.abs(loc) <= np.asarray(FISH) / 2 + margin, axis=1)))


def below_surface(objects, pts) -> bool:
    """Is any point more than 1 mm into the board or the counter under it?"""
    bo, br = _frame(objects["board"])
    loc = (np.asarray(pts, float) - bo) @ br
    over = (np.abs(loc[:, 0]) <= BOARD_L / 2) & (np.abs(loc[:, 1]) <= BOARD_W / 2)
    floor = np.where(over, bo[2] + BOARD_T, TOP)
    return bool(np.any(np.asarray(pts)[:, 2] < floor - 0.001))


def evaluate_cut(objects, gap=SHUT) -> dict:
    """What a closing that ends at the shears' current (shut) pose cuts, for blades last open by ``gap`` rad:
    {'end': 'px'|'nx'|None, 'y': (lo, hi) across that end (m), 'at': mean distance beyond its panel (m),
    'band': bool, 'fish': bool, 'cross': deg, 'solid': bool}."""
    wedge = _wedge(objects, -gap / 2, gap / 2)
    fish = _in_fish(objects, wedge, 0.001)
    po, pr = _frame(objects["pouch"])
    so, sr = _frame(objects["shears"])
    nf, nb = pr[:, 2], sr[:, 2]
    cross = math.asin(min(1.0, float(np.linalg.norm(np.cross(nf, nb)))))
    best = dict(end=None, y=None, at=None, band=False, fish=fish, cross=round(math.degrees(cross), 1),
                solid=below_surface(objects, wedge))
    if cross < MIN_CROSS - EPS or best["solid"]:
        return best
    # the line where the flat film's plane meets the blades' plane
    l = np.cross(nf, nb)
    l /= np.linalg.norm(l)
    pf = po + pr @ np.array([0.0, 0.0, 0.0008])
    a = np.array([nf, nb, l])
    p0 = np.linalg.solve(a, np.array([nf @ pf, nb @ so, l @ so]))
    line = p0 + np.arange(-0.20, 0.20, 0.0005)[:, None] * l
    loc_s = (line - so) @ sr
    along, perp = loc_s @ _u1, loc_s @ _perp1
    inside = (np.hypot(along, perp) <= BLADE + EPS) & (along > 0.0) & (np.abs(np.arctan2(perp, along)) <= gap / 2 + EPS)
    if not np.any(inside):
        return best
    loc = (line[inside] - po) @ pr
    for name, sgn in (("px", 1), ("nx", -1)):
        if float(objects["pouch"].joints.get(f"end_{name}", 0.0)) > 0.2:
            continue  # already open: its flat end has swung away
        d = sgn * loc[:, 0] - END_X
        wide = np.abs(loc[:, 1]) <= FLAT_W / 2 + EPS
        cut = wide & (d >= -0.004 - EPS) & (d <= LOOSE + EPS)
        best["band"] = best["band"] or bool(np.any(wide & (d > LOOSE) & (d <= LOOSE + BAND + EPS)))
        if np.any(cut):
            ys = loc[cut, 1]
            best.update(end=name, y=(round(float(ys.min()), 4), round(float(ys.max()), 4)),
                        at=round(float(d[cut].mean()), 4))
    return best


def _union(spans) -> float:
    total, cur = 0.0, None
    for lo, hi in sorted(spans):
        if cur is None or lo > cur[1]:
            if cur is not None:
                total += cur[1] - cur[0]
            cur = [lo, hi]
        else:
            cur[1] = max(cur[1], hi)
    return total + (cur[1] - cur[0] if cur else 0.0)


def biting(objects, prev_jaw) -> bool:
    """The jaw is closing (by more than 0.5 degree since the last step) with the fish between the blades."""
    th = float(objects["shears"].joints.get("jaw", 0.0))
    if th <= prev_jaw + D(0.5):
        return False
    return _in_fish(objects, _wedge(objects, -HALF + th, HALF, base_frame=True), 0.0)


def closing_on_hand(objects, prev_jaw, hands) -> bool:
    """The jaw is closing (by more than 0.5 degree since the last step) with a gripper or fingertip between the blades,
    within 3 mm of their plane."""
    th = float(objects["shears"].joints.get("jaw", 0.0))
    if hands is None or th <= prev_jaw + D(0.5):
        return False
    o, r = _frame(objects["shears"])
    near = hands[np.linalg.norm(hands - o, axis=1) < BLADE + 0.01]
    if len(near) == 0:
        return False
    loc = (near - o) @ r
    ang = np.arctan2(loc[:, 1], loc[:, 0])
    rad = np.hypot(loc[:, 0], loc[:, 1])
    return bool(np.any((np.abs(loc[:, 2]) <= 0.003) & (rad <= BLADE) & (rad > 0.012) & (ang >= -HALF + th - EPS)
                       & (ang <= HALF + EPS)))


def pierced(objects) -> bool:
    """A blade tip 3 mm or more inside the fish."""
    fo, fr = _frame(objects["salmon"])
    loc = (blade_tips(objects) - fo) @ fr
    return bool(np.any(np.all(np.abs(loc) <= np.asarray(FISH) / 2 - 0.003, axis=1)))


# -- the per-trial record (kept in parts' namespace so near_misses.py and the simulator share it) ---------------------
_rec = parts.__dict__.setdefault("salmon_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(zero=False, why="", opened={"px": False, "nx": False}, spans={"px": [], "nx": []}, gaps=None,
                known=True, reason="", open_t=-1e9, open_th=0.0, shut=False, prev_jaw=0.0, cuts=[], t=0.0, steps=0,
                last_t=None)


if not _rec:
    reset_record()


def instant_zero(objects) -> str:
    """A whole-trial failure visible in this state, or ''."""
    for name in ("board", "pouch", "salmon", "shears", "dish", "bowl"):
        if objects[name].pos[2] < TOP - 0.05:
            return f"{name} off the counter"
    if np.any(in_dish(objects, bag_points(objects))):
        return "film in the dish"
    fc = fish_corners(objects)
    fo = np.asarray(objects["salmon"].pos, float)
    low = float(fc[:, 2].min())
    for c in fc[fc[:, 2] < TOP + 0.004]:  # any corner down at the counter, not over the board or the dish
        if not _over(objects, "board", c[:2]) and not _over(objects, "dish", c[:2]):
            return "fish on the counter"
    bo = np.asarray(objects["bowl"].pos, float)
    if np.any(np.hypot(fc[:, 0] - bo[0], fc[:, 1] - bo[1]) <= BOWL_R_TOP) and low < bo[2] + BOWL_H:
        return "fish in the scrap bowl"
    if tilt_deg(objects["dish"]) > 30.0:
        return "dish tipped"
    if pierced(objects):
        return "fish pierced by a blade tip"
    return ""


def _latch(why: str) -> None:
    if why and not _rec["zero"]:
        _rec["zero"], _rec["why"] = True, why


def observe(objects, gaps, t, known=True, hands=None) -> None:
    """One control step: latch whole-trial failures, apply the cut rule, keep the hands' distances."""
    _rec["steps"] += 1
    _rec["t"] = t
    if not known:
        _rec["known"] = False
        return
    _rec["gaps"] = gaps
    _latch(instant_zero(objects))
    th = float(objects["shears"].joints.get("jaw", 0.0))
    if biting(objects, _rec["prev_jaw"]):
        _latch("shears closed on the fish")
    if closing_on_hand(objects, _rec["prev_jaw"], hands):
        _latch("shears closed on a gripper")
    _rec["prev_jaw"] = th
    if th <= OPEN_AT:
        _rec["open_t"], _rec["open_th"] = t, th
        _rec["shut"] = False
    elif th >= SHUT_AT and not _rec["shut"]:
        _rec["shut"] = True
        if t - _rec["open_t"] <= 2.0 + EPS:
            res = evaluate_cut(objects, SHUT - _rec["open_th"])
            res["t"] = round(t, 3)
            _rec["cuts"].append(res)
            if res["fish"]:
                _latch("fish cut by the shears")
            elif res["end"]:
                end = res["end"]
                _rec["spans"][end].append(res["y"])
                lo, hi = min(v[0] for v in _rec["spans"][end]), max(v[1] for v in _rec["spans"][end])
                if _union(_rec["spans"][end]) >= OPEN_SPAN - EPS:
                    _rec["opened"][end] = True
                if lo <= -FLAT_W / 2 + 0.001 + EPS and hi >= FLAT_W / 2 - 0.001 - EPS:  # cut right through
                    po, pr = _frame(objects["pouch"])
                    sgn = 1 if end == "px" else -1
                    strip = po + pr @ np.array([sgn * (END_X + LOOSE + BAND / 2), 0.0, 0.0])
                    if _over(objects, "dish", strip[:2]):
                        _latch("cut-off strip fell into the dish")


def _gap_ok(name, d) -> bool:
    g = _rec.get("gaps")
    return g is not None and name in g and g[name] is not None and g[name] >= d - EPS


def away_from_dish(objects) -> bool:
    """No film and no part of the shears in or over the dish (its outline grown by 10 mm), at any height."""
    o, r = _frame(objects["dish"])
    for pts in (bag_points(objects), _corners(_part_boxes(SHEARS, objects["shears"], collide_only=False))):
        loc = (np.asarray(pts) - o) @ r
        if np.any((np.abs(loc[:, 0]) <= DISH_L / 2 + 0.010) & (np.abs(loc[:, 1]) <= DISH_W / 2 + 0.010)):
            return False
    return True


def score(objects):
    """0 for the trial if a whole-trial rule broke. Else 0.25 the film cut open across an end; 0.5 also the whole
    fish lying in the dish, let go; 0.75 also no film and no shears in or over the dish; 1.0 also both hands 30 mm
    clear of fish, dish, film and shears, and everything still."""
    if _rec["zero"] or not _rec["known"] or instant_zero(objects):
        return 0.0
    if not any(_rec["opened"].values()):
        return 0.0
    if not (fish_in_dish(objects) and _gap_ok("salmon", 0.005) and objects["salmon"].resting()):
        return 0.25
    if not away_from_dish(objects):
        return 0.5
    still = all(objects[n].resting() for n in ("salmon", "pouch", "shears", "dish"))
    clear = all(_gap_ok(n, 0.030) for n in ("salmon", "pouch", "shears", "dish"))
    if not (still and clear):
        return 0.75
    return 1.0


def success(objects):
    return score(objects) >= 1.0 - EPS


def distance(objects):
    if not any(_rec["opened"].values()):
        return 1.0
    f = np.asarray(objects["salmon"].pos, float)
    d = np.asarray(objects["dish"].pos, float)
    return float(np.linalg.norm(f[:2] - d[:2]))


# -- the hands, as points sampled over their collision hulls (faces every 3 to 5 mm) ----------------------------------
HAND_LINKS = ("link5", "gripper", "tip_left", "tip_right")
STEP = {"link5": 0.006, "gripper": 0.005, "tip_left": 0.003, "tip_right": 0.003}
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


def link_points(name, step) -> np.ndarray:
    key = (name, step)
    if key not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        _FACES[key] = np.concatenate([_hull_face_points(h, step) for h in hs]) if hs else np.zeros((0, 3))
    return _FACES[key]


def hand_points(link_poses: dict) -> np.ndarray:
    return np.concatenate([link_points(name, STEP[name]) @ m[:3, :3].T + m[:3, 3]
                           for (_, name), m in link_poses.items() if name in HAND_LINKS])


_PARTS = {"salmon": FISH_PART, "pouch": POUCH, "shears": SHEARS, "dish": DISH}


def hand_gaps(objects, hands) -> dict:
    out = {}
    for name, part in _PARTS.items():
        st = objects[name]
        near = hands[np.linalg.norm(hands - np.asarray(st.pos, float), axis=1) < 0.40]
        if len(near) == 0:
            out[name] = 0.40
            continue
        out[name] = min(_box_gap(near, c, r, s) for c, r, s in _part_boxes(part, st, collide_only=False))
    return out


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


def _restore(sim) -> None:
    for j in ("end_px", "end_nx"):
        try:
            sim.set_stiffness("pouch", j, K_SEAL)
        except Exception:  # noqa: BLE001
            pass


@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _restore(sim)


_stock_apply = lots._apply


def _apply_lot(emb) -> bool:
    moved = _stock_apply(emb)
    reset_record()
    if os.environ.get("SALMON_PLACE_LOG"):
        base = emb._base
        org = parts._np(base.scene.env_origins)[0]
        st = {o.name: np.round(parts._np(base.scene[o.name].data.root_link_pose_w)[0, :3] - org[:3], 3).tolist()
              for o in OBJECTS}
        print(f"[salmon] lot {lots.current().name} placed: {st}", file=sys.stderr, flush=True)
    return moved


lots._apply = _apply_lot


def _log_path():
    if os.environ.get("TRACE_OUT"):
        return str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("salmon.jsonl"))
    return os.environ.get("SALMON_LOG")


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:
        return
    _rec["last_t"] = sim.t
    known, gaps, err, hpts = True, None, "", None
    try:
        objects = sim.states()
        poses = _arm_poses(sim)
        if poses is None:
            raise ValueError("arm poses not finite")
        hpts = hand_points(poses)
        gaps = hand_gaps(objects, hpts)
        _rec["tcps"] = {k[0]: m for k, m in poses.items() if k[1] == "gripper"}
    except Exception as e:  # noqa: BLE001
        known, err = False, str(e)
        objects = None
    if objects is None:
        _rec["steps"] += 1
        _rec["known"] = False
        _rec["reason"] = err
        return
    before = dict(_rec["opened"])
    observe(objects, gaps, float(sim.t), known, hpts)
    for end, now in _rec["opened"].items():
        if now and not before[end]:
            try:
                sim.set_stiffness("pouch", f"end_{end}", 0.0)
            except Exception as e:  # noqa: BLE001
                _rec["known"], _rec["reason"] = False, f"could not free the end: {e}"
    log = _log_path()
    if log and (_rec["steps"] % 5 == 0 or _rec["opened"] != before):
        try:
            sh = objects["shears"]
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), zero=_rec["zero"],
                       why=_rec["why"], opened=_rec["opened"], cuts=_rec["cuts"][-1:],
                       jaw=round(float(sh.joints.get("jaw", 0.0)), 3),
                       ends=[round(float(objects["pouch"].joints.get(k, 0.0)), 3) for k in ("end_px", "end_nx")],
                       fish=np.round(objects["salmon"].pos, 4).tolist(), pouch=np.round(objects["pouch"].pos, 4).tolist(),
                       pouch_q=np.round(objects["pouch"].quat_xyzw, 3).tolist(),
                       shears=np.round(sh.pos, 4).tolist(), shears_q=np.round(sh.quat_xyzw, 3).tolist(),
                       dish=np.round(objects["dish"].pos, 4).tolist(),
                       gaps={k: round(float(v), 4) for k, v in (gaps or {}).items()},
                       fish_in_dish=fish_in_dish(objects), bag_in_bowl=bag_in_bowl(objects),
                       shears_down=shears_down(objects), known=_rec["known"])
            if _rec["steps"] % 5 == 0 and objects.get("shears") is not None:
                row["cut_now"] = evaluate_cut(objects)
                row["tcp"] = {k: np.round(m[:3, 3], 4).tolist() for k, m in (_rec.get("tcps") or {}).items()}
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the layout the top camera shows ------------------------------------------------------------
# The arm on the dish's side takes the portion by the film round its far (+x) end, lifts it, turns it a quarter turn so
# it lies across the arms (its other end toward the dish's side) over the dish, and leans it 45 degrees sideways so
# that end hangs down. The other arm picks up the shears by their grips, carries them blades-first toward the hanging
# end and slides the open blades either side of its flat film, 16 mm beyond the end panel, then squeezes them shut;
# the end opens and the frozen fish slides out into the dish. The shears are let open by their spring and put back on
# the board; the empty film is dropped into the scrap bowl. SALMON_DEMO=seal cuts through the seal band instead
# (nothing opens: 0); SALMON_DEMO=bag puts the sealed portion in the dish (0).
LEAN = 45.0
POUR_UP = 0.170  # the holding hand's TCP above the counter while it leans the portion over the dish
POUR_SIDE = 0.105  # ...and this far from the dish's middle, away from the dish's side (y)
HOLD_SAG = (0.0, 0.0, 0.0)  # measured correction to the holding hand's aim (x, toward its own side, z)
SHEAR_SAG = (-0.0073, 0.0, 0.004)  # measured (trace, lot A): the shears close 7 mm further out and 4 mm lower
# relative to the film than commanded (the hand sags and the shears shift as they shut); the hand aims to undo that
GRIP_IN = 0.025  # the hand holds the film this far in from its far end panel
GRIP_UP = 0.016  # TCP above the film's underside when it takes hold
SQUEEZE = -(W_OUT / 2 - 0.0035)
CUT_AT = 0.016  # the cut 16 mm beyond the end panel
TIP_SHORT = 0.0055  # shut, the blade tips stop 5.5 mm short of the flat end's far side (the strip stays on)
SH_HOLD = -(grip_span(HOLD_JAW) / 2 - 0.001)  # holds the grips squeezed to HOLD_JAW (their faces square to the pads)
SH_OPEN = -0.040
SH_CLOSE = -(grip_span(SHUT) / 2 - 0.007)  # 7 mm inside the shut grips: the fingers' drive needs that to beat the spring
SH_TCP_UP = BOARD_T + 0.016 - (BOARD_T + HANDLE_T / 2)  # the hand holds the grips 6.5 mm above their middle (finger
# tips 3 mm above the board), so the blades' plane is this far below the TCP


def _down(yaw_deg=0.0):
    return tilt.rotation(D(yaw_deg), 0.0, D(yaw_deg))


def _lean_rot(yaw_deg, deg=LEAN):
    """The grip turned ``yaw_deg``, its fingertips leaned ``deg`` toward that same heading."""
    return tilt.rotation(D(yaw_deg), float(deg), D(yaw_deg))


def _w(d, p, finger, label, rot):
    return Waypoint(float(d), tuple(float(v) for v in p), float(finger), label, tilt._TiltedYaw(0.0, rot))


def _rest(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), rot


def pour_geometry(dish_xy, side):
    """The holding hand's TCP, the cut point, the lean's rotation (relative to the straight-down grip turned ``psi``)
    and the turn ``psi`` (deg), for a dish at dish_xy on the side ``side`` (-1 right, +1 left)."""
    psi = -side * 90.0  # the portion's far end toward the middle, its near end toward the dish's side
    g = np.array([dish_xy[0], dish_xy[1] - side * POUR_SIDE, TOP + POUR_UP])
    rel = _lean_rot(psi) @ _down(psi).T
    rz = _down(psi) @ _down(0.0).T  # the quarter turn (the portion was picked up lying along x)
    tcp_local = np.array([END_X - GRIP_IN, 0.0, GRIP_UP])
    cut_local = np.array([-(END_X + CUT_AT), 0.0, 0.0008])
    cut = g + rel @ (rz @ (cut_local - tcp_local))
    return g, cut, rel, psi


def demo():
    lot = lots.current()
    side = lot.side  # -1: the dish is on the right (lot A); +1: on the left
    hold_arm, cut_arm = ("right", "left") if side < 0 else ("left", "right")
    mode = os.environ.get("SALMON_DEMO", "")
    px, py, pz = lot.pos("pouch")
    sx, sy, sz = lot.pos("shears")
    dx, dy, _ = lot.pos("dish")
    bx, by, _ = lot.pos("bowl")
    g, cut, _, psi = pour_geometry((dx, dy), side)
    d0 = _down(0.0)
    dpsi = _down(psi)
    lean = _lean_rot(psi)

    # the holding hand: take the film round the fish's far end, lift, turn, lean over the dish
    grip = np.array([px + END_X - GRIP_IN, py, pz + GRIP_UP])
    hold = [
        _w(2.0, grip + [0, 0, 0.09], FINGER_OPEN, "over the film's far end", d0),
        _w(1.5, grip, FINGER_OPEN, "down round the film", d0),
        _w(0.8, grip, SQUEEZE, "take hold", d0),
        _w(1.2, grip + [0, 0, 0.06], SQUEEZE, "lift it off the board", d0),
    ]
    if mode == "bag":  # the literal errand: the sealed portion into the dish
        hold += [_w(2.5, (dx, dy, TOP + 0.12), SQUEEZE, "over the dish", dpsi),
                 _w(1.5, (dx, dy - side * (END_X - GRIP_IN), TOP + DISH_T + GRIP_UP + 0.006), SQUEEZE, "into the dish", dpsi),
                 _w(0.6, (dx, dy - side * (END_X - GRIP_IN), TOP + DISH_T + GRIP_UP + 0.006), FINGER_OPEN, "let go", dpsi),
                 _w(1.5, (dx, dy, TOP + 0.15), FINGER_OPEN, "up", dpsi)]
        rest, rr = _rest(hold_arm)
        hold += [_w(2.0, rest, FINGER_OPEN, "rest", rr), _w(3.0, rest, FINGER_OPEN, "wait", rr)]
        orest, orr = _rest(cut_arm)
        return {hold_arm: hold, cut_arm: [_w(sum(w.duration for w in hold), orest, FINGER_CLOSED, "stay", orr)]}
    g_cmd = g + np.array([HOLD_SAG[0], side * HOLD_SAG[1], HOLD_SAG[2]])
    hold += [
        _w(2.5, g_cmd, SQUEEZE, "turn it across the arms, over the dish", dpsi),
        _w(2.0, g_cmd, SQUEEZE, "lean the near end down", lean),
    ]
    t_ready = sum(w.duration for w in hold)

    # the cutting hand: pick the shears by the grips (blades away from the arms) and carry them blades-first
    sgrip = np.array([sx - grip_reach(HOLD_JAW), sy, sz + SH_TCP_UP])
    piv_x = cut[0] + (FLAT_W / 2 - TIP_SHORT) - BLADE  # shut, the tips stop 4 mm short of the flat end's far side
    if mode == "seal":
        cut = cut + np.array([0.0, side * 0.7071, -0.7071]) * (LOOSE + BAND / 2 - CUT_AT)
    tcp_cut = np.array([piv_x - grip_reach(SHUT), cut[1], cut[2] + SH_TCP_UP])
    tcp_cut = tcp_cut + np.array([SHEAR_SAG[0], -side * SHEAR_SAG[1], SHEAR_SAG[2]])
    # the last approach is level, at the cutting height, from where the open blade tips are 5 mm short of the flat end
    back = FLAT_W - TIP_SHORT - BLADE - grip_reach(SHUT) + 0.005 + grip_reach(HOLD_JAW) + BLADE * math.cos(HALF)
    stage = tcp_cut + np.array([-back, 0.0, 0.0])
    shear = [
        _w(2.0, sgrip + [0, 0, 0.08], SH_OPEN, "over the shears' grips", d0),
        _w(1.5, sgrip, SH_OPEN, "down round the grips", d0),
        _w(0.8, sgrip, SH_HOLD, "take hold", d0),
        _w(1.5, sgrip + [0, 0, 0.08], SH_HOLD, "lift the shears", d0),
    ]
    t_lifted = sum(w.duration for w in shear)
    if t_ready > t_lifted:  # the other arm turns and leans the portion first
        shear.append(_w(t_ready - t_lifted, sgrip + [0, 0, 0.08], SH_HOLD, "wait for the portion", d0))
    shear += [  # blade tips kept short of the film and the fish: back toward the arms first, then across
        _w(2.0, stage + [0, -side * 0.17, 0.04], SH_HOLD, "draw the shears back toward the arms", d0),
        _w(2.0, stage + [0, 0, 0.04], SH_HOLD, "across to in front of the hanging end", d0),
        _w(1.0, stage, SH_HOLD, "down to the cutting height, blades short of the film", d0),
    ]
    shear += [
        _w(2.0, tcp_cut, SH_HOLD, "slide the blades either side of the flat film", d0),
        _w(1.2, tcp_cut, SH_CLOSE, "close the shears", d0),
        _w(0.4, tcp_cut, SH_CLOSE, "hold them shut", d0),
    ]
    t_cut = sum(w.duration for w in shear)
    put_tcp = np.array([sx - grip_reach(HOLD_JAW), sy, BOARD_TOP + HANDLE_T / 2 + SH_TCP_UP + 0.004])
    shear += [
        _w(0.8, tcp_cut, SH_HOLD, "let the spring open them", d0),
        _w(1.5, tcp_cut + [-0.09, 0, 0.01], SH_HOLD, "draw them back", d0),
        _w(1.0, tcp_cut + [-0.10, -side * 0.04, 0.05], SH_HOLD, "up and back toward its own side", d0),
        _w(2.5, put_tcp + [0, 0, 0.06], SH_HOLD, "over the board", d0),
        _w(1.5, put_tcp, SH_HOLD, "down onto the board", d0),
        _w(0.6, put_tcp, SH_OPEN, "let go", d0),
        _w(1.0, put_tcp + [0, 0, 0.08], SH_OPEN, "up", d0),
    ]
    srest, srr = _rest(cut_arm)
    shear += [_w(2.0, srest, SH_OPEN, "back to rest", srr)]

    # the holding hand waits for the cut, for the fish to slide out and for the shears to be back on the board, then
    # takes the empty film to the bowl
    t_now = sum(w.duration for w in hold)
    t_clear = sum(w.duration for w in shear[:[w.label for w in shear].index("down onto the board") + 1])
    hold.append(_w(max(t_cut + 3.0, t_clear) - t_now, g_cmd, SQUEEZE,
                   "hold while the end is cut and the fish slides out", lean))
    drop = np.array([bx, by, TOP + BOWL_H + 0.045])
    hold += [
        _w(1.5, g_cmd, SQUEEZE, "level the empty film", dpsi),
        _w(2.5, drop + [0, 0, 0.02], SQUEEZE, "over the scrap bowl", d0),
        _w(1.0, drop, SQUEEZE, "down to the bowl", d0),
        _w(0.6, drop, FINGER_OPEN, "let go", d0),
        _w(1.0, drop + [0, 0, 0.03], FINGER_OPEN, "up", d0),
    ]
    hrest, hrr = _rest(hold_arm)
    hold += [_w(2.0, hrest, FINGER_OPEN, "back to rest", hrr)]
    end = max(sum(w.duration for w in hold), sum(w.duration for w in shear)) + 3.0
    for plan, rest, rr, f in ((hold, hrest, hrr, FINGER_OPEN), (shear, srest, srr, SH_OPEN)):
        plan.append(_w(end - sum(w.duration for w in plan), rest, f, "wait", rr))
    return {hold_arm: hold, cut_arm: shear}
