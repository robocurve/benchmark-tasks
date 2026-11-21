"""A kitchen counter. An 8 oz bar of orange cheddar lies bare on a white side plate; it has a blue-green and white
patch on its top face at one end. In the middle of the counter stands a beechwood wire cheese slicer: a board with a
slot across it and a hinged black steel arm, standing upright, that carries a cutting wire down into the slot. A
beech cheese board with a row of round crackers at one end lies on one side of the counter, and an open stainless
food-waste tub stands on the other. Which end of the bar has the patch, and which side the cheese board and the tub are on,
change between trials.
"""
import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402  (parts before lots: its reset hooks run before the lots move objects)
import lots  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s released-and-still hold is counted inside the whole-trial record
DEMO_TOUCHES = ("slicer", "side_plate")
EPS = 1e-9

# -- the bar of cheddar (m, kg) ---------------------------------------------------------------------------------------
# 120 x 56 x 31 mm, 226 g. It is built from 24 slabs 5 mm thick, joined face to face; a join lets go once the slicer's
# arm has swung the wire through the whole cross-section there, with the bar lying square to the wire. Slab 0 is at
# the mouldy end; a slab's own +x points away from that end.
N_SLAB, SLAB_T, BAR_W, BAR_H = 24, 0.005, 0.056, 0.031
BAR_L, BAR_MASS = N_SLAB * SLAB_T, 0.226
SLAB_GAP = 0.00002  # the colliders stop 0.01 mm short of each join so neighbours never start overlapping
# The mould patch: an irregular blue-green patch with a white fringe on the top face toward one corner of the mouldy
# end. It runs from the end edge to 27.5 mm along the bar, is up to 30 mm across, and comes 10 mm down the end face.
PATCH_FROM, PATCH_TO = 0.0, 0.0275
MARGIN = 0.0254  # 1 inch
LIMIT = PATCH_TO + MARGIN  # 52.9 mm from the mouldy end: a cut nearer than this is too close to the patch
FIRST_CLEAN = math.ceil(LIMIT / SLAB_T - 1e-9)  # slab 11: cheese from here on (55 mm) is clear of the patch
CHEDDAR, MOULD, FRINGE = (0.93, 0.60, 0.12), (0.10, 0.40, 0.38), (0.90, 0.93, 0.88)
SLAB_NAMES = tuple(f"cheddar_{i:02d}" for i in range(N_SLAB))
# slab: (from, to) along the bar from the mouldy end, then (low, high) across the bar; all in metres
PATCH_TOP = {0: (0.0000, 0.0050, -0.020, 0.002), 1: (0.0050, 0.0100, -0.022, 0.006), 2: (0.0100, 0.0150, -0.022, 0.008),
             3: (0.0150, 0.0200, -0.021, 0.007), 4: (0.0200, 0.0250, -0.018, 0.003), 5: (0.0250, 0.0275, -0.014, -0.002)}
PATCH_END = (-0.018, 0.000, 0.010)  # on the end face: (low, high) across the bar, and how far down from the top edge
PATCH_SPECKS = {1: (0.0060, 0.0100, 0.011, 0.015), 3: (0.0155, 0.0195, 0.012, 0.016)}  # two 4 mm specks beside it
FRINGE_W = 0.004


def _patch_boxes(i):
    """The parts of the patch on slab i: (size, centre, colour) boxes in the slab's frame (origin at its centre).
    The white fringe comes first, the blue-green middle lies on top of it."""
    out = []
    if i in PATCH_TOP:
        x0, x1, y0, y1 = PATCH_TOP[i]
        mid = (x0 + x1) / 2 - (i + 0.5) * SLAB_T
        out.append(((x1 - x0, y1 - y0, 0.0006), (mid, (y0 + y1) / 2, BAR_H / 2 + 0.0003), FRINGE))
        if y1 - y0 > 2 * FRINGE_W + 0.002 and i < 5:
            x1 = min(x1, PATCH_TO - FRINGE_W)
            mid = (x0 + x1) / 2 - (i + 0.5) * SLAB_T
            out.append(((x1 - x0, y1 - y0 - 2 * FRINGE_W, 0.0006), (mid, (y0 + y1) / 2, BAR_H / 2 + 0.0006), MOULD))
    if i in PATCH_SPECKS:
        x0, x1, y0, y1 = PATCH_SPECKS[i]
        out.append(((x1 - x0, y1 - y0, 0.0006), ((x0 + x1) / 2 - (i + 0.5) * SLAB_T, (y0 + y1) / 2, BAR_H / 2 + 0.0003), MOULD))
    if i == 0:
        y0, y1, down = PATCH_END
        out.append(((0.0006, y1 - y0, down), (-SLAB_T / 2 - 0.0003, (y0 + y1) / 2, BAR_H / 2 - down / 2), FRINGE))
        out.append(((0.0006, y1 - y0 - 2 * FRINGE_W, down - 0.003),
                    (-SLAB_T / 2 - 0.0006, (y0 + y1) / 2, BAR_H / 2 - (down - 0.003) / 2), MOULD))
    return out


PATCH = {i: _patch_boxes(i) for i in range(N_SLAB) if _patch_boxes(i)}  # slabs 0..5

# -- fixed things -------------------------------------------------------------------------------------------------
WOOD, STEEL, BLACK, WHITE, PAN, CRACKER = ((0.76, 0.60, 0.40), (0.86, 0.87, 0.88), (0.08, 0.08, 0.09),
                                           (0.95, 0.95, 0.93), (0.70, 0.72, 0.74), (0.90, 0.72, 0.38))
BOARD_WOOD = (0.62, 0.45, 0.28)  # the cheese board, oiled: darker than the slicer so the two read apart
WIRE_GREY = (0.33, 0.34, 0.36)  # the wire is drawn 2 mm thick and dark so the cameras can see it
PLATE_R, PLATE_H = 0.100, 0.015
PLATE_XY = (0.180, -0.260)
PLATE = parts.Part("side_plate", pos=(PLATE_XY[0], PLATE_XY[1], TOP), friction=(0.5, 0.4),
                   shapes=[parts.cylinder(PLATE_R, PLATE_H, (0, 0, PLATE_H / 2), WHITE)])

# The slicer: board 241 x 152 x 22 mm, its short end toward the robot, slot 65 mm in from that end. The arm turns
# about a line 8 mm inside the board's left long edge, at mid-thickness. Joint 0 is the arm standing straight up;
# the joint angle grows as the arm comes down to the right, and at 90 degrees the wire lies in the slot.
SL_L, SL_W, SL_H = 0.241, 0.152, 0.022
SL_POS = (0.2855, 0.0, TOP)
X_NEAR = SL_POS[0] - SL_L / 2  # 0.165
X_SLOT = X_NEAR + 0.065  # 0.230
PIVOT_Y, PIVOT_Z = SL_W / 2 - 0.008, SL_H / 2  # 0.068, 0.011 above the table
WIRE_L, LEG_R, ROD_R = 0.152, 0.165, 0.0025
LEG_X = -SL_L / 2 - 0.005  # the arm's first leg swings 5 mm outside the board's near end
SLOT_X = X_SLOT - SL_POS[0]
HANDLE = (0.020, 0.020, 0.040)  # along the board, across the swing, along the arm
ARM_MASS = 0.07
HINGE_FRICTION = 0.15  # N m: holds the arm wherever it is left
CUT_TORQUE = 0.50  # N m more while the wire is in uncut cheese (about 7 N at the middle of the bar)
ARM_DAMPING = 500.0  # N m s/rad, capped at the torques above: a friction hinge, not a spring
ARM = parts.Hinge(
    "arm", pivot=((LEG_X + SLOT_X) / 2, PIVOT_Y, PIVOT_Z), axis=(1.0, 0.0, 0.0), lower=-0.15, upper=math.pi / 2,
    mass=ARM_MASS, spring=0.0, damping=ARM_DAMPING, max_effort=HINGE_FRICTION,
    shapes=[
        parts.cylinder(ROD_R, LEG_R, (LEG_X, PIVOT_Y, PIVOT_Z + LEG_R / 2), BLACK, axis="z"),
        parts.cylinder(ROD_R, SLOT_X - LEG_X, ((LEG_X + SLOT_X) / 2, PIVOT_Y, PIVOT_Z + LEG_R), BLACK, axis="x"),
        parts.box(HANDLE, (SLOT_X, PIVOT_Y, PIVOT_Z + LEG_R + 0.003), BLACK),
        parts.cylinder(0.001, WIRE_L, (SLOT_X, PIVOT_Y, PIVOT_Z + WIRE_L / 2), WIRE_GREY, axis="z", solid=False),
    ])
SLICER = parts.Part(
    "slicer", pos=SL_POS, friction=(0.5, 0.4), joints=[ARM],
    shapes=[parts.box((SL_L, SL_W, SL_H), (0, 0, SL_H / 2), WOOD),
            parts.box((0.003, SL_W, 0.0004), (SLOT_X, 0, SL_H + 0.0002), (0.25, 0.18, 0.10), solid=False)])
WIRE_A = np.array([SLOT_X, PIVOT_Y, PIVOT_Z]) - ARM.origin  # the wire's fixed end, in the arm link's frame

# -- loose things the lots move --------------------------------------------------------------------------------
BOARD_X, BOARD_Y, BOARD_H = 0.150, 0.300, 0.016
BOARD = parts.Part(
    "cheese_board", pos=(0.370, -0.270, TOP + 0.0005), mass=0.38, friction=(0.6, 0.5),
    shapes=[parts.box((BOARD_X, BOARD_Y, BOARD_H), (0, 0, BOARD_H / 2), BOARD_WOOD)]
    + [parts.cylinder(0.0235, 0.004, (x, 0.125, BOARD_H + 0.002), CRACKER) for x in (-0.050, 0.0, 0.050)])
TUB_X, TUB_Y, TUB_H, TUB_WALL = 0.162, 0.176, 0.065, 0.003
TUB = parts.Part(
    "waste_tub", pos=(0.280, 0.260, TOP + 0.0005), mass=0.25, friction=(0.5, 0.4),
    shapes=[parts.box((TUB_X, TUB_Y, TUB_WALL), (0, 0, TUB_WALL / 2), PAN),
            parts.box((TUB_WALL, TUB_Y, TUB_H), (TUB_X / 2 - TUB_WALL / 2, 0, TUB_H / 2), PAN),
            parts.box((TUB_WALL, TUB_Y, TUB_H), (-TUB_X / 2 + TUB_WALL / 2, 0, TUB_H / 2), PAN),
            parts.box((TUB_X - 2 * TUB_WALL, TUB_WALL, TUB_H), (0, TUB_Y / 2 - TUB_WALL / 2, TUB_H / 2), PAN),
            parts.box((TUB_X - 2 * TUB_WALL, TUB_WALL, TUB_H), (0, -TUB_Y / 2 + TUB_WALL / 2, TUB_H / 2), PAN)])

BAR_Z = TOP + PLATE_H + BAR_H / 2 + 0.0005
BAR_X0 = PLATE_XY[0] - BAR_L / 2  # the bar lies along x on the plate, from 0.120 to 0.240


def _slab(i):
    shapes = [parts.box((SLAB_T - SLAB_GAP, BAR_W, BAR_H), (0, 0, 0), CHEDDAR)]
    shapes += [parts.box(size, at, colour, solid=False) for size, at, colour in PATCH.get(i, [])]
    return parts.Part(SLAB_NAMES[i], pos=(BAR_X0 + (i + 0.5) * SLAB_T, PLATE_XY[1], BAR_Z), mass=BAR_MASS / N_SLAB,
                      friction=(0.6, 0.5), shapes=shapes)


SLABS = [_slab(i) for i in range(N_SLAB)]
OBJECTS = [PLATE, SLICER, BOARD, TUB, *SLABS]

# Lot A (the screenshot): the patch is at the bar's near end, the cheese board is on the right, the tub on the left.
# Lot B: the bar is turned end for end (patch at the far end) and the cheese board and the tub change places.
_A = {o.name: tuple(o.pos) for o in (BOARD, TUB, *SLABS)}
_B = {SLAB_NAMES[i]: (BAR_X0 + BAR_L - (i + 0.5) * SLAB_T, PLATE_XY[1], BAR_Z) for i in range(N_SLAB)}
_B.update({BOARD.name: tuple(TUB.pos), TUB.name: tuple(BOARD.pos)})
LOTS = lots.declare(OBJECTS, lots.Lot("A", place=_A),
                    lots.Lot("B", place=_B, yaw={n: math.pi for n in SLAB_NAMES}))


# -- geometry helpers ---------------------------------------------------------------------------------------------
def rot(q):
    return parts._quat_to_matrix(np.asarray(q, float))


def pieces_of(cuts):
    """The bar's pieces as lists of slab numbers, given the joins that have let go (join k is between k-1 and k)."""
    edges = [0, *sorted(cuts), N_SLAB]
    return [list(range(a, b)) for a, b in zip(edges, edges[1:])]


def slab_frames(objects):
    return ([np.asarray(objects[n].pos, float) for n in SLAB_NAMES], [rot(objects[n].quat_xyzw) for n in SLAB_NAMES])


_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * \
    np.array([SLAB_T, BAR_W, BAR_H]) / 2


def slab_points(p, r):
    """A slab's eight corners and its centre (world)."""
    return np.vstack([p + _CORNERS @ r.T, p])


def patch_boxes(P, R):
    """The mould patch as (centre, rotation, size) boxes in the world."""
    return [(P[i] + R[i] @ np.asarray(at), R[i], size) for i, boxes in PATCH.items() for size, at, _ in boxes]


def patch_points(P, R):
    """Sample points through the mould patch (world), at most 1 mm apart."""
    out = []
    for i, boxes in PATCH.items():
        for size, at, _ in boxes:
            axes = [np.linspace(-v / 2, v / 2, int(math.ceil(v / 0.001)) + 1) for v in size]
            g = np.array([[x, y, z] for x in axes[0] for y in axes[1] for z in axes[2]]) + np.asarray(at)
            out.append(P[i] + g @ R[i].T)
    return np.vstack(out)


def dist_to_box(pts, centre, r, size):
    """Distance of each point to a box (0 inside)."""
    return np.linalg.norm(np.maximum(np.abs((pts - centre) @ r) - np.asarray(size) / 2, 0.0), axis=1)


def wire_ends(slicer):
    """The cutting wire's two ends (world): its fixed end on the hinge line at the slot, and its end at the handle."""
    pos, quat = slicer.links["arm"]
    r = rot(quat)
    a = np.asarray(pos, float) + r @ WIRE_A
    return a, a + r @ np.array([0.0, 0.0, WIRE_L])


# Each join's cross-section is sampled on a grid; the join lets go when the wire has passed every grid point.
GRID_W = np.linspace(-BAR_W / 2 + 0.001, BAR_W / 2 - 0.001, 9)
GRID_H = np.linspace(-BAR_H / 2 + 0.001, BAR_H / 2 - 0.001, 5)
GRID = np.array([[w, h] for w in GRID_W for h in GRID_H])


SQUARE_DEG = 10.0  # a cut counts only with the bar within 10 degrees of square to the plane the wire swings in
ARM_MOVING = 1e-4  # rad per control step: the wire cuts only while the slicer's arm is swinging down
ARM_BACK = 0.2  # rad (11 degrees): an arm raised this far above its lowest point starts a fresh cut


def grid_sides(c, r, a, b):
    """For each grid point of a cross-section (piece frame: origin c, axes r): which side of the wire it is on, seen
    along the bar, and whether the wire reaches across it."""
    la, lb = (a - c) @ r, (b - c) @ r
    d2 = (lb - la)[1:]
    rel = GRID - la[1:]
    u = rel @ d2 / max(float(d2 @ d2), 1e-12)
    return d2[0] * rel[:, 1] - d2[1] * rel[:, 0], (u >= 0.0) & (u <= 1.0)


def wire_in_mould_end(P, R, a, b):
    """Whether the wire is inside the bar (within TOUCH) anywhere from its mouldy end to the patch's far edge, read
    in the frame of the bar's first slab (the patch's slabs move as one: a cut among them is already a zero)."""
    r = R[0]
    c = P[0] - r @ np.array([SLAB_T / 2, 0.0, 0.0])
    la, lb = (a - c) @ r, (b - c) @ r
    line = la[None, :] + np.linspace(0.0, 1.0, int(WIRE_L / 0.001) + 1)[:, None] * (lb - la)[None, :]
    return bool(np.any((np.abs(line[:, 1]) <= BAR_W / 2 + TOUCH) & (np.abs(line[:, 2]) <= BAR_H / 2 + TOUCH)
                       & (line[:, 0] >= -TOUCH) & (line[:, 0] <= PATCH_TO + TOUCH)))


def wire_on_piece(P, R, piece, a, b, n):
    """Where the wire (ends a, b; it swings in the plane through a with unit normal n) meets one piece. None if the
    wire does not cross the piece's centre plane within its length. Otherwise a dict: ``join`` the join the cut
    belongs to (None near the piece's ends), ``lo``/``hi`` the nearest and farthest the wire's plane comes to the
    mouldy end across the bar's width (m), ``inside`` whether the wire is inside the cheese now, ``skew`` the
    bar's angle off square to the wire's plane (degrees), and the piece's frame."""
    first = piece[0]
    r = R[first]
    c = P[first] - r @ np.array([SLAB_T / 2, 0.0, 0.0])  # centre of the piece's face toward the mouldy end
    la, lb = (a - c) @ r, (b - c) @ r  # (along the bar, across, up) in the piece's frame
    if la[1] * lb[1] > 0:
        return None
    s0 = la[1] / (la[1] - lb[1]) if abs(la[1] - lb[1]) > 1e-12 else 0.0
    along = float(la[0] + s0 * (lb[0] - la[0]))
    if along < 0.0 or along > len(piece) * SLAB_T:
        return None
    ts = np.linspace(0.0, 1.0, 61)
    line = la[None, :] + ts[:, None] * (lb - la)[None, :]
    inside = bool(np.any((np.abs(line[:, 1]) <= BAR_W / 2) & (np.abs(line[:, 2]) <= BAR_H / 2)
                         & (line[:, 0] >= 0.0) & (line[:, 0] <= len(piece) * SLAB_T)))
    square = float(r[:, 0] @ n)
    skew = math.degrees(math.acos(min(1.0, abs(square))))
    if abs(square) > 0.1:  # where the wire's plane crosses the bar's two long edges
        edges = [float((a - (c + r[:, 1] * w)) @ n) / square for w in (-BAR_W / 2, BAR_W / 2)]
    else:
        edges = [along, along]
    lo, hi = first * SLAB_T + min(*edges, along), first * SLAB_T + max(*edges, along)
    j = int(math.floor(lo / SLAB_T + 0.5))  # the join nearest the wire's closest approach to the mouldy end
    if j >= FIRST_CLEAN and lo < LIMIT - 1e-9:
        j = FIRST_CLEAN - 1  # a wire less than 25.4 mm from the patch never counts as a clear cut
    join = j if first < j <= piece[-1] else None
    return dict(join=join, lo=lo, hi=hi, inside=inside, skew=skew, c=c, r=r)


# -- where things are -------------------------------------------------------------------------------------------------
def _local(obj, pts):
    return (np.atleast_2d(pts) - np.asarray(obj.pos, float)) @ rot(obj.quat_xyzw)


def over_board(board, p, r):
    """Any corner or the centre of a slab is above the cheese board's outline (at any height above its underside)."""
    loc = _local(board, slab_points(p, r))
    return bool(np.any((np.abs(loc[:, 0]) <= BOARD_X / 2 + EPS) & (np.abs(loc[:, 1]) <= BOARD_Y / 2 + EPS)
                       & (loc[:, 2] >= 0.0)))


def piece_on_board(board, P, R, piece):
    """Every slab's centre is inside the board's outline and the piece's lowest point is on the board's top (3 mm)."""
    cen = _local(board, np.array([P[i] for i in piece]))
    if not np.all((np.abs(cen[:, 0]) <= BOARD_X / 2 + EPS) & (np.abs(cen[:, 1]) <= BOARD_Y / 2 + EPS)):
        return False
    low = min(float(_local(board, slab_points(P[i], R[i]))[:, 2].min()) for i in piece)
    return -0.001 - EPS <= low - BOARD_H <= 0.003 + EPS


def piece_in_tub(tub, P, R, piece):
    """Every slab's centre is inside the tub's inner outline, below its rim and above its floor."""
    cen = _local(tub, np.array([P[i] for i in piece]))
    return bool(np.all((np.abs(cen[:, 0]) <= TUB_X / 2 - TUB_WALL + EPS) & (np.abs(cen[:, 1]) <= TUB_Y / 2 - TUB_WALL + EPS)
                       & (cen[:, 2] <= TUB_H + EPS) & (cen[:, 2] >= 0.0)))


def end_level(objects, cuts):
    """The level the scene shows as it stands (hands, stillness and the whole-trial rules are checked elsewhere)."""
    P, R = slab_frames(objects)
    board, tub = objects[BOARD.name], objects[TUB.name]
    pieces = pieces_of(cuts)
    served = [p for p in pieces if piece_on_board(board, P, R, p)]
    binned = [p for p in pieces if piece_in_tub(tub, P, R, p)]
    clean_exists = any(p[0] >= FIRST_CLEAN for p in pieces)
    if served:
        if any(p[0] < FIRST_CLEAN for p in served):
            return 0.0
        return 1.0 if all(p in served or p in binned for p in pieces) else 0.75
    if len(binned) == len(pieces):
        return 0.25
    if clean_exists and all(p in binned for p in pieces if p[0] < FIRST_CLEAN):
        return 0.5
    return 0.25 if clean_exists else 0.0


# -- whole-trial record -----------------------------------------------------------------------------------------------
TOUCH = 0.0015  # a gripper or the wire "touches" within 1.5 mm
CLEAR = 0.005  # released: no gripper within 5 mm of any cheese, the cheese board or the tub
HOLD_S = 1.0
LEVELS = (0.25, 0.5, 0.75, 1.0)
ARMS = ("left", "right")
_rec: dict = {}


def _fresh():
    _rec.clear()
    _rec.update(armed=False, pending=False, invalid=False, known=False, t=None, q=None, cuts=set(), prev={},
                progress={}, in_cheese=False, zero=None, dirty={a: False for a in ARMS},
                soiled=np.zeros(N_SLAB, bool), held={lv: 0.0 for lv in LEVELS}, since={lv: None for lv in LEVELS}, level=0.0, clear=False, still=False,
                cert=None, lot=None)


_fresh()


def _arm_record(objects):
    """Start the record for a trial: the bar must be whole and straight, the wire clear of it."""
    _fresh()
    P, R = slab_frames(objects)
    straight = all(np.linalg.norm(P[i] - (P[0] + R[0] @ np.array([i * SLAB_T, 0.0, 0.0]))) <= 0.001 for i in range(N_SLAB))
    _rec.update(armed=True, invalid=not straight)


def stamp(objects):
    out = []
    for n in (*SLAB_NAMES, BOARD.name, TUB.name):
        s = objects[n]
        out += [*np.round(np.r_[s.pos, s.quat_xyzw], 7)]
    return (*out, round(float(objects[SLICER.name].joints["arm"]), 7))


def _zero(why):
    if _rec["zero"] is None:
        _rec["zero"] = why


def _update(objects, clouds, t):
    """Advance the record by one control step. ``clouds``: {arm: points on that gripper's surface} or None if the
    grippers' state is unknown. Returns the joins that let go at this step."""
    newly = []
    if not _rec["armed"]:
        return newly
    dt = 0.0 if _rec["t"] is None else float(t) - _rec["t"]
    if _rec["t"] is not None and not 0 < dt <= 0.100001:
        _rec["invalid"] = True
    _rec["t"] = float(t)
    P, R = slab_frames(objects)
    board, tub, slicer = objects[BOARD.name], objects[TUB.name], objects[SLICER.name]
    a, b = wire_ends(slicer)
    mould = patch_points(P, R)

    # the wire against each piece: resistance, cutting progress, and the wire on the patch
    q = float(slicer.joints["arm"])
    # Only new downward travel cuts: a step counts when the arm goes lower than it has been since it was last raised,
    # so an arm jiggled up and down in place cuts nothing. Raising it 11 degrees starts a fresh cut.
    if _rec["q"] is None or q < _rec["q"] - ARM_BACK:
        _rec["q"], swinging = q, False
        _rec["progress"].clear()
    else:
        swinging = q - _rec["q"] >= ARM_MOVING
        _rec["q"] = max(_rec["q"], q)
    hinge = rot(slicer.links["arm"][1])[:, 0]  # the wire swings in the plane square to the hinge line
    in_cheese, seen = False, {}
    if wire_in_mould_end(P, R, a, b):
        _zero("the wire went into the bar at the mould patch")
    for piece in pieces_of(_rec["cuts"]):
        hit = wire_on_piece(P, R, piece, a, b, hinge)
        if hit is None:
            continue
        if hit["inside"]:
            in_cheese = True
            if hit["lo"] <= PATCH_TO + TOUCH and hit["hi"] >= PATCH_FROM - TOUCH:
                _zero("the wire went into the bar at the mould patch")
        join = hit["join"]
        if join is None:
            continue
        seen[join] = (*grid_sides(hit["c"], hit["r"], a, b), hit["c"], hit["r"])
        before = _rec["prev"].get(join)
        if before is not None and swinging and hit["skew"] <= SQUARE_DEG + 1e-9:
            # Only the wire's own downward swing counts: the bar is taken where it was a step ago, so cheese pushed
            # through a standing wire, moved while the arm swings, or lifted through by the arm, is not cut by that.
            sides, span = grid_sides(before[2], before[3], a, b)
            crossed = (before[0] * sides < 0) & (before[1] | span)
            if crossed.any():
                done = _rec["progress"].setdefault(join, np.zeros(len(GRID), bool))
                done |= crossed
                if done.all():
                    newly.append(join)
    _rec["prev"] = seen
    for join in newly:
        _rec["cuts"].add(join)
        _rec["progress"].pop(join, None)
    _rec["in_cheese"] = in_cheese
    wire = a + np.linspace(0.0, 1.0, int(WIRE_L / 0.001) + 1)[:, None] * (b - a)
    spots = patch_boxes(P, R)
    if min(float(dist_to_box(wire, c, r, size).min()) for c, r, size in spots) <= TOUCH + EPS:
        _zero("the wire touched the mould patch")
    sl_r = rot(slicer.quat_xyzw)
    sl_c = np.asarray(slicer.pos, float) + sl_r @ np.array([0.0, 0.0, SL_H / 2])
    if float(dist_to_box(mould, sl_c, sl_r, (SL_L, SL_W, SL_H)).min()) <= TOUCH + EPS:
        _zero("the mould patch touched the slicer board")

    # where the cheese is
    pieces = pieces_of(_rec["cuts"])
    for i in range(N_SLAB):
        z = P[i][2]
        if z < TOP - 0.03 or abs(P[i][0] - 0.35) > 0.42 or abs(P[i][1]) > 0.62:
            _zero("cheese left the table")
        if (i < FIRST_CLEAN or _rec["soiled"][i]) and over_board(board, P[i], R[i]):
            _zero("cheese from the mouldy end was on or over the cheese board" if i < FIRST_CLEAN
                  else "a gripper that had touched the mouldy offcut touched cheese that went on the cheese board")

    # the grippers
    clear = False
    if clouds is None or any(k not in clouds for k in ARMS):
        _rec["known"] = False
        _rec["invalid"] = True
    else:
        _rec["known"] = True
        clear = True
        cut_made = len(pieces) > 1
        mouldy = {i for p in pieces if p[0] < FIRST_CLEAN for i in p}
        centre = np.mean(P, axis=0)
        for arm in ARMS:
            pts = np.asarray(clouds[arm], float)
            if pts.ndim != 2 or pts.shape[1] != 3 or not len(pts) or not np.isfinite(pts).all():
                _rec["known"], _rec["invalid"], clear = False, True, False
                continue
            for obj, spec in ((board, BOARD), (tub, TUB)):
                for _, c, r, size in spec.world_boxes({"base": (np.asarray(obj.pos, float), rot(obj.quat_xyzw))}):
                    if float(dist_to_box(pts, c, r, size).min()) <= CLEAR + EPS:
                        clear = False
            near = pts[np.linalg.norm(pts - centre, axis=1) < 0.60]
            if not len(near):
                continue
            if min(float(dist_to_box(near, c, r, size).min()) for c, r, size in spots) <= TOUCH + EPS:
                _zero("a gripper touched the mould patch")
            touched = []
            for i in range(N_SLAB):
                d = float(dist_to_box(near, P[i], R[i], (SLAB_T, BAR_W, BAR_H)).min())
                if d <= CLEAR + EPS:
                    clear = False
                if d <= TOUCH + EPS:
                    touched.append(i)
            if cut_made and any(i in mouldy for i in touched):
                _rec["dirty"][arm] = True
            if _rec["dirty"][arm]:
                for i in touched:
                    if i >= FIRST_CLEAN:  # any cheese that could count as clean, on whichever piece it is now
                        _rec["soiled"][i] = True
    still = all(objects[n].resting(0.005, 0.2) for n in (*SLAB_NAMES, BOARD.name, TUB.name))
    level = end_level(objects, _rec["cuts"]) if (clear and still and _rec["zero"] is None) else 0.0
    for lv in LEVELS:  # time since the scene was first seen at this level or better, without a break
        if level >= lv - EPS:
            if _rec["since"][lv] is None:
                _rec["since"][lv] = float(t)
            _rec["held"][lv] = float(t) - _rec["since"][lv]
        else:
            _rec["since"][lv], _rec["held"][lv] = None, 0.0
    _rec.update(level=level, clear=clear, still=still, cert=stamp(objects))
    return newly


def score(objects):
    try:
        if (not _rec["armed"] or _rec["invalid"] or not _rec["known"] or _rec["zero"] is not None
                or _rec["cert"] != stamp(objects)):
            return 0.0
        now = end_level(objects, _rec["cuts"])
        best = 0.0
        for lv in LEVELS:
            if now >= lv - EPS and _rec["held"][lv] >= HOLD_S - 1e-9:
                best = lv
        return best
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


# -- simulator side: the joins between slabs, the hinge's resistance, the grippers' surfaces ----------------------
_JOINS: dict = {}  # ("c", k): neighbour join between slabs k-1 and k; ("a", first, i): slab i to its piece's first slab
_ON: dict = {}  # which joins are switched on
_SIM = {"gen": 0, "effort": None, "broken": None}
_MESH: dict = {}


def _stage():
    from isaaclab.sim.utils import get_current_stage
    return get_current_stage()


def _body_path(base, name):
    from pxr import Usd, UsdPhysics
    env = base.scene.env_prim_paths[0]
    stage = _stage()
    path = f"{env}/{name}"
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return path
    for prim in Usd.PrimRange(stage.GetPrimAtPath(env)):
        if prim.GetName() == name and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(prim.GetPath())
    raise ValueError(f"no rigid body named {name} under {env}")


def _join(base, key, j, i):
    """A rigid join holding slab i where it belongs relative to slab j (i > j)."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    stage = _stage()
    env = base.scene.env_prim_paths[0]
    UsdGeom.Scope.Define(stage, f"{env}/cheddar_joins")
    _SIM["gen"] += 1
    path = f"{env}/cheddar_joins/j{_SIM['gen']}_{j}_{i}"
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(_body_path(base, SLAB_NAMES[j]))])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(_body_path(base, SLAB_NAMES[i]))])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float((i - j) * SLAB_T), 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateJointEnabledAttr().Set(True)  # last, after its frames
    _JOINS[key] = path


def _sync_joins(sim, cuts):
    """Make the joins match the bar's pieces: within a piece every slab is joined to its neighbour and to the piece's
    first slab; nothing joins two pieces. A join that is no longer wanted is switched off, never deleted."""
    from pxr import UsdPhysics
    base = sim.env.unwrapped
    want = {}
    for piece in pieces_of(cuts):
        for i in piece[1:]:
            want[("c", i)] = (i - 1, i)
        for i in piece[2:]:
            want[("a", piece[0], i)] = (piece[0], i)
    stage = _stage()
    for key, path in _JOINS.items():
        on = key in want
        if _ON.get(key) != on:
            joint = UsdPhysics.FixedJoint.Get(stage, path)
            if not joint:
                raise ValueError(f"join {key} is missing")
            joint.GetJointEnabledAttr().Set(on)
            _ON[key] = on
    for key, (j, i) in want.items():
        if key not in _JOINS:
            _join(base, key, j, i)
            _ON[key] = True


def _set_arm_effort(sim, value):
    if _SIM["effort"] == value:
        return
    asset = sim.env.unwrapped.scene[SLICER.name]
    ids = asset.find_joints(["arm"], preserve_order=True)[0]
    asset.write_joint_effort_limit_to_sim_index(limits=float(value), joint_ids=ids)
    _SIM["effort"] = value


def _hand_mesh():
    """Surface points of each hand link, its hull faces sampled every 3 mm (cached once)."""
    if _MESH:
        return _MESH
    from yam_arena import analysis
    for name in ("gripper", "tip_left", "tip_right"):
        pts = []
        for h in analysis.link_hulls().get(name, []):
            pts.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    raise ValueError("unknown hull face")
                c = face.mean(0)
                u = face[0] - c
                u /= np.linalg.norm(u)
                v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for p, q in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(max(np.linalg.norm(p - c), np.linalg.norm(q - c), np.linalg.norm(p - q)) / 0.003))
                    for i in range(n + 1):
                        for k in range(n - i + 1):
                            pts.append(c + (p - c) * (i / n) + (q - c) * (k / n))
        if not pts:
            raise ValueError("missing hand geometry")
        pts = np.asarray(pts)
        _, keep = np.unique(np.floor(pts / 0.002).astype(int), axis=0, return_index=True)
        _MESH[name] = pts[keep]
    return _MESH


def _hand_clouds(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    clouds = {}
    for arm in ARMS:
        asset = base.scene[f"{arm}_robot"]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        if poses.ndim != 2 or poses.shape[1] != 7 or not np.isfinite(poses).all():
            raise ValueError("unknown robot pose")
        chunks = []
        for name, p in _hand_mesh().items():
            pose = poses[asset.body_names.index(name)]
            chunks.append(p @ rot(pose[3:]).T + pose[:3] - origin)
        clouds[arm] = np.vstack(chunks)
    return clouds


def _broken(why, e):
    """Something in the simulator could not be set up or read: the trial gets no credit, and says why."""
    if _SIM["broken"] is None:
        _SIM["broken"] = why
    _rec["invalid"] = True
    print(f"[cheddar] {why}: {e!r}", file=sys.stderr, flush=True)


@parts.on_reset
def _on_reset(sim):
    _fresh()
    _SIM["broken"] = None
    try:
        _sync_joins(sim, set())
    except Exception as e:  # noqa: BLE001  (no joins, no credit)
        _broken("could not join the bar", e)
    try:
        _SIM["effort"] = None
        _set_arm_effort(sim, HINGE_FRICTION)
    except Exception as e:  # noqa: BLE001
        _broken("could not set the hinge's resistance", e)
    _rec["pending"] = True


@parts.on_step
def _on_step(sim):
    states = sim.states()
    if _rec["pending"]:
        _arm_record(states)
        if _SIM["broken"] is not None:
            _rec["invalid"] = True
    try:
        clouds = _hand_clouds(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        clouds = None
    try:
        parted = _update(states, clouds, sim.t)
    except Exception as e:  # noqa: BLE001  (anything unknown: no credit)
        _broken("scoring stopped", e)
        return
    if parted:
        try:
            _sync_joins(sim, _rec["cuts"])
        except Exception as e:  # noqa: BLE001
            _broken("could not part the bar", e)
        for k in parted:
            print(f"[cheddar] t={sim.t:.2f}s the bar parted {k * SLAB_T * 1000:.0f} mm from its mouldy end "
                  f"(arm at {math.degrees(states[SLICER.name].joints['arm']):.1f} deg)", file=sys.stderr, flush=True)
    try:
        _set_arm_effort(sim, HINGE_FRICTION + (CUT_TORQUE if _rec["in_cheese"] else 0.0))
    except Exception as e:  # noqa: BLE001
        _broken("could not set the hinge's resistance", e)
    if os.environ.get("TRACE_OUT"):
        P, R = slab_frames(states)
        sag = max(float(np.linalg.norm(P[i] - (P[p[0]] + R[p[0]] @ np.array([(i - p[0]) * SLAB_T, 0, 0]))))
                  for p in pieces_of(_rec["cuts"]) for i in p)
        tcp = {}
        try:
            base = sim.env.unwrapped
            for arm in ARMS:
                asset = base.scene[f"{arm}_robot"]
                poses = parts._np(asset.data.body_link_pose_w)[0]
                tcp[arm] = np.round(poses[asset.body_names.index("gripper")][:3] - parts._np(base.scene.env_origins[0]), 4).tolist()
        except Exception:  # noqa: BLE001
            pass
        row = dict(t=round(float(sim.t), 3), lot=lots.current().name, arm_deg=round(math.degrees(states[SLICER.name].joints["arm"]), 2),
                   in_cheese=_rec["in_cheese"], cuts=sorted(_rec["cuts"]),
                   progress={int(k): int(v.sum()) for k, v in _rec["progress"].items()}, zero=_rec["zero"],
                   invalid=_rec["invalid"], broken=_SIM["broken"], clear=_rec["clear"], still=_rec["still"], level=_rec["level"],
                   held={str(k): round(v, 2) for k, v in _rec["held"].items()}, dirty=dict(_rec["dirty"]),
                   sag_mm=round(1000 * sag, 2), s0=np.round(P[0], 4).tolist(), s23=np.round(P[23], 4).tolist(),
                   s0_up=round(float(R[0][2, 2]), 3), board=np.round(states[BOARD.name].pos, 4).tolist(),
                   tub=np.round(states[TUB.name].pos, 4).tolist(), gripper=tcp, score=score(states))
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("cheddar.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# -- scripted demo: open loop from the published layout of the lot -------------------------------------------------
OPEN_BAR, GRIP_BAR = -0.040, -0.0225  # finger targets: 12 mm clear of the bar's sides, 5.5 mm past them
OPEN_ROD, GRIP_ROD = -0.012, 0.0
END_GRIP = 0.012  # the bar is carried by its far end, pads centred 12 mm from the end face
# Where the bar is laid: the distance from its near end to the wire's plane. The cut has to be 52.5 mm or more from
# the mouldy end, and the near piece has to fit on the 65 mm of board in front of the slot.
CUT_AT = {"patch near": 0.0575, "patch far": 0.050}
TCP_UP = 0.016  # grip point above the underside of the cheese
ROD_UP = 0.006  # grip point above the arm's top bar
SWING_TO = math.radians(96)  # commanded past flat: the arm gives while the wire is in the cheese
R_HOME, L_HOME = (0.20, -0.42, 0.18), (0.20, 0.42, 0.18)  # where the arms end: out of the top camera's view of both boards


def _naive(lot, W):
    """Control for the README, never used for scoring: the literal errand, the whole bar carried to the cheese
    board. Where the board is out of the right arm's reach the right arm sets the bar down on the far part of the
    slicer's board and the left arm carries it on."""
    xs = [lot.pos(n)[0] for n in SLAB_NAMES]
    x, y = (max(xs) + min(xs)) / 2, lot.pos(SLAB_NAMES[0])[1]
    bx, by, _ = lot.pos(BOARD.name)
    z = PLATE_H + TCP_UP
    down = BOARD_H + TCP_UP + 0.003
    right = [W(2.5, x, y, 0.12, OPEN_BAR, "over the bar"), W(2.0, x, y, z, OPEN_BAR, "down around it"),
             W(1.0, x, y, z, GRIP_BAR, "grip"), W(1.5, x, y, 0.12, GRIP_BAR, "lift the bar")]
    if by < 0:
        right += [W(3.0, bx, by, 0.12, GRIP_BAR, "over the cheese board"), W(1.5, bx, by, down, GRIP_BAR, "lower it"),
                  W(1.0, bx, by, down, OPEN_BAR, "let go"), W(1.5, bx, by, 0.16, OPEN_BAR, "lift away"),
                  W(2.0, *R_HOME, OPEN_BAR, "clear"), W(3.0, *R_HOME, OPEN_BAR, "wait")]
        return {"right": right, "left": [W(sum(w.duration for w in right), 0.20, 0.30, 0.20, OPEN_BAR, "wait")]}
    sx, sz = 0.335, SL_H + TCP_UP
    right += [W(3.0, sx, 0.0, 0.12, GRIP_BAR, "over the slicer's board"), W(1.5, sx, 0.0, sz + 0.002, GRIP_BAR, "set it down"),
              W(1.0, sx, 0.0, sz + 0.002, OPEN_BAR, "let go"), W(1.0, sx, 0.0, 0.13, OPEN_BAR, "lift away"),
              W(2.0, *R_HOME, OPEN_BAR, "clear")]
    t = sum(w.duration for w in right)
    left = [W(t - 3.0, 0.20, 0.30, 0.20, OPEN_BAR, "wait"), W(2.0, sx, 0.20, 0.15, OPEN_BAR, "come round"),
            W(2.0, sx, 0.0, 0.12, OPEN_BAR, "over the bar"), W(1.5, sx, 0.0, sz, OPEN_BAR, "down around it"),
            W(1.0, sx, 0.0, sz, GRIP_BAR, "grip"), W(1.5, sx, 0.0, 0.13, GRIP_BAR, "lift the bar"),
            W(1.5, sx, 0.15, 0.13, GRIP_BAR, "carry it"), W(2.0, bx, by, 0.13, GRIP_BAR, "over the cheese board"),
            W(1.5, bx, by, down, GRIP_BAR, "lower it"), W(1.0, bx, by, down, OPEN_BAR, "let go"),
            W(1.5, bx, by, 0.16, OPEN_BAR, "lift away"), W(2.0, *L_HOME, OPEN_BAR, "clear"), W(3.0, *L_HOME, OPEN_BAR, "wait")]
    right.append(W(sum(w.duration for w in left) - t, *R_HOME, OPEN_BAR, "wait"))
    return {"right": right, "left": left}


def demo():
    lot = lots.current()
    xs = [lot.pos(n)[0] for n in SLAB_NAMES]
    bar_y = lot.pos(SLAB_NAMES[0])[1]
    far_end = max(xs) + SLAB_T / 2
    bar_mid = (max(xs) + min(xs)) / 2
    patch_x = float(np.mean([lot.pos(SLAB_NAMES[i])[0] for i in PATCH]))  # where the blue-green patch is seen
    far_piece_clean = patch_x < bar_mid  # the patch is on the near half: the far half is the one to serve
    board, tub = lot.pos(BOARD.name), lot.pos(TUB.name)
    # The right arm lays the bar across the slot and later takes the half it laid on the far side of the wire; the
    # left arm swings the slicer's arm down and then takes the near half. The half without the patch goes on the
    # cheese board, the other in the tub.
    right_goal, left_goal = (board, tub) if far_piece_clean else (tub, board)
    if right_goal[1] > 0 or left_goal[1] < 0:
        raise ValueError("this demo covers layouts where the piece to serve and the cheese board are on the same side")

    def W(t, x, y, z, f, label):
        return Waypoint(float(t), (float(x), float(y), float(TOP + z)), float(f), label)

    if os.environ.get("CHEDDAR_DEMO") == "naive":
        return _naive(lot, W)

    def put(goal, is_board):
        """Waypoints that lower a held half onto the cheese board, or into the tub, and let go."""
        if is_board:
            x, y, z = goal[0], goal[1], BOARD_H + TCP_UP + 0.003
        else:
            x, y, z = goal[0], goal[1], TUB_WALL + TCP_UP + 0.020
        return [W(2.5, x, y, 0.13, GRIP_BAR, "carry it over"), W(1.5, x, y, z, GRIP_BAR, "lower it"),
                W(1.0, x, y, z, OPEN_BAR, "let go"), W(1.5, x, y, 0.16, OPEN_BAR, "lift away")]

    cut_at = CUT_AT["patch near" if far_piece_clean else "patch far"]
    gx = far_end - END_GRIP
    px = X_SLOT + (BAR_L - cut_at) - END_GRIP
    rod_x = X_NEAR + 0.027
    rod_z = PIVOT_Z + LEG_R + ROD_UP
    r_park, l_park, l_wait = (0.24, -0.38, 0.16), (0.20, 0.30, 0.20), (0.20, 0.28, 0.16)
    # right arm: carry the bar by its far end and lay it across the slot
    right = [
        W(2.5, gx, bar_y, 0.12, OPEN_BAR, "over the bar's far end"),
        W(2.0, gx, bar_y, PLATE_H + TCP_UP, OPEN_BAR, "down around it"),
        W(1.0, gx, bar_y, PLATE_H + TCP_UP, GRIP_BAR, "grip"),
        W(0.5, gx, bar_y, PLATE_H + TCP_UP, GRIP_BAR, "settle"),
        W(1.5, gx, bar_y, 0.11, GRIP_BAR, "lift the bar"),
        W(3.0, px, 0.0, 0.11, GRIP_BAR, "over the slicer"),
        W(2.0, px, 0.0, SL_H + TCP_UP + 0.002, GRIP_BAR, "lay it across the slot"),
        W(1.0, px, 0.0, SL_H + TCP_UP + 0.002, OPEN_BAR + 0.002, "let go"),
        W(1.0, px, 0.0, 0.15, OPEN_BAR + 0.002, "straight up"),
    ]
    t_laid = sum(w.duration for w in right)
    right.append(W(2.0, *r_park, OPEN_BAR, "out of the way"))
    # left arm: pinch the top bar of the slicer's arm and swing it down through the cheese
    left = [
        W(t_laid, *l_park, OPEN_ROD, "wait"),
        W(2.5, rod_x, PIVOT_Y, rod_z + 0.026, OPEN_ROD, "over the arm's top bar"),
        W(1.0, rod_x, PIVOT_Y, rod_z, OPEN_ROD, "down around the bar"),
        W(1.0, rod_x, PIVOT_Y, rod_z, GRIP_ROD, "pinch it"),
    ]
    steps = 12
    for k in range(1, steps + 1):
        q = SWING_TO * k / steps
        left.append(W(0.5, rod_x, PIVOT_Y - LEG_R * math.sin(q), PIVOT_Z + LEG_R * math.cos(q) + ROD_UP, GRIP_ROD,
                      "swing the arm down through the cheese"))
    flat_y, flat_z = PIVOT_Y - LEG_R, PIVOT_Z + ROD_UP
    left += [
        W(0.5, rod_x, PIVOT_Y - LEG_R * math.sin(SWING_TO), PIVOT_Z + LEG_R * math.cos(SWING_TO) + ROD_UP, GRIP_ROD, "hold"),
        W(0.5, rod_x, flat_y, flat_z, GRIP_ROD, "ease off"),
        W(0.7, rod_x, flat_y, flat_z, -0.0105, "let the arm go"),
        W(1.0, rod_x, flat_y, 0.12, -0.0105, "lift away"),
    ]
    t_cut = sum(w.duration for w in left)
    left.append(W(2.0, *l_wait, OPEN_BAR, "back to the left"))
    # right arm: the half on the far side of the wire
    far_x = X_SLOT + (BAR_L - cut_at) / 2
    right += [
        W(t_cut + 1.0 - sum(w.duration for w in right), *r_park, OPEN_BAR, "wait"),
        W(2.5, far_x, 0.0, 0.12, OPEN_BAR, "over the far half"),
        W(1.5, far_x, 0.0, SL_H + TCP_UP, OPEN_BAR, "down around it"),
        W(1.0, far_x, 0.0, SL_H + TCP_UP, GRIP_BAR, "grip"),
        W(1.5, far_x, 0.0, 0.13, GRIP_BAR, "lift it"),
    ]
    t_free = sum(w.duration for w in right)  # the far half is off the slicer from here on
    right += put(right_goal, far_piece_clean)
    right.append(W(2.0, *R_HOME, OPEN_BAR, "clear"))
    # left arm: the half on the near side
    near_x = X_SLOT - cut_at / 2
    left += [
        W(t_free + 1.0 - sum(w.duration for w in left), *l_wait, OPEN_BAR, "wait"),
        W(2.5, near_x, 0.0, 0.12, OPEN_BAR, "over the near half"),
        W(1.5, near_x, 0.0, SL_H + TCP_UP, OPEN_BAR, "down around it"),
        W(1.0, near_x, 0.0, SL_H + TCP_UP, GRIP_BAR, "grip"),
        W(1.5, near_x, 0.0, 0.13, GRIP_BAR, "lift it"),
    ]
    left += put(left_goal, not far_piece_clean)
    left.append(W(2.0, *L_HOME, OPEN_BAR, "clear"))
    end = max(sum(w.duration for w in right), sum(w.duration for w in left)) + 3.0
    right.append(W(end - sum(w.duration for w in right), *R_HOME, OPEN_BAR, "wait"))
    left.append(W(end - sum(w.duration for w in left), *L_HOME, OPEN_BAR, "wait"))
    return {"right": right, "left": left}
