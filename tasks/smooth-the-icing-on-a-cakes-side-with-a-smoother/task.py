"""A cake stands on a revolving cake stand: a decorator's foam practice round on a cake board, iced on its side about
5 mm thick and turned once under the coarse teeth of a decorating comb held in a jig, so that ridges run round it and
the comb's take-off has left a scar down the side. The stand's aluminium top turns freely on its cast-iron base, which
stands loose on its pad. A decorating comb and icing smoother, an aluminium plate with three toothed edges and one
straight edge, stands on a short edge in the groove of a beech block, leaning on the block's back, its straight edge
upright.

How it is built. The stand's top turns on a free hinge on the base; its bearing drags with a small computed torque.
The cake (dummy and board) is one free body resting on the top on its non-slip mat. The icing is a field of
thicknesses on the side, 96 cells round by 34 up, that turns with the cake; each cell is drawn as a thin tile on the
cake, moved in and out with its thickness and hidden where the foam shows. Every control step the smoother's
straight edge is found in the cake's frame, row by row; where it has passed a cell since the last step, it cuts the
icing down to its own distance from the foam and keeps what it cut as a roll on the blade, or fills a lower cell from
that roll up to its distance; the cut pushes the blade back along the turning and outward and slows the cake by the
same force. Where the edge leaves the icing, what is left of the roll stays there as a mark. Only the straight edge
works the icing.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder, hinge, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Smooth the icing on the side of the cake with the smoother, then put the smoother back in its block."
RUN_LIMIT_STEPS = 2700  # 135 s at 20 Hz: 35 % over the first submission's reference run (100 s), 42 % over the revised one (94.8 s)
TOP = TABLE_TOP[SCENE]

# --- the icing as a material: every number the simulation uses, with its source (one table; the README has it) ---
MATERIAL = {
    "mean thickness": (0.005, "m", "iced on the side with an offset spatula, about 5 mm (assumed)"),
    "ridge pitch": (0.006, "m", "the comb's coarse teeth, combed once round (assumed (a))"),
    "ridge height": (0.003, "m", "high to low (assumed (a))"),
    "scar height": (0.002, "m", "the comb's take-off, a ridge down the side, place by seed (assumed)"),
    "out of round": (0.001, "m", "high to low once round, by seed, at most: the ridge tops within 0.5 mm of their radius, the reset's tolerance (assumed)"),
    "cut push": (4000.0, "Pa", "0.4 N per mm of depth per 100 mm of edge, along the turning and outward each (assumed, kitchen test)"),
    "roll": (10.0e-6, "m2", "the roll of icing along the edge that it fills from, in section: 10 mm2; what it cuts beyond that stays on the blade's face (assumed)"),
    "fill reach": (0.002, "m", "a cell this far under the edge is still filled from the roll (assumed)"),
    "bare": (0.001, "m", "icing thinner than this shows the foam (assumed)"),
}
THICK, PITCH, RIDGE, SCAR, OUT_OF_ROUND, CUT_PUSH, ROLL, FILL_REACH, BARE = (MATERIAL[k][0] for k in MATERIAL)

# The icing is tinted pale blue with gel food colour and the dummy is white foam: bare foam shows white on blue, to a
# camera and to a bench photo, and the ridges throw a visible shade (white on a white dummy showed neither).
FOAM, ICING, BOARD_GOLD, ALUMINIUM, IRON, BEECH, PAD = (0.95, 0.95, 0.94), (0.66, 0.80, 0.94), (0.80, 0.70, 0.42), (0.78, 0.80, 0.82), (0.16, 0.16, 0.17), (0.82, 0.64, 0.44), (0.12, 0.12, 0.12)

# --- the revolving cake stand: Ateco 612 (retailers' figures: aluminium top 305 mm across, 127 mm high, about 3 kg;
# the base's sizes and the share of the mass are assumed (a)) ---
CENTRE = np.array([0.30, 0.0])
STAND_H, TOP_R, TOP_T = 0.127, 0.1525, 0.003  # m: the top's upper face over the bench; its radius; its thickness
FOOT_R, FOOT_H, COLUMN_R, HUB_R = 0.085, 0.025, 0.030, 0.045  # (a)
STAND_MASS, TOP_MASS = 2.4, 0.6  # kg: cast-iron base; aluminium top with its hub (a)
BEARING_DRAG = 0.01  # N m (a): to be measured by the coast-down from one turn a second
DRAG_FULL = 0.2  # rad/s from which the drag is all there (below, in proportion: no chatter at rest)
body("stand", [cylinder(FOOT_R, FOOT_H, pos=(0.0, 0.0, FOOT_H / 2), friction=(0.8, 0.7), friction_combine="max"),  # the non-slip pad
               cylinder(COLUMN_R, STAND_H - TOP_T - FOOT_H, pos=(0.0, 0.0, (FOOT_H + STAND_H - TOP_T) / 2))],
     pos=(*CENTRE, TOP), mass=STAND_MASS, color=IRON)  # fmt: skip
TURN_AT = (*CENTRE, TOP + STAND_H - TOP_T / 2)
body("turntable", [cylinder(TOP_R, TOP_T), cylinder(HUB_R, 0.008, pos=(0.0, 0.0, -TOP_T / 2 - 0.004), solid=False)],
     pos=TURN_AT, mass=TOP_MASS, color=ALUMINIUM, friction=(0.45, 0.4))  # fmt: skip
hinge("spin", "turntable", "stand", anchor=TURN_AT, axis=(0.0, 0.0, 1.0), limits=None, damping=0.0002)

# --- the cake: a foam practice round 152 mm across and 102 mm high (Wilton/decorators' dummies) on a 203 mm cake
# board on a non-slip mat; about 450 g with board and icing (a). The icing (Wilton Creamy White Decorator Icing,
# medium consistency, 20 to 22 C) is drawn as tiles; its weight is in the cake's mass ---
DUMMY_R, DUMMY_H, BOARD_R, BOARD_T, MAT_T = 0.076, 0.102, 0.1015, 0.003, 0.0015
CAKE_MASS = 0.45
COLUMNS, ROWS = 96, 34
ROW_H = DUMMY_H / ROWS  # 3 mm: two rows to a ridge
CELL_ANGLE = 2.0 * math.pi / COLUMNS
TILE_DEPTH = 0.011  # m radially: more than the thickest icing, so a tile reaches into the foam
CAKE_AT = np.array([*CENTRE, TOP + STAND_H + MAT_T + BOARD_T])  # the cake's frame: on its axis, at the board's top face
_cake = [
    cylinder(BOARD_R, MAT_T, pos=(0.0, 0.0, -BOARD_T - MAT_T / 2), color=PAD, friction=(1.0, 0.9), friction_combine="max"),  # the mat
    cylinder(BOARD_R, BOARD_T, pos=(0.0, 0.0, -BOARD_T / 2), color=BOARD_GOLD, friction=(0.3, 0.25)),
    cylinder(DUMMY_R, DUMMY_H, pos=(0.0, 0.0, DUMMY_H / 2), color=FOAM, friction=(0.5, 0.45)),
]
TILE0 = len(_cake)  # tile of column j, row k: shape TILE0 + j * ROWS + k
_tile_w = 2.0 * (DUMMY_R + THICK) * math.tan(CELL_ANGLE / 2) * 1.08  # a little over the cell's width: no gaps between tiles
for _j in range(COLUMNS):
    _a = (_j + 0.5) * CELL_ANGLE
    for _k in range(ROWS):
        _r = DUMMY_R + THICK - TILE_DEPTH / 2
        _cake.append(box((TILE_DEPTH, _tile_w, ROW_H * 1.02), pos=(_r * math.cos(_a), _r * math.sin(_a), (_k + 0.5) * ROW_H), yaw=_a, color=ICING, solid=False))
body("cake", _cake, pos=tuple(CAKE_AT), mass=CAKE_MASS, color=FOAM)

# --- the smoother: Ateco 1447 decorating comb and icing smoother, aluminium plate 6 x 3.8 in, 152 x 96.5 mm (the maker's
# catalogue; retailers list 6 x 4 in), 2 mm thick (a), 79 g by its size (a); three toothed edges and one straight edge;
# which edge is straight is not given: a long one is assumed (a) ---
EDGE_L, PLATE_W, PLATE_T, SMOOTHER_MASS = 0.152, 0.0965, 0.002, 0.079
# Its frame: the plate's middle; x across the plate towards the straight edge (at x = PLATE_W / 2), y through the
# plate, z along the straight edge.
# --- the beech block (a): a back 39 mm high, leaning LEAN from upright, and at its foot a groove 6 mm wide and 4 mm
# deep with a lip in front and a non-slip strip on its floor, along the block's 140 mm, on a base 140 x 80 x 15 mm that reaches out under the leaning
# plate; it stands loose on the bench. The
# smoother stands in the groove on a short edge and leans on the back, its straight edge upright along the back and
# facing FACING (the way the hand points when it takes it). Upright, the plate showed the top camera, which hangs almost
# over the block, only its 2 mm top edge (recording b2k5); leaning, its face shows. A slot 3 mm wide and 18 mm deep
# with a jaw on either side jammed the plate as it was drawn out, and the block came away on it (b2k20, b2k28, b2k30):
# with nothing in front of the plate but the low lip 4.8 mm off, it cannot jam. A back 30 mm high could not hold the
# plate: leaning on its top edge, the plate's foot needs friction 0.89 to stay (39 mm: 0.55); on beech it slid to the
# lip and the block, 50 mm wide, tipped over onto its back before anything moved (rest trial b2k37). The back cannot be
# higher than 39 mm, where the closing fingertip passes 3 mm over it (44 mm: 2.0 mm into it, b2k40); the non-slip strip
# (friction 1.0) holds the foot. ---
BLOCK_AT, FACING, LEAN = np.array([0.17, -0.20]), math.radians(65.0), math.radians(-30.0)  # where the level pinch, its approach from behind and the draw out of the groove are all reached (b2k12)
BLOCK_L, BASE_W, BASE_OUT, BASE_H, BLOCK_MASS = 0.140, 0.080, 0.015, 0.015, 0.18  # BASE_OUT: the base's middle off the groove's line, towards the lean; 0.18 kg: beech, 0.7 g/cm3
GROOVE_W, LIP_H, LIP_T, BACK_H, BACK_T, GROOVE_FLOOR = 0.006, 0.004, 0.006, 0.039, 0.010, 0.008
SLOT_TILT = mk.axis_angle((1.0, 0.0, 0.0), LEAN)  # the groove's frame in the block's: x along it, y across it towards the back, z up the back
_BACK_FACE = PLATE_T / 2 + 0.0002  # the back's face across the groove: the plate, its middle plane at 0, settles 0.2 mm onto it
_y_low, _y_high = _BACK_FACE - GROOVE_W - LIP_T, _BACK_FACE + BACK_T  # the groove's floor across, from the lip's front face to the back's
SLOT_AT = np.array([0.0, 0.0, BASE_H - min(float((SLOT_TILT @ np.array([0.0, y, -GROOVE_FLOOR]))[2]) for y in (_y_low, _y_high))])  # the groove's floor where the plate stands, in the block's frame
_tilt_q = tuple(mk.matrix_to_quat(SLOT_TILT))


def _in_slot(point) -> tuple[float, ...]:
    """A point of the groove's frame in the block's frame."""
    return tuple(float(v) for v in SLOT_AT + SLOT_TILT @ np.asarray(point, dtype=float))


_block = [
    box((BLOCK_L, BASE_W, BASE_H), pos=(0.0, BASE_OUT, BASE_H / 2)),
    box((BLOCK_L, _y_high - _y_low, GROOVE_FLOOR), pos=_in_slot((0.0, (_y_low + _y_high) / 2, -GROOVE_FLOOR / 2)), quat=_tilt_q,
        friction=(1.0, 0.9), friction_combine="max"),  # the groove's floor, lined with a non-slip strip (a)
    box((BLOCK_L, BACK_T, BACK_H), pos=_in_slot((0.0, _BACK_FACE + BACK_T / 2, BACK_H / 2)), quat=_tilt_q),  # the back the plate leans on
    box((BLOCK_L, LIP_T, LIP_H), pos=_in_slot((0.0, _BACK_FACE - GROOVE_W - LIP_T / 2, LIP_H / 2)), quat=_tilt_q),  # the lip in front of the groove
]
BLOCK_DROP = 0.001  # m the block is built over the bench, so that it settles at the start: a body that never moves was not drawn in the first pictures (b2k44, b2k46: the block showed only once the hand had stirred it)
body("block", _block, pos=(*BLOCK_AT, TOP + BLOCK_DROP), yaw=FACING, mass=BLOCK_MASS, color=BEECH, friction=(0.5, 0.45))
_block_r = mk.axis_angle((0.0, 0.0, 1.0), FACING)


def _settled_lean() -> float:
    """How much further than the groove the plate leans where it comes to rest (rad): its foot on the floor against the
    lip, its back face on the back's top edge. With h = PLATE_T / 2, in the groove's frame:
    (GROOVE_W - h cos d) cos d - (BACK_H - h sin d) sin d = h."""
    h = PLATE_T / 2

    def short(d: float) -> float:  # how far the back's top edge stands off the plate's back face (m)
        return (GROOVE_W - h * math.cos(d)) * math.cos(d) - (BACK_H - h * math.sin(d)) * math.sin(d) - h

    low, high = 0.0, 0.5
    for _ in range(60):
        middle = (low + high) / 2
        low, high = (middle, high) if short(middle) > 0.0 else (low, middle)
    return (low + high) / 2


SETTLED = _settled_lean()  # 5.87 degrees: 35.9 from upright in all; the place rest trial b2k47 found it at, to 0.01 mm
SMOOTHER_LIFT = 0.001  # m the plate is built up the groove from that place and drops into it at the start, as the block drops onto the bench: built upright in the groove (it settled 4 mm in the first step), it was missing from the harness's first two pictures, the block not (model run b2k50)
_settle_r = mk.axis_angle((1.0, 0.0, 0.0), -SETTLED)
_foot = np.array([0.0, _BACK_FACE - GROOVE_W + PLATE_T / 2 * math.cos(SETTLED), PLATE_T / 2 * math.sin(SETTLED) + SMOOTHER_LIFT])  # the middle of the plate's lower edge, in the groove's frame
PLATE_IN_BLOCK = _block_r @ SLOT_TILT @ _settle_r  # the plate's frame in the world, standing in the groove: x along it, z up the back
SMOOTHER_AT = np.array([*BLOCK_AT, TOP + BLOCK_DROP]) + _block_r @ (SLOT_AT + SLOT_TILT @ (_foot + _settle_r @ np.array([0.0, 0.0, EDGE_L / 2])))
# A check of the hand's hold, not the task (trials.py hold): CAKE_SMOOTHER_AT = "x,y,z,qx,qy,qz,qw" builds the smoother
# there, where the hand will pinch it, welded to the world until the check cuts the weld.
HELD_AT = [float(v) for v in os.environ["CAKE_SMOOTHER_AT"].split(",")] if os.environ.get("CAKE_SMOOTHER_AT") else None
body("smoother", [box((PLATE_W, PLATE_T, EDGE_L))], pos=tuple(HELD_AT[:3]) if HELD_AT else tuple(SMOOTHER_AT), quat=tuple(HELD_AT[3:]) if HELD_AT else tuple(mk.matrix_to_quat(PLATE_IN_BLOCK)),
     mass=SMOOTHER_MASS, color=ALUMINIUM, friction=(0.6, 0.5))  # fmt: skip
if HELD_AT:
    weld("smoother_held", "smoother", None)
OBJECTS = list(mk.MECH.bodies.values())

# --- the rules: the side is read as a bench reads it, at 16 marks on the cake board, every 22.5 degrees ---
BAND = range(3, 31)  # rows read for gaps and bare places: from 9 mm over the board to 9 mm under the top
MARKS = 16
MARK_COLUMNS = np.arange(MARKS) * (COLUMNS // MARKS)  # the marks stand at the middles of columns 0, 6, ..., 90
# Up: a steel rule held upright against the side at each mark rests on the side's two outermost points over its whole
# height; the gaps under it are read in the band. Round: the arc gauge, a 50 mm chord of a circle of radius 81 mm (the
# dummy's 76 mm and 5 mm), lies level on a spacer on the board at three heights and is centred on each mark in turn;
# its 16 placements overlap (36 degrees each, 22.5 apart). Its edge, 3 mm thick, covers one row of the field.
GAUGE_R, GAUGE_CHORD = 0.081, 0.050
GAUGE_ROWS = (8, 16, 25)  # on spacers 24, 48 and 75 mm high: its edge touches 24 to 27, 48 to 51 and 75 to 78 mm over the board
GAUGE_HALF = math.asin(GAUGE_CHORD / 2 / GAUGE_R) / CELL_ANGLE  # columns from the gauge's middle to either end: 4.79
EVEN = 0.0012  # m: the largest gap under the rule or the gauge for 1.0 (a 1.2 mm feeler leaf does not go in)
MARK_MOST, MARK_WIDE = 0.002, 4  # m a mark may stand at 0.75; columns it may span round
BARE_SMALL = 2.0e-4  # m2: a bare place smaller than this (2 cm2) is one flaw
LOW_RIDGES = 0.0015  # m: the gap under the upright rule under this at half the marks or more: 0.5
THICK_ROW = 16  # the thickness is read with a depth pin at the marks, 48 to 51 mm over the board, after every other reading
THIN_MEAN = 0.003  # m: the mean of those 16 readings at least this for 1.0
CENTRED, OFF_PLACE = 0.005, 0.015  # m: the cake this near the top's axis for 1.0; the cake or the stand this far off its place: 0
WORKED_QUARTER = COLUMNS // 4  # columns the edge has cut over at least half the band: 0.25
CLEAR = 0.03  # m both hands' collision hulls keep from the smoother, the cake, the stand's top and the block
SIDES = ("left", "right")
STATE: dict = {}


# --- the icing field ---
def column_angle(j) -> np.ndarray:
    return (np.asarray(j) + 0.5) * CELL_ANGLE


def start_field(seed: int | None) -> tuple[np.ndarray, int, float, float]:
    """The icing as combed: (thickness[column, row] in m, the scar's column, the out-of-round high to low, its angle).
    Ridges run round: the rows alternate a half ridge above and below the mean; by seed, the scar's place and the
    out-of-round (up to OUT_OF_ROUND high to low, once round)."""
    rng = np.random.default_rng(seed)
    scar = int(rng.integers(COLUMNS)) if seed is not None else 0
    oor = float(rng.uniform(0.0, OUT_OF_ROUND)) if seed is not None else OUT_OF_ROUND / 2
    phase = float(rng.uniform(0.0, 2.0 * math.pi)) if seed is not None else 0.0
    rows = np.arange(ROWS)
    profile = THICK + np.where(rows % 2 == 0, RIDGE / 2, -RIDGE / 2)  # pitch: two rows of 3 mm
    field = profile[None, :] + (oor / 2) * np.cos(column_angle(np.arange(COLUMNS)) - phase)[:, None]
    field[scar, :] += SCAR
    return field, scar, oor, phase


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    field, scar, oor, phase = start_field(seed)
    STATE.clear()
    STATE.update(seed=seed, field=field, scar=scar, out_of_round=oor, oor_phase=phase, roll=0.0, carried=0.0, contact=None, cut_rows=np.zeros((COLUMNS, ROWS), dtype=bool),
                 drawn=None, hidden=set(), force=np.zeros(3), hands=None, cut_now=0.0, marks_left=0, stand_at=None)  # fmt: skip
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    if mk.MECH.environment is not None:
        draw(all_cells=True)


# --- where the straight edge is ---
def edge_points() -> tuple[np.ndarray, np.ndarray]:
    """The straight edge's two ends in the world: its lower end (the plate's -z end) first."""
    position, rotation = mk.pose("smoother")
    return position + rotation @ np.array([PLATE_W / 2, 0.0, -EDGE_L / 2]), position + rotation @ np.array([PLATE_W / 2, 0.0, EDGE_L / 2])


def to_cake(point) -> np.ndarray:
    position, rotation = mk.pose("cake")
    return rotation.T @ (np.asarray(point, dtype=float) - position)


def edge_rows() -> dict:
    """Row -> (angle in the cake's frame, distance from the foam, the point in the world) where the straight edge
    crosses that row's middle height; rows the edge does not reach are left out."""
    a_w, b_w = edge_points()
    a, b = to_cake(a_w), to_cake(b_w)
    out = {}
    if abs(b[2] - a[2]) < 1e-6:
        return out
    for k in range(ROWS):
        s = ((k + 0.5) * ROW_H - a[2]) / (b[2] - a[2])
        if 0.0 <= s <= 1.0:
            q = a + s * (b - a)
            out[k] = (math.atan2(q[1], q[0]) % (2.0 * math.pi), math.hypot(q[0], q[1]) - DUMMY_R, a_w + s * (b_w - a_w))
    return out


def turned(start: float, end: float) -> float:
    """The angle from ``start`` to ``end`` the shorter way round, in (-pi, pi]."""
    turn = (end - start) % (2.0 * math.pi)
    return turn - 2.0 * math.pi if turn > math.pi else turn


def crossed(start: float, end: float) -> list[tuple[int, float]]:
    """The columns whose middles the edge passed going from angle ``start`` to ``end`` (the shorter way), in the order
    passed, each with how far along the way (0 to 1) it was passed; then the column it stands in now (at 1)."""
    turn = turned(start, end)
    out = []
    if turn > 0.0:  # middles in (start, start + turn]
        j = math.floor(start / CELL_ANGLE - 0.5) + 1
        while (j + 0.5) * CELL_ANGLE <= start + turn + 1e-12:
            out.append((j % COLUMNS, ((j + 0.5) * CELL_ANGLE - start) / turn))
            j += 1
    elif turn < 0.0:  # middles in [start + turn, start)
        j = math.ceil(start / CELL_ANGLE - 0.5) - 1
        while (j + 0.5) * CELL_ANGLE >= start + turn - 1e-12:
            out.append((j % COLUMNS, ((j + 0.5) * CELL_ANGLE - start) / turn))
            j -= 1
    here = int(end // CELL_ANGLE) % COLUMNS
    if not out or out[-1][0] != here:
        out.append((here, 1.0))
    return [(j, min(1.0, max(0.0, along))) for j, along in out]


TINY = 1e-9  # m: thickness differences under this are rounding, not a cut or a fill


def cell_area(j: int, k: int) -> float:
    """m2 of icing surface a cell stands for (at the mean surface)."""
    return (DUMMY_R + THICK) * CELL_ANGLE * ROW_H


def work_icing() -> tuple[float, np.ndarray]:
    """Once per control step: the edge cuts and fills where it has passed since the last step. Returns (m2 of cut
    section now, the cut's middle point in the world); keeps STATE['contact'] (row -> angle, distance) for the next."""
    field, rows = STATE["field"], edge_rows()
    before = STATE["contact"] or {}
    contact, changed = {}, STATE.setdefault("changed", set())
    cut_section, weighted = 0.0, np.zeros(3)
    area = cell_area(0, 0)
    for k, (angle, distance, point) in rows.items():
        here = int(angle // CELL_ANGLE) % COLUMNS
        if distance > field[here, k] + FILL_REACH:  # not on the icing in this row
            continue
        contact[k] = (angle, distance)
        start_angle, start_distance = before.get(k, (angle, distance))
        turn = turned(start_angle, angle)
        # the depth the edge cuts at now: what stands over it in the column it is moving into (it cut the one it is in)
        ahead = (here + (1 if turn > 0 else -1)) % COLUMNS if abs(turn) > 1e-9 else here
        depth_now = max(0.0, field[ahead, k] - max(0.0, distance), field[here, k] - max(0.0, distance))
        for j, along in crossed(start_angle, angle):
            d = max(0.0, start_distance + along * (distance - start_distance))
            t = field[j, k]
            if t > d + TINY:  # cut down to the edge, onto the roll
                STATE["roll"] += (t - d) * area
                field[j, k] = d
                STATE["cut_rows"][j, k] = True
                changed.add((j, k))
                depth_now = max(depth_now, t - d)
            elif t < d - TINY and d - t <= FILL_REACH and STATE["roll"] > 0.0:  # fill from the roll up to the edge
                add = min(d - t, STATE["roll"] / area)
                field[j, k] = t + add
                STATE["roll"] -= add * area
                changed.add((j, k))
        cut_section += depth_now * ROW_H
        weighted += depth_now * ROW_H * point
    # rows the edge has left: their share of the roll stays where the edge left them, as a mark
    left = [k for k in before if k not in contact]
    if left and STATE["roll"] > 0.0:
        share = STATE["roll"] / len(before)
        for k in left:
            j = int(before[k][0] // CELL_ANGLE) % COLUMNS
            field[j, k] += share / area
            changed.add((j, k))
        STATE["roll"] = STATE["roll"] - share * len(left) if contact else 0.0
        STATE["marks_left"] += 1
    # what the roll along the rows still worked cannot hold stays on the blade's face, out of the work
    most = ROLL * ROW_H * len(contact)
    if STATE["roll"] > most:
        STATE["carried"] += STATE["roll"] - most
        STATE["roll"] = most
    STATE["contact"] = contact or None
    return cut_section, (weighted / cut_section if cut_section > 0 else np.zeros(3))


def centre_of_mass(name: str) -> np.ndarray:
    data = mk.MECH.environment.unwrapped.scene.rigid_objects[name].data
    origin = mk.MECH.environment.unwrapped.scene.env_origins[0].detach().cpu().numpy().astype(float)
    return data.root_com_pos_w.torch[0].detach().cpu().numpy().astype(float) - origin


def cut_force(section: float, at: np.ndarray) -> np.ndarray:
    """N on the blade: CUT_PUSH times the cut section, along the icing's motion past the edge and outward from the
    cake's axis, each; nothing while the edge does not move through the icing."""
    if section <= 0.0:
        return np.zeros(3)
    cake_p, cake_r = mk.pose("cake")
    radial = at - cake_p
    radial -= (radial @ cake_r[:, 2]) * cake_r[:, 2]
    if np.linalg.norm(radial) < 1e-9:
        return np.zeros(3)
    out = radial / np.linalg.norm(radial)
    spin = mk.velocity("cake")[1]
    blade_v = mk.velocity("smoother")[0]
    icing_v = mk.velocity("cake")[0] + np.cross(spin, at - cake_p) - blade_v  # the icing's motion past the edge
    icing_v -= (icing_v @ out) * out
    icing_v -= (icing_v @ cake_r[:, 2]) * cake_r[:, 2]
    speed = float(np.linalg.norm(icing_v))
    if speed < 1e-6:
        return np.zeros(3)
    along = icing_v / speed
    scale = min(1.0, speed / 0.002)  # in full from 2 mm/s
    return CUT_PUSH * section * scale * (along + out)


def bearing() -> None:
    spin = float(mk.velocity("turntable")[1][2])
    mk.push("turntable", torque=(0.0, 0.0, -BEARING_DRAG * max(-1.0, min(1.0, spin / DRAG_FULL))))


@mk.on_step
def step() -> None:
    if mk.MECH.environment is None:
        return
    if STATE.get("stand_at") is None:
        STATE["stand_at"] = mk.pose("stand")[0].copy()
    bearing()
    section, at = work_icing()
    force = cut_force(section, at)
    STATE["force"], STATE["cut_now"] = force, section
    if np.any(force != 0.0):
        mk.push("smoother", force=tuple(force), torque=tuple(np.cross(at - centre_of_mass("smoother"), force)))
        mk.push("cake", force=tuple(-force), torque=tuple(np.cross(at - centre_of_mass("cake"), -force)))
    else:
        mk.push("smoother")
        mk.push("cake")
    read_hands()
    draw()


# --- drawing the icing ---
def tile(j: int, k: int) -> int:
    return TILE0 + j * ROWS + k


def tile_place(j: int, k: int, thickness: float) -> tuple[float, float, float]:
    r = DUMMY_R + thickness - TILE_DEPTH / 2
    a = float(column_angle(j))
    return r * math.cos(a), r * math.sin(a), (k + 0.5) * ROW_H


def draw(all_cells: bool = False) -> None:
    """Move the tiles of the cells that changed to their thickness; hide the bare ones (in the simulator only)."""
    if mk.MECH.environment is None or "cake" not in mk.MECH.body_prims:
        STATE["changed"] = set()
        return
    from isaaclab.sim.utils.stage import get_current_stage

    field = STATE["field"]
    cells = [(j, k) for j in range(COLUMNS) for k in range(ROWS)] if all_cells else sorted(STATE.get("changed", ()))
    stage = get_current_stage()
    base = mk.MECH.body_prims["cake"]
    for j, k in cells:
        attribute = stage.GetPrimAtPath(f"{base}/geometry/shape{tile(j, k)}").GetAttribute("xformOp:translate")
        kind = type(attribute.Get())
        attribute.Set(kind(*tile_place(j, k, float(field[j, k]))))
        bare = field[j, k] < BARE
        if bare != ((j, k) in STATE["hidden"]):
            mk.show("cake", tile(j, k), not bare)
            (STATE["hidden"].add if bare else STATE["hidden"].discard)((j, k))
    STATE["changed"] = set()


# --- the hands: their collision hulls (the gripper and both finger tips, from the arm's meshes) ---
_HULLS: dict = {}
_LINKS: dict = {}  # (environment, side, link) -> the robot's body index


def read_hands() -> None:
    """Every hull corner of both hands in the world, from the simulator (STATE['hands']: side -> points)."""
    from yam_arena import analysis

    if not _HULLS:
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in analysis.link_hulls().get(link, [])])
    base = mk.MECH.environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    hands = {}
    for side in SIDES:
        robot = base.scene[f"{side}_robot"]
        points = []
        for link, local in _HULLS.items():
            key = (id(base), side, link)
            if key not in _LINKS:
                found = robot.find_bodies(link)[0]
                if not found:
                    raise RuntimeError(f"the {side} arm has no body {link!r} to read its hull from")
                _LINKS[key] = found[0]
            pose = robot.data.body_link_pose_w.torch[0, _LINKS[key]].detach().cpu().numpy().astype(float)
            points.append(local @ mk.quat_to_matrix(pose[3:7]).T + pose[:3] - origin)
        hands[side] = np.vstack(points)
    STATE["hands"] = hands


# --- what the rules read ---
def band() -> np.ndarray:
    return STATE["field"][:, BAND.start : BAND.stop]


def gaps_under(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """The gap at each point (x increasing, y outward) under a straight edge pressed onto the points from outside:
    its depth under their outer convex hull. The edge rests on two hull points with the place between them, rocked
    either way on a crown; a gap is read only between the points it rests on."""
    hull: list[int] = []
    for i in range(len(x)):
        while len(hull) >= 2 and (x[hull[-1]] - x[hull[-2]]) * (y[i] - y[hull[-1]]) - (y[hull[-1]] - y[hull[-2]]) * (x[i] - x[hull[-1]]) >= 0.0:
            hull.pop()  # the last point is not outside the line from the one before it to this one
        hull.append(i)
    return np.interp(x, x[hull], y[hull]) - y


ROW_MIDDLES = (np.arange(ROWS) + 0.5) * ROW_H


def upright_gaps(field: np.ndarray | None = None) -> np.ndarray:
    """Per mark, the largest gap under the steel rule held upright against the side there, in the band (m)."""
    field = STATE["field"] if field is None else field
    return np.array([gaps_under(ROW_MIDDLES, field[j])[BAND.start : BAND.stop].max() for j in MARK_COLUMNS])


def round_gaps(field: np.ndarray | None = None) -> np.ndarray:
    """[height, mark]: the largest gap under the arc gauge at that height, centred on the mark (m). The gauge's ends
    stand GAUGE_HALF columns either side of the mark's column, where the thickness is taken in a straight line between
    the columns' middles. The thickness stands for the radius: the gauge's own curve is taken as the side's (its 81 mm
    against a finished radius of 79 to 81 mm adds at most 0.1 mm at its ends; see the README)."""
    field = STATE["field"] if field is None else field
    n = math.ceil(GAUGE_HALF)  # the columns under the gauge: the mark's and n - 1 either side wholly, the n-th in part
    reach = GAUGE_HALF - (n - 1)
    x = np.concatenate([[-GAUGE_HALF], np.arange(1 - n, n), [GAUGE_HALF]])
    out = np.zeros((len(GAUGE_ROWS), MARKS))
    for a, k in enumerate(GAUGE_ROWS):
        for b, j in enumerate(MARK_COLUMNS):
            t = field[(j + np.arange(-n, n + 1)) % COLUMNS, k]
            y = np.concatenate([[t[1] + reach * (t[0] - t[1])], t[1:-1], [t[-2] + reach * (t[-1] - t[-2])]])
            out[a, b] = gaps_under(x, y).max()
    return out


def largest_gap(field: np.ndarray | None = None) -> float:
    """The largest gap under the rule or the gauge anywhere they are laid (m)."""
    return max(float(upright_gaps(field).max()), float(round_gaps(field).max()))


def thickness_readings(field: np.ndarray | None = None) -> np.ndarray:
    """The depth pin's readings at the 16 marks, 48 to 51 mm over the board (m)."""
    field = STATE["field"] if field is None else field
    return field[MARK_COLUMNS, THICK_ROW]


def regions(field: np.ndarray | None = None) -> dict:
    """For the readings: the largest gap under the rule or the gauge laid clear of the comb's scar, and laid over it
    (the gauge over the scar's column, the rule at the scar's column) (m)."""
    field = STATE["field"] if field is None else field
    up, round_ = upright_gaps(field), round_gaps(field)
    scar = STATE["scar"]
    rule_over = MARK_COLUMNS == scar
    gauge_over = np.array([abs((scar - j + COLUMNS // 2) % COLUMNS - COLUMNS // 2) <= math.ceil(GAUGE_HALF) for j in MARK_COLUMNS])  # the scar's column under the gauge, wholly or in part
    over = []
    if rule_over.any():
        over.append(float(up[rule_over].max()))
    if gauge_over.any():
        over.append(float(round_[:, gauge_over].max()))
    return {"plain side": max(float(up[~rule_over].max()), float(round_[:, ~gauge_over].max())), "over the scar": max(over, default=0.0)}


def groups(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """Connected groups of True cells (neighbours up, down, round and across corners; round wraps)."""
    seen = np.zeros_like(mask, dtype=bool)
    out = []
    columns, rows = mask.shape
    for j0, k0 in zip(*np.nonzero(mask)):
        if seen[j0, k0]:
            continue
        group, todo = [], [(int(j0), int(k0))]
        seen[j0, k0] = True
        while todo:
            j, k = todo.pop()
            group.append((j, k))
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    jj, kk = (j + dj) % columns, k + dk
                    if 0 <= kk < rows and mask[jj, kk] and not seen[jj, kk]:
                        seen[jj, kk] = True
                        todo.append((jj, kk))
        out.append(group)
    return out


def bare_cells(field: np.ndarray | None = None) -> np.ndarray:
    """The band's cells (columns x band rows) where the foam shows."""
    field = STATE["field"] if field is None else field
    return field[:, BAND.start : BAND.stop] < BARE


def ridges_under(field: np.ndarray | None = None) -> int:
    """The marks at which the gap under the upright rule is under LOW_RIDGES."""
    return int((upright_gaps(field) < LOW_RIDGES).sum())


def one_flaw(field: np.ndarray | None = None) -> str | None:
    """'mark' or 'bare' when the side is even but for one flaw: one mark (cells standing out above the side's median,
    connected, spanning at most MARK_WIDE columns round, at most MARK_MOST high) or one bare place in the band smaller
    than BARE_SMALL; mended (its cells set to the median), no gap under the rule or the gauge is over EVEN. None
    otherwise."""
    field = STATE["field"] if field is None else field
    median = float(np.median(field[:, BAND.start : BAND.stop]))
    bare = bare_cells(field)
    if bare.any():
        places = groups(bare)
        if len(places) != 1 or len(places[0]) * cell_area(0, 0) >= BARE_SMALL:
            return None
        mended = field.copy()
        for j, k in places[0]:
            mended[j, BAND.start + k] = median
        return "bare" if largest_gap(mended) <= EVEN else None
    high = groups(field > median + EVEN / 2)  # over the whole height: a mark runs down the side
    if not high:
        return None
    mark = max(high, key=lambda g: float(max(field[j, k] for j, k in g)))
    if len({j for j, _ in mark}) > MARK_WIDE:  # a mark is narrow: a band of ridges round the cake is not one
        return None
    if max(field[j, k] for j, k in mark) - median > MARK_MOST:
        return None
    mended = field.copy()
    for j, k in mark:
        mended[j, k] = median
    return "mark" if largest_gap(mended) <= EVEN else None


def off_centre() -> float:
    """m the cake's axis stands off the top's axis."""
    cake, top = mk.pose("cake")[0], mk.pose("turntable")[0]
    return float(math.hypot(*(cake[:2] - top[:2])))


def stand_moved() -> float:
    start = STATE.get("stand_at")
    built = np.array(mk.built_pose("stand")[0]) if start is None else start
    return float(math.hypot(*(mk.pose("stand")[0][:2] - built[:2])))


def slot_frame() -> tuple[np.ndarray, np.ndarray]:
    """The groove's frame in the world as the block stands now: on its floor, where the plate's middle plane stands
    when the plate leans on the back; x along it, y across it towards the back, z up the back (leaning LEAN)."""
    block_p, block_r = mk.pose("block")
    return block_p + block_r @ SLOT_AT, block_r @ SLOT_TILT


def in_block() -> bool:
    """The smoother's two corners deepest in the block's groove inside it: along the groove, between its lip and the
    back, and under the lip's top (either short edge down)."""
    position, rotation = mk.pose("smoother")
    slot_p, slot_r = slot_frame()
    corners = [slot_r.T @ (position + rotation @ np.array([x, 0.0, z]) - slot_p) for x in (-PLATE_W / 2, PLATE_W / 2) for z in (-EDGE_L / 2, EDGE_L / 2)]
    for local in sorted(corners, key=lambda c: c[2])[:2]:
        if not (abs(local[0]) <= BLOCK_L / 2 and _BACK_FACE - GROOVE_W - 0.0005 <= local[1] <= _BACK_FACE + 0.0005 and -0.002 <= local[2] <= LIP_H):
            return False
    return True


def _box_distance(points: np.ndarray, name: str, low, high) -> float:
    position, rotation = mk.pose(name)
    local = (points - position) @ rotation
    gap = np.maximum(np.maximum(np.asarray(low) - local, local - np.asarray(high)), 0.0)
    return float(np.linalg.norm(gap, axis=1).min())


def _cylinder_distance(points: np.ndarray, name: str, radius: float, low: float, high: float) -> float:
    position, rotation = mk.pose(name)
    local = (points - position) @ rotation
    radial = np.maximum(np.hypot(local[:, 0], local[:, 1]) - radius, 0.0)
    axial = np.maximum(np.maximum(low - local[:, 2], local[:, 2] - high), 0.0)
    return float(np.hypot(radial, axial).min())


def hands_clear() -> float:
    """m from the nearest hull corner of either hand to the smoother, the iced cake, the stand's top or the block."""
    hands = STATE.get("hands")
    if not hands:
        return 0.0
    points = np.vstack([hands[side] for side in SIDES if side in hands])
    return min(
        _box_distance(points, "smoother", (-PLATE_W / 2, -PLATE_T / 2, -EDGE_L / 2), (PLATE_W / 2, PLATE_T / 2, EDGE_L / 2)),
        _cylinder_distance(points, "cake", DUMMY_R + float(STATE["field"].max()), -BOARD_T - MAT_T, DUMMY_H),
        _cylinder_distance(points, "cake", BOARD_R, -BOARD_T - MAT_T, 0.0),
        _cylinder_distance(points, "turntable", TOP_R, -TOP_T / 2, TOP_T / 2),
        _box_distance(points, "block", *mk.MECH.bodies["block"].bounds()),
    )


def worked_columns() -> int:
    cut = STATE["cut_rows"][:, BAND.start : BAND.stop]
    return int((cut.sum(axis=1) >= len(BAND) / 2).sum())


def level() -> float:
    """The score by the icing alone, without the smoother's and the hands' places (see ``score``)."""
    field = STATE["field"]
    bare = bare_cells(field)
    if off_centre() > OFF_PLACE or stand_moved() > OFF_PLACE or bare.mean() > 1 / 3:
        return 0.0
    thick = float(thickness_readings(field).mean()) >= THIN_MEAN
    if largest_gap(field) <= EVEN and not bare.any() and thick and off_centre() <= CENTRED:
        return 1.0
    if one_flaw(field) is not None and thick and off_centre() <= CENTRED:
        return 0.75
    if ridges_under(field) >= MARKS / 2:
        return 0.5
    if worked_columns() >= WORKED_QUARTER:
        return 0.25
    return 0.0


def score(objects) -> float:
    """By the end state of the side, read at the 16 marks on the board. 1.0: a 1.2 mm feeler leaf goes in nowhere
    under the steel rule held upright at each mark (read from 9 mm over the board to 9 mm under the top) nor under the
    arc gauge centred on each mark at the three heights; no bare place; the depth pin's 16 readings 3 mm or more on
    average; the cake within 5 mm of the top's axis, the smoother in its block and both hands 3 cm clear. 0.75: the
    same but for one flaw (one mark up to 2 mm high, or one bare place under 2 cm2). 0.5: the gap under the upright
    rule under 1.5 mm at 8 or more of the 16 marks. 0.25: the edge worked at least a quarter of the way round. 0: less;
    or the cake or the stand pushed 15 mm or more off its place; or more than a third of the band bare."""
    found = level()
    if found >= 0.75 and not (in_block() and hands_clear() >= CLEAR):
        return 0.5 if ridges_under() >= MARKS / 2 else 0.25
    return found


def success(objects) -> bool:
    return score(objects) == 1.0


def report() -> dict:
    out = {"roll on the edge (mm3)": round(1e9 * STATE.get("roll", 0.0), 1), "on the blade's face (mm3)": round(1e9 * STATE.get("carried", 0.0), 1),
           "marks left": STATE.get("marks_left", 0), "scar column": STATE.get("scar"), "out of round (mm)": round(1000 * STATE.get("out_of_round", 0.0), 3)}  # fmt: skip
    if "field" in STATE:
        values = band()
        out.update({"largest gap, rule upright (mm)": round(1000 * float(upright_gaps().max()), 3), "largest gap, arc gauge (mm)": round(1000 * float(round_gaps().max()), 3),
                    "pin readings' mean (mm)": round(1000 * float(thickness_readings().mean()), 3), "mean thickness of the band (mm)": round(1000 * float(values.mean()), 3),
                    "thinnest, thickest (mm)": [round(1000 * float(values.min()), 3), round(1000 * float(values.max()), 3)],
                    "marks with the rule's gap under 1.5 mm": ridges_under(), "columns worked": worked_columns(),
                    "bare cells": int(bare_cells().sum()), "one flaw": one_flaw()})  # fmt: skip
    if mk.MECH.poses:
        out.update({"cake off the top's axis (mm)": round(1000 * off_centre(), 2), "stand moved (mm)": round(1000 * stand_moved(), 2), "in the block": in_block(),
                    "hands clear (mm)": round(1000 * hands_clear(), 1), "score": score(None)})  # fmt: skip
    return out
