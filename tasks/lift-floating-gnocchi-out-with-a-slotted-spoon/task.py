"""A small serving of gnocchi is cooking in a pan of simmering water on the front zone of a built-in hob between the
arms: twelve pieces, tipped in together on the far side of the pan as the trial starts, lie on the
pan's floor under cloudy water. Each floats up when it is done, at a time of its own inside a window after the
start (FLOAT_FROM to FLOAT_TO below), in an order drawn from the seed. A
plate stands in front of the right arm; beyond it and to its right a slotted spoon with a cranked handle lies with its
bowl on a spoon rest and its handle towards the arms. The hob's lamp is red while the heat is on; its on/off key is on
the front strip.

The water is not simulated as a fluid: each piece gets buoyancy from the part of it that is under the surface,
drag, and a slow drift. The surface is drawn as an opaque sheet: a piece on the floor cannot be seen, a floating
one stands a millimetre out of it.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, cylinder  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "The gnocchi are cooking; each is done when it floats up. Lift them out onto the plate with the slotted spoon as they are done, then put the spoon down and switch the heat off."
TOP = TABLE_TOP[SCENE]

# --- the timing: three numbers, and what follows from them ---
# The trial starts when the batch has gone into the water. FLOAT_FROM, FLOAT_TO: the seconds after the start between
# which the pieces reach the surface, each at a time of its own drawn from the seed. LATE_S: how long a piece may
# stay in the water after it has reached the surface and still count.
# None of the three is measured on this task's own bench. They are read from a published kitchen test of the named
# gnocchi: all pieces of a batch afloat 1 minute and 20 to 30 seconds after they went into boiling water, and
# pieces cooked for up to 180 s in all still judged cooked, the first stickiness from 150 s (README, "Float times:
# source and status"). A bench that builds the task cooks three batches by the setup's procedure, reads each
# piece's time off the overhead video and sets these numbers; everything below follows from them.
FLOAT_FROM, FLOAT_TO = 80.0, 90.0
LATE_S = 90.0
# What follows from them.
# A piece is done, and turns buoyant, RISE_TRAVEL_S before it is at the surface: from the pan's floor through 5.5 cm
# of water it takes 2.35 to 2.39 s (run l7ru, trials.py float), so the times drawn for the surface are put that much
# earlier as the times the pieces are done. It belongs to these pieces in this depth of water: trials.py float reads
# it again and says so if it is off.
RISE_TRAVEL_S = 2.37
# The run limit: FLOAT_TO + LATE_S is the last moment at which a piece can still leave the water and count; CLEAR_S
# more for the last spoonfuls, laying the spoon down and the key (the reference run that lets three pieces float too
# long ended 20.2 s after the last piece's time was up: run l7ref late), rounded up to ten seconds.
CLEAR_S = 40.0
CONTROL_HZ = 20  # control steps a second, as in the run command


def timing(float_from: float, float_to: float, late_s: float) -> dict:
    """What follows from the three numbers: the window in which the pieces are done (s after the start) and the run
    limit (s, control steps). They have to make a trial: the first piece cannot be at the surface before it has had
    the time to rise, the window has a width, and a floating piece is allowed some time."""
    if not RISE_TRAVEL_S <= float_from < float_to:
        raise ValueError(f"the pieces reach the surface between {float_from} and {float_to} s: need {RISE_TRAVEL_S} <= from < to")
    if not late_s > 0:
        raise ValueError(f"a floating piece may stay {late_s} s in the water: need more than 0")
    limit_s = 10.0 * math.ceil((float_to + late_s + CLEAR_S) / 10.0 - 1e-9)
    return {"done_from": float_from - RISE_TRAVEL_S, "done_to": float_to - RISE_TRAVEL_S, "run_limit_s": limit_s, "run_limit_steps": int(round(limit_s * CONTROL_HZ))}


def set_timing(float_from: float, float_to: float, late_s: float | None = None) -> dict:
    """Other numbers than the task's, for a what-if run (demo.py window=...): the three constants and the run limit
    are set together, before the trial is reset. Returns what follows from them."""
    global FLOAT_FROM, FLOAT_TO, LATE_S, RUN_LIMIT_STEPS
    late_s = LATE_S if late_s is None else late_s
    derived = timing(float_from, float_to, late_s)
    FLOAT_FROM, FLOAT_TO, LATE_S, RUN_LIMIT_STEPS = float(float_from), float(float_to), float(late_s), derived["run_limit_steps"]
    return derived


RUN_LIMIT_STEPS = timing(FLOAT_FROM, FLOAT_TO, LATE_S)["run_limit_steps"]  # 4,400: 220 s at 20 Hz

GLASS, RED, STEEL, MILK, DOUGH, WHITE, NYLON, SLOT, GREY = (
    (0.08, 0.08, 0.09), (0.95, 0.10, 0.08), (0.74, 0.76, 0.79), (0.36, 0.34, 0.29), (0.99, 0.94, 0.74),
    (0.96, 0.96, 0.94), (0.10, 0.10, 0.11), (0.45, 0.45, 0.47), (0.55, 0.57, 0.60),
)  # fmt: skip

# --- the hob: a built-in two-zone ("domino") induction hob, fixed, with the glass of the Bosch PIB375FB1E: 306 mm
# wide and 527 mm deep, zones of 21 cm and 14.5 cm (maker's product page, read 2026-11-18). It lies 6 mm proud of
# the bench; where the zones sit on the glass the page does not say. Its frame: the middle of its underside. The pan
# stands on the front zone; the on/off key and its lamp are on the front strip, towards the arms.
HOB_X, HOB_Y, HOB_H = 0.527, 0.306, 0.006
HOB_XY = (0.1275 + HOB_X / 2, 0.0)  # its front edge is 12.75 cm in front of the arms' bases
HOB_AT = (*HOB_XY, TOP)
ZONES = ((0.305 - HOB_XY[0], 0.0, 0.105), (0.520 - HOB_XY[0], 0.0, 0.0725))  # the zones' middles in the hob's frame and their radii: front 21 cm, rear 14.5 cm
KEY_XY, KEY_R = (-HOB_X / 2 + 0.018, 0.070), 0.008  # the key's middle on the hob's top, in the hob's frame
LAMP_XY, LAMP_R = (-HOB_X / 2 + 0.018, 0.045), 0.005


def _ring(x: float, y: float, radius: float, pieces: int = 32) -> list:
    """A zone's mark: a thin grey circle printed on the glass, drawn only."""
    length = 2 * radius * math.tan(math.pi / pieces)
    return [
        box((length, 0.0015, 0.0004), pos=(x + radius * math.cos(a), y + radius * math.sin(a), HOB_H + 0.0002), yaw=a + math.pi / 2, color=GREY, solid=False)
        for a in (2 * math.pi * k / pieces for k in range(pieces))
    ]


body(
    "hob",
    [
        box((HOB_X, HOB_Y, HOB_H), pos=(0.0, 0.0, HOB_H / 2)),
        cylinder(KEY_R, 0.0006, pos=(*KEY_XY, HOB_H + 0.0003), color=GREY, solid=False),  # the key: a disc printed on the glass
        *[mark for zone in ZONES for mark in _ring(*zone)],
    ],
    pos=HOB_AT,
    color=GLASS,
    friction=(0.9, 0.8),
)
# The lamp: a red tile that glows, a body of its own that the task keeps on the hob's glass while the heat is on and
# sinks under it when it is off, telling the renderer to forget its earlier frames at that step (mechkit.redraw:
# without that the lamp stayed in the cameras' pictures for seconds after it had gone, runs l7b to l7g).
LAMP_SUNK = 0.02
body("hob_lamp", [box((2 * LAMP_R, 2 * LAMP_R, 0.0006), solid=False, glow=1.0)], pos=(HOB_XY[0] + LAMP_XY[0], HOB_XY[1] + LAMP_XY[1], TOP + HOB_H + 0.0003), kinematic=True, color=RED)

# --- the pan: a 24 cm stainless saute pan, 7 cm deep inside, with 5.5 cm of water, 4.0 kg with the water (1.5 kg
# empty), standing loose on the front zone. Its frame: the middle of its underside. Its handle points away from the
# arms. (5.5 cm of water: the spoon's bowl, 15 mm deep, has to pass under a floating piece and over the pieces that
# still lie on the floor; with 3.5 cm it came down on them and could not get under: run l7i.)
PAN_XY, PAN_R, PAN_WALL, PAN_BASE, PAN_DEPTH, WATER = (HOB_XY[0] + ZONES[0][0], 0.0), 0.120, 0.002, 0.005, 0.070, 0.055
PAN_AT = (*PAN_XY, TOP + HOB_H)
PAN_SIDES = 24
SURFACE = PAN_BASE + WATER  # the water's surface, up from the pan's underside
_piece = 2 * (PAN_R + PAN_WALL) * math.tan(math.pi / PAN_SIDES)
body(
    "pan",
    [
        cylinder(PAN_R + PAN_WALL, PAN_BASE, pos=(0.0, 0.0, PAN_BASE / 2)),
        *[
            box((PAN_WALL, _piece, PAN_DEPTH), pos=((PAN_R + PAN_WALL / 2) * math.cos(a), (PAN_R + PAN_WALL / 2) * math.sin(a), PAN_BASE + PAN_DEPTH / 2), yaw=a)
            for a in (2 * math.pi * k / PAN_SIDES for k in range(PAN_SIDES))
        ],
        box((0.18, 0.025, 0.015), pos=(PAN_R + PAN_WALL + 0.09, 0.0, PAN_BASE + PAN_DEPTH - 0.012)),  # the handle
        cylinder(PAN_R - 0.0005, 0.0006, pos=(0.0, 0.0, SURFACE), color=MILK, solid=False),  # the water's surface: grey-beige with starch, drawn opaque
    ],
    pos=PAN_AT,
    mass=4.0,
    color=STEEL,
    friction=(0.5, 0.4),
)

# --- the gnocchi: 12 pieces of 26 x 18 x 15 mm and 6 g (72 g, a small serving), on the pan's floor at the start ---
PIECES = 12
PIECE = (0.026, 0.018, 0.015)
PIECE_MASS = 0.006
NAMES = [f"gnocco{k:02d}" for k in range(PIECES)]
# Where they lie at the start: on the side of the pan away from the arm that holds the spoon. The spoon's handle has
# to leave over the pan's rim towards that arm, about 12 cm from the bowl's middle, so the arm cannot take a piece
# that floats in the part of the pan nearest to it (mech/tasks/gnocchi_lift/skim.py; on a 3.5 cm grid a whole scoop
# is possible at 21 of 25 places, with 10 degrees or more to spare at the joints on the far side of a straight line
# 2.5 cm from the pan's middle). A piece's middle lies at least 15 mm inside that line and within 9 cm of the pan's
# middle; any two pieces are at least 8 mm apart; places and directions are drawn from the seed.
AWAY = (math.sqrt(0.5), math.sqrt(0.5))  # the level direction from the right arm's base to the pan's middle
REGION_R, REGION_EDGE, GAP = 0.090, -0.010, 0.008  # m: farthest from the pan's middle; least along AWAY from it; between two pieces


def in_region(x: float, y: float) -> bool:
    """A place (from the pan's middle) where a piece may lie at the start."""
    return math.hypot(x, y) <= REGION_R and x * AWAY[0] + y * AWAY[1] >= REGION_EDGE


def outline(x: float, y: float, yaw: float) -> np.ndarray:
    """The four corners of a piece lying at (x, y), turned ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[x + c * a - s * b, y + s * a + c * b] for a, b in ((PIECE[0] / 2, PIECE[1] / 2), (-PIECE[0] / 2, PIECE[1] / 2), (-PIECE[0] / 2, -PIECE[1] / 2), (PIECE[0] / 2, -PIECE[1] / 2))])


def apart(a: np.ndarray, b: np.ndarray) -> float:
    """The distance between two pieces' outlines; 0 if they overlap."""
    for poly in (a, b):  # a line along one of their sides that separates them?
        for i in range(4):
            edge = poly[(i + 1) % 4] - poly[i]
            normal = np.array([-edge[1], edge[0]])
            pa, pb = a @ normal, b @ normal
            if pa.max() < pb.min() or pb.max() < pa.min():
                break
        else:
            continue
        break
    else:
        return 0.0
    best = math.inf
    for p, q in ((a, b), (b, a)):  # the nearest pair is a corner of one and a side of the other
        for corner in p:
            for i in range(4):
                start, side = q[i], q[(i + 1) % 4] - q[i]
                along = float(np.clip((corner - start) @ side / (side @ side), 0.0, 1.0))
                best = min(best, float(np.linalg.norm(corner - (start + along * side))))
    return best


def lay(rng: np.random.Generator, pieces: int = PIECES) -> tuple[np.ndarray, np.ndarray]:
    """(places, yaws) of the pieces: one after the other a place in the region and a direction are drawn, and the
    piece is kept if it lies at least GAP from those already laid. If the region fills up before all are laid, all
    are drawn again (the same generator goes on: the result still follows from the seed alone)."""
    reach, far = PIECE[1] + GAP, math.hypot(PIECE[0], PIECE[1]) + GAP  # nearer than this between middles: too near; farther: far enough
    for _ in range(500):
        laid: list[tuple[float, float, float, np.ndarray]] = []
        for _ in range(1500):
            x, y = (float(v) for v in rng.uniform(-REGION_R, REGION_R, 2))
            yaw = float(rng.uniform(-math.pi, math.pi))
            if not in_region(x, y):
                continue
            corners, fits = None, True
            for px, py, _, other in laid:
                d = math.hypot(x - px, y - py)
                if d >= far:
                    continue
                corners = outline(x, y, yaw) if corners is None else corners
                if d < reach or apart(corners, other) < GAP:
                    fits = False
                    break
            if fits:
                laid.append((x, y, yaw, outline(x, y, yaw)))
                if len(laid) == pieces:
                    return np.array([[p[0], p[1]] for p in laid]), np.array([p[2] for p in laid])
    raise RuntimeError(f"{pieces} pieces do not fit in the region {GAP * 1000:.0f} mm apart")


GRID, _YAWS = lay(np.random.default_rng(0))  # where they are built; every reset lays them anew from its seed
for _k, (_x, _y) in enumerate(GRID):
    body(NAMES[_k], [box(PIECE)], pos=(PAN_XY[0] + _x, PAN_XY[1] + _y, PAN_AT[2] + PAN_BASE + PIECE[2] / 2 + 0.001), yaw=float(_YAWS[_k]), mass=PIECE_MASS, color=DOUGH, friction=(0.4, 0.3))

# --- the plate: 20 cm, with a low rim, 450 g; the spoon rest and the slotted spoon ---
PLATE_XY, PLATE_R, PLATE_T, PLATE_RIM = (0.325, -0.26), 0.100, 0.008, 0.006  # where the arm can tip the spoon out: its nose 5 cm nearer the arm than the plate's middle, the pieces land 6 to 8 cm on (skim.py, run l7m)
PLATE_AT = (*PLATE_XY, TOP)
body(
    "plate",
    [
        cylinder(PLATE_R, PLATE_T, pos=(0.0, 0.0, PLATE_T / 2)),
        *[
            box((0.004, 2 * PLATE_R * math.tan(math.pi / 16), PLATE_RIM), pos=((PLATE_R - 0.002) * math.cos(a), (PLATE_R - 0.002) * math.sin(a), PLATE_T + PLATE_RIM / 2), yaw=a)
            for a in (2 * math.pi * k / 16 for k in range(16))
        ],
    ],
    pos=PLATE_AT,
    mass=0.45,
    color=WHITE,
    friction=(0.6, 0.5),
)
# The spoon: a nylon slotted spoon with a cranked handle, 334.5 mm long, 60 g. Its frame: the middle of the bowl on
# the plane of its rim; x from the bowl's nose to the handle's end, z up. The bowl is a dish 90 x 69 mm and 15 mm
# deep: a flat floor 30 x 24 mm with eight flat walls sloping up to the rim (nose, rear, two sides, four corners;
# the gaps between them at the rim are under 15 mm, less than a piece's smallest side). A neck rises from the
# rim's rear end to the handle, 20 x 8 mm, which runs parallel to the rim plane 18 mm above it (its top 22 mm
# above: 37 mm in all). The slots are drawn.
BOWL_L, BOWL_W, BOWL_D, BOWL_FLOOR, SHELL = 0.090, 0.069, 0.015, (0.030, 0.024), 0.0025
SPOON_L, NECK_RUN, HANDLE_Z, HANDLE_W, HANDLE_T = 0.3345, 0.024, 0.018, 0.020, 0.008
HANDLE_FROM, HANDLE_TO = BOWL_L / 2 + NECK_RUN, SPOON_L - BOWL_L / 2  # along x from the bowl's middle
CORNER_W = 0.012  # width of a corner wall
_NECK_SLOPE = math.atan2(HANDLE_Z + SHELL, NECK_RUN)


def _wall(foot: tuple[float, float], top: tuple[float, float], width: float):
    """A flat wall of the bowl from the floor's edge at ``foot`` to the rim at ``top`` (x, y in the spoon's frame),
    ``width`` wide: the middle of its thickness runs from the floor's to the rim's."""
    run = math.hypot(top[0] - foot[0], top[1] - foot[1])
    rise = BOWL_D - SHELL
    out, slope = math.atan2(top[1] - foot[1], top[0] - foot[0]), math.atan2(rise, run)
    turn = mk.matrix_to_quat(mk.quat_to_matrix(mk.yaw_quat(out)) @ mk.axis_angle((0.0, 1.0, 0.0), -slope))  # its length points outwards and up
    return box((math.hypot(run, rise), width, SHELL), pos=((foot[0] + top[0]) / 2, (foot[1] + top[1]) / 2, -BOWL_D / 2), quat=turn)


_FX, _FY, _RX, _RY = BOWL_FLOOR[0] / 2, BOWL_FLOOR[1] / 2, BOWL_L / 2, BOWL_W / 2
_DIAGONAL = math.sqrt(0.5)  # the corner walls meet the oval rim half way round each quarter
BOWL_WALLS = [
    *[_wall((s * _FX, 0.0), (s * _RX, 0.0), BOWL_FLOOR[1]) for s in (-1, 1)],  # nose and rear
    *[_wall((0.0, s * _FY), (0.0, s * _RY), BOWL_FLOOR[0]) for s in (-1, 1)],  # sides
    *[_wall((sx * _FX, sy * _FY), (sx * _RX * _DIAGONAL, sy * _RY * _DIAGONAL), CORNER_W) for sx in (-1, 1) for sy in (-1, 1)],  # corners
]
# The rest stands beyond the plate and to its right: there the top camera sees the spoon's bowl and the first 10 cm
# or more of its handle while both arms are in their ready pose, and the right arm can take the spoon and lay it
# back. (At (0.30, -0.47), right of the plate, the right arm hid all of it in the first picture: run l7v.)
REST_XY, REST_X, REST_Y, REST_T = (0.71, -0.37), 0.11, 0.09, 0.008
# The spoon lies with its bowl on the rest and its handle's end on the bench, the handle towards the arms' side of
# the bench: 7.8 degrees nose up. (Where it comes to rest in the simulator; trials.py rest.)
SPOON_YAW, SPOON_NOSE_UP = math.pi, math.radians(7.77)
SPOON_QUAT = mk.matrix_to_quat(mk.quat_to_matrix(mk.yaw_quat(SPOON_YAW)) @ mk.axis_angle((0.0, 1.0, 0.0), SPOON_NOSE_UP))
SPOON_AT = (REST_XY[0] + 0.002 * math.cos(SPOON_YAW), REST_XY[1] + 0.002 * math.sin(SPOON_YAW), TOP + 0.02526)
body(
    "spoon",
    [
        box((BOWL_FLOOR[0], BOWL_FLOOR[1], SHELL), pos=(0.0, 0.0, -BOWL_D + SHELL / 2)),  # the bowl's floor
        *BOWL_WALLS,
        box((math.hypot(NECK_RUN, HANDLE_Z + SHELL) + 0.006, 0.014, HANDLE_T), pos=(BOWL_L / 2 + NECK_RUN / 2, 0.0, (HANDLE_Z - SHELL) / 2), quat=mk.matrix_to_quat(mk.axis_angle((0.0, 1.0, 0.0), -_NECK_SLOPE))),  # the neck
        box((HANDLE_TO - HANDLE_FROM, HANDLE_W, HANDLE_T), pos=((HANDLE_FROM + HANDLE_TO) / 2, 0.0, HANDLE_Z)),  # the handle
        *[box((0.024, 0.004, 0.0006), pos=(0.0, y, -BOWL_D + SHELL + 0.0003), color=SLOT, solid=False) for y in (-0.007, 0.0, 0.007)],  # the slots
    ],
    pos=SPOON_AT,
    quat=SPOON_QUAT,
    mass=0.060,
    color=NYLON,
    friction=(0.35, 0.25),
)
body(
    "rest",
    [
        box((REST_X, REST_Y, REST_T), pos=(0.0, 0.0, REST_T / 2)),
    ],
    pos=(*REST_XY, TOP),
    mass=0.150,
    color=GREY,
    friction=(0.8, 0.7),
)

OBJECTS = list(mk.MECH.bodies.values())

# --- the water: what it does to a piece (computed once per control step, held for the step) ---
GRAVITY, WATER_DENSITY = 9.81, 1000.0
VOLUME = PIECE[0] * PIECE[1] * PIECE[2]
OCTANTS = np.array([[sx * PIECE[0] / 4, sy * PIECE[1] / 4, sz * PIECE[2] / 4] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])  # the middles of a piece's eight parts
PART_H = PIECE[2] / 2  # the height a part is taken to have, however the piece lies
RAW, DONE = 0.78, 0.916  # share of the water's lift a piece gets: raw it sinks (it weighs 0.85 of the water it displaces); done it floats with 1 mm showing
RISE_S = 1.0  # a piece turns from raw to done over this long
DRAG, SPIN_DRAG = 0.17, 1.1e-5  # N s/m and N m s/rad on a piece wholly under water
DRIFT = 0.0003  # N: the simmer's push on a piece

# --- the rules ---
UP_TO_SURFACE = 0.002  # m: a piece whose top is within this of the surface (or above) has floated
OUT_OF_WATER = 0.003  # m its lowest part is above the surface: it has left the water
PAN_PUSHED = 0.03  # m the pan has been pushed from where it stood: the trial scores 0
# Tipping the pan is not scored: no reference run can show such a rule. A hand that pinched the pan's wall and went
# 7 cm up slid along it and left the pan level (0.0 degrees, run l7u tip_only; lifting that side takes about 20 N,
# the pinch on a 2 mm wall holds less), the pan's handle is beyond both arms' reach with the hand pointing down
# (the IK misses it by 5 to 13 cm), and a hand that pushes the pan slides it (36.7 mm, 0.07 degrees, same run).
# The pan's tilt is still read and reported.
TOUCH = 0.020  # m: a gripper's grasp point this near a piece's middle has touched it
KEY_NEAR, KEY_LOW, KEY_HIGH, KEY_S = 0.015, 0.0, 0.030, 0.2  # a fingertip on the key: grasp point within 15 mm of it, up to 30 mm over the glass, for 0.2 s
RELEASED = 0.04  # m: the grippers' distance from the spoon's handle when it has been put down
SIDES = ("left", "right")
LEVELS = ((11, 1.0), (9, 0.75), (6, 0.5), (3, 0.25))  # pieces that count -> level
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    rng = np.random.default_rng(0 if seed is None else seed)
    done_at = rng.uniform(FLOAT_FROM, FLOAT_TO, PIECES) - RISE_TRAVEL_S  # drawn: when each is at the surface
    place, yaw = lay(rng)
    STATE.clear()
    STATE.update(
        seed=seed,
        done_at=done_at,  # when each piece is done and turns buoyant
        place=place,  # where each lies at the start, from the pan's middle
        yaw=yaw,
        phase=rng.uniform(0.0, 2 * math.pi, PIECES),  # of each piece's drift
        floated=np.full(PIECES, np.nan),  # when each first reached the surface
        left=np.full(PIECES, np.nan),  # when each last left the water
        in_water=np.ones(PIECES, dtype=bool),
        left_raw=np.zeros(PIECES, dtype=bool),  # it was taken out before it had floated
        touched=np.zeros(PIECES, dtype=bool),  # a gripper has touched it
        heat=True,
        key_since=None,  # since when a fingertip has been on the key
        key_used=False,  # this touch has already switched
        zero=None,  # why the trial scores 0, once it does
        pan_from=None,  # where the pan stood at the first reading
        pan_tilt=0.0,  # the most it has been tilted (rad): reported, not scored
        told=None,
    )
    return STATE


def lamp_place() -> np.ndarray:
    """Where the lamp belongs now: on the hob's glass while the heat is on, sunk under it when it is off."""
    return np.array([HOB_XY[0] + LAMP_XY[0], HOB_XY[1] + LAMP_XY[1], TOP + HOB_H + 0.0003 - (0.0 if STATE["heat"] else LAMP_SUNK)])


def switch_heat() -> None:
    """The key has been touched: the heat goes off, or on again, and the lamp with it at this step."""
    STATE["heat"] = not STATE["heat"]
    mk.redraw()


@mk.on_reset
def start(seed) -> None:
    begin(seed)
    mk.write_pose("hob_lamp", lamp_place(), (0.0, 0.0, 0.0, 1.0))
    for k, name in enumerate(NAMES):
        x, y = STATE["place"][k]
        mk.write_pose(name, (PAN_XY[0] + x, PAN_XY[1] + y, PAN_AT[2] + PAN_BASE + PIECE[2] / 2 + 0.001), mk.yaw_quat(float(STATE["yaw"][k])))


def tilt(rotation: np.ndarray) -> float:
    return math.acos(float(np.clip(rotation[2, 2], -1.0, 1.0)))


def within(name: str, point: np.ndarray) -> np.ndarray:
    position, rotation = mk.pose(name)
    return rotation.T @ (point - position)


def lift_share(k: int, now: float) -> float:
    """The share of the water's lift piece ``k`` gets at time ``now``: RAW until it is done, DONE a second later."""
    return RAW + (DONE - RAW) * float(np.clip((now - STATE["done_at"][k]) / RISE_S, 0.0, 1.0))


def water_load(k: int, now: float) -> tuple[np.ndarray, np.ndarray, float]:
    """(force, torque about its middle, share of it under water) the water puts on piece ``k`` now: lift on each
    of its eight parts by how much of that part is under the surface, drag against its motion, and the simmer's
    drift. Zero outside the pan."""
    position, rotation = mk.pose(NAMES[k])
    velocity, spin = mk.velocity(NAMES[k])
    pan_p, pan_r = mk.pose("pan")
    middle = pan_r.T @ (position - pan_p)
    if math.hypot(middle[0], middle[1]) > PAN_R or middle[2] > PAN_BASE + PAN_DEPTH + PIECE[0]:
        return np.zeros(3), np.zeros(3), 0.0
    arms = OCTANTS @ rotation.T  # the parts' middles from the piece's, in the world
    heights = (arms + position - pan_p) @ pan_r[:, 2]  # up from the pan's underside
    under = np.clip((SURFACE - (heights - PART_H / 2)) / PART_H, 0.0, 1.0)  # how much of each part is under water
    wet = float(under.mean())
    if wet == 0.0:
        return np.zeros(3), np.zeros(3), 0.0
    lifts = lift_share(k, now) * WATER_DENSITY * GRAVITY * VOLUME / len(OCTANTS) * under
    up = pan_r[:, 2]
    force = lifts.sum() * up - DRAG * wet * velocity
    torque = np.cross(arms, np.outer(lifts, up)).sum(axis=0) - SPIN_DRAG * wet * spin
    radial = np.array([middle[0], middle[1], 0.0]) / max(math.hypot(middle[0], middle[1]), 1e-6)
    swirl = np.array([-radial[1], radial[0], 0.0])
    pull = DRIFT * (math.sin(0.35 * now + STATE["phase"][k]) * swirl + 0.5 * math.sin(0.23 * now + 2.0 * STATE["phase"][k]) * radial)
    return force + wet * (pan_r @ pull), torque, wet


@mk.on_step
def step() -> None:
    """Once per control step: the water's load on each piece; which pieces have floated, left the water, been
    touched; the pan; the heat key."""
    now = mk.now()
    pan_p, pan_r = mk.pose("pan")
    if STATE["pan_from"] is None:
        STATE["pan_from"] = pan_p.copy()
    STATE["pan_tilt"] = max(STATE["pan_tilt"], tilt(pan_r))
    if STATE["zero"] is None and float(np.linalg.norm((pan_p - STATE["pan_from"])[:2])) > PAN_PUSHED:
        STATE["zero"] = "the pan was pushed"
    grasps = [mk.gripper(side)[0] for side in SIDES]
    for k, name in enumerate(NAMES):
        force, torque, wet = water_load(k, now)
        if mk.MECH.environment is not None:
            mk.push(name, force, torque)
        position, rotation = mk.pose(name)
        heights = (OCTANTS @ rotation.T + position - pan_p) @ pan_r[:, 2]
        top, low = float(heights.max()) + PART_H / 2, float(heights.min()) - PART_H / 2
        over_pan = math.hypot(*(pan_r.T @ (position - pan_p))[:2]) <= PAN_R
        if np.isnan(STATE["floated"][k]) and over_pan and wet > 0 and now >= STATE["done_at"][k] and top >= SURFACE - UP_TO_SURFACE:
            STATE["floated"][k] = now
        in_water = over_pan and low < SURFACE + OUT_OF_WATER
        if STATE["in_water"][k] and not in_water:  # it has just left the water
            STATE["left"][k] = now
            STATE["left_raw"][k] = bool(np.isnan(STATE["floated"][k]))
        STATE["in_water"][k] = in_water
        if any(float(np.linalg.norm(grasp - position)) < TOUCH for grasp in grasps):
            STATE["touched"][k] = True

    key = np.array([HOB_XY[0] + KEY_XY[0], HOB_XY[1] + KEY_XY[1], TOP + HOB_H])  # the hob is fixed
    on_key = False
    for grasp in grasps:
        local = grasp - key
        on_key = on_key or (math.hypot(local[0], local[1]) <= KEY_NEAR and KEY_LOW <= local[2] <= KEY_HIGH)
    if not on_key:
        STATE["key_since"], STATE["key_used"] = None, False
    else:
        if STATE["key_since"] is None:
            STATE["key_since"] = now
        if not STATE["key_used"] and now - STATE["key_since"] >= KEY_S - 1e-9:  # one touch switches once
            STATE["key_used"] = True
            switch_heat()
    if mk.MECH.environment is not None:
        mk.write_pose("hob_lamp", lamp_place(), (0.0, 0.0, 0.0, 1.0))

    told = progress()
    if told != STATE["told"]:
        STATE["told"] = told
        print(f"[gnocchi] {now:7.2f} s: {told}", flush=True)


def on_plate(k: int) -> bool:
    """Piece ``k`` lies on the plate, at rest."""
    local = within("plate", mk.pose(NAMES[k])[0])
    return math.hypot(local[0], local[1]) <= PLATE_R and 0.0 < local[2] < PLATE_T + 0.04 and float(np.linalg.norm(mk.velocity(NAMES[k])[0])) < 0.02


def good() -> np.ndarray:
    """Which pieces count: left the water after floating and no more than LATE_S later, lie on the plate at rest,
    never touched by a gripper."""
    in_time = ~np.isnan(STATE["floated"]) & ~np.isnan(STATE["left"]) & ~STATE["left_raw"] & (STATE["left"] - STATE["floated"] <= LATE_S)
    return in_time & ~STATE["touched"] & np.array([on_plate(k) for k in range(PIECES)])


def raw_served() -> bool:
    """A piece that never floated lies on the plate."""
    return any(np.isnan(STATE["floated"][k]) and on_plate(k) for k in range(PIECES))


def spoon_handle() -> tuple[np.ndarray, np.ndarray]:
    """The two ends of the spoon's handle in the world."""
    position, rotation = mk.pose("spoon")
    return position + rotation @ np.array([HANDLE_FROM, 0.0, HANDLE_Z]), position + rotation @ np.array([HANDLE_TO, 0.0, HANDLE_Z])


def point_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-18), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def spoon_down() -> bool:
    """The spoon has been put down: at rest, no gripper within 4 cm of its handle."""
    ends = spoon_handle()
    resting = float(np.linalg.norm(mk.velocity("spoon")[0])) < 0.02
    return resting and all(point_segment(mk.gripper(side)[0], *ends) >= RELEASED for side in SIDES)


def score(objects) -> float:
    """0 if the pan was pushed, or a piece that never floated lies on the plate. Else by the pieces
    that count: 3 a quarter, 6 a half, 9 three quarters, 11 with the heat off and the spoon put down 1.0. Never
    above a half while the heat is on or the spoon is in a gripper."""
    if STATE["zero"] is not None or raw_served():
        return 0.0
    count = int(good().sum())
    tidy = not STATE["heat"] and spoon_down()
    for needed, level in LEVELS:
        if count >= needed:
            if level == 1.0 and not tidy:
                continue
            return level if tidy else min(level, 0.5)
    return 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


def progress() -> str:
    """How far the trial has got, in one line for the run's log."""
    floated, out = int((~np.isnan(STATE["floated"])).sum()), int((~STATE["in_water"]).sum())
    flags = [text for text, on in (
        (f"ZERO: {STATE['zero']}", STATE["zero"] is not None), ("RAW PIECE ON THE PLATE", raw_served()), ("heat off", not STATE["heat"]), ("spoon put down", spoon_down()),
        (f"{int(STATE['touched'].sum())} touched by a gripper", bool(STATE["touched"].any())), (f"{int(STATE['left_raw'].sum())} taken out raw", bool(STATE["left_raw"].any())),
    ) if on]  # fmt: skip
    return f"{floated} of {PIECES} floated, {out} out of the water, {int(good().sum())} count; level {score(None):.2f}" + "".join(f"; {flag}" for flag in flags)


def report() -> dict:
    """For logs and probes: the numbers behind the score."""
    return {
        "done at (s)": [round(float(v), 1) for v in STATE["done_at"]],
        "floated at (s)": [None if np.isnan(v) else round(float(v), 2) for v in STATE["floated"]],
        "left the water at (s)": [None if np.isnan(v) else round(float(v), 2) for v in STATE["left"]],
        "taken out raw": [int(k) for k in np.flatnonzero(STATE["left_raw"])],
        "touched by a gripper": [int(k) for k in np.flatnonzero(STATE["touched"])],
        "heat on": bool(STATE["heat"]),
        "zero": STATE["zero"],
        "pan tilted at most (deg)": round(math.degrees(STATE["pan_tilt"]), 2),
        **({"pieces that count": int(good().sum()), "spoon put down": spoon_down(), "score": score(None)} if mk.MECH.poses and mk.MECH.grippers else {}),
    }
