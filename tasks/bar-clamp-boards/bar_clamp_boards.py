"""On the station table, on a dark rubber work mat (40 x 46 cm, 3 mm): two pine boards (20 x 10 x 1.8 cm, 180 g each)
lie stacked face to face, edges aligned, on two fixed walnut spacers (4 x 10 x 3 cm) under their ends, so the
middle of the stack overhangs a 3 cm gap. The stack lies along x at (0.36, -0.12). A red mini bar clamp (130 g)
stands upright on its foot to the stack's left, its jaws pointing at the stack's long edge: a steel bar 11 cm tall, a
red fixed jaw at its foot (10.6 cm long, 2 cm thick, its face 1 cm below the stack's underside) with a red and black
handle (10 cm) behind the bar, and a red sliding jaw (30 g) on the bar, its face 6 cm above the fixed jaw's. The
sliding jaw rides a one-way ratchet: pushed down toward the fixed jaw (about 0.6 N for each 0.3 mm click), it holds
wherever it is put and never slides back. A rubber mallet and a steel speed square lie at the far corners, fixed.

The task: put the bar clamp over the two stacked boards and push its sliding jaw down until both jaws grip the stack,
then let go. The clamp must hang on the boards by itself.

The left arm holds the clamp by its handle, slides its fixed jaw under the overhang and lifts it against the bottom
board. The right arm pushes the sliding jaw down onto the top board. Each arm works in its own half of the table.
``bonds.py`` makes the clamp (one rigid body of four boxes) and the ratchet, ``fingers.py`` reads where the fingers
are (to know the clamp was let go), ``simready.py`` places the mallet and the square, and ``looks.py`` dresses the
boxes in the Blender meshes of ``looks/``.

Run it:  inspect-robots run --embodiment yam_arena -E task=bar_clamp_boards.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402  (before bonds: its hook on task_object then runs after bonds')
from bonds import Compound, Ratchet, on_reset, on_step, set_upper, upper  # noqa: E402
from looks import dress  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0  # the clamp hangs on the boards, let go, for 1 s

# -- the mat ---------------------------------------------------------------------------------------------------------
MAT_SIZE = (0.40, 0.46, 0.003)
MAT_XY = (0.33, -0.02)
BASE = TOP + MAT_SIZE[2]  # the mat's top: everything stands on it
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.13, 0.14, 0.15))

# -- the stack: two boards on two spacers ----------------------------------------------------------------------------
STACK_XY = (0.36, -0.12)
BOARD_SIZE = (0.20, 0.10, 0.018)  # long (x), wide (y), thick
BOARD_MASS = 0.18
SPACER_SIZE = (0.04, 0.10, 0.03)
SPACER_DX = 0.075  # m from the stack's centre: under the boards' ends, 9 cm clear between them
DROP = 0.0005  # m: everything free spawns this far above what it rests on
WALNUT = (0.30, 0.19, 0.11)
SPACERS = [
    Box(f"spacer_{side}", pos=(STACK_XY[0] + s * SPACER_DX, STACK_XY[1], BASE + SPACER_SIZE[2] / 2),
        size=SPACER_SIZE, color=WALNUT)
    for side, s in (("near", -1), ("far", 1))
]
UNDER = BASE + SPACER_SIZE[2]  # the bottom board's underside
PINE = (0.86, 0.70, 0.46)
BOARD_LOW = Box("board_bottom", pos=(*STACK_XY, UNDER + DROP + BOARD_SIZE[2] / 2), size=BOARD_SIZE,
                mass=BOARD_MASS, color=PINE)
BOARD_UP = Box("board_top", pos=(*STACK_XY, UNDER + 2 * DROP + 1.5 * BOARD_SIZE[2]), size=BOARD_SIZE,
               mass=BOARD_MASS, color=PINE)
STACK_TOP = UNDER + 2 * BOARD_SIZE[2]
STACK_T = 2 * BOARD_SIZE[2]  # 3.6 cm

# -- the clamp -------------------------------------------------------------------------------------------------------
# Drawn from its origin O: the bar's axis at the height of the fixed jaw's face. The jaws point along -y (at the
# stack's long edge), the handle along +y. Parts: (name, centre from O, size).
BAR = ("bar", (0.0, 0.0, 0.045), (0.008, 0.020, 0.090))
FIXED_JAW = ("fixed_jaw", (0.0, -0.043, -0.010), (0.024, 0.106, 0.020))
SLICK = (0.35, 0.3)  # the steel jaw and the handle's nylon foot, on the mat, the boards and the pads
GRIP = ("handle", (0.0, 0.060, -0.001), (0.030, 0.100, 0.022))  # the handle's rubber grip, from 8 mm up
FOOT = ("foot", (0.0, 0.060, -0.016), (0.030, 0.100, 0.008), SLICK)  # under the grip, on the mat
BAR, FIXED_JAW = (*BAR, SLICK), (*FIXED_JAW, SLICK)
PARTS = (BAR, FIXED_JAW, GRIP, FOOT)
CLAMP_LOW = (-0.015, -0.096, -0.020)  # the clamp's box from O: x, y, z
CLAMP_HIGH = (0.015, 0.110, 0.090)
O_TO_CENTRE = tuple((a + b) / 2 for a, b in zip(CLAMP_LOW, CLAMP_HIGH))  # the box's centre from O
CLAMP_SIZE = tuple(b - a for a, b in zip(CLAMP_LOW, CLAMP_HIGH))
CLAMP_MASS = 0.13  # kg: a mini one-hand bar clamp, steel bar, nylon jaws and handle
CLAMP_COM = (0.0, 0.008, 0.015)  # from O: in the bar, just behind its axis (the steel bar and the pawl's spring)
EDGE_GAP = 0.008  # m: the bar's face this far from the boards' long edge, clamped
BAR_Y_CLAMPED = STACK_XY[1] + BOARD_SIZE[1] / 2 + EDGE_GAP + BAR[2][1] / 2  # -0.032
SLIDE = 0.10  # m: the clamp starts this far to the left of where it clamps, its jaw tips 2.2 cm off the edge
O_START = (STACK_XY[0], BAR_Y_CLAMPED + SLIDE, BASE + DROP + 0.020)
JAW_OPEN = 0.060  # m: the sliding jaw's face starts this far above the fixed jaw's face
JAW_LOW = (-0.016, -0.096, JAW_OPEN)  # the sliding jaw's box from O (its collar rides round the bar)
JAW_HIGH = (0.016, 0.015, JAW_OPEN + 0.020)
O_TO_JAW = tuple((a + b) / 2 for a, b in zip(JAW_LOW, JAW_HIGH))
JAW_SIZE = tuple(b - a for a, b in zip(JAW_LOW, JAW_HIGH))
PAD_Y = -0.060  # the jaw pads' centres from the bar's axis, 4.2 cm in from the boards' edge when clamped
PAD_ENDS = (-0.090, -0.040)  # the pads' two ends (y from the bar's axis): a pad touches where it comes closest
JAW_HOLD = 2000.0  # N/m: the ratchet's spring: the jaw sags 0.15 mm under its weight, 0.6 N pushes it a click on


def _from_o(p) -> tuple[float, float, float]:
    """A point given from the clamp's O, in the clamp box's own frame (from its centre)."""
    return tuple(a - c for a, c in zip(p, O_TO_CENTRE))


RED = (0.80, 0.08, 0.06)
CLAMP = Compound(
    "clamp",
    pos=tuple(o + c for o, c in zip(O_START, O_TO_CENTRE)),
    size=CLAMP_SIZE,
    mass=CLAMP_MASS,
    color=RED,
    parts=tuple((name, _from_o(c), s, *f) for name, c, s, *f in PARTS),
    com=_from_o(CLAMP_COM),
)
JAW = Ratchet(
    "sliding_jaw",
    pos=tuple(o + c for o, c in zip(O_START, O_TO_JAW)),
    size=JAW_SIZE,
    mass=0.03,
    color=RED,
    to="clamp",
    axis="Z",
    lower=-(JAW_OPEN - 0.002),
    upper=0.0,
    hold=JAW_HOLD,
    collide=False,
)

# -- the workbench props: fixed, out of the way ----------------------------------------------------------------------
MALLET = SimReady("mallet", pos=(0.585, 0.33, TOP), size=(0.394, 0.131, 0.071), yaw=math.pi / 2,
                  package="handTool_hammer_rubberMallet_a01", usd="sm_handTool_hammer_rubberMallet_a01_01.usd")
SQUARE = SimReady("speed_square", pos=(0.58, -0.40, TOP), size=(0.29, 0.218, 0.008), yaw=0.35,
                  package="measurement_ruler_speedsquare_a02", usd="sm_measurement_ruler_speedsquare_a02_01.usd")

OBJECTS = [MAT, *SPACERS, BOARD_LOW, BOARD_UP, CLAMP, JAW, MALLET, SQUARE]

LOOKS = {}
if (pathlib.Path(__file__).resolve().parent / "looks" / "clamp.usdc").is_file():
    LOOKS = {"mat": "mat.usdc", "board_bottom": "board_bottom.usdc", "board_top": "board_top.usdc",
             "spacer_near": "spacer.usdc", "spacer_far": "spacer.usdc", "clamp": "clamp.usdc",
             "sliding_jaw": "jaw.usdc"}
    dress(LOOKS)

# -- geometry of the state -------------------------------------------------------------------------------------------
GAP_CLOSED = 0.001  # m: a jaw this close to its board grips it
GAP_FLOOR = -0.003  # m: a jaw deeper in its board than this is nonsense (a numerical blow-up)
ON_BOARD = 0.01  # m: a pad centre at least this far inside a board's outline is on it
ALIGNED = 0.003  # m: the boards' corners within this of each other
MIDDLE = BOARD_SIZE[0] / 6  # m: the middle third of the boards' length, either side of their centre
CLEAR = 0.01  # m: every finger at least this far from the clamp: let go
HANG_CLEAR = 0.001  # m: the clamp's lowest point this far above the mat: it hangs on the boards
OFF_TABLE_Z = -0.02  # m: an object whose centre is lower than this from the table top fell off

# What the episode remembers: whether every finger was clear of the clamp at the last step (from on_step).
STATE = {"clear": True}


@on_reset
def reset_memory(env=None) -> None:
    STATE["clear"] = True


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) and np.all(np.isfinite(s.lin_vel))
               for s in objects.values())


def _clamp_point(objects, p_from_o) -> np.ndarray:
    """A point of the clamp, given from its O, in the world."""
    c = objects["clamp"]
    return np.asarray(c.pos) + _rot(c.quat_xyzw) @ np.asarray(_from_o(p_from_o))


def fixed_pad(objects, y=PAD_Y) -> np.ndarray:
    """A point on the fixed jaw's face (its pad's centre by default), ``y`` from the bar's axis."""
    return _clamp_point(objects, (0.0, y, 0.0))


def sliding_pad(objects, y=PAD_Y) -> np.ndarray:
    """A point on the sliding jaw's face, its underside (its pad's centre by default), ``y`` from the bar's axis."""
    j = objects["sliding_jaw"]
    return np.asarray(j.pos) + _rot(j.quat_xyzw) @ np.array([0.0, y - O_TO_JAW[1], -JAW_SIZE[2] / 2])


def _local(objects, board: str, p) -> np.ndarray:
    b = objects[board]
    return _rot(b.quat_xyzw).T @ (np.asarray(p) - np.asarray(b.pos))


def on_board(objects, board: str, p) -> bool:
    """The point lies over the board's face, at least 1 cm inside its outline."""
    x, y, _ = _local(objects, board, p)
    return bool(abs(x) <= BOARD_SIZE[0] / 2 - ON_BOARD and abs(y) <= BOARD_SIZE[1] / 2 - ON_BOARD)


def gaps(objects) -> tuple[float, float]:
    """(fixed, sliding): how close each jaw's pad comes to the face of the board it should grip (m, along the board's
    normal, the least over the pad's two ends and centre; negative inside it). The fixed jaw under the bottom board,
    the sliding jaw on the top board. A pad tilted with its clamp touches at one end, as a feeler gauge finds it."""
    ys = (PAD_ENDS[0], PAD_Y, PAD_ENDS[1])
    fixed = min(-BOARD_SIZE[2] / 2 - _local(objects, "board_bottom", fixed_pad(objects, y))[2] for y in ys)
    sliding = min(_local(objects, "board_top", sliding_pad(objects, y))[2] - BOARD_SIZE[2] / 2 for y in ys)
    return float(fixed), float(sliding)


def straddles(objects) -> bool:
    """The clamp is round the stack: the fixed jaw's face under the bottom board and the sliding jaw's face over the
    top board, each pad centre at least 1 cm inside the boards' outline."""
    fixed, sliding = gaps(objects)
    return bool(on_board(objects, "board_bottom", fixed_pad(objects))
                and on_board(objects, "board_top", sliding_pad(objects))
                and fixed >= GAP_FLOOR and sliding >= GAP_FLOOR)


def clamped(objects) -> bool:
    """Round the stack with both jaws on their boards: both gaps under 1 mm."""
    fixed, sliding = gaps(objects)
    return bool(straddles(objects) and fixed < GAP_CLOSED and sliding < GAP_CLOSED)


def hanging(objects) -> bool:
    """No part of the clamp is lower than 1 mm above the mat: it hangs on the boards by its jaws, not standing."""
    c = objects["clamp"]
    half = np.asarray(CLAMP_SIZE) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    low = float(np.min((np.asarray(c.pos) + corners @ _rot(c.quat_xyzw).T)[:, 2]))
    return low >= BASE + HANG_CLEAR


def in_middle(objects) -> bool:
    """The jaws' centre is in the middle third of the boards' length (within 3.3 cm of their centre)."""
    return bool(abs(_local(objects, "board_top", fixed_pad(objects))[0]) <= MIDDLE
                and abs(_local(objects, "board_bottom", fixed_pad(objects))[0]) <= MIDDLE)


def _corners(objects, board: str) -> np.ndarray:
    b = objects[board]
    hx, hy = BOARD_SIZE[0] / 2, BOARD_SIZE[1] / 2
    pts = np.array([[sx * hx, sy * hy, 0.0] for sx in (-1, 1) for sy in (-1, 1)])
    return np.asarray(b.pos) + pts @ _rot(b.quat_xyzw).T


def misalignment(objects) -> float:
    """How far apart (m, across the table) the two boards' matching corners are, at worst."""
    d = _corners(objects, "board_top")[:, :2] - _corners(objects, "board_bottom")[:, :2]
    return float(np.max(np.linalg.norm(d, axis=1)))


def aligned(objects) -> bool:
    return misalignment(objects) < ALIGNED


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return any(objects[n].pos[2] < TOP + OFF_TABLE_Z for n in ("board_bottom", "board_top", "clamp", "sliding_jaw"))


# -- the ratchet, and the fingers ------------------------------------------------------------------------------------
TOOTH = 0.0003  # m: the jaw has been pushed this far past its stop toward the fixed jaw, so the stop follows it


def jaw_travel(objects) -> float:
    """How far the sliding jaw is from where it started along the bar (m, negative toward the fixed jaw)."""
    c, j = objects["clamp"], objects["sliding_jaw"]
    return float((_rot(c.quat_xyzw).T @ (np.asarray(j.pos) - np.asarray(c.pos)))[2] - (O_TO_JAW[2] - O_TO_CENTRE[2]))


def _clamp_boxes():
    """Every clamp box as (centre from O, size): the bar, the fixed jaw, the handle's grip and foot."""
    return [(np.asarray(c), np.asarray(s)) for _, c, s, *_ in PARTS]


def finger_distance(objects, fingers) -> float:
    """The closest any finger point comes to the clamp or its sliding jaw (m)."""
    if fingers is None or len(fingers) == 0:
        return math.inf
    pts = np.asarray(fingers, float)
    best = math.inf
    c, j = objects["clamp"], objects["sliding_jaw"]
    rc = _rot(c.quat_xyzw)
    o = np.asarray(c.pos) - rc @ np.asarray(O_TO_CENTRE)
    local = (pts - o) @ rc
    for centre, size in _clamp_boxes():
        best = min(best, float(np.min(np.linalg.norm(np.maximum(np.abs(local - centre) - size / 2, 0.0), axis=1))))
    jl = (pts - np.asarray(j.pos)) @ _rot(j.quat_xyzw)
    best = min(best, float(np.min(np.linalg.norm(np.maximum(np.abs(jl) - np.asarray(JAW_SIZE) / 2, 0.0), axis=1))))
    return best


@on_step
def ratchet_and_fingers(env) -> None:
    """Every control step: the ratchet's stop follows the sliding jaw toward the fixed jaw (never back), and the
    fingers are checked against the clamp."""
    from fingers import finger_points
    from yam_arena.scene import object_states

    objects = object_states(env, ["clamp", "sliding_jaw"])
    if not _finite(objects):
        return
    travel = jaw_travel(objects)
    if travel < upper("sliding_jaw") - TOOTH:
        set_upper("sliding_jaw", travel)
    STATE["clear"] = finger_distance(objects, finger_points(env)) >= CLEAR


# -- scoring ---------------------------------------------------------------------------------------------------------


def success(objects) -> bool:
    """The clamp grips the stack in its middle third, the boards still aligned, let go, hanging off the mat and
    still."""
    if _violation(objects):
        return False
    return bool(clamped(objects) and in_middle(objects) and aligned(objects) and STATE["clear"] and hanging(objects)
                and objects["clamp"].resting())


def score(objects) -> float:
    """1.0: success. 0.5: both jaws grip the stack (gaps under 1 mm), but off the middle third, with the boards
    misaligned by 3 mm or more, still held, standing on the mat or still moving. 0.25: the clamp is round the stack (fixed jaw under it, sliding jaw over
    it, both 1 cm or more inside the boards' outline) but a jaw is 1 mm or more off its board. 0 otherwise, and 0 if a
    board or the clamp fell off the table."""
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    if clamped(objects):
        return 0.5
    return 0.25 if straddles(objects) else 0.0


def distance(objects) -> float:
    """Progress: the larger jaw gap, or how far the fixed jaw's pad is from under the stack."""
    if not _finite(objects):
        return math.inf
    pad = fixed_pad(objects)
    target = np.array([STACK_XY[0], BAR_Y_CLAMPED + PAD_Y, UNDER])
    return float(np.linalg.norm(pad - target) + max(0.0, max(gaps(objects))))


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Left arm, straight-down grip across the handle (fingers closing along x, turned -90 degrees, the way its wrist
# reaches, its camera facing away from the right arm): grip the handle 9 cm behind the bar, slide the clamp 10 cm to
# the right along the mat (its fixed jaw goes under the stack's overhang, 1 cm below the bottom board), lift it
# 1.8 cm (1 cm to the board, the rest so the arm leans the fixed jaw on it) and hold it there. Right arm, fingers
# closed, turned +90 degrees: from beside the clamp, push the sliding jaw down 2.4 cm onto the top board and 1.2 cm
# past it (the arm is position controlled and needs the error to push), and hold it there while the left arm lifts
# 1.2 cm more (the ratchet takes up the slack: the jaws squeeze the stack), lets go and goes up. Then the right arm
# goes up. Numbers are TCP positions.
GRIP_DX = -0.006  # m: at this low reach the left arm's fingers close 6 mm forward of their target (measured in
# the simulator): aimed this much nearer, they close on the handle without pushing the clamp aside
GRIP_Y = 0.090  # the left arm's grip on the handle, from the bar's axis, 2 cm from its end
GRIP_TIP_Z = BASE + 0.010  # the pad tips 1 cm above the mat, 2 cm down the grip's sides
GRIP_Z = GRIP_TIP_Z + PAD_TIP_PAST_TCP
LIFT = 0.010 + 0.008
SQUEEZE = 0.012  # m: the left arm lifts this much more once the sliding jaw is down, squeezing the stack
HOVER_Z = BASE + 0.13
PUSH_FROM_BAR = -0.086  # the right arm pushes the jaw 1 cm from its tip, over the boards, away from the left arm
PUSH_Y = BAR_Y_CLAMPED + PUSH_FROM_BAR
PUSH_LEAN = 0.012
PUSH_Z = STACK_TOP + JAW_SIZE[2] - PUSH_LEAN + PAD_TIP_PAST_TCP
PUSH = 1.0  # 0 (an ablation): the right arm comes down only to the sliding jaw's top, lifted, without pushing
LEFT_YAW, RIGHT_YAW = -math.pi / 2, math.pi / 2


def demo():
    w = Waypoint
    x = STACK_XY[0]
    gx = x + GRIP_DX
    y0, y1 = O_START[1] + GRIP_Y, BAR_Y_CLAMPED + GRIP_Y
    jaw_start_top = O_START[2] + JAW_HIGH[2] + PAD_TIP_PAST_TCP
    push_z = jaw_start_top + LIFT + PUSH * (PUSH_Z - jaw_start_top - LIFT)
    return {
        "left": [
            w(1.5, (gx, y0, HOVER_Z), FINGER_OPEN, "over the handle", LEFT_YAW),
            w(1.5, (gx, y0, GRIP_Z), FINGER_OPEN, "down round the handle", LEFT_YAW),
            w(0.8, (gx, y0, GRIP_Z), FINGER_CLOSED, "grip", LEFT_YAW),
            w(3.0, (gx, y1, GRIP_Z), FINGER_CLOSED, "slide the fixed jaw under the stack", LEFT_YAW),
            w(1.2, (gx, y1, GRIP_Z + LIFT), FINGER_CLOSED, "lift against the bottom board", LEFT_YAW),
            w(2.5, (gx, y1, GRIP_Z + LIFT), FINGER_CLOSED, "hold while the jaw is pushed", LEFT_YAW),
            w(0.8, (gx, y1, GRIP_Z + LIFT + SQUEEZE), FINGER_CLOSED, "squeeze the stack", LEFT_YAW),
            w(0.5, (gx, y1, GRIP_Z + LIFT + SQUEEZE), FINGER_CLOSED, "hold", LEFT_YAW),
            w(0.6, (gx, y1, GRIP_Z + LIFT + SQUEEZE), FINGER_OPEN, "let go", LEFT_YAW),
            w(0.8, (x, y1 + 0.06, GRIP_Z + LIFT + SQUEEZE), FINGER_OPEN, "off the handle's end", LEFT_YAW),
            w(1.2, (gx - 0.04, y1 + 0.12, HOVER_Z), FINGER_OPEN, "back to its side", LEFT_YAW),
        ],
        "right": [
            w(2.5, (x, PUSH_Y - 0.03, jaw_start_top + LIFT + 0.03), FINGER_CLOSED, "beside the sliding jaw", RIGHT_YAW),
            w(5.5, (x, PUSH_Y - 0.03, jaw_start_top + LIFT + 0.03), FINGER_CLOSED, "wait for the clamp", RIGHT_YAW),
            w(0.8, (x, PUSH_Y, jaw_start_top + LIFT + 0.01), FINGER_CLOSED, "down to the jaw", RIGHT_YAW),
            w(1.5, (x, PUSH_Y, push_z), FINGER_CLOSED, "push the jaw down", RIGHT_YAW),
            w(3.3, (x, PUSH_Y, push_z), FINGER_CLOSED, "hold while the left arm lets go", RIGHT_YAW),
            w(0.8, (x, PUSH_Y, push_z + 0.04), FINGER_CLOSED, "up", RIGHT_YAW),
            w(1.2, (x - 0.04, PUSH_Y - 0.08, HOVER_Z), FINGER_CLOSED, "back to its side", RIGHT_YAW),
            w(2.0, (x - 0.04, PUSH_Y - 0.08, HOVER_Z), FINGER_CLOSED, "wait: the clamp hangs on the boards", RIGHT_YAW),
        ],
    }
