"""On the station table, on a dark rubber work mat (46 x 40 cm, 3 mm): two pine boards (20 x 6 x 1.8 cm, 108 g each)
lie flat side by side, their long edges 2 mm apart, ready to be glued edge to edge into a 12.2 cm wide panel. They lie
along x at (0.429, -0.059). A red one-hand bar clamp (a 6 in mini clamp, 0.32 kg) lies flat on the mat just in front
of the panel, its bar across the panel (along y), its jaws open 14 cm and pointing away from the robot at the panel's
near end, its pistol grip and trigger pointing at the robot. The clamp is a handle (a housing round the bar with
the fixed jaw on its front and the pistol grip below), a trigger hinged in the housing in front of the grip, and a
steel bar that slides through the housing with the moving jaw on its far end. Squeezing the trigger toward the grip
drives the bar through the housing by up to 3.5 mm, and a brake holds it there; let go, the trigger springs back
and the next squeeze drives the bar on. The fixed jaw's black rubber pad is a spring (40 N per mm, 3 mm of travel)
that stands for the jaws' and the bar's flex and the pads' rubber: its compression is the clamping force. A rubber
mallet and a steel speed square lie at the far corners, fixed.

The task: clamp the two boards together, edge to edge, with the bar clamp: its jaws on the panel's outer edges, the
joint between the boards closed, and a clamping force of 20 N or more, then let go of the clamp.

The scripted demo: the right arm grips the clamp's pistol grip and trigger together, slides the clamp so its jaws go
round the panel's near end, and pumps the trigger, stroke by stroke, until the jaws press the boards together; the left
arm holds its wrist camera over the panel's near end and touches nothing. ``bonds.py`` makes the
clamp's bodies and joints (the handle, the hinged trigger, the sliding bar and the pad's spring) and the trigger's
one-way drive on the bar, ``fingers.py`` reads where the fingers are (to know the clamp and the boards were let go),
``simready.py`` places the mallet and the square, and ``looks.py`` dresses the mat and the boards in the Blender
meshes of ``looks/``.

Run it:  inspect-robots run --embodiment yam_arena -E task=bar_clamp_boards.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402  (before bonds: its hook on task_object then runs after bonds')
from bonds import (  # noqa: E402
    Compound, Hinge, Ratchet, Slide, low, on_reset, on_spawn, on_step, one_way, set_low, set_upper, upper,
)
from looks import dress  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every full-success condition held for HOLD_S: the run ends when it holds
HOLD_S = 1.0  # s: every full-success condition held without a break this long
TAG = "[bar_clamp]"
DEBUG = False  # more log lines: where the clamp and the boards are

# -- the mat ---------------------------------------------------------------------------------------------------------
MAT_SIZE = (0.40, 0.46, 0.003)  # turned a quarter turn: 46 cm along x, 40 cm along y
MAT_XY = (0.34, -0.02)
BASE = TOP + MAT_SIZE[2]  # the mat's top: everything lies on it
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, yaw=math.pi / 2, color=(0.13, 0.14, 0.15))
DROP = 0.0005  # m: everything free spawns this far above the mat

# -- where the clamp clamps: its origin O -----------------------------------------------------------------------------
# O is the centre of the fixed jaw's pad face, at rest, half way up the clamp lying flat. Everything of the clamp is
# drawn from O, in the world's axes (the clamp lies with its bar along y, its jaws toward +x). The clamp is 24 mm
# thick lying flat, so O is 12 mm above the mat.
CLAMP_T = 0.024
O_CLAMPED = (0.36, -0.12, BASE + CLAMP_T / 2)

# -- the boards ------------------------------------------------------------------------------------------------------
BOARD_SIZE = (0.20, 0.060, 0.018)  # long (x), wide (y), thick
BOARD_MASS = 0.108  # pine, 500 kg/m3
JOINT_GAP = 0.002  # m: the boards' long edges start this far apart
BAR_CLEAR = 0.004  # m: the clamped bar's face this far in front of the boards' near ends
PINE = (0.86, 0.70, 0.46)
BOARD_X = O_CLAMPED[0] - 0.035 + BAR_CLEAR + BOARD_SIZE[0] / 2  # 0.429: the boards' centre along x
BOARD_Z = BASE + DROP + BOARD_SIZE[2] / 2
BOARD_R = Box("board_right", pos=(BOARD_X, O_CLAMPED[1] + BOARD_SIZE[1] / 2, BOARD_Z), size=BOARD_SIZE,
              mass=BOARD_MASS, color=PINE)
BOARD_L = Box("board_left", pos=(BOARD_X, O_CLAMPED[1] + 1.5 * BOARD_SIZE[1] + JOINT_GAP, BOARD_Z), size=BOARD_SIZE,
              mass=BOARD_MASS, color=PINE)
PANEL_W = 2 * BOARD_SIZE[1] + JOINT_GAP  # 12.2 cm across, laid out

# -- the clamp -------------------------------------------------------------------------------------------------------
# Modelled on a 6 in one-hand mini bar clamp (IRWIN QUICK-GRIP mini: 6 in / 152 mm opening, 2-7/16 in / 62 mm throat,
# rated 140 lb / 623 N). Parts from O: (name, centre, size[, friction]). The handle: housing round the bar, the fixed
# jaw on its front (+y face toward the moving jaw), the pistol grip pointing at the robot (-x).
SLICK = (0.35, 0.3)  # nylon jaws and housing, the steel bar: they slide on the mat
HOUSING = ("housing", (-0.043, -0.0325, 0.0), (0.038, 0.059, CLAMP_T), SLICK)  # 59 mm along the bar
FIXED_JAW = ("fixed_jaw", (-0.004, -0.010, 0.0), (0.062, 0.014, CLAMP_T), SLICK)  # 62 mm throat past the bar
GRIP = ("grip", (-0.106, -0.0495, 0.0), (0.088, 0.025, 0.022))  # rubber, 1 mm clear of the mat
HANDLE_PARTS = (HOUSING, FIXED_JAW, GRIP)
HANDLE_LOW, HANDLE_HIGH = (-0.150, -0.062, -CLAMP_T / 2), (0.027, -0.003, CLAMP_T / 2)
HANDLE_MASS = 0.12
HANDLE_COM = (-0.061, -0.035, 0.0)
GRIP_BACK = -0.062  # y of the grip's back face, from O
RED, BLACK, STEEL = (0.80, 0.08, 0.06), (0.05, 0.05, 0.05), (0.62, 0.64, 0.66)

# The fixed jaw's pad: a 40 x 20 mm rubber pad 5 mm thick on a spring, its face (at O) 3 mm proud of the jaw's own
# face. Pressed by the work it gives way along the bar, up to 3 mm, and its compression times PAD_K is the force.
PAD_SIZE = (0.040, 0.005, 0.020)
PAD_CENTRE = (0.0, -PAD_SIZE[1] / 2, 0.0)
PAD_TRAVEL = 0.003
PAD_K = 40000.0  # N/m: 40 N per mm
PAD_MASS = 0.01
PAD_DAMPING = 40.0  # N s/m, near critical for PAD_K and PAD_MASS
FORCE_MIN = 20.0  # N: the least clamping force for full credit (0.5 mm of the pad's travel)

# The trigger: a lever hinged in the housing beside the bar, in front of the grip, pointing at the robot. Squeezed
# toward the grip it turns about +z, up to 0.40 rad (23 degrees, where its tip nearly meets the grip). Its drive arm,
# 9 mm from the pivot, pushes the bar toward the fixed jaw: the jaw advances 9 mm x sin(angle), 3.5 mm a full stroke,
# and a finger 36 mm from the pivot pushes with a 4 to 1 lever. A torsion spring holds it open with 8 N at 40 mm.
TRIGGER_LOW, TRIGGER_HIGH = (-0.126, -0.007, -0.008), (-0.052, 0.005, 0.008)
PIVOT = (-0.056, -0.001, 0.0)
DRIVE_ARM = 0.009  # m: pivot to where the trigger pushes the drive plate
NOSE = (PIVOT[0] - DRIVE_ARM, PIVOT[1], 0.0)
TRIGGER_MAX = 0.40  # rad
TRIGGER_MASS = 0.015
TRIGGER_SPRING = 0.37  # N m/rad
TRIGGER_REST = -0.86  # rad: the spring's target beyond the open stop: 0.32 N m (8 N at 40 mm) held open
TRIGGER_DAMPING = 0.003

# The bar and the moving jaw: the bar (16 x 4.8 mm steel, 231 mm) slides along y through the housing, the moving jaw
# (62 mm throat, black pad) on its far end, its face 14 cm from the fixed jaw's pad at the start.
OPEN = 0.140
MOVING_JAW = ("moving_jaw", (-0.004, OPEN + 0.007, 0.0), (0.062, 0.014, CLAMP_T), SLICK)
BAR = ("steel_bar", (-0.043, OPEN + 0.014 - 0.1155, 0.0), (0.016, 0.231, 0.0048), SLICK)
BAR_PARTS = (MOVING_JAW, BAR)
BAR_LOW, BAR_HIGH = (-0.051, OPEN + 0.014 - 0.231, -CLAMP_T / 2), (0.027, OPEN + 0.014, CLAMP_T / 2)
BAR_MASS = 0.17
BAR_COM = (-0.036, 0.068, 0.0)
BRAKE_HOLD = 300.0  # N/m: the brake's stop holds the bar with a soft spring (0.3 N per mm): it slides freely shut
BRAKE_DAMPING = 3.0

SLIDE_IN = 0.08  # m: the clamp starts this far in front (-x) of where it clamps, its jaw tips 2.2 cm off the boards
START_DY = -0.005  # m: and its fixed jaw's pad 5 mm off the right board's edge
O_START = (O_CLAMPED[0] - SLIDE_IN, O_CLAMPED[1] + START_DY, BASE + DROP + CLAMP_T / 2)


def _centre(lo, hi):
    return tuple((a + b) / 2 for a, b in zip(lo, hi))


def _size(lo, hi):
    return tuple(b - a for a, b in zip(lo, hi))


def _world(p_from_o, o=O_START):
    return tuple(a + b for a, b in zip(o, p_from_o))


O_TO_HANDLE = _centre(HANDLE_LOW, HANDLE_HIGH)
O_TO_TRIGGER = _centre(TRIGGER_LOW, TRIGGER_HIGH)
O_TO_BAR = _centre(BAR_LOW, BAR_HIGH)


def _local(c, p):
    """A point given from O, in the frame of a box whose centre is ``c`` from O."""
    return tuple(a - b for a, b in zip(p, c))


CLAMP = Compound(
    "clamp",
    pos=_world(O_TO_HANDLE),
    size=_size(HANDLE_LOW, HANDLE_HIGH),
    mass=HANDLE_MASS,
    color=RED,
    parts=tuple((n, _local(O_TO_HANDLE, c), s, *f) for n, c, s, *f in HANDLE_PARTS),
    com=_local(O_TO_HANDLE, HANDLE_COM),
    visible=True,
    colors=(("grip", BLACK),),
)
PAD = Ratchet(
    "pad", pos=_world(PAD_CENTRE), size=PAD_SIZE, mass=PAD_MASS, color=BLACK, to="clamp", axis="Y",
    lower=-PAD_TRAVEL, upper=0.0, hold=PAD_K, damping=PAD_DAMPING, collide=False,
)
BAR_BODY = Slide(
    "bar",
    pos=_world(O_TO_BAR),
    size=_size(BAR_LOW, BAR_HIGH),
    mass=BAR_MASS,
    color=RED,
    parts=tuple((n, _local(O_TO_BAR, c), s, *f) for n, c, s, *f in BAR_PARTS),
    com=_local(O_TO_BAR, BAR_COM),
    visible=True,
    colors=(("steel_bar", STEEL),),
    decor=(("moving_pad", _local(O_TO_BAR, (0.0, OPEN + 0.001, 0.0)), (0.040, 0.003, 0.020), BLACK),),
    to="clamp",
    axis="Y",
    lower=-(OPEN - 0.002),
    upper=0.0,
    hold=BRAKE_HOLD,
    damping=BRAKE_DAMPING,
    collide=False,
)
TRIGGER = Hinge(
    "trigger",
    pos=_world(O_TO_TRIGGER),
    size=_size(TRIGGER_LOW, TRIGGER_HIGH),
    mass=TRIGGER_MASS,
    color=RED,
    to="clamp",
    pivot=_local(O_TO_TRIGGER, PIVOT),
    axis="Z",
    lower=0.0,
    upper=TRIGGER_MAX,
    spring=TRIGGER_SPRING,
    rest=TRIGGER_REST,
    damping=TRIGGER_DAMPING,
)
DRIVE = one_way("drive", "bar", "trigger", at=_world(NOSE), axis="Y")

# -- the workbench props: fixed, out of the way ----------------------------------------------------------------------
MALLET = SimReady("mallet", pos=(0.585, 0.33, TOP), size=(0.394, 0.131, 0.071), yaw=math.pi / 2,
                  package="handTool_hammer_rubberMallet_a01", usd="sm_handTool_hammer_rubberMallet_a01_01.usd")
SQUARE = SimReady("speed_square", pos=(0.58, -0.40, TOP), size=(0.29, 0.218, 0.008), yaw=0.35,
                  package="measurement_ruler_speedsquare_a02", usd="sm_measurement_ruler_speedsquare_a02_01.usd")

OBJECTS = [MAT, BOARD_R, BOARD_L, CLAMP, PAD, BAR_BODY, TRIGGER, MALLET, SQUARE]
CLAMP_BODIES = ("clamp", "pad", "bar", "trigger")
BOARDS = ("board_right", "board_left")

SOLVER_ITERATIONS = (32, 8)  # position, velocity: the clamp and the boards press each other with 100 N and more


@on_spawn
def _accurate_solver(name: str, prim) -> None:
    """More solver iterations for the clamp's bodies and the boards than PhysX's default 4 and 1 (the robot has 16 and
    4): the clamped panel's force is held by a loop of joint limits and contacts between light bodies."""
    if name in (*CLAMP_BODIES, *BOARDS):
        from pxr import PhysxSchema

        api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        api.CreateSolverPositionIterationCountAttr(SOLVER_ITERATIONS[0])
        api.CreateSolverVelocityIterationCountAttr(SOLVER_ITERATIONS[1])


LOOKS = {}
if (pathlib.Path(__file__).resolve().parent / "looks" / "board_right.usdc").is_file():
    LOOKS = {"mat": "mat.usdc", "board_right": "board_right.usdc", "board_left": "board_left.usdc"}
    dress(LOOKS)

# -- geometry of the state -------------------------------------------------------------------------------------------
TOUCH = 0.0005  # m: a jaw is on its board's edge when its gap is 0.5 mm or less
JOINT_SHUT = 0.0005  # m: the joint between the boards is closed when no point of it is open more than this
GAP_FLOOR = -0.003  # m: a jaw deeper in its board than this is nonsense (a numerical blow-up)
ON_EDGE = 0.01  # m: a pad centre at least this far inside a board's ends is on its edge
ALIGNED = 0.003  # m: the boards' ends within this of each other
FLUSH = 0.001  # m: the boards' top faces within this of each other along the joint
CLEAR = 0.03  # m: every finger at least this far from the clamp and the boards: let go
OFF_TABLE_Z = -0.02  # m: an object whose centre is lower than this from the table top fell off
JAW_POINTS = (-0.018, 0.0, 0.018)  # x of the points along each jaw's pad (from its centre) whose gaps are measured
STILL_STEPS = 10  # control steps (0.5 s at 20 Hz): the clamp and the boards must not have moved over this long
STILL_DIST = 0.0005  # m
STILL_TURN = 0.01  # rad
TOOTH = 0.0001  # m: the brake and the drive plate grip the bar in steps this fine
HZ = 20.0


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) and np.all(np.isfinite(s.lin_vel))
               for s in objects.values())


_CENTRES = {"clamp": O_TO_HANDLE, "pad": PAD_CENTRE, "bar": O_TO_BAR, "trigger": O_TO_TRIGGER}


def point(objects, body: str, p_from_o) -> np.ndarray:
    """A point of a clamp body, given from O as the clamp spawned, in the world now."""
    s = objects[body]
    return np.asarray(s.pos) + _rot(s.quat_xyzw) @ (np.asarray(p_from_o) - np.asarray(_CENTRES[body]))


def _in_handle(objects, body: str, p_from_o) -> np.ndarray:
    """Where a point of ``body`` (given from O at spawn) is now, from O in the handle's frame."""
    h = objects["clamp"]
    return _rot(h.quat_xyzw).T @ (point(objects, body, p_from_o) - np.asarray(h.pos)) + np.asarray(O_TO_HANDLE)


def pad_compression(objects) -> float:
    """How far the fixed jaw's pad is pressed in along the bar (m), from its rest on the handle. Only something
    pushing on its face presses it in, so a pad that touches nothing reads 0."""
    return float(PAD_CENTRE[1] - _in_handle(objects, "pad", PAD_CENTRE)[1])


def clamping_force(objects) -> float:
    """The force the jaws press the work with (N): the pad's compression times its spring's stiffness."""
    return float(PAD_K * max(0.0, pad_compression(objects)))


def bar_travel(objects) -> float:
    """How far the bar has moved through the handle from the start (m, negative: toward the fixed jaw)."""
    return float(_in_handle(objects, "bar", O_TO_BAR)[1] - O_TO_BAR[1])


def trigger_angle(objects) -> float:
    """How far the trigger is squeezed (rad, 0 open, TRIGGER_MAX against its stop)."""
    r = _rot(objects["clamp"].quat_xyzw).T @ _rot(objects["trigger"].quat_xyzw)
    return float(math.atan2(r[1, 0], r[0, 0]))


def drive_offset(objects) -> float:
    """Where the trigger's drive arm is along the bar (m), from the drive plate's start on the bar: the one-way drive's
    coordinate. It cannot go below the drive's stop: there the trigger pushes the bar."""
    b = objects["bar"]
    along = _rot(b.quat_xyzw)[:, 1]
    return float((point(objects, "trigger", NOSE) - point(objects, "bar", NOSE)) @ along)


def fixed_pad_points(objects) -> list[np.ndarray]:
    return [point(objects, "pad", (x, 0.0, 0.0)) for x in JAW_POINTS]


def moving_pad_points(objects) -> list[np.ndarray]:
    return [point(objects, "bar", (x, OPEN, 0.0)) for x in JAW_POINTS]


def _board_local(objects, board: str, p) -> np.ndarray:
    b = objects[board]
    return _rot(b.quat_xyzw).T @ (np.asarray(p) - np.asarray(b.pos))


def gaps(objects) -> tuple[float, float]:
    """(fixed, moving): how close each jaw's pad comes to the outer edge of the board it should press (m, along the
    board's width, the least over three points along the pad; negative inside it). The fixed jaw on the right board's
    outer (-y) edge, the moving jaw on the left board's outer (+y) edge. A gap says where a jaw is, not how hard it
    presses: that is :func:`clamping_force`."""
    half = BOARD_SIZE[1] / 2
    fixed = min(-half - _board_local(objects, "board_right", p)[1] for p in fixed_pad_points(objects))
    moving = min(_board_local(objects, "board_left", p)[1] - half for p in moving_pad_points(objects))
    return float(fixed), float(moving)


def joint_gap(objects) -> float:
    """How far open the joint between the boards is at its worst (m): the left board's inner edge, at both its ends,
    from the right board's inner edge."""
    hx, hy = BOARD_SIZE[0] / 2, BOARD_SIZE[1] / 2
    b = objects["board_left"]
    ends = [np.asarray(b.pos) + _rot(b.quat_xyzw) @ np.array([sx * hx, -hy, 0.0]) for sx in (-1, 1)]
    return float(max(_board_local(objects, "board_right", e)[1] - hy for e in ends))


def _on_edge(objects, board: str, p) -> bool:
    """The pad centre ``p`` is beside the board's edge: at least 1 cm inside its ends and within its thickness."""
    x, _, z = _board_local(objects, board, p)
    return bool(abs(x) <= BOARD_SIZE[0] / 2 - ON_EDGE and abs(z) <= BOARD_SIZE[2] / 2)


def round_panel(objects) -> bool:
    """The clamp is round the panel: each jaw's pad centre beside its board's outer edge (1 cm or more inside the
    board's ends, within its thickness) and outside it (no deeper in than GAP_FLOOR)."""
    fixed, moving = gaps(objects)
    return bool(_on_edge(objects, "board_right", point(objects, "pad", (0.0, 0.0, 0.0)))
                and _on_edge(objects, "board_left", point(objects, "bar", (0.0, OPEN, 0.0)))
                and fixed >= GAP_FLOOR and moving >= GAP_FLOOR)


def closed_on(objects) -> bool:
    """Round the panel, both jaws on their edges (gaps 0.5 mm or less) and the joint closed (0.5 mm or less)."""
    fixed, moving = gaps(objects)
    return bool(round_panel(objects) and fixed <= TOUCH and moving <= TOUCH and joint_gap(objects) <= JOINT_SHUT)


def clamped(objects) -> bool:
    """Closed on the panel and pressing it with FORCE_MIN or more (the pad pressed in 0.5 mm or more)."""
    return bool(closed_on(objects) and clamping_force(objects) >= FORCE_MIN)


def _corners(objects, board: str, side: float) -> np.ndarray:
    """The board's two top corners on its long edge at ``side`` (-1: -y edge, +1: +y edge), near end first."""
    b = objects[board]
    hx, hy, hz = (s / 2 for s in BOARD_SIZE)
    pts = np.array([[sx * hx, side * hy, hz] for sx in (-1, 1)])
    return np.asarray(b.pos) + pts @ _rot(b.quat_xyzw).T


def misalignment(objects) -> float:
    """How far apart (m, along the boards' length) the boards' matching ends are at the joint, at worst."""
    d = _corners(objects, "board_left", -1) - _corners(objects, "board_right", 1)
    along = _rot(objects["board_right"].quat_xyzw)[:, 0]
    return float(np.max(np.abs(d @ along)))


def step_height(objects) -> float:
    """How far apart (m, up) the boards' top faces are along the joint, at worst: a board ridden up on the other."""
    d = _corners(objects, "board_left", -1)[:, 2] - _corners(objects, "board_right", 1)[:, 2]
    return float(np.max(np.abs(d)))


def aligned(objects) -> bool:
    return bool(misalignment(objects) < ALIGNED and step_height(objects) <= FLUSH)


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return any(objects[n].pos[2] < TOP + OFF_TABLE_Z for n in (*BOARDS, *CLAMP_BODIES))


# -- the fingers -----------------------------------------------------------------------------------------------------


def _boxes():
    """Every collider the fingers must keep clear of, as (body, centre from O at spawn, size)."""
    out = [("clamp", c, s) for _, c, s, *_ in HANDLE_PARTS]
    out += [("bar", c, s) for _, c, s, *_ in BAR_PARTS]
    out += [("pad", PAD_CENTRE, PAD_SIZE), ("trigger", O_TO_TRIGGER, _size(TRIGGER_LOW, TRIGGER_HIGH))]
    return out


def finger_distance(objects, fingers, boards: bool = True) -> float:
    """The closest any finger point comes to the clamp (every part of it) or, with ``boards``, to the boards (m)."""
    if fingers is None or len(fingers) == 0:
        return math.inf
    pts = np.asarray(fingers, float)
    best = math.inf
    for body, centre, size in _boxes():
        s = objects[body]
        local = (pts - np.asarray(s.pos)) @ _rot(s.quat_xyzw) - (np.asarray(centre) - np.asarray(_CENTRES[body]))
        best = min(best, float(np.min(np.linalg.norm(np.maximum(np.abs(local) - np.asarray(size) / 2, 0.0), axis=1))))
    for board in (BOARDS if boards else ()):
        b = objects[board]
        local = (pts - np.asarray(b.pos)) @ _rot(b.quat_xyzw)
        best = min(best, float(np.min(np.linalg.norm(np.maximum(np.abs(local) - np.asarray(BOARD_SIZE) / 2, 0.0),
                                                     axis=1))))
    return best


# -- what the episode remembers --------------------------------------------------------------------------------------
STATE = {"step": 0, "clear": True, "still": False, "history": [], "met_since": None, "was": None}


@on_reset
def reset_memory(env=None) -> None:
    STATE.update(step=0, clear=True, still=False, history=[], met_since=None, was=None)


def pose(objects) -> tuple:
    """The clamp's handle, its bar and both boards' positions and turns, to tell whether they have moved."""
    return tuple((np.asarray(objects[n].pos, float), np.asarray(objects[n].quat_xyzw, float))
                 for n in ("clamp", "bar", *BOARDS))


def settled(history) -> bool:
    """The clamp and both boards have not moved: at every one of the last STILL_STEPS control steps (0.5 s) each was
    within 0.5 mm and 0.01 rad of where it is now. ``history`` is the poses of the last steps, oldest first."""
    if len(history) <= STILL_STEPS:
        return False
    new = history[-1]
    for old in history[-1 - STILL_STEPS:-1]:
        for (p0, q0), (p1, q1) in zip(old, new):
            turn = 2 * math.acos(min(1.0, abs(float(np.dot(q0, q1)) / (np.linalg.norm(q0) * np.linalg.norm(q1)))))
            if np.linalg.norm(p1 - p0) >= STILL_DIST or turn >= STILL_TURN:
                return False
    return True


def conditions_met(objects, clear: bool, still: bool) -> bool:
    """Every full-success condition, as things stand: clamped with FORCE_MIN or more, the boards aligned and flush,
    every finger clear of the clamp and the boards, the clamp and the boards still, and no violation."""
    return bool(not _violation(objects) and clamped(objects) and aligned(objects) and clear and still)


def next_brake(objects, stop: float) -> float:
    """The brake's stop (m along the bar, negative toward the fixed jaw) after this control step: it follows the bar
    whenever the bar has gone 0.1 mm past it toward the fixed jaw, never back."""
    travel = bar_travel(objects)
    return travel if travel < stop - TOOTH else stop


def next_drive(objects, stop: float) -> float:
    """The drive plate's stop after this control step: it follows the trigger's drive arm whenever the arm has gone
    back 0.1 mm past it (the trigger springing open), never the other way. A squeeze then pushes the bar from there."""
    offset = drive_offset(objects)
    return offset if offset > stop + TOOTH else stop


@on_step
def mechanism_and_memory(env) -> None:
    """Every control step: the brake and the drive plate grip the bar where it now is, then :func:`remember`."""
    from fingers import finger_points
    from yam_arena.scene import object_states

    objects = object_states(env, [*CLAMP_BODIES, *BOARDS])
    if not _finite(objects):
        STATE["step"] += 1
        return
    brake = next_brake(objects, upper("bar"))
    if brake != upper("bar"):
        set_upper("bar", brake)
    drive = next_drive(objects, low("drive"))
    if drive != low("drive"):
        set_low("drive", drive)
    remember(objects, finger_points(env))
    _log(objects)


def remember(objects, fingers) -> None:
    """One control step of the episode's memory: whether every finger is clear of the clamp and the boards, whether
    the clamp and the boards have been still for 0.5 s, and since which step every full-success condition has held
    without a break (any lapse, a finger coming back or a board moving, restarts the count)."""
    STATE["step"] += 1
    STATE["clear"] = finger_distance(objects, fingers) >= CLEAR
    STATE["history"] = (STATE["history"] + [pose(objects)])[-(STILL_STEPS + 1):]
    STATE["still"] = settled(STATE["history"])
    met = conditions_met(objects, STATE["clear"], STATE["still"])
    t = STATE["step"] / HZ
    if met and STATE["met_since"] is None:
        STATE["met_since"] = STATE["step"]
        print(f"{TAG} t={t:.2f}s  success conditions met: timing {HOLD_S:g} s", flush=True)
    elif not met and STATE["met_since"] is not None:
        STATE["met_since"] = None
        print(f"{TAG} t={t:.2f}s  success conditions lapsed: timing restarts", flush=True)


def _log(objects) -> None:
    fixed, moving = gaps(objects)
    now = (round_panel(objects), closed_on(objects), clamped(objects))
    if now != STATE["was"] or STATE["step"] % 10 == 0:
        print(f"{TAG} t={STATE['step'] / HZ:.2f}s trigger={math.degrees(trigger_angle(objects)):5.1f}deg "
              f"bar={bar_travel(objects) * 1000:6.1f}mm brake={upper('bar') * 1000:6.1f} "
              f"drive={drive_offset(objects) * 1000:5.2f}/{low('drive') * 1000:5.2f} "
              f"force={clamping_force(objects):5.1f}N gaps={fixed * 1000:5.1f},{moving * 1000:5.1f} "
              f"joint={joint_gap(objects) * 1000:5.1f} align={misalignment(objects) * 1000:4.1f} "
              f"round={now[0]} closed={now[1]} clamped={now[2]} clear={STATE['clear']} still={STATE['still']}",
              flush=True)
        if DEBUG:
            h = objects["clamp"]
            r = _rot(h.quat_xyzw)
            o = np.asarray(h.pos) - r @ np.asarray(O_TO_HANDLE) - np.asarray(O_CLAMPED)
            print(f"{TAG}   O-O_clamped=({o[0] * 1000:.1f},{o[1] * 1000:.1f},{o[2] * 1000:.1f})mm "
                  f"yaw={math.degrees(math.atan2(r[1, 0], r[0, 0])):.2f} tilt={math.degrees(math.acos(min(1, r[2, 2]))):.2f} "
                  f"boardR=({(objects['board_right'].pos[0] - BOARD_R.pos[0]) * 1000:.1f},"
                  f"{(objects['board_right'].pos[1] - BOARD_R.pos[1]) * 1000:.1f}) "
                  f"boardL=({(objects['board_left'].pos[0] - BOARD_L.pos[0]) * 1000:.1f},"
                  f"{(objects['board_left'].pos[1] - BOARD_L.pos[1]) * 1000:.1f})", flush=True)
        STATE["was"] = now


# -- scoring ---------------------------------------------------------------------------------------------------------


def held_s() -> float:
    """How long every full-success condition has held without a break (s), 0 while one fails."""
    return 0.0 if STATE["met_since"] is None else (STATE["step"] - STATE["met_since"]) / HZ


def success(objects) -> bool:
    """Every full-success condition (:func:`conditions_met`) has held without a break for HOLD_S (1 s): clamped with
    20 N or more, the boards aligned and flush, let go, and the clamp and the boards still."""
    if _violation(objects) or STATE["met_since"] is None:
        return False
    return bool(conditions_met(objects, STATE["clear"], STATE["still"]) and held_s() >= HOLD_S - 1e-6)


def score(objects) -> float:
    """1.0: success. 0.75: clamped with 20 N or more (jaws on both outer edges, the joint closed) but not yet
    successful (the boards misaligned or not flush, the clamp or a board still held or moving, or not yet for 1 s).
    0.5: the jaws on both outer edges and the joint closed, but under 20 N. 0.25: the clamp round the panel (each
    jaw's pad beside its board's outer edge, 1 cm or more inside its ends). 0 otherwise, and 0 if a board or the
    clamp fell off the table."""
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    if clamped(objects):
        return 0.75
    if closed_on(objects):
        return 0.5
    return 0.25 if round_panel(objects) else 0.0


def distance(objects) -> float:
    """Progress: how far the fixed jaw's pad is from where it clamps (along the bar, beyond 5 mm), plus the larger jaw gap, the joint's gap and the
    force still missing (1 mm for each 40 N)."""
    if not _finite(objects):
        return math.inf
    off = point(objects, "pad", (0.0, 0.0, 0.0)) - np.asarray(O_CLAMPED)
    off[1] = max(0.0, abs(off[1]) - 0.005)  # along the bar the boards may sit a few mm either way
    missing = max(0.0, FORCE_MIN - clamping_force(objects)) / PAD_K
    return float(np.linalg.norm(off) + max(0.0, *gaps(objects)) + max(0.0, joint_gap(objects)) + missing)


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm, straight down, fingers closing along y (along the bar), held as a hand holds a one-hand clamp: the back
# finger is the palm on the pistol grip's back and stays there, the front finger is the index finger on the trigger,
# 50 mm from its pivot. The TCP moves with the opening (half of it), so the back finger stays put while the front
# finger squeezes the trigger (commanded 7 mm past its full stroke, it stops there) and lets it spring back (opened to
# OPEN_W, 2 cm clear of it). The first squeeze holds the clamp while the arm slides it 8 cm forward, its jaws round the
# panel's near end; then it pumps the trigger: the bar takes up the slack, pushes the left board onto the right one and
# both onto the fixed jaw, and the last strokes press the pad in. It lets go and goes up.
# Open loop: the waypoints are fixed TCP positions; nothing is read from the simulation.
TCP_X = -0.106  # from O
BACK_Y = GRIP_BACK + 0.0005  # from O: the back finger's pad, just pressing the grip's back
TCP_Z = BASE + 0.004 + PAD_TIP_PAST_TCP  # the pad tips 4 mm above the mat
HOVER_Z = BASE + 0.13
SQUEEZE_W = 0.040  # m: the fingers' opening commanded for a squeeze (the trigger's full stroke is 47 mm)
OPEN_W = 0.090  # m: and between squeezes (the front finger 2 cm clear of the open trigger)
APPROACH_BACK = 0.008  # m: the back finger comes down this far behind the grip
STROKES = 2  # pumps after the slide
CLOSE_S, SQUEEZE_HOLD_S, OPEN_S, OPEN_HOLD_S = 0.5, 0.3, 0.4, 0.2


def _fingers(width: float) -> float:
    """The finger joint target for an opening of ``width`` (m)."""
    return -width / 2


def _tcp_y(width: float, back: float = BACK_Y) -> float:
    """The TCP's y (from O) that puts the back finger's pad at ``back`` with the fingers opened ``width``."""
    return back + width / 2


def demo():
    w = Waypoint
    x0, x1 = O_START[0] + TCP_X, O_START[0] + TCP_X + SLIDE_IN
    oy = O_START[1]
    wide = -FINGER_OPEN * 2  # the fingers fully open (94 mm)
    y_down = oy + _tcp_y(wide, BACK_Y - APPROACH_BACK)
    y_open, y_squeeze = oy + _tcp_y(OPEN_W), oy + _tcp_y(SQUEEZE_W)
    plan = [
        w(2.0, (x0, y_down, HOVER_Z), FINGER_OPEN, "over the grip and trigger"),
        w(1.5, (x0, y_down, TCP_Z), FINGER_OPEN, "down round them"),
        w(0.5, (x0, y_open, TCP_Z), _fingers(OPEN_W), "back finger on the grip"),
        w(CLOSE_S + 0.1, (x0, y_squeeze, TCP_Z), _fingers(SQUEEZE_W), "squeeze: hold the clamp"),
        w(SQUEEZE_HOLD_S, (x0, y_squeeze, TCP_Z), _fingers(SQUEEZE_W), "hold"),
        w(2.5, (x1, y_squeeze, TCP_Z), _fingers(SQUEEZE_W), "slide the jaws round the panel"),
    ]
    for k in range(STROKES):
        plan += [
            w(OPEN_S, (x1, y_open, TCP_Z), _fingers(OPEN_W), f"let the trigger open ({k + 1})"),
            w(OPEN_HOLD_S, (x1, y_open, TCP_Z), _fingers(OPEN_W), "open"),
            w(CLOSE_S, (x1, y_squeeze, TCP_Z), _fingers(SQUEEZE_W), f"squeeze ({k + 1})"),
            w(SQUEEZE_HOLD_S, (x1, y_squeeze, TCP_Z), _fingers(SQUEEZE_W), "hold"),
        ]
    y_wide = oy + _tcp_y(wide, BACK_Y - APPROACH_BACK)
    if os.environ.get("BAR_CLAMP_DEMO") == "by_hand":
        return _push_shut_by_hand(plan[:6], x1, y_wide)
    plan += [
        w(OPEN_S, (x1, y_open, TCP_Z), _fingers(OPEN_W), "let the trigger open"),
        w(OPEN_S, (x1, y_wide, TCP_Z), FINGER_OPEN, "let go"),
        w(1.0, (x1, y_wide, HOVER_Z), FINGER_OPEN, "up"),
        w(1.0, (x1 - 0.06, y_wide - 0.06, HOVER_Z), FINGER_OPEN, "back to its side"),
        w(2.5, (x1 - 0.06, y_wide - 0.06, HOVER_Z), FINGER_OPEN, "wait: the clamp holds the boards"),
    ]
    return {"right": plan, "left": _camera_boom(sum(p.duration for p in plan))}


# The left arm only films: it holds its wrist camera (13 cm above its fingers, looking down) 24 cm above the mat, over
# the panel's near end, for a close view of the jaws, the bar, the handle and the joint. Its fingers stay closed 9 cm
# above the mat, over the left board, and touch nothing.
CAMERA_TCP = (0.33, -0.02, BASE + 0.24 - 0.134)
CAMERA_YAW = 0.0


def _camera_boom(duration: float) -> list:
    w = Waypoint
    return [
        w(3.0, CAMERA_TCP, FINGER_CLOSED, "camera over the clamp", CAMERA_YAW),
        w(max(0.1, duration - 3.0), CAMERA_TCP, FINGER_CLOSED, "film", CAMERA_YAW),
    ]


# The check that the trigger is needed (``BAR_CLAMP_DEMO=by_hand``): the same start and slide, then, instead of
# pumping the trigger, the right arm lets go of the clamp and the left arm pushes the moving jaw shut with its closed
# fingers, as a hand slides the jaw of a one-hand clamp shut, 25 mm past where the jaws meet the panel, and lets go.
FINGER_T = 0.010  # m: a finger's thickness behind its pad, at the tip
PUSH_PAST = 0.025  # m: how far past the panel the fingers push the jaw


def _push_shut_by_hand(plan: list, x1: float, y_wide: float) -> dict:
    """The right arm slides the clamp in as the demo does and lets go; the left arm then pushes the moving jaw shut."""
    w = Waypoint
    right = plan + [
        w(OPEN_S, (x1, y_wide, TCP_Z), FINGER_OPEN, "let go"),
        w(1.0, (x1, y_wide, HOVER_Z), FINGER_OPEN, "up"),
        w(1.0, (x1 - 0.06, y_wide - 0.06, HOVER_Z), FINGER_OPEN, "back to its side"),
    ]
    jaw_x = O_CLAMPED[0] + MOVING_JAW[1][0]
    face = O_CLAMPED[1] + OPEN + MOVING_JAW[2][1] - 0.0035  # the moving jaw's back after the first squeeze
    slack = face - MOVING_JAW[2][1] - (O_CLAMPED[1] + PANEL_W)  # how far it is from the panel
    y_start, y_end = face + FINGER_T + 0.015, face + FINGER_T - slack - PUSH_PAST
    wait = sum(p.duration for p in right) - 1.0
    left = [
        w(2.0, (jaw_x, y_start, HOVER_Z), FINGER_CLOSED, "over the moving jaw, fingers closed"),
        w(wait - 2.0, (jaw_x, y_start, HOVER_Z), FINGER_CLOSED, "wait for the slide"),
        w(1.5, (jaw_x, y_start, TCP_Z), FINGER_CLOSED, "down behind it"),
        w(3.0, (jaw_x, y_end, TCP_Z), FINGER_CLOSED, "push the jaw shut"),
        w(1.0, (jaw_x, y_end, TCP_Z), FINGER_CLOSED, "hold"),
        w(1.0, (jaw_x, y_end + 0.03, TCP_Z), FINGER_CLOSED, "back off"),
        w(1.0, (jaw_x, y_end + 0.03, HOVER_Z), FINGER_CLOSED, "up"),
        w(3.0, (jaw_x, y_end + 0.03, HOVER_Z), FINGER_CLOSED, "wait"),
    ]
    return {"right": right, "left": left}
