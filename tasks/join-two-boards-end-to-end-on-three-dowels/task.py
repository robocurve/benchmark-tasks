"""Two boards lie on the bench, one beside each arm, their ends facing each other across the midline: a cherry board
with three beech dowels glued into its end, and a walnut board with three round sockets drilled into its end at the
same pitch and height. Slid together square and in line, the dowels go into the sockets and the ends
meet; brought together more than a dowel's chamfer out of line, the boards stand on the dowels' tips.

How it is built. Each board is one free body lying on the bench; nothing clamps, guides or stops either. The dowels
are part of the cherry board (glued): each a cylinder 8 mm across standing 20 mm out of the board's end, its last
millimetre a 45 degree chamfer of sixteen thin plates round a flat tip 6 mm across. Each socket is a round bore
22 mm deep: twenty-four flat staves round the bore's axis inside the walnut board's end, their inner faces a polygon
whose flats stand half the bore from the axis. A dry fit: no glue and no press fit.
"""

from __future__ import annotations

import math
import os
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
INSTRUCTION = "Join the two boards on the three dowels so that their ends meet, flat on the bench, then let go."
RUN_LIMIT_STEPS = 1200  # three times the full run, rounded up to a hundred steps, at most 1,200: the full run takes 836 steps (recording c5l), so 1,200
TOP = TABLE_TOP[SCENE]

CHERRY, WALNUT, BEECH = (0.55, 0.30, 0.18), (0.30, 0.20, 0.13), (0.78, 0.60, 0.40)  # sRGB

# --- the boards: 18 x 80 x 150 mm, lying flat, long axes along y; the cherry one's middle 36 cm in front of the line
# of the arm bases and 33 cm to the right of the midline, the walnut one's 33 cm to the left; their facing ends 51 cm
# apart ---
BOARD_W, BOARD_L, BOARD_T = 0.080, 0.150, 0.018
CHERRY_XY, WALNUT_XY = (0.36, -0.33), (0.36, 0.33)
CHERRY_MASS, WALNUT_MASS = 0.125, 0.140  # cherry and walnut of that size: assumed, to be weighed
FRICTION = (0.25, 0.25)  # wood on the waxed laminate bench (a spring scale pulls the cherry board with about 0.3 N), and every other contact of the boards: assumed
NAMES = ("cherry", "walnut")

# --- the dowels: beech, 8 x 40 mm with chamfered ends, glued 20 mm deep into the cherry board's end at 25 mm pitch on
# its centre line, at mid-thickness ---
DOWEL_D, DOWEL_OUT, CHAMFER = 0.008, 0.020, 0.001  # the chamfer, 1 mm at 45 degrees: assumed, to be measured with a caliper
PITCH = 0.025
DOWEL_X = (-PITCH, 0.0, PITCH)
CHAMFER_PLATES = 16

# --- the sockets: 22 mm deep, drilled with an 8 mm brad-point bit, at the same pitch and height in the walnut board's end.
# In the simulator a bore is a polygon of staves, 8.2 mm across: the tightest of the bores tried (8.2 and 8.3 mm; the
# earlier build used 8.4); boards pushed together in line close on it (trials.py checks: recording c5m; see the README) ---
SOCKET_DEPTH = 0.022
BORE = float(os.environ.get("DOWEL_BORE_MM", "8.2")) / 1000
STAVES = 24
OPENING = 0.006  # m: half the square opening round each bore that the staves fill


def _frame(columns) -> tuple:
    return mk.matrix_to_quat(np.column_stack(columns))


def _dowel(x: float) -> list:
    """One dowel, in the cherry board's frame: a cylinder from just inside the board's end, the chamfer's plates and
    the flat tip."""
    end, straight, tip_r = BOARD_L / 2, DOWEL_OUT - CHAMFER, DOWEL_D / 2 - CHAMFER
    y0, y1 = end - 0.001, end + straight
    shapes = [cylinder(DOWEL_D / 2, y1 - y0, pos=(x, (y0 + y1) / 2, 0.0), axis="y", color=BEECH), cylinder(tip_r, CHAMFER, pos=(x, end + DOWEL_OUT - CHAMFER / 2, 0.0), axis="y", color=BEECH)]
    slant, thick = math.sqrt(2.0) * CHAMFER, 0.0005
    width = 2 * (DOWEL_D / 2 - CHAMFER / 2) * math.tan(math.pi / CHAMFER_PLATES) * 1.08
    for k in range(CHAMFER_PLATES):
        a = 2 * math.pi * k / CHAMFER_PLATES
        radial, along = np.array([math.cos(a), 0.0, math.sin(a)]), np.array([0.0, 1.0, 0.0])
        tangent = np.array([-math.sin(a), 0.0, math.cos(a)])
        out = (radial + along) / math.sqrt(2.0)  # the chamfer's outward normal: away from the axis and towards the tip
        middle = np.array([x, end + straight + CHAMFER / 2, 0.0]) + (DOWEL_D / 2 - CHAMFER / 2) * radial - thick / 2 * out
        shapes.append(box((width, slant, thick), pos=tuple(middle), quat=_frame([-tangent, (along - radial) / math.sqrt(2.0), out]), color=BEECH))
    return shapes


def _socket_end() -> list:
    """The walnut board's first 22 mm, in its frame: two strips and four pillars round three square openings, and in
    each opening the staves of a round bore."""
    y = -BOARD_L / 2 + SOCKET_DEPTH / 2
    rim = BOARD_T / 2 - OPENING
    shapes = [box((BOARD_W, SOCKET_DEPTH, rim), pos=(0.0, y, s * (OPENING + rim / 2))) for s in (-1, 1)]
    edges = [-BOARD_W / 2, *[e for x in DOWEL_X for e in (x - OPENING, x + OPENING)], BOARD_W / 2]
    for left, right in zip(edges[0::2], edges[1::2]):
        shapes.append(box((right - left, SOCKET_DEPTH, 2 * OPENING), pos=((left + right) / 2, y, 0.0)))
    inner = BORE / 2
    width = 2 * inner * math.tan(math.pi / STAVES) * 1.15
    along = np.array([0.0, 1.0, 0.0])
    for x in DOWEL_X:
        for k in range(STAVES):
            a = 2 * math.pi * (k + 0.5) / STAVES
            radial, tangent = np.array([math.cos(a), 0.0, math.sin(a)]), np.array([-math.sin(a), 0.0, math.cos(a)])
            reach = OPENING / max(abs(math.cos(a)), abs(math.sin(a))) + 0.0005  # into the strips and pillars
            middle = np.array([x, y, 0.0]) + (inner + reach) / 2 * radial
            shapes.append(box((width, SOCKET_DEPTH, reach - inner), pos=tuple(middle), quat=_frame([-tangent, along, radial])))
    return shapes


BOARD_CHERRY = body("cherry", [box((BOARD_W, BOARD_L, BOARD_T)), *[shape for x in DOWEL_X for shape in _dowel(x)]], pos=(*CHERRY_XY, TOP + BOARD_T / 2), mass=CHERRY_MASS, color=CHERRY, friction=FRICTION,
                    friction_combine="min")  # fmt: skip
BOARD_WALNUT = body("walnut", [box((BOARD_W, BOARD_L - SOCKET_DEPTH, BOARD_T), pos=(0.0, SOCKET_DEPTH / 2, 0.0)), *_socket_end()], pos=(*WALNUT_XY, TOP + BOARD_T / 2), mass=WALNUT_MASS, color=WALNUT,
            friction=FRICTION, friction_combine="min")  # fmt: skip
OBJECTS = list(mk.MECH.bodies.values())
BOXES = {"cherry": [(np.array([-BOARD_W / 2, -BOARD_L / 2, -BOARD_T / 2]), np.array([BOARD_W / 2, BOARD_L / 2, BOARD_T / 2])),
                  (np.array([-PITCH - DOWEL_D / 2, BOARD_L / 2, -DOWEL_D / 2]), np.array([PITCH + DOWEL_D / 2, BOARD_L / 2 + DOWEL_OUT, DOWEL_D / 2]))],
         "walnut": [(np.array([-BOARD_W / 2, -BOARD_L / 2, -BOARD_T / 2]), np.array([BOARD_W / 2, BOARD_L / 2, BOARD_T / 2]))]}  # fmt: skip

# --- the rules ---
OVERLAP, SAME_HEIGHT, FLAT = 0.002, 0.002, math.radians(3.0)  # every level: gaps not under -2 mm, heights within 2 mm, both flat within 3 degrees
TIPS_IN, HALF_IN, CLOSED = 0.019, 0.010, 0.0005  # m: both seam gaps at most this for 0.25, 0.5, 0.75
IN_LINE, SQUARE = 0.001, math.radians(2.0)  # the long edges in line (and flush) within 1 mm; the boards square within 2 degrees
RESTING, CLEAR = 0.02, 0.03  # m/s under which a board is at rest; m both grippers keep from both boards when they have let go
SIDES = ("left", "right")
STATE: dict = {}


def begin(seed: int | None = None) -> dict:
    """A new trial's state (no simulator needed)."""
    STATE.clear()
    STATE.update(seed=seed, narrowest=None)
    return STATE


@mk.on_reset
def start(seed) -> None:
    begin(seed)


def to_world(name: str, point) -> np.ndarray:
    position, rotation = mk.pose(name)
    return position + rotation @ np.asarray(point, dtype=float)


def seam() -> dict:
    """The seam between the boards' facing ends. ``gaps``: at the two corners (the cherry end's two upright edges, at
    mid-thickness), how far each stands in front of the walnut end's face, along the walnut board (m; negative: past
    it). ``across``: how far the cherry end's middle stands to the side of the walnut end's middle, across the walnut
    board (m). ``turned``: the angle between the two boards' long axes (rad). ``height``: the cherry board's middle over
    the walnut board's (m). ``tilts``: each board's top face from level (rad)."""
    _, walnut = mk.pose("walnut")
    _, cherry = mk.pose("cherry")
    face, out, side = to_world("walnut", (0.0, -BOARD_L / 2, 0.0)), walnut @ np.array([0.0, -1.0, 0.0]), walnut @ np.array([1.0, 0.0, 0.0])
    corners = {edge: to_world("cherry", (sign * BOARD_W / 2, BOARD_L / 2, 0.0)) for edge, sign in (("near", -1.0), ("far", 1.0))}
    middle = to_world("cherry", (0.0, BOARD_L / 2, 0.0))
    long_axes = cherry @ np.array([0.0, 1.0, 0.0]), walnut @ np.array([0.0, 1.0, 0.0])
    return {"gaps": {edge: float((p - face) @ out) for edge, p in corners.items()}, "across": float((middle - face) @ side),
            "turned": float(math.acos(min(1.0, max(-1.0, float(long_axes[0] @ long_axes[1]))))),
            "height": float(mk.pose("cherry")[0][2] - mk.pose("walnut")[0][2]),
            "tilts": {name: float(math.acos(min(1.0, float(mk.pose(name)[1][2, 2])))) for name in NAMES}}  # fmt: skip


def to_board(name: str, point) -> float:
    """How far a point is from a board, the cherry one's dowels included (m; 0 inside)."""
    position, rotation = mk.pose(name)
    local = rotation.T @ (np.asarray(point, dtype=float) - position)
    return min(float(np.linalg.norm(np.maximum(np.maximum(low - local, local - high), 0.0))) for low, high in BOXES[name])


def let_go() -> bool:
    return all(to_board(name, mk.gripper(side)[0]) >= CLEAR for side in SIDES for name in NAMES)


def at_rest() -> bool:
    for name in NAMES:
        linear, angular = mk.velocity(name)
        if float(np.linalg.norm(linear)) >= RESTING or float(np.linalg.norm(angular)) * BOARD_L / 2 >= RESTING:
            return False
    return True


@mk.on_step
def step() -> None:
    s = seam()
    widest = max(s["gaps"].values())
    if STATE.get("narrowest") is None or widest < STATE["narrowest"]:
        STATE["narrowest"] = widest


def level() -> float:
    """The score without the release: see ``score``."""
    s = seam()
    gaps = list(s["gaps"].values())
    if min(gaps) < -OVERLAP or abs(s["height"]) > SAME_HEIGHT or max(s["tilts"].values()) > FLAT:
        return 0.0
    if not (max(gaps) <= TIPS_IN and abs(s["across"]) <= IN_LINE and s["turned"] <= SQUARE):
        return 0.0
    if max(gaps) <= CLOSED:
        return 0.75
    return 0.5 if max(gaps) <= HALF_IN else 0.25


def score(objects) -> float:
    """By how the two boards lie at the end. None of the levels unless both seam gaps are no less than -2 mm (the
    boards do not overlap), the boards' middles are within 2 mm of the same height and both lie flat within 3
    degrees. Both gaps 19 mm or less (the dowels' tips in), the long edges in line within 1 mm and the boards square
    within 2 degrees: a quarter. Both gaps 10 mm or less: a half. Both 0.5 mm or less: three quarters; and both
    grippers 3 cm or more from both boards, everything at rest: 1.0."""
    found = level()
    if found == 0.75 and let_go() and at_rest():
        return 1.0
    return found


def success(objects) -> bool:
    return score(objects) == 1.0


def progress() -> str:
    s = seam()
    return (f"seam (mm): near {1000 * s['gaps']['near']:.2f}, far {1000 * s['gaps']['far']:.2f}; across {1000 * s['across']:.2f} mm, turned {math.degrees(s['turned']):.2f} deg; "
            f"level {score(None):.2f}")  # fmt: skip


def report() -> dict:
    """For logs and probes: the numbers behind the score."""
    out: dict = {"narrowest seam (mm)": None if STATE.get("narrowest") is None else round(1000 * STATE["narrowest"], 3), "bore (mm)": round(1000 * BORE, 2)}
    if mk.MECH.poses:
        s = seam()
        out.update({"seam gaps now (mm)": {k: round(1000 * v, 3) for k, v in s["gaps"].items()}, "across (mm)": round(1000 * s["across"], 3), "turned (deg)": round(math.degrees(s["turned"]), 3),
                    "height (mm)": round(1000 * s["height"], 3), "tilts (deg)": {k: round(math.degrees(v), 3) for k, v in s["tilts"].items()}})  # fmt: skip
        if mk.MECH.grippers:
            out.update({"at rest": at_rest(), "let go": let_go(), "score": score(None)})
    return out
