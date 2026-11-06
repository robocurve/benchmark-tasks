"""On the station table: a gas line with a quarter-turn ball valve, and a lockout pin parked upright in a holder.

The valve's handle points along the pipe (open). Locking out the line means: close the valve (turn the handle a quarter
turn until it lies across the pipe, against its stop, pointing at the robots), then put the lockout pin down through the
hole in the handle and the hole in the yellow lockout tab under it, so the handle cannot be turned back.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from labkit import Joint, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL = (0.62, 0.63, 0.66)
YELLOW = (0.95, 0.78, 0.10)
BRASS = (0.78, 0.62, 0.28)
RED = (0.85, 0.12, 0.12)
DARK = (0.18, 0.19, 0.21)

# -- the gas line and valve (one fixed part; frame origin on the table under the valve centre) -------------------------
VX, VY = 0.30, -0.02
PIPE_Z, PIPE_R, PIPE_L = 0.060, 0.016, 0.30  # the pipe runs along y
BODY = 0.060  # brass valve body, a 6 cm cube round the pipe
STEM_TOP = 0.105
HANDLE_Z = 0.1075  # handle mid-plane; it is 5 mm thick: underside 0.105, top 0.110
HANDLE_T, HANDLE_W, HANDLE_L = 0.005, 0.022, 0.130
HOLE = 0.014  # square lock holes in the handle and the tab, 14 mm (3 mm of play round the 8 mm pin)
HOLE_R = 0.045  # the handle's lock hole is 4.5 cm from the pivot
GRIP_R0, GRIP_R1 = 0.075, 0.130  # red grip sleeve along the handle's outer part
SLEEVE_W, SLEEVE_T = 0.026, 0.012
TAB_Z0, TAB_T, TAB = 0.097, 0.004, 0.036  # the tab is a 3.6 cm plate, 4 mm thick, 4 mm under the closed handle
CLOSED = math.pi / 2  # the handle's stop: turned from +y (along the pipe) to -x (across it, towards the robots)


def _frame(cx, cy, z, t, outer, hole, colour):
    """A square plate with a square hole in the middle, as four boxes: (centre, size, colour)."""
    side = (outer - hole) / 2
    off = (outer + hole) / 4
    return [
        ((cx, cy + off, z), (outer, side, t), colour),
        ((cx, cy - off, z), (outer, side, t), colour),
        ((cx + off, cy, z), (side, hole, t), colour),
        ((cx - off, cy, z), (side, hole, t), colour),
    ]


# The handle at its zero (open) position lies along +y from the pivot, with the lock hole at y = HOLE_R.
_rail = (HANDLE_W - HOLE) / 2
HANDLE_BOXES = [
    ((0.0, (HOLE_R - HOLE / 2) / 2, HANDLE_Z), (HANDLE_W, HOLE_R - HOLE / 2, HANDLE_T), STEEL),
    ((HOLE / 2 + _rail / 2, HOLE_R, HANDLE_Z), (_rail, HOLE, HANDLE_T), STEEL),
    ((-HOLE / 2 - _rail / 2, HOLE_R, HANDLE_Z), (_rail, HOLE, HANDLE_T), STEEL),
    (((0.0, (HOLE_R + HOLE / 2 + HANDLE_L) / 2, HANDLE_Z)), (HANDLE_W, HANDLE_L - HOLE_R - HOLE / 2, HANDLE_T), STEEL),
    ((0.0, (GRIP_R0 + GRIP_R1) / 2, HANDLE_Z), (SLEEVE_W, GRIP_R1 - GRIP_R0, SLEEVE_T), RED),
]
HANDLE = Joint(
    "handle", kind="hinge", axis="Z", anchor=(0.0, 0.0, HANDLE_Z), lower=0.0, upper=CLOSED, initial=0.0,
    stiffness=0.0, damping=0.5, target=0.0, mass=0.12,
    boxes=HANDLE_BOXES, cylinders=[((0.0, 0.0, HANDLE_Z), 0.012, HANDLE_T, STEEL)],
)

# The tab sits under the closed handle's hole: at (-HOLE_R, 0) in the valve frame.
TAB_X = -HOLE_R
VALVE = part(
    "valve", pos=(VX, VY, TOP), fixed=True, friction=0.8,
    boxes=[
        ((0.0, 0.0, PIPE_Z), (BODY, BODY, BODY), BRASS),
        ((0.0, PIPE_L / 2 - 0.02, (PIPE_Z - PIPE_R) / 2), (0.04, 0.03, PIPE_Z - PIPE_R), DARK),  # pipe supports
        ((0.0, -PIPE_L / 2 + 0.02, (PIPE_Z - PIPE_R) / 2), (0.04, 0.03, PIPE_Z - PIPE_R), DARK),
        ((-BODY / 2 + 0.001, 0.0, (PIPE_Z + BODY / 2 + TAB_Z0) / 2), (0.006, TAB, TAB_Z0 - PIPE_Z - BODY / 2), YELLOW),
        *_frame(TAB_X, 0.0, TAB_Z0 + TAB_T / 2, TAB_T, TAB, HOLE, YELLOW),
    ],
    cylinders=[
        ((0.0, 0.0, PIPE_Z), PIPE_R, PIPE_L, YELLOW, "Y"),
        ((0.0, 0.0, (PIPE_Z + BODY / 2 + STEM_TOP) / 2), 0.007, STEM_TOP - PIPE_Z - BODY / 2, STEEL),
    ],
    joints=[HANDLE],
)

# -- the lockout pin and its holder ------------------------------------------------------------------------------------
# Pin frame: origin at the underside of its head, on the axis. A red T head (3 x 1 x 1 cm) on an 8 mm steel shaft 2.4 cm
# long, ending in a 5 mm tip 6 mm long: 3 cm below the head in all.
SHAFT_R, SHAFT_L, TIP_R, TIP_L = 0.004, 0.024, 0.0025, 0.006
PIN_LEN = SHAFT_L + TIP_L
HEAD = (0.030, 0.010, 0.010)
HOLDER_X, HOLDER_Y, HOLDER_H = 0.18, -0.16, 0.032
PIN = part(
    "pin", pos=(HOLDER_X, HOLDER_Y, TOP + HOLDER_H), mass=0.04, friction=0.8,
    boxes=[((0.0, 0.0, HEAD[2] / 2), HEAD, RED)],
    cylinders=[
        ((0.0, 0.0, -SHAFT_L / 2), SHAFT_R, SHAFT_L, STEEL),
        ((0.0, 0.0, -SHAFT_L - TIP_L / 2), TIP_R, TIP_L, STEEL),
    ],
)
HOLDER = part(
    "holder", pos=(HOLDER_X, HOLDER_Y, TOP), fixed=True, friction=0.8,
    boxes=_frame(0.0, 0.0, HOLDER_H / 2, HOLDER_H, 0.036, HOLE, DARK),
)

OBJECTS = [VALVE, HOLDER, PIN]

# -- scoring ----------------------------------------------------------------------------------------------------------
CLOSED_TOL = math.radians(5.0)
HOLE_WORLD = (VX + TAB_X, VY)  # both holes, with the handle closed
AXIS_TOL = 0.004  # the pin's axis within 4 mm of the holes' centre (3 mm is the play)
SEAT_TOL = 0.003  # head underside within 3 mm of the handle's top
TILT_TOL = 10.0
HANDLE_UNDER = TOP + HANDLE_Z - HANDLE_T / 2
HANDLE_TOP = TOP + HANDLE_Z + HANDLE_T / 2
TAB_UNDER = TOP + TAB_Z0
SUCCESS_HOLD_S = 1.0


def _closed(objects) -> bool:
    return objects["valve"].joints["handle"] >= CLOSED - CLOSED_TOL


def _pin(objects):
    """(axis offset from the holes' centre in m, tip height, head-underside height, tilt in degrees)."""
    p = objects["pin"]
    head = p.to_world((0.0, 0.0, 0.0))
    tip = p.to_world((0.0, 0.0, -PIN_LEN))
    off = max(math.hypot(head[0] - HOLE_WORLD[0], head[1] - HOLE_WORLD[1]),
              math.hypot(tip[0] - HOLE_WORLD[0], tip[1] - HOLE_WORLD[1]))
    return off, float(tip[2]), float(head[2]), p.tilt_deg()


def score(objects):
    """0 if the handle is not closed (more than 5 degrees short of its stop). 0.25 if it is closed. 0.5 if it is closed
    and the pin's tip has gone down through the handle's hole (below the handle's underside, axis within 4 mm of the
    hole's centre). 1.0 if it is closed and the pin is seated: through both holes (tip below the tab's underside), head
    resting on the handle (its underside within 3 mm of the handle's top), axis within 4 mm of the holes' centre and
    tilted no more than 10 degrees."""
    if not _closed(objects):
        return 0.0
    off, tip_z, head_z, tilt = _pin(objects)
    if off > AXIS_TOL:
        return 0.25
    seated = tip_z < TAB_UNDER and head_z - HANDLE_TOP <= SEAT_TOL and tilt <= TILT_TOL
    if seated:
        return 1.0
    return 0.5 if tip_z < HANDLE_UNDER else 0.25


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """Handle angle short of closed (rad) plus the pin's offset from the holes (m)."""
    off, *_ = _pin(objects)
    return max(0.0, CLOSED - objects["valve"].joints["handle"]) + off


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: grips the red sleeve across its width and carries it round a quarter-circle (wrist turning with it) to the
# stop, lets go and parks. Right arm: waits until the left arm is clear, takes the pin from its holder by the head, carries it over the holes, lowers
# it until the head is 3 mm above the handle and lets it drop the rest of the way.
TIP_ABOVE_TCP = 0.013
GRIP_R = 0.105
SLEEVE_GRIP_Z = TOP + HANDLE_Z - SLEEVE_T / 2 + 0.003 + TIP_ABOVE_TCP  # pad tips 3 mm above the sleeve's underside
SLEEVE_HOVER_Z = TOP + HANDLE_Z + 0.06
FINGER_WIDE = -0.03
ARC_STEPS = 6
LEFT_PARK = (0.15, 0.25, TOP + 0.14)
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)
PIN_GRIP_Z = TOP + HOLDER_H + 0.004 + TIP_ABOVE_TCP  # pad tips 4 mm above the head's underside
PIN_TRAVEL_Z = HANDLE_TOP + PIN_LEN + 0.015 + 0.004 + TIP_ABOVE_TCP  # tip 1.5 cm above the handle
PIN_ENTRY_Z = HANDLE_TOP + PIN_LEN + 0.003 + 0.004 + TIP_ABOVE_TCP  # tip 3 mm above the handle
PIN_DROP_Z = HANDLE_TOP + 0.003 + 0.004 + TIP_ABOVE_TCP

PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    ("valve body", (VX, VY, TOP + PIPE_Z), (BODY, BODY, BODY)),
    ("pipe", (VX, VY, TOP + PIPE_Z), (2 * PIPE_R, PIPE_L, 2 * PIPE_R)),
    ("tab", (VX + TAB_X, VY, TOP + TAB_Z0 + TAB_T / 2), (TAB, TAB, TAB_T)),
    ("holder", (HOLDER_X, HOLDER_Y, TOP + HOLDER_H / 2), (0.036, 0.036, HOLDER_H)),
]


def _grip_point(theta):
    return (VX - GRIP_R * math.sin(theta), VY + GRIP_R * math.cos(theta))


def _dur(wps):
    return sum(w.duration for w in wps)


def demo():
    p0 = _grip_point(0.0)
    y0 = -math.pi / 2  # fingers close across the handle: along x while it lies along y
    left = [
        Waypoint(2.0, (*p0, SLEEVE_HOVER_Z), FINGER_WIDE, "above the red sleeve", y0),
        Waypoint(1.5, (*p0, SLEEVE_GRIP_Z), FINGER_WIDE, "down round it", y0),
        Waypoint(0.8, (*p0, SLEEVE_GRIP_Z), FINGER_CLOSED, "grip", y0),
    ]
    for i in range(1, ARC_STEPS + 1):
        th = CLOSED * i / ARC_STEPS
        left.append(Waypoint(0.9, (*_grip_point(th), SLEEVE_GRIP_Z), FINGER_CLOSED, "turn the handle closed", y0 + th))
    p1, y1 = _grip_point(CLOSED), y0 + CLOSED
    left += [
        Waypoint(0.8, (*p1, SLEEVE_GRIP_Z), FINGER_WIDE, "let go", y1),
        Waypoint(1.0, (*p1, SLEEVE_HOVER_Z), FINGER_WIDE, "up", y1),
        Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"),
    ]
    t_free = _dur(left)
    hx, hy = HOLE_WORLD
    right = [
        Waypoint(t_free, RIGHT_PARK, FINGER_WIDE, "wait for the handle to be closed and the left arm clear", 0.0),
        Waypoint(2.0, (HOLDER_X, HOLDER_Y, PIN_TRAVEL_Z), FINGER_WIDE, "above the pin", 0.0),
        Waypoint(1.5, (HOLDER_X, HOLDER_Y, PIN_GRIP_Z), FINGER_WIDE, "down round its head", 0.0),
        Waypoint(0.8, (HOLDER_X, HOLDER_Y, PIN_GRIP_Z), FINGER_CLOSED, "grip the head", 0.0),
        Waypoint(1.5, (HOLDER_X, HOLDER_Y, PIN_TRAVEL_Z), FINGER_CLOSED, "lift it out", 0.0),
    ]
    right += [
        Waypoint(2.0, (hx, hy, PIN_TRAVEL_Z), FINGER_CLOSED, "over the holes", 0.0),
        Waypoint(1.0, (hx, hy, PIN_TRAVEL_Z), FINGER_CLOSED, "settle over the holes", 0.0),
        Waypoint(1.5, (hx, hy, PIN_ENTRY_Z), FINGER_CLOSED, "tip just above the handle's hole", 0.0),
        Waypoint(1.0, (hx, hy, PIN_ENTRY_Z), FINGER_CLOSED, "let the pin stop swinging", 0.0),
        Waypoint(3.0, (hx, hy, PIN_DROP_Z), FINGER_CLOSED, "slowly down through both holes", 0.0),
        Waypoint(0.8, (hx, hy, PIN_DROP_Z), FINGER_WIDE, "let go", 0.0),
        Waypoint(1.0, (hx, hy, PIN_TRAVEL_Z), FINGER_WIDE, "up", 0.0),
        Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"),
    ]
    t_end = max(_dur(left), _dur(right)) + 2.0
    left.append(Waypoint(t_end - _dur(left), LEFT_PARK, FINGER_OPEN, "hold"))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "hold"))
    return {"left": left, "right": right}
