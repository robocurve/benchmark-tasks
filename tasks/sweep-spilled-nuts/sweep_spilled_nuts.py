"""On the station table: ten steel M8 hex nuts lying flat in two piles of
five, one far out to the right and one far out to the left, each beyond the
other arm's reach. A plastic dustpan stands between them with its mouth
facing away from the robot and a crosswise handle bar toward the robot,
where either hand can take it. A hand brush (a wooden block) lies to the
right, near the robot.

Neither arm can sweep both piles: one hand holds the pan while the other
sweeps its own pile in, then they swap the brush and the pan, and finally
one lifts the pan off the table (nuts left on the lip fall off).

Demo: the left holds the pan while the right sweeps the right pile in (a
pass toward the middle, then one into the pan) and puts the brush down in
front of the pan. The left lets go, the right takes the handle, the left
picks the brush up and sweeps the left pile in the same way, and the right
lifts the pan.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  
import yam_arena.constants 
import yam_arena.demo  
import yam_arena.task_file  

SCENE = "station"
TOP = yam_arena.task_file.TABLE_TOP[SCENE]
OPEN = yam_arena.constants.FINGER_OPEN
CLOSED = yam_arena.constants.FINGER_CLOSED

PAN_XY = (0.20, 0.0)
PAN_YAW = math.pi / 2
PAN_INSIDE = (0.20, 0.12)  # floor width, depth (m)
PAN_FLOOR = 0.002
PAN_WALL = 0.004
PAN_BACK_HEIGHT = 0.04
PAN_SIDE_HEIGHT = 0.03
PAN_LIP_LENGTH = 0.025

PAN_HANDLE = (0.0, 0.12, 0.045)  
PAN_HANDLE_SIZE = (0.14, 0.02, 0.02)
PAN_COLOR = (0.15, 0.35, 0.75)

NUT_FLATS = 0.013
NUT_HEIGHT = 0.0065
NUT_XY = (
    
    (0.30, -0.17), (0.315, -0.215), (0.33, -0.165), (0.345, -0.235),
    (0.36, -0.19),
    # Left pile: out of the right arm's reach.
    (0.305, 0.215), (0.315, 0.175), (0.33, 0.235), (0.345, 0.19),
    (0.36, 0.16),
)
NUT_COLOR = (0.70, 0.70, 0.72)

BRUSH_SIZE = (0.14, 0.04, 0.03)  
BRUSH_COLOR = (0.55, 0.38, 0.22)

MAX_PAN_TILT_DEG = 30.0
MIN_LIFT = 0.05  
UNLIFTED_FACTOR = 0.5

CARRIED_HEIGHT = 0.02

AT_START = 0.002

# Demo. Positions are for the right arm: the left mirrors them in y.
HANDLE_GRIP = (0.08, -0.03, TOP + 0.045)  # on the bar, right of the middle
HANDLE_GRIP_OPEN = -0.025  # fingers 5 cm apart: clear of the back wall
GRIP_Z = TOP + 0.02  # holding the brush with its underside on the table
CARRY_Z = TOP + 0.045  # brush 2.5 cm up: clear of the nuts
CLEAR_Z = TOP + 0.08
# While held, the brush's centre sits this far from the grip, toward -y at
# yaw 0 (turning with the grip), and goes back under it when let go
# (measured in the simulator).
BRUSH_OFFSET = 0.016
# Sideways pass, brush lengthwise (along x): its centre's x and its start and
# end y, from 2 cm outside the pile to where its face is 4.5 cm from the
# middle, in line with the middle of the mouth.
SIDE_PASS_X = 0.355
SIDE_PASS_Y = (-0.28, -0.065)
# Pass into the pan, brush crosswise: its centre's y, and x where it goes
# down 1 cm beyond the nuts (the left arm's reach), reaches the lip, has
# crossed it, and stops (its front face 6 cm onto the 12 cm floor).
INTO_PASS_Y = -0.02
INTO_PASS_X = (0.40, 0.32, 0.30, 0.22)
LIP_CLEARANCE = 0.004  # brush raised over the lip and floor (m)
PUT_DOWN_X = 0.33  # brush set down just beyond the lip
BRUSH_OUT_LABEL = "carry the brush out of the pan"
LET_GO_LABEL = "clear of the brush"
LIFT_HEIGHT = 0.08


def _pan_shapes() -> list:
    """Floor, sloped front lip, back and side walls, and handle."""
    half_x, half_y = PAN_INSIDE[0] / 2, PAN_INSIDE[1] / 2
    side_x = half_x + PAN_WALL / 2
    side_y = PAN_INSIDE[1] + PAN_WALL
    back_y = half_y + PAN_WALL / 2
    lip_thickness = 0.001
    # The lip rises from the table to the floor's top edge over 2.5 cm, so a
    # swept nut slides up it instead of catching on the floor's edge.
    lip_low = (0.0, -half_y - PAN_LIP_LENGTH, lip_thickness / 2 + 0.0001)
    lip_high = (0.0, -half_y, PAN_FLOOR - lip_thickness / 2)
    shapes = [
        bodykit.BoxShape((0.0, 0.0, PAN_FLOOR / 2),
                         (PAN_INSIDE[0], PAN_INSIDE[1], PAN_FLOOR),
                         PAN_COLOR),
        bodykit.plate(lip_low, lip_high, PAN_INSIDE[0], lip_thickness,
                      (0.0, 0.0, 1.0), PAN_COLOR),
        bodykit.BoxShape((0.0, back_y, PAN_BACK_HEIGHT / 2),
                         (PAN_INSIDE[0] + 2 * PAN_WALL, PAN_WALL,
                          PAN_BACK_HEIGHT),
                         PAN_COLOR),
        bodykit.BoxShape((-side_x, 0.0, PAN_SIDE_HEIGHT / 2),
                         (PAN_WALL, side_y, PAN_SIDE_HEIGHT), PAN_COLOR),
        bodykit.BoxShape((side_x, 0.0, PAN_SIDE_HEIGHT / 2),
                         (PAN_WALL, side_y, PAN_SIDE_HEIGHT), PAN_COLOR),
        bodykit.BoxShape(PAN_HANDLE, PAN_HANDLE_SIZE, PAN_COLOR),
    ]
    # Two posts join the bar's ends to the back wall, clear of both grips.
    for x in (-0.06, 0.06):
        wall_top = (x, half_y + PAN_WALL, PAN_BACK_HEIGHT - 0.005)
        bar = (x, PAN_HANDLE[1], PAN_HANDLE[2])
        shapes.append(bodykit.segment(wall_top, bar, 0.01, PAN_COLOR))
    return shapes


def _nut_shapes() -> list:
    """A hexagon as three overlapping boxes turned 0, 60 and 120 degrees."""
    side = NUT_FLATS / math.sqrt(3)
    return [
        bodykit.BoxShape((0.0, 0.0, NUT_HEIGHT / 2),
                         (NUT_FLATS, side, NUT_HEIGHT), NUT_COLOR,
                         yaw=i * math.pi / 3)
        for i in range(3)
    ]


PAN = bodykit.Body.build("dustpan", pos=(*PAN_XY, TOP), mass=0.15,
                         shapes=_pan_shapes(), yaw=PAN_YAW,
                         friction=(0.3, 0.25))
NUTS = [
    bodykit.Body.build(f"nut_{i}", pos=(x, y, TOP), mass=0.005,
                       shapes=_nut_shapes(), friction=(0.4, 0.35))
    for i, (x, y) in enumerate(NUT_XY)
]
# Lengthwise along x, to the right of the pan and nearer the robot than the
# right pile.
BRUSH = bodykit.Body.build(
    "brush", pos=(0.16, -0.20, TOP), mass=0.08,
    shapes=[bodykit.BoxShape((0.0, 0.0, BRUSH_SIZE[2] / 2), BRUSH_SIZE,
                             BRUSH_COLOR)],
    friction=(0.4, 0.3))
OBJECTS = [PAN, *NUTS, BRUSH]


def _in_pan(nut, pan) -> bool:
    """Whether the nut's centre is over the pan's floor, inside its walls."""
    local = pan.to_local(nut.pos)
    inside_x = abs(local[0]) < PAN_INSIDE[0] / 2
    inside_y = abs(local[1]) < PAN_INSIDE[1] / 2
    on_floor = -0.005 < local[2] < PAN_SIDE_HEIGHT
    return bool(inside_x and inside_y and on_floor)


def _lifted(pan) -> bool:
    """Whether the pan's lowest point is at least 5 cm above the table."""
    return bool(pan.low[2] - TOP >= MIN_LIFT)


class CarriedNuts:
    """Nuts lifted off the table outside the pan during the current trial.

    The embodiment calls ``score()`` after every step but has no reset hook,
    so a new trial is recognised by every nut being back at its start.

    Attributes:
        names: names of the nuts carried so far.
    """

    def __init__(self) -> None:
        self.names: set[str] = set()

    def update(self, objects: dict) -> None:
        """Forget the last trial at a fresh start, then mark carried nuts."""
        if all(_at_start(objects[nut.name], nut) for nut in NUTS):
            self.names.clear()
        pan = objects["dustpan"]
        for nut in NUTS:
            state = objects[nut.name]
            raised = state.low[2] - TOP > CARRIED_HEIGHT
            if raised and not _in_pan(state, pan):
                self.names.add(nut.name)


def _at_start(state, nut) -> bool:
    """Whether a nut is still where it started, on the table."""
    shift = math.dist(state.pos[:2], nut.pos[:2])
    return bool(shift < AT_START and state.low[2] - TOP < AT_START)


# Kept across score() calls: which nuts this trial has carried.
CARRIED = CarriedNuts()


def score(objects: dict) -> float:
    """Share of nuts in the pan, halved unless the pan is lifted.

    Nuts that were carried rather than swept count 0. The score is 0 if the
    pan is tilted more than 30 degrees (dropped, upside down or tipped).
    """
    CARRIED.update(objects)
    pan = objects["dustpan"]
    if math.degrees(pan.tilt()) > MAX_PAN_TILT_DEG:
        return 0.0
    in_pan = sum(
        _in_pan(objects[nut.name], pan) and nut.name not in CARRIED.names
        for nut in NUTS
    )
    factor = 1.0 if _lifted(pan) else UNLIFTED_FACTOR
    return factor * in_pan / len(NUTS)


def success(objects: dict) -> bool:
    """Every nut in the pan, and the pan lifted, must hold for 1 s."""
    return score(objects) == 1.0


def _crosswise_yaw(sign: int) -> float:
    """Grip yaw closing the fingers along x: +90 degrees right, -90 left.

    The wrist reaches the middle of the table only turned toward its own
    side. ``sign`` is -1 for the right arm and +1 for the left.
    """
    return -sign * math.pi / 2


def _take_handle(sign: int) -> list[yam_arena.demo.Waypoint]:
    """Come down on this arm's side of the handle bar and close on it."""
    grip = (HANDLE_GRIP[0], -sign * HANDLE_GRIP[1], HANDLE_GRIP[2])
    above = (grip[0], grip[1], TOP + 0.10)
    yaw = _crosswise_yaw(sign)
    waypoint = yam_arena.demo.Waypoint
    return [
        waypoint(2.0, above, HANDLE_GRIP_OPEN, "above the handle", yaw),
        waypoint(1.5, grip, HANDLE_GRIP_OPEN, "round the handle", yaw),
        waypoint(1.0, grip, CLOSED, "take the handle", yaw),
    ]


def _grip_xy(brush_x: float, brush_y: float,
             yaw: float) -> tuple[float, float]:
    """Where to hold the grip so the held brush's centre is at x, y."""
    return (brush_x - BRUSH_OFFSET * math.sin(yaw),
            brush_y + BRUSH_OFFSET * math.cos(yaw))


def _sweep(sign: int) -> list[yam_arena.demo.Waypoint]:
    """Brush in hand: sweep this side's pile into the pan, put it down.

    One pass pushes the pile sideways, in line with the mouth, then the brush
    turns crosswise and pushes the nuts over the lip onto the floor.
    """
    side_x = SIDE_PASS_X
    side_start, side_end = (-sign * y for y in SIDE_PASS_Y)
    into_y = -sign * INTO_PASS_Y
    beyond, lip, over_lip, end = INTO_PASS_X
    lengthwise, crosswise = 0.0, _crosswise_yaw(sign)
    low, raised = GRIP_Z + LIP_CLEARANCE, TOP + 0.035
    # Duration, the brush's centre x, y, the grip's z, label, yaw.
    steps = [
        (2.0, side_x, side_start, CARRY_Z, "outside the pile", lengthwise),
        (1.0, side_x, side_start, GRIP_Z, "brush down", lengthwise),
        (3.0, side_x, side_end, GRIP_Z, "push the pile toward the middle",
         lengthwise),
        (1.0, side_x, side_end, CARRY_Z, "brush up", lengthwise),
        (1.5, side_x, side_end, CARRY_Z, "turn the brush crosswise",
         crosswise),
        (1.5, beyond, into_y, CARRY_Z, "beyond the nuts", crosswise),
        (1.0, beyond, into_y, GRIP_Z, "brush down", crosswise),
        (3.0, lip, into_y, GRIP_Z, "push the nuts to the lip", crosswise),
        (0.5, over_lip, into_y, low, "over the lip", crosswise),
        (2.0, end, into_y, low, "into the pan", crosswise),
        (1.0, end, into_y, raised, "lift the brush", crosswise),
    ]
    waypoint = yam_arena.demo.Waypoint
    sweeping = [
        waypoint(duration, (*_grip_xy(x, y, yaw), z), CLOSED, label, yaw)
        for duration, x, y, z, label, yaw in steps
    ]
    # Once let go, the brush lies under the grip: put it down by the grip.
    return [
        *sweeping,
        waypoint(1.5, (PUT_DOWN_X, into_y, raised), CLOSED, BRUSH_OUT_LABEL,
                 crosswise),
        waypoint(1.0, (PUT_DOWN_X, into_y, GRIP_Z), CLOSED, "set it down",
                 crosswise),
        waypoint(1.0, (PUT_DOWN_X, into_y, GRIP_Z), OPEN, "let go",
                 crosswise),
        waypoint(1.0, (PUT_DOWN_X, into_y, CLEAR_Z), OPEN, LET_GO_LABEL,
                 crosswise),
    ]


def _right_plan(hold_s: float) -> list[yam_arena.demo.Waypoint]:
    """Sweep the right pile, take the handle, hold for ``hold_s``, lift."""
    brush = (BRUSH.pos[0], BRUSH.pos[1])
    grip = (HANDLE_GRIP[0], HANDLE_GRIP[1], HANDLE_GRIP[2])
    lifted = (grip[0], grip[1], grip[2] + LIFT_HEIGHT)
    yaw = _crosswise_yaw(-1)
    waypoint = yam_arena.demo.Waypoint
    return [
        waypoint(2.0, (*brush, CLEAR_Z), OPEN, "above the brush"),
        waypoint(1.5, (*brush, GRIP_Z), OPEN, "round the brush"),
        waypoint(1.0, (*brush, GRIP_Z), CLOSED, "take the brush"),
        waypoint(1.0, (*brush, CARRY_Z), CLOSED, "lift the brush"),
        *_sweep(-1),
        *_take_handle(-1),
        waypoint(hold_s, grip, CLOSED, "hold the pan down", yaw),
        waypoint(2.0, lifted, CLOSED, "lift the pan", yaw),
        waypoint(3.0, lifted, CLOSED, "hold it up", yaw),
    ]


def _left_plan(hold_s: float,
               wait_s: float) -> list[yam_arena.demo.Waypoint]:
    """Hold the pan for ``hold_s``, step aside for ``wait_s``, then sweep.

    The brush is where the right arm put it down, in front of the pan.
    """
    grip = (HANDLE_GRIP[0], -HANDLE_GRIP[1], HANDLE_GRIP[2])
    up = (grip[0], grip[1], TOP + 0.10)
    aside = (0.22, 0.20, TOP + 0.10)
    brush = (PUT_DOWN_X, INTO_PASS_Y)
    yaw = _crosswise_yaw(1)
    waypoint = yam_arena.demo.Waypoint
    return [
        *_take_handle(1),
        waypoint(hold_s, grip, CLOSED, "hold the pan down", yaw),
        waypoint(1.0, grip, HANDLE_GRIP_OPEN, "let go of the handle", yaw),
        waypoint(1.0, up, HANDLE_GRIP_OPEN, "up off the handle", yaw),
        waypoint(1.5, aside, OPEN, "move aside", yaw),
        waypoint(wait_s, aside, OPEN, "wait for the right hand", yaw),
        waypoint(2.0, (*brush, CLEAR_Z), OPEN, "above the brush", yaw),
        waypoint(1.5, (*brush, GRIP_Z), OPEN, "round the brush", yaw),
        waypoint(1.0, (*brush, GRIP_Z), CLOSED, "take the brush", yaw),
        waypoint(1.0, (*brush, CARRY_Z), CLOSED, "lift the brush", yaw),
        *_sweep(1),
    ]


def _time_until(plan: list[yam_arena.demo.Waypoint], label: str) -> float:
    """Seconds from the plan's start to the end of the waypoint ``label``."""
    elapsed = 0.0
    for waypoint in plan:
        elapsed += waypoint.duration
        if waypoint.label == label:
            return elapsed
    raise ValueError(f"no waypoint {label!r}")


def demo() -> dict:
    """Waypoints per arm, timed so the hands swap the brush and the pan.

    The left lets go half a second after the brush is out of the pan, takes
    the brush half a second after the right has the handle, and the right
    lifts half a second after the left has let go of the brush.
    """
    minimum = 0.1  # a waypoint needs some duration
    right_out_s = _time_until(_right_plan(minimum), BRUSH_OUT_LABEL)
    right_take_s = _time_until(_right_plan(minimum), "take the handle")
    left_take_s = _time_until(_left_plan(minimum, minimum),
                              "take the handle")
    hold_s = max(minimum, right_out_s + 0.5 - left_take_s)
    aside_s = _time_until(_left_plan(hold_s, minimum), "move aside")
    wait_s = max(minimum, right_take_s + 0.5 - aside_s)
    left = _left_plan(hold_s, wait_s)
    left_done_s = _time_until(left, LET_GO_LABEL)
    right_hold_s = max(minimum, left_done_s + 0.5 - right_take_s)
    return {"left": left, "right": _right_plan(right_hold_s)}


def main() -> None:
    """Offline check, no simulator: load the file and print its objects."""
    task = yam_arena.task_file.load(__file__)
    for obj in task.objects:
        print(f"{obj.name}: pos {obj.pos}, size {obj.size}, mass {obj.mass}")
    for side, plan in task.demo().items():
        duration = sum(waypoint.duration for waypoint in plan)
        print(f"{side} arm: {len(plan)} waypoints, {duration:.1f} s")


if __name__ == "__main__":
    main()
