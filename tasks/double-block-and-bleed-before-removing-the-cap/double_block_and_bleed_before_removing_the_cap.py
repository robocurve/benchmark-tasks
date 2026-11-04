"""On the station table: a yellow pipe runs across the far side of the reach from the robot's left to its right, on
two dark supports. Three valves sit on it. The block valves near the left and right ends are brass blocks, each with
a dark handle bar standing on top that slides along the body away from the arms: back is OPEN, forward is CLOSED. The
smaller bleed valve in the middle has a red handle that slides the same way: back is SHUT, forward is OPEN. Handles
stay where they are put. In front of the pipe, a little left of the middle, a grey seat holds a red test cap resting
on it. Behind the pipe, between the bleed and the left valve, the pressure lamp sits on a short post, lit red at the
start. A yellow tray sits in front of the left valve.

Run it:  inspect-robots run --embodiment yam_arena -E task=double_block_and_bleed_before_removing_the_cap.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from benchkit import Slider, Tool, ToolState, on_reset, on_step, tool  # noqa: E402
from plantkit import (  # noqa: E402
    DARK, GREY, HZ, LAMP_OFF, LAMP_RED, RED, YELLOW, Clock, Lamp, distance_xy, over, pushed, tilt_deg,
)
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Take the test cap off the spool: close the left valve and the right valve, open the bleed valve, wait until the "
    "pressure light goes out, then lift the cap off and put it in the tray."
)

BRASS = (0.72, 0.58, 0.28)

# -- the pipe: a spool along y at PIPE_X, its axis PIPE_Z up, in fixed lengths between the valve bodies ---------------
PIPE_X = 0.300
PIPE_Z = TOP + 0.045  # the spool's axis
PIPE_SECTION = 0.030
SPOOL_END = 0.200  # the spool runs from y -0.20 to +0.20
GAP = 0.001  # between a fixed pipe end and the loose valve body beside it
SUPPORT_Y = 0.080
SUPPORT = (0.040, 0.030, PIPE_Z - PIPE_SECTION / 2 - TOP)  # from the table up to the pipe's underside

# -- the valves: loose 3 kg tools, each a block on a base plate with a handle that slides +x along its top -----------
BLOCK_Y = 0.160  # the block valves: left at +y, right at -y
BLOCK = 0.050  # the block valves' body, a cube
BLEED = 0.040  # the bleed valve's body
HANDLE = (0.030, 0.020, 0.040)  # x, y, z: 4 cm tall, so its top stays within 2.2 cm of the TCP (see FINGER_X)
HANDLE_TRAVEL = 0.030
PLATE_MARGIN = 0.040  # each body's base plate is this much wider than the body, 2 cm a side, so a push can't tip it
THROWN_AT = 0.7  # past this fraction of its travel a block valve is closed, the bleed open


def _valve(name: str, y: float, body: float, handle_colour) -> Tool:
    """A valve body centred on the pipe's axis, standing on a base plate down to the table that is 2 cm wider than the
    body all round. Its handle rests flush with the body's near face and slides forward (+x) along the top; no spring,
    so it stays where it is pushed."""
    plate = (body + PLATE_MARGIN, body + PLATE_MARGIN, PIPE_Z - body / 2 - TOP)
    return tool(
        name,
        pos=(PIPE_X, y, PIPE_Z),
        boxes=[
            ((0.0, 0.0, 0.0), (body, body, body), BRASS),
            ((0.0, 0.0, -body / 2 - plate[2] / 2), plate, DARK),
        ],
        sliders=[Slider("handle", rest=(-body / 2 + HANDLE[0] / 2, 0.0, body / 2 + HANDLE[2] / 2), size=HANDLE,
                        axis="X", lower=0.0, upper=HANDLE_TRAVEL, color=handle_colour, rate=0.0, damping=8.0,
                        mass=0.02)],
        mass=3.0,  # heavy enough to stay put when pushed; taped down on a real bench
    )


LEFT_VALVE = _valve("left_valve", BLOCK_Y, BLOCK, DARK)
RIGHT_VALVE = _valve("right_valve", -BLOCK_Y, BLOCK, DARK)
BLEED_VALVE = _valve("bleed_valve", 0.0, BLEED, RED)
VALVES = {v.name: v for v in (LEFT_VALVE, RIGHT_VALVE, BLEED_VALVE)}


def body_top(spec: Tool) -> float:
    """World z of a valve body's top face, the surface its handle slides over."""
    return spec.pos[2] + spec.boxes[0][1][2] / 2


def handle_face(spec: Tool) -> float:
    """World x of a handle's near face at rest."""
    s = spec.sliders[0]
    return spec.pos[0] + s.rest[0] - s.size[0] / 2


def _pipe(name: str, y0: float, y1: float) -> Box:
    return Box(name, pos=(PIPE_X, (y0 + y1) / 2, PIPE_Z), size=(PIPE_SECTION, y1 - y0, PIPE_SECTION), color=YELLOW)


PIPES = [
    _pipe("spool_left_end", BLOCK_Y + BLOCK / 2 + GAP, SPOOL_END),
    _pipe("spool_left", BLEED / 2 + GAP, BLOCK_Y - BLOCK / 2 - GAP),
    _pipe("spool_right", -(BLOCK_Y - BLOCK / 2 - GAP), -(BLEED / 2 + GAP)),
    _pipe("spool_right_end", -SPOOL_END, -(BLOCK_Y + BLOCK / 2 + GAP)),
]

# -- the test port, its cap, the lamp and the tray -----------------------------------------------------------------
SEAT_XY = (0.230, 0.040)
SEAT = (0.050, 0.050, 0.020)
SEAT_TOP = TOP + SEAT[2]
CAP_R = 0.020
CAP_H = 0.030
CAP_MASS = 0.06
CAP = Cylinder("cap", pos=(*SEAT_XY, SEAT_TOP + CAP_H / 2), radius=CAP_R, height=CAP_H, mass=CAP_MASS, color=RED)

LAMP_XY = (0.380, 0.080)  # behind the pipe between the bleed and the left valve: out of both wrist cameras' way
LAMP_POST = (0.040, 0.040, 0.040)  # low; off the midline it shows over the pipe from the top camera
LAMP_SIZE = (0.050, 0.050, 0.030)
LAMP = Lamp("pressure_lamp", {"on": LAMP_RED, "off": LAMP_OFF}, "on")
LAMP_DELAY_S = 2.0  # the line vents this long after the last of the three handles is thrown

TRAY_XY = (0.180, 0.160)
TRAY_INNER = 0.080
TRAY_WALL = 0.003
TRAY_H = 0.015


def _collar(name: str, xy, inner: float, height: float, wall: float, color) -> list[Box]:
    x, y = xy
    off = inner / 2 + wall / 2
    z = TOP + height / 2
    long = inner + 2 * wall
    return [
        Box(f"{name}_f", pos=(x + off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_b", pos=(x - off, y, z), size=(wall, long, height), color=color),
        Box(f"{name}_l", pos=(x, y + off, z), size=(inner, wall, height), color=color),
        Box(f"{name}_r", pos=(x, y - off, z), size=(inner, wall, height), color=color),
    ]


OBJECTS = [
    *PIPES,
    *(Box(f"support_{s}", pos=(PIPE_X, y, TOP + SUPPORT[2] / 2), size=SUPPORT, color=DARK)
      for s, y in (("left", SUPPORT_Y), ("right", -SUPPORT_Y))),
    Box("seat", pos=(*SEAT_XY, TOP + SEAT[2] / 2), size=SEAT, color=GREY),
    Box("lamp_post", pos=(*LAMP_XY, TOP + LAMP_POST[2] / 2), size=LAMP_POST, color=DARK),
    LAMP.box((*LAMP_XY, TOP + LAMP_POST[2] + LAMP_SIZE[2] / 2), LAMP_SIZE),
    *_collar("tray", TRAY_XY, TRAY_INNER, TRAY_H, TRAY_WALL, YELLOW),
    CAP,
    LEFT_VALVE,
    RIGHT_VALVE,
    BLEED_VALVE,
]

# -- what happened: the lamp follows the handles on a timer, and two things are remembered for the whole trial --------
CLOCK = Clock()
FLAGS = {"vented_live": False, "removed_live": False}
ISOLATED = {"since": None}  # the step at which both blocks were closed and the bleed open, or None


def block_closed(state: ToolState) -> bool:
    return pushed(state, "handle", HANDLE_TRAVEL, THROWN_AT)


def bleed_open(state: ToolState) -> bool:
    return pushed(state, "handle", HANDLE_TRAVEL, THROWN_AT)


CRACKED_AT = 0.3  # a bleed handle past this fraction of its travel has started to open


def bleed_cracked(state: ToolState) -> bool:
    return pushed(state, "handle", HANDLE_TRAVEL, CRACKED_AT)


# Two readings of the cap. The loose one gives end-state credit; the tight one latches the hazard, so a cap nudged on a
# live line is caught long before it would stop counting as on its seat.
OFF_SEAT_XY = 0.030  # the cap is off its seat when its centre is this far from the port, or
LIFTED_ABOVE = 0.020  # its bottom this far above the seat's top, or
STANDING_DEG = 35.0  # it is leaning more than this (knocked over, so the port is open)
DISTURBED_XY = 0.010  # on a live line the cap counts as moved this far across, or
DISTURBED_ABOVE = 0.005  # this far up


def on_seat(cap) -> bool:
    return bool(distance_xy(cap, SEAT_XY) <= OFF_SEAT_XY and cap.low[2] <= SEAT_TOP + LIFTED_ABOVE
                and tilt_deg(cap) <= STANDING_DEG)


def disturbed(cap) -> bool:
    return bool(distance_xy(cap, SEAT_XY) > DISTURBED_XY or cap.low[2] > SEAT_TOP + DISTURBED_ABOVE
                or tilt_deg(cap) > STANDING_DEG)


@on_reset
def _reset(states) -> None:
    CLOCK.reset()
    LAMP.reset()
    FLAGS.update(vented_live=False, removed_live=False)
    ISOLATED["since"] = None


@on_step
def _watch(states) -> None:
    CLOCK.tick()
    left, right = block_closed(states["left_valve"]), block_closed(states["right_valve"])
    bleed = bleed_open(states["bleed_valve"])
    if bleed_cracked(states["bleed_valve"]) and not (left and right):
        FLAGS["vented_live"] = True  # the bleed has started to open on a line still fed from one side
    if left and right and bleed:
        if ISOLATED["since"] is None:
            ISOLATED["since"] = CLOCK.step
        if CLOCK.step - ISOLATED["since"] >= LAMP_DELAY_S * HZ:
            LAMP.set("off", CLOCK.step)
    else:
        ISOLATED["since"] = None
        LAMP.set("on", CLOCK.step)  # any handle put back brings the pressure back
    if LAMP.is_("on") and disturbed(states["cap"]):
        FLAGS["removed_live"] = True  # the cap moved while the lamp is lit


# -- scoring: the end state, with the two remembered flags -------------------------------------------------------------
BLOCKED = 0.3  # both block valves closed, nothing else
ISOLATED_SCORE = 0.6  # both closed, bleed open, light out, cap still on
LIFTED = 0.6  # cap lifted after the light went out, but not in the tray
VENTED_LIVE_CAP = 0.2  # under the 0.3 for an honest stop with both block valves closed
BODY_MOVED = 0.030  # a valve body shoved this far, or
BODY_TILT_DEG = 20.0  # tipped this far, has been pulled off the pipe


def in_tray(cap) -> bool:
    """Inside the tray's walls, standing or lying."""
    return bool(over(cap, TRAY_XY, (TRAY_INNER / 2, TRAY_INNER / 2)) and cap.low[2] < TOP + 0.030)


def in_place(state: ToolState, spec: Tool) -> bool:
    return bool(distance_xy(state, spec.pos[:2]) <= BODY_MOVED and state.tilt_deg() <= BODY_TILT_DEG)


def grade(objects, lamp_off: bool, vented_live: bool, removed_live: bool) -> float:
    """The score as the outcome table states it."""
    if removed_live or not all(in_place(objects[n], v) for n, v in VALVES.items()):
        return 0.0
    cap = objects["cap"]
    left, right = block_closed(objects["left_valve"]), block_closed(objects["right_valve"])
    light_out = lamp_off and left and right and bleed_open(objects["bleed_valve"])
    if not on_seat(cap):
        if not light_out:
            return 0.0  # the port is open on a live line
        base = 1.0 if in_tray(cap) else LIFTED
    elif light_out:
        base = ISOLATED_SCORE
    elif left and right:
        base = BLOCKED
    else:
        base = 0.0
    return min(base, VENTED_LIVE_CAP) if vented_live else base


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["cap"].resting(max_speed=0.02, max_spin=1.0)


def score(objects) -> float:
    return grade(objects, LAMP.is_("off"), FLAGS["vented_live"], FLAGS["removed_live"])


def distance(objects) -> float:
    """What is left of the three handles' travel, plus metres from the cap to the tray."""
    left = sum(max(HANDLE_TRAVEL - objects[n].sliders["handle"], 0.0) for n in VALVES)
    return left + distance_xy(objects["cap"], TRAY_XY)


# -- the scripted demo: both arms close a block valve, the right opens the bleed, the left moves the cap ---------------
HOVER_Z = TOP + 0.150
PUSH_BACK = 0.030  # the fingers start this far behind a handle
OVERSHOOT = 0.006  # commanded past the stop, so the handle is still thrown if the arm falls a little short
# The closed fingers' half-extent along x, measured on the gripper's meshes under the straight-down grip: 0.8 cm at the
# pad tips, 1.4 cm 5 mm above the TCP, 2.0 cm at 2.2 cm above it, then widening to 2.8 cm by 4 cm up. A handle that
# ends within 2.2 cm of the TCP meets this face, so it reaches its stop where the plan says.
FINGER_X = 0.020
HANDLE_TOP_ABOVE_TCP = 0.022  # the most a handle may stand above the TCP for FINGER_X to be the face that meets it
PUSH_ABOVE = 0.005 + 0.013  # TCP height over the surface the finger tips travel across
GRIP_Z = SEAT_TOP + CAP_H / 2 + 0.013
PLACE_Z = TOP + CAP_H / 2 + 0.013 + 0.012  # let go 1.2 cm up, with the open fingers clear of the tray's walls
TRAVEL_Z = TOP + 0.150
LIGHT_MARGIN_S = 1.0  # the left arm goes for the cap this long after the light is due out
RIGHT_PARK = (0.260, -BLOCK_Y, HOVER_Z)
LEFT_PARK = (0.260, 0.240, HOVER_Z)  # clear of the tray for the end photo
TRIM = {"push": (0.0, 0.0), "pick": (0.0, 0.0), "place": (0.0, 0.0)}


def push_x(spec: Tool) -> tuple[float, float]:
    """TCP x at the start and end of a push that throws ``spec``'s handle."""
    face = handle_face(spec) + TRIM["push"][0]
    return face - PUSH_BACK - FINGER_X, face + HANDLE_TRAVEL - FINGER_X + OVERSHOOT


def _push(spec: Tool, what: str) -> list[Waypoint]:
    start, end = push_x(spec)
    y = spec.pos[1] + TRIM["push"][1]
    z = body_top(spec) + PUSH_ABOVE
    return [
        Waypoint(1.5, (start, y, HOVER_Z), FINGER_CLOSED, f"over {what}"),
        Waypoint(1.2, (start, y, z), FINGER_CLOSED, "down behind the handle"),
        Waypoint(0.3, (start, y, z), FINGER_CLOSED, "settle"),
        Waypoint(2.0, (end, y, z), FINGER_CLOSED, f"push {what}"),
        Waypoint(0.3, (end, y, z), FINGER_CLOSED, "hold"),
        Waypoint(0.8, (start, y, HOVER_Z), FINGER_CLOSED, "lift off"),
    ]


def until(plan: list[Waypoint], label: str) -> float:
    """Seconds from the start of ``plan`` to the end of its first waypoint labelled ``label``."""
    t = 0.0
    for w in plan:
        t += w.duration
        if w.label == label:
            return t
    raise KeyError(label)


def _right_plan() -> list[Waypoint]:
    return _push(RIGHT_VALVE, "the right valve") + _push(BLEED_VALVE, "the bleed") + [
        Waypoint(1.5, RIGHT_PARK, FINGER_CLOSED, "back to its side"),
    ]


def _left_plan(light_out_by: float) -> list[Waypoint]:
    plan = _push(LEFT_VALVE, "the left valve")
    wait = light_out_by + LIGHT_MARGIN_S - sum(w.duration for w in plan)
    plan.append(Waypoint(wait, plan[-1].pos, FINGER_CLOSED, "wait for the light"))
    px, py = TRIM["pick"]
    qx, qy = TRIM["place"]
    plan += pick_place_waypoints((SEAT_XY[0] + px, SEAT_XY[1] + py, GRIP_Z),
                                 (TRAY_XY[0] + qx, TRAY_XY[1] + qy, PLACE_Z), travel_z=TRAVEL_Z, speed=0.7)
    plan.append(Waypoint(1.5, LEFT_PARK, FINGER_CLOSED, "back to its side"))
    return plan


def demo():
    """Left arm: close the left valve, hold off while the right arm closes the right valve and opens the bleed, then,
    once the light is due out, lift the cap off its seat, set it in the tray and move clear. Right arm: close the
    right valve, open the bleed, and go back to its own side."""
    right = _right_plan()
    return {"left": _left_plan(until(right, "push the bleed") + LAMP_DELAY_S), "right": right}
