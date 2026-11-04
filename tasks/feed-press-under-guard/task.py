"""On the station table, between the arms: a small press bed under a fixed mesh hood. The only way in is from the left,
under a polycarbonate guard that slides up and down in two channels and falls shut when let go. A flat aluminium blank
lies on the infeed lane left of the guard, and a wooden push stick lies on the table in front of the left arm. The goal
is to feed the blank: raise the guard no more than 8 cm, push the blank in to the back stop with the stick, take the
stick out, and lower the guard gently.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Raise the guard no more than 8 cm, push the blank in to the back stop with the push stick, take the stick out and lower the guard." \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, MESH, ALU, WOOD, RED = (0.40, 0.42, 0.45), (0.28, 0.30, 0.33), (0.78, 0.80, 0.82), (0.78, 0.62, 0.40), (0.80, 0.15, 0.12)

X = 0.22  # the feed line runs along world y at this x
GUARD_Y = 0.0
GUARD_SIZE = (0.14, 0.02, 0.16)
GUARD_MASS = 0.5
LANE_HALF = 0.032  # half the width between the guides: 4 mm of play in all for the 6 cm blank
HOOD_HALF = 0.036  # the hood is 8 mm wider, so a blank that has turned a little still enters it
STOP_FACE = -0.15  # the back stop's face, along y
HOOD_H = 0.09
BLANK_SIZE = (0.06, 0.10, 0.02)
BLANK_START = (X, 0.10, TOP + BLANK_SIZE[2] / 2)
STICK_SIZE = (0.02, 0.20, 0.03)
STICK_START = (0.16, 0.30, TOP + STICK_SIZE[2] / 2)

GUARD = Box("guard", pos=(X, GUARD_Y, TOP + GUARD_SIZE[2] / 2), size=GUARD_SIZE, mass=GUARD_MASS, color=RED)
BLANK = Box("blank", pos=BLANK_START, size=BLANK_SIZE, mass=0.3, color=ALU)
STICK = Box("push_stick", pos=STICK_START, size=STICK_SIZE, mass=0.08, color=WOOD)


def _machine():
    """The fixed parts: guides, hood walls, back stop, mesh roof, and the guard's two channels."""
    wall_t, gap = 0.01, GUARD_SIZE[1] / 2 + 0.002  # the guard runs in a 24 mm channel
    wall_x, hood_x = LANE_HALF + wall_t / 2, HOOD_HALF + wall_t / 2
    out = []
    for i, s in enumerate((-1, 1)):
        hood_y0, hood_y1 = STOP_FACE - 0.01, GUARD_Y - gap
        out.append(Box(f"hood_wall_{i}", pos=(X + s * hood_x, (hood_y0 + hood_y1) / 2, TOP + HOOD_H / 2),
                       size=(wall_t, hood_y1 - hood_y0, HOOD_H), color=STEEL))
        lane_y0, lane_y1 = GUARD_Y + gap, 0.18
        out.append(Box(f"guide_{i}", pos=(X + s * wall_x, (lane_y0 + lane_y1) / 2, TOP + 0.0075),
                       size=(wall_t, lane_y1 - lane_y0, 0.015), color=STEEL))
        # A channel: a post either side of the guard's end, and one closing it, 13 cm tall, below the hand that holds the guard.
        ex = GUARD_SIZE[0] / 2
        for j, t in enumerate((-1, 1)):
            out.append(Box(f"channel_{i}_{j}", pos=(X + s * (ex - 0.01), GUARD_Y + t * (gap + 0.005), TOP + 0.065),
                           size=(0.03, 0.01, 0.13), color=STEEL))
        out.append(Box(f"channel_end_{i}", pos=(X + s * (ex + 0.0095), GUARD_Y, TOP + 0.065),
                       size=(0.005, 2 * gap + 0.02, 0.13), color=STEEL))
    out.append(Box("back_stop", pos=(X, STOP_FACE - 0.005, TOP + HOOD_H / 2), size=(2 * HOOD_HALF, 0.01, HOOD_H), color=STEEL))
    # The mesh: bars across the bed with 15 mm gaps, too narrow for a finger, so the blank can be seen but not reached.
    y = STOP_FACE
    i = 0
    while y < GUARD_Y - gap - 0.01:
        out.append(Box(f"mesh_{i}", pos=(X, y + 0.005, TOP + HOOD_H + 0.005), size=(2 * hood_x + wall_t, 0.01, 0.01), color=MESH))
        y += 0.025
        i += 1
    return out


OBJECTS = [GUARD, BLANK, STICK, *_machine()]

MAX_OPEN, SOFT_OPEN = 0.08, 0.12  # m the guard may be raised: for full score, and for 0.75
OUT_OF_CHANNELS = 0.13  # m: raised this far, the guard has left its channels
MAX_FALL = 0.3  # m/s: faster than this downward counts as the guard being dropped
CLOSED = 0.003  # m: the guard's lower edge within this of the table
AT_STOP = 0.005  # m: the blank's front edge within this of the back stop


class Watch:
    """What the guard has done so far in this trial. The blank jumping more than 3 cm back toward its start in one control
    step means a reset; pushing only ever moves it the other way."""

    def __init__(self):
        self.last, self.opened, self.fell = None, 0.0, 0.0

    def update(self, objects):
        pos = np.asarray(objects["blank"].pos, dtype=float)
        if self.last is not None and pos[1] > self.last[1] + 0.03:
            self.opened, self.fell = 0.0, 0.0
        self.last = pos
        guard = objects["guard"]
        self.opened = max(self.opened, float(guard.low[2]) - TOP)
        self.fell = max(self.fell, -float(guard.lin_vel[2]))


WATCH = Watch()


def fed(objects) -> bool:
    """The blank lies flat with its front edge at the back stop."""
    blank = objects["blank"]
    return bool(blank.low[1] < STOP_FACE + AT_STOP and blank.high[2] < TOP + BLANK_SIZE[2] + 0.005)


def closed(objects) -> bool:
    """The guard is down, and the stick is out from under it."""
    return bool(objects["guard"].low[2] < TOP + CLOSED and objects["push_stick"].low[1] > GUARD_Y + GUARD_SIZE[1] / 2)


def success(objects):
    """Fed and closed, the guard never raised over 8 cm or falling over 0.3 m/s, blank and guard still; must hold for 1 s.
    Runs first in every control step, so it also keeps the record of what the guard has done."""
    WATCH.update(objects)
    return bool(
        fed(objects) and closed(objects) and WATCH.opened <= MAX_OPEN and WATCH.fell <= MAX_FALL
        and objects["blank"].resting() and objects["guard"].resting()
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.75 if fed and closed with the guard raised 8 to 12 cm and never dropped; 0.5
    if fed and closed outside those limits; 0.25 if the blank is wholly past the guard but not finished; 0 if the guard ever left its
    channels."""
    if WATCH.opened > OUT_OF_CHANNELS:
        return 0.0
    if fed(objects) and closed(objects):
        return 0.75 if WATCH.opened <= SOFT_OPEN and WATCH.fell <= MAX_FALL else 0.5
    return 0.25 if objects["blank"].high[1] < GUARD_Y - GUARD_SIZE[1] / 2 else 0.0


# How far the stick sits from where the demo grips it across the lane, measured in a traced run (closing the fingers
# shifts it a few millimetres, the same way every run with -E object_jitter=0).
STICK_GRIP_SHIFT = 0.004
PUSH_LAG = 0.016  # a traced push stopped this far short of its target: the stick gives a little in the grip


def demo():
    """Right arm: grip the guard's top edge, raise it 6 cm, hold it, and lower it slowly once the stick is out. Left arm,
    meanwhile: take the stick by its tail, set it in the lane behind the blank, push the blank to the stop and draw the
    stick back out."""
    hover = TOP + 0.19
    part = FINGER_OPEN * 0.5
    grip_z = TOP + GUARD_SIZE[2] - 0.03
    raised = grip_z + 0.06
    right = [
        Waypoint(1.5, (X, GUARD_Y, hover), part, "above the guard"),
        Waypoint(1.5, (X, GUARD_Y, grip_z), part, "down to its top edge"),
        Waypoint(0.8, (X, GUARD_Y, grip_z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (X, GUARD_Y, raised), FINGER_CLOSED, "raise it 6 cm"),
        Waypoint(9.5, (X, GUARD_Y, raised), FINGER_CLOSED, "hold it up while the blank goes in"),
        Waypoint(3.0, (X, GUARD_Y, grip_z + 0.001), FINGER_CLOSED, "lower it slowly"),
        Waypoint(0.8, (X, GUARD_Y, grip_z + 0.001), part, "let go"),
        Waypoint(1.2, (X, GUARD_Y, hover), part, "back up"),
    ]
    yaw = -np.pi / 2  # fingers across the stick
    g = 0.07  # the grip, this far toward the tail from the stick's centre
    z_grip = TOP + 0.02
    carry = z_grip + 0.06
    tip_start = BLANK_START[1] + BLANK_SIZE[1] / 2 + 0.01
    tip_end = STOP_FACE + BLANK_SIZE[1] - PUSH_LAG
    half = STICK_SIZE[1] / 2
    start_y, end_y = tip_start + half + g, tip_end + half + g
    s0 = (STICK_START[0], STICK_START[1] + g)
    lane_x = X - STICK_GRIP_SHIFT
    left = [
        Waypoint(1.5, (*s0, hover), part, "above the stick's tail", yaw),
        Waypoint(1.5, (*s0, z_grip), part, "down to it", yaw),
        Waypoint(0.8, (*s0, z_grip), FINGER_CLOSED, "grip", yaw),
        Waypoint(1.0, (*s0, carry), FINGER_CLOSED, "lift it", yaw),
        Waypoint(1.8, (lane_x, start_y, carry), FINGER_CLOSED, "over the lane, behind the blank", yaw),
        Waypoint(1.0, (lane_x, start_y, z_grip), FINGER_CLOSED, "down into the lane", yaw),
        Waypoint(3.5, (lane_x, end_y, z_grip), FINGER_CLOSED, "push the blank in to the stop", yaw),
        Waypoint(3.0, (lane_x, start_y, z_grip), FINGER_CLOSED, "draw the stick back out", yaw),
        Waypoint(1.0, (lane_x, start_y, carry), FINGER_CLOSED, "lift it clear", yaw),
    ]
    return {"left": left, "right": right}
