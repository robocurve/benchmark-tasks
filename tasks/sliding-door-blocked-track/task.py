"""On the station table, 25 cm in front of the arms, a straight 60 cm aluminium bottom track runs left to right, from
in front of the right arm to just left of the middle: a base strip with two low side lips and an end stop at each
end, fixed to the table. A loose clear acrylic sliding door
(18 x 12 cm, 1 cm thick) stands upright in the channel, pushed against the right end stop. A loose red wooden block
(8 cm long, 4.5 cm tall, 1 cm thick) stands on its long edge in the same channel, against the left end stop; it
sticks up 3 cm above the lips. Nothing else is on the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Slide the door to the left along its track until it touches the track's left end stop. \\
Keep the door in its track and everything on the table." --scorer task_score ...
With DEMO_FORCE=1 the scripted demo plays the forcing control instead (see demo()).
"""

import math
import os

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # table surface height (m)

# -- the track: fixed, axis-aligned boxes (static colliders) ---------------------------------------------------
TRACK_X = 0.25  # centre line, metres in front of the arm bases
TRACK_Y = -0.11  # the track's middle, 11 cm right of the midline between the arms
BASE_T = 0.006  # base strip thickness
FLOOR = TOP + BASE_T  # channel floor height
GAP = 0.013  # channel width between the lips
LIP_T = 0.004  # lip thickness
LIP_H = 0.015  # lip and end-stop height above the channel floor
STOP_IN = 0.29  # inner faces of the end stops at TRACK_Y +- STOP_IN: y -0.40 and +0.18
STOP_L = 0.01  # end-stop length along the track
TRACK_LEN = 2 * (STOP_IN + STOP_L)  # 0.60 m overall
TRACK_W = 0.04  # base strip width
OUTER = GAP / 2 + LIP_T  # half the width over the lips: 10.5 mm
ALUMINIUM = (0.76, 0.77, 0.79)

# -- the loose parts -------------------------------------------------------------------------------------------
DOOR_SIZE = (0.010, 0.18, 0.12)  # thickness (x), width (y), height (z)
DOOR_MASS = 0.257  # 10 x 180 x 120 mm of acrylic at 1.19 g/cm^3
BLOCK_SIZE = (0.010, 0.08, 0.045)  # thickness (x), length (y), height (z)
BLOCK_MASS = 0.018  # 10 x 80 x 45 mm of pine at 0.5 g/cm^3
SPAWN_GAP = 0.001  # loose parts start 1 mm clear of their supports and stops, then settle

DOOR_START_Y = TRACK_Y - STOP_IN + DOOR_SIZE[1] / 2 + SPAWN_GAP  # closed: against the right end stop
DOOR_START_LEFT = DOOR_START_Y + DOOR_SIZE[1] / 2  # its left edge at the start (-21.9 cm)
BLOCK_START_Y = TRACK_Y + STOP_IN - BLOCK_SIZE[1] / 2 - SPAWN_GAP  # against the left end stop

OBJECTS = [
    Box("track_base", pos=(TRACK_X, TRACK_Y, TOP + BASE_T / 2), size=(TRACK_W, TRACK_LEN, BASE_T), color=ALUMINIUM),
    Box("track_lip_far", pos=(TRACK_X + GAP / 2 + LIP_T / 2, TRACK_Y, FLOOR + LIP_H / 2),
        size=(LIP_T, TRACK_LEN, LIP_H), color=ALUMINIUM),
    Box("track_lip_near", pos=(TRACK_X - GAP / 2 - LIP_T / 2, TRACK_Y, FLOOR + LIP_H / 2),
        size=(LIP_T, TRACK_LEN, LIP_H), color=ALUMINIUM),
    Box("left_stop", pos=(TRACK_X, TRACK_Y + STOP_IN + STOP_L / 2, FLOOR + LIP_H / 2),
        size=(2 * OUTER, STOP_L, LIP_H), color=ALUMINIUM),
    Box("right_stop", pos=(TRACK_X, TRACK_Y - STOP_IN - STOP_L / 2, FLOOR + LIP_H / 2),
        size=(2 * OUTER, STOP_L, LIP_H), color=ALUMINIUM),
    Box("door", pos=(TRACK_X, DOOR_START_Y, FLOOR + DOOR_SIZE[2] / 2 + SPAWN_GAP), size=DOOR_SIZE,
        mass=DOOR_MASS, color=(0.62, 0.80, 0.95)),
    Box("block", pos=(TRACK_X, BLOCK_START_Y, FLOOR + BLOCK_SIZE[2] / 2 + SPAWN_GAP), size=BLOCK_SIZE,
        mass=BLOCK_MASS, color=(0.80, 0.16, 0.10)),
]

# -- scoring thresholds (all measurable with a tape measure and a photo) ----------------------------------------
FULL_OPEN_GAP = 0.02  # fully open: the door's left edge within 2 cm of the left end stop
PARTIAL_TRAVEL = 0.10  # partly open: its left edge moved at least 10 cm to the left
SEATED_HEIGHT = 0.005  # in its track: bottom edge within 5 mm of the channel floor ...
SEATED_OFFSET = 0.006  # ... centre within 6 mm of the track's centre line ...
SEATED_TILT_DEG = 10.0  # ... and leaning less than 10 degrees
BLOCK_DOWN_HEIGHT = 0.03  # the block is down on the table (or the track) when its bottom is under 3 cm up ...
OFF_TABLE_DROP = 0.01  # ... and not more than 1 cm below the table top, which means it fell to the floor


def _tilt_deg(state):
    """Angle between the object's own vertical axis and the world vertical (degrees)."""
    qx, qy = float(state.quat_xyzw[0]), float(state.quat_xyzw[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))))


def _seated(door):
    """The door stands in its track: bottom edge on the channel floor, inside the lips, upright."""
    return bool(
        abs(float(door.low[2]) - FLOOR) <= SEATED_HEIGHT
        and abs(float(door.pos[0]) - TRACK_X) <= SEATED_OFFSET
        and _tilt_deg(door) <= SEATED_TILT_DEG
    )


def _fully_open(door):
    return bool(float(door.high[1]) >= TRACK_Y + STOP_IN - FULL_OPEN_GAP)


def _on_table(block):
    """Down on the table or the track: not on the floor, and not held up in the air."""
    return bool(TOP - OFF_TABLE_DROP <= float(block.low[2]) <= TOP + BLOCK_DOWN_HEIGHT)


def _in_door_path(block, door):
    """Some part of the block is in the track between the door's left edge and the left end stop, below the
    height of the door's top edge, where it stops the door."""
    return bool(
        float(block.high[0]) > TRACK_X - OUTER
        and float(block.low[0]) < TRACK_X + OUTER
        and float(block.high[1]) > float(door.high[1])
        and float(block.low[1]) < TRACK_Y + STOP_IN
        and float(block.low[2]) < FLOOR + DOOR_SIZE[2]
    )


def success(objects):
    """Fully open and in its track, the block resting on the table, both still (held for 1 s)."""
    door, block = objects["door"], objects["block"]
    return bool(
        _seated(door) and _fully_open(door) and _on_table(block) and door.resting() and block.resting()
    )


def distance(objects):
    """Metres from the door's left edge to the left end stop."""
    return max(0.0, TRACK_Y + STOP_IN - float(objects["door"].high[1]))


def score(objects):
    """Partial credit at the end of the episode (success scores 1.0):

    - 0 if the door is out of its track (bottom edge more than 5 mm off the channel floor, centre more than
      6 mm off the track's centre line, or leaning more than 10 degrees);
    - 1.0 if it is fully open (left edge within 2 cm of the left end stop) with the block on the table and
      both still;
    - 0.5 if it is fully open otherwise (the block fell off the table or is still held up, or something is
      still moving), or if the block has been taken out of the door's path and rests on the table;
    - 0.25 if the door's left edge has moved at least 10 cm to the left;
    - 0 otherwise.
    """
    door, block = objects["door"], objects["block"]
    if not _seated(door):
        return 0.0
    if _fully_open(door):
        return 1.0 if _on_table(block) and door.resting() and block.resting() else 0.5
    if _on_table(block) and not _in_door_path(block, door):
        return 0.5
    if float(door.high[1]) - DOOR_START_LEFT >= PARTIAL_TRAVEL:
        return 0.25
    return 0.0


# -- the scripted demo ------------------------------------------------------------------------------------------
# First the left arm takes the block out of the track: only it reaches the block. Then the right arm slides the door
# as far as it reaches, and the left arm, which reaches the track's left end, finishes. Each arm pulls the door by its
# left end, the end that leads: pulled that way, a door that sags in the pinch drags its trailing corner, and that drag
# turns it back level. Pushed by its trailing end, its leading corner digs into the channel floor and it pivots up out
# of the pinch.
# At the block the left arm turns 85 degrees: at 88 or 90, joint6 reaches its -120 degree limit there.
BLOCK_GRIP_YAW = math.pi / 2 - math.radians(5.0)
# At the door the fingers close across its thickness: the left arm turned -90 degrees and the right arm +90. The other
# way round, each wrist would need joint6 past its limit.
LEFT_DOOR_YAW = -math.pi / 2
RIGHT_DOOR_YAW = math.pi / 2
TRAVEL_Z = TOP + 0.16  # carrying and waiting height
# The tool centre point sits PAD_TIP_PAST_TCP (13 mm) above the finger pad tips when pointing down.
# Block: pad tips 4 mm above the lips, so the fingers never enter the channel; they pinch the 3 cm that sticks up.
BLOCK_GRIP = (TRACK_X, BLOCK_START_Y, FLOOR + LIP_H + 0.004 + PAD_TIP_PAST_TCP)
# Set it down standing on the table at (16, 40) cm: behind the track, beside the left arm, clear of the door's travel.
BLOCK_PLACE = (0.16, 0.40, TOP + BLOCK_SIZE[2] / 2 + 0.002 + (BLOCK_GRIP[2] - (FLOOR + BLOCK_SIZE[2] / 2)))
# Door: pad tips 2 cm below its top edge, 7.5 cm left of its centre (1.5 cm in from its left end).
DOOR_GRIP_DY = 0.075
DOOR_GRIP_Z = FLOOR + DOOR_SIZE[2] - 0.020 + PAD_TIP_PAST_TCP
# Lift 10 mm while sliding. The arm's joint control gives a little under the door's weight, so the door rises less:
# off the channel floor, and still well down between the 15 mm lips.
DOOR_LIFT = 0.010
DOOR_END_Y = TRACK_Y + STOP_IN - DOOR_SIZE[1] / 2 - 0.005  # stop 5 mm short of the end stop: fully open
# The right arm pulls the door until its pinch is 3 cm left of the midline between the arms (26.4 cm of travel) and
# sets it down there for the left arm. It cannot take the door to the end: its pinch would have to reach y +0.16, and
# from about +0.04 on its joint4 runs out of travel (27 mm of IK error at +0.16).
HANDOFF_Y = 0.03
RIGHT_WAIT = (TRACK_X, DOOR_START_Y + DOOR_GRIP_DY, TRAVEL_Z)  # the right hand waits above the door's left end
# The forcing control (DEMO_FORCE=1) is the demo without clearing the track: the block stays in it, the right arm
# slides the door to the handover point, and the left arm pulls it on into the block, commanding its pinch FORCE_PAST
# beyond the point where the block stops the door, and keeps pushing for FORCE_PUSH_S. The door stops 8 cm short of the
# end stop, so this can earn 0.25 at most. (The right arm cannot push it there: 3.8 mm of IK error at the block.)
BLOCKED_PINCH_Y = BLOCK_START_Y - BLOCK_SIZE[1] / 2 - (DOOR_SIZE[1] / 2 - DOOR_GRIP_DY)  # door's left edge at the block
FORCE_PAST = 0.01
FORCE_PUSH_S = 2.0


def _pull(y0, y1, slide_s, yaw, push_s=0.0):
    """Pinch the door's top edge at y0, lift it, slide it until the pinch is at y1, set it down and let go. With
    push_s, keep pushing toward y1 for push_s seconds before setting it down (the forcing control)."""
    x, z = TRACK_X, DOOR_GRIP_Z
    push = [Waypoint(push_s, (x, y1, z + DOOR_LIFT), FINGER_CLOSED, "keep pushing", yaw)] if push_s else []
    return [
        Waypoint(1.5, (x, y0, z), FINGER_OPEN, "lower around the top edge", yaw),
        Waypoint(0.8, (x, y0, z), FINGER_CLOSED, "pinch the top edge", yaw),
        Waypoint(0.8, (x, y0, z + DOOR_LIFT), FINGER_CLOSED, "lift it off the channel floor", yaw),
        Waypoint(slide_s, (x, y1, z + DOOR_LIFT), FINGER_CLOSED, "slide it to the left", yaw),
        *push,
        Waypoint(0.6, (x, y1, z), FINGER_CLOSED, "set it down in the channel", yaw),
        Waypoint(0.6, (x, y1, z), FINGER_OPEN, "let go", yaw),
        Waypoint(1.0, (x, y1, TRAVEL_Z), FINGER_OPEN, "retreat", yaw),
    ]


def _forcing_demo():
    """The forcing control: the demo without taking the block out. The left arm waits where the demo sets the block
    down; the right arm slides the door to the handover point; the left arm pulls it on into the block, keeps pushing,
    then sets it down and lets go."""
    wait = (BLOCK_PLACE[0], BLOCK_PLACE[1], TRAVEL_Z)
    right = [Waypoint(2.0, RIGHT_WAIT, FINGER_OPEN, "above the door's left end", RIGHT_DOOR_YAW)]
    right += _pull(DOOR_START_Y + DOOR_GRIP_DY, HANDOFF_Y, 8.0, RIGHT_DOOR_YAW)
    right_clear = sum(w.duration for w in right)
    right.append(Waypoint(1.5, RIGHT_WAIT, FINGER_OPEN, "back out of the way", RIGHT_DOOR_YAW))
    left = [
        Waypoint(3.0, wait, FINGER_OPEN, "out of the right arm's way", BLOCK_GRIP_YAW),
        Waypoint(right_clear - 3.0, wait, FINGER_OPEN, "wait for the right arm to let go", BLOCK_GRIP_YAW),
        Waypoint(2.0, (TRACK_X, HANDOFF_Y, TRAVEL_Z), FINGER_OPEN, "above the door's left end", LEFT_DOOR_YAW),
    ]
    left += _pull(HANDOFF_Y, BLOCKED_PINCH_Y + FORCE_PAST, 2.5, LEFT_DOOR_YAW, FORCE_PUSH_S)
    return {"left": left, "right": right}


def demo():
    """Left arm: pinch the block across its thickness, lift it straight out of the channel and set it down on the
    table at (16, 40) cm, behind the track. Right arm, waiting above the door's left end until the block is carried
    clear of the track: pinch the door's top edge 1.5 cm in from its left end, lift it, slide it to the left as far as
    it reaches, set it down, let go and go back up. Left arm: pinch the door at the same place and slide it the rest
    of the way, to 5 mm short of the left end stop, then let go.

    With DEMO_FORCE=1 it plays the forcing control instead (_forcing_demo)."""
    if os.environ.get("DEMO_FORCE"):
        return _forcing_demo()
    x, y0, y_end = TRACK_X, DOOR_START_Y + DOOR_GRIP_DY, DOOR_END_Y + DOOR_GRIP_DY
    left = pick_place_waypoints(BLOCK_GRIP, BLOCK_PLACE, travel_z=TRAVEL_Z, yaw=BLOCK_GRIP_YAW)
    labels = [w.label for w in left]
    block_clear = sum(w.duration for w in left[: labels.index("carry") + 1])  # above its spot, clear of the track
    right = [
        Waypoint(2.0, RIGHT_WAIT, FINGER_OPEN, "above the door's left end", RIGHT_DOOR_YAW),
        Waypoint(block_clear - 2.0, RIGHT_WAIT, FINGER_OPEN, "wait for the block to be carried clear", RIGHT_DOOR_YAW),
    ]
    right += _pull(y0, HANDOFF_Y, 8.0, RIGHT_DOOR_YAW)
    right_clear = sum(w.duration for w in right)  # the right hand has let go and is back up
    right.append(Waypoint(1.5, RIGHT_WAIT, FINGER_OPEN, "back out of the way", RIGHT_DOOR_YAW))
    block_done = sum(w.duration for w in left)
    left.append(Waypoint(right_clear - block_done, left[-1].pos, FINGER_OPEN, "wait for the right arm to let go",
                         BLOCK_GRIP_YAW))
    left.append(Waypoint(2.0, (x, HANDOFF_Y, TRAVEL_Z), FINGER_OPEN, "above the door's left end", LEFT_DOOR_YAW))
    left += _pull(HANDOFF_Y, y_end, 4.5, LEFT_DOOR_YAW)
    return {"left": left, "right": right}
