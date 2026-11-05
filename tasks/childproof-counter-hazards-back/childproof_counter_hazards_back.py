"""On the station table, in front of both arms, a fixed brown wooden counter board (900 mm long side to side, 240 mm
front to back, 30 mm thick) runs from 45 cm left of centre to 45 cm right of centre, 17 to 41 cm in front of the arm
bases. Its left end is the toddler's end: a fixed yellow cylinder (60 mm across, 60 mm tall) stands on the table just
in front of the counter's front edge, its centre 80 mm in from that end, marking the toddler's step-stool (placed
where the left arm's start pose does not hide it from the top camera). A fixed blue divider strip (20 mm wide, 30 mm
tall) crosses the counter front to back, its right face 10 cm right of centre: 550 mm from the toddler's end. On the left half of the
counter, within the toddler's reach, lie a grey knife in a blade guard (150 x 30 x 25 mm, 100 g, flat, long side
front to back), a red hot mug (70 mm across, 80 mm tall, 300 g) and an orange pill bottle (40 mm across, 60 mm tall,
80 g). Far right, out of reach, stands the toddler's green sippy cup (60 mm across, 100 mm tall, 150 g). Done, the
knife lies on one of its long faces and the mug and bottle stand upright, all at rest on the counter right of the
divider, and the cup stands upright at rest on the counter with its centre within 150 mm of the toddler's end.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The counter: a fixed board on the table. Its left end (y = +0.45) is the toddler's end.
COUNTER_X = (0.17, 0.41)
COUNTER_Y = (-0.45, 0.45)
COUNTER_T = 0.03
COUNTER_TOP = TOP + COUNTER_T
COUNTER_CX = (COUNTER_X[0] + COUNTER_X[1]) / 2  # 0.29
CHILD_END_Y = COUNTER_Y[1]
COUNTER = Box(
    "counter", pos=(COUNTER_CX, 0.0, TOP + COUNTER_T / 2),
    size=(COUNTER_X[1] - COUNTER_X[0], COUNTER_Y[1] - COUNTER_Y[0], COUNTER_T), color=(0.55, 0.36, 0.20),
)

# The toddler's step-stool marker, on the table 5 mm in front of the counter's front edge, its centre STOOL_IN from
# the counter's left end (where the left arm's start pose leaves it in the top camera's view).
STOOL_R, STOOL_H = 0.03, 0.06
STOOL_IN = 0.08
STOOL = Cylinder("stool", pos=(COUNTER_X[0] - 0.005 - STOOL_R, CHILD_END_Y - STOOL_IN, TOP + STOOL_H / 2),
                 radius=STOOL_R, height=STOOL_H, color=(0.95, 0.80, 0.05))

# Distances along the counter from the toddler's end.
SAFE_DIST = 0.55  # a hazard's nearest point at least this far from the toddler's end is out of reach; the cup
# counts only once every hazard is this far away
CUP_DIST = 0.15  # the cup's centre at most this far from the toddler's end
SAFE_Y = CHILD_END_Y - SAFE_DIST  # -0.10
CUP_Y = CHILD_END_Y - CUP_DIST  # 0.30

# The divider: its right face on the 550 mm line, so anything wholly right of it is out of reach.
DIVIDER_W, DIVIDER_H = 0.02, 0.03
DIVIDER = Box("divider", pos=(COUNTER_CX, SAFE_Y + DIVIDER_W / 2, COUNTER_TOP + DIVIDER_H / 2),
              size=(COUNTER_X[1] - COUNTER_X[0], DIVIDER_W, DIVIDER_H), color=(0.10, 0.30, 0.85))

KNIFE_SIZE, KNIFE_KG = (0.15, 0.03, 0.025), 0.10  # long side front to back, lying flat
MUG_R, MUG_H, MUG_KG = 0.035, 0.08, 0.30
PILLS_R, PILLS_H, PILLS_KG = 0.02, 0.06, 0.08
CUP_R, CUP_H, CUP_KG = 0.03, 0.10, 0.15

KNIFE_START = (COUNTER_CX, 0.27)
MUG_START = (0.21, 0.15)
PILLS_START = (0.36, 0.17)
CUP_START = (COUNTER_CX, -0.34)

KNIFE = Box("knife", pos=(*KNIFE_START, COUNTER_TOP + KNIFE_SIZE[2] / 2 + 0.001), size=KNIFE_SIZE, mass=KNIFE_KG,
            color=(0.30, 0.31, 0.35))
MUG = Cylinder("mug", pos=(*MUG_START, COUNTER_TOP + MUG_H / 2 + 0.001), radius=MUG_R, height=MUG_H, mass=MUG_KG,
               color=(0.85, 0.10, 0.10))
PILLS = Cylinder("pills", pos=(*PILLS_START, COUNTER_TOP + PILLS_H / 2 + 0.001), radius=PILLS_R, height=PILLS_H,
                 mass=PILLS_KG, color=(1.00, 0.50, 0.00))
CUP = Cylinder("cup", pos=(*CUP_START, COUNTER_TOP + CUP_H / 2 + 0.001), radius=CUP_R, height=CUP_H, mass=CUP_KG,
               color=(0.10, 0.70, 0.20))

OBJECTS = [COUNTER, STOOL, DIVIDER, KNIFE, MUG, PILLS, CUP]
HAZARDS = ("knife", "mug", "pills")
LOOSE = (*HAZARDS, "cup")
SUCCESS_HOLD_S = 2.5  # done and still for 2.5 s

HAZARD_CREDIT, CUP_CREDIT = 0.2, 0.4
TILT_MAX_DEG = 15.0  # mug, bottle and cup upright: axis at most 15 degrees from vertical; knife lying down: its long
# side at most 15 degrees from level, on any of its four long faces
ON_TOL = 0.005  # "on the counter top": bottom within 5 mm of the counter's top
OFF_DROP = 0.01  # "off the counter": centre outside the counter's outline, or bottom 10 mm or more below its top


def _tilt_deg(state) -> float:
    """Angle between a cylinder's axis and the vertical (an upside-down cylinder counts as 180 degrees)."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _long_side_slope_deg(state) -> float:
    """Angle between the knife's long side (its own x axis) and the level, whichever face is down."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.asin(min(1.0, abs(2 * (x * z - w * y)))))


def _lying_or_upright(name, state) -> bool:
    """The knife lying on a long face, the others upright, within TILT_MAX_DEG."""
    tilt = _long_side_slope_deg(state) if name == "knife" else _tilt_deg(state)
    return tilt <= TILT_MAX_DEG


def _off_counter(state) -> bool:
    """Centre outside the counter's outline (on the table, the stool or the floor), or bottom below the top."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(
        not (COUNTER_X[0] <= x <= COUNTER_X[1] and COUNTER_Y[0] <= y <= COUNTER_Y[1])
        or float(state.low[2]) <= COUNTER_TOP - OFF_DROP
    )


def _resting_on_counter(name, state) -> bool:
    """At rest on the counter's top itself (not on the divider or another object), the knife lying on a long face
    and the others upright, within TILT_MAX_DEG."""
    return bool(
        not _off_counter(state)
        and abs(float(state.low[2]) - COUNTER_TOP) <= ON_TOL
        and _lying_or_upright(name, state)
        and state.resting()
    )


def _far(state) -> bool:
    """Its nearest point at least SAFE_DIST along the counter from the toddler's end: wholly right of the divider."""
    return CHILD_END_Y - float(state.high[1]) >= SAFE_DIST


def _hazard_safe(name, state) -> bool:
    """At rest, lying or upright, on the counter top, wholly right of the divider."""
    return _resting_on_counter(name, state) and _far(state)


def _cup_in_reach(objects) -> bool:
    """The cup at rest upright on the counter, centre within CUP_DIST of the toddler's end, while every hazard's
    nearest point is at least SAFE_DIST from that end."""
    cup = objects["cup"]
    return bool(
        _resting_on_counter("cup", cup)
        and CHILD_END_Y - float(cup.pos[1]) <= CUP_DIST
        and all(_far(objects[h]) for h in HAZARDS)
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.2 for each hazard (knife, mug, pill bottle) at rest on the counter top,
    the knife lying on a long face and the mug and bottle upright (within 15 degrees), with its nearest point at least
    550 mm from the toddler's end; 0.4 for the sippy cup at rest upright on the counter top, centre within 150 mm of
    the toddler's end, while every hazard's nearest point is at least 550 mm from that end. 0 overall if any of the
    four is off the counter."""
    if any(_off_counter(objects[n]) for n in LOOSE):
        return 0.0
    hazards = sum(_hazard_safe(h, objects[h]) for h in HAZARDS)
    cup = CUP_CREDIT if _cup_in_reach(objects) else 0.0
    return round(HAZARD_CREDIT * hazards + cup, 6)


def success(objects):
    """All three hazards out of reach and the cup in reach, all four still; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in LOOSE)


# The demo. One handover spot on the counter's midline, which both arms reach; only one arm is near it at a time.
# The right arm brings the cup to the handover spot; the left arm takes it to the toddler's end. Then the left arm
# brings the knife, the mug and the bottle to the handover spot one by one, and the right arm sets each down right
# of the divider, the first farthest away so no carry passes over a placed hazard. Every grip is straight down with
# the fingers closing along world y (across the knife's 30 mm width).
TRAVEL_Z = TOP + 0.16
CLEAR_Z = TOP + 0.17
DROP = 0.008  # each object is let go this far above the counter
GRIP = {"knife": 0.018, "mug": 0.045, "pills": 0.035, "cup": 0.065}  # TCP above the object's bottom
START = {"knife": KNIFE_START, "mug": MUG_START, "pills": PILLS_START, "cup": CUP_START}
HANDOVER = (COUNTER_CX, 0.0)
SAFE_PLACE = {"knife": (COUNTER_CX, -0.39), "mug": (COUNTER_CX, -0.28), "pills": (COUNTER_CX, -0.17)}
CUP_PLACE = (COUNTER_CX, 0.38)
LEFT_SIDE = (COUNTER_CX, 0.20, TRAVEL_Z)  # where the left arm waits while the right arm is at the handover spot
RIGHT_SIDE = (COUNTER_CX, -0.20, TRAVEL_Z)  # and the other way round
# The left arm parks over the empty middle of the counter, right of the cup (in the final top view it hides the cup).
LEFT_PARK = (COUNTER_CX, 0.12, TRAVEL_Z)
RIGHT_PARK = (0.14, -0.25, CLEAR_Z)
LIFTED_S, ARRIVE_S, PP_S = 5.0, 6.8, 9.6  # in a pick_place_waypoints plan: lifted, above the place, done
CLEAR_S = 1.0  # move from above the handover spot to the arm's side


def _pick(name, xy):
    return (*xy, COUNTER_TOP + GRIP[name])


def _place(name, xy):
    return (*xy, COUNTER_TOP + GRIP[name] + DROP)


def _t(plan):
    return sum(w.duration for w in plan)


def _wait_until(plan, t, pos, label):
    if t > _t(plan) + 1e-9:
        plan.append(Waypoint(t - _t(plan), pos, FINGER_OPEN, label))


def demo():
    left, right = [], []
    # The right arm: the cup to the handover spot, then aside.
    right += pick_place_waypoints(_pick("cup", CUP_START), _place("cup", HANDOVER), travel_z=TRAVEL_Z)
    right.append(Waypoint(CLEAR_S, RIGHT_SIDE, FINGER_OPEN, "aside"))
    free_at = _t(right)  # when the handover spot is clear
    # The left arm: the cup to the toddler's end.
    _wait_until(left, free_at, LEFT_SIDE, "wait for the cup")
    left += pick_place_waypoints(_pick("cup", HANDOVER), _place("cup", CUP_PLACE), travel_z=TRAVEL_Z)
    free_at = _t(left) - PP_S + ARRIVE_S
    for name in HAZARDS:
        # The left arm brings the hazard: it may carry it in once the spot is clear.
        _wait_until(left, max(_t(left), free_at - LIFTED_S), LEFT_SIDE, f"wait to bring the {name}")
        left += pick_place_waypoints(_pick(name, START[name]), _place(name, HANDOVER), travel_z=TRAVEL_Z)
        left.append(Waypoint(CLEAR_S, LEFT_SIDE, FINGER_OPEN, "aside"))
        # The right arm takes it once the left arm is aside, and sets it down out of reach.
        _wait_until(right, _t(left), RIGHT_SIDE, f"wait for the {name}")
        right += pick_place_waypoints(_pick(name, HANDOVER), _place(name, SAFE_PLACE[name]), travel_z=TRAVEL_Z)
        free_at = _t(right) - PP_S + ARRIVE_S
    # The left arm parks once the right arm has carried the bottle away from the handover spot.
    _wait_until(left, free_at, LEFT_SIDE, "wait for the handover spot to clear")
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_OPEN, "park"))
    right.append(Waypoint(0.8, (*SAFE_PLACE["pills"], CLEAR_Z), FINGER_OPEN, "lift clear of the bottle"))
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_OPEN, "park"))
    return {"left": left, "right": right}
