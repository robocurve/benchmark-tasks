"""On the station table stand two small light stands, one in front of each arm, fixed to the table. Each has a black
round post (30 mm across, 38 cm tall) and three dark grey legs (flat bars 16 cm long, 40 mm wide, 20 mm tall) lying
on the table and running out from the post 120 degrees apart. A short yellow boom stub (45 x 20 x 20 mm) sticks out
from the top of each post over one of its legs. The left stand's post is 40 cm in front of the left arm's base and its
boom points back toward the arms; the right stand's post is 30 cm in front of the right arm's base and 10 cm further
out to the right, and its boom points toward the middle of the table. Two red sandbags (rigid plastic boxes 70 x 50 x 35 mm filled with steel shot to 400 g each) lie flat between
the arms, 17 cm in front of the arm bases and 12 cm either side of the centre line, their long sides pointing away
from the arms. The stands are fixed; the bags are loose. Done, each stand has a bag lying flat (either 70 x 50 mm
face down) along the leg under its boom, touching it, its centre over the leg 7 to 13 cm out from the post, and no
bag lies on any other leg. Set convention puts the sandbag on the leg under the boom.
"""

import math

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# The stands: a round post, three low legs 120 degrees apart starting at the post's surface, and a boom stub near
# the top of the post over one leg. Headings are degrees from world +x (away from the arms) toward +y (left).
POST_R, POST_H = 0.015, 0.38
LEG_LEN, LEG_W, LEG_H = 0.16, 0.04, 0.02
LEG_IN = POST_R  # a leg starts at the post's surface ...
LEG_END = LEG_IN + LEG_LEN  # ... and ends 175 mm from the post's axis
LEG_TOP = TOP + LEG_H
BOOM_LEN, BOOM_W = 0.045, 0.02  # the stub reaches 60 mm from the post's axis, short of the bag zone
BOOM_Z = TOP + POST_H - BOOM_W / 2
POST_BLACK = (0.08, 0.08, 0.09)
LEG_GREY = (0.26, 0.27, 0.30)
BOOM_YELLOW = (0.95, 0.78, 0.05)

STANDS = {
    "l": {"post": (0.40, 0.305), "boom": 180, "legs": (180, 300, 60)},  # boom toward the arms
    "r": {"post": (0.30, -0.405), "boom": 90, "legs": (90, 210, 330)},  # boom toward the middle of the table
}  # the posts stand clear of the arms' ready pose, over (0.32, +-0.30); each boom-leg spot is in line with its arm


def _out(post, heading_deg, r):
    """The point ``r`` metres out from a post's axis along a heading."""
    a = math.radians(heading_deg)
    return (post[0] + r * math.cos(a), post[1] + r * math.sin(a))


def _stand(side, spec):
    post, out = spec["post"], []
    out.append(Cylinder(f"post_{side}", pos=(*post, TOP + POST_H / 2), radius=POST_R, height=POST_H,
                        color=POST_BLACK))
    for h in spec["legs"]:
        out.append(Box(f"leg_{side}_{h:03d}", pos=(*_out(post, h, LEG_IN + LEG_LEN / 2), TOP + LEG_H / 2),
                       size=(LEG_LEN, LEG_W, LEG_H), color=LEG_GREY, yaw=math.radians(h)))
    out.append(Box(f"boom_{side}", pos=(*_out(post, spec["boom"], POST_R + BOOM_LEN / 2), BOOM_Z),
                   size=(BOOM_LEN, BOOM_W, BOOM_W), color=BOOM_YELLOW, yaw=math.radians(spec["boom"])))
    return out


FIXTURES = [obj for side, spec in STANDS.items() for obj in _stand(side, spec)]

# The bags: long side along the box's own x.
BAG_SIZE, BAG_KG = (0.07, 0.05, 0.035), 0.40
BAG_RED = (0.78, 0.10, 0.07)
BAG_START = {"bag_l": (0.17, 0.12), "bag_r": (0.17, -0.12)}
BAGS = [
    Box(name, pos=(x, y, TOP + BAG_SIZE[2] / 2 + 0.001), size=BAG_SIZE, mass=BAG_KG, color=BAG_RED)
    for name, (x, y) in BAG_START.items()
]  # each starts 1 mm above the table, and settles
BAG_NAMES = tuple(BAG_START)

OBJECTS = [*FIXTURES, *BAGS]
SUCCESS_HOLD_S = 2.5  # both bags placed and still for 2.5 s

# Where the bag goes on each stand: 100 mm out along the boom leg, centred on it.
SPOT_OUT = 0.10
L_SPOT_X, L_SPOT_Y = _out(STANDS["l"]["post"], STANDS["l"]["boom"], SPOT_OUT)
R_SPOT_X, R_SPOT_Y = _out(STANDS["r"]["post"], STANDS["r"]["boom"], SPOT_OUT)
BAG_ON_LEG_Z = LEG_TOP + BAG_SIZE[2] / 2  # a flat bag's centre height on a leg

ZONE_OUT = (0.07, 0.13)  # the bag's centre 70 to 130 mm out from the post's axis, along the boom leg
LATERAL_TOL = 0.02  # the bag's centre at most 20 mm to either side of the leg's centre line (over the leg)
BOTTOM_ABOVE, BOTTOM_BELOW = 0.0015, 0.003  # the bag's lowest point 1.5 mm above to 3 mm below the leg's top:
# touching it (contact offsets are under 1 mm), so a bag held just above the leg does not count
FLAT_DEG = 10.0  # the bag's 70 x 50 mm face (either one) down, within 10 degrees of level
ALIGN_DEG = 20.0  # the bag's long side within 20 degrees of the leg's direction
ON_LEG_CREDIT, ALIGN_CREDIT = 0.35, 0.15  # per stand


def _tilt_deg(state) -> float:
    """Angle between the box's own z axis (normal to its 70 x 50 mm faces) and the vertical, either way up: 0 when
    the bag lies on either 70 x 50 mm face, 90 when it rests on a side or end face."""
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def _long_side_deg(state) -> float:
    """Heading of the bag's long side (its own x axis) in the horizontal plane, degrees from world +x."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return math.degrees(math.atan2(2 * (x * y + z * w), 1 - 2 * (y * y + z * z)))


def _along_across(state, post, heading_deg):
    """The bag centre's distance out along a leg's centre line from the post's axis, and to the side of it."""
    dx, dy = float(state.pos[0]) - post[0], float(state.pos[1]) - post[1]
    a = math.radians(heading_deg)
    return dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)


def _bottom_on_leg_top(state) -> bool:
    return LEG_TOP - BOTTOM_BELOW <= float(state.low[2]) <= LEG_TOP + BOTTOM_ABOVE


def _on_boom_leg(state, spec) -> bool:
    """Lying on a 70 x 50 mm face on the leg under the boom, its centre over the leg and 70 to 130 mm out."""
    along, across = _along_across(state, spec["post"], spec["boom"])
    return bool(
        _bottom_on_leg_top(state)
        and _tilt_deg(state) <= FLAT_DEG
        and abs(across) <= LATERAL_TOL
        and ZONE_OUT[0] <= along <= ZONE_OUT[1]
    )


def _aligned(state, spec) -> bool:
    d = (_long_side_deg(state) - spec["boom"] + 90.0) % 180.0 - 90.0
    return abs(d) <= ALIGN_DEG


def _on_wrong_leg(state) -> bool:
    """Lying on a leg that is not under its stand's boom: its lowest point at that leg's top and its centre over
    the leg, anywhere from the post to the leg's end."""
    if not _bottom_on_leg_top(state):
        return False
    for spec in STANDS.values():
        for h in spec["legs"]:
            if h == spec["boom"]:
                continue
            along, across = _along_across(state, spec["post"], h)
            if abs(across) <= LATERAL_TOL and LEG_IN <= along <= LEG_END:
                return True
    return False


def _stand_credit(objects, spec) -> float:
    best = 0.0
    for name in BAG_NAMES:
        bag = objects[name]
        if _on_boom_leg(bag, spec):
            best = max(best, ON_LEG_CREDIT + (ALIGN_CREDIT if _aligned(bag, spec) else 0.0))
    return best


def score(objects):
    """Partial credit (success scores 1.0), per stand: 0.35 for a bag lying on the leg under its boom (lowest point
    1.5 mm above to 3 mm below the leg's top, a 70 x 50 mm face down, either one, within 10 degrees of level, centre
    within 20 mm of the leg's centre line and 70 to 130 mm out from the post), plus 0.15 if that bag's long side is
    within 20 degrees of the leg's direction. 0 overall if any bag lies on a leg that is not under its stand's
    boom (lowest point in the same band at that leg's top, centre within 20 mm of its centre line). A bag still in
    the gripper counts if its lowest point is in the band: the score does not check release."""
    if any(_on_wrong_leg(objects[name]) for name in BAG_NAMES):
        return 0.0
    return round(sum(_stand_credit(objects, spec) for spec in STANDS.values()), 6)


def success(objects):
    """Both stands weighted on the boom leg, the bags lengthwise along it; must hold for SUCCESS_HOLD_S."""
    return score(objects) == 1.0 and all(objects[name].resting() for name in BAG_NAMES)


# The demo: both arms at once. The left arm picks up the left bag across its 50 mm width (fingers closing along
# world y) and lays it along the left stand's boom leg, which runs toward the arms, so the grip keeps yaw 0. The
# right arm picks up the right bag the same way and turns it a quarter turn while carrying it, so it lies along the
# right stand's boom leg, which runs across the table.
TRAVEL_Z = TOP + 0.15  # a carried bag's bottom clears the legs by 11 cm
BAG_GRIP = 0.020  # TCP above a bag's bottom: the pad tips 7 mm above what it stands on, the pads cover 28 mm
DROP = 0.008  # each bag is let go this far above the leg's top
PLACE_Z = LEG_TOP + DROP + BAG_GRIP
PARK_Z = TOP + 0.17  # after the retreat each arm parks over its bag's starting mark, clear of the stands
PARK_S = 2.0


def demo():
    plans = {}
    for arm, name, spot, yaw in (
        ("left", "bag_l", (L_SPOT_X, L_SPOT_Y), 0.0),
        ("right", "bag_r", (R_SPOT_X, R_SPOT_Y), math.pi / 2),
    ):
        x, y = BAG_START[name]
        plan = pick_place_waypoints((x, y, TOP + BAG_GRIP), (*spot, PLACE_Z), travel_z=TRAVEL_Z, place_yaw=yaw)
        plan.append(Waypoint(PARK_S, (x, y, PARK_Z), FINGER_OPEN, "park", yaw))
        plans[arm] = plan
    return plans
