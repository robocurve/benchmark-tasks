"""On the table: five fixed grey posts (6 cm tall, 4 cm across) and two soup cans. Two taped zones (yellow tape
outlines, 24 cm x 15 cm, one in front of each arm) are on the table. Each can starts inside its own arm's zone.
Stand the cans so that every post is within 18 cm of one of the cans (centre to centre), each can is inside its
own arm's taped zone (centre inside the tape), and no can is closer than 12 cm to any post or to the other can.
"""

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
GREY = (0.45, 0.45, 0.48)
TAPE = (0.95, 0.80, 0.10)

# Posts: (x, y) in metres, fixed.
POSTS = {
    "post_1": (0.272, -0.092),
    "post_2": (0.252, 0.043),
    "post_3": (0.436, 0.235),
    "post_4": (0.449, -0.056),
    "post_5": (0.397, 0.120),
}
POST_R, POST_H = 0.02, 0.06

# Taped zones: (x_min, x_max, y_min, y_max), metres. The left arm's is at +y, the right arm's at -y.
ZONE_L = (0.18, 0.42, 0.19, 0.34)
ZONE_R = (0.18, 0.40, -0.34, -0.19)
TAPE_W, TAPE_H = 0.015, 0.002

CAN_START = {"can_left": (0.31, 0.28), "can_right": (0.29, -0.23)}  # each in front of the arm that places it
CAN_EXTENT = 0.067
COVER, KEEP_OFF = 0.18, 0.12


def _tape(prefix, z):
    x0, x1, y0, y1 = z
    cx, cy, w, h = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    return [
        Box(f"{prefix}_n", pos=(x1, cy, TOP + TAPE_H / 2), size=(TAPE_W, h + TAPE_W, TAPE_H), color=TAPE),
        Box(f"{prefix}_s", pos=(x0, cy, TOP + TAPE_H / 2), size=(TAPE_W, h + TAPE_W, TAPE_H), color=TAPE),
        Box(f"{prefix}_e", pos=(cx, y1, TOP + TAPE_H / 2), size=(w + TAPE_W, TAPE_W, TAPE_H), color=TAPE),
        Box(f"{prefix}_w", pos=(cx, y0, TOP + TAPE_H / 2), size=(w + TAPE_W, TAPE_W, TAPE_H), color=TAPE),
    ]


OBJECTS = [
    *[Cylinder(n, pos=(x, y, TOP + POST_H / 2), radius=POST_R, height=POST_H, color=GREY) for n, (x, y) in POSTS.items()],
    *_tape("tape_l", ZONE_L),
    *_tape("tape_r", ZONE_R),
    *[Asset(n, "alphabet_soup_can_hope_robolab", xy=(float(x), float(y)), extent=CAN_EXTENT, mass=0.2)
      for n, (x, y) in CAN_START.items()],
]


def _inside(x, y, z):
    return bool(z[0] <= x <= z[1] and z[2] <= y <= z[3])


def _upright(o):
    qx, qy = float(o.quat_xyzw[0]), float(o.quat_xyzw[1])
    return bool(1 - 2 * (qx * qx + qy * qy) > 0.99)


def _dist(a, b):
    return float(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5)


def score(objects):
    """Scored at the end. Conditions that must hold for any credit: one can has its centre inside each taped zone,
    both cans are upright and resting on the table, and each can is at least 12 cm (centre to centre) from every
    post and from the other can. Then 1.0 if every post is within 18 cm of its nearest can, 0.5 if exactly one
    post is farther than 18 cm, otherwise 0."""
    a, b = objects["can_left"], objects["can_right"]
    ca = (float(a.pos[0]), float(a.pos[1]))
    cb = (float(b.pos[0]), float(b.pos[1]))
    in_zones = (_inside(*ca, ZONE_L) and _inside(*cb, ZONE_R)) or (_inside(*ca, ZONE_R) and _inside(*cb, ZONE_L))
    upright = _upright(a) and _upright(b)
    on_table = float(a.low[2]) < TOP + 0.01 and float(b.low[2]) < TOP + 0.01
    dists = [(_dist(ca, p), _dist(cb, p)) for p in POSTS.values()]
    clear = all(min(d) >= KEEP_OFF for d in dists) and _dist(ca, cb) >= KEEP_OFF
    far = sum(1 for d in dists if min(d) > COVER)
    if not (in_zones and upright and on_table and clear):
        return 0.0
    return 1.0 if far == 0 else 0.5 if far == 1 else 0.0


TRAVEL_Z = TOP + 0.16
TARGET = {"can_left": (0.274, 0.206), "can_right": (0.384, -0.206)}
PARK = {"left": (0.16, 0.32, TOP + 0.20), "right": (0.12, -0.38, TOP + 0.20)}
CAN_H = 0.079  # measured bounding-box height of the can


def _carry(name):
    x, y = CAN_START[name]
    tip_up = min(0.4 * CAN_H, 0.035)  # pad tips this far above the table
    z = TOP + PAD_TIP_PAST_TCP + tip_up
    tx, ty = TARGET[name]
    return pick_place_waypoints((x, y, z), (tx, ty, z + 0.003), travel_z=TRAVEL_Z)


def demo():
    """Each arm carries its own can to the spot where it covers its posts, then moves clear of the top camera."""
    left, right = _carry("can_left"), _carry("can_right")
    left.append(Waypoint(2.0, PARK["left"], FINGER_OPEN, "move clear"))
    right.append(Waypoint(2.0, PARK["right"], FINGER_OPEN, "move clear"))
    return {"left": left, "right": right}
