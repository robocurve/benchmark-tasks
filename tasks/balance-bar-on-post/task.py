"""On the station table, between the two arms:

- A square wooden post (50 x 50 mm, 110 mm tall) fixed upright to the table on the midline, a little way out
  from the arms. Its flat top is the only support for the bar.
- Nearer the arms, a loose pine bar (300 mm long, 64 mm wide, 19 mm thick) lying flat on the table with its
  long side running from left to right.
- Level with the post, one loose 40 mm aluminium cube on the table on each side: one on the left, one on the
  right, both clear of where the bar's ends will be.

Nothing is attached to anything: the bar only rests on the post. Set level and centred across the post top, the
empty bar balances; a cube on one end alone tips it, and a cube on each end leaves it level.

The post is fixed to the table; the bar and the two cubes move.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

GRIPPER_MAX_OPENING = 0.094  # pad gap at FINGER_OPEN (yam_arena.constants; the embodiment's test_constants.py)
SPAWN_GAP = 0.001  # loose objects start this far above their support and settle onto it

# -- the post: a fixed square column on the midline -----------------------------------------------------------
POST_X, POST_Y = 0.25, 0.0
POST_W = 0.050  # its top face along the bar's length (y) and across it (x)
POST_H = 0.110
POST_TOP = TOP + POST_H

# -- the bar --------------------------------------------------------------------------------------------------
BAR_W, BAR_L, BAR_T = 0.064, 0.300, 0.019  # across (x), length (y), thickness: a 1x3 pine board cut to length
BAR_MASS = 0.165  # pine at about 450 kg/m^3
BAR_START = (0.14, 0.0)
BAR_TOP = POST_TOP + BAR_T  # top face of the bar balanced on the post

# -- the two cubes --------------------------------------------------------------------------------------------
CUBE = 0.040
CUBE_MASS = 0.173  # solid 6061 aluminium, 2.70 g/cm^3
CUBE_X = POST_X
CUBE_Y0 = 0.22  # start spots at (CUBE_X, +-CUBE_Y0)
SPOT_D = 0.125  # where the demo puts each cube: centre this far from the bar's middle, outer face 5 mm in from the end

POST_WOOD = (0.36, 0.25, 0.16)
PINE = (0.80, 0.66, 0.45)
ALUMINIUM = (0.62, 0.64, 0.68)

POST = Box("post", pos=(POST_X, POST_Y, TOP + POST_H / 2), size=(POST_W, POST_W, POST_H), color=POST_WOOD)
BAR = Box(
    "bar",
    pos=(BAR_START[0], BAR_START[1], TOP + BAR_T / 2 + SPAWN_GAP),
    size=(BAR_W, BAR_L, BAR_T),
    mass=BAR_MASS,
    color=PINE,
)
CUBES = [
    Box(name, pos=(CUBE_X, s * CUBE_Y0, TOP + CUBE / 2 + SPAWN_GAP), size=(CUBE,) * 3, mass=CUBE_MASS, color=ALUMINIUM)
    for name, s in (("block_left", 1.0), ("block_right", -1.0))
]
CUBE_NAMES = tuple(c.name for c in CUBES)
LOOSE = (BAR.name, *CUBE_NAMES)

OBJECTS = [POST, BAR, *CUBES]

# -- scoring --------------------------------------------------------------------------------------------------
TILT_MAX = math.radians(5.0)  # the bar's top face within 5 deg of level
SEAT_ABOVE = 0.003  # "resting on": a bottom face within 3 mm above the top face under it ...
SEAT_BELOW = 0.002  # ... or up to 2 mm into it (contact penetration)
CENTRE_ALONG = 0.010  # the bar's middle within 10 mm of the post's axis along the bar ...
CENTRE_ACROSS = 0.025  # ... and within 25 mm across it (the post then lies under the bar's width)
END_ZONE = 0.050  # a cube "on an end": its centre within 50 mm of that end (outer face within 30 mm of it)
STACK_ABOVE = 2 * CUBE  # a cube counts as carried by the bar up to one cube height above a seated one


def rotation(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def bar_frame(objects):
    """(centre, rotation) of the bar; columns of the rotation: across (x), along (y), normal (z)."""
    b = objects[BAR.name]
    return np.asarray(b.pos, dtype=float), rotation(b.quat_xyzw)


def tilt(objects):
    """Angle (rad) between the bar's top-face normal and the vertical."""
    _, r = bar_frame(objects)
    return math.acos(min(1.0, abs(float(r[2, 2]))))


def on_floor(objects):
    """Any loose object with its centre below the table top: dropped or falling off the table."""
    return any(objects[n].pos[2] < TOP for n in LOOSE)


def bar_balanced(objects):
    """The bar rests level on the post top, centred across it, and is still."""
    c, r = bar_frame(objects)
    if r[2, 2] <= 0 or tilt(objects) >= TILT_MAX or not objects[BAR.name].resting():
        return False
    gap = c[2] - BAR_T / 2 * math.cos(tilt(objects)) - POST_TOP  # bar underside above the post top, at the middle
    if not -SEAT_BELOW <= gap <= SEAT_ABOVE:
        return False
    d = np.array([POST_X, POST_Y]) - c[:2]  # post axis relative to the bar's middle, horizontally
    along, across = float(d @ r[:2, 1]), float(d @ r[:2, 0])
    return abs(along) <= CENTRE_ALONG and abs(across) <= CENTRE_ACROSS


def in_bar_frame(objects, name):
    c, r = bar_frame(objects)
    return r.T @ (np.asarray(objects[name].pos, dtype=float) - c)


def over_bar(objects, name):
    """The cube's centre over the bar's top face, from seated on it up to one cube height higher (stacked)."""
    u = in_bar_frame(objects, name)
    seat = BAR_T / 2 + CUBE / 2
    return abs(u[0]) <= BAR_W / 2 and abs(u[1]) <= BAR_L / 2 and seat - SEAT_BELOW <= u[2] <= seat + STACK_ABOVE


def seated_on_end(objects, name):
    """The cube resting directly on the bar's top face with its centre within END_ZONE of an end; +1/-1 for the
    end (the bar's +y or -y end), 0 if not."""
    u = in_bar_frame(objects, name)
    seat = BAR_T / 2 + CUBE / 2
    if not (abs(u[0]) <= BAR_W / 2 and abs(u[1]) <= BAR_L / 2 and seat - SEAT_BELOW <= u[2] <= seat + SEAT_ABOVE):
        return 0
    if not objects[name].resting() or abs(u[1]) < BAR_L / 2 - END_ZONE:
        return 0
    return 1 if u[1] > 0 else -1


FULL, ONE_LOADED, EMPTY_BALANCED = 1.0, 0.5, 0.3


def score(objects):
    """1.0: the bar balanced on the post (level within 5 deg, underside on the post top, middle within 10 mm of the
    post's axis along the bar) with one cube resting on each end, everything still. 0.5: the bar balanced with
    cubes on it but not one on each end. 0.3: the bar balanced and empty. 0: the bar not balanced on the post,
    both cubes on the same half of the bar, or anything on the floor."""
    if on_floor(objects) or not bar_balanced(objects):
        return 0.0
    over = [n for n in CUBE_NAMES if over_bar(objects, n)]
    if not over:
        return EMPTY_BALANCED
    if len(over) == 2:
        sides = [in_bar_frame(objects, n)[1] >= 0 for n in over]
        if sides[0] == sides[1]:
            return 0.0
        ends = sorted(seated_on_end(objects, n) for n in over)
        if ends == [-1, 1]:
            return FULL
    return ONE_LOADED


def success(objects):
    return score(objects) == FULL


GOAL_BAR = np.array([POST_X, POST_Y, POST_TOP + BAR_T / 2])
SPOTS = [np.array([POST_X, POST_Y + s * SPOT_D, BAR_TOP + CUBE / 2]) for s in (1.0, -1.0)]


def distance(objects):
    """Metres from the bar's middle to its place on the post, plus the two cubes' distances to the two end spots
    (whichever cube goes to whichever end, the smaller sum)."""
    total = float(np.linalg.norm(np.asarray(objects[BAR.name].pos) - GOAL_BAR))
    p = [np.asarray(objects[n].pos, dtype=float) for n in CUBE_NAMES]
    pairings = (
        np.linalg.norm(p[0] - SPOTS[0]) + np.linalg.norm(p[1] - SPOTS[1]),
        np.linalg.norm(p[0] - SPOTS[1]) + np.linalg.norm(p[1] - SPOTS[0]),
    )
    return total + float(min(pairings))


# -- scripted demo ------------------------------------------------------------------------------------------
# The left arm sets the bar on the post (fingers closing along x, across the bar's width: yaw -90 deg), then both
# arms carry one cube each over its end and let go at the same moment (fingers closing along y: yaw 0, the only
# cube yaw both arms reach at these heights). The pad tips are 13 mm past the TCP.
BAR_YAW, CUBE_YAW = -math.pi / 2, 0.0
SIGN = {"left": 1.0, "right": -1.0}
BAR_GRIP_ABOVE_CENTRE = 0.007  # pad tips 6 mm below the bar's middle, 3.5 mm above its underside
CUBE_GRIP_ABOVE_CENTRE = 0.005  # pad tips 8 mm below the cube's centre
GRIP_AIM = 0.006  # place aim toward the gripper's +y: the measured grip offset runs toward its -y
BAR_RELEASE_GAP = 0.003  # the bar is let go with its underside this far above the post top
CUBE_RELEASE_GAP = 0.005  # a cube is let go with its bottom this far above the bar's top
CLEAR = 0.015  # a carried part's bottom this far above what it passes over
LIFT_AFTER_RELEASE = 0.03

BAR_PICK_Z = TOP + BAR_T / 2 + BAR_GRIP_ABOVE_CENTRE
BAR_CARRY_Z = POST_TOP + CLEAR + BAR_T / 2 + BAR_GRIP_ABOVE_CENTRE
BAR_RELEASE_Z = POST_TOP + BAR_RELEASE_GAP + BAR_T / 2 + BAR_GRIP_ABOVE_CENTRE
CUBE_PICK_Z = TOP + CUBE / 2 + CUBE_GRIP_ABOVE_CENTRE
CUBE_CARRY_Z = BAR_TOP + CLEAR + CUBE / 2 + CUBE_GRIP_ABOVE_CENTRE
CUBE_RELEASE_Z = BAR_TOP + CUBE_RELEASE_GAP + CUBE / 2 + CUBE_GRIP_ABOVE_CENTRE
HOVER_Z = TOP + 0.06
PARK = {"left": (0.20, 0.25, TOP + 0.12), "right": (0.20, -0.25, TOP + 0.12)}
END_PARK = {"left": (0.15, 0.28, TOP + 0.15), "right": (0.15, -0.28, TOP + 0.15)}
STEADY_S = 0.5  # both hands still over the ends before letting go
FINAL_HOLD_S = 2.0


def aim(yaw):
    """World (x, y) offset of GRIP_AIM toward the gripper's +y at this yaw."""
    return (-math.sin(yaw) * GRIP_AIM, math.cos(yaw) * GRIP_AIM)


def bar_place_xy():
    ax, ay = aim(BAR_YAW)
    return (POST_X + ax, POST_Y + ay)


def cube_place_xy(side):
    ax, ay = aim(CUBE_YAW)
    return (POST_X + ax, POST_Y + SIGN[side] * SPOT_D + ay)


class _Track:
    """One arm's waypoint list with its running end time."""

    def __init__(self, side, yaw):
        self.side, self.yaw, self.wps, self.t = side, yaw, [], 0.0
        self.pos, self.finger = PARK[side], FINGER_OPEN

    def go(self, duration, pos=None, finger=None, label="", yaw=None):
        if duration <= 0:
            return
        self.pos = self.pos if pos is None else tuple(float(v) for v in pos)
        self.finger = self.finger if finger is None else finger
        self.yaw = self.yaw if yaw is None else yaw
        self.wps.append(Waypoint(duration, self.pos, self.finger, label, self.yaw))
        self.t += duration

    def wait_until(self, t, label="wait"):
        self.go(t - self.t, label=label)

    def pick_cube(self):
        x, y = CUBE_X, SIGN[self.side] * CUBE_Y0
        self.go(1.6, (x, y, self.pos[2]), FINGER_OPEN, f"over the {self.side} cube", yaw=CUBE_YAW)
        self.go(1.4, (x, y, CUBE_PICK_Z), label="descend")
        self.go(0.7, finger=FINGER_CLOSED, label="close")
        self.go(1.6, (x, y, CUBE_CARRY_Z), label="lift the cube")

    def release(self, what):
        x, y, z = self.pos
        self.go(0.6, finger=FINGER_OPEN, label=f"let go of {what}")
        self.go(0.8, (x, y, z + LIFT_AFTER_RELEASE), label="lift off")


def _plan(load_right=True):
    left, right = _Track("left", BAR_YAW), _Track("right", CUBE_YAW)
    left.go(1.5, PARK["left"], FINGER_OPEN, "to the park spot")
    right.go(1.5, PARK["right"], FINGER_OPEN, "to the park spot")

    # the bar onto the post
    bx, by = BAR_START
    px, py = bar_place_xy()
    left.go(1.5, (bx, by, HOVER_Z), label="over the bar")
    left.go(1.2, (bx, by, BAR_PICK_Z), label="descend")
    left.go(0.7, finger=FINGER_CLOSED, label="close")
    left.go(1.6, (bx, by, BAR_CARRY_Z), label="lift the bar")
    left.go(1.6, (px, py, BAR_CARRY_Z), label="carry over the post")
    left.go(1.2, (px, py, BAR_RELEASE_Z), label="lower onto the post")
    left.release("the bar on the post")

    # meanwhile the right arm takes its cube up beside the bar's right end
    if load_right:
        right.go(0.5, label="wait")
        right.pick_cube()

    left.pick_cube()
    arms = (left, right) if load_right else (left,)
    sync = max(tr.t for tr in arms)
    for tr in arms:
        tr.wait_until(sync, "wait for the other arm")
    for tr in arms:  # identical durations from here on: the two cubes come down together
        x, y = cube_place_xy(tr.side)
        tr.go(1.5, (x, y, CUBE_CARRY_Z), label=f"carry over the {tr.side} end")
        tr.go(1.0, (x, y, CUBE_RELEASE_Z), label=f"lower onto the {tr.side} end")
        tr.go(STEADY_S, label="steady")
        tr.release(f"the {tr.side} cube on the bar")
        tr.go(1.5, END_PARK[tr.side], label="to the final park spot")
        tr.go(FINAL_HOLD_S, label="hold")
    if not load_right:
        right.wait_until(left.t, "stay parked")
    return {"left": left.wps, "right": right.wps}


def demo():
    """The left arm lifts the bar off the table and sets it level and centred on the post, while the right arm
    picks up its cube and holds it beside the bar's right end. The left arm then picks up its cube. Both arms carry
    their cubes over the two ends with the same durations, lower them, and let go at the same moment, so neither
    end is ever loaded alone. Both hands rise 3 cm, move off to the sides and hold still."""
    return _plan(load_right=True)


def one_end_first_demo():
    """A check, not a hook (README "Verify on Brev"): the same plan with the right arm staying parked, so the left
    cube is set on its end alone. The bar should tip onto its left end and shed the cube; the run should end at 0.3
    (the empty bar back level, the cube on the table) or 0, never 0.5 or 1."""
    return _plan(load_right=False)
