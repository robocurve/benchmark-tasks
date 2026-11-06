"""On the station table, from left to right:

- In front of the left arm, a wooden ball rack: a 12 mm board lying on the table with three 50 x 50 mm square
  holes in a row running away from the arms. A wooden ball sits in each hole.
- Between the arms, an empty square display tray: a 5 mm floor with a 25 mm rim, 168 x 168 mm inside.
- In front of the right arm, a second ball rack of the same kind with two holes, a ball in each.

The five balls are identical: 76 mm across, one colour. The tray and both racks are fixed to the table; only the
balls move. Four balls fit side by side on the tray floor, two by two; a fifth fits only on top of them.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the balls (m) ---------------------------------------------------------------------------------------------
GRIPPER_MAX_OPENING = 0.0939  # pad gap at FINGER_OPEN: 2 x 0.04695 m (yam_arena.constants)
D = 0.076  # 3 in hardwood craft ball
R = D / 2
DENSITY = 700.0  # kg/m^3, beech or maple
BALL_MASS = round(DENSITY * 4 / 3 * math.pi * R**3, 3)  # 0.161 kg
SPAWN_GAP = 0.001  # balls start this far above their seat and settle onto it

# -- display tray (between the arms) ---------------------------------------------------------------------------
TRAY_X, TRAY_Y = 0.30, 0.0
TRAY_IN = 0.168  # square interior
WALL = 0.010
FLOOR_H = 0.005
WALL_H = 0.025  # rim height above the tray floor: below the pads at every release
FLOOR_TOP = TOP + FLOOR_H
RIM_TOP = FLOOR_TOP + WALL_H
TRAY_OUT = TRAY_IN + 2 * WALL

# A ball on the floor touches the rim at the rim's top inner edge (the rim is lower than the ball's equator), so
# its centre can come no closer than EDGE_REACH to the rim's inner face.
EDGE_REACH = math.sqrt(R**2 - (R - WALL_H) ** 2)  # 35.7 mm
CORNER = TRAY_IN / 2 - EDGE_REACH  # 48.3 mm: centre offset (in x and in y) of a ball against both rims
SPREAD = 2 * CORNER - D  # 20.6 mm: how far the bottom layer can open beyond touching
# The top ball's centre above the bottom balls' centres when it rests on all four:
TOP_RISE_TIGHT = D / math.sqrt(2)  # 53.7 mm, bottom balls touching each other
TOP_RISE_CORNERS = math.sqrt(D**2 - 2 * CORNER**2)  # 33.3 mm, bottom balls pushed into the corners
FALL_THROUGH_SPREAD = (math.sqrt(2) - 1) * D  # 31.5 mm: a wider spread would let the top ball drop through

# -- ball racks (12 mm boards on the table with square holes) ---------------------------------------------------
RACK_T = 0.012
HOLE = 0.050
RACK_SIDE = 0.020  # board on each side of the holes (y)
RACK_END = 0.025  # board beyond the first and last hole (x)
RACK_PITCH = 0.095
RACK_TOP = TOP + RACK_T
SEAT_Z = RACK_TOP + math.sqrt(R**2 - (HOLE / 2) ** 2)  # a ball's centre sitting in a hole: 28.6 mm over the board
RACK_L_Y, RACK_L_X = 0.215, (0.160, 0.255, 0.350)
RACK_R_Y, RACK_R_X = -0.215, (0.210, 0.305)

TRAY_WOOD = (0.55, 0.40, 0.26)
RACK_WOOD = (0.30, 0.30, 0.32)
BALL_RED = (0.78, 0.16, 0.12)


def _tray():
    """Floor slab and four rim strips standing on it, all fixed."""
    cx, cy, w = TRAY_X, TRAY_Y, TRAY_IN
    z = FLOOR_TOP + WALL_H / 2
    return [
        Box("tray_floor", pos=(cx, cy, TOP + FLOOR_H / 2), size=(TRAY_OUT, TRAY_OUT, FLOOR_H), color=TRAY_WOOD),
        Box("tray_rim_near", pos=(cx - (w + WALL) / 2, cy, z), size=(WALL, TRAY_OUT, WALL_H), color=TRAY_WOOD),
        Box("tray_rim_far", pos=(cx + (w + WALL) / 2, cy, z), size=(WALL, TRAY_OUT, WALL_H), color=TRAY_WOOD),
        Box("tray_rim_left", pos=(cx, cy + (w + WALL) / 2, z), size=(w, WALL, WALL_H), color=TRAY_WOOD),
        Box("tray_rim_right", pos=(cx, cy - (w + WALL) / 2, z), size=(w, WALL, WALL_H), color=TRAY_WOOD),
    ]


def _rack(prefix, y, xs):
    """A board with a row of square holes along x: two side strips and a cross piece before, between and after
    the holes."""
    z = TOP + RACK_T / 2
    x0, x1 = xs[0] - HOLE / 2 - RACK_END, xs[-1] + HOLE / 2 + RACK_END
    length = x1 - x0
    side_y = HOLE / 2 + RACK_SIDE / 2
    out = [
        Box(f"{prefix}_side_l", pos=((x0 + x1) / 2, y + side_y, z), size=(length, RACK_SIDE, RACK_T), color=RACK_WOOD),
        Box(f"{prefix}_side_r", pos=((x0 + x1) / 2, y - side_y, z), size=(length, RACK_SIDE, RACK_T), color=RACK_WOOD),
    ]
    edges = [x0] + [e for x in xs for e in (x - HOLE / 2, x + HOLE / 2)] + [x1]
    for i in range(0, len(edges), 2):
        lo, hi = edges[i], edges[i + 1]
        out.append(Box(f"{prefix}_cross_{i // 2}", pos=((lo + hi) / 2, y, z), size=(hi - lo, HOLE, RACK_T), color=RACK_WOOD))
    return out


BALL_NAMES = ("ball_a", "ball_b", "ball_c", "ball_d", "ball_e")
BALL_START = {
    "ball_a": (RACK_L_X[0], RACK_L_Y),
    "ball_b": (RACK_L_X[1], RACK_L_Y),
    "ball_c": (RACK_L_X[2], RACK_L_Y),
    "ball_d": (RACK_R_X[0], RACK_R_Y),
    "ball_e": (RACK_R_X[1], RACK_R_Y),
}
BALLS = [
    Sphere(n, pos=(x, y, SEAT_Z + SPAWN_GAP), radius=R, mass=BALL_MASS, color=BALL_RED) for n, (x, y) in BALL_START.items()
]

OBJECTS = [*_tray(), *_rack("rack_left", RACK_L_Y, RACK_L_X), *_rack("rack_right", RACK_R_Y, RACK_R_X), *BALLS]

# -- scoring ---------------------------------------------------------------------------------------------------
PER_BOTTOM = 0.15
TOP_CREDIT = 0.4
FLOOR_Z_TOL = 0.004  # a bottom ball's centre within 4 mm of R above the tray floor
INSIDE_TOL = 0.003  # and no further out than a ball against the rims, plus 3 mm
TOP_XY_TOL = 0.015  # the top ball's centre within 15 mm of the tray's centre (horizontally)
TOP_RISE_TOL = 0.005
TOP_RISE_MIN = TOP_RISE_CORNERS - TOP_RISE_TOL  # 28.3 mm
TOP_RISE_MAX = TOP_RISE_TIGHT + TOP_RISE_TOL  # 58.7 mm
BOTTOM_Z = FLOOR_TOP + R


def _on_tray_floor(s):
    lim = CORNER + INSIDE_TOL
    return (
        abs(s.pos[2] - BOTTOM_Z) <= FLOOR_Z_TOL
        and abs(s.pos[0] - TRAY_X) <= lim
        and abs(s.pos[1] - TRAY_Y) <= lim
    )


def _on_top(s):
    rise = s.pos[2] - BOTTOM_Z
    off = math.hypot(s.pos[0] - TRAY_X, s.pos[1] - TRAY_Y)
    return off <= TOP_XY_TOL and TOP_RISE_MIN <= rise <= TOP_RISE_MAX


def _tally(objects):
    """(balls resting on the tray floor, whether a ball rests on top of a full bottom layer)."""
    states = [objects[n] for n in BALL_NAMES]
    bottom = sum(1 for s in states if s.resting() and _on_tray_floor(s))
    top = bottom == 4 and any(s.resting() and _on_top(s) for s in states)
    return bottom, top


def score(objects):
    """0.15 per ball resting on the tray floor inside the rim (at most four fit: 0.6), plus 0.4 when, with all four
    in place, the fifth rests on top of them: centre within 15 mm of the tray's centre and 28.3-58.7 mm above the
    bottom balls' centres."""
    bottom, top = _tally(objects)
    return round(PER_BOTTOM * min(bottom, 4) + (TOP_CREDIT if top else 0.0), 6)


def success(objects):
    """Four balls resting on the tray floor and the fifth resting on top of them."""
    bottom, top = _tally(objects)
    return bottom == 4 and top


def _to_floor(p):
    lo = np.array([TRAY_X - CORNER, TRAY_Y - CORNER, BOTTOM_Z])
    hi = np.array([TRAY_X + CORNER, TRAY_Y + CORNER, BOTTOM_Z])
    return float(np.linalg.norm(np.maximum(lo - p, 0) + np.maximum(p - hi, 0)))


def _to_top(p):
    return float(np.linalg.norm(p - np.array([TRAY_X, TRAY_Y, BOTTOM_Z + TOP_RISE_CORNERS])))


def distance(objects):
    """Metres from the goal: for the best choice of which ball goes on top, that ball's distance to the top spot
    plus each other ball's distance to the tray floor's ball region (0 when on it)."""
    ps = {n: np.asarray(objects[n].pos, dtype=float) for n in BALL_NAMES}
    return min(_to_top(ps[t]) + sum(_to_floor(ps[n]) for n in BALL_NAMES if n != t) for t in BALL_NAMES)


# -- scripted demo ---------------------------------------------------------------------------------------------
GRIP_ABOVE_CENTRE = 0.006  # TCP this far above a ball's centre: the pads (tips 13 mm below the TCP) span its equator
MAX_GRIP_OFFSET = (GRIPPER_MAX_OPENING - D) / 2  # 9 mm: a held ball can sit at most this far off the TCP
TRACK_ERR = 0.005  # wrist error at rest
TRAVEL_Z = TOP + 0.140  # carrying height: a held ball's bottom 15 mm above the bottom balls' tops
CLEAR_Z = TOP + 0.105  # pad tips 11 mm above the bottom balls' tops
RELEASE_DROP = 0.005  # a bottom ball is let go with its bottom this high over the tray floor
RELEASE_Z = BOTTOM_Z + RELEASE_DROP + GRIP_ABOVE_CENTRE
DIAG = CORNER * math.sqrt(2)  # 68.3 mm: a cornered ball's centre from the tray centre, along the diagonal
# Release the bottom balls on their diagonal this far out from the tray centre, so that with the grip offset and
# the wrist error either way the ball lands between the corner and its two cornered neighbours.
RELEASE_U = DIAG - MAX_GRIP_OFFSET - TRACK_ERR  # 54.3 mm
# The corner push: closed fingers (a block 20 mm along the push, 40 mm across) lowered to the ball's equator on its
# inner side, then moved out along the diagonal to 2 mm into a cornered ball, and held there (CORNER_HOLD_S).
FINGER_BLOCK_HALF = 0.010
PUSH_START_U = -0.012
PUSH_END_U = DIAG - R - FINGER_BLOCK_HALF + 0.002  # 22.3 mm: 2 mm into a cornered ball (2 mm short left it free
# to roll along a rim in simulation); the soft wrist makes this a gentle press
PUSH_Z = BOTTOM_Z + GRIP_ABOVE_CENTRE
CORNER_HOLD_S = 1.5
RELEASE_SETTLE_S = 1.0
TOP_DROP = 0.012  # the top ball is let go 12 mm above its seat on four cornered balls: high enough that a ball
# held 9 mm off the TCP (the most the open pads allow) still clears the two balls it is shifted toward
TOP_RELEASE_Z = BOTTOM_Z + TOP_RISE_CORNERS + TOP_DROP + GRIP_ABOVE_CENTRE
LIFT_AFTER_RELEASE = 0.03
HOLD_S = 3.0  # still hold at the end: the top ball settles, then success() must hold for 1 s

# corner name -> (sign x, sign y); the grip yaw closes the fingers along that corner's diagonal
CORNERS = {"near_left": (-1, 1), "far_left": (1, 1), "near_right": (-1, -1), "far_right": (1, -1)}


def corner_yaw(corner):
    sx, sy = CORNERS[corner]
    return math.radians(45.0 if sx * sy < 0 else -45.0)


def on_diagonal(corner, u, z):
    """The point u metres out from the tray centre along a corner's diagonal, at height z."""
    sx, sy = CORNERS[corner]
    return (TRAY_X + sx * u / math.sqrt(2), TRAY_Y + sy * u / math.sqrt(2), z)


class _Track:
    """A waypoint list with its running end time; yaw stays until changed."""

    def __init__(self):
        self.wps, self.t, self.pos, self.finger, self.yaw = [], 0.0, None, FINGER_OPEN, 0.0

    def go(self, duration, pos=None, finger=None, label="", yaw=None):
        if duration <= 1e-9:
            return
        self.pos = self.pos if pos is None else pos
        self.finger = self.finger if finger is None else finger
        self.yaw = self.yaw if yaw is None else yaw
        self.wps.append(Waypoint(duration, self.pos, self.finger, label, self.yaw))
        self.t += duration

    def wait_until(self, t, label="wait"):
        self.go(t - self.t, label=label)


def _pick(track, ball):
    x, y = BALL_START[ball]
    track.go(1.5, (x, y, TRAVEL_Z), FINGER_OPEN, f"over {ball}", yaw=0.0)
    track.go(1.2, (x, y, SEAT_Z + GRIP_ABOVE_CENTRE), label="descend")
    track.go(0.6, finger=FINGER_CLOSED, label="close")
    track.go(1.2, (x, y, TRAVEL_Z), label="lift out of the rack")


def _place_bottom(track, ball, corner, park):
    """Carry the held ball over its corner, lower it, let go, then push it into the corner. Returns the time the
    arm has left the tray."""
    yaw = corner_yaw(corner)
    track.go(1.8, on_diagonal(corner, RELEASE_U, TRAVEL_Z), label=f"carry {ball} over the {corner} corner", yaw=yaw)
    track.go(2.5, on_diagonal(corner, RELEASE_U, RELEASE_Z), label="lower into the tray")
    # let the lagging wrist come to rest first: a ball let go while still moving 3 cm/s rolled on (GPU round 3)
    track.go(RELEASE_SETTLE_S, label="settle before letting go")
    track.go(0.5, finger=FINGER_OPEN, label="let go")
    track.go(0.6, on_diagonal(corner, RELEASE_U, RELEASE_Z + LIFT_AFTER_RELEASE), label="lift off the ball")
    track.go(0.6, on_diagonal(corner, RELEASE_U, CLEAR_Z), label="clear the ball tops")
    track.go(0.4, finger=FINGER_CLOSED, label="close for the push")
    track.go(1.0, on_diagonal(corner, PUSH_START_U, CLEAR_Z), label="to the inner side")
    track.go(1.2, on_diagonal(corner, PUSH_START_U, PUSH_Z), label="lower to the equator")
    track.go(2.5, on_diagonal(corner, PUSH_END_U, PUSH_Z), label="push into the corner")
    # The lagging wrist leaves the ball rolling into the corner; with no rolling resistance it bounces back off the
    # rims and drifts out (GPU round 3: 40 mm in 7 s). Holding the fingers here stops it against them.
    track.go(CORNER_HOLD_S, label="hold the ball in the corner")
    track.go(0.8, on_diagonal(corner, PUSH_END_U, CLEAR_Z), label="up from the push")
    track.go(1.4, park, FINGER_OPEN, "out of the tray", yaw=0.0)
    return track.t


def demo():
    """Each arm lifts a ball out of its rack and sets it down on the tray floor on the diagonal of a corner, the
    fingers closing along that diagonal, then pushes it into the corner with its closed fingers. The arms take turns
    in the tray: left near-left corner, right far-right, left far-left, right near-right. Then the left arm lowers
    the fifth ball onto the middle of the four and lets go 12 mm above its seat. Grips are straight down."""
    left, right = _Track(), _Track()
    park_l = (RACK_L_X[1], RACK_L_Y, TRAVEL_Z)
    park_r = (RACK_R_X[1], RACK_R_Y, TRAVEL_Z)
    tray_free = 0.0

    _pick(left, "ball_a")
    _pick(right, "ball_d")
    left.wait_until(tray_free)
    tray_free = _place_bottom(left, "ball_a", "near_left", park_l)

    right.wait_until(tray_free)
    _pick(left, "ball_b")
    tray_free = _place_bottom(right, "ball_d", "far_right", park_r)

    _pick(right, "ball_e")
    left.wait_until(tray_free)
    tray_free = _place_bottom(left, "ball_b", "far_left", park_l)

    _pick(left, "ball_c")
    right.wait_until(tray_free)
    tray_free = _place_bottom(right, "ball_e", "near_right", park_r)

    left.wait_until(tray_free)
    centre = (TRAY_X, TRAY_Y)
    left.go(1.8, (*centre, TRAVEL_Z), label="carry ball_c over the tray centre", yaw=0.0)
    left.go(2.0, (*centre, TOP_RELEASE_Z), label="lower onto the four")
    left.go(0.5, finger=FINGER_OPEN, label="let go on top")
    released = left.t
    left.go(0.6, (*centre, TOP_RELEASE_Z + LIFT_AFTER_RELEASE), label="lift off the top ball")
    left.go(1.4, park_l, label="out of the tray")
    end = max(left.t, right.t, released + HOLD_S) + 1.5
    left.wait_until(end, "hold still")
    right.wait_until(end, "hold still")
    return {"left": left.wps, "right": right.wps}
