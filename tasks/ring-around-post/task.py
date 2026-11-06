"""On the station table: a dark round post stands on the midline between the arms, 30 cm out from them. It is a
3.0 cm diameter, 6.0 cm tall wooden dowel fixed to the table. Six loose blue wooden cubes, 3.0 cm on each side,
wait in a straight row across the near part of the table, 15 cm out from the arms: three on the left (toward the
left arm) and three on the right, 8 cm apart centre to centre, with a 24 cm gap on the midline.

Nothing on the table marks a ring, a radius or an angle: no template, no lines, no dots. Distances are measured
from the post's axis (its centre), angles around it.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the post: a hardwood dowel, painted, glued upright to the table (fixed) -------------------------------------------

POST_XY = (0.30, 0.0)
POST_RADIUS = 0.015  # 30 mm diameter
POST_HEIGHT = 0.060
POST = Cylinder(
    "post",
    pos=(POST_XY[0], POST_XY[1], TOP + POST_HEIGHT / 2),
    radius=POST_RADIUS,
    height=POST_HEIGHT,
    color=(0.22, 0.20, 0.20),
)

# -- the blocks: six painted pine cubes, loose ------------------------------------------------------------------------

CUBE = 0.030  # edge (m)
CUBE_MASS = 0.014  # kg: 27 cm^3 of pine at about 500 kg/m^3
BLUE = (0.15, 0.30, 0.85)
SPAWN_GAP = 0.001  # cubes spawn this far above the table and settle onto it
STAGE_X = 0.15  # the staging row, this far out from the arm bases
STAGE_Y = (0.28, 0.20, 0.12, -0.12, -0.20, -0.28)  # block_1 .. block_6, left to right


def _cube(i: int, y: float) -> Box:
    return Box(
        f"block_{i}",
        pos=(STAGE_X, y, TOP + CUBE / 2 + SPAWN_GAP),
        size=(CUBE, CUBE, CUBE),
        mass=CUBE_MASS,
        color=BLUE,
    )


BLOCKS = {b.name: b for b in (_cube(i + 1, y) for i, y in enumerate(STAGE_Y))}
OBJECTS = [POST, *BLOCKS.values()]

# -- the goal, as numbers ---------------------------------------------------------------------------------------------

RING_R = 0.100  # each block's centre 10 cm from the post's axis
RING_TOL = 0.020  # +-20 mm (80-120 mm); see README "Why the tolerances are what they are"
GAP_DEG = 60.0  # six blocks evenly round: 60 deg between neighbours
GAP_TOL_DEG = 10.0  # +-10 deg (50-70 deg)
N_BLOCKS = len(BLOCKS)

FLAT_DEG = 10.0  # a face within 10 deg of flat on the table
ON_TABLE = 0.005  # bottom of the bounding box within 5 mm of the table top (not stacked, held, or on the floor)


# -- geometry helpers -------------------------------------------------------------------------------------------------


def _up_column(state) -> np.ndarray:
    """The world z components of the cube's three body axes (the third row of its rotation matrix)."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)])


def flat_on_table(state) -> bool:
    """Sitting on one of its faces directly on the table: some body axis within 10 deg of vertical (a cube may
    land on any face), and the bottom of its bounding box within 5 mm of the table top."""
    face_down = float(np.abs(_up_column(state)).max()) >= math.cos(math.radians(FLAT_DEG))
    return bool(face_down and abs(float(state.low[2]) - TOP) <= ON_TABLE)


def radius(state) -> float:
    """Distance from the post's axis to the block's centre, in the table plane (m)."""
    return float(np.hypot(state.pos[0] - POST_XY[0], state.pos[1] - POST_XY[1]))


def angle_deg(state) -> float:
    """The block's direction from the post, degrees anticlockwise from +x (straight out from the arms), 0..360."""
    return math.degrees(math.atan2(state.pos[1] - POST_XY[1], state.pos[0] - POST_XY[0])) % 360.0


def in_ring(state) -> bool:
    """Flat on the table, still, and its centre 80-120 mm from the post's axis."""
    return bool(flat_on_table(state) and state.resting() and abs(radius(state) - RING_R) <= RING_TOL)


def ring_gaps(objects) -> dict[str, tuple[float, float]]:
    """For each block in the ring: (gap to the neighbour before it, gap to the neighbour after it), in degrees,
    with the ring's blocks sorted by angle around the post. Blocks outside the ring are left out entirely; with
    fewer than two blocks in the ring each gap is 360."""
    members = sorted((angle_deg(objects[n]), n) for n in BLOCKS if in_ring(objects[n]))
    out = {}
    k = len(members)
    for i, (a, name) in enumerate(members):
        before = (a - members[i - 1][0]) % 360.0 if k > 1 else 360.0
        after = (members[(i + 1) % k][0] - a) % 360.0 if k > 1 else 360.0
        out[name] = (before, after)
    return out


def _gap_ok(gap: float) -> bool:
    return abs(gap - GAP_DEG) <= GAP_TOL_DEG


def placed(objects) -> list[str]:
    """Blocks in the ring whose gaps to both neighbours round the post are 50-70 deg."""
    return [n for n, (b, a) in ring_gaps(objects).items() if _gap_ok(b) and _gap_ok(a)]


# -- hooks ------------------------------------------------------------------------------------------------------------


def score(objects):
    """1/6 per placed block: in the ring (flat on the table, still, 80-120 mm from the post's axis) with both of its
    neighbours round the post 50-70 deg away. 0..1."""
    return len(placed(objects)) / N_BLOCKS


def success(objects):
    """All six blocks placed and still; must hold for 1 s."""
    return bool(len(placed(objects)) == N_BLOCKS)


def distance(objects):
    """Metres: for every block, how far its centre is from the 10 cm circle (0.3 m for a block not flat on the
    table), plus the 10 cm arc length by which each gap between neighbouring blocks on the table (sorted round the
    post) misses 60 deg."""
    on_table = [objects[n] for n in BLOCKS if flat_on_table(objects[n])]
    total = 0.3 * (N_BLOCKS - len(on_table)) + sum(abs(radius(s) - RING_R) for s in on_table)
    angles = sorted(angle_deg(s) for s in on_table)
    if len(angles) > 1:
        gaps = [(angles[(i + 1) % len(angles)] - a) % 360.0 for i, a in enumerate(angles)]
        total += sum(RING_R * math.radians(abs(g - GAP_DEG)) for g in gaps)
    return float(total)


# -- demo -------------------------------------------------------------------------------------------------------------
# Each arm serves its own half of the ring: the left arm places three blocks at 30, 90 and 150 deg round the post
# (on its side, y > 0), the right arm three at 210, 270 and 330 deg. The left arm starts with its far slot and the
# right arm with its near one, and the right arm starts 5 s later (half a block's cycle): run in step, the two
# forearms met over the midline (the offline hull sweep found them 11 mm apart, and overlapping with other orders).
# Staggered, they stay more than 6 cm apart.
#
# Grip offset: closing pads do not centre a block (one finger leads; 6-20 mm off in the first GPU round, roughly the
# approach clearance per pad). So the demo (a) opens only 8 mm past each face on the way down, and (b) turns the grip
# during the carry so that over its slot the fingers close along the ring's radius. Any off-centre hold then moves
# the block toward or away from the post (absorbed by the +-20 mm radius band) and leaves its angle alone.

TRAVEL_Z = TOP + 0.100  # carry height of the TCP: the held block's bottom is 8 cm up, 2 cm over the post
GRIP_Z = TOP + 0.020  # TCP height at the grip: pad tips 7 mm above the table, pads on the block's upper 23 mm
RELEASE_GAP = 0.003  # the block is let go this far above the table
CLEAR_RISE = 0.030  # after letting go, rise this far before any sideways move
APPROACH_CLEARANCE = 0.012  # each pad this far off the block on the way down (8 mm tipped block_6 on GPU round 3)
LOOSEN = 0.006  # each pad backs off this far to let go
FINAL_HOLD_S = 2.0  # still hold at the end (at least 1.5 s): success needs 1 s of rest
PARK = {"left": (0.15, 0.20, TRAVEL_Z), "right": (0.15, -0.20, TRAVEL_Z)}  # clear of the ring for the last frames
RIGHT_DELAY_S = 5.0  # the right arm waits this long at its park spot before it starts

# Which block goes to which slot, in the order each arm places them: (block, slot angle round the post, grip yaw at
# the pick, grip yaw over the slot), all in degrees. At yaw 0 the fingers close along world y, so at yaw psi they
# close along (-sin psi, cos psi). Over each slot that is the ring's radius (yaw = slot angle - 90, or that plus or
# minus 180: a parallel grip is the same turned half a turn). At the pick the yaw is a multiple of 90 deg, because a
# cube in the row sits square to the table and the pads must meet two faces. Of the equivalent choices, these keep
# joint6 inside its +-120 deg limit (the yaw stays within 70 deg of the arm's heading); each block turns 30 deg or
# less in the carry.
DEMO_PLAN = {
    "left": (("block_1", 30.0, -90.0, -60.0), ("block_2", 90.0, 0.0, 0.0), ("block_3", 150.0, -90.0, -120.0)),
    "right": (("block_4", 210.0, 90.0, 120.0), ("block_5", 270.0, 0.0, 0.0), ("block_6", 330.0, 90.0, 60.0)),
}


def slot_xy(angle: float) -> np.ndarray:
    """Where the demo sets a block down: 10 cm from the post's axis, ``angle`` degrees round it."""
    a = math.radians(angle)
    return np.array([POST_XY[0] + RING_R * math.cos(a), POST_XY[1] + RING_R * math.sin(a)])


def closing_direction(yaw: float) -> np.ndarray:
    """The direction (world x, y) the fingers close along at grip yaw ``yaw`` (rad)."""
    return np.array([-math.sin(yaw), math.cos(yaw)])


def _finger(clearance: float) -> float:
    """Finger joint target leaving ``clearance`` between each pad and the 30 mm cube."""
    return -(CUBE / 2 + clearance)


def move_waypoints(name: str, angle: float, pick_yaw_deg: float, place_yaw_deg: float) -> list[Waypoint]:
    """Pick one block from the row and set it down at its slot: hover, descend, close, lift, carry and turn, lower,
    let go 3 mm above the table, rise 3 cm, go up."""
    px, py, _ = BLOCKS[name].pos
    qx, qy = (float(v) for v in slot_xy(angle))
    y0, y1 = math.radians(pick_yaw_deg), math.radians(place_yaw_deg)
    approach, loose = _finger(APPROACH_CLEARANCE), _finger(LOOSEN)
    release_z = GRIP_Z + RELEASE_GAP
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), approach, f"{name}: over it", y0),
        Waypoint(1.2, (px, py, GRIP_Z), approach, f"{name}: descend", y0),
        Waypoint(0.8, (px, py, GRIP_Z), FINGER_CLOSED, f"{name}: close", y0),
        Waypoint(1.0, (px, py, TRAVEL_Z), FINGER_CLOSED, f"{name}: lift", y0),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), FINGER_CLOSED, f"{name}: carry", y1),
        Waypoint(1.2, (qx, qy, release_z), FINGER_CLOSED, f"{name}: lower", y1),
        Waypoint(0.6, (qx, qy, release_z), loose, f"{name}: release", y1),
        Waypoint(0.8, (qx, qy, release_z + CLEAR_RISE), loose, f"{name}: rise clear", y1),
        Waypoint(0.8, (qx, qy, TRAVEL_Z), approach, f"{name}: up", y1),
    ]


def arm_waypoints(side: str) -> list[Waypoint]:
    """One arm's whole plan. The right arm first waits at its park spot for RIGHT_DELAY_S; the left arm holds that
    much longer at the end, so both plans last the same time."""
    out = []
    if side == "right":
        out.append(Waypoint(RIGHT_DELAY_S, PARK[side], _finger(APPROACH_CLEARANCE), "wait", 0.0))
    for step in DEMO_PLAN[side]:
        out += move_waypoints(*step)
    last = out[-1]
    out.append(Waypoint(1.5, PARK[side], last.finger, "park", 0.0))
    hold = FINAL_HOLD_S + (RIGHT_DELAY_S if side == "left" else 0.0)
    out.append(Waypoint(hold, PARK[side], last.finger, "hold still", 0.0))
    return out


def demo():
    """Both arms, 38.2 s each: each picks its three blocks from the row and sets them round the post at 60 deg steps,
    10 cm out, then parks clear of the ring and holds still; the right arm runs 5 s behind the left."""
    return {side: arm_waypoints(side) for side in DEMO_PLAN}
