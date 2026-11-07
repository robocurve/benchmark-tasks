"""On the station table: the left end of a car's steering rack, fixed, 25 cm out and 12 cm left of the midline, with
a steel tie rod hanging from its inner ball joint, 14.1 cm above the table. The tie rod runs 36 cm to the right, toward
the robot's right arm, and rests near its outer end on a wooden block, level. At its outer end is the outer tie rod
end: a black socket housing, 40 mm across and 35 mm tall, with a steel ball stud hanging straight down out of it. The
stud swivels on its ball up to 20 degrees each way about two axes, and the tie rod swings up, down and sideways on the
rack's inner joint. 6.9 cm further out from the robot than the housing stands a fixed cast-iron steering knuckle. Its
steering arm carries a tapered hole, 18.0 mm across at its top and 14.7 mm at its bottom, 19.3 mm deep, its axis
upright and its top 90.6 mm above the table.

The stud goes down into the tapered hole until its taper seats in the hole's taper, square to it, with the tie rod
sloping 3.3 degrees down from the rack. The stud's taper is the same as the hole's; its 13.5 mm threaded end, 22 mm
long, comes out under the steering arm.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Asset, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HERE = pathlib.Path(__file__).resolve().parent


def _register():
    """Add the knuckle (a USD file next to this one, written by make_meshes.py) to Arena's asset registry, so
    ``Asset`` can name it. Outside the simulator (tests, checks) there is nothing to register."""
    try:
        from isaaclab_arena.assets.registries import AssetRegistry
    except ImportError:
        return
    reg = AssetRegistry()
    name = "tie_rod_end_knuckle"
    if not reg.is_registered(name, ensure_loaded=False):
        reg.register(type(name, (), {"name": name, "tags": ["object"], "usd_path": str(HERE / f"{name}.usda")}), name)


_register()

# -- sizes (m) -----------------------------------------------------------------------------------------------------
# The stud's taper, from a passenger-car outer tie rod end [READ: NAPA 269-3444: large end .71 in, small end .58 in,
# taper length .76 in, stud thread M14 x 1.5]. The hole in the steering arm matches it (make_meshes.py).
TAPER_LARGE_R = 0.71 * 0.0254 / 2
TAPER_SMALL_R = 0.58 * 0.0254 / 2
TAPER_L = 0.76 * 0.0254
SLOPE = (TAPER_LARGE_R - TAPER_SMALL_R) / TAPER_L  # radius lost per metre down the taper: 0.0855 (4.9 deg a side)
THREAD_R, THREAD_L = 0.00675, 0.022  # the M14 thread as a 13.5 mm rod [EST length]
CHAMFER = 0.001                     # the thread's end: a 1 mm step stands in for its chamfer
BALL_TO_TAPER = 0.030               # ball centre to the taper's large end [EST]
STEPS, STEP_CLEAR = 4, 0.00005      # the taper as 4 steps, each corner 0.05 mm inside the cone
# WHY: with each corner 0.05 mm inside the cone the stud seats 0.05 / SLOPE = 0.58 mm deeper than flush
SINK = STEP_CLEAR / SLOPE
HOUSING_R, HOUSING_LOW, HOUSING_HIGH = 0.020, -0.015, 0.020  # about the ball centre [EST]
OUTER_END_L, OUTER_END_R = 0.183, 0.011  # stud centre to the end of the outer tie rod end [READ: NAPA 269-3444, 7.2]
ROD_R = 0.008                       # the inner tie rod, M16 [READ: NAPA 269-3444 rod end thread M16 x 1.5]
L = 0.360                           # inner ball joint to the stud's ball [EST]
SWIVEL = math.radians(20)           # the stud's swing either way on its ball [EST]

# -- layout --------------------------------------------------------------------------------------------------------
INNER = np.array([0.25, 0.12])      # the rack's inner joint, world (x, y)
SEAT_YAW = math.radians(11.0)       # the tie rod turned this far about the inner joint brings the stud over the hole
BOSS_TOP = 0.0906                   # the hole's top above the table
HOLE_TOP_Z = TOP + BOSS_TOP
# WHY: seated, the stud's ball sits 21 mm below the inner joint (the tie rod slopes 3.3 deg). The outer end moves on a
# sphere about the inner joint, so lowering it also moves it toward or away from the joint; with the slope split
# about level over the stud's last 42 mm down, it moves under 1 mm sideways as the stud goes in
SEAT_DROP = 0.021
JOINT_Z = BOSS_TOP + BALL_TO_TAPER - SINK + SEAT_DROP  # the inner joint above the table, and the ball at the start
HOLE = INNER + math.sqrt(L * L - SEAT_DROP * SEAT_DROP) * np.array([math.sin(SEAT_YAW), -math.cos(SEAT_YAW)])
START_BALL = np.array([INNER[0], INNER[1] - L, TOP + JOINT_Z])
BLOCK_Y = START_BALL[1] + 0.07      # the wooden block under the outer end's body
STEEL, BLACK, RUBBER, ALU = (0.62, 0.63, 0.66), (0.08, 0.08, 0.09), (0.05, 0.05, 0.05), (0.55, 0.57, 0.60)
WOOD = (0.62, 0.45, 0.25)
MATERIALS = {"default": (1.2, 1.0), "steel": (0.5, 0.4)}


def _cyl(c, r, length, axis, color, material="default"):
    return Shape("cylinder", tuple(c), (r, length), color, axis=axis, material=material, contact_offset=0.001)


def _stud_shapes():
    """The stud in the mechanism's frame at rest, hanging straight down from its ball: a neck, the taper as four
    steps, and the threaded end."""
    y, z0 = -L, JOINT_Z
    top = z0 - BALL_TO_TAPER
    # the neck runs from the housing's bottom (z0 + HOUSING_LOW) down to the taper's large end (top)
    shapes = [_cyl((0, y, (z0 + HOUSING_LOW + top) / 2), TAPER_LARGE_R - 0.0005, z0 + HOUSING_LOW - top, "Z", STEEL,
                   "steel")]
    step = TAPER_L / STEPS
    for k in range(STEPS):
        r = TAPER_LARGE_R - SLOPE * step * (k + 1) - STEP_CLEAR
        shapes.append(_cyl((0, y, top - step * (k + 0.5)), r, step, "Z", STEEL, "steel"))
    low = top - TAPER_L
    shapes.append(_cyl((0, y, low - (THREAD_L - CHAMFER) / 2), THREAD_R, THREAD_L - CHAMFER, "Z", STEEL, "steel"))
    shapes.append(_cyl((0, y, low - THREAD_L + CHAMFER / 2), THREAD_R - CHAMFER, CHAMFER, "Z", STEEL, "steel"))
    return tuple(shapes)


def _rod_shapes():
    """The tie rod with the outer tie rod end's body and its socket housing, along -y from the inner joint."""
    outer = (-L + HOUSING_R - 0.002, -L + OUTER_END_L)
    return (
        _cyl((0, -(0.005 + L - OUTER_END_L) / 2, JOINT_Z), ROD_R, L - OUTER_END_L - 0.005, "Y", STEEL, "steel"),
        _cyl((0, sum(outer) / 2, JOINT_Z), OUTER_END_R, outer[1] - outer[0], "Y", BLACK),
        _cyl((0, -L, JOINT_Z + (HOUSING_LOW + HOUSING_HIGH) / 2), HOUSING_R, HOUSING_HIGH - HOUSING_LOW, "Z", BLACK),
    )


def _rack_shapes():
    return (
        _cyl((0, 0.10, JOINT_Z), 0.022, 0.14, "Y", ALU),
        _cyl((0, 0.015, JOINT_Z), 0.016, 0.03, "Y", RUBBER),
        Shape("box", (0, 0.10, (JOINT_Z - 0.015) / 2), (0.05, 0.10, JOINT_Z - 0.015), ALU),
    )


def _gimbal(at):
    """A small hidden link between the two turns of a ball joint."""
    return (_cyl(at, 0.005, 0.010, "Z", BLACK),)


TIE_ROD = mechanism(
    "tie_rod", (float(INNER[0]), float(INNER[1]), TOP),
    base=Link("rack", 1.0, _rack_shapes()),
    links=(Link("inner_gimbal", 0.01, _gimbal((0, 0, JOINT_Z))),
           Link("rod", 0.12, _rod_shapes()),
           Link("outer_gimbal", 0.01, _gimbal((0, -L, JOINT_Z))),
           Link("stud", 0.03, _stud_shapes())),
    joints=(
        # the rack's inner ball joint, as a turn about the vertical and a turn about x [EST limits]
        Joint("inner_yaw", "revolute", "inner_gimbal", (0, 0, JOINT_Z), "Z", -0.6, 0.6, damping=0.02),
        Joint("inner_pitch", "revolute", "rod", (0, 0, JOINT_Z), "X", -0.45, 0.45, damping=0.02,
              parent="inner_gimbal"),
        # the outer ball joint: the stud swings about x and y on its ball
        Joint("swivel_x", "revolute", "outer_gimbal", (0, -L, JOINT_Z), "X", -SWIVEL, SWIVEL, damping=0.005,
              parent="rod"),
        Joint("swivel_y", "revolute", "stud", (0, -L, JOINT_Z), "Y", -SWIVEL, SWIVEL, damping=0.005,
              parent="outer_gimbal"),
    ),
    materials=MATERIALS,
    # WHY: the stud's top sits inside the housing and the gimbals inside the joints, as the real ball sits in its
    # socket; the swivel limits stand in for the stud's neck meeting the socket's mouth
    self_collide=False,
)

OBJECTS = [
    TIE_ROD,
    Asset("knuckle", "tie_rod_end_knuckle", xy=tuple(float(v) for v in HOLE + np.array([0.044, 0.0])), extent=0.132),
    Box("block", size=(0.04, 0.03, JOINT_Z - OUTER_END_R - 0.0005), pos=(float(START_BALL[0]), float(BLOCK_Y),
        TOP + (JOINT_Z - OUTER_END_R - 0.0005) / 2), color=WOOD),
]


# -- scoring -------------------------------------------------------------------------------------------------------
def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


MECH_ORIGIN = np.array([INNER[0], INNER[1], TOP])
TAPER_TOP_M = np.array([0.0, -L, JOINT_Z - BALL_TO_TAPER])  # on the stud, in the mechanism's frame at rest
TIP_M = np.array([0.0, -L, JOINT_Z - BALL_TO_TAPER - TAPER_L - THREAD_L])

SEAT_XY = 0.001        # the taper's large end from the hole's axis
SEAT_DEPTH = SINK      # the taper's large end below the hole's top when seated
SEAT_Z = 0.0005        # either way: 0.5 mm proud of home is a stud wedged high, not seated
MAX_TILT = 2.0         # degrees between the stud's axis and the hole's
# WHY: the thread is inside the mouth only with the stud's axis within 18.03/2 - 13.5/2 = 2.27 mm of the hole's at the
# hole's top. A tip resting on the rim dips at most 1 mm (the chamfer step) plus under 0.7 mm (a lean of up to 6
# degrees), so 2 mm down means the thread is in the hole, not on its edge
IN_XY = TAPER_LARGE_R - THREAD_R + 0.00015  # 2.42 mm
IN_TIP = 0.002         # the tip below the hole's top
# WHY: a stud wedged in the self-locking taper can sit cocked up to about 8.5 degrees; under 2 degrees it is at most
# about 3 mm from home and square, so partial credit never pays a cocked jam
IN_TILT = 2.0
STILL_V, STILL_W = 0.01, 0.5  # m/s at the stud's taper and the tie rod's outer end; rad/s (as ObjectState.resting)


def stud_pose(objects):
    """(taper's large end, threaded tip, both world (m); tilt of the stud's axis from the hole's, deg)."""
    s = objects["tie_rod"].links["stud"]
    r = _rot(s.quat_xyzw)
    origin = np.asarray(s.pos, dtype=float)
    top = origin + r @ TAPER_TOP_M
    tip = origin + r @ TIP_M
    tilt = math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))  # the stud's axis is the frame's z
    return top, tip, tilt


def seated(objects):
    """The stud's taper home in the hole's taper and square to it."""
    top, _, tilt = stud_pose(objects)
    depth = HOLE_TOP_Z - top[2]
    return (float(np.linalg.norm(top[:2] - HOLE)) <= SEAT_XY and abs(depth - SEAT_DEPTH) <= SEAT_Z
            and tilt <= MAX_TILT)


def in_hole(objects):
    """The stud's tip down in the hole, the stud near square: started, not yet home."""
    top, tip, tilt = stud_pose(objects)
    axis = (tip - top) / np.linalg.norm(tip - top)
    at_top = top + axis * (HOLE_TOP_Z - top[2]) / axis[2] if abs(axis[2]) > 1e-6 else top
    depth = HOLE_TOP_Z - top[2]
    return bool(tip[2] < HOLE_TOP_Z - IN_TIP and depth <= SEAT_DEPTH + SEAT_Z
                and np.linalg.norm(at_top[:2] - HOLE) < IN_XY and tilt <= IN_TILT)


def _point_still(s, point_m):
    """A link still at a point of it (given in the mechanism's frame at rest). A link's reported velocity is that of
    its frame's origin, the mechanism's origin carried by the link, far from the part itself."""
    arm = _rot(s.quat_xyzw) @ point_m
    w = np.asarray(s.ang_vel, dtype=float)
    v = np.asarray(s.lin_vel, dtype=float) + np.cross(w, arm)
    return float(np.linalg.norm(v)) < STILL_V and float(np.linalg.norm(w)) < STILL_W


BALL_M = np.array([0.0, -L, JOINT_Z])  # the stud's ball, in the mechanism's frame at rest


def _still(objects):
    links = objects["tie_rod"].links
    return _point_still(links["stud"], TAPER_TOP_M) and _point_still(links["rod"], BALL_M)


def success(objects):
    """The stud seated square in the taper and the tie rod still. Holds for 1 s."""
    return seated(objects) and _still(objects)


def distance(objects):
    top, _, _ = stud_pose(objects)
    return float(np.linalg.norm([top[0] - HOLE[0], top[1] - HOLE[1], HOLE_TOP_Z - SEAT_DEPTH - top[2]]))


def score(objects):
    """1.0 only when seated and still (the trial ends at 1.0 once that has held 1 s). 0.6 seated but still moving.
    0.3 with the stud's thread 2 mm down in the hole and the stud within 2 degrees of square, not seated. A stud
    beside the hole, on its rim, or cocked more than 2 degrees scores nothing."""
    if success(objects):
        return 1.0
    if seated(objects):
        return 0.6
    if in_hole(objects):
        return 0.3
    return 0.0


# -- demo ----------------------------------------------------------------------------------------------------------
GRIP_YAW = math.pi / 2          # the fingers close across the housing, square to the tie rod
TCP_OVER_BALL = 0.005           # pad tips 8 mm under the ball centre, on the housing's side
# WHY: each sim finger is capped at 50 N; aimed 30 mm apart on the 40 mm housing each pad presses 5 mm, under the cap
GRIP_FINGER = -0.015
LIFT = 0.030                    # the ball this far above the inner joint: the threaded tip 7.8 mm over the hole's top
# WHY: the stud is let go short of home and drops onto its seat, so it is never seated while the fingers hold it and
# the 1 s still hold starts only once they are off. Held, it sits about 1.1 mm under the aim (logged sag at the hover),
# so aimed 3.5 mm above home it stops about 2.4 mm proud, outside the 0.5 mm seat window
RELEASE = 0.0035
AIM = (0.0, 0.0)                # added to every aim at the hole (m, world x and y); margin runs shift it


def ball_at(yaw, dz):
    """The stud's ball centre (world) with the tie rod turned ``yaw`` about the inner joint and its outer end ``dz``
    above the joint: the outer end moves on a sphere around the inner joint."""
    h = math.sqrt(L * L - dz * dz)
    return np.array([INNER[0] + h * math.sin(yaw), INNER[1] - h * math.cos(yaw), TOP + JOINT_Z + dz])


def demo(delay=0.0):
    """The right arm grips the housing from above, lifts the outer end off its block, swings it out over the hole on
    its arc around the inner joint, waits for the stud to stop swinging, lowers it into the taper to just short of
    home, lets go so it drops onto its seat, and backs off."""
    def tcp(yaw, rise, dz=0.0):
        b = ball_at(yaw, rise)
        if yaw == SEAT_YAW:
            b[:2] += AIM
        return (float(b[0]), float(b[1]), float(b[2] + TCP_OVER_BALL + dz))

    w = [Waypoint(2.0 + delay, tcp(0, 0, 0.06), FINGER_OPEN, "over the housing", GRIP_YAW),
         Waypoint(1.0, tcp(0, 0, 0.06), FINGER_OPEN, "settle", GRIP_YAW),
         Waypoint(1.5, tcp(0, 0), FINGER_OPEN, "down", GRIP_YAW),
         Waypoint(1.0, tcp(0, 0), FINGER_OPEN, "settle", GRIP_YAW),
         Waypoint(0.8, tcp(0, 0), GRIP_FINGER, "grip", GRIP_YAW),
         Waypoint(1.5, tcp(0, LIFT), GRIP_FINGER, "lift", GRIP_YAW)]
    for k in range(1, 5):
        a = SEAT_YAW * k / 4
        w.append(Waypoint(0.8, tcp(a, LIFT), GRIP_FINGER, "swing out", GRIP_YAW + a))
    w += [Waypoint(2.0, tcp(SEAT_YAW, LIFT), GRIP_FINGER, "settle", GRIP_YAW + SEAT_YAW)]
    above = BOSS_TOP + (BALL_TO_TAPER + TAPER_L + THREAD_L) + 0.004 - JOINT_Z  # the tip 4 mm over the hole's top
    low = tcp(SEAT_YAW, -SEAT_DROP + RELEASE)
    w += [Waypoint(1.5, tcp(SEAT_YAW, above), GRIP_FINGER, "over the hole", GRIP_YAW + SEAT_YAW),
          Waypoint(1.5, tcp(SEAT_YAW, above), GRIP_FINGER, "settle", GRIP_YAW + SEAT_YAW),
          Waypoint(3.0, low, GRIP_FINGER, "into the taper", GRIP_YAW + SEAT_YAW),
          Waypoint(1.0, low, GRIP_FINGER, "settle", GRIP_YAW + SEAT_YAW),
          Waypoint(0.3, low, FINGER_OPEN, "let go", GRIP_YAW + SEAT_YAW),
          Waypoint(0.8, (low[0], low[1], low[2] + 0.06), FINGER_OPEN, "back off", GRIP_YAW + SEAT_YAW),
          Waypoint(2.0, (low[0], low[1], low[2] + 0.06), FINGER_OPEN, "clear", GRIP_YAW + SEAT_YAW)]
    return {"right": w}
