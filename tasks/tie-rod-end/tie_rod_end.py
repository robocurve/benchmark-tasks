"""On the station table: the left end of a car's steering rack, fixed, 25 cm out and 12 cm left of the midline, with
a steel tie rod hanging from its inner ball joint, 14.1 cm above the table. The tie rod runs 36 cm to the right, toward
the robot's right arm, and rests near its outer end on a wooden block, level. At its outer end is the outer tie rod
end: a black socket housing, 40 mm across and 35 mm tall, with a steel ball stud hanging straight down out of it. The
stud swivels on its ball up to 20 degrees each way about two axes, and the tie rod swings up, down and sideways on the
rack's inner joint. 6.9 cm further out from the robot than the housing stands a fixed cast-iron steering knuckle. Its
steering arm carries a tapered hole, 18.0 mm across at its top and 14.7 mm at its bottom, 19.3 mm deep, its axis
upright and its top 90.6 mm above the table.

The stud goes down into the tapered hole until its taper seats in the hole's taper, square to it, with the tie rod
sloping 3.3 degrees down from the rack. The stud's taper is the same as the hole's; seated, its 13.5 mm threaded end
comes out 22.6 mm under the steering arm. Then the gripper lets go and moves away: it counts only once no part of
either arm is within 10 mm of the tie rod, its outer end or the stud, with the stud still seated.
"""

import functools
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism, on_reset, on_step  # noqa: E402
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
MAX_TILT = 2.0         # degrees between the stud's axis and the hole's
# Home, as on the real parts. MOOG's installation tip (moogparts.com, MG211049, step 1) has the technician test-fit the
# clean stud in the clean tapered hole by hand: it must seat firmly without rocking, and only its thread may come out
# under the knuckle. Home is where the matched stud stops in that test fit, read as the thread's protrusion below the
# steering arm's bottom face (a caliper's depth rod standing on that face, its tip on the thread's end).
# WHY: the sim's matched parts give that protrusion from their geometry: the thread below the taper plus how far the
# stepped taper sinks past flush (SINK). A stud stopped 0.5 mm short of it is one held off its seat (by a burr, grease,
# a cocked entry or a hand), not seated
ARM_BOTTOM_Z = HOLE_TOP_Z - TAPER_L  # the steering arm's bottom face: the boss is as thick as the taper is long
P_HOME = THREAD_L + SINK             # 22.58 mm in this sim
P_TOL = 0.0005                       # either way
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


def protrusion(objects):
    """How far the stud's threaded end stands below the steering arm's bottom face (m), along the hole's axis."""
    _, tip, _ = stud_pose(objects)
    return float(ARM_BOTTOM_Z - tip[2])


def seated(objects):
    """The stud's taper home in the hole's taper and square to it: its thread out under the steering arm by its home
    protrusion, within 0.5 mm."""
    top, _, tilt = stud_pose(objects)
    return (float(np.linalg.norm(top[:2] - HOLE)) <= SEAT_XY and abs(protrusion(objects) - P_HOME) <= P_TOL
            and tilt <= MAX_TILT)


def in_hole(objects):
    """The stud's tip down in the hole, the stud near square: started, not yet home."""
    top, tip, tilt = stud_pose(objects)
    axis = (tip - top) / np.linalg.norm(tip - top)
    at_top = top + axis * (HOLE_TOP_Z - top[2]) / axis[2] if abs(axis[2]) > 1e-6 else top
    return bool(tip[2] < HOLE_TOP_Z - IN_TIP and protrusion(objects) <= P_HOME + P_TOL
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


# -- the hands: let go and clear ----------------------------------------------------------------------------------
# success() and score() get object states only. A hook run before each scored step (mechanism.on_step) reads every
# body of both YAM arms from the simulator, so the scorer can see a hand still holding the tie rod.
ARMS = {"left": "left_robot", "right": "right_robot"}  # the embodiment's scene keys
# every rigid body of a YAM arm: the URDF's links, the bodies of the converted USD
ARM_LINKS = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
# WHY 10 mm: any hand nearer than that may be touching (the arm lags its target 2 to 5 mm when moving, and the
# colliders' contact offsets add 1 to 2 mm), and on a real rig a 10 mm gap shows in a photo. Fully open pads stand
# (94 - 40) / 2 = 27 mm off the housing's sides, so opening alone clears the fingers; a hand that rests on the housing,
# props the rod from below or pushes the stud down does not clear
HAND_CLEAR = 0.010
SAMPLE = 0.002      # spacing of the points laid over the tie rod's and stud's colliders
CLEAR_CAP = 0.05    # clearances are reported up to this; farther is "at least 50 mm"
# this step's arm body poses (world, 4x4) and finger joints, from the hook; the hook's step count; the last clearance
_HANDS = {"poses": None, "finger": None, "step": 0, "memo": None}


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)  # Isaac Lab 3.0 wraps some buffers
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=float)


def read_arms(environment):
    """(poses, finger): the world pose (4x4, the task's frame) of every body of both arms, and each arm's finger
    joints (m, 0 closed, -0.047 open)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins)[0]
    poses, finger = {}, {}
    for arm, key in ARMS.items():
        robot = base.scene[key]
        bodies = _np(robot.data.body_link_pose_w)[0]
        names = list(robot.body_names)
        poses[arm] = {}
        for link in ARM_LINKS:
            p = bodies[names.index(link)]  # a missing body raises: the check never passes quietly
            t = np.eye(4)
            t[:3, :3] = _rot(p[3:7])
            t[:3, 3] = p[:3] - origin
            poses[arm][link] = t
        q = _np(robot.data.joint_pos)[0]
        joints = list(robot.joint_names)
        finger[arm] = [float(q[joints.index(j)]) for j in ("joint7", "joint8")]
    return poses, finger


@on_step
def _watch_arms(environment):
    _HANDS["poses"], _HANDS["finger"] = read_arms(environment)
    _HANDS["step"] += 1


@on_reset
def _forget_arms():
    _HANDS["poses"] = _HANDS["finger"] = _HANDS["memo"] = None


def _surface(shape, h=SAMPLE):
    """Points over a cylinder's side and end faces, at most ``h`` apart (mechanism frame)."""
    r, length = shape.size
    axis = "XYZ".index(shape.axis)
    others = [k for k in range(3) if k != axis]
    n_t = max(8, math.ceil(2 * math.pi * r / h))
    t = np.linspace(0, 2 * math.pi, n_t, endpoint=False)
    rings = [(r, a) for a in np.linspace(-length / 2, length / 2, max(2, math.ceil(length / h) + 1))]
    for rr in np.linspace(0, r, max(2, math.ceil(r / h) + 1))[:-1]:
        rings += [(rr, -length / 2), (rr, length / 2)]
    pts = []
    for rr, a in rings:
        ring = np.zeros((n_t, 3))
        ring[:, axis] = a
        ring[:, others[0]], ring[:, others[1]] = rr * np.cos(t), rr * np.sin(t)
        pts.append(ring)
    return np.vstack(pts) + np.asarray(shape.center, float)


# the tie rod's moving parts: the rod with the outer end's body and housing, and the stud (the gimbals sit inside them)
_PARTS = {"rod": np.vstack([_surface(s) for s in _rod_shapes()]),
          "stud": np.vstack([_surface(s) for s in _stud_shapes()])}


@functools.lru_cache(maxsize=1)
def _arm_hulls():
    """Each arm body's collision hulls in its own frame, as (vertices, unique face planes n . x = d), from the URDF's
    CoACD pieces (yam_arena.analysis.link_hulls)."""
    from yam_arena.analysis import link_hulls

    with np.errstate(all="ignore"):  # WHY: macOS numpy's matmul raises spurious FP flags here; the planes are checked
        hulls = link_hulls()
    out = {}
    for link in ARM_LINKS:
        out[link] = []
        for h in hulls[link]:
            planes = np.unique(np.round(h.planes, 9), axis=0)
            if not np.isfinite(planes).all():
                raise ValueError(f"{link}: a hull plane is not finite")
            out[link].append((h.vertices, planes))
    return out


def part_points(objects):
    """The points over the tie rod's and stud's colliders, in the world."""
    links = objects["tie_rod"].links
    with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags on macOS numpy (seen 2026-11-12); checked below
        world = np.vstack([np.asarray(links[n].pos, float) + m @ _rot(links[n].quat_xyzw).T for n, m in _PARTS.items()])
    if not np.isfinite(world).all():
        raise ValueError("tie rod points are not finite")
    return world


def arm_clearance(points, poses):
    """A lower bound (m) on the gap between any body of one arm and the tie rod's moving parts, capped at CLEAR_CAP;
    -SAMPLE means touching or overlapping. For each point over the parts, the bound is the larger of the distance to a
    hull's box and the hull's face-plane distance max_i(n_i . p - d_i): neither is more than the true distance to that
    convex hull. Every point of the parts' surfaces is within SAMPLE of a laid point, so the true gap is at least the
    smallest point bound minus SAMPLE."""
    with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags on macOS numpy (seen 2026-11-12); checked below
        gap = _gap(points, poses)
    if not math.isfinite(gap):
        raise ValueError("arm clearance is not finite")
    return gap


def _gap(points, poses):
    lo, hi = points.min(axis=0), points.max(axis=0)
    best = CLEAR_CAP + SAMPLE  # the smallest point bound so far
    for link, t in poses.items():
        r, p = t[:3, :3], t[:3, 3]
        for verts, planes in _arm_hulls()[link]:
            v = verts @ r.T + p
            vlo, vhi = v.min(axis=0), v.max(axis=0)
            if (vlo - hi).max() >= best or (lo - vhi).max() >= best:
                continue  # this hull's box is farther than the best bound from every point
            box = np.linalg.norm(np.maximum(np.maximum(vlo - points, points - vhi), 0.0), axis=1)
            near = box < best
            if not near.any():
                continue
            n = planes[:, :3] @ r.T
            d = planes[:, 3] + n @ p
            bound = np.maximum((points[near] @ n.T - d).max(axis=1), box[near])
            best = min(best, float(bound.min()))
    return min(best - SAMPLE, CLEAR_CAP)


def hand_clearance(objects, poses=None):
    """{arm: lower bound on its gap to the tie rod's moving parts (m)} for this step, or None if no arm poses were
    read (the hook has not run since the reset)."""
    if poses is not None:
        return {arm: arm_clearance(part_points(objects), poses[arm]) for arm in ARMS}
    if _HANDS["poses"] is None:
        return None
    # WHY: success(), score() and a logging copy each ask once per step; the answer is the same for the same step
    # (the hook's count, and the very poses object, which the memo keeps alive) and the same tie rod pose
    links = objects["tie_rod"].links
    key = (_HANDS["step"], *(float(v) for n in _PARTS for v in (*links[n].pos, *links[n].quat_xyzw)))
    memo = _HANDS["memo"]
    if memo is None or memo[0] is not _HANDS["poses"] or memo[1] != key:
        gaps = {arm: arm_clearance(part_points(objects), _HANDS["poses"][arm]) for arm in ARMS}
        _HANDS["memo"] = memo = (_HANDS["poses"], key, gaps)
    return dict(memo[2])


def hands_clear(objects):
    """Both arms at least 10 mm from the tie rod, its outer end and the stud. Unknown counts as not clear."""
    gaps = hand_clearance(objects)
    return gaps is not None and all(gaps[arm] >= HAND_CLEAR for arm in ARMS)


def success(objects):
    """The stud seated square in the taper, the tie rod still, and both hands let go and at least 10 mm clear of the
    tie rod and stud. Holds for 1 s."""
    return seated(objects) and _still(objects) and hands_clear(objects)


def distance(objects):
    top, tip, _ = stud_pose(objects)
    return float(np.linalg.norm([top[0] - HOLE[0], top[1] - HOLE[1], tip[2] - (ARM_BOTTOM_Z - P_HOME)]))


TOP_SCORE = 0.8


def score(objects):
    """0.8 seated, still and both hands clear: only success() holding for 1 s pays 1.0 (the trial then ends at 1.0).
    0.6 seated but still moving, or with a hand within 10 mm of the tie rod or stud. 0.3 with the stud's thread 2 mm
    down in the hole and the stud within 2 degrees of square, not seated. A stud beside the hole, on its rim, or
    cocked more than 2 degrees scores nothing."""
    if success(objects):
        return TOP_SCORE
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
