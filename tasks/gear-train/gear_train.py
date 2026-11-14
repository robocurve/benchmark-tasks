"""On the station table, right of the midline: a fixed grey gear plate (150 x 75 mm) whose long side
runs out from the robot, with three upright shafts along its centre line, their tips 26 mm above the table. The shafts
are 28.0, 33.0 and 36.1 cm out from the arm bases (about 50.5 mm, then 30.5 mm apart), all 15 cm right of the
midline. Three loose spur gears, each 25 mm thick, lie flat on the table beside the plate: a large one (62 mm across)
40 cm out and 32 cm right, a medium one (42 mm) 24 cm out and 30 cm right, and a small one (22 mm) 20 cm out and
24 cm right.

The large gear goes on the nearest shaft, the medium on the middle one and the small on the farthest, where the medium
gear's teeth mesh with both others. A gear slides down its shaft only when its teeth line up with the gaps of the gear
it meshes with; out of phase it stops on the other gear's teeth, 7 to 10 mm above its seat. Seated, a gear's bottom is
6 mm above the table. Then the gripper lets go and moves away: the train counts only once no part of either arm is
within 10 mm of any gear, with all three gears still seated.
"""

import functools
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from arm_watch import on_reset, on_step  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Asset  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PLATE_XY = (0.31, -0.15)
GEARS = ("large_gear", "medium_gear", "small_gear")
SHAFT_DX = {"large_gear": -0.0303, "medium_gear": 0.0203, "small_gear": 0.0508}  # along the plate, from its centre
START = {"large_gear": (0.40, -0.32), "medium_gear": (0.24, -0.30), "small_gear": (0.20, -0.24)}
DIAMETER = {"large_gear": 0.062, "medium_gear": 0.042, "small_gear": 0.0218}
MASS = {"large_gear": 0.06, "medium_gear": 0.03, "small_gear": 0.01}

OBJECTS = [Asset("plate", "gear_base", xy=PLATE_XY, extent=0.150)] + [
    Asset(n, n, xy=START[n], extent=DIAMETER[n], mass=MASS[n]) for n in GEARS]

SEAT = TOP + 0.006  # a seated gear's bottom: measured in the sim, 5.9 to 6.6 mm over three gears
SEAT_TOL = 0.001
AXIS_TOL = 0.002  # gear centre to shaft axis; the shafts are 30 mm or more apart
MAX_TILT = 3.0  # degrees


def shaft(name):
    """The (x, y) of the shaft ``name`` belongs on."""
    return np.array([PLATE_XY[0] + SHAFT_DX[name], PLATE_XY[1]])


def tilt_deg(o):
    """Angle between the gear's axis and vertical."""
    x, y = o.quat_xyzw[0], o.quat_xyzw[1]
    return math.degrees(math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def off_axis(objects, name):
    return float(np.linalg.norm(objects[name].pos[:2] - shaft(name)))


def bottom(objects, name):
    """Height of the centre of the gear's bottom face. ``low`` is the lowest corner of the gear's own box turned with
    it, so a tilted gear's ``low`` sits below its bottom face by the box corners' drop; add that drop back."""
    # WHY: with low alone, a 62 mm gear tilted 3 deg about a box diagonal read 2.3 mm low, so it passed the 1 mm seat
    # test with its bottom 3.2 mm proud (hole hunt, 2026-11-12)
    o = objects[name]
    x, y, z, w = (float(v) for v in o.quat_xyzw)
    r20, r21 = 2 * (x * z - w * y), 2 * (y * z + w * x)  # the world-z row of the gear's rotation, local x and y
    return float(o.low[2]) + (abs(r20) + abs(r21)) * DIAMETER[name] / 2


def seated(objects, name):
    """All the way down its own shaft and level."""
    return (off_axis(objects, name) <= AXIS_TOL and abs(bottom(objects, name) - SEAT) <= SEAT_TOL
            and tilt_deg(objects[name]) <= MAX_TILT)


# -- the hands: let go and clear ----------------------------------------------------------------------------------
# success() and score() get object states only. A hook run before each scored step (arm_watch.on_step) reads every
# body of both YAM arms from the simulator, so the scorer can see a hand still holding a gear (robocurve
# benchmark-tasks#914 review: full credit must wait until the gripper has let go and cleared)
ARMS = {"left": "left_robot", "right": "right_robot"}  # the embodiment's scene keys
# every rigid body of a YAM arm: the URDF's links, the bodies of the converted USD
ARM_LINKS = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
# WHY 10 mm: any hand nearer than that may be touching (the arm lags its target 2 to 5 mm when moving, and the loose
# gears' contact offset is 5 mm), and on a real bench a 10 mm gap shows in a photo. A hand resting on a gear's top,
# holding one down or still closed on a rim does not clear. The check is a lower bound and errs toward "not clear":
# random poses near the train (hole hunt 2026-11-12) read up to about 7 mm under the true gap to the cylinders, never
# over it (the least slack 1.9 mm; some CoACD pieces are up to 0.08 mm non-convex, which that slack covers)
HAND_CLEAR = 0.010
SAMPLE = 0.002      # spacing of the points laid over each gear's bounding cylinder
CLEAR_CAP = 0.05    # clearances are reported up to this; farther is "at least 50 mm"
# WHY: each gear is stood in for by its bounding cylinder: the sim mesh's tip radius and overall length (README,
# measured from the USDs at spawn scale: tip 62.05, 42.08, 21.97 mm; overall 25.02, 25.05, 24.97 mm). The cylinder
# contains the gear, so a gap to it is never more than the gap to the gear
TIP_R = {"large_gear": 0.031025, "medium_gear": 0.02104, "small_gear": 0.010985}
OVERALL = 0.02505
# this step's arm body poses (world, 4x4) and finger joints, from the hook; the hook's step count; the last clearance
_HANDS = {"poses": None, "finger": None, "step": 0, "memo": None}


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


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


def _cylinder(r, length, h=SAMPLE):
    """Points over a solid cylinder's side and end faces (axis z, centred on the origin), at most ``h`` apart."""
    n_t = max(8, math.ceil(2 * math.pi * r / h))
    t = np.linspace(0, 2 * math.pi, n_t, endpoint=False)
    rings = [(r, a) for a in np.linspace(-length / 2, length / 2, max(2, math.ceil(length / h) + 1))]
    for rr in np.linspace(0, r, max(2, math.ceil(r / h) + 1))[:-1]:
        rings += [(rr, -length / 2), (rr, length / 2)]
    return np.vstack([np.c_[rr * np.cos(t), rr * np.sin(t), np.full(n_t, a)] for rr, a in rings])


_GEAR_POINTS = {n: _cylinder(TIP_R[n], OVERALL) for n in GEARS}


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


def gear_points(objects):
    """The points over the three gears' bounding cylinders, in the world. Each cylinder is centred on the middle of
    the gear's box ((low + high) / 2: the box is the gear's own box turned with it, so its middle is the gear's) and
    turned with the gear."""
    with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags on macOS numpy; checked below
        world = np.vstack([(np.asarray(objects[n].low, float) + np.asarray(objects[n].high, float)) / 2
                           + _GEAR_POINTS[n] @ _rot(objects[n].quat_xyzw).T for n in GEARS])
    if not np.isfinite(world).all():
        raise ValueError("gear points are not finite")
    return world


def arm_clearance(points, poses):
    """A lower bound (m) on the gap between any body of one arm and the gears, capped at CLEAR_CAP; -SAMPLE means
    touching or overlapping. For each point over the gears, the bound is the larger of the distance to a hull's box
    and the hull's face-plane distance max_i(n_i . p - d_i): neither is more than the true distance to that convex
    hull. Every point of the cylinders' surfaces is within SAMPLE of a laid point, so the true gap is at least the
    smallest point bound minus SAMPLE."""
    with np.errstate(all="ignore"):  # WHY: spurious matmul FP flags on macOS numpy; checked below
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
    """{arm: lower bound on its gap to the gears (m)} for this step, or None if no arm poses were read (the hook has
    not run since the reset)."""
    if poses is not None:
        return {arm: arm_clearance(gear_points(objects), poses[arm]) for arm in ARMS}
    if _HANDS["poses"] is None:
        return None
    # WHY: success(), score() and a logging copy each ask once per step; the answer is the same for the same step
    # (the hook's count, and the very poses object, which the memo keeps alive) and the same gear poses
    key = (_HANDS["step"], *(float(v) for n in GEARS for a in ("low", "high", "quat_xyzw")
                             for v in getattr(objects[n], a)))
    memo = _HANDS["memo"]
    if memo is None or memo[0] is not _HANDS["poses"] or memo[1] != key:
        gaps = {arm: arm_clearance(gear_points(objects), _HANDS["poses"][arm]) for arm in ARMS}
        _HANDS["memo"] = memo = (_HANDS["poses"], key, gaps)
    return dict(memo[2])


def hands_clear(objects):
    """Both arms at least 10 mm from every gear. Unknown (no reading this episode) counts as not clear."""
    gaps = hand_clearance(objects)
    return gaps is not None and all(gaps[arm] >= HAND_CLEAR for arm in ARMS)


def success(objects):
    """All three gears seated on their shafts and still, and both arms let go and at least 10 mm clear of every
    gear. Holds for 1 s."""
    return all(seated(objects, n) and objects[n].resting() for n in GEARS) and hands_clear(objects)


def distance(objects):
    return sum(off_axis(objects, n) for n in GEARS)


# WHY: only the medium gear meshes with the others (the large and small shafts are 81 mm apart, their tips 42 mm
# across), so credit goes to meshes: seating the large and small gears with no phase matching pays only 0.2
PAIRS = (("large_gear", "medium_gear"), ("medium_gear", "small_gear"))


# WHY: the trial scores score() at the last step unless success() has held for 1 s (yam_arena scoring.py), so the
# top state here pays below 1.0; 1.0 comes only from the hold. All three seated with both arms clear at the last step
# pays 0.8 still and 0.7 with any gear moving. All three seated with an arm within 10 mm (a hand still on a gear) pays
# 0.6, still or not: the pair sum (0.9) would pay a held train more than a let-go one. All three above the best
# partial state (0.5: one meshed pair)
ALL_STILL, ALL_MOVING, ALL_HELD = 0.8, 0.7, 0.6


def score(objects):
    """Score at the last step (1.0 only through success() held for 1 s): with all three seated, 0.8 when still and
    both arms are clear, 0.7 when clear with at least one gear moving, 0.6 when an arm is within 10 mm of a gear (or
    no arm reading); otherwise 0.1 for each seated gear and 0.3 for each meshing pair with both gears seated, so 0.2
    with no mesh and 0.5 with one."""
    down = {n for n in GEARS if seated(objects, n)}
    if len(down) == len(GEARS):
        if not hands_clear(objects):
            return ALL_HELD
        return ALL_STILL if all(objects[n].resting() for n in GEARS) else ALL_MOVING
    return round(0.1 * len(down) + 0.3 * sum(a in down and b in down for a, b in PAIRS), 2)


GRIP_Z = 0.016  # TCP height above a lying gear's bottom when the pads close on its rim
# Gear centre minus TCP at wrist yaw 0 after the lift, measured in the sim for this pick order
GRIP = {"large_gear": (0.0004, -0.0011), "medium_gear": (0.0007, -0.0013), "small_gear": (0.0006, -0.0001)}
# WHY: the arm wanders up to 3 mm off its target while holding a gear and the bore catches the tip only within about
# 2.5 mm, so each gear rests on its shaft's tip and the wrist spirals in from 3 mm until the bore drops over it
SPIRAL_R, SPIRAL_TURNS, SPIRAL_STEPS, SPIRAL_S = 0.003, 2, 16, 4.0
PRESS = 0.004  # during the phase search the TCP aims 4 mm above its seated height: light contact on the teeth
# WHY: friction drags the meshed gear along while a gear turns on its teeth, so only about a third of the turn changes
# the phase (measured). The sweeps cover 2.2 tooth pitches of the medium gear (9 deg) and 2.2 of the small one (18 deg)
SWEEP = {"medium_gear": math.radians(10), "small_gear": math.radians(20)}  # half-widths
SWEEP_S = {"medium_gear": 8.0, "small_gear": 10.0}


def demo(delay=0.0):
    """The right arm seats the large gear, then the medium and the small ones. It knows neither where the arm will
    hold the gear to the millimetre nor how the teeth below are turned, so each insertion searches by contact: it
    rests the gear on its shaft's tip and spirals until the bore drops over it, then presses the gear lightly onto the
    teeth it meshes with and turns it through more than a tooth until it drops into mesh, then pushes it home. After
    the last gear it opens fully, waits, and backs off upward so both arms end clear of the gears."""
    hi = TOP + 0.10
    seat_tcp = SEAT + GRIP_Z
    above = seat_tcp + 0.029  # gear bottom 35 mm up: clear of the shaft tips and the seated gears' tops (31 mm)
    tip_tcp = TOP + 0.020 + GRIP_Z  # gear bottom 20 mm up: over the shaft if the bore caught its tip, above the teeth
    w = []

    def pick(name, first=0.0):
        x, y = START[name]
        return [Waypoint(2.0 + first, (x, y, hi), FINGER_OPEN, f"to {name}"),
                Waypoint(2.0, (x, y, hi), FINGER_OPEN, "settle over"),  # WHY: an off-centre grip tilts the gear 3 deg
                Waypoint(1.5, (x, y, TOP + GRIP_Z), FINGER_OPEN, "down"),
                Waypoint(1.0, (x, y, TOP + GRIP_Z), FINGER_OPEN, "settle"),
                Waypoint(0.8, (x, y, TOP + GRIP_Z), FINGER_CLOSED, "grip"),
                Waypoint(1.5, (x, y, hi), FINGER_CLOSED, "lift")]

    def insert(name, via_top=True, dwell=1.5, last=False):
        def at(a, z, dx=0.0, dy=0.0):
            sx, sy = shaft(name)
            gx, gy = GRIP[name]
            c, s = math.cos(a), math.sin(a)
            return (sx + dx - (c * gx - s * gy), sy + dy - (s * gx + c * gy), z)
        half = SWEEP.get(name, 0.0)
        w = [Waypoint(2.0, at(0.0, hi), FINGER_CLOSED, "over shaft")] if via_top else []
        w += [Waypoint(2.0, at(0.0, above), FINGER_CLOSED, "above"),
              Waypoint(dwell, at(0.0, above), FINGER_CLOSED, "settle"),
              Waypoint(1.0, at(-half, above), FINGER_CLOSED, "turn", -half),
              Waypoint(1.5, at(-half, tip_tcp), FINGER_CLOSED, "onto the tip", -half)]
        for k in range(1, SPIRAL_STEPS + 1):
            r, ang = SPIRAL_R * (1 - k / SPIRAL_STEPS), 2 * math.pi * SPIRAL_TURNS * k / SPIRAL_STEPS
            w.append(Waypoint(SPIRAL_S / SPIRAL_STEPS, at(-half, tip_tcp, r * math.cos(ang), r * math.sin(ang)),
                              FINGER_CLOSED, "spiral", -half))
        if half:
            w += [Waypoint(1.5, at(-half, seat_tcp + PRESS), FINGER_CLOSED, "onto the teeth", -half),
                  Waypoint(SWEEP_S[name], at(half, seat_tcp + PRESS), FINGER_CLOSED, "turn into mesh", half)]
        w += [Waypoint(1.5, at(half, seat_tcp + 0.001), FINGER_CLOSED, "push home", half),
              Waypoint(1.0, at(half, seat_tcp + 0.001), FINGER_CLOSED, "settle low", half),
              Waypoint(0.6, at(half, seat_tcp + 0.001), FINGER_OPEN, "release", half)]
        if not last:
            return w + [Waypoint(1.0, at(half, hi), FINGER_OPEN, "up", half)]
        # WHY: success() needs both arms 10 mm clear of every gear, so the last release waits with the fingers fully
        # open before rising (rising on half-open pads could drag the small gear up its shaft), then backs off slowly
        return w + [Waypoint(0.8, at(half, seat_tcp + 0.001), FINGER_OPEN, "let go", half),
                    Waypoint(1.5, at(half, hi), FINGER_OPEN, "back off", half)]

    w += pick("large_gear", first=delay) + insert("large_gear")  # WHY delay: start-delay checks (0, 0.05, 0.10 s)
    w += pick("medium_gear") + insert("medium_gear")
    # WHY: the small gear goes straight in low; a high approach left the arm 1.8 mm short in a probe
    w += pick("small_gear") + insert("small_gear", via_top=False, dwell=2.0, last=True)
    return {"right": w}
