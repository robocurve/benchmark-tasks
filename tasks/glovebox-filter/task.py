"""Replacing a car's cabin air filter behind the glove box.

A passenger-side glove box mock-up stands on the table, opening towards the robots, already unlatched and resting
open on its two stops. The cabin air filter sits in its housing behind it, reachable only once the box swings down
past the stops. Each stop is a lug on the top of a side wall that catches on the dash: a technician squeezes both
side walls in at the same time so both lugs clear, lets the box swing down, slides the old filter out by its pull
tab and slides the new one in. Squeezing one side at a time does nothing, and forcing the box past a stop without
squeezing breaks it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X, Y, Z = np.eye(3)

# -- the dash mock-up (fixed) ----------------------------------------------------------------------------------------
# Part frame: origin on the table under the glove box's hinge, x into the dash (away from the robots), y to the left,
# z up. The box is shown closed here (its door vertical); it starts swung open onto its stops.
DASH_X = 0.32  # the hinge line's world x
GREY, DARK, BLACK = (0.55, 0.56, 0.58), (0.30, 0.31, 0.33), (0.12, 0.12, 0.13)
HZ = 0.030  # the hinge's height above the table
W = 0.175  # the box's half width (side walls' outer faces)
DEPTH, HEIGHT = 0.120, 0.160  # the box: front to back, and its side and back walls' height
DOOR_H = 0.210  # the door is taller than the box: closed, it covers the filter slot behind it
WALL, DOOR_T = 0.0025, 0.010  # wall and door thickness
CHEEK_IN = 0.183  # the dash cheeks' inner faces (y), 8 mm clear of the box's sides
CHEEK_T, DASH_D, DASH_H = 0.012, 0.200, 0.400
CHEEK_H = DASH_H  # the cheeks run up to the dash's top: the filter is reached only through the glove box's opening
LUG = (0.115, 0.090)  # each stop lug's centre on a side wall: from the hinge, back (x) and up (z)
LUG_S = (0.010, 0.006, 0.010)  # lug size: along x, out from the wall (y), along z
STOP_ANG = math.radians(45.0)  # the box rests open this far, its lugs on the dash's stops
PANEL_Z = 0.080  # the side walls' upper halves flex in (modelled as sliding panels): from this height up
SQUEEZE = 0.010  # how far each panel can be pushed in
PANEL_K = 300.0  # N/m: 3 N to push a panel fully in


def _rot_y(a, p):
    """p (x, z from the hinge) turned by a (rad) about the hinge's +y axis."""
    c, s = math.cos(a), math.sin(a)
    return np.array([c * p[0] + s * p[1], -s * p[0] + c * p[1]])


# Each stop: a block on a cheek's inner face, just ahead (towards the robots) of where the lug sits at STOP_ANG.
_lug = _rot_y(-STOP_ANG, LUG)
_lead = _lug[0] - (LUG_S[0] + LUG_S[2]) / 2 * math.cos(math.pi / 4)  # the turned lug's front edge
STOP_X1 = _lead - 0.0005
STOP_Z = HZ + _lug[1]
STOP_Y0 = W + 0.0015  # the stop's inner face: 1.5 mm clear of the side wall, so the lug (6 mm proud) catches 4.5 mm

# The filter housing: a slot in the dash's back wall at this height, opening towards the robots at x = SLOT_X.
SLOT_X, SLOT_Z0, SLOT_H, SLOT_W = 0.190, 0.190, 0.035, 0.206
HOUSING_D = 0.104  # the housing holds one 100 mm deep element
DASH_BOXES = [
    ((DASH_D / 2, CHEEK_IN + CHEEK_T / 2, CHEEK_H / 2), (DASH_D, CHEEK_T, CHEEK_H), GREY),  # cheeks
    ((DASH_D / 2, -CHEEK_IN - CHEEK_T / 2, CHEEK_H / 2), (DASH_D, CHEEK_T, CHEEK_H), GREY),
    ((DASH_D / 2, 0.0, DASH_H + 0.005), (DASH_D, 2 * (CHEEK_IN + CHEEK_T), 0.010), GREY),  # top
    # back wall round the slot (below and above it), then the housing behind
    ((SLOT_X + 0.005, 0.0, (SLOT_Z0 - 0.008) / 2), (0.010, 2 * CHEEK_IN, SLOT_Z0 - 0.008), DARK),
    ((SLOT_X + 0.005, 0.0, (SLOT_Z0 + SLOT_H + 0.008 + DASH_H) / 2), (0.010, 2 * CHEEK_IN, DASH_H - SLOT_Z0 - SLOT_H - 0.008), DARK),
    ((SLOT_X + 0.005, (SLOT_W / 2 + CHEEK_IN) / 2, SLOT_Z0 + SLOT_H / 2), (0.010, CHEEK_IN - SLOT_W / 2, SLOT_H + 0.016), DARK),
    ((SLOT_X + 0.005, -(SLOT_W / 2 + CHEEK_IN) / 2, SLOT_Z0 + SLOT_H / 2), (0.010, CHEEK_IN - SLOT_W / 2, SLOT_H + 0.016), DARK),
    ((SLOT_X + HOUSING_D / 2, 0.0, SLOT_Z0 - 0.004), (HOUSING_D, SLOT_W + 0.016, 0.008), BLACK),  # housing floor
    ((SLOT_X + HOUSING_D / 2, 0.0, SLOT_Z0 + SLOT_H + 0.004), (HOUSING_D, SLOT_W + 0.016, 0.008), BLACK),  # roof
    ((SLOT_X + HOUSING_D / 2, SLOT_W / 2 + 0.004, SLOT_Z0 + SLOT_H / 2), (HOUSING_D, 0.008, SLOT_H), BLACK),
    ((SLOT_X + HOUSING_D / 2, -SLOT_W / 2 - 0.004, SLOT_Z0 + SLOT_H / 2), (HOUSING_D, 0.008, SLOT_H), BLACK),
    ((SLOT_X + HOUSING_D + 0.004, 0.0, SLOT_Z0 + SLOT_H / 2), (0.008, SLOT_W + 0.016, SLOT_H + 0.016), BLACK),
]
for _s in (1, -1):  # the two stops
    DASH_BOXES.append((((0.0 + STOP_X1) / 2, _s * (STOP_Y0 + CHEEK_IN) / 2, STOP_Z),
                       (STOP_X1, CHEEK_IN - STOP_Y0, 0.024), BLACK))

# The box: door, floor, back and the side walls' lower halves turn on the hinge; the upper halves (with the lugs)
# ride on it and slide in when squeezed, sprung back out.
TAN = (0.80, 0.76, 0.68)
BOX_BOXES = [
    ((-DOOR_T / 2, 0.0, HZ + DOOR_H / 2), (DOOR_T, 2 * W, DOOR_H), TAN),  # door
    ((-DOOR_T - 0.004, 0.0, HZ + DOOR_H - 0.040), (0.008, 0.110, 0.016), DARK),  # handle
    ((DEPTH / 2, 0.0, HZ + WALL / 2), (DEPTH, 2 * W, WALL), TAN),  # floor
    ((DEPTH - WALL / 2, 0.0, HZ + HEIGHT / 2), (WALL, 2 * W, HEIGHT), TAN),  # back
    ((DEPTH / 2, W - WALL / 2, HZ + PANEL_Z / 2), (DEPTH, WALL, PANEL_Z), TAN),  # side walls, lower halves
    ((DEPTH / 2, -W + WALL / 2, HZ + PANEL_Z / 2), (DEPTH, WALL, PANEL_Z), TAN),
]


def _panel(name, s):
    """A side wall's upper half with its lug; s = +1 left, -1 right. Pushed in = towards the box's middle."""
    yw = s * (W - WALL / 2)
    boxes = [((DEPTH / 2, yw, HZ + (PANEL_Z + HEIGHT) / 2), (DEPTH, WALL, HEIGHT - PANEL_Z), TAN),
             ((LUG[0], s * (W + LUG_S[1] / 2), HZ + LUG[1]), LUG_S, DARK)]
    lo, hi = (-SQUEEZE, 0.0) if s > 0 else (0.0, SQUEEZE)
    return Joint(name, kind="slide", axis="Y", anchor=(DEPTH / 2, yw, HZ + (PANEL_Z + HEIGHT) / 2), boxes=boxes,
                 lower=lo, upper=hi, stiffness=PANEL_K, damping=4.0, target=0.0, initial=0.0, mass=0.040,
                 parent="box")


OPEN_LIMIT = math.radians(92.0)  # how far the box swings once past its stops (it hangs on its hinge)
BOX = Joint("box", kind="hinge", axis="Y", anchor=(0.0, 0.0, HZ), boxes=BOX_BOXES, lower=-OPEN_LIMIT, upper=0.0,
            damping=0.02, initial=-(STOP_ANG - math.radians(0.4)), mass=0.55)
DASH = part("dash", pos=(DASH_X, 0.0, TOP), fixed=True, boxes=DASH_BOXES,
            joints=[BOX, _panel("panel_left", 1), _panel("panel_right", -1)], friction=0.5,
            self_collide=True)  # (the lugs, on the panels, catch on the stops, on the dash)

# -- the cabin air filter (loose) ------------------------------------------------------------------------------------
# Part frame: origin at the filter's middle; x along its length (towards the back of the housing), z up. A pleated
# paper element in a 25 mm deep frame, 100 mm deep (the half-length elements of a two-piece housing), with a pull tab
# on the end that faces out of the housing.
F_L, F_W, F_T = 0.100, 0.200, 0.025
TAB_L, TAB_W, TAB_T = 0.020, 0.040, 0.0025
WHITE, PLEAT = (0.93, 0.93, 0.90), (0.80, 0.80, 0.74)
DIRTY, DIRTY_PLEAT = (0.52, 0.47, 0.40), (0.38, 0.34, 0.29)
ARROW = (0.15, 0.35, 0.75)


def _filter_boxes(frame, pleat):
    out = [((0.0, 0.0, 0.0), (F_L, F_W, F_T), frame),
           ((-F_L / 2 - TAB_L / 2, 0.0, 0.0), (TAB_L, TAB_W, TAB_T), frame)]
    out += [((0.0, -0.08 + 0.02 * i, F_T / 2 + 0.0005), (F_L - 0.010, 0.006, 0.001), pleat) for i in range(9)]
    out += [((-F_L / 2 + 0.030, F_W / 2 + 0.0005, 0.0), (0.030, 0.001, 0.004), ARROW)]  # airflow mark, left edge
    return out


F_SEAT_X = SLOT_X + F_L / 2 + 0.002  # seated: 2 mm inside the slot's face
F_Z = SLOT_Z0 + F_T / 2 + 0.0005
OLD = part("old_filter", pos=(DASH_X + F_SEAT_X, 0.0, TOP + F_Z), mass=0.040, friction=0.5,
           boxes=_filter_boxes(DIRTY, DIRTY_PLEAT))
# The new filter waits on a stand left of the dash, at the slot's height, its tab towards the robots.
STAND_XY = np.array([0.360, 0.330])  # (clear of the left hand while it carries the box down)
STAND_H = SLOT_Z0 - 0.001
STAND = part("filter_stand", pos=(STAND_XY[0], STAND_XY[1], TOP), fixed=True,
             boxes=[((0.010, 0.0, STAND_H / 2), (0.060, 0.120, STAND_H), GREY)])  # (the corners overhang it)
NEW = part("new_filter", pos=(STAND_XY[0], STAND_XY[1], TOP + STAND_H + F_T / 2 + 0.0005), mass=0.040, friction=0.5,
           boxes=_filter_boxes(WHITE, PLEAT))

OBJECTS = [DASH, OLD, STAND, NEW]

# -- scoring --------------------------------------------------------------------------------------------------------
UP_AGAIN = math.radians(46.0)  # back up past its stops: open no more than this (resting on them, or closed)
FORCE_WIN = (math.radians(46.5), math.radians(54.0))  # in here each lug overlaps its stop by over 1 mm
LUG_CLEAR = 0.0045  # a panel pushed in less than this there means its lug was forced past the stop (breaking it)
OUT_GAP = 0.010  # the old filter is out once all of it is at least this far in front of the slot's face
SEAT_TOL, SIDE_TOL, YAW_TOL_DEG = 0.005, 0.003, 3.0  # the new one seated: within 5 mm of home, 3 mm off, 3 deg
RELEASE = 0.08  # every fingertip at least this far from the new filter's and the box's nearest point
STILL_STEPS, STILL_TOL = 20, 0.001
PASSED = FORCE_WIN[1]  # the box has passed its stops once it is open more than this
MOVED = 0.005  # the old filter has been drawn from its housing once its centre is this far from where it started


def _in_housing(dash, filt):
    """Any of the filter past the slot's face, within the slot's width and height (it has gone into the housing)."""
    for p in ((F_L / 2, 0.0, 0.0), (F_L / 2, F_W / 2, 0.0), (F_L / 2, -F_W / 2, 0.0)):
        q = dash.to_local(filt.to_world(p))
        if q[0] > SLOT_X and abs(float(q[1])) < SLOT_W / 2 + 0.005 and SLOT_Z0 - 0.005 < q[2] < SLOT_Z0 + SLOT_H + 0.005:
            return True
    return False


def _box_angle(dash):
    """How far the box is open (rad, positive)."""
    return -float(dash.joints["box"])


def _squeezed(dash):
    return abs(float(dash.joints["panel_left"])), abs(float(dash.joints["panel_right"]))


def _filter_out(dash, filt):
    """All of the filter (and its tab) at least OUT_GAP in front of the slot's face."""
    hi_x = max(float(dash.to_local(filt.to_world((sx * F_L / 2, sy * F_W / 2, sz * F_T / 2)))[0])
               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1))
    return hi_x <= SLOT_X - OUT_GAP


_BOX_PTS = ([np.array([x, y, HZ + z]) for x in (0.0, DEPTH / 2, DEPTH) for y in (-W, 0.0, W) for z in (0.0, HEIGHT / 2, HEIGHT)]
            + [np.array([0.0, y, HZ + z]) for y in (-W, 0.0, W) for z in (0.21, DOOR_H)])


def _box_world(dash):
    a = float(dash.joints["box"])
    c, s = math.cos(a), math.sin(a)
    out = []
    for p in _BOX_PTS:
        x, z = p[0], p[2] - HZ
        out.append(dash.to_world((c * x + s * z, p[1], HZ - s * x + c * z)))
    return out


def _seated(dash, filt):
    """The new filter home in its housing: within SEAT_TOL of its seat along the slot, SIDE_TOL across and up,
    square to the slot and top side up (its airflow mark then points the right way)."""
    c = dash.to_local(filt.pos)
    rel = dash.rotation().T @ filt.rotation()
    yaw = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    return (abs(float(c[0]) - F_SEAT_X) <= SEAT_TOL and abs(float(c[1])) <= SIDE_TOL
            and abs(float(c[2]) - F_Z) <= SIDE_TOL and abs(yaw) <= YAW_TOL_DEG
            and float(rel[2, 2]) >= math.cos(math.radians(YAW_TOL_DEG)))


def _filter_pts(filt):
    return [filt.to_world((x, y, z)) for x in (-F_L / 2 - TAB_L, -F_L / 2, 0.0, F_L / 2)
            for y in (-F_W / 2, 0.0, F_W / 2) for z in (-F_T / 2, F_T / 2)]


_w = {"bad": "", "hist": [], "still": False, "far": False, "passed": False, "home": None}


@on_reset
def _w_reset(env):
    _w["bad"], _w["hist"], _w["still"], _w["far"] = "", [], False, False
    _w["passed"], _w["home"] = False, None


@on_step
def _watch(env):
    """Latches a stop forced (the box opened past a stop without that wall squeezed in), latches the box having
    passed its stops, latches the old filter drawn from its housing or the new one put into it before that (picking
    the new one up off its stand is fine), and tracks release and stillness."""
    dash = labkit._part_state(env, DASH)
    old = labkit._part_state(env, OLD)
    new = labkit._part_state(env, NEW)
    a = _box_angle(dash)
    if FORCE_WIN[0] < a < FORCE_WIN[1] and min(_squeezed(dash)) < LUG_CLEAR and not _w["bad"]:
        _w["bad"] = "a stop forced"
    if _w["home"] is None:
        _w["home"] = np.asarray(old.pos, float).copy()
    if a > PASSED and not _w["bad"]:
        _w["passed"] = True
    if not _w["passed"] and not _w["bad"]:
        if float(np.linalg.norm(np.asarray(old.pos, float) - _w["home"])) > MOVED:
            _w["bad"] = "the old filter was drawn from its housing before the box passed its stops"
        elif _in_housing(dash, new):
            _w["bad"] = "the new filter went into the housing before the box passed its stops"
    tips = _tips(env)
    pts = _box_world(dash) + _filter_pts(new)
    _w["far"] = min(float(np.linalg.norm(np.asarray(t) - np.asarray(p))) for t in tips for p in pts) >= RELEASE
    snap = np.concatenate([np.asarray(new.pos, float), np.asarray(old.pos, float), [a * 0.2]])  # (angle as 0.2 m/rad)
    h = _w["hist"]
    h.append(snap)
    del h[:-STILL_STEPS]
    _w["still"] = len(h) == STILL_STEPS and max(float(np.max(np.abs(p - h[-1]))) for p in h) < STILL_TOL


def score(objects):
    if _w["bad"] or not _w["passed"]:
        return 0.0
    dash, old, new = objects["dash"], objects["old_filter"], objects["new_filter"]
    if not _filter_out(dash, old):
        return 0.0
    if not _seated(dash, new):
        return 0.25
    if not (_w["far"] and _w["still"]):
        return 0.5
    return 1.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    """How far (m) the new filter is from its seat."""
    dash, new = objects["dash"], objects["new_filter"]
    return float(np.linalg.norm(dash.to_local(new.pos) - np.array([F_SEAT_X, 0.0, F_Z])))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Both hands pinch the two side walls' top edges, squeeze them in together and carry the box down past its stops, then
# let it swing down. The right hand turns to point into the filter slot, pinches the pull tab and draws the filter out.
# Everything it aims at is fixed scenery or seen: the dash and the box's and filter's start.


from yam_arena import ik as _ik  # noqa: E402
from yam_arena.constants import READY as _READY  # noqa: E402


def _tips(env):
    from yam_arena import runtime as _rt

    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    out = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        mid = _rt.pad_tips_world(robot) - o
        g = _rt.link_poses(robot)["gripper"]
        ids = robot.find_joints(["joint7"], preserve_order=True)[0]
        half = abs(float(robot.data.joint_pos.torch[0, ids].cpu().numpy()[0]))
        out += [mid + half * g[:3, 0], mid - half * g[:3, 0]]
    return out


def tilt_deg(R, down=False):
    c = float(R[2, 2]) * (-1.0 if down else 1.0)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))

def _ik_err(pe, re):
    return pe * 1000.0 + math.degrees(re)


FREE_MOVES = ("wrist turned", "hand turned", "up and clear")


def _endpoints(plan):
    """Joint angles for every waypoint's end, chained: warm-started from the previous one, and, where that solve misses,
    the closest of several seeded solves that reach it (so the wrist turns the way the waypoints mean)."""
    rng = np.random.default_rng(7)
    seeds = [np.array(_READY, float)] + [np.array(_READY, float) + rng.normal(0.0, 0.8, 6) for _ in range(10)]
    q_prev, out, switch, forced = plan.q.copy(), [], set(), set()
    for i, w in enumerate(plan.waypoints):
        pos, quat = np.asarray(w.pos, float), _ik.matrix_to_quat_xyzw(_demo.grip_rotation(w.yaw))
        q, pe, re = _ik.solve(pos, quat, q_prev, base_pos=plan.base_pos, iters=300)
        if any(m in w.label for m in FREE_MOVES):
            forced.add(i)  # a free-space move: always a joint-space blend
        if _ik_err(pe, re) > 1.5:
            switch.add(i)
            best = (_ik_err(pe, re), 0.0, q)
            for s in [q_prev + rng.normal(0.0, 0.3, 6) for _ in range(4)] + seeds:
                qs, ps, rs = _ik.solve(pos, quat, s, base_pos=plan.base_pos, iters=200)
                e, d = _ik_err(ps, rs), float(np.max(np.abs(qs - q_prev)))
                if (e < 1.0 and (best[0] >= 1.0 or d < best[1])) or (best[0] >= 1.0 and e < best[0]):
                    best = (e, d, qs)
            q = best[2]
        elif (float(np.max(np.abs(q[3:5] - q_prev[3:5]))) > math.radians(70.0)
              or float(np.max(np.abs(q - q_prev))) > math.radians(100.0)):
            switch.add(i)  # reached, but by turning the wrist over
        out.append(q)
        q_prev = q
    # then backwards: where the arm changes its wrist's branch between two waypoints, move the change earlier if the
    # earlier waypoint can be reached in the later branch (so it happens in free space, before an approach)
    for i in range(len(out) - 1, 0, -1):
        if i not in switch or i in forced:
            continue
        w = plan.waypoints[i - 1]
        pos, quat = np.asarray(w.pos, float), _ik.matrix_to_quat_xyzw(_demo.grip_rotation(w.yaw))
        q, pe, re = _ik.solve(pos, quat, out[i], base_pos=plan.base_pos, iters=300)
        if (_ik_err(pe, re) < 1.5 and float(np.max(np.abs(q - out[i])))
                < float(np.max(np.abs(out[i - 1] - out[i]))) - math.radians(10.0)):
            out[i - 1] = q
            switch.discard(i)
            switch.add(i - 1)
    return out, switch | forced


def _seg(plan, t):
    acc = 0.0
    for i, w in enumerate(plan.waypoints):
        if t < acc + w.duration:
            return i, _ik.smoothstep((t - acc) / w.duration)
        acc += w.duration
    return len(plan.waypoints) - 1, 1.0


def _action7(self, t):
    """[j1..j6, grip] at time t: IK warm-started from the previous step, or, where that misses, from the joint-space
    blend of the segment's end solutions. It also sets the plan clock the trims run on."""
    _demo._plan_clock = float(t)
    if getattr(self, "_qends", None) is None:
        self._qends, self._switch = _endpoints(self)
    pos, rot, finger, label = self.target(t)
    quat = _ik.matrix_to_quat_xyzw(rot)
    i, s = _seg(self, t)
    q0 = self._qends[i - 1] if i > 0 else self._q_start
    seed = q0 + s * (self._qends[i] - q0)
    if i in self._switch:
        # the wrist changes branch on this (free-space) segment: follow the joint-space blend
        self.q = seed
        grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
        return np.r_[seed, grip], label
    q, pe, re = _ik.solve(pos, quat, self.q, base_pos=self.base_pos, iters=80)
    if _ik_err(pe, re) > 2.0:
        q2, p2, r2 = _ik.solve(pos, quat, seed, base_pos=self.base_pos, iters=80)
        if _ik_err(p2, r2) < _ik_err(pe, re) - 1.0:
            q, pe, re = q2, p2, r2
    self.q = q
    self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
    self.worst_ik_deg = max(self.worst_ik_deg, float(np.degrees(re)))
    grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
    return np.r_[q, grip], label


_prev_init = _demo.ArmPlan.__init__
if not getattr(_prev_init, "_fa", False):
    def _init(self, *a, _prev=_prev_init, **k):
        _prev(self, *a, **k)
        self._q_start, self._qends = self.q.copy(), None

    _init._fa = True
    _demo.ArmPlan.__init__ = _init
    _action7._clock = True
    _demo.ArmPlan.action7 = _action7


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_prev_grip = _demo.grip_rotation
if not getattr(_prev_grip, "_rot", False):
    def _grip(yaw=0.0):
        return yaw.rot if hasattr(yaw, "rot") else _prev_grip(yaw)

    _grip._rot = True
    _demo.grip_rotation = _grip

TIP_ABOVE_TCP = 0.013

class _Plan:
    """Two arms' waypoints, built in phases that start together; the clock is the phases' running time."""

    def __init__(self):
        self.L, self.R = [], []
        self.t = 0.0

    def hold(self, plan, dur, label="hold"):
        w = plan[-1]
        plan.append(Waypoint(dur, w.pos, w.finger, label, w.yaw))

    def phase(self, right=(), left=()):
        """Each list: (duration, pos, finger, label, yaw) tuples. The shorter one is padded with a hold."""
        tr, tl = sum(w[0] for w in right), sum(w[0] for w in left)
        dur = max(tr, tl)
        for plan, ws, tot in ((self.R, right, tr), (self.L, left, tl)):
            for w in ws:
                plan.append(Waypoint(*w))
            if dur - tot > 1e-9:
                self.hold(plan, dur - tot)
        t0 = self.t
        self.t += dur
        return t0

    def window(self, t0, t1, key, fn, watch):
        _ct["wins"].append((t0, t1, key, fn, watch))

    def gate(self, t0, t1, key, cond):
        _ct["gates"].append((t0, t1, key, cond))


# trims: each a 3-vector added to the TCPs of _Dyn waypoints with its key; integrated in windows from seen positions
_ct = {"trim": {}, "wins": [], "last": {}, "gates": [], "gate": {}}


@on_reset
def _reset_ct(env):
    _ct["trim"] = {}
    _ct["last"] = {}
    _ct["gate"] = {}


@on_step
def _gtrack(env):
    """Gates: from t0 on, latch ``key`` once ``cond`` sees what it waits for (at t1 at the latest)."""
    t = getattr(_demo, "_plan_clock", 0.0)
    for t0, t1, key, cond in _ct["gates"]:
        if t0 <= t and not _ct["gate"].get(key) and (t >= t1 or cond(env)):
            _ct["gate"][key] = t


class _Gate:
    """A TCP that is ``a`` until the gate ``key`` latches, then ``b``."""

    def __init__(self, a, b, key):
        self.a, self.b, self.key = np.asarray(a, float), np.asarray(b, float), key

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.b if _ct["gate"].get(self.key) else self.a, dtype=dtype or float)


def _seen(env, name):
    return np.asarray(labkit._part_state(env, labkit._SPECS[name]).pos, float)


@on_step
def _ctrack(env):
    t = getattr(_demo, "_plan_clock", 0.0)
    for t0, t1, key, fn, watch in _ct["wins"]:
        if not t0 <= t < t1:
            continue
        p = _seen(env, watch)
        last = _ct["last"].get(key)
        _ct["last"][key] = p
        if last is None or np.max(np.abs(p - last)) > 0.0005:
            continue  # only while what is watched holds still
        e = np.asarray(fn(env), float)
        tr = _ct["trim"].get(key, np.zeros(3))
        _ct["trim"][key] = np.clip(tr + np.clip(0.2 * e, -0.0003, 0.0003), -0.020, 0.020)


class _Dyn:
    """A TCP plus the live trim ``key``."""

    def __init__(self, tcp, key):
        self.tcp, self.key = np.asarray(tcp, float), key

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.tcp + _ct["trim"].get(self.key, np.zeros(3)), dtype=dtype or float)


GRIP_OPEN = -0.012  # 24 mm between the pads
R0 = _prev_grip(0.0)  # straight down, the fingers closing along y
HORIZ = axis_angle_matrix(X, math.radians(90.0)) @ axis_angle_matrix(Y, math.radians(-90.0)) @ R0  # pointing +x, closing along z
HINGE = np.array([DASH_X, 0.0, TOP + HZ])
PINCH_S = 0.035  # the hands pinch each side wall's top edge this far back from the door
PINCH_DEPTH = 0.010  # the pads' tips this far below the edge
SQUEEZE_CMD = 0.016  # each hand moves in this far (the wall stops at 10 mm; the arm gives a little)
SWING_TO = math.radians(62.0)  # the hands carry the squeezed box this far open before letting go
TAB_REACH = 0.004  # the pads' tips stop this far in front of the slot's face
PULL = 0.306  # then draw the filter out this far: all of it is then in front of the dash's face

RIGHT_PARK = np.array([0.22, -0.28, TOP + 0.18])


def _turn(a):
    """The box's rotation when open by a (rad)."""
    return axis_angle_matrix(Y, -a)


def _edge(s, side, a):
    """World point on a side wall's top edge, s back from the door, the box open by a."""
    x, z = _rot_y(-a, (s, HEIGHT))
    return np.array([DASH_X + x, side * (W - WALL / 2), TOP + HZ + z])


_TIP0 = {side: _edge(PINCH_S, side, STOP_ANG) - PINCH_DEPTH * Z for side in (1, -1)}


def _pinch(side, a, inward=0.0):
    """TCP and hand rotation pinching a side wall at the box's opening a, moved in by ``inward``."""
    rel = _turn(a - STOP_ANG)
    tip = HINGE + rel @ (_TIP0[side] - HINGE) - side * inward * Y
    return tip + rel @ (TIP_ABOVE_TCP * Z), Orient(rel @ R0)


def _tab_tcp(dx=0.0):
    """TCP pinching the filter's pull tab (hand pointing into the slot), drawn back dx."""
    tip = np.array([DASH_X + SLOT_X - TAB_REACH - dx, 0.0, TOP + F_Z])
    return tip - TIP_ABOVE_TCP * X


LIFT = 0.035  # a held filter is carried this far above the slot's floor (its far end droops a little)
OLD_DROP_Y = -0.320  # the old filter is let go here, on the table right of the dash
NEW_TIP_IN = 0.006  # the pads' tips stop this far short of where a tab meets its filter
DROOP_UP = 0.012  # a filter held by its tab hangs about 3 deg nose-down: start it into the slot this much higher
RAISE_TO = math.radians(42.0)  # carry the squeezed box up to here (its lugs then clear above the stops), then let go


def _new_tab(dx=0.0, y=None, dz=0.0):
    """TCP pinching the new filter's tab, from where it lies on its stand: moved dx along x, to y, up dz."""
    tip = np.array([STAND_XY[0] - F_L / 2 - NEW_TIP_IN + dx, STAND_XY[1] if y is None else y,
                    TOP + STAND_H + F_T / 2 + 0.0005 + dz])
    return tip - TIP_ABOVE_TCP * X


CORNER_OPEN = -0.020  # 40 mm between the pads, round the filter's 25 mm frame
CORNER_IN, CORNER_Y = 0.020, 0.040  # the pads' tips this far in from the near end, this far from the filter's centre line
#   towards the robots' middle (near the line it is pushed along; further out the far side rolls down onto the slot's lip)
SLIDE_UP = 0.009  # held by its corner the filter droops ~3 deg and rides ~2 mm low: slide it in this much high
BACK_NEAR = 0.215  # backed off, the near end here: the far end (100 mm on) then clears the dash's face at DASH_X
CORNER_STOP = DASH_X + SLOT_X - 0.025  # slide it in by the frame until its near end is 25 mm short of the slot's face
#   (three quarters in: it rests in the slot when let go)
PUSH_HOME = DASH_X + SLOT_X + 0.003  # then push its tab's end with the fingertips until the near end is 3 mm inside
OLD_SET_Y, OLD_SET_DZ = 0.100, -0.022  # the old filter is set down here on the hanging box (to the left, out of the
#   new filter's way and the left hand's), and down
RIGHT_WAIT = np.array([0.200, -0.360, TOP + 0.150])


def _corner(near, c, dz=0.0):
    """TCP pinching a filter's frame at its inner front corner (hand pointing +x, closing along z), the filter's near end
    at x = near and its centre line at y = c, raised dz from the slot's height."""
    tip = np.array([near + CORNER_IN, c - CORNER_Y, TOP + F_Z + dz])
    return tip - TIP_ABOVE_TCP * X


def _push(near):
    """TCP with the closed fingertips on the end of a filter's tab (on its centre line), the filter's near end at
    x = near, at the slot's height."""
    tip = np.array([near - TAB_L, 0.0, TOP + F_Z])
    return tip - TIP_ABOVE_TCP * X


NEW_INSERT = (DASH_X + SLOT_X + 0.002) - (STAND_XY[0] - F_L / 2) + 0.007  # tab to seat, plus 7 mm (the pads let it lag)


def demo():
    from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS

    P = _Plan()
    up = Z
    side_of = {"left": 1, "right": -1}
    a0 = STOP_ANG

    def both(dur, fn, finger, label):
        ws = {}
        for arm, s in side_of.items():
            tcp, o = fn(s)
            ws[arm] = [(dur, tuple(tcp), finger, label, o)]
        P.phase(right=ws["right"], left=ws["left"])

    # 1. both hands squeeze the side walls in together and carry the box down past its stops
    both(2.5, lambda s: (_pinch(s, a0)[0] + 0.05 * up + s * 0.02 * Y, _pinch(s, a0)[1]), GRIP_OPEN,
         "above the side walls' top edges")
    both(1.5, lambda s: _pinch(s, a0), GRIP_OPEN, "down round each top edge")
    both(0.8, lambda s: _pinch(s, a0), FINGER_CLOSED, "pinch both walls")
    both(1.5, lambda s: _pinch(s, a0, SQUEEZE_CMD), FINGER_CLOSED, "squeeze both walls in together")
    both(0.5, lambda s: _pinch(s, a0, SQUEEZE_CMD), FINGER_CLOSED, "hold squeezed")
    for k in (1, 2, 3):
        a = a0 + (SWING_TO - a0) * k / 3
        both(1.0, lambda s, a=a: _pinch(s, a, SQUEEZE_CMD), FINGER_CLOSED, "swing the box down past its stops")
    both(0.6, lambda s: _pinch(s, SWING_TO, SQUEEZE_CMD), GRIP_OPEN, "let go: the box swings down")
    both(1.0, lambda s: (_pinch(s, SWING_TO, SQUEEZE_CMD)[0] + 0.05 * up, _pinch(s, SWING_TO)[1]),
         GRIP_OPEN, "straight up, off the walls' edges")
    both(1.2, lambda s: (_pinch(s, SWING_TO, SQUEEZE_CMD)[0] + 0.07 * up + s * 0.05 * Y, _pinch(s, SWING_TO)[1]),
         GRIP_OPEN, "out to the sides")
    # 2. the right hand draws the old filter out by its tab, all the way out of the dash's opening, and sets it down on
    #    the left half of the hanging glove box; then it waits out of the way
    o_h = Orient(HORIZ)
    lift = LIFT * up
    P.phase(right=[(3.0, tuple(_tab_tcp(0.200) + 0.010 * up), GRIP_OPEN, "hand turned to point into the slot", o_h),
                   (2.5, tuple(_tab_tcp(0.040)), GRIP_OPEN, "in, above the hanging box, in line with the tab", o_h),
                   (1.5, tuple(_tab_tcp()), GRIP_OPEN, "pads round the tab", o_h),
                   (0.8, tuple(_tab_tcp()), FINGER_CLOSED, "pinch the old filter's tab", o_h),
                   *[(1.2, tuple(_tab_tcp(PULL * k / 3)), FINGER_CLOSED, "draw the old filter out, out of the opening", o_h)
                     for k in (1, 2, 3)],
                   (1.0, tuple(_tab_tcp(PULL) + lift), FINGER_CLOSED, "lift it clear", o_h),
                   (1.2, tuple(_tab_tcp(PULL) + lift + OLD_SET_Y * Y), FINGER_CLOSED, "over the hanging box", o_h),
                   (1.2, tuple(_tab_tcp(PULL) + OLD_SET_DZ * up + OLD_SET_Y * Y), FINGER_CLOSED, "down onto the box", o_h),
                   (0.6, tuple(_tab_tcp(PULL) + OLD_SET_DZ * up + OLD_SET_Y * Y), GRIP_OPEN, "let go of it", o_h),
                   (1.2, tuple(_tab_tcp(PULL + 0.015) + 0.03 * up + OLD_SET_Y * Y), GRIP_OPEN, "back and up", o_h),
                   (2.0, tuple(RIGHT_WAIT), GRIP_OPEN, "out of the way", O0)],
            left=[])  # (the left hand waits while the right sets the old filter down on its side of the box)
    # 3. the left hand takes the new filter by its inner front corner, backs it off so all of it is in front of the
    #    dash's face, carries it across in front of the opening, slides it in, then pushes it home with its fingertips
    n0, c0 = STAND_XY[0] - F_L / 2, STAND_XY[1]
    nb = BACK_NEAR  # (the filter's near end, backed off: its far end then clears the dash's face)
    P.phase(left=[(1.2, tuple(_corner(n0, c0) + 0.09 * up - 0.03 * X), CORNER_OPEN,
                   "up and over, well clear of the new filter", o_h),
                  (1.2, tuple(_corner(n0 - 0.045, c0) + 0.04 * up), CORNER_OPEN,
                   "in front of the new filter's inner front corner, above it", o_h),
                  (1.0, tuple(_corner(n0 - 0.045, c0)), CORNER_OPEN, "down to the filter's height, in front of its corner", o_h),
                  (1.5, tuple(_corner(n0, c0)), CORNER_OPEN, "in: pads above and below the frame at the corner", o_h),
                  (0.8, tuple(_corner(n0, c0)), FINGER_CLOSED, "pinch the new filter's frame", o_h),
                  (1.0, tuple(_corner(n0, c0, LIFT)), FINGER_CLOSED, "lift it off the stand", o_h),
                  (1.0, tuple(_corner(nb + 0.025, c0, LIFT)), FINGER_CLOSED, "back it off", o_h),
                  (1.0, tuple(_corner(nb, c0 - 0.030, LIFT)), FINGER_CLOSED, "wholly in front of the dash's face", o_h),
                  (0.8, tuple(_corner(nb, c0 - 0.030, LIFT)), FINGER_CLOSED, "hold still (the arm catches up)", o_h),
                  *[(0.9, tuple(_corner(nb, (c0 - 0.030) * (1 - k / 4), LIFT)), FINGER_CLOSED,
                     "across in front of the opening, in line with the slot", o_h) for k in (1, 2, 3, 4)],
                  (1.0, tuple(_corner(nb, 0.0, DROOP_UP)), FINGER_CLOSED, "down to the slot (held high: it droops)", o_h),
                  *[(1.2, tuple(_corner(nb + (CORNER_STOP - nb) * k / 3, 0.0, SLIDE_UP)), FINGER_CLOSED,
                     "slide the new filter in (held a little high: it droops)", o_h) for k in (1, 2, 3)],
                  (0.6, tuple(_corner(CORNER_STOP, 0.0, SLIDE_UP)), CORNER_OPEN, "let go of the corner", o_h),
                  (1.0, tuple(_corner(CORNER_STOP - 0.040, 0.0, SLIDE_UP)), CORNER_OPEN, "back off", o_h),
                  (1.0, tuple(_push(CORNER_STOP - 0.030)), FINGER_CLOSED, "fingers closed, behind the tab's end", o_h),
                  (1.0, tuple(_push(CORNER_STOP - 0.001)), FINGER_CLOSED, "fingertips on the tab's end", o_h),
                  *[(1.0, tuple(_push(CORNER_STOP + (PUSH_HOME - CORNER_STOP) * k / 3)), FINGER_CLOSED,
                     "push it home on its centre line", o_h) for k in (1, 2, 3)],
                  (1.2, tuple(_push(PUSH_HOME - 0.060) + 0.030 * up), FINGER_CLOSED, "back out", o_h)],
            right=[])  # (the right hand waits, out of the way, until the new filter is home)
    # 4. both hands park, clear of the box and the filters
    p_l, r_l = _ik.tcp_pose(np.array(_READY, float), np.array(LEFT_BASE_POS))
    P.phase(right=[(2.5, tuple(RIGHT_PARK), GRIP_OPEN, "up and clear, to park", O0),
                   (2.0, tuple(RIGHT_PARK), GRIP_OPEN, "hold", O0)],
            left=[(2.5, tuple(p_l), FINGER_OPEN, "up and clear, to park", Orient(r_l)),
                  (2.0, tuple(p_l), FINGER_OPEN, "hold", Orient(r_l))])
    return {"left": P.L, "right": P.R}


O0 = Orient(R0)
