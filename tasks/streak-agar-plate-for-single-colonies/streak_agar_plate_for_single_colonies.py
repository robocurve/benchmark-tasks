"""A 90 mm blood agar plate (lid on) with a specimen already spread on its far quarter,
a sterile 10 uL plastic inoculating loop standing in a slotted rack, and a discard tray.
The loop's thin neck flexes when its tip rests on the agar; where the tip rides on the
agar it leaves a pale streak line. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl
from yam_joints import Mechanism, Link, Hinge
from yam_mesh import MeshPart, revolve
from yam_arena import demo as _motion
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint, grip_rotation as _grip_rotation0
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ---------------------------------------------------------------- geometry
# World: x forward, y left, z up (m). Every part has zero yaw.
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 2600
DEMO_PACE = 1.0

# 90 x 15 mm polystyrene Petri dish (EN ISO 24998 nominal size) with 5 % sheep blood agar.
DISH_XY = np.array([.165, 0.])
BASE_R, WALL_T, FLOOR_T, BASE_H = .0440, .0008, .0010, .0140
AGAR_R, AGAR_T = .0430, .0040
AGAR_TOP = FLOOR_T + AGAR_T            # 5.0 mm above the bench, dish frame
LID_R, LID_T, SKIRT_H = .0455, .0010, .0080
RIM_CHAMFER = .0004                    # moulded edge rounding, modelled as a 0.4 mm chamfer
LID_SEAT_Z = BASE_H - SKIRT_H          # lid origin (skirt rim) height when seated: 6.0 mm
PS_DENSITY, AGAR_DENSITY = 1050., 1030.
_floor_v = math.pi*BASE_R**2*FLOOR_T
_wall_v = math.pi*(BASE_R**2-(BASE_R-WALL_T)**2)*(BASE_H-FLOOR_T)
_agar_v = math.pi*AGAR_R**2*AGAR_T
DISH_MASS = round(PS_DENSITY*(_floor_v+_wall_v) + AGAR_DENSITY*_agar_v, 4)       # 0.0334 kg
LID_MASS = round(PS_DENSITY*(math.pi*LID_R**2*LID_T +
                             math.pi*(LID_R**2-(LID_R-WALL_T)**2)*SKIRT_H), 4)  # 0.0087 kg
AGAR_COLOR = (.55, .07, .07)
PS_CLEAR = (.86, .89, .92)

DISH = MeshPart(
    "dish", (*DISH_XY, TOP),
    shapes=[cyl((0, 0, FLOOR_T/2), BASE_R, FLOOR_T, color=PS_CLEAR),
            cyl((0, 0, FLOOR_T+AGAR_T/2), AGAR_R, AGAR_T, color=AGAR_COLOR)],
    mass=DISH_MASS, color=PS_CLEAR, friction=(.4, .3), contact_offset=.0005,
    meshes=(revolve([(BASE_R-WALL_T, FLOOR_T), (BASE_R, FLOOR_T), (BASE_R, BASE_H-RIM_CHAMFER),
                     (BASE_R-RIM_CHAMFER, BASE_H), (BASE_R-WALL_T, BASE_H)], samples=128, name="wall", color=PS_CLEAR,
                    proxy=tuple(box((sx*(BASE_R-WALL_T/2), sy*(BASE_R-WALL_T/2), (FLOOR_T+BASE_H)/2),
                                    (WALL_T if sx else .03, WALL_T if sy else .03, BASE_H-FLOOR_T))
                                for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1)))),))

LID = MeshPart(
    "lid", (*DISH_XY, TOP+LID_SEAT_Z),
    shapes=[cyl((0, 0, SKIRT_H+LID_T/2), LID_R, LID_T, color=PS_CLEAR)],
    mass=LID_MASS, color=PS_CLEAR, friction=(1., .9), contact_offset=.0005,
    meshes=(revolve([(LID_R-RIM_CHAMFER, 0.), (LID_R, 0.), (LID_R, SKIRT_H), (LID_R-WALL_T, SKIRT_H),
                     (LID_R-WALL_T, RIM_CHAMFER+.0002)],
                    samples=128, name="skirt", color=PS_CLEAR,
                    proxy=tuple(box((sx*(LID_R-WALL_T/2), sy*(LID_R-WALL_T/2), SKIRT_H/2),
                                    (WALL_T if sx else .03, WALL_T if sy else .03, SKIRT_H))
                                for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1))) +
                    # Offline-only grip proxy: the +y edge region the left hand pinches across a
                    # chord (y 16-36 mm; 85 mm wide at its inner edge), at lid-top height.
                    (box((0, .026, SKIRT_H+LID_T/2), (.0852, .020, LID_T)),)),),
    grip=5)

# Sterile 10 uL polystyrene loop, 200 mm long. Part frame: z along the loop, origin at
# the shoulder where the 5 mm handle steps down to the 1.6 mm neck; tip at z = -100 mm.
# The upper 56 mm of the handle is the flat finger grip, 7.0 x 2.6 mm (thin along x).
# The last 60 mm of neck is one link on a sprung hinge: the lumped bending of the thin
# neck. Its ring is a 5 mm disc 0.8 mm thick whose plane contains the loop axis (x-z).
HANDLE_R, HANDLE_L = .0025, .100
GRIP_Z0, GRIP_W, GRIP_T = .044, .0070, .0026
NECK_R1, NECK_R2 = .0008, .0006
HINGE_Z, TIP_Z = -.040, -.100
RING_R, RING_T = .0025, .0008
RING_C = np.array([0., 0., TIP_Z+RING_R])
NECK_L = HINGE_Z-TIP_Z                      # 60 mm hinge-to-tip
NECK_K = .015                               # N m/rad, = 3EI/L of a 1.2 mm PS neck
NECK_PRELOAD_DEG = -2.
FLEX_MAX_DEG = 20.
_handle_v = (math.pi*HANDLE_R**2*GRIP_Z0 + GRIP_W*GRIP_T*(HANDLE_L-GRIP_Z0) +
             math.pi*NECK_R1**2*(-HINGE_Z))
_neck_v = math.pi*NECK_R2**2*(NECK_L-2*RING_R) + math.pi*RING_R**2*RING_T
LOOP_BASE_MASS = round(PS_DENSITY*_handle_v, 5)     # 2.01 g
LOOP_NECK_MASS = round(PS_DENSITY*_neck_v, 5)       # 0.09 g

RACK_XY = np.array([.130, -.200])
SHOULDER_Z = .108                                    # rack ledge top above the bench
LOOP = Mechanism(
    "loop", (*RACK_XY, TOP+SHOULDER_Z+.0002),
    shapes=[cyl((0, 0, GRIP_Z0/2), HANDLE_R, GRIP_Z0, color=(.10, .35, .85)),
            box((0, 0, (GRIP_Z0+HANDLE_L)/2), (GRIP_T, GRIP_W, HANDLE_L-GRIP_Z0), color=(.10, .35, .85)),
            cyl((0, 0, HINGE_Z/2), NECK_R1, -HINGE_Z, color=(.10, .35, .85))],
    links=[Link("neck", [cyl((0, 0, (HINGE_Z+TIP_Z+2*RING_R)/2), NECK_R2, NECK_L-2*RING_R),
                         cyl(tuple(RING_C), RING_R, RING_T, axis="y")],
                Hinge("x", (0, 0, HINGE_Z), 0., FLEX_MAX_DEG, damping=1e-3, spring=NECK_K,
                      rest=NECK_PRELOAD_DEG, armature=1e-5),
                LOOP_NECK_MASS, color=(.10, .35, .85), friction=(.10, .08))],
    mass=LOOP_BASE_MASS, color=(.10, .35, .85), friction=(1., .8), contact_offset=.0002, grip=1)

# Two-tier slotted acrylic loop rack, slots open toward +x (away from the arms). The lower
# tier's 2.4 mm slot takes the neck and carries the shoulder; the upper tier's 5.6 mm slot
# steadies the handle. Both closed ends sit 0.3 mm behind the loop, so it can only lean
# forward along the slot (x), across the closing pads, which straighten it.
RACK_T = .004
LOWER_SLOT, UPPER_SLOT, UPPER_Z = .0030, .0064, .040
_tine = lambda slot, z, back: [box((.008, s*(slot/2+.004), z-RACK_T/2), (.030, .008, RACK_T)) for s in (-1, 1)] +     [box((back-.008, 0, z-RACK_T/2), (.016, slot+.016, RACK_T))]
RACK = Part("rack", (*RACK_XY, TOP),
            _tine(LOWER_SLOT, SHOULDER_Z, -NECK_R1-.0003) +
            _tine(UPPER_SLOT, SHOULDER_Z+UPPER_Z, -HANDLE_R-.0003) +
            [box((-.020, 0, (SHOULDER_Z+UPPER_Z)/2), (.010, .030, SHOULDER_Z+UPPER_Z))],
            color=(.80, .85, .88), friction=(.4, .3), contact_offset=.0002)

# Polypropylene discard tray, 230 x 100 x 30 mm outside, 2.5 mm walls; floor top 2.5 mm.
TRAY_XY = np.array([.275, -.150])
TRAY_IN = np.array([.095, .225])
TRAY_H, TRAY_W = .030, .0025
TRAY = Part("tray", (*TRAY_XY, TOP),
            [box((0, 0, TRAY_W/2), (TRAY_IN[0]+2*TRAY_W, TRAY_IN[1]+2*TRAY_W, TRAY_W))] +
            [box((s*(TRAY_IN[0]+TRAY_W)/2, 0, TRAY_H/2), (TRAY_W, TRAY_IN[1]+2*TRAY_W, TRAY_H)) for s in (-1, 1)] +
            [box((0, s*(TRAY_IN[1]+TRAY_W)/2, TRAY_H/2), (TRAY_IN[0]+2*TRAY_W, TRAY_W, TRAY_H)) for s in (-1, 1)],
            color=(.85, .20, .15), friction=(.5, .4), contact_offset=.0005)

OBJECTS = [DISH, LID, LOOP, RACK, TRAY]

# ---------------------------------------------------------------- specimen and quadrants
# Dish frame: the specimen (primary inoculum) is on the far (+x) quarter.
PRIMARY_DIR_DEG = 0.
PATCH_R = (.016, .038)
PATCH_HALF_DEG = 30.
TRACE_R_MAX = .041          # tip at least 2 mm inside the wall
CONTACT_Z_TOL = .002        # tip within 2 mm of the agar surface
FLEX_ON_DEG = 1.0           # neck bent >= 1 deg (tip pushed back >= 1.0 mm): resting on the agar
FLEX_GOUGE_DEG = 18.0       # neck bent >= 18 deg on the agar: tip load >= 0.15 N, gouge
BOUNDARY_CLEAR = .003       # a point is clearly in a quadrant 3 mm past both its boundary lines
SECTOR_MIN_LENGTH = .050    # streak line inside each new quadrant
SECTOR_MIN_LINES = 3        # back-and-forth lines across each new quadrant
LINE_MIN = .008             # a line runs at least 8 mm across the quadrant
CROSS_TOL = .001            # lines within 1 mm cross (the ring is 5 mm wide)
SAME_TOUCH_GAP = .010       # a touch crossing its own line must be 10 mm of path apart
LID_AWAY = .080             # lid centre farther than this from the dish centre: off the plate
LID_SEAT_XY, LID_SEAT_Z_TOL, LID_SEAT_TILT = .0025, .002, 3.
DISH_TIP_DEG = 15.
STILL_SPEED, STILL_SPIN = .010, .50     # m/s; rad/s for body spin and the neck joint


def _rot(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def tip_point(loop):
    """Lowest point of the ring disc, from the neck link's reported pose."""
    c = loop.link_to_world("neck", RING_C)
    n = loop.link_rotation("neck") @ np.array([0., 1., 0.])
    down = np.array([0., 0., -1.]) - n*(-n[2])
    norm = np.linalg.norm(down)
    return c + RING_R*down/norm if norm > 1e-9 else c


def flex_deg(loop):
    return math.degrees(loop.joints["neck"])


def on_agar(loop, dish):
    """(touching, tip in dish frame): the neck is bent by a load at its tip and the tip
    sits at the agar surface inside the dish, so the agar is what carries it."""
    local = dish.to_local(tip_point(loop))
    inside = math.hypot(local[0], local[1]) <= TRACE_R_MAX
    at_surface = abs(local[2]-AGAR_TOP) <= CONTACT_Z_TOL
    return bool(inside and at_surface and flex_deg(loop) >= FLEX_ON_DEG), local


def gouged(loop, dish):
    local = dish.to_local(tip_point(loop))
    inside = math.hypot(local[0], local[1]) <= TRACE_R_MAX
    near = abs(local[2]-AGAR_TOP) <= CONTACT_Z_TOL
    return bool(inside and ((near and flex_deg(loop) >= FLEX_GOUGE_DEG) or
                            local[2] < AGAR_TOP-CONTACT_Z_TOL))


def in_patch(p):
    """Inside the specimen area spread on the far quarter (dish frame xy)."""
    q = _rot(-PRIMARY_DIR_DEG) @ np.asarray(p, float)[:2]
    r, a = math.hypot(*q), math.degrees(math.atan2(q[1], q[0]))
    return PATCH_R[0] <= r <= PATCH_R[1] and abs(a) <= PATCH_HALF_DEG


def quadrant(p, d):
    """(quadrant 1-4 or 0 for the boundary band, plain quadrant 1-4) of a dish-frame point.
    Quadrant 1 holds the specimen; 2, 3, 4 follow round the plate in direction d (+1 = CCW)."""
    q = _rot(-PRIMARY_DIR_DEG) @ np.asarray(p, float)[:2]
    a = math.degrees(math.atan2(q[1], q[0]))*d
    k = int(((a+45.) % 360.)//90.)+1
    # distances to the two diagonal boundary lines through the centre
    u = np.array([1., 1.])/math.sqrt(2)
    v = np.array([1., -1.])/math.sqrt(2)
    clear = min(abs(q @ u), abs(q @ v)) >= BOUNDARY_CLEAR
    return (k if clear else 0), k


def _seg_dist(p1, p2, q1, q2):
    """Distance between segments p1p2 (Nx2 each) and q1q2 (Mx2 each); returns NxM."""
    p1, p2, q1, q2 = (np.asarray(a, float) for a in (p1, p2, q1, q2))
    P1, P2 = p1[:, None, :], p2[:, None, :]
    Q1, Q2 = q1[None, :, :], q2[None, :, :]
    d1, d2, r = P2-P1, Q2-Q1, P1-Q1
    cross = lambda a, b: a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]
    den = cross(d1, d2)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = cross(Q1-P1, d2)/den
        t = cross(Q1-P1, d1)/den
    hit = (np.abs(den) > 1e-15) & (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)

    def pt_seg(p, a, b):
        ab = b-a
        u = np.clip(np.sum((p-a)*ab, -1)/np.maximum(np.sum(ab*ab, -1), 1e-30), 0, 1)
        return np.linalg.norm(p-(a+u[..., None]*ab), axis=-1)
    d = np.minimum.reduce([pt_seg(P1, Q1, Q2), pt_seg(P2, Q1, Q2), pt_seg(Q1, P1, P2), pt_seg(Q2, P1, P2)])
    return np.where(hit, 0., d)


def _monotone_runs(u):
    """Number of back-and-forth strokes of at least LINE_MIN in a 1-D coordinate sequence
    (turns are recognised with LINE_MIN of hysteresis)."""
    if len(u) < 2:
        return 0
    runs, direction, anchor, extreme = 0, 0, u[0], u[0]
    for x in u[1:]:
        if direction == 0:
            if abs(x-anchor) >= LINE_MIN:
                direction = 1 if x > anchor else -1
                extreme = x
                runs = 1
        elif (x-extreme)*direction > 0:
            extreme = x
        elif abs(x-extreme) >= LINE_MIN:
            direction, anchor, extreme = -direction, extreme, x
            runs += 1
    return runs


def evaluate(touches):
    """Quadrant streak record from the trace: a list of touches, each an (N,2) array of
    dish-frame tip points in time order. Returns a dict with the credited sectors."""
    touches = [np.asarray(t, float).reshape(-1, 2) for t in touches if len(t)]
    flat = [(i, j, p) for i, t in enumerate(touches) for j, p in enumerate(t)]
    # Direction: the side of the first point that leaves quadrant 1 sideways.
    d = 1
    for _, _, p in flat:
        clear, _ = quadrant(p, 1)
        if clear in (2, 4):
            d = 1 if clear == 2 else -1
            break
    labels = [[quadrant(p, d) for p in t] for t in touches]
    out = {"direction": d, "sectors": 0, "detail": {}}
    path_s = []
    for t in touches:
        s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(t, axis=0), axis=1))] if len(t) else np.zeros(0)
        path_s.append(s)
    entries = {k: next(((i, j) for i, lab in enumerate(labels) for j, l in enumerate(lab) if l[0] == k), None)
               for k in (2, 3, 4, 5)}
    for k in (2, 3, 4):
        info = {}
        out["detail"][k] = info
        # (b) streak length clearly inside quadrant k
        length = 0.
        for t, lab in zip(touches, labels):
            for j in range(len(t)-1):
                if lab[j][0] == k and lab[j+1][0] == k:
                    length += float(np.linalg.norm(t[j+1]-t[j]))
        info["length_mm"] = round(length*1000, 1)
        # (b') a streak, not a single line: at least SECTOR_MIN_LINES back-and-forth lines
        # across the quadrant, each running >= LINE_MIN along its width
        lines = 0
        axis = _rot(PRIMARY_DIR_DEG + d*90.*(k-1)) @ np.array([0., 1.])
        for t, lab in zip(touches, labels):
            run = []
            for p, l in zip(t, lab):
                if l[0] == k:
                    run.append(float(p @ axis))
                    continue
                lines += _monotone_runs(run)
                run = []
            lines += _monotone_runs(run)
        info["lines"] = lines
        # first point clearly in quadrant k
        entry = entries[k]
        info["entry"] = entry
        if entry is None:
            break
        i, j = entry
        t = touches[i]
        # (a) the entering touch first picks up from quadrant k-1
        crossed = False
        if k == 2:
            crossed = any(in_patch(p) for p in t[:j+1])
        else:
            prev = []
            for ii in range(i+1):
                tt, lab = touches[ii], labels[ii]
                for jj in range(len(tt)-1):
                    if lab[jj][1] == k-1 and lab[jj+1][1] == k-1:
                        prev.append((ii, jj))
            if prev and j >= 1:
                a1, a2 = t[:j], t[1:j+1]
                for ii, jj in prev:
                    if ii == i:
                        # own line only if well back along the path
                        far = path_s[i][1:j+1] - path_s[i][jj+1] >= SAME_TOUCH_GAP
                        if not np.any(far):
                            continue
                        d_ = _seg_dist(a1[far], a2[far], touches[ii][jj:jj+1], touches[ii][jj+1:jj+2])
                    else:
                        d_ = _seg_dist(a1, a2, touches[ii][jj:jj+1], touches[ii][jj+1:jj+2])
                    if np.any(d_ <= CROSS_TOL):
                        crossed = True
                        break
        info["crossed"] = crossed
        # (c) from the start of quadrant k until quadrant k+1 starts, nothing clearly in
        # quadrants <= k-2 (a later quadrant's own stray is charged to that quadrant)
        back = False
        if k >= 3:
            end = entries.get(k+1) if k < 4 else None
            for ii in range(i, len(touches)):
                lo = j if ii == i else 0
                hi = end[1] if end is not None and ii == end[0] else len(labels[ii])
                if end is not None and ii > end[0]:
                    break
                if any(0 < l[0] <= k-2 for l in labels[ii][lo:hi]):
                    back = True
                    break
        info["re_entered_earlier"] = back
        if not (crossed and length >= SECTOR_MIN_LENGTH and lines >= SECTOR_MIN_LINES and not back):
            break
        out["sectors"] = k-1
    return out


# ---------------------------------------------------------------- trace record
TEST_TRACE = None           # list of (N,2) dish-frame arrays replaces the recorded trace
TEST_EVER = None            # {"gouge"|"lid_on_bench"|"lid_off_plate"|"dish_tipped": True} history
_TRACE = []
_TRACE_EPISODE = None
_LAST_TOKEN = None
_PREV_TOUCH = False
_EVAL_CACHE = [None, None]


def reset_trace():
    global _TRACE_EPISODE, _LAST_TOKEN, _PREV_TOUCH
    _TRACE.clear()
    _TRACE_EPISODE = _RUN["episode"]
    _LAST_TOKEN, _PREV_TOUCH = None, False
    _EVAL_CACHE[:] = [None, None]


def record_trace(loop, dish):
    """Append this control step's tip point when it rests on the agar (once per step)."""
    global _LAST_TOKEN, _PREV_TOUCH
    if _TRACE_EPISODE != _RUN["episode"]:
        reset_trace()
    token = (_RUN["episode"], _RUN["step"])
    if _RUN["t"] is None or token == _LAST_TOKEN:
        return
    if _LAST_TOKEN is not None and token[1] != _LAST_TOKEN[1]+1:
        _PREV_TOUCH = False
    touching, local = on_agar(loop, dish)
    if touching:
        if not _PREV_TOUCH:
            _TRACE.append([])
        _TRACE[-1].append(local[:2].copy())
    _PREV_TOUCH, _LAST_TOKEN = touching, token


def trace():
    return TEST_TRACE if TEST_TRACE is not None else _TRACE


def _evaluate_cached(touches):
    key = (id(touches), len(touches), len(touches[-1]) if len(touches) else 0)
    if _EVAL_CACHE[0] != key:
        _EVAL_CACHE[:] = [key, evaluate(touches)]
    return _EVAL_CACHE[1]


def lid_centre(lid):
    return lid.to_world((0., 0., SKIRT_H+LID_T/2))


def lid_seated(lid, dish):
    local = dish.to_local(lid.origin)
    return bool(math.hypot(local[0], local[1]) <= LID_SEAT_XY and
                abs(local[2]-LID_SEAT_Z) <= LID_SEAT_Z_TOL and
                _rel_tilt_deg(lid, dish) <= LID_SEAT_TILT)


def _rel_tilt_deg(a, b):
    za, zb = a.rotation()[:, 2], b.rotation()[:, 2]
    return math.degrees(math.acos(float(np.clip(za @ zb, -1, 1))))


def _boxes_apart(a, b, gap):
    return bool(np.any(np.asarray(a.low) > np.asarray(b.high)+gap) or
                np.any(np.asarray(b.low) > np.asarray(a.high)+gap))


def loop_discarded(loop, tray):
    """The loop lies inside the tray: its whole box within the tray's inner walls, below the rim."""
    lo, hi = np.asarray(loop.low), np.asarray(loop.high)
    c = np.asarray(tray.origin)
    inner_lo = c+np.r_[-TRAY_IN/2, TRAY_W-.002]
    inner_hi = c+np.r_[TRAY_IN/2, TRAY_H+.010]
    return bool(np.all(lo[:2] >= inner_lo[:2]) and np.all(hi[:2] <= inner_hi[:2]) and
                lo[2] >= inner_lo[2] and hi[2] <= inner_hi[2])


_POSES = {}
STEADY_MM, STEADY_DEG = 1.0, 1.0


def steady(key, state):
    """Moved less than STEADY_MM and STEADY_DEG over the last STILL_S seconds (online); offline,
    at rest by its reported velocities."""
    t = _RUN["t"]
    if t is None:
        return bool(state.resting(max_speed=STILL_SPEED, max_spin=STILL_SPIN))
    _episode_memory()
    if _POSES.get("_episode") != _RUN["episode"]:
        _POSES.clear()
        _POSES["_episode"] = _RUN["episode"]
    hist = _POSES.setdefault(key, [])
    p, R = np.asarray(state.origin, float), state.rotation()
    if not hist or hist[-1][0] != _RUN["step"]:
        hist.append((_RUN["step"], t, p, R))
    while len(hist) > 2 and t - hist[1][1] >= STILL_S:
        hist.pop(0)
    if t - hist[0][1] < STILL_S - 1e-9:
        return False
    for _, _, q, Q in hist:
        if np.linalg.norm(q-p)*1000 > STEADY_MM:
            return False
        c = float(np.clip((np.trace(Q.T @ R)-1)/2, -1, 1))
        if math.degrees(math.acos(c)) > STEADY_DEG:
            return False
    return True


def _hist(key, cond):
    if TEST_EVER is not None and TEST_EVER.get(key):
        return True
    return ever(key, cond)


def _status(objects):
    dish, lid, loop, tray = objects["dish"], objects["lid"], objects["loop"], objects["tray"]
    record_trace(loop, dish)
    touches = trace()
    ev = _evaluate_cached(touches)
    centre = lid_centre(lid)
    away = math.hypot(*(centre[:2]-np.asarray(dish.origin)[:2]))
    seated = lid_seated(lid, dish)
    viol = {
        "gouge": _hist("gouge", gouged(loop, dish)),
        "lid_on_bench": _hist("lid_on_bench", (not seated) and lid.low[2] <= TOP+.002),
        "lid_off_plate": _hist("lid_off_plate", away > LID_AWAY),
        "dish_tipped": _hist("dish_tipped", dish.tilt_deg() > DISH_TIP_DEG),
    }
    lid_placed = seated and released(lid)
    loop_placed = (loop_discarded(loop, tray) and released(loop) and
                   _boxes_apart(loop, dish, .005) and _boxes_apart(loop, lid, .005))
    # Released for 1 s, and over that second the parts moved < 1 mm and < 1 degree
    # (windowed, because resting contacts make instantaneous velocities chatter).
    lid_steady, dish_steady, loop_steady = steady("lid", lid), steady("dish", dish), steady("loop", loop)
    lid_still = lid_steady and dish_steady
    loop_still = loop_steady and abs(loop.joint_vel.get("neck", 0.)) < STILL_SPIN
    lid_s = held_for("lid_closed_released", lid_placed)
    loop_s = held_for("loop_discarded_released", loop_placed)
    lid_done = lid_placed and lid_still and lid_s >= STILL_S
    loop_done = loop_placed and loop_still and loop_s >= STILL_S
    return ev, viol, seated, lid_done, loop_done, lid_s, loop_s


_LAST_LOG = None
LOG_EVERY = 20


def score(objects):
    global _LAST_LOG
    ev, viol, seated, lid_done, loop_done, lid_s, loop_s = _status(objects)
    n = ev["sectors"]
    value = 0. if any(viol.values()) else .2*n
    if value > 0:
        value += .2*lid_done + .2*loop_done
    value = round(value, 6)
    token = (_RUN["episode"], _RUN["step"])
    if _RUN["env"] is not None and token != _LAST_LOG and token[1] % LOG_EVERY == 0:
        loop = objects["loop"]
        hands = grippers() or {}
        print("HANDS " + " ".join(f"{k}=({v[0][0]:.4f},{v[0][1]:.4f},{v[0][2]:.4f}) open={v[1]:.3f}"
                                  for k, v in hands.items()), flush=True)
        print(f"STREAK t={_RUN['t']:.2f} score={value} sectors={n} dir={ev['direction']} "
              f"detail={ev['detail']} touches={len(trace())} viol={[k for k, v in viol.items() if v]} "
              f"lid_seated={seated} lid_done={lid_done}/{lid_s:.2f} loop_done={loop_done}/{loop_s:.2f} "
              f"lid_rest={objects['lid'].resting()} dish_rest={objects['dish'].resting()} "
              f"lid_rel={released(objects['lid'])} loop_in={loop_discarded(objects['loop'], objects['tray'])} "
              f"flex_deg={flex_deg(loop):.2f} tip={np.round(tip_point(loop), 4).tolist()}", flush=True)
        _LAST_LOG = token
        if token[1] % 400 == 0:
            for i, t in enumerate(trace()):
                pts = " ".join(f"{p[0]*1000:.0f},{p[1]*1000:.0f}" for p in np.asarray(t)[::4])
                print(f"TRACE {i} n={len(t)} {pts}", flush=True)
    return value


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- scripted demo
# Tilted-wrist waypoints: a float yaw (pad closing axis, for coarse offline replay) that
# carries the full grip rotation for the arm's IK. GPU-proven pattern (hoist task).
class GripOrientation(float):
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_streak_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    if isinstance(yaw, np.ndarray) and yaw.shape == (3, 3):
        return yaw
    return _original_grip_rotation(yaw)


_grip_rotation._streak_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation


def _rx(deg):
    return axis_angle_matrix(np.array([1., 0., 0.]), math.radians(deg))


LOOP_TILT_DEG = 35.                        # loop axis 35 deg from vertical, tip toward +y
R_LOOP_V = _original_grip_rotation(math.pi/2)
R_LOOP_T = _rx(LOOP_TILT_DEG) @ R_LOOP_V
LID_TILT_DEG = 30.                         # clamshell: the lid's -y edge rises
R_LID = _original_grip_rotation(-math.pi/2)
R_LID_T = _rx(-LID_TILT_DEG) @ R_LID
GRASP_ABOVE_SHOULDER = .065                # pad tips on the flat grip, 8 mm above its foot
S_TIP = GRASP_ABOVE_SHOULDER - TIP_Z       # TCP to tip along the loop axis, 0.162 m
RIDE = 0.                                 # measured: the loop does not slip along the pads
PRESS = .0050                              # tip commanded this far below the agar surface
HOVER = .008                               # tip clearance between touches
# Bent about 8 degrees at the working press, the 60 mm neck carries the tip about 9 mm
# forward (+y) of where a straight loop would put it; the plan aims that much short.
TIP_SHIFT = np.array([0., -.009])
DRAW_SPEED = .020
# The fingertips taper to about 16-22 mm wide near their ends, so with the TCP 16 mm off
# centre the pads' inner edges meet the skirt 5-8 mm off centre, on an 89-90 mm chord.
LID_PINCH_Y = .016
LID_CHORD = 2*math.sqrt(LID_R**2-.007**2)
LID_SQUEEZE = .0020                        # each finger commanded 2 mm past the skirt (about 10 N)
LID_FINGER = -(LID_CHORD/2) + LID_SQUEEZE
LID_GRASP_Z = TOP+LID_SEAT_Z+.0010+.013    # pad tips 1 mm above the skirt rim
LID_HOLD = np.array([0., .045, .065])      # grasp point offset when held open


def _a(R):
    return R[:, 2]


def tcp_for_tip(tip_world, R=R_LOOP_T):
    return np.asarray(tip_world, float) - _a(R)*(S_TIP+RIDE)


def _world_tip(p_local, dz):
    p = DISH_XY + np.asarray(p_local[:2], float) + TIP_SHIFT
    return np.array([p[0], p[1], TOP+AGAR_TOP+dz])


def polar(r, deg):
    a = math.radians(deg)
    return np.array([r*math.cos(a), r*math.sin(a)])


def sector_plan(k, d=1):
    """Dish-frame tip path for quadrant k (2-4): two passes from the previous quadrant's
    streaks (or the specimen) into quadrant k, then a zigzag kept inside quadrant k."""
    phi = PRIMARY_DIR_DEG + d*90.*(k-1)
    prev = phi - d*90.
    # Near the loop hand (-y) the tip stays 14 mm inside the wall, so the rising neck
    # clears the rim.
    near = k == 4
    pts = [polar(.024, prev - d*12.), polar(.029 if near else .033, phi - d*30.),
           polar(.026 if near else .028, prev + d*8.), polar(.022, phi - d*22.)]
    radii = (.030, .027, .024, .021, .018, .015) if near else (.034, .029, .024, .019, .014)
    for i, r in enumerate(radii):
        side = -1 if i % 2 == 0 else 1
        pts.append(polar(r, phi + d*side*(30. if near else 28.)))
    return np.array(pts)


def densified(path, step=.001):
    """A polyline resampled every `step` metres (a recorded trace has one point per 50 ms)."""
    path = np.asarray(path, float)
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        n = max(1, int(np.linalg.norm(b-a)/step))
        out += [a+(b-a)*(i+1)/n for i in range(n)]
    return np.array(out)


def nominal_trace(d=1):
    return [densified(sector_plan(k, d)) for k in (2, 3, 4)]


def straight(*pts):
    return densified(np.array([polar(r, a) for r, a in pts]))


def spiral(turns=2., r0=.034, r1=.006):
    """One continuous inward spiral from the specimen (a cheat fixture for tests)."""
    a = np.linspace(0., 360.*turns, int(40*turns)+1)
    return densified(np.array([polar(r0+(r1-r0)*x/a[-1], x) for x in a]))


def _densify(path, speed):
    """Waypoint list (dt, point) along a polyline at a steady speed (one waypoint per corner)."""
    out = []
    for a, b in zip(path[:-1], path[1:]):
        out.append((max(float(np.linalg.norm(b-a))/speed, .25), b))
    return out


def demo():
    W = Waypoint
    plan = {"right": [], "left": []}
    G = GripOrientation
    state = {"r": None, "l": None}

    def add(dt, label, right=None, left=None, rf=None, lf=None, rr=None, lr=None):
        r = right if right is not None else state["r"][0]
        l = left if left is not None else state["l"][0]
        rf = state["r"][1] if rf is None else rf
        lf = state["l"][1] if lf is None else lf
        rr = state["r"][2] if rr is None else rr
        lr = state["l"][2] if lr is None else lr
        state["r"], state["l"] = (np.asarray(r, float), rf, rr), (np.asarray(l, float), lf, lr)
        plan["right"].append(W(dt*DEMO_PACE, tuple(float(v) for v in r), rf, label, G(rr)))
        plan["left"].append(W(dt*DEMO_PACE, tuple(float(v) for v in l), lf, label, G(lr)))

    rack_grasp = np.array([*RACK_XY, TOP+SHOULDER_Z+.0002+GRASP_ABOVE_SHOULDER])
    lid_grasp = np.array([DISH_XY[0], DISH_XY[1]+LID_PINCH_Y, LID_GRASP_Z])
    lid_open = lid_grasp + LID_HOLD
    # The right fingers come in beside the loop (40 mm toward -y, below the handle top)
    # and slide sideways onto it, so the open fingers never sweep across the handle.
    state["r"] = (rack_grasp+[0, -.040, .004], FINGER_OPEN, R_LOOP_V)
    state["l"] = (lid_grasp+[0, 0, .06], FINGER_OPEN, R_LID)
    add(2.0, "approach loop and lid")
    add(1.2, "descend", right=rack_grasp, left=lid_grasp)
    add(.8, "close", rf=FINGER_CLOSED, lf=LID_FINGER)
    add(.8, "lift loop off ledge", right=rack_grasp+[0, 0, .008])
    add(1.2, "slide loop out of rack", right=rack_grasp+[.045, 0, .008])
    # Lid: rise, then open as a clamshell over the plate.
    # Straight up until the skirt clears the base wall, then open over the plate.
    add(.8, "lid straight up", right=rack_grasp+[.045, 0, .020], left=lid_grasp+[0, 0, .012])
    add(1.2, "lid open", left=lid_open, lr=R_LID_T)
    # Loop: tilt and carry to above the first stroke.
    first = sector_plan(2)
    tip0 = _world_tip(first[0], .020)
    add(2.0, "tilt loop", right=tcp_for_tip(_world_tip(first[0], .045)), rr=R_LOOP_T)
    add(1.0, "over first stroke", right=tcp_for_tip(tip0))
    for k in (2, 3, 4):
        path = sector_plan(k)
        add(1.0, f"Q{k} hover", right=tcp_for_tip(_world_tip(path[0], HOVER)))
        add(.6, f"Q{k} settle")
        add(.8, f"Q{k} touch down", right=tcp_for_tip(_world_tip(path[0], -PRESS)))
        for i, (dt, p) in enumerate(_densify(path, DRAW_SPEED)):
            add(dt, f"Q{k} streak {i:02}", right=tcp_for_tip(_world_tip(p, -PRESS)))
        add(.3, f"Q{k} dwell")
        add(.7, f"Q{k} lift", right=tcp_for_tip(_world_tip(path[-1], HOVER+.010)))
    # Loop up and out over the tray; lid back down and seated.
    out_tip = np.array([DISH_XY[0], DISH_XY[1]-.070, TOP+.050])
    add(1.2, "loop out of plate", right=tcp_for_tip(out_tip))
    add(1.5, "lid back over plate", left=lid_grasp+[0, 0, .012], lr=R_LID)
    add(.8, "lid settle")
    add(1.0, "lid down", left=lid_grasp+[0, 0, .002])
    add(.8, "lid release", lf=FINGER_OPEN)
    add(1.0, "left clear up", left=lid_grasp+[0, 0, .070])
    tray_tip = np.array([TRAY_XY[0], TRAY_XY[1]+.100, TOP+.040])
    add(1.8, "loop over tray", right=tcp_for_tip(tray_tip))
    add(1.0, "loop into tray", right=tcp_for_tip(tray_tip+[0, 0, -.024]))
    add(.8, "loop release", rf=FINGER_OPEN)
    add(1.0, "right clear up", right=state["r"][0]+[0, -.040, .050])
    add(3.5, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])


import streak_ink
streak_ink.install(sys.modules.get(__name__) or globals())
