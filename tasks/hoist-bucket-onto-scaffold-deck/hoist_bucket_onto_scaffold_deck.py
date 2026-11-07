"""Hoist, draw in and land a pail using a passive gin wheel."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
# A long-lived checker may hold another folder's (or an older) copy of these
# task-local helpers; always import this folder's current files.
for _helper in ('hoist_geometry', 'hoist_runtime', 'hoist_physics', 'hoist_motion'):
    sys.modules.pop(_helper, None)
import math
from collections import deque
from types import SimpleNamespace
import numpy as np
from yam_arena.constants import FINGER_OPEN
from hoist_geometry import TOP, wheel, make_rope, deck, rotation, PITCH, RADIUS, HALF_GAP, PITCH_R
from hoist_runtime import install, RopeState, seating, nominal_state
from hoist_motion import demo

SCENE = 'station'
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 1.0
AXLE = (.180, -.030, TOP+.400)
HOIST = wheel('hoist_a', AXLE)
ROPE = make_rope('rope', AXLE, 1, .30, 'hoist_a', 'right', (.60, .62, .65))
DECK = deck()
OBJECTS = [HOIST, ROPE, DECK]
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

# All scoring thresholds are physical distances, angles or elapsed video time.
DECK_TOP = TOP+.140
DECK_EDGES = (.100, .260, .050, .210)
PAIL_HALF = .030
PAIL_BOTTOM = -.035
PAIL_RIM = .035
START_BOTTOM = TOP+.002
DROP_SPEED = .400
SPILL_DEG = 45.0
RAISED_DELTA = .010
HANDLIFT_TOP_GAP = .005
HAND_CLEAR = .012
LOAD_ROPE_CLEAR = .015
CLOSED_FRACTION = .50
UNDER_HEIGHT = .030
# Landed is one-sided: resting contact may sink up to LANDED_BELOW, but a pail
# held or hung more than LANDED_ABOVE over the boards is not resting on them.
LANDED_BELOW = .005
LANDED_ABOVE = .001
LANDED_DEG = 10.0
INSIDE_MARGIN = .020
# Hanging starts where landed ends, so no bottom height falls between tiers.
HANGING_GAP = LANDED_ABOVE
STILL_CHANGE = .001
HANGING_CHANGE = .005
SEAT_TOP = .010
ANCHOR_GAP = .010
TAUT_EXTRA = .008
OBSERVATION_GAP = .100
SEAT_RADIUS_GATE = .055
SEAT_MIN_SURFACE = .025
SEAT_MIN_CENTRE = .028
SEAT_AXIAL_TOL = .0001
# While the pail is raised below the deck, the rope must carry it: a load fall
# that is detached or slack for longer than this is a sticky zero.
ROPE_LOAD_S = .5
# Offline layouts only (states.yaml): re-lay the load fall with this much extra
# rope in metres, bowed sideways, as when a hand rather than the rope carries
# the pail. GPU trials leave it None and measure the simulated rope.
TEST_LOAD_SLACK = None
NUMERIC_EPS = 1.e-9
_POSES = deque()
_PREVIOUS = None
_POSE_EPISODE = None
LAST = {}


def box_view(low, high):
    return SimpleNamespace(low=np.asarray(low), high=np.asarray(high))


def point_box_distance(tcp, view):
    return float(np.linalg.norm(np.maximum(np.maximum(view.low-tcp, tcp-view.high), 0.)))


def pail_points(st):
    p, q = st.poses['bucket']
    r = rotation(q)
    return p+np.array([(x, y, z) for x in (-PAIL_HALF, PAIL_HALF)
                      for y in (-PAIL_HALF, PAIL_HALF) for z in (PAIL_BOTTOM, PAIL_RIM)])@r.T


def views(st):
    p, q = st.poses['bucket']
    r = rotation(q)
    points = pail_points(st)
    out = [box_view(points.min(0), points.max(0))]
    # Exact per-primitive rotated bounds, including the ears and the wire arch.
    # Keep the Part class that owns these shapes when the checker reloads kits.
    from hoist_geometry import Part
    for shape in st.spec.bucket_part().shapes[5:]:
        lo, hi = Part('bail_piece', (0, 0, 0), [shape]).aabb(p, r)
        out.append(box_view(lo, hi))
    for c, a in zip(st.centres, st.axes):
        extent = RADIUS+(PITCH/2-RADIUS)*np.abs(a)
        out.append(box_view(c-extent, c+extent))
    return out


def observe(st):
    """Corner, centre and bottom diameters over the same complete second."""
    global _PREVIOUS, _POSE_EPISODE
    t, step = _RUN['t'], _RUN['step']
    points = pail_points(st)
    bottom = float(points[:, 2].min())
    if t is None:
        return bottom, False, 0., 0., 0., True
    if _POSE_EPISODE != _RUN['episode']:
        _POSES.clear()
        _PREVIOUS = None
        _POSE_EPISODE = _RUN['episode']
    drop = False
    if _PREVIOUS is not None:
        ps, pt, pb = _PREVIOUS
        if step != ps and t > pt:
            drop = (pb-bottom)/(t-pt) > DROP_SPEED
        if step not in (ps, ps+1) or t < pt or t-pt > OBSERVATION_GAP:
            _POSES.clear()
    if _PREVIOUS is None or step != _PREVIOUS[0]:
        _POSES.append((t, points.copy()))
        _PREVIOUS = (step, t, bottom)
    while len(_POSES) > 1 and _POSES[1][0] <= t-STILL_S+NUMERIC_EPS:
        _POSES.popleft()
    history = np.array([row[1] for row in _POSES])
    change = float(np.max(np.linalg.norm(history[:, None]-history[None, :], axis=-1)))
    centres = history.mean(axis=1)
    centre_change = float(np.max(np.linalg.norm(centres[:, None]-centres[None, :], axis=-1)))
    bottoms = history[:, :, 2].min(axis=1)
    bottom_change = float(np.ptp(bottoms))
    complete = bool(_POSES and t-_POSES[0][0] >= STILL_S-NUMERIC_EPS)
    return bottom, drop, change, centre_change, bottom_change, complete


def in_footprint(tcp, points):
    # Convex hull of the current eight projected pail corners, no margin.
    vertices = sorted(set(map(tuple, points[:, :2])))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for v in vertices:
        while len(lower)>=2 and cross(lower[-2], lower[-1], v)<=0: lower.pop()
        lower.append(v)
    for v in reversed(vertices):
        while len(upper)>=2 and cross(upper[-2], upper[-1], v)<=0: upper.pop()
        upper.append(v)
    hull = lower[:-1]+upper[:-1]
    return all(cross(a,b,tcp[:2]) >= -NUMERIC_EPS for a,b in zip(hull,hull[1:]+hull[:1]))


def measure(objects):
    st = objects.get('rope')
    if not isinstance(st, RopeState):
        # A generic box/asset state cannot represent 54 measured rope bodies.
        return None
    if (set(st.poses) != set(st.spec.body_names()) or
        not np.isfinite(np.concatenate([np.r_[p, q] for p, q in st.poses.values()])).all() or
        any(np.linalg.norm(q) < NUMERIC_EPS for p, q in st.poses.values())):
        return None
    if TEST_LOAD_SLACK:
        p, q = st.poses['bucket']
        st = nominal_state(st.spec, tuple(map(float, p)), tuple(rotation(q).ravel()), float(TEST_LOAD_SLACK))
    points = pail_points(st)
    bottom, drop, change, centre_change, bottom_change, complete = observe(st)
    p, q = st.poses['bucket']
    tilt = math.degrees(math.acos(np.clip(rotation(q)[2, 2], -1., 1.)))
    yaw_deg = math.degrees(math.atan2(rotation(q)[1, 0], rotation(q)[0, 0]))
    xmin, xmax, ymin, ymax = DECK_EDGES
    margins = np.array((points[:, 0].min()-xmin, xmax-points[:, 0].max(),
                        points[:, 1].min()-ymin, ymax-points[:, 1].max()))
    centred = xmin <= p[0] <= xmax and ymin <= p[1] <= ymax
    landed = (DECK_TOP-LANDED_BELOW-NUMERIC_EPS <= bottom <= DECK_TOP+LANDED_ABOVE+NUMERIC_EPS
              and centred and tilt <= LANDED_DEG)
    seat = seating(st.spec, st.centres, st.axes)
    rows = seat['segments']
    seated = any(st.centres[int(row['segment'][-2:]), 2] >= AXLE[2]+SEAT_TOP
                 and not row['outside_flange']
                 and row['surface_radius_mm'] >= 1000*SEAT_MIN_SURFACE
                 for row in rows) and not seat['tunnelling']
    apex = int(np.argmin(np.linalg.norm(st.centres-(np.array(AXLE)+(0, 0, PITCH_R)), axis=1)))
    load = st.centres[:apex]
    hands = grippers()
    obj_views = views(st)
    handlift = False
    under = False
    raised_low = START_BOTTOM+RAISED_DELTA < bottom < DECK_TOP-HANDLIFT_TOP_GAP
    if hands:
        for tcp, opening in hands.values():
            tcp = np.asarray(tcp)
            if raised_low:
                handlift |= any(point_box_distance(tcp, view) < HAND_CLEAR for view in obj_views[:-st.spec.segments])
                handlift |= opening <= CLOSED_FRACTION and any(
                    point_box_distance(tcp, view) < LOAD_ROPE_CLEAR for view in obj_views[-st.spec.segments:][:apex])
            # Test the current rotated footprint in pail coordinates, no margin.
            # A landed pail rests on the boards and is no longer a suspended load.
            under |= (not landed and in_footprint(tcp, points) and tcp[2] < bottom
                      and bottom > TOP+UNDER_HEIGHT)
    anchor = p+rotation(q)@st.spec.bail_offset()
    first = st.centres[0]-st.axes[0]*PITCH/2
    # The load fall must be attached and taut, measured from link endpoints.
    # Exclude the curved wrap by ending at the first link entering the wheel.
    entry = next((int(row['segment'][-2:]) for row in rows), apex)
    end = st.centres[entry]-st.axes[entry]*PITCH/2
    taut = entry*PITCH-np.linalg.norm(end-anchor) <= TAUT_EXTRA
    attached = np.linalg.norm(first-anchor) <= ANCHOR_GAP
    # A pail raised below the deck must hang on an attached, taut load fall;
    # otherwise a hand, wrist or forearm is carrying it (timed in score()).
    unloaded = raised_low and not (attached and taut)
    inside = bool(np.all(margins >= INSIDE_MARGIN-NUMERIC_EPS))
    # Reuse this call's joint-state FK, so all boxes see identical hand poses.
    clear = hands is None or all(point_box_distance(np.asarray(tcp), view) >= RELEASE_CLEAR
                                 for tcp, _ in hands.values() for view in obj_views)
    still = complete and change < STILL_CHANGE
    # A free swivel may turn while the load is held steadily by the rope.
    # Landed tiers retain the stricter corner-pose test above.
    hanging_still = complete and max(centre_change, bottom_change) < HANGING_CHANGE
    # Footprint and vertical separation jointly exclude deck contact.
    no_deck_contact = bottom > DECK_TOP+HANGING_GAP+NUMERIC_EPS
    hanging = no_deck_contact and seated and attached and taut
    return dict(bottom=bottom, centre=p, tilt=tilt, yaw_deg=yaw_deg, margins=margins, seated=bool(seated),
                load_links=len(load), drop=bool(drop), spill=tilt > SPILL_DEG,
                handlift=bool(handlift), under=bool(under), derail=not seated, unloaded=bool(unloaded),
                landed=bool(landed), inside=inside, released=clear, still=still,
                hanging=bool(hanging), hanging_still=hanging_still,
                pose_change=change, centre_change_mm=centre_change*1000,
                bottom_change_mm=bottom_change*1000, attached=bool(attached), taut=bool(taut))


def score(objects):
    global LAST
    data = measure(objects)
    valid = data is not None
    if not valid:
        data = {key: False for key in ('drop','spill','handlift','under','derail','unloaded',
                                      'landed','inside','released','still','hanging','hanging_still')}
    # Every sticky condition and every timer is evaluated on every call.
    flags = {key: ever(key, bool(data[key])) for key in ('drop','spill','handlift','under','derail')}
    flags['invalid'] = ever('invalid', not valid)
    unloaded_s = held_for('unloaded', valid and data['unloaded'])
    flags['unloaded'] = ever('unloaded', unloaded_s > ROPE_LOAD_S)
    full = valid and data['landed'] and data['inside'] and data['seated'] and data['released'] and data['still']
    half = valid and data['landed'] and data['still']
    quarter = valid and data['hanging'] and data['hanging_still']
    # Pose windows already cover the required second. The geometric conditions
    # must cover it too; avoid imposing a second extra stillness interval.
    full_s = held_for('full', valid and data['landed'] and data.get('inside', False)
                      and data.get('seated', False) and data['released'])
    half_s = held_for('half', valid and data['landed'])
    quarter_s = held_for('quarter', valid and data['hanging'])
    value = 0.
    if not any(flags.values()):
        if full and full_s >= STILL_S-NUMERIC_EPS:
            value = 1.
        elif half and half_s >= STILL_S-NUMERIC_EPS:
            value = .5
        elif quarter and quarter_s >= STILL_S-NUMERIC_EPS:
            value = .25
    LAST = dict(data, sticky=flags, tiers=dict(full=bool(full), half=bool(half), quarter=bool(quarter)), score=value)
    return value


def success(objects):
    return score(objects) >= 1.0


install((ROPE,))
