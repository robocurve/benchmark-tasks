"""Filling a robot axis's cable carrier: an igus E2 mini B15.025.038.0 snap-open cable carrier (energy chain),
12 links of 30.5 mm pitch (366 mm), lies straight on the bench along the y axis, openable side up, with its
fixed-end bracket screwed down. Every link's crossbar is hinged on the far side plate and stands open. Two cables,
a 7 mm grey motor cable and a 5 mm orange encoder cable, each 560 mm, are tie-wrapped to a strain-relief plate
100 mm behind the fixed end; from there they curve out and lie loose on the table beside the carrier, on the
robot's side. A crossbar closes when its free end is pressed past the sprung catch on the near side plate, which
then snaps over it. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import ball, box, cyl
from yam_joints import Hinge, Link, Mechanism, Slide
from yam_mesh import Mesh, MeshMechanism
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

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


# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001

# Carrier frame: x across the chain (+x away from the robot, the far side), y along it (the fixed end at y = 0,
# the moving end at y = -366 mm), z up from the table. The carrier is yaw 0, so carrier and world axes agree.
XC, Y0 = 0.170, 0.183                  # chain centreline x; world y of the fixed end of link 1
N_LINKS = 12
PITCH = 30.5 * MM                      # igus E2 mini B15: 30.5 mm pitch
LENGTH = N_LINKS * PITCH               # 366 mm
BI, BA = 25.0 * MM, 36.0 * MM          # inner / outer width (B15.025.038.0)
FLOOR = 3.0 * MM                       # bottom crossbar top: inner height 17 mm up to the crossbar underside
PLATE_TOP = 20.0 * MM                  # side plates; the crossbar sits on them, outer height 23 mm
BAR_TOP = 23.0 * MM
HINGE_X, HINGE_Z = 16.5 * MM, 21.5 * MM
BAR_END_X = -15.0 * MM                 # free end of the crossbar (over the near side plate)
BAR_W = 10.0 * MM                      # crossbar width along the chain
BAR_OPEN = 100.0                       # degrees: open crossbars lean on their open stop, 10 deg past upright
TONGUE_W = 8.0 * MM
HOOK = ((-18.0, 23.8), (-12.5, 23.8), (-12.5, 24.6), (-16.5, 28.6), (-18.0, 28.6))   # catch profile (x, z) mm
TONGUE_K = 300.0                       # N/m catch spring, preloaded 3 mm: 0.9 N at rest, 1.7 N when pushed clear
TONGUE_PRELOAD = 3.0 * MM
NYLON = (0.30, 0.25)
BLACK = (0.10, 0.10, 0.11)
DARK = (0.18, 0.18, 0.20)


def link_y(k):
    """Carrier-frame y of link k's centre (k = 1 at the fixed end ... 12 at the moving end)."""
    return -(k - 0.5) * PITCH


def _prism_mesh(name, profile_xz, y0, y1, color=None):
    """Closed convex prism: a convex (x, z) polygon extruded along y from y0 to y1 (carrier frame)."""
    prof = [tuple(map(float, p)) for p in profile_xz]
    n = len(prof)
    pts = np.array([(x, y, z) for y in (y0, y1) for (x, z) in prof], float)
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    for k in range(1, n - 1):
        faces += [(0, k + 1, k), (n, n + k, n + k + 1)]
    faces = np.array(faces, int)
    tri = pts[faces]
    if np.einsum('ij,ij->i', tri[:, 0] - pts.mean(0),
                 np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])).sum() < 0:
        faces = faces[:, ::-1]
    proxy = (box((pts.min(0) + pts.max(0)) / 2, np.ptp(pts, axis=0)),)
    return Mesh(name, pts, faces, collision="convex", color=color, proxy=proxy)


def _carrier():
    base = []
    for k in range(1, N_LINKS + 1):
        y = link_y(k)
        for side in (-1, 1):                               # side plates, 29 mm of each 30.5 mm link
            base.append(box((side * (BI + BA) / 4, y, PLATE_TOP / 2), ((BA - BI) / 2, 29.0 * MM, PLATE_TOP), BLACK))
        base.append(box((0.0, y, FLOOR / 2), (BI, 12.0 * MM, FLOOR), BLACK))           # fixed bottom crossbar
    # Fixed-end mounting bracket (screwed to the bench): base plate and two lugs on the side plates.
    base.append(box((0.0, 11.0 * MM, 1.5 * MM), (BA, 22.0 * MM, 3.0 * MM), DARK))
    for side in (-1, 1):
        base.append(box((side * (BI + BA) / 4, 6.0 * MM, PLATE_TOP / 2), ((BA - BI) / 2, 12.0 * MM, PLATE_TOP), DARK))
    # Moving-end bracket (lies free on the bench until the carrier is fitted to the axis's carriage).
    base.append(box((0.0, -LENGTH - 11.0 * MM, 1.5 * MM), (BA, 22.0 * MM, 3.0 * MM), DARK))
    for side in (-1, 1):
        base.append(box((side * (BI + BA) / 4, -LENGTH - 6.0 * MM, PLATE_TOP / 2),
                        ((BA - BI) / 2, 12.0 * MM, PLATE_TOP), DARK))
    # Strain-relief plate with two tie-wrap saddles, screwed to the bench 100 mm behind the fixed end.
    base.append(box((0.0, RELIEF_Y, 1.5 * MM), (40.0 * MM, 16.0 * MM, 3.0 * MM), DARK))
    links, meshes = [], {}
    for k in range(1, N_LINKS + 1):
        y = link_y(k)
        links.append(Link(f"bar{k:02d}", [
            cyl((HINGE_X, y, HINGE_Z), 1.5 * MM, BAR_W, axis="y"),
            box(((BAR_END_X + HINGE_X) / 2, y, (PLATE_TOP + BAR_TOP) / 2), (HINGE_X - BAR_END_X, BAR_W, BAR_TOP - PLATE_TOP)),
        ], Hinge("y", (HINGE_X, y, HINGE_Z), 0.0, BAR_OPEN, damping=3e-4, initial=BAR_OPEN, armature=2e-6),
            mass=0.0011, color=(0.22, 0.22, 0.24), friction=NYLON))
        links.append(Link(f"catch{k:02d}", [
            box((-17.0 * MM, y, 15.9 * MM), (2.0 * MM, TONGUE_W, 15.8 * MM)),
        ], Slide("x", (-17.0 * MM, y, 15.9 * MM), -4.5 * MM, 0.0, damping=2.0, spring=TONGUE_K, rest=TONGUE_PRELOAD,
                 armature=0.02), mass=0.0005, color=BLACK, friction=NYLON))
        meshes[f"catch{k:02d}"] = (_prism_mesh(f"hook{k:02d}", [(x * MM, z * MM) for x, z in HOOK],
                                               y - TONGUE_W / 2, y + TONGUE_W / 2, color=BLACK),)
    return MeshMechanism("carrier", (XC, Y0, TOP), base, links, mass=None, color=BLACK, friction=NYLON,
                         contact_offset=0.001, self_collide=True, meshes=meshes)


# Cables: capsule chains of 20 mm segments with alternate horizontal (z) and vertical (x) bend hinges, rooted at
# their tie-wrap on the strain-relief plate. The tie-wrap grips the cable compliantly: the first link sits on a stiff
# axial spring (3,000 N/m), so a 15 N pull draws it 5 mm through the tie, which the scorer counts as torn out.
RELIEF_Y = 100.0 * MM                   # strain-relief plate, carrier frame y
SEG = 20.0 * MM
N_SEG = 28                              # 560 mm cables
RELIEF_K = 3000.0                       # N/m: 15 N of pull draws the cable 5 mm through a hand-tight tie-wrap
CABLES = {
    # name: (x in the channel, radius, mass, bend stiffness N m/rad per joint, colour, z-joints turned out)
    "grey": (+6.0 * MM, 3.5 * MM, 0.042, 0.030, (0.50, 0.52, 0.55), 4),
    "orange": (-6.0 * MM, 2.5 * MM, 0.023, 0.015, (0.95, 0.42, 0.08), 6),
}
TURN_DEG = 30.0


def _axis(i):
    return "z" if i % 2 else "x"


def cable_start_q(name):
    """Start joints (degrees): out of the tie-wrap, turn 30 deg toward the robot, then back parallel to the chain."""
    n_out = CABLES[name][5]
    q = {}
    q["c01"] = -TURN_DEG
    q[f"c{1 + n_out:02d}"] = TURN_DEG
    return q


def _cable(name):
    x, r, mass, k, color, _ = CABLES[name]
    start = cable_start_q(name)
    m = mass / N_SEG
    links = [Link("c00", [cyl((0, -SEG / 2, 0), r, SEG, axis="y")],
                  Slide("y", (0, 0, 0), -0.020, 0.020, damping=10.0, spring=RELIEF_K, armature=0.05), mass=m)]
    for i in range(1, N_SEG):
        links.append(Link(f"c{i:02d}", [ball((0, -i * SEG, 0), r), cyl((0, -(i + 0.5) * SEG, 0), r, SEG, axis="y")],
                          Hinge(_axis(i), (0, -i * SEG, 0), -60.0, 60.0, damping=0.002, spring=k,
                                initial=start.get(f"c{i:02d}", 0.0), armature=5e-6),
                          mass=m, parent=f"c{i - 1:02d}"))
    band = 1.2 * MM
    tie = [box((0, 0, r + band / 2), (2 * r + 2 * band, 3.0 * MM, band), (0.9, 0.9, 0.88))]
    tie += [box((s * (r + band / 2), 0, 0), (band, 3.0 * MM, 2 * r), (0.9, 0.9, 0.88)) for s in (-1, 1)]
    return Mechanism(name, (XC + x, Y0 + RELIEF_Y, TOP + FLOOR + r + 0.3 * MM), tie, links, mass=None,
                     color=color, friction=(0.45, 0.35), contact_offset=0.0015)


CARRIER = _carrier()
GREY = _cable("grey")
ORANGE = _cable("orange")
OBJECTS = [CARRIER, GREY, ORANGE]
PRESSED_PARTS = ("carrier", "grey", "orange")

# ------------------------------------------------------------------ scoring
LATCH_DEG = 2.5          # closed: free end within 1.4 mm of flush (latched play 1.45 deg; resting on the catch 7.5)
CATCH_ENGAGED = -0.5 * MM  # the catch back within 0.5 mm of its rest position, over the bar
IN_TOL = 1.5 * MM        # a cable's centre within its own radius + 1.5 mm of the inner walls
ORDER_GAP = 1.0 * MM     # grey's centre at least 1 mm farther from the robot than orange's, every 5 mm
OUT_PAST = 60.0 * MM     # both cable ends at least 60 mm out of the moving end
SLIP = 5.0 * MM          # sticky 0: a cable pulled 5 mm through its tie-wrap
STEP_M, STEP_DEG = 0.0005, 0.25
TEST_STICKY = ()


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def cable_points(state, name, step=5.0 * MM):
    """World points along a cable's centreline, from the tie-wrap to the free end."""
    pts = []
    for i in range(N_SEG):
        link = f"c{i:02d}"
        R = state.link_rotation(link)
        o = np.asarray(state.links[link][0], float)
        for s in np.arange(0.0, SEG - 1e-9, step):
            pts.append(o + R @ np.array([0.0, -s, 0.0]))
    i = N_SEG - 1
    pts.append(np.asarray(state.links[f"c{i:02d}"][0], float) + state.link_rotation(f"c{i:02d}") @ np.array([0, -SEG, 0]))
    return np.array(pts)


def to_carrier(p):
    return np.asarray(p, float) - np.array([XC, Y0, TOP])


def cable_in_link(pts_c, r, k):
    """Every centreline point over link k's middle 24 mm lies inside the channel, under the crossbar line."""
    y = link_y(k)
    sel = pts_c[np.abs(pts_c[:, 1] - y) <= 12.0 * MM]
    if len(sel) == 0:
        return False
    return bool(np.all(np.abs(sel[:, 0]) <= BI / 2 - r + IN_TOL)
                and np.all(sel[:, 2] <= PLATE_TOP - r + IN_TOL) and np.all(sel[:, 2] >= FLOOR + r - 2.0 * MM))


def latched(car, k):
    return (car.angle_deg(f"bar{k:02d}") <= LATCH_DEG
            and car.joints[f"catch{k:02d}"] >= CATCH_ENGAGED)


def _x_at(pts_c, ys):
    order = np.argsort(pts_c[:, 1])
    return np.interp(ys, pts_c[order, 1], pts_c[order, 0])


def carrier_state(objects):
    car = objects["carrier"]
    pts = {n: to_carrier(cable_points(objects[n], n)) for n in CABLES}
    inside = {k: all(cable_in_link(pts[n], CABLES[n][1], k) for n in CABLES) for k in range(1, N_LINKS + 1)}
    lat = {k: latched(car, k) for k in range(1, N_LINKS + 1)}
    good = sum(1 for k in inside if inside[k] and lat[k])
    all_in = all(inside.values())
    ys = np.arange(-5.0 * MM, -LENGTH + 4.0 * MM, -5.0 * MM)
    ordered = bool(all_in and np.all(_x_at(pts["grey"], ys) - _x_at(pts["orange"], ys) >= ORDER_GAP))
    ends_out = all(pts[n][-1][1] <= -LENGTH - OUT_PAST for n in CABLES)
    return dict(good=good, all_in=all_in, ordered=ordered, all_latched=all(lat.values()), ends_out=ends_out,
                inside=inside, latched=lat)


_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS = None
_QUIET = False


def at_rest(objects):
    """Pose differences between consecutive control steps (contact chatter makes joint speeds unreliable)."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS, _QUIET
    car = objects["carrier"]
    if _RUN["t"] is None:
        return all(objects[n].still(0.01, 0.2) for n in ("carrier", "grey", "orange"))
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS, _POSE_TOKEN, _QUIET = None, None, False
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"], float(_RUN["t"]))
    if token != _POSE_TOKEN:
        pose = (np.concatenate([cable_points(objects[n], n, SEG / 2).ravel() for n in CABLES]),
                np.array([car.angle_deg(f"bar{k:02d}") for k in range(1, N_LINKS + 1)]))
        _QUIET = False
        if _PREVIOUS is not None:
            step, old = _PREVIOUS
            _QUIET = bool(token[0] == step + 1 and np.all(np.abs(pose[0] - old[0]) < STEP_M)
                          and np.all(np.abs(pose[1] - old[1]) < STEP_DEG))
        _PREVIOUS = (token[0], pose)
        _POSE_TOKEN = token
    return _QUIET


def score(objects):
    car, grey, orange = objects["carrier"], objects["grey"], objects["orange"]
    slipped = sticky("cable pulled through its strain relief",
                     abs(grey.joints["c00"]) > SLIP or abs(orange.joints["c00"]) > SLIP)
    st = carrier_state(objects)
    quiet = at_rest(objects)
    clear = released(car) and released(grey) and released(orange)
    settled = clear and quiet
    seconds = held_for("released and still", settled)
    if slipped:
        return 0.0
    if not (settled and seconds + 1e-9 >= STILL_S):
        return 0.0
    total = 0.05 * st["good"] + 0.2 * float(st["ordered"]) + 0.2 * float(st["all_latched"] and st["all_in"] and st["ends_out"])
    return round(total, 6)


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ reference layouts for the scorer tests
def _start_point(mech, link, s):
    st = mech.offline_state(mech.pos, 0.0, {})
    return st.link_to_world(link, (0.0, -(int(link[1:]) * SEG + s), 0.0))


# ------------------------------------------------------------------ scripted demo (start layout and time only)
TIP = 0.013                              # pad tips below the TCP
R_YAW, L_YAW = math.pi / 2, -math.pi / 2  # pads close along x (across the cables / the chain)
GAP16 = -0.008                           # finger target for a 16 mm opening
HIGH = 0.150                             # pad-tip height while carrying a cable over the carrier
PINCH_TIP = {"grey": 0.0015, "orange": 0.0005}   # pad tips this far above the table when pinching
OPEN_GAP = {"grey": GAP16, "orange": -0.015}     # 30 mm around the orange cable (alone on the table by then)
NEAR_AIM_X = {"grey": 0.0, "orange": 0.006}      # the orange cable's 30 deg turn relaxes ~8 mm inward as it settles
NEAR_S = 0.190                           # the left hand takes each cable 190 mm out of its tie-wrap
FAR_S = 0.500                            # the right hand 500 mm out (60 mm from the end, clear of its arm base)
NEAR_TIP = 0.040                         # left pad tips while holding the cable over the channel
RELEASE_TIP = 0.026                      # left pad tips when letting go, 3 mm above the side plates
PRESS_TIP = 0.010                        # pad tips 13 mm below a shut bar's top: about 7 N
PRESS_X = -0.005                         # press 10 mm in from the free end, pads 4 mm clear of the catch
LAY_TIP = 0.008                          # right pad tips when laying the far end, just past the moving-end bracket
DEMO_PACE = 1.0


def tcp(x, y, tip_z):
    return (float(x), float(y), float(TOP + tip_z + TIP))


def demo():
    plan = {"left": [], "right": []}
    st = {"right": None, "left": None}

    def add(arm, dt, label, p, finger, yaw):
        plan[arm].append(Waypoint(dt * DEMO_PACE, tuple(p), finger, label, yaw))

    def right(dt, label, p, finger):
        add("right", dt, label, p, finger, R_YAW)

    def left(dt, label, p, finger):
        add("left", dt, label, p, finger, L_YAW)

    # Cables, grey first. The left hand lifts each cable near the fixed end and holds it over the channel, so the
    # line from the tie-wrap clears the near side plates; the right hand then lifts the free end, carries it over
    # the channel and lowers it in; last the left hand lowers its part and lets go above the side plates.
    for name, mech in (("grey", GREY), ("orange", ORANGE)):
        r = CABLES[name][1]
        x_lay = XC + CABLES[name][0]
        tie_y = Y0 + RELIEF_Y
        gl = _start_point(mech, "c09", NEAR_S - 9 * SEG) + np.array([NEAR_AIM_X[name], 0.0, 0.0])
        gap = OPEN_GAP[name]
        gr = _start_point(mech, "c25", FAR_S - 25 * SEG)
        y_l = tie_y - 0.97 * math.sqrt(NEAR_S ** 2 - (NEAR_TIP - FLOOR) ** 2)
        span = FAR_S - NEAR_S                           # cable between the two hands
        y_high = y_l - 0.99 * math.sqrt(span ** 2 - (HIGH - NEAR_TIP) ** 2)
        y_low = y_l - 0.995 * math.sqrt(span ** 2 - (NEAR_TIP - LAY_TIP) ** 2)
        near_l, near_r = PINCH_TIP[name], PINCH_TIP[name]
        both = [
            # (seconds, label, left (pos, finger), right (pos, finger))
            (2.0, "above the cable", (tcp(gl[0], gl[1], 0.060), gap), (tcp(gr[0], gr[1], 0.060), gap)),
            (1.2, "pads around it", (tcp(gl[0], gl[1], near_l), gap), (tcp(gr[0], gr[1], near_r), gap)),
            (0.6, "pinch", (tcp(gl[0], gl[1], near_l), FINGER_CLOSED), (tcp(gr[0], gr[1], near_r), FINGER_CLOSED)),
            (0.8, "left lifts", (tcp(gl[0], gl[1], NEAR_TIP), FINGER_CLOSED), (tcp(gr[0], gr[1], near_r), FINGER_CLOSED)),
            (1.2, "left over the channel", (tcp(x_lay, y_l, NEAR_TIP), FINGER_CLOSED),
             (tcp(gr[0], gr[1], 0.040), FINGER_CLOSED)),
            (2.5, "right up and over the channel", (tcp(x_lay, y_l, NEAR_TIP), FINGER_CLOSED),
             (tcp(x_lay, y_high, HIGH), FINGER_CLOSED)),
            (2.5, "right lowers it in", (tcp(x_lay, y_l, NEAR_TIP), FINGER_CLOSED),
             (tcp(x_lay, y_low, LAY_TIP), FINGER_CLOSED)),
            (0.6, "right lets go, left lowers", (tcp(x_lay, y_l, RELEASE_TIP), FINGER_CLOSED),
             (tcp(x_lay, y_low, LAY_TIP), GAP16)),
            (0.5, "left lets go", (tcp(x_lay, y_l, RELEASE_TIP), GAP16), (tcp(x_lay, y_low, 0.030), GAP16)),
            (0.8, "both up", (tcp(x_lay, y_l, 0.070), GAP16), (tcp(x_lay, y_low, 0.070), GAP16)),
        ]
        for dt, label, (pl, fl), (pr, fr) in both:
            left(dt, f"{name}: {label}", pl, fl)
            right(dt, f"{name}: {label}", pr, fr)

    # Both arms close crossbars at once: left 6 -> 1, right 12 -> 7 (always 183 mm apart).
    def bar_steps(k):
        y = Y0 + link_y(k)
        return [
            (1.0, f"bar {k}: outside it", tcp(XC + 0.034, y, 0.075)),
            (0.6, f"bar {k}: down beside it", tcp(XC + 0.034, y, 0.040)),
            (1.0, f"bar {k}: push it over", tcp(XC + 0.008, y, 0.042)),
            (0.6, f"bar {k}: follow it down", tcp(XC - 0.003, y, 0.034)),
            (0.5, f"bar {k}: onto the catch", tcp(XC - 0.006, y, 0.026)),
            (0.3, f"bar {k}: let it settle", tcp(XC - 0.006, y, 0.026)),
            (0.8, f"bar {k}: press it shut", tcp(XC + PRESS_X, y, PRESS_TIP)),
            (0.6, f"bar {k}: hold", tcp(XC + PRESS_X, y, PRESS_TIP)),
            (0.7, f"bar {k}: up", tcp(XC + PRESS_X, y, 0.075)),
        ]
    for kl, kr in zip(range(6, 0, -1), range(12, 6, -1)):
        for (dl, ll, pl), (dr, lr, pr) in zip(bar_steps(kl), bar_steps(kr)):
            left(dl, ll, pl, FINGER_CLOSED)
            right(dr, lr, pr, FINGER_CLOSED)
    left(1.2, "left: clear", tcp(0.130, 0.250, 0.120), FINGER_CLOSED)
    right(1.2, "right: clear", tcp(0.130, -0.250, 0.120), FINGER_CLOSED)
    left(4.0, "stillness tail", tcp(0.130, 0.250, 0.120), FINGER_CLOSED)
    right(4.0, "stillness tail", tcp(0.130, -0.250, 0.120), FINGER_CLOSED)
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
