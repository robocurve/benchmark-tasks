"""A patient's ear temperature at the bedside. An ear-examination manikin (life-size adult head and shoulders,
soft skin-tone resin) stands on the table facing the robot's left, so its left ear faces the robot; the left ear
canal opens about 255 mm above the table at the bottom of the concha bowl. A tympanic ear thermometer
(152 x 44 x 33 mm, 146 g, a clear probe cover already fitted) lies display up on the table to the robot's right,
probe pointing forward. Its MEASURE button sits on the display face just below the probe. The reading follows
the thermometer's directions for use: probe seated snugly in the canal, MEASURE pressed and released, then the
probe kept steady until the long beep; if the probe moves, the thermometer shows POS and gives no reading.
Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import ball, box, cyl
from yam_joints import Link, Slide
from yam_mesh import Mesh, MeshMechanism, MeshPart, revolve, triangulate_polygon
from yam_arena import demo as _motion
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.ik import matrix_to_quat_xyzw
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


# ------------------------------------------------------------------ geometry helpers
def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _axis_angle(axis, angle):
    x, y, z = _unit(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array(((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
                     (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
                     (z * x * C - y * s, z * y * C + x * s, c + z * z * C)))


def _closed(points, faces):
    """Orient a closed triangle mesh outward (positive signed volume)."""
    P, F = np.asarray(points, float), np.asarray(faces, int)
    tri = P[F] - P.mean(0)
    if np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        F = F[:, ::-1]
    return P, F


def _ring_mesh(rings, cap_start=True, cap_end=True):
    """Closed mesh through rings of equal length (each a list of 3-D points), with fan caps at both ends."""
    n = len(rings[0])
    pts, rows = [], []
    if cap_start:
        rows.append([0] * n)
        pts.append(np.mean(rings[0], axis=0))
    for ring in rings:
        rows.append(list(range(len(pts), len(pts) + n)))
        pts.extend(ring)
    if cap_end:
        rows.append([len(pts)] * n)
        pts.append(np.mean(rings[-1], axis=0))
    faces = []
    for r0, r1 in zip(rows, rows[1:]):
        for i in range(n):
            k = (i + 1) % n
            for tri in ((r0[i], r0[k], r1[k]), (r0[i], r1[k], r1[i])):
                if len(set(tri)) == 3:
                    faces.append(tri)
    return _closed(pts, faces)


def _rounded_rect(cx, half_w, half_t, r, n_corner=6):
    """Rounded rectangle in the (y, z) plane at x = cx: half width (y), half thickness (z), corner radius."""
    out = []
    for (sy, sz, a0) in ((1, 1, 0.0), (-1, 1, 90.0), (-1, -1, 180.0), (1, -1, 270.0)):
        oy, oz = sy * (half_w - r), sz * (half_t - r)
        for k in range(n_corner + 1):
            a = math.radians(a0 + 90.0 * k / n_corner)
            out.append((cx, oy + r * math.cos(a), oz + r * math.sin(a)))
    return out


def _tube_mesh(name, path, radius, color, n=12, collision="exact"):
    """A closed tube of circular section along a polyline (caps at both ends)."""
    path = [np.asarray(p, float) for p in path]
    rings = []
    ref = np.array([0.0, 0.0, 1.0])
    for i, p in enumerate(path):
        t = _unit(path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)])
        a = ref - np.dot(ref, t) * t
        if np.linalg.norm(a) < 1e-6:
            a = np.array([1.0, 0, 0]) - t[0] * t
        a = _unit(a)
        b = np.cross(t, a)
        rings.append([p + radius * (math.cos(2 * math.pi * k / n) * a + math.sin(2 * math.pi * k / n) * b)
                      for k in range(n)])
    P, F = _ring_mesh(rings)
    return Mesh(name, P, F, collision=collision, color=color)


def _ellipsoid_mesh(name, center, axes, R, color, n=16, m=10, collision="exact", proxy=()):
    pts = [np.array([0, 0, -1.0])]
    for i in range(1, m):
        th = math.pi * i / m - math.pi / 2
        for k in range(n):
            ph = 2 * math.pi * k / n
            pts.append(np.array([math.cos(th) * math.cos(ph), math.cos(th) * math.sin(ph), math.sin(th)]))
    pts.append(np.array([0, 0, 1.0]))
    faces = []
    for k in range(n):
        faces.append((0, 1 + (k + 1) % n, 1 + k))
    for i in range(m - 2):
        for k in range(n):
            a, b = 1 + i * n + k, 1 + i * n + (k + 1) % n
            c, d = a + n, b + n
            faces += [(a, b, d), (a, d, c)]
    top = len(pts) - 1
    for k in range(n):
        faces.append((top, 1 + (m - 2) * n + k, 1 + (m - 2) * n + (k + 1) % n))
    P = np.array([np.asarray(center) + np.asarray(R) @ (np.asarray(axes) * p) for p in pts])
    P, F = _closed(P, faces)
    return Mesh(name, P, F, collision=collision, color=color, proxy=proxy)


# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SKIN = (0.93, 0.76, 0.58)                         # MW12 light skin tone (11391-000)
SKIN_DARK = (0.84, 0.64, 0.48)
SHIRT = (0.22, 0.22, 0.24)                        # the bust's charcoal shoulders
LID = (0.62, 0.45, 0.36)
LIP = (0.72, 0.42, 0.38)

# Manikin: Kyoto Kagaku MW12 EAR examination simulator II, 420 x 220 x 390 mm, 1.75 kg, head and shoulders.
# It faces +y (the robot's left), so its left ear faces the robot (-x). Head frame = world axes:
# x = the patient's right (medial from the left ear), y = anterior (the face), z = superior.
EAR_Z = 0.255                                     # left canal entrance above the table top
CANAL_E = np.array([0.195, 0.0, TOP + EAR_Z])     # canal entrance centre (world)
CANAL_D = _unit([math.cos(math.radians(10)) * math.cos(math.radians(15)),
                 math.cos(math.radians(10)) * math.sin(math.radians(15)), math.sin(math.radians(10))])
CANAL_HALF = (0.0045, 0.00375)                    # entrance half axes: vertical (9 mm), front-back (7.5 mm)
CANAL_DEPTH = 0.025                               # entrance to eardrum
HEAD_A, HEAD_B, HEAD_C = 0.076, 0.096, 0.115      # head half breadth, half length, half height
_E_HEAD = np.array([-0.0735, -0.006, -0.012])     # canal entrance in the head frame (from the head centre)
HEAD_O = CANAL_E - _E_HEAD                        # head centre (world)
MANIKIN_POS = (float(HEAD_O[0]), -0.010, TOP)     # part origin: base centre on the table
_MO = np.array(MANIKIN_POS)


def _head_r(d):
    x, y, z = d
    b = HEAD_B * (1 - 0.12 * max(0.0, -z) * max(0.0, y))      # slightly narrower jaw
    return 1.0 / math.sqrt((x / HEAD_A) ** 2 + (y / b) ** 2 + (z / HEAD_C) ** 2)


def _head_mesh(n=48, rings=26, theta0=math.radians(22)):
    """Skull as a star-shaped closed surface about the head centre. Its pole at the left ear is pushed in to
    form the concha bowl and a blind ear canal (elliptical, 9 x 7.5 mm, 25 mm deep, along CANAL_D)."""
    e, f1, f2 = np.array([-1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
    phis = np.arange(n) * 2 * math.pi / n
    a = CANAL_D
    g1 = _unit(f1 - np.dot(f1, a) * a)
    g2 = np.cross(g1, a)
    if np.dot(g2, f2) < 0:
        g2 = -g2
    # pole-frame phi runs f1 -> f2 about e; the canal rings run g1 -> g2 the same way round
    rings_pts = []
    for depth in (CANAL_DEPTH - 0.003, 0.016, 0.008, 0.0):
        rings_pts.append([_E_HEAD + depth * a + CANAL_HALF[1] * math.cos(p) * g1 + CANAL_HALF[0] * math.sin(p) * g2
                          for p in phis])
    rings_pts.append([_E_HEAD - 0.0025 * a + 0.0105 * (math.cos(p) * g1 + math.sin(p) * g2) for p in phis])
    for th in np.linspace(theta0, math.pi, rings)[:-1]:
        ring = []
        for p in phis:
            d = math.cos(th) * e + math.sin(th) * (math.cos(p) * f1 + math.sin(p) * f2)
            ring.append(_head_r(d) * d)
        rings_pts.append(ring)
    pts, rows = [_E_HEAD + CANAL_DEPTH * a], [[0] * n]
    for ring in rings_pts:
        rows.append(list(range(len(pts), len(pts) + n)))
        pts.extend(ring)
    rows.append([len(pts)] * n)
    pts.append(_head_r(-e) * -e)
    faces = []
    for r0, r1 in zip(rows, rows[1:]):
        for i in range(n):
            k = (i + 1) % n
            for tri in ((r0[i], r0[k], r1[k]), (r0[i], r1[k], r1[i])):
                if len(set(tri)) == 3:
                    faces.append(tri)
    P = np.array(pts) + (HEAD_O - _MO)
    P, F = _closed(P, faces)
    proxy = tuple(ball(tuple(HEAD_O - _MO + c), r) for c, r in (
        ((0.012, 0.0, 0.020), 0.060), ((0.012, 0.030, -0.030), 0.050), ((0.012, -0.030, -0.030), 0.050)))
    return Mesh("skull", P, F, collision="exact", color=SKIN, proxy=proxy)


PINNA_BACK, PINNA_UP = 0.016, 0.006              # helix centre behind and above the canal
PINNA_RA, PINNA_RV = 0.017, 0.031                 # helix path half width and half height (ear 62 x 34 mm)
CONCHA_R = 0.019                                  # the web stops 19 mm from the canal (the concha cavity)
PINNA_OUT = (0.0010, 0.0030)                      # helix centre line 1-4 mm proud of the skull (low relief)
HELIX_R = 0.0025


def _yz_area(outline):
    p = np.asarray(outline, float)
    return float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]) / 2)


def _pinna(side):
    """Auricle: helix rim (a tube on a C-shaped path open toward the face), a crescent web and the lobule.
    side -1 = left ear (toward -x), +1 = right ear. Pinna frame: lateral, anterior (+y), up (+z)."""
    lat = np.array([float(side), 0.0, 0.0])
    tag = "l" if side < 0 else "r"
    # root on the skull's side surface, level with the canal; the canal opens at the bottom of the concha
    root = HEAD_O + np.array([side * (HEAD_A - 0.001), _E_HEAD[1], _E_HEAD[2]])
    centre = root + np.array([0.0, -PINNA_BACK, PINNA_UP])
    ra, rv = PINNA_RA, PINNA_RV                # half width (front-back) and half height of the helix path
    path = []
    for a in np.linspace(math.radians(70), math.radians(285), 24):
        out = PINNA_OUT[0] + PINNA_OUT[1] * (1 - math.cos(a)) / 2   # stands out most at the back of the ear
        path.append(centre + out * lat + np.array([0.0, ra * math.cos(a), rv * math.sin(a)]))
    helix = _tube_mesh(f"helix_{tag}", [p - _MO for p in path], HELIX_R, SKIN_DARK)
    # web: crescent between the helix path and an arc round the concha, 3 mm thick, in the y-z plane
    cy, cz = PINNA_BACK, -PINNA_UP                    # the canal, seen from the helix centre
    outer = [(ra * math.cos(a) * 0.92, rv * math.sin(a) * 0.92)
             for a in np.linspace(math.radians(80), math.radians(280), 18)]
    inner = [(cy + CONCHA_R * math.cos(a), cz + CONCHA_R * math.sin(a))
             for a in np.linspace(math.radians(245), math.radians(115), 12)]
    outline = np.array(outer + inner)
    if _yz_area(outline) < 0:
        outline = outline[::-1]
    x_in, x_out = centre[0] + side * 0.0005, centre[0] + side * 0.0025
    n = len(outline)
    P = np.array([(x, centre[1] + y, centre[2] + z) for x in (x_in, x_out) for (y, z) in outline])
    F = []
    for i in range(n):
        k = (i + 1) % n
        F += [(i, k, n + k), (i, n + k, n + i)]
    for a, b, c in triangulate_polygon(outline):
        F += [(c, b, a), (n + a, n + b, n + c)]
    P, F = _closed(P - _MO, F)
    web = Mesh(f"web_{tag}", P, F, collision="exact", color=SKIN_DARK)
    lobe = _ellipsoid_mesh(f"lobe_{tag}", centre + np.array([0.0, 0.004, -rv + 0.002]) - _MO
                           + 0.0005 * lat, (0.0025, 0.008, 0.009), np.eye(3), SKIN_DARK, n=12, m=8)
    return (helix, web, lobe)


def _body_meshes():
    """Shoulders, neck and nose (part frame: origin on the table under the base centre)."""
    sections = []
    for z, hw, hd, r in ((0.000, 0.210, 0.110, 0.040), (0.070, 0.210, 0.108, 0.050), (0.100, 0.195, 0.100, 0.060),
                         (0.120, 0.150, 0.088, 0.060), (0.138, 0.085, 0.070, 0.055)):
        ring = _rounded_rect(0.0, hw, hd, r)
        sections.append([np.array([y, z_, z]) for (_, y, z_) in ring])   # (x, y) = (half width, half depth)
    P, F = _ring_mesh(sections)
    shoulders = Mesh("shoulders", P, F, collision="exact", color=SHIRT,
                     proxy=(box((0.0, 0.0, 0.050), (0.40, 0.20, 0.100)),))
    neck_c = HEAD_O - _MO + np.array([0.0, -0.012, 0.0])
    neck = revolve([(0.0, 0.120), (0.054, 0.120), (0.050, neck_c[2] - 0.080), (0.0, neck_c[2] - 0.080)], 32,
                   name="neck", collision="exact", color=SKIN, proxy=())
    NP = np.asarray(neck.points) + np.array([neck_c[0], neck_c[1], 0.0])
    neck = Mesh("neck", NP, np.asarray(neck.faces), collision="exact", color=SKIN,
                proxy=(cyl((neck_c[0], neck_c[1], (0.12 + neck_c[2] - 0.08) / 2), 0.045, neck_c[2] - 0.08 - 0.12),))
    # nose: a lofted wedge on the face's midline
    face_y = HEAD_O[1] - _MO[1] + HEAD_B * 0.985
    nz = HEAD_O[2] - _MO[2]
    rings = []
    for dz, w, d in ((-0.052, 0.017, 0.024), (-0.040, 0.013, 0.022), (-0.012, 0.006, 0.012), (0.004, 0.004, 0.004)):
        rings.append([np.array([HEAD_O[0] - _MO[0] + s, face_y - 0.016 + t, nz + dz])
                      for s, t in ((-w, 0.0), (0.0, d), (w, 0.0), (0.0, -0.006))])
    P, F = _ring_mesh(rings)
    nose = Mesh("nose", P, F, collision="exact", color=SKIN_DARK)
    # brow ridge and chin as flattened ellipsoids (visual landmarks of a face)
    brow = _ellipsoid_mesh("brow", HEAD_O - _MO + np.array([0.0, HEAD_B * 0.86, 0.020]), (0.055, 0.016, 0.010),
                           np.eye(3), SKIN, n=16, m=8)
    chin = _ellipsoid_mesh("chin", HEAD_O - _MO + np.array([0.0, HEAD_B * 0.62, -0.098]), (0.030, 0.022, 0.018),
                           np.eye(3), SKIN, n=16, m=8)
    # closed eyelids and lips (the MW12's moulded face)
    face = []
    for name, dx, dz, axes, col in (("eye_l", -0.032, 0.002, (0.014, 0.004, 0.005), LID),
                                    ("eye_r", 0.032, 0.002, (0.014, 0.004, 0.005), LID),
                                    ("lips", 0.0, -0.068, (0.020, 0.005, 0.006), LIP)):
        y = HEAD_O[1] - _MO[1] + _head_r(_unit([dx, 0.97, dz / HEAD_C])) * 0.97 - 0.002
        face.append(_ellipsoid_mesh(name, np.array([HEAD_O[0] - _MO[0] + dx, y, HEAD_O[2] - _MO[2] + dz]), axes,
                                    np.eye(3), col, n=12, m=6))
    return (shoulders, neck, nose, brow, chin) + tuple(face)


MANIKIN = MeshPart("manikin", MANIKIN_POS, (), color=SKIN, friction=(0.6, 0.5), contact_offset=0.001,
                   meshes=(_head_mesh(),) + _pinna(-1) + _pinna(+1) + _body_meshes())

# Thermometer: Braun ThermoScan PRO 6000 (Welch Allyn / Hillrom), 152 x 44 x 33 mm, 100 g + two AA cells.
# Part frame T: x along the body from the base end toward the probe, y across the 44 mm width, z out of the
# display face; origin on the body axis at the base end. The probe is bent 20 deg toward the back (-z).
BODY_L, BODY_W, BODY_T = 0.137, 0.044, 0.033
FLAT_L = 0.127                                     # constant section, then the head tapers to 30 x 24 mm
PROBE_BEND = math.radians(20.0)
P_T = np.array([math.cos(PROBE_BEND), 0.0, -math.sin(PROBE_BEND)])    # probe axis in T
PROBE_BASE_T = np.array([BODY_L, 0.0, 0.0])
COLLAR_L, CONE_L = 0.003, 0.0135                   # blue collar, then the covered cone
TIP_T = PROBE_BASE_T + (COLLAR_L + CONE_L) * P_T   # covered probe tip (front of the cover's film window)
TIP_R, CONE_R = 0.0030, 0.0060                     # cover cone radius at the tip and at its base
BUTTON_X, BUTTON_R = 0.118, 0.0090                 # MEASURE button centre and radius (18 mm disc)
BUTTON_PROUD, BUTTON_TRAVEL = 0.0015, 0.0010
BUTTON_K, BUTTON_REST = 300.0, 0.0050              # 1.5 N preload at rest, 1.8 N bottomed
GRIP_X = 0.0155                                    # the hand pinches the 44 mm sides 9-22 mm from the base
GRIP_ZONE = 0.036                                  # the base-end grip zone (offline grip shape)
WHITE = (0.95, 0.95, 0.95)
PROBE_BLUE = (0.62, 0.80, 0.90)
COVER = (0.86, 0.90, 0.92)
DISPLAY = (0.20, 0.32, 0.28)
RING_GREEN = (0.20, 0.75, 0.30)
BUTTON_BLUE = (0.55, 0.66, 0.78)
GREY_BTN = (0.80, 0.80, 0.80)


def _probe_frame():
    z = P_T
    x = np.array([0.0, 1.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def _thermo_meshes():
    flat = [_rounded_rect(x, BODY_W / 2, BODY_T / 2, 0.010) for x in (0.0, FLAT_L)]
    P, F = _ring_mesh([[np.array(p) for p in r] for r in flat])
    lower = Mesh("body", P, F, collision="convex", color=WHITE,
                 proxy=(box((GRIP_ZONE / 2, 0.0, 0.0), (GRIP_ZONE, BODY_W, BODY_T)),       # where the hand pinches
                        box(((GRIP_ZONE + FLAT_L) / 2, 0.0, 0.0), (FLAT_L - GRIP_ZONE, BODY_W, BODY_T))))
    head = [_rounded_rect(FLAT_L, BODY_W / 2, BODY_T / 2, 0.010), _rounded_rect(FLAT_L + 0.006, 0.018, 0.014, 0.010),
            _rounded_rect(BODY_L, 0.011, 0.010, 0.008)]
    P, F = _ring_mesh([[np.array(p) for p in r] for r in head])
    head = Mesh("head", P, F, collision="convex", color=WHITE,
                proxy=(box(((FLAT_L + BODY_L) / 2, 0.0, 0.0), (BODY_L - FLAT_L, 0.030, 0.024)),))
    R = _probe_frame()
    prof = [(0.0, -0.002), (0.0080, -0.002), (0.0080, COLLAR_L), (CONE_R, COLLAR_L),
            (TIP_R, COLLAR_L + CONE_L), (0.0, COLLAR_L + CONE_L)]
    collar = revolve(prof[:4] + [(0.0, COLLAR_L)], 32, name="collar", collision="convex", color=PROBE_BLUE)
    cone = revolve([(0.0, COLLAR_L), (CONE_R, COLLAR_L), (TIP_R, COLLAR_L + CONE_L), (0.0, COLLAR_L + CONE_L)], 32,
                   name="cover", collision="convex", color=COVER)
    out = [lower, head]
    for m in (collar, cone):
        P = np.asarray(m.points) @ R.T + PROBE_BASE_T
        mid = PROBE_BASE_T + (COLLAR_L + CONE_L / 2) * P_T
        out.append(Mesh(m.name, P, np.asarray(m.faces), collision="convex", color=m.color,
                        proxy=(ball(tuple(mid), 0.004),) if m.name == "cover" else ()))
    return tuple(out)


_FACE = BODY_T / 2
THERMO_SHAPES = [
    box((0.093, 0.0, _FACE + 0.00015), (0.028, 0.026, 0.0003), DISPLAY),           # display window
    box((0.070, -0.009, _FACE + 0.00025), (0.008, 0.012, 0.0005), GREY_BTN),       # C/F
    box((0.070, 0.009, _FACE + 0.00025), (0.008, 0.012, 0.0005), GREY_BTN),        # Mem
    cyl((0.047, 0.0, _FACE + 0.00025), 0.007, 0.0005, color=GREY_BTN),             # timer button
    cyl((BUTTON_X, 0.0, _FACE + 0.00015), 0.01025, 0.0003, color=RING_GREEN),      # ready ring round MEASURE
]
THERMO_X0, THERMO_Y0 = 0.090, -0.200               # base end, lying display up, probe toward +x
THERMO_START = (THERMO_X0, THERMO_Y0, TOP + BODY_T / 2)
THERMO_MASS = 0.146
THERMOMETER = MeshMechanism(
    "thermometer", THERMO_START, THERMO_SHAPES,
    [Link("button", [cyl((BUTTON_X, 0.0, _FACE - 0.001 + (BUTTON_PROUD + 0.001) / 2), BUTTON_R,
                         BUTTON_PROUD + 0.001, color=BUTTON_BLUE)],
          Slide("z", (BUTTON_X, 0.0, _FACE), -BUTTON_TRAVEL, 0.0, damping=0.5, spring=BUTTON_K, rest=BUTTON_REST),
          mass=0.002, color=BUTTON_BLUE)],
    mass=THERMO_MASS - 0.002, color=WHITE, friction=(0.6, 0.5), contact_offset=0.001,
    grip=len(THERMO_SHAPES) + 1, meshes={"base": _thermo_meshes()})   # the body's proxy box
OBJECTS = [MANIKIN, THERMOMETER]
PRESSED_PARTS = ("thermometer",)

# ------------------------------------------------------------------ the seated pose (for tests and the demo)
SEAT_DEPTH = 0.0035                                # cover tip inside the entrance plane when snug (cone vs 7.5 mm)
_W = _unit(np.cross([0.0, 0.0, 1.0], CANAL_D))     # thermometer width axis at the ear (horizontal)
_FT = np.column_stack([P_T, [0.0, 1.0, 0.0], np.cross(P_T, [0.0, 1.0, 0.0])])
_FW = np.column_stack([CANAL_D, _W, np.cross(CANAL_D, _W)])
R_SEAT = _FW @ _FT.T                               # T -> world with the probe along the canal, display up


def seated_origin(depth=SEAT_DEPTH, R=R_SEAT):
    return CANAL_E + depth * (R @ P_T) - R @ TIP_T


GOAL_SEAT = seated_origin()
SEAT_QUAT = tuple(float(v) for v in matrix_to_quat_xyzw(R_SEAT))
REST_POS = np.array(THERMO_START)                  # laid back where it lay

# ------------------------------------------------------------------ scoring
SEATED_DEPTH = 0.0020        # cover tip at least 2 mm past the entrance plane
SEAT_LATERAL = 0.0030        # tip within 3 mm of the canal axis
SEAT_ANGLE = 20.0            # probe axis within 20 deg of the canal axis
DEEP = 0.012                 # sticky zero: tip 12 mm in (the MW12's painful-insertion zone, past the cartilage)
PRESS_Q = -0.0007            # MEASURE counts as pressed at 0.7 mm of travel ...
RELEASE_Q = -0.0003          # ... and as released below 0.3 mm
MEASURE_S = 3.0              # measurement time after release (directions for use: 2-3 s)
RELEASE_STEPS = 2            # released for two control steps (0.1 s) starts the measurement
LOG_PHASES = False           # print measuring-phase changes to the console (debug copies)
STEADY_MM, STEADY_DEG = 0.0015, 3.0
HOLD_CLEAR = 0.0             # held: a gripper centre inside the thermometer's box, pads at least 10 % closed
DROP_Z = TOP - 0.020
TEST_STICKY = ()             # offline tests: sticky keys that already fired
TEST_PHASE = None            # offline tests: the thermometer's measuring phase


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def probe_state(th):
    """(tip depth past the entrance plane, lateral offset from the canal axis, angle to the canal axis)."""
    tip = th.to_world(TIP_T)
    axis = th.rotation() @ P_T
    rel = tip - CANAL_E
    depth = float(np.dot(rel, CANAL_D))
    lateral = float(np.linalg.norm(rel - depth * CANAL_D))
    angle = math.degrees(math.acos(float(np.clip(np.dot(axis, CANAL_D), -1.0, 1.0))))
    return depth, lateral, angle, tip, axis


def is_seated(th):
    depth, lateral, angle, _, _ = probe_state(th)
    return depth >= SEATED_DEPTH and lateral <= SEAT_LATERAL and angle <= SEAT_ANGLE


def is_held(th):
    hands = grippers()
    if hands is None:
        return True
    low, high = np.asarray(th.low) - HOLD_CLEAR, np.asarray(th.high) + HOLD_CLEAR
    return any(np.all(tcp >= low) and np.all(tcp <= high) and opening <= 0.9 for tcp, opening in hands.values())


_MEAS = {"episode": None, "token": None, "phase": "ready", "ref": None, "t0": None}


def measuring_phase(th, seated, held):
    """The thermometer's directions for use as a state machine, advanced once per control step:
    ready -> (MEASURE pressed while seated and held) armed -> (released) measuring -> (MEASURE_S steady) done.
    Pressed while not seated: 'air' (a reading of room air; the cover is used up). Moved, unseated or let go while
    armed or measuring: 'pos' (position error; needs a new cover, so no reading this run)."""
    if _RUN["t"] is None:
        return TEST_PHASE or "ready"
    if _MEAS["episode"] != _RUN["episode"]:
        _MEAS.update(episode=_RUN["episode"], token=None, phase="ready", ref=None, t0=None, up=0)
    token = (_RUN["step"], float(_RUN["t"]))
    if token == _MEAS["token"]:
        return _MEAS["phase"]
    _MEAS["token"] = token
    q = float(th.joints["button"])
    phase = _MEAS["phase"]
    if phase == "ready" and q <= PRESS_Q:
        phase = "armed" if (seated and held) else "air"
    elif phase == "armed":
        _MEAS["up"] = _MEAS.get("up", 0) + 1 if q >= RELEASE_Q else 0
        if not (seated and held):
            phase = "pos"
        elif _MEAS["up"] >= RELEASE_STEPS:            # released (debounced, like the switch electronics)
            _, _, _, tip, axis = probe_state(th)
            _MEAS.update(ref=(tip, axis), t0=float(_RUN["t"]))
            phase = "measuring"
    elif phase == "measuring":
        _, _, _, tip, axis = probe_state(th)
        tip0, axis0 = _MEAS["ref"]
        moved = (np.linalg.norm(tip - tip0) > STEADY_MM
                 or math.degrees(math.acos(float(np.clip(np.dot(axis, axis0), -1, 1)))) > STEADY_DEG)
        if not (seated and held) or moved:
            phase = "pos"
        elif float(_RUN["t"]) - _MEAS["t0"] >= MEASURE_S:
            phase = "done"
    if LOG_PHASES and phase != _MEAS["phase"]:
        depth, lateral, angle, tip, _ = probe_state(th)
        print(f"[ear] t={float(_RUN['t']):.2f} {_MEAS['phase']} -> {phase}: q={q*1000:.2f} mm seated={seated} "
              f"held={held} depth={depth*1000:.1f} lat={lateral*1000:.1f} ang={angle:.1f}", flush=True)
    _MEAS["phase"] = phase
    return phase


def on_table(th):
    depth = probe_state(th)[0]
    return bool(abs(float(th.low[2]) - TOP) <= 0.005 and depth < -0.020)


def score(objects):
    th = objects["thermometer"]
    depth = probe_state(th)[0]
    too_deep = sticky("probe past the cartilage", depth > DEEP)
    dropped = sticky("thermometer off the table", th.low[2] < DROP_Z)
    seated = is_seated(th)
    held = is_held(th)
    seated_s = held_for("seated", seated)
    seated_1s = ever("seated 1 s", seated_s + 1e-9 >= 1.0)
    phase = measuring_phase(th, seated, held)
    done = phase == "done"
    laid = done and on_table(th) and released(th) and th.resting()
    laid_s = held_for("laid down", laid)
    if too_deep or dropped:
        return 0.0
    if done:
        return 1.0 if laid_s + 1e-9 >= STILL_S else 0.75
    if phase in ("armed", "measuring", "pos"):
        return 0.5
    return 0.25 if seated_1s else 0.0


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo
# Tilted-wrist waypoints: a float yaw (for the offline checker's coarse replay) carrying the full grip
# rotation for the arm's IK, the same pattern the tilted-loop, hoist and oximeter demos used.
class GripOrientation(float):
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_ear_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


_grip_rotation._ear_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation


def _tool(approach, closing):
    a = _unit(approach)
    c = _unit(closing - np.dot(closing, a) * a)
    return np.column_stack([c, np.cross(a, c), a])


# Hold: pads on the 44 mm sides at GRIP_X, the gripper coming from behind the base end over the display,
# tilted 30 deg toward the probe; the TCP 10 mm short of the body axis along the approach.
A_T = _unit([math.sin(math.radians(30)), 0.0, -math.cos(math.radians(30))])
GRIP_T = np.array([GRIP_X, 0.0, 0.0]) - 0.010 * A_T
HOLD_FINGER = -(BODY_W - 0.010) / 2                # 5 mm squeeze per pad (about 25 N each)
R_HOLD_T = _tool(A_T, np.array([0.0, 1.0, 0.0]))


def hand_pose(origin, R):
    """TCP position and rotation for the holding hand when the thermometer is at (origin, R)."""
    return np.asarray(origin) + R @ GRIP_T, R @ R_HOLD_T


# Press: closed fingertips on MEASURE, approach tilted 20 deg off the button normal about the body axis and
# 20 deg about the width axis (the reachable press at this height, from the IK scan).
_U = R_SEAT @ np.array([1.0, 0.0, 0.0])
_DISP = R_SEAT @ np.array([0.0, 0.0, 1.0])
A_PRESS = _axis_angle(_U, math.radians(-20)) @ _axis_angle(_W, math.radians(20)) @ (-_DISP)
R_PRESS = _tool(A_PRESS, _W)
BUTTON_TOP_T = np.array([BUTTON_X, 0.0, _FACE + BUTTON_PROUD])
LEFT_PARK = (0.120, 0.150, 0.170)                  # left hand waits straight down, clear of the head
PRESS_WAIT = 0.035                                 # pre-press: fingertips 22 mm off the button
PRESS_OVER = 0.006                                 # commanded past the bottomed button
DEMO_PACE = 1.0
SEAT_OVER = 0.004                                  # commanded past the snug depth (the hand rides compliant)
HAND_DROP = np.array([-0.0020, 0.0005, 0.0102])    # the held thermometer rides 10 mm low at the ear (GPU run d1)


def demo():
    G = GripOrientation
    plan = {"left": [], "right": []}
    st = {}

    def add(dt, label, right=None, left=None, rf=None, lf=None, rr=None, lr=None):
        for key, val in (("r", right), ("l", left)):
            if val is not None:
                st[key] = np.asarray(val, float)
        for key, val in (("rf", rf), ("lf", lf), ("rr", rr), ("lr", lr)):
            if val is not None:
                st[key] = val
        plan["right"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in st["r"]), st["rf"], label, G(st["rr"])))
        plan["left"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in st["l"]), st["lf"], label, G(st["lr"])))

    R0 = np.eye(3)
    o0 = np.array(THERMO_START)
    grasp, r_grasp = hand_pose(o0, R0)
    seat_o = seated_origin(SEAT_DEPTH + SEAT_OVER) + HAND_DROP
    seat_tcp, r_seat = hand_pose(seat_o, R_SEAT)
    back = 0.030 * CANAL_D
    lift_tcp, _ = hand_pose(o0 + np.array([0.0, 0.0, 0.060]), R0)
    btn = seated_origin(SEAT_DEPTH) + R_SEAT @ BUTTON_TOP_T          # where the seated button really is
    press_touch = btn - 0.013 * A_PRESS
    press_in = btn + (BUTTON_TRAVEL + PRESS_OVER) * A_PRESS - 0.013 * A_PRESS
    press_wait = btn - PRESS_WAIT * A_PRESS
    left_park = np.array(LEFT_PARK) + [0.0, 0.0, TOP]
    r_left_park = _original_grip_rotation(0.0)

    st.update(r=grasp + np.array([0.0, 0.0, 0.070]), l=left_park, rf=FINGER_OPEN, lf=FINGER_CLOSED,
              rr=r_grasp, lr=r_left_park)
    add(2.5, "approach")
    add(1.5, "right: down to the thermometer's base end", right=grasp)
    add(0.8, "right: pinch the sides", rf=HOLD_FINGER)
    add(1.2, "right: lift", right=lift_tcp)
    add(2.5, "right: turn the probe to the canal, 30 mm out", right=seat_tcp - back, rr=r_seat,
        left=press_wait, lr=R_PRESS)
    add(1.0, "settle in line")
    add(1.5, "seat the covered probe snugly", right=seat_tcp)
    add(1.0, "hold seated")
    add(1.0, "left: fingertips to MEASURE", left=press_touch)
    add(0.6, "press MEASURE", left=press_in)
    add(0.3, "hold the press")
    add(0.6, "release MEASURE", left=press_wait)
    add(4.0, "keep the probe still until the long beep")
    add(1.0, "left: clear", left=press_wait)
    add(1.5, "withdraw along the canal", right=seat_tcp - back)
    add(2.5, "lower toward the table", right=lift_tcp, rr=r_grasp)
    add(1.5, "lay it down", right=grasp + np.array([0.0, 0.0, 0.002]))
    add(0.8, "let go", rf=FINGER_OPEN)
    add(1.2, "rise clear", right=grasp + np.array([0.010, 0.0, 0.080]))
    add(3.5, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
