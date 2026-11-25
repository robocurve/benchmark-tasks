"""A bedroom floor (the table top) with a two-drawer pine chest of drawers standing against the wall, and an AA
battery that has rolled under it.

The chest is an IKEA KRONOREN chest of 2 drawers at real size (76 x 48 x 55 cm), loaded with clothes: four slim
pine legs, bottom rails on all four sides whose undersides stand 27 mm off the floor, two shut drawers and a top
that overhangs the frame. A Duracell AA battery lies on its side under the chest, about 11 cm in from the front
rail, out of reach of a hand and out of sight. A white plastic tubular coat hanger lies flat on the floor in front
of the chest, its hook just under the front rail. The task: get the battery out from under the chest with the
hanger so that it lies on the floor in front of the chest, then put the hanger down and let go. 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts
import yam_mesh
from yam_parts import Part, box, cyl, ball
from yam_mesh import Mesh, MeshPart
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


# ------------------------------------------------------------------ layout (metres; x forward, y left, z up)
TOP = TABLE_TOP[SCENE]           # the table top stands for a lacquered wooden bedroom floor
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- chest of drawers: IKEA KRONOREN chest of 2 drawers, untreated solid pine, 76 x 48 x 55 cm ----------------
# Chest frame: origin on the floor at the front face of the front legs, on the chest's centre line; u (= world x)
# runs back toward the wall. Maker's figures: 76 x 48 x 55 cm overall, legs spanning 64 x 39 cm, free height under
# the furniture 2.7 cm, drawers 60 x 38 cm inside. Sections of legs and rails, drawer heights: from the maker's
# measurement drawing (estimates).
CHEST_X = 0.355                  # front face of the front legs and of the front bottom rail (clear of the arms' ready pose)
CHEST_W, CHEST_D, CHEST_H = 0.76, 0.48, 0.55
LEG_SPAN_Y, LEG_SPAN_X = 0.64, 0.39
LEG_X, LEG_Y = 0.028, 0.024      # leg section: 28 mm front to back, 24 mm side to side
RAIL_T, RAIL_H = 0.018, 0.022    # bottom rails: 18 mm thick, 22 mm tall
FREE_H = 0.027                   # rail undersides above the floor
TOP_T = 0.018
TOP_OVER_X = (CHEST_D - LEG_SPAN_X) / 2      # 45 mm front and back
LOW_DRAWER_Z = 0.063             # underside of the lower drawer
CHEST_MASS = 23.0                # 13.5 kg chest (package 14.78 kg) + about 9.5 kg of folded clothes
PINE = (0.88, 0.77, 0.58)
PINE_TOP = (0.90, 0.80, 0.63)
PINE_RAIL = (0.85, 0.73, 0.54)
PINE_FRONT = (0.89, 0.78, 0.60)
DRAWER_GAP = (0.42, 0.33, 0.22)  # shadowed gap and end grain lines between the drawers
HALF_SPAN_Y = LEG_SPAN_Y / 2


def _chest_shapes():
    s = []
    yo, yi = HALF_SPAN_Y, HALF_SPAN_Y - LEG_Y
    leg_h = CHEST_H - TOP_T
    for u0 in (0.0, LEG_SPAN_X - LEG_X):                       # four legs
        for side in (1, -1):
            s.append(box((u0 + LEG_X / 2, side * (yo - LEG_Y / 2), leg_h / 2), (LEG_X, LEG_Y, leg_h), PINE))
    zr = FREE_H + RAIL_H / 2
    s.append(box((RAIL_T / 2, 0.0, zr), (RAIL_T, 2 * yi, RAIL_H), PINE_RAIL))                       # front rail
    s.append(box((LEG_SPAN_X - RAIL_T / 2, 0.0, zr), (RAIL_T, 2 * yi, RAIL_H), PINE_RAIL))          # back rail
    inner_u = LEG_SPAN_X - 2 * LEG_X
    for side in (1, -1):                                       # side frames: bottom, slide and top rails
        for z0, h in ((FREE_H, RAIL_H), (0.272, 0.024), (0.484, 0.048)):
            s.append(box((LEG_SPAN_X / 2, side * (yo - RAIL_T / 2), z0 + h / 2), (inner_u, RAIL_T, h), PINE_RAIL))
    # top, overhanging 6 cm at the sides and 4.5 cm front and back
    s.append(box((LEG_SPAN_X / 2, 0.0, leg_h + TOP_T / 2), (CHEST_D, CHEST_W, TOP_T), PINE_TOP))
    # two shut drawers (fronts flush with the legs, 2 mm behind the leg faces); boxes full of folded clothes
    dw = 2 * yi - 0.004
    for z0, z1 in ((LOW_DRAWER_Z, 0.268), (0.300, 0.505)):
        s.append(box((0.002 + 0.009, 0.0, (z0 + z1) / 2), (0.018, dw, z1 - z0), PINE_FRONT))        # front board
        s.append(box((0.020 + (inner_u - 0.004) / 2, 0.0, (z0 + z1) / 2), (inner_u - 0.004, dw, z1 - z0), PINE))
    s.append(box((0.003, 0.0, 0.284), (0.004, dw, 0.030), DRAWER_GAP))                              # gap shadow
    return s


CHEST = Part("chest", (CHEST_X, 0.0, TOP + 0.0005), _chest_shapes(), mass=CHEST_MASS, color=PINE,
             friction=(0.30, 0.30), contact_offset=0.002)
# Leg feet (chest frame) for the "chest moved" check.
FEET = tuple((u, side * (HALF_SPAN_Y - LEG_Y / 2), 0.0) for u in (LEG_X / 2, LEG_SPAN_X - LEG_X / 2) for side in (1, -1))

# -- wall and skirting board behind the chest (fixed) ----------------------------------------------------------
WALL_X = 0.800
WALL = Part("wall", (WALL_X + 0.010, 0.0, TOP),
            [box((0.0, 0.0, 0.35), (0.020, 1.10, 0.70), (0.93, 0.92, 0.88)),
             box((-0.015, 0.0, 0.035), (0.010, 1.10, 0.070), (0.98, 0.98, 0.97))],
            mass=None, color=(0.93, 0.92, 0.88), friction=(0.3, 0.3))

# -- AA battery: Duracell alkaline AA (MN1500/LR6), 14.5 mm x 50.5 mm, 23.6 g ----------------------------------
BAT_R, BAT_L, BAT_MASS = 0.00725, 0.0505, 0.0236
NUB_R, NUB_L = 0.0027, 0.0012
BAT_START = (0.485, 0.090)       # centre; axis 38 deg left of straight ahead (it rolled in toward the back-right)
BAT_YAW = math.radians(38.0)
# The simulated cylinder settles about 15 mm along its rolling direction (back-right) in the first 3 s after the
# scene starts (measured on the GPU, runs fb-p1 and fb-p2); this is where it then lies, 11.6 cm behind the rail face.
BAT_REST = (0.495, 0.077)
BAT_BLACK = (0.07, 0.07, 0.07)
BAT_COPPER = (0.78, 0.45, 0.20)
BAT_STEEL = (0.55, 0.56, 0.58)
_can = BAT_L - NUB_L
_x0 = -BAT_L / 2                  # negative end
_xc = _x0 + _can - 0.013          # copper band: the last 13 mm of the can at the positive end
BATTERY = Part("battery", (BAT_START[0], BAT_START[1], TOP + BAT_R), [
    cyl(((_x0 + _xc) / 2, 0, 0), BAT_R, _xc - _x0, "x", BAT_BLACK),
    cyl(((_xc + _x0 + _can) / 2, 0, 0), BAT_R, _x0 + _can - _xc, "x", BAT_COPPER),
    cyl((_x0 + _can + NUB_L / 2, 0, 0), NUB_R, NUB_L, "x", BAT_STEEL),
    cyl((_x0 + 0.0004, 0, 0), 0.0062, 0.0010, "x", BAT_STEEL),
], mass=BAT_MASS, color=BAT_BLACK, yaw=BAT_YAW, friction=(0.20, 0.20), contact_offset=0.001)
# Rolling resistance (the simulator has none): angular damping on the battery, authored at spawn below.
BAT_ANGULAR_DAMPING = 4.0        # 1/s; matches a rolling-resistance coefficient of 0.01 at 5 cm/s (see README)

# -- coat hanger: white polypropylene heavy-duty tubular hanger (Uline S-22340: 16 5/8 in wide, 3/8 in tube) ----
# Hanger frame: origin at the middle of the bottom bar, x along the bar, y up the hanger toward the hook.
R_TUBE = 0.00475
BAR_HALF = 0.204                 # bar between the corner centres (422 mm outside width)
APEX_Y = 0.124
HOOK_C, HOOK_R = (0.0, 0.197), 0.022
LOOP_C, LOOP_R = (0.0, 0.095), 0.022
HANGER_MASS = 0.065
WHITE = (0.95, 0.95, 0.94)


def _hook_polylines():
    lines = [[(-BAR_HALF, 0.0), (0.0, APEX_Y)], [(BAR_HALF, 0.0), (0.0, APEX_Y)],
             [(0.0, APEX_Y), (HOOK_C[0] - HOOK_R, HOOK_C[1])]]
    lines.append([(HOOK_C[0] + HOOK_R * math.cos(a), HOOK_C[1] + HOOK_R * math.sin(a))
                  for a in np.radians(np.linspace(180, -20, 7))])
    lines.append([(LOOP_C[0] + LOOP_R * math.cos(a), LOOP_C[1] + LOOP_R * math.sin(a))
                  for a in np.radians(np.linspace(60, -190, 10))])
    return lines


def _tube_mesh(name, p, q, extend):
    """An octagonal tube 9.5 mm across the flats from p to q (hanger frame, z = 0 at the tube's centre), a flat
    underneath, each end extended by `extend` to close the joints."""
    p, q = np.array([p[0], p[1], 0.0]), np.array([q[0], q[1], 0.0])
    u = (q - p) / np.linalg.norm(q - p)
    v = np.cross([0.0, 0.0, 1.0], u)
    w = np.array([0.0, 0.0, 1.0])
    rc = R_TUBE / math.cos(math.pi / 8)
    ring = [rc * (math.cos(t) * v + math.sin(t) * w) for t in np.radians(22.5 + 45 * np.arange(8))]
    a, b = p - extend * u, q + extend * u
    pts = np.array([a + r for r in ring] + [b + r for r in ring])
    faces = []
    for k in range(1, 7):
        faces.append((0, k + 1, k))
        faces.append((8, 8 + k, 8 + k + 1))
    for k in range(8):
        k1 = (k + 1) % 8
        faces.append((k, k1, 8 + k1))
        faces.append((k, 8 + k1, 8 + k))
    faces = np.array(faces)
    tri = (pts - pts.mean(0))[faces]
    if np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    mid = (p + q) / 2
    n = max(1, int(np.linalg.norm(q - p) / 0.03))
    proxy = tuple(ball(tuple(p + (q - p) * (i + 0.5) / n), R_TUBE) for i in range(n))
    return Mesh(name, pts, faces.astype(np.int64), collision="convex", color=WHITE, proxy=proxy)


def _hanger_meshes():
    meshes = []
    for i, line in enumerate(_hook_polylines()):
        for j, (p, q) in enumerate(zip(line[:-1], line[1:])):
            meshes.append(_tube_mesh(f"tube_{i}_{j}", p, q, 0.8 * R_TUBE if i < 3 else 0.4 * R_TUBE))
    return meshes


GRIP_G = 0.144                   # the pinch: on the bar 60 mm in from its right-hand corner
HANGER_PHI0 = -math.pi / 2       # lying with the bar across the chest's front, hook toward the chest
BAR_DIR = np.array([math.cos(BAT_YAW), math.sin(BAT_YAW)])          # bar direction when slid in (far end ahead)
BAR_BACK = np.array([math.sin(BAT_YAW), -math.cos(BAT_YAW)])        # the hanger's body side when slid in
INSERT_CLEAR, INSERT_PAST, SLIDE = 0.016, 0.024, 0.12


def _rot2(phi):
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, -s], [s, c]])


def _hanger_origin_for_grip(grip_xy, phi):
    return np.asarray(grip_xy) - _rot2(phi) @ np.array([GRIP_G, 0.0])


# Pinch positions of the scripted demo (planned in the README): slid in, the bar lies parallel to the battery,
# 10 mm behind it, its far corner 30 mm past the battery's end.
_far = np.array(BAT_REST) + (BAT_L / 2 - BAT_R + BAT_R) * BAR_DIR
_bar_point = _far + BAR_BACK * (BAT_R + R_TUBE + INSERT_CLEAR) + INSERT_PAST * BAR_DIR
GRIP_IN = _bar_point - (GRIP_G + BAR_HALF) * BAR_DIR
GRIP_START = GRIP_IN - SLIDE * BAR_DIR
HANGER_START = _hanger_origin_for_grip(GRIP_START, HANGER_PHI0)
SWEEP_END_DEG = 86.0             # the sweep ends with the bar nearly across the chest's front
SWEEP_BACK = np.array([-0.025, 0.045])   # the pinch also moves toward the bar's far end, so the battery stays on the bar
HANGER_END = _hanger_origin_for_grip(GRIP_IN + SWEEP_BACK, math.radians(SWEEP_END_DEG) + math.pi)
HANGER_END_YAW_DEG = SWEEP_END_DEG + 180.0
BAT_END = (0.274, 0.301)         # where the demo leaves the battery (GPU run fb-n2)

HANGER = MeshPart("hanger", (float(HANGER_START[0]), float(HANGER_START[1]), TOP + R_TUBE + 0.0005),
                  [cyl(((GRIP_G - 0.010 - BAR_HALF) / 2, 0.0, 0.0), R_TUBE, GRIP_G - 0.010 + BAR_HALF, "x", WHITE),
                   cyl((GRIP_G, 0.0, 0.0), R_TUBE, 0.020, "x", WHITE),
                   cyl(((GRIP_G + 0.010 + BAR_HALF) / 2, 0.0, 0.0), R_TUBE, BAR_HALF - GRIP_G - 0.010, "x", WHITE)],
                  mass=HANGER_MASS, color=WHITE,
                  yaw=HANGER_PHI0, friction=(0.05, 0.05), contact_offset=0.0015, grip=1, meshes=_hanger_meshes())

OBJECTS = [CHEST, WALL, BATTERY, HANGER]


# -- spawn-time authoring the kit has no option for: battery rolling resistance, matte plastic hanger -------------
def _install_authoring():
    if getattr(yam_parts._spawn_part, "_fish_battery", False):
        return
    original_part, original_mesh = yam_parts._spawn_part, yam_mesh._author_mesh

    def spawn_part(prim_path, cfg, *args, **kwargs):
        prim = original_part(prim_path, cfg, *args, **kwargs)
        if cfg.part_name == BATTERY.name:
            from pxr import PhysxSchema
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateAngularDampingAttr().Set(BAT_ANGULAR_DAMPING)
        return prim

    def author_mesh(stage, parent, mesh, anchor, color, contact_offset):
        original_mesh(stage, parent, mesh, anchor, color, contact_offset)
        import isaaclab.sim as sim_utils
        from yam_arena.station import linear_rgb
        look_path = f"{parent}/mesh_{mesh.name}_matte"
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(mesh.color or color), roughness=0.45, metallic=0.0)
        look.func(look_path, look)
        sim_utils.bind_visual_material(f"{parent}/mesh_{mesh.name}", look_path, stage=stage)

    spawn_part._fish_battery = True
    yam_parts._spawn_part = spawn_part
    yam_mesh._author_mesh = author_mesh


_install_authoring()


# ------------------------------------------------------------------ geometry helpers
def battery_axis(bat):
    return bat.rotation()[:, 0]


def battery_front_back(bat):
    """(lowest x, highest x) of the battery: its can's extent along world x."""
    a = battery_axis(bat)
    ext = abs(a[0]) * BAT_L / 2 + BAT_R * math.sqrt(max(0.0, 1.0 - a[0] ** 2))
    c = np.asarray(bat.origin, float)
    return c[0] - ext, c[0] + ext


def hanger_points(hanger):
    """World points along the hanger's tube centre lines (bar and every tube segment)."""
    pts = [(-BAR_HALF, 0.0), (0.0, 0.0), (BAR_HALF, 0.0)]
    for line in _hook_polylines():
        pts.extend(line)
    return np.array([hanger.to_world((p[0], p[1], 0.0)) for p in pts])


def _seg_dist(p, a, b):
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(a + t * ab - p))


def hanger_distance(hanger, p):
    """Distance from point p to the hanger's tube surfaces."""
    segs = [((-BAR_HALF, 0.0), (BAR_HALF, 0.0))]
    for line in _hook_polylines():
        segs.extend(zip(line[:-1], line[1:]))
    p = np.asarray(p, float)
    d = min(_seg_dist(p, hanger.to_world((a[0], a[1], 0.0)), hanger.to_world((b[0], b[1], 0.0))) for a, b in segs)
    return d - R_TUBE


def battery_distance(bat, p):
    a = battery_axis(bat)
    c = np.asarray(bat.origin, float)
    return _seg_dist(np.asarray(p, float), c - a * (BAT_L / 2 - BAT_R), c + a * (BAT_L / 2 - BAT_R)) - BAT_R


def feet_shift(chest):
    """Largest horizontal shift and lift (m) of any of the chest's four feet from where it stood."""
    start = np.array([CHEST.pos[0], CHEST.pos[1], CHEST.pos[2]])
    shift, lift = 0.0, 0.0
    for f in FEET:
        now = chest.to_world(f)
        ref = start + np.asarray(f)
        shift = max(shift, float(np.linalg.norm(now[:2] - ref[:2])))
        lift = max(lift, float(now[2] - TOP))
    return shift, lift


# ------------------------------------------------------------------ scoring
RAIL_FACE = CHEST_X              # OUT: every part of the battery in front of the front rail's face
NEAR_1, NEAR_2 = BAT_REST[0] - 0.10, BAT_REST[0] - 0.04   # partial credit: battery centre 10 cm / 4 cm nearer
DEEP_X = BAT_REST[0] + 0.12      # sticky zero: battery's centre pushed 12 cm deeper
SIDE_Y = HALF_SPAN_Y             # sticky zero: battery out through either side of the chest
FOOT_SHIFT, FOOT_LIFT = 0.010, 0.003
DROP_Z = TOP - 0.020
STILL_M = 0.002                  # STILL: battery centre and hanger points move less than 2 mm over the last 1 s
HAND_CLEAR = 0.030
OPEN_MIN = 0.25                  # a gripper counts as open above a quarter of its full opening
TEST_STICKY = ()
TEST_STILL = None                # offline tests: None = judge by the objects' own speeds


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


_HIST = {"episode": None, "rows": []}


def _still(objects):
    bat, hanger = objects["battery"], objects["hanger"]
    if TEST_STILL is not None:
        return bool(TEST_STILL)
    if _RUN["t"] is None:
        return bat.resting() and hanger.resting()
    if _HIST["episode"] != _RUN["episode"]:
        _HIST["episode"], _HIST["rows"] = _RUN["episode"], []
    t = float(_RUN["t"])
    rows = _HIST["rows"]
    if not rows or rows[-1][0] < t:
        rows.append((t, np.r_[np.asarray(bat.origin, float), hanger_points(hanger).ravel()]))
    while len(rows) > 2 and rows[1][0] <= t - STILL_S:
        rows.pop(0)
    if rows[-1][0] - rows[0][0] < STILL_S - 1e-6:
        return False
    window = np.array([r[1] for r in rows])
    spread = window.max(0) - window.min(0)
    return bool(np.all(spread < STILL_M))


def hands_clear(objects):
    hands = grippers()
    if hands is None:
        return True
    bat, hanger = objects["battery"], objects["hanger"]
    return all(opening >= OPEN_MIN and battery_distance(bat, tcp) >= HAND_CLEAR and hanger_distance(hanger, tcp) >= HAND_CLEAR
               for tcp, opening in hands.values())


def battery_out(bat):
    _, back = battery_front_back(bat)
    return bool(back < RAIL_FACE and bat.origin[2] <= TOP + BAT_R + 0.003)


def hanger_down(hanger):
    return bool(hanger.tilt_deg() <= 10.0 and hanger.origin[2] <= TOP + R_TUBE + 0.004)


def status(objects):
    bat, hanger, chest = objects["battery"], objects["hanger"], objects["chest"]
    shift, lift = feet_shift(chest)
    return dict(out=battery_out(bat), centre_x=float(bat.origin[0]), hanger_down=hanger_down(hanger),
                shift=shift, lift=lift)


def score(objects):
    bat, hanger, chest = objects["battery"], objects["hanger"], objects["chest"]
    c = np.asarray(bat.origin, float)
    shift, lift = feet_shift(chest)
    deep = sticky("battery pushed deeper", c[0] > DEEP_X)
    side = sticky("battery out through a side", abs(c[1]) > SIDE_Y and c[0] > RAIL_FACE)
    moved = sticky("chest moved", shift > FOOT_SHIFT or lift > FOOT_LIFT)
    fell = sticky("off the table", c[2] < DROP_Z or hanger.origin[2] < DROP_Z
                  or chest.origin[2] < TOP - 0.02)
    still = _still(objects)
    clear = hands_clear(objects)
    if deep or side or moved or fell or not still:
        return 0.0
    if c[2] > TOP + BAT_R + 0.003:   # not resting on the floor (held up, or lying on the hanger)
        return 0.0
    if battery_out(bat):
        if clear and hanger_down(hanger):
            return 1.0
        return 0.7
    if c[0] < NEAR_1:
        return 0.4
    if c[0] < NEAR_2:
        return 0.2
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo (open loop: start layout and time)
GRASP_Z = TOP + 0.0155           # TCP: pad tips 2.5 mm above the floor, across the 9.5 mm tube
HOVER_Z = TOP + 0.080
PINCH_OPEN = -0.0125             # pads 25 mm apart: room on either side of the bar, clear of the shoulder tube
SWING_LAG_DEG = 2.0
DEMO_PACE = 1.0


def _yaw(phi):
    """Gripper yaw that closes the pads across the bar when the hanger's x axis points along phi."""
    y = (phi + math.pi / 2) % math.pi - math.pi / 2
    return y if y > -1e-9 else y + math.pi


def demo():
    plan = {"left": [], "right": []}
    st = {"pos": None, "f": FINGER_OPEN, "yaw": 0.0}

    def add(dt, label, pos=None, f=None, yaw=None):
        if pos is not None:
            st["pos"] = (float(pos[0]), float(pos[1]), float(pos[2]))
        if f is not None:
            st["f"] = f
        if yaw is not None:
            st["yaw"] = float(yaw)
        plan["right"].append(Waypoint(dt * DEMO_PACE, st["pos"], st["f"], label, st["yaw"]))

    phi_in = BAT_YAW + math.pi
    g0, g1 = GRIP_START, GRIP_IN
    g2 = g1 + SWEEP_BACK
    # the hanger drags on the floor and lags the wrist by about 2 deg in the swing (GPU runs fb-p2, fb-n1)
    yaw0, yaw1, yaw2 = _yaw(HANGER_PHI0), _yaw(phi_in) - math.radians(SWING_LAG_DEG), _yaw(math.radians(SWEEP_END_DEG) + math.pi)
    add(2.0, "over the hanger's bar, near its right corner", (g0[0], g0[1], HOVER_Z), PINCH_OPEN, yaw0)
    add(1.5, "down around the bar", (g0[0], g0[1], GRASP_Z))
    add(0.8, "pinch the bar", f=FINGER_CLOSED)
    add(0.5, "settle")
    add(3.0, "swing the hanger flat on the floor until the bar lines up with the battery", yaw=yaw1)
    add(0.5, "settle")
    add(3.5, "slide the hanger along its bar, under the rail and past the battery's end, behind it",
        (g1[0], g1[1], GRASP_Z))
    add(0.8, "settle")
    add(7.0, "sweep the bar round toward the chest's front; the battery rolls out ahead of it",
        (g2[0], g2[1], GRASP_Z), yaw=yaw2)
    add(1.5, "let the battery come to rest")
    add(0.6, "let go of the hanger", f=PINCH_OPEN)
    add(1.2, "lift clear", (g2[0], g2[1], TOP + 0.10))
    add(1.5, "back to the side", (0.12, -0.20, TOP + 0.15))
    add(3.0, "stillness tail")
    total = sum(w.duration for w in plan["right"])
    plan["left"] = [Waypoint(2.0, (0.12, 0.42, TOP + 0.15), FINGER_OPEN, "the left hand has no job: out of the way to the left", 0.0),
                    Waypoint(total - 2.0, (0.12, 0.42, TOP + 0.15), FINGER_OPEN, "wait", 0.0)]
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
