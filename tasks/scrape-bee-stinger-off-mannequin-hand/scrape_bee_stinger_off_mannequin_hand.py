"""A bee sting on the back of a hand: a life-size silicone mannequin right hand lies palm down on the table,
fingers pointing away from the robot, thumb toward the left arm. A honeybee stinger stand-in (a 3 mm black
shaft with its venom-sac bead) stands in a small red weal on the back of the hand, over the middle-finger bone,
half-way between the wrist and the knuckles. A plain blue plastic bank-size card (CR80, 85.6 x 54 x 0.76 mm) stands
on its long edge in a wooden place-card holder on the right of the table. First-aid guidance says to scrape the
stinger out sideways with a card's edge and never to pinch it. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box
from yam_joints import Hinge, Link, Slide
from yam_mesh import Mesh, MeshMechanism
from yam_arena import ik as _ik
from yam_arena import station as _station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"

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



# ------------------------------------------------------------------ mesh helpers (all our own)
def _loft(name, rings, color, proxy=None):
    """Closed mesh through a list of rings (each an (n, 3) array, same n), capped at both ends.
    PhysX collides it as its convex hull; the drawn surface follows the rings."""
    rings = [np.asarray(r, float) for r in rings]
    n, m = len(rings[0]), len(rings)
    pts = np.vstack(rings + [rings[0].mean(0), rings[-1].mean(0)])
    s, e = m * n, m * n + 1
    faces = []
    for i in range(m - 1):
        for j in range(n):
            a, b = i * n + j, i * n + (j + 1) % n
            c, d = (i + 1) * n + (j + 1) % n, (i + 1) * n + j
            faces += [(a, b, c), (a, c, d)]
    for j in range(n):
        faces.append((s, (j + 1) % n, j))
        faces.append((e, (m - 1) * n + j, (m - 1) * n + (j + 1) % n))
    faces = np.array(faces, int)
    tri = pts[faces]
    vol = np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum()
    if vol < 0:
        faces = faces[:, ::-1]
    if proxy is None:
        lo, hi = pts.min(0), pts.max(0)
        proxy = (box((lo + hi) / 2, hi - lo),)
    return Mesh(name, pts, faces, collision="convex", color=color, proxy=tuple(proxy))


def _frame(d):
    d = np.asarray(d, float) / np.linalg.norm(d)
    u = np.cross((0.0, 0.0, 1.0), d)
    u = u / np.linalg.norm(u)
    return d, u, np.cross(d, u)


def _ellipse(c, d, a, b, n=16):
    """Ring of n points round centre c, in the plane normal to d: half-width a (horizontal), half-height b."""
    _, u, v = _frame(d)
    phi = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.asarray(c, float) + np.outer(a * np.cos(phi), u) + np.outer(b * np.sin(phi), v)


def _segment(name, p0, p1, a0, b0, a1, b1, color, round0=0.6, round1=0.6, k=4):
    """A finger bone: an elliptic section from p0 (half sizes a0, b0) to p1 (a1, b1), each end domed."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    rings = []
    for t in np.linspace(1, 0, k, endpoint=False):                     # start dome, from its tip to full size
        ang = t * math.pi / 2 * 0.92
        rings.append(_ellipse(p0 - d * round0 * b0 * math.sin(ang), d, a0 * math.cos(ang), b0 * math.cos(ang)))
    rings.append(_ellipse(p0, d, a0, b0))
    rings.append(_ellipse(p1, d, a1, b1))
    for t in np.linspace(0, 1, k + 1)[1:]:
        ang = t * math.pi / 2 * 0.92
        rings.append(_ellipse(p1 + d * round1 * b1 * math.sin(ang), d, a1 * math.cos(ang), b1 * math.cos(ang)))
    return _loft(name, rings, color)


def _ellipsoid(name, c, rx, ry, rz, color, n=14, k=9):
    rings = []
    for t in np.linspace(-1, 1, k + 2)[1:-1]:
        s = math.sqrt(1 - t * t)
        rings.append(_ellipse(np.asarray(c, float) + (rx * t, 0, 0), (1, 0, 0), ry * s, rz * s, n))
    return _loft(name, rings, color)


# ------------------------------------------------------------------ layout (metres; world x forward, y left)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
SKIN = (0.88, 0.70, 0.60)
NAIL = (0.95, 0.83, 0.80)
WEAL = (0.86, 0.30, 0.30)
BLACK = (0.04, 0.03, 0.03)
BLUE = (0.10, 0.25, 0.65)
WOOD = (0.62, 0.45, 0.28)

# Hand frame = world axes (yaw 0): x along the fingers (away from the robot), y toward the thumb, z up; origin on
# the table under the middle of the knuckle line.
HAND_POS = (0.270, -0.100, TOP)
HAND_MASS = 0.45                    # real hand: silicone over a wire armature (estimate: ~0.4 L at 1.1 g/cm^3);
# the hand is taped palm down to the table, so it is a fixed body here (mass=None below).
WRIST_X, KNUCKLE_X = -0.100, -0.004  # wrist crease and the back of the knuckle line (hand frame)
CROWN_WRIST, CROWN_KNUCKLE = 0.0270, 0.0230    # back-of-hand height on the middle bone line
HALF_WRIST, HALF_KNUCKLE = 0.033, 0.040        # half width of the hand
SUPER = 2.5                                    # cross-section: flat palm, superelliptic back


def crown(x):
    """Height of the back of the hand on its centre line (hand frame x)."""
    f = (x - WRIST_X) / (KNUCKLE_X - WRIST_X)
    return CROWN_WRIST + f * (CROWN_KNUCKLE - CROWN_WRIST)


def _half(x):
    f = (x - WRIST_X) / (KNUCKLE_X - WRIST_X)
    return HALF_WRIST + f * (HALF_KNUCKLE - HALF_WRIST)


def _back_ring(x, n_arc=15):
    w, h = _half(x), crown(x)
    pts = [(x, -(w - 0.004), 0.0), (x, w - 0.004, 0.0)]
    for k in range(n_arc):
        phi = k * math.pi / (n_arc - 1)
        c, s = math.cos(phi), math.sin(phi)
        pts.append((x, w * c, 0.003 + (h - 0.003) * abs(s) ** (2.0 / SUPER)))
    return np.array(pts)


def _back_of_hand():
    xs = np.linspace(WRIST_X, KNUCKLE_X, 9)
    # Offline proxy: boxes inscribed under the arch (the true collider is the mesh's convex hull).
    proxy = []
    for x0, x1 in zip(xs[:-1], xs[1:]):
        w, h = min(_half(x0), _half(x1)), min(crown(x0), crown(x1))
        proxy.append(box(((x0 + x1) / 2, 0, 0.42 * h), (x1 - x0, 1.2 * w, 0.84 * h)))
    return _loft("back_of_hand", [_back_ring(x) for x in xs], SKIN, proxy)


# Fingers lying flat, slightly spread: knuckle point (x, y), direction (deg from +x), bone lengths, widths,
# thicknesses (proximal, middle, distal), knuckle dome centre.
FINGERS = {
    "index":  ((0.000, 0.026), 7.0, (0.040, 0.023, 0.018), (0.019, 0.017, 0.015), (0.017, 0.015, 0.013)),
    "middle": ((0.002, 0.008), 1.0, (0.044, 0.027, 0.019), (0.019, 0.017, 0.015), (0.017, 0.015, 0.013)),
    "ring":   ((0.000, -0.010), -5.0, (0.041, 0.026, 0.019), (0.018, 0.016, 0.014), (0.016, 0.014, 0.012)),
    "little": ((-0.006, -0.027), -12.0, (0.032, 0.019, 0.017), (0.016, 0.014, 0.012), (0.014, 0.012, 0.011)),
}


def _fingers():
    out = []
    for name, ((kx, ky), deg, lengths, widths, thick) in FINGERS.items():
        a = math.radians(deg)
        d = np.array((math.cos(a), math.sin(a), 0.0))
        p = np.array((kx, ky, 0.0))
        out.append(_ellipsoid(f"{name}_knuckle", (kx - 0.003, ky, 0.0135), 0.010, widths[0] / 2 + 0.0015,
                              0.0100, SKIN))
        for i, bone in enumerate(("proximal", "middle", "distal")):
            q = p + d * lengths[i]
            b0, b1 = thick[i] / 2, (thick[i + 1] if i < 2 else thick[i] * 0.9) / 2
            a0, a1 = widths[i] / 2, (widths[i + 1] if i < 2 else widths[i] * 0.9) / 2
            out.append(_segment(f"{name}_{bone}", p + (0, 0, b0), q + (0, 0, b1), a0, b0, a1, b1, SKIN,
                                round0=0.5, round1=1.0 if i == 2 else 0.5))
            p = q
        # Nail on the distal bone's back: a thin pale shell.
        tip_back = p - d * 0.008 + (0, 0, thick[2] * 0.9)
        out.append(_ellipsoid(f"{name}_nail", (0, 0, 0), 0.0055, widths[2] * 0.33, 0.0012, NAIL, n=12, k=5))
        nail = out[-1]
        R = np.array(((d[0], -d[1], 0), (d[1], d[0], 0), (0, 0, 1)))
        pts = nail.points @ R.T + tip_back
        lo, hi = pts.min(0), pts.max(0)
        out[-1] = Mesh(nail.name, pts, nail.faces, collision="convex", color=NAIL, proxy=(box((lo + hi) / 2, hi - lo),))
    return out


def _thumb():
    base, mid, tip = np.array((-0.080, 0.030, 0.0)), np.array((-0.040, 0.052, 0.0)), np.array((-0.012, 0.064, 0.0))
    return [
        _segment("thumb_pad", base + (0, 0, 0.012), mid + (0, 0, 0.0105), 0.014, 0.012, 0.0115, 0.0105, SKIN),
        _segment("thumb_tip", mid + (0, 0, 0.0095), tip + (0, 0, 0.0085), 0.0105, 0.0095, 0.0095, 0.0085, SKIN,
                 round1=1.0),
    ]


def _wrist():
    rings = [_ellipse((x, 0.0, 0.0215), (1, 0, 0), 0.031, 0.0215, 18) for x in (-0.128, -0.112, -0.097)]
    return _loft("wrist", rings, SKIN)


# Sting site: on the back of the hand over the middle-finger bone, half-way along it.
STING_X = -0.050
SITE_L = np.array((STING_X, 0.0, crown(STING_X)))     # hand frame (the arch's top vertex line: exact)
SLOPE = math.atan2(CROWN_WRIST - CROWN_KNUCKLE, KNUCKLE_X - WRIST_X)   # back of hand falls toward the knuckles


def _weal():
    """A 12 mm red weal round the sting, its top 0.15 mm proud of the skin, its rim sunk into it."""
    n = np.array((math.sin(SLOPE), 0.0, math.cos(SLOPE)))
    rings = [_ellipse(SITE_L - n * 0.0004, n, 0.006, 0.006, 20), _ellipse(SITE_L + n * 0.00015, n, 0.0035, 0.0035, 20)]
    return _loft("weal", rings, WEAL, proxy=())


# Stinger stand-in: a 0.25 mm black monofilament, 1.5 mm in the skin and 1.5 mm out, with a 1.5 mm black bead on
# its outer end (the venom sac). It collides as a 0.8 mm square shaft and a 1.5 mm bead.
EMBED, SHAFT_OUT, SAC_D = 0.0015, 0.0015, 0.0015
STING_MASS = 0.0002                 # each link; raised from ~5 mg for solver stability (its weight stays negligible)
PULL_OUT_N = 0.11                   # barbed shaft's hold (measured ~0.11 N pull-out, rabbit skin)
TILT_HOLD_NM = 2.0e-5               # stands upright until something pushes it
DRAG_LIMIT, TILT_LIMIT = 0.012, 80.0
SITE_W = np.array(HAND_POS) + SITE_L
SAC_L = SITE_L + (0, 0, SHAFT_OUT + SAC_D / 2)        # bead centre (part frame, q = 0)
STING_LINKS = (
    Link("sting_root", [ball(tuple(SITE_L - (0, 0, 0.0009)), 0.0004)],
         Slide("x", tuple(SITE_L), -DRAG_LIMIT, DRAG_LIMIT, damping=0.02, armature=2e-4,
               static_friction=PULL_OUT_N, dynamic_friction=0.09),
         mass=STING_MASS, color=BLACK, friction=(0.6, 0.5)),
    Link("stinger", [box(tuple(SITE_L + (0, 0, (SHAFT_OUT - EMBED) / 2)), (0.0008, 0.0008, EMBED + SHAFT_OUT)),
                     ball(tuple(SAC_L), SAC_D / 2)],
         Hinge("y", tuple(SITE_L), -TILT_LIMIT, TILT_LIMIT, damping=2e-6, armature=1e-9,
               static_friction=TILT_HOLD_NM, dynamic_friction=1.0e-5),
         mass=STING_MASS, parent="sting_root", color=BLACK, friction=(0.6, 0.5)),
)
HAND = MeshMechanism(
    "hand", HAND_POS, [ball((-0.050, 0.0, 0.011), 0.009)], STING_LINKS, mass=None, color=SKIN,
    friction=(0.9, 0.8), contact_offset=0.001,
    meshes={"base": tuple([_back_of_hand(), _wrist(), *_thumb(), *_fingers(), _weal()])})

# Card holder (fixed, taped down): a beech place-card block, 70 mm long (along y) x 24 x 16 mm, with a 1.2 mm slot
# 10 mm deep along its length.
HOLDER_X, HOLDER_Y = 0.200, -0.300
HOLDER_L, HOLDER_W, HOLDER_H, SLOT_W, SLOT_FLOOR = 0.070, 0.024, 0.016, 0.0012, 0.006
_jaw = (HOLDER_W - SLOT_W) / 2
HOLDER = Part("card_holder", (HOLDER_X, HOLDER_Y, TOP), [
    box((0, 0, SLOT_FLOOR / 2), (HOLDER_W, HOLDER_L, SLOT_FLOOR)),
    box((-(SLOT_W + _jaw) / 2, 0, (SLOT_FLOOR + HOLDER_H) / 2), (_jaw, HOLDER_L, HOLDER_H - SLOT_FLOOR)),
    box(((SLOT_W + _jaw) / 2, 0, (SLOT_FLOOR + HOLDER_H) / 2), (_jaw, HOLDER_L, HOLDER_H - SLOT_FLOOR)),
], color=WOOD, friction=(0.4, 0.3), contact_offset=0.0005)

# Card: plain blue PVC, ISO/IEC 7810 ID-1 (CR80), 85.6 x 54 x 0.76 mm, 4.8 g; standing on its long edge in the slot.
CARD_T, CARD_L, CARD_H, CARD_MASS = 0.00076, 0.0856, 0.054, 0.0048
CARD_START = (HOLDER_X, HOLDER_Y, TOP + SLOT_FLOOR + CARD_H / 2 + 0.0002)
CARD = Part("card", CARD_START, [box((0, 0, 0), (CARD_T, CARD_L, CARD_H))], CARD_MASS, BLUE,
            friction=(0.6, 0.5), contact_offset=0.0005, grip=0)

PRESSED_PARTS = ("hand",)            # the card's edge is drawn along the back of the hand on purpose
OBJECTS = [HOLDER, HAND, CARD]
# Layout helpers for the scorer tests (states.yaml).
CARD_FLAT_Z = TOP + CARD_T / 2
PUT_X, PUT_Y = 0.220, -0.200

# ------------------------------------------------------------------ scoring
TIPPED_DEG = 30.0          # 0.25: stinger tipped over by 30 degrees or more
OUT_DRAG = 0.003           # 0.5: drawn 3 mm (twice its 1.5 mm in the skin) along the skin ...
OUT_TILT_DEG = 45.0        # ... and lying over at least 45 degrees the way it was drawn
CARD_DOWN_TOL = 0.003      # card's lowest point within 3 mm of the table top
HAND_GAP = 0.005           # card's box at least 5 mm clear of the hand's box (not on or against the hand)
PINCH_GAP = 0.004          # pads closed to 4 mm or less ...
PINCH_REACH = 0.010        # ... with the bead between them (within 10 mm of the pad centre across the pad width)
STEP_M, STEP_DEG = 0.0005, 0.5
TEST_STICKY = ()           # offline tests: sticky keys that already fired earlier in the episode
TEST_FRAMES = None         # offline tests: {"right": (tcp, R, opening)}
_CLOSE_AXIS = int(np.argmax(np.abs(_ik.quat_xyzw_to_matrix(np.asarray((0.0, 1.0, 0.0, 0.0)))[1]))) \
    if hasattr(_ik, "quat_xyzw_to_matrix") else 1


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def _angle_deg(R):
    c = (np.trace(np.asarray(R)) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def gripper_frames():
    """{arm: (tcp, rotation, opening fraction)} from the robot's joint state; None offline."""
    if TEST_FRAMES is not None:
        return TEST_FRAMES
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
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
        tcp, R = _ik.tcp_pose(q[arm_ids], _station.BASES[arm])
        out[arm] = (np.asarray(tcp), np.asarray(R), float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0)))
    return out


def _closing_axis(R):
    """Unit vector along which the pads close, for a TCP rotation R."""
    down = _closing_axis.down
    # At yaw 0 the pads close along world y: pick the rotation column that does so in the reference grip.
    return R[:, down]


_closing_axis.down = None


def _init_closing_axis():
    from yam_arena.demo import grip_rotation
    R0 = grip_rotation(0.0)
    _closing_axis.down = int(np.argmax(np.abs(R0[1])))


def pinched(sac_world):
    """The bead is between a gripper's pads and the pads are closed on it (4 mm gap or less)."""
    frames = gripper_frames()
    if not frames:
        return False
    if _closing_axis.down is None:
        _init_closing_axis()
    for tcp, R, opening in frames.values():
        R = np.asarray(R, float)
        d = np.asarray(sac_world, float) - np.asarray(tcp, float)
        approach, close = R[:, 2], _closing_axis(R)
        width = np.cross(approach, close)
        along = float(d @ approach)
        gap = opening * 0.094
        if (gap <= PINCH_GAP and -0.012 <= along <= PAD_TIP_PAST_TCP + 0.002
                and abs(float(d @ width)) <= PINCH_REACH and abs(float(d @ close)) <= gap / 2 + 0.001):
            return True
    return False


def sting_state(hand):
    drag = float(hand.joints["sting_root"])
    tilt = math.degrees(float(hand.joints["stinger"]))
    tipped = abs(tilt) >= TIPPED_DEG
    out = abs(drag) >= OUT_DRAG and abs(tilt) >= OUT_TILT_DEG and drag * tilt > 0
    return drag, tilt, tipped, out


def card_in_holder(card):
    c = np.asarray(card.pos, float)
    return abs(c[0] - HOLDER_X) < 0.010 and abs(c[1] - HOLDER_Y) < 0.030 and card.low[2] < TOP + HOLDER_H


def card_down(card, hand):
    if card.low[2] > TOP + CARD_DOWN_TOL:
        return False
    lo, hi = np.asarray(hand.low, float), np.asarray(hand.high, float)
    clo, chi = np.asarray(card.low, float), np.asarray(card.high, float)
    gap = np.maximum(np.maximum(lo[:2] - chi[:2], clo[:2] - hi[:2]), 0.0)
    return float(np.max(gap)) >= HAND_GAP


_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS = None
_QUIET = False


def at_rest(objects):
    """Pose differences between consecutive control steps for the hand, card and stinger."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS, _QUIET
    hand, card = objects["hand"], objects["card"]
    if _RUN["t"] is None:
        return card.resting() and abs(hand.joint_vel["stinger"]) < 0.05 and abs(hand.joint_vel["sting_root"]) < 0.005
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS, _POSE_TOKEN, _QUIET = None, None, False
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"], float(_RUN["t"]))
    if token != _POSE_TOKEN:
        pose = (np.r_[card.to_world((0, 0.04, 0.025)),
                      card.to_world((0, -0.04, -0.025)), float(hand.joints["sting_root"])],
                math.degrees(float(hand.joints["stinger"])))
        _QUIET = False
        if _PREVIOUS is not None:
            step, old = _PREVIOUS
            _QUIET = bool(token[0] == step + 1 and np.all(np.abs(pose[0] - old[0]) < STEP_M)
                          and abs(pose[1] - old[1]) < STEP_DEG)
        _PREVIOUS = (token[0], pose)
        _POSE_TOKEN = token
    return _QUIET


def score(objects):
    hand, card = objects["hand"], objects["card"]
    sac = hand.link_to_world("stinger", SAC_L)
    pinch = sticky("stinger pinched", pinched(sac))
    lifted = ever("card lifted", card.low[2] > TOP + HOLDER_H + 0.005)
    used = lifted or not card_in_holder(card)
    _, _, tipped, out = sting_state(hand)
    down = card_down(card, hand) and released(card)
    quiet = at_rest(objects)
    full = used and out and down and released(hand) and quiet
    seconds = held_for("stinger out, card down, all clear and still", full)
    if pinch or not used:
        return 0.0
    if not tipped:
        return 0.0
    if not out:
        return 0.25
    if not down:
        return 0.5
    if full and seconds + 1e-9 >= STILL_S:
        return 1.0
    return 0.75


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo (straight-down grips only)
YAW = math.pi / 2                      # right gripper: pads close along x, so the card's faces face +-x
PINCH_DEPTH = 0.018                    # pads cover the card's top 18 mm
CARD_TCP_ABOVE_EDGE = CARD_H - PINCH_DEPTH + PAD_TIP_PAST_TCP   # TCP height above the card's lower long edge
PICK = np.array((HOLDER_X, HOLDER_Y, CARD_START[2] - CARD_H / 2 + CARD_TCP_ABOVE_EDGE))
PRESS = 0.0008                         # edge commanded 0.8 mm into the skin (light; the arm is soft)
SCRAPE_FROM, SCRAPE_TO = STING_X - 0.016, STING_X + 0.020   # hand-frame x of the card plane: behind -> past
HAND_W = np.array(HAND_POS)


def edge_tcp(x_hand, above):
    """TCP for the card's lower edge at hand-frame x, `above` metres over the back of the hand's crown line."""
    return HAND_W + (x_hand, 0.0, crown(x_hand) + above + CARD_TCP_ABOVE_EDGE)


PUT = np.array((PUT_X, PUT_Y, TOP + CARD_TCP_ABOVE_EDGE))
_LEFT_HOME = (0.200, 0.300, TOP + 0.150)  # the left arm is not needed: it waits, open, clear of the scene
DEMO_PACE = 1.0


def demo():
    plan = {"left": [], "right": []}
    st = {"r": PICK + (0, 0, 0.060), "rf": FINGER_OPEN}

    def add(dt, label, r=None, rf=None):
        if r is not None:
            st["r"] = np.asarray(r, float)
        if rf is not None:
            st["rf"] = rf
        plan["right"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in st["r"]), st["rf"], label, YAW))
        plan["left"].append(Waypoint(dt * DEMO_PACE, _LEFT_HOME, FINGER_OPEN, label, 0.0))

    add(2.5, "over the card in its holder")
    add(1.5, "pads round the card's top edge", r=PICK)
    add(1.0, "pinch the card", rf=FINGER_CLOSED)
    add(1.5, "lift it out of the holder", r=PICK + (0, 0, 0.070))
    add(2.5, "over the back of the hand, behind the sting", r=edge_tcp(SCRAPE_FROM, 0.030))
    add(1.5, "edge down near the skin", r=edge_tcp(SCRAPE_FROM, 0.004))
    add(1.0, "edge on the skin", r=edge_tcp(SCRAPE_FROM, -PRESS))
    add(3.5, "scrape across the sting", r=edge_tcp(SCRAPE_TO, -PRESS))
    add(1.0, "lift the card off the hand", r=edge_tcp(SCRAPE_TO, 0.040))
    add(2.0, "carry it to the free table beside the hand", r=PUT + (0, 0, 0.050))
    add(1.5, "edge down on the table", r=PUT + (0, 0, -0.001))
    add(0.8, "let go", rf=FINGER_OPEN)
    # Straight up beside the hand: from there the right wrist camera looks across the back of the hand.
    add(1.5, "hand up, clear of the card and the hand", r=PUT + (0, 0, 0.075))
    add(3.0, "still")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
