"""Draw two cafe-curtain panels shut along their rod until their leading edges meet.

A small window on the table: two upright pine boards (1 x 3, 38 cm tall) 46 cm apart inside, each screwed to a 1 x 4
foot clamped to the table, and between them a white 7/16 in spring tension rod (Kenney KN630/1, 18-28 in) 36.2 cm
above the table top, 17 cm in front of the line through the robot's arm bases. A pair of cafe tier curtains (two
panels, each 24 x 12 in = 61 x 30.5 cm) hangs from it on twelve 1 in black clip rings, six per panel along the top
hem, hems 1 cm above the table. The curtains are open: each panel is pushed back toward its own board, its leading
(inner) edge 10.5 cm from the middle, the rest of its cloth gathered in deep folds. The robot draws both panels shut so
that their leading edges meet in the middle, then lets go.

The panels are PhysX cloth (cloth_kit.py, a surface deformable, every node simulated). Each ring is a rigid body that
hangs on the rod and slides or swings on it by contact and friction; each clip holds its bite of cloth by a fixed
vertex attachment (curtain_kit.py). Pads hold cloth only by cloth_kit's contact-gated constraint (both pads stopped
on the cloth with a measured squeeze; opening releases it). Rod and boards are fixed. Metres; x forward from the
midpoint between the arm bases, y to the robot's left, z up.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box, cyl  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
import cloth_kit as ck  # noqa: E402
import curtain_kit as cu  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 1200   # 60 s at 20 Hz
DIAG = False             # GPU diagnostic copies print CURTAIN lines
DEMO_ONE_HAND = False    # wrong-way copy: only the left hand draws its panel; the right hand stays parked
TEST_GRIPPERS = None     # offline tests: {"left": (tcp xyz, opening 0..1), "right": ...}
TEST_HELD = False        # offline tests: pretend a hand still holds a panel
TEST_CLOTH = None        # offline tests: {"curtain_l": nodes (n, 3), "curtain_r": ...} for a state without nodes
TEST_RINGS = None        # offline tests: {"ring_l0": (x, y, z), ...} ring centres for a state without them


# ================================================================================================ scoring (first)
# The scorer reads both panels' nodes (objects["curtain_l"].nodes) and the rings' poses. What a person checks in a
# real trial is given with each test.
CLOSED_OK = 0.020        # m: widest gap between the leading edges for full credit
STILL_WINDOW, STILL_MOVE = 1.0, 0.010   # still: no node moved more than 10 mm over the last second (video)
CLEAR = 0.020            # m: a hand has let go when its tool point is this far from every cloth node
DRAWN_MIN = 0.050        # m: a panel counts as drawn once its leading clip has moved this far toward the middle
OFF_ROD = 0.020          # m: a ring whose centre is this far from where it hangs on the rod has come off it


def leading_edges(objects):
    """{side: (rows, 3)} each panel's leading edge from its top (clip) to its hem, node by node (real trial: the
    panel's inner side hem)."""
    out = {}
    for side, name in (("left", "curtain_l"), ("right", "curtain_r")):
        nodes = cloth_nodes(objects, name)
        out[side] = nodes[LEAD]
    return out


def edge_gap(objects):
    """Widest gap (m) between the two leading edges, height by height from the clips to the hems (negative where the
    edges overlap). Real trial: lay a ruler across the two inner side hems at the clips, halfway down and at the hems,
    and take the widest reading."""
    e = leading_edges(objects)
    return float(np.max(e["left"][:, 1] - e["right"][:, 1]))


def ring_centres(objects):
    out = {}
    for name in RING_NAMES:
        if TEST_RINGS is not None and name in TEST_RINGS:
            out[name] = np.asarray(TEST_RINGS[name], float)
        else:
            o = objects[name]
            org = getattr(o, "origin", None)
            out[name] = np.asarray(org if org is not None else getattr(o, "pos", RING0[name]), float)
    return out


def rings_on_rod(objects):
    """Every ring still hangs on the rod between its brackets (real trial: every ring round the rod)."""
    for name, c in ring_centres(objects).items():
        if math.hypot(c[0] - ROD_X, c[2] - (TOP + ROD_Z - cu.RING_DROP)) > OFF_ROD or abs(c[1]) > BRACKET_Y:
            return False
    return True


def drawn(objects):
    """(left, right) how far (m) each panel's leading clip has moved toward the middle from where it started."""
    c = ring_centres(objects)
    return (float(RING0["ring_l0"][1] - c["ring_l0"][1]), float(c["ring_r0"][1] - RING0["ring_r0"][1]))


def over_rod(objects):
    """Any cloth higher than the rod's top (a panel flung over the rod; real trial: cloth seen above the rod)."""
    top = TOP + ROD_Z + cu.ROD_R
    return bool(max(cloth_nodes(objects, n)[:, 2].max() for n in ("curtain_l", "curtain_r")) > top)


def hands_clear(objects):
    """Both hands have let go: no gated hold is on and each tool point is CLEAR from every cloth node (real trial: both
    grippers open and off the curtains in the final photo)."""
    if TEST_HELD:
        return False
    rule = ck.grip_rule()
    if rule is not None and any(a.on for a in rule._att.values()):
        return False
    hands = grippers()
    if hands is None:
        return True
    n = np.vstack([cloth_nodes(objects, "curtain_l"), cloth_nodes(objects, "curtain_r")])
    return all(float(np.min(np.linalg.norm(n - np.asarray(tcp, float), axis=1))) >= CLEAR for tcp, _ in hands.values())


def score_terms(objects):
    """All the scored quantities and the score: 0 if a hand still holds a curtain, the curtains are still moving, a
    ring is off the rod, or either panel was not drawn (its leading clip moved less than 5 cm toward the middle);
    otherwise the closure band, at most 0.5 if cloth lies over the rod."""
    _episode_memory()
    gap = edge_gap(objects)
    dl, dr = drawn(objects)
    on_rod = ever_all("on_rod", rings_on_rod(objects))
    flung = over_rod(objects)
    move = cloth_motion(objects)
    still = move <= STILL_MOVE
    released = hands_clear(objects)
    terms = dict(gap=gap, drawn=(dl, dr), on_rod=on_rod, over_rod=flung, move=move, still=still, released=released)
    if not released or not still or not on_rod or min(dl, dr) < DRAWN_MIN:
        terms["score"] = 0.0
        return terms
    band_close = 1.0 if gap <= CLOSED_OK else 0.75 if gap <= 0.050 else 0.5 if gap <= 0.100 else 0.25
    band_rod = 0.5 if flung else 1.0
    terms.update(bands=(band_close, band_rod))
    terms["score"] = min(band_close, band_rod)
    return terms


def score(objects):
    t = score_terms(objects)
    if DIAG:
        _diag(objects, t)
    return t["score"]


def success(objects):
    return score_terms(objects)["score"] >= 1.0


# ================================================================================================ run context
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
        context["step"] = 0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY = {"episode": None}
_EVER, _HIST = {}, []


def _episode_memory():
    if _MEMORY["episode"] != _RUN["episode"]:
        _EVER.clear()
        _HIST.clear()
        _MEMORY["episode"] = _RUN["episode"]


def ever_all(key, cond):
    """True while the condition has held at every check this episode (offline each call stands alone)."""
    if _RUN["t"] is None:
        return bool(cond)
    _EVER[key] = bool(_EVER.get(key, True) and cond)
    return _EVER[key]


def cloth_nodes(objects, name):
    nodes = getattr(objects[name], "nodes", None)
    if nodes is None:
        if TEST_CLOTH is not None and name in TEST_CLOTH:
            return np.asarray(TEST_CLOTH[name], float)
        return NODES0[name]
    return np.asarray(nodes, float)


def cloth_motion(objects):
    """The largest distance any cloth node moved over the last STILL_WINDOW s (m); before a full second (and for a
    state outside a run) the fastest node's speed times the window."""
    nodes = np.vstack([cloth_nodes(objects, "curtain_l"), cloth_nodes(objects, "curtain_r")])
    t = _RUN["t"]
    if t is None:
        speed = 0.0
        for n in ("curtain_l", "curtain_r"):
            vel = getattr(objects[n], "node_vel", None)
            if vel is not None:
                speed = max(speed, float(np.linalg.norm(np.asarray(vel, float), axis=1).max()))
        return speed * STILL_WINDOW
    if not _HIST or t > _HIST[-1][0] + 1e-6:
        _HIST.append((float(t), nodes.copy()))
    while len(_HIST) > 1 and _HIST[1][0] <= t - STILL_WINDOW + 1e-6:
        _HIST.pop(0)
    t0, n0 = _HIST[0]
    if t - t0 < STILL_WINDOW - 1e-6:
        return 1.0
    return float(np.linalg.norm(nodes - n0, axis=1).max())


def grippers():
    """{side: (tcp xyz, opening 0..1)} from the simulator (or TEST_GRIPPERS offline)."""
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    hands = ck.read_hands(env)
    return {s: (h.tcp, float(np.clip(h.gap / (2 * ck.FINGER_TRAVEL), 0, 1))) for s, h in hands.items()}


# ================================================================================================ the scene
ROD_WHITE = (0.93, 0.93, 0.91)
PINE = (0.80, 0.66, 0.46)
CURTAIN = (0.93, 0.89, 0.80)

# Kenney KN630/1 spring tension rod, white, 7/16 in (11.1 mm), adjustable 18-28 in, sprung between the boards' inner
# faces (46 cm apart), its axis ROD_Z above the table at x = ROD_X. The boards: pine 1 x 3 (19 x 64 mm), 38 cm tall,
# each screwed upright to a 20 cm length of pine 1 x 4 (19 x 89 mm) lying outside the window and clamped to the table
# (so the rod's spring cannot spread them). All fixed in the simulation. The window is narrow because the arms' start
# pose (wrists 32 cm up, 25-38 cm to each side) leaves room for a rod at this height only between them.
ROD_X = 0.170
ROD_Z = 0.362
JAMB_Y = 0.230                   # inner faces of the jambs
JAMB_T, JAMB_D, JAMB_H = 0.019, 0.064, 0.380
FOOT_X, FOOT_Y, FOOT_T = 0.200, 0.089, 0.019
BRACKET_Y = JAMB_Y


def _jamb_shapes(sign):
    yj = sign * (JAMB_Y + JAMB_T / 2)
    yf = sign * (JAMB_Y + FOOT_Y / 2)
    return [box((ROD_X, yj, FOOT_T + JAMB_H / 2), (JAMB_D, JAMB_T, JAMB_H), PINE),
            box((ROD_X + 0.02, yf, FOOT_T / 2), (FOOT_X, FOOT_Y, FOOT_T), PINE),
            cyl((ROD_X, sign * (JAMB_Y - 0.002), ROD_Z), 0.008, 0.004, axis="y", color=ROD_WHITE)]   # rubber end cap


RAIL = Part("rail", (0.0, 0.0, TOP), [
    cyl((ROD_X, 0.0, ROD_Z), cu.ROD_R, 2 * JAMB_Y, axis="y", color=ROD_WHITE),
    *_jamb_shapes(1.0), *_jamb_shapes(-1.0)], color=ROD_WHITE, friction=cu.RING_FRICTION)

# Cafe tier curtain pair, two panels of 24 x 12 in (61 x 30.5 cm), linen-cotton-polyester blend, flat top hem. Mass
# 32 g per panel and the sim thickness 2.5 mm are estimates (kitchen scale; the real fabric is thinner, its hems
# doubled). Six clip rings per panel (twelve of the set's fourteen) at 12.2 cm along the top hem, one at each corner.
# Open: the leading clip 10.5 cm from the middle, the next 6 cm further out, the other four 1.45 cm apart up to the
# jamb; the cloth between each two clips in one deep fold, the first behind the rod, then alternately in front and
# behind; the cloth's top edge CLOTH_DROP below the rod's axis, so the hem hangs 1 cm above the table.
WIDTH, DROP = 0.610, 0.305
RING_U = tuple(i * 0.610 / 5 for i in range(6))
LEAD_Y0 = 0.105                  # the panels' leading edges start this far from the middle
CLIP_IN = 0.006                  # the leading clip's middle sits this far in from its panel's edge (clip side flush)
RING_Y = (LEAD_Y0, 0.165, 0.1792, 0.1934, 0.2075, 0.2215)
BULGE = (1, -1, 1, -1, 1)        # the cloth between clips falls in front (-1) or behind (+1) the rod
TOP_Z = ROD_Z - cu.CLOTH_DROP     # the cloth's top edge above the table
CLOTH = dict(thickness=0.0025, mass_kg=0.032, youngs=1.0e6, poisson=0.25, bend=300.0, damping=1.0, bend_damping=5.0,
             friction=1.0, rest_offset=0.00125, contact_offset=0.00525, iterations=24, self_collide=True, edge=0.010,
             color=CURTAIN)

RINGS = {}
PANELS = {}
for side, sign, tag in (("left", 1.0, "l"), ("right", -1.0, "r")):
    ys = tuple(sign * y for y in RING_Y)
    RINGS[side] = [cu.clip_ring(f"ring_{tag}{k}", (ROD_X, y + (sign * CLIP_IN if k == 0 else 0.0),
                                                   TOP + ROD_Z - cu.RING_DROP)) for k, y in enumerate(ys)]
    PANELS[side] = cu.panel(f"curtain_{tag}", rod_x=ROD_X, top_z=TOP + TOP_Z, ring_u=RING_U, ring_y=ys, bulge=BULGE,
                            width=WIDTH, length=DROP, **CLOTH)
    cu.hang(PANELS[side], RINGS[side])
RING_NAMES = tuple(r.name for side in ("left", "right") for r in RINGS[side])
RING0 = {r.name: r.pos for side in ("left", "right") for r in RINGS[side]}
ck.use_grip_rule("curtain_l", "curtain_r")
PRESSED_PARTS = ("ring_l0", "ring_r0")   # for the offline checker: the hands push these on purpose

OBJECTS = [RAIL, *RINGS["left"], *RINGS["right"], PANELS["left"], PANELS["right"]]
NODES0 = {p.name: p.mesh(world=True)[0] for p in PANELS.values()}
UV = PANELS["left"].uv()
_ROWS = np.unique(np.round(UV[:, 1], 6))
LEAD = np.array([int(np.flatnonzero((np.abs(UV[:, 0]) < 1e-6) & (np.abs(UV[:, 1] - v) < 1e-6))[0]) for v in _ROWS])


def test_closed(gap=0.010, flung=False, left_only=False):
    """Offline test shape: both panels hanging straight down from their clips, the leading edges ``gap`` apart (only the
    left panel drawn if ``left_only``), the rings spread evenly to the jamb, the cloth between clips in shallow
    zig-zags; the right panel's lower outer part thrown over the rod if ``flung``. Returns (TEST_CLOTH, TEST_RINGS). Not
    used by the simulator."""
    cloth, rings = {}, {}
    for side, sign, tag in (("left", 1.0, "l"), ("right", -1.0, "r")):
        lead = gap / 2 if (side == "left" or not left_only) else LEAD_Y0
        ys = np.linspace(lead, RING_Y[-1], len(RING_U))
        y = sign * np.interp(UV[:, 0], RING_U, ys)
        z = TOP + TOP_Z - UV[:, 1]
        x = ROD_X + 0.03 * np.sin(UV[:, 0] / RING_U[1] * np.pi)
        if flung and side == "right":
            z = np.where((UV[:, 1] > 0.2) & (UV[:, 0] > 0.4), TOP + ROD_Z + 0.03, z)
        cloth[f"curtain_{tag}"] = np.stack([x, y, z], 1)
        for k in range(len(RING_U)):
            rings[f"ring_{tag}{k}"] = (ROD_X, sign * (ys[k] + (CLIP_IN if k == 0 else 0.0)), TOP + ROD_Z - cu.RING_DROP)
    return cloth, rings


# ================================================================================================ the demonstration
# Both hands start behind the window, outside the jambs (the harness's start pose). Each raises its hand high over its
# own shoulder, swings round over its panel's gathered stack (well above the rod and the jamb), turns the fingers to
# point forward and 20 deg toward the middle (closed, stacked one above the other), comes down in front of the rod
# just on the outer side of its panel's leading clip, moves in under the rod, puts the side of its fingers against the
# clip's outer side and pushes that clip along the rod to the middle; the panel follows on its rings. Then it backs
# out, rises and stays clear over its panel. Both hands rise and swing together; the right hand pushes first and the
# left waits over its stack until the right is backing out (the two tool points stay over 10 cm apart). Way points up
# to the stack follow a joint-space path (tool poses computed from the joint key poses); then straight tool moves.
G = ck.FINGER_TRAVEL
OPEN, CLOSED = -G, 0.0
PUSH_YAW = 20.0                      # fingers forward and this far toward the middle, level, closing axis vertical
PUSH_DX = -0.010                     # tool point this far in front of the rod's axis while pushing
PUSH_DZ = -0.019                     # tool point this far below the rod's axis (fingertips under the rod, hand above the folds)
PUSH_OFF = 0.022                     # tool point beside the leading ring's middle as it comes down and in
PUSH_LAG = 0.008                     # the pushed ring's middle stays this far inside the tool point (measured in the sim)
RING_END = 0.007                     # each leading ring's middle ends this far from the middle (clips 2 mm apart)
OVER_DZ = 0.088                      # over the stack: this far above the rod's axis
# left-arm joint key poses (rad): the harness start, hand raised over the shoulder, hand over the stack
TUCK_Q = (0.0, 0.42, 1.6, -0.19, 0.0, 0.0)
OVER_Q = (-1.743, 0.788, 1.168, -0.379, -1.394, -1.569)
VIA_STEPS = 30                       # tool way points per joint segment (keeps the IK within 0.5 mm of them)
VIA_S = (3.0, 4.0)                   # seconds to raise the hand, to swing it over the stack
LEFT_WAIT = 6.5                      # the left hand waits this long over its stack
REST = (0.140, 0.120, 0.430)         # left rest pose over its panel (mirrored for the right)


def episode():
    return _RUN["episode"]


def _push_rot(side):
    w = math.radians(PUSH_YAW)
    rot = ck.grip_matrix((math.cos(w), -math.sin(w), 0.0), (0.0, 0.0, 1.0))
    return _mirror((0.0, 0.0, 0.0), rot, side)[1]


def _live_ring(name):
    env = _RUN["env"]
    if env is None:
        return np.asarray(RING0[name], float)
    obj = env.unwrapped.scene[name]
    return ck._np(obj.data.root_pos_w)[0] - ck._np(env.unwrapped.scene.env_origins)[0]


def _lazy(fn):
    cache = {}

    def f():
        key = _RUN["episode"]
        if key not in cache:
            cache[key] = tuple(float(v) for v in fn())
        return cache[key]
    return f


def _mirror(pos, rot, side):
    """A left-arm tool pose (world) as the right arm's mirror image: y -> -y, rotation M R diag(-1, 1, 1) (the
    right arm's joints are the left's with joints 1, 5 and 6 negated)."""
    if side == "left":
        return tuple(pos), rot
    return (pos[0], -pos[1], pos[2]), np.diag([1.0, -1.0, 1.0]) @ rot @ np.diag([-1.0, 1.0, 1.0])


def via_ways(side):
    """Start pose -> hand raised over the shoulder -> over the stack, as tool way points along the joint path."""
    from yam_arena import ik, station
    from yam_arena.constants import READY
    base = station.BASES["left"]
    keys = [np.array(READY, float), np.array(TUCK_Q), np.array(OVER_Q)]
    durs = VIA_S
    ways = []
    for k in range(1, len(keys)):
        for j in range(1, VIA_STEPS + 1):
            q = keys[k - 1] + (keys[k] - keys[k - 1]) * j / VIA_STEPS
            pos, rot = ik.tcp_pose(q, base)
            pos, rot = _mirror(np.asarray(pos, float), np.asarray(rot, float), side)
            ways.append(ck.Way(durs[k - 1] / VIA_STEPS, pos, CLOSED, "raise" if k == 1 else "swing", rot=rot))
    return ways


def push_ways(side, wait=0.0):
    s = 1.0 if side == "left" else -1.0
    lead = "ring_l0" if side == "left" else "ring_r0"
    rot = _push_rot(side)
    ring = _lazy(lambda: _live_ring(lead))
    x, z = ROD_X + PUSH_DX, TOP + ROD_Z + PUSH_DZ
    y_in = lambda: ring()[1] + s * PUSH_OFF     # noqa: E731
    over = via_ways(side)
    if wait > 0:
        over.append(ck.Way(wait, over[-1].pos, CLOSED, "wait", rot=rot))
    return over + [
        ck.Way(1.2, _lazy(lambda: (ROD_X - 0.030, y_in(), TOP + ROD_Z + OVER_DZ)), CLOSED, "over clip", rot=rot,
               episode=episode),
        ck.Way(1.6, _lazy(lambda: (ROD_X - 0.030, y_in(), z + 0.012)), CLOSED, "down", rot=rot, episode=episode),
        ck.Way(1.2, _lazy(lambda: (x, y_in(), z)), CLOSED, "under rod", rot=rot, episode=episode),
        ck.Way(0.8, _lazy(lambda: (x, ring()[1] + s * (PUSH_LAG + 0.004), z)), CLOSED, "touch", rot=rot,
               episode=episode),
        ck.Way(4.5, (x, s * (RING_END + PUSH_LAG), z), CLOSED, "push", rot=rot),
        ck.Way(0.8, (x, s * (RING_END + PUSH_LAG + 0.010), z), CLOSED, "ease off", rot=rot),
        ck.Way(1.2, (ROD_X - 0.040, s * (RING_END + PUSH_LAG + 0.010), z + 0.015), CLOSED, "back", rot=rot),
        ck.Way(1.5, (REST[0], s * REST[1], TOP + REST[2]), CLOSED, "up", rot=rot),
        ck.Way(1.0 + (LEFT_WAIT - wait), (REST[0], s * REST[1], TOP + REST[2]), CLOSED, "rest", rot=rot),
    ]


def parked_ways(side):
    """The wrong-way copy's idle hand: stays at its start pose, fingers open."""
    from yam_arena import ik, station
    from yam_arena.constants import READY
    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
    pos, rot = _mirror(np.asarray(pos, float), np.asarray(rot, float), side)
    return [ck.Way(sum(w.duration for w in push_ways("left", wait=LEFT_WAIT)), pos, OPEN, "parked", rot=rot)]


def demo():
    """Right hand first, the left once the right is backing out; with DEMO_ONE_HAND the right hand stays at its start
    pose (wrong-way copy)."""
    right = parked_ways("right") if DEMO_ONE_HAND else push_ways("right")
    return {"left": push_ways("left", wait=LEFT_WAIT), "right": right}


# ================================================================================================ diagnostics
_LOG = {"step": -1}


def _diag(objects, t):
    step = _RUN["step"]
    if step == _LOG["step"] or _RUN["t"] is None:
        return
    _LOG["step"] = step
    if not (step in (1, 2, 10, 20, 40) or step % 10 == 0):
        return
    rule = ck.grip_rule()
    st = "" if rule is None else " | ".join(f"{k[1][0]}{k[0][-1]}:{'HELD' if v.get('attached') else 'ok' if v.get('ok') else 'no'}"
                                           f" p={v.get('pinch', 0):.2f} g={v.get('gap', 0) * 1e3:.2f} b={v.get('between', '-')}"
                                           for k, v in sorted(rule.status.items()) if v.get('attached') or v.get('between'))
    c = ring_centres(objects)
    e = leading_edges(objects)
    ry = " ".join(f"{n[-2:]}:{c[n][1]:+.3f}" for n in RING_NAMES)
    tilt = max(objects[n].tilt_deg() for n in RING_NAMES if hasattr(objects[n], "tilt_deg"))
    lz = e["left"]
    print(f"CURTAIN ep={_RUN['episode']} step={step} t={_RUN['t']:.2f} score={t['score']} gap={t['gap'] * 1e3:.0f}mm "
          f"drawn={t['drawn'][0] * 1e3:.0f}/{t['drawn'][1] * 1e3:.0f}mm on_rod={t['on_rod']} over_rod={t['over_rod']} "
          f"move={t['move'] * 1e3:.1f}mm released={t['released']} | rings {ry} | max ring tilt {tilt:.1f} | "
          f"L edge y top {lz[0, 1]:+.3f} mid {lz[len(lz) // 2, 1]:+.3f} hem {lz[-1, 1]:+.3f} | R edge y top "
          f"{e['right'][0, 1]:+.3f} hem {e['right'][-1, 1]:+.3f} | {st}", flush=True)
