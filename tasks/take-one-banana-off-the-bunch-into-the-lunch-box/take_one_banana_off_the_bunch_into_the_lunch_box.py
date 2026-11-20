"""Take one banana off the bunch and put it in the lunch box.

On the table lies a hand of four ripe Cavendish bananas (all yellow, colour stage 6), each about 19.5 cm along its
outer curve, 3.6 cm thick and 150 g, joined by their short stems to the brown crown (the cut cushion of the bunch),
curved side down. To the right stands an open Sistema Lunch 2 L box (235 x 170 x 80 mm, clear base) with its blue lid
lying flat beyond it. Each stem snaps when it is bent or twisted with about 0.6 N m at its root (about 5 N lifting
the banana 14 cm out), or pulled with 80 N; pulled on its own, the light bunch slides and turns with the banana before
the stem gives. Squeezing a banana with more than 40 N for 0.2 s dents it. Metres; x forward from the arm bases,
y left, z up.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts  # noqa: E402,F401  (the compound-parts kit)
import yam_mesh  # noqa: E402,F401
from yam_parts import Part, box, cyl  # noqa: E402
sys.modules.pop("bunch_kit", None)  # bind the bunch module to the kit modules loaded with this task file
import bunch_kit as bk  # noqa: E402
from bunch_kit import BunchPart  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None
TEST_SQUEEZE = None  # offline layouts: {"banana_1": peak squeeze N held 0.2 s}
TEST_CROWN_START = None  # offline layouts: [x, y, z] or [x, y, z, yaw_deg] where the crown started
DIAG = False  # GPU diagnostic copies print BANANA_DIAG lines
DEMO_WRONG = ""  # wrong-way copy: "pull" drags the banana straight out along its stem instead of bending it


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
        tcp = ik.tcp_pose(q[arm_ids], station.BASES[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


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
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
MM = 0.001
DEG = math.pi / 180

# ---------------------------------------------------------------- the bananas (one hand of four)
# Ripe Cavendish, colour stage 6. One banana: centre line 20.0 cm from the stem root to the tip (2 cm of stem, 18 cm
# of fruit, about 19.5 cm along the outer curve: a USDA "medium", 7 to 7 7/8 in); 3.6 cm thick at the middle; 150 g
# (estimates within the published medium range). Curved on a 20 cm radius (estimate), lying curved side down.
STATIONS = [(0.000, 0.015), (0.020, 0.016), (0.040, 0.025), (0.062, 0.032), (0.090, 0.036), (0.125, 0.036),
            (0.158, 0.033), (0.182, 0.026), (0.194, 0.016), (0.200, 0.008)]
BEND_R, S_LOW = 0.20, 0.085
ROLL = 35.0 * math.pi / 180  # the hand lies partly on its side: each curve's plane leans 35 deg from upright
STEM_GREEN = (0.42, 0.44, 0.20)  # greenish-brown stem
NECK_GREEN = (0.66, 0.70, 0.26)
PEEL = (0.97, 0.82, 0.20)
TIP_BROWN = (0.18, 0.12, 0.06)
PIECE_COLORS = [STEM_GREEN, NECK_GREEN, PEEL, PEEL, PEEL, PEEL, PEEL, PEEL, TIP_BROWN]
BANANA_MASS = 0.150
BANANA_FRICTION = (0.70, 0.60)  # peel on nitrile-covered pads and on the table (estimate)
GRIP_S = 0.150  # the demo pinches the target here, 15 cm from its stem root
BREAK_TORQUE, BREAK_FORCE = 0.60, 80.0  # N m and N at the stem root (estimates, see README)
_PTS = bk.fruit_points(STATIONS, BEND_R, S_LOW, roll=ROLL)
_AXIS = bk.fruit_axis(STATIONS, BEND_R, S_LOW, ROLL)
STEM_Z = -float(_PTS[:, 2].min()) + 0.4 * MM  # the stem root's height when the banana rests on its belly

# The crown: the cut cushion that joins the four stems, 9.0 x 3.2 x 5.0 cm (estimate), 150 g, with the cut stub of
# the bunch's main stalk on its back.
CROWN_XY = (0.130, 0.140)
CROWN_SIZE = (0.090, 0.032, 0.050)
CROWN_MASS = 0.150
CROWN_BROWN = (0.40, 0.33, 0.17)
CUT_FACE = (0.80, 0.74, 0.52)
STEM_X = (-0.035, -0.012, 0.010, 0.032)  # stem roots along the crown's front face
FAN_DEG = (-24.0, -5.0, 6.0, 18.0)  # each banana turned from straight ahead (-y), + toward +x
TARGET = "banana_1"  # the demo takes the outer banana nearest the robot


def make_crown():
    sx, sy, sz = CROWN_SIZE
    body = bk.rounded_block("cushion", CROWN_SIZE, 0.011, CROWN_BROWN, z0=0.0)
    shapes = [cyl((0.0, sy / 2 + 0.006, 0.026), 0.015, 0.012, axis="y", color=(0.46, 0.40, 0.20)),
              cyl((0.0, sy / 2 + 0.0125, 0.026), 0.0125, 0.001, axis="y", color=CUT_FACE)]
    return BunchPart("crown", (*CROWN_XY, TOP + 0.4 * MM), shapes, mass=CROWN_MASS, color=CROWN_BROWN,
                     friction=(0.6, 0.5), contact_offset=0.0015, meshes=(body,))


def make_banana(i):
    name = f"banana_{i + 1}"
    meshes, grip = bk.fruit_meshes(STATIONS, BEND_R, S_LOW, PIECE_COLORS, grip_s=GRIP_S, name="peel", roll=ROLL)
    root = (CROWN_XY[0] + STEM_X[i], CROWN_XY[1] - CROWN_SIZE[1] / 2, TOP + STEM_Z)
    return BunchPart(name, root, (), mass=BANANA_MASS, color=PEEL, yaw=FAN_DEG[i] * DEG,
                     friction=BANANA_FRICTION, contact_offset=0.0015, grip=grip, meshes=tuple(meshes))


CROWN = make_crown()
BANANAS = [make_banana(i) for i in range(4)]
STEMS = {b.name: bk.stem(f"s{i + 1}", CROWN, b, BREAK_TORQUE, BREAK_FORCE) for i, b in enumerate(BANANAS)}

# ---------------------------------------------------------------- the lunch box and its lid
# Sistema Lunch 2 L rectangular (maker: 235 x 170 x 80 mm closed, 212 g with lid). Base 70 mm deep with a 4 mm rim
# lip; walls and floor 3 mm, clear plastic (estimates); lid 12 mm with four clips, set flat beyond the box.
BOX_XY = (0.270, -0.215)
BOX_L, BOX_W, BOX_H = 0.235, 0.170, 0.070  # along y, along x, base height
WALL, FLOOR, LIP = 0.003, 0.003, 0.004
BOX_MASS, LID_MASS = 0.150, 0.062
CLEAR = (0.70, 0.82, 0.90)  # clear polypropylene renders as a pale blue-grey (no transparency)
LID_BLUE = (0.16, 0.45, 0.82)
CLIP_BLUE = (0.08, 0.30, 0.66)
INNER_X = BOX_W / 2 - LIP - WALL  # half inner sizes below the lip
INNER_Y = BOX_L / 2 - LIP - WALL


def make_box():
    hx, hy = BOX_W / 2 - LIP, BOX_L / 2 - LIP  # the body's outer half sizes; the lip stands out LIP more
    shapes = [box((0, 0, FLOOR / 2), (2 * hx, 2 * hy, FLOOR)),
              box((hx - WALL / 2, 0, BOX_H / 2), (WALL, 2 * hy, BOX_H)),
              box((-hx + WALL / 2, 0, BOX_H / 2), (WALL, 2 * hy, BOX_H)),
              box((0, hy - WALL / 2, BOX_H / 2), (2 * hx - 2 * WALL, WALL, BOX_H)),
              box((0, -hy + WALL / 2, BOX_H / 2), (2 * hx - 2 * WALL, WALL, BOX_H)),
              box((hx + LIP / 2, 0, BOX_H - 0.003), (LIP, 2 * hy + 2 * LIP, 0.006)),
              box((-hx - LIP / 2, 0, BOX_H - 0.003), (LIP, 2 * hy + 2 * LIP, 0.006)),
              box((0, hy + LIP / 2, BOX_H - 0.003), (2 * hx, LIP, 0.006)),
              box((0, -hy - LIP / 2, BOX_H - 0.003), (2 * hx, LIP, 0.006))]
    return Part("lunch_box", (*BOX_XY, TOP + 0.4 * MM), shapes, mass=BOX_MASS, color=CLEAR, friction=(0.5, 0.4),
                contact_offset=0.0015)


LID_XY = (0.465, -0.215)


def make_lid():
    hx, hy, t, skirt = BOX_W / 2, BOX_L / 2, 0.003, 0.012
    shapes = [box((0, 0, skirt - t / 2), (2 * hx, 2 * hy, t)),
              box((hx - 0.002, 0, skirt / 2), (0.004, 2 * hy, skirt)),
              box((-hx + 0.002, 0, skirt / 2), (0.004, 2 * hy, skirt)),
              box((0, hy - 0.002, skirt / 2), (2 * hx - 0.008, 0.004, skirt)),
              box((0, -hy + 0.002, skirt / 2), (2 * hx - 0.008, 0.004, skirt)),
              box((0, 0, skirt + 0.0008), (2 * hx - 0.030, 2 * hy - 0.030, 0.0016), color=(0.42, 0.66, 0.92))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            shapes.append(box((sx * (hx + 0.006), sy * 0.055, skirt / 2), (0.012, 0.040, skirt), color=CLIP_BLUE))
    return Part("lid", (*LID_XY, TOP + 0.4 * MM), shapes, mass=LID_MASS, color=LID_BLUE, friction=(0.5, 0.4),
                contact_offset=0.0015)


BOX = make_box()
LID = make_lid()
OBJECTS = [CROWN, *BANANAS, BOX, LID]
PRESSED_PARTS = ("crown",)  # the left hand presses the crown down on the table

# ---------------------------------------------------------------- scoring
# Judged at the end (a photo and a ruler): exactly one banana off the bunch by its stem; where it lies; how hard it
# was squeezed (a dent); how far the crown was dragged or turned. score = min(placement, damage, crown) after the gates.
DENT_N, CRUSH_N, DENT_S = 40.0, 48.0, 0.2  # squeeze over 40 N (dent over 5 mm) / at the 50 N limit (over 8 mm)
SPEED = 0.01
DRAG_TIERS = ((0.010, 1.0), (0.030, 0.75), (0.060, 0.5))  # crown moved up to (m): score; more: 0.25
TURN_DEG = 10.0  # crown turned more than this about the vertical: at most 0.75
OFF_TABLE = 0.05  # anything this far below the table top has fallen off it


def fruit_world(state):
    """Surface points of a banana in the world, from its state."""
    return np.asarray(state.origin, float) + _PTS @ state.rotation().T


def box_frame_points(points, box_state):
    return (points - np.asarray(box_state.origin, float)) @ box_state.rotation()


def placement_of(fruit, box_state):
    """'inside': every point within the inner walls and below the rim; 'over': some point over the box's outline;
    'table': otherwise."""
    p = box_frame_points(fruit_world(fruit), box_state)
    inside = bool(np.all(np.abs(p[:, 0]) <= INNER_X) and np.all(np.abs(p[:, 1]) <= INNER_Y)
                  and np.all(p[:, 2] <= BOX_H) and np.all(p[:, 2] >= FLOOR - 2 * MM))
    over = bool(np.any((np.abs(p[:, 0]) <= BOX_W / 2) & (np.abs(p[:, 1]) <= BOX_L / 2) & (p[:, 2] >= -2 * MM)))
    return "inside" if inside else "over" if over else "table"


def _hands_now():
    """(TCPs, squeeze per arm) read once per scorer call; ({}, {}) offline or if the read fails."""
    env = _RUN["env"]
    if env is None:
        return {}, {}
    try:
        return grippers() or {}, bk.finger_squeeze(env)
    except Exception as e:  # noqa: BLE001
        print(f"BANANA_SQUEEZE read failed: {e!r}", flush=True)
        return {}, {}


def squeeze_on(name, state, hands, squeeze):
    """Squeeze (N) a hand is putting on this banana now (0 when no hand pinches it)."""
    if TEST_SQUEEZE is not None:
        return float(TEST_SQUEEZE.get(name, 0.0))
    worst = 0.0
    R, o = state.rotation(), np.asarray(state.origin, float)
    for arm, (tcp, _) in hands.items():
        q = R.T @ (np.asarray(tcp, float) - o)  # the TCP in the banana's frame
        near = any(np.linalg.norm(q - bk.rolled_frame(s, BEND_R, S_LOW, ROLL)[0]) <= d / 2 + 0.010
                   for s, d in STATIONS)
        if near and arm in squeeze and squeeze[arm][0] > 0.010:
            worst = max(worst, squeeze[arm][1])
    return worst


def _yaw_of(state):
    R = state.rotation()
    return math.atan2(R[1, 0], R[0, 0])


def _crown_start(crown):
    """(position, yaw) of the crown at the first scored step of the episode (its tape mark)."""
    if TEST_CROWN_START is not None:
        s = list(TEST_CROWN_START)
        return np.asarray(s[:3], float), (s[3] * DEG if len(s) > 3 else CROWN.yaw)
    if _RUN["t"] is None:
        return np.asarray(CROWN.pos, float), CROWN.yaw
    _episode_memory()
    if "crown start" not in _EVER:
        _EVER["crown start"] = (np.asarray(crown.origin, float).copy(), _yaw_of(crown))
    return _EVER["crown start"]


def released(state):
    hands = grippers()
    if hands is None:
        return True
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= RELEASE_CLEAR
               for tcp, _ in hands.values())


def score_terms(objects):
    crown, lunch_box = objects["crown"], objects["lunch_box"]
    off = [b.name for b in BANANAS if bk.detached(STEMS[b.name], crown, objects[b.name])]
    dent, crush = {}, {}
    hands, squeeze = _hands_now()
    for b in BANANAS:
        f = squeeze_on(b.name, objects[b.name], hands, squeeze)
        dent[b.name] = ever(f"dent {b.name}", held_for(f"over {DENT_N} {b.name}", f > DENT_N) >= DENT_S)
        crush[b.name] = ever(f"crush {b.name}", held_for(f"over {CRUSH_N} {b.name}", f >= CRUSH_N) >= DENT_S)
    fallen = [n for n in ("crown", *(b.name for b in BANANAS))
              if np.asarray(objects[n].low)[2] < TOP - OFF_TABLE]
    start, yaw0 = _crown_start(crown)
    drag = float(np.linalg.norm((np.asarray(crown.origin, float) - start)[:2]))
    turn = abs(math.degrees(math.remainder(_yaw_of(crown) - yaw0, 2 * math.pi)))
    t = {"off": off, "dent": dent, "crush": crush, "fallen": fallen, "drag": drag, "turn": turn, "placement": None,
         "released": None, "still": None}
    if len(off) == 1:
        fruit = objects[off[0]]
        t["placement"] = placement_of(fruit, lunch_box)
        t["released"] = released(fruit)
        t["still"] = bool(np.linalg.norm(fruit.lin_vel) < SPEED)
    if DIAG:
        _diag(objects, t)
    return t


def score(objects):
    t = score_terms(objects)
    if len(t["off"]) != 1 or t["fallen"]:
        return 0.0  # nothing off yet, two or more off, or fruit/crown fallen off the table
    if any(t["dent"][b.name] for b in BANANAS if b.name not in t["off"]):
        return 0.0  # a banana left on the bunch was dented
    name = t["off"][0]
    if t["placement"] == "inside" and t["released"] and t["still"]:
        place = 1.0
    elif t["placement"] in ("inside", "over"):
        place = 0.5
    else:
        place = 0.25
    damage = 0.25 if t["crush"][name] else 0.5 if t["dent"][name] else 1.0
    moved = next((v for limit, v in DRAG_TIERS if t["drag"] <= limit), 0.25)
    if t["turn"] > TURN_DEG:
        moved = min(moved, 0.75)
    return min(place, damage, moved)


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


def _diag(objects, t):
    step = _RUN["step"]
    if _RUN["t"] is None or step == _DIAG["step"] or step % 4:
        return
    _DIAG["step"] = step
    crown = objects["crown"]
    errs = " ".join(f"{b.name[-1]}=({bk.stem_error(STEMS[b.name], crown, objects[b.name])[0]:.1f},"
                    f"{bk.stem_error(STEMS[b.name], crown, objects[b.name])[1]:.1f})" for b in BANANAS)
    hands = grippers() or {}
    sq = bk.finger_squeeze(_RUN["env"]) if _RUN["env"] is not None else {}
    tcp = " ".join(f"{a[0]}=({','.join(f'{v * 1000:.0f}' for v in hands[a][0])};g{sq.get(a, (0, 0))[0] * 1000:.0f}"
                   f"mm;{sq.get(a, (0, 0))[1]:.1f}N)" for a in hands)
    tg = objects[TARGET]
    o = np.asarray(tg.origin, float)
    print(f"BANANA_DIAG t={_RUN['t']:.2f} off={t['off']} drag={t['drag'] * 1000:.1f} turn={t['turn']:.1f} "
          f"err[mm,deg] {errs} "
          f"tgt=({o[0] * 1000:.0f},{o[1] * 1000:.0f},{(o[2] - TOP) * 1000:.0f}) place={t['placement']} "
          f"rel={t['released']} still={t['still']} {tcp} F={_hand_force('right')} FL={_hand_force('left')}",
          flush=True)


def _hand_force(arm):
    """The arm's force on what it holds (N, world x, y, z) from its joints' PD error: J^T F = K (q_target - q)."""
    env = _RUN["env"]
    if env is None:
        return "-"
    try:
        from yam_arena import ik, station
        from yam_arena.constants import ARM_JOINTS, SHOULDER, WRIST
        k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        art = env.unwrapped.scene[arm + "_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        q = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0][ids].detach().cpu().numpy().astype(float)
        qt = getattr(art.data.joint_pos_target, "torch", art.data.joint_pos_target)[0][ids]
        qt = qt.detach().cpu().numpy().astype(float)
        tau = k * (qt - q)
        p0, r0 = ik.tcp_pose(q, station.BASES[arm])
        jac = np.zeros((6, 6))
        for j in range(6):
            dq = q.copy()
            dq[j] += 1e-5
            p1, r1 = ik.tcp_pose(dq, station.BASES[arm])
            w = r1 @ r0.T
            jac[:3, j] = (p1 - p0) / 1e-5
            jac[3:, j] = np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]]) / 2 / 1e-5
        f = np.linalg.lstsq(jac.T, tau, rcond=None)[0]
        return f"({f[0]:.1f},{f[1]:.1f},{f[2]:.1f})"
    except Exception as e:  # noqa: BLE001
        return f"err:{type(e).__name__}"


# ---------------------------------------------------------------- demo
# The left hand comes down on the crown with its fingers closed and presses it to the table (about 15 N in
# simulation). The right hand comes down over the outer banana nearest the robot with its fingers either side of
# it, 15 cm out from the stem, pinches it (about 15 N) and lifts: the banana bends up about its stem root until the
# stem snaps (about 5 N at the hand), the crown and the other bananas stay on the table. The right hand lifts the
# banana clear, turns it along the box and lowers it in, lets go and backs away; the left hand lets go of the crown
# once the banana is clear.
# The demo reads the banana's, the crown's and the box's poses (scripted demos may read simulator state); it never
# moves an object.
PAD_TIP = 0.013
OPEN_R = -0.026  # pads 52 mm apart round the 34 mm section: about 9 mm clear each side
CROWN_PRESS = 0.015  # the closed fingertips aimed this far into the crown's top
PARK = {"left": np.array([0.160, 0.240, TOP + 0.160]), "right": np.array([0.160, -0.240, TOP + 0.160])}
CARRY_Z = TOP + 0.150
PLACE_GAP = 0.015  # the banana's lowest point is let go this far above the box floor
_READS = {"episode": None, "values": {}}


def _now_state(name):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, [name])[name]
    except Exception as e:  # noqa: BLE001
        print(f"BANANA_READ {name} failed: {e!r}", flush=True)
        return None


def _cached(key, fn):
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    if key not in _READS["values"]:
        _READS["values"][key] = fn()
    return _READS["values"][key]


def _nominal(spec):
    return bk._Pose(spec.pos, (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)))


def _grip_yaw(axis_xy, prefer):
    """Yaw of a top-down grip whose fingers close across a horizontal direction axis_xy, nearest to prefer."""
    c = np.array([axis_xy[1], -axis_xy[0]])  # closing axis: across the banana
    base = math.atan2(-c[0], c[1])
    options = [base + k * math.pi for k in (-2, -1, 0, 1, 2)]
    return min(options, key=lambda a: abs(a - prefer))


def grasp_plan(state=None):
    """(TCP point, yaw, pads' closing target, unit horizontal direction toward the tip) for the pinch on the target
    banana, from its pose."""
    st = state or _nominal(BANANAS[0])
    c, tangent, _ = bk.rolled_frame(GRIP_S, BEND_R, S_LOW, ROLL)
    R, o = st.rotation(), np.asarray(st.origin, float)
    point = o + R @ c
    along = (R @ tangent)[:2]
    along = along / np.linalg.norm(along)
    d = float(np.interp(GRIP_S, [s for s, _ in STATIONS], [d for _, d in STATIONS]))
    return point, _grip_yaw(along, 1.2), -(d / 2 - 0.003), np.r_[along, 0.0]


def _grasp():
    def read():
        st = _now_state(TARGET)
        plan = grasp_plan(st)
        if st is not None:
            point, yaw = plan[0], plan[1]
            print(f"BANANA_READ grasp t={_RUN['t']:.2f} point_mm=({point[0] * 1000:.1f},{point[1] * 1000:.1f},"
                  f"{(point[2] - TOP) * 1000:.1f}) yaw_deg={math.degrees(yaw):.1f}", flush=True)
        return plan
    return _cached("grasp", read)


def _crown_top():
    def read():
        st = _now_state("crown")
        o = np.asarray(st.origin if st is not None else CROWN.pos, float)
        return np.array([o[0] + 0.012, o[1] + 0.002, o[2] + CROWN_SIZE[2]])
    return _cached("crown", read)


def place_plan(banana=None, tcp=None, yaw=None, box_state=None):
    """Hand (point, yaw) that sets the held banana into the box: its middle over the box's centre, its length along
    the box, its lowest point PLACE_GAP over the floor. From the banana's pose relative to the hand."""
    if banana is None:  # offline: the banana held as it was pinched, lifted to the carry height
        point, yaw = grasp_plan()[:2]
        banana = _nominal(BANANAS[0])
        lift = np.array([0.0, 0.0, CARRY_Z - point[2]])
        banana = bk._Pose(np.asarray(banana.origin) + lift, banana.quat_xyzw)
        tcp = point + lift
    bx = box_state or bk._Pose(BOX.pos, (0.0, 0.0, 0.0, 1.0))
    pts = fruit_world(banana)
    mid = (pts.max(axis=0) + pts.min(axis=0)) / 2
    root, tip = (np.asarray(banana.origin, float) + banana.rotation() @ p for p in _AXIS)
    axis = tip - root
    a = math.atan2(axis[1], axis[0])
    box_axis = math.atan2(*(bx.rotation()[:2, 1][::-1]))  # the box's long side (its y) in the world
    turns = [box_axis - a + k * math.pi for k in (-2, -1, 0, 1, 2)]
    d = min(turns, key=abs)
    Rz = bk.rot_z(d)
    off = mid - np.asarray(tcp, float)
    new_off = Rz @ off
    floor = float(np.asarray(bx.origin)[2]) + FLOOR
    low = float(pts[:, 2].min()) - float(np.asarray(tcp)[2])
    target = np.asarray(bx.origin, float)[:2] - new_off[:2]
    return np.array([target[0], target[1], floor + PLACE_GAP - low]), yaw + d


def _place():
    def read():
        st = _now_state(TARGET)
        bx = _now_state("lunch_box")
        hands = grippers()
        if st is None or bx is None or hands is None:
            return place_plan()
        yaw = _grasp()[1]
        point, new_yaw = place_plan(st, hands["right"][0], yaw, bx)
        print(f"BANANA_READ place t={_RUN['t']:.2f} hand_mm=({point[0] * 1000:.1f},{point[1] * 1000:.1f},"
              f"{(point[2] - TOP) * 1000:.1f}) yaw_deg={math.degrees(new_yaw):.1f}", flush=True)
        return point, new_yaw
    return _cached("place", read)


class Live(Waypoint):
    """A waypoint worked out when it is first played, from the reads above."""

    def __init__(self, duration, where, finger, label, yaw_of=None):
        self.duration, self.label = duration, label
        self._where, self._finger, self._yaw_of = where, finger, yaw_of

    @property
    def pos(self):
        return tuple(float(v) for v in self._where())

    @property
    def finger(self):
        return float(self._finger() if callable(self._finger) else self._finger)

    @property
    def yaw(self):
        return float(self._yaw_of() if self._yaw_of else 0.0)


def demo():
    g = lambda dz=0.0, dy=0.0: _grasp()[0] + np.array([0.0, dy, dz])  # noqa: E731
    gyaw = lambda: _grasp()[1]  # noqa: E731
    pinch = lambda: _grasp()[2]  # noqa: E731
    over_box = lambda: np.r_[_place()[0][:2], CARRY_Z]  # noqa: E731
    in_box = lambda: _place()[0]  # noqa: E731
    pyaw = lambda: _place()[1]  # noqa: E731
    right = [Live(2.5, lambda: g(0.080), OPEN_R, "over the outer banana", gyaw),
             Live(1.5, lambda: g(), OPEN_R, "fingers down either side of it", gyaw),
             Live(1.0, lambda: g(), pinch, "pinch it 15 cm out from the stem", gyaw)]
    if DEMO_WRONG == "pull":  # one hand pulls the banana straight out along its length; nothing holds the crown
        pulled = lambda: _grasp()[0] + 0.080 * _grasp()[3]  # noqa: E731  (along the banana, toward its tip)
        right += [Live(3.0, pulled, pinch, "pull it straight out along its length", gyaw),
                  Live(0.5, pulled, pinch, "hold", gyaw),
                  Live(0.8, pulled, OPEN_R, "let go", gyaw),
                  Live(1.2, lambda: pulled() + np.array([0.0, 0.0, 0.100]), OPEN_R, "lift the fingers out", gyaw),
                  Live(2.0, lambda: PARK["right"], OPEN_R, "park", gyaw),
                  Live(3.0, lambda: PARK["right"], OPEN_R, "hold clear", gyaw)]
        return {"left": [Live(sum(w.duration for w in right), lambda: PARK["left"], FINGER_OPEN, "wait")],
                "right": right}
    else:
        right += [Live(2.5, lambda: g(0.045), pinch, "bend it up until the stem snaps", gyaw),
                  Live(1.5, lambda: g(CARRY_Z - _grasp()[0][2], -0.030), pinch, "lift it clear of the bunch", gyaw)]
    lifted = right[-1]
    right += [Live(0.5, lambda: lifted.pos, pinch, "steady", gyaw),
              Live(2.5, over_box, pinch, "carry it over the lunch box, turned along it", pyaw),
              Live(1.5, in_box, pinch, "lower it into the box", pyaw),
              Live(0.8, in_box, OPEN_R, "let go", pyaw),
              Live(1.2, lambda: in_box() + np.array([0.0, 0.0, 0.100]), OPEN_R, "lift the fingers out", pyaw),
              Live(2.0, lambda: PARK["right"], OPEN_R, "park", pyaw),
              Live(3.0, lambda: PARK["right"], OPEN_R, "hold clear", pyaw)]
    t_clear = sum(w.duration for w in right[:5])  # the banana is lifted clear of the bunch
    top = lambda dz: _crown_top() + np.array([0.0, 0.0, PAD_TIP + dz])  # noqa: E731
    left = [Live(2.5, lambda: top(0.050), FINGER_CLOSED, "over the crown, fingers closed"),
            Live(1.5, lambda: top(0.002), FINGER_CLOSED, "fingertips on the crown"),
            Live(1.0, lambda: top(-CROWN_PRESS), FINGER_CLOSED, "press the crown to the table")]
    left.append(Live(t_clear - sum(w.duration for w in left), lambda: top(-CROWN_PRESS), FINGER_CLOSED,
                     "hold the bunch down while the stem is bent"))
    left += [Live(1.2, lambda: top(0.060), FINGER_CLOSED, "lift off the crown"),
             Live(2.0, lambda: PARK["left"], FINGER_OPEN, "park")]
    total = sum(w.duration for w in right)
    left.append(Live(max(total - sum(w.duration for w in left), 0.5), lambda: PARK["left"], FINGER_OPEN, "wait"))
    return {"left": left, "right": right}


# ---------------------------------------------------------------- offline test layouts (states.yaml)
def _quat_z(a):
    return [0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]


def banana_at(x, y, z_lift=0.0, yaw_deg=None, i=0):
    """[pos, quat] of banana i lying curved side down with its stem root at (x, y), lifted z_lift."""
    yaw = FAN_DEG[i] * DEG if yaw_deg is None else yaw_deg * DEG
    return [x, y, TOP + STEM_Z + z_lift], _quat_z(yaw)


def banana_in_box(dx=0.0, dy=0.0, dz=0.0):
    """[pos, quat] of a banana lying in the box along its length, centred (box at its start)."""
    yaw = 0.0  # the stem root toward +y: the banana heads -y, along the box
    pts = _PTS @ bk.rot_z(yaw).T
    mid = (pts.max(axis=0) + pts.min(axis=0)) / 2
    root = np.array([BOX_XY[0] + dx, BOX_XY[1] + dy, 0.0]) - np.r_[mid[:2], 0.0]
    root[2] = TOP + FLOOR + 0.4 * MM - pts[:, 2].min() + dz
    return [float(v) for v in root], _quat_z(yaw)
