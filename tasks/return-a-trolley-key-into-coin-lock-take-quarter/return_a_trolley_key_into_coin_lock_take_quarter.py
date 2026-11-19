"""Return a supermarket trolley and get the quarter back: pick up the chain key of the trolley in front, push it home
into the coin lock of the returning trolley, then take the quarter the lock pushes out and lay it on the mat.

Two shopping-cart coin locks (lock body 90 x 70 x 68 mm, 260 mm stainless chain, coins 23-28 mm) sit on two 300 mm
lengths of 25 mm handle tube, nested 178 mm apart (a tight-nesting cart's nesting distance), on low stands. Metres;
x forward from the arm bases, y left, z up. Lock frame: origin on the tube's axis under the lock's middle, world
aligned; the key goes in along +y through the lock's -y end; the coin pusher comes out of the lock's -x side.
The lock mechanism and chain are modelled in trolley_lock.py.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import trolley_lock as TL  # noqa: E402
from trolley_lock import KEY_GRIP, KEY_TIP, KEY_BACK, Z_T  # noqa: E402
from yam_parts import Part, box  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
DIAG = False  # GPU diagnostic copies print TL_DIAG lines
DEMO_WAY = "home"  # "home": key pushed home; "short": wrong way, the key slid 17 mm in and left there
DEMO_PACE = 1.0
TEST_GRIPPERS = None
PROBE_OFFSET = None  # probe copies: per-episode (dx, dy) mm added to the key pick, to test the reads

TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz


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

# ---------------------------------------------------------------- layout
NEAR_XY = np.array([0.150, -0.110])  # the returning trolley's lock (near the arms)
FRONT_XY = NEAR_XY + np.array([TL.NEST, 0.0])  # the trolley in front, 178 mm further on
MAT_XY = np.array([0.060, -0.040])
MAT_SIZE, MAT_T, MAT_MASS = 0.110, 0.002, 0.035
RUBBER = (0.08, 0.08, 0.09)

NEAR = TL.near_lock("near_lock", (NEAR_XY[0], NEAR_XY[1], TOP + Z_T))
FRONT = TL.front_lock("front_lock", (FRONT_XY[0], FRONT_XY[1], TOP + Z_T))
COIN = TL.quarter("quarter", NEAR)
MAT = Part("mat", (MAT_XY[0], MAT_XY[1], TOP + MAT_T / 2), (box((0, 0, 0), (MAT_SIZE, MAT_SIZE, MAT_T)),),
           mass=MAT_MASS, color=RUBBER, friction=(1.0, 0.9), contact_offset=0.001)
OBJECTS = [NEAR, FRONT, COIN, MAT]

# ---------------------------------------------------------------- scoring
MOUTH_DEPTH = 0.005  # the key's tip at least this far past the mouth (into the narrow slot): "entered"


def key_points(front):
    """World (tip, grip point, back end) of the chain key."""
    return tuple(np.asarray(front.link_to_world("key", p), float) for p in (KEY_TIP, KEY_GRIP, KEY_BACK))


def score_terms(objects):
    near, front, coin, mat = objects["near_lock"], objects["front_lock"], objects["quarter"], objects["mat"]
    tip, grip, back = key_points(front)
    t_tip, t_back = near.to_local(tip), near.to_local(back)
    depth = t_tip[1] - TL.MOUTH_Y  # how far the tip is past the slot's mouth (lock frame)
    in_line = abs(t_tip[0]) < 0.004 and abs(t_tip[2] - TL.KEY_Z) < 0.003 and t_back[1] < t_tip[1]
    entered = bool(in_line and depth >= MOUTH_DEPTH)
    latched = bool(in_line and TL.LATCH_DEPTH[0] <= depth <= TL.LATCH_DEPTH[1])
    ball_down = near.joints["ball"] < 0.001
    pusher_out = near.joints["pusher"] < TL.PUSHER_OUT + 0.004
    locked = bool(latched and ball_down and pusher_out)
    c = np.asarray(coin.origin, float)
    c_lock = near.to_local(c)
    in_pusher = TL.coin_in_pusher(near, c)
    m = np.asarray(mat.origin, float)
    on_mat = bool(abs(c[0] - m[0]) < MAT_SIZE / 2 - 0.004 and abs(c[1] - m[1]) < MAT_SIZE / 2 - 0.004
                  and TL.coin_low_z(coin) - (m[2] + MAT_T / 2) < 0.003)  # resting on the mat, either way up
    if DIAG:
        _diag(objects, depth, t_tip, c_lock)
    return {"entered": entered, "locked": locked, "depth": depth, "in_pusher": in_pusher, "on_mat": on_mat}


def score(objects):
    t = score_terms(objects)
    if t["locked"]:
        if t["on_mat"]:
            return 1.0
        if not t["in_pusher"]:
            return 0.75
        return 0.5
    return 0.25 if t["entered"] else 0.0


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- demo (right arm alone; the left arm stays at rest)
# Key: the hand comes straight down over the key lying on the bench, fingers closing across its 15 mm width 50 mm
# behind its tip, stops with the pad tips 8 mm up, reads where it really is, corrects sideways, lowers the tips to
# 0.5 mm over the bench and pinches the key's two long edges. It lifts the key and turns it half a turn (tip now along
# +y) on the way to the near lock, holds it 15 mm short of the slot, reads the key in its hand and corrects, puts the
# tip 3 mm into the mouth, reads and corrects again across the slot, and slides it 17 mm in, past the locking ball
# (reading the depth once and making up any shortfall). Further in, the sides of the fingers would meet the lock. It
# lets go, rises, turns its closed fingers square to the key behind its eye end, comes down to the key's height and
# pushes the key home by its end with the front finger's outer face, as a thumb does (sent 22 mm past the stop: the
# wrist gives about 5 mm per newton this way). The lock latches the key and pushes the quarter out.
# Coin: the hand comes down over the quarter standing in the pusher, pinches its exposed half across its faces, draws
# it straight up, carries it to the mat and lets it go 3 mm over the mat.
PAD_TIP = 0.013
TIP_Z = 0.0005
TIP_LOW = 0.008
GRIP_OPEN = -0.020  # pads 40 mm apart
KEY_PINCH = -0.006  # pads sent 12 mm apart on the 15 mm key: 1.5 mm into each edge, ~7 N
CLOSED = 0.0
COIN_PINCH = 0.0  # pads sent together on the 1.75 mm coin: ~0.9 mm into each face, ~4 N
HOVER = TOP + 0.060
CARRY_Z = TOP + Z_T + 0.060
PRE_GAP = 0.015  # the key's tip this far short of the mouth while the hand reads it
IN_MOUTH = 0.003
STAGE_A = 0.017  # how far the pinched key goes in (past the ball) before the hand changes to pushing its end
STAGE_A_MAX = 0.006  # the second try at stage A adds at most this much for the shortfall the first one read
END_GAP = 0.003  # the front finger's face comes down this far behind the key's end
FINGER_T = 0.010  # closed fingers: the front finger's outer face is this far ahead of the hand centre (grip axis)
TIPS_ABOVE_KEY = 0.0003  # closed fingertips this far above the key end's underside (as read) when pushing it
PUSH_LEAD = 0.022  # sent this far past where the plunger stops the key (the wrist gives ~5 mm per N this way)
CORR_MAX, HAND_MAX = 0.015, 0.004
COIN_TIP_ABOVE = 0.0015  # pad tips this far above the pusher's top when they close on the coin
COIN_LIFT = 0.045
COIN_OFF_X = 0.003  # pads centred 3 mm outboard of the coin's centre, so the fingertips clear the lock's side
COIN_DROP = 0.003
INSERT_YAW = math.pi / 2  # key along +y: fingers close along x
PARK = np.array([0.18, -0.30, TOP + 0.20])
FINAL_HOLD_S = 1.5
_READS = {"episode": None, "values": {}}
_TARGETS = {}


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def lock_point(lock, p_local):
    """World point of a lock-frame point (the locks are fixed and world aligned)."""
    return np.asarray(lock.pos, float) + np.asarray(p_local, float)


def _objects_now(names):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, list(names))
    except Exception as e:  # noqa: BLE001
        print(f"TL_READ failed: {e!r}", flush=True)
        return None


def _values():
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
        _TARGETS.clear()
    return _READS["values"]


def _mm(p):
    return tuple(round(float(x) * 1000, 1) for x in p)


def read_key(tag):
    """(tip, grip, back) of the key, read once per tag (offline: the start layout, or the planned pose)."""
    v = _values()
    key = f"key_{tag}"
    if key not in v:
        st = _objects_now(["front_lock"])
        if st is None:
            v[key] = None
        else:
            v[key] = key_points(st["front_lock"])
            print(f"TL_READ {key} t={_RUN['t']:.2f} tip={_mm(v[key][0])} back={_mm(v[key][2])}", flush=True)
    return v[key]


def start_key():
    return key_points(FRONT.offline_state(FRONT.pos, 0.0, {}))


def read_hand(tag, commanded):
    """Where the right hand's centre point really is, minus where it was sent (clamped), read once per tag."""
    v = _values()
    key = f"hand_{tag}"
    if key not in v:
        hands = grippers() if _RUN["env"] is not None else None
        if not hands:
            v[key] = np.zeros(3)
        else:
            err = np.asarray(hands["right"][0], float) - np.asarray(commanded, float)
            v[key] = np.clip(err, -HAND_MAX, HAND_MAX)
            print(f"TL_READ {key} t={_RUN['t']:.2f} hand_err_mm={tuple(round(float(x) * 1000, 2) for x in err)}",
                  flush=True)
    return v[key]


def pick_key():
    k = read_key("pick")
    return start_key() if k is None else k


def pick_yaw():
    tip, _, back = pick_key()
    d = tip - back
    return math.atan2(d[1], d[0])  # the key's heading; a grip at this yaw closes across it


def pick_point():
    off = np.zeros(3)
    if PROBE_OFFSET is not None:
        dx, dy = PROBE_OFFSET[(_RUN["episode"] - 1) % len(PROBE_OFFSET)]
        off = np.array([dx, dy, 0.0]) / 1000.0
    return pick_key()[1] + off


def _turn(yaw0):
    """The turn from the pick yaw to INSERT_YAW, positive (about half a turn; the wrist has room that way)."""
    return (INSERT_YAW - yaw0) % (2 * math.pi)


def slot_tip(depth):
    """The key tip's wanted world position, `depth` past the mouth (negative: short of it)."""
    return lock_point(NEAR, (0.0, TL.MOUTH_Y + depth, TL.KEY_Z))


def nominal_tip_from_tcp():
    """The tip relative to the hand centre for an ideal pinch (offline, and the first guess on the GPU)."""
    along = float(np.linalg.norm(KEY_GRIP - KEY_TIP))
    return _rz(INSERT_YAW) @ np.array([along, 0.0, 0.0]) + np.array([0.0, 0.0, TL.KEY_T / 2 - PAD_TIP - TIP_Z])


def tip_offset(tag, tcp_cmd):
    """Measured tip minus the commanded hand centre while the hand holds still (read once per tag)."""
    v = _values()
    key = f"off_{tag}"
    if key not in v:
        st = _objects_now(["front_lock"])
        if st is None:
            v[key] = None
        else:
            tip = key_points(st["front_lock"])[0]
            v[key] = tip - np.asarray(tcp_cmd, float)
            print(f"TL_READ {key} t={_RUN['t']:.2f} tip={_mm(tip)} tip_from_tcp={_mm(v[key])}", flush=True)
    return v[key]


def stage_a_extra():
    """Shortfall of the first stage-A slide (wanted minus read depth), clamped: the second try adds it."""
    v = _values()
    if "extra_a" not in v:
        k = read_key("stage_a")
        if k is None:
            v["extra_a"] = 0.0
        else:
            depth = float(NEAR.offline_state(NEAR.pos, 0.0, {}).to_local(k[0])[1] - TL.MOUTH_Y)
            v["extra_a"] = float(np.clip(STAGE_A - depth, 0.0, STAGE_A_MAX))
            print(f"TL_READ stage_a depth_mm={depth * 1000:.1f} extra_mm={v['extra_a'] * 1000:.1f}", flush=True)
    return v["extra_a"]


def insert_target(kind):
    """Hand centre while the pinched key goes in: 'pre' (15 mm short), 'mouth' (3 mm in), 'stage_a' (17 mm in),
    'stage_a2' (17 mm plus the shortfall read after the first try)."""
    depth = {"pre": -PRE_GAP, "mouth": IN_MOUTH, "stage_a": STAGE_A, "stage_a2": STAGE_A}[kind]
    if kind == "stage_a2":
        depth += stage_a_extra()
    want = slot_tip(depth)
    guess = want - nominal_tip_from_tcp()
    if kind == "pre":
        return guess
    off = tip_offset("pre", _TARGETS.get("pre", guess))
    if off is None:
        return guess
    tcp = want - off
    if kind in ("stage_a", "stage_a2"):
        off_m = tip_offset("mouth", _TARGETS.get("mouth", tcp))
        if off_m is not None:
            tcp = tcp + np.array([1.0, 0.0, 1.0]) * ((want - off_m) - tcp)  # across the slot only
    shift = tcp - guess
    n = float(np.linalg.norm(shift))
    if n > CORR_MAX:
        tcp = guess + shift * CORR_MAX / n
    return tcp


PUSH_YAW = 0.0  # fingers close along y: the front finger's broad outer face (36 mm across) pushes the key's end


def push_targets():
    """(behind, low, push) hand centres for pushing the key home by its end with the closed fingers."""
    k = read_key("let_go")
    if k is None:
        tip = slot_tip(STAGE_A)
        back = tip - np.array([0.0, TL.KEY_L, 0.0])
    else:
        tip, _, back = k
    home_back = slot_tip(TL.HOME_DEPTH)[1] - TL.KEY_L
    x = float(back[0])
    z_low = float(back[2]) - TL.KEY_T / 2 + TIPS_ABOVE_KEY + PAD_TIP
    z_low = max(z_low, float(NEAR.pos[2]) + TL.TUBE_R + 0.0012 + PAD_TIP)  # never into the handle tube
    y_behind = float(back[1]) - END_GAP - FINGER_T
    y_push = home_back + PUSH_LEAD - FINGER_T
    behind = np.array([x, y_behind, z_low + 0.025])
    return behind, np.array([x, y_behind, z_low]), np.array([x, y_push, z_low])


def coin_now():
    v = _values()
    if "coin" not in v:
        st = _objects_now(["quarter", "near_lock"])
        if st is None:
            v["coin"] = lock_point(NEAR, TL.coin_out_local())
        else:
            c = np.asarray(st["quarter"].origin, float)
            print(f"TL_READ coin t={_RUN['t']:.2f} coin={_mm(c)} pusher_mm={st['near_lock'].joints['pusher'] * 1000:.1f}",
                  flush=True)
            v["coin"] = c
    return v["coin"]


def hand_target(kind):
    """(hand centre, top-down grip yaw) for one step of the plan."""
    if kind in ("k_above", "k_over", "k_low", "k_pick", "k_lift"):
        g, yaw = pick_point(), pick_yaw()
        if kind in ("k_above", "k_lift"):
            return np.array([g[0], g[1], CARRY_Z]), yaw
        if kind == "k_over":
            return np.array([g[0], g[1], HOVER]), yaw
        low = np.array([g[0], g[1], TOP + PAD_TIP + TIP_LOW])
        if kind == "k_low":
            return low, yaw
        err = read_hand("pick", low) * np.array([1.0, 1.0, 0.0])
        return np.array([g[0], g[1], TOP + PAD_TIP + TIP_Z]) - err, yaw
    if kind in ("turn_a", "turn_b"):
        yaw0, g = pick_yaw(), pick_point()
        frac = 0.33 if kind == "turn_a" else 0.67
        pre = insert_target("pre")
        mid = g + frac * (pre - g)
        return np.array([mid[0], mid[1], CARRY_Z]), yaw0 + frac * _turn(yaw0)
    if kind in ("pre", "mouth", "stage_a", "stage_a2"):
        tcp = insert_target(kind)
        _TARGETS[kind] = tcp
        return tcp, INSERT_YAW
    if kind == "rise":
        return _TARGETS.get("stage_a2", insert_target("stage_a2")) + np.array([0.0, 0.0, 0.025]), INSERT_YAW
    if kind in ("behind", "low", "push"):
        behind, low, push = push_targets()
        return {"behind": behind, "low": low, "push": push}[kind], PUSH_YAW
    if kind == "away":
        return push_targets()[1] + np.array([0.0, -0.020, 0.050]), PUSH_YAW
    if kind in ("c_above", "c_over", "c_pick", "c_lift"):
        c = coin_now()
        top = lock_point(NEAR, (0.0, 0.0, TL.PUSHER_TOP))[2]
        pick = np.array([c[0] - COIN_OFF_X, c[1], top + COIN_TIP_ABOVE + PAD_TIP])
        lift = {"c_above": 0.050, "c_over": 0.020, "c_pick": 0.0, "c_lift": COIN_LIFT}[kind]
        return pick + np.array([0.0, 0.0, lift]), 0.0
    if kind in ("m_over", "m_low", "m_back", "m_rise"):
        c = coin_now()
        top = lock_point(NEAR, (0.0, 0.0, TL.PUSHER_TOP))[2]
        held = c[2] - (top + COIN_TIP_ABOVE + PAD_TIP)  # coin centre relative to the hand centre at the pick
        z_low = TOP + MAT_T + COIN_DROP + TL.COIN_R - held
        if kind == "m_rise":
            return np.array([MAT_XY[0], MAT_XY[1], PARK[2]]), 0.0
        up = {"m_over": 0.040, "m_low": 0.0, "m_back": 0.060}[kind]
        return np.array([MAT_XY[0], MAT_XY[1], z_low + up]), 0.0
    raise ValueError(kind)


class TrolleyWaypoint(Waypoint):
    """A right-hand waypoint worked out when it is played (reads happen the first time a waypoint needs them, at its
    start, when the hand is at the previous waypoint)."""

    def __init__(self, duration, kind, finger, label, keep=False):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.kind, self.finger, self.label = kind, finger, label

    @property
    def pos(self):
        return tuple(float(x) for x in hand_target(self.kind)[0])

    @property
    def yaw(self):
        return float(hand_target(self.kind)[1])


def demo():
    O, K, C = GRIP_OPEN, KEY_PINCH, COIN_PINCH
    W = TrolleyWaypoint
    right = [
        W(1.6, "k_above", O, "over the key"),
        W(1.2, "k_over", O, "down over the key"),
        W(1.0, "k_low", O, "pad tips 8 mm over the bench"),
        W(0.3, "k_low", O, "hold low"),
        W(0.6, "k_pick", O, "pad tips to the bench either side of the key"),
        W(0.8, "k_pick", K, "pinch the key's long edges", keep=True),
        W(0.3, "k_pick", K, "settle"),
        W(1.0, "k_lift", K, "lift the key"),
        W(1.0, "turn_a", K, "turn it"),
        W(1.0, "turn_b", K, "turn it on"),
        W(1.4, "pre", K, "key lined up 15 mm short of the slot"),
        W(0.6, "pre", K, "hold"),
        W(1.0, "mouth", K, "tip 3 mm into the slot's mouth"),
        W(0.5, "mouth", K, "hold"),
        W(1.2, "stage_a", K, "slide it 17 mm in"),
        W(0.4, "stage_a", K, "hold"),
        W(0.6, "stage_a2", K, "make up what it fell short"),
        W(0.6, "stage_a2", O, "let go of the key", keep=True),
        W(0.8, "rise", O, "rise clear of the key"),
    ]
    if DEMO_WAY == "home":
        right += [
            W(1.4, "behind", CLOSED, "fingers closed, turned square to the key, behind its end"),
            W(0.8, "low", CLOSED, "down to the key's height"),
            W(2.5, "push", CLOSED, "push the key home by its end"),
            W(1.5, "push", CLOSED, "hold it home"),
            W(1.0, "away", O, "back off"),
            W(1.4, "c_above", O, "over the quarter"),
            W(0.8, "c_over", O, "down over it"),
            W(0.6, "c_pick", O, "pads either side of the quarter"),
            W(0.8, "c_pick", C, "pinch it across its faces", keep=True),
            W(0.3, "c_pick", C, "settle"),
            W(1.0, "c_lift", C, "draw it straight up"),
            W(1.6, "m_over", C, "over the mat"),
            W(0.8, "m_low", C, "lower it to 3 mm over the mat"),
            W(0.6, "m_low", O, "let go", keep=True),
            W(0.8, "m_back", O, "back up"),
            W(1.0, "m_rise", O, "rise clear of the locks"),
        ]
    right += [
        Waypoint(1.5 * DEMO_PACE, tuple(float(x) for x in PARK), GRIP_OPEN, "park", 0.0),
        Waypoint(FINAL_HOLD_S, tuple(float(x) for x in PARK), GRIP_OPEN, "hold clear", 0.0),
    ]
    return {"right": right}


# ---------------------------------------------------------------- offline test layouts (states.yaml)
KEY_HOME_Q = TL.KEY_HOME_Q  # front_lock joints (deg) that put the key home in the near lock (offline solve)
KEY_MOUTH_Q = TL.KEY_MOUTH_Q  # ... the key's tip 8 mm into the slot, not home
PUSHER_OUT_M = TL.PUSHER_OUT
COIN_T, COIN_R = TL.COIN_T, TL.COIN_R


def coin_on_mat():
    return [float(MAT_XY[0]), float(MAT_XY[1]), TOP + MAT_T + TL.COIN_T / 2]


def coin_out_world():
    return [float(x) for x in lock_point(NEAR, TL.coin_out_local())]


def coin_flat_quat():
    """Lying flat: the authored on-edge coin turned 90 degrees about x."""
    return [math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)]


def coin_edge_quat():
    return list(TL.COIN_EDGE_QUAT)


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


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
        out[arm] = (tcp, float(np.mean(q[finger_ids])))
    return out


def _diag(objects, depth, t_tip, c_lock):
    step = _RUN["step"]
    if step == _DIAG["step"] or step % 4:
        return
    _DIAG["step"] = step
    near = objects["near_lock"]
    j = near.joints
    hands = grippers() or {}
    h = " ".join(f"{a}=({','.join(f'{x * 1000:.1f}' for x in hands[a][0])};{hands[a][1] * 1000:.1f})" for a in hands)
    print(f"TL_DIAG t={_RUN['t'] or 0.0:.2f} tip_lock_mm=({t_tip[0] * 1000:.1f},{t_tip[1] * 1000:.1f},"
          f"{t_tip[2] * 1000:.1f}) depth={depth * 1000:.1f} plunger={j['plunger'] * 1000:.2f} ball={j['ball'] * 1000:.2f} "
          f"pusher={j['pusher'] * 1000:.1f} coin_lock_mm=({c_lock[0] * 1000:.1f},{c_lock[1] * 1000:.1f},"
          f"{c_lock[2] * 1000:.1f};{objects['quarter'].tilt_deg():.0f}) {h}", flush=True)
