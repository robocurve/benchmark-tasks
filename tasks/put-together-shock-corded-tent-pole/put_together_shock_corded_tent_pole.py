"""Put a shock-corded tent pole together.

On the bare table, in front of the robot, lies one folded aluminium tent pole (Six Moon Designs 45 in Ultralight
Tent Pole, 3-section): three dark grey tubes 9.5 mm across and 38 cm long, joined by a black elastic cord that runs
inside them. Each of the first two sections has a bare aluminium insert sticking 5 cm out of one end; it fits into
the open end of the next section. Folded, the three tubes lie side by side along the robot's left-right axis
(about 15 cm in front of the arms' bases): the middle section nearest the robot, the left end section beside it, and
the right end section outermost, fanned out a little from the right-hand joint. Assembled, the pole is one straight
114 cm tube.

Metres; x forward from the arm bases, y to the robot's left, z up.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts  # noqa: E402,F401
import yam_mesh  # noqa: E402,F401
sys.modules.pop("elastic2_kit", None)
import elastic2_kit as e2  # noqa: E402
sys.modules.pop("corded_pole", None)
import corded_pole as cp  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
DIAG = False
DEMO_MODE = "full"  # "full": both joints; "early": lets go of the left section part-way round (wrong way)
TEST_GRIPPERS = None

# ---------------------------------------------------------------- the pole (all three sections free)
MASS = 0.085 / 3  # 85 g pole (maker), split evenly (estimate)
CORD_K = 8.5  # N/m of the cord's whole run (estimate: 3.0 N assembled, 4.0 N folded)
CORD_REST = 0.787  # m relaxed (estimate: stretched 45 % to the pole's 1.14 m when assembled)
CORD_C = 0.2  # N s/m
X_M = 0.150  # middle section's axis
Z_AXIS = TOP + cp.R_OUT * cp.FLAT + 0.0005  # a section lying on a flat, 0.5 mm up
LANE = 2 * cp.R_OUT * cp.FLAT + 0.0005  # axis to axis of two sections side by side


def _fan_angle():
    """Angle of the right end section (A) off the middle one: its insert's tip beside M's mouth, its tube just
    clearing the free end of B."""
    tip = np.array((X_M + cp.R_OUT * cp.FLAT + cp.R_FERRULE * cp.FLAT + 0.0005, -cp.HALF))
    corner = np.array((X_M + LANE + cp.R_OUT * cp.FLAT, cp.HALF + cp.FERRULE_OUT - cp.TUBE_L))
    for k in range(0, 3000):
        phi = math.radians(k * 0.01)
        e = np.array((math.sin(phi), math.cos(phi)))
        d = corner - tip
        side = d[0] * e[1] - d[1] * e[0]  # > 0: the corner lies on the line's +x side
        dist = abs(side)
        if side < 0 and dist >= cp.R_OUT + 0.0005:
            return phi, tip
    raise ValueError("no fan angle")


FAN_A, TIP_A0 = _fan_angle()
E_A0 = np.array((math.sin(FAN_A), math.cos(FAN_A)))  # from A's insert tip toward its free end
_ca = TIP_A0 + (cp.HALF + cp.FERRULE_OUT) * E_A0
A0 = (float(_ca[0]), float(_ca[1]), Z_AXIS)
B0 = (X_M + LANE, cp.HALF + cp.FERRULE_OUT - cp.HALF, Z_AXIS)  # B's mouth level with M's insert tip

TUBE_A = cp.section("tube_a", A0, math.pi - FAN_A, MASS, ferrule=True, mouth=False)
TUBE_M = cp.section("tube_m", (X_M, 0.0, Z_AXIS), 0.0, MASS, ferrule=True, mouth=True)
TUBE_B = cp.section("tube_b", B0, math.pi, MASS, ferrule=False, mouth=True)
CORD = cp.cord("cord", [TUBE_A, TUBE_M, TUBE_B], rest=CORD_REST, stiffness=CORD_K, damping=CORD_C,
               spawn=(0.86, -0.60, TOP + 0.0023))  # drawn pieces: spawned at the far corner, drawn at the joints
OBJECTS = [TUBE_A, TUBE_M, TUBE_B, *CORD.segments]
SECTIONS = ("tube_a", "tube_m", "tube_b")
# For the offline checker: the closed left fingertips press on the tops of B and M (side by side) to roll B out
# from M; its replay cannot roll B, so it also sees the left fingers come down on M beside an unrolled B.
PRESSED_PARTS = ("tube_b", "tube_m")

# ---------------------------------------------------------------- scoring constants
SEAT_GAP = 0.001  # joint home: at most 1 mm of insert showing
NEAR_GAP = 0.005  # 0.75 band
STRAIGHT_FULL, STRAIGHT_HALF = 0.005, 0.020  # largest side distance from a straight edge laid tip to tip
CLEAR = 0.030  # each gripper's centre point at least 3 cm from the pole
STILL_MM = 0.003  # no point of the pole moves more than 3 mm over the last 1 s
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)  # the calibrated table's top (station.py)
_RUN = e2._RUN


def frames(states):
    return {n: e2.frame_of(states[n]) for n in SECTIONS}


def joints(fr):
    """Joint 1: A's insert into M's mouth; joint 2: M's insert into B's mouth."""
    out = []
    for ins, rec in (("tube_a", "tube_m"), ("tube_m", "tube_b")):
        j = cp.joint(fr[ins], fr[rec])
        j["entered"] = j["depth"] > 0.001 and j["side"] < cp.R_BORE + 0.0005
        out.append(j)
    return out


def centre_line(fr):
    """The pole's centre line from A's free end to B's free end, through the four tube ends at the joints."""
    def pt(n, local):
        o, R = fr[n]
        return o + R @ np.asarray(local, float)
    return [pt("tube_a", (0, -cp.HALF, 0)), pt("tube_a", (0, cp.HALF, 0)), pt("tube_m", (0, -cp.HALF, 0)),
            pt("tube_m", (0, cp.HALF, 0)), pt("tube_b", (0, -cp.HALF, 0)), pt("tube_b", (0, cp.HALF, 0))]


def straightness(fr):
    """Largest distance (m) of the centre line from the straight edge laid from tip to tip."""
    pts = centre_line(fr)
    a, b = pts[0], pts[-1]
    u = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    return max(float(np.linalg.norm((p - a) - ((p - a) @ u) * u)) for p in pts)


def on_table(fr):
    for p in centre_line(fr):
        if not (TABLE_X[0] <= p[0] <= TABLE_X[1] and TABLE_Y[0] <= p[1] <= TABLE_Y[1] and p[2] > TOP - 0.01):
            return False
    return True


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    fr, fs = e2.tcp_frames(env), e2.finger_state(env)
    return {a: (fr[a][0], fs[a][0]) for a in fr}


def _seg_dist(p, a, b):
    d = b - a
    u = float(np.clip((p - a) @ d / max(d @ d, 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + u * d)))


def hands_clear(fr):
    """Each gripper's centre point (between the pads) at least CLEAR from every tube's surface (offline: True)."""
    hands = grippers()
    if hands is None:
        return True
    pts = centre_line(fr)
    segs = [(pts[0], pts[1]), (pts[2], pts[3]), (pts[4], pts[5])]
    return all(_seg_dist(np.asarray(tcp, float), a, b) - cp.R_OUT >= CLEAR for tcp, _ in hands.values()
               for a, b in segs)


_HIST = {"ep": None, "rows": [], "flags": {}}


def _episode():
    if _HIST["ep"] != _RUN["episode"]:
        _HIST.update(ep=_RUN["episode"], rows=[], flags={})


def ever(key, cond):
    """Sticky for the episode in the simulator; offline each call stands alone."""
    if _RUN["env"] is None:
        return bool(cond)
    _episode()
    if cond:
        _HIST["flags"][key] = True
    return bool(_HIST["flags"].get(key, False))


def moved_1s(now):
    """Largest change of any centre-line coordinate over the last second (None until a second has been seen;
    offline 0). Judged by position: a body pressed on a contact reports a steady phantom speed."""
    t = _RUN["t"]
    if t is None or _RUN["env"] is None:
        return 0.0
    _episode()
    rows = _HIST["rows"]
    if not rows or t > rows[-1][0] + 1e-6:
        rows.append((float(t), np.array(now, float)))
    while len(rows) > 1 and rows[1][0] <= t - 1.0 + 1e-6:
        rows.pop(0)
    if t - rows[0][0] < 1.0 - 1e-6:
        return None
    return float(np.max(np.abs(np.array(now, float) - rows[0][1])))


def score_terms(states):
    fr = frames(states)
    js = joints(fr)
    entered = sum(j["entered"] for j in js)
    gaps = [j["gap"] for j in js]
    d = straightness(fr)
    fell = ever("off_table", not on_table(fr))
    clear = hands_clear(fr)
    moved = moved_1s(np.concatenate(centre_line(fr)))
    still = moved is not None and moved <= STILL_MM
    if entered == 2 and max(gaps) <= SEAT_GAP:
        j_band = 1.0
    elif entered == 2 and max(gaps) <= NEAR_GAP:
        j_band = 0.75
    elif entered == 2:
        j_band = 0.5
    elif entered == 1:
        j_band = 0.25
    else:
        j_band = 0.0
    if entered < 2 or d > STRAIGHT_HALF:
        s_band = 0.25
    elif d > STRAIGHT_FULL:
        s_band = 0.5
    else:
        s_band = 1.0
    r_band = (1.0 if still else 0.5) if clear else 0.25
    t = {"joints": js, "entered": entered, "gaps": gaps, "straight": d, "fell": fell, "clear": clear,
         "moved": moved, "still": still, "bands": (j_band, s_band, r_band)}
    if DIAG:
        _diag(states, t)
    return t


def score(states):
    t = score_terms(states)
    if t["fell"] or t["entered"] == 0:
        return 0.0
    return float(min(t["bands"]))


def success(states):
    return score(states) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU runs)
_D = {"step": -1}


def _diag(states, t):
    env, step = _RUN["env"], _RUN["step"]
    if env is None or step == _D["step"] or step % 4:
        return
    _D["step"] = step
    try:
        fr = frames(states)
        live = e2.live("cord")
        hands = e2.tcp_frames(env)
        cmd = e2.tcp_frames(env, target=True)
        fs = e2.finger_state(env)
        oM, RM = fr["tube_m"]
        rel = {}
        for n in ("tube_a", "tube_b"):
            o, R = fr[n]
            p = RM.T @ (o - oM)
            yaw = math.degrees(math.atan2(-(RM.T @ R)[0, 1], (RM.T @ R)[1, 1]))
            rel[n] = f"({p[0] * 1000:.1f},{p[1] * 1000:.1f},{p[2] * 1000:.1f}) yaw={yaw:.1f}"
        js = t["joints"]
        sag = {a: (cmd[a][0] - hands[a][0]) * 1000 for a in hands}
        print(f"TP_DIAG ep={_RUN['episode']} t={_RUN['t']:.2f} T={live['tension']:.2f}N L={live['length'] * 1000:.0f}"
              f" route={live['active']} | J1 d={js[0]['depth'] * 1000:.1f} side={js[0]['side'] * 1000:.2f} "
              f"gap={js[0]['gap'] * 1000:.1f} ang={js[0]['angle']:.1f} | J2 d={js[1]['depth'] * 1000:.1f} "
              f"side={js[1]['side'] * 1000:.2f} gap={js[1]['gap'] * 1000:.1f} ang={js[1]['angle']:.1f} | "
              f"M=({oM[0] * 1000:.1f},{oM[1] * 1000:.1f},{oM[2] * 1000:.1f}) A/M {rel['tube_a']} B/M {rel['tube_b']} | "
              f"straight={t['straight'] * 1000:.1f}mm clear={t['clear']} moved={t['moved']} bands={t['bands']} | "
              f"R gap={fs['right'][0] * 1000:.1f} f={fs['right'][1]:.1f} sag=({sag['right'][0]:.1f},"
              f"{sag['right'][1]:.1f},{sag['right'][2]:.1f}) L gap={fs['left'][0] * 1000:.1f} f={fs['left'][1]:.1f} "
              f"sag=({sag['left'][0]:.1f},{sag['left'][1]:.1f},{sag['left'][2]:.1f})", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"TP_DIAG failed: {e!r}", flush=True)


# ---------------------------------------------------------------- demo helpers
class Grip(float):
    """A waypoint 'yaw' that carries a full TCP rotation matrix (as the elastic2 probe did); its float value is the
    top-down yaw whose closing axis matches (-sin yaw, cos yaw), unwrapped near ``near``, for the offline checker."""
    def __new__(cls, matrix, near=None):
        m = np.array(matrix, float)
        y = math.atan2(-m[0, 0], m[1, 0])
        if near is not None:
            y += 2 * math.pi * round((near - y) / (2 * math.pi))
        v = super().__new__(cls, y)
        v.grip_matrix = m
        return v


def _install_grip_matrix():
    from yam_arena import demo as arena_demo
    original = arena_demo.grip_rotation
    if getattr(original, "_grip_matrix", False):
        return

    def grip_rotation(yaw=0.0):
        m = getattr(yaw, "grip_matrix", None)
        return m.copy() if m is not None else original(float(yaw))

    grip_rotation._grip_matrix = True
    arena_demo.grip_rotation = grip_rotation


_install_grip_matrix()


def rot(close, approach=(0.0, 0.0, -1.0)):
    """TCP rotation: fingers closing along ``close`` (made horizontal), approaching along ``approach``."""
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    j = np.asarray(close, float) - (np.asarray(close, float) @ a) * a
    j /= np.linalg.norm(j)
    return np.column_stack([j, np.cross(a, j), a])


class Live(Waypoint):
    """A waypoint whose target is computed when the plan reaches it (from the live poses)."""
    def __init__(self, duration, where, finger, label, R):
        self.duration, self.label = duration, label
        self._where, self._finger, self._R = where, finger, R
        self.prev, self._yc = None, (None, None)

    @property
    def pos(self):
        return tuple(float(v) for v in self._where())

    @property
    def finger(self):
        return float(self._finger() if callable(self._finger) else self._finger)

    @property
    def yaw(self):
        key = (_RUN["episode"], _RUN["step"], _RUN["env"] is None)
        if self._yc[0] != key:
            near = float(self.prev.yaw) if self.prev is not None else None
            self._yc = (key, Grip(self._R() if callable(self._R) else self._R, near))
        return self._yc[1]


def _states(names):
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import scene as arena_scene
    return arena_scene.object_states(env, list(names))


_PLAN = {"ep": None, "cache": {}, "servo": {}}


def _ep_cache():
    if _PLAN["ep"] != _RUN["episode"]:
        _PLAN.update(ep=_RUN["episode"], cache={}, servo={})
    return _PLAN["cache"]


def _yaw_of(R):
    return math.atan2(-R[0, 1], R[1, 1])  # the angle that turns +y to the section's +y about z


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


_LIVE = {"key": None, "fr": None}


def _level(fr):
    """Each section's origin (on its axis) and heading only. The tubes are round and roll about their own axes on
    the table (M by up to 70 degrees while the last joint closes); a frame that kept the roll would tip M's sideways
    axis out of the horizontal and hide part of a side error from the servo."""
    return {n: (o, _rz(_yaw_of(R))) for n, (o, R) in fr.items()}


def live_frames():
    """Current (origin, level R) of the three sections, read once per control step; offline: the planned poses."""
    if _RUN["env"] is None:
        return _level({n: _OFFLINE.get(n) or e2.spec_frame(e2.SPECS[n]) for n in SECTIONS})
    key = (_RUN["episode"], _RUN["step"])
    if _LIVE["key"] != key:
        _LIVE.update(key=key, fr=_level(frames(_states(SECTIONS))))
    return _LIVE["fr"]


_OFFLINE = {}  # offline stand-ins for sections the plan has moved (set by the offline plan check)


def in_m(fr, name):
    """(x, y, yaw) of a section in M's frame (2-D)."""
    oM, RM = fr["tube_m"]
    o, R = fr[name]
    p = RM.T @ (o - oM)
    return np.array((p[0], p[1])), _yaw_of(RM.T @ R)


def to_world(fr, xy, z, yaw):
    """A point and yaw given in M's frame -> world point and world yaw."""
    oM, RM = fr["tube_m"]
    p = oM + RM @ np.array((xy[0], xy[1], 0.0))
    return np.array((p[0], p[1], z)), yaw + _yaw_of(RM)


def read(tag, fn):
    """Evaluate ``fn`` once per episode under ``tag``."""
    c = _ep_cache()
    if tag not in c:
        c[tag] = fn()
    return c[tag]


def anchor(tag):
    """M's frame read once per episode under ``tag``. The hands hold M and swing the end sections against this fixed
    frame, not against M's live pose: the cord pulls M toward the section being swung, and targets that followed M
    let it drift without limit."""
    return {"tube_m": read("anchor_" + tag, lambda: live_frames()["tube_m"])}


PHASE = {"tube_a": "joint1", "tube_b": "joint2"}


# ---------------------------------------------------------------- the plan's numbers
PAD_TIP = 0.0128
Z_GRIP = TOP + 0.0015 + PAD_TIP  # pad tips 1.5 mm above the table, pads across the tube's middle
Z_HOVER = TOP + 0.070
Z_PRESS = TOP + 2 * cp.R_OUT * cp.FLAT - 0.0025 + PAD_TIP  # closed fingertips 2.5 mm into a section's top
OPEN = -0.015  # pads 30 mm apart
OPEN_LOOSE = -0.010  # 20 mm: lets a section slide between the pads
OPEN_NEAR = -0.011  # 22 mm: approaching a section that lies near another
HOLD_ONE = -0.0032  # 6.4 mm on a 9.2 mm section: 1.4 mm into each side
HOLD_PAIR = -0.0068  # 13.6 mm on two sections side by side (about 17.5 mm as squeezed): about 10 N at 5 N/mm a pad
HOLD_FIRM = -0.0025  # 5 mm on one section (9.1 mm as squeezed): about 10 N, so the joined pair does not slip
#                      under the cord's 4 N
CLOSED = 0.0
D_A = 0.140  # right hand grips A this far from its insert's tip (14 cm: the inner finger then clears B)
D_B_MIN = 0.060  # left hand grips B at least this far from its mouth
Y_PAIR = 0.170  # left hand holds M and B together here during joint 1 (wrists 24 cm apart in y)
D_PUSH = 0.130  # left fingertips roll B out from this far along it from its mouth
PUSH = 0.085  # fingertip travel across (B's middle moves about half of it)
PRE = 0.0015  # aligned, the insert's tip this far short of the mouth
INSERT = 0.008  # then the tip is taken this far into the bore before the hand opens
SWING_STEPS = 12
SWING_STEP_S = 0.45
SERVO_GAIN = 0.15
CLOSE_SIGN = {"tube_a": -1.0, "tube_b": 1.0}  # which way the fingers close across each section (offline IK)
PARK = {"left": np.array([0.200, 0.215, TOP + 0.170]), "right": np.array([0.200, -0.300, TOP + 0.170])}
R_ALONG_X = rot((1.0, 0.0, 0.0))


def hand_R(world_yaw, name):
    """TCP rotation pinching a section whose +y points along world yaw ``world_yaw``."""
    c = _rz(world_yaw) @ np.array((CLOSE_SIGN[name], 0.0, 0.0))
    return rot(c)


def section_point(fr, name, local):
    o, R = fr[name]
    return o + R @ np.asarray(local, float)


# Joint geometry in M's frame: the swung section's joint point and where it must end.
JOINT = {
    "tube_a": {"pt": cp.TIP_LOCAL, "goal": np.array((0.0, -cp.HALF)), "dir": -1.0, "grip_y": cp.TIP_LOCAL[1] - D_A},
    "tube_b": {"pt": cp.MOUTH_LOCAL, "goal": np.array((0.0, cp.HALF + cp.FERRULE_OUT)), "dir": +1.0, "grip_y": None},
}


def goal_pose(name, depth):
    """(origin xy, yaw) in M's frame of the swung section when lined up with M and its joint point ``depth`` past
    the other part's end (negative: short of it)."""
    j = JOINT[name]
    if name == "tube_a":  # A's insert tip on M's axis at the mouth, A pointing like M
        tip = j["goal"] + np.array((0.0, depth))
        return tip - np.array((0.0, cp.TIP_LOCAL[1])), 0.0
    mouth = j["goal"] - np.array((0.0, depth))  # B's mouth past M's insert tip, B pointing like M
    return mouth - np.array((0.0, cp.MOUTH_LOCAL[1])), 0.0


def pivot(p0, yaw0, p1, yaw1, pt_local):
    """Centre (M frame) of the turn that takes the section's joint point from where it is to where it must go."""
    def jp(p, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return p + np.array((c * pt_local[0] - s * pt_local[1], s * pt_local[0] + c * pt_local[1]))
    a0, a1 = jp(p0, yaw0), jp(p1, yaw1)
    dyaw = yaw1 - yaw0
    c, s = math.cos(dyaw), math.sin(dyaw)
    Rm = np.array(((c, -s), (s, c)))
    ctr = np.linalg.solve(np.eye(2) - Rm, a1 - Rm @ a0)
    return ctr, dyaw


def swung(name, u, start):
    """Pose (xy, yaw) in M's frame after fraction u of the turn from ``start`` = (xy, yaw)."""
    p0, y0 = start
    p1, y1 = goal_pose(name, -PRE)
    y1 = y0 + _turn(name, y0, y1)
    ctr, dyaw = pivot(p0, y0, p1, y1, JOINT[name]["pt"])
    a = u * dyaw
    c, s = math.cos(a), math.sin(a)
    return ctr + np.array(((c, -s), (s, c))) @ (p0 - ctr), y0 + a


def _turn(name, y0, y1):
    """The turn (rad) the section makes: out through +x (A turns clockwise seen from above, B anticlockwise)."""
    d = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
    if name == "tube_a" and d > 0:
        d -= 2 * math.pi
    if name == "tube_b" and d < 0:
        d += 2 * math.pi
    return d


def grip_local(name):
    """Point of the section under the hand (section frame)."""
    if name == "tube_a":
        return np.array((0.0, JOINT["tube_a"]["grip_y"], 0.0))
    return np.array((0.0, read("grip_b_y", _grip_b_y), 0.0))


def _grip_b_y():
    """B is gripped where the inner finger clears M by 4 mm, at least D_B_MIN from its mouth."""
    fr = live_frames()
    p, yaw = in_m(fr, "tube_b")
    beta = abs(math.sin(yaw))  # B's +y against M's -y
    need = cp.R_OUT * cp.FLAT + 0.001 + 0.010 + (-OPEN_NEAR)  # M's side, margin, a 10 mm finger, half the opening
    c, s = math.cos(yaw), math.sin(yaw)
    lateral0 = p[0] - s * cp.MOUTH_LOCAL[1]  # B's axis off M's axis at its mouth (about one lane)
    d = D_B_MIN if beta < 1e-3 else max(D_B_MIN, (need - lateral0) / max(beta, 1e-3))
    d = min(d, 0.12)
    return float(cp.MOUTH_LOCAL[1] + d)


def hand_at(fr, name, xy, yaw, z=Z_GRIP):
    """TCP position and rotation that hold section ``name`` at (xy, yaw) in M's frame by its grip point."""
    g = grip_local(name)
    c, s = math.cos(yaw), math.sin(yaw)
    gxy = xy + np.array((c * g[0] - s * g[1], s * g[0] + c * g[1]))
    p, wyaw = to_world(fr, gxy, z, yaw)
    return p, hand_R(wyaw, name)


def start_of(name, tag="start"):
    """The section's pose in the phase's anchored M frame, read once under ``tag`` when the plan first needs it. The
    hover over a section uses an early read ("pre"); the hand goes down on a fresh read ("start"), because the cord can
    still be settling a jittered section for a second or so (probe 2: 14 mm and 3.5 deg in the first 1.3 s)."""
    def now():
        fr = dict(live_frames())
        fr["tube_m"] = anchor(PHASE[name])["tube_m"]
        return tuple(in_m(fr, name))
    return read(f"{tag}_{name}", now)


def swing_target(name, u, z=Z_GRIP, tag="start"):
    fr = anchor(PHASE[name])
    xy, yaw = swung(name, u, start_of(name, tag))
    return hand_at(fr, name, xy, yaw, z)


def servo_target(name, depth, z=Z_GRIP, window=None):
    """Hold section ``name`` lined up with M at ``depth``, correcting each control step by SERVO_GAIN of the
    measured side and angle error (the arms give about 4-5 mm per newton sideways)."""
    fr = live_frames()
    xy, yaw = goal_pose(name, depth)
    sv = _PLAN["servo"].setdefault(name, {"off": np.zeros(2), "yaw": 0.0, "step": None})
    t = _RUN["t"]
    if (window is not None and _RUN["env"] is not None and t is not None and window[0] <= t < window[1]
            and sv["step"] != _RUN["step"]):
        sv["step"] = _RUN["step"]
        p, y = in_m(fr, name)
        jp_local = JOINT[name]["pt"]
        c, s = math.cos(y), math.sin(y)
        jp_now = p + np.array((c * jp_local[0] - s * jp_local[1], s * jp_local[0] + c * jp_local[1]))
        err_x = 0.0 - jp_now[0]  # side error of the joint point from M's axis
        err_yaw = (yaw - y + math.pi) % (2 * math.pi) - math.pi
        sv["off"] = np.clip(sv["off"] + SERVO_GAIN * np.array((err_x, 0.0)), -0.015, 0.015)
        sv["yaw"] = float(np.clip(sv["yaw"] + SERVO_GAIN * err_yaw, -0.2, 0.2))
        if _RUN["step"] % 10 == 0:
            print(f"TP_SERVO {name} t={_RUN['t']:.2f} side_err={err_x * 1000:.2f}mm yaw_err={math.degrees(err_yaw):.2f}"
                  f" off=({sv['off'][0] * 1000:.1f}) yaw_off={math.degrees(sv['yaw']):.2f}", flush=True)
    return hand_at(anchor(PHASE[name]), name, xy + sv["off"], yaw + sv["yaw"], z)


def over(fn, up):
    """Same as a target from ``fn`` but ``up`` higher."""
    p, R = fn()
    return p + np.array((0.0, 0.0, up)), R


def W(duration, fn, finger, label):
    """A live waypoint from a function returning (TCP position, rotation)."""
    return Live(duration, lambda: fn()[0], finger, label, lambda: fn()[1])


def pair_hold(z=Z_GRIP):
    """Across M and B side by side at M-frame y = Y_PAIR (M's frame and B's offset fixed at the start)."""
    fr = anchor("joint1")
    pbx = read("pair_bx", lambda: float(in_m(live_frames(), "tube_b")[0][0]))
    p, _ = to_world(fr, (pbx / 2, Y_PAIR), z, 0.0)
    return p, rot(fr["tube_m"][1] @ np.array((-1.0, 0.0, 0.0)))  # the left arm closes along -x


def push_at(frac, z):
    """Closed fingertips on B's top, D_PUSH from its mouth, moved out across M's axis by frac x PUSH."""
    def f():
        fr = read("push_frame", live_frames)
        pb, yb = in_m(fr, "tube_b")
        c, s = math.cos(yb), math.sin(yb)
        local = (0.0, cp.MOUTH_LOCAL[1] + D_PUSH)
        xy = pb + np.array((c * local[0] - s * local[1], s * local[0] + c * local[1])) + np.array((0.001 + frac * PUSH, 0.0))
        p, _ = to_world(fr, xy, z, 0.0)
        return p, rot(fr["tube_m"][1] @ np.array((-1.0, 0.0, 0.0)))  # the left arm closes along -x
    return f


# ---------------------------------------------------------------- the plan
_CALLS = [0]


def swing(name, finger, upto=SWING_STEPS, t_step=SWING_STEP_S, label="turn"):
    return [W(t_step, (lambda u=(k + 1) / SWING_STEPS: swing_target(name, u)), finger,
              f"{label} {int(round(100 * (k + 1) / SWING_STEPS))} %") for k in range(upto)]


def _t(ws):
    return sum(w.duration for w in ws)


def joint_steps(name, t0, label_line, label_in):
    """Line up, insert and open for section ``name`` (the cord then draws it home through the open fingers); the servo
    corrects only inside its own waypoints."""
    w1, w2 = (t0, t0 + 0.6), (t0 + 0.6, t0 + 1.6)
    return [W(0.6, lambda: servo_target(name, -PRE, window=w1), HOLD_ONE, label_line),
            W(1.0, lambda: servo_target(name, INSERT, window=w2), HOLD_ONE, label_in),
            W(0.5, lambda: servo_target(name, INSERT), OPEN_LOOSE, "let the cord pull it home"),
            W(0.7, lambda: servo_target(name, INSERT), OPEN_LOOSE, "wait")]


def demo():
    _CALLS[0] += 1
    mode = DEMO_MODE
    print(f"TP_PLAN call={_CALLS[0]} mode={mode} fan={math.degrees(FAN_A):.2f}deg", flush=True)
    a, b = "tube_a", "tube_b"
    right = [W(1.6, lambda: over(lambda: swing_target(a, 0.0, tag="pre"), Z_HOVER - Z_GRIP), OPEN_NEAR, "over the right section"),
             W(1.0, lambda: swing_target(a, 0.0), OPEN_NEAR, "down beside it"),
             W(0.7, lambda: swing_target(a, 0.0), HOLD_ONE, "pinch the right section"),
             W(0.3, lambda: swing_target(a, 0.0), HOLD_ONE, "settle")]
    right += swing(a, HOLD_ONE, label="swing it round")
    right += joint_steps(a, _t(right), "line it up with the middle section", "insert the tip")
    t_released_a = _t(right)  # hand open and the section home
    # The right hand closes on its section again where it now lies and holds the joined pair against the cord
    # while the left section comes round (no move to M: probe 2's 180 deg wrist turn there flipped the arm).
    right += [W(0.5, lambda: servo_target(a, INSERT), HOLD_FIRM, "close on it again")]
    t_right_hold = _t(right)
    left = [W(1.6, lambda: over(pair_hold, Z_HOVER - Z_GRIP), OPEN, "over the middle and left sections"),
            W(1.0, pair_hold, OPEN, "down"),
            W(0.7, pair_hold, HOLD_PAIR, "hold them together")]
    left += [W(t_released_a - _t(left), pair_hold, HOLD_PAIR, "hold while the right joint closes"),
             W(0.5, pair_hold, OPEN, "let go"),
             W(0.6, lambda: over(pair_hold, 0.040), OPEN, "up")]
    left += [W(max(0.8, t_right_hold - _t(left)), lambda: over(push_at(0.0, Z_PRESS), 0.040), CLOSED,
               "over the left section"),
             W(0.6, push_at(0.0, Z_PRESS), CLOSED, "fingertips on top of it"),
             W(1.5, push_at(1.0, Z_PRESS), CLOSED, "roll it out from the middle section"),
             W(0.5, lambda: over(push_at(1.0, Z_PRESS), 0.040), CLOSED, "up"),
             W(1.0, lambda: over(lambda: swing_target(b, 0.0, tag="pre"), Z_HOVER - Z_GRIP), OPEN_NEAR, "over the left section"),
             W(0.9, lambda: swing_target(b, 0.0), OPEN_NEAR, "down beside it"),
             W(0.7, lambda: swing_target(b, 0.0), HOLD_ONE, "pinch the left section"),
             W(0.3, lambda: swing_target(b, 0.0), HOLD_ONE, "settle")]
    if mode == "early":  # wrong way: let go two thirds of the way round, the section not yet in line
        k = SWING_STEPS * 2 // 3
        left += swing(b, HOLD_ONE, upto=k, label="swing it round")
        left += [W(0.5, lambda: swing_target(b, k / SWING_STEPS), OPEN, "let go part-way round"),
                 W(0.6, lambda: over(lambda: swing_target(b, k / SWING_STEPS), 0.040), OPEN, "up")]
    else:
        left += swing(b, HOLD_ONE, label="swing it round")
        left += joint_steps(b, _t(left), "line it up with the middle section", "slide its mouth over the insert")
        left += [W(0.6, lambda: servo_target(b, INSERT, z=Z_GRIP + 0.040), OPEN, "up")]
    t_left_done = _t(left)
    left += [W(1.4, lambda: (PARK["left"], rot((1.0, 0.0, 0.0))), OPEN, "park"),
             W(3.4, lambda: (PARK["left"], rot((1.0, 0.0, 0.0))), OPEN, "hold clear")]
    right += [W(max(0.5, t_left_done - _t(right)), lambda: servo_target(a, INSERT), HOLD_FIRM,
                "hold the joined sections while the left joint closes"),
              W(0.5, lambda: servo_target(a, INSERT), OPEN, "let go"),
              W(0.6, lambda: servo_target(a, INSERT, z=Z_GRIP + 0.050), OPEN, "up"),
              W(1.4, lambda: (PARK["right"], rot((-1.0, 0.0, 0.0))), OPEN, "park"),
              W(2.4, lambda: (PARK["right"], rot((-1.0, 0.0, 0.0))), OPEN, "hold clear")]
    for plan in (left, right):
        for w0, w1 in zip(plan, plan[1:]):
            w1.prev = w0
    return {"left": left, "right": right}
