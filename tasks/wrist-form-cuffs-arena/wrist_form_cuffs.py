"""Chain handcuffs and two fixed wrist forms on the station table (bench drill, no person involved).

Two upright wrist forms stand in a line away from the robot. One pair of chain handcuffs lies flat on the table
beside them, both swinging bars open. Each cuff has a lock case (darker block at one end), a fixed double bar, a
swinging single bar hinged at the other end, and a yellow double-lock lever tab on the end of the lock case.
The task: put one cuff round each form, close each bar until the fit is right (a 6 mm rod passes between cuff and
form, a 12 mm rod does not), push both lever tabs home, and move both grippers away.

Frame: x forward from the arm bases, y left, z up, metres.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape, CylShape, segment  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0

PROBE = os.environ.get("CUFF_PROBE", "")  # "m1": one cuff already on its form, pull and squeeze tests after the demo
ONE = os.environ.get("CUFF_SCENE", "one" if PROBE == "m1" else "full") == "one"
RATCHET = os.environ.get("CUFF_RATCHET", "limit")  # "limit": the hinge's stop follows the notch; "drive": a stiff spring
CUFFS = ("a",) if ONE else ("a", "b")

# -- the wrist forms ---------------------------------------------------------------------------------------------------
LINE_Y = 0.050
FORM_X = {"near": 0.120, "far": 0.292}
FORM_OF = {"a": "near", "b": "far"}
FORM_L, FORM_W, FORM_H = 0.054, 0.040, 0.030  # post: 54 x 40 mm with round ends (154 mm round), 30 mm tall
CAP_L, CAP_W, CAP_H = 0.070, 0.036, 0.010  # the hand stand-in on top: longer than the closed cuff's opening
SKIN, SKIN2 = (0.86, 0.70, 0.60), (0.80, 0.62, 0.52)


def _stadium_shapes(length: float, width: float, z0: float, z1: float, color) -> list:
    r, half = width / 2, (length - width) / 2
    zc, h = (z0 + z1) / 2, z1 - z0
    return [BoxShape((0.0, 0.0, zc), (2 * half, width, h), color),
            CylShape((half, 0.0, zc), r, h, color), CylShape((-half, 0.0, zc), r, h, color)]


def _form(name: str) -> Body:
    shapes = _stadium_shapes(FORM_L, FORM_W, 0.0, FORM_H, SKIN) + _stadium_shapes(CAP_L, CAP_W, FORM_H, FORM_H + CAP_H, SKIN2)
    return Body.build(f"form_{name}", pos=(FORM_X[name], LINE_Y, TOP), mass=0.5, shapes=shapes, color=SKIN,
                      kinematic=True, friction=(0.6, 0.5))


def stadium_outline(length: float = FORM_L, width: float = FORM_W, n: int = 64) -> np.ndarray:
    """Points round the post's outline (form frame, long side along x)."""
    r, half = width / 2, (length - width) / 2
    t = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.column_stack([np.sign(np.cos(t)) * half + r * np.cos(t), r * np.sin(t)])


# -- one cuff (dimensions of the plain-MuJoCo build: Smith & Wesson Model 100 size) -------------------------------------
# Cuff frame: origin at the bar's pivot rivet, +Y toward the lock case, the swinging bar on the -X side, Z through
# the cuff's thickness. Every body is spawned world-aligned, its shapes already turned into place (BASE).
BASE = {"a": np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),  # lock case away from the robot
        "b": np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])}  # turned over: lock case toward the robot
SIGN = {"a": 1.0, "b": -1.0}  # the cuff's Z against the world's
TOOTH_R, TOOTH_PITCH = 0.0800, 0.0030  # tooth tips 80 mm from the rivet, 3.0 mm apart
DELTA = math.degrees(TOOTH_PITCH / TOOTH_R)  # hinge angle per notch (2.149 degrees)
N_POS = 23  # locking positions (maker's figure)
BAR_OPEN = 95.0  # degrees
BAR_STOP = 106.5
BAR_PTS = np.array([[0, 0], [-0.020, 0.006], [-0.038, 0.022], [-0.050, 0.042]]
                   + [[-0.0765 * math.sin(math.radians(a)), 0.0765 * math.cos(math.radians(a))]
                      for a in (47, 40, 33, 26, 19, 12, 5, -3)])
BOW_PTS = np.array([[0, 0], [0.016, 0.005], [0.028, 0.020], [0.032, 0.040], [0.029, 0.058], [0.020, 0.072]])
BAR_HW, BAR_T = 0.0032, 0.004  # single bar: 6.4 mm wide, 4 mm thick
BOW_HW, BOW_T = 0.004, 0.010  # double bar: 8 mm wide, 10 mm over both cheeks
CASE_C, CASE = (0.0, 0.08725), (0.034, 0.0335, 0.013)  # lock case: across, along, thick
BOSS_R = 0.007
EYE = (0.012, 0.104)  # chain swivel eye on the case end
TAB_C, TAB = (-0.012, 0.110), (0.0055, 0.010, 0.008)  # double-lock lever tab: across (travel), along, tall
LEVER_TRAVEL = 0.0055  # the tab slides this far toward +X (cuff frame) to double-lock
LEVER_K = 400.0  # N/m: 2.2 N on the tab to push it home
LEVER_HOME = LEVER_TRAVEL - 0.0007
WRIST_IN_CUFF_Y = 0.038  # the form's centre on the cuff's Y axis when the cuff is on it
WRIST_X_IN_CUFF = 0.006  # and on its X axis: the form 2 mm off the double bar
MASS = {"body": 0.118, "bar": 0.036, "lever": 0.004, "link": 0.006}  # pair 328 g with the chain
LINK_LEN = 0.028  # two chain links, 56 mm between the swivel eyes
STEEL, STEEL2, YELLOW = (0.78, 0.80, 0.82), (0.50, 0.52, 0.56), (0.95, 0.75, 0.15)
Z_CUFF = TOP + CASE[2] / 2 + 0.0005
MU_CUFF = (0.30, 0.25)

T_PASS = 3.5 * TOOTH_R  # N m: 3.5 N at the teeth to pass one (the plain-MuJoCo build's measured figure)
K_RATCHET = T_PASS / DELTA  # N m/deg: the push rises to T_PASS over one tooth, then drops (a click)
K_HARD = 2.0  # N m/deg ("drive" ratchet only): the pawl against a tooth face
HINGE_DAMPING = 0.002  # N m s/deg
BACKLASH = 0.15  # degrees of play at a notch

PIVOT_ON = {"a": (FORM_X["near"] - WRIST_IN_CUFF_Y, LINE_Y + WRIST_X_IN_CUFF),
            "b": (FORM_X["far"] + WRIST_IN_CUFF_Y, LINE_Y + WRIST_X_IN_CUFF)}
PIVOT_START = {"a": (0.082, -0.020), "b": (0.325, 0.008)}


def _to_body(c: str, x: float, y: float, z: float = 0.0) -> np.ndarray:
    return BASE[c] @ np.array([x, y, z])


def _seg_box(c: str, p1, p2, width: float, thick: float, color, pad: float = 0.003) -> BoxShape:
    a, b = _to_body(c, *p1), _to_body(c, *p2)
    d = b - a
    return BoxShape(tuple(((a + b) / 2).tolist()), (float(np.linalg.norm(d)) + pad, width, thick), color,
                    yaw=math.atan2(d[1], d[0]))


def _cuff_shapes(c: str) -> list:
    shapes = [BoxShape(tuple(_to_body(c, *CASE_C).tolist()), (CASE[1], CASE[0], CASE[2]), STEEL2),
              CylShape((0.0, 0.0, 0.0), BOSS_R, BOW_T, STEEL2)]
    shapes += [_seg_box(c, p1, p2, 2 * BOW_HW, BOW_T, STEEL) for p1, p2 in zip(BOW_PTS[:-1], BOW_PTS[1:])]
    shapes.append(CylShape(tuple(_to_body(c, *EYE).tolist()), 0.003, 0.004, STEEL2, collide=False))
    return shapes


def _bar_shapes(c: str) -> list:
    return [_seg_box(c, p1, p2, 2 * BAR_HW, BAR_T, STEEL) for p1, p2 in zip(BAR_PTS[:-1], BAR_PTS[1:])]


def _lever_shapes(c: str) -> list:
    return [BoxShape(tuple(_to_body(c, *TAB_C).tolist()), (TAB[1], TAB[0], TAB[2]), YELLOW)]


def _eye_world(c: str, pivot_xy, yaw: float = 0.0) -> np.ndarray:
    e = _rz(yaw) @ _to_body(c, *EYE)
    return np.array([pivot_xy[0] + e[0], pivot_xy[1] + e[1]])


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _zquat(a: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def chain_mid(eye_a, eye_b) -> np.ndarray:
    """Where the two links meet for these eye positions (the chain bent toward -y)."""
    gap = np.asarray(eye_b, float) - np.asarray(eye_a, float)
    dist = float(np.linalg.norm(gap))
    if dist > 2 * LINK_LEN - 1e-4:
        raise ValueError(f"layout needs {1000 * dist:.1f} mm of chain, have {2000 * LINK_LEN:.0f}")
    heading, bend = math.atan2(gap[1], gap[0]), math.acos(dist / (2 * LINK_LEN))
    return np.asarray(eye_a, float) + LINK_LEN * np.array([math.cos(heading - bend), math.sin(heading - bend)])


def _build() -> tuple[list, dict]:
    objects: list = [_form("near")] if ONE else [_form("near"), _form("far")]
    nominal: dict = {}
    for c in CUFFS:
        px, py = PIVOT_START[c]
        pos = (px, py, Z_CUFF)
        s = SIGN[c]
        objects.append(Body.build(f"cuff_{c}", pos=pos, mass=MASS["body"], shapes=_cuff_shapes(c), color=STEEL,
                                  friction=MU_CUFF, solver_iterations=32))
        objects.append(Body.build(f"bar_{c}", pos=pos, mass=MASS["bar"], shapes=_bar_shapes(c), color=STEEL,
                                  friction=MU_CUFF, solver_iterations=32, ignore=(f"cuff_{c}",)))
        objects.append(Body.build(f"lever_{c}", pos=pos, mass=MASS["lever"], shapes=_lever_shapes(c), color=YELLOW,
                                  friction=(0.8, 0.7), solver_iterations=32, ignore=(f"cuff_{c}", f"bar_{c}")))
        lim = (-((N_POS - 1) * DELTA + 1.7), BAR_STOP)
        bodykit.hinge(f"hinge_{c}", f"cuff_{c}", f"bar_{c}", anchor=pos, axis="Z",
                      limits_deg=lim if s > 0 else (-lim[1], -lim[0]), spring=0.0, damping=HINGE_DAMPING)
        tab = np.array(pos) + _to_body(c, *TAB_C)
        bodykit.slider(f"lever_slide_{c}", f"cuff_{c}", f"lever_{c}", anchor=tuple(tab.tolist()), axis="Y",
                       limits=(-LEVER_TRAVEL - 0.0002, 0.0002), stiffness=LEVER_K, damping=4.0, target=0.0,
                       max_force=50.0)
        nominal[c] = pos
    if not ONE:
        ea, eb = _eye_world("a", PIVOT_START["a"]), _eye_world("b", PIVOT_START["b"])
        mid = chain_mid(ea, eb)
        for name, eye in (("link1", ea), ("link2", eb)):
            d = np.array([*(mid - eye), 0.0])
            objects.append(Body.build(name, pos=(eye[0], eye[1], Z_CUFF), mass=MASS["link"],
                                      shapes=[segment((0.0, 0.0, 0.0), tuple(d.tolist()), 0.005, STEEL2)],
                                      color=STEEL2, friction=MU_CUFF, solver_iterations=32, damping=(0.2, 0.2)))
            nominal[name] = (eye, mid)
        # order in OBJECTS: a joint is authored when its second body is spawned
        order = {o.name: o for o in objects}
        objects = [order[n] for n in ("form_near", "form_far", "cuff_a", "bar_a", "lever_a", "link1", "link2",
                                      "cuff_b", "bar_b", "lever_b")]
        d1 = tuple(np.array([*(mid - ea), 0.0]) / LINK_LEN)
        bodykit.ball("chain1", "cuff_a", "link1", anchor=(ea[0], ea[1], Z_CUFF), axis=d1, swing_damping=1e-4,
                     twist_damping=1e-4)
        bodykit.ball("chain2", "link1", "link2", anchor=(mid[0], mid[1], Z_CUFF), axis=d1, swing_damping=1e-4,
                     twist_damping=1e-4)
        bodykit.ball("chain3", "link2", "cuff_b", anchor=(eb[0], eb[1], Z_CUFF), axis=d1, swing_damping=1e-4,
                     twist_damping=1e-4)
    return objects, nominal


OBJECTS, _NOMINAL = _build()
NAMES = [o.name for o in OBJECTS]


# -- reading the mechanism from poses ------------------------------------------------------------------------------------
def bar_deg(cuff, bar, c: str) -> float:
    """The swinging bar's angle in the cuff's own sense (degrees): 0 at the first tooth, positive open, negative
    further in (one notch every DELTA)."""
    rel = cuff.rotation().T @ bar.rotation()
    return SIGN[c] * math.degrees(math.atan2(rel[1, 0], rel[0, 0]))


def lever_travel(cuff, lever) -> float:
    """How far the lever tab has slid toward its double-locked end (m)."""
    return -float((cuff.rotation().T @ (np.asarray(lever.pos, float) - np.asarray(cuff.pos, float)))[1])


def notch_of(q: float) -> int:
    """Locking position reached at bar angle ``q``: 0 not latched, 1 the first tooth, up to N_POS."""
    if q > 0.02:
        return 0
    return int(min(N_POS, math.floor((-q + 0.02) / DELTA) + 1))


def notch_deg(n: int) -> float:
    return -(n - 1) * DELTA


def cuff_axes(cuff, c: str) -> tuple[np.ndarray, np.ndarray]:
    """(origin, 3x3 whose columns are the cuff frame's X, Y, Z in the world)."""
    return np.asarray(cuff.pos, float), cuff.rotation() @ BASE[c]


# -- episode state and the lock rules -----------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["CUFF_DEBUG"], "w") if os.environ.get("CUFF_DEBUG") else None
_LIMIT_ATTRS: dict = {}


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, hands={}, report={}, probe={}, applied={})
    for c in CUFFS:
        _EP[c] = dict(n=0, locked=False, q=BAR_OPEN, lever=0.0, clicks=[], soft=None)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def _set_limits(c: str, lo: float, hi: float) -> None:
    """Move the hinge's stops (degrees, cuff sense)."""
    name = f"hinge_{c}"
    if name not in bodykit._HINGE_PATHS:
        return
    if name not in _LIMIT_ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        joint = UsdPhysics.RevoluteJoint(get_current_stage().GetPrimAtPath(bodykit._HINGE_PATHS[name]))
        _LIMIT_ATTRS[name] = (joint.GetLowerLimitAttr(), joint.GetUpperLimitAttr())
    lo_w, hi_w = (lo, hi) if SIGN[c] > 0 else (-hi, -lo)
    _LIMIT_ATTRS[name][0].Set(float(lo_w))
    _LIMIT_ATTRS[name][1].Set(float(hi_w))


LO_MIN = -((N_POS - 1) * DELTA + 1.7)


def _apply_lock(c: str) -> None:
    """Set cuff ``c``'s hinge to what its lock allows now. This is the rule that stands in for the teeth and the pawl:
    the bar turns freely until its first tooth reaches the pawl; from then on it cannot come back out past the last
    notch it reached, and pushing it one notch further in takes a force rising to 3.5 N at the teeth. Double-locked,
    it cannot move either way (and an open cuff cannot latch)."""
    e = _EP[c]
    n, q = e["n"], e["q"]
    if e["locked"]:
        want = ("lock", n)
        if e["soft"] != want:
            if n >= 1:
                _set_limits(c, notch_deg(n) - BACKLASH, notch_deg(n) + BACKLASH)
            else:
                _set_limits(c, 0.3, BAR_STOP)
            bodykit.set_hinge_drive(f"hinge_{c}", SIGN[c] * (notch_deg(n) if n else BAR_OPEN), 0.0, HINGE_DAMPING)
            e["soft"] = want
        return
    if n == 0:
        want = ("free", q < DELTA)
        if e["soft"] != want:
            _set_limits(c, LO_MIN, BAR_STOP)
            bodykit.set_hinge_drive(f"hinge_{c}", SIGN[c] * DELTA, K_RATCHET if q < DELTA else 0.0, HINGE_DAMPING)
            e["soft"] = want
        return
    if RATCHET == "limit":
        want = ("notch", n)
        if e["soft"] != want:
            _set_limits(c, LO_MIN, notch_deg(n) + BACKLASH)
            bodykit.set_hinge_drive(f"hinge_{c}", SIGN[c] * notch_deg(n), K_RATCHET, HINGE_DAMPING)
            e["soft"] = want
    else:  # the pawl as a spring switched at the control rate: stiff against opening, the tooth's ramp against closing
        closing = q < notch_deg(n) - 0.05
        want = ("drive", n, closing)
        if e["soft"] != want:
            bodykit.set_hinge_drive(f"hinge_{c}", SIGN[c] * notch_deg(n), K_RATCHET if closing else K_HARD,
                                    HINGE_DAMPING * (1.0 if closing else 10.0))
            e["soft"] = want


def _hand_points(base) -> dict:
    """Points on each gripper's fingers (world), from the arm's own joint readings."""
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = {}
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out[arm] = finger_points(p, r, half)
    return out


def finger_points(p, r, half: float) -> list[np.ndarray]:
    """Points on the two fingers of a gripper whose TCP is at ``p`` with rotation ``r`` and whose pads are each
    ``half`` m from the centre line: pad tips, pad middles and finger roots."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    for c in CUFFS:
        e = _EP[c]
        cuff, bar, lever = st[f"cuff_{c}"], st[f"bar_{c}"], st[f"lever_{c}"]
        e["q"], e["lever"] = bar_deg(cuff, bar, c), lever_travel(cuff, lever)
        if not e["locked"]:
            n = notch_of(e["q"])
            if n > e["n"]:
                e["n"] = n
                e["clicks"].append((_EP["step"], n))
            if e["lever"] >= LEVER_HOME:
                e["locked"] = True
                bodykit.set_slider_drive(f"lever_slide_{c}", -LEVER_TRAVEL, 3000.0, 4.0)  # it stays home
        _apply_lock(c)
    try:
        _EP["hands"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    if PROBE == "m1":
        _probe(env, st)
    _EP["step"] += 1
    if _DBG is not None:
        _debug(st)


# -- scoring: the bench tests of the plain-MuJoCo build, thresholds unchanged -------------------------------------------
CLEAR_MM = 50.0  # both grippers at least this far from every cuff part
PULL_N, PULL_S = 30.0, 2.0
SQUEEZE_N, SQUEEZE_S = 20.0, 1.0
GAUGE_N = 3.0
GO_MM, NOGO_MM = 6.0, 12.0
POINTS = {"latched": 0.20, "gap": 0.15, "double_locked": 0.15}


def _turn2(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def loop_parts(q: float) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """The cuff's parts round the opening as thick segments (cuff frame, m): (end, end, half width)."""
    bar = BAR_PTS @ _turn2(q).T
    parts = [(a, b, BOW_HW) for a, b in zip(BOW_PTS[:-1], BOW_PTS[1:])]
    parts += [(a, b, BAR_HW) for a, b in zip(bar[:-1], bar[1:])]
    parts.append((np.array([-CASE[0] / 2 + CASE[1] / 2, CASE_C[1]]), np.array([CASE[0] / 2 - CASE[1] / 2, CASE_C[1]]),
                  CASE[1] / 2))
    parts.append((np.zeros(2), np.array([1e-6, 0.0]), BOSS_R))
    return parts


def _clearance(points: np.ndarray, parts) -> float:
    best = float("inf")
    for a, b, hw in parts:
        ab = b - a
        t = np.clip(((points - a) @ ab) / float(ab @ ab), 0.0, 1.0)
        d = np.linalg.norm(points - (a + t[:, None] * ab), axis=1) - hw
        best = min(best, float(d.min()))
    return best


def _inside(point, polygon) -> bool:
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, np.roll(polygon, -1, axis=0)):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def surrounds(cuff, c: str, q: float, forms: dict) -> str | None:
    """Name of the form whose post passes through the closed loop of cuff ``c``, or None."""
    pos, axes = cuff_axes(cuff, c)
    bar = BAR_PTS @ _turn2(q).T
    if q > 0.0 and np.linalg.norm(bar[-1] - np.array([0.0, 0.0765])) > 0.012:
        return None  # the bar's tip is not at the lock case: the loop is open
    if abs(axes[2, 2]) < 0.8:
        return None  # the cuff must lie across the post
    if not TOP - 0.002 < pos[2] < TOP + FORM_H + 0.005:
        return None
    loop = np.vstack([BOW_PTS, [[0.0, 0.0765]], bar[::-1][1:]])
    for name, form in forms.items():
        rel = axes.T @ (np.array([form.pos[0], form.pos[1], pos[2]]) - pos)
        if _inside(rel[:2], loop):
            return name
    return None


def play_mm(q: float) -> float:
    """Side play of the cuff on the form (mm) with the bar at ``q`` degrees: the cuff squared up on the form (its
    rivet-to-lock-case line along the form's long side, the form midway between rivet and lock case), then slid
    across the form from the double bar touching it to the bar touching it. It is the thickest gauge rod that goes
    between the bar and the form with the form against the double bar. It depends only on the notch, not on where the
    robot left the cuff."""
    pts = stadium_outline()[:, ::-1] + np.array([0.0, WRIST_IN_CUFF_Y])  # the form's long side along the cuff's Y
    parts = loop_parts(q)
    step, x, hi = 0.0001, 0.030, None
    while x > -0.040:  # from the double bar's side toward the bar's
        free = _clearance(pts + np.array([x, 0.0]), parts) > 0.0
        if free and hi is None:
            hi = x
        if hi is not None and not free:
            return 1000.0 * (hi - x - step)
        x -= step
    return 0.0


def cuff_points(objects) -> list[tuple[np.ndarray, float]]:
    """Points on every cuff part with the part's radius round each (world, m)."""
    pts = []
    for c in CUFFS:
        cuff = objects[f"cuff_{c}"]
        pos, axes = cuff_axes(cuff, c)
        q = _EP[c]["q"] if c in _EP else BAR_OPEN
        bar = BAR_PTS @ _turn2(q).T
        for line, r in ((BOW_PTS, BOW_HW), (bar, BAR_HW)):
            for a, b in zip(line[:-1], line[1:]):
                for t in (0.0, 0.5, 1.0):
                    p2 = a + t * (b - a)
                    pts.append((pos + axes @ np.array([p2[0], p2[1], 0.0]), r))
        for dx in (-0.012, 0.012):
            for dy in (-0.012, 0.012):
                pts.append((pos + axes @ np.array([CASE_C[0] + dx, CASE_C[1] + dy, 0.0]), 0.007))
        pts.append((np.asarray(objects[f"lever_{c}"].pos, float) + objects[f"lever_{c}"].rotation() @ _to_body(c, *TAB_C),
                    0.005))
    for name in ("link1", "link2"):
        if name in objects:
            eye, mid = _NOMINAL[name]
            o = objects[name]
            d = np.array([*(mid - eye), 0.0])
            for t in (0.0, 0.5, 1.0):
                pts.append((np.asarray(o.pos, float) + o.rotation() @ (t * d), 0.0025))
    return pts


def clearance_mm(objects) -> float | None:
    """The smallest distance (mm) from either gripper's fingers to any cuff part; None without the robot."""
    hands = _EP.get("hands") or {}
    if not hands:
        return None
    targets = cuff_points(objects)
    tp = np.array([t for t, _ in targets])
    tr = np.array([r for _, r in targets])
    best = float("inf")
    for pts in hands.values():
        for p in pts:
            best = min(best, float((np.linalg.norm(tp - p, axis=1) - tr).min()))
    return 1000.0 * best


def report(objects) -> dict:
    """Every measurement behind the score."""
    forms = {n: objects[f"form_{n}"] for n in FORM_X if f"form_{n}" in objects}
    clear_mm = clearance_mm(objects)
    out = {"clearance_mm": clear_mm, "cuffs": {}, "forms": {}}
    on = {}
    for c in CUFFS:
        q = bar_deg(objects[f"cuff_{c}"], objects[f"bar_{c}"], c)
        on[c] = surrounds(objects[f"cuff_{c}"], c, q, forms)
    total = 0.0
    for name, form in forms.items():
        cuffs = [c for c in CUFFS if on[c] == name]
        entry = {"cuffs_round_it": cuffs, "score": 0.0}
        if len(cuffs) == 1:
            c = cuffs[0]
            e = _EP[c]
            q = bar_deg(objects[f"cuff_{c}"], objects[f"bar_{c}"], c)
            # The 30 N pull: the lock rule lets a latched bar back out only to its last notch (BACKLASH), so a bar
            # that has reached a notch passes it and one that has not fails it.
            latched = e["n"] >= 1 and q <= notch_deg(e["n"]) + 1.0
            detail = {"notch": e["n"], "bar_deg": round(q, 2), "latched": bool(latched), "lever_home": bool(e["locked"])}
            if latched:
                entry["score"] += POINTS["latched"]
                play = play_mm(notch_deg(e["n"]))
                # The 20 N squeeze: a bar that is not double-locked goes in a notch (3 mm) under 3.5 N while there
                # is room; double-locked it cannot move.
                locked = bool(play >= GO_MM and e["locked"])
                detail.update(play_mm=round(play, 2), gap_ok=bool(GO_MM <= play < NOGO_MM), double_locked=locked)
                entry["score"] += POINTS["gap"] * detail["gap_ok"] + POINTS["double_locked"] * locked
            out["cuffs"][c] = detail
        entry["score"] = round(entry["score"], 2)
        out["forms"][name] = entry
        total += entry["score"]
    clear = clear_mm is None or clear_mm >= CLEAR_MM
    out["grippers_clear"] = bool(clear)
    out["score"] = round(min(1.0, total) if clear else 0.0, 2)
    return out


def score(objects) -> float:
    """Per form with exactly one cuff round it: 0.20 latched, +0.15 play at least 6 mm and under 12 mm, +0.15
    double-locked with at least 6 mm of play. Sum over the two forms. 0 for the trial unless both grippers end at
    least 50 mm from every cuff part."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= (0.5 if ONE else 1.0) - 1e-9


def distance(objects) -> float:
    return float(sum(np.linalg.norm(np.asarray(objects[f"cuff_{c}"].pos, float)[:2] - np.array(PIVOT_ON[c]))
                     for c in CUFFS))


# -- the seeded start ----------------------------------------------------------------------------------------------------
VARY_A = (0.012, 0.015)  # each trial moves cuff A by up to this in x and y (m)
VARY_B = (0.008, 0.008)  # and cuff B, on its own, by up to this
VARY_YAW = 6.0  # and turns each cuff by up to this (degrees)
# and moves the whole pair this far toward -y, away from the forms (m): the gripper's 60 mm opening and its closing
# fingers take up smaller offsets, so a plan made for the centre layout picked the cuffs up at 14 mm off (seed 0)
PAIR_DY = (-0.035, -0.015)


def layout(seed: int | None) -> dict:
    """World poses (position, quaternion x y z w) of every loose body for a seed (None or 0: the nominal layout in the
    setup text)."""
    rng = np.random.default_rng(seed) if seed else None
    while True:
        if rng is None:
            shift = {c: (0.0, 0.0, 0.0) for c in CUFFS}
        else:
            pair_dy = 0.0 if ONE else rng.uniform(*PAIR_DY)
            shift = {"a": (rng.uniform(-VARY_A[0], VARY_A[0]), pair_dy + rng.uniform(-VARY_A[1], VARY_A[1]),
                           math.radians(rng.uniform(-VARY_YAW, VARY_YAW))),
                     "b": (rng.uniform(-VARY_B[0], VARY_B[0]), pair_dy + rng.uniform(-VARY_B[1], VARY_B[1]),
                           math.radians(rng.uniform(-VARY_YAW, VARY_YAW)))}
        pose = {}
        for c in CUFFS:
            dx, dy, yaw = shift[c]
            if PROBE == "m1":
                pivot, yaw, q0 = PIVOT_ON[c], 0.0, 30.0
            else:
                pivot, q0 = (PIVOT_START[c][0] + dx, PIVOT_START[c][1] + dy), BAR_OPEN
            p = np.array([pivot[0], pivot[1], Z_CUFF])
            pose[f"cuff_{c}"] = (p, _zquat(yaw))
            pose[f"lever_{c}"] = (p, _zquat(yaw))
            pose[f"bar_{c}"] = (p, _zquat(yaw + SIGN[c] * math.radians(q0)))
            pose[f"_pivot_{c}"] = (pivot, yaw)
        if ONE:
            return pose
        ea = _eye_world("a", *pose["_pivot_a"])
        eb = _eye_world("b", *pose["_pivot_b"])
        if float(np.linalg.norm(eb - ea)) > 2 * LINK_LEN - 0.002:
            if rng is None:
                raise ValueError("the nominal layout over-stretches the chain")
            continue  # this draw over-stretches the chain: draw again
        mid = chain_mid(ea, eb)
        for name, eye in (("link1", ea), ("link2", eb)):
            eye0, mid0 = _NOMINAL[name]
            turn = math.atan2(mid[1] - eye[1], mid[0] - eye[0]) - math.atan2(mid0[1] - eye0[1], mid0[0] - eye0[0])
            pose[name] = (np.array([eye[0], eye[1], Z_CUFF]), _zquat(turn))
        return pose


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put the bodies where this trial's seed says, let them settle, and return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie

    base = emb._base
    lay = layout(seed)
    _blank()
    for c in CUFFS:
        _set_limits(c, LO_MIN, BAR_STOP)
        base.scene[f"bar_{c}"].permanent_wrench_composer.reset()
        base.scene[f"cuff_{c}"].permanent_wrench_composer.reset()
    for name, (pos, quat) in lay.items():
        if name.startswith("_"):
            continue
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(pos, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    if _DBG is not None:
        _DBG.write(f"seed {seed}: " + " ".join(f"{k}=({v[0][0]:.4f},{v[0][1]:.4f})" for k, v in lay.items()
                                                if k.startswith("cuff_")) + "\n")
    return emb._observation()


# -- M1 probe: pull the latched bar open with 30 N for 2 s, then squeeze it shut with 20 N for 1 s ------------------------
def _bar_mid(st, c: str) -> np.ndarray:
    pos, axes = cuff_axes(st[f"cuff_{c}"], c)
    p2 = _turn2(_EP[c]["q"]) @ BAR_PTS[3]
    return pos + axes @ np.array([p2[0], p2[1], 0.0])


def _push_pair(env, st, c: str, newtons: float) -> None:
    """``newtons`` on the bar at its middle along the cuff's +X (closing; negative opens), and the same force the
    other way on the cuff body at the same point: a scale hooked on the bar with the lock case held."""
    import torch

    base = env.unwrapped
    dev = base.device
    origin = base.scene.env_origins[0]
    axis = cuff_axes(st[f"cuff_{c}"], c)[1][:, 0]
    point = (torch.tensor(_bar_mid(st, c), dtype=torch.float32, device=dev) + origin).reshape(1, 1, 3)
    zero = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
    for name, sign in ((f"bar_{c}", 1.0), (f"cuff_{c}", -1.0)):
        obj = base.scene[name]
        obj.permanent_wrench_composer.reset()
        if newtons:
            f = torch.tensor(sign * newtons * axis, dtype=torch.float32, device=dev).reshape(1, 1, 3)
            obj.permanent_wrench_composer.add_forces_and_torques(f, zero, positions=point, body_ids=[0], is_global=True)


def _probe(env, st) -> None:
    c, p = "a", _EP["probe"]
    k = _EP["step"] - int(round(M1_END * CONTROL_HZ))
    q = _EP[c]["q"]
    hz = int(CONTROL_HZ)
    if k == hz:  # 1 s after the hands are clear
        p["q0"] = q
        _push_pair(env, st, c, -PULL_N)
    elif hz < k < 3 * hz:
        _push_pair(env, st, c, -PULL_N)
        p["q_pull_max"] = max(p.get("q_pull_max", q), q)
    elif k == 3 * hz:
        _push_pair(env, st, c, 0.0)
        p["q_pull_end"] = q
    elif k == 3 * hz + 10:
        p["q1"] = q
        p["backout_mm"] = math.radians(max(p["q_pull_max"], p["q_pull_end"], q) - p["q0"]) * TOOTH_R * 1000.0
        p["rest_backout_mm"] = math.radians(q - p["q0"]) * TOOTH_R * 1000.0
        _push_pair(env, st, c, SQUEEZE_N)
    elif 3 * hz + 10 < k < 4 * hz + 10:
        _push_pair(env, st, c, SQUEEZE_N)
    elif k == 4 * hz + 10:
        _push_pair(env, st, c, 0.0)
    elif k == 4 * hz + 20:
        p["advance_mm"] = math.radians(p["q1"] - q) * TOOTH_R * 1000.0
        p["n_end"] = _EP[c]["n"]
        if _DBG is not None:
            _DBG.write("PROBE " + " ".join(f"{a}={b:.3f}" for a, b in p.items()) + "\n")
        print("CUFF PROBE " + " ".join(f"{a}={b:.3f}" for a, b in p.items()), flush=True)


def _debug(st) -> None:
    rep = _EP.get("report") or {}
    parts = [f"{_EP['step']:4d}"]
    for c in CUFFS:
        e, cuff = _EP[c], st[f"cuff_{c}"]
        d = (rep.get("cuffs") or {}).get(c, {})
        parts.append(f"{c}: q={e['q']:7.2f} n={e['n']:2d} lock={int(e['locked'])} lev={e['lever'] * 1000:5.2f} "
                     f"p=({cuff.pos[0]:.4f},{cuff.pos[1]:.4f},{cuff.pos[2]:.4f}) "
                     f"yaw={math.degrees(math.atan2(cuff.rotation()[1, 0], cuff.rotation()[0, 0])):6.2f} "
                     f"tilt={math.degrees(cuff.tilt()):5.2f} play={d.get('play_mm', float('nan')):5.2f}")
    clr = rep.get("clearance_mm")
    parts.append(f"clr={clr if clr is None else round(clr, 1)} score={rep.get('score')}")
    _DBG.write(" ".join(parts) + "\n")
    _DBG.flush()


# -- the scripted demo -----------------------------------------------------------------------------------------------------
DOWN = grip_rotation(0.0)  # straight down; the fingers close along world y
PUSH_R = 0.062  # the fingertip pushes the bar this far from the rivet
PUSH_ALPHA0 = 50.0  # polar angle (degrees from the cuff's +Y toward -X) of that point on the bar at the first tooth
LEAD = float(os.environ.get("CUFF_LEAD_DEG", "7.0"))  # the fingertip's centre ahead of the bar's (degrees)
PUSH_PITCH = math.radians(float(os.environ.get("CUFF_PUSH_PITCH", "35")))
HOLD_PITCH = math.radians(float(os.environ.get("CUFF_HOLD_PITCH", "15")))
GRIP_Y = 0.095  # the holder takes the lock case here on the cuff's Y axis
OPEN_60 = -0.030  # finger joint target for a 60 mm opening
GRIP_30N = -(CASE[0] / 2 - 0.006)  # 6 mm past the case's faces at 5000 N/m: a 30 N grip
TIP = 0.013  # pad tips past the TCP


def _ry(a: float) -> np.ndarray:
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def hold_rot(c: str) -> np.ndarray:
    """The holder's grip: fingers closing along world y, leaning away from the form."""
    return _ry(HOLD_PITCH if c == "a" else -HOLD_PITCH) @ DOWN


def push_rot(heading, flip: bool = True) -> np.ndarray:
    """A closed gripper leaning back from the horizontal direction ``heading`` it pushes in: a broad finger face
    leads. ``flip`` turns the hand half a turn about its own axis (the same grip; it keeps the wrist roll in range)."""
    h = np.array([heading[0], heading[1], 0.0]) / float(np.hypot(heading[0], heading[1]))
    z = h * math.sin(PUSH_PITCH) + np.array([0.0, 0.0, -math.cos(PUSH_PITCH)])
    x = h * math.cos(PUSH_PITCH) + np.array([0.0, 0.0, math.sin(PUSH_PITCH)])
    if flip:
        x = -x
    return np.column_stack([x, np.cross(z, x), z])


def cuff_point(c: str, pivot, yaw: float, x: float, y: float) -> np.ndarray:
    """World (x, y) of a cuff-frame point for a cuff whose rivet is at ``pivot``, turned ``yaw`` from its nominal
    heading."""
    p = _rz(yaw) @ _to_body(c, x, y)
    return np.array([pivot[0] + p[0], pivot[1] + p[1]])


def push_tcp(c: str, pivot, yaw: float, alpha_deg: float, rot: np.ndarray, radius: float = PUSH_R) -> tuple:
    """TCP position that puts the closed fingertip's centre at polar angle ``alpha_deg`` round the rivet."""
    a = math.radians(alpha_deg)
    xy = cuff_point(c, pivot, yaw, -radius * math.sin(a), radius * math.cos(a))
    tip = np.array([xy[0], xy[1], TOP + 0.002 + 0.004 * math.sin(PUSH_PITCH)])
    return tuple((tip - TIP * rot[:, 2]).tolist())


def hold_tcp(c: str, pivot, yaw: float, lift: float = 0.0) -> tuple:
    xy = cuff_point(c, pivot, yaw, 0.0, GRIP_Y)
    z = TOP + TIP * math.cos(HOLD_PITCH) + 0.0077 * math.sin(HOLD_PITCH) + 0.002 + lift
    return (float(xy[0]), float(xy[1]), float(z))


def sweep(c: str, pivot, yaw: float, a_from: float, a_to: float, rot, seconds_per_deg: float = 0.10,
          label: str = "push the bar round") -> list[Waypoint]:
    n = max(1, int(math.ceil(abs(a_to - a_from) / 5.0)))
    out = []
    for k in range(1, n + 1):
        a = a_from + (a_to - a_from) * k / n
        out.append(Waypoint(abs(a_to - a_from) / n * seconds_per_deg, push_tcp(c, pivot, yaw, a, rot), FINGER_CLOSED,
                            label, rot))
    return out


M1_NOTCH = int(os.environ.get("CUFF_M1_NOTCH", "8"))


def m1_plans() -> dict[str, list[Waypoint]]:
    """Cuff A already on its form with the bar 30 degrees open: the right hand holds the lock case, the left
    fingertip pushes the bar in to notch M1_NOTCH, both let go."""
    c, pivot, yaw = "a", PIVOT_ON["a"], 0.0
    hr = hold_rot(c)
    g, g_up = hold_tcp(c, pivot, yaw), hold_tcp(c, pivot, yaw, 0.08)
    heading = (math.sin(math.radians(55.0)), -math.cos(math.radians(55.0)))
    pr = push_rot(heading)
    a0 = PUSH_ALPHA0 + 30.0 + LEAD + 10.0
    a1 = PUSH_ALPHA0 + LEAD + notch_deg(M1_NOTCH) - 0.4 * DELTA
    start = push_tcp(c, pivot, yaw, a0, pr)
    above = (start[0], start[1], start[2] + 0.07)
    left = [Waypoint(3.0, above, FINGER_CLOSED, "over the bar", pr),
            Waypoint(2.0, above, FINGER_CLOSED, "wait for the holder", pr),
            Waypoint(2.0, start, FINGER_CLOSED, "down beside the bar", pr)]
    left += sweep(c, pivot, yaw, a0, a1, pr, 0.14)
    left += [Waypoint(1.0, push_tcp(c, pivot, yaw, a1, pr), FINGER_CLOSED, "hold the push", pr),
             Waypoint(1.0, push_tcp(c, pivot, yaw, a1 + 6.0, pr), FINGER_CLOSED, "ease off", pr),
             Waypoint(2.0, above, FINGER_CLOSED, "clear", pr),
             Waypoint(1.0, above, FINGER_CLOSED, "wait", pr)]
    t_left = sum(w.duration for w in left)
    right = [Waypoint(3.0, g_up, OPEN_60, "over the lock case", hr),
             Waypoint(2.0, g, OPEN_60, "round the lock case", hr),
             Waypoint(1.0, g, GRIP_30N, "grip the lock case", hr)]
    right.append(Waypoint(t_left - 3.0 - sum(w.duration for w in right), g, GRIP_30N, "hold the cuff", hr))
    right += [Waypoint(1.0, g, OPEN_60, "let go", hr), Waypoint(2.0, g_up, OPEN_60, "clear", hr)]
    return {"left": left, "right": right}


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    """TCP position and rotation of an arm at the ready pose every trial starts from."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def placeholder_plans() -> dict[str, list[Waypoint]]:
    """Both arms wait at the ready pose (the real plan is made from the first camera frame)."""
    return {arm: [park(arm, 1.0, label="look")] for arm in ("left", "right")}


def demo():
    if PROBE == "m1":
        return m1_plans()
    return placeholder_plans()


# -- the full demo: planned from the camera, one cuff at a time ------------------------------------------------------------
ONLY = os.environ.get("CUFF_ONLY", "")  # "a": stop after cuff A (development)
FAIL = os.environ.get("CUFF_FAIL", "")  # a wrong outcome on purpose (README): loose, tight, nolock, holding (cuff B), fixed
LOOKS = 0 if PROBE == "m1" else (1 if ONLY == "a" else 2)
STAGE_CUFF = ("a", "b")
# Camera estimate minus truth, measured once on the first frames of three seeded starts (m): the shape fit sits this far
# off (the cuff rests 1 degree nose-down and the render's edges are soft).
EST_BIAS = {"a": (0.0023, 0.0004), "b": (0.0029, 0.0009)}
# how far from EST_PRIOR a cuff is looked for (m, m, degrees); the chain turns cuff B as cuff A is carried to its form
# (by 24 degrees on seed 0)
EST_REACH = {"a": (0.024, 0.032, 12.0), "b": (0.030, 0.050, 40.0)}
EST_PRIOR = {"a": (0.0, -0.025), "b": (0.0, -0.010)}  # the search's centre, from the nominal start (the pair shift's middle)
Q_PLAN = float(os.environ.get("CUFF_Q_PLAN", "-17.5"))  # the bar angle the fingertip is sent to (degrees): notch 8 to 9
TIP_HALF = (0.0077, 0.004)  # a closed fingertip, grip straight down, fingers along world y: half extent in x and in y
Z_PUSH = TOP + TIP + 0.002
Z_LEVER = TOP + TIP + 0.003
BACK_Y = 0.097  # the steadying fingertip rests against the lock case here on the cuff's Y axis (its pad reaches the end)
BACK_GAP = 0.0  # and this far off the case's side as planned (the cuff sits within a millimetre of where it was set)
TAB_OUT = 0.004  # the left fingertip's centre this far past the tab's centre on the cuff's Y axis: clear of the case end
LEVER_OVER = 0.007  # and sent this far past the lever's home (at 4 mm the lever went 4.6 mm of the 4.8 that lock it)
APPROACH = {"a": 0.055, "b": 0.034}  # the cuff is lined up this far short of the form, then slid on (cuff B
#                                        less: the chain to cuff A, already on its form, is 56 mm)


def _world2(c: str, pts) -> np.ndarray:
    return np.atleast_2d(np.asarray(pts, float)) @ BASE[c][:2, :2].T


def contact_alpha(c: str, q: float, radius: float = PUSH_R) -> float:
    """Polar angle (degrees) of a straight-down closed fingertip's centre, ``radius`` from the rivet, at which it
    touches the outside of the bar when the bar is at ``q`` degrees (coming round from the open side)."""
    bar = _world2(c, BAR_PTS @ _turn2(q).T)
    ex, ey = TIP_HALF
    edge = np.array([[x, y] for x in np.linspace(-ex, ex, 17) for y in (-ey, ey)]
                    + [[x, y] for y in np.linspace(-ey, ey, 9) for x in (-ex, ex)])
    a = 50.0 + q + 45.0
    while a > 50.0 + q - 20.0:
        centre = _world2(c, [[-radius * math.sin(math.radians(a)), radius * math.cos(math.radians(a))]])[0]
        pts = edge + centre
        best = float("inf")
        for p1, p2 in zip(bar[:-1], bar[1:]):
            ab = p2 - p1
            t = np.clip(((pts - p1) @ ab) / float(ab @ ab), 0.0, 1.0)
            best = min(best, float(np.linalg.norm(pts - (p1 + t[:, None] * ab), axis=1).min()))
        if best <= BAR_HW:
            return a
        a -= 0.1
    raise ValueError("no contact found")


def flat_tcp(xy, z: float) -> tuple:
    return (float(xy[0]), float(xy[1]), float(z))


def lean_tcp(tip_xy, rot: np.ndarray, low: float = 0.003, back: float = 0.0) -> tuple:
    """TCP of a leaning closed gripper whose fingertip's centre is over ``tip_xy`` with its lowest corner ``low``
    above the table, drawn ``back`` m up along its own axis."""
    tip = np.array([tip_xy[0], tip_xy[1], TOP + low + 0.004 * math.sin(PUSH_PITCH)])
    return tuple((tip - (TIP + back) * rot[:, 2]).tolist())


CORNER = 0.004 * math.cos(PUSH_PITCH)  # a leaning fingertip's leading corner ahead of its centre (m)
LEAN_L = push_rot((0.0, -1.0), os.environ.get("CUFF_FLIP_L", "0") == "1")  # left hand pushing toward -y, leaning back to +y
LEAN_R = push_rot((0.0, 1.0), os.environ.get("CUFF_FLIP_R", "1") == "1")  # right hand facing it, leaning back to -y
HOLD_PUSH_S = 2.5  # the fingertip waits this long at the end of the push for the arm to come up to its target


def cuff_plans(c: str, p0, yaw0: float, fail: str = "") -> dict[str, list[Waypoint]]:
    """Both hands' plans for one cuff whose rivet is seen at ``p0``, turned ``yaw0`` from its nominal heading. The
    right hand takes the lock case, lines the cuff up beside its form, slides it on and holds it. The left fingertip
    swings the bar round the form and pushes it in to the fit. The right hand lets go and rests a fingertip against
    the lock case's end; the left fingertip pushes the lever tab home against it. Both go back to the ready pose."""
    p1 = PIVOT_ON[c]
    pre = (p1[0], p1[1] - APPROACH[c])
    hr0, hr = _rz(yaw0) @ hold_rot(c), hold_rot(c)
    right = [Waypoint(2.5, hold_tcp(c, p0, yaw0, 0.08), OPEN_60, "over the lock case", hr0),
             Waypoint(1.5, hold_tcp(c, p0, yaw0), OPEN_60, "round the lock case", hr0),
             Waypoint(0.8, hold_tcp(c, p0, yaw0), GRIP_30N, "grip the lock case", hr0),
             Waypoint(0.6, hold_tcp(c, p0, yaw0, 0.004), GRIP_30N, "lift it clear of the table", hr0),
             Waypoint(2.5, hold_tcp(c, pre, 0.0, 0.004), GRIP_30N, "line the cuff up beside the form", hr),
             Waypoint(3.0, hold_tcp(c, p1, 0.0, 0.004), GRIP_30N, "slide the cuff round the form", hr),
             Waypoint(0.6, hold_tcp(c, p1, 0.0), GRIP_30N, "set it down", hr)]
    t_on = sum(w.duration for w in right)

    def tip(alpha):
        a = math.radians(alpha)
        return flat_tcp(cuff_point(c, p1, 0.0, -PUSH_R * math.sin(a), PUSH_R * math.cos(a)), Z_PUSH)

    a_start = PUSH_ALPHA0 + BAR_STOP + 22.0
    a_mid = PUSH_ALPHA0 + 26.0
    q_plan = {"loose": notch_deg(3) - 0.5 * DELTA, "tight": notch_deg(16) - 0.5 * DELTA}.get(fail, Q_PLAN)
    a_end = contact_alpha(c, q_plan)
    up = np.array([0.0, 0.0, 0.07])
    left = [Waypoint(3.0, tuple(np.array(tip(a_start)) + up), FINGER_CLOSED, "over the open bar", 0.0)]
    left.append(Waypoint(max(0.2, t_on - 3.0), tuple(np.array(tip(a_start)) + up), FINGER_CLOSED, "wait for the cuff", 0.0))
    left.append(Waypoint(1.2, tip(a_start), FINGER_CLOSED, "down behind the bar", 0.0))

    def arc(a_from, a_to, seconds_per_deg, label):
        n = max(1, int(math.ceil(abs(a_to - a_from) / 5.0)))
        for k in range(1, n + 1):
            left.append(Waypoint(abs(a_to - a_from) / n * seconds_per_deg, tip(a_from + (a_to - a_from) * k / n),
                                 FINGER_CLOSED, label, 0.0))

    arc(a_start, a_mid, 0.035, "swing the bar round the form")
    arc(a_mid, a_end + 6.0, 0.12, "push the bar into the lock")
    arc(a_end + 6.0, a_end, 0.30, "push the bar to the fit")
    end_up = np.array(tip(a_end + 6.0)) + up
    left += [Waypoint(HOLD_PUSH_S, tip(a_end), FINGER_CLOSED, "hold the push", 0.0),
             Waypoint(0.8, tip(a_end + 6.0), FINGER_CLOSED, "ease off", 0.0),
             Waypoint(1.0, tuple(end_up), FINGER_CLOSED, "up", 0.0)]
    t_fit = sum(w.duration for w in left) - 1.0
    right.append(Waypoint(t_fit - t_on, hold_tcp(c, p1, 0.0), GRIP_30N, "hold the cuff", hr))

    if fail in ("nolock", "holding"):  # wrong on purpose: the lever is never pushed (and, "holding", the hand stays on)
        left += [park("left", 2.5), park("left", 0.8, label="look")]
        if fail == "holding":
            right.append(Waypoint(sum(w.duration for w in left) - t_fit, hold_tcp(c, p1, 0.0), GRIP_30N,
                                  "hold the cuff", hr))
        else:
            right += [Waypoint(0.8, hold_tcp(c, p1, 0.0), OPEN_60, "let go", hr),
                      Waypoint(1.0, hold_tcp(c, p1, 0.0, 0.06), OPEN_60, "up", hr),
                      park("right", 2.5), park("right", 0.8, label="look")]
        _pad(left, right)
        return {"left": left, "right": right}

    # The lever. Both hands lean away from each other so their bodies clear (with the right hand still holding the lock
    # case the two arms met above it). The right hand lets go and rests a closed fingertip against the lock case's far
    # side at the case's end, as near the tab as it goes, so the push turns the cuff as little as it can (resting
    # 20 mm short of the tab, the cuff turned 4 degrees and the lever went 2 mm of its 5.5). The left fingertip takes
    # the tab at its outer end, clear of the case's end, and is sent LEVER_OVER past the lever's home to take up what
    # the cuff still gives.
    tab = cuff_point(c, p1, 0.0, TAB_C[0], TAB_C[1] + TAB_OUT)
    y_face = tab[1] + TAB[0] / 2 + CORNER  # the left fingertip's centre with its leading corner on the tab's face
    lev0, lev1 = (tab[0], y_face + 0.004), (tab[0], y_face - LEVER_TRAVEL - LEVER_OVER)
    backxy = cuff_point(c, p1, 0.0, CASE[0] / 2 + CORNER + BACK_GAP, BACK_Y)
    left += [Waypoint(1.5, (float(end_up[0]), 0.175, float(end_up[2]) + 0.02), FINGER_CLOSED, "out to the left", 0.0),
             Waypoint(2.0, lean_tcp(lev0, LEAN_L, back=0.07), FINGER_CLOSED, "over the lever tab", LEAN_L)]
    right += [Waypoint(0.8, hold_tcp(c, p1, 0.0), OPEN_60, "let go", hr),
              Waypoint(1.0, hold_tcp(c, p1, 0.0, 0.06), OPEN_60, "up", hr),
              Waypoint(2.0, lean_tcp(backxy, LEAN_R, back=0.07), FINGER_CLOSED, "over the lock case's far side", LEAN_R),
              Waypoint(1.2, lean_tcp(backxy, LEAN_R), FINGER_CLOSED, "fingertip down beside the lock case", LEAN_R)]
    t_back = sum(w.duration for w in right)
    left.append(Waypoint(max(0.2, t_back - sum(w.duration for w in left)), lean_tcp(lev0, LEAN_L, back=0.07),
                         FINGER_CLOSED, "wait for the steadying hand", LEAN_L))
    left += [Waypoint(1.2, lean_tcp(lev0, LEAN_L), FINGER_CLOSED, "down beside the lever tab", LEAN_L),
             Waypoint(1.5, lean_tcp(lev1, LEAN_L), FINGER_CLOSED, "push the lever tab home", LEAN_L),
             Waypoint(1.5, lean_tcp(lev1, LEAN_L), FINGER_CLOSED, "hold it home", LEAN_L),
             Waypoint(0.6, lean_tcp(lev0, LEAN_L), FINGER_CLOSED, "ease off", LEAN_L)]
    t_lock = sum(w.duration for w in left)
    left += [Waypoint(1.0, lean_tcp(lev0, LEAN_L, back=0.07), FINGER_CLOSED, "up", LEAN_L),
             park("left", 2.5), park("left", 0.8, label="look")]
    right.append(Waypoint(t_lock - t_back, lean_tcp(backxy, LEAN_R), FINGER_CLOSED, "steady the lock case", LEAN_R))
    right += [Waypoint(1.0, lean_tcp(backxy, LEAN_R, back=0.07), FINGER_CLOSED, "up", LEAN_R),
              park("right", 2.5), park("right", 0.8, label="look")]
    _pad(left, right)
    return {"left": left, "right": right}


def _pad(left: list, right: list) -> None:
    """Make both plans the same length (the shorter hand waits at the ready pose)."""
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    if tl < tr - 1e-9:
        left.append(park("left", tr - tl, label="look"))
    elif tr < tl - 1e-9:
        right.append(park("right", tl - tr, label="look"))


def plan_stage(stage: int, img: np.ndarray, module) -> dict[str, list[Waypoint]]:
    """The next stretch of the demo, planned from a top-camera frame taken with both arms at the ready pose."""
    c = STAGE_CUFF[stage]
    if FAIL == "fixed":  # wrong on purpose: no camera, the nominal start in the setup text
        xy, yaw, fit = np.array(PIVOT_START[c]), 0.0, 0.0
    else:
        prior = (PIVOT_START[c][0] + EST_PRIOR[c][0], PIVOT_START[c][1] + EST_PRIOR[c][1])
        xy, yaw, fit = cuffdemo.fit_cuff(img, module, c, prior, 0.0, EST_REACH[c])
        xy = xy - np.array(EST_BIAS[c])
    print(f"cuffdemo: cuff {c} rivet at ({xy[0]:.4f}, {xy[1]:.4f}), turned {math.degrees(yaw):+.2f} deg "
          f"(shape on steel {fit:.2f})", flush=True)
    # "holding" spoils only the last cuff: cuff A is done properly and the right hand is still on cuff B at the end
    fail = "" if FAIL == "holding" and c != STAGE_CUFF[-1] else FAIL
    return cuff_plans(c, xy, yaw, fail)


def nominal_plans() -> dict[str, list[Waypoint]]:
    """Both cuffs' plans for the nominal layout, end to end (for reach checks; the demo plans from the camera)."""
    out = placeholder_plans()
    for c in STAGE_CUFF:
        more = cuff_plans(c, PIVOT_START[c], 0.0)
        for arm in out:
            out[arm] = out[arm] + more[arm]
    return out



M1_END = max(sum(w.duration for w in plan) for plan in m1_plans().values()) if PROBE == "m1" else 0.0

import cuffdemo  # noqa: E402  (the demo's control hooks for yam_scripted)

cuffdemo.install()
cuffdemo_task = True  # tells cuffdemo this task uses its hooks
