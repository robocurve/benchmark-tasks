"""Streak an agar plate for isolated colonies: four quadrants with a disposable inoculating loop, without gouging it.

On the station table, in front of the right arm, an uncovered agar plate rests on the bench (Fisherbrand FB0875712
Petri dish base, 92 mm outside / 88 mm inside, 20 mL of agar 3.3 mm deep). A marker cross on its underside divides it
into four quadrants, numbered 1 to 4 (1 far right, 2 far left, 3 near left, 4 near right, seen from the robot), and
a 10 uL drop of the sample (blue-dyed water) sits in quadrant 1 near the rim. A sterile 10 uL disposable loop (200 mm,
polypropylene) lies on the bench to the right of the plate, and a discard tray stands between the arms. The robot
must streak the plate as the CDC's four-quadrant method does: drag the loop through the drop and streak quadrant 1
back and forth, then streak quadrants 2, 3 and 4 in turn, each starting with a few strokes back through the quadrant
before it (quadrant 4 trailing off without touching quadrant 1), with the loop gliding on the agar, never digging
in, and finally put the loop in the discard tray.

What makes it hard: a 3.6 mm handle has to be picked up off the bench and held like a pen, and its 4.5 mm ring
drawn over a soft gel in long controlled strokes with almost no force: lifted off it streaks nothing, pressed past
about a quarter of a newton it tears the agar; and each quadrant's strokes must dip back into the previous one's.

Simulation: the dish and agar are fixed rigid bodies; the loop a free rigid body with compliant contact (its
flexible polypropylene shaft). The streak is recorded from the loop's contact with the agar (contact force from
PhysX reports): where the ring rests on the agar with a force of at least F_TOUCH it draws; above F_GOUGE it tears
the agar (from the gel strength of 1.5 % bacteriological agar, see README).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Streak the plate for isolated colonies: drag the loop through the drop and streak quadrant 1, then "
               "streak quadrants 2, 3 and 4 in turn, each starting with a few strokes back through the one before, "
               "without digging into the agar. Then put the loop in the discard tray.")

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- the plate (sizes as work/streak/make_parts.py builds them)
DISH_OD, DISH_ID, WALL_H, FLOOR_T = 92 * MM, 88 * MM, 13.5 * MM, 1.2 * MM
AGAR_DEPTH = 20e-6 / (math.pi * (DISH_ID / 2) ** 2)
AGAR_TOP_H = FLOOR_T + AGAR_DEPTH  # above the table
AGAR_R = DISH_ID / 2 - 0.1 * MM
PLATE_XY = (0.34, -0.06)
DROP_XY = np.array([26.4 * MM, -11.2 * MM])  # in the plate's frame
DROP_R = 2.5 * MM
PLATE = UsdBody("plate", usd="assets/plate.usdc", pos=(PLATE_XY[0], PLATE_XY[1], TOP), mass=None,
                size=(DISH_OD, DISH_OD, WALL_H), friction=(0.4, 0.35))
AGAR = UsdBody("agar", usd="assets/agar.usdc", pos=(PLATE_XY[0], PLATE_XY[1], TOP), mass=None,
               size=(DISH_ID, DISH_ID, AGAR_DEPTH), friction=(0.15, 0.1))

# -- the loop: origin at the butt, along +x; ring 4.5 mm across, its centre 197.75 mm from the butt
LOOP_L, HANDLE_R = 200 * MM, 1.8 * MM
RING_R, WIRE_R = 2.25 * MM, 0.4 * MM
RING_C = LOOP_L - RING_R
LOOP_MASS = 1.6e-3
GRASP_AT = 140 * MM  # where the demo holds the handle, from the butt: 60 mm short of the ring's far edge
HEAD = math.radians(180.0)  # the loop lies pointing straight back at the robot, ring nearest
H = np.array([math.cos(HEAD), math.sin(HEAD), 0.0])
W_ = np.array([-H[1], H[0], 0.0])
GRASP_XY = np.array([0.26, -0.20])
_butt = np.append(GRASP_XY, 0.0) - GRASP_AT * H
NECK_X = 175 * MM  # the neck's hinge (the handle's end), from the butt
NECK_K = 0.02  # N m/rad: the flexible neck; 0.1 N at the ring's far edge bends it about 3 mm
LOOP = UsdBody("loop", usd="assets/loop.usdc", pos=(float(_butt[0]), float(_butt[1]), TOP + HANDLE_R + 0.2 * MM),
               rot_xyzw=(0.0, 0.0, math.sin(HEAD / 2), math.cos(HEAD / 2)), mass=LOOP_MASS,
               size=(LOOP_L, 2 * HANDLE_R, 2 * HANDLE_R), articulation=True, material=False, solver_iters=32,
               joint_pos=(("neck", 0.0),), joint_stiffness=NECK_K, joint_damping=2e-4, joint_armature=1e-6)

# -- the discard tray between the arms: an autoclavable polypropylene pan, 25 x 18 cm, 4 cm deep
TRAY_XY, TRAY_SIZE, TRAY_H, TRAY_T = (0.16, 0.05), (0.25, 0.18), 0.04, 0.003
_tc = (0.75, 0.75, 0.72)
TRAY = [Shape("tray_floor", pos=(TRAY_XY[0], TRAY_XY[1], TOP + TRAY_T / 2), size=(*TRAY_SIZE, TRAY_T), color=_tc)]
for _n, (_dx, _dy, _sx, _sy) in {"n": (-1, 0, TRAY_T, TRAY_SIZE[1]), "f": (1, 0, TRAY_T, TRAY_SIZE[1]),
                                 "r": (0, -1, TRAY_SIZE[0], TRAY_T), "l": (0, 1, TRAY_SIZE[0], TRAY_T)}.items():
    TRAY.append(Shape(f"tray_wall_{_n}", pos=(TRAY_XY[0] + _dx * (TRAY_SIZE[0] - TRAY_T) / 2,
                                              TRAY_XY[1] + _dy * (TRAY_SIZE[1] - TRAY_T) / 2, TOP + TRAY_H / 2),
                      size=(_sx, _sy, TRAY_H), color=_tc))

OBJECTS = [PLATE, AGAR, LOOP, *TRAY]

# -- the streak rule and scoring
F_TOUCH = 0.01  # N: the ring resting on the agar
F_GOUGE = 0.25  # N: 550 g/cm2 (the lowest gel strength suppliers give for 1.5 % agar) over the ring's 0.045 cm2 patch
NEAR = 0.6 * MM  # a ring point this close to the agar's surface is on it
L_MIN = {1: 40 * MM, 2: 30 * MM, 3: 30 * MM, 4: 30 * MM}  # streak drawn in each quadrant, in its own turn
ENTRIES_MIN = 2  # strokes from quadrant k back into quadrant k-1
IN_TRAY_ABOVE = 20 * MM  # full credit needs the loop in the discard tray, no part higher than this above its walls
GRID = 0.5 * MM

_gx = np.arange(-AGAR_R, AGAR_R + GRID / 2, GRID)
_GX, _GY = np.meshgrid(_gx, _gx, indexing="ij")
_IN = _GX ** 2 + _GY ** 2 <= (AGAR_R - 0.5 * MM) ** 2
AGAR_PTS = np.stack([_GX[_IN], _GY[_IN]], -1)  # the agar's surface, in the plate's frame (m)
_A = np.linspace(0, 2 * math.pi, 24, endpoint=False)
RING_PTS = np.stack([RING_C - NECK_X + RING_R * np.cos(_A), RING_R * np.sin(_A), np.zeros_like(_A)], -1)  # tip frame
HANDLE_PTS = np.stack([np.linspace(0, NECK_X, 18), np.zeros(18), np.zeros(18)], -1)  # handle frame


def quadrant(xy) -> int:
    """1 far right (x > 0, y < 0), 2 far left, 3 near left, 4 near right, in the plate's frame."""
    x, y = float(xy[0]), float(xy[1])
    if x >= 0:
        return 1 if y < 0 else 2
    return 3 if y >= 0 else 4


def new_state() -> dict:
    return dict(phase=0, length={q: 0.0 for q in range(1, 5)}, entries={q: 0 for q in range(1, 5)}, prev=None,
                prev_q=0, drop=False, redip=False, q4_in_q1=False, gouge=False, gouge_at=None, fingers=False,
                streak=np.zeros(len(AGAR_PTS), np.int8), f=0.0, fmax=0.0, t=0.0, last=-1.0, clear=True)


_STATE = new_state()


def advance(st: dict, contact, force: float, marks=None) -> None:
    """One control step. ``contact``: the ring's contact point on the agar in the plate's frame (m), or None when the
    ring is off it; ``force``: its smoothed force (N); ``marks``: agar points (indices) under the ring."""
    if force > F_GOUGE:
        if not st["gouge"]:
            st["gouge_at"] = None if contact is None else tuple(np.round(np.asarray(contact) * 1e3, 1))
        st["gouge"] = True
    if contact is None:
        st["prev"] = None
        return
    c = np.asarray(contact, float)
    q = quadrant(c)
    if q == st["phase"] + 1:
        st["phase"] = q  # the next quadrant's streak begins
    ph = st["phase"]
    if st["prev"] is not None:
        step = float(np.linalg.norm(c - st["prev"]))
        if q == ph and ph >= 1:
            st["length"][q] += step
        if ph >= 2 and st["prev_q"] == ph and q == ph - 1:
            st["entries"][ph] += 1
    if np.linalg.norm(c - DROP_XY) <= DROP_R + 1 * MM:
        if ph == 1:
            st["drop"] = True
        elif ph >= 2:
            st["redip"] = True
    if ph == 4 and q == 1:
        st["q4_in_q1"] = True
    if marks is not None and len(marks):
        st["streak"][marks] = np.where(st["streak"][marks] == 0, max(ph, 1), st["streak"][marks])
    st["prev"], st["prev_q"] = c, q


def quadrant_ok(st: dict, q: int) -> bool:
    if st["length"][q] < L_MIN[q]:
        return False
    if q == 1:
        return st["drop"]
    if q == 4 and st["q4_in_q1"]:
        return False
    return st["entries"][q] >= ENTRIES_MIN


def score(objects=None, st=None) -> float:
    """0.25 for each quadrant streaked in turn (1, then 2, 3, 4): at least 40 mm of streak in quadrant 1 drawn through
    the drop; at least 30 mm in each later quadrant with at least 2 strokes back into the one before (quadrant 4 never
    touching quadrant 1). 0 if the agar was gouged or a gripper touched it; at most 0.25 if the loop went back to the
    drop after quadrant 1; at most 0.75 until the loop lies in the discard tray, let go."""
    st = _STATE if st is None else st
    if st["gouge"] or st["fingers"]:
        return 0.0
    n = 0
    for q in (1, 2, 3, 4):
        if not quadrant_ok(st, q):
            break
        n += 1
    s = 0.25 * n
    if st["redip"]:
        s = min(s, 0.25)
    if not st["clear"]:
        s = min(s, 0.75)
    return s


def success(objects) -> bool:
    return score() == 1.0


@on_reset
def _reset(environment) -> None:
    _STATE.clear()
    _STATE.update(new_state())


def plate_frame(p) -> np.ndarray:
    """World xy (or xyz) to the plate's frame (the plate is fixed, unrotated)."""
    p = np.asarray(p, float)
    return p[..., :2] - np.asarray(PLATE_XY)


def ring_points(tip) -> np.ndarray:
    """The ring's points (world) from the tip link's pose (pos, R)."""
    return np.asarray(tip[0]) + RING_PTS @ np.asarray(tip[1]).T


def ring_contact(tip):
    """The ring's points on the agar (world), or an empty array."""
    pts = ring_points(tip)
    h = pts[:, 2] - WIRE_R - (TOP + AGAR_TOP_H)
    on = pts[h <= max(NEAR, h.min() + 0.3 * MM)] if h.min() <= NEAR else pts[:0]
    return on


def loop_in_tray(handle, tip) -> bool:
    """The whole loop (handle and ring) inside the discard tray's walls and lower than 2 cm above their top."""
    pts = np.vstack([np.asarray(handle[0]) + HANDLE_PTS @ np.asarray(handle[1]).T, ring_points(tip)])
    half = np.array(TRAY_SIZE) / 2 - TRAY_T
    inside = np.all(np.abs(pts[:, :2] - np.array(TRAY_XY)) <= half, axis=1)
    return bool(np.all(inside & (pts[:, 2] < TOP + TRAY_H + IN_TRAY_ABOVE)))


_TREE: list = []
_TRACE: list = []


def _quat_R(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def link_poses(environment):
    """((pos, R) of the handle link, (pos, R) of the tip link), station frame."""
    base = environment.unwrapped
    asset = base.scene["loop"]
    origin = getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy()
    data = asset.data
    pose = getattr(data, "body_link_pose_w", None)
    pose = getattr(pose, "torch", pose)
    pose = pose[0].detach().cpu().numpy().astype(float)
    names = list(asset.body_names)
    out = []
    for n in ("handle", "tip"):
        p = pose[names.index(n)]
        out.append((p[:3] - origin, _quat_R(p[3:7])))
    return out[0], out[1]


@on_step
def _watch(environment) -> None:
    from yam_arena.scene import object_states

    import contactkit
    import touch

    dt = environment.unwrapped.step_dt
    loop = object_states(environment, ["loop"])["loop"]
    handle, tip = link_poses(environment)
    f = 0.5 * contactkit.loop_on_agar(environment) + 0.5 * _STATE["f"]
    _STATE["f"] = f
    _STATE["fmax"] = max(_STATE["fmax"], f)
    on = ring_contact(tip)
    _pts = ring_points(tip)
    _hh = _pts[:, 2] - WIRE_R - (TOP + AGAR_TOP_H)
    _STATE["h"] = float(_hh.min())
    if os.environ.get("STREAK_TRACE"):  # the ring's lowest point every step, for calibrating the demo's heights
        lo = _pts[int(np.argmin(_hh))]
        _TRACE.append((_STATE["t"], *plate_frame(lo), _STATE["h"], f))
        if len(_TRACE) % 20 == 0:
            np.save(os.environ["STREAK_TRACE"], np.array(_TRACE))
    contact, marks = None, None
    if len(on) and f >= F_TOUCH:
        contact = plate_frame(on).mean(0)
        if not _TREE:
            from scipy.spatial import cKDTree
            _TREE.append(cKDTree(AGAR_PTS))
        idx = _TREE[0].query_ball_point(plate_frame(on), r=NEAR)
        idx = [np.asarray(i, int) for i in idx]
        marks = np.unique(np.concatenate(idx)) if len(idx) else None
    advance(_STATE, contact, f, marks)
    t = touch.touching(environment, ["agar", "loop"])
    if t["agar"]:
        _STATE["fingers"] = True
    _STATE["clear"] = loop_in_tray(handle, tip) and not t["loop"]  # lying in the tray, let go
    _STATE["t"] += dt
    if os.environ.get("STREAK_LOG") and (_STATE["t"] - _STATE["last"] >= 0.5 or (contact is not None and f > 0.15)):
        _STATE["last"] = _STATE["t"]
        L = _STATE["length"]
        c = "-" if contact is None else f"({contact[0] * 1e3:.0f},{contact[1] * 1e3:.0f})"
        print(f"LOG t={_STATE['t']:.2f} f={f:.3f} fmax={_STATE['fmax']:.3f} at={c} phase={_STATE['phase']} "
              f"len={L[1] * 1e3:.0f}/{L[2] * 1e3:.0f}/{L[3] * 1e3:.0f}/{L[4] * 1e3:.0f}mm "
              f"entries={_STATE['entries'][2]}/{_STATE['entries'][3]}/{_STATE['entries'][4]} drop={_STATE['drop']} "
              f"redip={_STATE['redip']} gouge={_STATE['gouge']}{_STATE['gouge_at'] or ''} fingers={_STATE['fingers']} "
              f"clear={_STATE['clear']} h={_STATE['h'] * 1e3:.2f}mm loop_z={(loop.pos[2] - TOP) * 1e3:.1f}mm score={score():.2f}", flush=True)


# -- the scripted demo: the right arm picks the loop up, holds it like a pen and streaks the four quadrants ---------
HOVER = TOP + 0.15
TILT = math.radians(50.0)  # the loop's angle to the agar while streaking
U_T = math.cos(TILT) * H - math.sin(TILT) * np.array([0.0, 0.0, 1.0])  # along the tilted loop, towards the ring
REACH = RING_C + RING_R - GRASP_AT  # from the hand to the ring's far edge
DEPTH = float(os.environ.get("STREAK_DEPTH", "0.003"))
LIFT_ALL = float(os.environ.get("STREAK_LIFT", "0"))  # calibration runs: the whole plan this much higher  # how far past the surface the plan aims the ring (the neck flexes)
SPEED = 0.020  # m/s along the agar

# each quadrant's zigzag in its own coordinates (mm): u along the boundary it shares with the quadrant before it
# (away from the centre), v into the quadrant; v < 0 is the quadrant before (quadrant 1: u along x, v along -y).
PATH_SCALE = 0.8
Q1_PATH = [(36, 4), (33, 14), (28, 4), (26, 22), (20, 4), (18, 26), (12, 4), (10, 30)]
QN_PATH = [(34, 8), (30, -6), (26, 10), (22, -6), (18, 10), (14, -6), (10, 10), (30, 18), (8, 24), (22, 30)]
_TO_PLATE = {1: lambda u, v: (u, -v), 2: lambda u, v: (u, v), 3: lambda u, v: (-v, u), 4: lambda u, v: (-u, -v)}


def quadrant_path(q: int) -> list:
    """Quadrant q's streak as plate-frame points (m)."""
    pts = Q1_PATH if q == 1 else QN_PATH
    return [np.array(_TO_PLATE[q](u, v)) * MM * PATH_SCALE for u, v in pts]


_CAL = pathlib.Path(__file__).resolve().parent / "assets" / "height_cal.npz"


def height_error(xy_plate) -> float:
    """How far above its planned height the ring rides at plate point ``xy_plate`` (m), measured once by running the
    plan 3 mm up, off the agar (assets/height_cal.npz: plate xy and error); 0 without that file."""
    if os.environ.get("STREAK_NOCAL") or not _CAL.is_file():
        return 0.0
    d = np.load(_CAL)
    dist = np.linalg.norm(d["xy"] - np.asarray(xy_plate)[None], axis=1)
    near = np.argsort(dist)[:6]
    w = 1.0 / (dist[near] + 1e-3) ** 2
    local = float((d["err"][near] * w).sum() / w.sum())
    a = float(np.clip((dist[near[0]] - 0.004) / 0.006, 0.0, 1.0))  # beyond the measured cells: the mean error
    return (1 - a) * local + a * float(d["err"].mean())


def hand_at(xy_plate, lift: float = 0.0) -> tuple:
    """The TCP with the ring's far edge at plate point ``xy_plate`` (m), DEPTH into the agar, or ``lift`` above it."""
    z = TOP + AGAR_TOP_H - DEPTH + lift + WIRE_R + LIFT_ALL - height_error(xy_plate)
    p = np.array([xy_plate[0] + PLATE_XY[0], xy_plate[1] + PLATE_XY[1], z])
    return tuple(float(c) for c in p - REACH * U_T)


def demo():
    """Right arm: over the loop's handle, down, grip, lift, turn the loop down to 35 degrees, then for each quadrant:
    down onto its first point, draw its zigzag at 20 mm/s, lift 8 mm; then carry the loop over the discard tray and
    let go. The left arm stays at its start."""
    from tiltkit import grip, install
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    install()
    approach_t = -math.sin(TILT) * H - math.cos(TILT) * np.array([0.0, 0.0, 1.0])
    flat = grip(approach=(0, 0, -1), closing=W_)
    tilted = grip(approach=tuple(approach_t), closing=tuple(W_), joint=True)
    tilted_move = grip(approach=tuple(approach_t), closing=tuple(W_))
    gx, gy = GRASP_XY
    pick_z = TOP + HANDLE_R + 12 * MM  # pads' tips about 1 mm above the bench, round the handle's middle
    HOLD = 0.0  # fingers commanded shut on the 3.6 mm handle (a firm pinch)
    right = [
        W(2.0, (0.22, -0.30, HOVER), O, "start", flat),
        W(1.5, (gx, gy, TOP + 0.06), -0.012, "over the loop's handle", flat),
        W(1.5, (gx, gy, pick_z), -0.012, "down round the handle", flat),
        W(1.0, (gx, gy, pick_z), HOLD, "grip", flat),
        W(2.0, (gx, gy, TOP + 0.14), HOLD, "lift", flat),
    ]
    first = quadrant_path(1)[0]
    right.append(W(4.0, hand_at(first, 0.06), HOLD, "turn the loop down, high over quadrant 1", tilted))
    right.append(W(1.5, hand_at(first, 0.015), HOLD, "down over quadrant 1", tilted_move))
    for q in (1, 2, 3, 4):
        path = quadrant_path(q)
        if q > 1:
            right.append(W(1.5, hand_at(path[0], 0.015), HOLD, f"over quadrant {q}", tilted_move))
        right.append(W(0.8, hand_at(path[0], 0.002), HOLD, f"down to quadrant {q}", tilted_move))
        right.append(W(1.5, hand_at(path[0]), HOLD, f"onto quadrant {q}", tilted_move))
        for k in range(1, len(path)):
            d = float(np.linalg.norm(path[k] - path[k - 1]))
            right.append(W(max(0.4, d / SPEED), hand_at(path[k]), HOLD, f"streak {q}", tilted_move))
        right.append(W(0.8, hand_at(path[-1], 0.015), HOLD, f"lift off quadrant {q}", tilted_move))
    # over the tray the loop is turned level again (the pickup grip) and laid on the tray's floor along x: ring end
    # towards the robot, from 2 cm inside the near wall
    tx = TRAY_XY[0] - TRAY_SIZE[0] / 2 + TRAY_T + 0.02 + (RING_C + RING_R - GRASP_AT)
    lay_z = TOP + TRAY_T + HANDLE_R + 12 * MM + 6 * MM  # the handle about 6 mm above the tray's floor
    right += [
        W(1.5, hand_at(quadrant_path(4)[-1], 0.06), HOLD, "lift clear", tilted_move),
        W(3.0, (tx, TRAY_XY[1], TOP + 0.12), HOLD, "level, over the discard tray", grip(approach=(0, 0, -1), closing=W_, joint=True)),
        W(1.5, (tx, TRAY_XY[1], lay_z), HOLD, "down into the tray", flat),
        W(0.8, (tx, TRAY_XY[1], lay_z), -0.012, "let go", flat),
        W(1.0, (tx, TRAY_XY[1], TOP + 0.10), -0.012, "up", flat),
        W(1.5, (0.22, -0.30, HOVER), -0.012, "back", flat),
    ]
    return {"right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-made contact sequences."""

    def run(points_by_q, force=0.05, extra=None, clear=True):
        st = new_state()
        for q, pts in points_by_q:
            dense = []
            for a, b in zip(pts[:-1], pts[1:]):
                n = max(2, int(np.linalg.norm(b - a) / (1 * MM)))
                dense += [a + (b - a) * t for t in np.linspace(0, 1, n, endpoint=False)]
            dense.append(pts[-1])
            for c in dense:
                advance(st, c, force)
            advance(st, None, 0.0)
        if extra:
            extra(st)
        st["clear"] = clear
        return score(st=st), st

    full = [(q, quadrant_path(q)) for q in (1, 2, 3, 4)]
    cases = {
        "the demo's four quadrants": (full, {}, 1.0),
        "quadrant 1 only": (full[:1], {}, 0.25),
        "quadrants 1-2": (full[:2], {}, 0.5),
        "quadrants 1-3": (full[:3], {}, 0.75),
        "all four, loop not yet in the tray": (full, {"clear": False}, 0.75),
        "pressed too hard": (full, {"force": 0.4}, 0.0),
    }
    for name, (seq, kw, want) in cases.items():
        got, st = run(seq, **kw)
        assert abs(got - want) < 1e-9, (name, got, want, st["length"], st["entries"], st["drop"])
    # quadrant 1 missing the drop: nothing counts
    miss = [np.array(p) for p in [(0.012, -0.004), (0.010, -0.030), (0.016, -0.004), (0.018, -0.030),
                                  (0.006, -0.004), (0.004, -0.030)]]
    assert run([(1, miss)] + full[1:])[0] == 0.0
    # quadrant 2 drawn without strokes back into quadrant 1: only quadrant 1 counts
    q2_own = [p for p in quadrant_path(2) if p[1] > 0.003]
    assert run(full[:1] + [(2, q2_own)] + full[2:])[0] == 0.25
    # a quadrant skipped: quadrant 3 drawn straight after 1 does not count
    assert run([full[0], full[2], full[3]])[0] == 0.25
    # back to the drop after quadrant 1 (re-inoculating): at most 0.25
    redip = full[:2] + [(2, [np.array([0.010, 0.010]), DROP_XY + 0.0, np.array([0.010, 0.010])])] + full[2:]
    assert run(redip)[0] == 0.25
    # quadrant 4 running into quadrant 1: quadrant 4 doesn't count
    q4_bad = quadrant_path(4) + [np.array([-0.010, -0.020]), np.array([0.010, -0.020])]
    assert run(full[:3] + [(4, q4_bad)])[0] == 0.75
    # a gripper touching the agar: 0
    assert run(full, extra=lambda st: st.update(fingers=True))[0] == 0.0
    # hovering (no contact) draws nothing
    st = new_state()
    for c in quadrant_path(1):
        advance(st, None, 0.0)
    assert score(st=st) == 0.0
    # the loop lying in the tray counts; held over the plate or above the tray does not
    I = np.eye(3)
    flat_in = ((np.array([TRAY_XY[0] + 0.10, TRAY_XY[1], TOP + 0.005]), _quat_R((0, 0, 1, 0))),)
    tip_in = (flat_in[0][0] + np.array([-NECK_X, 0, 0]), _quat_R((0, 0, 1, 0)))
    assert loop_in_tray(flat_in[0], tip_in)
    held_high = (flat_in[0][0] + np.array([0, 0, 0.10]), flat_in[0][1])
    assert not loop_in_tray(held_high, (tip_in[0] + np.array([0, 0, 0.10]), tip_in[1]))
    over_plate = (np.array([PLATE_XY[0] + 0.1, PLATE_XY[1], TOP + 0.03]), _quat_R((0, 0, 1, 0)))
    assert not loop_in_tray(over_plate, (over_plate[0] + np.array([-NECK_X, 0, 0]), I))
    # geometry: every planned point on the agar, clear of the wall by the ring's size, and the drop in quadrant 1
    for q in (1, 2, 3, 4):
        for p in quadrant_path(q):
            assert np.linalg.norm(p) < AGAR_R - 2 * RING_R, (q, p)
    assert quadrant(DROP_XY) == 1
    # the tilted loop's shaft clears the dish's wall (13.5 mm) wherever the ring works: along the shaft (+x from the
    # ring) the wall is reached only once the shaft is 2 mm or more above its rim
    for q in (1, 2, 3, 4):
        for p in quadrant_path(q):
            to_wall = math.sqrt((DISH_ID / 2) ** 2 - p[1] ** 2) - p[0]
            assert to_wall * math.tan(TILT) > WALL_H - AGAR_TOP_H + 2 * MM, (q, p, to_wall)
    assert abs(AGAR_DEPTH - 3.29 * MM) < 0.05 * MM
    assert 2 * HANDLE_R < 0.094
    # the loop lies clear of the plate and the tray at the start
    ring0 = np.append(GRASP_XY, 0) + (RING_C - GRASP_AT) * H
    assert np.linalg.norm(ring0[:2] - PLATE_XY) > DISH_OD / 2 + 0.01
    # the tray clears the plate: its nearest point at least 1 cm outside the dish
    near = np.clip(np.asarray(PLATE_XY), np.array(TRAY_XY) - np.array(TRAY_SIZE) / 2, np.array(TRAY_XY) + np.array(TRAY_SIZE) / 2)
    assert np.linalg.norm(near - PLATE_XY) > DISH_OD / 2 + 0.01
    assert TRAY_SIZE[0] - 2 * TRAY_T > LOOP_L + 0.02  # the loop lies in it lengthwise
    return len(cases) + 8


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass; agar points {len(AGAR_PTS)}")
    task = load(__file__)
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
