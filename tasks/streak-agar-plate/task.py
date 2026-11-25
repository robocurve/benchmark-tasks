"""Streak an agar plate for isolated colonies: four quadrants, a fresh disposable loop for each, without gouging it.

On the station table, in front of the right arm, an uncovered agar plate rests on the bench (Fisherbrand FB0875712
Petri dish base, 92 mm outside / 88 mm inside, 20 mL of agar 3.3 mm deep). A marker cross on its underside divides it
into four quadrants, numbered 1 to 4 (1 far right, 2 far left, 3 near left, 4 near right, seen from the robot), and
a 10 uL drop of the sample (an E. coli K-12 culture) sits in quadrant 1 near the rim. Four sterile 10 uL disposable
loops (200 mm, polypropylene) lie side by side on the bench to the right of the plate, and a discard tray stands
between the arms. The robot streaks the plate the way the CDC's four-quadrant method does it with plastic loops: a
new loop for each quadrant, discarded after it. The first loop goes through the drop and streaks quadrant 1; each
next loop starts with strokes that cross the streaks just drawn in the quadrant before, then streaks its own
quadrant (quadrant 4 never touching quadrant 1), with the ring gliding on the agar, never digging in; each used loop
goes into the discard tray. After overnight incubation colonies grow wherever a ring rested on the agar, so the
plate itself records every streak, every crossing and any gouge.

What makes it hard: four times, a 3.6 mm handle has to be picked up off the bench and held like a pen, and its
4.5 mm ring drawn over a soft gel in long controlled strokes with almost no force: lifted off it streaks nothing,
pressed past about a quarter of a newton it tears the agar; and each new loop has to find and cross the thin
streaks the previous one left.

Simulation: the dish and agar are fixed rigid bodies; each loop is a handle and, on a sprung hinge, its flexible
neck and ring. The streak is recorded from each loop's contact with the agar (contact force from PhysX reports):
where the ring rests on the agar with a force of at least F_TOUCH it marks the agar under it; above F_GOUGE it tears
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

INSTRUCTION = ("Streak the plate for isolated colonies, using a new loop for each quadrant: with the first loop, "
               "drag through the drop and streak quadrant 1; then with a fresh loop each time, streak quadrants 2, 3 "
               "and 4 in turn, each starting with a few strokes that cross the streaks of the quadrant before. Don't "
               "dig into the agar. Put each used loop in the discard tray.")

MM = 0.001
TOP = TABLE_TOP[SCENE]

# -- the plate (sizes as work/streak/make_parts.py builds them)
DISH_OD, DISH_ID, WALL_H, FLOOR_T = 92 * MM, 88 * MM, 13.5 * MM, 1.2 * MM
AGAR_DEPTH = 20e-6 / (math.pi * (DISH_ID / 2) ** 2)
AGAR_TOP_H = FLOOR_T + AGAR_DEPTH  # above the table
AGAR_R = DISH_ID / 2 - 0.1 * MM
PLATE_XY = (0.34, -0.06)
DROP_XY = np.array([16.0 * MM, -14.0 * MM])  # in the plate's frame
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
LOOP_X = 0.26  # the demo's grasp point on every loop's handle, x (station frame)
LOOP_YS = (-0.165, -0.200, -0.235, -0.270)  # the four loops side by side, 35 mm apart; used in this order by the demo
LOOP_NAMES = tuple(f"loop{k}" for k in range(1, 5))
GRASP_XY = np.array([LOOP_X, LOOP_YS[0]])
SHEET_T = 0.3 * MM  # the sterile paper sheet under the loops
NECK_X = 175 * MM  # the neck's hinge (the handle's end), from the butt
NECK_K = 0.02  # N m/rad: the flexible neck; 0.1 N at the ring's far edge bends it about 3 mm


def _loop(name: str, y: float) -> UsdBody:
    butt = np.array([LOOP_X, y, 0.0]) - GRASP_AT * H
    return UsdBody(name, usd="assets/loop.usdc", pos=(float(butt[0]), float(butt[1]), TOP + HANDLE_R + 0.2 * MM),
                   rot_xyzw=(0.0, 0.0, math.sin(HEAD / 2), math.cos(HEAD / 2)), mass=LOOP_MASS,
                   size=(LOOP_L, 2 * HANDLE_R, 2 * HANDLE_R), articulation=True, material=False, solver_iters=32,
                   joint_pos=(("neck", 0.0),), joint_stiffness=NECK_K, joint_damping=2e-4, joint_armature=1e-6)


LOOPS = [_loop(n, y) for n, y in zip(LOOP_NAMES, LOOP_YS)]
# the sterile field the loops lie on (an autoclaved paper sheet, 24 x 15 cm): their rings touch nothing else before use.
# Drawn only (0.3 mm of paper): it doesn't collide, and the loops rest on the table through it, its top just under them
SHEET = Shape("sterile_sheet", pos=(LOOP_X + (LOOP_L / 2 - GRASP_AT) * H[0], float(np.mean(LOOP_YS)),
                                    TOP + 0.1 * MM - SHEET_T / 2),
              size=(0.24, 0.15, SHEET_T), color=(0.97, 0.97, 0.95), collide=False)

# -- the discard tray between the arms: an autoclavable polypropylene pan, 25 x 18 cm, 4 cm deep
TRAY_XY, TRAY_SIZE, TRAY_H, TRAY_T = (0.16, 0.05), (0.25, 0.18), 0.04, 0.003
_tc = (0.75, 0.75, 0.72)
TRAY = [Shape("tray_floor", pos=(TRAY_XY[0], TRAY_XY[1], TOP + TRAY_T / 2), size=(*TRAY_SIZE, TRAY_T), color=_tc)]
for _n, (_dx, _dy, _sx, _sy) in {"n": (-1, 0, TRAY_T, TRAY_SIZE[1]), "f": (1, 0, TRAY_T, TRAY_SIZE[1]),
                                 "r": (0, -1, TRAY_SIZE[0], TRAY_T), "l": (0, 1, TRAY_SIZE[0], TRAY_T)}.items():
    TRAY.append(Shape(f"tray_wall_{_n}", pos=(TRAY_XY[0] + _dx * (TRAY_SIZE[0] - TRAY_T) / 2,
                                              TRAY_XY[1] + _dy * (TRAY_SIZE[1] - TRAY_T) / 2, TOP + TRAY_H / 2),
                      size=(_sx, _sy, TRAY_H), color=_tc))

OBJECTS = [PLATE, AGAR, SHEET, *LOOPS, *TRAY]

# -- the streak rule and scoring
F_TOUCH = 0.01  # N: the ring resting on the agar
F_GOUGE = 0.25  # N: 550 g/cm2 (the lowest gel strength suppliers give for 1.5 % agar) over the ring's 0.045 cm2 patch
NEAR = 0.6 * MM  # a ring point this close to the agar's surface is on it
MARK_R = 0.6 * MM  # agar within this of a ring point on the agar is streaked (the self-test's stand-in ring: 1.2 mm)
L_MIN = {1: 40 * MM, 2: 30 * MM, 3: 30 * MM, 4: 30 * MM}  # each loop's streak in its own quadrant
CROSS_MIN = 2  # each later loop crosses the previous loop's streak at least this many times
CROSS_GAP = 2 * MM  # two crossings are separate when the ring travels this far off the previous streak between them
LINE_TOL = 2 * MM  # contact within this of the marker cross is on the line: never streaking out of place
IN_TRAY_ABOVE = 20 * MM  # a loop in the discard tray: no part higher than this above its walls
GRID = 0.5 * MM

_gx = np.arange(-AGAR_R, AGAR_R + GRID / 2, GRID)
_GX, _GY = np.meshgrid(_gx, _gx, indexing="ij")
_IN = _GX ** 2 + _GY ** 2 <= (AGAR_R - 0.5 * MM) ** 2
AGAR_PTS = np.stack([_GX[_IN], _GY[_IN]], -1)  # the agar's surface, in the plate's frame (m)
_A = np.linspace(0, 2 * math.pi, 24, endpoint=False)
RING_PTS = np.stack([RING_C - NECK_X + RING_R * np.cos(_A), RING_R * np.sin(_A), np.zeros_like(_A)], -1)  # tip frame
HANDLE_PTS = np.stack([np.linspace(0, NECK_X, 18), np.zeros(18), np.zeros(18)], -1)  # handle frame
_TREE: list = []


def agar_tree():
    if not _TREE:
        from scipy.spatial import cKDTree

        _TREE.append(cKDTree(AGAR_PTS))
    return _TREE[0]


def quadrant(xy) -> int:
    """1 far right (x > 0, y < 0), 2 far left, 3 near left, 4 near right, in the plate's frame."""
    x, y = float(xy[0]), float(xy[1])
    if x >= 0:
        return 1 if y < 0 else 2
    return 3 if y >= 0 else 4


def quadrants_near(xy, tol: float = LINE_TOL) -> set:
    """Every quadrant within ``tol`` of plate point ``xy`` (on the marker line: both sides)."""
    x, y = float(xy[0]), float(xy[1])
    return {quadrant((x + dx, y + dy)) for dx in (-tol, 0.0, tol) for dy in (-tol, 0.0, tol)}


def new_state() -> dict:
    return dict(order={}, phase=0, length={q: 0.0 for q in range(1, 5)}, drawn={q: 0.0 for q in range(1, 5)},
                cross={q: 0 for q in range(1, 5)},
                prev=None, prev_loop=None, on_prev=False, off_run=1.0, drop=False, redip=False, stop_at=5,
                gouge=False, gouge_at=None, fingers=False, streak=np.zeros(len(AGAR_PTS), np.int8), f=0.0,
                fmax=0.0, t=0.0, last=-1.0, clear=True, used_in_tray=0)


_STATE = new_state()


def advance(st: dict, contact, force: float, marks=None, loop: str = "loop1") -> None:
    """One control step. ``contact``: the ring's contact point on the agar in the plate's frame (m), or None when no
    ring is on it; ``force``: its smoothed force (N); ``marks``: agar points (indices) under the ring (None: a 1.2 mm
    disc round ``contact``); ``loop``: which loop's ring it is.

    Loops are numbered in the order they first touch the agar: loop k streaks quadrant k. Loop k may touch only its
    own quadrant and (k >= 2) the one before, where it crosses the previous loop's streak; loop 4 never quadrant 1.
    Streaking a later quadrant out of turn ends the credit before that quadrant, an earlier one before the loop's
    own; an earlier loop touching the agar again once a later one has ends it after the quadrants already begun."""
    if force > F_GOUGE:
        if not st["gouge"]:
            st["gouge_at"] = None if contact is None else tuple(np.round(np.asarray(contact) * 1e3, 1))
        st["gouge"] = True
    if contact is None:
        st["prev"], st["prev_loop"], st["on_prev"], st["off_run"] = None, None, False, 1.0
        return
    c = np.asarray(contact, float)
    if loop not in st["order"]:
        st["order"][loop] = len(st["order"]) + 1
    k = st["order"][loop]
    if k < st["phase"]:
        st["stop_at"] = min(st["stop_at"], st["phase"] + 1)  # an old loop back on the agar: nothing after counts
    st["phase"] = max(st["phase"], k)
    near = quadrants_near(c)
    allowed = {k, k - 1} if k >= 2 else {1}
    q_here = quadrant(c)
    if q_here not in allowed and not (near & allowed):
        # streaking out of place: a later quadrant (not yet diluted) counts no more from there; an earlier one spoils
        # this loop's own quadrant
        st["stop_at"] = min(st["stop_at"], q_here if q_here > k else k)
    if marks is None:
        marks = np.asarray(agar_tree().query_ball_point(c, r=2 * MARK_R), int)
    step = 0.0 if (st["prev"] is None or st["prev_loop"] != loop) else float(np.linalg.norm(c - st["prev"]))
    if quadrant(c) == k and k <= 4:
        st["drawn"][k] += step
        # a fresh loop carries no bacteria until it has passed through the drop (loop 1) or crossed the previous
        # loop's streak (later loops): what it draws before then grows nothing on the plate, so only streak drawn
        # after that counts (each quadrant has to start by crossing the one before)
        if (k == 1 and st["drop"]) or (k >= 2 and st["cross"][k] >= 1):
            st["length"][k] += step
    if k >= 2 and len(marks):
        on_prev = bool(np.any(st["streak"][marks] == k - 1))
        if on_prev and not st["on_prev"] and st["off_run"] >= CROSS_GAP:
            st["cross"][k] += 1
        st["off_run"] = 0.0 if on_prev else st["off_run"] + step
        st["on_prev"] = on_prev
    if np.linalg.norm(c - DROP_XY) <= DROP_R + 1 * MM:
        if k == 1:
            st["drop"] = True
        else:
            st["redip"] = True
    if len(marks):
        st["streak"][marks] = np.where(st["streak"][marks] == 0, min(k, 4), st["streak"][marks])
    st["prev"], st["prev_loop"] = c, loop


def quadrant_ok(st: dict, q: int) -> bool:
    if q >= st["stop_at"] or st["length"][q] < L_MIN[q]:
        return False
    return st["drop"] if q == 1 else st["cross"][q] >= CROSS_MIN


def score(objects=None, st=None) -> float:
    """0.25 for each quadrant streaked in turn with its own fresh loop (1, then 2, 3, 4): loop 1 drawing at least
    40 mm in quadrant 1 after passing through the drop; each later loop crossing the previous loop's streak at least
    twice and drawing at least 30 mm in its quadrant after its first crossing. 0 if the agar was gouged or a gripper touched it; at most 0.25 if a later loop
    touched the drop; at most 0.75 until every loop that touched the agar lies in the discard tray, let go."""
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


_TRACE: list = []


def _quat_R(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def link_poses(environment, name: str):
    """((pos, R) of the handle link, (pos, R) of the tip link) of loop ``name``, station frame."""
    base = environment.unwrapped
    asset = base.scene[name]
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
    import contactkit
    import touch

    dt = environment.unwrapped.step_dt
    forces = contactkit.loops_on_agar(environment, LOOP_NAMES)
    poses = {n: link_poses(environment, n) for n in LOOP_NAMES}
    if max(forces) > 0:
        i = int(np.argmax(forces))  # the loop on the agar (one is held at a time)
    else:  # none on it: the one whose ring is nearest the plate (the one in the hand, while over it)
        i = int(np.argmin([np.linalg.norm(ring_points(poses[n][1]).mean(0)[:2] - np.asarray(PLATE_XY)) for n in LOOP_NAMES]))
    name = LOOP_NAMES[i]
    handle, tip = poses[name]
    f = 0.5 * forces[i] + 0.5 * (_STATE["f"] if _STATE.get("f_loop") == name else 0.0)
    _STATE["f"], _STATE["f_loop"] = f, name
    _STATE["fmax"] = max(_STATE["fmax"], f)
    on = ring_contact(tip)
    _pts = ring_points(tip)
    _hh = _pts[:, 2] - WIRE_R - (TOP + AGAR_TOP_H)
    _STATE["h"] = float(_hh.min())
    if os.environ.get("STREAK_TRACE"):  # the ring's lowest point every step, for calibrating the demo's heights
        lo = _pts[int(np.argmin(_hh))]
        _TRACE.append((_STATE["t"], *plate_frame(lo), _STATE["h"], f, i + 1))
        if len(_TRACE) % 20 == 0:
            np.save(os.environ["STREAK_TRACE"], np.array(_TRACE))
    contact, marks = None, None
    if len(on) and f >= F_TOUCH:
        contact = plate_frame(on).mean(0)
        idx = agar_tree().query_ball_point(plate_frame(on), r=MARK_R)
        idx = [np.asarray(j, int) for j in idx]
        marks = np.unique(np.concatenate(idx)) if len(idx) else np.zeros(0, int)
    advance(_STATE, contact, f, marks, loop=name)
    t = touch.touching(environment, ["agar", *LOOP_NAMES])
    if t["agar"]:
        _STATE["fingers"] = True
    used = list(_STATE["order"])  # every loop that touched the agar: lying in the tray, let go
    in_tray = [n for n in used if loop_in_tray(*poses[n]) and not t[n]]
    _STATE["used_in_tray"] = len(in_tray)
    _STATE["clear"] = bool(used) and len(in_tray) == len(used)
    _STATE["t"] += dt
    if os.environ.get("STREAK_LOG") and (_STATE["t"] - _STATE["last"] >= 0.5 or (contact is not None and f > 0.15)):
        _STATE["last"] = _STATE["t"]
        L, X = _STATE["length"], _STATE["cross"]
        c = "-" if contact is None else f"({contact[0] * 1e3:.0f},{contact[1] * 1e3:.0f})"
        k = _STATE["order"].get(name, 0)
        ax = handle[1][:, 0]  # the handle's axis (butt to ring) in the station frame
        tilt_deg = math.degrees(math.asin(float(np.clip(-ax[2], -1, 1))))
        yaw_deg = math.degrees(math.atan2(ax[1], ax[0]))
        print(f"LOG t={_STATE['t']:.2f} loop={name}(#{k}) tilt={tilt_deg:.1f} yaw={yaw_deg:.1f} f={f:.3f} fmax={_STATE['fmax']:.3f} at={c} "
              f"len={L[1] * 1e3:.0f}/{L[2] * 1e3:.0f}/{L[3] * 1e3:.0f}/{L[4] * 1e3:.0f}mm "
              f"drawn={_STATE['drawn'][1] * 1e3:.0f}/{_STATE['drawn'][2] * 1e3:.0f}/{_STATE['drawn'][3] * 1e3:.0f}/"
              f"{_STATE['drawn'][4] * 1e3:.0f}mm cross={X[2]}/{X[3]}/{X[4]} "
              f"drop={_STATE['drop']} redip={_STATE['redip']} stop_at={_STATE['stop_at']} gouge={_STATE['gouge']}"
              f"{_STATE['gouge_at'] or ''} fingers={_STATE['fingers']} in_tray={_STATE['used_in_tray']}/{len(used)} "
              f"h={_STATE['h'] * 1e3:.2f}mm score={score():.2f}", flush=True)


# -- the scripted demo: the right arm picks the loop up, holds it like a pen and streaks the four quadrants ---------
HOVER = TOP + 0.15
TILT = math.radians(50.0)  # the loop's angle to the agar while streaking
U_T = math.cos(TILT) * H - math.sin(TILT) * np.array([0.0, 0.0, 1.0])  # along the tilted loop, towards the ring
REACH = RING_C + RING_R - GRASP_AT  # from the hand to the ring's far edge
DEPTH = float(os.environ.get("STREAK_DEPTH", "0.002"))
LIFT_ALL = float(os.environ.get("STREAK_LIFT", "0"))  # calibration runs: the whole plan this much higher  # how far past the surface the plan aims the ring (the neck flexes)
SPEED = 0.020  # m/s along the agar

# The loop leans 50 degrees back from its ring towards the hand (+x), so on the far half of the plate (quadrants 1 and 2)
# its neck passes over the dish's far wall low: the ring works only where the shaft clears the wall's rim (see
# wall_clearance). Quadrants 1 and 2 are drawn in plate millimetres inside that reach; quadrants 3 and 4 (the near
# half, the shaft leaning away over the plate) in each quadrant's own coordinates (mm): u along the line it shares with
# the quadrant before it (away from the centre), v into the quadrant; v < 0 is the quadrant before.
# Quadrant 1: through the drop at (16, -14), zigzags, then a stroke along y = -5 just inside the line to quadrant 2.
Q1_MM = [(17, -4), (16, -14), (14, -20), (4, -6), (10, -26), (3, -14), (5, -30), (3, -5), (16, -5)]
# Quadrant 2: seven strokes back across the line to y = -8, each crossing quadrant 1's y = -5 stroke; zigzags; then
# a stroke down its far side at x = 4, which loop 3's strokes back across the line will cross.
Q2_MM = [(16, 6), (14, -8), (12, 6), (10, -8), (8, 6), (6, -8), (4, 6), (16, 12), (4, 16), (13, 22), (4, 26), (4, 28),
         (4, 6)]
PATH_SCALE = 0.8
QN_PATH = [(34, 8), (30, -10), (26, 10), (22, -10), (18, 10), (14, -10), (10, 10), (30, 18), (8, 24), (22, 30)]
# quadrant 3 finishes with a stroke down its far side, 4 mm inside the line to quadrant 4: loop 4 crosses it
FAR_SIDE = [(5, 32), (5, 8)]
_TO_PLATE = {3: lambda u, v: (-v, u), 4: lambda u, v: (-u, -v)}
WALL_XERR = 8 * MM  # the ring has ridden up to about this much further +x than planned (the grip on the round handle)
WALL_MARGIN = 2 * MM


def wall_clearance(p, tilt: float = math.radians(47.0)) -> float:
    """How far the tilted shaft passes above the dish's rim (m) with the ring's far edge at plate point ``p``, allowing
    for the ring riding WALL_XERR further +x and leaning a little flatter (47 degrees) than planned."""
    back = p[0] + 2 * RING_R * math.cos(tilt) + WALL_XERR  # the ring's +x edge
    xw = math.sqrt(max((DISH_ID / 2) ** 2 - p[1] ** 2, 0.0))  # the dish's inner wall along +x
    h = 2 * RING_R * math.sin(tilt) + (xw - back) * math.tan(tilt) - HANDLE_R
    return h - (WALL_H - AGAR_TOP_H)


def quadrant_path(q: int) -> list:
    """Quadrant q's streak as plate-frame points (m)."""
    if q == 1:
        return [np.array(p, float) * MM for p in Q1_MM]
    if q == 2:
        return [np.array(p, float) * MM for p in Q2_MM]
    pts = QN_PATH + (FAR_SIDE if q == 3 else [])
    return [np.array(_TO_PLATE[q](u, v)) * MM * PATH_SCALE for u, v in pts]


_CAL = pathlib.Path(__file__).resolve().parent / "assets" / "height_cal.npz"


def height_error(xy_plate, loop: int = 1) -> float:
    """How far above its planned height loop ``loop``'s ring rides at plate point ``xy_plate`` (m), measured once by
    running the plan 3 mm up, off the agar (assets/height_cal.npz: loop, plate xy and error; each loop sits a little
    differently in the hand); 0 without that file."""
    if os.environ.get("STREAK_NOCAL") or not _CAL.is_file():
        return 0.0
    d = np.load(_CAL)
    sel = d["loop"] == loop if "loop" in d.files else np.ones(len(d["err"]), bool)
    if not sel.any():
        return 0.0
    xy, err = d["xy"][sel], d["err"][sel]
    dist = np.linalg.norm(xy - np.asarray(xy_plate)[None], axis=1)
    near = np.argsort(dist)[:6]
    w = 1.0 / (dist[near] + 1e-3) ** 2
    local = float((err[near] * w).sum() / w.sum())
    a = float(np.clip((dist[near[0]] - 0.004) / 0.006, 0.0, 1.0))  # beyond the measured cells: the loop's mean error
    return (1 - a) * local + a * float(err.mean())


def ring_offset(loop: int) -> np.ndarray:
    """How far loop ``loop``'s ring lands from where the plan aims it, sideways (m; from assets/height_cal.npz)."""
    if os.environ.get("STREAK_NOCAL") or not _CAL.is_file():
        return np.zeros(2)
    d = np.load(_CAL)
    return np.asarray(d["off"][loop - 1], float) if "off" in d.files else np.zeros(2)


def hand_at(xy_plate, lift: float = 0.0, loop: int = 1) -> tuple:
    """The TCP with loop ``loop``'s ring's far edge at plate point ``xy_plate`` (m), DEPTH into the agar, or ``lift``
    above it (aimed off by the loop's measured offsets, so the ring lands on the point)."""
    z = TOP + AGAR_TOP_H - DEPTH + lift + WIRE_R + LIFT_ALL - height_error(xy_plate, loop)
    aim = np.asarray(xy_plate, float) - ring_offset(loop)
    p = np.array([aim[0] + PLATE_XY[0], aim[1] + PLATE_XY[1], z])
    return tuple(float(c) for c in p - REACH * U_T)


TRAY_DROP_YS = (0.000, 0.030, 0.000, 0.030)  # where each used loop is laid in the tray (y): alternating, on top of each other


def demo():
    """Right arm, for each quadrant in turn: over the next fresh loop's handle, down, grip, lift, turn the loop down
    to 50 degrees, down onto the quadrant's first point, draw its zigzag at 20 mm/s (later quadrants starting with
    strokes back across the previous quadrant's streak), lift, carry the loop over the discard tray, turn it level,
    lay it in and let go. The left arm stays at its start."""
    from tiltkit import grip, install
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    install()
    approach_t = -math.sin(TILT) * H - math.cos(TILT) * np.array([0.0, 0.0, 1.0])
    flat = grip(approach=(0, 0, -1), closing=W_)
    flat_joint = grip(approach=(0, 0, -1), closing=W_, joint=True)
    tilted = grip(approach=tuple(approach_t), closing=tuple(W_), joint=True)
    tilted_move = grip(approach=tuple(approach_t), closing=tuple(W_))
    pick_z = TOP + HANDLE_R + 12 * MM  # pads' tips about 1 mm above the bench, round the handle's middle
    HOLD = 0.0  # fingers commanded shut on the 3.6 mm handle (a firm pinch)
    OPEN = -0.012
    # over the tray the loop is laid along x, ring end towards the robot, from 2 cm inside the near wall
    tx = TRAY_XY[0] - TRAY_SIZE[0] / 2 + TRAY_T + 0.02 + (RING_C + RING_R - GRASP_AT)
    lay_z = TOP + TRAY_T + HANDLE_R + 12 * MM + 15 * MM  # the handle 15 mm above the tray's floor: it drops onto the others
    right = [W(2.0, (0.22, -0.30, HOVER), O, "start", flat)]
    for q in (1, 2, 3, 4):
        gx, gy = LOOP_X, LOOP_YS[q - 1]
        path = quadrant_path(q)
        right += [
            W(2.0 if q == 1 else 3.0, (gx, gy, TOP + 0.06), OPEN, f"over loop {q}'s handle", flat_joint if q > 1 else flat),
            W(1.5, (gx, gy, pick_z), OPEN, f"down round loop {q}'s handle", flat),
            W(1.0, (gx, gy, pick_z), HOLD, f"grip loop {q}", flat),
            W(2.0, (gx, gy, TOP + 0.14), HOLD, f"lift loop {q}", flat),
            W(4.0, hand_at(path[0], 0.06, q), HOLD, f"turn loop {q} down, high over quadrant {q}", tilted),
            W(1.5, hand_at(path[0], 0.015, q), HOLD, f"down over quadrant {q}", tilted_move),
            W(0.8, hand_at(path[0], 0.002, q), HOLD, f"down to quadrant {q}", tilted_move),
            W(1.5, hand_at(path[0], 0.0, q), HOLD, f"onto quadrant {q}", tilted_move),
        ]
        for k in range(1, len(path)):
            d = float(np.linalg.norm(path[k] - path[k - 1]))
            right.append(W(max(0.4, d / SPEED), hand_at(path[k], 0.0, q), HOLD, f"streak {q}", tilted_move))
        right += [
            W(0.8, hand_at(path[-1], 0.015, q), HOLD, f"lift off quadrant {q}", tilted_move),
            W(1.5, hand_at(path[-1], 0.06, q), HOLD, "lift clear", tilted_move),
            W(3.0, (tx, TRAY_DROP_YS[q - 1], TOP + 0.12), HOLD, "level, over the discard tray", flat_joint),
            W(1.5, (tx, TRAY_DROP_YS[q - 1], lay_z), HOLD, "down into the tray", flat),
            W(0.8, (tx, TRAY_DROP_YS[q - 1], lay_z), OPEN, f"let go of loop {q}", flat),
            W(1.0, (tx, TRAY_DROP_YS[q - 1], TOP + 0.10), OPEN, "up", flat),
        ]
    right.append(W(2.0, (0.22, -0.30, HOVER), OPEN, "back", flat_joint))
    return {"right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-made contact sequences (each streak drawn densely; a 1.2 mm disc of agar marked)."""

    def dense(pts):
        out = []
        for a_, b_ in zip(pts[:-1], pts[1:]):
            n = max(2, int(np.linalg.norm(b_ - a_) / (0.5 * MM)))
            out += [a_ + (b_ - a_) * t for t in np.linspace(0, 1, n, endpoint=False)]
        return out + [pts[-1]]

    def run(seq, force=0.05, extra=None, clear=True):
        """``seq``: (loop name, plate points) streaks in order, the ring lifted between them."""
        st = new_state()
        for loop, pts in seq:
            for c in dense(pts):
                advance(st, c, force, loop=loop)
            advance(st, None, 0.0)
        if extra:
            extra(st)
        st["clear"] = clear
        return score(st=st), st

    full = [(f"loop{q}", quadrant_path(q)) for q in (1, 2, 3, 4)]
    got, st = run(full)
    assert got == 1.0, (got, st["length"], st["cross"], st["drop"], st["stop_at"])
    for q in (2, 3, 4):
        assert st["cross"][q] >= CROSS_MIN + 1, (q, st["cross"])  # the demo crosses with a margin
    cases = {
        "quadrant 1 only": (full[:1], {}, 0.25),
        "quadrants 1-2": (full[:2], {}, 0.5),
        "quadrants 1-3": (full[:3], {}, 0.75),
        "all four, loops not yet in the tray": (full, {"clear": False}, 0.75),
        "pressed too hard": (full, {"force": 0.4}, 0.0),
        # one loop for every quadrant: it carries undiluted sample into quadrant 2
        "one loop throughout": ([("loop1", p) for _, p in full], {}, 0.25),
        # a loop for quadrant 2, then the first loop again for quadrant 3, back through quadrant 2's streak: it
        # carries undiluted sample into quadrant 2, which no longer counts
        "re-using loop 1 through quadrant 2": (full[:2] + [("loop1", quadrant_path(3)), ("loop4", quadrant_path(4))],
                                               {}, 0.25),
        # ... staying inside quadrant 3: quadrants 1 and 2 still count
        "re-using loop 1 in quadrant 3 only": (full[:2] + [("loop1", [p for p in quadrant_path(3) if p[0] < -0.003])],
                                               {}, 0.5),
        # the gripper on the agar
        "a gripper on the agar": (full, {"extra": lambda st: st.update(fingers=True)}, 0.0),
    }
    for name, (seq, kw, want) in cases.items():
        got, st = run(seq, **kw)
        assert abs(got - want) < 1e-9, (name, got, want, st["length"], st["cross"], st["stop_at"])
    # quadrant 1 missing the drop: nothing counts
    miss = [np.array(p) * MM for p in [(9, -4), (7, -30), (5, -4), (4, -30), (8, -4), (6, -30)]]
    assert run([("loop1", miss)] + full[1:])[0] == 0.0
    # quadrant 2 drawn without strokes back into quadrant 1: only quadrant 1 counts
    q2_own = [p for p in quadrant_path(2) if p[1] > 0.003]
    assert run(full[:1] + [("loop2", q2_own)] + full[2:])[0] == 0.25
    # quadrant 2's strokes cross the marker line into quadrant 1 where quadrant 1 has no streak: they don't count
    q1_far = [np.array(p) * MM for p in [(16, -9), (16, -14), (14, -22), (10, -9), (8, -26), (5, -9), (4, -28)]]
    q2_line = [np.array(p) * MM for p in [(14, 12), (12, -4), (10, 12), (8, -4), (6, 12), (14, 20), (6, 28), (12, 26)]]
    got, st = run([("loop1", q1_far), ("loop2", q2_line)])
    assert got == 0.25 and st["cross"][2] == 0 and st["drawn"][2] >= L_MIN[2], (got, st["cross"], st["drawn"])
    # loop 2 streaks its own quadrant first and only crosses quadrant 1's streak at the end: what it drew before the
    # crossing carried no bacteria, so quadrant 2 doesn't count
    own_first = [p for p in quadrant_path(2) if p[1] > 0.003] + [np.array([12, 6]) * MM, np.array([12, -8]) * MM,
                                                               np.array([8, 6]) * MM, np.array([8, -8]) * MM]
    got, st = run(full[:1] + [("loop2", own_first)])
    assert got == 0.25 and st["cross"][2] >= CROSS_MIN and st["drawn"][2] >= L_MIN[2] > st["length"][2], (got, st["cross"])
    # quadrant 1 streaked before the loop reaches the drop: only what follows the drop counts
    got, st = run([("loop1", [np.array(p) * MM for p in [(9, -4), (7, -30), (5, -4), (4, -30), (16, -14)]])])
    assert got == 0.0 and st["drop"] and st["drawn"][1] >= L_MIN[1] > st["length"][1], (got, st["length"])
    # a quadrant skipped: loop 2 streaks quadrant 3 straight after quadrant 1
    assert run([full[0], ("loop2", quadrant_path(3)), ("loop3", quadrant_path(4))])[0] == 0.25
    # a later loop back to the drop (re-inoculating): at most 0.25
    redip = full[:2] + [("loop2", [np.array([0.010, 0.010]), DROP_XY + 0.0, np.array([0.010, 0.010])])] + full[2:]
    assert run(redip)[0] == 0.25
    # quadrant 4 running into quadrant 1: quadrant 4 doesn't count
    q4_bad = quadrant_path(4) + [np.array([-0.010, -0.020]), np.array([0.010, -0.020])]
    assert run(full[:3] + [("loop4", q4_bad)])[0] == 0.75
    # hovering (no contact) draws nothing
    st = new_state()
    for c in quadrant_path(1):
        advance(st, None, 0.0)
    assert score(st=st) == 0.0
    # a loop lying in the tray counts; held over the plate or above the tray does not
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
    # the tilted loop's shaft clears the dish's rim wherever the ring works, with room for the grip's offsets
    for q in (1, 2, 3, 4):
        for p in quadrant_path(q):
            assert wall_clearance(p) > WALL_MARGIN, (q, np.round(p * 1e3, 1), wall_clearance(p))
    assert wall_clearance(np.array([26.4, -11.2]) * MM) < 0  # (the first version's drop: out of reach)
    assert abs(AGAR_DEPTH - 3.29 * MM) < 0.05 * MM
    # the loops lie clear of the plate, the tray and each other (35 mm apart: an open hand's pads clear the next one)
    for y in LOOP_YS:
        ring0 = np.array([LOOP_X, y]) + (RING_C - GRASP_AT) * H[:2]
        assert np.linalg.norm(ring0 - PLATE_XY) > DISH_OD / 2 + 0.01
        assert abs(y - PLATE_XY[1]) > DISH_OD / 2 + 0.05  # the handle passes beside the plate
    assert min(np.diff(sorted(LOOP_YS))) >= 0.035 - 1e-9
    sx, sy = SHEET.pos[0], SHEET.pos[1]
    for y in LOOP_YS:  # every loop, ring to butt, lies on the sheet with 1 cm to spare
        for d in (RING_C + RING_R, 0.0):
            px = LOOP_X + (d - GRASP_AT) * H[0]
            assert abs(px - sx) < SHEET.size[0] / 2 - 0.01 and abs(y - sy) < SHEET.size[1] / 2 - 0.01, (y, d, px)
    assert abs(sy - PLATE_XY[1]) - SHEET.size[1] / 2 > DISH_OD / 2 + 0.01  # clear of the plate
    near = np.clip(np.asarray(PLATE_XY), np.array(TRAY_XY) - np.array(TRAY_SIZE) / 2, np.array(TRAY_XY) + np.array(TRAY_SIZE) / 2)
    assert np.linalg.norm(near - PLATE_XY) > DISH_OD / 2 + 0.01
    assert TRAY_SIZE[0] - 2 * TRAY_T > LOOP_L + 0.02  # a loop lies in it lengthwise
    for y in TRAY_DROP_YS:
        assert abs(y - TRAY_XY[1]) < TRAY_SIZE[1] / 2 - TRAY_T - 0.02
    return len(cases) + 17


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
