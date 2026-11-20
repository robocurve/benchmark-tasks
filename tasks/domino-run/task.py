"""Set up a run of six dominoes and topple it. Between the two arms, a straight lane is marked on the table with blue
tape (60 mm wide inside the tape, running away from the arms), with six thin red tick marks across it 30 mm apart. Six
standard double-six dominoes (48 x 24 x 7.5 mm, 10 g) lie flat and face up on the table, three on the left of the lane
and three on the right, long side pointing away from the arms.

Task: stand all six dominoes on their short ends, one on each tick, faces toward the arms, without knocking any over
for good; then tip the domino on the tick nearest the arms away from them so that it knocks the next one, and so on,
until all six have fallen in order. A standing domino tips over past 9 degrees, so each has to be set down square and
let go of cleanly, and the hand must not brush its neighbours. Only the first domino may be touched once all six stand.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from domkit import Body, BoxShape, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Stand all six dominoes on their short ends, one on each red tick in the taped lane, faces toward you. "
               "Then tip the domino nearest you away from you, touching no other domino, so that all six fall in a "
               "chain.")

# -- the dominoes ---------------------------------------------------------------------------------------------------
# A standard double-six domino, 48 x 24 x 7.5 mm, 10 g (melamine/urea or hardwood). Domino frame: origin at its centre,
# x through its thickness (from the pip face to the back), y across its 24 mm width, z along its 48 mm length.
DOM_T, DOM_W, DOM_L = 0.0075, 0.024, 0.048
DOM_MASS = 0.010
DOM_FRICTION = (0.40, 0.30)     # estimate: melamine or lacquered wood on a laminate table top, and on each other
N = 6
IVORY = (0.95, 0.93, 0.85)
PIP = (0.08, 0.08, 0.10)
VALUES = ((6, 6), (5, 3), (4, 2), (6, 1), (3, 3), (2, 0))

# -- the lane on the table (5 mm blue tape; 3 mm red tick marks across it) -------------------------------------------
RUN_Y = 0.0                     # the lane's centre line runs along x (away from the arms) at y = 0
U = np.array([1.0, 0.0, 0.0])  # the run's direction: from the first tick (nearest the arms) to the last
SPACING = 0.030
TICK_X = tuple(round(0.22 + SPACING * k, 4) for k in range(N))   # the ticks' centres (world x, m)
LANE_HALF = 0.030               # inner edges of the side tapes, either side of the centre line
LANE_X0, LANE_X1 = 0.19, 0.47   # inner edges of the end tapes
TAPE_W, TICK_W = 0.005, 0.003
BLUE = (0.15, 0.35, 0.85)
RED = (0.85, 0.12, 0.12)

# Where the dominoes start: flat, face up (pip face up), long side along x, three either side of the lane.
FLAT_X = 0.215
FLAT_Y = (0.125, 0.175, 0.225, -0.125, -0.175, -0.225)
FLAT_XYZW = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))   # 90 deg about y: the domino's z (length) along world x,
                                                          # its x (thickness) pointing down, so the pip face is up
START = [(FLAT_X, y, TOP + DOM_T / 2 + 0.0003) for y in FLAT_Y]


def _pips(value: int, zc: float) -> list:
    """The pips of one half of the pip face (the face at x = -DOM_T/2), centred at height zc in the domino frame."""
    s = 0.0065
    spots = {0: [], 1: [(0, 0)], 2: [(-1, 1), (1, -1)], 3: [(-1, 1), (0, 0), (1, -1)],
             4: [(-1, 1), (1, 1), (-1, -1), (1, -1)], 5: [(-1, 1), (1, 1), (0, 0), (-1, -1), (1, -1)],
             6: [(-1, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (1, -1)]}[value]
    return [BoxShape((-DOM_T / 2 - 0.0001, a * s, zc + b * s), (0.0002, 0.0032, 0.0032), PIP, collide=False)
            for a, b in spots]


def _domino(i: int) -> Body:
    hi, lo = VALUES[i]
    shapes = [BoxShape((0.0, 0.0, 0.0), (DOM_T, DOM_W, DOM_L), IVORY),
              BoxShape((-DOM_T / 2 - 0.0001, 0.0, 0.0), (0.0002, DOM_W - 0.006, 0.0012), PIP, collide=False),
              *_pips(hi, DOM_L / 4), *_pips(lo, -DOM_L / 4)]
    return Body.build(f"domino_{i}", pos=START[i], mass=DOM_MASS, shapes=shapes, color=IVORY, friction=DOM_FRICTION,
                      contact_offset=0.002, solver_iterations=32, damping=(0.0, 0.05), start_xyzw=FLAT_XYZW)


def _tape() -> Body:
    """The tape: visual only (0.2 mm proud of the table), plus a small collider hidden inside the table top, which a
    fixed body needs and which nothing can reach."""
    z = 0.0001
    mid, length = (LANE_X0 + LANE_X1) / 2, LANE_X1 - LANE_X0 + 2 * TAPE_W
    shapes = [BoxShape((0.0, 0.0, -0.02), (0.01, 0.01, 0.01), None)]
    for s in (-1, 1):
        shapes.append(BoxShape((mid, s * (LANE_HALF + TAPE_W / 2), z), (length, TAPE_W, 0.0002), BLUE, collide=False))
    for x in (LANE_X0 - TAPE_W / 2, LANE_X1 + TAPE_W / 2):
        shapes.append(BoxShape((x, 0.0, z), (TAPE_W, 2 * LANE_HALF, 0.0002), BLUE, collide=False))
    for x in TICK_X:
        shapes.append(BoxShape((x, 0.0, z), (TICK_W, 2 * LANE_HALF, 0.0002), RED, collide=False))
    return Body.build("lane_tape", pos=(0.0, RUN_Y, TOP), mass=0.01, kinematic=True, color=BLUE, shapes=shapes)


OBJECTS = [_tape(), *(_domino(i) for i in range(N))]

# -- scoring numbers ---------------------------------------------------------------------------------------------------
UPRIGHT_DEG = 10.0     # standing on its short end: long side within 10 deg of vertical (it tips over past 8.9 deg)
ON_TICK = 0.006        # its base centre within 6 mm of a tick's centre along the lane ...
OFF_LINE = 0.008       # ... and within 8 mm of the lane's centre line
SQUARE_DEG = 15.0      # its faces within 15 deg of square to the lane
ON_TABLE = 0.003       # its base within 3 mm of the table (not standing on something)
START_DEG = 15.0       # a domino has started to fall once it leans 15 deg
DOWN_TOP = 0.030       # fallen: its highest point at most 30 mm above the table (48 mm standing; 46 mm leaning on an
                       # upright neighbour; about 19 mm resting on a fallen one)
FLAT_TOP = 0.010       # lying flat: its highest point at most 10 mm above the table (7.5 mm flat)
FORWARD = 0.5          # fallen forward: its (former) top end at least 0.5 along the run's direction
TOUCH = 0.001          # the robot touches a domino: some part within 1 mm of it
CLEAR = 0.020          # hands off: every part of both arms at least 2 cm from every domino
STILL = 0.001          # still: no corner moved more than 1 mm over the window
SET_STEPS = 10         # standing set for 0.5 s (at 20 Hz)
HOLD_STEPS = 20        # the finished run held for 1 s
FAR = 0.05             # distance reported when no part of the robot is within 3 cm


# -- geometry (pure; used both in simulation and by the offline tests) ------------------------------------------------

def corners(pos, r) -> np.ndarray:
    """The 8 corners of a domino (world, m)."""
    pos, r = np.asarray(pos, float), np.asarray(r, float)
    h = np.array([DOM_T, DOM_W, DOM_L]) / 2
    return np.array([pos + r @ (h * np.array([a, b, c])) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])


def tilt_deg(r) -> float:
    """How far a domino's long side leans from vertical (degrees; 0 standing on either end, 90 lying down)."""
    return math.degrees(math.acos(min(1.0, abs(float(np.asarray(r, float)[2, 2])))))


def top_height(pos, r) -> float:
    """Its highest point above the table (m)."""
    return float(corners(pos, r)[:, 2].max() - TOP)


def tick_of(pos, r) -> int | None:
    """The tick (0 = nearest the arms) a domino stands on, upright on its short end and square to the lane, or None."""
    pos, r = np.asarray(pos, float), np.asarray(r, float)
    if tilt_deg(r) > UPRIGHT_DEG:
        return None
    a = r[:, 2] * (1.0 if r[2, 2] > 0 else -1.0)     # its long side, pointing up
    base = pos - a * DOM_L / 2
    if abs(base[2] - TOP) > ON_TABLE or abs(base[1] - RUN_Y) > OFF_LINE:
        return None
    t = r[:2, 0]
    if np.linalg.norm(t) < 1e-6 or abs(t @ U[:2]) / np.linalg.norm(t) < math.cos(math.radians(SQUARE_DEG)):
        return None
    k = int(np.argmin([abs(base[0] - x) for x in TICK_X]))
    return k if abs(base[0] - TICK_X[k]) <= ON_TICK else None


def fallen_forward(pos, r, up_sign: float) -> bool:
    """Down (highest point at most 30 mm up) with the end that was on top now pointing along the run."""
    top_end = np.asarray(r, float)[:, 2] * up_sign
    return bool(top_height(pos, r) <= DOWN_TOP and float(top_end @ U) >= FORWARD)


def in_lane(pos, r) -> bool:
    """Every corner inside the tape's inner edges."""
    c = corners(pos, r)
    return bool(np.all(np.abs(c[:, 1] - RUN_Y) <= LANE_HALF) and np.all((c[:, 0] >= LANE_X0) & (c[:, 0] <= LANE_X1)))


def _surface_points(step: float = 0.004) -> np.ndarray:
    """Points over a domino's surface, at most ``step`` apart (domino frame)."""
    h = np.array([DOM_T, DOM_W, DOM_L]) / 2
    axes = [np.linspace(-v, v, max(2, int(math.ceil(2 * v / step)) + 1)) for v in h]
    pts = []
    for i in range(3):
        j, k = [a for a in range(3) if a != i]
        gj, gk = np.meshgrid(axes[j], axes[k], indexing="ij")
        for s in (-1, 1):
            p = np.zeros((gj.size, 3))
            p[:, i], p[:, j], p[:, k] = s * h[i], gj.ravel(), gk.ravel()
            pts.append(p)
    return np.unique(np.vstack(pts).round(9), axis=0)


SURFACE = _surface_points()


def robot_distance(pos, r, hulls) -> float:
    """The least distance (m) from the robot to a domino: over the convex collision hulls ``hulls`` (a list of
    (vertices, planes) in the world, planes as outward n . x = d), the smaller of the hull vertices' exact distance to
    the domino's box and the domino's surface points' distance to the hull's face planes (a lower bound outside the
    hull). Hulls more than 3 cm away are skipped; FAR if all are."""
    pos, r = np.asarray(pos, float), np.asarray(r, float)
    h = np.array([DOM_T, DOM_W, DOM_L]) / 2
    c = corners(pos, r)
    lo, hi = c.min(0), c.max(0)
    pts = None
    best = FAR
    for verts, planes in hulls:
        vlo, vhi = verts.min(0), verts.max(0)
        if np.maximum(vlo - hi, lo - vhi).max() > 0.03:
            continue
        local = (verts - pos) @ r
        q = np.abs(local) - h
        d1 = float((np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)).min())
        if pts is None:
            pts = pos + SURFACE @ r.T
        d2 = float((pts @ planes[:, :3].T - planes[:, 3]).max(axis=1).min())
        best = min(best, d1, d2)
    return best


# -- the score, step by step ---------------------------------------------------------------------------------------

_EP: dict = {}
_DEBUG = bool(os.environ.get("DOMINO_DEBUG"))


@on_reset
def _reset_state() -> None:
    _EP.clear()
    _EP.update(best=0.0, run=None, held=0, recent=[], score=0.0, done=False, steps=0, info={})


_reset_state()


def _still(recent, n: int) -> np.ndarray:
    """Per domino: no corner moved more than STILL over the last n steps."""
    if len(recent) < n + 1:
        return np.zeros(N, bool)
    window = np.stack(recent[-(n + 1):])          # (n+1, N, 8, 3)
    return np.abs(window - window[-1]).max(axis=(0, 2, 3)) <= STILL


def observe(doms, dist) -> None:
    """One control step. ``doms``: each domino's (pos, rotation matrix); ``dist``: the robot's least distance (m) to
    each domino."""
    doms = [(np.asarray(p, float), np.asarray(r, float)) for p, r in doms]
    dist = [float(d) for d in dist]
    snap = np.stack([corners(p, r) for p, r in doms])
    recent = _EP["recent"]
    recent.append(snap)
    del recent[:-(HOLD_STEPS + 1)]
    still_set, still_hold = _still(recent, SET_STEPS), _still(recent, HOLD_STEPS)
    touch = [d <= TOUCH for d in dist]
    ticks = [tick_of(p, r) for p, r in doms]
    standing = {}                                   # tick -> domino standing on it on its own, still
    for i, k in enumerate(ticks):
        if k is not None and not touch[i] and still_set[i]:
            standing.setdefault(k, i)
    if len(standing) >= 3:
        _EP["best"] = max(_EP["best"], 0.25)
    run = _EP["run"]
    if len(standing) == N:                          # all six stand, one per tick: a fresh run starts from here
        _EP["best"] = max(_EP["best"], 0.5)
        run = _EP["run"] = {"order": [standing[k] for k in range(N)],
                            "up": [1.0 if doms[standing[k]][1][2, 2] > 0 else -1.0 for k in range(N)],
                            "started": [False] * N, "fouled": None, "lead": 0}
        is_set = True
    else:
        is_set = False
    full = False
    if run is not None and not is_set and not run["fouled"]:
        order, up = run["order"], run["up"]
        for k, i in enumerate(order):
            if not run["started"][k] and tilt_deg(doms[i][1]) >= START_DEG:
                run["started"][k] = True
                if k > 0 and not run["started"][k - 1]:
                    run["fouled"] = f"the domino on tick {k + 1} started to fall before the one on tick {k}"
        for k, i in enumerate(order):
            if touch[i] and k > 0:
                run["fouled"] = run["fouled"] or f"the robot touched the domino on tick {k + 1}"
            if touch[i] and k == 0 and run["started"][1]:
                run["fouled"] = run["fouled"] or "the robot touched the first domino after the second started to fall"
        fallen = [fallen_forward(*doms[i], up[k]) for k, i in enumerate(order)]
        lead = 0
        while lead < N and fallen[lead]:
            lead += 1
        run["lead"] = lead
        if not run["fouled"]:
            if lead >= 2:
                _EP["best"] = max(_EP["best"], 0.75)
            last = order[-1]
            full = bool(lead == N and top_height(*doms[last]) <= FLAT_TOP
                        and all(in_lane(p, r) for p, r in doms)
                        and min(dist) >= CLEAR and bool(still_hold.all()))
    _EP["held"] = _EP["held"] + 1 if full else 0
    score = 1.0 if _EP["held"] >= HOLD_STEPS else _EP["best"]
    _EP.update(score=score, done=full, steps=_EP["steps"] + 1,
               info={"set": is_set, "standing": sorted(standing), "ticks": ticks, "touch": touch,
                     "fouled": run["fouled"] if run else None, "lead": run["lead"] if run else 0,
                     "min_dist": min(dist), "full": full})


def success(objects) -> bool:
    """True only on steps where score() is 1.0: all six have fallen in order, in a run started by tipping only the first,
    and have lain still in the lane, hands off, for 1 s. The runtime then ends the episode after its own 1 s of this."""
    return _EP["score"] >= 1.0


def score(objects) -> float:
    """1.0 once, after all six stood on the ticks, the run was started by touching only the first domino and all six
    fell forward in order (each with its top at most 30 mm up, the last lying flat), every domino inside the lane, both
    arms 2 cm clear and everything still, for 1 s; 0.75 if, in such a run, at least the first two fell forward; 0.5
    if all six at some point stood on their own, one on each tick, still for 0.5 s; 0.25 if three or more did; else
    0."""
    return float(_EP["score"])


# -- in the simulator: read the state and score it ------------------------------------------------------------------

_HULLS: dict = {}


def _robot_hulls(base, origin) -> list:
    """Every collision hull of both arms (all links but the bases), in the world frame: (vertices, planes)."""
    if not _HULLS:
        from yam_arena.analysis import link_hulls

        for link, hs in link_hulls().items():
            if link != "base":
                _HULLS[link] = [(h.vertices, h.planes) for h in hs]
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        for j, n in enumerate(robot.body_names):
            link = n.rsplit("/", 1)[-1]
            if link not in _HULLS:
                continue
            r, p = rotation(poses[j][3:7]), poses[j][:3] - origin
            for verts, planes in _HULLS[link]:
                n_w = planes[:, :3] @ r.T
                out.append((verts @ r.T + p, np.c_[n_w, planes[:, 3] + n_w @ p]))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    doms = []
    for i in range(N):
        p = as_numpy(base.scene[f"domino_{i}"].data.root_link_pose_w)[0]
        doms.append((p[:3] - origin, rotation(p[3:7])))
    hulls = _robot_hulls(base, origin)
    dist = [robot_distance(p, r, hulls) for p, r in doms]
    observe(doms, dist)
    if _DEBUG and (_EP["steps"] % 10 == 0 or _EP["info"]["set"] or _EP["info"]["full"]):
        inf = _EP["info"]
        print(f"[domino] step {_EP['steps']} score {_EP['score']} held {_EP['held']} set {inf['set']} "
              f"standing {inf['standing']} ticks {inf['ticks']} lead {inf['lead']} fouled {inf['fouled']} "
              f"dist_mm {[round(1000 * d, 1) for d in dist]} "
              f"tilt {[round(tilt_deg(r), 1) for _, r in doms]} "
              f"top_mm {[round(1000 * top_height(p, r), 1) for p, r in doms]}", file=sys.stderr, flush=True)
