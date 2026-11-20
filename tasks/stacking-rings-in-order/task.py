"""Stack a baby's ring-stacking toy in size order. A classic five-ring rocking stacker stands in the middle of the
table: a rounded, weighted base that rocks when it is pushed, with a tapered post standing up from it. Its five rings
(red, orange, yellow, green and blue, from the largest, 124 mm across, to the smallest, 89 mm) lie flat on the table
around it, mixed up.

Task: put all five rings on the post in size order, the largest at the bottom and the smallest at the top. The post is
tapered and each ring's hole is sized to its own place on it: a ring that goes on before the larger ones below it stops
part way down the post, and the larger rings then cannot get past it; so the rings have to go on largest first.

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
from stackkit import Body, BoxShape, CylShape, HullShape, SphereShape, as_numpy, on_reset, on_step, rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Stack all five rings on the toy's post in order of size, the largest at the bottom and the smallest at "
               "the top, and leave the toy standing upright.")

# -- the toy ----------------------------------------------------------------------------------------------------------
# Base frame: origin on the post's axis at the base's flat top (where the largest ring rests), z up the post.
BASE_XY = (0.30, 0.0)
# Sourced (see README): overall 197 mm tall and 130 x 120 mm across, 235 g in all (8.3 oz); the base 1 in (25 mm)
# tall and 5 x 4-3/4 in across; the rings 4-7/8, 4-1/2, 4-1/8, 3-3/4 and 3-1/2 in across. The rest is estimated.
BASE_H = 0.025             # from the bottom of the rocking dome to the base's top face (sourced: 1 in)
BASE_R = 0.0625            # the base's radius (sourced: 5 x 4-3/4 in across; modelled round, 125 mm)
DOME_H = 0.012             # estimate: the rocking dome's depth below the base's straight wall
ROCK_R = (BASE_R ** 2 + DOME_H ** 2) / (2 * DOME_H)      # the dome's radius of curvature: 169 mm
BASE_MASS = 0.115          # estimate: weighted base with its post (toy 235 g in all, less the rings)
BASE_COM_Z = -0.015        # estimate: its centre of mass, 10 mm above the bottom of the dome (weighted)
ROCKER_MASS = 0.005        # of BASE_MASS, carried by the rocker (below)
ROCK_DAMPING = 1.5         # 1/s: angular damping standing in for the rolling resistance that stops a real toy rocking
POST_D0 = 0.060            # estimate: the post's diameter at the base's top face ...
POST_D1 = 0.0316           # ... and at the top of its taper
POST_L = 0.155             # the taper's length
POST_STEP = 0.005          # the taper is modelled as 5 mm tall cylinder sections
TIP_R = 0.016              # a ball on top of the post
POST_TOP = POST_L + TIP_R  # the top of the post above the base's top face (171 mm; the toy 196 mm tall)

RING_H = 0.030             # estimate: every ring is 30 mm tall (five stack to 150 mm under the post's ball)
RING_OD = (0.124, 0.114, 0.105, 0.095, 0.089)          # sourced: 4-7/8, 4-1/2, 4-1/8, 3-3/4, 3-1/2 in
RING_MASS = (0.030, 0.027, 0.024, 0.021, 0.018)        # estimate (120 g of the toy's 235 g)
CLEARANCE = 0.003          # each ring's hole is 3 mm wider than the post where it sits
N_RINGS = 5
N_SEG = 20                 # a ring is modelled as 20 straight segments


def post_d(z: float) -> float:
    """The post's (smooth) diameter at height z above the base's top face."""
    return POST_D0 + (POST_D1 - POST_D0) * min(max(z, 0.0), POST_L) / POST_L


def seat(i: int) -> float:
    """Where ring i (0 = largest) sits, correctly stacked: its bottom, above the base's top face (m)."""
    return i * RING_H


RING_ID = tuple(round(post_d(seat(i)) + CLEARANCE, 5) for i in range(N_RINGS))


def section_d(k: int) -> float:
    """The diameter of the post's k-th 5 mm section (its diameter at the section's bottom)."""
    return post_d(k * POST_STEP)


N_SECTIONS = round(POST_L / POST_STEP)


def stop_height(i: int) -> float:
    """Where ring i comes to rest on the bare post (nothing below it): on top of the highest section wider than its
    hole (its bottom, above the base's top face)."""
    k = max([k for k in range(N_SECTIONS) if section_d(k) > RING_ID[i]], default=-1)
    return (k + 1) * POST_STEP if k >= 0 else 0.0


RED, ORANGE, YELLOW, GREEN, BLUE = ((0.85, 0.12, 0.12), (0.95, 0.50, 0.10), (0.95, 0.82, 0.10), (0.20, 0.65, 0.25),
                                    (0.15, 0.35, 0.85))
RING_COLORS = (RED, ORANGE, YELLOW, GREEN, BLUE)
BASE_COLOR = (0.30, 0.55, 0.85)
POST_COLOR = (0.95, 0.85, 0.30)


DOME_FRACS = (0.05, 0.1, 0.2, 0.3, 0.45, 0.6, 0.8)    # polar angles drawn, as fractions of the rim's (and the rim)
DOME_AZ = 32


def _dome() -> HullShape:
    """The base's rocking bottom as drawn: a spherical cap of 169 mm radius of curvature, 125 mm across and 12 mm deep
    (a closed triangle mesh, drawn only: its contact with the table is the rocker below)."""
    pts, faces = [(0.0, 0.0, -BASE_H)], []
    a_max = math.asin(BASE_R / ROCK_R)
    rings = [(ROCK_R * math.sin(a), -BASE_H + ROCK_R * (1 - math.cos(a)))
             for a in [a_max * f for f in DOME_FRACS] + [a_max]]
    for r, z in rings:
        for j in range(DOME_AZ):
            t = 2 * math.pi * j / DOME_AZ
            pts.append((r * math.cos(t), r * math.sin(t), z))
    pts.append((0.0, 0.0, rings[-1][1]))
    top = len(pts) - 1

    def idx(ring, j):
        return 1 + ring * DOME_AZ + j % DOME_AZ

    last = len(rings) - 1
    for j in range(DOME_AZ):
        faces.append((0, idx(0, j + 1), idx(0, j)))
        for ring in range(last):
            a, b, c, d = idx(ring, j), idx(ring + 1, j), idx(ring + 1, j + 1), idx(ring, j + 1)
            faces += [(a, c, b), (a, d, c)]
        faces.append((top, idx(last, j), idx(last, j + 1)))
    return HullShape(tuple(pts), tuple(faces), BASE_COLOR, collide=False)


def _toy() -> Body:
    upper = BASE_H - DOME_H                                   # the straight part above the cap: 13 mm
    shapes = [_dome(), CylShape((0.0, 0.0, -upper / 2), BASE_R, upper, BASE_COLOR)]
    for k in range(N_SECTIONS):
        shapes.append(CylShape((0.0, 0.0, (k + 0.5) * POST_STEP), section_d(k) / 2, POST_STEP, POST_COLOR))
    shapes.append(SphereShape((0.0, 0.0, POST_L), TIP_R, POST_COLOR))
    rest = BASE_MASS - ROCKER_MASS          # the rocker's share sits at its centre: the whole keeps its COM at BASE_COM_Z
    com_z = (BASE_MASS * BASE_COM_Z - ROCKER_MASS * ROCKER_Z) / rest
    return Body.build("stacker", pos=(*BASE_XY, TOP + BASE_H + 0.0005), mass=rest, color=BASE_COLOR,
                      shapes=shapes, com=(0.0, 0.0, com_z), friction=(0.5, 0.4), solver_iterations=32,
                      damping=(0.0, ROCK_DAMPING))


# The rocking bottom's contact with the table. A convex mesh would rest on its flat facets (a 249-point dome stood
# stably 8 degrees over), so the dome's contact is a true sphere of its 169 mm radius of curvature, in a body welded
# to the toy (a fixed joint) that collides only with the table: never with the rings or the arms (or the toy itself).
# Only its bottom 12 mm, the drawn dome, can reach the table: past 21.7 degrees of tilt the base's rim, which lies on
# the same sphere, touches it.
ROCKER_Z = ROCK_R - BASE_H     # its centre, in the base frame: 144 mm above the base's top face


def _rocker() -> Body:
    return Body.build("rocker", pos=(*BASE_XY, TOP + BASE_H + 0.0005 + ROCKER_Z), mass=ROCKER_MASS, color=BASE_COLOR,
                      shapes=[SphereShape((0.0, 0.0, 0.0), ROCK_R, None)], friction=(0.5, 0.4), solver_iterations=32,
                      damping=(0.0, ROCK_DAMPING), weld_to="stacker", weld_at=(0.0, 0.0, ROCKER_Z),
                      ignore=tuple(f"ring_{i}" for i in range(5)), ignore_robots=True)


# Ring i lies flat on the table at RING_START[i] (its centre); body frame: origin at the ring's centre, z along its axis.
RING_START = ((0.38, 0.19), (0.22, -0.19), (0.22, 0.17), (0.40, -0.18), (0.15, 0.01))


def _ring(i: int) -> Body:
    od, idd = RING_OD[i], RING_ID[i]
    w = (od - idd) / 2
    rm = (od + idd) / 4
    seg = od * math.tan(math.pi / N_SEG) + 0.0005        # the outer edge's length (segments overlap at the inner edge)
    shapes = [BoxShape((rm * math.cos(t), rm * math.sin(t), 0.0), (w, seg, RING_H), RING_COLORS[i], yaw=t)
              for t in (2 * math.pi * k / N_SEG for k in range(N_SEG))]
    x, y = RING_START[i]
    return Body.build(f"ring_{i}", pos=(x, y, TOP + RING_H / 2 + 0.0005), mass=RING_MASS[i], color=RING_COLORS[i],
                      shapes=shapes, friction=(0.45, 0.35), solver_iterations=24, damping=(0.0, 0.05))


OBJECTS = [_toy(), _rocker(), *(_ring(i) for i in range(N_RINGS))]

# -- scoring numbers ---------------------------------------------------------------------------------------------------
SEAT_TOL = 0.005           # a ring rests where it should: its bottom within 5 mm of the top of the ring below
TILT_DEG = 10.0            # a ring's axis within 10 degrees of the post's; the post within 10 degrees of vertical
TOUCH = 0.005              # a ring counts only while neither gripper is within 5 mm of it
CLEAR = 0.02               # full score: both grippers at least 2 cm from the toy and every ring
STILL = 0.002              # full score: nothing moved more than 2 mm over the last 1 s
HOLD_STEPS = 20            # 1 s at 20 Hz
PER_RING = 0.15            # partial credit per ring stacked in order from the bottom
ALL_FIVE_HELD = 0.75       # all five in order, but not yet left standing still on its own for 1 s


# -- geometry (pure; used both in simulation and by the offline tests) ------------------------------------------------

def tilt_deg(r) -> float:
    """The angle (degrees) between a rotation's z axis and world vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.asarray(r, float)[2, 2])))))


def ring_in_base(base_pos, base_r, ring_pos, ring_r) -> dict:
    """Ring pose in the base frame: its bottom's height above the base's top face, its centre's distance from the
    post's axis, and its tilt from the post's axis."""
    br = np.asarray(base_r, float)
    rr = np.asarray(ring_r, float)
    p = br.T @ (np.asarray(ring_pos, float) - np.asarray(base_pos, float))
    cosang = abs(float(br[:, 2] @ rr[:, 2]))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
    return {"bottom": float(p[2] - RING_H / 2), "offset": float(math.hypot(p[0], p[1])), "tilt": tilt}


def ring_correct(i: int, g: dict) -> bool:
    """Ring i is on the post (the post's axis passes through its hole), square to it, and resting at its own place
    in the stack (its bottom within SEAT_TOL of where it sits on the larger rings below it)."""
    return (g["offset"] < RING_ID[i] / 2 and g["tilt"] <= TILT_DEG and abs(g["bottom"] - seat(i)) <= SEAT_TOL)


def ring_distance(points, i: int, ring_pos, ring_r) -> float:
    """The least distance (m) from the points to ring i (an annulus of its inner and outer diameter and height)."""
    if points is None or len(points) == 0:
        return float("inf")
    local = (np.asarray(points, float) - np.asarray(ring_pos, float)) @ np.asarray(ring_r, float)
    r = np.hypot(local[:, 0], local[:, 1])
    dr = np.maximum(np.maximum(RING_ID[i] / 2 - r, r - RING_OD[i] / 2), 0.0)
    dz = np.maximum(np.abs(local[:, 2]) - RING_H / 2, 0.0)
    return float(np.hypot(dr, dz).min())


def toy_distance(points, base_pos, base_r) -> float:
    """The least distance (m) from the points to the toy (its base as a 60 mm radius cylinder, the post as a 23 mm
    radius cylinder up to its top)."""
    if points is None or len(points) == 0:
        return float("inf")
    local = (np.asarray(points, float) - np.asarray(base_pos, float)) @ np.asarray(base_r, float)
    r = np.hypot(local[:, 0], local[:, 1])
    z = local[:, 2]
    d_base = np.hypot(np.maximum(r - BASE_R, 0.0), np.maximum(np.maximum(-BASE_H - z, z), 0.0))
    d_post = np.hypot(np.maximum(r - POST_D0 / 2, 0.0), np.maximum(np.maximum(-z, z - POST_TOP), 0.0))
    return float(min(d_base.min(), d_post.min()))


def post_top(base_pos, base_r) -> np.ndarray:
    return np.asarray(base_pos, float) + np.asarray(base_r, float)[:, 2] * POST_TOP


# -- the score, step by step --------------------------------------------------------------------------------------------

_EP: dict = {}
_DEBUG = bool(os.environ.get("STACK_DEBUG"))


@on_reset
def _reset_state() -> None:
    _EP.clear()
    _EP.update(held=0, recent=[], score=0.0, done=False, steps=0, info={})


_reset_state()


def observe(base_pos, base_r, rings, gripper_points) -> None:
    """One control step: work out the score. ``rings`` is a list of (pos, rotation matrix) for rings 0 (largest) to 4;
    ``gripper_points`` the grippers' points (world, m)."""
    base_pos, base_r = np.asarray(base_pos, float), np.asarray(base_r, float)
    upright = tilt_deg(base_r) <= TILT_DEG
    geo = [ring_in_base(base_pos, base_r, p, r) for p, r in rings]
    touch = [ring_distance(gripper_points, i, p, r) for i, (p, r) in enumerate(rings)]
    k = 0
    if upright:
        while k < N_RINGS and ring_correct(k, geo[k]) and touch[k] >= TOUCH:
            k += 1
    snap = np.vstack([base_pos, post_top(base_pos, base_r)] + [np.asarray(p, float) for p, _ in rings])
    recent = _EP["recent"]
    recent.append(snap)
    del recent[:-(HOLD_STEPS + 1)]
    still = len(recent) == HOLD_STEPS + 1 and all(float(np.abs(r - snap).max()) <= STILL for r in recent)
    clear = (min(touch) >= CLEAR and toy_distance(gripper_points, base_pos, base_r) >= CLEAR)
    done = bool(k == N_RINGS and clear and still)
    _EP["held"] = _EP["held"] + 1 if done else 0
    if _EP["held"] >= HOLD_STEPS:
        score = 1.0
    elif k == N_RINGS:
        score = ALL_FIVE_HELD
    else:
        score = round(PER_RING * k, 4)
    _EP.update(score=score, done=done, steps=_EP["steps"] + 1,
               info={"k": k, "upright": upright, "clear": clear, "still": still, "geo": geo, "touch": touch})


def success(objects) -> bool:
    """True only on steps where score() is 1.0: all five rings on the post in size order, the toy upright, both
    grippers 2 cm clear and everything still, for 1 s. The runtime then ends the episode after its own 1 s of this."""
    return _EP["score"] >= 1.0


def score(objects) -> float:
    """1.0 once all five rings have sat on the post in size order (each resting on the next larger one, the largest on
    the base), the toy upright, both grippers 2 cm clear of the toy and the rings, and nothing moving more than 2 mm,
    for 1 s (20 steps in a row); 0.75 with all five in order but not yet so for 1 s; otherwise 0.15 for each ring in
    order from the bottom (the largest on the base, then the next on it, ...), counting only rings neither gripper is
    within 5 mm of, and only while the toy is upright (within 10 degrees)."""
    return float(_EP["score"])


# -- in the simulator: read the state ----------------------------------------------------------------------------------

_HULLS: dict = {}


def _gripper_points(base, origin) -> np.ndarray:
    """Points on both grippers (the vertices of their collision hulls, world frame)."""
    if not _HULLS:
        from yam_arena.analysis import link_hulls

        hulls = link_hulls()
        for link in ("gripper", "tip_left", "tip_right"):
            _HULLS[link] = np.vstack([h.vertices for h in hulls[link]])
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        for j, n in enumerate(robot.body_names):
            link = n.rsplit("/", 1)[-1]
            if link in _HULLS:
                out.append((_HULLS[link] @ rotation(poses[j][3:7]).T) + poses[j][:3] - origin)
    return np.vstack(out) if out else np.zeros((0, 3))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])

    def pose(name):
        p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
        return p[:3] - origin, rotation(p[3:7])

    base_pos, base_r = pose("stacker")
    rings = [pose(f"ring_{i}") for i in range(N_RINGS)]
    observe(base_pos, base_r, rings, _gripper_points(base, origin))
    if _DEBUG and _EP["steps"] % 10 == 0:
        inf = _EP["info"]
        msg = [f"[stack] step {_EP['steps']} score {_EP['score']} k {inf['k']} held {_EP['held']} "
               f"upright {inf['upright']} tilt {tilt_deg(base_r):.1f} clear {inf['clear']} still {inf['still']} "
               f"base {np.round(1000 * base_pos, 1).tolist()}"]
        for i, (g, t) in enumerate(zip(inf["geo"], inf["touch"])):
            msg.append(f"  ring{i} bottom {1000 * g['bottom']:.1f} (seat {1000 * seat(i):.0f}) "
                       f"offset {1000 * g['offset']:.1f} tilt {g['tilt']:.1f} touch {1000 * min(t, 9.999):.0f} mm")
        print("\n".join(msg), file=sys.stderr, flush=True)
