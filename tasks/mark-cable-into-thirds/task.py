"""Mark a cable into equal thirds, measured along its route.

On the station table a black 6 mm cable, 75 cm long between its two black plug bodies (30 x 14 x 10 mm each), is
laid in an S round three grey pegs (44 mm round, 35 mm tall): from its first plug at
x 42, y +25.5 cm it runs to the right along x 42 cm, turns round peg 1 (centre x 39, y +13 cm), runs towards the
robot along y +10 cm, turns round peg 2 (x 26, y +7 cm), runs to the right along x 23 cm, turns round peg 3
(x 26, y -15 cm) and runs away from the robot along y -18 cm to its second plug (cable end at x 39.6, y -18 cm). Two free orange marker pucks
(24 mm round, 25 mm tall, 15 g) stand near the robot, one in front of each arm (x 12, y +20 and x 12, y -22).

Goal: without moving the cable, stand one marker beside it at one third and the other at two thirds of its length,
measured along the cable from plug to plug, then let go. The straight line between the plugs says nothing about
where the thirds are (the cable winds round the pegs): the robot has to measure along the cable. The one-third point
(on the y +10 cm run) is only reachable by the left arm and the two-thirds point (on the x 23 cm run) only by the
right arm, so each arm places one marker.

Kit note: cablekit spawns cables straight. This task wraps cablekit._spawn_cable so the cable's segments are authored
along the S route and every bend joint's rest angle is its laid angle (the simulated cable springs back towards the
laid S if disturbed). Joint position 0 is the laid shape, so resets restore it exactly.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Without moving the black cable, stand one orange marker on the table beside it at one third and the "
               "other at two thirds of its length, measured along the cable from plug to plug. Then let go.")

# -- route: waypoints joined by straight runs and 3 cm-radius turns round the pegs ------------------------------------
R = 0.003  # 6 mm cable
CORD_LEN = 0.75  # between the plug bodies
RC = 0.030  # cable centreline radius round a peg
WAYPOINTS = ((0.42, 0.29), (0.42, 0.10), (0.23, 0.10), (0.23, -0.18), (0.45, -0.18))
U0 = 0.035  # route position of the cable's first end (start plug face)
SLIDE = float(os.environ.get("MARK_SLIDE", "0"))  # test only: cable slid along its route (VARIATION check)


def _route(step: float = 0.0005):
    """Dense route centreline (K, 2) and its arclength (K,)."""
    W = [np.array(w, float) for w in WAYPOINTS]
    pts, centres = [W[0]], []
    for i in range(1, len(W) - 1):
        a, b, c = W[i - 1], W[i], W[i + 1]
        d1, d2 = (b - a) / np.linalg.norm(b - a), (c - b) / np.linalg.norm(c - b)
        p1 = b - d1 * RC
        turn = np.sign(d1[0] * d2[1] - d1[1] * d2[0])
        cen = p1 + np.array([-d1[1], d1[0]]) * turn * RC
        centres.append(cen)
        a0 = math.atan2(*(p1 - cen)[::-1])
        a1 = math.atan2(*(b + d2 * RC - cen)[::-1])
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        for t in np.linspace(0.0, 1.0, 60):
            pts.append(cen + RC * np.array([math.cos(a0 + da * t), math.sin(a0 + da * t)]))
    pts.append(W[-1])
    P = np.array(pts)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    return P, s, centres


ROUTE, ROUTE_S, PEG_XY = _route()


def route_at(u: float) -> np.ndarray:
    return np.array([np.interp(u, ROUTE_S, ROUTE[:, 0]), np.interp(u, ROUTE_S, ROUTE[:, 1])])


def _chain(u0: float, n: int, seg: float) -> np.ndarray:
    """n+1 joint points on the route, consecutive ones exactly `seg` apart (straight-line), from route position u0."""
    out, u = [route_at(u0)], u0
    for _ in range(n):
        lo, hi = u, u + 2 * seg
        for _ in range(50):  # bisection on the route position of the next joint
            mid = (lo + hi) / 2
            if np.linalg.norm(route_at(mid) - out[-1]) < seg:
                lo = mid
            else:
                hi = mid
        u = (lo + hi) / 2
        out.append(route_at(u))
    return np.array(out)


_N = max(2, round(CORD_LEN / 0.02))
_SEG = CORD_LEN / _N
JOINTS = _chain(U0 + SLIDE, _N, _SEG)  # (N+1, 2): cable start tip, bend joints, end tip
HEADINGS = np.arctan2(np.diff(JOINTS[:, 1]), np.diff(JOINTS[:, 0]))  # per segment
Z0 = TOP + R + 0.0005

PLUG = Plug(size=(0.030, 0.014, 0.010), mass=0.010, color=(0.05, 0.05, 0.05))
CORD = cable("cord", start=(float(JOINTS[0, 0]), float(JOINTS[0, 1]), Z0), yaw=float(HEADINGS[0]),
             length=CORD_LEN, radius=R, start_plug=PLUG, end_plug=PLUG, color=(0.03, 0.03, 0.03),
             bend_stiffness=0.01, bend_damping=0.001)
assert CORD.segments == _N
START_CENTRES = np.c_[(JOINTS[:-1] + JOINTS[1:]) / 2, np.full(_N, Z0)]  # laid segment centres


def _install_laid_shape() -> None:
    """Author the cable's segments along the S route and give every bend joint its laid angle as rest angle."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_laid_shape", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        stage = get_current_stage()
        th0 = HEADINGS[0]
        c0 = START_CENTRES[0, :2]
        rot0 = np.array([[math.cos(-th0), -math.sin(-th0)], [math.sin(-th0), math.cos(-th0)]])

        def set_pose(path, xy_world, th_world):
            loc = rot0 @ (np.asarray(xy_world) - c0)  # in the cable root frame (seg_0 centre, heading th0)
            th = th_world - th0
            ops = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
            ops[0].Set(Gf.Vec3d(float(loc[0]), float(loc[1]), 0.0))
            ops[1].Set(Gf.Quatd(math.cos(th / 2), 0.0, 0.0, math.sin(th / 2)))

        for i in range(_N):
            set_pose(f"{prim_path}/seg_{i}", START_CENTRES[i, :2], HEADINGS[i])
        for i in range(1, _N):
            d = float((HEADINGS[i - 1] - HEADINGS[i] + math.pi) % (2 * math.pi) - math.pi)
            j = stage.GetPrimAtPath(f"{prim_path}/seg_{i}/bend_{i}")
            j.GetAttribute("physics:localRot1").Set(Gf.Quatf(math.cos(d / 2), 0.0, 0.0, math.sin(d / 2)))
        th = HEADINGS[-1]
        e = JOINTS[-1] + (PLUG.size[0] / 2) * np.array([math.cos(th), math.sin(th)])
        set_pose(f"{prim_path}/end_plug", e, th)
        return prim

    spawn._laid_shape = True
    cablekit._spawn_cable = spawn


_install_laid_shape()

# -- pegs and markers ------------------------------------------------------------------------------------------------
PEG_R, PEG_H = 0.022, 0.035
PEGS = [Cylinder(f"peg_{i + 1}", pos=(float(x), float(y), TOP + PEG_H / 2), radius=PEG_R, height=PEG_H,
                 color=(0.50, 0.51, 0.54)) for i, (x, y) in enumerate(PEG_XY)]
MK_R, MK_H, MK_MASS = 0.012, 0.025, 0.015
ORANGE = (0.95, 0.45, 0.05)
MARKER_XY = {"marker_l": (0.12, 0.20), "marker_r": (0.12, -0.22)}
MARKERS = [Cylinder(n, pos=(x, y, TOP + MK_H / 2 + 0.0005), radius=MK_R, height=MK_H, mass=MK_MASS, color=ORANGE)
           for n, (x, y) in MARKER_XY.items()]
MARKER_NAMES = list(MARKER_XY)

OBJECTS = [*PEGS, CORD, *MARKERS]

# Seeded variation for a later runner (not applied by the reference demo). "slide": the cable is laid along the same
# route but shifted along it by up to +-3 cm (both thirds move by the same amount along the route); "layout": pegs
# and cable shifted/rotated together as one rigid layout; markers jittered on their own.
VARIATION = {
    "cable:cord": {"slide": 0.03},
    "layout:pegs+cord": {"dx": 0.02, "dy": 0.015, "yaw": 0.05},
    "obj:marker_l": {"dx": 0.02, "dy": 0.02, "yaw": 3.14},
    "obj:marker_r": {"dx": 0.02, "dy": 0.02, "yaw": 3.14},
}

# -- scoring ---------------------------------------------------------------------------------------------------------
NEAR = 0.030  # marker centre within 3 cm of the cable centreline (its side within ~1.5 cm of the cable)
TOL_FULL, TOL_HALF = 0.015, 0.025  # along-cable error for full / half credit per marker
MOVE_MAX = 0.025  # no point of the cable more than 2.5 cm from where it was laid
RELEASE_DIST = 0.030
STAND_Z = TOP + MK_H / 2 + 0.004  # marker standing on the table (not on the cable, a peg or the other marker)
TARGETS = (CORD_LEN / 3, 2 * CORD_LEN / 3)


def _cable_line(c):
    """Polyline start tip -> centres -> end tip with its along-cable length (segment chain, no stretch)."""
    pts = np.vstack([c.start_tip, c.points, c.end_tip])
    s = np.r_[0.0, (np.arange(len(c.points)) + 0.5) * c.seg_len, len(c.points) * c.seg_len]
    return pts, s


def locate(objects, name):
    """(horizontal distance from the marker centre to the cable centreline, along-cable position of that nearest
    point from the start plug face) in m."""
    pts, s = _cable_line(objects["cord"])
    p = np.asarray(objects[name].pos)[:2]
    best = (float("inf"), 0.0)
    for a, b, sa, sb in zip(pts[:-1, :2], pts[1:, :2], s[:-1], s[1:]):
        ab = b - a
        t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
        d = float(np.linalg.norm(a + t * ab - p))
        if d < best[0]:
            best = (d, sa + t * (sb - sa))
    return best


def placed(objects, name) -> float | None:
    """Along-cable position of a marker standing on the table beside the cable, else None."""
    m = objects[name]
    if m.pos[2] > STAND_Z:
        return None
    d, s = locate(objects, name)
    return s if d <= NEAR else None


def _credit(err: float) -> float:
    return 0.5 if err <= TOL_FULL else 0.25 if err <= TOL_HALF else 0.0


def thirds_credit(objects) -> float:
    """Best pairing of the two markers with the 1/3 and 2/3 points; 0.5 / 0.25 / 0 per marker."""
    s = [placed(objects, n) for n in MARKER_NAMES]
    best = 0.0
    for a, b in ((0, 1), (1, 0)):
        tot = 0.0
        for k, i in enumerate((a, b)):
            if s[i] is not None:
                tot += _credit(abs(s[i] - TARGETS[k]))
        best = max(best, tot)
    return best


def cable_moved(objects) -> float:
    """Max horizontal distance (m) of a segment centre from its laid position."""
    return float(np.linalg.norm(objects["cord"].points[:, :2] - START_CENTRES[:, :2], axis=1).max())


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_thirds_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._thirds_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(objects) -> bool:
    """Every gripper TCP at least 3 cm from the cable, its plugs and both markers."""
    c = objects["cord"]
    pts = np.vstack([c.polyline()] + [p[None] for p in (c.start_plug, c.end_plug) if p is not None])
    for g in grippers():
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_DIST:
            return False
        for n in MARKER_NAMES:
            m = objects[n]
            if float(np.linalg.norm(g - np.clip(g, m.low, m.high))) < RELEASE_DIST:
                return False
    return True


def score(objects) -> float:
    """0 if a gripper is within 3 cm of the cable, its plugs or a marker, or any part of the cable is more than 2.5 cm from where
    it was laid. Otherwise per marker standing beside the cable (centre within 3 cm of it): 0.5 if within 1.5 cm
    along the cable of its third, 0.25 if within 2.5 cm; markers paired with the 1/3 and 2/3 points the best way."""
    if cable_moved(objects) > MOVE_MAX or not released(objects):
        return 0.0
    return thirds_credit(objects)


def success(objects) -> bool:
    return score(objects) >= 0.999 and all(objects[n].resting(max_speed=0.01) for n in MARKER_NAMES)


def distance(objects) -> float:
    """Sum of the along-cable errors of the best pairing (1 m for a marker not beside the cable)."""
    s = [placed(objects, n) for n in MARKER_NAMES]
    e = [[abs(x - t) if x is not None else 1.0 for t in TARGETS] for x in s]
    return float(min(e[0][0] + e[1][1], e[1][0] + e[0][1]))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    s = [placed(objects, n) for n in MARKER_NAMES]
    lifted = sum(float(np.linalg.norm(np.asarray(objects[n].pos[:2]) - MARKER_XY[n]) > 0.03) for n in MARKER_NAMES)
    return {"markers_moved": lifted / 2, "beside_cable": sum(x is not None for x in s) / 2,
            "at_thirds": thirds_credit(objects), "cable_kept": float(cable_moved(objects) <= MOVE_MAX),
            "released": float(released(objects))}


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------

PAD = 0.010  # TCP above the marker centre when gripping
HI = TOP + 0.09
GAP = 0.021  # marker centre this far from the cable centreline (6 mm clear of the cable side)


def cable_point(objects, s_target):
    """(xy, unit tangent) of the live cable at along-cable position s_target."""
    pts, s = _cable_line(objects["cord"])
    k = int(np.clip(np.searchsorted(s, s_target) - 1, 0, len(s) - 2))
    t = (s_target - s[k]) / (s[k + 1] - s[k])
    xy = pts[k, :2] + t * (pts[k + 1, :2] - pts[k, :2])
    tan = pts[k + 1, :2] - pts[k, :2]
    return xy, tan / np.linalg.norm(tan)


def beside(objects, s_target):
    """Marker target beside the cable at s_target, on the side away from the nearest peg."""
    xy, tan = cable_point(objects, s_target)
    nrm = np.array([-tan[1], tan[0]])
    cands = [xy + nrm * GAP, xy - nrm * GAP]
    return max(cands, key=lambda c: min(np.linalg.norm(c - np.asarray(p)) for p in PEG_XY))


def _log(ctx, tag):
    st = ctx.states()
    info = " ".join(f"{n}: pos={np.round(st[n].pos, 3)} loc={np.round(locate(st, n), 4)}" for n in MARKER_NAMES)
    print(f"[demo t={ctx.t:5.1f}] {tag}: moved={cable_moved(st) * 1000:.1f}mm rel={released(st)} "
          f"score={score(st):.3f} prog={progress(st)}\n    {info}", flush=True)


def _place(ctx, arm, name, target_fn, hold=False):
    """Pick marker `name`, carry it with the wrist camera facing the drop spot, stand it at target_fn(states) (re-read
    from the live cable just before the drop), let go and lift clear."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    st = ctx.states()
    m = st[name].pos
    tgt = target_fn(st)
    yaw = math.atan2(tgt[1] - m[1], tgt[0] - m[0])  # wrist camera (8 cm along the yaw) looks at the drop spot
    yield from arm.move((m[0], m[1], HI), yaw=yaw, finger=FINGER_OPEN, duration=1.3)
    m = ctx.states()[name].pos
    yield from arm.move((m[0], m[1], m[2] + PAD), yaw=yaw, duration=0.9)
    yield from arm.grip(FINGER_CLOSED)
    off = ctx.states()[name].pos[:2] - np.array(m[:2])  # marker offset from the TCP after closing
    yield from arm.move((m[0], m[1], HI), yaw=yaw, duration=0.7)
    tgt = target_fn(ctx.states())
    tx, ty = tgt[0] - off[0], tgt[1] - off[1]
    place_z = TOP + MK_H / 2 + PAD + 0.002
    yield from arm.move((tx, ty, HI), yaw=yaw, duration=1.4)
    yield from arm.move((tx, ty, place_z + 0.02), yaw=yaw, duration=0.8)
    # fine correction from the marker's measured position before setting it down
    st = ctx.states()
    err = target_fn(st) - st[name].pos[:2]
    tx, ty = tx + err[0], ty + err[1]
    yield from arm.move((tx, ty, place_z), yaw=yaw, duration=0.7)
    if hold:
        return
    yield from arm.grip(FINGER_OPEN, 0.5)
    yield from arm.move((tx, ty, HI), yaw=yaw, duration=0.7)
    yield from arm.move((0.15, math.copysign(0.25, arm.base[1]), HI), yaw=0.0, duration=1.0)


def _nominal_beside(s_target):
    """Marker spot beside the cable as laid in the NOMINAL layout (no slide): what a memorised policy would use."""
    joints = _chain(U0, _N, _SEG)
    s = np.arange(_N + 1) * _SEG
    xy = np.array([np.interp(s_target, s, joints[:, 0]), np.interp(s_target, s, joints[:, 1])])
    k = int(min(s_target // _SEG, _N - 1))
    tan = (joints[k + 1] - joints[k]) / _SEG
    nrm = np.array([-tan[1], tan[0]])
    return max([xy + nrm * GAP, xy - nrm * GAP], key=lambda c: min(np.linalg.norm(c - p) for p in PEG_XY))


def demo_script(ctx):
    """Left arm stands its marker at one third of the cable (along-cable position 25 cm from the start plug, on the
    y +10 cm run), right arm stands its marker at two thirds (50 cm, on the x 23 cm run). Targets are read from the
    live cable. Wrist cameras face the drop spot while carrying.

    MARK_CHEAT (test only): none (no motion) | chord (markers beside the cable at the points nearest the 1/3 and 2/3
    points of the straight line between the plugs) | middle (both markers beside the middle of the cable) | hold
    (correct placement but the right arm keeps holding its marker) | memo (markers at the nominal thirds; run with
    MARK_SLIDE=0.03 so the cable lies 3 cm further along its route)."""
    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("MARK_CHEAT", "")
    _log(ctx, "start")
    if cheat == "none":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if cheat == "chord":
        def chord(k):
            def f(st):
                c = st["cord"]
                p = c.start_tip[:2] + (c.end_tip[:2] - c.start_tip[:2]) * TARGETS[k] / CORD_LEN
                pts, s = _cable_line(c)
                i = int(np.argmin(np.linalg.norm(pts[:, :2] - p, axis=1)))
                return beside(st, s[i])
            return f
        ta, tb = chord(0), chord(1)
    elif cheat == "middle":
        ta, tb = (lambda st: beside(st, CORD_LEN / 2 - 0.012)), (lambda st: beside(st, CORD_LEN / 2 + 0.012))
    elif cheat == "memo":
        a, b = _nominal_beside(TARGETS[0]), _nominal_beside(TARGETS[1])
        ta, tb = (lambda st: a), (lambda st: b)
    else:
        ta, tb = (lambda st: beside(st, TARGETS[0])), (lambda st: beside(st, TARGETS[1]))
    # each arm places the marker whose target lies on its side of the midline (reach)
    first = ta(ctx.states())
    if first[1] < 0:
        ta, tb = tb, ta
    yield from _place(ctx, L, "marker_l", ta)
    _log(ctx, "left placed")
    yield from _place(ctx, Rt, "marker_r", tb, hold=(cheat == "hold"))
    _log(ctx, "right placed")
    yield from ctx.both(L.wait(1.0), Rt.wait(1.0))
    _log(ctx, "end")
