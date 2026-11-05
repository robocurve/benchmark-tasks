"""On the station table, a 4 cm deep cable trench runs left-right (along y) 30 cm in front of the robot: two grey
ground slabs, 4 cm thick, leave a 6 cm wide slot between them (x 27 .. 33 cm), closed at both ends (y +-31 cm). The
rim of the slot (the trench edge) is orange: a 1 cm strip on each long side and the tops of both 2 cm end blocks. A free black 8 mm cable, 50 cm long with
a black end cap (35 x 16 x 12 mm, 15 g) on each end, lies straight on the near slab, parallel to the trench and 8 cm
in front of it (x 19 cm, y -25 .. +25 cm). The robot must lower the whole cable into the trench without the cable
ever touching the trench edge, then let go. The cable is too long to be lifted clear of the slab with one
straight-down grasp: two arms lift it by its end caps, carry it over the slot and lower it straight in.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Lower the whole black cable, end caps included, down into the trench without letting any part of it "
               "touch the orange trench edge, then let go of it.")

# -- trench: two ground slabs with a slot between them, closed at both ends; orange 1 cm rim strips -----------------
DEPTH = 0.04  # slab thickness = trench depth
G = TOP + DEPTH  # ground (slab top) level
X0, X1 = 0.27, 0.33  # slot (trench) between x 0.27 and 0.33 (6 cm wide)
YT = 0.31  # slot runs y -0.31 .. +0.31
RIM = 0.01  # orange rim strip width
SLAB_X0, SLAB_X1 = 0.14, 0.40
YE = YT + 0.02  # slabs and end blocks end at y +-0.33
GREY = (0.55, 0.53, 0.50)
ORANGE = (1.0, 0.45, 0.0)


def _slab(name, x0, x1, y0, y1, color):
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + DEPTH / 2), size=(x1 - x0, y1 - y0, DEPTH), color=color)


TRENCH = [
    _slab("ground_near", SLAB_X0, X0 - RIM, -YE, YE, GREY),
    _slab("rim_near", X0 - RIM, X0, -YE, YE, ORANGE),
    _slab("rim_far", X1, X1 + RIM, -YE, YE, ORANGE),
    _slab("ground_far", X1 + RIM, SLAB_X1, -YE, YE, GREY),
    _slab("rim_end_left", X0, X1, YT, YE, ORANGE),
    _slab("rim_end_right", X0, X1, -YE, -YT, ORANGE),
]

# -- the cable: free, straight on the near slab, parallel to the trench ------------------------------------------------
R = 0.004
CABLE_LEN = 0.50
CABLE_X = 0.19
CAP = Plug(size=(0.035, 0.016, 0.012), mass=0.015, color=(0.08, 0.08, 0.08))
CAP_R = 0.010  # end cap cross-section half-diagonal (8 x 6 mm half sides), used as its "radius" for the edge check
CORD = cable("cord", start=(CABLE_X, -CABLE_LEN / 2, G + CAP.size[2] / 2 + 0.0005), yaw=math.pi / 2,
             length=CABLE_LEN, radius=R, start_plug=CAP, end_plug=CAP, cable_mass=0.05, color=(0.05, 0.05, 0.05),
             bend_stiffness=0.01, bend_damping=0.001)

OBJECTS = [*TRENCH, CORD]

TWIN_EXTRA = [
    {"name": "end_caps", "kind": "joint", "real": "3D-printed PLA cap (35 x 16 x 12 mm, 15 g) glued on each cord end"},
    {"name": "rim_chalk", "kind": "decal", "size_mm": [10, 660], "colour": "orange",
     "pose": [X0 - RIM / 2, 0.0, DEPTH], "note": "rim strips are painted orange and rubbed with chalk before a trial; "
     "chalk marks on the cord show where it touched the edge"},
]

# -- trench-edge contact monitor -----------------------------------------------------------------------------------
# The cable's link positions are read straight from PhysX after EVERY physics substep. The cable (and both end caps)
# is sampled every ~2 mm along its length; a sample whose centre comes within its radius + 2 mm of the trench edge
# (the rim line where the slab top meets the slot wall, all four sides) marks that sample as "touched". The set of
# touched samples is latched for the episode; its total length is the length of cable that touched the edge (in a
# real trial: the length of chalk marks the chalked rim leaves on the cord).
TOUCH_TOL = 0.002
SUB = 10  # samples per polyline piece (seg 2 cm -> 2 mm)
CAP_SUB = 8
_MON: dict = {"marked": None, "env": None, "art": None, "idx": None, "polls": 0, "err": None, "first": None}


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    try:
        import warp as wp

        return wp.to_torch(x).detach().cpu().numpy()
    except Exception:  # pragma: no cover
        return np.asarray(x)


def samples(pts: np.ndarray, plug_s, plug_e) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Dense samples along cap - cable - cap: (points (M,3), radius (M,), length weight (M,))."""
    pts = np.asarray(pts, dtype=float)
    sd = pts[0] - pts[1]
    ed = pts[-1] - pts[-2]
    sd /= np.linalg.norm(sd) + 1e-12
    ed /= np.linalg.norm(ed) + 1e-12
    L = CORD.seg_len
    st, et = pts[0] + sd * L / 2, pts[-1] + ed * L / 2
    poly = np.vstack([st, pts, et])
    t = (np.arange(SUB) + 0.5) / SUB
    pieces = [a + (b - a) * t[:, None] for a, b in zip(poly[:-1], poly[1:])]
    cab = np.vstack(pieces)
    w = np.r_[np.full(SUB, L / 2 / SUB), np.full(SUB * (len(poly) - 3), L / SUB), np.full(SUB, L / 2 / SUB)]
    out_p, out_r, out_w = [cab], [np.full(len(cab), R)], [w]
    tc = (np.arange(CAP_SUB) + 0.5) / CAP_SUB
    for tip, c in ((st, plug_s), (et, plug_e)):
        if c is None:
            continue
        ax = np.asarray(c, dtype=float) - tip  # the cap is welded at the tip: tip -> centre is its axis
        ax /= np.linalg.norm(ax) + 1e-12
        out_p.append(tip + ax * (CAP.size[0] * tc)[:, None])
        out_r.append(np.full(CAP_SUB, CAP_R))
        out_w.append(np.full(CAP_SUB, CAP.size[0] / CAP_SUB))
    return np.vstack(out_p), np.concatenate(out_r), np.concatenate(out_w)


def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    t = np.clip(((p - a) @ ab) / float(ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


_RIM_LINES = [((X0, -YT, G), (X0, YT, G)), ((X1, -YT, G), (X1, YT, G)),
              ((X0, YT, G), (X1, YT, G)), ((X0, -YT, G), (X1, -YT, G))]
_RIM_LINES = [(np.array(a, float), np.array(b, float)) for a, b in _RIM_LINES]


def edge_dist(p: np.ndarray) -> np.ndarray:
    """Distance (m) of each point from the trench edge (the rim line on all four sides)."""
    return np.min([_seg_dist(p, a, b) for a, b in _RIM_LINES], axis=0)


def touching(p: np.ndarray, r: np.ndarray) -> np.ndarray:
    return edge_dist(p) <= r + TOUCH_TOL


def _reset_monitor() -> None:
    _MON.update(marked=None, polls=0, first=None)


def _mark(p, r, w) -> None:
    if not np.isfinite(p).all():
        return
    hit = touching(p, r)
    if _MON["marked"] is None or len(_MON["marked"]) != len(hit):
        _MON["marked"] = np.zeros(len(hit), bool)
    if hit.any() and _MON["first"] is None:
        i = int(np.argmax(hit))
        _MON["first"] = (np.round(p[i], 3).tolist(), _MON["polls"])
        if os.environ.get("TRENCH_DEBUG"):
            print(f"[trench] edge touched at {np.round(p[i], 3)}", flush=True)
    _MON["marked"] |= hit
    _MON["w"] = w


def _poll(mgr) -> None:
    """Called after every physics substep: mark cable samples touching the trench edge."""
    env = _MON["env"]
    if env is None:
        return
    try:
        if _MON["art"] is None:
            art = env.unwrapped.scene[CORD.name]
            names = list(art.body_names)
            seg = [names.index(f"seg_{i}") for i in range(CORD.segments)]
            _MON["art"], _MON["idx"] = art, (seg, names.index("start_plug"), names.index("end_plug"))
            _MON["origin"] = _np(env.unwrapped.scene.env_origins[0]).astype(float)
        art = _MON["art"]
        seg, ps, pe = _MON["idx"]
        tf = _np(art.root_physx_view.get_link_transforms()).reshape(-1, len(art.body_names), 7)[0, :, :3]
        tf = tf.astype(float) - _MON["origin"]
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[trench] edge monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    _mark(*samples(tf[seg], tf[ps], tf[pe]))


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_trench_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        base_step(cls)
        _poll(cls)

    step._trench_hooked = True
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def _install_hooks() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_trench_hooked", False):
        return
    base_reset = cls.reset

    def reset(self, *a, **k):
        _MON["env"], _MON["art"] = None, None  # the env may have been rebuilt
        out = base_reset(self, *a, **k)
        _reset_monitor()  # the reset settle steps leave the cable on the slab, 7 cm from the edge
        _MON["env"] = self._env
        return out

    reset._trench_hooked = True
    cls.reset = reset


_install_hooks()


def touched_len() -> float:
    """Length (m) of cable (and caps) that has touched the trench edge this episode."""
    m = _MON["marked"]
    if m is None or not m.any():
        return 0.0
    return float(_MON["w"][m].sum())


# -- grippers (release check) ---------------------------------------------------------------------------------------


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (yam_arena.runtime.tcp_world, env 0); [] if unknown (offline)."""
    env = _MON["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = _np(base.scene.env_origins[0]).astype(np.float64)
    return [np.asarray(_np(tcp_world(base.scene[k])), dtype=np.float64).reshape(-1)[:3] - origin
            for k in ("left_robot", "right_robot")]


# -- scoring ---------------------------------------------------------------------------------------------------------
DRAG_LEN = 0.01  # more than 1 cm of cable touched the edge = dragged over it or rested on it (<= 1 cm: a graze)
RELEASE_DIST = 0.03  # every gripper TCP at least 3 cm from the cable and caps


def _state_samples(objects):
    c = objects[CORD.name]
    return samples(c.points, c.start_plug, c.end_plug)


def in_trench_mask(p: np.ndarray) -> np.ndarray:
    """Sample centre inside the slot (between the slot walls and end walls) and below the rim."""
    return (p[:, 0] > X0) & (p[:, 0] < X1) & (np.abs(p[:, 1]) < YT) & (p[:, 2] < G)


def in_fraction(objects) -> float:
    p, _, w = _state_samples(objects)
    if not np.isfinite(p).all():
        return 0.0
    return float(w[in_trench_mask(p)].sum() / w.sum())


def released(objects) -> bool:
    p = _state_samples(objects)[0]
    return all(float(np.linalg.norm(p - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 if more than 1 cm of the cable (end caps included) ever touched the trench edge (dragged over it or rested
    on it), if any part of the cable or caps is not inside the trench below the rim at the end, or a gripper is within
    3 cm of it. Else 1.0 when the cable never touched the edge, 0.5 when it touched it over at most 1 cm (a graze)."""
    p, _, _ = _state_samples(objects)
    if not np.isfinite(p).all():
        return 0.0
    tl = touched_len()
    if tl > DRAG_LEN or not in_trench_mask(p).all() or not released(objects):
        return 0.0
    return 1.0 if tl == 0.0 else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CORD.name].resting(max_speed=0.02)


def distance(objects) -> float:
    """Length of cable (m) still outside the trench; 1 m once dragged over the edge."""
    if touched_len() > DRAG_LEN:
        return 1.0
    return float((1.0 - in_fraction(objects)) * (CABLE_LEN + 2 * CAP.size[0]))


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    c = objects[CORD.name]
    lifted = float(np.isfinite(c.points).all() and (c.points[:, 2] > G + R + 0.01).all())
    return {"edge_clean": float(touched_len() == 0.0), "not_dragged": float(touched_len() <= DRAG_LEN),
            "lifted_clear": lifted, "in_trench": in_fraction(objects), "released": float(released(objects))}


VARIATION = {
    "cable:cord": {"dx": 0.015, "dy": 0.015, "yaw": 0.05},
}

# -- closed-loop scripted demo (cabledemo.py) -----------------------------------------------------------------------
XC = (X0 + X1) / 2


def demo_script(ctx):
    """TRENCH_DEMO=correct (default) | nothing | push (both arms push the cable sideways over the edge into the
    trench) | onearm (one arm lifts the cable by its middle as high as it reaches, carries it over the trench and
    lowers it in) | oneend (one end cap is laid in the trench first, then the other cap is carried over, low) |
    hold (correct, but the grippers keep holding the caps above the trench).

    Correct: each arm takes the end cap on its side (fingers across the cap, both wrist cameras on the -y side), both
    lift it 15 cm above the slab with the cable held nearly straight, carry it over the slot's centre line (aiming
    the measured caps, not the TCPs), wait until the cable stops swinging, lower it straight down until the caps are
    5 mm above the rim, let go (the cable drops about 4 cm onto the trench floor) and back away.
    """
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    mode = os.environ.get("TRENCH_DEMO", "correct")
    yl = yr = -math.pi / 2  # fingers close along x across each cap; both wrist cameras on the -y side

    def log(tag):
        st = ctx.states()
        c = st[CORD.name]
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st):.2f} touched={touched_len() * 100:.1f}cm "
              f"in={in_fraction(st):.2f} rel={released(st)} caps={np.round(c.start_plug, 3)} "
              f"{np.round(c.end_plug, 3)} zmin={c.points[:, 2].min() - G:+.3f} xrange="
              f"{c.points[:, 0].min():.3f}..{c.points[:, 0].max():.3f} polls={_MON['polls']} first={_MON['first']}",
              flush=True)

    log("start")
    if mode == "nothing":
        yield from L.wait(4.0)
        log("end")
        return
    c = ctx.states()[CORD.name]
    if mode == "push":  # closed fingers behind the cable, push it across the slab and over the edge
        z = G + 0.016
        yield from ctx.both(L.move((CABLE_X - 0.04, 0.12, z + 0.05), yaw=0.0, finger=FINGER_CLOSED, duration=1.5),
                            Rt.move((CABLE_X - 0.04, -0.12, z + 0.05), yaw=0.0, finger=FINGER_CLOSED, duration=1.5))
        yield from ctx.both(L.move((CABLE_X - 0.04, 0.12, z), yaw=0.0, duration=0.8),
                            Rt.move((CABLE_X - 0.04, -0.12, z), yaw=0.0, duration=0.8))
        yield from ctx.both(L.move((X1 - 0.012, 0.12, z), yaw=0.0, duration=4.0),
                            Rt.move((X1 - 0.012, -0.12, z), yaw=0.0, duration=4.0))
        yield from ctx.both(L.move((X1 - 0.012, 0.12, z + 0.08), yaw=0.0, duration=0.8),
                            Rt.move((X1 - 0.012, -0.12, z + 0.08), yaw=0.0, duration=0.8))
        for _ in range(4):
            yield from L.wait(0.5)
            log("pushed")
        return
    if mode == "onearm":  # left arm alone, by the middle, as high as a straight-down grip reaches
        mid = c.points[len(c.points) // 2]
        yield from L.move((mid[0], mid[1], G + 0.05), yaw=-math.pi / 2, finger=FINGER_OPEN, duration=1.5)
        yield from L.move((mid[0], mid[1], mid[2] + 0.010), yaw=-math.pi / 2, duration=1.0)
        yield from L.grip(FINGER_CLOSED)
        yield from L.move((mid[0], mid[1], TOP + 0.20), yaw=-math.pi / 2, duration=2.0)
        log("lifted")
        yield from L.move((XC, mid[1], TOP + 0.16), yaw=-math.pi / 2, duration=2.5)
        yield from L.move((XC, mid[1], G + 0.02), yaw=-math.pi / 2, duration=2.5)
        yield from L.grip(FINGER_OPEN)
        yield from L.move((XC - 0.08, mid[1] + 0.05, G + 0.12), yaw=-math.pi / 2, duration=1.0)
        for _ in range(4):
            yield from L.wait(0.5)
            log("released")
        return
    pl, pr = c.end_plug, c.start_plug  # left arm takes the +y cap, right arm the -y cap
    hi = G + 0.05
    yield from ctx.both(L.move((pl[0], pl[1], hi), yaw=yl, finger=FINGER_OPEN, duration=1.5),
                        Rt.move((pr[0], pr[1], hi), yaw=yr, finger=FINGER_OPEN, duration=1.5))
    yield from ctx.both(L.move((pl[0], pl[1], pl[2] + 0.010), yaw=yl, duration=1.0),
                        Rt.move((pr[0], pr[1], pr[2] + 0.010), yaw=yr, duration=1.0))
    yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
    if mode == "oneend":  # lay the left cap in the trench, then bring the right cap over low (the cable drags)
        yield from L.move((pl[0], pl[1], TOP + 0.15), yaw=yl, duration=1.0)
        yield from L.move((XC, pl[1] - 0.01, TOP + 0.15), yaw=yl, duration=1.5)
        yield from L.move((XC, pl[1] - 0.01, G + 0.02), yaw=yl, duration=1.5)
        yield from L.grip(FINGER_OPEN)
        yield from L.move((XC - 0.06, pl[1] + 0.03, G + 0.12), yaw=yl, duration=1.0)
        log("left cap in")
        yield from Rt.move((pr[0], pr[1], G + 0.03), yaw=yr, duration=0.8)
        yield from Rt.move((XC, pr[1] + 0.01, G + 0.03), yaw=yr, duration=3.0)
        yield from Rt.move((XC, pr[1] + 0.01, G + 0.02), yaw=yr, duration=0.5)
        yield from Rt.grip(FINGER_OPEN)
        yield from Rt.move((XC - 0.06, pr[1] - 0.03, G + 0.12), yaw=yr, duration=1.0)
        for _ in range(4):
            yield from L.wait(0.5)
            log("released")
        return
    carry = TOP + 0.20
    yield from ctx.both(L.move((pl[0], pl[1], carry), yaw=yl, duration=1.5),
                        Rt.move((pr[0], pr[1], carry), yaw=yr, duration=1.5))
    ycap = float(os.environ.get("TRENCH_YCAP", "0.263"))
    yield from ctx.both(L.move((pl[0], ycap, carry), yaw=yl, duration=0.6),
                        Rt.move((pr[0], -ycap, carry), yaw=yr, duration=0.6))
    yield from L.wait(0.3)
    log("lifted")
    # carry over the slot, aiming each cap (not the TCP) at the slot centre line: the pinched caps sit up to ~1 cm
    # off the TCP
    c = ctx.states()[CORD.name]
    ox_l, ox_r = float(c.end_plug[0] - pl[0]), float(c.start_plug[0] - pr[0])
    yield from ctx.both(L.move((XC - ox_l, ycap, carry), yaw=yl, duration=3.5),
                        Rt.move((XC - ox_r, -ycap, carry), yaw=yr, duration=3.5))
    # wait for the hanging cable to stop swinging (closed loop, at most 5 s)
    prev = ctx.states()[CORD.name].points
    for _ in range(20):
        yield from L.wait(0.25)
        c = ctx.states()[CORD.name]
        moved = float(np.linalg.norm(c.points - prev, axis=1).max())
        prev = c.points
        if np.abs(c.points[:, 0] - XC).max() < 0.006 and moved < 0.003:
            break
    log("over trench")
    c = ctx.states()[CORD.name]
    ox_l, ox_r = float(c.end_plug[0] - L.tcp()[0]), float(c.start_plug[0] - Rt.tcp()[0])
    drop = G + CAP.size[2] / 2 + 0.005 + 0.010  # TCP with the cap bottom 5 mm above the rim
    def trace(n):
        for k in range(n):
            yield
            if os.environ.get("TRENCH_DEBUG") and k % 5 == 0:
                c = ctx.states()[CORD.name]
                p = c.points
                print(f"  down t={ctx.t:5.2f} zmid={p[len(p) // 2, 2] - G:+.3f} x={p[:, 0].min():.3f}..{p[:, 0].max():.3f}"
                      f" at y={p[np.argmin(p[:, 0]), 1]:+.3f}/{p[np.argmax(p[:, 0]), 1]:+.3f} touched={touched_len():.3f}",
                      flush=True)

    yield from ctx.both(ctx.both(L.move((XC - ox_l, ycap, drop), yaw=yl, duration=5.0),
                                 Rt.move((XC - ox_r, -ycap, drop), yaw=yr, duration=5.0)), trace(100))
    yield from L.wait(0.8)
    log("lowered")
    if mode == "hold":  # the cable is held level with the rim over the slot; the grippers never let go
        for _ in range(6):
            yield from L.wait(0.5)
            log("held")
        return
    yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))
    log("opened")
    yield from ctx.both(L.move((XC - ox_l, ycap, drop + 0.06), yaw=yl, duration=0.8),
                        Rt.move((XC - ox_r, -ycap, drop + 0.06), yaw=yr, duration=0.8))
    yield from ctx.both(L.move((XC - 0.10, ycap + 0.02, drop + 0.08), yaw=yl, duration=1.0),
                        Rt.move((XC - 0.10, -ycap - 0.02, drop + 0.08), yaw=yr, duration=1.0))
    log("released")
    for _ in range(4):
        yield from L.wait(0.5)
        log("settle")
    log("end")
