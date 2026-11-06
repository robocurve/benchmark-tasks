"""Mooring with a messenger line on the station table. The left half of the table is the boat's deck, the right half
the dock; between them a 6 cm wide blue tape strip along x marks the water gap (y -3 to +3 cm). On the boat, far from
both arms, a fixed white bow-cleat block holds the start of a heavy white 12 mm dock line (50 cm). The dock line lies
straight out towards the far left corner and ends in a red rigid square eye (60 x 60 x 12 mm of 12 mm bars, 36 mm
opening). A thin yellow 6 mm messenger line (65 cm) is tied to the eye's left side bar and lies straight back along
the boat's deck towards the left arm; its free end is the only part of the line rig the left arm can reach. The eye
and the whole dock line lie at least 61 cm from both arm bases, out of reach. On the dock, out of the left arm's reach,
stands a fixed black bollard (20 mm round, 50 mm tall). The robot must haul the eye across the gap with the messenger
and drop it over the bollard, then let go of the eye and the dock line.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: HAUL_DEMO=correct (default) | nothing | reach | hold | beside.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Moor the boat: drop the red eye of the heavy white dock line over the black bollard on the dock. "
               "The eye is out of reach, so haul it across the water gap with the thin yellow messenger line tied "
               "to it. Then let go of the eye and the dock line.")

# -- water gap (blue tape strip): boat deck y > +3 cm, dock y < -3 cm --------------------------------------------------
GAP_HALF = 0.03
DOCK_EDGE_Y = -GAP_HALF
TAPE_T = 0.0006
WATER = Box("water_gap", pos=(0.40, 0.0, TOP + TAPE_T / 2), size=(0.90, 2 * GAP_HALF, TAPE_T),
            color=(0.15, 0.45, 0.85))

# -- dock line: start fixed at the bow cleat, square rigid eye on its free end -----------------------------------------
R = 0.006  # 12 mm dock line
LINE_LEN = 0.50
LINE_YAW = math.radians(70.0)
U0 = np.array([math.cos(LINE_YAW), math.sin(LINE_YAW)])
V0 = np.array([-U0[1], U0[0]])
A = np.array([0.60, 0.075])  # where the line leaves the cleat (>= 0.61 m from both arm bases)
BAR = 0.012  # eye bar width and height
EYE_W = 0.060  # outer size of the square eye
EYE_IN = EYE_W - 2 * BAR  # 36 mm opening
EYE_PLUG = Plug(size=(BAR, EYE_W, BAR), mass=0.020, color=(0.85, 0.08, 0.06))  # throat bar; the rest added below
EYE_C_OFF = EYE_W / 2 - BAR / 2  # eye centre from the throat-bar centre, along the line (24 mm)
Z_LINE = TOP + R + 0.0005
LINE = cable("dock_line", start=(float(A[0]), float(A[1]), Z_LINE), yaw=LINE_YAW, length=LINE_LEN, radius=R,
             anchor_start=True, end_plug=EYE_PLUG, cable_mass=0.15, color=(0.95, 0.95, 0.93),
             bend_stiffness=0.015, bend_damping=0.0015)
LINE_END = A + LINE_LEN * U0
PLUG0 = LINE_END + BAR / 2 * U0  # throat-bar centre at the start
EYE0 = PLUG0 + EYE_C_OFF * U0  # eye centre at the start
CLEAT_S = (0.030, 0.030, 0.024)
CLEAT_C = A - (0.006 + CLEAT_S[0] / 2) * U0
CLEAT = Box("bow_cleat", pos=(float(CLEAT_C[0]), float(CLEAT_C[1]), TOP + CLEAT_S[2] / 2), size=CLEAT_S,
            yaw=LINE_YAW, color=(0.92, 0.92, 0.90))

# -- messenger: tied to the eye's left side bar, lying straight back along the deck -----------------------------------
MR = 0.003  # 6 mm messenger
MSG_LEN = 0.65
MSG_YAW = math.radians(185.0)
MSG_S = EYE0 + (EYE_C_OFF + BAR / 2 + 0.004) * V0  # 4 mm outside the left side bar's outer face
MSG = cable("messenger", start=(float(MSG_S[0]), float(MSG_S[1]), TOP + MR + 0.0005), yaw=MSG_YAW, length=MSG_LEN,
            radius=MR, cable_mass=0.03, color=(0.95, 0.80, 0.05), bend_stiffness=0.01, bend_damping=0.001)

# -- bollard on the dock -------------------------------------------------------------------------------------------------
POST_XY = np.array([0.38, -0.15])
POST_R, POST_H = 0.010, 0.050
POST = Cylinder("bollard", pos=(float(POST_XY[0]), float(POST_XY[1]), TOP + POST_H / 2), radius=POST_R,
                height=POST_H, color=(0.08, 0.08, 0.09))

OBJECTS = [WATER, CLEAT, POST, LINE, MSG]

TWIN_EXTRA = [
    {"name": "eye_loop", "kind": "body", "size_mm": [60, 60, 12], "colour": "red",
     "pose": [float(EYE0[0]), float(EYE0[1]), 0.006],
     "note": "square eye of 12 mm bars (36 mm opening), printed in one piece; the dock line is knotted through the "
             "throat bar"},
    {"name": "messenger_tie", "kind": "joint", "real": "messenger knotted round the eye's left side bar",
     "note": "messenger start tied to the eye: holds position, free to turn"},
    {"name": "cleat_tie", "kind": "joint", "real": "dock line start knotted through a hole in the cleat block",
     "note": "dock line start fixed at the cleat"},
]

# -- extra eye colliders + messenger tie, authored right after the messenger (the last cable) -------------------------


def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_haul_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if name not in (LINE.name, MSG.name):
            return prim
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        if name == LINE.name:  # the eye: two side bars and the far bar on the throat-bar body
            plug = f"{prim_path}/end_plug"
            for nm, (cx, cy), size in (
                ("eye_side_a", (EYE_C_OFF, EYE_C_OFF), (EYE_W, BAR, BAR)),
                ("eye_side_b", (EYE_C_OFF, -EYE_C_OFF), (EYE_W, BAR, BAR)),
                ("eye_far", (2 * EYE_C_OFF, 0.0), (BAR, EYE_W, BAR)),
            ):
                g = UsdGeom.Cube.Define(stage, f"{plug}/{nm}")
                g.CreateSizeAttr(1.0)
                g.AddTranslateOp().Set(Gf.Vec3d(cx, cy, 0.0))
                g.AddScaleOp().Set(Gf.Vec3f(*size))
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
                cp = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
                cp.CreateContactOffsetAttr(0.002)
                cp.CreateRestOffsetAttr(0.0)
                bind_physics_material(str(g.GetPath()), f"{prim_path}/Looks/cable_physics", stage=stage)
                bind_visual_material(str(g.GetPath()), f"{prim_path}/Looks/end_plug", stage=stage)
            return prim
        # messenger seg_0 tied to the eye (ball joint: position held, rotation free)
        plug = f"{parent}/{LINE.name}/end_plug"
        rel = MSG_S - PLUG0
        p0 = (float(rel @ U0), float(rel @ V0), float(TOP + MR + 0.0005 - Z_LINE))
        j = UsdPhysics.Joint.Define(stage, f"{prim_path}/eye_tie")
        j.CreateBody0Rel().SetTargets([Sdf.Path(plug)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(*p0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-MSG.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        for dof in ("transX", "transY", "transZ"):  # low > high = locked
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)
        j.CreateExcludeFromArticulationAttr(True)
        if os.environ.get("HAUL_DEBUG"):
            print(f"[haul] eye bars + messenger tie {p0} authored", flush=True)
        return prim

    spawn._haul_task = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- env handle (gripper release check) --------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_haul_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._haul_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (yam_arena.runtime.tcp_world, env 0), [] if unknown."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ----------------------------------------------------------------------------------------------------------


def eye_frame(objects):
    """(centre (3,), along-line axis u, across axis v) of the eye, from the last segment and the throat bar."""
    c = objects[LINE.name]
    p = np.asarray(c.end_plug, dtype=float)
    u = p - c.points[-1]
    u[2] = 0.0
    u /= np.linalg.norm(u) + 1e-12
    v = np.array([-u[1], u[0], 0.0])
    return p + u * EYE_C_OFF, u, v


def eye_bars(objects) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    ctr, u, v = eye_frame(objects)
    return {"throat": (ctr - u * EYE_C_OFF, v), "far": (ctr + u * EYE_C_OFF, v),
            "side_a": (ctr + v * EYE_C_OFF, u), "side_b": (ctr - v * EYE_C_OFF, u)}


def _eye_points(objects) -> np.ndarray:
    pts = []
    for mid, d in eye_bars(objects).values():
        pts += [mid + d * t for t in np.linspace(-EYE_C_OFF, EYE_C_OFF, 7)]
    return np.array(pts)


def _finite(objects) -> bool:
    return bool(np.isfinite(objects[LINE.name].points).all() and np.isfinite(objects[MSG.name].points).all())


def eye_on_post(objects) -> bool:
    """The bollard's axis passes through the eye's 36 mm opening (top view) and the eye's centre is at least one bar
    (12 mm) below the bollard top."""
    ctr, u, v = eye_frame(objects)
    d = np.r_[POST_XY, 0.0] - ctr
    half = EYE_IN / 2
    return bool(abs(d @ u) < half and abs(d @ v) < half and ctr[2] < TOP + POST_H - BAR)


def eye_low(objects) -> bool:
    ctr, _, _ = eye_frame(objects)
    return bool(ctr[2] < TOP + 0.02)


def eye_on_dock(objects) -> bool:
    ctr, _, _ = eye_frame(objects)
    return bool(ctr[1] < DOCK_EDGE_Y)


def post_dist(objects) -> float:
    ctr, _, _ = eye_frame(objects)
    return float(np.linalg.norm(ctr[:2] - POST_XY))


RELEASE_DIST = 0.035  # every gripper TCP at least this far from the eye bars and the dock line
NEAR_POST = 0.08


def released(objects) -> bool:
    pts = np.vstack([_eye_points(objects), objects[LINE.name].polyline()])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless the states are finite and both grippers are >= 3.5 cm from the eye and the dock line. Then 1.0 if
    the eye is over the bollard (axis inside the 36 mm opening, eye centre >= 12 mm below the bollard top); 0.5 if
    the eye lies on the dock (centre beyond the water tape's dock edge, < 2 cm above the table) within 8 cm of the
    bollard axis; 0.25 if it only lies on the dock; else 0."""
    if not _finite(objects) or not released(objects):
        return 0.0
    if eye_on_post(objects):
        return 1.0
    if not (eye_on_dock(objects) and eye_low(objects)):
        return 0.0
    return 0.5 if post_dist(objects) <= NEAR_POST else 0.25


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[LINE.name].resting(max_speed=0.02)


def distance(objects) -> float:
    """Eye centre's distance to the bollard axis (m, top view)."""
    if not _finite(objects):
        return 3.0
    return post_dist(objects)


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    if not _finite(objects):
        return {"finite": 0}
    ctr, _, _ = eye_frame(objects)
    m = objects[MSG.name]
    tail0 = MSG.point_at(MSG_LEN)[:2]
    d0 = float(np.linalg.norm(EYE0 - POST_XY))
    return {
        "messenger_taken": int(float(np.linalg.norm(m.end_tip[:2] - tail0)) > 0.05),
        "messenger_across": int(float(m.end_tip[1]) < GAP_HALF),
        "eye_to_post": round(float(np.clip(1.0 - post_dist(objects) / d0, 0.0, 1.0)), 3),
        "eye_on_dock": int(eye_on_dock(objects)),
        "eye_near_post": int(post_dist(objects) <= NEAR_POST),
        "eye_on_post": int(eye_on_post(objects)),
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner (the closed-loop demo follows the live messenger and eye). The cleat moves
# with the dock line start; the messenger is tied to the eye, so it moves with the dock line.
VARIATION = {
    "cable:dock_line": {"dx": 0.01, "dy": 0.01, "yaw": 0.05, "with": ["bow_cleat", "messenger"]},
    "fixed:bollard": {"dx": 0.01, "dy": 0.01},
}

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------------

PASS_XY = np.array([0.30, -0.005])  # the boat arm lays the messenger end here, across the water
GRAB_XY = np.array([0.40, -0.02])  # the dock arm takes the messenger where it crosses near here ...
HAUL_XY = np.array([0.14, -0.10])  # ... and hauls it to here, hand over hand


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _heading(pts: np.ndarray, i: int) -> float:
    a, b = pts[max(i - 1, 0)], pts[min(i + 1, len(pts) - 1)]
    return math.atan2(b[1] - a[1], b[0] - a[0])


def demo_script(ctx):
    """Boat arm (left) takes the messenger 3 cm from its free end and lays it across the water gap. Dock arm (right)
    hauls it hand over hand: it pinches the messenger where it runs past a grab point near the gap, draws it back
    towards its base, lets go and re-takes it, until the eye has come across onto the dock (closed loop on the live
    messenger and eye). Then it pinches the eye's nearest side bar, carries the eye over the bollard, lowers it to just
    above the bollard top and lets go, so the eye drops over the bollard; both arms move clear."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    mode = os.environ.get("HAUL_DEMO", "correct")
    hi = TOP + 0.07

    def log(tag):
        st = ctx.states()
        ctr, _, _ = eye_frame(st)
        print(f"[demo t={ctx.t:5.1f}] {tag}: eye={np.round(ctr, 3)} tail={np.round(st[MSG.name].end_tip, 3)} "
              f"dock={eye_on_dock(st)} d_post={post_dist(st):.3f} on_post={eye_on_post(st)} rel={released(st)} "
              f"score={score(st):.2f}", flush=True)

    log("start")
    if mode == "nothing":
        yield from L.wait(4.0)
        log("end")
        return
    if mode == "reach":  # both arms stretch for the eye / dock line by hand (no messenger), grab, pull back, let go
        st = ctx.states()
        ctr, _, _ = eye_frame(st)
        lp = st[LINE.name].points
        near = lp[np.argmin(np.linalg.norm(lp[:, :2] - [0.0, -0.305], axis=1))]

        def stretch(base, p):
            d = np.asarray(p[:2]) - np.asarray(base[:2])
            q = np.asarray(base[:2]) + d / np.linalg.norm(d) * min(np.linalg.norm(d), 0.50)
            return (float(q[0]), float(q[1]), TOP + 0.02)

        yield from ctx.both(L.move(stretch(L.base, ctr), yaw=0.0, finger=FINGER_OPEN, duration=2.0),
                            Rt.move(stretch(Rt.base, near), yaw=0.0, finger=FINGER_OPEN, duration=2.0))
        yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(FINGER_CLOSED))
        yield from ctx.both(L.move((0.30, 0.30, hi), yaw=0.0, duration=1.5),
                            Rt.move((0.30, -0.30, hi), yaw=0.0, duration=1.5))
        yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))
        yield from L.wait(1.5)
        log("end")
        return

    # 1) Boat arm takes the messenger 3 cm from its free end and lays it across the water gap.
    m = ctx.states()[MSG.name]
    i = len(m.points) - 2
    p = m.points[i]
    gy = _grip_yaw(_heading(m.points, i))
    yield from L.move((p[0], p[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=0.8)
    yield from L.grip(FINGER_CLOSED, 0.5)
    yield from L.move((p[0], p[1], hi), yaw=gy, duration=0.6)
    yield from L.move((PASS_XY[0], PASS_XY[1] + 0.06, hi), yaw=0.0, duration=2.0)
    yield from L.move((PASS_XY[0], PASS_XY[1], TOP + 0.015), yaw=0.0, duration=1.0)
    yield from L.grip(FINGER_OPEN, 0.4)
    yield from L.move((PASS_XY[0], PASS_XY[1] + 0.03, hi), yaw=0.0, duration=0.6)
    yield from L.move((0.15, 0.40, TOP + 0.15), yaw=0.0, duration=1.0)
    log("messenger passed across")

    # 2) Dock arm hauls hand over hand until the eye is on the dock side, near the grab point.
    for stroke in range(8):
        st = ctx.states()
        ctr, _, _ = eye_frame(st)
        if ctr[1] < 0.0:  # the eye has come across the gap, within the dock arm's reach
            break
        mp = st[MSG.name].points
        ok = np.arange(len(mp)) <= len(mp) - 3  # not the last 4 cm at the eye
        d = np.linalg.norm(mp[:, :2] - GRAB_XY, axis=1) + np.where(ok, 0.0, 9.0)
        i = int(np.argmin(d))
        p = mp[i]
        gy = _grip_yaw(_heading(mp, i))
        yield from Rt.move((p[0], p[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.0)
        yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=0.6)
        yield from Rt.grip(FINGER_CLOSED, 0.4)
        yield from Rt.move((p[0], p[1], TOP + 0.03), yaw=gy, duration=0.4)
        # pull no further than the eye can follow (the dock line is tied at the cleat)
        yield from Rt.move((HAUL_XY[0], HAUL_XY[1], TOP + 0.03), yaw=gy, duration=2.0)
        yield from Rt.grip(FINGER_OPEN, 0.3)
        yield from Rt.move((HAUL_XY[0], HAUL_XY[1], hi), yaw=gy, duration=0.4)
        log(f"stroke {stroke}")

    # 3) Dock arm pinches the eye's side bar nearest to it, carries the eye over the bollard and drops it on.
    st = ctx.states()
    bars = eye_bars(st)
    name = min((n for n in ("side_a", "side_b", "far")), key=lambda n: bars[n][0][1] + 0.3 * bars[n][0][0])
    mid, d = bars[name]
    yw = _grip_yaw(math.atan2(d[1], d[0]))  # fingers close across the bar
    yield from Rt.move((mid[0], mid[1], hi), yaw=yw, finger=FINGER_OPEN, duration=1.2)
    yield from Rt.move((mid[0], mid[1], mid[2] + 0.010), yaw=yw, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((mid[0], mid[1], TOP + 0.09), yaw=yw, duration=0.8)
    log(f"eye in hand ({name})")

    z_drop = TOP + POST_H + 0.008 + BAR / 2 + 0.010
    for z, dur in ((TOP + 0.09, 1.5), (z_drop, 0.8)):
        st = ctx.states()
        ctr, _, _ = eye_frame(st)
        off = ctr[:2] - Rt.tcp()[:2]
        q = POST_XY - off
        yield from Rt.move((q[0], q[1], z), yaw=yw, duration=dur)
    yield from Rt.wait(0.4)
    st = ctx.states()
    ctr, _, _ = eye_frame(st)
    off = ctr[:2] - Rt.tcp()[:2]
    q = POST_XY - off
    if mode == "beside":  # partial-credit check: drop the eye 6 cm off the bollard, on the dock
        q = q + np.array([-0.06, 0.0])
        yield from Rt.move((q[0], q[1], TOP + 0.03), yaw=yw, duration=0.8)
    yield from Rt.move((q[0], q[1], z_drop if mode != "beside" else TOP + 0.03), yaw=yw, duration=0.5)
    log("eye over the bollard (held)")
    if mode == "hold":
        yield from Rt.move((q[0], q[1], TOP + 0.022), yaw=yw, duration=0.8)
        yield from Rt.wait(2.0)
        log("end (held)")
        return
    yield from Rt.grip(FINGER_OPEN, duration=0.5)
    yield from Rt.move((q[0], q[1], TOP + 0.15), yaw=yw, duration=0.8)
    yield from Rt.move((0.20, -0.35, TOP + 0.15), yaw=0.0, duration=1.0)
    yield from Rt.wait(1.5)
    log("end")
