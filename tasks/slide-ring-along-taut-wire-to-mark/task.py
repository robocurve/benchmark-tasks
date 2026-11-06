"""On the station table a 5 mm black steel messenger wire runs straight along y, 30 cm in front of the robot and 9 cm
above the table. Its right end runs through a 12 mm channel in a dark grey anchor post (y -30 to -20 cm) to a spring
tensioner; its left end has a black ferrule block (30 x 20 x 20 mm) that sits in the near notch of a grey two-notch
tension bracket (notches centred at y +12 and y +20 cm). In the near notch the spring barely pulls and the wire sags
about 3 cm; in the far notch it pulls about 1 N and the wire runs nearly straight. An orange square hanger ring
(25 mm opening, 5 mm bars, a 16 mm tall grab plate on top) hangs on the wire at y -4 cm. A red tape mark on the table
under the wire at y -13 cm marks where the ring must hang.

One arm tensions the wire and the other places the ring: the left arm cannot reach the mark and the right arm cannot
reach the bracket.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Tension the black messenger wire by moving its end block into the far notch of the grey bracket, "
               "and leave the orange hanger ring hanging on the wire right above the red mark. Let go of both.")

# -- layout (m; x forward from the robot, y to the robot's left) ------------------------------------------------------
X0 = 0.30  # wire line (runs along y)
H = TOP + 0.09  # wire centre height at both ends
R = 0.0025  # 5 mm wire rope
GREY = (0.58, 0.60, 0.62)
DARK = (0.30, 0.32, 0.34)

A_Y = -0.200  # the wire leaves the anchor post's face here
POST_Y0 = -0.300  # post back face; the wire runs in a 12 x 12 mm channel through the post
WIRE_Y0 = POST_Y0 + 0.006 + 0.006  # wire start, inside the channel, 6 mm in front of the back plate
CH = 0.006  # channel half width / half height
_PW, _PT = 0.04, 0.012  # post width (x) and wall thickness
_py = (POST_Y0 + A_Y) / 2
_pl = A_Y - POST_Y0
ANCHOR = [
    Box("anchor_base", pos=(X0, _py, (TOP + H - CH) / 2), size=(_PW, _pl, H - CH - TOP), color=DARK),
    Box("anchor_cap", pos=(X0, _py, H + CH + _PT / 2), size=(_PW, _pl, _PT), color=DARK),
    Box("anchor_side_a", pos=(X0 - CH - (_PW / 2 - CH) / 2, _py, H), size=(_PW / 2 - CH, _pl, 2 * CH), color=DARK),
    Box("anchor_side_b", pos=(X0 + CH + (_PW / 2 - CH) / 2, _py, H), size=(_PW / 2 - CH, _pl, 2 * CH), color=DARK),
    Box("anchor_back", pos=(X0, POST_Y0 + 0.003, H), size=(2 * CH, 0.006, 2 * CH), color=DARK),
]

FERRULE = Plug(size=(0.03, 0.02, 0.02), mass=0.005, color=(0.08, 0.08, 0.08))
N1_Y, N2_Y = 0.12, 0.20  # notch centres (ferrule centre when seated)
WIRE_LEN = N1_Y - FERRULE.size[0] / 2 - WIRE_Y0  # ferrule's anchor-side face at the near notch's slit wall gap
WIRE = cable("wire", start=(X0, WIRE_Y0, H), yaw=math.pi / 2, length=WIRE_LEN, radius=R, end_plug=FERRULE,
             cable_mass=0.010, color=(0.06, 0.06, 0.07), bend_stiffness=0.01, bend_damping=0.001)

# spring tensioner in the anchor post (prismatic joint on the wire's first segment, see _install_spawn)
TENS_K = 10.0  # N/m
TENS_PRELOAD = 0.020  # m -> 0.2 N at the stop, ~1.0 N with 8 cm drawn out
TENS_PAYOUT = 0.10  # hard stop

# two-notch tension bracket: a block whose top is the floor, slit cross walls, low side lips
FLOOR_Z = H - FERRULE.size[2] / 2
BR_Y0, BR_Y1 = 0.090, 0.238
BRACKET = Box("bracket_block", pos=(X0, (BR_Y0 + BR_Y1) / 2, TOP + (FLOOR_Z - TOP) / 2),
              size=(0.06, BR_Y1 - BR_Y0, FLOOR_Z - TOP), color=GREY)
WALL_T, WALL_H, SLIT = 0.006, 0.015, 0.009
W1_Y = N1_Y - FERRULE.size[0] / 2 - 0.003 - WALL_T / 2
W2_Y = N2_Y - FERRULE.size[0] / 2 - 0.003 - WALL_T / 2
WEND_Y = N2_Y + FERRULE.size[0] / 2 + 0.011 + WALL_T / 2  # 14 mm of play: the ferrule drops in clear of both walls


def _wall(name, y, x0, x1):
    return Box(name, pos=(X0 + (x0 + x1) / 2, y, FLOOR_Z + WALL_H / 2), size=(x1 - x0, WALL_T, WALL_H), color=GREY)


WALLS = [_wall("slit_wall_1a", W1_Y, -0.03, -SLIT / 2), _wall("slit_wall_1b", W1_Y, SLIT / 2, 0.03),
         _wall("slit_wall_2a", W2_Y, -0.03, -SLIT / 2), _wall("slit_wall_2b", W2_Y, SLIT / 2, 0.03),
         _wall("end_wall", WEND_Y, -0.03, 0.03)]
LIP_H = 0.004
LIPS = [Box(f"side_lip_{k}", pos=(X0 + s * 0.027, (BR_Y0 + BR_Y1) / 2, FLOOR_Z + LIP_H / 2),
            size=(0.006, BR_Y1 - BR_Y0, LIP_H), color=GREY) for k, s in (("a", -1), ("b", 1))]

# hanger ring: four free bars welded into one square loop (spawn hook); the top bar is the ring's reference body
RING_OPEN = 0.025
BAR = 0.005
RING_T = 0.008  # thickness along the wire
RING_Y0 = -0.04
RING_TOP_Z = H + R + 0.0005 + BAR / 2  # top bar centre, its underside just above the wire
RING_DROP = BAR / 2 + RING_OPEN / 2  # top bar centre -> ring centre
RING_W = RING_OPEN + 2 * BAR
ORANGE = (1.0, 0.45, 0.0)
RING = Box("ring", pos=(X0, RING_Y0, RING_TOP_Z), size=(RING_W, RING_T, BAR), mass=0.001, color=ORANGE)
RING_REL = {"ring_side_a": (-(RING_OPEN + BAR) / 2, 0.0, -RING_DROP), "ring_side_b": ((RING_OPEN + BAR) / 2, 0.0, -RING_DROP),
            "ring_bottom": (0.0, 0.0, -2 * RING_DROP), "ring_tab": (0.0, 0.0, BAR / 2 + 0.008)}
RING_SIZE = {"ring_side_a": (BAR, RING_T, RING_OPEN), "ring_side_b": (BAR, RING_T, RING_OPEN),
             "ring_bottom": (RING_W, RING_T, BAR), "ring_tab": (RING_W, RING_T, 0.016)}
RING_PARTS = [Box(n, pos=(X0 + d[0], RING_Y0 + d[1], RING_TOP_Z + d[2]), size=RING_SIZE[n], mass=0.001, color=ORANGE)
              for n, d in RING_REL.items()]

MARK_Y = -0.13
MARK = Box("red_mark", pos=(X0, MARK_Y, TOP + 0.0005), size=(0.06, 0.008, 0.001), color=(0.85, 0.05, 0.05))

OBJECTS = [*ANCHOR, BRACKET, *WALLS, *LIPS, MARK, RING, *RING_PARTS, WIRE]

TWIN_EXTRA = [
    {"name": "wire_tensioner", "kind": "joint", "real": "extension spring (10 N/m) + 10 cm slide in the anchor post",
     "note": "wire end slides along the wire line; 0.2 N at the stop, ~1.0 N with 8 cm drawn out"},
    {"name": "ring_welds", "kind": "joint", "real": "one-piece printed square ring", "note": "four bars form one body"},
]

# the four ring bars are one welded body: apply the same dy to all four
VARIATION = {f"obj:{n}": {"dx": 0.0, "dy": 0.02, "yaw": 0.0} for n in ("ring", *RING_REL)}


# -- spawn hook: tensioner joint on the wire start, ring bars --------------------------------------------------------
def _install_spawn() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_ring_task", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != WIRE.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        seg = f"{prim_path}/seg_0"
        world = UsdGeom.Xformable(stage.GetPrimAtPath(seg)).ComputeLocalToWorldTransform(0)
        q = world.ExtractRotationQuat()
        tip = world.Transform(Gf.Vec3d(-WIRE.seg_len / 2, 0.0, 0.0))
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/tensioner")
        j.CreateBody1Rel().SetTargets([Sdf.Path(seg)])
        j.CreateAxisAttr("X")
        j.CreateLocalPos0Attr(Gf.Vec3f(*tip))
        j.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), *q.GetImaginary()))
        j.CreateLocalPos1Attr(Gf.Vec3f(-WIRE.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLowerLimitAttr(0.0)
        j.CreateUpperLimitAttr(TENS_PAYOUT)
        j.CreateExcludeFromArticulationAttr(True)
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(TENS_K)
        drv.CreateDampingAttr(0.5)
        drv.CreateTargetPositionAttr(-TENS_PRELOAD)
        # ring: weld the side and bottom bars to the top bar
        root = str(prim_path).rsplit("/", 1)[0]
        ring = f"{root}/{RING.name}"
        for name, rel in RING_REL.items():
            w = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld_{name}")
            w.CreateBody0Rel().SetTargets([Sdf.Path(ring)])
            w.CreateBody1Rel().SetTargets([Sdf.Path(f"{root}/{name}")])
            w.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rel)))
            w.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
            w.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            w.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
            w.CreateExcludeFromArticulationAttr(True)
        parts = [ring] + [f"{root}/{n}" for n in RING_REL]
        for pth in parts:
            fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(pth))
            fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(q) for q in parts if q != pth])
        if os.environ.get("RING_DEBUG"):
            print(f"[ring] tensioner at {tuple(round(v, 3) for v in tip)}; ring bars under {ring}", flush=True)
        return prim

    spawn._ring_task = True
    cablekit._spawn_cable = spawn


_install_spawn()

# -- env handle (release check) -------------------------------------------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_ring_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._ring_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (env 0), [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


# -- geometry ---------------------------------------------------------------------------------------------------------
SAG_MAX = 0.010  # wire sag allowed between the anchor post and the bracket
NEAR, FAR = 0.010, 0.025  # ring centre within 1 cm / 2.5 cm (along the wire) of the mark
RELEASE = 0.03  # both grippers at least 3 cm from the ferrule, the ring (top bar and centre) and the wire


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(200, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def ring_centre(objects) -> np.ndarray:
    r = objects["ring"]
    return np.asarray(r.pos, dtype=float) + _rot(r.quat_xyzw) @ np.array([0.0, 0.0, -RING_DROP])


def _finite(objects) -> bool:
    w, r = objects["wire"], objects["ring"]
    return bool(np.isfinite(w.points).all() and np.isfinite(w.end_plug).all() and np.isfinite(r.pos).all()
                and np.isfinite(r.quat_xyzw).all())


def sag(objects) -> float:
    """Lowest drop (m) of the wire's centre line below the straight line between its two supports (height H at the
    anchor post and at the bracket's first slit wall)."""
    p = _dense(objects["wire"].polyline())
    m = (p[:, 1] > A_Y + 0.005) & (p[:, 1] < W1_Y - WALL_T / 2)
    return float(H - p[m, 2].min()) if m.any() else 1.0


def seated_far(objects) -> bool:
    """Ferrule resting in the far notch (centre within its notch, on the floor)."""
    f = objects["wire"].end_plug
    return bool(W2_Y + WALL_T / 2 <= f[1] <= WEND_Y - WALL_T / 2 and abs(f[0] - X0) <= 0.02
                and abs(f[2] - H) <= 0.005)


def ring_on_wire(objects) -> bool:
    """The wire passes through the ring's opening (ring centre within 2 cm of the wire, ring upright-ish)."""
    c = ring_centre(objects)
    p = _dense(objects["wire"].polyline())
    up = _rot(objects["ring"].quat_xyzw)[:, 2]
    return bool(np.linalg.norm(p - c, axis=1).min() <= 0.02 and up[2] >= math.cos(math.radians(45)))


def ring_offset(objects) -> float:
    return float(abs(ring_centre(objects)[1] - MARK_Y))


def released(objects) -> bool:
    f = objects["wire"].end_plug
    r = np.asarray(objects["ring"].pos, dtype=float)
    c = ring_centre(objects)
    w = _dense(objects["wire"].polyline())
    return all(min(np.linalg.norm(g - f), np.linalg.norm(g - r), np.linalg.norm(g - c),
                   float(np.linalg.norm(w - g, axis=1).min())) >= RELEASE for g in grippers())


def score(objects) -> float:
    """0 if the ferrule is not resting in the far notch, the wire sags more than 10 mm anywhere between the anchor
    post and the bracket, the ring is off the wire, or a gripper is within 3 cm of the ferrule, the ring or the wire.
    Otherwise
    by the ring centre's distance along the wire from the red mark: <= 10 mm 1.0; <= 25 mm 0.5; farther 0."""
    if not _finite(objects):
        return 0.0
    if not (seated_far(objects) and sag(objects) <= SAG_MAX and ring_on_wire(objects) and released(objects)):
        return 0.0
    d = ring_offset(objects)
    return 1.0 if d <= NEAR else 0.5 if d <= FAR else 0.0


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and objects["ring"].resting(0.01, 0.5))


def distance(objects) -> float:
    if not _finite(objects):
        return 1.0
    return float(ring_offset(objects) + max(0.0, sag(objects) - SAG_MAX) + (0.0 if seated_far(objects) else 0.08))


def progress(objects) -> dict:
    if not _finite(objects):
        return {"tensioned": 0, "ring_near": 0, "ring_at_mark": 0, "released": 0}
    t = seated_far(objects) and sag(objects) <= SAG_MAX
    d = ring_offset(objects)
    return {"tensioned": int(t), "ring_near": int(d <= FAR), "ring_at_mark": int(d <= NEAR),
            "released": int(released(objects))}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
GRASP_OPEN = -0.016  # pads ~32 mm apart
L_YAW = math.pi / 2  # left wrist camera faces the far notch the ferrule is carried to
R_YAW = 0.0  # right arm pinches the ring's grab tab across the wire direction
TAB_Z = BAR / 2 + 0.008  # tab centre above the top bar centre
RELEASE_OPEN = -0.030  # pads ~60 mm apart when letting go of the ring



def _log(ctx, tag=""):
    st = ctx.states()
    if not _finite(st):
        print(f"[t={ctx.t:5.2f}] {tag}: NON-FINITE STATE", flush=True)
        return
    w = st["wire"]
    print(f"[t={ctx.t:5.2f}] {tag}: ferrule={np.round(w.end_plug, 3)} start_tip_y={w.start_tip[1]:.3f} "
          f"sag={sag(st) * 1000:.1f}mm ring_c={np.round(ring_centre(st), 3)} off={ring_offset(st) * 1000:.1f}mm "
          f"on={ring_on_wire(st)} seated={seated_far(st)} rel={released(st)} score={score(st):.2f}", flush=True)
    if os.environ.get("RING_DEBUG"):
        print(f"   ring pos={np.round(st['ring'].pos, 4)} q={np.round(st['ring'].quat_xyzw, 3)} "
              f"tcp={[np.round(g, 3) for g in grippers()]}", flush=True)


def _take_ferrule(ctx):
    """Left arm: open above the ferrule and pinch it."""
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    f = np.asarray(ctx.states()["wire"].end_plug, dtype=float)
    yield from L.move((f[0], f[1], f[2] + 0.07), yaw=L_YAW, finger=GRASP_OPEN, duration=1.5)
    f = np.asarray(ctx.states()["wire"].end_plug, dtype=float)
    yield from L.move((f[0], f[1], f[2] + 0.010), yaw=L_YAW, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)


def _draw_ferrule(ctx):
    """Left arm: lift the ferrule over the slit wall, draw it past the far slit wall (closed loop on the ferrule,
    which lags the fingers under the spring's pull) and lower it into the far notch."""
    L = ctx.left
    f = np.asarray(ctx.states()["wire"].end_plug, dtype=float)
    lift = H + 0.010 + 0.028
    yield from L.move((X0, f[1], lift), yaw=L_YAW, duration=0.8)
    ty = N2_Y + 0.008
    yield from L.move((X0, ty, lift), yaw=L_YAW, duration=2.5)
    for _ in range(2):
        fy = float(ctx.states()["wire"].end_plug[1])
        if fy >= N2_Y + 0.002:
            break
        ty = min(ty + (N2_Y + 0.004 - fy), WEND_Y - 0.012)
        yield from L.move((X0, ty, lift), yaw=L_YAW, duration=0.6)
    _log(ctx, "ferrule over far notch")
    yield from L.move((X0, ty, H + 0.010 + 0.002), yaw=L_YAW, duration=1.0)
    if os.environ.get("RING_DEMO") == "holdferrule":
        _log(ctx, "ferrule held in far notch")
        return
    yield from L.grip(GRASP_OPEN, 0.5)
    yield from L.move((X0, ty, H + 0.10), yaw=L_YAW, duration=0.8)
    _log(ctx, "ferrule seated")


def _pinch_wire(ctx):
    """Left arm: close on the taut wire beside the bracket and stay (holdwire variant)."""
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    yield from L.move((X0, 0.06, H + 0.05), yaw=L_YAW, duration=1.0)
    p = ctx.states()["wire"].polyline()
    wz = float(p[np.argmin(np.abs(p[:, 1] - 0.06)), 2])
    yield from L.move((X0, 0.06, wz + 0.010), yaw=L_YAW, duration=0.8)
    yield from L.grip(FINGER_CLOSED, 0.6)
    _log(ctx, "left arm on the wire")


def _take_ring(ctx):
    """Right arm: open above the ring's grab plate and pinch it."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    r = np.asarray(ctx.states()["ring"].pos, dtype=float)
    yield from Rt.move((r[0], r[1], r[2] + TAB_Z + 0.07), yaw=R_YAW, finger=GRASP_OPEN, duration=1.5)
    r = np.asarray(ctx.states()["ring"].pos, dtype=float)
    yield from Rt.move((X0, r[1], r[2] + TAB_Z + 0.010), yaw=R_YAW, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)


def _slide_ring(ctx, goal_y=MARK_Y):
    """Right arm, ring pinched: slide it along the wire to goal_y, closed loop on the ring centre."""
    Rt = ctx.right
    r = np.asarray(ctx.states()["ring"].pos, dtype=float)
    py, rz = float(r[1]), float(r[2]) + TAB_Z + 0.010
    for k in range(4):
        err = goal_y - ring_centre(ctx.states())[1]
        if k and abs(err) < 0.003:
            break
        py += err
        yield from Rt.move((X0, py, rz), yaw=R_YAW, duration=4.0 if k == 0 else 0.8)
        yield from Rt.wait(0.4)
        _log(ctx, f"slide {k}")


def _drop_ring(ctx):
    """Right arm: open wide, let the ring settle on the wire, back straight up."""
    Rt = ctx.right
    p = np.asarray(ctx.states()["ring"].pos, dtype=float)
    z = float(p[2]) + TAB_Z + 0.010
    yield from Rt.grip(RELEASE_OPEN, 0.8)
    yield from Rt.wait(0.5)
    _log(ctx, "ring let go")
    yield from Rt.move((X0, p[1], z + 0.02), yaw=R_YAW, duration=1.0)
    yield from Rt.move((X0, p[1], z + 0.08), yaw=R_YAW, duration=0.8)


def _clear(ctx):
    yield from ctx.both(ctx.left.move((0.20, 0.25, TOP + 0.18), yaw=0.0, duration=1.0),
                        ctx.right.move((0.20, -0.25, TOP + 0.18), yaw=0.0, duration=1.0))
    _log(ctx, "released")
    yield from ctx.left.wait(3.0)
    _log(ctx, "end")


def demo_script(ctx):
    """RING_DEMO=correct (default); shortcut and diagnostic variants: nothing | slack (slide the ring to the mark,
    never tension) | hold (correct, but the right arm keeps pinching the ring at the mark) | holdferrule (correct, then
    the left arm pinches the seated ferrule again and stays) | holdwire (correct, but the left arm lets go of the
    ferrule and keeps pinching the wire) | tension (tension only, ring never touched) | ringfirst (ring to the mark
    first and let go, then tension with the ring unheld) | tier05 (correct, but the ring is left 18 mm from the mark:
    partial-credit check, expect 0.5)."""
    mode = os.environ.get("RING_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    yield from L.wait(1.0)
    _log(ctx, "settled")
    if mode == "nothing":
        yield from L.wait(4.0)
        _log(ctx, "end")
        return
    if mode == "slack":
        yield from _take_ring(ctx)
        yield from _slide_ring(ctx)
        yield from _drop_ring(ctx)
        yield from _clear(ctx)
        return
    if mode == "tension":
        yield from _take_ferrule(ctx)
        yield from _draw_ferrule(ctx)
        yield from _clear(ctx)
        return
    if mode == "ringfirst":
        yield from _take_ring(ctx)
        yield from _slide_ring(ctx)
        yield from _drop_ring(ctx)
        yield from _take_ferrule(ctx)
        yield from _draw_ferrule(ctx)
        yield from _clear(ctx)
        return
    # 1) right arm pinches the ring's grab plate and holds it; left arm takes the ferrule
    yield from ctx.both(_take_ring(ctx), _take_ferrule(ctx))
    _log(ctx, "gripped")
    # 2) left arm draws the ferrule into the far notch while the ring is held
    yield from _draw_ferrule(ctx)
    # 3) the held ring kept the wire pressed down: let go so the ring rides up with the taut wire, re-pinch it where
    #    it now hangs, then slide it along the wire to the mark
    r = np.asarray(ctx.states()["ring"].pos, dtype=float)
    yield from Rt.grip(GRASP_OPEN, 0.5)
    yield from Rt.move((X0, r[1], r[2] + TAB_Z + 0.03), yaw=R_YAW, duration=0.6)
    yield from Rt.wait(0.8)
    yield from _take_ring(ctx)
    _log(ctx, "ring re-pinched")
    yield from _slide_ring(ctx, MARK_Y + 0.018 if mode == "tier05" else MARK_Y)
    if mode == "holdwire":
        yield from _pinch_wire(ctx)
    if mode == "hold":
        yield from Rt.wait(3.0)
        _log(ctx, "end")
        return
    yield from _drop_ring(ctx)
    if mode in ("holdferrule", "holdwire"):
        yield from Rt.move((0.20, -0.25, TOP + 0.18), yaw=0.0, duration=1.0)
        yield from L.wait(3.0)
        _log(ctx, "end")
        return
    yield from _clear(ctx)
