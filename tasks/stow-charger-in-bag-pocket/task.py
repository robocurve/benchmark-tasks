"""Travel-packing mock-up on the station table: a flat navy travel bag (a fixed block 26 x 58 x 5 cm) lies in front
of the robot. Its top has a pocket: a 5 x 20 cm opening, 5 cm deep (down to the table), closed by a stiff blue flap
(7 x 22 cm, 4 mm thick, 40 g) hinged along the pocket's far edge. The hinge stops at 80 degrees, so the flap always
falls shut under its own weight unless it is held up by its pull knob (an 18 mm grey ball on the flap's front edge,
9 cm left of the robot's midline). A white phone charger lies on the bag top in front of the pocket, straight along
y on the right-hand side: a 6 mm white cable, 30 cm long, with a white charger brick (36 x 26 x 16 mm) on its left
end and a grey USB-C plug (22 x 12 x 7 mm) on its right end. The robot must hold the flap open, put the whole charger
(brick, cable and plug) into the pocket, let the flap fall shut over it and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants: STOW_DEMO=correct (default) | nothing | brick_only | hold | onearm_push | prop | drape.
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
from yam_arena.task_file import TABLE_TOP, Box, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Pack the white phone charger into the bag's top pocket: hold the pocket flap open by its grey knob, "
               "put the brick, the whole cable and the plug inside, then let the flap fall shut over it and let go "
               "of everything.")

NAVY = (0.10, 0.14, 0.30)
FLAP_NAVY = (0.30, 0.40, 0.62)
WHITE = (0.92, 0.92, 0.90)
GREY = (0.45, 0.46, 0.48)

# -- bag (fixed): a 5 cm thick block with a pocket cavity in its top ------------------------------------------------
BX0, BX1 = 0.14, 0.40
BY0, BY1 = -0.43, 0.15
BH = 0.050
BT = TOP + BH  # bag top
PX0, PX1 = 0.25, 0.30  # pocket opening (x), 5 cm
PY0, PY1 = -0.10, 0.10  # pocket opening (y), 20 cm


def _bag() -> list[Box]:
    zc = TOP + BH / 2

    def blk(name, x0, x1, y0, y1):
        return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, zc), size=(x1 - x0, y1 - y0, BH), color=NAVY)

    return [
        blk("bag_front", BX0, PX0, BY0, BY1),
        blk("bag_back", PX1, BX1, BY0, BY1),
        blk("bag_side_r", PX0, PX1, BY0, PY0),
        blk("bag_side_l", PX0, PX1, PY1, BY1),
    ]


# -- flap (free, hinged to the world along the pocket's far edge) and its pull knob (welded) ---------------------
FL_X, FL_Y, FL_T = 0.070, 0.220, 0.004
HINGE_X = PX1 + 0.005  # 0.305
FLAP_Z0 = BT + FL_T / 2 + 0.0003
FLAP = Box("pocket_flap", pos=(HINGE_X - FL_X / 2, 0.0, FLAP_Z0), size=(FL_X, FL_Y, FL_T), mass=0.04,
           color=FLAP_NAVY)
KNOB_R = 0.009
KNOB_P0 = np.array([HINGE_X - FL_X - 0.006, 0.09, BT + 0.012])  # in front of the flap's front edge
KNOB = Sphere("flap_knob", pos=tuple(KNOB_P0.tolist()), radius=KNOB_R, mass=0.005, color=GREY)
HINGE_MAX = 80.0  # deg: hinge stop; the flap falls shut from any angle

# -- charger: free cable, USB-C plug on the start (right), brick on the end (left) ---------------------------------
R = 0.003
CORD_LEN = 0.30
BRICK = (0.036, 0.026, 0.016)
USBC = (0.022, 0.012, 0.007)
CORD_X = 0.175
CORD_Y0 = -0.400  # start (USB-C) end; the cable runs to the left (+y)
CORD = cable("charger", start=(CORD_X, CORD_Y0, BT + BRICK[2] / 2 + 0.0005), yaw=math.pi / 2, length=CORD_LEN,
             radius=R, cable_mass=0.032,
             start_plug=Plug(size=USBC, mass=0.004, color=GREY),
             end_plug=Plug(size=BRICK, mass=0.02, color=WHITE),
             bend_stiffness=0.01, bend_damping=0.001, color=WHITE)

OBJECTS = [*_bag(), FLAP, KNOB, CORD]


# -- hinge + knob weld, authored right after the cable (the kit's only spawn hook) --------------------------------

def _install_joints() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_stow_joints", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        stage = get_current_stage()
        parent = str(prim_path).rsplit("/", 1)[0]
        flap = f"{parent}/{FLAP.name}"
        w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
        h = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/flap_hinge")
        h.CreateBody1Rel().SetTargets([Sdf.Path(flap)])
        h.CreateAxisAttr("Y")
        h.CreateLocalPos0Attr(Gf.Vec3f(float(w[0] + HINGE_X), float(w[1]), float(w[2] + FLAP_Z0)))
        h.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        h.CreateLocalPos1Attr(Gf.Vec3f(float(FL_X / 2), 0.0, 0.0))
        h.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        h.CreateLowerLimitAttr(-3.0)
        h.CreateUpperLimitAttr(HINGE_MAX)
        h.CreateExcludeFromArticulationAttr(True)
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/knob_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(flap)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{KNOB.name}")])
        rel = KNOB_P0 - np.array(FLAP.pos)
        j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in rel)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._stow_joints = True
    cablekit._spawn_cable = spawn


_install_joints()

# -- grippers (score() needs them for the release check) ----------------------------------------------------------
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (no embodiment: offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_stow_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._stow_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (13 mm above the finger pad tips); [] if unknown (offline)."""
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


# -- scoring --------------------------------------------------------------------------------------------------------
RELEASE_DIST = 0.035  # both TCPs at least 3.5 cm from the charger, the flap and its knob
CLOSED_DEG = 3.0  # flap shut: within 3 degrees of flat (front edge within ~4 mm of the bag top)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def flap_deg(objects) -> float:
    """Flap opening angle about its hinge (deg, 0 = shut)."""
    st = objects[FLAP.name]
    if not np.isfinite(st.quat_xyzw).all():
        return 90.0
    Rm = _rot(st.quat_xyzw)
    front = Rm @ np.array([-1.0, 0.0, 0.0])
    return float(math.degrees(math.atan2(front[2], -front[0])))


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    return np.asarray(out)


def _in_pocket(p, top_z, margin=0.0) -> np.ndarray:
    p = np.atleast_2d(p)
    return ((p[:, 0] >= PX0 + margin) & (p[:, 0] <= PX1 - margin) & (p[:, 1] >= PY0 + margin)
            & (p[:, 1] <= PY1 - margin) & (p[:, 2] <= top_z))


def charger_points(objects) -> np.ndarray:
    """Cable centre line (densified) plus both plug centres."""
    c = objects[CORD.name]
    pts = [_dense(c.polyline())]
    for p in (c.start_plug, c.end_plug):
        if p is not None:
            pts.append(np.asarray(p)[None])
    return np.vstack(pts)


def fraction_inside(objects, top_z=BT - R) -> float:
    pts = charger_points(objects)
    return float(_in_pocket(pts, top_z).mean())


def released(objects) -> bool:
    pts = charger_points(objects)
    f, k = objects[FLAP.name], objects[KNOB.name]
    for g in grippers():
        if float(np.min(np.linalg.norm(pts - g, axis=1))) < RELEASE_DIST:
            return False
        if float(np.linalg.norm(np.maximum(np.maximum(f.low - g, g - f.high), 0.0))) < RELEASE_DIST:
            return False
        if float(np.linalg.norm(k.pos - g)) < RELEASE_DIST:
            return False
    return True


def score(objects) -> float:
    """1.0 if, at the end, both grippers are at least 3.5 cm from the charger, flap and knob, the flap is shut (within
    3 deg of flat), and every point of the cable centre line and both plug centres lie inside the pocket opening's
    footprint and at least 3 mm below the bag top; otherwise 0. (No partial rung: with the pocket 5 cm deep, anything
    that keeps the flap from shutting must stick out of the opening or lie outside it.)"""
    pts = charger_points(objects)
    if not (np.isfinite(pts).all() and np.isfinite(objects[FLAP.name].pos).all()):
        return 0.0
    if not released(objects):
        return 0.0
    if flap_deg(objects) > CLOSED_DEG or not _in_pocket(pts, BT - R).all():
        return 0.0
    return 1.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and objects[FLAP.name].resting()
                and float(np.linalg.norm(objects[CORD.name].lin_vel)) < 0.02)


def distance(objects) -> float:
    """Fraction of the charger still outside the pocket (0..1)."""
    return 1.0 - fraction_inside(objects)


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    c = objects[CORD.name]
    brick = c.end_plug
    return {
        "brick_in_pocket": float(brick is not None and bool(_in_pocket(np.asarray(brick), BT).all())),
        "charger_fraction_in": round(fraction_inside(objects), 3),
        "flap_shut": float(flap_deg(objects) <= CLOSED_DEG),
        "released": float(released(objects)),
    }


# Seeded variation for a later paper runner only; official trials use object_jitter=0 and never jitter the cable.
VARIATION = {
    "cable:charger": {"dx": 0.01, "dy": 0.01, "yaw": 0.05},
}

# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# Left arm: pinch the flap's knob, swing it up along the hinge arc to 70 deg and hold it there.
# Right arm (meanwhile): pick the brick up across its width; once the flap is open, lower the brick into the right
# half of the opening and let go. Take the USB-C plug, carry it low to above the opening and raise it as high as the
# arm reaches so the cable hangs into the pocket from the plug; lower it to the pocket floor, let go, press down
# anything sticking up, move away.
# Left arm: lower the flap back along its arc to 8 deg, open the fingers (the flap falls shut), move away.

FINGER_PINCH = -0.020
FINGER_CORD = -0.014
HINGE = np.array([HINGE_X, FLAP_Z0])


def _knob_at(deg: float) -> np.ndarray:
    rel = np.array([KNOB_P0[0] - HINGE[0], KNOB_P0[2] - HINGE[1]])
    r = float(np.linalg.norm(rel))
    a0 = math.atan2(rel[1], -rel[0])
    a = a0 + math.radians(deg)
    return np.array([HINGE[0] - r * math.cos(a), KNOB_P0[1], HINGE[1] + r * math.sin(a)])


def _log(ctx, tag=""):
    st = ctx.states()
    c = st[CORD.name]
    print(f"[t={ctx.t:5.2f}] {tag} flap={flap_deg(st):5.1f}deg inside={fraction_inside(st):.2f} "
          f"brick={np.round(c.end_plug, 3)} usbc={np.round(c.start_plug, 3)} zmax={charger_points(st)[:, 2].max() - BT:+.3f} "
          f"rel={released(st)} score={score(st)} prog={progress(st)}", flush=True)
    print(f"    knob={np.round(st[KNOB.name].pos, 4)} flap_pos={np.round(st[FLAP.name].pos, 4)} grippers={[np.round(g, 3) for g in grippers()]}", flush=True)


class _Flags:
    open = False
    stowed = False


_GRASP = {"off": np.array([0.0, 0.0, 0.010])}
OPEN_DEG = 70.0


def _open_flap(ctx, flags, deg_open=OPEN_DEG):
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    k = ctx.states()[KNOB.name].pos
    yield from L.move((k[0], k[1], k[2] + 0.05), yaw=0.0, finger=FINGER_PINCH, duration=1.5)
    yield from L.move((k[0], k[1], k[2] + 0.010), yaw=0.0, duration=1.0)
    yield from L.wait(0.3)
    yield from L.grip(FINGER_CLOSED, 0.6)
    _GRASP["off"] = np.array([k[0], k[1], k[2] + 0.010]) - ctx.states()[KNOB.name].pos  # commanded TCP - knob
    _log(ctx, "knob pinched")
    for d in np.linspace(6.0, deg_open, 13):
        yield from L.move(_knob_at(float(d)) + _GRASP["off"], yaw=0.0, duration=0.4)
    yield from L.wait(0.4)
    _log(ctx, "flap open")
    flags.open = True


def _close_flap(ctx, deg_open=OPEN_DEG):
    from yam_arena.constants import FINGER_CLOSED

    L = ctx.left
    for d in np.linspace(deg_open, 8.0, 10):
        yield from L.move(_knob_at(float(d)) + _GRASP["off"], yaw=0.0, duration=0.4)
    yield from L.grip(FINGER_PINCH, 0.4)
    p = _knob_at(8.0)
    yield from L.move((p[0] - 0.03, p[1] + 0.02, p[2] + 0.08), yaw=0.0, duration=1.0)
    yield from L.wait(0.8)
    yield from L.move((0.16, 0.22, BT + 0.15), yaw=0.0, finger=FINGER_CLOSED, duration=1.0)
    _log(ctx, "flap closed")


DROP_X = 0.270  # where the right hand lowers things into the opening (x)
BRICK_Y = -0.060  # the brick goes into the right half of the pocket
HANG_Y = -0.075  # the cable is hung and lowered in here
HI_Z = TOP + 0.19  # highest the right arm reaches over the opening
RYAW = math.pi / 2  # right grip across a cable along y (the wrist limit forbids -pi/2): wrist camera +y, over the opening
Y_MAX_RIGHT = -0.025  # the right hand stays right of this while the left arm holds the flap (wrist camera clearance)


def _grip_yaw(d) -> float:
    """Grip yaw across a part heading ``d`` (closing across it), in the right wrist's range."""
    a = math.atan2(d[1], d[0])
    while a > math.pi / 2:
        a -= math.pi
    while a < -math.pi / 2:
        a += math.pi
    return a + math.pi if a < -1.4 else a  # same grip turned half a turn: the right wrist reaches ~+1.75 rad


def _brick_in(ctx):
    """Pick the brick up across its width (it lies on the bag top at the charger's left end, nearest the midline)."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    c = ctx.states()[CORD.name]
    b = c.end_plug
    by = _grip_yaw(c.end_dir)
    print(f"[demo] brick at {np.round(b, 3)} grip yaw {by:.2f}", flush=True)
    yield from Rt.move((b[0], b[1], max(b[2], BT) + 0.06), yaw=by, finger=FINGER_PINCH, duration=1.5)
    yield from Rt.move((b[0], b[1], b[2] + 0.004), yaw=by, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((b[0], b[1], BT + 0.06), yaw=by, duration=0.8)


def _brick_down(ctx):
    """Carry the brick over the right end of the opening, lower it in and let go."""
    Rt = ctx.right
    yield from Rt.move((DROP_X, BRICK_Y, BT + 0.06), yaw=RYAW, duration=1.5)
    yield from Rt.move((DROP_X, BRICK_Y, BT - 0.008), yaw=RYAW, duration=1.0)
    yield from Rt.wait(0.3)
    yield from Rt.grip(FINGER_PINCH, 0.4)
    yield from Rt.move((DROP_X, BRICK_Y, BT + 0.06), yaw=RYAW, duration=0.7)
    _log(ctx, "brick in")


def _plug_over(ctx):
    """Take the USB-C plug where it lies, carry it low to just above the opening, then raise it as high as the arm
    reaches: the cable now hangs from the plug down into the pocket, where the brick holds its other end."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    c = ctx.states()[CORD.name]
    u = c.start_plug
    uy = _grip_yaw(c.start_dir)
    print(f"[demo] USB-C plug at {np.round(u, 3)} grip yaw {uy:.2f}", flush=True)
    yield from Rt.move((u[0], u[1], max(u[2], BT) + 0.05), yaw=uy, finger=FINGER_CORD, duration=1.2)
    yield from Rt.move((u[0], u[1], u[2] + 0.006), yaw=uy, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((u[0], u[1], u[2] + 0.02), yaw=uy, duration=0.4)
    yield from Rt.move((DROP_X, HANG_Y, BT + 0.06), yaw=RYAW, duration=2.0)
    yield from Rt.move((DROP_X, HANG_Y, HI_Z), yaw=RYAW, duration=1.5)
    yield from Rt.wait(1.0)
    _log(ctx, "cable hanging")


def _stow(ctx, keep_hold=False):
    """Brick in first; then hang the cable from its USB-C plug over the opening and lower it in, plug last; press
    down anything sticking up out of the opening. ``keep_hold`` (cheat): never let go of the plug."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    yield from _brick_down(ctx)
    yield from _plug_over(ctx)
    yield from Rt.move((DROP_X, HANG_Y, BT - 0.010), yaw=RYAW, duration=2.5)
    yield from Rt.wait(0.4)
    if keep_hold:
        _log(ctx, "lowered in, still holding the plug")
        return
    yield from Rt.grip(FINGER_PINCH, 0.5)
    yield from Rt.move((DROP_X, HANG_Y, BT + 0.06), yaw=RYAW, duration=0.6)
    _log(ctx, "lowered in")
    pts = charger_points(ctx.states())
    up = pts[_in_pocket(pts, 1.0) & (pts[:, 2] > BT - R - 0.002)]
    for q in up[np.argsort(-up[:, 2])][:2]:
        y = min(q[1], Y_MAX_RIGHT)
        yield from Rt.move((DROP_X, y, BT + 0.06), yaw=RYAW, finger=FINGER_CLOSED, duration=0.8)
        yield from Rt.move((DROP_X, y, BT - 0.012), yaw=RYAW, duration=0.8)
        yield from Rt.move((DROP_X, y, BT + 0.06), yaw=RYAW, duration=0.6)
    _log(ctx, "stowed")


def _onearm_push(ctx):
    """Cheat (right arm only, flap never held): set the brick down against the closed flap's front edge, let go and
    shove it in under the flap towards the hinge, press down; then do the same with the USB-C plug."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    Rt = ctx.right
    y = -0.06
    yield from Rt.move((0.215, y, BT + 0.06), yaw=RYAW, duration=1.5)
    yield from Rt.move((0.215, y, BT + 0.012), yaw=RYAW, duration=1.0)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((0.215, y, BT + 0.07), yaw=RYAW, duration=0.6)
    for x_end in (0.30, 0.32):  # shove the brick at / under the flap edge, twice
        yield from Rt.move((0.185, y, BT + 0.07), yaw=RYAW, finger=FINGER_CLOSED, duration=0.6)
        yield from Rt.move((0.185, y, BT + 0.016), yaw=RYAW, duration=0.6)
        yield from Rt.move((x_end, y, BT + 0.016), yaw=RYAW, duration=2.0)
        yield from Rt.move((x_end, y, BT + 0.07), yaw=RYAW, duration=0.6)
    yield from Rt.move((0.27, y, BT + 0.07), yaw=RYAW, duration=0.6)
    yield from Rt.move((0.27, y, BT + 0.004), yaw=RYAW, duration=0.8)  # press down on whatever is there
    yield from Rt.move((0.27, y, BT + 0.07), yaw=RYAW, duration=0.6)
    _log(ctx, "brick pushed")
    c = ctx.states()[CORD.name]
    u = c.start_plug
    uy = _grip_yaw(c.start_dir)
    yield from Rt.move((u[0], u[1], max(u[2], BT) + 0.05), yaw=uy, finger=FINGER_CORD, duration=1.2)
    yield from Rt.move((u[0], u[1], u[2] + 0.006), yaw=uy, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((0.215, -0.08, BT + 0.06), yaw=RYAW, duration=1.5)
    yield from Rt.move((0.215, -0.08, BT + 0.010), yaw=RYAW, duration=0.8)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((0.215, -0.08, BT + 0.07), yaw=RYAW, duration=0.6)
    yield from Rt.move((0.185, -0.08, BT + 0.07), yaw=RYAW, finger=FINGER_CLOSED, duration=0.6)
    yield from Rt.move((0.185, -0.08, BT + 0.014), yaw=RYAW, duration=0.6)
    yield from Rt.move((0.31, -0.08, BT + 0.014), yaw=RYAW, duration=2.0)
    yield from Rt.move((0.31, -0.08, BT + 0.07), yaw=RYAW, duration=0.6)
    yield from Rt.move((0.27, -0.07, BT + 0.004), yaw=RYAW, duration=0.8)
    yield from Rt.move((0.27, -0.07, BT + 0.07), yaw=RYAW, duration=0.6)
    _log(ctx, "pushed")


def _plug_on_rim(ctx, x):
    """Cheat helper: brick in, cable hung into the pocket, but the USB-C plug set down at ``x`` on the bag top
    (front rim of the opening: the flap shuts onto it; further out: the cable drapes over the front edge)."""
    from yam_arena.constants import FINGER_CLOSED

    Rt = ctx.right
    yield from _brick_down(ctx)
    yield from _plug_over(ctx)
    yield from Rt.move((DROP_X, HANG_Y, BT + 0.03), yaw=RYAW, duration=1.5)
    yield from Rt.move((x, HANG_Y, BT + 0.03), yaw=RYAW, duration=1.0)
    yield from Rt.move((x, HANG_Y, BT + 0.012), yaw=RYAW, duration=0.6)
    yield from Rt.grip(FINGER_PINCH, 0.4)
    yield from Rt.move((x, HANG_Y, BT + 0.08), yaw=RYAW, finger=FINGER_CLOSED, duration=0.6)
    _log(ctx, f"plug left at x={x}")


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    # test only: correct | nothing | brick_only | hold | onearm_push | prop | drape
    mode = os.environ.get("STOW_DEMO", "correct")
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(5):
            yield from ctx.left.wait(1.0)
            _log(ctx)
        return
    if mode == "onearm_push":  # cheat: one arm only, force the charger in against the closed flap
        yield from _brick_in(ctx)
        yield from _onearm_push(ctx)
        yield from ctx.right.move((0.16, -0.24, BT + 0.15), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        yield from ctx.right.wait(1.5)
        _log(ctx, "end")
        return
    flags = _Flags()

    def right():
        yield from _brick_in(ctx)
        while not flags.open:
            yield
        if mode == "brick_only":  # cheat: brick into the pocket, cable left lying on the bag
            yield from _brick_down(ctx)
        elif mode == "prop":  # cheat: plug left on the opening's front rim, the flap shuts onto it (propped)
            yield from _plug_on_rim(ctx, PX0 - 0.004)
        elif mode == "drape":  # cheat: cable draped over the front edge, plug on the bag top outside
            yield from _plug_on_rim(ctx, 0.20)
        elif mode == "hold":  # cheat: pack everything but keep holding the USB-C plug inside the pocket
            yield from _stow(ctx, keep_hold=True)
            flags.stowed = True
            return
        else:
            yield from _stow(ctx)
        yield from ctx.right.move((0.16, -0.24, BT + 0.15), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        flags.stowed = True

    def left():
        yield from _open_flap(ctx, flags)
        while not flags.stowed:
            yield
        yield from _close_flap(ctx)

    yield from ctx.both(left(), right())
    yield from ctx.left.wait(1.5)
    _log(ctx, "end")
