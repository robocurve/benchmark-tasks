"""On the station table, a black 8 mm power cord crosses a cart lane. It lies straight along y, 28 cm in front of the
robot, between two spring-loaded cord retractors: a grey floor box in front of the right arm (cord exit at y -20 cm)
and a yellow wall reel at y +38 cm behind a grey partition (220 mm long, 70 mm tall, along the left side of the cart
lane); the cord passes the partition through an 18 x 20 mm notch at its base. Each retractor pays out up to 12 cm of
cord (0.3 N pull at rest, ~1 N at 12 cm) and winds it back in. A white inline switch box (50 x 30 x 22 mm) sits on
the cord at y -2 .. +3 cm. A light blue cart block (140 x 50 x 30 mm, 200 g) stands in the lane in front of the left
arm, its front 2.4 cm before the cord; two orange marks 6 cm beyond the cord line mark the finish line (x 34 cm).
The cart must be slid along the table (never lifted) fully past the finish line without ever touching the cord: one
arm lifts the cord and holds it up, the other moves the cart under it. Scored: the cart passes under the cord, i.e.
when the cart's centre crosses the cord line x 28 cm it is within 5 cm of y 12 cm (a cart taken round either end of
the cord scores 0) and the cord's underside over the cart's width is at or above the cart's top (3 cm). Then the cord
is laid back down on its line and let go.

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

INSTRUCTION = ("Get the blue cart past the orange finish marks without it ever touching the black power cord: lift "
               "the cord with one arm and slide the cart under it with the other. Keep the cart on the table. Then "
               "lay the cord back down along its line and let go of it.")

# -- layout (m; x forward from the robot, y to the robot's left) ------------------------------------------------------
LANE_X = 0.28  # the cord's line (runs along y)
R = 0.004  # 8 mm cord
CORD_Y0 = -0.20  # cord leaves the outlet box here
# inline switch box on the cord (the cord is two kit cables: outlet -> switch box, switch box -> reel)
SWITCH = Plug(size=(0.05, 0.03, 0.022), mass=0.025, color=(0.92, 0.92, 0.90))
SWITCH_Y0 = -0.02  # switch box spans y -0.02 .. +0.03
CORD_Z = TOP + SWITCH.size[2] / 2 + 0.0005
CORD_END_Y = 0.38  # the cord's end enters the reel here (11 cm behind the partition)
CORD_A = cable("cord_a", start=(LANE_X, CORD_Y0, CORD_Z), yaw=math.pi / 2, length=SWITCH_Y0 - CORD_Y0, radius=R,
               end_plug=SWITCH, cable_mass=0.025, color=(0.05, 0.05, 0.05), bend_stiffness=0.01,
               bend_damping=0.001)
_B0 = SWITCH_Y0 + SWITCH.size[0]
CORD_B = cable("cord_b", start=(LANE_X, _B0, CORD_Z), yaw=math.pi / 2, length=CORD_END_Y - _B0, radius=R,
               cable_mass=0.045, color=(0.05, 0.05, 0.05), bend_stiffness=0.01, bend_damping=0.001)
GREY = (0.60, 0.62, 0.64)
OUTLET_SIZE = (0.05, 0.04, 0.03)
OUTLET = Box("floor_outlet", pos=(LANE_X, CORD_Y0 - 0.006 - OUTLET_SIZE[1] / 2, TOP + OUTLET_SIZE[2] / 2),
             size=OUTLET_SIZE, color=(0.42, 0.44, 0.47))

# desk partition (plane y = PANEL_Y) with a cable notch at its bottom edge, centred on the cord line
PANEL_Y, PANEL_T, PANEL_H = 0.27, 0.012, 0.07
PANEL_X0, PANEL_X1 = 0.18, 0.40
NOTCH_W, NOTCH_H = 0.018, 0.020
_nx0, _nx1 = LANE_X - NOTCH_W / 2, LANE_X + NOTCH_W / 2


def _pbox(name, x0, x1, z0, z1):
    return Box(name, pos=((x0 + x1) / 2, PANEL_Y, TOP + (z0 + z1) / 2), size=(x1 - x0, PANEL_T, z1 - z0), color=GREY)


PARTITION = [_pbox("partition_a", PANEL_X0, _nx0, 0.0, PANEL_H), _pbox("partition_b", _nx1, PANEL_X1, 0.0, PANEL_H),
             _pbox("partition_top", _nx0, _nx1, NOTCH_H, PANEL_H)]

# retractable cord reel behind the partition: the cord's end enters it (spring-loaded, see _install_reel)
REEL_SIZE = (0.08, 0.06, 0.06)
REEL = Box("cord_reel", pos=(LANE_X, CORD_END_Y + 0.012 + REEL_SIZE[1] / 2, TOP + REEL_SIZE[2] / 2), size=REEL_SIZE,
           color=(0.85, 0.75, 0.10))
REEL_PAYOUT = 0.12  # m of cord each retractor can pay out
REEL_K = 6.0  # N/m spring
REEL_PRELOAD = 0.05  # m: spring target beyond the stop -> 0.3 N pull at rest, 1.0 N with 12 cm paid out

# cart (free, slides on the table) and the finish line
CART_SIZE = (0.14, 0.05, 0.03)
CART_XY = np.array([0.18, 0.12])
CART = Box("cart", pos=(float(CART_XY[0]), float(CART_XY[1]), TOP + CART_SIZE[2] / 2 + 0.0005), size=CART_SIZE,
           mass=0.20, color=(0.25, 0.50, 0.90))
FINISH_X = 0.34  # finish line (6 cm beyond the cord line)
CART_LANE_TOL = 0.05  # cart centre must cross the cord line and end within 5 cm (y) of the lane centre line
_ORANGE = (1.0, 0.45, 0.0)
MARK_T = 0.001
FINISH = [Box(f"finish_mark_{k}", pos=(FINISH_X + 0.005, y, TOP + MARK_T / 2), size=(0.01, 0.02, MARK_T),
              color=_ORANGE) for k, y in (("r", CART_XY[1] - 0.05), ("l", CART_XY[1] + 0.05))]

OBJECTS = [OUTLET, *PARTITION, REEL, CORD_A, CORD_B, CART, *FINISH]


# -- the reel: cablekit only welds a cable end to the world, so the cable's spawn function is wrapped to tie its last
# segment to the world with a prismatic joint along the cord (the reel's slot) with a preloaded linear spring drive:
# the cord can be drawn out of the reel up to 15 cm and is pulled back in when let go. The joint is excluded from the
# cable articulation.


def _reel_joint(stage, prim_path: str, seg: str, spec, at_end: bool) -> None:
    """Prismatic joint along the cord at one cord end (world <-> that end segment) with a preloaded spring drive:
    the cord can be drawn out by up to REEL_PAYOUT and is pulled back in."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    sgn = 1.0 if at_end else -1.0  # +x of the segment frame points along the cord, from its start to its end
    world = UsdGeom.Xformable(stage.GetPrimAtPath(seg)).ComputeLocalToWorldTransform(0)
    q = world.ExtractRotationQuat()
    tip = world.Transform(Gf.Vec3d(sgn * spec.seg_len / 2, 0.0, 0.0))
    j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/reel")
    j.CreateBody1Rel().SetTargets([Sdf.Path(seg)])
    j.CreateAxisAttr("X")
    j.CreateLocalPos0Attr(Gf.Vec3f(*tip))
    j.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), *q.GetImaginary()))
    j.CreateLocalPos1Attr(Gf.Vec3f(sgn * spec.seg_len / 2, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    # drawing the cord out moves the end segment towards the cord's middle: -x at the end, +x at the start
    j.CreateLowerLimitAttr(-REEL_PAYOUT if at_end else 0.0)
    j.CreateUpperLimitAttr(0.0 if at_end else REEL_PAYOUT)
    j.CreateExcludeFromArticulationAttr(True)
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drv.CreateTypeAttr("force")
    drv.CreateStiffnessAttr(REEL_K)
    drv.CreateDampingAttr(0.5)
    drv.CreateTargetPositionAttr(sgn * REEL_PRELOAD)
    if os.environ.get("CART_DEBUG"):
        print(f"[reel] {prim_path}/reel at {tuple(round(v, 3) for v in tip)}", flush=True)


def _install_reel() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_reel", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        name = getattr(cfg, "cable_name", "")
        if name not in (CORD_A.name, CORD_B.name):
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        if name == CORD_A.name:  # floor retractor on the right
            _reel_joint(stage, prim_path, f"{prim_path}/seg_0", CORD_A, at_end=False)
            return prim
        # weld: cord_b's first segment to the switch box (cord_a's end plug)
        sw = f"{str(prim_path).rsplit('/', 1)[0]}/{CORD_A.name}/end_plug"
        w = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/switch_weld")
        w.CreateBody0Rel().SetTargets([Sdf.Path(sw)])
        w.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        w.CreateLocalPos0Attr(Gf.Vec3f(SWITCH.size[0] / 2, 0.0, 0.0))
        w.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        w.CreateLocalPos1Attr(Gf.Vec3f(-CORD_B.seg_len / 2, 0.0, 0.0))
        w.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        w.CreateExcludeFromArticulationAttr(True)
        # reel behind the partition on the left
        _reel_joint(stage, prim_path, f"{prim_path}/seg_{CORD_B.segments - 1}", CORD_B, at_end=True)
        return prim

    spawn._reel = True
    cablekit._spawn_cable = spawn


_install_reel()


def paid_out(objects) -> tuple[float, float]:
    """Cord drawn out of the right and the left retractor (m)."""
    return (float(objects["cord_a"].start_tip[1] - CORD_Y0), float(CORD_END_Y - objects["cord_b"].end_tip[1]))

# -- scoring ---------------------------------------------------------------------------------------------------------
TOUCH = 0.002  # cart surface within 2 mm of the cord's surface = touching
LIFT_MAX = 0.010  # cart bottom more than 1 cm above the table = lifted (it may only roll/slide)
TILT_MAX = 15.0  # deg
LINE_TOL = 0.03  # cord back in its line: every point between outlet and partition within 3 cm of x = LANE_X ...
FLOOR_TOL = 0.015  # ... and lying on the table (centre at most 1.5 cm above it)
RELEASE_DIST = 0.03  # both grippers at least 3 cm from the cord


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    poly = np.nan_to_num(np.asarray(poly, dtype=float), nan=1e3, posinf=1e3, neginf=-1e3)
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = min(1000, max(1, int(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def cord_points(objects) -> tuple[np.ndarray, np.ndarray]:
    """Dense points along the whole cord (outlet to reel, through the switch box) and the cord's radius there (for
    the switch box: half its height, sampled along its axis)."""
    a, b = objects["cord_a"], objects["cord_b"]
    pa, pb = _dense(a.polyline()), _dense(b.polyline())
    ps = np.array([a.end_tip + a.end_dir * t for t in np.linspace(0.0, SWITCH.size[0], 8)])
    pts = np.vstack([pa, ps, pb])
    rad = np.r_[np.full(len(pa), R), np.full(len(ps), SWITCH.size[2] / 2), np.full(len(pb), R)]
    return pts, rad


def cart_cord_gap(objects) -> float:
    """Smallest distance (m) between the cart's surface and the cord's surface (switch box included)."""
    cart = objects["cart"]
    pts, rad = cord_points(objects)
    local = (pts - cart.pos) @ _rot(cart.quat_xyzw)
    d = np.linalg.norm(np.maximum(np.abs(local) - np.array(CART_SIZE) / 2, 0.0), axis=1)
    return float((d - rad).min())


UNDER_BAND = 0.01  # cord checked over the cart's width plus 1 cm on each side


def cord_clear_above_cart(objects) -> float:
    """Height (m) of the cord's underside above the cart's top, taking the lowest cord point (switch box included)
    whose y lies over the cart's width (+-1 cm); negative = some of the cord there is below the cart top, or no cord
    crosses the cart's width at all (-1). The cart passes under the cord when this is >= 0 as its centre crosses x 28."""
    cart = objects["cart"]
    pts, rad = cord_points(objects)
    m = np.abs(pts[:, 1] - cart.pos[1]) <= CART_SIZE[1] / 2 + UNDER_BAND
    if not m.any():
        return -1.0
    return float((pts[m, 2] - rad[m]).min() - cart.high[2])


def cart_tilt_deg(cart) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, _rot(cart.quat_xyzw)[2, 2]))))


def cart_bottom(cart) -> float:
    return float(cart.low[2] - TOP)


def cart_past(objects) -> bool:
    """Cart fully past the finish line (its rear edge beyond it), in the lane, upright on the table."""
    cart = objects["cart"]
    return bool(cart.low[0] >= FINISH_X and abs(cart.pos[1] - CART_XY[1]) <= CART_LANE_TOL
                and cart_bottom(cart) <= 0.004 and cart_tilt_deg(cart) <= 5.0)


def _crossing_part(objects) -> np.ndarray:
    """Segment centres between the outlet box and the partition."""
    p = np.vstack([objects["cord_a"].points, objects["cord_a"].end_plug[None], objects["cord_b"].points])
    return p[p[:, 1] < PANEL_Y - PANEL_T / 2]


def cord_in_line(objects) -> bool:
    p = _crossing_part(objects)
    return bool(len(p) and np.abs(p[:, 0] - LANE_X).max() <= LINE_TOL and (p[:, 2] - TOP).max() <= FLOOR_TOL)



# Episode memory: every "at any time" rule (touched / lifted / tipped / broken / crossed outside the lane / crossed
# without the cord above the cart) latches and is cleared only by the embodiment reset at the start of an episode.
_EP = {"touched": False, "lifted": False, "tipped": False, "broken": False, "min_gap": float("inf"),
       "max_bottom": 0.0, "far_side": None, "crossed": False, "off_lane": False, "not_under": False,
       "under_clear": None}


def _reset_episode() -> None:
    _EP.update(touched=False, lifted=False, tipped=False, broken=False, min_gap=float("inf"), max_bottom=0.0,
               far_side=None, crossed=False, off_lane=False, not_under=False, under_clear=None)


def _install_reset_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    cls.reset._task_reset_hooks.append(_reset_episode)


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_cart_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._cart_hooked = True
    cls._task_step = _task_step


_install_reset_hook()
_install_env_hook()


def _track(objects) -> None:
    cart = objects["cart"]
    pts = np.vstack([objects["cord_a"].points, objects["cord_b"].points])
    if not (np.isfinite(pts).all() and np.isfinite(cart.pos).all()):
        if not _EP["broken"]:
            bad = [k for k in ("cord_a", "cord_b") if not np.isfinite(objects[k].points).all()]
            print(f"[cart task] non-finite state in {bad or ['cart']}: simulation broke", flush=True)
        _EP["broken"] = True  # the simulation broke (e.g. the cord was ripped apart): no score
        return
    # The cart must pass under the cord: whenever the cart's centre crosses the cord line (x = LANE_X), in either
    # direction, it must be inside the lane (centre within 5 cm of y 12; going round an end of the cord crosses more
    # than 30 cm from it) and the cord must be above the cart there (cord_clear_above_cart() >= 0). Either failure is
    # latched as a foul. A forward crossing that passes both checks is recorded as "crossed".
    far = bool(cart.pos[0] >= LANE_X)
    if _EP["far_side"] is not None and far != _EP["far_side"]:
        clear = cord_clear_above_cart(objects)
        if _EP["under_clear"] is None or clear < _EP["under_clear"]:
            _EP["under_clear"] = clear
        if abs(cart.pos[1] - CART_XY[1]) > CART_LANE_TOL:
            _EP["off_lane"] = True
        elif clear < 0.0:
            _EP["not_under"] = True
        elif far:
            _EP["crossed"] = True
        if os.environ.get("CART_DEBUG"):
            print(f"[cart task] crossing x {LANE_X}: cart y {cart.pos[1]:.3f}, cord clear above cart {clear:.3f} m, "
                  f"off_lane={_EP['off_lane']} not_under={_EP['not_under']} crossed={_EP['crossed']}", flush=True)
    _EP["far_side"] = far
    g = cart_cord_gap(objects)
    _EP["min_gap"] = min(_EP["min_gap"], g)
    if g <= TOUCH:
        if not _EP["touched"] and os.environ.get("CART_DEBUG"):
            pts, rad = cord_points(objects)
            local = (pts - cart.pos) @ _rot(cart.quat_xyzw)
            d = np.linalg.norm(np.maximum(np.abs(local) - np.array(CART_SIZE) / 2, 0.0), axis=1) - rad
            i = int(np.argmin(d))
            print(f"[cart task] TOUCH at cord point {np.round(pts[i], 3)} (cart-local {np.round(local[i], 3)}), "
                  f"grippers {[np.round(p, 3) for p in grippers()]}", flush=True)
        _EP["touched"] = True
    b = cart_bottom(cart)
    _EP["max_bottom"] = max(_EP["max_bottom"], b)
    if b > LIFT_MAX:
        _EP["lifted"] = True
    if cart_tilt_deg(cart) > TILT_MAX:
        _EP["tipped"] = True


def fouled() -> bool:
    return (_EP["touched"] or _EP["lifted"] or _EP["tipped"] or _EP["broken"] or _EP["off_lane"]
            or _EP["not_under"])


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
    pts = cord_points(objects)[0]
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """The cart passes under the cord: each time the cart's centre crosses the cord line (x 28 cm), it must be within
    5 cm of the lane centre line (y 12 cm) and the underside of the cord over the cart's width (+-1 cm) must be at or
    above the cart's top. 0 if that ever fails, or if the cart ever came within 2 mm of the cord, was ever lifted more
    than 1 cm off the table or tilted more than 15 deg (all latched for the episode). 0 too if it never crossed that
    line under the cord, or at the end is not fully past the finish line (rear edge beyond it, centre within 5 cm of
    the lane centre line, upright on the table). Otherwise 0.7 with the cart past the line and
    the cord not back or still held; 1.0 with the cart past the line, the cord between the outlet box and the partition
    on the table within 3 cm of its line and both grippers at least 3 cm from the cord."""
    _track(objects)
    if fouled() or not _EP["crossed"] or not cart_past(objects):
        return 0.0
    if cord_in_line(objects) and released(objects):
        return 1.0
    return 0.7


def success(objects) -> bool:
    return (score(objects) >= 1.0 and objects["cart"].resting() and objects["cord_a"].resting(0.02, 1.0)
            and objects["cord_b"].resting(0.02, 1.0))


def distance(objects) -> float:
    """How far the cart's rear edge still is from the finish line (m); 1 m once fouled."""
    _track(objects)
    if fouled():
        return 1.0
    return float(max(0.0, FINISH_X - objects["cart"].low[0]))


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------

LIFT_Y = SWITCH_Y0 + SWITCH.size[0] / 2  # the right arm lifts the cord by its switch box
LIFT_TCP = TOP + 0.15
PUSH_Z = TOP + 0.02
PUSH_STOP = LANE_X - 0.045  # the pushing gripper stays this far before the cord line
GRIP_DEPTH = 0.012  # TCP below the cart top when gripping it
GRIP_BACK = 0.015  # TCP this far behind the cart's front end when gripping it
CART_GOAL_REAR = FINISH_X + 0.02


def _log(ctx, tag=""):
    st = ctx.states()
    cart = st["cart"]
    p = _crossing_part(st)
    dev = float(np.abs(p[:, 0] - LANE_X).max()) if len(p) else -1
    print(f"[t={ctx.t:5.2f}] {tag}: cart={np.round(cart.pos, 3)} rear={cart.low[0]:.3f} gap={cart_cord_gap(st):.3f} "
          f"min_gap={_EP['min_gap']:.3f} bottom={_EP['max_bottom']:.4f} paid={np.round(paid_out(st), 3)} "
          f"line_dev={dev:.3f} maxz={p[:, 2].max() - TOP if len(p) else -1:.3f} past={cart_past(st)} "
          f"rel={released(st)} fouled={fouled()} score={score(st):.2f} tilt={cart_tilt_deg(cart):.1f} "
          f"cord_over_cart_minz={_over_cart_z(st):.3f} crossed={_EP['crossed']} off_lane={_EP['off_lane']} "
          f"not_under={_EP['not_under']} under_clear={_EP['under_clear']} touched={_EP['touched']} "
          f"score_without_lane_rule={_score_without_lane_rule(st):.2f}", flush=True)


def _score_without_lane_rule(st) -> float:
    """What score() would give with no crossing rule (log only): shows what the lane rule takes away."""
    if _EP["touched"] or _EP["lifted"] or _EP["tipped"] or _EP["broken"] or not cart_past(st):
        return 0.0
    return 1.0 if cord_in_line(st) and released(st) else 0.7


def _over_cart_z(st) -> float:
    p = cord_points(st)[0]
    m = np.abs(p[:, 1] - st["cart"].pos[1]) <= CART_SIZE[1] / 2 + 0.01
    return float(p[m, 2].min() - TOP) if m.any() else -1.0




def _round_the_outlet(ctx):
    """Shortcut: the cord stays on the table. The cart is gripped and slid round the outlet's open end: left arm to the
    midline, right arm beyond the outlet (y -30.5 cm), past x 28 and back to the midline, left arm into the lane."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    gz = float(ctx.states()["cart"].high[2]) - GRIP_DEPTH
    hi, mid_y, side_y, far_x, near_x = TOP + 0.10, 0.01, -0.305, 0.425, 0.17
    back = 0.03  # the right arm grips this far behind the cart's centre (reach at the far midline)
    c = ctx.states()["cart"].pos
    yield from L.move((c[0], c[1], hi), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((c[0], c[1], gz), yaw=0.0, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((near_x, mid_y, gz), yaw=0.0, duration=2.0)
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((near_x, mid_y, hi), yaw=0.0, duration=0.8)
    yield from L.move((0.20, 0.30, TOP + 0.15), yaw=0.0, finger=FINGER_CLOSED, duration=1.0)
    _log(ctx, "cart at the midline")
    c = ctx.states()["cart"].pos
    yield from Rt.move((c[0] - back, c[1], hi), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move((c[0] - back, c[1], gz), yaw=0.0, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    yield from Rt.move((near_x - back, side_y, gz), yaw=0.0, duration=2.5)
    _log(ctx, "cart beyond the outlet")
    yield from Rt.move((far_x - back, side_y, gz), yaw=0.0, duration=2.5)
    _log(ctx, "cart past x 28")
    yield from Rt.move((far_x - back, mid_y, gz), yaw=0.0, duration=2.5)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((far_x - back, mid_y, gz + 0.045), yaw=0.0, duration=0.6)
    yield from Rt.move((0.25, -0.30, TOP + 0.15), yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    _log(ctx, "cart back at the midline, beyond the cord")
    c = ctx.states()["cart"].pos
    gx = float(c[0]) - 0.02
    yield from L.move((gx, c[1], hi), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((gx, c[1], gz), yaw=0.0, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((gx, CART_XY[1], gz), yaw=0.0, duration=1.5)
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((gx, CART_XY[1], TOP + 0.12), yaw=0.0, duration=0.8)
    yield from L.move((0.20, 0.30, TOP + 0.15), yaw=0.0, duration=1.0)
    _log(ctx, "released")
    yield from L.wait(1.5)
    _log(ctx, "end")


def demo_script(ctx):
    """CART_DEMO=correct (default) | nothing | through (push the cart straight through the cord) | carry (lift the
    cart over the cord) | round (never lift the cord: slide the cart round the floor outlet's open end) | lifthold
    (lift the cord and keep holding it, cart unmoved) | low (the cord is gripped but raised only ~1 cm, below the
    cart's top, and the cart is pushed across in the lane) | hold (correct, but the cord is kept lifted at the end)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("CART_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return

    cart = ctx.states()["cart"]
    cy = float(cart.pos[1])
    rear = float(cart.low[0])
    # 1) left arm behind the cart (closed fingers as a pusher); right arm takes the cord
    c0 = ctx.states()["cord_a"].end_plug
    if mode == "carry":
        yield from L.move((cart.pos[0], cy, TOP + 0.08), yaw=0.0, finger=FINGER_OPEN, duration=1.5)
        yield from L.move((cart.pos[0], cy, cart.high[2] - GRIP_DEPTH), yaw=0.0, duration=1.0)
        yield from L.grip(FINGER_CLOSED, 0.6)
        yield from L.move((cart.pos[0], cy, TOP + 0.10), yaw=0.0, duration=1.0)
        yield from L.move((CART_GOAL_REAR + 0.08, cy, TOP + 0.10), yaw=0.0, duration=2.0)
        yield from L.move((CART_GOAL_REAR + 0.08, cy, cart.high[2] - GRIP_DEPTH + 0.001), yaw=0.0, duration=1.0)
        yield from L.grip(FINGER_OPEN, 0.6)
        yield from L.move((CART_GOAL_REAR + 0.08, cy, TOP + 0.12), yaw=0.0, duration=0.8)
        _log(ctx, "end")
        return
    if mode == "round":
        yield from _round_the_outlet(ctx)
        return
    yield from ctx.both(L.move((rear - 0.04, cy, PUSH_Z + 0.05), yaw=0.0, finger=FINGER_CLOSED, duration=1.5),
                        Rt.move((c0[0], c0[1], TOP + 0.06), yaw=math.pi / 2, finger=FINGER_OPEN, duration=1.5))
    yield from ctx.both(L.move((rear - 0.04, cy, PUSH_Z), yaw=0.0, duration=0.8),
                        Rt.move((c0[0], c0[1], c0[2] + 0.008), yaw=math.pi / 2, duration=1.0))
    if mode == "through":
        yield from L.move((CART_GOAL_REAR - 0.02, cy, PUSH_Z), yaw=0.0, duration=5.0)
        _log(ctx, "end")
        return
    yield from Rt.grip(FINGER_CLOSED, 0.6)
    # 2) right arm lifts the cord high; left arm pushes the cart until its front half is under/past the cord
    lift_tcp = c0[2] + 0.008 + 0.012 if mode == "low" else LIFT_TCP
    yield from ctx.both(Rt.move((LANE_X, LIFT_Y, lift_tcp), yaw=math.pi / 2, duration=2.0),
                        L.wait(0.5))
    _log(ctx, "cord lifted")
    yield from L.wait(1.0)
    _log(ctx, "cord held 1 s")
    if mode == "lifthold":
        for _ in range(12):
            yield from L.wait(0.5)
            _log(ctx, "holding")
        return
    yield from L.move(((PUSH_STOP + rear - 0.04) / 2, cy, PUSH_Z), yaw=0.0, duration=1.5)
    _log(ctx, "pushing")
    yield from L.move((PUSH_STOP, cy, PUSH_Z), yaw=0.0, duration=1.5)
    _log(ctx, "pushed under")
    # 3) left arm goes up over the cord, grips the cart's front end (already past the cord) and slides it on
    cart = ctx.states()["cart"]
    yield from L.move((PUSH_STOP, cy, TOP + 0.17), yaw=0.0, finger=FINGER_CLOSED, duration=1.0)
    gx = float(cart.high[0]) - GRIP_BACK
    yield from L.move((gx, cy, TOP + 0.17), yaw=0.0, duration=1.2)
    yield from L.move((gx, cy, TOP + 0.09), yaw=0.0, finger=FINGER_OPEN, duration=1.0)
    _log(ctx, "over cart front")
    cart = ctx.states()["cart"]
    gx, gy = float(cart.high[0]) - GRIP_BACK, float(cart.pos[1])
    gz = float(cart.high[2]) - GRIP_DEPTH
    yield from L.move((gx, gy, gz), yaw=0.0, duration=1.0)
    _log(ctx, "at cart front")
    yield from L.grip(FINGER_CLOSED, 0.6)
    _log(ctx, "cart gripped")
    cart = ctx.states()["cart"]
    dx = CART_GOAL_REAR - float(cart.low[0])
    yield from L.move((gx + dx, CART_XY[1], gz), yaw=0.0, duration=2.5)
    _log(ctx, "cart past")
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((gx + dx, CART_XY[1], TOP + 0.12), yaw=0.0, duration=0.8)
    if mode == "hold":
        yield from L.wait(2.0)
        _log(ctx, "end")
        return
    # 4) the right arm lays the cord back down on its line and lets go (the reel takes the slack back in); both
    #    arms move clear
    yield from Rt.move((LANE_X, LIFT_Y, TOP + 0.04), yaw=math.pi / 2, duration=2.0)
    yield from Rt.move((LANE_X, LIFT_Y, CORD_Z + 0.009), yaw=math.pi / 2, duration=1.0)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((LANE_X, LIFT_Y, TOP + 0.08), yaw=math.pi / 2, duration=0.8)
    yield from ctx.both(L.move((0.20, 0.30, TOP + 0.15), yaw=0.0, duration=1.0),
                        Rt.move((0.20, -0.30, TOP + 0.15), yaw=0.0, duration=1.0))
    _log(ctx, "released")
    yield from L.wait(1.5)
    _log(ctx, "end")
