"""On the station table, close to the robot: a dark-grey action camera (50 x 40 x 40 mm box, 80 g, free) in front of
the left arm and a blue power bank (60 x 35 x 28 mm box, 120 g, free) in front of the right arm. They are joined by a
black 6 mm, 28 cm USB cord whose ends are fixed into both devices (it leaves the camera's right side and the bank's
left side at table level) and that lies straight and just taut between them. Farther from the robot is a fixed
light-wood shelf board (12 x 40 cm, 3 cm high) straddling the midline. Both devices and the cord must end up on the
shelf, released. Each device is reachable only by its own arm, and the cord is already taut: moving one device
forward alone drags the other along the table by its cord. The arms lift and carry both (the demo carries them
together in lockstep), keeping the cord slack, and set them down on the shelf.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score \\
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
from cablekit import cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Put the action camera, the power bank and the cord joining them on the wooden shelf, then let go. "
               "The cord is already taut: lift and carry the devices so the cord never pulls on either one. Never "
               "drag, slide or hang a device by the cord, and leave no part of the cord hanging off the shelf.")

# -- the two devices (free rigid boxes) ------------------------------------------------------------------------------
CAM_SIZE, CAM_MASS = (0.050, 0.040, 0.040), 0.08
BANK_SIZE, BANK_MASS = (0.060, 0.035, 0.028), 0.12
R = 0.003  # 6 mm cord
CORD_LEN = 0.28
X0 = 0.16
CAM_XY = np.array([X0, 0.17])
CORD_START = (X0, float(CAM_XY[1] - CAM_SIZE[1] / 2), TOP + R + 0.0005)  # on the camera's right (-y) face
CORD_YAW = -math.pi / 2
BANK_XY = np.array([X0, CORD_START[1] - CORD_LEN - BANK_SIZE[1] / 2])  # cord end on the bank's left (+y) face
CAMERA = Box("action_camera", pos=(float(CAM_XY[0]), float(CAM_XY[1]), TOP + CAM_SIZE[2] / 2), size=CAM_SIZE,
             mass=CAM_MASS, color=(0.18, 0.18, 0.20))
BANK = Box("power_bank", pos=(float(BANK_XY[0]), float(BANK_XY[1]), TOP + BANK_SIZE[2] / 2), size=BANK_SIZE,
           mass=BANK_MASS, color=(0.15, 0.35, 0.80))
CORD = cable("cord", start=CORD_START, yaw=CORD_YAW, length=CORD_LEN, radius=R, anchor_start=False,
             color=(0.04, 0.04, 0.04), bend_stiffness=0.01, bend_damping=0.001)
DEVICES = (CAMERA, BANK)

# -- shelf (fixed): light wood board 12 cm (x) x 40 cm (y) x 3 cm high ------------------------------------------------
SHELF_C = np.array([0.37, 0.0])
SHELF_SIZE = (0.12, 0.40, 0.03)
SHELF_TOP = TOP + SHELF_SIZE[2]
SHELF = Box("shelf_board", pos=(float(SHELF_C[0]), float(SHELF_C[1]), TOP + SHELF_SIZE[2] / 2), size=SHELF_SIZE,
            color=(0.80, 0.66, 0.45))

OBJECTS = [CAMERA, BANK, CORD, SHELF]


# -- the cord-to-device welds -----------------------------------------------------------------------------------------
# cablekit can only weld a cable end to the world. The cable's spawn function is wrapped so that, right after the cable
# is authored, fixed joints tie its first segment to the camera and its last segment to the power bank. The joints are
# excluded from the cable articulation, so both devices stay ordinary rigid objects.


def _install_welds() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_pair_weld", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORD.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        q = Gf.Quatf(math.cos(CORD_YAW / 2), 0.0, 0.0, math.sin(CORD_YAW / 2))
        end_tip = np.asarray(CORD.point_at(CORD_LEN))
        for tag, dev, seg, tip, sgn in (("cam", CAMERA, 0, np.asarray(CORD_START), -1.0),
                                        ("bank", BANK, CORD.segments - 1, end_tip, 1.0)):
            j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{tag}_weld")
            j.CreateBody0Rel().SetTargets([Sdf.Path(f"{root}/{dev.name}")])
            j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_{seg}")])
            off = tip - np.asarray(dev.pos)  # cord tip in the device frame (device yaw 0)
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, off)))
            j.CreateLocalRot0Attr(q)
            j.CreateLocalPos1Attr(Gf.Vec3f(sgn * CORD.seg_len / 2, 0.0, 0.0))
            j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            j.CreateExcludeFromArticulationAttr(True)
        return prim

    spawn._pair_weld = True
    cablekit._spawn_cable = spawn


_install_welds()

# -- sim handle for gripper positions (release / dangling checks) ----------------------------------------------------
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_pair_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._pair_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World TCP positions of both grippers (between the finger pads, 13 mm above the pad tips); [] if unknown."""
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


# -- geometry helpers ------------------------------------------------------------------------------------------------
SLIDE_TOL = 0.03  # m a device may shift while resting on a surface (it may only travel while lifted)
SUPPORT = 0.008  # device bottom within 8 mm of the table / shelf top = resting on it
DANGLE = 0.015  # device bottom more than 15 mm above any surface with no gripper on it = hanging by the cord
TIP_DEG = 30.0
UPRIGHT_DEG = 10.0
HOLD_MARGIN = 0.015  # a gripper TCP within the device's box grown by this = gripping it
RELEASE_DIST = 0.02  # every TCP at least this far from both devices and the cord at the end


def tilt_deg(o) -> float:
    x, y, z, w = o.quat_xyzw
    cz = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cz))))


def _over_shelf_any(o) -> bool:
    lo, hi = o.low[:2], o.high[:2]
    s_lo = SHELF_C - np.array(SHELF_SIZE[:2]) / 2
    s_hi = SHELF_C + np.array(SHELF_SIZE[:2]) / 2
    return bool(np.all(hi > s_lo) and np.all(lo < s_hi))


def _over_shelf_all(o) -> bool:
    s_lo = SHELF_C - np.array(SHELF_SIZE[:2]) / 2
    s_hi = SHELF_C + np.array(SHELF_SIZE[:2]) / 2
    return bool(np.all(o.low[:2] >= s_lo) and np.all(o.high[:2] <= s_hi))


def height_above_support(o) -> float:
    """Device bottom height above the highest surface under its footprint (m)."""
    surf = SHELF_TOP if _over_shelf_any(o) else TOP
    return float(o.low[2] - surf)


def supported(o) -> bool:
    if o.low[2] - TOP <= SUPPORT:
        return True
    return _over_shelf_any(o) and o.low[2] - SHELF_TOP <= SUPPORT


def gripped(o, tcps) -> bool:
    lo, hi = o.low - HOLD_MARGIN, o.high + HOLD_MARGIN
    return any(bool(np.all(g >= lo) and np.all(g <= hi)) for g in tcps)


def on_shelf(o) -> bool:
    """Whole footprint over the shelf, bottom on the shelf top, upright."""
    return _over_shelf_all(o) and abs(o.low[2] - SHELF_TOP) <= SUPPORT and tilt_deg(o) <= UPRIGHT_DEG


def cord_on_shelf(objects) -> bool:
    pts = objects["cord"].polyline()
    s_lo = SHELF_C - np.array(SHELF_SIZE[:2]) / 2
    s_hi = SHELF_C + np.array(SHELF_SIZE[:2]) / 2
    return bool(np.all(pts[:, :2] >= s_lo) and np.all(pts[:, :2] <= s_hi) and np.all(pts[:, 2] <= SHELF_TOP + 0.012))


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.asarray(out)


def _box_dist(p, o) -> float:
    d = np.maximum(np.maximum(o.low - p, p - o.high), 0.0)
    return float(np.linalg.norm(d))


def released(objects) -> bool:
    pts = _dense(objects["cord"].polyline())
    for g in grippers():
        if min(_box_dist(g, objects[d.name]) for d in DEVICES) < RELEASE_DIST:
            return False
        if float(np.linalg.norm(pts - g, axis=1).min()) < RELEASE_DIST:
            return False
    return True


# -- episode memory: any device slid, dangled from the cord, tipped or fell at any step ------------------------------
_START = {d.name: np.array(d.pos[:2], dtype=float) for d in DEVICES}
_EP: dict = {}


def _reset_episode() -> None:
    _EP.clear()
    _EP.update(violation="", anchor={k: v.copy() for k, v in _START.items()}, max_slide=0.0, max_tilt=0.0,
               max_dangle=0.0)


_reset_episode()


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


_install_reset_hook()
_START_PTS = CORD.start_points()


def _flag(why: str) -> None:
    if not _EP["violation"]:
        _EP["violation"] = why


def _track(objects) -> None:
    c = objects["cord"]
    if (np.abs(c.points - _START_PTS).max() < 0.003
            and all(np.linalg.norm(objects[d.name].pos[:2] - _START[d.name]) < 0.002 for d in DEVICES)):
        _reset_episode()  # untouched starting layout => new episode
    tcps = grippers()
    for d in DEVICES:
        o = objects[d.name]
        t = tilt_deg(o)
        _EP["max_tilt"] = max(_EP["max_tilt"], t)
        if t > TIP_DEG:
            _flag(f"{d.name} tipped")
        if o.pos[2] < TOP - 0.02:
            _flag(f"{d.name} fell")
        if supported(o):
            if _EP["anchor"][d.name] is None:
                _EP["anchor"][d.name] = o.pos[:2].copy()  # set down here
            slide = float(np.linalg.norm(o.pos[:2] - _EP["anchor"][d.name]))
            _EP["max_slide"] = max(_EP["max_slide"], slide)
            if slide > SLIDE_TOL:
                _flag(f"{d.name} slid")
        else:
            _EP["anchor"][d.name] = None  # lifted: a new set-down point is taken when it lands
            h = height_above_support(o)
            if tcps and not gripped(o, tcps):
                _EP["max_dangle"] = max(_EP["max_dangle"], h)
                if h > DANGLE:
                    _flag(f"{d.name} hung by the cord")


def violated() -> bool:
    return bool(_EP["violation"])


def score(objects) -> float:
    """0 if at any step a device slid more than 3 cm along the table or shelf from where it rested (it may only travel
    while lifted; dragging by the cord or pushing counts), hung more than 15 mm above any surface with no gripper on
    it (pulled up by the cord), leaned more than 30 deg or fell; 0 if a gripper is still within 2 cm of a device or the
    cord at the end. Otherwise 1.0 = both devices on the shelf (whole footprint over it, flat on it, leaning <= 10 deg)
    and the whole cord over the shelf lying on it; 0.5 = both devices on the shelf, cord partly off; 0.25 = exactly
    one device on the shelf; else 0."""
    _track(objects)
    if violated() or not released(objects):
        return 0.0
    n = sum(on_shelf(objects[d.name]) for d in DEVICES)
    if n == 2:
        return 1.0 if cord_on_shelf(objects) else 0.5
    return 0.25 if n == 1 else 0.0


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and all(objects[d.name].resting() for d in DEVICES)
                and objects["cord"].resting(0.02, 1.0))


def distance(objects) -> float:
    """Sum of both device centres' distances to the shelf centre line x (m); 1 m once a rule is broken."""
    _track(objects)
    if violated():
        return 1.0
    return float(sum(abs(objects[d.name].pos[0] - SHELF_C[0]) for d in DEVICES))


def progress(objects) -> dict:
    """Stage diagnostic for the paper (not the score)."""
    _track(objects)
    tcps = grippers()
    lifted = [not supported(objects[d.name]) and bool(tcps) and gripped(objects[d.name], tcps) for d in DEVICES]
    return {
        "both_lifted": float(all(lifted)),
        "camera_on_shelf": float(on_shelf(objects[CAMERA.name])),
        "bank_on_shelf": float(on_shelf(objects[BANK.name])),
        "cord_on_shelf": float(cord_on_shelf(objects)),
        "released": float(released(objects)),
        "never_dragged": float(not violated()),
    }


# Seeded variation ranges (applied by a later runner). The pair moves together (the cord welds tie them); the shelf
# is fixed. Kept small so both devices stay in their own arm's reach and the cord stays straight between them.
# The camera, cord and bank must get the SAME dx/dy (the welds are authored at the nominal relative layout).
VARIATION = {
    "cable:cord": {"dx": 0.02, "dy": 0.015, "yaw": 0.0},
    "obj:action_camera": {"dx": 0.02, "dy": 0.015, "yaw": 0.0},
    "obj:power_bank": {"dx": 0.02, "dy": 0.015, "yaw": 0.0},
    "joint": ["cable:cord", "obj:action_camera", "obj:power_bank"],
}

# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------


def _log(ctx, tag=""):
    st = ctx.states()
    cam, bank, c = st[CAMERA.name], st[BANK.name], st["cord"]
    chord = float(np.linalg.norm(c.end_tip - c.start_tip))
    print(f"[t={ctx.t:5.2f}] {tag}: cam={np.round(cam.pos, 3)} bank={np.round(bank.pos, 3)} chord={chord:.3f} "
          f"cam_on={on_shelf(cam)} bank_on={on_shelf(bank)} cord_on={cord_on_shelf(st)} rel={released(st)} "
          f"maxslide={_EP['max_slide']:.3f} maxtilt={_EP['max_tilt']:.1f} maxdangle={_EP['max_dangle']:.3f} "
          f"viol='{_EP['violation']}' score={score(st):.2f}", flush=True)


CAM_DEPTH = 0.014  # TCP this far below a device's top while gripping it
BANK_DEPTH = 0.012
CARRY_Z = SHELF_TOP + 0.045  # device bottoms this high while carried (cord sag clears the shelf edge)
INWARD = 0.008  # each device carried this much closer to the other: 16 mm of slack in the cord
GOAL_DX = SHELF_C[0] - X0  # both devices go straight forward onto the shelf


def _carry(A, xy, size, depth, goal, f_open, f_closed, *, inward_sign, place=True):
    top_rel = size[2] - depth  # TCP height above the device bottom while gripping
    x, y = xy
    yield from A.move((x, y, TOP + size[2] + 0.05), yaw=0.0, finger=f_open, duration=1.6)
    yield from A.move((x, y, TOP + top_rel), yaw=0.0, duration=1.0)
    yield from A.grip(f_closed, 0.8)
    # lift straight up while easing 8 mm towards the other device (slack before any travel)
    y_in = y + inward_sign * INWARD
    yield from A.move((x, y_in, CARRY_Z + top_rel), yaw=0.0, duration=1.2)
    gx, gy = goal[0], goal[1] + inward_sign * INWARD
    yield from A.move((gx, gy, CARRY_Z + top_rel), yaw=0.0, duration=3.0)
    yield from A.move((gx, gy, SHELF_TOP + top_rel + 0.002), yaw=0.0, duration=1.2)
    if not place:
        return
    yield from A.grip(f_open, 0.6)
    yield from A.move((gx, gy, SHELF_TOP + size[2] + 0.08), yaw=0.0, duration=1.0)


def demo_script(ctx):
    """PAIR_DEMO=correct (default) | nothing | drag (left arm carries the camera to the shelf alone; the cord drags
    the bank) | hold (both carried and set on the shelf, but the grippers keep holding) | lone (the right arm lifts the
    bank alone towards the shelf, the camera is pulled along by the cord)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    mode = os.environ.get("PAIR_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    st = ctx.states()
    cam_xy = st[CAMERA.name].pos[:2].copy()
    bank_xy = st[BANK.name].pos[:2].copy()
    cam_goal = cam_xy + np.array([GOAL_DX, 0.0])
    bank_goal = bank_xy + np.array([GOAL_DX, 0.0])
    if mode == "nothing":
        yield from L.wait(5.0)
        _log(ctx, "end")
        return
    if mode == "drag":
        yield from _carry(L, cam_xy, CAM_SIZE, CAM_DEPTH, cam_goal, FINGER_OPEN, FINGER_CLOSED, inward_sign=-1)
        _log(ctx, "camera placed")
        yield from L.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "lone":
        yield from _carry(Rt, bank_xy, BANK_SIZE, BANK_DEPTH, bank_goal, FINGER_OPEN, FINGER_CLOSED, inward_sign=1)
        _log(ctx, "bank placed")
        yield from Rt.wait(1.0)
        _log(ctx, "end")
        return
    place = mode != "hold"
    # Correct: both arms grip their device at the same time, lift together while each eases 8 mm inward (cord slack),
    # carry both straight forward in lockstep (the cord hangs in a shallow loop between them) and set them down on
    # the shelf side by side; then release and retreat upward. Wrist cameras (yaw 0) look forward at the shelf.
    yield from ctx.both(
        _carry(L, cam_xy, CAM_SIZE, CAM_DEPTH, cam_goal, FINGER_OPEN, FINGER_CLOSED, inward_sign=-1, place=place),
        _carry(Rt, bank_xy, BANK_SIZE, BANK_DEPTH, bank_goal, FINGER_OPEN, FINGER_CLOSED, inward_sign=1,
               place=place))
    _log(ctx, "carried")
    if not place:
        yield from L.wait(1.5)
        _log(ctx, "end (held)")
        return
    yield from ctx.both(L.move((0.22, 0.25, TOP + 0.15), duration=1.0), Rt.move((0.22, -0.25, TOP + 0.15),
                                                                                duration=1.0))
    _log(ctx, "end")
