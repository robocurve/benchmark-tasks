"""Beach-seine mock-up on the station table. Three free cans (50 mm across, 60 mm tall, 60 g, smooth steel, friction
~0.5) stand in a row across the table 30 cm in front of the robot, 12 cm apart (left, middle, right). A free blue
8 mm braided rope (40 cm, black 20 x 15 x 15 mm end caps) lies straight across the table behind them, 40 cm in front of
the robot, its ends reaching 8 cm beyond each outer can, one in front of each arm. A green landing bay (15 cm wide,
11 cm deep, a flat decal) is marked on the table at the front centre, 9.5-20.5 cm in front of the robot. The robot must
haul all three cans into the bay with the rope alone, like a beach seine: never touching a can with a gripper, never
tipping one, never lifting the rope over a can, and letting go of the rope at the end. The bay is narrower than the
row of cans, so pulling both ends straight back leaves the outer cans outside it: the two wings must be closed
(pursed) as they come in, so the cans gather into a cluster inside the bight.

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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Haul all three cans into the green bay using only the blue rope, the way a beach seine is hauled. "
               "Never touch a can with a gripper, never tip one over and never lift the rope over a can. "
               "Let go of the rope when you are done.")

# -- layout (m; x away from the robot, y left) ------------------------------------------------------------------------
GREEN = (0.20, 0.62, 0.30)

# landing bay: flat green decal, 15 cm across (y) x 11 cm deep (x), its top 0.2 mm above the table (cans slide on)
BAY_C = np.array([0.15, 0.0])
BAY_W, BAY_D = 0.15, 0.11  # across (y), deep (x)
BAY_T = 0.0004
BAY = Box("bay", pos=(float(BAY_C[0]), float(BAY_C[1]), TOP - BAY_T / 2 + 0.0002), size=(BAY_D, BAY_W, BAY_T),
          color=GREEN)

# cans: free cylinders, smooth (friction 0.5 against everything)
CAN_R, CAN_H, CAN_MASS = 0.025, 0.06, 0.06
CAN_FRICTION = 0.5
CAN_X = 0.30
CAN_Y = (0.12, 0.0, -0.12)
CAN_COLORS = ((0.80, 0.12, 0.10), (0.92, 0.70, 0.10), (0.85, 0.40, 0.08))
CANS = [Cylinder(f"can_{tag}", pos=(CAN_X, y, TOP + CAN_H / 2 + 0.0005), radius=CAN_R, height=CAN_H, mass=CAN_MASS,
                 color=col)
        for tag, y, col in zip(("left", "mid", "right"), CAN_Y, CAN_COLORS)]
CAN_NAMES = tuple(c.name for c in CANS)

# seine rope: free, straight across behind the cans, black end caps
R = 0.004
ROPE_LEN = 0.40
ROPE_X = 0.40
CAP = Plug(size=(0.020, 0.015, 0.015), mass=0.008, color=(0.06, 0.06, 0.06))
ROPE_Z = TOP + CAP.size[2] / 2 + 0.0005  # caps rest on the table; the rope settles 3.5 mm onto it
ROPE = cable("rope", start=(ROPE_X, -ROPE_LEN / 2, ROPE_Z), yaw=math.pi / 2, length=ROPE_LEN, radius=R,
             anchor_start=False, start_plug=CAP, end_plug=CAP, color=(0.10, 0.30, 0.85), bend_stiffness=0.01,
             bend_damping=0.001, cable_mass=0.04)

OBJECTS = [BAY, *CANS, ROPE]

# -- task-local physics: smooth cans ------------------------------------------------------------------------------------


def _low_friction_wrap(scene) -> None:
    if getattr(scene.task_object, "_seine_friction", False):
        return
    prev = scene.task_object

    def task_object(spec):
        if getattr(spec, "name", None) not in CAN_NAMES:
            return prev(spec)
        import isaaclab.sim as sim_utils

        old = scene._grippy
        scene._grippy = lambda: sim_utils.RigidBodyMaterialCfg(
            static_friction=CAN_FRICTION, dynamic_friction=CAN_FRICTION, friction_combine_mode="min")
        try:
            return prev(spec)
        finally:
            scene._grippy = old

    task_object._seine_friction = True
    scene.task_object = task_object


def _install_friction() -> None:
    if "yam_arena.scene" in sys.modules:
        mod = sys.modules["yam_arena.scene"]
        cablekit._patch_scene(mod)
        _low_friction_wrap(mod)
        return
    base = cablekit._patch_scene
    if getattr(base, "_seine_friction", False):
        return

    def patch(scene):
        base(scene)
        _low_friction_wrap(scene)

    patch._seine_friction = True
    cablekit._patch_scene = patch


_install_friction()

# -- robot-can contact monitor (pattern of tasks_dev/move-live-wire-with-stick) -------------------------------------
# A PhysX rigid-contact view, read after every physics substep (240 Hz): sensors = all robot links, filters = the three
# cans. Any contact (separation <= 1 mm) marks the trial as "can touched by the robot". Rope-can contact is not in the
# view, so hauling the cans with the rope never trips it.
TOUCH_SEP = 0.001
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
_MON: dict = {"touched": False, "view": None, "stage": None, "err": None, "polls": 0}


def _reset_monitor() -> None:
    _MON.update(touched=False, polls=0)


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover
        pass


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _make_view(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    sensors, filters = [], []
    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" not in p:
            continue
        if any(r in p for r in _ROBOT_PRIMS) and "PhysxContactReportAPI" in prim.GetAppliedSchemas():
            sensors.append(p)
        elif p.rsplit("/", 1)[-1] in CAN_NAMES and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            filters.append(p)
    if not sensors or not filters:
        raise RuntimeError(f"seine contact monitor: {len(sensors)} robot links, {len(filters)} cans")
    view = sv.create_rigid_contact_view(sensors, filter_patterns=[filters] * len(sensors),
                                        max_contact_data_count=64 * len(sensors) * len(filters))
    _MON["view"], _MON["stage"] = view, stage
    print(f"[seine] contact monitor: {len(sensors)} robot links x {len(filters)} cans", flush=True)


def _poll_contacts(mgr) -> None:
    try:
        if _MON["view"] is None or _MON.get("stage") is None:
            _make_view(mgr)
        view = _MON["view"]
        if view is None:
            return
        _, _, _, sep, counts, starts = view.get_contact_data(dt=mgr.get_physics_dt())
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[seine] contact monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    counts = _np(counts).reshape(-1)
    if not counts.any():
        return
    starts = _np(starts).reshape(-1)
    sep = _np(sep).reshape(-1)
    idx = np.concatenate([np.arange(s, s + c) for s, c in zip(starts[counts > 0], counts[counts > 0])])
    m = float(sep[idx].min())
    if m <= TOUCH_SEP and not _MON["touched"]:
        _MON["touched"] = True
        print(f"[seine] robot touched a can (separation {m * 1000:.2f} mm)", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_seine_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._seine_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
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


def _patch_physics() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())


# -- per-trial history ------------------------------------------------------------------------------------------------


class _Episode:
    def __init__(self):
        self.last = None
        self.tipped = False  # a can tilted more than TIP_DEG
        self.over = False  # the rope passed over the top of a can
        self.max_tilt = 0.0


_EP = _Episode()
_ENV: dict = {"env": None}


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _install_hooks() -> None:
    _patch_physics()
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_seine_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _MON["view"] = None  # rebuilt on the next physics step
            reset_episode()
            out = base_reset(self, *a, **k)
            _reset_monitor()  # the settle steps hold the arms at READY, far from the cans
            reset_episode()
            return out

        reset._seine_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_seine_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._seine_hooked = True
        cls._task_step = _task_step


_install_hooks()

# -- geometry ---------------------------------------------------------------------------------------------------------
TIP_DEG = 30.0
RELEASE_DIST = 0.05  # m: each gripper TCP at least this far from every rope point and cap at the end
OVER_R = CAN_R - 0.003  # a rope centre-line point within this top-view distance of a can's axis is over the can


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def can_axis(c) -> np.ndarray:
    return _rot(c.quat_xyzw)[:, 2]


def can_tilt_deg(c) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(can_axis(c)[2])))))


def can_base(c) -> np.ndarray:
    """Centre of the can's bottom face (world)."""
    return np.asarray(c.pos) - can_axis(c) * CAN_H / 2


def can_in_bay(c) -> bool:
    """The can's whole base disc lies inside the bay rectangle (top view), base on the table."""
    b = can_base(c)
    if not np.isfinite(b).all():
        return False
    return bool(abs(b[0] - BAY_C[0]) <= BAY_D / 2 - CAN_R and abs(b[1] - BAY_C[1]) <= BAY_W / 2 - CAN_R
                and b[2] <= TOP + 0.01)


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.array(out)


def _rope_pts(objects) -> np.ndarray:
    r = objects[ROPE.name]
    pts = _dense(r.polyline())
    caps = [p[None] for p in (r.start_plug, r.end_plug) if p is not None]
    return np.vstack([pts, *caps])


def rope_over_can(objects) -> bool:
    """Some rope point lies over a can in top view (closer to the can's axis than its wall, so not beside it)."""
    pts = _rope_pts(objects)[:, :2]
    for n in CAN_NAMES:
        c = objects[n]
        a = can_base(c)[:2]
        b = (np.asarray(c.pos) + can_axis(c) * CAN_H / 2)[:2]
        ab = b - a
        t = np.clip(((pts - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0) if ab @ ab > 1e-12 else np.zeros(len(pts))
        d = np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)
        if float(d.min()) < OVER_R:
            return True
    return False


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (env-origin corrected); [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64).reshape(-1)[:3] - origin
            for k in ("left_robot", "right_robot")]


def released(objects) -> bool:
    pts = _rope_pts(objects)
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def _update(objects) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    for n in CAN_NAMES:
        c = objects[n]
        if not np.isfinite(c.pos).all():
            _EP.tipped = True
            continue
        t = can_tilt_deg(c)
        _EP.max_tilt = max(_EP.max_tilt, t)
        if t > TIP_DEG:
            _EP.tipped = True
    if not _EP.over and rope_over_can(objects):
        _EP.over = True


def disqualified() -> bool:
    return _EP.tipped or _EP.over or bool(_MON["touched"])


def cans_in_bay(objects) -> int:
    return sum(can_in_bay(objects[n]) for n in CAN_NAMES)


def score(objects) -> float:
    """0 if, at any time, a robot link touched a can, a can tilted more than 30 deg, or the rope passed over a can;
    0 also if at the end a gripper TCP is within 5 cm of the rope or its caps. Otherwise 1.0 if all three cans have
    their whole base inside the bay, 0.5 if exactly two do, else 0."""
    _update(objects)
    if disqualified() or not released(objects):
        return 0.0
    n = cans_in_bay(objects)
    return 1.0 if n == 3 else 0.5 if n == 2 else 0.0


def success(objects) -> bool:
    _update(objects)
    return bool(score(objects) == 1.0 and all(objects[n].resting(0.01, 0.3) for n in CAN_NAMES))


def distance(objects) -> float:
    """Sum over cans of how far (m) each can's base centre is outside its allowed region in the bay; 1 m once
    disqualified."""
    _update(objects)
    if disqualified():
        return 1.0
    d = 0.0
    for n in CAN_NAMES:
        b = can_base(objects[n])
        ex = max(0.0, abs(b[0] - BAY_C[0]) - (BAY_D / 2 - CAN_R))
        ey = max(0.0, abs(b[1] - BAY_C[1]) - (BAY_W / 2 - CAN_R))
        d += math.hypot(ex, ey)
    return float(d)


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    _update(objects)
    xs = [float(objects[n].pos[0]) for n in CAN_NAMES]
    ys = [float(objects[n].pos[1]) for n in CAN_NAMES]
    return {
        "cans_moving": int(min(xs) < CAN_X - 0.03),
        "gathered": round(float(np.clip((0.24 - (max(ys) - min(ys))) / (0.24 - 0.08), 0.0, 1.0)), 3),
        "hauled_in": round(float(np.clip((CAN_X - max(xs)) / (CAN_X - (BAY_C[0] + BAY_D / 2 - CAN_R)), 0.0, 1.0)), 3),
        "cans_in_bay": cans_in_bay(objects) / 3.0,
        "never_tipped": int(not _EP.tipped),
        "never_over": int(not _EP.over),
        "never_touched": int(not _MON["touched"]),
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner (the closed-loop demo tolerates these).
VARIATION = {
    "cable:rope": {"dx": 0.01, "dy": 0.01, "yaw": 0.03},
    "obj:can_left": {"dx": 0.01, "dy": 0.01, "yaw": 0.0},
    "obj:can_mid": {"dx": 0.01, "dy": 0.01, "yaw": 0.0},
    "obj:can_right": {"dx": 0.01, "dy": 0.01, "yaw": 0.0},
}

TWIN_EXTRA = [
    {"name": "bay", "kind": "decal", "size_mm": [110, 150], "colour": "green", "pose": [0.15, 0.0, 0.0],
     "note": "flat vinyl/tape decal, cans slide over it"},
]

# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
HI = TOP + 0.08


def _log(ctx, tag=""):
    st = ctx.states()
    r = st[ROPE.name]
    cans = " ".join(f"{n[4:]}=({st[n].pos[0]:.3f},{st[n].pos[1]:+.3f})" for n in CAN_NAMES)
    print(f"[t={ctx.t:5.2f}] {tag}: {cans} tilt={_EP.max_tilt:.1f} touched={_MON['touched']} over={_EP.over} "
          f"inbay={cans_in_bay(st)} rel={released(st)} score={score(st)} "
          f"caps={np.round(r.start_plug[:2], 3)} {np.round(r.end_plug[:2], 3)} polls={_MON['polls']}", flush=True)
    if os.environ.get("SEINE_ROPELOG"):
        print("   rope:", " ".join(f"({p[0]:.2f},{p[1]:+.2f})" for p in r.points[::3]), flush=True)


def demo_script(ctx):
    """SEINE_CHEAT= (correct, default) | none (do nothing) | hold (do the task, keep holding the caps) | straight
    (pull both ends straight back without closing the wings) | push (push the cans into the bay with the grippers)
    | onearm (haul with the left end only)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    cheat = os.environ.get("SEINE_CHEAT", "")
    L, Rt = ctx.left, ctx.right
    yl, yr = -math.pi / 2, math.pi / 2  # fingers close across the caps; both wrist cameras look in at the cans
    _log(ctx, "start")
    if cheat == "none":
        for _ in range(5):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return
    if cheat == "push":  # sweep the cans into the bay with the closed grippers
        z = TOP + 0.03
        yield from ctx.both(L.move((0.36, 0.13, HI), yaw=0.0, finger=FINGER_CLOSED, duration=1.5),
                            Rt.move((0.36, -0.13, HI), yaw=0.0, finger=FINGER_CLOSED, duration=1.5))
        yield from ctx.both(L.move((0.36, 0.13, z), yaw=0.0, duration=1.0), Rt.move((0.36, -0.13, z), yaw=0.0,
                                                                                    duration=1.0))
        yield from ctx.both(L.move((0.20, 0.04, z), yaw=0.0, duration=4.0), Rt.move((0.20, -0.04, z), yaw=0.0,
                                                                                    duration=4.0))
        yield from ctx.both(L.move((0.20, 0.25, HI), duration=1.0), Rt.move((0.20, -0.25, HI), duration=1.0))
        yield from L.wait(1.5)
        _log(ctx, "end")
        return

    def grasp(arm, cap, yaw):
        yield from arm.move((cap[0], cap[1], HI), yaw=yaw, finger=FINGER_OPEN, duration=1.5)
        yield from arm.move((cap[0], cap[1], cap[2] + 0.010), yaw=yaw, duration=1.0)
        yield from arm.grip(FINGER_CLOSED, 0.5)

    st = ctx.states()
    r = st[ROPE.name]
    cl, cr = r.end_plug, r.start_plug  # end cap is at +y (left)
    yield from ctx.both(grasp(L, cl, yl), grasp(Rt, cr, yr))
    _log(ctx, "caps grasped")
    gz = float(cl[2]) + 0.012  # caps dragged on the table

    def path(arm, pts, yaw, dur):
        for p in pts:
            yield from arm.move((p[0], p[1], gz), yaw=yaw, duration=dur)

    if cheat == "straight":
        yield from ctx.both(path(L, [(0.10, float(cl[1]))], yl, 9.0), path(Rt, [(0.10, float(cr[1]))], yr, 9.0))
    elif cheat == "onearm":
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((0.20, -0.30, HI), yaw=yr, duration=1.0)
        yield from path(L, [(0.30, 0.25), (0.20, 0.12), (0.10, 0.04)], yl, 3.0)
    else:
        WP = [(0.32, 0.22), (0.24, 0.17), (0.16, 0.12), (0.10, 0.085)]
        for wp in WP:
            yield from ctx.both(path(L, [wp], yl, 3.0), path(Rt, [(wp[0], -wp[1])], yr, 3.0))
            _log(ctx, f"wp {wp}")
    _log(ctx, "hauled")
    if cheat not in ("straight", "onearm"):
        # cinch: draw both ends apart along the near edge, so the bight hauls the gathered cans further in (closed loop)
        y = 0.085
        for _ in range(24):
            st = ctx.states()
            if cans_in_bay(st) == 3:
                break
            y = min(y + 0.008, 0.26)
            yield from ctx.both(L.move((0.10, y, gz), yaw=yl, duration=0.6), Rt.move((0.10, -y, gz), yaw=yr,
                                                                                     duration=0.6))
            _log(ctx, f"cinch y={y:.3f}")
    for _ in range(2):
        yield from L.wait(0.5)
        _log(ctx, "settle")
    if cheat == "hold":
        for _ in range(4):
            yield from L.wait(0.5)
        _log(ctx, "end")
        return
    # let go: open, rise clear of the rope, park wide
    yield from ctx.both(L.grip(FINGER_OPEN, 0.5), Rt.grip(FINGER_OPEN, 0.5))
    yield from ctx.both(L.move((0.12, 0.12, HI + 0.04), yaw=yl, duration=1.0),
                        Rt.move((0.12, -0.12, HI + 0.04), yaw=yr, duration=1.0))
    yield from ctx.both(L.move((0.20, 0.30, HI + 0.04), yaw=yl, duration=1.0),
                        Rt.move((0.20, -0.30, HI + 0.04), yaw=yr, duration=1.0))
    for _ in range(4):
        yield from L.wait(0.5)
        _log(ctx, "end")
