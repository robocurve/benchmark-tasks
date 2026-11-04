"""Hazardous-zone mock-up on the station table. On the far side of the table, out of reach of both arms, sits a small
orange inspection crawler (80 x 60 x 40 mm body, 120 g, low-friction skids; a black camera head on its front). Its
blue 6 mm tether (65 cm) leaves the crawler's rear and lies straight on the table towards the robot's left, passing
8 cm beyond the end post of a fixed grey wall (20 x 2 x 6 cm, with a 30 mm round end post 8 cm tall); a black
handle (50 x 20 x 16 mm) is on the tether's free end, in front of the left arm. The crawler sits directly behind the
wall. In front of the wall (towards the robot), in the wall's lee, a 15 x 15 cm recovery zone is taped on the table in
green. The crawler must be recovered into the zone by its tether only: a direct pull towards the zone drags it into
the wall's end post, so the tether must first be hauled out along the clear lane past the wall end and then the
crawler steered sideways around the end into the zone, without ever touching it with the grippers and without it
striking the wall or tipping over. The left arm alone reaches the tether's free part, the right arm alone reaches the
right side the final pull must come from.

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

INSTRUCTION = ("Recover the orange crawler into the green taped zone using only its blue tether: never touch the "
               "crawler, don't let it hit the grey wall or its post or tip over. Steer it around the wall's end, "
               "then let go of the tether.")

# -- layout (m, world frame: x away from the robot, y left) --------------------------------------------------------
GREY = (0.55, 0.56, 0.58)
GREEN = (0.20, 0.62, 0.30)

WALL_C = np.array([0.47, -0.03])
WALL_SIZE = (0.02, 0.20, 0.06)
WALL_Y0, WALL_Y1 = WALL_C[1] - WALL_SIZE[1] / 2, WALL_C[1] + WALL_SIZE[1] / 2  # -0.13 .. +0.07
WALL_FRONT_X = WALL_C[0] - WALL_SIZE[0] / 2  # 0.46: robot-side face
WALL_BACK_X = WALL_C[0] + WALL_SIZE[0] / 2
POST_C = np.array([WALL_C[0], WALL_Y1])  # end post centred on the wall's left end
POST_R, POST_H = 0.015, 0.08
WALL = Box("wall", pos=(float(WALL_C[0]), float(WALL_C[1]), TOP + WALL_SIZE[2] / 2), size=WALL_SIZE, color=GREY)
POST = Cylinder("wall_post", pos=(float(POST_C[0]), float(POST_C[1]), TOP + POST_H / 2), radius=POST_R,
                height=POST_H, color=(0.42, 0.43, 0.45))

# recovery zone: 15 x 15 cm inside the tape (tape 12 mm wide, 0.6 mm thick, outside the square)
ZONE_C = np.array([0.32, -0.04])
ZONE_HALF = 0.075
TAPE_W, TAPE_T = 0.012, 0.0006


def _zone() -> list[Box]:
    x, y = map(float, ZONE_C)
    h, w = ZONE_HALF, TAPE_W
    z = TOP + TAPE_T / 2
    L = 2 * (h + w)
    return [
        Box("zone_tape_far", pos=(x + h + w / 2, y, z), size=(w, L, TAPE_T), color=GREEN),
        Box("zone_tape_near", pos=(x - h - w / 2, y, z), size=(w, L, TAPE_T), color=GREEN),
        Box("zone_tape_left", pos=(x, y + h + w / 2, z), size=(2 * h, w, TAPE_T), color=GREEN),
        Box("zone_tape_right", pos=(x, y - h - w / 2, z), size=(2 * h, w, TAPE_T), color=GREEN),
    ]


# crawler: free box, rear faces along the tether; camera head welded on top at its front
CR_SIZE = (0.08, 0.06, 0.04)
CR_MASS = 0.115
CR_C = np.array([0.66, -0.02])
CR_FRICTION = 0.3
CAM_SIZE = (0.016, 0.034, 0.016)
CAM_MASS = 0.005
CAM_LOCAL = np.array([CR_SIZE[0] / 2 - CAM_SIZE[0] / 2 - 0.004, 0.0, CR_SIZE[2] / 2 + CAM_SIZE[2] / 2])

R = 0.003  # 6 mm tether
TETHER_LEN = 0.65
TETHER_Z = TOP + R + 0.0005
LANE_PT = np.array([WALL_C[0], 0.17])  # the tether's straight line crosses the wall line here (8.5 cm past the post edge)
REAR_GAP = 0.003  # tether start this far behind the crawler's rear face


def _tether_heading() -> float:
    h = math.atan2(LANE_PT[1] - CR_C[1], LANE_PT[0] - CR_C[0])
    for _ in range(8):
        r = CR_C + (CR_SIZE[0] / 2 + REAR_GAP) * np.array([math.cos(h), math.sin(h)])
        h = math.atan2(LANE_PT[1] - r[1], LANE_PT[0] - r[0])
    return h


T_YAW = _tether_heading()  # ~136.5 deg: from the crawler's rear towards the robot's left
CR_YAW = T_YAW - math.pi  # crawler front points away from the tether
T_DIR = np.array([math.cos(T_YAW), math.sin(T_YAW)])
T_START = CR_C + (CR_SIZE[0] / 2 + REAR_GAP) * T_DIR
CR_Z = TOP + CR_SIZE[2] / 2 + 0.0005

CRAWLER = Box("crawler", pos=(float(CR_C[0]), float(CR_C[1]), CR_Z), size=CR_SIZE, mass=CR_MASS, yaw=CR_YAW,
              color=(0.95, 0.52, 0.08))
_cam_off = np.array([math.cos(CR_YAW) * CAM_LOCAL[0], math.sin(CR_YAW) * CAM_LOCAL[0]])
CAM = Box("crawler_cam", pos=(float(CR_C[0] + _cam_off[0]), float(CR_C[1] + _cam_off[1]), float(CR_Z + CAM_LOCAL[2])),
          size=CAM_SIZE, mass=CAM_MASS, yaw=CR_YAW, color=(0.05, 0.05, 0.06))

HANDLE = Plug(size=(0.05, 0.02, 0.016), mass=0.015, color=(0.06, 0.06, 0.06))
TETHER = cable("tether", start=(float(T_START[0]), float(T_START[1]), TETHER_Z), yaw=T_YAW, length=TETHER_LEN,
               radius=R, anchor_start=False, end_plug=HANDLE, color=(0.10, 0.30, 0.85), bend_stiffness=0.01,
               bend_damping=0.001, cable_mass=0.04)

OBJECTS = [WALL, POST, *_zone(), CRAWLER, CAM, TETHER]
CRAWLER_PARTS = (CRAWLER.name, CAM.name)


# -- task-local physics: tether and camera head welded to the crawler; low-friction crawler skids -------------------


def _install_welds() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_crawler_weld", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != TETHER.name:
            return prim
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdPhysics

        stage = get_current_stage()
        env = str(prim_path).rsplit("/", 1)[0]
        cr = f"{env}/{CRAWLER.name}"
        # tether start tip at the crawler's rear, in the crawler frame (crawler yaw = tether heading - pi)
        j = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/crawler_weld")
        j.CreateBody0Rel().SetTargets([Sdf.Path(cr)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/seg_0")])
        j.CreateLocalPos0Attr(Gf.Vec3f(-(CR_SIZE[0] / 2 + REAR_GAP), 0.0, float(TETHER_Z - CR_Z)))
        j.CreateLocalRot0Attr(Gf.Quatf(0.0, 0.0, 0.0, 1.0))  # yaw pi
        j.CreateLocalPos1Attr(Gf.Vec3f(-TETHER.seg_len / 2, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateExcludeFromArticulationAttr(True)
        # camera head on the crawler's top
        c = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/cam_weld")
        c.CreateBody0Rel().SetTargets([Sdf.Path(cr)])
        c.CreateBody1Rel().SetTargets([Sdf.Path(f"{env}/{CAM.name}")])
        c.CreateLocalPos0Attr(Gf.Vec3f(*map(float, CAM_LOCAL)))
        c.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        c.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        c.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        if os.environ.get("CRAWLER_DEBUG"):
            print(f"[crawler] welds: {cr} <-> {prim_path}/seg_0, {env}/{CAM.name}", flush=True)
        return prim

    spawn._crawler_weld = True
    cablekit._spawn_cable = spawn


def _low_friction_wrap(scene) -> None:
    if getattr(scene.task_object, "_crawler_friction", False):
        return
    prev = scene.task_object

    def task_object(spec):
        if getattr(spec, "name", None) not in CRAWLER_PARTS:
            return prev(spec)
        import isaaclab.sim as sim_utils

        old = scene._grippy
        scene._grippy = lambda: sim_utils.RigidBodyMaterialCfg(
            static_friction=CR_FRICTION, dynamic_friction=CR_FRICTION, friction_combine_mode="min")
        try:
            return prev(spec)
        finally:
            scene._grippy = old

    task_object._crawler_friction = True
    scene.task_object = task_object


def _install_friction() -> None:
    if "yam_arena.scene" in sys.modules:
        mod = sys.modules["yam_arena.scene"]
        cablekit._patch_scene(mod)
        _low_friction_wrap(mod)
        return
    base = cablekit._patch_scene
    if getattr(base, "_crawler_friction", False):
        return

    def patch(scene):
        base(scene)
        _low_friction_wrap(scene)

    patch._crawler_friction = True
    cablekit._patch_scene = patch


_install_welds()
_install_friction()

# -- robot-crawler contact monitor (pattern of tasks_dev/move-live-wire-with-stick) ----------------------------------
# A PhysX rigid-contact view, read after every physics substep: sensors = all robot links, filters = the crawler body
# and its camera head. Any contact (separation <= 1 mm) marks the trial as "crawler touched by the robot".
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
        elif p.rsplit("/", 1)[-1] in CRAWLER_PARTS and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            filters.append(p)
    if not sensors or not filters:
        raise RuntimeError(f"crawler contact monitor: {len(sensors)} robot links, {len(filters)} crawler bodies")
    view = sv.create_rigid_contact_view(sensors, filter_patterns=[filters] * len(sensors),
                                        max_contact_data_count=64 * len(sensors) * len(filters))
    _MON["view"], _MON["stage"] = view, stage
    print(f"[crawler] contact monitor: {len(sensors)} robot links x {filters}", flush=True)


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
            print(f"[crawler] contact monitor error: {e!r}", flush=True)
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
        print(f"[crawler] robot touched the crawler (separation {m * 1000:.2f} mm)", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_crawler_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._crawler_hooked = True
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


# -- per-trial history ---------------------------------------------------------------------------------------------


class _Episode:
    def __init__(self):
        self.last = None
        self.tipped = False
        self.near = False  # a gripper TCP came within NEAR of the crawler
        self.struck = False  # crawler touched the wall or post
        self.max_tilt = 0.0
        self.min_tcp = 1.0
        self.min_wall = 1.0
        self.moved = 0.0


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
    if not getattr(cls.reset, "_crawler_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _MON["view"] = None  # rebuilt on the next physics step
            reset_episode()
            out = base_reset(self, *a, **k)
            _reset_monitor()  # the settle steps hold the arms at READY, far from the crawler
            reset_episode()
            return out

        reset._crawler_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_crawler_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._crawler_hooked = True
        cls._task_step = _task_step


_install_hooks()

# -- geometry helpers ----------------------------------------------------------------------------------------------
TIP_DEG = 45.0
NEAR = 0.03  # m: TCP to crawler body
RELEASE_DIST = 0.035  # m: TCP to tether / handle at the end
STRIKE = 0.003  # m: crawler footprint to wall/post = struck


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def crawler_tilt_deg(cr) -> float:
    cz = _rot(cr.quat_xyzw)[2, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(cz)))))


def crawler_corners(cr) -> np.ndarray:
    """The 8 body corners (world)."""
    Rm = _rot(cr.quat_xyzw)
    h = np.array(CR_SIZE) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return np.asarray(cr.pos) + (signs * h) @ Rm.T


def crawler_yaw(cr) -> float:
    Rm = _rot(cr.quat_xyzw)
    return math.atan2(Rm[1, 0], Rm[0, 0])


def _box_dist(p, cr) -> float:
    """Distance from world point p to the crawler body (0 inside)."""
    Rm = _rot(cr.quat_xyzw)
    local = (np.asarray(p) - np.asarray(cr.pos)) @ Rm
    d = np.maximum(np.abs(local) - np.array(CR_SIZE) / 2, 0.0)
    return float(np.linalg.norm(d))


def _seg_rect_gap(poly: np.ndarray) -> float:
    """2-D gap between the crawler footprint (convex polygon, corners) and the wall rectangle + post circle."""
    # sample the footprint's edges densely and take the minimum distance to wall box / post circle
    pts = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        for t in np.linspace(0.0, 1.0, 21):
            pts.append(a + t * (b - a))
    pts = np.array(pts)
    lo = np.array([WALL_FRONT_X, WALL_Y0])
    hi = np.array([WALL_BACK_X, WALL_Y1])
    d = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    dw = np.linalg.norm(d, axis=1)
    inside = np.all((pts >= lo) & (pts <= hi), axis=1)
    dw[inside] = 0.0
    dp = np.linalg.norm(pts - POST_C, axis=1) - POST_R
    return float(min(dw.min(), dp.min()))


def footprint(cr) -> np.ndarray:
    """The crawler's top-view footprint (4 corners, counter-clockwise) at its current yaw."""
    yaw = crawler_yaw(cr)
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = CR_SIZE[0] / 2, CR_SIZE[1] / 2
    loc = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
    return np.asarray(cr.pos[:2]) + loc @ np.array([[c, s], [-s, c]])


def wall_gap(objects) -> float:
    return _seg_rect_gap(footprint(objects[CRAWLER.name]))


def in_zone(objects) -> bool:
    """Every corner of the crawler's footprint inside the taped square (inner tape edges)."""
    cr = objects[CRAWLER.name]
    xy = crawler_corners(cr)[:, :2]
    return bool(np.all(np.abs(xy - ZONE_C) <= ZONE_HALF))


def in_lee(objects) -> bool:
    """Came around the wall end: whole footprint on the robot's side of the wall's front face and the crawler's centre
    within the wall's left-right extent."""
    cr = objects[CRAWLER.name]
    xy = crawler_corners(cr)[:, :2]
    return bool(np.all(xy[:, 0] < WALL_FRONT_X) and WALL_Y0 <= cr.pos[1] <= WALL_Y1)


def past_wall_end(objects) -> bool:
    cr = objects[CRAWLER.name]
    return bool(np.all(crawler_corners(cr)[:, 0] < WALL_FRONT_X))


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
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


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
    return np.array(out)


def released(objects) -> bool:
    t = objects[TETHER.name]
    pts = _dense(t.polyline())
    if t.end_plug is not None:
        pts = np.vstack([pts, t.end_plug[None]])
    g = grippers()
    return all(float(np.linalg.norm(pts - x, axis=1).min()) >= RELEASE_DIST for x in g)


def _update(objects) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    cr = objects[CRAWLER.name]
    tilt = crawler_tilt_deg(cr)
    _EP.max_tilt = max(_EP.max_tilt, tilt)
    if tilt > TIP_DEG or cr.pos[2] < TOP - 0.02:
        _EP.tipped = True
    for g in grippers():
        d = _box_dist(g, cr)
        _EP.min_tcp = min(_EP.min_tcp, d)
        if d < NEAR:
            _EP.near = True
    gap = wall_gap(objects)
    _EP.min_wall = min(_EP.min_wall, gap)
    if gap <= STRIKE:
        _EP.struck = True
    _EP.moved = max(_EP.moved, float(np.linalg.norm(cr.pos[:2] - CR_C)))


def disqualified() -> bool:
    return _EP.tipped or _EP.near or _EP.struck or bool(_MON["touched"])


def score(objects) -> float:
    """0 if, at any time, the crawler tilted more than 45 deg, a gripper (TCP) came within 3 cm of the crawler body or
    any robot part touched it, or the crawler came within 3 mm of the wall or its post; 0 also if at the end a gripper
    TCP is within 3.5 cm of the tether or handle. Otherwise 1.0 if the crawler's footprint is entirely inside the taped
    square; 0.5 if it has come around the wall end (footprint entirely on the robot's side of the wall's front face,
    centre within the wall's left-right extent) but is not entirely in the square; else 0."""
    _update(objects)
    if disqualified() or not released(objects):
        return 0.0
    if in_zone(objects):
        return 1.0
    if in_lee(objects):
        return 0.5
    return 0.0


def success(objects) -> bool:
    _update(objects)
    cr = objects[CRAWLER.name]
    return bool(score(objects) == 1.0 and cr.resting(0.01, 0.3))


def distance(objects) -> float:
    """Crawler centre to the zone centre (m); 1 m once disqualified."""
    _update(objects)
    if disqualified():
        return 1.0
    return float(np.linalg.norm(objects[CRAWLER.name].pos[:2] - ZONE_C))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    _update(objects)
    t = objects[TETHER.name]
    pts = _dense(t.polyline())
    g = grippers()
    held = any(float(np.linalg.norm(pts - x, axis=1).min()) < 0.02 for x in g)
    return {
        "tether_grasped": int(held or _EP.moved > 0.02),
        "crawler_moving": int(_EP.moved > 0.02),
        "past_wall_end": int(past_wall_end(objects)),
        "around_wall": int(in_lee(objects) or in_zone(objects)),
        "in_zone": int(in_zone(objects)),
        "never_tipped": int(not _EP.tipped),
        "never_touched_or_struck": int(not (_EP.near or _EP.struck or _MON["touched"])),
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner (crawler and tether move together: the tether is welded to the crawler).
VARIATION = {"obj:crawler": {"dx": 0.015, "dy": 0.015, "yaw": 0.0}, "cable:tether": {"dx": 0.015, "dy": 0.015,
                                                                                      "yaw": 0.0},
             "note": "apply the same dx, dy to crawler, crawler_cam and tether"}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
FINGER_TETHER = -0.012  # fingers ~28 mm apart above the 6 mm tether
GRASP_Z = TETHER_Z + 0.010
HI = TOP + 0.07


def _log(ctx, tag=""):
    st = ctx.states()
    cr = st[CRAWLER.name]
    print(f"[t={ctx.t:5.2f}] {tag}: crawler={np.round(cr.pos, 3)} yaw={math.degrees(crawler_yaw(cr)):.0f} "
          f"tilt={crawler_tilt_deg(cr):.1f} (max {_EP.max_tilt:.1f}) wallgap={wall_gap(st) * 1000:.0f}mm "
          f"(min {_EP.min_wall * 1000:.0f}) tcp_min={_EP.min_tcp * 1000:.0f}mm touched={_MON['touched']} "
          f"zone={in_zone(st)} lee={in_lee(st)} rel={released(st)} score={score(st)} "
          f"handle={np.round(st[TETHER.name].end_plug, 3)}", flush=True)


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _cam_yaw(heading: float) -> float:
    """Grip yaw whose wrist camera looks along ``heading`` (towards the crawler) as far as |yaw| <= pi/2 allows."""
    h = (heading + math.pi) % (2 * math.pi) - math.pi
    if abs(h) <= math.pi / 2:
        return h
    if abs(h) <= math.pi / 2 + math.radians(30):
        return math.copysign(math.pi / 2, h)
    return _grip_yaw(h)


def _tether_point(st,s_from_crawler: float):
    """Point on the live tether polyline ``s`` m from the crawler end, and the local heading towards the crawler."""
    poly = st[TETHER.name].polyline()
    seg = np.linalg.norm(np.diff(poly[:, :2], axis=0), axis=1)
    cum = np.r_[0.0, np.cumsum(seg)]
    i = int(np.clip(np.searchsorted(cum, s_from_crawler) - 1, 0, len(seg) - 1))
    t = (s_from_crawler - cum[i]) / max(seg[i], 1e-9)
    p = poly[i] + t * (poly[i + 1] - poly[i])
    d = poly[i, :2] - poly[i + 1, :2]
    return p, math.atan2(d[1], d[0])


def _haul(arm, ctx, s, to, *, cam_heading=None, dur=3.0, f_closed=None):
    """Grip the tether ``s`` m from the crawler, drag the grip point to ``to`` (x, y), open and lift."""
    from yam_arena.constants import FINGER_CLOSED

    fc = FINGER_CLOSED if f_closed is None else f_closed
    p, head = _tether_point(ctx.states(), s)
    yaw = _cam_yaw(head if cam_heading is None else cam_heading)
    yield from arm.move((p[0], p[1], HI), yaw=yaw, finger=FINGER_TETHER, duration=1.5)
    p, _ = _tether_point(ctx.states(), s)
    yield from arm.move((p[0], p[1], GRASP_Z), yaw=yaw, duration=1.0)
    yield from arm.grip(fc, 0.5)
    yield from arm.move((to[0], to[1], GRASP_Z + 0.004), yaw=yaw, duration=dur)
    yield from arm.grip(FINGER_TETHER, 0.4)
    yield from arm.move((to[0], to[1], HI), yaw=yaw, duration=0.6)


def demo_script(ctx):
    """CRAWLER_DEMO=correct (default) | nothing | direct (haul the tether straight towards the zone) | straight
    (haul it straight in along its lane towards the robot, then let go) | push (haul it in, then push the crawler
    into the zone by hand)."""
    mode = os.environ.get("CRAWLER_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(5):
            yield from L.wait(1.0)
            _log(ctx)
        return
    if mode == "probe":  # physics probe: one haul stroke along the lane
        yield from _haul(L, ctx, 0.25, (0.24, 0.34))
        for _ in range(3):
            yield from L.wait(1.0)
            _log(ctx, "after stroke")
        return
    if mode == "direct":  # cheat: haul the tether straight towards the zone (the wall is in the way)
        yield from _haul(L, ctx, 0.25, (0.30, 0.02), dur=4.0)
        _log(ctx, "direct 1")
        yield from _haul(Rt, ctx, 0.12, (0.28, -0.22), cam_heading=math.pi / 2, dur=4.0)
        _log(ctx, "direct 2")
        yield from ctx.both(L.move((0.20, 0.25, HI), duration=1.0), Rt.move((0.20, -0.30, HI), duration=1.0))
        yield from L.wait(1.5)
        _log(ctx, "end")
        return

    # Correct. Left arm: two strokes straight in along the clear lane (the tether's line passes 8.5 cm beyond the
    # wall's end post) until the crawler is past the wall line, then a third stroke that steers it towards the robot's
    # right, around the end post. Right arm: takes the tether where the left arm laid it down and pulls the crawler
    # sideways into the zone (only the right arm reaches that far right), then both let go and move clear.
    yield from _haul(L, ctx, 0.25, LANE_STROKE1)
    _log(ctx, "stroke 1")
    yield from _haul(L, ctx, 0.10, LANE_STROKE2)
    _log(ctx, "stroke 2")
    if mode == "straight":  # cheat: haul straight in along the lane and stop there
        yield from _haul(L, ctx, 0.10, (0.20, 0.40))
        yield from L.move((0.20, 0.25, HI), duration=1.0)
        yield from L.wait(1.5)
        _log(ctx, "end")
        return
    if mode == "push":  # cheat: haul it in, then push the crawler into the zone with the closed gripper
        from yam_arena.constants import FINGER_CLOSED

        cr = ctx.states()[CRAWLER.name].pos
        goal = np.array([ZONE_C[0], ZONE_C[1] + 0.005])
        u = (goal - cr[:2]) / np.linalg.norm(goal - cr[:2])
        gy = _grip_yaw(math.atan2(u[1], u[0]) + math.pi / 2)  # fingers across the push direction
        s = cr[:2] - 0.08 * u
        e = goal - 0.055 * u
        z = TOP + 0.022
        yield from L.move((s[0], s[1], HI), yaw=gy, finger=FINGER_CLOSED, duration=1.5)
        yield from L.move((s[0], s[1], z), yaw=gy, duration=1.0)
        yield from L.move((e[0], e[1], z), yaw=gy, duration=4.0)
        _log(ctx, "pushed")
        yield from L.move((e[0], e[1], HI), yaw=gy, duration=0.8)
        yield from L.move((0.20, 0.25, HI), duration=1.0)
        yield from L.wait(1.5)
        _log(ctx, "end")
        return
    yield from _haul(L, ctx, 0.10, STEER_TO)
    _log(ctx, "steered")
    yield from ctx.both(L.move((0.22, 0.30, HI), duration=1.0), _final_pull(Rt, ctx))
    _log(ctx, "pulled in")
    yield from Rt.move((0.18, -0.32, HI + 0.04), duration=1.0)
    for _ in range(3):
        yield from L.wait(0.7)
        _log(ctx, "end")


LANE_STROKE1 = (0.265, 0.35)
LANE_STROKE2 = (0.30, 0.33)
STEER_TO = (0.28, 0.0)


def _final_pull(arm, ctx):
    """Grip the tether ~12 cm from the crawler and pull towards the robot's right until the crawler's centre is at the
    zone centre's y (closed loop), then let go."""
    from yam_arena.constants import FINGER_CLOSED

    st = ctx.states()
    p, head = _tether_point(st, 0.12)
    yaw = float(np.clip(head, -math.pi / 2, math.pi / 2))  # wrist camera towards the crawler / wall end
    yield from arm.move((p[0], p[1], HI), yaw=yaw, finger=FINGER_TETHER, duration=1.5)
    p, _ = _tether_point(ctx.states(), 0.12)
    yield from arm.move((p[0], p[1], GRASP_Z), yaw=yaw, duration=1.0)
    yield from arm.grip(FINGER_CLOSED, 0.5)
    g = np.array([p[0], p[1]])
    for _ in range(12):
        cr = ctx.states()[CRAWLER.name].pos
        err = cr[1] - ZONE_C[1]
        if err <= 0.005:
            break
        step = min(0.04, err)
        g = g + np.array([(ZONE_C[0] - cr[0]) * 0.3, -step])
        yield from arm.move((g[0], g[1], GRASP_Z + 0.004), yaw=yaw, duration=max(0.4, step / 0.05))
    yield from arm.wait(0.4)
    yield from arm.grip(FINGER_TETHER, 0.4)
    yield from arm.move((g[0], g[1], HI), yaw=yaw, duration=0.6)
