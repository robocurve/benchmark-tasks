"""A car front-end rig on the station table: a steel hood (140 x 100 cm, 14 kg) on its two rear hinges, gas struts,
and a grille cross-member with the hood's safety-catch striker loop.

The hood has been popped from the cabin: its primary latch is open and its pop-up spring holds it 3 cm up at the
front edge, where the safety hook under the hood (near the middle of the front edge) catches the striker loop and
stops it rising further. The hook swings on a sprung pivot; its release lever ends in a black tab just behind the
front edge, reached through the 3 cm gap. Pushing the tab 2 cm towards the robot's left swings the hook off the
striker; the hood can then be lifted. Below about 12 deg of opening (front edge about 21 cm up) it takes about 6 N
at the front edge to lift; above 15 deg the gas struts carry it on their own to the full-open stop at 50 deg.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "The hood has been popped from inside. Open it fully."

HOOD_COL = (0.72, 0.74, 0.77)  # silver paint
BODY_COL = (0.16, 0.17, 0.19)  # black grille / cross-member
FENDER_COL = (0.70, 0.72, 0.75)
ENGINE_COL = (0.35, 0.36, 0.38)
HOOK_COL = (0.08, 0.08, 0.08)
STRIKER_COL = (0.55, 0.56, 0.58)

# -- hood geometry (metres; hood local frame: x from the front edge to the hinges, y left, z up out of the skin) --------
HOOD_L, HOOD_W, HOOD_T = 1.00, 1.40, 0.025
HINGE = np.array([1.35, 0.0, TOP + 0.30])  # hinge axis (along y) at the hood's mid-thickness, rear corners
FRONT_CLOSED = np.array([0.35, TOP + 0.125 + HOOD_T / 2])  # (x, z) of the front edge mid-thickness, hood shut
_d = np.array([HINGE[0], HINGE[2]]) - FRONT_CLOSED
HOOD_R = float(np.hypot(*_d))  # hinge to front edge (1.013 m)
PHI_C = math.degrees(math.atan2(_d[1], _d[0]))  # hood slope when shut (deg, rising to the rear)
HOOD_L = HOOD_R  # the box runs from the front edge to the hinge line
POP = 0.030  # popped: front edge 3 cm up
A0 = math.degrees(math.asin(POP / HOOD_R))  # opening angle when popped (1.70 deg)
A_OPEN = 50.0  # full-open stop
A_SELF = 15.0  # above this the struts lift it on their own
HOOD_MASS = 14.0

# -- hinge drive: gas struts (+ pop-up spring) as a joint drive whose set point keeps the net torque profile -----------
HINGE_K = 1.0  # N m / deg
HINGE_C = 0.30  # N m s / deg (strut damping)
NET_POP = float(os.environ.get("HOOD_NETPOP", "2.0"))  # N m opening, below A0 + 0.9 deg: the pop-up spring presses the hood up against the safety hook
NET_HOLD = float(os.environ.get("HOOD_NETHOLD", "-6.0"))  # N m (about 6 N at the front edge) needed to lift, from A0 + 1.8 to 12 deg
NET_SELF = float(os.environ.get("HOOD_NETSELF", "5.0"))  # N m opening above 15 deg: the struts take over
# measured: the joint drive delivers about 6 N m less than its set point asks (hinge + drive compliance); added back
STRUT_CAL = float(os.environ.get("HOOD_CAL", "6.0"))


def weight_torque(a_deg: float) -> float:
    """Closing torque of the hood's weight about the hinge (N m), opening angle a."""
    return HOOD_MASS * 9.81 * HOOD_R / 2 * math.cos(math.radians(PHI_C - a_deg))


def net_torque(a_deg: float) -> float:
    """Designed net opening torque (struts + pop spring - weight), N m."""
    p0, p1 = A0 + 0.9, A0 + 1.8
    if a_deg <= p0:
        return NET_POP
    if a_deg <= p1:
        return NET_POP + (NET_HOLD - NET_POP) * (a_deg - p0) / (p1 - p0)
    if a_deg <= 12.0:
        return NET_HOLD
    if a_deg <= A_SELF:
        return NET_HOLD + (NET_SELF - NET_HOLD) * (a_deg - 12.0) / (A_SELF - 12.0)
    return NET_SELF


def _rot_y(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def hood_R(a_deg: float) -> np.ndarray:
    """Hood local-to-world rotation at opening angle a: local x rises to the rear at PHI_C - a."""
    return _rot_y(-(PHI_C - a_deg))


def hood_centre(a_deg: float) -> np.ndarray:
    return HINGE + hood_R(a_deg) @ np.array([-HOOD_L / 2, 0.0, 0.0])


def hood_point(local, a_deg: float) -> np.ndarray:
    """World position of a hood-local point (origin at the hood box centre)."""
    return hood_centre(a_deg) + hood_R(a_deg) @ np.asarray(local, float)


def _quat_xyzw(R) -> tuple:
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


def _R_from_quat(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# -- safety hook and lever (hood local frame; underside of the skin at z = -HOOD_T/2) --------------------------------
UNDER = -HOOD_T / 2
FRONT_X = -HOOD_L / 2
PIVOT_L = np.array([FRONT_X + 0.060, 0.0, UNDER - 0.006])  # hook pivot, 6 cm behind the front edge
ARM_R = 0.030  # hook arm: pivot to the hook post (along -y)
POST_H = 0.030
LIP = 0.014
LEVER_TIP_X = FRONT_X + 0.014  # the tab sits 1.4 cm behind the front edge
REL_DEG = 19.5  # hook clear of the striker beyond this lever swing (lip end passes the bar)
LEVER_MAX = 30.0
HOOK_K = 0.004  # N m / deg return spring (about 2 N at the tab, 20 deg)
HOOK_C = 0.0005
HOOK_PRELOAD = 3.0  # deg: spring set point beyond the rest stop, so the hook rests on its stop with a little preload
HOOK_MASS = 0.12

# hook parts (hood-local centres and sizes); the first is the spawned root box: the lever bar
_post_x, _post_y = PIVOT_L[0], -ARM_R
HOOK_PARTS = [
    # name, centre, size
    ("lever", ((PIVOT_L[0] + LEVER_TIP_X) / 2, 0.0, PIVOT_L[2]), (PIVOT_L[0] - LEVER_TIP_X + 0.006, 0.010, 0.005)),
    ("tab", (LEVER_TIP_X, 0.0, PIVOT_L[2] - 0.010), (0.010, 0.016, 0.018)),
    ("arm", (_post_x, -ARM_R / 2, PIVOT_L[2]), (0.012, ARM_R + 0.006, 0.005)),
    ("post", (_post_x, _post_y, PIVOT_L[2] - POST_H / 2), (0.008, 0.008, POST_H)),
    ("lip", (_post_x + LIP / 2 - 0.002, _post_y, PIVOT_L[2] - POST_H + 0.002), (LIP, 0.008, 0.004)),
]
HOOK_ROOT_L = np.array(HOOK_PARTS[0][1])
LIP_TOP_L = np.array([_post_x, _post_y, PIVOT_L[2] - POST_H + 0.004])
# striker loop crossbar (fixed, along world y) sits on the lip's top, over its rear part, at the popped pose
BAR_R = 0.004
_bar_l = np.array([_post_x + 0.006, _post_y, LIP_TOP_L[2] + BAR_R + 0.0005])
BAR_W = hood_point(_bar_l, A0)

# -- fixed body ----------------------------------------------------------------------------------------------------
MEMBER_TOP = TOP + 0.12
MEMBER_X = 0.39  # front face of the grille cross-member: the hood's front edge overhangs it by 4.4 cm
OBJ_FIXED = [
    # grille cross-member, with an opening in the middle for the hook (two halves and a low middle)
    Box("member_l", pos=(MEMBER_X + 0.035, 0.375, TOP + 0.06), size=(0.07, 0.65, 0.12), color=BODY_COL),
    Box("member_r", pos=(MEMBER_X + 0.035, -0.375, TOP + 0.06), size=(0.07, 0.65, 0.12), color=BODY_COL),
    Box("member_m", pos=(MEMBER_X + 0.035, 0.0, TOP + 0.03), size=(0.07, 0.10, 0.06), color=BODY_COL),
    Box("striker", pos=tuple(float(v) for v in BAR_W), size=(2 * BAR_R, 0.040, 2 * BAR_R), color=STRIKER_COL),
    Box("fender_l", pos=(0.85, 0.735, TOP + 0.09), size=(1.0, 0.05, 0.18), color=FENDER_COL),
    Box("fender_r", pos=(0.85, -0.735, TOP + 0.09), size=(1.0, 0.05, 0.18), color=FENDER_COL),
    Box("engine", pos=(0.82, 0.0, TOP + 0.07), size=(0.50, 0.70, 0.14), color=ENGINE_COL),
    Box("cowl", pos=(1.42, 0.0, TOP + 0.14), size=(0.10, 1.40, 0.28), color=BODY_COL),
]
STRIKER_LEGS = [(float(BAR_W[0]), float(BAR_W[1]) + s * 0.018) for s in (-1, 1)]

HOOD_POS0 = hood_centre(A0)
HOOD_Q0 = _quat_xyzw(hood_R(A0))
HOOK_POS0 = hood_point(HOOK_ROOT_L, A0)


class HoodSpec(Box):
    """The hood (custom spawner: the rig's joints are authored with it)."""


class HookSpec(Box):
    """The safety hook + lever (custom spawner: extra colliders)."""


HOOD = HoodSpec("hood", pos=tuple(float(v) for v in HOOD_POS0), size=(HOOD_L, HOOD_W, HOOD_T), mass=HOOD_MASS,
                color=HOOD_COL)
HOOK = HookSpec("safety_hook", pos=tuple(float(v) for v in HOOK_POS0), size=tuple(HOOK_PARTS[0][2]), mass=HOOK_MASS,
                color=HOOK_COL)
OBJECTS = [*OBJ_FIXED, HOOK, HOOD]  # hood last: its spawner authors the joints once both bodies exist

TWIN_EXTRA = [
    {"name": "hood_hinge", "kind": "joint", "real": "hood's two rear hinges", "note": "opening 0-50 deg"},
    {"name": "gas_struts", "kind": "joint", "real": "two hood gas struts + pop-up spring",
     "note": "net 12 N m opening below 2.6 deg (hood held on its hook), about 6 N at the front edge to lift up to "
     "12 deg, opening on their own above 15 deg"},
    {"name": "hook_pivot", "kind": "joint", "real": "safety hook pivot with return spring",
     "note": "lever swings 0-30 deg; hook clear of the striker beyond 19.5 deg"},
]

_P: dict = {}


# -- USD authoring -----------------------------------------------------------------------------------------------------
def _cube(stage, path, centre, size, color, collide=True):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    g.CreateDisplayColorAttr([Gf.Vec3f(*map(float, color))])
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
    x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    if collide:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        pc = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
        pc.CreateContactOffsetAttr(0.002)
        pc.CreateRestOffsetAttr(0.0)
    return g


def _build_hook(stage, root):
    for name, c, s in HOOK_PARTS[1:]:
        _cube(stage, f"{root}/{name}", np.asarray(c) - HOOK_ROOT_L, s, HOOK_COL)


def _author_rig(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)
    q_id = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    Rh = hood_R(A0)
    qh = HOOD_Q0

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def drive(j, kind, k, c, target=0.0):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(k)
        d.CreateDampingAttr(c)
        d.CreateTargetPositionAttr(target)
        d.CreateMaxForceAttr(1.0e7)

    # hinge: world -> hood, axis world y; joint position = opening angle - A0
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{parent}/hood/hinge")
    j.CreateLocalPos0Attr(v(W + HINGE))
    j.CreateLocalRot0Attr(q_id)
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/hood")])
    j.CreateLocalPos1Attr(v(Rh.T @ (HINGE - HOOD_POS0)))
    j.CreateLocalRot1Attr(Gf.Quatf(float(qh[3]), -float(qh[0]), -float(qh[1]), -float(qh[2])))
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(-A0)
    j.CreateUpperLimitAttr(A_OPEN - A0)
    j.CreateExcludeFromArticulationAttr(True)
    drive(j, "angular", HINGE_K, HINGE_C, (weight_torque(A0) + net_torque(A0) + STRUT_CAL) / HINGE_K)
    _P["hinge"] = str(j.GetPath())
    # hook pivot: hood -> hook, axis = hood normal (both spawned with the hood's orientation)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{parent}/hood/hook_pivot")
    j.CreateBody0Rel().SetTargets([Sdf.Path(f"{parent}/hood")])
    j.CreateLocalPos0Attr(v(PIVOT_L))
    j.CreateLocalRot0Attr(q_id)
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/safety_hook")])
    j.CreateLocalPos1Attr(v(PIVOT_L - HOOK_ROOT_L))
    j.CreateLocalRot1Attr(q_id)
    j.CreateAxisAttr("Z")
    j.CreateLowerLimitAttr(-LEVER_MAX)
    j.CreateUpperLimitAttr(0.0)
    j.CreateExcludeFromArticulationAttr(True)
    drive(j, "angular", HOOK_K, HOOK_C, float(os.environ.get("HOOD_PROBE_HOOK", HOOK_PRELOAD)))
    _P["hook"] = str(j.GetPath())
    # visuals: striker loop legs, hinge brackets, a hood front lip line
    for i, (x, y) in enumerate(STRIKER_LEGS):
        zb = MEMBER_TOP - 0.06
        h = float(BAR_W[2]) - zb
        _cube(stage, f"{parent}/member_m/striker_leg{i}", (x - (MEMBER_X + 0.035), y, zb + h / 2 - (TOP + 0.03)), (0.008, 0.006, h),
              STRIKER_COL, collide=False)
    for s in (-1, 1):
        _cube(stage, f"{parent}/cowl/hinge_bracket{s + 1}", (HINGE[0] - 1.42, s * 0.66, HINGE[2] - (TOP + 0.14) - 0.02),
              (0.06, 0.02, 0.06), (0.25, 0.25, 0.27), collide=False)


def _spawn_compound(prim_path, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    stage = get_current_stage()
    name = str(prim_path).rsplit("/", 1)[1]
    if name == HOOK.name:
        _build_hook(stage, str(prim_path))
    else:
        _author_rig(stage, str(prim_path).rsplit("/", 1)[0])
    return prim


def _register_module():
    if __name__ not in sys.modules:  # configclass needs this module registered (task files load unregistered)
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _compound_object(spec):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg, MassPropertiesCfg, RigidBodyPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class CompoundCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_compound)

    fr = (0.6, 0.5) if isinstance(spec, HoodSpec) else (0.3, 0.25)  # painted steel hood; greased steel hook
    cfg = CompoundCfg(size=tuple(spec.size), rigid_props=RigidBodyPropertiesCfg(),
                      mass_props=MassPropertiesCfg(mass=spec.mass),
                      collision_props=CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                      physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=fr[0], dynamic_friction=fr[1]),
                      visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.35))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(float(x) for x in spec.pos), rotation_xyzw=HOOD_Q0))


def _patch_scene(scene) -> None:
    if getattr(scene, "_hood_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, (HoodSpec, HookSpec)):
            return _compound_object(spec)
        if spec.name == "striker":  # the striker bar's underside lies square to the hook's lip (no sideways push)
            yaw_q = scene._yaw_xyzw
            scene._yaw_xyzw = lambda _yaw: HOOD_Q0
            try:
                return orig(spec)
            finally:
                scene._yaw_xyzw = yaw_q
        return orig(spec)

    scene.task_object = task_object
    scene._hood_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _hood = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
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
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_hood", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- episode state -------------------------------------------------------------------------------------------------------
_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.a_max = A0
        self.freed = False  # hook swung clear of the striker at some point
        self.pinched = False
        self.why = ""
        self.armed = False  # lifted 10 cm and not yet settled again: a sinking hood here can catch fingers
        self.prev_a = None  # hood angle at the previous control step
        self.still_s = 0.0  # how long the hood has been still (s)
        self.prev_pts = None  # gripper points at the previous control step
        self.park_s = 0.0  # how long both grippers have been clear and still (s)
        self.last_set = None
        self.last = None


_EP = _Episode()


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _struts(a: float) -> None:
    """Gas struts + pop spring: keep the hinge drive's set point so it gives weight + designed net torque."""
    if not math.isfinite(a):
        return
    target = (a - A0) + (weight_torque(a) + net_torque(a) + STRUT_CAL) / HINGE_K
    if _EP.last_set is not None and abs(target - _EP.last_set) < 0.05:
        return
    _EP.last_set = target
    try:
        _stage().GetPrimAtPath(_P["hinge"]).GetAttribute("drive:angular:physics:targetPosition").Set(float(target))
    except Exception as e:  # noqa: BLE001
        print(f"[hood] strut update failed: {e!r}", flush=True)


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_hood_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._hood_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_hood_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                st = object_states(self._env, [HOOD.name, HOOK.name])
                _struts(hood_angle(st))
                _update(st)
                _watch_pinch(st, 1.0 / float(self.info.control_hz))
                _watch_park(st, 1.0 / float(self.info.control_hz))
            except Exception as e:  # noqa: BLE001
                print(f"[hood] step hook failed: {e!r}", flush=True)
            return out

        _task_step._hood_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- readouts --------------------------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def hood_angle(objects) -> float:
    """Opening angle (deg): 0 shut, A0 popped, 50 full open. From the hood's centre about the hinge line."""
    p = objects[HOOD.name].pos
    if not _finite(p):
        return float("nan")
    phi = math.degrees(math.atan2(HINGE[2] - p[2], HINGE[0] - p[0]))
    return PHI_C - phi


def front_rise(objects) -> float:
    """Height of the front edge above its shut position (m)."""
    a = hood_angle(objects)
    if not math.isfinite(a):
        return float("nan")
    return HOOD_R * (math.sin(math.radians(a)))


def lever_angle(objects) -> float:
    """Lever swing (deg, >= 0) of the hook relative to the hood: 0 at rest, 19.5 hook clear."""
    qh, qk = objects[HOOD.name].quat_xyzw, objects[HOOK.name].quat_xyzw
    if not (_finite(qh) and _finite(qk)):
        return float("nan")
    Rrel = _R_from_quat(qh).T @ _R_from_quat(qk)
    return -math.degrees(math.atan2(Rrel[1, 0], Rrel[0, 0]))


# gap under the hood's front: hood-local box in which a gripper is pinched if the hood comes down
GAP_DEPTH = 0.15


def _grip_points() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        pts.append(np.asarray(tcp_world(rob), dtype=np.float64) - origin)
        pts.append(np.asarray(pad_tips_world(rob), dtype=np.float64) - origin)
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
    return pts


def _hood_frame(objects):
    p = np.asarray(objects[HOOD.name].pos, float)
    R = _R_from_quat(objects[HOOD.name].quat_xyzw)
    return p, R


def in_gap(objects, pts) -> bool:
    """A gripper point under the hood's front 15 cm, above the cross-member top."""
    p, R = _hood_frame(objects)
    for q in pts:
        if not _finite(q):
            continue
        l = R.T @ (q - p)
        if l[0] < FRONT_X + GAP_DEPTH and l[0] > FRONT_X - 0.005 and abs(l[1]) < HOOD_W / 2 and l[2] < UNDER and \
                q[2] > MEMBER_TOP:
            return True
    return False


def _update(objects) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    a = hood_angle(objects)
    b = lever_angle(objects)
    if math.isfinite(a):
        _EP.a_max = max(_EP.a_max, a)
    if math.isfinite(b) and b >= REL_DEG:
        _EP.freed = True


SINK_RATE = 0.2  # deg/s: the hood counts as coming down when its angle falls faster (front edge ~3.5 mm/s)
STILL_RATE = 0.1  # deg/s: slower than this counts as still
SETTLE_S = 1.0  # s: still this long, low down = settled (the pinch watch disarms until the next 10 cm lift)


def _watch_pinch(objects, dt: float) -> None:
    """Once per control step: latch a pinch if the hood, coming back down from a 10 cm lift, sinks below 7 cm over its
    popped height with a gripper point under its front 15 cm (above the cross-member top)."""
    a = hood_angle(objects)
    prev, _EP.prev_a = _EP.prev_a, a
    if not math.isfinite(a) or prev is None or not math.isfinite(prev) or _EP.pinched:
        return
    rate = (a - prev) / dt
    if a >= A_LIFTED:
        _EP.armed = True
    _EP.still_s = _EP.still_s + dt if abs(rate) < STILL_RATE else 0.0
    if _EP.still_s >= SETTLE_S - 1e-6 and a <= A_SETTLED:
        _EP.armed = False  # the hood has settled back down: reaching under it again is a fresh attempt
    if _EP.armed and a <= A_PINCH and rate < -SINK_RATE and in_gap(objects, _grip_points()):
        _EP.pinched, _EP.why = True, f"gripper under the hood as it came down ({a:.1f} deg, {rate:.1f} deg/s)"
        print(f"[hood] pinch: {_EP.why}", flush=True)


LIFT_10CM = 0.10
A_LIFTED = math.degrees(math.asin(min(1.0, (POP + LIFT_10CM) / HOOD_R)))  # 7.4 deg
A_PINCH = math.degrees(math.asin((POP + 0.07) / HOOD_R))  # front edge back below 7 cm over its popped height (5.7 deg)
A_SETTLED = math.degrees(math.asin((POP + 0.03) / HOOD_R))  # back down within 3 cm of its popped height (3.4 deg)
OPEN_WIN = 1.0
CLEAR = 0.15


MEMBER_LO = np.array([MEMBER_X, -0.70, TOP])
MEMBER_HI = np.array([MEMBER_X + 0.07, 0.70, MEMBER_TOP])


def _dist_obox(q, centre, R, half) -> float:
    l = R.T @ (np.asarray(q) - centre)
    d = np.maximum(np.abs(l) - half, 0.0)
    return float(np.linalg.norm(d))


def released(objects) -> bool:
    """Every gripper point at least 15 cm from the hood (oriented box), the hook and lever, and the cross-member."""
    pts = _grip_points()
    if not pts:
        return False
    p, R = _hood_frame(objects)
    half = np.array([HOOD_L, HOOD_W, HOOD_T]) / 2
    hk = objects[HOOK.name]
    for q in pts:
        if not _finite(q):
            return False
        if _dist_obox(q, p, R, half) < CLEAR:
            return False
        lo, hi = np.asarray(hk.low), np.asarray(hk.high)
        if float(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0))) < CLEAR:
            return False
        if float(np.linalg.norm(np.maximum(np.maximum(MEMBER_LO - q, q - MEMBER_HI), 0.0))) < CLEAR:
            return False  # cross-member: its 7 x 140 x 12 cm box (front face, rear face, ends and top all bound it)
    return True


PARK_SPEED = 0.02  # m/s: every gripper point slower than this counts as still
PARK_S = 0.5  # s: clear and still this long = parked


def _watch_park(objects, dt: float) -> None:
    pts = _grip_points()
    prev, _EP.prev_pts = _EP.prev_pts, pts
    if not pts or prev is None or len(prev) != len(pts):
        _EP.park_s = 0.0
        return
    speed = max(float(np.linalg.norm(np.asarray(p) - np.asarray(q))) for p, q in zip(pts, prev)) / dt
    _EP.park_s = _EP.park_s + dt if (speed < PARK_SPEED and released(objects)) else 0.0


def parked(objects) -> bool:
    """Every gripper point 15 cm clear of the hood, hook and cross-member, slower than 2 cm/s, held 0.5 s."""
    return bool(released(objects) and _EP.park_s >= PARK_S - 1e-6)


def hood_open(objects) -> bool:
    a = hood_angle(objects)
    return bool(math.isfinite(a) and a >= A_OPEN - OPEN_WIN)


def gated(objects) -> bool:
    _update(objects)
    return bool(_EP.pinched)


def score(objects) -> float:
    """0 if, coming back down from a 10 cm lift (before settling), the hood sank below 7 cm over its popped height with
    a gripper under its front 15 cm. Otherwise 0.25 hook swung clear
    of the striker; 0.5 hood lifted 10 cm above its popped height; 1.0 hood at its full-open stop with every gripper
    point 15 cm from hood, hook and cross-member, both grippers still (under 2 cm/s) for 0.5 s."""
    if gated(objects):
        return 0.0
    if hood_open(objects) and parked(objects):
        return 1.0
    if _EP.a_max >= A_LIFTED:
        return 0.5
    if _EP.freed:
        return 0.25
    return 0.0


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    a = hood_angle(objects)
    return float(max(0.0, (A_OPEN - (a if math.isfinite(a) else 0.0))) / A_OPEN)


def progress(objects) -> dict:
    _update(objects)
    return {"hood_deg": hood_angle(objects), "front_rise_cm": front_rise(objects) * 100,
            "lever_deg": lever_angle(objects), "freed": float(_EP.freed), "a_max": _EP.a_max,
            "open": float(hood_open(objects)), "released": float(released(objects)), "parked": float(parked(objects)),
            "pinched": float(_EP.pinched)}


VARIATION: dict = {}
