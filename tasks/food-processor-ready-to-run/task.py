"""Set up a 3-cup mini food processor (Cuisinart Mini-Prep Plus DLC-2A class) so that it will run.

The motor base stands on the counter with the work bowl set on it but not yet turned to lock. The S-blade stands on
the counter beside it, and the lid lies upside up next to it. The machine only runs when the bowl is turned
counterclockwise (seen from above) until it locks on the base, the blade sits down on the bowl's centre post, and the lid
is turned counterclockwise on the bowl until its tab pushes the safety-switch plunger in the bowl's handle home.

The base is light (0.95 kg on rubber feet), and the bowl's lock is stiff: turning the bowl without holding the base
turns the base with it. So one hand holds the base still while the other turns the bowl. The blade is handled only by
its plastic hub.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Put the food processor together so it is ready to run."

WHITE = (0.93, 0.93, 0.91)
BOWL_COL = (0.80, 0.84, 0.88)  # clear plastic, shown light grey-blue
LID_COL = (0.86, 0.89, 0.92)
STEEL = (0.72, 0.73, 0.76)
HUB_COL = (0.20, 0.20, 0.22)
RED = (0.85, 0.12, 0.10)
TAPE = (0.20, 0.45, 0.85)

deg = math.pi / 180.0

# -- layout (metres; z above the counter) -----------------------------------------------------------------------------
C = np.array([0.33, 0.0])  # bowl axis
BASE_SIZE = (0.18, 0.14, 0.075)
BASE_POS = (0.31, 0.0, BASE_SIZE[2] / 2)  # extends 4.4 cm in front of the bowl: the front deck
BASE_MASS = 0.95

FLOOR_T = 0.004
FLOOR_Z = BASE_SIZE[2] + FLOOR_T / 2  # bowl body origin: centre of its floor
FLOOR_TOP = BASE_SIZE[2] + FLOOR_T
R_IN, WALL_T = 0.062, 0.004  # bowl inside radius, wall thickness
R_OUT = R_IN + WALL_T
WALL_H = 0.090
RIM = FLOOR_TOP + WALL_H  # 0.169 above the counter
N_WALL = 20
BOWL_MASS = 0.20
LOCK_DEG = 30.0  # bowl turned this far (counterclockwise) = locked
SNAP_DEG = 26.0  # past this its lugs drop into the lock notch (a spring snaps it to the stop)
BOWL_K = 0.042  # N m / deg: the lugs ride up the base ramps; resisting torque peaks just before the snap
BOWL_SNAP_K = 0.05  # N m / deg: holds it locked
BOWL_DAMP = 0.002  # N m s / deg

H0 = -110.0  # handle direction (deg from +x) with the bowl unlocked; locked it is at -80
HANDLE_R = (R_OUT, 0.106)
HANDLE_W = 0.030
POST_H = 0.045  # centre post above the floor
POST_W = 0.011

# safety-switch plunger on the handle top, slides along the rim direction (counterclockwise = pushed in)
PL_LEN, PL_W, PL_H = 0.020, 0.008, 0.006
PL_R = 0.084
PL_S0 = -0.002  # plunger centre, along the counterclockwise tangent from the handle's centre line
PL_TRAVEL = 0.006
PL_K = 15.0  # N/m return spring (light: about 0.1 N at full travel)
PL_IN = 0.004  # pushed in at least this far = the switch is made

# lid: disc on the rim, skirt inside the bowl, two bayonet lugs under two rim ledges, tab over the handle
LID_R, LID_T = 0.068, 0.006
LID_Z = RIM + LID_T / 2  # seated
SKIRT_R, SKIRT_H = 0.053, 0.012
LUG = (0.007, 0.010, 0.003)  # radial, tangential, height
LUG_RC = 0.0565
TAB = (0.030, 0.015, 0.005)
TAB_RC = 0.083
LEDGE_RIN = 0.0575
LEDGE_SPAN = 30.0
LEDGE_TOP = 0.003  # ledge top below the rim
LEDGE_T = 0.0035  # ledge thickness: 2.5 mm clear above the lid's lugs once the lid sits on the rim
LID_MASS = 0.10
TAB_REL = math.degrees(math.atan2(PL_S0 - PL_LEN / 2 + PL_TRAVEL - TAB[1] / 2, TAB_RC)) + 2.6  # tab angle from the handle at home: the tab holds the plunger at its 6 mm stop (+2.6 deg: the square tab touches with its corner)
LID_TURN = 30.0  # the lid goes on this far short of home (lugs in the ledge gaps) and is turned on
BAR = (0.050, 0.014, 0.025)  # raised grip bar on the lid
BAR_REL = -18.8  # bar direction relative to the tab

# S-blade: capped plastic hub with a square bore, two steel blades
HUB_W, HUB_BORE, HUB_H, CAP_T = 0.034, 0.022, 0.058, 0.005
BLADE = (0.040, 0.014, 0.002)
BLADE_Z = 0.008  # blade centre above the hub bottom
BLADE_MASS = 0.06
BLADE_XY = (0.22, -0.22)
LID_XY = (0.13, -0.10)

LID0_YAW = H0 + LOCK_DEG + TAB_REL - LID_TURN  # lid start yaw: ready to drop on in the ledge gaps


def _rz(a_deg: float) -> np.ndarray:
    a = a_deg * deg
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _pol(r: float, a_deg: float) -> np.ndarray:
    return np.array([r * math.cos(a_deg * deg), r * math.sin(a_deg * deg)])


# objects ------------------------------------------------------------------------------------------------------------
BASE = Box("motor_base", pos=(BASE_POS[0], BASE_POS[1], TOP + BASE_POS[2]), size=BASE_SIZE, mass=BASE_MASS,
           color=WHITE)
BOWL = Cylinder("work_bowl", pos=(float(C[0]), float(C[1]), TOP + FLOOR_Z), radius=R_OUT, height=FLOOR_T,
                mass=BOWL_MASS, color=BOWL_COL)
_pl_xy = C + _pol(PL_R, H0) + PL_S0 * _pol(1.0, H0 + 90.0)
PL_POS0 = np.array([_pl_xy[0], _pl_xy[1], TOP + RIM + 0.0005 + PL_H / 2])  # plunger at rest, bowl unlocked
PLUNGER = Box("switch_plunger", pos=tuple(float(v) for v in PL_POS0), size=(PL_LEN, PL_W, PL_H), mass=0.01, color=RED,
              yaw=(H0 + 90.0) * deg)
BLADE_CAP = Box("blade_hub", pos=(BLADE_XY[0], BLADE_XY[1], TOP + HUB_H + CAP_T / 2), size=(HUB_W, HUB_W, CAP_T), mass=BLADE_MASS, color=HUB_COL)
LID = Cylinder("lid", pos=(LID_XY[0], LID_XY[1], TOP + 0.015 + 0.0005), radius=LID_R, height=LID_T, mass=LID_MASS,
               color=LID_COL, yaw=LID0_YAW * deg)
GRIP = Box("lid_grip", pos=(LID_XY[0], LID_XY[1], TOP + 0.015 + 0.0005 + LID_T / 2 + BAR[2] / 2), size=BAR, mass=0.01,
           color=LID_COL, yaw=(LID0_YAW + BAR_REL) * deg)


class TapeMark(Box):
    """A tape mark on the counter (fixed, flat); the last one's spawner also authors the joints and compound parts."""


_bx0, _bx1 = BASE_POS[0] - BASE_SIZE[0] / 2, BASE_POS[0] + BASE_SIZE[0] / 2
_by = BASE_SIZE[1] / 2
MARKS = [Box("mark_fl", pos=(_bx0 - 0.01, _by + 0.01, TOP + 0.0005), size=(0.03, 0.006, 0.001), color=TAPE),
         Box("mark_fr", pos=(_bx0 - 0.01, -_by - 0.01, TOP + 0.0005), size=(0.03, 0.006, 0.001), color=TAPE)]
HOOK = TapeMark("mark_bl", pos=(_bx1 + 0.01, _by + 0.01, TOP + 0.0005), size=(0.03, 0.006, 0.001), color=TAPE)

OBJECTS = [BASE, BOWL, PLUNGER, BLADE_CAP, LID, GRIP, *MARKS, HOOK]


# compound parts (body-frame boxes: centre, size, yaw deg), added as colliders under each body -----------------------
def bowl_parts() -> list[tuple]:
    z0 = -FLOOR_T / 2  # body origin = floor centre
    parts = []
    w = 2 * R_OUT * math.tan(math.pi / N_WALL)
    for i in range(N_WALL):
        a = H0 + (i + 0.5) * 360.0 / N_WALL
        c = _pol(R_IN + WALL_T / 2, a)
        parts.append((f"wall{i}", (c[0], c[1], z0 + FLOOR_T + WALL_H / 2), (WALL_T, w, WALL_H), a, BOWL_COL))
    home_tab = H0 + TAB_REL
    for k, side in enumerate((90.0, -90.0)):
        for j in (-1, 0, 1):
            a = home_tab + side + j * LEDGE_SPAN / 3
            rc = (LEDGE_RIN + R_IN) / 2
            c = _pol(rc, a)
            parts.append((f"ledge{k}{j + 1}", (c[0], c[1], z0 + FLOOR_T + WALL_H - LEDGE_TOP - LEDGE_T / 2),
                          (R_IN - LEDGE_RIN + 0.001, 2 * rc * math.tan(LEDGE_SPAN / 6 * deg) + 0.0005, LEDGE_T), a,
                          BOWL_COL))
    for k, a in enumerate((0.0, 45.0)):
        parts.append((f"post{k}", (0.0, 0.0, z0 + FLOOR_T + POST_H / 2), (POST_W, POST_W, POST_H), a, WHITE))
    hr = (HANDLE_R[0] + HANDLE_R[1]) / 2
    c = _pol(hr, H0)
    hh = WALL_H - 0.011
    parts.append(("handle", (c[0], c[1], z0 + FLOOR_T + 0.010 + hh / 2), (HANDLE_R[1] - HANDLE_R[0], HANDLE_W, hh), H0,
                  BOWL_COL))
    return parts


def lid_parts() -> list[tuple]:
    z0 = -LID_T / 2
    parts = [("skirt", (0.0, 0.0, z0 - SKIRT_H / 2), ("cyl", SKIRT_R, SKIRT_H), 0.0, LID_COL)]
    for k, a in enumerate((90.0, -90.0)):
        c = _pol(LUG_RC, a)
        parts.append((f"lug{k}", (c[0], c[1], z0 - 0.009 - LUG[2] / 2), LUG, a, LID_COL))
    c = _pol(TAB_RC, 0.0)
    parts.append(("tab", (c[0], c[1], z0 + TAB[2] / 2), TAB, 0.0, LID_COL))
    return parts


def blade_parts() -> list[tuple]:
    zc = -CAP_T / 2 - HUB_H / 2  # cap is the body; walls hang below it
    t = (HUB_W - HUB_BORE) / 2
    o = HUB_W / 2 - t / 2
    parts = [("hub_xp", (o, 0.0, zc), (t, HUB_BORE, HUB_H), 0.0, HUB_COL),
             ("hub_xn", (-o, 0.0, zc), (t, HUB_BORE, HUB_H), 0.0, HUB_COL),
             ("hub_yp", (0.0, o, zc), (HUB_W, t, HUB_H), 0.0, HUB_COL),
             ("hub_yn", (0.0, -o, zc), (HUB_W, t, HUB_H), 0.0, HUB_COL)]
    zb = -CAP_T / 2 - HUB_H + BLADE_Z
    for k, s in enumerate((1, -1)):
        parts.append((f"blade{k}", (s * (HUB_W / 2 + BLADE[0] / 2 - 0.001), 0.0, zb), BLADE, 8.0 * s, STEEL))
    return parts


TWIN_EXTRA = [
    {"name": "bowl_lock", "kind": "joint", "real": "the bowl's twist lock on the base",
     "note": "revolute 0-30 deg about the bowl axis; ramp spring (0.042 N m/deg) resists up to 1.1 N m, snaps locked past 26 deg"},
    {"name": "switch_plunger", "kind": "joint", "real": "the safety-switch plunger in the bowl handle",
     "note": "prismatic 6 mm along the rim, 15 N/m return spring; pushed only by the lid tab's contact"},
    {"name": "lid", "kind": "weld", "real": "lid with skirt, two bayonet lugs, tab and grip bar",
     "note": "compound rigid body; lugs pass the ledge gaps and slide under the two ledges"},
    {"name": "blade", "kind": "weld", "real": "S-blade", "note": "capped square-bore hub with two steel blades"},
]

# -- Isaac side ---------------------------------------------------------------------------------------------------------
_P: dict = {}


def _author(stage, parent: str) -> None:
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    mat = UsdShade.Material.Define(stage, f"{parent}/{HOOK.name}/fp_material")
    mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mapi.CreateStaticFrictionAttr(1.2)
    mapi.CreateDynamicFrictionAttr(1.0)

    def add(body: str, name: str, centre, size, yaw_deg, color):
        bp = f"{parent}/{body}/{name}"
        if size[0] == "cyl":
            g = UsdGeom.Cylinder.Define(stage, bp)
            g.CreateRadiusAttr(float(size[1]))
            g.CreateHeightAttr(float(size[2]))
            g.CreateAxisAttr("Z")
            x = UsdGeom.Xformable(g)
            x.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        else:
            g = UsdGeom.Cube.Define(stage, bp)
            g.CreateSizeAttr(1.0)
            x = UsdGeom.Xformable(g)
            x.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
            x.AddRotateZOp().Set(float(yaw_deg))
            x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        pc = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
        pc.CreateContactOffsetAttr(0.004)
        pc.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    for p in bowl_parts():
        add(BOWL.name, *p)
    for p in lid_parts():
        add(LID.name, *p)
    for p in blade_parts():
        add(BLADE_CAP.name, *p)

    def q(yaw_deg):
        h = yaw_deg * deg / 2
        return Gf.Quatf(math.cos(h), 0.0, 0.0, math.sin(h))

    def local(o, p_world):
        d = np.asarray(p_world, float) - np.array(o.pos)
        return tuple(_rz(-math.degrees(o.yaw)) @ d)

    def joint(kind, name, b0, b1, p_world, axis="Z", lo=None, hi=None, rot0=0.0, rot1=0.0):
        j = kind.Define(stage, f"{parent}/{HOOK.name}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{parent}/{b0.name}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{parent}/{b1.name}")])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, local(b0, p_world))))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, local(b1, p_world))))
        j.CreateLocalRot0Attr(q(rot0))
        j.CreateLocalRot1Attr(q(rot1))
        if kind is not UsdPhysics.FixedJoint:
            j.CreateAxisAttr(axis)
            if lo is not None:
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
        j.CreateExcludeFromArticulationAttr(True)
        return j

    def drive(j, kind, k, c, target=0.0):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(k)
        d.CreateDampingAttr(c)
        d.CreateTargetPositionAttr(target)

    # bowl on the base: turns about its axis between unlocked (0) and locked (LOCK_DEG)
    j = joint(UsdPhysics.RevoluteJoint, "bowl_lock", BASE, BOWL, (C[0], C[1], TOP + BASE_SIZE[2]), lo=0.0,
              hi=LOCK_DEG)
    drive(j, "angular", BOWL_K, BOWL_DAMP, 0.0)
    _P["bowl_lock"] = str(j.GetPath())
    # plunger on the bowl handle: slides along the counterclockwise tangent; frames turned so the joint's X is it
    j = joint(UsdPhysics.PrismaticJoint, "plunger", BOWL, PLUNGER, PLUNGER.pos, axis="X", lo=0.0, hi=PL_TRAVEL,
              rot0=H0 + 90.0, rot1=0.0)
    drive(j, "linear", PL_K, 2.0, -0.002)
    # grip bar welded on the lid
    joint(UsdPhysics.FixedJoint, "weld_grip", LID, GRIP, GRIP.pos, rot0=BAR_REL, rot1=0.0)


def _spawn_hook(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _hook_object(spec: TapeMark):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class HookCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_hook)

    cfg = HookCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(),
                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.6))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_fp_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, TapeMark):
            return _hook_object(spec)
        return orig(spec)

    scene.task_object = task_object
    scene._fp_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _fp = True

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
    elif not any(getattr(f, "_fp", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- episode state: the bowl's lock spring, blade-touch watch -----------------------------------------------------------
_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.locked = False  # the bowl's lugs have dropped into the lock notch
        self.mode = None
        self.touched_blade = False
        self.steps = 0
        self.prev_pts = None  # gripper points at the previous control step
        self.still_s = 0.0  # how long both grippers have been clear and still (s)


_EP = _Episode()


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def _set_bowl(mode: str) -> None:
    if _EP.mode == mode:
        return
    _EP.mode = mode
    d = _stage().GetPrimAtPath(_P["bowl_lock"])
    k, tgt = {"ramp": (BOWL_K, 0.0), "locked": (BOWL_SNAP_K, LOCK_DEG + 5.0)}[mode]
    d.GetAttribute("drive:angular:physics:stiffness").Set(k)
    d.GetAttribute("drive:angular:physics:targetPosition").Set(tgt)


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    _MON["view"] = None
    _MON["body"] = None
    _MON["hits"] = 0
    if not _P:
        return
    try:
        _set_bowl("locked" if _EP.locked else "ramp")
    except Exception:  # noqa: BLE001
        pass


def _step(objects, dt: float = 0.05) -> None:
    _EP.steps += 1
    a = bowl_angle(objects)
    if math.isfinite(a) and a >= SNAP_DEG:
        _EP.locked = True
    _set_bowl("locked" if _EP.locked else "ramp")
    pts = gripper_points()
    hits, _MON["hits"] = _MON["hits"], 0
    if not _EP.touched_blade and hits:
        _EP.touched_blade = True
        print(f"[fp] robot touched the blade steel at step {_EP.steps}", flush=True)
    prev, _EP.prev_pts = _EP.prev_pts, pts
    if not pts or prev is None or len(prev) != len(pts):
        _EP.still_s = 0.0
        return
    speed = max(float(np.linalg.norm(np.asarray(p, float) - np.asarray(q, float))) for p, q in zip(pts, prev)) / dt
    _EP.still_s = _EP.still_s + dt if (speed < PARK_SPEED and _clear(pts)) else 0.0


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_fp_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._fp_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_fp_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            try:  # update the latched state first, so this step's score sees it
                from yam_arena.scene import object_states

                _step(object_states(self._env, [o.name for o in OBJECTS[:6]]), 1.0 / float(self.info.control_hz))
            except Exception as e:  # noqa: BLE001
                print(f"[fp] step failed: {e!r}", flush=True)
            return base(self, *a, **k)

        _task_step._fp_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- geometry readouts --------------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def _rotm(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(c) for c in q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _yaw(o) -> float:
    """Yaw (deg) of a body's x axis."""
    r = _rotm(o.quat_xyzw)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _tilt(o) -> float:
    """How far (deg) a body's z axis leans from vertical."""
    r = _rotm(o.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, r[2, 2]))))


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def bowl_angle(objects) -> float:
    """How far (deg, counterclockwise) the bowl is turned on the base from its unlocked position."""
    b, w = objects[BASE.name], objects[BOWL.name]
    if not (_finite(b.quat_xyzw) and _finite(w.quat_xyzw)):
        return float("nan")
    return _wrap(_yaw(w) - _yaw(b))


def base_shift(objects) -> tuple[float, float]:
    """(how far the base's centre moved from its marks, m; how far it turned, deg)."""
    b = objects[BASE.name]
    if not (_finite(b.pos) and _finite(b.quat_xyzw)):
        return float("inf"), float("inf")
    d = float(np.linalg.norm(np.asarray(b.pos[:2], float) - np.array(BASE_POS[:2])))
    return d, abs(_wrap(_yaw(b)))


def _bowl_frame(objects):
    """Bowl body origin (world, from the base, which carries it on the joint) and its rotation."""
    b, w = objects[BASE.name], objects[BOWL.name]
    rb = _rotm(b.quat_xyzw)
    origin = np.asarray(b.pos, float) + rb @ (np.array([C[0], C[1], TOP + FLOOR_Z]) - np.array(BASE.pos))
    return origin, _rotm(w.quat_xyzw)


def bowl_locked(objects) -> bool:
    a = bowl_angle(objects)
    return bool(math.isfinite(a) and a >= LOCK_DEG - 2.0)


def blade_seated(objects) -> bool:
    """Hub over the centre post (axis within 5 mm), standing upright with its bottom within 3 mm of the bowl floor."""
    h = objects[BLADE_CAP.name]
    if not (_finite(h.pos) and _finite(h.quat_xyzw)):
        return False
    origin, _ = _bowl_frame(objects)
    centre = np.asarray(h.pos, float)  # the blade body is symmetric about its axis: its bbox centre is on the axis
    off = float(np.linalg.norm(centre[:2] - origin[:2]))
    bottom = float(h.low[2])
    floor_top = origin[2] + FLOOR_T / 2
    return bool(off <= 0.005 and _tilt(h) <= 5.0 and abs(bottom - floor_top) <= 0.003)


def plunger_in(objects) -> float:
    """How far (m) the switch plunger is pushed in along the rim from its rest stop."""
    p = objects[PLUNGER.name]
    if not _finite(p.pos):
        return float("nan")
    origin, rw = _bowl_frame(objects)
    rest = origin + rw @ (PL_POS0 - np.array([C[0], C[1], TOP + FLOOR_Z]))
    t = rw @ np.array([math.cos((H0 + 90) * deg), math.sin((H0 + 90) * deg), 0.0])
    return float(np.dot(np.asarray(p.pos, float) - rest, t))


def lid_turn(objects) -> float:
    """Lid angle (deg) relative to the bowl, 0 = home (tab against the pushed-in plunger)."""
    g, w = objects[GRIP.name], objects[BOWL.name]
    if not (_finite(g.quat_xyzw) and _finite(w.quat_xyzw)):
        return float("nan")
    return _wrap(_yaw(g) - BAR_REL - (_yaw(w) + H0 + TAB_REL))


def lid_seated(objects) -> bool:
    """Lid centred on the bowl (within 1 cm) and down on its rim (grip bar within 3 mm of its seated height), level."""
    g = objects[GRIP.name]
    if not (_finite(g.pos) and _finite(g.quat_xyzw)):
        return False
    origin, _ = _bowl_frame(objects)
    seated_z = origin[2] + (LID_Z + LID_T / 2 + BAR[2] / 2 - FLOOR_Z)
    off = float(np.linalg.norm(np.asarray(g.pos[:2], float) - origin[:2]))
    return bool(off <= 0.010 and abs(float(g.pos[2]) - seated_z) <= 0.003 and _tilt(g) <= 3.0)


def lid_home(objects) -> bool:
    t = lid_turn(objects)
    pin = plunger_in(objects)
    return bool(lid_seated(objects) and math.isfinite(t) and abs(t) <= 4.0 and math.isfinite(pin) and pin >= PL_IN)


# -- grippers -------------------------------------------------------------------------------------------------------------
def gripper_points() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

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


TOUCH = 0.001  # a contact point this close to a blade's steel box is on the steel (not on the hub)
TOUCH_SEP = 0.001  # contact separation at or below this counts as touching


def _blade_frame(objects):
    h = objects[BLADE_CAP.name]
    if not (_finite(h.pos) and _finite(h.quat_xyzw)):
        return None
    r = _rotm(h.quat_xyzw)
    # bbox centre of the blade body sits at the middle of cap-top..hub-bottom; cap centre is above it by HUB_H/2
    return r, np.asarray(h.pos, float) - r @ np.array([0.0, 0.0, -(HUB_H) / 2])


def on_steel(objects, p) -> bool:
    """Is point p (env frame) on either steel blade (within TOUCH of its box)?"""
    f = _blade_frame(objects)
    if f is None or not _finite(p):
        return False
    r, body = f
    for name, c, size, yaw_d, _ in blade_parts():
        if not name.startswith("blade"):
            continue
        d = (r @ _rz(yaw_d)).T @ (np.asarray(p, float) - (body + r @ np.array(c)))
        if float(np.linalg.norm(np.maximum(np.abs(d) - np.array(size) / 2, 0.0))) < TOUCH:
            return True
    return False


def _on_steel_body(r, body, p) -> bool:
    """Is world point p on either steel blade of the blade body at pose (r, body origin = cap centre)?"""
    for name, c, size, yaw_d, _ in blade_parts():
        if not name.startswith("blade"):
            continue
        d = (r @ _rz(yaw_d)).T @ (p - (body + r @ np.array(c)))
        if float(np.linalg.norm(np.maximum(np.abs(d) - np.array(size) / 2, 0.0))) < TOUCH:
            return True
    return False


# -- contact monitor: every robot link x the blade body, polled each physics substep -------------------------------
_ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
_MON: dict = {"view": None, "body": None, "hits": 0, "err": None, "polls": 0}


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _make_view(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return
    stage = get_current_stage()
    paths = [p.GetPath().pathString for p in stage.Traverse() if "/env_0/" in p.GetPath().pathString]
    robot = [p for p in paths if any(r in p for r in _ROBOT_PRIMS)
             and "PhysxContactReportAPI" in stage.GetPrimAtPath(p).GetAppliedSchemas()]
    blade = [p for p in paths if p.endswith(f"/{BLADE_CAP.name}")]
    if not robot or len(blade) != 1:
        raise RuntimeError(f"fp contact monitor: robot {len(robot)}, blade {blade}")
    v = sv.create_rigid_contact_view(robot, filter_patterns=[blade] * len(robot),
                                     max_contact_data_count=64 * len(robot))
    _MON["view"] = (v, len(robot))
    _MON["body"] = sv.create_rigid_body_view(blade[0])  # the blade's pose at each substep
    print(f"[fp] contact monitor: {len(robot)} robot links x the blade body", flush=True)


def _poll_contacts(mgr) -> None:
    try:
        if _MON["view"] is None:
            _make_view(mgr)
        if _MON["view"] is None:
            return
        v, ns = _MON["view"]
        _, pts, _, sep, counts, starts = v.get_contact_data(dt=mgr.get_physics_dt())
        counts = _np(counts).reshape(-1)
        _MON["polls"] += 1
        if not counts.any():
            return
        starts = _np(starts).reshape(-1)
        pts = _np(pts).reshape(-1, 3)
        sep = _np(sep).reshape(-1)
        tf = _np(_MON["body"].get_transforms()).reshape(-1)[:7].astype(np.float64)  # world pos + quat (x, y, z, w)
        r, body = _rotm(tf[3:7]), tf[:3]
        for j in np.nonzero(counts)[0]:
            s0 = int(starts[j])
            for k in range(s0, s0 + int(counts[j])):
                if sep[k] <= TOUCH_SEP and _on_steel_body(r, body, pts[k].astype(np.float64)):
                    _MON["hits"] += 1  # classified on the blade's pose at this same substep
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[fp] contact monitor error: {e!r}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_fp_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        try:
            import carb

            carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        except Exception:  # pragma: no cover
            pass

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._fp_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):

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


if _PHYSX_MGR in sys.modules:
    _patch_manager(sys.modules[_PHYSX_MGR])
elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _MgrFinder())


CLEAR = 0.10


PARK_SPEED = 0.02  # m/s: a gripper point slower than this counts as still
PARK_S = 0.5  # s: both grippers clear and still for this long = parked


def released(objects) -> bool:
    """Every gripper point at least 10 cm from the processor (base, bowl with handle, lid)."""
    return _clear(gripper_points())


def parked(objects) -> bool:
    """Both grippers at least 10 cm clear of the processor, every gripper point slower than 2 cm/s, held 0.5 s."""
    return bool(released(objects) and _EP.still_s >= PARK_S - 1e-6)


def _clear(pts) -> bool:
    if not pts:
        return False
    lo = np.array([BASE_POS[0] - BASE_SIZE[0] / 2, C[1] - HANDLE_R[1], TOP])
    hi = np.array([C[0] + HANDLE_R[1], C[1] + HANDLE_R[1], TOP + RIM + 0.03])
    for p in pts:
        if not _finite(p):
            return False
        if float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0))) < CLEAR:
            return False
    return True


def gate(objects) -> str:
    """Why the trial scores 0, or ''."""
    if _EP.touched_blade:
        return "a gripper touched the blade's steel"
    d, a = base_shift(objects)
    if d > 0.02 or a > 10.0:
        return f"base off its marks ({d * 100:.1f} cm, {a:.0f} deg)"
    for o in (BLADE_CAP, LID, GRIP):
        s = objects[o.name]
        if not _finite(s.pos) or float(s.low[2]) < TOP - 0.05:
            return f"{o.name} off the counter"
    return ""


def score(objects) -> float:
    """0 if gate(); else 1.0 bowl locked + blade seated + lid home with the switch made + grippers parked; 0.5 bowl
    locked + blade seated; 0.25 bowl locked; otherwise 0."""
    if gate(objects):
        return 0.0
    if not bowl_locked(objects):
        return 0.0
    if not blade_seated(objects):
        return 0.25
    if lid_home(objects) and parked(objects):
        return 1.0
    return 0.5


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    a = bowl_angle(objects)
    t = lid_turn(objects)
    return float((0 if not math.isfinite(a) else max(0.0, LOCK_DEG - a)) / 100 + (0.05 if not blade_seated(objects)
                 else 0.0) + (abs(t) / 100 if math.isfinite(t) else 1.0))


def progress(objects) -> dict:
    d, a = base_shift(objects)
    return {"bowl_deg": bowl_angle(objects), "locked": float(bowl_locked(objects)),
            "blade_seated": float(blade_seated(objects)), "lid_turn": lid_turn(objects),
            "lid_seated": float(lid_seated(objects)), "plunger_mm": plunger_in(objects) * 1000,
            "base_cm": d * 100, "base_deg": a, "touched": float(_EP.touched_blade), "released": float(released(objects)), "still_s": _EP.still_s}


VARIATION: dict = {}
