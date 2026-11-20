"""Pump a soft volleyball up to 0.3 bar with a hand ball pump and its inflation needle.

On the bench: an indoor volleyball (21 cm across) is strapped down in a low ring stand (fixed), its valve on the side
facing the pump, soft (0.2 bar). In front of it lie a hand ball pump (single action, 22 mm bore, 12 cm stroke) with a 0-0.6 bar
dial gauge and its inflation needle screwed on, and a wet sponge.

The pump: barrel -weld- head (needle) and barrel -slide- rod -weld- T-handle; the gauge sits on the barrel. Air is
modelled: the barrel's chamber fills from the room as the handle comes out and, as it goes in, shares its air with
the ball through the needle once above the ball's pressure (the ball's valve is a non-return valve); the air pushes
back on the handle at full strength, and a let-go handle stays where it is. The needle (drawn: it does not collide;
the head does) goes into the ball only through its valve, square (within 12 deg of the valve's axis and 3 mm of its
centre); pushed in anywhere else, or levered while in (12 deg, or its line 15 mm off the valve centre), it bends and
tears the valve (latched). It delivers air once 12 mm in. While it is in, the valve holds it: a light sideways spring, and a
drag along the axis while it slides (before the head seats). Wet: its tip held 0.3 s in the sponge before it first goes in.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0

INSTRUCTION = "Pump the ball up to 0.3 bar on the pump's gauge, then take the pump off and set it down."

WHITE = (0.93, 0.93, 0.91)
BLACK = (0.08, 0.08, 0.09)
PUMP_COL = (0.10, 0.32, 0.70)
HANDLE_COL = (0.12, 0.12, 0.13)
RING_COL = (0.85, 0.35, 0.10)
SPONGE_COL = (0.95, 0.82, 0.20)
BLUE = (0.10, 0.25, 0.65)
YELLOW = (0.98, 0.80, 0.10)

# -- ball: indoor volleyball (FIVB: 65-67 cm round, 260-280 g, 0.30-0.325 kg/cm2 = 0.294-0.319 bar) ----------------
BALL_R = 0.105
BALL_C = np.array([0.27, -0.175, TOP + 0.107])  # resting in the ring stand (2 cm tall, 6.5 cm in radius)
VALVE_N0 = np.array([0.0, 1.0, 0.0])  # the valve faces +y (towards the pump) in the ball's own frame
P0 = 0.20  # starting pressure (bar, gauge): soft
BALL_V = 4.0 / 3.0 * math.pi * BALL_R ** 3 * 1000.0  # litres (4.85)
TARGET = (0.29, 0.33)  # bar: "0.3 bar on the gauge" (the FIVB range, read on the dial)
WIDE = (0.25, 0.40)
BURST = 0.50  # far over: the ball's seams and bladder are strained
GAUGE_MAX = 0.6  # bar, full scale of the pump's low-pressure dial (0-0.6 bar / 0-9 psi)
PUMPED = 0.03  # bar: any credit needs at least this much pumped in

# -- pump: a single-action hand ball pump (22 mm bore, 12 cm stroke, ~45 ml a stroke) with a dial gauge -----------------
BAR_R, BAR_L, BAR_M = 0.020, 0.18, 0.12
HEAD_R, HEAD_L, HEAD_M = 0.016, 0.02, 0.03
NEEDLE_L = 0.035  # inflation needle (visual), screwed into the head
ROD_R, ROD_L, ROD_M = 0.005, 0.16, 0.02
TH_SIZE, TH_M = (0.07, 0.025, 0.025), 0.03  # T-handle: 7 cm across
BORE = 0.022
AREA = math.pi * BORE ** 2 / 4
STROKE = 0.12
DEAD = 0.010  # dead volume (barrel end, hose, gauge) as barrel length: 8 ml
DEPTH_ON = 0.012  # needle depth past the valve face at which its side hole is inside
MAX_DEG, MAX_OFF = 12.0, 0.003  # going in: inside the valve's 3.5 mm-radius face
HOLD_KS = 800.0  # N/m: the valve's rubber across the needle (8 N at 1 cm)
HOLD_CS = 20.0  # N s/m
VALVE_C = 60.0  # N s/m: drag of the valve on a sliding wetted needle (dry: twice)
SEATED = 0.030  # m: needle depth with the head (nearly) on the ball; past it the needle no longer slides in the valve
MAX_OFF_IN = 0.015  # once in: the needle's line moved this far off the valve centre has torn it sideways
ROD_C = 1.0  # N s/m: the cup seal's drag; the drive's force on the rod is ROD_C * (target velocity - velocity)
HOLD_K = 2000.0  # N/m: let go, the cup seal's friction holds the handle where it is
GAUGE_S = 0.11  # gauge on the barrel's top, this far behind the head

# start: lying along y on the bench at x PX, needle end towards -y (towards the sponge)
PX = 0.12
HEAD_FRONT_Y = -0.02
AX_Z = TOP + BAR_R


def _on_axis(s: float) -> tuple[float, float, float]:
    """Point s metres along the pump from the head's front face, start pose."""
    return (PX, HEAD_FRONT_Y + s, AX_Z)


def _rotm(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


ROT: dict = {}
LYING = (-math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))  # cylinder axis (z) onto +y; local -y is up

HEAD = Cylinder("pump_head", pos=_on_axis(HEAD_L / 2), radius=HEAD_R, height=HEAD_L, mass=HEAD_M, color=HANDLE_COL)
BARREL = Cylinder("pump_barrel", pos=_on_axis(HEAD_L + BAR_L / 2), radius=BAR_R, height=BAR_L, mass=BAR_M,
                  color=PUMP_COL)
ROD_OUT = 0.025  # rod showing between the barrel and the handle, handle all the way in
ROD = Cylinder("pump_rod", pos=_on_axis(HEAD_L + BAR_L + ROD_OUT - ROD_L / 2), radius=ROD_R, height=ROD_L, mass=ROD_M,
               color=(0.75, 0.76, 0.78))
THANDLE = Box("pump_handle", pos=_on_axis(HEAD_L + BAR_L + ROD_OUT + TH_SIZE[2] / 2), size=TH_SIZE, mass=TH_M,
              color=HANDLE_COL)
PUMP = [HEAD, BARREL, ROD, THANDLE]
for _o in PUMP:
    ROT[_o.name] = LYING

BALL = Sphere("ball", pos=tuple(map(float, BALL_C)), radius=BALL_R, color=WHITE)  # fixed: held in its ring stand
RING = [Box(f"ring_{k}", pos=(float(BALL_C[0] + 0.065 * math.cos(k * math.pi / 6)),
                              float(BALL_C[1] + 0.065 * math.sin(k * math.pi / 6)), TOP + 0.01), size=(0.012, 0.036, 0.02), yaw=k * math.pi / 6, color=RING_COL)
        for k in range(12)]


class Sponge(Box):
    """The wet sponge (fixed); its spawner also authors the pump's joints and the ball's markings."""


SPONGE_C, SPONGE_S = (0.12, -0.11, TOP + 0.0125), (0.07, 0.05, 0.025)
SPONGE = Sponge("sponge", pos=SPONGE_C, size=SPONGE_S, color=SPONGE_COL)

OBJECTS = [*RING, BALL, *PUMP, SPONGE]
NAMES = [BALL.name, BARREL.name, ROD.name]

TWIN_EXTRA = [
    {"name": "pump", "kind": "joint", "real": "hand ball pump, single action, 22 mm bore, 12 cm stroke, dial gauge",
     "note": "head -weld- barrel -slide- rod -weld- T-handle; modelled air: chamber, non-return valve, ball volume"},
    {"name": "needle", "kind": "visual", "real": "standard ball inflation needle, 3.5 cm",
     "note": "drawn on the head; goes into the ball only through its valve (model)"},
    {"name": "ball", "kind": "fixed", "real": "indoor volleyball, 21 cm, strapped into a ring stand", "note": "valve marked"},
]

VARIATION: dict = {}

# -- Isaac side ---------------------------------------------------------------------------------------------------
_P: dict = {}


def _author_joints(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    q0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def path(o):
        return Sdf.Path(f"{parent}/{o.name}")

    def local(o, p_world):
        d = np.asarray(p_world, float) - np.array(o.pos)
        q = ROT.get(o.name)
        return tuple(d) if q is None else tuple(_rotm(q).T @ d)

    def joint(kind, name, b0, b1, p_world, axis="Z", lo=None, hi=None):
        j = kind.Define(stage, f"{parent}/{SPONGE.name}/{name}")
        j.CreateBody0Rel().SetTargets([path(b0)])
        j.CreateLocalPos0Attr(v(local(b0, p_world)))
        j.CreateBody1Rel().SetTargets([path(b1)])
        j.CreateLocalPos1Attr(v(local(b1, p_world)))
        j.CreateLocalRot0Attr(q0)
        j.CreateLocalRot1Attr(q0)
        if kind is not UsdPhysics.FixedJoint:
            j.CreateAxisAttr(axis)
            if lo is not None:
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
        j.CreateExcludeFromArticulationAttr(True)
        return j

    joint(UsdPhysics.FixedJoint, "weld_head", BARREL, HEAD, _on_axis(HEAD_L))
    j = joint(UsdPhysics.PrismaticJoint, "slide_rod", BARREL, ROD, ROD.pos, lo=0.0, hi=STROKE)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(ROD_C)
    d.CreateTargetVelocityAttr(0.0)  # the air's push is written as a target velocity: force = ROD_C * (v_t - v)
    _P["slide"] = str(j.GetPath())
    joint(UsdPhysics.FixedJoint, "weld_handle", ROD, THANDLE, _on_axis(HEAD_L + BAR_L + ROD_OUT))
    # the needle in the valve: a free joint between the world (at the valve, x along its axis) and the pump's head.
    # Its drives are switched on only while the needle is in: springs across the axis (y, z: the valve's rubber
    # holds the needle's line lightly), and a drag along it while the needle slides through the valve
    vc0 = BALL_C + BALL_R * VALVE_N0
    rz = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # x -> +y (VALVE_N0)
    g = UsdPhysics.Joint.Define(stage, f"{parent}/{SPONGE.name}/valve_hold")
    g.CreateLocalPos0Attr(v(vc0))
    g.CreateLocalRot0Attr(Gf.Quatf(math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)))
    g.CreateBody1Rel().SetTargets([path(HEAD)])
    g.CreateLocalPos1Attr(v((0.0, 0.0, 0.0)))
    R1 = _rotm(ROT[HEAD.name]).T @ rz if HEAD.name in ROT else rz
    w = math.sqrt(max(0.0, 1.0 + R1[0, 0] + R1[1, 1] + R1[2, 2])) / 2
    qv = np.array([R1[2, 1] - R1[1, 2], R1[0, 2] - R1[2, 0], R1[1, 0] - R1[0, 1]]) / (4 * w)
    g.CreateLocalRot1Attr(Gf.Quatf(float(w), *map(float, qv)))
    g.CreateExcludeFromArticulationAttr(True)
    for ax_ in ("transX", "transY", "transZ"):
        gd = UsdPhysics.DriveAPI.Apply(g.GetPrim(), ax_)
        gd.CreateTypeAttr("force")
        gd.CreateStiffnessAttr(0.0)
        gd.CreateDampingAttr(0.0)
        gd.CreateTargetPositionAttr(0.0)
        gd.CreateTargetVelocityAttr(0.0)
    _P["hold"] = str(g.GetPath())

    def deco(prim, name, center, size, color, kind="cube", axis="Z"):
        if kind == "cube":
            g = UsdGeom.Cube.Define(stage, f"{prim}/{name}")
            g.CreateSizeAttr(1.0)
        else:
            g = UsdGeom.Cylinder.Define(stage, f"{prim}/{name}")
            g.CreateRadiusAttr(0.5)
            g.CreateHeightAttr(1.0)
            g.CreateAxisAttr(axis)
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        return g

    # local frames of the lying pump: z along the pump towards the handle, -y up
    hp = f"{parent}/{HEAD.name}"
    deco(hp, "needle", (0.0, 0.0, -HEAD_L / 2 - NEEDLE_L / 2), (0.0019, 0.0019, NEEDLE_L), (0.78, 0.78, 0.80), "cyl")
    bp = f"{parent}/{BARREL.name}"
    gz = -BAR_L / 2 + GAUGE_S
    deco(bp, "gauge_case", (0.0, -BAR_R - 0.006, gz), (0.050, 0.012, 0.050), (0.15, 0.15, 0.16), "cyl", "Y")
    deco(bp, "gauge_face", (0.0, -BAR_R - 0.0125, gz), (0.044, 0.001, 0.044), (0.97, 0.97, 0.95), "cyl", "Y")
    for k in range(7):  # ticks every 0.1 bar, 0 to 0.6 bar over 270 deg
        a = math.radians(-135 + 45 * k)
        deco(bp, f"tick{k}", (0.019 * math.sin(a), -BAR_R - 0.0132, gz - 0.019 * math.cos(a)), (0.0025, 0.0006, 0.0025),
             BLACK)
    for m in range(5):  # green band over the target
        p = TARGET[0] + (TARGET[1] - TARGET[0]) * m / 4
        a = math.radians(-135 + 270 * p / GAUGE_MAX)
        deco(bp, f"band{m}", (0.0175 * math.sin(a), -BAR_R - 0.0133, gz - 0.0175 * math.cos(a)), (0.004, 0.0006, 0.004),
             (0.10, 0.65, 0.20))
    ptr = UsdGeom.Xform.Define(stage, f"{bp}/gauge_ptr")
    px = UsdGeom.Xformable(ptr)
    px.AddTranslateOp().Set(Gf.Vec3d(0.0, -BAR_R - 0.0137, gz))
    _P["ptr_rot"] = px.AddRotateYOp()
    _P["ptr_rot"].Set(float(gauge_angle(0.0)))
    g = UsdGeom.Cube.Define(stage, f"{bp}/gauge_ptr/arm")
    g.CreateSizeAttr(1.0)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.08, 0.06)])
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.008))
    x.AddScaleOp().Set(Gf.Vec3f(0.002, 0.0006, 0.017))

    # the ball's valve and coloured panels (they turn with it)
    blp = f"{parent}/{BALL.name}"

    def disc(name, n, r, color, h=0.002):
        n = np.asarray(n, float) / np.linalg.norm(n)
        g = UsdGeom.Cylinder.Define(stage, f"{blp}/{name}")
        g.CreateRadiusAttr(1.0)
        g.CreateHeightAttr(1.0)
        g.CreateAxisAttr("Z")
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        z = np.array([0.0, 0.0, 1.0])
        c = np.cross(z, n)
        sn, cs = float(np.linalg.norm(c)), float(z @ n)
        if sn < 1e-9:
            q = Gf.Quatf(1.0, 0.0, 0.0, 0.0) if cs > 0 else Gf.Quatf(0.0, 1.0, 0.0, 0.0)
        else:
            ang = math.atan2(sn, cs)
            u = c / sn * math.sin(ang / 2)
            q = Gf.Quatf(math.cos(ang / 2), *map(float, u))
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, n * (BALL_R - h / 2 + 0.0008))))
        x.AddOrientOp().Set(q)
        x.AddScaleOp().Set(Gf.Vec3f(float(r), float(r), float(h)))

    gr = (1 + 5 ** 0.5) / 2
    ico = [(-1, gr, 0), (1, gr, 0), (-1, -gr, 0), (1, -gr, 0), (0, -1, gr), (0, 1, gr), (0, -1, -gr), (0, 1, -gr),
           (gr, 0, -1), (gr, 0, 1), (-gr, 0, -1), (-gr, 0, 1)]
    for k, n in enumerate(ico):
        n = np.array(n, float) / np.linalg.norm(n)
        if n @ VALVE_N0 > 0.8:
            continue
        disc(f"panel{k}", n, 0.036, BLUE if k % 2 else YELLOW)
    # the hook-and-loop strap that holds the ball down in its stand: over the top and under the stand's base,
    # across the ball (in the plane through its centre at right angles to the valve's side)
    rs, ns = BALL_R + 0.0025, 36
    for k in range(ns):  # flat segments laid round the ball
        th = 2 * math.pi * k / ns
        g = UsdGeom.Cube.Define(stage, f"{blp}/strap{k}")
        g.CreateSizeAttr(1.0)
        g.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.10, 0.12)])
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(rs * math.sin(th), 0.0, rs * math.cos(th)))
        x.AddRotateYOp().Set(math.degrees(th))
        x.AddScaleOp().Set(Gf.Vec3f(2 * math.pi * rs / ns * 1.08, 0.025, 0.004))
    disc("valve_ring", VALVE_N0, 0.008, (0.25, 0.25, 0.27), 0.003)
    disc("valve", VALVE_N0, 0.0035, BLACK, 0.004)
    _P["parent"] = parent


def gauge_angle(p_bar: float) -> float:
    """Pointer turn (deg, about the barrel's local y) for a reading: 0 bar at -135 deg, full scale at +135 deg."""
    a = -135.0 + 270.0 * min(max(p_bar, 0.0), GAUGE_MAX) / GAUGE_MAX
    return -a


def _spawn_sponge(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_joints(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _sponge_object(spec: Sponge):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class SpongeCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_sponge)

    cfg = SpongeCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(),
                    physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0],
                                                                    dynamic_friction=GRIPPY_FRICTION[1]),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.9))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_ballpump_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, Sponge):
            return _sponge_object(spec)
        q = ROT.get(spec.name)
        if q is None:
            return orig(spec)
        yaw_q = scene._yaw_xyzw
        scene._yaw_xyzw = lambda _yaw: q
        try:
            return orig(spec)
        finally:
            scene._yaw_xyzw = yaw_q

    scene.task_object = task_object
    scene._ballpump_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _ballpump = True

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
    elif not any(getattr(f, "_ballpump", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- the pump's air and the needle, run every control step -----------------------------------------------------------
_ENV: dict = {"env": None}
HOLD_S = 2.0
CLEAR = 0.03
OPEN_GAP = 0.02
STILL_V = 0.01
WET_S = 0.3


class _Episode:
    def __init__(self):
        self.t = 0.0
        self.p_ball = P0  # bar gauge
        self.p_ch = 1.0  # chamber, bar absolute
        self.m_ch = None  # chamber air (bar abs x barrel length)
        self.bent = False
        self.bent_why = ""
        self.inside = False  # needle tip inside the ball (through the valve)
        self.wet_t = 0.0
        self.wet = False
        self.wet_on_entry = None  # whether the needle was wet when it first went in
        self.entries = 0
        self.connected = False
        self.strokes = 0
        self._ext_max = 0.0
        self.ext_hold = None
        self.held = False
        self.clear_t = 0.0
        self.hist = []  # (t, tier) while clear
        self.tcp_prev = None
        self.last = {}


_EP = _Episode()


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


class _State:
    __slots__ = ("pos", "quat_xyzw", "lin_vel")

    def __init__(self, pos, quat, vel):
        self.pos, self.quat_xyzw, self.lin_vel = pos, quat, vel


def states(env, names=None) -> dict:
    """Body-origin poses (not bounding-box centres: the gauge and markings would shift those) and velocities."""
    base = env.unwrapped
    world = base.arena_world
    o = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    out = {}
    for n in names or NAMES:
        p = world.get_pose_w(n)[0].detach().cpu().numpy().astype(np.float64)
        moving = n in base.scene.rigid_objects
        v = world.get_root_linear_velocity_w(n)[0].detach().cpu().numpy().astype(np.float64) if moving else np.zeros(3)
        out[n] = _State(p[:3] - o, p[3:], v)
    return out


def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def pump_axis(objects):
    b = objects[BARREL.name]
    if not _finite(b.quat_xyzw):
        return None
    return _rotm(b.quat_xyzw)[:, 2]  # from the needle end towards the handle


def needle_tip(objects):
    b = objects[BARREL.name]
    ax = pump_axis(objects)
    if ax is None or not _finite(b.pos):
        return None
    return np.asarray(b.pos, float) - ax * (BAR_L / 2 + HEAD_L + NEEDLE_L)


def rod_ext(objects) -> float:
    """How far (m) the handle is pulled out from all the way in."""
    b, r = objects[BARREL.name], objects[ROD.name]
    if not all(_finite(a) for a in (b.pos, b.quat_xyzw, r.pos)):
        return float("nan")
    d = _rotm(b.quat_xyzw).T @ (np.asarray(r.pos, float) - np.asarray(b.pos, float))
    return float(d[2] - (BAR_L / 2 + ROD_OUT - ROD_L / 2))


def valve(objects):
    """(valve centre, its outward axis), world."""
    bl = objects[BALL.name]
    n = _rotm(bl.quat_xyzw) @ VALVE_N0
    return np.asarray(bl.pos, float) + BALL_R * n, n


def needle_geometry(objects):
    """(depth of the tip past the valve's face along its axis, distance of the needle's line from the valve centre,
    angle in deg between the needle and the valve's axis), or None."""
    tip, ax = needle_tip(objects), pump_axis(objects)
    if tip is None or not _finite(objects[BALL.name].pos):
        return None
    vc, vn = valve(objects)
    d = -ax  # needle direction (tip-wards)
    depth = float((vc - tip) @ vn)
    w = vc - tip
    off = float(np.linalg.norm(w - (w @ d) * d))
    ang = math.degrees(math.acos(float(np.clip(d @ (-vn), -1, 1))))
    return depth, off, ang


def in_sponge(p) -> bool:
    c, s = np.array(SPONGE_C), np.array(SPONGE_S) / 2
    return bool(np.all(np.abs(np.asarray(p) - c) <= s))


def _step_dt() -> float:
    env = _ENV["env"]
    try:
        return float(env.unwrapped.step_dt)
    except Exception:  # noqa: BLE001
        return 0.1


def _bend(why: str) -> None:
    if not _EP.bent:
        _EP.bent, _EP.bent_why = True, why


def handle_held(objects, ext, hands) -> bool:
    """A closed gripper on the T-handle: its TCP within 4 cm of the handle's centre, jaws closed past 8 cm."""
    if hands is None:
        return False
    _pts, ap, tcp = hands
    if len(ap) != 2:
        return False
    b = objects[BARREL.name]
    hc = np.asarray(b.pos, float) + pump_axis(objects) * (BAR_L / 2 + ROD_OUT + max(ext, 0.0) + TH_SIZE[2] / 2)
    return any(float(np.linalg.norm(tcp[k] - hc)) < 0.04 and ap[k] < 0.08 for k in range(2))


def _pump_step(objects, hands=None) -> None:
    ep = _EP
    dt = _step_dt()
    ep.t += dt
    tip, ext, geo = needle_tip(objects), rod_ext(objects), needle_geometry(objects)
    if tip is None or not math.isfinite(ext) or geo is None:
        return
    depth, off, ang = geo
    # wetting: the needle's tip held in the sponge
    ep.wet_t = ep.wet_t + dt if in_sponge(tip) else 0.0
    if ep.wet_t >= WET_S:
        ep.wet = True
    # the needle and the ball: in only through the valve, square; levered while in, it bends and tears the valve
    inside = float(np.linalg.norm(tip - np.asarray(objects[BALL.name].pos))) < BALL_R - 0.0005
    if inside and not ep.inside:
        if off > MAX_OFF or ang > MAX_DEG or depth < 0:
            _bend(f"pushed in off the valve or askew (off {off * 1000:.1f} mm, {ang:.1f} deg)")
        else:
            ep.entries += 1
            if ep.wet_on_entry is None:
                ep.wet_on_entry = ep.wet
    if inside and ep.inside and (ang > MAX_DEG or off > MAX_OFF_IN):
        _bend(f"levered while in ({ang:.1f} deg, {off * 1000:.1f} mm)")
    ep.inside = inside
    ep.connected = inside and depth >= DEPTH_ON and not ep.bent
    on = inside and not ep.bent
    sliding = on and depth < SEATED  # the head not yet on the ball: the needle slides through the valve
    _write_hold(HOLD_KS if on else 0.0, HOLD_CS if on else 0.0,
                (VALVE_C if ep.wet_on_entry else 2 * VALVE_C) if sliding else 0.0)
    # air, isothermal: chamber length L = ext + DEAD; filled from the room (1 bar abs) while the handle comes out
    L = max(ext, 0.0) + DEAD
    if ep.m_ch is None:
        ep.m_ch = 1.0 * L
    p_ch = ep.m_ch / L
    if p_ch < 1.0:
        ep.m_ch, p_ch = 1.0 * L, 1.0
    pb_abs = ep.p_ball + 1.0
    if ep.connected and p_ch > pb_abs:  # the ball's valve opens: chamber and ball share the air
        vb = BALL_V / 1000.0 / AREA  # ball volume as barrel length
        p = (ep.m_ch + pb_abs * vb) / (L + vb)
        ep.m_ch, p_ch = p * L, p
        ep.p_ball = p - 1.0
    ep.p_ch = p_ch
    if ext > ep._ext_max:
        ep._ext_max = ext
    if ext < 0.01 and ep._ext_max > 0.8 * STROKE:
        ep.strokes += 1
        ep._ext_max = 0.0
    # the rod: with a hand on the handle, the air pushes it out through the drive (force = ROD_C (F / ROD_C - v));
    # let go, the cup seal's friction holds it where it is (a stiff hold at that place)
    held = handle_held(objects, ext, hands)
    if held:
        f = max(p_ch - 1.0, 0.0) * 1e5 * AREA  # the air's full push (12 N at 0.32 bar)
        _write_drive(0.0, ext, f / ROD_C)
        ep.ext_hold = None
    else:
        if ep.ext_hold is None:
            ep.ext_hold = ext
        _write_drive(HOLD_K, ep.ext_hold, 0.0)
    ep.held = held


def _write_hold(ks: float, cs: float, cx: float) -> None:
    """The valve's hold on the needle: sideways spring ks and damping cs, drag cx along the axis (0: off)."""
    key = (ks, cs, cx)
    if "hold" not in _P or _P.get("hold_set") == key:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        prim = get_current_stage().GetPrimAtPath(_P["hold"])
        prim.GetAttribute("drive:transX:physics:damping").Set(float(cx))
        for ax_ in ("transY", "transZ"):
            prim.GetAttribute(f"drive:{ax_}:physics:stiffness").Set(float(ks))
            prim.GetAttribute(f"drive:{ax_}:physics:damping").Set(float(cs))
        _P["hold_set"] = key
    except Exception:  # noqa: BLE001
        pass


def _write_drive(k: float, x: float, v: float) -> None:
    if "slide" not in _P:
        return
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        prim = get_current_stage().GetPrimAtPath(_P["slide"])
        for name, val in (("stiffness", k), ("targetPosition", x), ("targetVelocity", v)):
            a = prim.GetAttribute(f"drive:linear:physics:{name}")
            if a.Get() != float(val):
                a.Set(float(val))
    except Exception:  # noqa: BLE001
        pass


def gauge_reading(ep=None) -> float:
    """What the pump's gauge shows: the line pressure (the ball's while the needle is in, the chamber's while
    the handle is pushed in), 0 once the needle is out and the chamber is at room pressure."""
    ep = _EP if ep is None else ep
    if ep.connected:
        return max(ep.p_ball, ep.p_ch - 1.0)
    return max(ep.p_ch - 1.0, 0.0)


# -- grippers ------------------------------------------------------------------------------------------------------
def gripper_state():
    env = _ENV["env"]
    if env is None:
        return [], np.zeros(2), np.zeros((2, 3))
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts, ap, tcp = [], [], []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        t = np.asarray(tcp_world(rob), dtype=np.float64) - origin
        tcp.append(t)
        pts.append(t)
        pts += list(np.asarray(pad_tips_world(rob), dtype=np.float64).reshape(-1, 3) - origin)
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
        ids = rob.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q = rob.data.joint_pos.detach().cpu().numpy()[0, ids]
        ap.append(float(abs(q[0]) + abs(q[1])))
    return pts, np.array(ap), np.array(tcp)


def _seg_dist(a, b, p) -> float:
    d = b - a
    t = float(np.clip(np.dot(p - a, d) / max(float(np.dot(d, d)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * d)))


def hand_clearance(objects, pts) -> float:
    """Smallest distance (m) from any gripper point to the pump (as a 2 cm thick line, needle tip to handle) or
    the ball's surface."""
    b = objects[BARREL.name]
    ax = pump_axis(objects)
    if not pts or ax is None:
        return 0.0
    c = np.asarray(b.pos, float)
    lo = c - ax * (BAR_L / 2 + HEAD_L + NEEDLE_L)
    hi = c + ax * (BAR_L / 2 + ROD_OUT + max(rod_ext(objects), 0.0) + TH_SIZE[2])
    bc = np.asarray(objects[BALL.name].pos, float)
    return float(min(min(_seg_dist(lo, hi, p) - BAR_R, float(np.linalg.norm(p - bc)) - BALL_R) for p in pts))


def pump_down(objects) -> bool:
    """The pump set down on the bench (its barrel's axis no more than 3 cm up), still, its needle out of the ball."""
    b = objects[BARREL.name]
    if not (_finite(b.pos) and _finite(b.lin_vel)):
        return False
    return bool(b.pos[2] <= TOP + 0.03 and float(np.linalg.norm(b.lin_vel)) < 0.02 and not _EP.inside)


def tier(ep=None) -> float:
    ep = _EP if ep is None else ep
    p = ep.p_ball
    if p > BURST or p < P0 + PUMPED:
        return 0.0
    if TARGET[0] <= p <= TARGET[1]:
        return 1.0 if ep.wet_on_entry else 0.5
    if WIDE[0] <= p <= WIDE[1]:
        return 0.5
    return 0.25


def _track(objects, hands=None) -> None:
    ep = _EP
    pts, ap, tcp = gripper_state() if hands is None else hands
    clear_d = hand_clearance(objects, pts)
    speed = 0.0 if ep.tcp_prev is None else float(np.linalg.norm(tcp - ep.tcp_prev, axis=1).max() / _step_dt())
    ep.tcp_prev = tcp.copy()
    ok = (len(ap) == 2 and bool(np.all(ap >= OPEN_GAP)) and clear_d >= CLEAR and speed < STILL_V
          and pump_down(objects) and not ep.bent)
    ep.clear_t = ep.clear_t + _step_dt() if ok else 0.0
    if ok:
        ep.hist.append((ep.t, tier(ep)))
        ep.hist = [h for h in ep.hist if h[0] >= ep.t - HOLD_S - 1e-6]
    else:
        ep.hist = []
    ep.last = dict(clear=clear_d, speed=speed, ap=ap.tolist(), down=pump_down(objects))


def gate(ep=None) -> str | None:
    ep = _EP if ep is None else ep
    if ep.bent:
        return "needle bent / valve torn: " + ep.bent_why
    if ep.clear_t < HOLD_S - 1e-6:
        return "not yet 2 s with both hands open, clear and still and the pump down"
    return None


def score(objects=None) -> float:
    ep = _EP
    if gate(ep) or not ep.hist:
        return 0.0
    return float(min(t for _, t in ep.hist))


def success(objects=None) -> bool:
    return score(objects) >= 1.0


def distance(objects=None) -> float:
    return 1.0 - score(objects)


def progress(objects) -> dict:
    ep = _EP
    geo = needle_geometry(objects) or (float("nan"),) * 3
    return {"p_bar": ep.p_ball, "gauge": gauge_reading(ep), "strokes": ep.strokes, "connected": float(ep.connected),
            "inside": float(ep.inside), "bent": float(ep.bent), "wet": float(ep.wet),
            "wet_on_entry": float(bool(ep.wet_on_entry)), "depth_mm": geo[0] * 1000, "off_mm": geo[1] * 1000,
            "ang_deg": geo[2], "ext_mm": rod_ext(objects) * 1000, "clear_t": ep.clear_t, "score": score(objects)}


def summary(objects) -> str:
    pr = progress(objects)
    ep = _EP
    return (f"gate={gate() or 'ok'} score={score(objects)} p={ep.p_ball:.3f} gauge={pr['gauge']:.2f} "
            f"strokes={ep.strokes} conn={ep.connected} in={ep.inside} bent={ep.bent} wet={ep.wet}/{ep.wet_on_entry} "
            f"depth={pr['depth_mm']:.1f}mm off={pr['off_mm']:.1f}mm ang={pr['ang_deg']:.1f} ext={pr['ext_mm']:.0f}mm "
            f"clear={ep.last.get('clear', 0):.3f} v={ep.last.get('speed', 0):.3f} "
            f"down={ep.last.get('down')} ap={np.round(ep.last.get('ap', []), 3).tolist()} "
            f"ball={np.round(objects[BALL.name].pos, 3).tolist()} barrel={np.round(objects[BARREL.name].pos, 3).tolist()}")


def _render() -> None:
    if "ptr_rot" in _P:
        try:
            _P["ptr_rot"].Set(float(gauge_angle(gauge_reading())))
        except Exception:  # noqa: BLE001
            pass


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_ballpump_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            out = orig(self, *a, **k)
            _ENV["env"] = self._env
            _render()
            return out

        reset._ballpump_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_ballpump_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                objs = states(self._env)
                hands = gripper_state()
                _pump_step(objs, hands)
                _track(objs, hands)
                _render()
            except Exception as e:  # noqa: BLE001
                print(f"[ballpump] step failed: {e!r}", flush=True)
            return out

        _task_step._ballpump_hooked = True
        cls._task_step = _task_step


_install_hooks()


def tracker_summary() -> str:
    env = _ENV["env"]
    if env is None:
        return "(no env)"
    return summary(states(env))
