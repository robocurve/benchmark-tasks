"""disckit: a game disc, its case with a sprung hub, and fixed parts with their own friction, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from disckit import Case, Disc, Fixed, body_pose, on_reset, on_step, write_joint, write_pose

- :class:`Fixed` is a fixed box with its own friction and look (the console and the storage box are built from these).
- :class:`Disc` is a loose 120 mm optical disc with a 15 mm centre hole: a ring of thin boxes for its collider, drawn
  as a silver disc with a printed label side.
- :class:`Case` is an open game case lying flat: one articulation whose root is the case (tray half, spine and cover
  half, with a thumb cut-out in the tray's near edge and the hub's seat ring and post), with the hub's centre button on
  a sprung slide (``button``) and three teeth on radial slides (``tooth_0`` .. ``tooth_2``) that hold the disc down on
  the hub. The task file moves the teeth in when the button is pressed.

``write_joint`` changes a case joint's limits, spring or position in the running simulation, ``body_pose`` reads a
loose object's pose, ``write_pose`` sets one. ``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``). Pure Python at import time; Isaac Sim is reached
only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Case", "CaseState", "Disc", "Fixed", "TEETH", "body_pose", "env", "finger_points", "install", "on_reset",
           "on_step", "write_joint", "write_pose"]

TEETH = 3

# -- the disc (m): a 120 mm optical disc --------------------------------------------------------------------------------
DISC_R, HOLE_R, DISC_T = 0.060, 0.0075, 0.0012

# -- the case's hub, in the case frame (origin at the hub's axis on the underside of the case, z up) -------------------
FLOOR = 0.002                      # the tray's floor
SEAT_R, SEAT_H = 0.017, 0.0025     # the seat ring the disc rests on, above the floor
POST_R = 0.0072                    # the hub's post, through the disc's hole
TOOTH = (0.0035, 0.004, 0.0012)    # radial, across, tall
TOOTH_OUT = 0.0085                 # the teeth's outer ends, engaged (over the disc's hole edge)
TOOTH_STROKE = 0.0022              # how far they move in when the button is pressed
BUTTON_R, BUTTON_TRAVEL, BUTTON_CAP = 0.0060, 0.0025, 0.0070
POST_TOP = FLOOR + SEAT_H + DISC_T + 0.0004
BUTTON_TOP = POST_TOP + BUTTON_CAP         # the button's top, unpressed, above the case's underside


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction and look."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.6
    metallic: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Disc(Box):
    """A loose disc. ``pos`` is its centre, ``quat`` (xyzw) its orientation (its axis along the frame's z, the label on
    +z)."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    segments: int = 24
    friction: tuple = (0.5, 0.4)
    label: tuple = (0.10, 0.20, 0.55)
    meta: dict = field(default_factory=dict, compare=False, hash=False)


@dataclass(frozen=True)
class Case(Box):
    """An open game case lying flat. ``pos`` is the hub's axis on the case's underside; the tray half spans
    ``tray`` (x, y) around it, the cover half lies beyond the spine on +y. ``notch`` is the thumb cut-out in the near
    (-x) edge: (across, deep)."""

    tray: tuple = (0.135, 0.171)
    wall: float = 0.006
    notch: tuple = (0.044, 0.032)
    button_spring: float = 600.0           # N/m: about 1.5 N to press it home
    friction: tuple = (0.5, 0.4)
    shell: tuple = (0.10, 0.22, 0.62)
    hub: tuple = (0.55, 0.58, 0.62)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Case must be loose (give it a mass)")


@dataclass(frozen=True)
class CaseState(ObjectState):
    """The case as the simulator has it; ``joints`` maps button (m, negative pressed) and tooth_i (m, negative in) to
    their positions."""

    joints: dict = field(default_factory=dict)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


def _qz(angle):
    return (math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2))      # wxyz


# -- authoring helpers --------------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.5, metal=0.0, opacity=1.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal, opacity)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal,
                                                   opacity=opacity)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, pos, mass, lin_damp=0.05, ang_damp=0.1):
        from pxr import PhysxSchema, UsdPhysics

        xf = self.UsdGeom.Xform.Define(self.stage, path)
        if path != self.path:
            xf.AddTranslateOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*pos))
            xf.AddOrientOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        return prim

    def collide(self, prim, mat, offset=0.0008):
        from isaaclab.sim.utils import bind_physics_material
        from pxr import PhysxSchema, UsdPhysics

        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)

    def paint(self, prim, look):
        from isaaclab.sim.utils import bind_visual_material

        bind_visual_material(str(prim.GetPath()), look, stage=self.stage)

    def box(self, path, centre, size, look, mat=None, quat_wxyz=None):
        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if quat_wxyz is not None:
            cube.AddOrientOp().Set(self.Gf.Quatf(*(float(v) for v in quat_wxyz)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        if look:
            self.paint(cube.GetPrim(), look)
        if mat:
            self.collide(cube.GetPrim(), mat)
        if not look:
            self.UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        return cube.GetPrim()

    def cyl(self, path, r, z0, z1, look, mat=None, centre=(0.0, 0.0)):
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(z1 - z0))
        c.AddTranslateOp().Set(self.Gf.Vec3d(float(centre[0]), float(centre[1]), float(z0 + z1) / 2))
        self.paint(c.GetPrim(), look)
        if mat:
            self.collide(c.GetPrim(), mat)
        return c.GetPrim()


# -- spawning ------------------------------------------------------------------------------------------------------------

_SPECS: dict = {}


def _fixed_object(scene, spec: Fixed):
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness,
                                       metallic=spec.metallic)
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size),
                              physics_material=_material(sim_utils, spec.static_friction, spec.dynamic_friction),
                              visual_material=look, collision_props=scene.PhysxCollisionPropertiesCfg())
    return scene.Object(name=spec.name, object_type=scene.ObjectType.BASE, spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                                rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _spawn_disc(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec: Disc = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, None, spec.mass, ang_damp=0.2)
    m = a.mat("disc", *spec.friction)
    n = spec.segments
    r_mid = (DISC_R + HOLE_R) / 2
    width = 2 * math.pi * DISC_R / n * 1.02
    for i in range(n):                                                   # the collider: a ring of thin boxes
        th = 2 * math.pi * i / n
        a.box(f"{prim_path}/ring_{i}", (r_mid * math.cos(th), r_mid * math.sin(th), 0.0),
              (DISC_R - HOLE_R, width, DISC_T), None, m, quat_wxyz=_qz(th))
    silver = a.look((0.80, 0.82, 0.86), rough=0.15, metal=0.9)
    clear = a.look((0.90, 0.92, 0.95), rough=0.1, opacity=0.7)
    label = a.look(spec.label, rough=0.6)
    white = a.look((0.95, 0.95, 0.95), rough=0.6)
    a.cyl(f"{prim_path}/data", DISC_R - 0.0002, -DISC_T / 2, DISC_T / 2 - 0.0001, silver)
    a.cyl(f"{prim_path}/label", DISC_R - 0.0015, DISC_T / 2 - 0.0001, DISC_T / 2 + 0.0001, label)
    a.box(f"{prim_path}/label_band", (0.0, 0.032, DISC_T / 2 + 0.00015), (0.070, 0.012, 0.0001), white)
    a.cyl(f"{prim_path}/clamp", 0.0165, -DISC_T / 2 - 0.0001, DISC_T / 2 + 0.0002, clear)
    a.cyl(f"{prim_path}/hole", HOLE_R, -DISC_T / 2 - 0.0002, DISC_T / 2 + 0.0003, a.look((0.05, 0.05, 0.06)))
    return a.root


def _spawn_case(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The case: root body (shell and hub), button and teeth bodies, all authored relative to the hub's axis."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Case = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    root = f"{prim_path}/case"
    a.body(root, (0.0, 0.0, 0.0), spec.mass)
    shell = a.look(spec.shell, rough=0.35, opacity=0.92)
    cover = a.look((0.08, 0.12, 0.30), rough=0.5)
    art = a.look((0.85, 0.40, 0.15), rough=0.6)
    hubl = a.look(spec.hub, rough=0.4)
    m = a.mat("case", *spec.friction)
    tx, ty = spec.tray
    nw, nd = spec.notch
    x0, x1, y0, y1 = -tx / 2, tx / 2, -ty / 2, ty / 2

    def slab(name, xa, xb, ya, yb, za, zb, look=shell):
        a.box(f"{root}/{name}", ((xa + xb) / 2, (ya + yb) / 2, (za + zb) / 2), (xb - xa, yb - ya, zb - za), look, m)

    # tray half: floor with the thumb cut-out at the near edge, and its rim
    slab("floor_far", x0 + nd, x1, y0, y1, 0.0, FLOOR)
    slab("floor_l", x0, x0 + nd, nw / 2, y1, 0.0, FLOOR)
    slab("floor_r", x0, x0 + nd, y0, -nw / 2, 0.0, FLOOR)
    t = 0.002
    slab("rim_far", x1 - t, x1, y0, y1, FLOOR, spec.wall)
    slab("rim_near_l", x0, x0 + t, nw / 2, y1, FLOOR, spec.wall)
    slab("rim_near_r", x0, x0 + t, y0, -nw / 2, FLOOR, spec.wall)
    slab("rim_right", x0, x1, y0, y0 + t, FLOOR, spec.wall)
    slab("spine", x0, x1, y1, y1 + 0.014, 0.0, FLOOR, look=cover)
    # cover half, open flat beyond the spine, its inside up (a printed insert behind clear plastic)
    slab("cover", x0, x1, y1 + 0.014, y1 + 0.014 + ty, 0.0, FLOOR, look=cover)
    a.box(f"{root}/insert", (0.0, y1 + 0.014 + ty / 2, FLOOR + 0.0002), (tx - 0.012, ty - 0.012, 0.0003), art)
    # the hub: seat ring and post
    a.cyl(f"{root}/seat", SEAT_R, FLOOR, FLOOR + SEAT_H, hubl, m)
    post_top = POST_TOP
    a.cyl(f"{root}/post", POST_R, FLOOR, post_top, hubl, m)
    for i in range(6):                                                   # moulded ribs round the seat, drawn
        th = math.pi * i / 3
        a.box(f"{root}/rib_{i}", (0.022 * math.cos(th), 0.022 * math.sin(th), FLOOR + 0.0006), (0.008, 0.0015, 0.0012),
              hubl, quat_wxyz=_qz(th))

    # the button: the hub's centre, on a sprung slide down into the post
    button = f"{prim_path}/button"
    a.body(button, (0.0, 0.0, post_top), 0.002)
    a.cyl(f"{button}/cap", BUTTON_R, 0.0, BUTTON_CAP, a.look((0.70, 0.72, 0.76), rough=0.3), m)

    # the teeth: three small blocks over the disc's hole edge, sliding in radially
    tooth_z = FLOOR + SEAT_H + DISC_T + 0.0002 + TOOTH[2] / 2
    teeth = []
    for i in range(TEETH):
        th = 2 * math.pi * i / TEETH
        path = f"{prim_path}/tooth_{i}"
        r = TOOTH_OUT - TOOTH[0] / 2
        a.body(path, (r * math.cos(th), r * math.sin(th), tooth_z), 0.0005)
        a.box(f"{path}/block", (0.0, 0.0, 0.0), TOOTH, hubl, m, quat_wxyz=_qz(th))
        teeth.append((path, th))

    stage = a.stage

    def joint(name, b1, pos0, axis_rot, lower, upper, stiffness=0.0, damping=0.0):
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{b1}/{name}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(root)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
        j.CreateLocalRot0Attr(Gf.Quatf(*axis_rot))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(*axis_rot))
        j.CreateLowerLimitAttr(float(lower))
        j.CreateUpperLimitAttr(float(upper))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(stiffness))
        drive.CreateDampingAttr(float(damping))
        drive.CreateTargetPositionAttr(0.0)

    up = (math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0)                     # joint x axis along +z
    joint("button", button, (0.0, 0.0, post_top), up, -BUTTON_TRAVEL, 0.0, stiffness=spec.button_spring, damping=2.0)
    for i, (path, th) in enumerate(teeth):
        r = TOOTH_OUT - TOOTH[0] / 2
        q = _qz(th)                                                      # joint x axis radial, outward
        joint(f"tooth_{i}", path, (r * math.cos(th), r * math.sin(th), tooth_z), (q[0], q[1], q[2], q[3]),
              0.0, 0.0, stiffness=2000.0, damping=5.0)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(root))
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(root))
    art_api.CreateEnabledSelfCollisionsAttr(False)
    art_api.CreateSolverPositionIterationCountAttr(32)
    art_api.CreateSolverVelocityIterationCountAttr(8)
    return a.root


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                    rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _case_object(spec: Case):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class CaseSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_case)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"button": ImplicitActuatorCfg(joint_names_expr=["button"], stiffness=spec.button_spring,
                                                       damping=2.0),
                         "teeth": ImplicitActuatorCfg(joint_names_expr=["tooth_.*"], stiffness=2000.0, damping=5.0)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=CaseSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# -- run time ------------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def body_pose(environment, name: str):
    """A loose object's root pose: position (relative to the env origin) and quaternion (xyzw)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pose = _np(base.scene[name].data.root_link_pose_w)[0]
    return pose[:3] - origin, pose[3:7]


def write_pose(environment, name: str, pos, quat_xyzw) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def write_joint(environment, name: str, joint: str, *, limits=None, stiffness=None, position=None) -> None:
    """Change one joint of case ``name`` in the running simulation."""
    import torch

    asset = environment.unwrapped.scene[name]
    ids = [list(asset.joint_names).index(joint)]
    dev = _t(asset.data.joint_pos).device

    def call(stem, **kw):
        fn = getattr(asset, f"{stem}_index", None) or getattr(asset, stem)
        fn(**kw)

    if limits is not None:
        call("write_joint_position_limit_to_sim",
             limits=torch.tensor([[list(limits)]], dtype=torch.float32, device=dev), joint_ids=ids, env_ids=[0])
    if stiffness is not None:
        call("write_joint_stiffness_to_sim", stiffness=torch.full((1, 1), float(stiffness), device=dev),
             joint_ids=ids, env_ids=[0])
    if position is not None:
        pos = torch.full((1, 1), float(position), device=dev)
        call("write_joint_state_to_sim", position=pos, velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=[0])


def _case_state(environment, spec: Case) -> CaseState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    names, q = list(asset.joint_names), _np(asset.data.joint_pos)[0]
    joints = {j: float(q[names.index(j)]) for j in names}
    tx, ty = spec.tray
    pts = np.array([[sx * tx / 2, sy, sz] for sx in (-1, 1) for sy in (-ty / 2, ty / 2 + 0.014 + ty)
                    for sz in (0.0, spec.wall)]) @ _rotation(quat).T + pos
    return CaseState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pts.min(0),
                     high=pts.max(0), joints=joints)


def finger_points(environment) -> list:
    """The grasp point of each gripper (world, relative to the env origin)."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


# -- hooking yam_arena ------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_disckit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Case):
            return _case_object(spec)
        if isinstance(spec, Disc):
            return _rigid_object(spec, _spawn_disc)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        cases = {n for n in names if isinstance(_SPECS.get(n), Case)}
        plain = [n for n in names if n not in cases]
        out = stock_states(environment, plain) if plain else {}
        for n in cases:
            out[n] = _case_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._disckit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_disckit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._disckit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
