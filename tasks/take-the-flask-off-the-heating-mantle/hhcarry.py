"""hhcarry: a glass reagent bottle and a rubber safety bottle carrier for a yam_arena task file.

- :class:`Bottle`: one rigid body made of upright cylinders (body, shoulder, neck, cap), each a collider, spawned the
  way yam_arena spawns its own cylinder (the body) with the other cylinders added as extra colliders and looks.
- :class:`Carrier`: an open round bucket (floor plus a ring of wall segments) with a wire bail that pivots on two lugs
  on its sides and a grip sleeve that turns freely on the bail's top bar, as on real rubber bottle carriers. One PhysX
  articulation: root link ``bucket``, child ``bail`` (revolute about the bucket's own y, between two stops), and
  ``sleeve`` on the bail (revolute about the bar). Built the way the accepted cable task's cablekit spawns its cable.

Importing this module hooks ``yam_arena.scene`` so these specs in ``OBJECTS`` are spawned as above, and
``success()/score()`` receive a :class:`CarrierState` for a carrier (the bucket's pose plus the bail angle and the
sleeve's position). A bottle's state is yam_arena's own rigid-object state. yam_arena is not modified.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState


@dataclass(frozen=True)
class Bottle(Box):
    """An upright bottle. ``pos`` is its body cylinder's centre; ``parts`` lists the cylinders above the body as
    (name, radius, height, colour), stacked from the body's top up; ``size`` is its bounding box (set by the task).
    A part whose name starts with "bulb" is a sphere of that radius centred on the middle of its band (the band may be
    shorter than the sphere, so the sphere can reach below it, e.g. down past a tiny body to the flask's bottom)."""

    body_radius: float = 0.0375
    body_height: float = 0.115
    parts: tuple = ()
    friction: tuple = (1.0, 0.9)

    def part_bands(self) -> dict:
        """Each part's (z_low, z_high) above the bottle's base."""
        out, z = {"body": (0.0, self.body_height)}, self.body_height
        for name, _r, h, _c in self.parts:
            out[name] = (z, z + h)
            z += h
        return out

    @property
    def height(self) -> float:
        return self.body_height + sum(p[2] for p in self.parts)


@dataclass(frozen=True)
class Carrier(Box):
    """An open round carrier standing on its floor. ``pos`` is the centre of its floor's underside."""

    r_in: float = 0.050
    wall: float = 0.006
    height: float = 0.10
    floor: float = 0.008
    n_seg: int = 14
    pivot_z: float = 0.088  # the bail's pivots, above the floor's underside
    bail_len: float = 0.14  # pivot axis to the top bar's axis
    bail_rest_deg: float = 45.0  # the bail rests on its stops this far over toward the carrier's own +x (and -x)
    bail_mass: float = 0.06
    sleeve_r: float = 0.011
    sleeve_len: float = 0.07
    sleeve_mass: float = 0.015
    bail_damping: float = 0.002  # N m s/rad at the pivots (a little friction)
    friction: tuple = (1.0, 0.9)
    bail_color: tuple = (0.75, 0.76, 0.78)
    sleeve_color: tuple = (0.80, 0.10, 0.10)

    @property
    def free(self) -> bool:  # not jittered by yam_arena's reset (it writes rigid poses only)
        return False

    @property
    def r_out(self) -> float:
        return self.r_in + self.wall

    @property
    def pivot_y(self) -> float:
        return self.r_out + 0.008


@dataclass(frozen=True)
class CarrierState(ObjectState):
    bail_deg: float = 0.0  # 0 upright; +rest_deg resting toward the carrier's own +x
    sleeve_pos: np.ndarray = None
    base_pos: np.ndarray = None  # the floor's underside centre
    up: np.ndarray = None  # the carrier's own +z in the world


_SPECS: dict = {}


def _geom(stage, path, kind, dims, xyz, quat_wxyz=(1.0, 0.0, 0.0, 0.0), collide=True, phys=None, look=None):
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    if kind == "box":
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in xyz]))
        g.AddOrientOp().Set(Gf.Quatf(*[float(v) for v in quat_wxyz]))
        g.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in dims]))
    elif kind == "sph":  # a sphere of radius dims[0]
        g = UsdGeom.Sphere.Define(stage, path)
        g.CreateRadiusAttr(float(dims[0]))
        g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in xyz]))
    else:  # cylinder along its own z (or the axis the quat turns it to)
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateRadiusAttr(float(dims[0]))
        g.CreateHeightAttr(float(dims[1]))
        g.CreateAxisAttr("Z")
        g.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in xyz]))
        g.AddOrientOp().Set(Gf.Quatf(*[float(v) for v in quat_wxyz]))
    prim = g.GetPrim()
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.004)
        cp.CreateRestOffsetAttr(0.0)
        if phys:
            bind_physics_material(str(prim.GetPath()), phys, stage=stage)
    if look:
        bind_visual_material(str(prim.GetPath()), look, stage=stage)
    return prim


def _look(path, color, rough=0.5, metallic=0.0, opacity=1.0):
    import isaaclab.sim as sim_utils

    from yam_arena.station import linear_rgb

    lk = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metallic,
                                     opacity=opacity)
    lk.func(path, lk)
    return path


# -- the bottle ---------------------------------------------------------------------------------------------------
def _bottle_object(spec: Bottle, scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cylinder
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    parts = tuple(spec.parts)
    bh = spec.body_height

    @clone
    def _spawn(prim_path, cfg, *args, **kwargs):
        prim = spawn_cylinder.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        phys = f"{prim_path}/Looks/bottle_physics"
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
        pm.func(phys, pm)
        z = bh / 2  # body top, in the body's frame (centred)
        for name, r, h, color in parts:
            look = _look(f"{prim_path}/Looks/{name}", color, rough=0.3)
            kind = "sph" if name.startswith("bulb") else "cyl"  # a "bulb" part is a sphere centred on its band
            _geom(stage, f"{prim_path}/parts/{name}", kind, (r, h), (0.0, 0.0, z + h / 2), phys=phys, look=look)
            z += h
        return prim

    cfg = sim_utils.CylinderCfg(
        radius=spec.body_radius, height=bh, func=_spawn,
        rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0],
                                                        dynamic_friction=spec.friction[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.25))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


# -- the carrier --------------------------------------------------------------------------------------------------
def _quat_y(deg):  # wxyz, a turn about +y
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, math.sin(h), 0.0)


def _quat_z(rad):
    return (math.cos(rad / 2), 0.0, 0.0, math.sin(rad / 2))


def _link(stage, path, xyz, quat_wxyz, mass):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*xyz))
    xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*quat_wxyz))
    prim = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateLinearDampingAttr(0.05)
    rb.CreateAngularDampingAttr(0.2)
    return path


def _spawn_carrier(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    spec: Carrier = _SPECS[cfg.carrier_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    phys = f"{prim_path}/Looks/carrier_physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
    pm.func(phys, pm)
    rubber = _look(f"{prim_path}/Looks/rubber", spec.color, rough=0.8)
    steel = _look(f"{prim_path}/Looks/steel", spec.bail_color, rough=0.3, metallic=0.8)
    grip = _look(f"{prim_path}/Looks/grip", spec.sleeve_color, rough=0.6)

    bucket_mass = spec.mass if spec.mass is not None else 0.45
    b = _link(stage, f"{prim_path}/bucket", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), bucket_mass)
    _geom(stage, f"{b}/floor", "cyl", (spec.r_out, spec.floor), (0.0, 0.0, spec.floor / 2), phys=phys, look=rubber)
    rm = spec.r_in + spec.wall / 2
    seg = 2 * (spec.r_out) * math.tan(math.pi / spec.n_seg) + 0.002
    wh = spec.height - spec.floor
    for k in range(spec.n_seg):
        a = 2 * math.pi * k / spec.n_seg
        _geom(stage, f"{b}/wall_{k:02d}", "box", (spec.wall, seg, wh),
              (rm * math.cos(a), rm * math.sin(a), spec.floor + wh / 2), _quat_z(a), phys=phys, look=rubber)
    py = spec.pivot_y
    for s in (1, -1):  # the pivot lugs on the carrier's sides
        _geom(stage, f"{b}/lug_{'l' if s > 0 else 'r'}", "box", (0.016, py - spec.r_out + 0.002, 0.022),
              (0.0, s * (spec.r_out + (py - spec.r_out) / 2 - 0.001), spec.pivot_z), phys=phys, look=rubber)

    q_rest = _quat_y(spec.bail_rest_deg)
    bl = _link(stage, f"{prim_path}/bail", (0.0, 0.0, spec.pivot_z), q_rest, spec.bail_mass)
    rod = 0.005
    L = spec.bail_len
    for s in (1, -1):
        _geom(stage, f"{bl}/rod_{'l' if s > 0 else 'r'}", "box", (rod, rod, L), (0.0, s * (py + rod), L / 2),
              phys=phys, look=steel)
    _geom(stage, f"{bl}/bar", "box", (rod, 2 * (py + rod) + rod, rod), (0.0, 0.0, L), phys=phys, look=steel)
    s_pos = (L * math.sin(math.radians(spec.bail_rest_deg)), 0.0, spec.pivot_z + L * math.cos(math.radians(spec.bail_rest_deg)))
    sl = _link(stage, f"{prim_path}/sleeve", s_pos, q_rest, spec.sleeve_mass)
    _geom(stage, f"{sl}/grip", "cyl", (spec.sleeve_r, spec.sleeve_len), (0.0, 0.0, 0.0),
          (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0), phys=phys, look=grip)  # its axis along y

    def joint(name, body0, body1, pos0, lo, hi, damping):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{body1}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateAxisAttr("Y")
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if lo is not None:
            j.CreateLowerLimitAttr(lo)
            j.CreateUpperLimitAttr(hi)
        drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drv.CreateTypeAttr("force")
        drv.CreateStiffnessAttr(0.0)
        drv.CreateDampingAttr(damping * math.pi / 180)

    # joint frames: identity in both links, so the bail's angle is 0 upright and +rest_deg where it is spawned
    # (the joint angle is set to match at reset, see _arena_object)
    joint("bail_pivot", b, bl, (0.0, 0.0, spec.pivot_z), -abs(spec.bail_rest_deg), abs(spec.bail_rest_deg), spec.bail_damping)
    joint("sleeve_spin", bl, sl, (0.0, 0.0, L), None, None, 1e-5)
    root_link = stage.GetPrimAtPath(b)
    UsdPhysics.ArticulationRootAPI.Apply(root_link)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root_link)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _carrier_object(spec: Carrier):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class CarrierSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_carrier)
        carrier_name: str = ""

    rest = math.radians(spec.bail_rest_deg)

    class CarrierObject(Object):
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(
                prim_path=self.prim_path, spawn=self.spawner_cfg,
                actuators={"pivots": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=0.0,
                                                         damping=spec.bail_damping, armature=1e-5)})
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {"bail_pivot": rest, "sleeve_spin": 0.0}
            cfg.init_state.joint_vel = {".*": 0.0}
            return cfg

    return CarrierObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                         spawner_cfg=CarrierSpawnerCfg(carrier_name=spec.name),
                         initial_pose=Pose(position_xyz=tuple(spec.pos),
                                           rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _rot(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def carrier_state(spec: Carrier, base_pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0), bail_deg=None, vel=(0.0, 0.0, 0.0),
                  sleeve_pos=None) -> CarrierState:
    """A CarrierState from the bucket's base pose (also used offline by the tests)."""
    p = np.asarray(base_pos, float)
    R = _rot(quat_xyzw)
    up = R[:, 2]
    bail = spec.bail_rest_deg if bail_deg is None else float(bail_deg)
    if sleeve_pos is None:
        th = math.radians(bail)
        sleeve_pos = p + R @ np.array([spec.bail_len * math.sin(th), 0.0, spec.pivot_z + spec.bail_len * math.cos(th)])
    r, h = spec.r_out, spec.height
    corners = np.array([[sx * r, sy * r, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (0.0, h)])
    pts = p + corners @ R.T
    return CarrierState(pos=p + R @ np.array([0.0, 0.0, h / 2]), quat_xyzw=np.asarray(quat_xyzw, float),
                        lin_vel=np.asarray(vel, float), ang_vel=np.zeros(3), low=pts.min(0), high=pts.max(0),
                        bail_deg=bail, sleeve_pos=np.asarray(sleeve_pos, float), base_pos=p, up=up)


_LAST_STEP: dict = {}  # carrier name -> the episode step seen last (a smaller one means the scene was reset)


def _put_bail(art, spec: Carrier) -> None:
    """Lay the bail on its stop, at rest (the scene's reset leaves articulated parts where they were and does not
    apply the initial joint angle, so a bail spawned on its stop would otherwise start wherever it was left)."""
    import torch

    dev = _t(art.data.joint_pos).device
    jn = list(art.joint_names)
    ids = [jn.index("bail_pivot"), jn.index("sleeve_spin")]
    art.write_joint_position_to_sim_index(
        position=torch.tensor([[math.radians(spec.bail_rest_deg), 0.0]], dtype=torch.float32, device=dev), joint_ids=ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, 2), dtype=torch.float32, device=dev), joint_ids=ids)


def _carrier_state(env, spec: Carrier) -> CarrierState:
    base = env.unwrapped
    art = base.scene[spec.name]
    buf = getattr(base, "episode_length_buf", None)
    n = int(_np(buf)[0]) if buf is not None else 0
    if spec.name not in _LAST_STEP or n < _LAST_STEP[spec.name]:
        _put_bail(art, spec)
    _LAST_STEP[spec.name] = n
    origin = _np(base.scene.env_origins)[0]
    names = list(art.body_names)
    ib, isl = names.index("bucket"), names.index("sleeve")
    pos = _np(art.data.body_pos_w)[0] - origin
    q = _np(art.data.body_quat_w)[0][ib]  # Isaac Lab here gives (x, y, z, w)
    vel = _np(art.data.body_lin_vel_w)[0][ib]
    jn = list(art.joint_names)
    bail = math.degrees(float(_np(art.data.joint_pos)[0][jn.index("bail_pivot")]))
    return carrier_state(spec, pos[ib], q, bail, vel, pos[isl])


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhcarry", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Carrier):
            _SPECS[spec.name] = spec
            return _carrier_object(spec)
        if isinstance(spec, Bottle):
            return _bottle_object(spec, scene)
        return orig_task_object(spec)

    def object_states(env, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(env, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _carrier_state(env, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._hhcarry = task_object, object_states, True


class _Finder(importlib.abc.MetaPathFinder):
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


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
