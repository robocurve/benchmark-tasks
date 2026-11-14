"""spinnerkit: a hand-crank salad spinner, and fixed parts with their own friction, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from spinnerkit import Spinner, Fixed, on_reset, on_step

- :class:`Fixed` is a fixed box with its own friction and look.
- :class:`Spinner` is a loose 2-quart salad spinner: one articulation of the bowl with its lid (the root, free on the
  table), the crank on the lid, the crank's knob and the basket inside. In the spinner's frame z is up from the middle
  of its base. The crank and the basket turn about z; the knob spins freely on the crank's end.

  ``crank`` joint: the crank's angle on the lid (rad). Turning it clockwise seen from above (negative) spins the basket.
  ``knob`` joint: the knob turning on the crank (free, so a hand can hold it while the crank goes round).
  ``basket`` joint: the basket's angle in the bowl.

  The gearing is modelled in :func:`_mechanism`, run every control step. The basket is geared 1:RATIO to the crank
  through a one-way clutch: while the crank is turned forward faster than the basket's own speed allows, the basket is
  driven to RATIO times the crank's speed; otherwise it coasts, slowed by the drag of the lettuce in the air. The torque
  the gears take to spin the basket up, and the gears' own friction, are put back on the crank as resistance, and (as
  in a real gearbox) onto the lid and bowl.

``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

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

__all__ = ["Fixed", "KitState", "Spinner", "body_pose", "env", "finger_points", "install", "knob_point", "on_reset",
           "on_step", "turns"]

# -- a 2-quart hand-crank salad spinner (m), in its own frame -----------------------------------------------------------
BOWL_R, BOWL_H = 0.100, 0.110                    # the bowl: outer radius, height to its rim
WALL = 0.003
LID_R, LID_Z = 0.103, (0.110, 0.120)             # the lid sits on the rim
HUB_R, HUB_Z = 0.020, (0.120, 0.127)             # the gear housing at the lid's middle
CRANK_Z = 0.127                                  # the crank turns on top of the housing
ARM = (0.066, 0.016, 0.008)                      # the crank's arm: long (from the axis), wide, thick
KNOB_AT = 0.055                                  # the knob's axis, this far out from the crank's axis
KNOB_R, KNOB_H = 0.011, 0.034                    # the knob, standing up from the arm's end
BASKET_R, BASKET_Z = 0.088, (0.008, 0.100)       # the basket inside the bowl
TAB = (0.005, 0.060, 0.025)                      # the bowl's side handle on its +y side: thick (x), out (y), tall (z)
TAB_Z = 0.050                                    # the handle's middle, this high
RATIO = 5.0                                      # basket turns per crank turn
FORWARD = -1.0                                   # clockwise seen from above spins the basket
BASKET_I = 8.6e-4                                # kg m^2: the basket with 150 g of lettuce, about its axis
DRAG = 2.2e-4                                    # N m s/rad: the lettuce's drag on the spinning basket (coasts ~4 s)
GEAR_FRICTION = 0.020                            # N m s/rad: the gears' own friction, felt at the crank
COUPLE = 50.0                                    # how hard the clutch holds the basket to its geared speed ...
CLUTCH_MAX = 0.010                               # N m: ... up to this torque on the basket (it spins up in about 1.5 s)
CRANK_DRIVE = 0.05                               # N m s/rad: the crank joint's drive (applies the resistance above)


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction and look."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.6
    metallic: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Spinner(Box):
    """A salad spinner. ``pos`` is the middle of its base, ``quat`` (xyzw) its orientation, ``mass`` the bowl with its
    lid and gears (the crank, knob and basket with its lettuce are added)."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    friction: tuple = (0.25, 0.20)               # the bowl's plastic base on the worktop
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Spinner must be loose (give it a mass)")


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``joints`` maps the spinner's joints to their angles (rad)."""

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


def gear(crank_vel: float, basket_vel: float):
    """The gearing at joint speeds (rad/s, signed as the joints are): the torque the clutch puts on the basket, and the
    torque the gears put on the crank (resistance). Pure: tested offline.

    Forward is clockwise seen from above. The clutch engages only while the crank, geared up, would spin the basket
    forward faster than it is going; then it drives the basket toward that speed and the crank feels that torque
    times RATIO. Otherwise the basket coasts against the lettuce's drag. The gears' friction always resists the crank."""
    c, b = FORWARD * crank_vel, FORWARD * basket_vel           # forward speeds
    want = RATIO * c
    drive = min(CLUTCH_MAX, COUPLE * DRAG * (want - b)) if want > b else 0.0    # the clutch's torque on the basket
    on_basket = drive - DRAG * b
    on_crank = -RATIO * drive - GEAR_FRICTION * c
    return FORWARD * on_basket, FORWARD * on_crank


def knob_offset(crank: float) -> np.ndarray:
    """The knob's axis on the crank (spinner frame, at the arm's top) at crank angle ``crank``."""
    return np.array([KNOB_AT * math.cos(crank), KNOB_AT * math.sin(crank), CRANK_Z + ARM[2]])


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

    def body(self, path, pos, mass, lin_damp=0.05, ang_damp=0.05, inertia=None):
        from pxr import PhysxSchema, UsdPhysics

        xf = self.UsdGeom.Xform.Define(self.stage, path)
        if path != self.path and pos is not None:
            xf.AddTranslateOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*pos))
            xf.AddOrientOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(self.Gf.Vec3f(*(float(v) for v in inertia)))
            m.CreateCenterOfMassAttr(self.Gf.Vec3f(0.0, 0.0, 0.0))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        return prim

    def collide(self, prim, mat, offset=0.001):
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

    def box(self, path, centre, size, look, mat=None, yaw=None):
        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if yaw is not None:
            cube.AddOrientOp().Set(self.Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
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
        if look:
            self.paint(c.GetPrim(), look)
        if mat:
            self.collide(c.GetPrim(), mat)
        return c.GetPrim()


# -- spawning ------------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
JOINTS = ("crank", "knob", "basket")


def _spawn_spinner(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The bowl with its lid (the articulation's root); the crank and the basket on revolute joints about the spinner's
    axis; the knob on a revolute joint at the crank's end. Bodies' origins: bowl and basket at the base's middle, the
    crank on the axis at the housing's top, the knob on its own axis there."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Spinner = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    clear = a.look((0.90, 0.95, 0.95), rough=0.1, opacity=0.35)
    lid_look = a.look((0.88, 0.93, 0.93), rough=0.1, opacity=0.45)
    green = a.look((0.25, 0.65, 0.30), rough=0.6)
    grey = a.look((0.35, 0.37, 0.40), rough=0.5)
    white = a.look((0.96, 0.96, 0.94), rough=0.5, opacity=0.9)
    lettuce = a.look((0.45, 0.75, 0.30), rough=0.8)
    red = a.look((0.85, 0.15, 0.12), rough=0.5)
    m_bowl = a.mat("bowl", *spec.friction)
    m_grip = a.mat("grip", 1.2, 1.1)                                    # soft rubber on the knob and handle

    bowl = f"{prim_path}/bowl"
    a.body(bowl, (0.0, 0.0, 0.0), spec.mass)
    a.cyl(f"{bowl}/shell", BOWL_R, 0.0, BOWL_H, clear, m_bowl)              # one solid collider: nothing goes inside
    a.cyl(f"{bowl}/lid", LID_R, LID_Z[0], LID_Z[1], lid_look, m_bowl)
    a.cyl(f"{bowl}/hub", HUB_R, HUB_Z[0], HUB_Z[1], green, m_bowl)
    a.box(f"{bowl}/tab", (0.0, BOWL_R + TAB[1] / 2 - 0.002, TAB_Z), TAB, green, m_grip)
    a.box(f"{bowl}/mark", (0.0, LID_R - 0.012, LID_Z[1] + 0.0005), (0.004, 0.016, 0.001), red)   # crank count mark

    basket = f"{prim_path}/basket"
    a.body(basket, (0.0, 0.0, 0.0), 0.27, ang_damp=0.0, inertia=(1.0e-3, 1.0e-3, BASKET_I))
    a.cyl(f"{basket}/wall", BASKET_R, BASKET_Z[0], BASKET_Z[1], white)
    for k in range(7):                                                    # washed lettuce leaves
        ang = 2 * math.pi * k / 7
        r = 0.045 if k % 2 else 0.060
        a.box(f"{basket}/leaf_{k}", (r * math.cos(ang), r * math.sin(ang), 0.03 + 0.008 * (k % 3)),
              (0.050, 0.035, 0.012), lettuce, yaw=ang + 0.6)
    a.box(f"{basket}/stripe", (BASKET_R - 0.004, 0.0, BASKET_Z[1] - 0.006), (0.010, 0.020, 0.012), red)

    crank = f"{prim_path}/crank"
    a.body(crank, (0.0, 0.0, CRANK_Z), 0.020, ang_damp=0.0)
    a.box(f"{crank}/arm", (ARM[0] / 2 - 0.006, 0.0, ARM[2] / 2), ARM, grey, m_grip)
    a.cyl(f"{crank}/boss", 0.012, 0.0, ARM[2] + 0.004, grey)

    knob = f"{prim_path}/knob"
    a.body(knob, (KNOB_AT, 0.0, CRANK_Z + ARM[2]), 0.008, ang_damp=0.0)
    a.cyl(f"{knob}/grip", KNOB_R, 0.0, KNOB_H, green, m_grip)

    stage = a.stage
    for name, parent, child, at in (("crank", bowl, crank, (0.0, 0.0, CRANK_Z)),
                                    ("basket", bowl, basket, (0.0, 0.0, 0.0)),
                                    ("knob", crank, knob, (KNOB_AT, 0.0, ARM[2]))):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{child}/{name}")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(parent)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*at))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(bowl))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(bowl))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return a.root


def _spinner_object(spec: Spinner):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class SpinnerSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_spinner)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"crank": ImplicitActuatorCfg(joint_names_expr=["crank"], stiffness=0.0, damping=CRANK_DRIVE),
                         "basket": ImplicitActuatorCfg(joint_names_expr=["basket"], stiffness=0.0, damping=DRAG),
                         "knob": ImplicitActuatorCfg(joint_names_expr=["knob"], stiffness=0.0, damping=1.0e-5)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=SpinnerSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=tuple(float(v) for v in spec.quat)))


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
    """Change one joint of spinner ``name`` in the running simulation."""
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


def _joint_state(environment, name: str):
    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    q, v = _np(asset.data.joint_pos)[0], _np(asset.data.joint_vel)[0]
    return names, {n: float(q[names.index(n)]) for n in names}, {n: float(v[names.index(n)]) for n in names}


def _joints(environment, name: str) -> dict:
    return _joint_state(environment, name)[1]


_MECH: dict = {}


def turns(name: str) -> dict:
    """What the spinner's joints have done since the reset: forward turns of the crank on the lid (net, backward turns
    subtract), forward turns of the basket in the bowl, and the basket's speed now (turns per second)."""
    st = _MECH.get(name, {})
    return {"crank": st.get("crank_turns", 0.0), "basket": st.get("basket_turns", 0.0),
            "basket_rps": st.get("basket_rps", 0.0)}


def _velocity_targets(asset, names, targets: dict) -> None:
    import torch

    ids = [names.index(n) for n in targets]
    dev = _t(asset.data.joint_pos).device
    t = torch.tensor([list(targets.values())], dtype=torch.float32, device=dev)
    fn = getattr(asset, "set_joint_velocity_target_index", None)
    if fn is not None:
        fn(target=t, joint_ids=ids, env_ids=[0])
    else:
        asset.set_joint_velocity_target(t, joint_ids=ids)


def _mechanism(environment) -> None:
    """The gearing, every control step: drives with no stiffness and damping D apply D * (target - speed), so each
    torque is set through its velocity target."""
    for name, spec in _SPECS.items():
        if not isinstance(spec, Spinner):
            continue
        asset = environment.unwrapped.scene[name]
        names, q, v = _joint_state(environment, name)
        st = _MECH.setdefault(name, {"last": dict(q), "crank_turns": 0.0, "basket_turns": 0.0, "basket_rps": 0.0})
        for j, key in (("crank", "crank_turns"), ("basket", "basket_turns")):
            st[key] += FORWARD * math.remainder(q[j] - st["last"][j], 2 * math.pi) / (2 * math.pi)
        st["last"] = dict(q)
        st["basket_rps"] = FORWARD * v["basket"] / (2 * math.pi)
        on_basket, on_crank = gear(v["crank"], v["basket"])
        _velocity_targets(asset, names, {"basket": v["basket"] + on_basket / DRAG,
                                         "crank": v["crank"] + on_crank / CRANK_DRIVE,
                                         "knob": 0.0})


def _mechanism_reset(environment) -> None:
    _MECH.clear()


def _kit_state(environment, spec) -> KitState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    joints = _joints(environment, spec.name)
    lo, hi = np.array([-BOWL_R, -BOWL_R, 0.0]), np.array([BOWL_R, BOWL_R, CRANK_Z + ARM[2] + KNOB_H])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                    joints=joints)


def knob_point(pos, quat, crank: float, up: float = 0.0) -> np.ndarray:
    """A point on the knob's axis (world, relative to the env origin), ``up`` above the arm's top, for a spinner at
    ``pos``, ``quat`` with its crank at angle ``crank``."""
    return np.asarray(pos, float) + _rotation(quat) @ (knob_offset(crank) + np.array([0.0, 0.0, up]))


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
    if getattr(scene, "_spinnerkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Spinner):
            return _spinner_object(spec)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), Spinner)}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._spinnerkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_spinnerkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        _mechanism(self._env)
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        _mechanism_reset(self._env)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._spinnerkit_patched = True


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
