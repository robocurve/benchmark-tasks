"""binkit: fixed-base articulated hinged display bin for yam_arena task files.

Hooks ``yam_arena.scene`` (chaining cleanly with yamkit) so a :class:`HingedBin` spec in ``OBJECTS`` spawns as a
world-anchored PhysX articulation whose clear front door pivots freely under gravity about a horizontal ``+Y`` hinge
within ``[0, upper_deg]`` degrees with configurable viscous damping and zero holding torque.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["BinLook", "BinState", "HingedBin", "closed_bounds", "hinged_bin", "install"]


@dataclass(frozen=True)
class BinLook:
    """Visual surface properties matching yamkit.Look."""

    color: tuple[float, float, float]
    roughness: float = 0.5
    metallic: float = 0.0
    opacity: float = 1.0


BoxSpec = tuple  # (centre, size, color_or_BinLook[, collider: bool])


def _as_look(c) -> BinLook:
    return c if isinstance(c, BinLook) else BinLook(tuple(float(v) for v in c))


def _box_fields(spec) -> tuple[tuple[float, float, float], tuple[float, float, float], BinLook, bool]:
    centre, size, mat = spec[0], spec[1], spec[2]
    collider = bool(spec[3]) if len(spec) > 3 else True
    return tuple(float(v) for v in centre), tuple(float(v) for v in size), _as_look(mat), collider


@dataclass(frozen=True)
class HingedBin(Box):
    """Fixed countertop display bin with a gravity-closing hinged door turning about ``+Y`` at ``hinge``."""

    boxes: tuple[BoxSpec, ...] = ()
    door_boxes: tuple[BoxSpec, ...] = ()
    hinge: tuple[float, float, float] = (0.0, 0.0, 0.0)
    door_mass: float = 0.28
    upper_deg: float = 82.0
    damping: float = 0.004
    friction: float = 0.6
    bbox_centre: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a HingedBin is anchored; leave mass=None")
        if not (self.boxes and self.door_boxes):
            raise ValueError(f"{self.name}: requires both boxes and door_boxes")
        if not (self.door_mass > 0 and self.damping >= 0 and 0 < self.upper_deg < 90):
            raise ValueError(f"{self.name}: invalid door_mass, damping, or upper_deg in (0, 90)")


def _extent(boxes) -> tuple[np.ndarray, np.ndarray]:
    lo = np.min([np.array(b[0]) - np.array(b[1]) / 2 for b in boxes], axis=0)
    hi = np.max([np.array(b[0]) + np.array(b[1]) / 2 for b in boxes], axis=0)
    return lo, hi


def hinged_bin(
    name: str,
    pos,
    boxes,
    door_boxes,
    hinge,
    door_mass: float = 0.28,
    upper_deg: float = 82.0,
    damping: float = 0.004,
    yaw: float = 0.0,
    color=(0.82, 0.88, 0.90),
    friction: float = 0.6,
) -> HingedBin:
    """Build a HingedBin; computes closed world extent over base and closed door boxes."""
    hinge_v = np.asarray(hinge, float)
    all_shapes = list(boxes) + [((hinge_v + np.asarray(b[0], float)).tolist(), b[1], b[2]) for b in door_boxes]
    lo, hi = _extent(all_shapes)
    return HingedBin(
        name=name,
        pos=tuple(float(v) for v in pos),
        size=tuple((hi - lo).tolist()),
        color=tuple(float(v) for v in color),
        yaw=float(yaw),
        boxes=tuple(boxes),
        door_boxes=tuple(door_boxes),
        hinge=tuple(float(v) for v in hinge),
        door_mass=float(door_mass),
        upper_deg=float(upper_deg),
        damping=float(damping),
        friction=float(friction),
        bbox_centre=tuple(((lo + hi) / 2).tolist()),
    )


def closed_bounds(spec: HingedBin) -> tuple[np.ndarray, np.ndarray]:
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    corners = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]) * np.array(spec.size) / 2
    pts = np.array(spec.pos) + (rot @ (corners + np.array(spec.bbox_centre)).T).T
    return pts.min(axis=0), pts.max(axis=0)


def _quat_mul_xyzw(a, b) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rot_from_quat_xyzw(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


@dataclass(frozen=True)
class BinState(ObjectState):
    """Articulated bin state: housing pose plus door hinge ``angle`` (rad), ``door_vel`` (rad/s), and door frame."""

    angle: float = 0.0
    door_vel: float = 0.0
    door_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    door_quat_xyzw: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))

    def door_as_object(self) -> ObjectState:
        """Expose the door frame as an ObjectState for contact-guard checks."""
        return ObjectState(
            pos=self.door_pos,
            quat_xyzw=self.door_quat_xyzw,
            lin_vel=np.zeros(3),
            ang_vel=np.array([0.0, self.door_vel, 0.0]),
            low=self.low,
            high=self.high,
        )


def compute_door_frame(base_pos, base_quat_xyzw, hinge_local, angle_rad: float) -> tuple[np.ndarray, np.ndarray]:
    """Return world ``(door_pos, door_quat_xyzw)`` for hinge origin turned by ``angle_rad`` around base ``+Y``."""
    rb = _rot_from_quat_xyzw(base_quat_xyzw)
    d_pos = np.asarray(base_pos, float) + rb @ np.asarray(hinge_local, float)
    hy = (0.0, math.sin(angle_rad / 2.0), 0.0, math.cos(angle_rad / 2.0))
    d_quat = np.asarray(_quat_mul_xyzw(tuple(float(v) for v in base_quat_xyzw), hy), float)
    return d_pos, d_quat


# -- spawning & state readout (Isaac Sim, imported lazily) -----------------------------------------------------------

_SPECS: dict[str, HingedBin] = {}


def _joint_name(spec: HingedBin) -> str:
    return f"{spec.name}_door_hinge"


def _spawn_bin(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.bin_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    qx, qy, qz, qw = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))

    physics_material = f"{prim_path}/Looks/physics"
    material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    material.func(physics_material, material)
    looks: dict[BinLook, str] = {}

    def look_path(lk: BinLook) -> str:
        if lk not in looks:
            path = f"{prim_path}/Looks/m{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(
                diffuse_color=linear_rgb(lk.color),
                roughness=lk.roughness,
                metallic=lk.metallic,
                opacity=lk.opacity,
            )
            surface.func(path, surface)
            looks[lk] = path
        return looks[lk]

    def rigid_body(path: str, pos, mass: float):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        return prim

    def add_boxes(path: str, boxes):
        for i, item in enumerate(boxes):
            centre, size, lk, do_col = _box_fields(item)
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            prim = cube.GetPrim()
            if do_col:
                UsdPhysics.CollisionAPI.Apply(prim)
                col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                col.CreateContactOffsetAttr(0.001)
                col.CreateRestOffsetAttr(0.0)
                bind_physics_material(str(prim.GetPath()), physics_material, stage=stage)
            bind_visual_material(str(prim.GetPath()), look_path(lk), stage=stage)

    body_path = f"{prim_path}/body"
    door_path = f"{prim_path}/door"
    body_prim = rigid_body(body_path, (0.0, 0.0, 0.0), 5.0)
    add_boxes(body_path, spec.boxes)
    rigid_body(door_path, spec.hinge, spec.door_mass)
    add_boxes(door_path, spec.door_boxes)

    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{door_path}/{_joint_name(spec)}")
    joint.CreateAxisAttr("Y")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(door_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*spec.hinge))
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(spec.upper_deg)

    UsdPhysics.ArticulationRootAPI.Apply(body_prim)
    art = PhysxSchema.PhysxArticulationAPI.Apply(body_prim)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)

    anchor = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/anchor")
    anchor.CreateBody1Rel().SetTargets([Sdf.Path(body_path)])
    world = UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(0)
    where, rot = world.ExtractTranslation(), world.ExtractRotationQuat()
    anchor.CreateLocalPos0Attr(Gf.Vec3f(*where))
    anchor.CreateLocalRot0Attr(Gf.Quatf(rot.GetReal(), *rot.GetImaginary()))
    anchor.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    anchor.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    return root.GetPrim()


def _arena_object(spec: HingedBin):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BinSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bin)
        bin_name: str = ""

    class HingedBinObject(Object):
        def _generate_articulation_cfg(self):
            act = {"door": ImplicitActuatorCfg(joint_names_expr=[_joint_name(spec)], stiffness=0.0, damping=spec.damping)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=act)
            return self._add_initial_pose_to_cfg(cfg)

    return HingedBinObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=BinSpawnerCfg(bin_name=spec.name),
        initial_pose=Pose(
            position_xyz=tuple(spec.pos),
            rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2.0), math.cos(spec.yaw / 2.0)),
        ),
    )


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _bin_state(environment, spec: HingedBin) -> BinState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    bpos, bquat = pose[:3] - origin, pose[3:7]
    jidx = list(asset.joint_names).index(_joint_name(spec))
    angle = float(_np(asset.data.joint_pos)[0][jidx])
    vel = float(_np(asset.data.joint_vel)[0][jidx])
    dpos, dquat = compute_door_frame(bpos, bquat, spec.hinge, angle)
    low, high = closed_bounds(spec)
    return BinState(
        pos=bpos,
        quat_xyzw=bquat,
        lin_vel=np.zeros(3),
        ang_vel=np.zeros(3),
        low=low,
        high=high,
        angle=angle,
        door_vel=vel,
        door_pos=dpos,
        door_quat_xyzw=dquat,
    )


# -- hooking yam_arena.scene ----------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_binkit_patched", False):
        return
    base_task_object, base_object_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, HingedBin):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return base_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        states = base_object_states(environment, plain)
        for n in _SPECS:
            if n in names:
                states[n] = _bin_state(environment, _SPECS[n])
        return states

    scene.task_object = task_object
    scene.object_states = object_states
    scene._binkit_patched = True


class _SceneHook(importlib.abc.MetaPathFinder):
    target = "yam_arena.scene"

    def find_spec(self, fullname, path, target=None):
        if fullname != self.target:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    if _SceneHook.target in sys.modules:
        _patch_scene(sys.modules[_SceneHook.target])
    if not any(isinstance(f, _SceneHook) for f in sys.meta_path):
        sys.meta_path.insert(0, _SceneHook())


install()
