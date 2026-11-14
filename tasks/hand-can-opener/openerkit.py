"""openerkit: rigid parts built from boxes and cylinders, optionally with hinged links, for yam_arena task files.

Keep this file next to the task file:

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from openerkit import Hinge, part, on_step, on_reset, weld, clamp

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body made of several box and cylinder
colliders. Give it ``hinges`` and it becomes an articulation: each hinge is a link on a revolute joint about an axis
of the part frame, hung from the part's body or from another hinge, with optional angle limits and a spring and
damper drive (a spring toward its start angle, or a pure damper). A part without a mass is a fixed base: its body is
fixed to the world.

A box may carry a fourth element, a dict with ``yaw`` or ``pitch`` (rad about the part's z or y axis), ``visual`` (True: drawn,
but it does not collide) and/or ``hidden`` (True: not drawn); a cylinder a sixth, with ``visual`` and/or ``hidden``. ``meshes``
are drawn only: ``(points, faces, colour)`` in the part frame. A colour may carry a fourth value, its metalness. A part
may start in any orientation (``quat``).

``clamp(...)`` authors a revolute joint with an angular drive between two parts' bodies, disabled at the start;
the task switches it on and off (``set_clamp``) and moves its drive target (``set_clamp_target``).

``weld(part, to_part, link)`` joins a loose part to a link of another part with a fixed joint that the task can
release later with ``set_weld(part, False)``.

Importing this module hooks ``yam_arena.scene`` (and the embodiment's step and reset) without modifying yam_arena:
``success()/score()`` receive a :class:`PartState` (the body origin's pose plus joint angles) for parts.
``on_step`` and ``on_reset`` register callbacks that run after every control step and after every reset.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["Hinge", "Part", "PartState", "part", "weld", "set_weld", "clamp", "set_clamp", "set_clamp_target",
           "set_targets", "on_reset", "on_step", "env", "gripper_points"]

Color = tuple[float, float, float]
# box: (centre, size, colour); cylinder: (centre, radius, height, colour[, axis "X" | "Y" | "Z"]); part frame, m


@dataclass(frozen=True)
class Hinge:
    """A link on a revolute joint. Its shapes are given in the part frame with the joint at 0; ``anchor`` is a point on
    the joint axis (part frame) and ``axis`` one of "X", "Y", "Z". The link starts at ``start`` (rad). A drive of
    ``rate`` N m/rad pulls it toward ``start`` and ``damping`` N m s/rad resists its speed."""

    name: str
    anchor: tuple[float, float, float]
    axis: str = "Z"
    boxes: tuple = ()
    cylinders: tuple = ()
    meshes: tuple = ()
    parent: str = ""  # "" hangs it from the part's body, otherwise from the hinge of that name
    lower: float | None = None  # rad
    upper: float | None = None
    start: float = 0.0
    rate: float = 0.0
    damping: float = 0.0
    mass: float = 0.05
    armature: float = 0.0   # kg m^2 added about the joint (e.g. a gear train's inertia)
    friction: float = 0.0   # joint friction coefficient (PhysX)
    target: float | None = None  # spring's rest angle (rad), if not ``start`` (e.g. past a stop, so it presses on it)

    @property
    def rest(self) -> float:
        return self.start if self.target is None else self.target


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is its bounding box
    and ``pos`` is the body origin. ``mass=None`` fixes its body to the world."""

    boxes: tuple = ()
    cylinders: tuple = ()
    meshes: tuple = ()
    hinges: tuple[Hinge, ...] = ()
    friction: float = 0.8
    body_mass: float = 1.0
    quat: tuple | None = None  # start orientation (XYZW); overrides yaw
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not (self.boxes or self.cylinders):
            raise ValueError(f"{self.name}: a Part needs at least one shape")
        if self.mass is None and not self.hinges:
            raise ValueError(f"{self.name}: a fixed Part needs hinges (use a plain Box otherwise)")


def _cyl(spec):
    centre, radius, height, color = spec[:4]
    return centre, radius, height, color, (spec[4] if len(spec) > 4 else "Z")


def _cyl_half(spec):
    _, r, h, _, axis = _cyl(spec)
    return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])


def part(name, pos, boxes=(), mass=None, yaw=0.0, hinges=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         body_mass=1.0, quat=None, meshes=()) -> Part:
    """Build a Part; ``size`` is filled in from the body's shapes."""
    half = [np.array(b[1]) / 2 for b in boxes] + [_cyl_half(c) for c in cylinders]
    centres = [np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cylinders]
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), cylinders=tuple(cylinders), meshes=tuple(meshes), hinges=tuple(hinges), friction=friction,
                body_mass=body_mass if mass is None else mass, quat=None if quat is None else tuple(float(v) for v in quat),
                meta={"lo": tuple(lo.tolist()), "hi": tuple(hi.tolist())})


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each hinge's angle (rad)."""

    joints: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)


# -- welds --------------------------------------------------------------------------------------------------------

_WELDS: dict[str, tuple[str, str]] = {}  # loose part -> (other part, link name)
_WELD_PATHS: dict[str, list[str]] = {}


def weld(loose: Part, to: Part, link: str) -> None:
    """Join ``loose`` (at its start pose) to ``link`` of ``to`` with a fixed joint. List ``to`` before ``loose``."""
    _WELDS[loose.name] = (to.name, link)


def set_weld(name: str, enabled: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for path in _WELD_PATHS.get(name, []):
        stage.GetPrimAtPath(path).GetAttribute("physics:jointEnabled").Set(bool(enabled))


# -- clamps (a revolute joint with a drive between two parts' bodies, switched on and off by the task) ---------

_CLAMPS: dict[str, dict] = {}
_CLAMP_PATHS: dict[str, str] = {}


def clamp(part_: Part, other: Part, local_self, local_other, stiffness=20.0, damping=1.0, max_torque=5.0) -> None:
    """A revolute joint about z between ``other``'s body (frame at ``local_other``) and ``part_``'s body (frame at
    ``local_self``), both frames unrotated; an angular drive (N m/rad, N m s/rad, at most ``max_torque`` N m).
    List ``other`` before ``part_``. Disabled at the start."""
    _CLAMPS[part_.name] = dict(other=other.name, local_self=tuple(local_self), local_other=tuple(local_other),
                               stiffness=stiffness, damping=damping, max_torque=max_torque)


def _clamp_prim(name: str):
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage().GetPrimAtPath(_CLAMP_PATHS[name])


def set_clamp(name: str, enabled: bool) -> None:
    _clamp_prim(name).GetAttribute("physics:jointEnabled").Set(bool(enabled))


def set_clamp_target(name: str, angle: float) -> None:
    """Drive target (rad), wrapped to (-pi, pi]."""
    a = (angle + math.pi) % (2 * math.pi) - math.pi
    _clamp_prim(name).GetAttribute("drive:angular:physics:targetPosition").Set(float(math.degrees(a)))


# -- spawning -----------------------------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}


def _link_anchor(spec: Part, link: str) -> np.ndarray:
    if not link:
        return np.zeros(3)
    return np.array(next(h.anchor for h in spec.hinges if h.name == link), float)


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the body and links as siblings."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            metal = color[3] if len(color) > 3 else 0.0
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color[:3]), roughness=0.45 - 0.2 * metal,
                                              metallic=metal)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(path: str, pos, mass: float, wxyz=(1.0, 0.0, 0.0, 0.0)):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*wxyz))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateAngularDampingAttr(0.5)
        return prim

    def collide(prim, color, visual=False):
        if visual:
            bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
            return
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0005)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def shapes(path: str, box_specs, cyl_specs, offset=(0.0, 0.0, 0.0), mesh_specs=()):
        off = np.array(offset, float)
        for i, b in enumerate(box_specs):
            centre, size, color = b[:3]
            opt = b[3] if len(b) > 3 else {}
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            if opt.get("yaw"):
                cube.AddRotateZOp().Set(math.degrees(opt["yaw"]))
            if opt.get("pitch"):
                cube.AddRotateYOp().Set(math.degrees(opt["pitch"]))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color, opt.get("visual", False))
            if opt.get("hidden"):
                cube.MakeInvisible()
        for i, c in enumerate(cyl_specs):
            centre, radius, height, color, axis = _cyl(c)
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(axis)
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - off)))
            opt = c[5] if len(c) > 5 else {}
            collide(cyl.GetPrim(), color, opt.get("visual", False))
            if opt.get("hidden"):
                cyl.MakeInvisible()
        for i, (pts, faces, color) in enumerate(mesh_specs):
            mesh = UsdGeom.Mesh.Define(stage, f"{path}/mesh_{i}")
            mesh.CreatePointsAttr([Gf.Vec3f(*(np.array(q) - off)) for q in pts])
            mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
            mesh.CreateFaceVertexIndicesAttr([k for f in faces for k in f])
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            collide(mesh.GetPrim(), color, True)

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.body_mass)
    shapes(main, spec.boxes, spec.cylinders, mesh_specs=spec.meshes)
    paths = {"": main}
    for h in spec.hinges:
        path = f"{prim_path}/{h.name}"
        paths[h.name] = path
        # author the link already turned to its start angle (relative to its parent's start pose), so the
        # simulation starts without snapping it there
        ax = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[h.axis]
        half = h.start / 2
        body(path, h.anchor, h.mass, (math.cos(half), *(math.sin(half) * a for a in ax)))
        shapes(path, h.boxes, h.cylinders, offset=h.anchor, mesh_specs=h.meshes)
        parent_anchor = _link_anchor(spec, h.parent)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{h.name}_joint")
        j.CreateAxisAttr(h.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[h.parent])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(h.anchor) - parent_anchor)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if h.lower is not None:
            j.CreateLowerLimitAttr(math.degrees(h.lower))
            j.CreateUpperLimitAttr(math.degrees(h.upper))
        # USD drives are per degree; the spring pulls toward the start angle
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate * math.pi / 180)
        drive.CreateDampingAttr(h.damping * math.pi / 180)
        drive.CreateTargetPositionAttr(math.degrees(h.rest))
        state = PhysxSchema.JointStateAPI.Apply(j.GetPrim(), "angular")
        state.CreatePositionAttr(math.degrees(h.start))
        state.CreateVelocityAttr(0.0)
    if spec.mass is None:
        fix = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fix_to_world")
        fix.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        world = UsdGeom.Xformable(stage.GetPrimAtPath(main)).ComputeLocalToWorldTransform(0)
        fix.CreateLocalPos0Attr(Gf.Vec3f(*world.ExtractTranslation()))
        fix.CreateLocalRot0Attr(Gf.Quatf(world.ExtractRotationQuat()))
        fix.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        fix.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    if spec.hinges:
        art_prim = stage.GetPrimAtPath(prim_path if spec.mass is None else main)
        UsdPhysics.ArticulationRootAPI.Apply(art_prim)
        art = PhysxSchema.PhysxArticulationAPI.Apply(art_prim)
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    if spec.name in _WELDS:
        other, link = _WELDS[spec.name]
        ospec = _SPECS[other]
        other_path = prim_path.rsplit("/", 1)[0] + "/" + other
        target = f"{other_path}/body" if not link else f"{other_path}/{link}"
        # the loose part's origin in the link's frame at the start pose (the other part is not turned)
        rel = np.array(spec.pos) - np.array(ospec.pos) - _link_anchor(ospec, link)
        wj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld")
        wj.CreateBody0Rel().SetTargets([Sdf.Path(target)])
        wj.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        wj.CreateLocalPos0Attr(Gf.Vec3f(*rel))
        wj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        wj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        wj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        wj.CreateExcludeFromArticulationAttr(True)
        wj.CreateJointEnabledAttr(True)
        wj.CreateCollisionEnabledAttr(True)  # the parts still touch where they meet, and after the weld is released
        _WELD_PATHS.setdefault(spec.name, []).append(str(wj.GetPrim().GetPath()))
    if spec.name in _CLAMPS:
        c = _CLAMPS[spec.name]
        other_path = prim_path.rsplit("/", 1)[0] + "/" + c["other"]
        cj = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/clamp")
        cj.CreateAxisAttr("Z")
        cj.CreateBody0Rel().SetTargets([Sdf.Path(f"{other_path}/body")])
        cj.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        cj.CreateLocalPos0Attr(Gf.Vec3f(*c["local_other"]))
        cj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        cj.CreateLocalPos1Attr(Gf.Vec3f(*c["local_self"]))
        cj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        cj.CreateExcludeFromArticulationAttr(True)
        cj.CreateJointEnabledAttr(False)
        cj.CreateCollisionEnabledAttr(False)
        drive = UsdPhysics.DriveAPI.Apply(cj.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(c["stiffness"] * math.pi / 180)
        drive.CreateDampingAttr(c["damping"] * math.pi / 180)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateMaxForceAttr(c["max_torque"])
        _CLAMP_PATHS[spec.name] = str(cj.GetPrim().GetPath())
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            # Implicit actuators carry the springs and dampers; their target is each hinge's start angle.
            actuators = {h.name: ImplicitActuatorCfg(joint_names_expr=[f"{h.name}_joint"], stiffness=h.rate,
                                                     damping=h.damping, friction=h.friction,
                                                     armature=h.armature) for h in spec.hinges}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {f"{h.name}_joint": h.start for h in spec.hinges}
            cfg.init_state.joint_vel = {".*": 0.0}
            return cfg

    kind = ObjectType.ARTICULATION if spec.hinges else ObjectType.RIGID
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=spec.quat if spec.quat is not None else (
            0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    probe = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                      low=np.zeros(3), high=np.zeros(3))
    lo, hi = np.array(spec.meta["lo"]), np.array(spec.meta["hi"])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = np.array([probe.to_world(c) for c in corners])
    joints = {}
    if spec.hinges:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        joints = {h.name: float(q[names.index(f"{h.name}_joint")]) for h in spec.hinges}
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), joints=joints)


def set_targets(environment, name: str, targets: dict[str, float]) -> None:
    """Set drive targets (rad) of some hinges of an articulated part (env 0)."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    ids = [names.index(f"{k}_joint") for k in targets]
    t = torch.tensor([list(targets.values())], dtype=torch.float32, device=asset.device)
    asset.set_joint_position_target(t, joint_ids=ids)


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def gripper_points(environment) -> list[np.ndarray]:
    """World points (env frame) on both grippers: the palm, the centre between the finger pads and each pad tip."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = []
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        names = list(robot.body_names)
        g = _np(robot.data.body_link_pose_w)[0][names.index("gripper")]
        R, p = _rot(g[3:7]), g[:3] - origin
        jn = list(robot.joint_names)
        half = abs(float(_np(robot.data.joint_pos)[0][jn.index("joint7")])) + 0.008
        for local in ((0, 0, 0.06), (0, 0, 0.134), (half, 0, 0.147), (-half, 0, 0.147), (half, 0, 0.12),
                      (-half, 0, 0.12)):
            out.append(p + R @ np.array(local, float))
    return out


# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset, once objects have settled."""
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_hingekit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _part_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._hingekit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_hingekit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _ENV["env"] = self._env
        for spec in _SPECS.values():  # drive targets back to each hinge's start angle (springs, dampers)
            if spec.hinges:
                set_targets(self._env, spec.name, {h.name: h.rest for h in spec.hinges})
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._hingekit_patched = True


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
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
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
assert task_file
