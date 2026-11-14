"""partkit: rigid parts built from several boxes, optionally with spring-loaded sliders, for yam_arena task files.

A task file uses it like this (keep this file next to the task file):

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from partkit import Part, Slider, on_step, on_reset

    LID = Part("lid", pos=(0.2, 0.0, 0.76), yaw=0.0, mass=0.04,
               boxes=[((0, 0, 0), (0.114, 0.064, 0.0075), (0.1, 0.3, 0.9))])
    OBJECTS = [LID, ...]

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body with several box and cylinder colliders
(a box with a pocket, a lid with tabs, a cap with lugs). Give it ``sliders`` and it becomes an articulation whose root is the part and whose
sliders are boxes on prismatic joints along the part's z axis, each pushed up to its rest stop by a spring drive.
A box may carry a fourth element, its rotation in the part frame as a (w, x, y, z) quaternion, for chamfers and
ramps; a slider may have its own ``friction``, which wins its contacts (PhysX's "min" combine mode).

Importing this module hooks ``yam_arena.scene`` (and the embodiment's step and reset) without modifying yam_arena: ``success()/score()`` receive a :class:`PartState` (the body origin's pose, not the bounding-box centre that
yam_arena reports for primitives, plus slider positions). ``on_step`` and ``on_reset`` register callbacks that run
after every control step and after every reset, with the Isaac Lab environment.

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

__all__ = ["Hinge", "Part", "PartState", "Slider", "on_reset", "on_step", "env", "write_pose", "install"]

Color = tuple[float, float, float]
# centre, size, colour (part frame, m), and optionally the box's rotation in the part frame as a (w, x, y, z) quaternion
BoxSpec = tuple
CylSpec = tuple[tuple[float, float, float], float, float, Color]  # centre, radius, height (axis = part z), colour


@dataclass(frozen=True)
class Slider:
    """A box on a prismatic joint along the part's z axis. ``rest`` is where its frame sits in the part frame; it can
    move down by ``travel`` and a spring of ``rate`` N/m pushes it back up to ``rest``."""

    name: str
    rest: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...]
    travel: float = 0.004
    rate: float = 2500.0
    damping: float = 20.0
    mass: float = 0.002
    friction: float | None = None  # its own material, which wins its contacts; None uses the part's
    parent: str = "body"
    spring_zero: float = 0.0
    friction_effort: float = 0.0
    axis: str = "Z"  # the parent-frame axis it slides along
    pressed: float = -1.0  # -1: pressed toward -axis (limits -travel..0); +1: pressed toward +axis (0..travel)


@dataclass(frozen=True)
class Hinge:
    """A rigid arm on a revolute joint about the part's z axis through ``pivot`` (part frame). Its ``boxes`` and
    ``cylinders`` are given in the hinge frame (origin at the pivot) at angle 0. It starts at ``start`` (rad) and
    can turn from ``lower`` to ``upper``; a torsion spring of ``rate`` N m/rad pulls it toward ``spring_zero``,
    which may lie beyond a limit (a preloaded spring)."""

    name: str
    parent: str = field(kw_only=True, default="body")
    axis: str = field(kw_only=True, default="Z")
    friction_effort: float = field(kw_only=True, default=0.0)
    pivot: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...]
    cylinders: tuple = ()
    lower: float = -0.5
    upper: float = 0.0
    start: float = 0.0
    spring_zero: float = 0.0
    rate: float = 1.0
    damping: float = 0.02
    mass: float = 0.05
    friction: float | None = None  # its own material, which wins its contacts; None uses the part's


@dataclass(frozen=True)
class Part(Box):
    """A rigid part made of ``boxes``. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is
    its bounding box and ``pos`` is the body origin."""

    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple[CylSpec, ...] = ()
    sliders: tuple[Slider, ...] = ()
    hinges: tuple[Hinge, ...] = ()
    friction: float = 0.8
    patch_radius: float = 0.0  # minimum torsional patch radius (m) of its contacts: friction against twisting
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.boxes:
            raise ValueError(f"{self.name}: a Part needs at least one box")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Part must be loose (give it a mass)")


def _quat_matrix(wxyz) -> np.ndarray:
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _box(spec):
    """(centre, size, colour, rotation wxyz) of a box spec, with no rotation when it has none."""
    centre, size, color = spec[:3]
    return centre, size, color, (tuple(spec[3]) if len(spec) > 3 else (1.0, 0.0, 0.0, 0.0))


def part(name, pos, boxes, mass, yaw=0.0, sliders=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         hinges=(), patch_radius=0.0) -> Part:
    """Build a Part; ``size`` is filled in from the shapes' extent."""
    half = [np.abs(_quat_matrix(q)) @ (np.array(s) / 2) for _, s, _, q in map(_box, boxes)]
    half += [np.array([r, r, h / 2]) for _, r, h, _ in cylinders]
    centres = [np.array(b[0]) for b in boxes] + [np.array(c) for c, _, _, _ in cylinders]
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), cylinders=tuple(cylinders), sliders=tuple(sliders), friction=friction,
                hinges=tuple(hinges), patch_radius=patch_radius)


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each slider's position (m, 0 at its
    rest stop, negative when pressed)."""

    sliders: dict = field(default_factory=dict)
    hinges: dict = field(default_factory=dict)  # each hinge's angle (rad)

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)


# -- spawning -----------------------------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the body (and sliders) as siblings."""
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
    materials = {}
    for s in (*spec.sliders, *spec.hinges):
        if s.friction is not None:
            materials[s.name] = f"{prim_path}/Looks/physics_{s.name}"
            sm = sim_utils.RigidBodyMaterialCfg(static_friction=s.friction, dynamic_friction=s.friction,
                                                friction_combine_mode="min")
            sm.func(materials[s.name], sm)
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(path: str, pos, mass: float, yaw: float = 0.0):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateAngularDampingAttr(1.0)  # light damping: small stiff parts otherwise chatter in tight fits
        return prim

    def collide(prim, color, material=phys):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0005)  # below the 1 mm fits, so tight parts do not touch on every face at once
        cp.CreateRestOffsetAttr(0.0)
        if spec.patch_radius > 0:
            cp.CreateMinTorsionalPatchRadiusAttr(spec.patch_radius)
        bind_physics_material(str(prim.GetPath()), material, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def boxes(path: str, specs, offset=(0.0, 0.0, 0.0), material=phys):
        for i, (centre, size, color, rot) in enumerate(map(_box, specs)):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            if rot != (1.0, 0.0, 0.0, 0.0):
                cube.AddOrientOp().Set(Gf.Quatf(*rot))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color, material)

    def cylinders(path: str, specs, offset=(0.0, 0.0, 0.0)):
        for i, (centre, radius, height, color) in enumerate(specs):
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr("Z")
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            collide(cyl.GetPrim(), color)

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.mass)
    boxes(main, spec.boxes)
    cylinders(main, spec.cylinders)
    anchors = {"body": np.zeros(3)}
    anchors.update({s.name: np.array(s.rest) for s in spec.sliders})
    anchors.update({h.name: np.array(h.pivot) for h in spec.hinges})

    def passive_friction(joint, axis, effort):
        p = joint.GetPrim()
        p.AddAppliedSchema("PhysxJointAxisAPI:" + axis)
        for kind in ("staticFrictionEffort", "dynamicFrictionEffort"):
            p.CreateAttribute("physxJointAxis:" + axis + ":" + kind, Sdf.ValueTypeNames.Float).Set(effort)
        joint.CreateCollisionEnabledAttr(False)

    for s in spec.sliders:
        path = f"{prim_path}/{s.name}"
        body(path, s.rest, s.mass)
        boxes(path, s.boxes, offset=s.rest, material=materials.get(s.name, phys))
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{path}/{s.name}_slide")
        j.CreateAxisAttr(s.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{s.parent}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(s.rest) - anchors[s.parent])))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(-s.travel if s.pressed < 0 else 0.0)
        j.CreateUpperLimitAttr(0.0 if s.pressed < 0 else s.travel)
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(s.rate)
        drive.CreateDampingAttr(s.damping)
        drive.CreateTargetPositionAttr(s.spring_zero)
        passive_friction(j, "linear", s.friction_effort)
    for h in spec.hinges:
        path = f"{prim_path}/{h.name}"
        body(path, h.pivot, h.mass, yaw=h.start)
        boxes(path, h.boxes, material=materials.get(h.name, phys))
        cylinders(path, h.cylinders)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{h.name}_turn")
        j.CreateAxisAttr(h.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{h.parent}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(h.pivot) - anchors[h.parent])))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(math.degrees(h.lower))
        j.CreateUpperLimitAttr(math.degrees(h.upper))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate * math.pi / 180)
        drive.CreateDampingAttr(h.damping * math.pi / 180)
        drive.CreateTargetPositionAttr(math.degrees(h.spring_zero))
        passive_friction(j, "angular", h.friction_effort)
    if spec.sliders or spec.hinges:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
        # A hinged part's arm must touch its catch and pad (siblings); parent-child pairs never collide.
        art.CreateEnabledSelfCollisionsAttr(bool(spec.hinges))
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
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
            # Implicit actuators carry the springs; their default target is each slider's rest stop.
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[f"{s.name}_slide"], stiffness=s.rate,
                                                     damping=s.damping, friction=s.friction_effort, dynamic_friction=s.friction_effort) for s in spec.sliders}
            actuators.update({h.name: ImplicitActuatorCfg(joint_names_expr=[f"{h.name}_turn"], stiffness=h.rate,
                                                          damping=h.damping, friction=h.friction_effort, dynamic_friction=h.friction_effort) for h in spec.hinges})
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            joint_pos = {f"{s.name}_slide": 0.0 for s in spec.sliders}
            joint_pos.update({f"{h.name}_turn": h.start for h in spec.hinges})
            cfg.init_state.joint_pos = joint_pos
            return cfg

    kind = ObjectType.ARTICULATION if (spec.sliders or spec.hinges) else ObjectType.RIGID
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2),
                                                                       math.cos(spec.yaw / 2))),
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
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = np.array([probe.to_world(c) for c in corners])
    sliders, hinges = {}, {}
    if spec.sliders or spec.hinges:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        sliders = {s.name: float(q[names.index(f"{s.name}_slide")]) for s in spec.sliders}
        hinges = {h.name: float(q[names.index(f"{h.name}_turn")]) for h in spec.hinges}
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), sliders=sliders, hinges=hinges)



def _reset_joint_positions(environment) -> None:
    """Restore articulated parts to their declared start ONLY at an episode reset.

    The station reset event restores object root poses but does not restore these
    task-owned passive joints. Reset them before the SDK settles/renders the new
    episode, so a previous locked handle cannot carry over into its first frame.
    """
    import torch

    base = environment.unwrapped
    for spec in _SPECS.values():
        if not (spec.sliders or spec.hinges):
            continue
        asset = base.scene[spec.name]
        names = list(asset.joint_names)
        positions = torch.zeros_like(_t(asset.data.joint_pos))
        for hinge in spec.hinges:
            positions[:, names.index(f"{hinge.name}_turn")] = hinge.start
        asset.write_joint_state_to_sim_index(position=positions, velocity=torch.zeros_like(positions))
        # The SDK's subsequent reset settles with the same initial targets as
        # the first episode. Preloaded spring targets are restored afterward.
        asset.set_joint_position_target_index(target=positions)


def _set_spring_zeros(environment) -> None:
    """Point each hinge's spring at its zero (which may lie beyond a limit: a preloaded spring). Isaac Lab starts
    every drive at the joint's start position, so the spring is set after each reset."""
    import torch

    base = environment.unwrapped
    for spec in _SPECS.values():
        if not (spec.hinges or spec.sliders):
            continue
        asset = base.scene[spec.name]
        names = list(asset.joint_names)
        ids = [names.index(f"{h.name}_turn") for h in spec.hinges] + [names.index(f"{s.name}_slide") for s in spec.sliders]
        target = torch.tensor([[h.spring_zero for h in spec.hinges] + [s.spring_zero for s in spec.sliders]], dtype=torch.float32,
                              device=_t(base.scene.env_origins).device)
        asset.set_joint_position_target_index(target=target, joint_ids=ids)


def write_pose(environment, name: str, pos, quat_xyzw, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)) -> None:
    """Teleport a part's body (env 0) to a world pose (relative to the env origin) with the given velocity."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins[0])
    pose = torch.tensor([*pos, *quat_xyzw], dtype=origin.dtype, device=origin.device).unsqueeze(0)
    pose[:, :3] += origin
    asset.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([*lin_vel, *ang_vel], dtype=origin.dtype, device=origin.device).unsqueeze(0)
    asset.write_root_velocity_to_sim_index(root_velocity=v)


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
    if getattr(scene, "_partkit_patched", False):
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

    original_build = scene.build_task_environment
    def build(*args, **kwargs):
        environment = original_build(*args, **kwargs)
        task = kwargs.get("task")
        if task is not None and any(isinstance(o, Part) and o.name == "monitor" for o in task.objects):
            previous = environment.env_cfg_callback
            def configure(cfg):
                if previous is not None:
                    cfg = previous(cfg)
                # Physical 35 cm pedestals are declared task objects. This only
                # places the robot bases on them; robot hardware remains standard.
                for robot in (cfg.scene.left_robot, cfg.scene.right_robot):
                    xyz = robot.init_state.pos
                    robot.init_state.pos = (xyz[0], xyz[1], xyz[2] + .35)
                return cfg
            environment.env_cfg_callback = configure
        return environment
    scene.build_task_environment = build

    scene.task_object = task_object
    scene.object_states = object_states
    scene._partkit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_partkit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        if getattr(self, "_partkit_has_reset", False):
            _reset_joint_positions(self._env)
        obs = base_reset(self, *a, **k)
        self._partkit_has_reset = True
        _ENV["env"] = self._env
        _set_spring_zeros(self._env)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._partkit_patched = True


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
