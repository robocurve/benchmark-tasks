"""cartonkit: a corrugated carton as four rigid panels on four crease hinges, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from cartonkit import Carton, on_reset, on_step, set_crease_spring

    CARTON = Carton("carton", pos=(0.22, 0.0, 0.7475), across=0.20, tall=0.15, long=0.30, ...)
    OBJECTS = [CARTON]

A regular slotted carton lying on its side with both ends open is a tube of four panels joined along four creases.
Each panel here is one rigid board (its end flaps folded out flat, in line with it); each crease is a revolute joint
along the tube. The four joints form a closed loop: the bottom panel is the articulation's root, the near wall, the
top panel and the far wall hang from it in a chain, and the fourth crease (far wall to bottom) is a loop-closing
joint kept out of the articulation. So the carton has one way to move, racking: the cross-section shears from a
rectangle to a flat double layer, toward the far side or the near side.

Every crease has a spring toward the open (square) shape, which is how a used carton's creases hold it up. A task
file can switch the springs off at run time with :func:`set_crease_spring`, which is how the creases "set" once the
carton has been pressed flat.

``success()`` and ``score()`` receive a :class:`CartonState`: the root panel's pose plus ``rack_deg`` (how far the
cross-section has sheared, 0 square, +/-90 flat) and every panel's pose. ``on_step`` and ``on_reset`` register
callbacks ``fn(env)`` that run after every control step (before the task file is scored) and after every reset.

The hooks follow the kits used by tasks already in this repository (a Box subclass so the loader accepts the part,
a custom spawner, hooks installed through ``sys.meta_path``). Pure Python at import time; Isaac Sim is reached only
when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Carton", "CartonState", "PANELS", "env", "finger_points", "install", "on_reset", "on_step",
           "set_crease_spring"]

# Panels in chain order from the root, and the crease each one hangs from (the last crease closes the loop).
PANELS = ("bottom", "near", "top", "far")
CREASES = ("crease_near_bottom", "crease_near_top", "crease_far_top", "crease_far_bottom")
_SIGN = (1.0, -1.0, 1.0, -1.0)     # each crease turns by sign * rack angle as the carton racks toward the far side


@dataclass(frozen=True)
class Carton(Box):
    """A carton tube lying on its side, in its own frame: x across (near wall at x=0, far wall at x=``across``),
    y along the tube (centred), z up from the table. ``pos`` is where that frame's origin (the near bottom crease, at
    table level, middle of the tube) starts in the world; ``yaw`` turns it.

    ``across`` and ``tall`` are the cross-section between the boards' mid-planes, ``long`` the length of each panel
    including its two end flaps, ``flap`` the depth of a flap (drawn as a crease line only), ``board`` the board's
    thickness. ``crease`` is each crease's spring (N m/rad toward the square shape) and ``crease_damping`` its friction
    (N m s/rad). ``rack_limit`` (degrees) is how far the cross-section can shear before the boards meet."""

    across: float = 0.20
    tall: float = 0.15
    long: float = 0.30
    flap: float = 0.075
    board: float = 0.003
    panel_mass: tuple = (0.033, 0.025, 0.033, 0.025)    # bottom, near, top, far (kg)
    crease: float = 0.12
    crease_damping: float = 0.004
    rack_limit: float = 88.0
    static_friction: float = 0.45
    dynamic_friction: float = 0.38
    combine: str = "average"
    crease_color: tuple = (0.45, 0.33, 0.20)
    contact_offset: float = 0.002
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Carton must be loose (give it a mass)")
        if not 0 < self.rack_limit < 90:
            raise ValueError(f"{self.name}: rack_limit must be between 0 and 90 degrees")

    # -- geometry in the carton's frame, carton square -----------------------------------------------------------
    def corners(self) -> dict:
        """The four creases' lines (x, z) in the carton frame, on the boards' mid-planes."""
        t, a, h = self.board / 2, self.across, self.tall
        return {"near_bottom": (0.0, t), "far_bottom": (a, t), "far_top": (a, t + h), "near_top": (0.0, t + h)}

    def panel_boxes(self) -> dict:
        """Each panel's board as (centre, size) in the carton frame, square. Boards stop short of the creases so that
        neighbouring boards do not overlap."""
        t, a, h, L = self.board, self.across, self.tall, self.long
        return {"bottom": ((a / 2, 0.0, t / 2), (a - 2 * t, L, t)),
                "near": ((0.0, 0.0, t / 2 + h / 2), (t, L, h - 2 * t)),
                "top": ((a / 2, 0.0, t / 2 + h), (a - 2 * t, L, t)),
                "far": ((a, 0.0, t / 2 + h / 2), (t, L, h - 2 * t))}

    def hinge_points(self) -> tuple:
        c = self.corners()
        return (c["near_bottom"], c["near_top"], c["far_top"], c["far_bottom"])

    def panel_pose(self, rack_deg: float) -> dict:
        """Each panel's centre and its turn about y (radians) in the carton frame, for a given rack, with the bottom
        panel where it started. Positive rack shears toward the far side."""
        r = math.radians(rack_deg)
        c, s = math.cos(r), math.sin(r)
        (x0, z0), h = self.corners()["near_bottom"], self.tall
        boxes = self.panel_boxes()
        top_shift = np.array([h * s, 0.0, h * c - h])
        wall = lambda centre, pivot: np.array([pivot[0] + (centre[2] - pivot[1]) * s, 0.0, pivot[1] + (centre[2] - pivot[1]) * c])
        return {"bottom": (np.array(boxes["bottom"][0]), 0.0),
                "near": (wall(boxes["near"][0], (x0, z0)), r),
                "top": (np.array(boxes["top"][0]) + top_shift, 0.0),
                "far": (wall(boxes["far"][0], self.corners()["far_bottom"]), r)}


@dataclass(frozen=True)
class CartonState(ObjectState):
    """The carton as the simulator has it. ``pos``/``quat_xyzw`` are the bottom panel's pose (its frame's origin is
    the carton frame's origin). ``rack_deg`` is the shear of the cross-section (0 square, +/-90 flat), read from the
    near bottom crease. ``panels`` maps each panel to (centre, quat_xyzw, lin_vel); ``low``/``high`` bound all four."""

    rack_deg: float = 0.0
    panels: dict = field(default_factory=dict)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _yaw_xyzw(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _material(sim_utils, spec):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                              dynamic_friction=spec.dynamic_friction,
                                              friction_combine_mode=spec.combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                              dynamic_friction=spec.dynamic_friction)


# -- spawning ---------------------------------------------------------------------------------------------------------

_CARTONS: dict[str, Carton] = {}


def _spawn_carton(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the carton under ``prim_path`` (pxr only): four panel bodies and four crease joints."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Carton = _CARTONS[cfg.carton_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = _material(sim_utils, spec)
    pm.func(phys, pm)
    looks = {}
    for key, color in (("board", spec.color), ("crease", spec.crease_color)):
        path = f"{prim_path}/Looks/{key}"
        vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.85)
        vis.func(path, vis)
        looks[key] = path

    boxes = spec.panel_boxes()
    # The prim's origin is the bottom panel's centre (the articulation root is placed there); panels sit relative to it.
    root_c = np.array(boxes["bottom"][0])
    paths = {}
    for i, name in enumerate(PANELS):
        centre, size = boxes[name]
        path = f"{prim_path}/{name}"
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(np.array(centre) - root_c)))
        xform.AddOrientOp(precision=double).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(spec.panel_mass[i]))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateLinearDampingAttr(0.05)
        rb.CreateAngularDampingAttr(0.5)
        cube = UsdGeom.Cube.Define(stage, f"{path}/board")
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        cp = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        cp.CreateContactOffsetAttr(spec.contact_offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(cube.GetPath()), phys, stage=stage)
        bind_visual_material(str(cube.GetPath()), looks["board"], stage=stage)
        # The flaps' creases, drawn on both faces (no collider): thin dark lines across the board at each end.
        thin = np.argmin(size)
        for j, yy in enumerate((-spec.long / 2 + spec.flap, spec.long / 2 - spec.flap)):
            for k, side in enumerate((-1.0, 1.0)):
                line = UsdGeom.Cube.Define(stage, f"{path}/crease_line_{j}{k}")
                line.CreateSizeAttr(1.0)
                off = [0.0, yy, 0.0]
                off[thin] = side * (size[thin] / 2 + 0.0002)
                sc = list(size)
                sc[1], sc[thin] = 0.003, 0.0002
                line.AddTranslateOp().Set(Gf.Vec3d(*off))
                line.AddScaleOp().Set(Gf.Vec3f(*sc))
                bind_visual_material(str(line.GetPath()), looks["crease"], stage=stage)
        paths[name] = path

    chain = [("bottom", "near"), ("near", "top"), ("top", "far"), ("far", "bottom")]
    for i, ((a, b), crease, (hx, hz)) in enumerate(zip(chain, CREASES, spec.hinge_points())):
        ca, cb = np.array(boxes[a][0]), np.array(boxes[b][0])
        pivot = np.array([hx, 0.0, hz])
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{paths[b] if i < 3 else paths[a]}/{crease}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[a])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[b])])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(pivot - ca)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(pivot - cb)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        lim = float(spec.rack_limit)
        j.CreateLowerLimitAttr(-lim)
        j.CreateUpperLimitAttr(lim)
        if i == 3:
            # The loop-closing crease: a plain joint outside the articulation, with no spring of its own (the other
            # three creases carry the whole spring, see _body_object).
            j.CreateExcludeFromArticulationAttr(True)
        else:
            drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(math.radians(chain_spring(spec)))   # USD angular drives are per degree
            drive.CreateDampingAttr(math.radians(spec.crease_damping))
            drive.CreateTargetPositionAttr(0.0)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(paths["bottom"]))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(paths["bottom"]))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def chain_spring(spec: Carton) -> float:
    """Spring on each of the three articulated creases: together they hold the carton as the four creases would."""
    return spec.crease * 4.0 / 3.0


def _body_object(spec: Carton):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class CartonSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_carton)
        carton_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"creases": ImplicitActuatorCfg(joint_names_expr=list(CREASES[:3]), stiffness=chain_spring(spec),
                                                        damping=spec.crease_damping)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    # The articulation's root pose is the bottom panel's centre: the carton frame's origin moved by that offset.
    c, s_ = math.cos(spec.yaw), math.sin(spec.yaw)
    bx, by, bz = spec.panel_boxes()["bottom"][0]
    centre = (spec.pos[0] + c * bx - s_ * by, spec.pos[1] + s_ * bx + c * by, spec.pos[2] + bz)
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=CartonSpawnerCfg(carton_name=spec.name),
                     initial_pose=Pose(position_xyz=centre, rotation_xyzw=_yaw_xyzw(spec.yaw)))


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _carton_state(environment, spec: Carton) -> CartonState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    poses = _np(asset.data.body_link_pose_w)[0]
    vels = _np(asset.data.body_link_vel_w)[0] if hasattr(asset.data, "body_link_vel_w") else np.zeros((len(poses), 6))
    names = list(asset.body_names)
    boxes = spec.panel_boxes()
    panels, pts = {}, []
    for p in PANELS:
        k = names.index(p)
        pos, quat = poses[k][:3] - origin, poses[k][3:7]
        panels[p] = (pos, quat, vels[k][:3])
        half = np.array(boxes[p][1]) / 2
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
        pts.append(corners @ _rotation(quat).T + pos)
    pts = np.concatenate(pts)
    q = _np(asset.data.joint_pos)[0]
    rack = math.degrees(float(q[list(asset.joint_names).index(CREASES[0])]))
    # The bottom panel's body origin is its centre; report the carton frame's origin, as the spec's pos is given.
    bpos, bquat, bvel = panels["bottom"]
    frame_pos = bpos - _rotation(bquat) @ np.array(boxes["bottom"][0])
    root_vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    return CartonState(pos=frame_pos, quat_xyzw=bquat, lin_vel=bvel, ang_vel=root_vel[3:6], low=pts.min(0),
                       high=pts.max(0), rack_deg=rack, panels=panels)


def set_crease_spring(environment, carton_name: str, spring: float) -> None:
    """Set the carton's crease spring (N m/rad per crease, as ``Carton.crease``) in the running simulation."""
    import torch

    spec = _CARTONS[carton_name]
    asset = environment.unwrapped.scene[carton_name]
    names = list(asset.joint_names)
    ids = [names.index(c) for c in CREASES[:3]]
    value = torch.full((1, len(ids)), float(spring) * 4.0 / 3.0, device=_t(asset.data.joint_pos).device)
    write = getattr(asset, "write_joint_stiffness_to_sim_index", None)
    if write is not None:
        write(stiffness=value, joint_ids=ids, env_ids=[0])
    else:
        asset.write_joint_stiffness_to_sim(value, joint_ids=ids, env_ids=[0])
    del spec


def finger_points(environment) -> list:
    """The grasp point of each gripper (world, relative to the env origin), for "has it let go" checks in an
    ``on_step`` hook; ``score()`` itself does not see the robot."""
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
    if getattr(scene, "_cartonkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Carton):
            _CARTONS[spec.name] = spec
            return _body_object(spec)
        return stock_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _CARTONS]
        out = stock_states(environment, plain) if plain else {}
        for n in names:
            if n in _CARTONS:
                out[n] = _carton_state(environment, _CARTONS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._cartonkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_cartonkit_patched", False):
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
    cls._cartonkit_patched = True


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
