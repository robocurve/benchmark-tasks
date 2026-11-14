"""trialkit: per-trial layouts, labelled boxes and staged demos for yam_arena task files. MIT licence.

Keep this file next to the task file and import it before declaring objects:

- ``on_reset(fn)``: ``fn(env, rng)`` runs after every reset, before the trial starts. ``rng`` is seeded from the
  trial's seed, so each trial can lay the objects out differently and a fixed eval seed repeats the same layouts.
- ``place(env, name, xyz)``: move a free object (keeping its orientation) and stop it.
- ``env()``: the running Isaac Lab environment, or None outside a simulation.
- ``top_image()``: the top camera's current RGB image (640 x 360). ``grippers()``: each gripper's TCP and opening.
- ``Labelled``: a Box with thin coloured panels fixed to it; the panels are visual only, the box is the collider.
- ``staged(stages)``: a scripted demo in stages. ``stages`` is a generator: it yields the first plan, then receives a
  ``View`` of the policy's observation each time a stage ends and yields the next plan. A ``View`` holds only what
  the policy observes (camera images, joint positions), so a stage is chosen from observations, not simulator state.
  Every arm a later stage moves needs a waypoint in the first stage.
"""

import importlib.abc
import math
import importlib.machinery
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import ik
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box

ARMS = ("left", "right")
_RESETS, _PANELS, _S = {}, {}, {"env": None, "gen": None}
_PARTS, _JOINTS, _WEIGHTLESS, _PATHS, _POS, _HIDDEN_BASE = {}, {}, set(), {}, {}, set()


def parts(name, *items):
    """Extra shapes fixed to object ``name``: ("box", (dx, dy, dz), (sx, sy, sz), rgb, collide[, wxyz]) or
    ("cyl", (dx, dy, dz), (radius, height), rgb, collide[, wxyz]), offsets and rotations in the object's frame.
    An ``rgb`` of ("hidden", rgb) starts invisible until ``show`` turns it on."""
    _PARTS[name] = items


def slide(name, parent, offset, travel, damping):
    """Join ``name`` to ``parent`` on a vertical slider: ``offset`` is ``name``'s origin in ``parent``'s frame at
    the start, it can move ``travel`` m down from there, and every m/s of motion is resisted by ``damping`` N."""
    _JOINTS[name] = ("slide", parent, offset, travel, damping)


def hinge(name, parent, offset, axis, limits_deg, stiffness, damping=0.0):
    """Join ``name`` to ``parent`` on a hinge through ``offset`` (in ``parent``'s frame; ``name``'s frame at the
    hinge is ``offset`` minus their start offset) about ``axis`` ("X", "Y" or "Z"), between ``limits_deg``, sprung
    back to its start angle with ``stiffness`` N m per degree."""
    _JOINTS[name] = ("hinge", parent, offset, axis, limits_deg, stiffness, damping)


def lock(name, parent, at=None):
    """Fix ``name`` to ``parent``: where they start, or with ``name``'s origin at ``at`` in ``parent``'s frame (a
    seated pose it snaps into when switched on). ``set_locked`` switches it while the simulation runs."""
    _JOINTS[name] = ("lock", parent, at)


def set_locked(name, locked):
    stage, path = _PATHS.get(name, (None, None))
    if stage is not None:
        stage.GetPrimAtPath(f"{path}/joint").GetAttribute("physics:jointEnabled").Set(bool(locked))


def set_drive(name, target_deg, stiffness):
    """Retarget ``name``'s hinge spring while the simulation runs (a latch that holds a cover shut, say)."""
    stage, path = _PATHS.get(name, (None, None))
    if stage is not None:
        joint = stage.GetPrimAtPath(f"{path}/joint")
        joint.GetAttribute("drive:angular:physics:targetPosition").Set(float(target_deg))
        joint.GetAttribute("drive:angular:physics:stiffness").Set(float(stiffness))


def show(name, index, visible=True):
    """Make part ``index`` of object ``name`` visible or not while the simulation runs."""
    from pxr import UsdGeom

    stage, path = _PATHS.get(name, (None, None))
    if stage is None:
        return
    look = UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/part_{index}"))
    look.MakeVisible() if visible else look.MakeInvisible()


class Tilt(float):
    """A waypoint ``yaw`` that also leans the gripper ``tilt`` rad off straight down, its fingertips swinging
    toward the horizontal heading ``toward`` (rad, 0 = +x, away from the arm bases)."""

    def __new__(cls, yaw, tilt, toward):
        out = super().__new__(cls, yaw)
        out.tilt, out.toward = float(tilt), float(toward)
        return out


def hide_base(name):
    """Draw object ``name`` by its parts only; its own shape still collides (a solid core inside a thin shell)."""
    _HIDDEN_BASE.add(name)


def weightless(name):
    """Gravity off for ``name`` (a part held in place by friction in the real object, such as a plunger)."""
    _WEIGHTLESS.add(name)


@dataclass(frozen=True)
class Labelled(Box):
    """``panels``: ((dx, dy, dz), (sx, sy, sz), (r, g, b)) per panel, centre and size in the box's frame."""

    panels: tuple = ()

    def __post_init__(self):
        _PANELS[self.name] = self.panels


def on_reset(fn):
    _RESETS[fn.__qualname__] = fn
    return fn


def env():
    return _S["env"]


def place(environment, name, xyz):
    import torch

    base = environment.unwrapped
    obj = base.scene[name]
    pose = obj.data.default_root_pose.torch.clone()
    pose[:, :3] = torch.tensor(xyz, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
    obj.write_root_pose_to_sim_index(root_pose=pose)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))


def top_image():
    if _S["env"] is None:
        return None
    out = _S["env"].unwrapped.scene["top_cam"].data.output["rgb"]
    out = out.torch if hasattr(out, "torch") else out
    return out[0, ..., :3].cpu().numpy()


def grippers():
    """{arm: (TCP position, opening 0..1)} in the task frame, or {} outside a simulation."""
    if _S["env"] is None:
        return {}
    from yam_arena import runtime
    from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN

    base = _S["env"].unwrapped
    origin = base.scene.env_origins[0].cpu().numpy()
    out = {}
    for arm in ARMS:
        robot = base.scene[f"{arm}_robot"]
        ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        opening = float(robot.data.joint_pos.torch[0, ids].mean().item()) / FINGER_OPEN
        out[arm] = (runtime.tcp_world(robot) - origin, opening)
    return out


class View:
    """What the scripted policy observes when a stage ends."""

    def __init__(self, observation, bases):
        self.images = observation.images
        self.joints = np.asarray(next(iter(observation.state.values())), dtype=float).reshape(-1)
        self.bases = bases

    def tcp(self, arm):
        q = self.joints[:7] if arm == "left" else self.joints[7:]
        return ik.tcp_pose(q[:6], self.bases[arm])[0]


def staged(stages):
    def demo():
        _S["gen"] = stages()
        return next(_S["gen"])

    return demo


def _patch_scene(scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone

    @clone
    def spawn_labelled(prim_path, cfg, *args, **kwargs):
        from pxr import Gf, UsdGeom

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        for i, (offset, size, color) in enumerate(_PANELS[prim_path.rsplit("/", 1)[-1]]):
            path = f"{prim_path}/panel_{i}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, offset)))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(color), roughness=0.4)
            look.func(f"{path}_look", look)
            sim_utils.bind_visual_material(path, f"{path}_look", stage=stage)
        return prim

    def add_parts(stage, prim_path, items):
        from pxr import Gf, UsdGeom, UsdPhysics

        grip = sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.2)
        grip.func(f"{prim_path}/part_grip", grip)
        for i, (kind, offset, size, color, collide, *rot) in enumerate(items):
            path = f"{prim_path}/part_{i}"
            if kind == "box":
                geom = UsdGeom.Cube.Define(stage, path)
                geom.CreateSizeAttr(1.0)
            else:
                geom = UsdGeom.Cylinder.Define(stage, path)
                geom.CreateRadiusAttr(float(size[0]))
                geom.CreateHeightAttr(float(size[1]))
                geom.CreateAxisAttr("Z")
            geom.AddTranslateOp().Set(Gf.Vec3d(*map(float, offset)))
            if rot:
                geom.AddOrientOp().Set(Gf.Quatf(*map(float, rot[0])))
            if kind == "box":
                geom.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
            if color[0] == "hidden":
                geom.MakeInvisible()
                color = color[1]
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(color), roughness=0.35)
            look.func(f"{path}_look", look)
            sim_utils.bind_visual_material(path, f"{path}_look", stage=stage)
            if collide:
                UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
                sim_utils.bind_physics_material(path, f"{prim_path}/part_grip", stage=stage)

    def add_joint(stage, prim_path, spec):
        from pxr import Gf, UsdPhysics

        kind, parent, offset, *rest = spec
        root = prim_path.rsplit("/", 1)[0]
        if kind == "lock":
            child = prim_path.rsplit("/", 1)[-1]
            start = (np.asarray(_POS[parent]) - np.asarray(_POS[child]) if offset is None
                     else -np.asarray(offset, dtype=float))
            joint = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/joint")
            joint.CreateBody0Rel().SetTargets([prim_path])
            joint.CreateBody1Rel().SetTargets([f"{root}/{parent}"])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, start)))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
            joint.CreateJointEnabledAttr(offset is None)
            return
        if kind == "slide":
            travel, damping = rest
            joint = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/joint")
            joint.CreateAxisAttr("Z")
            joint.CreateLowerLimitAttr(-float(travel))
            joint.CreateUpperLimitAttr(0.0)
            own = (0.0, 0.0, 0.0)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
            stiffness = 0.0
        else:
            axis, (low, high), stiffness, damping = rest
            joint = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/joint")
            joint.CreateAxisAttr(axis)
            joint.CreateLowerLimitAttr(float(low))
            joint.CreateUpperLimitAttr(float(high))
            start = np.asarray(_POS[prim_path.rsplit("/", 1)[-1]]) - np.asarray(_POS[parent])
            own = tuple(np.asarray(offset, dtype=float) - start)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTargetPositionAttr(0.0)
        joint.CreateBody0Rel().SetTargets([f"{root}/{parent}"])
        joint.CreateBody1Rel().SetTargets([prim_path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, offset)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*map(float, own)))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0))
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(stiffness))
        drive.CreateDampingAttr(float(damping))
        drive.CreateMaxForceAttr(1.0e4)

    def compound(base):
        inner = getattr(base, "__wrapped__", base)

        @clone
        def spawn(prim_path, cfg, *args, **kwargs):
            prim = inner(prim_path, cfg, *args, **kwargs)
            name = prim_path.rsplit("/", 1)[-1]
            stage = prim.GetStage()
            _PATHS[name] = (stage, prim_path)
            if name in _HIDDEN_BASE:
                from pxr import Usd, UsdGeom

                for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
                    if prim.IsA(UsdGeom.Gprim) and "/part_" not in str(prim.GetPath()):
                        UsdGeom.Imageable(prim).MakeInvisible()
            add_parts(stage, prim_path, _PARTS.get(name, ()))
            if name in _JOINTS:
                add_joint(stage, prim_path, _JOINTS[name])
            if name in _WEIGHTLESS:
                from pxr import PhysxSchema

                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
            print(f"trialkit: built {prim_path}", flush=True)
            return prim

        return spawn

    base_object = scene.task_object

    def task_object(spec):
        _POS[spec.name] = tuple(getattr(spec, "pos", (0.0, 0.0, 0.0)))
        obj = base_object(spec)
        if spec.name in _PANELS:
            obj.spawner_cfg.func = obj.object_cfg.spawn.func = spawn_labelled
        elif spec.name in _PARTS or spec.name in _JOINTS or spec.name in _HIDDEN_BASE:
            obj.spawner_cfg.func = obj.object_cfg.spawn.func = compound(obj.spawner_cfg.func)
        if spec.name in _WEIGHTLESS:
            obj.spawner_cfg.rigid_props.disable_gravity = True
            obj.object_cfg.spawn.rigid_props.disable_gravity = True
        return obj

    scene.task_object = task_object


def _patch_embodiment(mod):
    cls = mod.YamArenaEmbodiment
    base_reset = cls.reset

    def reset(self, scene, *, seed=None):
        obs = base_reset(self, scene, seed=seed)
        _S["env"] = self._env
        if not _RESETS:
            return obs
        rng = np.random.default_rng(seed)
        for fn in _RESETS.values():
            fn(self._env, rng)
        for _ in range(max(1, round(mod.SETTLE_S / self._base.step_dt))):
            self._env.step(self._ready)
        for _ in range(mod.RESET_RENDERS):
            self._base.sim.render()
        return self._observation()

    cls.reset = reset


def _patch_scripted(mod):
    cls = mod.YamScriptedPolicy
    base_act = cls.act

    def act(self, observation):
        t = float(observation.state_time) + self._dt
        if _S["gen"] is not None and t > self._end:
            try:
                plan = _S["gen"].send(View(observation, self._bases))
            except StopIteration:
                _S["gen"] = None
            else:
                for arm, waypoints in plan.items():
                    p = self._plans[arm]
                    if self._end - p.duration > 1e-6:
                        w = p.waypoints[-1]
                        p.waypoints.append(Waypoint(self._end - p.duration, w.pos, w.finger, "wait", w.yaw))
                    p.waypoints.extend(waypoints)
                self._end = max(p.duration for p in self._plans.values())
        return base_act(self, observation)

    cls.act = act


def _patch_demo(mod):
    from yam_arena.urdf import axis_angle_matrix

    base_rotation = mod.grip_rotation

    def grip_rotation(yaw=0.0):
        out = base_rotation(float(yaw))
        if isinstance(yaw, Tilt) and yaw.tilt:
            lean = np.array([math.sin(yaw.toward), -math.cos(yaw.toward), 0.0])
            out = axis_angle_matrix(lean, yaw.tilt) @ out
        return out

    mod.grip_rotation = grip_rotation


_PATCHES = {"yam_arena.demo": _patch_demo, "yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted}


class _AfterImport(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name not in _PATCHES:
            return None
        spec = importlib.machinery.PathFinder.find_spec(name, path)
        if spec is not None and spec.loader is not None:
            run = spec.loader.exec_module

            def exec_module(module):
                run(module)
                _PATCHES[name](module)

            spec.loader.exec_module = exec_module
        return spec


if not getattr(sys, "_trialkit", False):
    sys._trialkit = True
    for _name, _patch in _PATCHES.items():
        if _name in sys.modules:
            _patch(sys.modules[_name])
    sys.meta_path.insert(0, _AfterImport())
