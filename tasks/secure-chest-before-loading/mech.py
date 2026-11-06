"""Mechanisms for yam_arena task files: joints (hinges, sliders, axles, welds), lights, driven parts, gripper positions.

A task file stays pure Python (it is read before the simulator starts). It imports this module, which hooks
yam_arena's scene builder and step when the simulator starts:

    sys.path.insert(0, str(Path(__file__).parent)); import mech
    mech.revolute("door", axis="Z", anchor=(x, y, z), lower=-5, upper=110, damping=0.002)   # hinge to the world
    mech.prismatic("drawer", axis="X", anchor=(x, y, z), lower=0, upper=0.1, parent="carcass")  # slides in a body
    mech.weld("stripe", parent="wheel")              # two free objects move as one rigid body
    mech.on_tick(fn)                                 # fn(sim) after every control step, before scoring
    mech.no_gravity("tongue")                        # a part clamped in a slide stays where it is pushed
    mech.ghost("blade")                              # seen, but passes through everything

``sim`` gives ``t`` (s since the episode started), ``states`` (as score() sees them) and
``set_color(name, rgb)`` / ``set_velocity(name, lin=None, ang=None)``.

score() and ``sim.states`` also hold "left_tcp" / "right_tcp" (the point between the fingertips) and
"left_gripper" / "right_gripper" (the gripper link). ``mech.orient(name, quat_xyzw)`` turns an object at spawn
(e.g. a wheel on a sideways axle), which the task file's own yaw cannot do.

Joint anchors are world points (m) in the starting layout; revolute limits are degrees, prismatic limits metres.
Objects with joints must start unrotated (yaw 0). Jointed bodies do not collide with each other.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import sys
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Joint:
    kind: str  # "revolute" | "prismatic" | "fixed"
    axis: str = "X"  # "X" | "Y" | "Z" (world axis)
    anchor: tuple[float, float, float] | None = None  # None: the child's centre
    lower: float | None = None  # None: no limit
    upper: float | None = None
    stiffness: float = 0.0
    damping: float = 0.0
    target: float = 0.0
    parent: str | None = None  # None: the world


SOLVER_POSITION_ITERS = 32  # per jointed body (PhysX default 4)
SOLVER_VELOCITY_ITERS = 4
JOINTS: dict[str, Joint] = {}
COMPOUNDS: dict[str, list] = {}  # extra boxes making a free object one rigid body: name -> [(size, offset, colour)]
ORIENT: dict[str, tuple[float, float, float, float]] = {}  # spawn rotation (x, y, z, w) by object name
NO_GRAVITY: set[str] = set()  # free objects gravity doesn't act on (e.g. a part held in a friction slot)
GHOSTS: set[str] = set()  # objects that collide with nothing (e.g. a dummy saw blade that passes through the work)
_TICKS: list = []
_SPECS: dict = {}  # task objects by name, recorded as the scene is built


def revolute(name, axis, anchor, lower=None, upper=None, stiffness=0.0, damping=0.0, target=0.0, parent=None):
    JOINTS[name] = Joint("revolute", axis, tuple(anchor), lower, upper, stiffness, damping, target, parent)


def prismatic(name, axis, anchor, lower=None, upper=None, stiffness=0.0, damping=0.0, target=0.0, parent=None):
    JOINTS[name] = Joint("prismatic", axis, tuple(anchor), lower, upper, stiffness, damping, target, parent)


def weld(name, parent):
    JOINTS[name] = Joint("fixed", parent=parent)


def compound(name, parts):
    """Make a task object one rigid body with extra boxes: ``parts`` is [(size, offset, rgb), ...] or
    [(size, offset, rgb, (rx, ry, rz)), ...], each box's size, its centre's offset from the object's centre (m, world
    axes at spawn) and optionally its rotation (degrees about x, y, z). The object's mass is the whole body's, spread
    over all its boxes. Unlike welded objects, a compound body is exactly rigid."""
    COMPOUNDS[name] = [(tuple(p[0]), tuple(p[1]), tuple(p[2]), tuple(p[3]) if len(p) > 3 else None) for p in parts]


def no_gravity(name):
    """Gravity doesn't act on this object: a part clamped in a slide (held by friction) stays where it is put."""
    NO_GRAVITY.add(name)


def ghost(name):
    """The object collides with nothing: it is seen but passes through everything."""
    GHOSTS.add(name)


def orient(name, quat_xyzw):
    ORIENT[name] = tuple(float(q) for q in quat_xyzw)


def on_tick(fn):
    _TICKS.append(fn)
    return fn


# -- building ------------------------------------------------------------------------------


def _rot(name) -> np.ndarray:
    """The spawn rotation matrix of a task object (identity unless oriented)."""
    x, y, z, w = ORIENT.get(name, (0.0, 0.0, 0.0, 1.0))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _inv_quatf(name):
    from pxr import Gf

    x, y, z, w = ORIENT.get(name, (0.0, 0.0, 0.0, 1.0))
    return Gf.Quatf(w, -x, -y, -z)


def _add_parts(prim, spec) -> None:
    """The extra boxes of a compound body: collision cubes with their own look, children of the body's prim."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    rot_t = _rot(spec.name).T
    for i, (size, offset, rgb, rpy) in enumerate(COMPOUNDS[spec.name]):
        path = prim.GetPath().AppendChild(f"part{i}")
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*(rot_t @ np.asarray(offset, dtype=float)).tolist()))
        if rpy is not None:
            xf.AddRotateXYZOp().Set(Gf.Vec3f(*rpy))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        material = UsdShade.Material.Define(stage, path.AppendChild("look"))
        shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("shader"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(c ** 2.2 for c in rgb)))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)


def _author_joint(prim, spec, j: Joint) -> None:
    from pxr import Gf, UsdPhysics

    stage = prim.GetStage()
    path = prim.GetPath().AppendChild("mech_joint")
    kind = {"revolute": UsdPhysics.RevoluteJoint, "prismatic": UsdPhysics.PrismaticJoint, "fixed": UsdPhysics.FixedJoint}
    joint = kind[j.kind].Define(stage, path)
    centre = np.asarray(spec.pos, dtype=float)
    anchor = centre if j.anchor is None else np.asarray(j.anchor, dtype=float)
    # Joint frames are world-aligned at spawn, expressed in each body's own frame.
    if j.parent is None:
        frame0, rot0 = anchor, Gf.Quatf(1.0, 0.0, 0.0, 0.0)  # world frame
    else:
        joint.CreateBody0Rel().SetTargets([prim.GetPath().GetParentPath().AppendChild(j.parent)])
        frame0 = _rot(j.parent).T @ (anchor - np.asarray(_SPECS[j.parent].pos, dtype=float))
        rot0 = _inv_quatf(j.parent)
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    # Joints between free bodies are solved iteratively; the default 4 iterations let welds and slides drift under
    # gravity. More iterations keep welded parts rigid and jointed parts where they are put.
    from pxr import PhysxSchema

    body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    body.CreateSolverPositionIterationCountAttr(SOLVER_POSITION_ITERS)
    body.CreateSolverVelocityIterationCountAttr(SOLVER_VELOCITY_ITERS)
    joint.CreateLocalPos0Attr(Gf.Vec3f(*frame0.tolist()))
    joint.CreateLocalRot0Attr(rot0)
    joint.CreateLocalPos1Attr(Gf.Vec3f(*(_rot(spec.name).T @ (anchor - centre)).tolist()))
    joint.CreateLocalRot1Attr(_inv_quatf(spec.name))
    if j.kind == "fixed":
        return
    joint.CreateAxisAttr(j.axis)
    if j.lower is not None:
        joint.CreateLowerLimitAttr(float(j.lower))
        joint.CreateUpperLimitAttr(float(j.upper))
    if j.stiffness or j.damping:
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if j.kind == "revolute" else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(j.stiffness))
        drive.CreateDampingAttr(float(j.damping))
        drive.CreateTargetPositionAttr(float(j.target))


def _patch_scene(scene) -> None:
    from isaaclab.sim.utils import clone

    from yam_arena.task_file import ObjectState

    base_task_object = scene.task_object
    base_states = scene.object_states

    def task_object(spec):
        _SPECS[spec.name] = spec
        obj = base_task_object(spec)
        if spec.name in ORIENT:
            from isaaclab_arena.utils.pose import Pose

            obj = type(obj)(
                name=spec.name,
                object_type=obj.object_type,
                spawner_cfg=obj.spawner_cfg,
                initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=ORIENT[spec.name]),
            )
        j = JOINTS.get(spec.name)
        if j is not None or spec.name in COMPOUNDS or spec.name in NO_GRAVITY or spec.name in GHOSTS:
            cfg = obj.object_cfg.spawn  # Arena builds object_cfg when the Object is made
            inner = getattr(cfg.func, "__wrapped__", cfg.func)

            @clone
            def spawn(prim_path, cfg_, *args, **kwargs):
                prim = inner(prim_path, cfg_, *args, **kwargs)
                if spec.name in COMPOUNDS:
                    _add_parts(prim, spec)
                if j is not None:
                    _author_joint(prim, spec, j)
                if spec.name in NO_GRAVITY:
                    from pxr import PhysxSchema

                    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
                if spec.name in GHOSTS:
                    from pxr import Usd, UsdPhysics

                    for part in Usd.PrimRange(prim):
                        if part.HasAPI(UsdPhysics.CollisionAPI):
                            UsdPhysics.CollisionAPI(part).CreateCollisionEnabledAttr(False)
                return prim

            cfg.func = spawn
        return obj

    def object_states(env, names):
        import torch
        from isaaclab.utils.math import quat_apply

        from yam_arena.constants import TCP_OFFSET

        out = base_states(env, names)
        base = env.unwrapped
        origin = base.scene.env_origins[0]
        for arm in ("left", "right"):
            robot = base.scene[f"{arm}_robot"]
            i = robot.find_bodies("gripper")[0][0]
            pose = robot.data.body_link_pose_w.torch[0:1, i]
            offset = torch.tensor([TCP_OFFSET], dtype=pose.dtype, device=pose.device)
            tcp = pose[:, :3] + quat_apply(pose[:, 3:7], offset) - origin
            quat = pose[0, 3:7].detach().cpu().numpy().astype(np.float64)
            z = np.zeros(3)
            for key, p in ((f"{arm}_gripper", pose[0, :3] - origin), (f"{arm}_tcp", tcp[0])):
                p = p.detach().cpu().numpy().astype(np.float64)
                out[key] = ObjectState(p, quat, z, z, p - 0.01, p + 0.01)
        return out

    scene.task_object = task_object
    scene.object_states = object_states


# -- stepping ------------------------------------------------------------------------------

_COLORS: dict[str, tuple] = {}


class Sim:
    def __init__(self, emb, states):
        self._emb = emb
        self.t = emb._t
        self.states = states

    def set_color(self, name: str, rgb) -> None:
        rgb = tuple(float(c) for c in rgb)
        if _COLORS.get(name) == rgb:
            return
        _COLORS[name] = rgb
        import omni.usd
        from pxr import Gf, Sdf, Usd, UsdShade

        stage = omni.usd.get_context().get_stage()
        for part in Usd.PrimRange(stage.GetPrimAtPath(f"/World/envs/env_0/{name}")):
            if part.IsA(UsdShade.Shader):
                shader = UsdShade.Shader(part)
                diffuse = shader.GetInput("diffuseColor")
                if diffuse:
                    diffuse.Set(Gf.Vec3f(*rgb))
                emissive = shader.GetInput("emissiveColor") or shader.CreateInput(
                    "emissiveColor", Sdf.ValueTypeNames.Color3f
                )
                emissive.Set(Gf.Vec3f(*(0.6 * c for c in rgb)))

    def set_velocity(self, name: str, lin=None, ang=None) -> None:
        torch = self._emb._torch
        obj = self._emb._base.scene[name]
        st = self.states[name]
        vel = torch.tensor([[*st.lin_vel, *st.ang_vel]], dtype=torch.float32, device=self._emb._base.device)
        if lin is not None:
            vel[0, :3] = torch.tensor(lin, dtype=vel.dtype, device=vel.device)
        if ang is not None:
            vel[0, 3:] = torch.tensor(ang, dtype=vel.dtype, device=vel.device)
        obj.write_root_velocity_to_sim_index(root_velocity=vel)


def _tick(emb) -> None:
    if not _TICKS:
        return
    from yam_arena.scene import object_states

    if emb._t <= emb._base.step_dt * 1.5:
        _COLORS.clear()  # a new episode: re-apply colours
    sim = Sim(emb, object_states(emb._env, emb.task.names))
    for fn in _TICKS:
        fn(sim)


def _patch_embodiment(mod) -> None:
    cls = mod.YamArenaEmbodiment
    if getattr(cls, "_mech_patched", False):
        return
    base_step = cls._task_step

    def _task_step(self):
        _tick(self)
        return base_step(self)

    cls._task_step = _task_step
    cls._mech_patched = True


class _Hook(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name not in ("yam_arena.scene", "yam_arena.inspect_embodiment"):
            return None
        spec = importlib.machinery.PathFinder.find_spec(name, path)
        if spec is None or spec.loader is None:
            return None
        base_exec = spec.loader.exec_module

        def exec_module(module):
            base_exec(module)
            (_patch_scene if name == "yam_arena.scene" else _patch_embodiment)(module)

        spec.loader.exec_module = exec_module
        return spec


if not any(isinstance(f, _Hook) for f in sys.meta_path):
    sys.meta_path.insert(0, _Hook())
for _name, _fn in (("yam_arena.scene", _patch_scene), ("yam_arena.inspect_embodiment", _patch_embodiment)):
    _mod = sys.modules.get(_name)
    if _mod is not None and not getattr(_mod, "_mech_done", False):
        _fn(_mod)
        _mod._mech_done = True
