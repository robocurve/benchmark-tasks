"""hhbungee: a bungee cord for yam_arena task files (campaign 50, own implementation).

Two loose hooks (``hhperson.CompoundBox`` bodies) are joined by a massless elastic line between an attachment point on
each. Before every physics step (240 Hz) each hook gets the line's pull, ``k * stretch`` plus a little damping, along
the line at its attachment point (force and the matching torque, world frame); a slack line pulls nothing. The line
is drawn by ``GhostBox`` segments: kinematic boxes with no collider, moved onto the line every control step, so the
cord is seen but never touched. Hook poses come straight from the simulator's body data (body origin, not the
bounding-box centre the task file reports), and are kept in ``LAST`` for the task's scorer.

Offline (no simulator) nothing here runs; tests fill ``LAST`` themselves. Isaac modules are imported lazily.
"""
from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box


@dataclass(frozen=True)
class GhostBox(Box):
    """A box that is only drawn: kinematic, no collider, moved by the task. ``mass`` stays None (not a loose object)."""


@dataclass(frozen=True)
class HookBox(Box):
    """A loose body whose collider is the box (``size``, hidden) plus ``parts``: (name, size, offset from the box
    centre in its own frame, sRGB colour, collides) boxes riding on it. Colliding parts join the body's compound
    collider (a hook's shank and its bent tip), the rest are drawn only."""

    parts: tuple = ()


@dataclass
class Line:
    """The elastic line: hook names, attachment points in each hook's body frame, rest length (m), stiffness (N/m),
    damping (N s/m) along the line, and the names of the segments that draw it."""
    hooks: tuple
    attach: tuple
    rest: float
    k: float
    c: float = 0.3
    segments: tuple = ()
    seg_len: float = 0.05


LINES: list[Line] = []
LAST: dict = {}   # name -> (origin xyz, quat xyzw, linear velocity) of each hook, refreshed every physics step
TENSION: dict = {"N": 0.0, "span": 0.0, "peak_N": 0.0}
PEAK: dict = {}   # name -> fastest speed (m/s) seen at any physics step since the task last took it (take_peak)
NEAR: dict = {"points": None, "gap": {}}  # optional (n, 3) points (a hand's skin): each hook's closest approach


def take_peak(name: str):
    """The fastest a hook moved at any physics step since the last call (None offline), and reset it."""
    return PEAK.pop(name, None)


def rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def attach_points(line: Line, poses=None) -> tuple[np.ndarray, np.ndarray]:
    poses = poses or LAST
    out = []
    for name, local in zip(line.hooks, line.attach, strict=True):
        p, q, _v = poses[name]
        out.append(np.asarray(p, float) + rot(q) @ np.asarray(local, float))
    return out[0], out[1]


def pull(line: Line, a, b, va=np.zeros(3), vb=np.zeros(3)) -> tuple[float, np.ndarray]:
    """The line's tension (N) and the unit vector from ``a`` to ``b``."""
    d = np.asarray(b, float) - np.asarray(a, float)
    span = float(np.linalg.norm(d))
    u = d / span if span > 1e-9 else np.zeros(3)
    s = span - line.rest
    if s <= 0.0:
        return 0.0, u
    rate = float((np.asarray(vb, float) - np.asarray(va, float)) @ u)
    return max(0.0, line.k * s + line.c * rate), u


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _read(env, name):
    base = env.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins)[0]
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_lin_vel_w)[0]
    com = _np(asset.data.root_com_pos_w)[0] - origin
    return pose[:3] - origin, pose[3:7], vel, com


def _substep(env) -> None:
    import torch

    for line in LINES:
        st = {n: _read(env, n) for n in line.hooks}
        for n, (p, q, v, _c) in st.items():
            LAST[n] = (p, q, v)
            PEAK[n] = max(PEAK.get(n, 0.0), float(np.linalg.norm(v)))
            if NEAR["points"] is not None:
                g = float(np.linalg.norm(NEAR["points"] - p, axis=1).min())
                NEAR["gap"][n] = min(NEAR["gap"].get(n, 9.0), g)
        a, b = attach_points(line)
        va, vb = st[line.hooks[0]][2], st[line.hooks[1]][2]
        f, u = pull(line, a, b, va, vb)
        TENSION["N"], TENSION["span"] = f, float(np.linalg.norm(b - a))
        TENSION["peak_N"] = max(TENSION["peak_N"], f)
        for name, point, sign in ((line.hooks[0], a, 1.0), (line.hooks[1], b, -1.0)):
            force = sign * f * u
            torque = np.cross(point - st[name][3], force)
            asset = env.unwrapped.scene[name]
            dev = asset.device
            asset.permanent_wrench_composer.set_forces_and_torques_index(
                forces=torch.tensor([[force.tolist()]], dtype=torch.float32, device=dev),
                torques=torch.tensor([[torque.tolist()]], dtype=torch.float32, device=dev), is_global=True)


def _quat_x_to(u) -> tuple:
    """xyzw rotation taking the local x axis onto unit vector ``u``."""
    x = np.array([1.0, 0.0, 0.0])
    c = float(np.clip(x @ u, -1.0, 1.0))
    ax = np.cross(x, u)
    s = float(np.linalg.norm(ax))
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (0.0, 0.0, 1.0, 0.0)
    ax /= s
    h = math.atan2(s, c) / 2
    return (*(ax * math.sin(h)).tolist(), math.cos(h))


def draw(env) -> None:
    """Put each line's segments on the line between the attachment points (every control step)."""
    import torch

    if not LAST:
        return
    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0]
    for line in LINES:
        if not all(n in LAST for n in line.hooks):
            continue
        a, b = attach_points(line)
        d = b - a
        span = float(np.linalg.norm(d))
        u = d / span if span > 1e-9 else np.array([0.0, 1.0, 0.0])
        q = _quat_x_to(u)
        n = len(line.segments)
        for i, name in enumerate(line.segments):
            p = a + d * (i + 0.5) / n
            t = torch.tensor([[*p.tolist(), *q]], dtype=torch.float32, device=origin.device)
            t[:, :3] += origin
            base.scene[name].write_root_pose_to_sim_index(root_pose=t)


_WRAPPED = set()


def on_step(trial) -> None:
    """hhkit.Trial on_step hook: install the physics-step pull once, then redraw the cord."""
    env = trial.env
    sc = env.unwrapped.scene
    if id(sc) not in _WRAPPED:
        orig = sc.write_data_to_sim

        def write_data_to_sim(*a, **k):
            _substep(env)
            return orig(*a, **k)

        sc.write_data_to_sim = write_data_to_sim
        _WRAPPED.add(id(sc))
        _substep(env)
    draw(env)


def _ghost_object(spec: GhostBox, scene):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size), rigid_props=PhysxRigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.7))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _hook_object(spec: HookBox, scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    parts = tuple(spec.parts)

    @clone
    def _spawn(prim_path, cfg, *args, **kwargs):
        from pxr import Gf, UsdGeom, UsdPhysics

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        mesh = stage.GetPrimAtPath(f"{prim_path}/geometry/mesh")
        if mesh.IsValid():
            UsdGeom.Imageable(mesh).MakeInvisible()
        for name, size, off, color, collides in parts:
            path = f"{prim_path}/parts/{name}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in off]))
            cube.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
            if collides:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(color), roughness=0.35, metallic=0.6)
            look.func(f"{path}_material", look)
            sim_utils.bind_visual_material(path, f"{path}_material", stage=stage)
        return prim

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size), func=_spawn,
        rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(), physics_material=scene._grippy(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhbungee", False):
        return
    task_object = scene.task_object

    def task_object_hook(spec):
        if isinstance(spec, GhostBox):
            return _ghost_object(spec, scene)
        if isinstance(spec, HookBox):
            return _hook_object(spec, scene)
        return task_object(spec)

    scene.task_object, scene._hhbungee = task_object_hook, True


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
        exec_module = spec.loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file and field
