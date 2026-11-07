"""yamkit: modeled props and gripper positions for a yam_arena task file.

A yam_arena task file declares boxes, cylinders and spheres. This module adds `Prop`, a `Box` that yam_arena's checks
accept, which spawns instead from a USD file written at start-up: visible meshes (boxes, cylinders and lathed shapes,
each with its own color) and separate invisible colliders (boxes and cylinders). The visible meshes are what the
cameras see. The colliders are what the physics and the grippers touch; keep the visible shape close to them.

Importing this module also gives score() and success() the two grippers. Next to the task's objects, `objects` then
holds "left_gripper" and "right_gripper": an ObjectState whose `pos` is the tool point the demos steer (between the
finger pads), whose `quat_xyzw` is the gripper's orientation, and whose `low` and `high` span from the tool point to the
pad tips. `opening(objects, side)` is how far the fingers are open, from 0 (closed) to 1 (fully open).

Nothing on disk changes. The module hooks yam_arena.scene when the simulator imports it (yam_arena reads the task file
before Isaac Sim starts, so the hook waits for that import), replacing two functions: task_object, which builds each
object, and object_states, which reads them after every control step. Plain Python and NumPy at import time.
"""

from __future__ import annotations

import hashlib
import importlib.abc
import importlib.util
import math
import os
import sys
import tempfile
from dataclasses import dataclass, field

import numpy as np
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET
from yam_arena.task_file import Box, ObjectState

GRIPPERS = {"left_gripper": "left_robot", "right_gripper": "right_robot"}
TIP_LINKS = ("tip_left", "tip_right")
_OPENING: dict[str, float] = {}
_PROPS: set[str] = set()


# -- shapes ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class Look:
    """A plain surface: sRGB color (0..1), roughness, metallic and opacity."""

    color: tuple[float, float, float]
    roughness: float = 0.5
    metallic: float = 0.0
    opacity: float = 1.0


@dataclass(frozen=True)
class Mesh:
    """Triangles and quads in the prop's own frame (metres), with one look."""

    points: tuple
    counts: tuple
    indices: tuple
    look: Look


def _rot(axis: str | np.ndarray) -> np.ndarray:
    """A rotation taking +z to `axis` ("x", "y", "z" or a vector)."""
    a = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}.get(axis, axis) if isinstance(axis, str) else axis
    a = np.asarray(a, float) / np.linalg.norm(a)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(z, a), float(np.dot(z, a))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * (1 / (1 + c))


def _mesh(points, counts, indices, look: Look) -> Mesh:
    return Mesh(tuple(map(tuple, np.round(np.asarray(points, float), 6))), tuple(counts), tuple(indices), look)


def box(center, size, look: Look) -> Mesh:
    """An axis-aligned box."""
    c, h = np.asarray(center, float), np.asarray(size, float) / 2
    p = [c + h * np.array([sx, sy, sz]) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return _mesh(p, [4] * 6, [i for f in faces for i in f], look)


def lathe(profile, look: Look, center=(0, 0, 0), axis="z", n: int = 40) -> Mesh:
    """A surface of revolution: `profile` is (radius, height) pairs from bottom to top, turned about `axis` through
    `center`. A radius of 0 closes the end."""
    r = _rot(axis)
    pts, counts, idx = [], [], []
    for rad, z in profile:
        for k in range(n):
            t = 2 * math.pi * k / n
            pts.append(np.asarray(center) + r @ np.array([rad * math.cos(t), rad * math.sin(t), z]))
    for i in range(len(profile) - 1):
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            counts.append(4)
            idx += [a, b, b + n, a + n]
    for end, flip in ((0, True), (len(profile) - 1, False)):  # flat caps where the profile ends off the axis
        if profile[end][0] > 0:
            ring = [end * n + k for k in range(n)]
            counts.append(n)
            idx += ring[::-1] if flip else ring
    return _mesh(pts, counts, idx, look)


def cylinder(center, radius: float, length: float, look: Look, axis="z", n: int = 32) -> Mesh:
    """A closed cylinder of `length` along `axis`, centered on `center`."""
    return lathe([(radius, -length / 2), (radius, length / 2)], look, center, axis, n)


def band(path, width: float, thickness: float, look: Look, normal=(1, 0, 0)) -> Mesh:
    """A flat strap following `path` (points), `width` wide along `normal` x tangent and `thickness` thick."""
    path = np.asarray(path, float)
    nrm = np.asarray(normal, float) / np.linalg.norm(normal)
    pts, counts, idx = [], [], []
    for i, p in enumerate(path):
        tan = path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)]
        tan /= np.linalg.norm(tan)
        out = np.cross(tan, nrm)
        out /= np.linalg.norm(out)
        for s in (-1, 1):
            for t in (-1, 1):
                pts.append(p + nrm * s * width / 2 + out * t * thickness / 2)
    for i in range(len(path) - 1):
        a, b = 4 * i, 4 * (i + 1)
        for e0, e1 in ((0, 1), (1, 3), (3, 2), (2, 0)):
            counts.append(4)
            idx += [a + e0, a + e1, b + e1, b + e0]
    for end, flip in ((0, True), (len(path) - 1, False)):
        ring = [4 * end + k for k in (0, 1, 3, 2)]
        counts.append(4)
        idx += ring[::-1] if flip else ring
    return _mesh(pts, counts, idx, look)


@dataclass(frozen=True)
class Collider:
    """An invisible box (`size`) or cylinder (`radius`, `height`, along z) collider in the prop's frame."""

    center: tuple
    size: tuple | None = None
    radius: float = 0.0
    height: float = 0.0


# -- props -----------------------------------------------------------------------------------


@dataclass(frozen=True)
class Prop(Box):
    """A modeled object. `pos`, `yaw`, `mass` and `name` work as for `Box`; `size` should bound the colliders (yam_arena
    checks it, and scorers can use it). `meshes` are what the cameras see and `colliders` what the physics touches,
    both in the prop's own frame with its origin at `pos`. `tilt_xyzw` turns the whole prop before `yaw` (for a part
    that starts leaning).

    Scorers get a prop's `pos` as its origin (where `pos` put it), and `low` and `high` as the bounds of its visible
    meshes."""

    meshes: tuple = field(default=())
    colliders: tuple = field(default=())
    tilt_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _fmt(v) -> str:
    return "(" + ", ".join(f"{float(x):.6g}" for x in v) + ")"


def _usda(prop: Prop) -> str:
    looks = list(dict.fromkeys(m.look for m in prop.meshes))
    out = ['#usda 1.0\n(\n    defaultPrim = "Root"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n']
    out.append('def Xform "Root" (\n    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]\n)\n{\n')
    if prop.mass is not None:
        out.append(f"    float physics:mass = {prop.mass}\n")
    out.append('    def Scope "Looks"\n    {\n')
    for i, lk in enumerate(looks):
        col = _fmt(_linear(c) for c in lk.color)
        out.append(f'        def Material "M{i}"\n        {{\n'
                   f'            token outputs:surface.connect = </Root/Looks/M{i}/S.outputs:surface>\n'
                   f'            def Shader "S"\n            {{\n'
                   f'                uniform token info:id = "UsdPreviewSurface"\n'
                   f'                color3f inputs:diffuseColor = {col}\n'
                   f'                float inputs:roughness = {lk.roughness}\n'
                   f'                float inputs:metallic = {lk.metallic}\n'
                   f'                float inputs:opacity = {lk.opacity}\n'
                   f'                token outputs:surface\n            }}\n        }}\n')
    out.append("    }\n")
    for i, m in enumerate(prop.meshes):
        out.append(f'    def Mesh "V{i}" (\n        prepend apiSchemas = ["MaterialBindingAPI"]\n    )\n    {{\n'
                   f'        point3f[] points = [{", ".join(_fmt(p) for p in m.points)}]\n'
                   f'        int[] faceVertexCounts = [{", ".join(map(str, m.counts))}]\n'
                   f'        int[] faceVertexIndices = [{", ".join(map(str, m.indices))}]\n'
                   f'        uniform token subdivisionScheme = "none"\n'
                   f'        uniform bool doubleSided = 1\n'
                   f'        rel material:binding = </Root/Looks/M{looks.index(m.look)}>\n    }}\n')
    for i, c in enumerate(prop.colliders):
        kind = "Cube" if c.size is not None else "Cylinder"
        out.append(f'    def {kind} "C{i}" (\n        prepend apiSchemas = ["PhysicsCollisionAPI"]\n    )\n    {{\n'
                   f'        uniform token purpose = "guide"\n')
        if c.size is not None:
            out.append("        double size = 1\n"
                       f"        double3 xformOp:translate = {_fmt(c.center)}\n"
                       f"        float3 xformOp:scale = {_fmt(c.size)}\n"
                       '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n')
        else:
            out.append(f'        uniform token axis = "Z"\n        double radius = {c.radius}\n'
                       f"        double height = {c.height}\n"
                       f"        double3 xformOp:translate = {_fmt(c.center)}\n"
                       '        uniform token[] xformOpOrder = ["xformOp:translate"]\n')
        out.append("    }\n")
    out.append("}\n")
    return "".join(out)


def _write(prop: Prop) -> str:
    text = _usda(prop)
    folder = os.path.join(tempfile.gettempdir(), "yamkit_usd")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{prop.name}_{hashlib.sha1(text.encode()).hexdigest()[:12]}.usda")
    if not os.path.exists(path):
        with open(path + ".tmp", "w") as f:
            f.write(text)
        os.replace(path + ".tmp", path)
    return path


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


# -- hooks into yam_arena.scene --------------------------------------------------------------


def _patch(scene) -> None:
    if getattr(scene, "_yamkit", False):
        return
    scene._yamkit = True
    original_task_object, original_states = scene.task_object, scene.object_states

    def task_object(spec):
        if not isinstance(spec, Prop):
            return original_task_object(spec)
        _PROPS.add(spec.name)
        yaw = (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))
        addon = {"physics_material": scene._grippy()}
        if spec.mass is None:
            addon.update(func=scene._spawn_fixture,
                         rigid_props=scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True))
        else:
            addon.update(collision_props=scene._contact_props())
        return scene.Object(
            name=spec.name,
            usd_path=_write(spec),
            object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_quat_mul(yaw, spec.tilt_xyzw)),
            spawn_cfg_addon=addon,
        )

    def object_states(env, names):
        out = original_states(env, [n for n in names if n not in GRIPPERS])
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        for name in _PROPS & set(out):  # a prop's pos is its own origin, not the middle of what it looks like
            p = base.arena_world.get_pose_w(name)[0, :3].detach().cpu().numpy().astype(np.float64) - origin
            s = out[name]
            out[name] = ObjectState(pos=p, quat_xyzw=s.quat_xyzw, lin_vel=s.lin_vel, ang_vel=s.ang_vel, low=s.low,
                                    high=s.high)
        for key, robot in GRIPPERS.items():
            try:
                art = base.scene[robot]
            except KeyError:
                continue
            i = art.body_names.index("gripper")
            pos = art.data.body_pos_w.torch[0, i].detach().cpu().numpy().astype(np.float64) - origin
            x, y, z, w = art.data.body_quat_w.torch[0, i].detach().cpu().numpy().astype(np.float64)  # (x, y, z, w)
            rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
            tcp = pos + rot @ np.asarray(TCP_OFFSET)  # the point the demos steer: between the finger pads
            tips = tcp + rot @ np.array([0.0, 0.0, PAD_TIP_PAST_TCP])
            j = art.joint_names.index("joint7")
            _OPENING[key] = float(abs(art.data.joint_pos.torch[0, j].item()) / abs(FINGER_OPEN))
            out[key] = ObjectState(pos=tcp, quat_xyzw=np.array([x, y, z, w]), lin_vel=np.zeros(3),
                                   ang_vel=np.zeros(3), low=np.minimum(tcp, tips), high=np.maximum(tcp, tips))
        return out

    scene.task_object, scene.object_states = task_object, object_states


class _Hook(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Patches yam_arena.scene as soon as it is imported."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None:
            return None
        self._inner = spec.loader
        spec.loader = self
        return spec

    def create_module(self, spec):
        return self._inner.create_module(spec)

    def exec_module(self, module):
        self._inner.exec_module(module)
        _patch(module)


def opening(objects, side: str) -> float:
    """How far a gripper's fingers are open, 0 (closed) to 1 (fully open), as of the last step; side is
    "left_gripper" or "right_gripper"."""
    return _OPENING.get(side, float("nan"))


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _Hook) for f in sys.meta_path):
    sys.meta_path.insert(0, _Hook())
