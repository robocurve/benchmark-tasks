"""yamkit: modeled props and gripper positions for a yam_arena task file.

A yam_arena task file declares boxes, cylinders and spheres. This module adds `Prop`, a `Box` that yam_arena's checks
accept, which spawns instead from a USD file written at start-up: visible meshes (boxes, cylinders and lathed shapes,
each with its own color) and separate invisible colliders (boxes and cylinders). The visible meshes are what the
cameras see. The colliders are what the physics and the grippers touch; keep the visible shape close to them.

Importing this module also gives score() and success() the two grippers. Next to the task's objects, `objects` then
holds "left_gripper" and "right_gripper": an ObjectState whose `pos` is the tool point the demos steer (between the
finger pads), whose `quat_xyzw` is the gripper's orientation, and whose `low` and `high` span from the tool point to the
pad tips. `opening(objects, side)` is how far the fingers are open, from 0 (closed) to 1 (fully open).

Mechanisms (Kit A): `Mechanism` is a `Prop` whose meshes and colliders are a base (fixed to the world unless it has a
mass), with `links` that move on hinges or slides (`Link`: kind "revolute" or "prismatic", axis, anchor, limits, start,
spring and rest, damping, friction, `floating` for a counterweighted part, optional `Latch`). Scorers then get
`objects["name"]` for the base, `objects["name/link"]` for each link, and `joint(objects, "name/link")` for the joint's
position, velocity and whether its latch has caught. `Tilted` is a demo waypoint whose grip can also roll and pitch.

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
_EPISODE = {"count": 0, "last": None}  # trials seen, and the env's step count within the current one
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
    """An invisible box (`size`) or cylinder (`radius`, `height`, along z) collider in the prop's frame; a box can be
    turned `rot_z` degrees about the prop's z axis (for a sloped edge;
    props only, not mechanism links)."""

    center: tuple
    size: tuple | None = None
    radius: float = 0.0
    height: float = 0.0
    rot_z: float = 0.0


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
    com: tuple | None = None  # centre of mass in the prop's frame (default: PhysX's, from the colliders)


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
    if getattr(prop, "com", None) is not None:
        out.append(f"    point3f physics:centerOfMass = {_fmt(prop.com)}\n")
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
                       f"        float3 xformOp:scale = {_fmt(c.size)}\n")
            if c.rot_z:
                out.append(f"        float xformOp:rotateZ = {c.rot_z:.6g}\n"
                           '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]\n')
            else:
                out.append('        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n')
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


# -- mechanisms: parts on hinges and slides (Kit A) ------------------------------------------


@dataclass(frozen=True)
class Latch:
    """A catch on a joint. Once the joint comes within `zone` of `at`, it is pulled to `at` with `hold` stiffness and
    stays there (the latch has caught); nothing in the scene releases it. `at` is in the joint's units (rad for a
    hinge, m for a slide)."""

    at: float
    zone: float
    hold: float = 200.0


@dataclass(frozen=True)
class Link:
    """A moving part of a `Mechanism`, joined to its base by a hinge (`kind="revolute"`) or a slide
    (`kind="prismatic"`) along the base's `axis` ("X", "Y" or "Z"). `anchor` is the joint's origin in the base's frame;
    `meshes` and `colliders` are in the link's own frame, which sits at the anchor and matches the base's frame at
    joint position 0. `lower`/`upper`/`start` are the joint's limits and starting position (rad or m).

    `spring` (N·m/rad or N/m) pulls the joint toward `rest` (default `start`); `damping` resists speed; `friction` is
    the joint's static friction. `floating=True` turns gravity off for the link, as a counterweight would (a fume-hood
    sash stays where it is left). `latch` adds a catch (see `Latch`). `stay` (N/m or N·m/rad) holds the part where it was at
the last control step, so it stays where it is left but gives way to a push, like a friction hinge or a balanced sash
(PhysX joint damping alone barely slows these light parts). `drag` (N·s/m or N·m·s/rad) is a force against the joint's
speed applied each control step, so a part moves only while it is pushed; it is capped at 0.8 x mass / control step
for a slide, to stay stable."""

    name: str
    kind: str
    axis: str
    anchor: tuple
    lower: float
    upper: float
    start: float = 0.0
    mass: float = 0.05
    meshes: tuple = ()
    colliders: tuple = ()
    spring: float = 0.0
    rest: float | None = None
    damping: float = 0.5
    friction: float = 0.0
    floating: bool = False
    latch: Latch | None = None
    stay: float = 0.0
    drag: float = 0.0
    law: object = None  # a mechkit law (OverCenter, Ratchet, Detents, Follow, Pump, Interlock): sets the drive each step


@dataclass(frozen=True)
class Mechanism(Prop):
    """A `Prop` (its meshes and colliders are the base) with moving `links`. Without `mass` the base is fixed to the
    world. Scorers get the base as `objects[name]` (pos = base origin), each link as `objects["name/link"]` (pos =
    the link's origin, at its anchor), and the joint through `joint(objects, "name/link")`."""

    links: tuple = field(default=())


@dataclass(frozen=True)
class JointState:
    """A link's joint: `pos` (rad or m), `vel`, and `latched` once its latch has caught."""

    pos: float
    vel: float
    latched: bool


_MECHS: dict[str, Mechanism] = {}
_JOINTS: dict[str, JointState] = {}
_LATCHED: dict[str, int] = {}  # "name/link" -> the episode in which its latch caught


_LAW_MEM: dict[str, tuple] = {}  # "name/link" -> (episode, memory dict) for the link's law


def _law_mem(key: str, lk) -> dict:
    """The law's memory for this trial; a fresh one (seeded by the law's `fresh`) after every scene reset."""
    ep, mem = _LAW_MEM.get(key, (None, None))
    if ep != _EPISODE["count"]:
        mem = {"start": lk.start}
        if hasattr(lk.law, "fresh"):
            mem.update(lk.law.fresh(lk))
        _LAW_MEM[key] = (_EPISODE["count"], mem)
    return mem


def law_state(key: str) -> dict:
    """What link `key`'s law has recorded this trial (e.g. a ratchet's "clicks", a pump's "level")."""
    return _LAW_MEM.get(key, (None, {}))[1]


def joint(objects, key: str) -> JointState:
    """The joint state of link `key` ("mechanism/link") at this control step."""
    return _JOINTS[key]


def _body_usda(name: str, translate, mass, meshes, colliders, looks, floating=False, indent="    ") -> list[str]:
    i2 = indent + "    "
    out = [f'{indent}def Xform "{name}" (\n{i2}prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
           f'"PhysxRigidBodyAPI"]\n{indent})\n{indent}{{\n',
           f"{i2}float physics:mass = {mass}\n",
           f"{i2}bool physxRigidBody:disableGravity = {1 if floating else 0}\n",
           f"{i2}double3 xformOp:translate = {_fmt(translate)}\n",
           f'{i2}uniform token[] xformOpOrder = ["xformOp:translate"]\n']
    for i, m in enumerate(meshes):
        out.append(f'{i2}def Mesh "V{i}" (\n{i2}    prepend apiSchemas = ["MaterialBindingAPI"]\n{i2})\n{i2}{{\n'
                   f'{i2}    point3f[] points = [{", ".join(_fmt(p) for p in m.points)}]\n'
                   f'{i2}    int[] faceVertexCounts = [{", ".join(map(str, m.counts))}]\n'
                   f'{i2}    int[] faceVertexIndices = [{", ".join(map(str, m.indices))}]\n'
                   f'{i2}    uniform token subdivisionScheme = "none"\n'
                   f'{i2}    uniform bool doubleSided = 1\n'
                   f'{i2}    rel material:binding = </Root/Looks/M{looks.index(m.look)}>\n{i2}}}\n')
    for i, c in enumerate(colliders):
        kind = "Cube" if c.size is not None else "Cylinder"
        out.append(f'{i2}def {kind} "C{i}" (\n{i2}    prepend apiSchemas = ["PhysicsCollisionAPI"]\n{i2})\n{i2}{{\n'
                   f'{i2}    uniform token purpose = "guide"\n')
        if c.size is not None:
            out.append(f"{i2}    double size = 1\n{i2}    double3 xformOp:translate = {_fmt(c.center)}\n"
                       f"{i2}    float3 xformOp:scale = {_fmt(c.size)}\n"
                       f'{i2}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n')
        else:
            out.append(f'{i2}    uniform token axis = "Z"\n{i2}    double radius = {c.radius}\n'
                       f"{i2}    double height = {c.height}\n{i2}    double3 xformOp:translate = {_fmt(c.center)}\n"
                       f'{i2}    uniform token[] xformOpOrder = ["xformOp:translate"]\n')
        out.append(f"{i2}}}\n")
    out.append(f"{indent}}}\n")
    return out


def _mech_usda(mech: Mechanism) -> str:
    """The base and links as one articulation: the base (fixed to the world by a root joint unless it has a mass),
    each link a rigid body on a revolute or prismatic joint with its limits."""
    meshes = list(mech.meshes) + [m for lk in mech.links for m in lk.meshes]
    looks = list(dict.fromkeys(m.look for m in meshes))
    out = ['#usda 1.0\n(\n    defaultPrim = "Root"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n',
           'def Xform "Root" (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)\n{\n',
           "    bool physxArticulation:enabledSelfCollisions = 0\n",
           "    int physxArticulation:solverPositionIterationCount = 32\n",
           "    int physxArticulation:solverVelocityIterationCount = 4\n",
           # never sleep: a sleeping articulation ignores effort targets, so laws and springs stop acting on it
           "    float physxArticulation:sleepThreshold = 0\n",
           "    float physxArticulation:stabilizationThreshold = 0\n",
           '    def Scope "Looks"\n    {\n']
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
    out += _body_usda("base", (0, 0, 0), mech.mass if mech.mass is not None else 1.0, mech.meshes, mech.colliders, looks)
    if mech.mass is None:
        out.append('    def PhysicsFixedJoint "root_joint"\n    {\n        rel physics:body1 = </Root/base>\n    }\n')
    for lk in mech.links:
        out += _body_usda(lk.name, lk.anchor, lk.mass, lk.meshes, lk.colliders, looks, floating=lk.floating)
        kind = {"revolute": "PhysicsRevoluteJoint", "prismatic": "PhysicsPrismaticJoint"}[lk.kind]
        scale = math.degrees(1.0) if lk.kind == "revolute" else 1.0  # USD hinge limits are in degrees
        lin = "angular" if lk.kind == "revolute" else "linear"
        # a link with a mechkit law gets a PhysX drive: without one PhysX ignores stiffness, damping and targets, and
        # the laws' gains must be implicit (PhysX's own drive) to stay stable on light links at a 20 Hz control rate
        drive_api = f', "PhysicsDriveAPI:{lin}"' if lk.law is not None else ""
        drive = (f'        uniform token drive:{lin}:physics:type = "force"\n'
                 f"        float drive:{lin}:physics:maxForce = 1000\n"
                 f"        float drive:{lin}:physics:stiffness = 0\n"
                 f"        float drive:{lin}:physics:damping = 0\n"
                 f"        float drive:{lin}:physics:targetPosition = 0\n") if lk.law is not None else ""
        out.append(f'    def {kind} "{lk.name}_joint" (\n        prepend apiSchemas = ["PhysxJointAPI", "PhysicsJointStateAPI:{lin}"{drive_api}]\n    )\n    {{\n' + drive +
                   f'        float state:{lin}:physics:position = {lk.start * scale:.7g}\n'
                   f'        float state:{lin}:physics:velocity = 0\n'
                   f'        uniform token physics:axis = "{lk.axis}"\n'
                   f"        rel physics:body0 = </Root/base>\n        rel physics:body1 = </Root/{lk.name}>\n"
                   f"        point3f physics:localPos0 = {_fmt(lk.anchor)}\n        quatf physics:localRot0 = (1, 0, 0, 0)\n"
                   f"        point3f physics:localPos1 = (0, 0, 0)\n        quatf physics:localRot1 = (1, 0, 0, 0)\n"
                   f"        float physics:lowerLimit = {(lk.lower - 1e-4) * scale:.7g}\n"
                   f"        float physics:upperLimit = {(lk.upper + 1e-4) * scale:.7g}\n"
                   f"        float physxJoint:jointFriction = {lk.friction}\n    }}\n")
    out.append("}\n")
    return "".join(out)


def _write_text(name: str, text: str) -> str:
    folder = os.path.join(tempfile.gettempdir(), "yamkit_usd")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{name}_{hashlib.sha1(text.encode()).hexdigest()[:12]}.usda")
    if not os.path.exists(path):
        with open(path + ".tmp", "w") as f:
            f.write(text)
        os.replace(path + ".tmp", path)
    return path


def _mech_object(scene, spec: Mechanism):
    from isaaclab.actuators import ImplicitActuatorCfg

    class _MechObject(scene.Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            # Implicit actuators carry each joint's spring and damping; their targets are set every step (_drive).
            # No stiffness here: the drive's position target starts at 0, not at the joint's start, so a stiff drive
            # would pull the part there while the scene settles. The spring is applied as a torque each step instead.
            cfg.actuators = {lk.name: ImplicitActuatorCfg(joint_names_expr=[f"{lk.name}_joint"], stiffness=0.0,
                                                          damping=lk.damping, friction=lk.friction)
                             for lk in spec.links}
            try:  # never sleep (a sleeping articulation ignores effort targets and position writes)
                from isaaclab_physx.sim.schemas import ArticulationRootPropertiesCfg

                if hasattr(cfg.spawn, "articulation_props"):
                    cfg.spawn.articulation_props = ArticulationRootPropertiesCfg(sleep_threshold=0.0,
                                                                                 stabilization_threshold=0.0)
            except ImportError:
                pass
            cfg.init_state.joint_pos = {f"{lk.name}_joint": lk.start for lk in spec.links}
            cfg.init_state.joint_vel = {f"{lk.name}_joint": 0.0 for lk in spec.links}
            return cfg

    yaw = (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))
    return _MechObject(name=spec.name, usd_path=_write_text(spec.name, _mech_usda(spec)),
                       object_type=scene.ObjectType.ARTICULATION,
                       initial_pose=scene.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_quat_mul(yaw, spec.tilt_xyzw)),
                       spawn_cfg_addon={"physics_material": scene._grippy()})


def _set_drive(art, j: int, target: float, stiffness: float, effort: float, damping: float | None = None) -> None:
    import torch

    dev = getattr(art.data.joint_pos, "torch", art.data.joint_pos).device
    one = lambda v: torch.tensor([[v]], dtype=torch.float32, device=dev)  # noqa: E731
    art.set_joint_position_target_index(target=one(target), joint_ids=[j])
    art.set_joint_effort_target_index(target=one(effort), joint_ids=[j])
    art.write_joint_stiffness_to_sim_index(stiffness=one(stiffness), joint_ids=[j])
    if damping is not None:
        art.write_joint_damping_to_sim_index(damping=one(damping), joint_ids=[j])


def _put_joint(art, j: int, value: float) -> None:
    """Move a joint to `value` at rest (the scene's reset leaves articulated parts where they were)."""
    import torch

    dev = getattr(art.data.joint_pos, "torch", art.data.joint_pos).device
    art.write_joint_position_to_sim_index(position=torch.tensor([[value]], dtype=torch.float32, device=dev),
                                          joint_ids=[j])
    art.write_joint_velocity_to_sim_index(velocity=torch.tensor([[0.0]], dtype=torch.float32, device=dev),
                                          joint_ids=[j])


def _watch_resets(base) -> None:
    """After every scene reset, release the mechanisms' drives (no latch hold, no spring torque) so that nothing
    left over from the last trial moves a part while the scene settles."""
    if getattr(base, "_yamkit_reset", False):
        return
    base._yamkit_reset = True
    original = base.reset

    def reset(*a, **k):
        out = original(*a, **k)
        for name, spec in _MECHS.items():
            try:
                art = base.scene[name]
            except KeyError:
                continue
            names = list(art.joint_names)
            for lk in spec.links:
                j = names.index(f"{lk.name}_joint")
                _set_drive(art, j, target=lk.start, stiffness=0.0, effort=0.0)
                _put_joint(art, j, lk.start)
        return out

    base.reset = reset


def _mech_states(base, name: str, origin) -> dict:
    """The base's and each link's states, the joints into _JOINTS, and each joint's spring or latch target (_drive)."""
    import torch

    spec = _MECHS[name]
    art = base.scene[name]
    np_ = lambda t: getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)  # noqa: E731
    bpos, bquat = np_(art.data.body_pos_w)[0], np_(art.data.body_quat_w)[0]
    bodies = list(art.body_names)
    q, qd = np_(art.data.joint_pos)[0], np_(art.data.joint_vel)[0]
    jnames = list(art.joint_names)
    out = {}

    def state(i, size):
        p = bpos[i] - origin
        half = np.asarray(size, float) / 2
        return ObjectState(pos=p, quat_xyzw=bquat[i], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half,
                           high=p + half)

    out[name] = state(bodies.index("base"), spec.size)
    for lk in spec.links:
        key = f"{name}/{lk.name}"
        j = jnames.index(f"{lk.name}_joint")
        latched = _LATCHED.get(key) == _EPISODE["count"]
        if lk.latch is not None and not latched and abs(q[j] - lk.latch.at) <= lk.latch.zone:
            _LATCHED[key] = _EPISODE["count"]
            latched = True
        _JOINTS[key] = JointState(pos=float(q[j]), vel=float(qd[j]), latched=latched)
        if os.environ.get("YAMKIT_DEBUG_JOINTS"):
            print(f"yamkit joint {key} q={q[j]:.4f} qd={qd[j]:.4f} latched={latched}", flush=True)
        ext = [max(abs(c.center[k]) + (c.size[k] / 2 if c.size is not None else c.radius) for c in lk.colliders)
               for k in range(3)] if lk.colliders else [0.01] * 3
        out[key] = state(bodies.index(lk.name), [2 * e for e in ext])
        # The joint's drive for the next step: a caught latch holds it at the catch (a stiff drive to that position);
        # otherwise the spring pushes it toward its rest as a torque, -spring * (q - rest).
        rest = lk.start if lk.rest is None else lk.rest
        if lk.law is not None and not latched:
            jq = {o.name: float(q[jnames.index(f"{o.name}_joint")]) for o in spec.links}
            out_ = lk.law.drive(key, float(q[j]), float(qd[j]), jq, _law_mem(key, lk))
            target, stiff, effort = out_[:3]
            damp_ = out_[3] if len(out_) > 3 else None  # implicit damping (N*m*s/rad or N*s/m), applied by PhysX
            if getattr(lk.law, "kinematic", False):
                # an indicator set, not pushed: move the joint toward the law's target by at most `rate` per control
                # step and hold it there softly (light parts such as gauge needles ring or blow up under an effort PD)
                goal = _law_mem(key, lk).get("target", target)
                nxt = float(q[j]) + min(max(goal - float(q[j]), -lk.law.rate), lk.law.rate)
                nxt = min(max(nxt, lk.lower), lk.upper)
                _put_joint(art, j, nxt)
                _set_drive(art, j, target=nxt, stiffness=lk.law.hold, effort=0.0, damping=0.0)
            else:
                _set_drive(art, j, target=target, stiffness=stiff, effort=effort, damping=damp_)
        elif latched:
            _set_drive(art, j, target=lk.latch.at, stiffness=lk.latch.hold, effort=0.0)
        else:
            drag = min(lk.drag, 0.8 * lk.mass / 0.05) if lk.kind == "prismatic" else lk.drag
            _set_drive(art, j, target=float(q[j]), stiffness=lk.stay,
                       effort=-lk.spring * (float(q[j]) - rest) - drag * float(qd[j]))
    return out


# -- tilted grips for demos --------------------------------------------------------------------


@dataclass
class Tilted:
    """A demo waypoint like yam_arena's `Waypoint`, with the straight-down grip also tilted: `roll` about world x, then
    `pitch` about world y (radians), after `yaw` about the vertical. Plays in the same plan as plain waypoints."""

    duration: float
    pos: tuple
    finger: float
    label: str = ""
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    rot: tuple | None = None  # optional full TCP rotation (3x3 rows); replaces yaw/roll/pitch when given


@dataclass
class Home:
    """A demo waypoint that takes the arm back to the pose it starts every trial in (yam_arena's READY), position and
    orientation, fingers open. Use it last, so the arms end where they began."""

    duration: float
    finger: float = FINGER_OPEN
    label: str = "home"
    pos: tuple = (0.0, 0.0, 0.0)  # unused; Waypoint-compatible
    yaw: float = 0.0


def _rot_xyz(roll: float, pitch: float) -> np.ndarray:
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def _patch_demo(demo) -> None:
    if getattr(demo, "_yamkit", False):
        return
    demo._yamkit = True
    plain = demo.grip_rotation

    def target(self, t: float):
        """yam_arena's ArmPlan.target, plus Tilted waypoints' extra rotation."""
        prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            if isinstance(w, Home):
                goal, rot = demo.ik.tcp_pose(np.asarray(demo.READY, dtype=float), self.base_pos)
                goal = np.asarray(goal, dtype=float)
            else:
                goal, rot = np.asarray(w.pos, dtype=float), plain(w.yaw)
            if isinstance(w, Tilted) and getattr(w, "rot", None) is not None:
                rot = np.asarray(w.rot, dtype=float).reshape(3, 3)
            elif isinstance(w, Tilted) and (w.roll or w.pitch):
                rot = _rot_xyz(w.roll, w.pitch) @ rot
            if t < w.duration:
                s = demo.ik.smoothstep(t / w.duration)
                return (prev_pos + s * (goal - prev_pos), demo.ik.slerp_matrix(prev_rot, rot, s),
                        prev_finger + s * (w.finger - prev_finger), w.label)
            t -= w.duration
            prev_pos, prev_rot, prev_finger = goal, rot, w.finger
        return prev_pos, prev_rot, prev_finger, "done"

    demo.ArmPlan.target = target


# -- hooks into yam_arena.scene --------------------------------------------------------------


def _patch(scene) -> None:
    if getattr(scene, "_yamkit", False):
        return
    scene._yamkit = True
    original_task_object, original_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Mechanism):
            _MECHS[spec.name] = spec
            return _mech_object(scene, spec)
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
        out = original_states(env, [n for n in names if n not in GRIPPERS and n not in _MECHS])
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        if buf is not None:  # the env counts steps since its last reset; a drop means a new trial began
            n = int(buf[0].item())
            if _EPISODE["last"] is not None and n < _EPISODE["last"]:
                _EPISODE["count"] += 1
            _EPISODE["last"] = n
            if os.environ.get("YAMKIT_DEBUG_EPISODE"):
                print(f"yamkit episode {_EPISODE['count']} step {n}", flush=True)
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        if _MECHS:
            _watch_resets(base)
        for name in [n for n in names if n in _MECHS]:
            out.update(_mech_states(base, name, origin))
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


def episode() -> int:
    """Which trial this is: 0 for the first, +1 each time the simulator resets the scene. Scorers that remember a
    run's history start it afresh when this changes."""
    return _EPISODE["count"]


def opening(objects, side: str) -> float:
    """How far a gripper's fingers are open, 0 (closed) to 1 (fully open), as of the last step; side is
    "left_gripper" or "right_gripper"."""
    return _OPENING.get(side, float("nan"))


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _Hook) for f in sys.meta_path):
    sys.meta_path.insert(0, _Hook())

import yam_arena.demo as _yam_demo  # noqa: E402  (plain Python: lets demos use Tilted waypoints)

_patch_demo(_yam_demo)
