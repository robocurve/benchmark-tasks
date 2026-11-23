# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of
# the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""linkkit: articulated rigid-link objects (hinged panels, strips, jointed arms, buttons) for yam_arena task files.

Keep this file next to the task file and import it before declaring OBJECTS:

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    import linkkit as lk

    t = lk.Tree("flap", pos=(0.30, -0.12, TOP + 0.002))
    t.link("base", [lk.box((0.08, 0.06, 0.004))], mass=0.06)
    t.link("lid", [lk.box((0.08, 0.06, 0.004))], mass=0.02, at=(0.0805, 0, 0))
    t.revolute("hinge", "base", "lid", anchor=(0.04025, 0, 0.002), axis=(0, -1, 0), limits_deg=(-5, 180),
               rest_deg=30, stiffness=0.005, damping=5e-4)
    FLAP = t.build()
    OBJECTS = [FLAP, ...]

What it adds to yam_arena, without modifying it (importing this module hooks ``yam_arena.scene`` and the
embodiment):

- a :class:`Linkage` in ``OBJECTS`` spawns as a PhysX articulation (or a single compound rigid body when it has no
  moving joint): links made of box / cylinder / capsule colliders with mass, friction, colour and per-collider
  contact and rest offsets; revolute and prismatic joints with limits, a spring drive to a rest position, damping,
  joint friction and armature; fixed joints; loop-closing joints kept out of the articulation;
- ``success()/score()`` get a :class:`LinkState` for it: every link's pose and velocity, every joint's position
  (rad or m, 0 at the design pose) and velocity;
- :func:`on_step` / :func:`on_reset` callbacks get a :class:`Tick` (env, time, step, lazily read states and gripper
  tools) after every control step / reset, before the task file is scored;
- runtime switches callable from on_step: :func:`brake`, :func:`set_rest` (detent / over-centre flip),
  :func:`set_spring`, :func:`set_limits` (ratchet), :func:`set_friction`, and the ready-made :func:`detent` and
  :func:`ratchet`;
- :class:`History`: per-episode peaks, lows, any-time / all-time flags, held durations and traces, updated by your
  on_step code every control step and cleared only when the simulator resets;
- :func:`grippers`: both grippers' measured tool points and openings; :func:`clearance` from a gripper to a body.

Units: SI (m, kg, rad, N, N·m) everywhere except the ``*_deg`` builder arguments. Frames: world x forward, y left,
z up; quaternions are (x, y, z, w). Pure Python at import time; Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = [
    "Brake", "History", "Joint", "Link", "LinkState", "Linkage", "Shape", "Tick", "Tool", "Tree",
    "box", "brake", "braked", "capsule", "chain", "clearance", "cylinder", "detent", "env", "grippers",
    "install", "on_reset", "on_step", "ratchet", "set_friction", "set_limits", "set_rest", "set_spring",
    "slack", "winding",
]

_SPECS: dict = {}  # linkage name -> Linkage, filled as linkages are built and spawned

MIN_LINK_MASS = 0.003  # kg: lighter links make PhysX articulations jitter
CONTACT_OFFSET = 0.002  # m, default per collider; thin panels want 0.001
REST_OFFSET = 0.0
POS_ITERS, VEL_ITERS = 32, 4
MAX_DEPENETRATION = 1.0  # m/s
GREY = (0.7, 0.7, 0.7)


# -- small rotation helpers (numpy) -----------------------------------------------------------------------------


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    r = np.asarray(r, dtype=float)
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w, x, y, z = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w, x, y, z = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w, x, y, z = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q / np.linalg.norm(q))


def axis_rotation(axis, angle: float) -> np.ndarray:
    """Right-handed rotation by ``angle`` (rad) about the unit vector ``axis``."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def rpy_matrix(rpy_deg) -> np.ndarray:
    """Roll about x, then pitch about y, then yaw about z (degrees, fixed axes)."""
    r, p, y = (math.radians(v) for v in rpy_deg)
    return axis_rotation((0, 0, 1), y) @ axis_rotation((0, 1, 0), p) @ axis_rotation((1, 0, 0), r)


def x_to(axis) -> np.ndarray:
    """A rotation taking +x onto ``axis`` (the joint frame: joints move about / along their local x)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    c = float(a[0])
    v = np.cross([1.0, 0.0, 0.0], a)
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return np.eye(3) if c > 0 else axis_rotation((0, 0, 1), math.pi)
    return axis_rotation(v / s, math.atan2(s, c))


def rotation_angle_about(r: np.ndarray, axis) -> float:
    """Signed angle (rad) of rotation ``r`` about ``axis`` (exact when ``r`` turns about that axis)."""
    q = np.array(matrix_to_quat(r))
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return 2.0 * math.atan2(float(q[:3] @ a), float(q[3]))


def _pose(pos=(0, 0, 0), rot=None) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = np.asarray(pos, dtype=float)
    if rot is not None:
        m[:3, :3] = rot
    return m


# -- what a task file declares ------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Shape:
    """One collider in a link's frame. Build with :func:`box`, :func:`cylinder` or :func:`capsule`."""

    kind: str  # "box", "cylinder" or "capsule"
    dims: tuple[float, ...]  # box: x, y, z edges; cylinder: radius, height; capsule: radius, straight length
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w in the link frame
    axis: str = "Z"  # long axis of a cylinder or capsule, in the shape frame
    color: tuple[float, float, float] | None = None  # sRGB 0..1; None: the link's colour
    contact_offset: float | None = None  # None: the link's
    rest_offset: float | None = None

    @property
    def rot(self) -> np.ndarray:
        return quat_to_matrix(self.quat)

    def half_extents(self) -> np.ndarray:
        """Half extents along the shape frame's x, y, z."""
        if self.kind == "box":
            return np.asarray(self.dims, dtype=float) / 2
        r = self.dims[0]
        half_len = self.dims[1] / 2 + (r if self.kind == "capsule" else 0.0)
        out = np.array([r, r, r])
        out["XYZ".index(self.axis)] = half_len
        return out

    def distance(self, p_local: np.ndarray) -> float:
        """Distance (m) from a point in the link frame to this collider (0 inside)."""
        p = self.rot.T @ (np.asarray(p_local, dtype=float) - np.asarray(self.pos))
        if self.kind == "box":
            return float(np.linalg.norm(np.maximum(np.abs(p) - self.half_extents(), 0.0)))
        i = "XYZ".index(self.axis)
        along = abs(p[i])
        radial = math.sqrt(max(float(p @ p) - p[i] ** 2, 0.0))
        r, h = self.dims[0], self.dims[1] / 2
        if self.kind == "capsule":
            return max(math.hypot(radial, max(along - h, 0.0)) - r, 0.0)
        return math.hypot(max(radial - r, 0.0), max(along - h, 0.0))


def _shape(kind, dims, at, rpy_deg, quat, axis, color, contact_offset, rest_offset) -> Shape:
    q = tuple(quat) if quat is not None else matrix_to_quat(rpy_matrix(rpy_deg))
    return Shape(kind, tuple(float(d) for d in dims), tuple(float(v) for v in at), q, axis, color, contact_offset,
                 rest_offset)


def box(size, at=(0.0, 0.0, 0.0), *, rpy_deg=(0.0, 0.0, 0.0), quat=None, color=None, contact_offset=None,
        rest_offset=None) -> Shape:
    """A box collider: ``size`` edges (m), centred ``at`` in the link frame, turned ``rpy_deg`` (or ``quat``)."""
    return _shape("box", size, at, rpy_deg, quat, "Z", color, contact_offset, rest_offset)


def cylinder(radius, height, at=(0.0, 0.0, 0.0), *, axis="Z", rpy_deg=(0.0, 0.0, 0.0), quat=None, color=None,
             contact_offset=None, rest_offset=None) -> Shape:
    """A cylinder collider along ``axis`` ("X", "Y" or "Z" of its own frame)."""
    return _shape("cylinder", (radius, height), at, rpy_deg, quat, axis, color, contact_offset, rest_offset)


def capsule(radius, length, at=(0.0, 0.0, 0.0), *, axis="X", rpy_deg=(0.0, 0.0, 0.0), quat=None, color=None,
            contact_offset=None, rest_offset=None) -> Shape:
    """A capsule collider: ``length`` is the straight part between the two hemispheres."""
    return _shape("capsule", (radius, length), at, rpy_deg, quat, axis, color, contact_offset, rest_offset)


@dataclass(frozen=True)
class Link:
    """A rigid body: its colliders, mass and surface, at its design pose in the linkage frame."""

    name: str
    shapes: tuple[Shape, ...]
    mass: float  # kg
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    color: tuple[float, float, float] = GREY
    friction: tuple[float, float] = (0.8, 0.8)  # static, dynamic
    restitution: float = 0.0
    com: tuple[float, float, float] | None = None  # centre of mass in the link frame; None: from the colliders
    contact_offset: float = CONTACT_OFFSET
    rest_offset: float = REST_OFFSET
    linear_damping: float = 0.0
    angular_damping: float = 0.05
    max_depenetration: float = MAX_DEPENETRATION

    @property
    def design(self) -> np.ndarray:
        return _pose(self.pos, quat_to_matrix(self.quat))


@dataclass(frozen=True)
class Brake:
    """A holding drive switched by :func:`brake`: stiff spring to wherever the joint was when it engaged."""

    stiffness: float = 20.0  # N·m/rad (revolute) or N/m (prismatic)
    damping: float = 1.0
    engaged: bool = False  # engaged at reset, holding the joint's initial position


@dataclass(frozen=True)
class Joint:
    """A joint between two links (positions: rad or m, 0 at the design pose)."""

    name: str
    kind: str  # "revolute", "prismatic" or "fixed"
    parent: str
    child: str
    anchor: tuple[float, float, float]  # on the axis, linkage frame at the design pose
    axis: tuple[float, float, float] = (0.0, 1.0, 0.0)  # positive motion: right-handed about / along it
    lower: float | None = None
    upper: float | None = None
    rest: float = 0.0  # where the spring drive pulls
    init: float = 0.0  # position at reset
    stiffness: float = 0.0
    damping: float = 0.0
    friction: float = 0.0  # static (Coulomb) friction effort, N·m or N
    dynamic_friction: float = 0.0
    viscous_friction: float = 0.0
    armature: float = 0.0
    brake: Brake | None = None
    loop: bool = False  # closes a loop: simulated outside the articulation (no readout, no drive)
    break_force: float | None = None  # loop joints only: N to part it
    break_torque: float | None = None

    @property
    def moving(self) -> bool:
        return self.kind in ("revolute", "prismatic") and not self.loop


@dataclass(frozen=True)
class Linkage(Box):
    """Links and joints spawned as one articulation. Build with :class:`Tree` or :func:`chain`.

    ``pos``/``yaw`` place the linkage frame (the root link, the first in ``links``, sits at its own design pose in
    it). ``size`` is the design-pose bounding box (nominal; yaw_arena only checks it is positive). It subclasses Box
    so yam_arena's task-file checks accept it.
    """

    links: tuple[Link, ...] = ()
    joints: tuple[Joint, ...] = ()
    fixed: bool = False  # root link welded to the world (a fixture with moving parts)
    self_collision: bool = True
    pos_iters: int = POS_ITERS
    vel_iters: int = VEL_ITERS
    sleep_threshold: float = 0.0  # 0: never sleeps (a sleeping strip ignores small pushes)
    stabilization_threshold: float = 0.0
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    @property
    def free(self) -> bool:  # keeps yam_arena's jitter (which writes root poses only) off it
        return False

    @property
    def moving_joints(self) -> tuple[Joint, ...]:
        return tuple(j for j in self.joints if j.moving)

    def link(self, name: str) -> Link:
        for lk in self.links:
            if lk.name == name:
                return lk
        raise KeyError(f"{self.name}: no link {name!r}")

    def joint(self, name: str) -> Joint:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"{self.name}: no joint {name!r}")

    @property
    def frame(self) -> np.ndarray:
        return _pose(self.pos, axis_rotation((0, 0, 1), self.yaw))

    def _check(self) -> None:
        super()._check()
        names = [lk.name for lk in self.links] + [j.name for j in self.joints]
        if not self.links:
            raise ValueError(f"{self.name}: a linkage needs at least one link")
        if len(set(names)) != len(names) or not all(n.isidentifier() for n in names):
            raise ValueError(f"{self.name}: link and joint names must be unique Python identifiers")
        links = {lk.name for lk in self.links}
        for lk in self.links:
            if not lk.mass >= MIN_LINK_MASS:
                raise ValueError(f"{self.name}.{lk.name}: mass {lk.mass} kg is under {MIN_LINK_MASS} kg")
            if not lk.shapes:
                raise ValueError(f"{self.name}.{lk.name}: a link needs at least one collider")
        parents: dict[str, str] = {}
        for j in self.joints:
            if j.kind not in ("revolute", "prismatic", "fixed"):
                raise ValueError(f"{self.name}.{j.name}: unknown joint kind {j.kind!r}")
            if j.parent not in links or j.child not in links:
                raise ValueError(f"{self.name}.{j.name}: parent and child must be links of this linkage")
            if j.lower is not None and j.upper is not None and not j.lower < j.upper:
                raise ValueError(f"{self.name}.{j.name}: lower limit must be below upper")
            if (j.break_force or j.break_torque) and not j.loop:
                raise ValueError(f"{self.name}.{j.name}: only loop joints can break (PhysX articulation joints cannot)")
            if not j.loop:
                if j.child in parents or j.child == self.links[0].name:
                    raise ValueError(f"{self.name}.{j.name}: link {j.child!r} already has a parent (mark loops loop=True)")
                parents[j.child] = j.parent
        for lk in self.links[1:]:
            seen, cur = set(), lk.name
            while cur != self.links[0].name:
                if cur not in parents or cur in seen:
                    raise ValueError(f"{self.name}: link {lk.name!r} is not joined to the root {self.links[0].name!r}")
                seen.add(cur)
                cur = parents[cur]
        if len(self.links) > 1 and not self.moving_joints:
            raise ValueError(f"{self.name}: several links need a moving joint; merge rigid parts into one link")

    # -- kinematics at any joint positions (pure numpy; the demo and scorer use it) --

    def link_poses(self, q: dict[str, float] | None = None, frame: np.ndarray | None = None) -> dict[str, np.ndarray]:
        """4x4 world pose of every link with joints at ``q`` (missing: ``init``), linkage at ``frame``."""
        q = q or {}
        world = {self.links[0].name: (self.frame if frame is None else frame) @ self.links[0].design}
        tree = [j for j in self.joints if not j.loop]
        while len(world) < len(self.links):
            grew = False
            for j in tree:
                if j.parent in world and j.child not in world:
                    v = q.get(j.name, j.init)
                    a = np.asarray(j.axis, dtype=float) / np.linalg.norm(j.axis)
                    if j.kind == "revolute":
                        m = _pose(j.anchor) @ _pose(rot=axis_rotation(a, v)) @ _pose(-np.asarray(j.anchor))
                    elif j.kind == "prismatic":
                        m = _pose(a * v)
                    else:
                        m = np.eye(4)
                    world[j.child] = world[j.parent] @ np.linalg.inv(self.link(j.parent).design) @ m @ self.link(
                        j.child).design
                    grew = True
            if not grew:
                break
        return world

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Design-pose world bounding box (yaw applied)."""
        pts = []
        for name, m in self.link_poses({j.name: 0.0 for j in self.joints}).items():
            for s in self.link(name).shapes:
                h = s.half_extents()
                for corner in np.array(np.meshgrid(*[[-1, 1]] * 3)).T.reshape(-1, 3) * h:
                    pts.append(m[:3, :3] @ (np.asarray(s.pos) + s.rot @ corner) + m[:3, 3])
        pts = np.array(pts)
        return pts.min(0), pts.max(0)


def _linkage(name, pos, yaw, color, links, joints, **kw) -> Linkage:
    probe = Linkage(name=name, pos=tuple(pos), size=(1, 1, 1), color=color, yaw=yaw, links=tuple(links),
                    joints=tuple(joints), **kw)
    lo, hi = probe.bounds()
    size = tuple(float(max(v, 1e-3)) for v in hi - lo)
    out = Linkage(name=name, pos=tuple(float(v) for v in pos), size=size, color=color, yaw=yaw, links=tuple(links),
                  joints=tuple(joints), **kw)
    out._check()
    _SPECS[name] = out  # known from import time on, so switches can be declared next to the objects
    return out


class Tree:
    """Builder for any tree of links (cartons, letters, limbs, fixtures with buttons). The first link is the root."""

    def __init__(self, name: str, pos, *, yaw_deg: float = 0.0, color=GREY, fixed: bool = False, **linkage_kw):
        self.name, self.pos, self.yaw, self.color, self.fixed = name, tuple(pos), math.radians(yaw_deg), color, fixed
        self.kw = linkage_kw
        self.links: list[Link] = []
        self.joints: list[Joint] = []

    def link(self, name: str, shapes: Iterable[Shape], mass: float, *, at=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0),
             color=None, friction=(0.8, 0.8), **kw) -> str:
        """Add a link at its design pose ``at``/``rpy_deg`` in the linkage frame. ``kw``: any :class:`Link` field."""
        f = (friction, friction) if isinstance(friction, int | float) else tuple(friction)
        self.links.append(Link(name=name, shapes=tuple(shapes), mass=float(mass), pos=tuple(float(v) for v in at),
                               quat=matrix_to_quat(rpy_matrix(rpy_deg)), color=color or self.color, friction=f, **kw))
        return name

    def revolute(self, name, parent, child, *, anchor, axis=(0.0, 1.0, 0.0), limits_deg=None, rest_deg=0.0,
                 init_deg=0.0, **kw) -> str:
        """A hinge about ``axis`` through ``anchor``. ``kw``: stiffness (N·m/rad), damping, friction, armature, brake."""
        lo, hi = (None, None) if limits_deg is None else (math.radians(limits_deg[0]), math.radians(limits_deg[1]))
        self.joints.append(Joint(name, "revolute", parent, child, tuple(anchor), tuple(axis), lo, hi,
                                 math.radians(rest_deg), math.radians(init_deg), **kw))
        return name

    def prismatic(self, name, parent, child, *, anchor, axis=(0.0, 0.0, 1.0), limits=None, rest=0.0, init=0.0,
                  **kw) -> str:
        """A slider along ``axis``. ``kw``: stiffness (N/m), damping, friction, armature, brake."""
        lo, hi = (None, None) if limits is None else (float(limits[0]), float(limits[1]))
        self.joints.append(Joint(name, "prismatic", parent, child, tuple(anchor), tuple(axis), lo, hi, float(rest),
                                 float(init), **kw))
        return name

    def fixed_joint(self, name, parent, child, *, anchor=None) -> str:
        """Weld ``child`` to ``parent`` as they are at the design pose."""
        a = anchor if anchor is not None else next(lk.pos for lk in self.links if lk.name == child)
        self.joints.append(Joint(name, "fixed", parent, child, tuple(a), (1.0, 0.0, 0.0)))
        return name

    def loop(self, name, parent, child, *, anchor, kind="revolute", axis=(0.0, 1.0, 0.0), break_force=None,
             break_torque=None) -> str:
        """A joint closing a loop (kept out of the articulation). It can break at ``break_force`` (N)."""
        self.joints.append(Joint(name, kind, parent, child, tuple(anchor), tuple(axis), loop=True,
                                 break_force=break_force, break_torque=break_torque))
        return name

    def build(self) -> Linkage:
        return _linkage(self.name, self.pos, self.yaw, self.color, self.links, self.joints, fixed=self.fixed, **self.kw)


def chain(name: str, n: int, *, start, pitch: float, shapes: Iterable[Shape], mass: float, yaw_deg: float = 0.0,
          kind: str = "revolute", axis=(0.0, 1.0, 0.0), limits_deg=(-150.0, 150.0), limits=None, rest_deg=0.0,
          stiffness: float = 0.0, damping: float = 0.0, friction: float = 0.0, armature: float = 0.0,
          color=GREY, colors=None, fixed: bool = False, link_kw: dict | None = None, **linkage_kw) -> Linkage:
    """``n`` identical links along +x (turned ``yaw_deg``), the first centred at ``start``, ``pitch`` apart, each
    joined to the next at the midpoint: straps, tape, slat mats, roller chains. ``axis`` is in the link frame
    ((0, 1, 0): bends up and down; (0, 0, 1): bends sideways). ``colors`` cycles per link."""
    t = Tree(name, start, yaw_deg=yaw_deg, color=color, fixed=fixed, **linkage_kw)
    shapes = tuple(shapes)
    for i in range(n):
        c = colors[i % len(colors)] if colors else color
        t.link(f"l{i}", shapes, mass, at=(i * pitch, 0.0, 0.0), color=c, **(link_kw or {}))
    for i in range(1, n):
        anchor = ((i - 0.5) * pitch, 0.0, 0.0)
        common = dict(stiffness=stiffness, damping=damping, friction=friction, armature=armature)
        if kind == "revolute":
            t.revolute(f"j{i}", f"l{i - 1}", f"l{i}", anchor=anchor, axis=axis, limits_deg=limits_deg,
                       rest_deg=rest_deg, **common)
        else:
            t.prismatic(f"j{i}", f"l{i - 1}", f"l{i}", anchor=anchor, axis=axis, limits=limits, **common)
    return t.build()


# -- state handed to success()/score() ------------------------------------------------------------------------------


@dataclass(frozen=True)
class LinkState(ObjectState):
    """ObjectState for a linkage: ``pos``/``quat_xyzw``/velocities are the root link's; ``low``/``high`` bound all
    colliders. Per link: ``poses`` (4x4 world), ``lin_vels``, ``ang_vels``; per moving joint: ``q`` (rad or m, 0 at
    the design pose) and ``qd``."""

    spec: Linkage = None
    poses: dict = field(default_factory=dict)
    lin_vels: dict = field(default_factory=dict)
    ang_vels: dict = field(default_factory=dict)
    q: dict = field(default_factory=dict)
    qd: dict = field(default_factory=dict)

    def deg(self, joint: str) -> float:
        return math.degrees(self.q[joint])

    def point(self, link: str, local=(0.0, 0.0, 0.0)) -> np.ndarray:
        m = self.poses[link]
        return m[:3, :3] @ np.asarray(local, dtype=float) + m[:3, 3]

    def rotation(self, link: str) -> np.ndarray:
        return self.poses[link][:3, :3]

    def max_speed(self) -> float:
        return max(float(np.linalg.norm(v)) for v in self.lin_vels.values())

    def max_spin(self) -> float:
        return max(float(np.linalg.norm(v)) for v in self.ang_vels.values())

    def resting(self, max_speed: float = 0.01, max_spin: float = 0.5) -> bool:
        return self.max_speed() < max_speed and self.max_spin() < max_spin

    def joint_error(self, joint: str) -> float:
        """Gap (m) between a joint's anchor as carried by its parent and by its child: ~0 while a joint holds.
        Use it on loop joints (not read back by the articulation) and to detect a broken loop."""
        j = self.spec.joint(joint)
        a = np.r_[np.asarray(j.anchor, dtype=float), 1.0]
        p = self.poses[j.parent] @ np.linalg.inv(self.spec.link(j.parent).design) @ a
        c = self.poses[j.child] @ np.linalg.inv(self.spec.link(j.child).design) @ a
        if j.kind == "prismatic":  # the anchors may slide apart along the axis
            d = (c - p)[:3]
            ax = self.poses[j.parent][:3, :3] @ np.linalg.inv(self.spec.link(j.parent).design)[:3, :3] @ np.asarray(j.axis)
            ax = ax / np.linalg.norm(ax)
            return float(np.linalg.norm(d - (d @ ax) * ax))
        return float(np.linalg.norm((c - p)[:3]))

    def distance(self, point, links: Iterable[str] | None = None) -> float:
        """Distance (m) from a world point to the nearest collider of ``links`` (default: all)."""
        best = math.inf
        for name in links or self.poses:
            m = self.poses[name]
            p = m[:3, :3].T @ (np.asarray(point, dtype=float) - m[:3, 3])
            for s in self.spec.link(name).shapes:
                best = min(best, s.distance(p))
        return best


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def _linkage_state(environment, spec: Linkage) -> LinkState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins)[0]
    if spec.moving_joints:
        names = list(asset.body_names)
        pos = _np(asset.data.body_pos_w)[0] - origin
        quat = _np(asset.data.body_quat_w)[0]
        lv = _np(asset.data.body_lin_vel_w)[0]
        av = _np(asset.data.body_ang_vel_w)[0]
        idx = {lk.name: names.index(lk.name) for lk in spec.links}
        jn = list(asset.joint_names)
        jp, jv = _np(asset.data.joint_pos)[0], _np(asset.data.joint_vel)[0]
        q = {j.name: float(jp[jn.index(j.name)]) + j.rest for j in spec.moving_joints}
        qd = {j.name: float(jv[jn.index(j.name)]) for j in spec.moving_joints}
    else:
        pos = (_np(asset.data.root_pos_w)[0] - origin)[None]
        quat = _np(asset.data.root_quat_w)[0][None]
        lv = _np(asset.data.root_lin_vel_w)[0][None]
        av = _np(asset.data.root_ang_vel_w)[0][None]
        idx = {spec.links[0].name: 0}
        q, qd = {}, {}
    poses = {n: _pose(pos[i], quat_to_matrix(quat[i])) for n, i in idx.items()}
    pts = []
    for n, m in poses.items():
        for s in spec.link(n).shapes:
            c = m[:3, :3] @ np.asarray(s.pos) + m[:3, 3]
            r = float(np.linalg.norm(s.half_extents()))
            pts += [c - r, c + r]
    pts = np.array(pts)
    root = spec.links[0].name
    return LinkState(pos=poses[root][:3, 3].copy(), quat_xyzw=quat[idx[root]].copy(), lin_vel=lv[idx[root]].copy(),
                     ang_vel=av[idx[root]].copy(), low=pts.min(0), high=pts.max(0), spec=spec, poses=poses,
                     lin_vels={n: lv[i].copy() for n, i in idx.items()},
                     ang_vels={n: av[i].copy() for n, i in idx.items()}, q=q, qd=qd)


# -- grippers -----------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Tool:
    """A gripper as measured in the simulator: TCP (between the pads, 13 mm short of the tips), its rotation
    (x: closing axis, z: approach), the opening between the pads (m) and the gripper link's speed."""

    side: str
    tcp: np.ndarray
    rot: np.ndarray
    opening: float
    speed: float

    def pad_points(self) -> np.ndarray:
        """Sample points on both pads (world), for clearance checks."""
        from yam_arena.constants import PAD_TIP_PAST_TCP

        pts = []
        for sign in (-1.0, 1.0):
            for z in (-0.02, -0.005, PAD_TIP_PAST_TCP):
                pts.append(self.tcp + self.rot @ np.array([sign * self.opening / 2, 0.0, z]))
        return np.array(pts + [self.tcp])


def grippers(environment=None) -> dict[str, Tool]:
    """Both grippers' measured tool points (``{"left": Tool, "right": Tool}``)."""
    from yam_arena.constants import FINGER_JOINTS, TCP_OFFSET

    base = (environment or env()).unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = list(robot.body_names).index("gripper")
        p = _np(robot.data.body_pos_w)[0, i] - origin
        r = quat_to_matrix(_np(robot.data.body_quat_w)[0, i])
        fid = [list(robot.joint_names).index(n) for n in FINGER_JOINTS]
        fq = _np(robot.data.joint_pos)[0, fid]
        v = float(np.linalg.norm(_np(robot.data.body_lin_vel_w)[0, i]))
        out[side] = Tool(side, r @ np.asarray(TCP_OFFSET) + p, r, float(-fq.sum()), v)
    return out


def clearance(tool: Tool, state, links: Iterable[str] | None = None) -> float:
    """Smallest distance (m) from the gripper's pads to an object (a LinkState's colliders, else its bounding box)."""
    if isinstance(state, LinkState):
        return min(state.distance(p, links) for p in tool.pad_points())
    lo, hi = np.asarray(state.low), np.asarray(state.high)
    return min(float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0))) for p in tool.pad_points())


# -- geometry helpers for strips -------------------------------------------------------------------------------


def winding(points, centre, axis=(0.0, 0.0, 1.0)) -> float:
    """Total signed angle (rad) a polyline turns around the line through ``centre`` along ``axis``."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    rel = np.asarray(points, dtype=float) - np.asarray(centre, dtype=float)
    rel = rel - np.outer(rel @ a, a)
    total = 0.0
    for u, v in zip(rel[:-1], rel[1:], strict=True):
        total += math.atan2(float(np.cross(u, v) @ a), float(u @ v))
    return total


def slack(points, a=None, b=None) -> float:
    """Path length minus the straight distance between its ends (or between ``a`` and ``b``), m."""
    p = np.asarray(points, dtype=float)
    length = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    a = p[0] if a is None else np.asarray(a, dtype=float)
    b = p[-1] if b is None else np.asarray(b, dtype=float)
    return length - float(np.linalg.norm(b - a))


# -- history --------------------------------------------------------------------------------------------------------

_HISTORIES: list[History] = []


class History:
    """Whole-episode record. Feed it from an on_step callback every control step; the kit clears it when (and only
    when) the simulator resets, so returning an object never erases a violation.

        H = lk.History()
        @lk.on_step
        def track(tick):
            s = tick.states["flap"]
            H.peak("lift", s.deg("hinge"))           # running maximum
            H.ever("dropped", s.pos[2] < TOP - 0.01)  # true from the first step it happens
            H.held("released", far and still)       # seconds this has held without a break
    """

    def __init__(self):
        self.reset()
        _HISTORIES.append(self)

    def reset(self) -> None:
        self.t = 0.0
        self.dt = 0.0
        self.peaks: dict[str, float] = {}
        self.lows: dict[str, float] = {}
        self.evers: dict[str, float] = {}  # key -> time first true
        self.nevers: dict[str, bool] = {}  # key -> still true at every step so far
        self.runs: dict[str, float] = {}  # key -> seconds held without a break
        self.best_runs: dict[str, float] = {}
        self.values: dict[str, float] = {}
        self.traces: dict[str, list] = {}
        self.counts: dict[str, int] = {}

    def _tick(self, t: float) -> None:
        self.dt = max(t - self.t, 0.0)
        self.t = t

    def peak(self, key: str, value: float) -> float:
        v = float(value)
        self.peaks[key] = max(self.peaks.get(key, -math.inf), v)
        return self.peaks[key]

    def low(self, key: str, value: float) -> float:
        v = float(value)
        self.lows[key] = min(self.lows.get(key, math.inf), v)
        return self.lows[key]

    def span(self, key: str, value: float) -> float:
        """Track a value's range; returns max - min so far."""
        return self.peak(key, value) - self.low(key, value)

    def ever(self, key: str, cond: bool) -> bool:
        if cond and key not in self.evers:
            self.evers[key] = self.t
        return key in self.evers

    def always(self, key: str, cond: bool) -> bool:
        self.nevers[key] = self.nevers.get(key, True) and bool(cond)
        return self.nevers[key]

    def held(self, key: str, cond: bool) -> float:
        self.runs[key] = self.runs.get(key, 0.0) + self.dt if cond else 0.0
        self.best_runs[key] = max(self.best_runs.get(key, 0.0), self.runs[key])
        return self.runs[key]

    def count(self, key: str, cond: bool = True) -> int:
        self.counts[key] = self.counts.get(key, 0) + (1 if cond else 0)
        return self.counts[key]

    def set(self, key: str, value) -> None:
        self.values[key] = value

    def trace(self, key: str, value, every: float = 0.0) -> None:
        tr = self.traces.setdefault(key, [])
        if not tr or every <= 0 or self.t - tr[-1][0] >= every - 1e-9:
            tr.append((round(self.t, 3), value))

    def when(self, key: str) -> float | None:
        return self.evers.get(key)

    def summary(self) -> dict:
        return {"t": self.t, "peaks": dict(self.peaks), "lows": dict(self.lows), "evers": dict(self.evers),
                "always": dict(self.nevers), "best_runs": dict(self.best_runs), "values": dict(self.values),
                "counts": dict(self.counts)}


# -- runtime: hooks, callbacks and joint switches ---------------------------------------------------------------

_STEP: list[Callable] = []
_RESET: list[Callable] = []
_CTX: dict = {"env": None, "emb": None, "t": 0.0, "n": 0}
_LIVE: dict[tuple[str, str], dict] = {}  # (linkage, joint) -> current drive settings


class Tick:
    """What a callback gets: ``env``, ``t`` (s since reset), ``n`` (control step), ``states`` (every task object,
    read once on first use) and ``tools`` (both grippers)."""

    def __init__(self, environment, t: float, n: int, emb):
        self.env, self.t, self.n, self.emb = environment, t, n, emb
        self._states = self._tools = None

    @property
    def states(self) -> dict:
        if self._states is None:
            from yam_arena.scene import object_states

            self._states = object_states(self.env, self.emb.task.names)
        return self._states

    @property
    def tools(self) -> dict[str, Tool]:
        if self._tools is None:
            self._tools = grippers(self.env)
        return self._tools


def on_step(fn):
    """Register ``fn(tick)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(tick)`` to run after every reset (objects settled, History cleared)."""
    _RESET.append(fn)
    return fn


def env():
    return _CTX["env"]


def _joint_handle(obj: str, joint: str):
    asset = env().unwrapped.scene[obj]
    return asset, list(asset.joint_names).index(joint), _SPECS[obj].joint(joint)


def _tensor(asset, values):
    import torch

    return torch.tensor([list(values)], dtype=torch.float32, device=asset.device)


def _write_drive(obj: str, joint: str, *, stiffness=None, damping=None, target=None) -> None:
    """Write a joint's drive (target in the joint's own units, measured like LinkState.q)."""
    asset, i, j = _joint_handle(obj, joint)
    live = _LIVE.setdefault((obj, joint), {"stiffness": j.stiffness, "damping": j.damping, "target": j.rest,
                                           "braked": False})
    if stiffness is not None:
        asset.write_joint_stiffness_to_sim_index(stiffness=_tensor(asset, [stiffness]), joint_ids=[i])
        live["stiffness"] = float(stiffness)
    if damping is not None:
        asset.write_joint_damping_to_sim_index(damping=_tensor(asset, [damping]), joint_ids=[i])
        live["damping"] = float(damping)
    if target is not None:
        value = _tensor(asset, [target - j.rest])  # the USD joint frame puts the spring rest at 0
        cmd = getattr(getattr(asset, "actuators", None), "target_command", None)
        if cmd is not None:
            cmd.set_position_index(value=value, joint_ids=[i])
        else:
            asset.set_joint_position_target(value, joint_ids=[i])
        live["target"] = float(target)


def joint_position(obj: str, joint: str) -> float:
    asset, i, j = _joint_handle(obj, joint)
    return float(_np(asset.data.joint_pos)[0, i]) + j.rest


def brake(obj: str, joint: str, on: bool = True) -> None:
    """Engage (hold where it is now, with the joint's Brake stiffness) or release (back to its spring)."""
    j = _SPECS[obj].joint(joint)
    b = j.brake or Brake()
    live = _LIVE.get((obj, joint), {})
    if on:
        _write_drive(obj, joint, stiffness=b.stiffness, damping=b.damping, target=joint_position(obj, joint))
    else:
        _write_drive(obj, joint, stiffness=j.stiffness, damping=j.damping, target=live.get("rest", j.rest))
    _LIVE[(obj, joint)]["braked"] = bool(on)


def braked(obj: str, joint: str) -> bool:
    """Whether a brake is on (internal state: fine for scoring, not for demos)."""
    return bool(_LIVE.get((obj, joint), {}).get("braked", False))


def set_rest(obj: str, joint: str, value: float) -> None:
    """Move a joint's spring rest (rad or m): detents, over-centre flips, crease memory that sets."""
    _LIVE.setdefault((obj, joint), {})["rest"] = float(value)
    if not braked(obj, joint):
        _write_drive(obj, joint, target=value)


def set_spring(obj: str, joint: str, stiffness: float | None = None, damping: float | None = None) -> None:
    _write_drive(obj, joint, stiffness=stiffness, damping=damping)


def set_limits(obj: str, joint: str, lower: float, upper: float) -> None:
    """Change a joint's limits (rad or m) at run time: a ratchet pawl, a stop that drops in."""
    asset, i, j = _joint_handle(obj, joint)
    import torch

    lim = torch.tensor([[[lower - j.rest, upper - j.rest]]], dtype=torch.float32, device=asset.device)
    asset.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=[i], warn_limit_violation=False)
    _LIVE.setdefault((obj, joint), {})["limits"] = (float(lower), float(upper))


def set_friction(obj: str, joint: str, static: float, dynamic: float | None = None,
                 viscous: float | None = None) -> None:
    """Joint friction efforts (N·m or N) at run time: a friction hinge that holds where it is put."""
    asset, i, _ = _joint_handle(obj, joint)
    asset.write_joint_friction_coefficient_to_sim_index(
        joint_friction_coeff=_tensor(asset, [static]),
        joint_dynamic_friction_coeff=None if dynamic is None else _tensor(asset, [dynamic]),
        joint_viscous_friction_coeff=None if viscous is None else _tensor(asset, [viscous]),
        joint_ids=[i])


def detent(obj: str, joint: str, stops: Iterable[float], *, stiffness: float | None = None) -> Callable:
    """Every control step, pull the joint's spring toward the nearest of ``stops`` (rad or m). Two stops make an
    over-centre (toggle) mechanism: the spring flips when the joint passes their midpoint."""
    stops = sorted(float(s) for s in stops)

    def fn(tick):
        q = tick.states[obj].q[joint]
        best = min(stops, key=lambda s: abs(s - q))
        if _LIVE.get((obj, joint), {}).get("rest") != best:
            if stiffness is not None:
                set_spring(obj, joint, stiffness=stiffness)
            set_rest(obj, joint, best)

    return on_step(fn)


def ratchet(obj: str, joint: str, *, direction: int = 1, backlash: float = 0.0) -> Callable:
    """Every control step, move the joint's limit so it can only advance in ``direction`` (+1 or -1)."""

    def fn(tick):
        j = _SPECS[obj].joint(joint)
        lo0 = -1e3 if j.lower is None else j.lower
        hi0 = 1e3 if j.upper is None else j.upper
        q = tick.states[obj].q[joint]
        live = _LIVE.setdefault((obj, joint), {})
        if direction > 0:
            best = max(live.get("ratchet", lo0), q - backlash)
            if best > live.get("ratchet", lo0) + 1e-4:
                set_limits(obj, joint, best, hi0)
                live["ratchet"] = best
        else:
            best = min(live.get("ratchet", hi0), q + backlash)
            if best < live.get("ratchet", hi0) - 1e-4:
                set_limits(obj, joint, lo0, best)
                live["ratchet"] = best

    return on_step(fn)


def _apply_defaults(environment) -> None:
    """Before every reset: joints to their initial positions, at rest, drives and limits as declared."""
    _CTX["env"] = environment
    _LIVE.clear()
    import torch

    scene = environment.unwrapped.scene
    for name, spec in _SPECS.items():
        if not spec.moving_joints:
            continue
        try:
            asset = scene[name]
        except KeyError:  # built but not in this task's OBJECTS
            continue
        jn = list(asset.joint_names)
        js = list(spec.moving_joints)
        ids = [jn.index(j.name) for j in js]
        dev = asset.device
        asset.write_joint_position_to_sim_index(
            position=torch.tensor([[j.init - j.rest for j in js]], dtype=torch.float32, device=dev), joint_ids=ids)
        asset.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, len(ids)), device=dev), joint_ids=ids)
        lim = torch.tensor([[[-1e3 if j.lower is None else j.lower - j.rest,
                              1e3 if j.upper is None else j.upper - j.rest] for j in js]],
                           dtype=torch.float32, device=dev)
        if any(j.lower is not None or j.upper is not None for j in js):
            asset.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)
        for j in js:
            if j.brake is not None and j.brake.engaged:
                _write_drive(name, j.name, stiffness=j.brake.stiffness, damping=j.brake.damping, target=j.init)
                _LIVE[(name, j.name)]["braked"] = True
            else:
                _write_drive(name, j.name, stiffness=j.stiffness, damping=j.damping, target=j.rest)


# -- Isaac Sim side -----------------------------------------------------------------------------------------------


def _set_xform(xf, pos, quat_xyzw=None, scale=None) -> None:
    from pxr import Gf, UsdGeom

    xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(float(v) for v in pos)))
    x, y, z, w = quat_xyzw if quat_xyzw is not None else (0.0, 0.0, 0.0, 1.0)
    xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(w), float(x), float(y), float(z)))
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in scale)))


def _spawn_linkage(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the linkage under ``prim_path`` (pxr only). ``cfg.linkage_name`` keys the Linkage spec."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Linkage = _SPECS[cfg.linkage_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    _set_xform(root, translation or (0.0, 0.0, 0.0), orientation)  # Isaac Lab 3.0 orientation: (x, y, z, w)

    looks: dict = {}

    def look(color) -> str:
        key = tuple(round(float(c), 4) for c in color)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(key), roughness=0.55)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    moving = bool(spec.moving_joints)
    for lk in spec.links:
        lp = f"{prim_path}/{lk.name}"
        xf = UsdGeom.Xform.Define(stage, lp)
        _set_xform(xf, lk.pos, lk.quat)
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        if spec.fixed and not moving:
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(lk.mass)
        if lk.com is not None:
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*lk.com))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(lk.max_depenetration)
        rb.CreateLinearDampingAttr(lk.linear_damping)
        rb.CreateAngularDampingAttr(lk.angular_damping)
        rb.CreateSolverPositionIterationCountAttr(spec.pos_iters)
        rb.CreateSolverVelocityIterationCountAttr(spec.vel_iters)
        rb.CreateSleepThresholdAttr(spec.sleep_threshold)
        rb.CreateStabilizationThresholdAttr(spec.stabilization_threshold)
        phys = f"{prim_path}/Looks/{lk.name}_surface"
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=lk.friction[0], dynamic_friction=lk.friction[1],
                                             restitution=lk.restitution)
        mat.func(phys, mat)
        for k, s in enumerate(lk.shapes):
            sp = f"{lp}/{s.kind}{k}"
            if s.kind == "box":
                g = UsdGeom.Cube.Define(stage, sp)
                g.CreateSizeAttr(1.0)
                _set_xform(g, s.pos, s.quat, scale=s.dims)
            elif s.kind == "cylinder":
                g = UsdGeom.Cylinder.Define(stage, sp)
                g.CreateAxisAttr(s.axis)
                g.CreateRadiusAttr(s.dims[0])
                g.CreateHeightAttr(s.dims[1])
                _set_xform(g, s.pos, s.quat)
            elif s.kind == "capsule":
                g = UsdGeom.Capsule.Define(stage, sp)
                g.CreateAxisAttr(s.axis)
                g.CreateRadiusAttr(s.dims[0])
                g.CreateHeightAttr(s.dims[1])
                _set_xform(g, s.pos, s.quat)
            else:
                raise ValueError(f"{spec.name}.{lk.name}: unknown shape {s.kind!r}")
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            pc = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
            pc.CreateContactOffsetAttr(s.contact_offset if s.contact_offset is not None else lk.contact_offset)
            pc.CreateRestOffsetAttr(s.rest_offset if s.rest_offset is not None else lk.rest_offset)
            bind_physics_material(sp, phys, stage=stage)
            bind_visual_material(sp, look(s.color or lk.color), stage=stage)

    def local(link: str, point, rot) -> tuple:
        m = np.linalg.inv(spec.link(link).design) @ _pose(point, rot)
        return Gf.Vec3f(*(float(v) for v in m[:3, 3])), matrix_to_quat(m[:3, :3])

    def gquat(q) -> Gf.Quatf:
        return Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2]))

    for j in spec.joints:
        jp = f"{prim_path}/{j.name}"
        axis = np.asarray(j.axis, dtype=float) / np.linalg.norm(j.axis)
        a = x_to(axis)
        anchor = np.asarray(j.anchor, dtype=float)
        if j.kind == "revolute":
            uj = UsdPhysics.RevoluteJoint.Define(stage, jp)
            uj.CreateAxisAttr("X")
            # spring rest baked into the parent frame: joint 0 (the drive's target) is the rest angle
            p0, q0 = local(j.parent, anchor, a @ axis_rotation((1, 0, 0), j.rest))
        elif j.kind == "prismatic":
            uj = UsdPhysics.PrismaticJoint.Define(stage, jp)
            uj.CreateAxisAttr("X")
            p0, q0 = local(j.parent, anchor + j.rest * axis, a)
        else:
            uj = UsdPhysics.FixedJoint.Define(stage, jp)
            p0, q0 = local(j.parent, anchor, a)
        p1, q1 = local(j.child, anchor, a)
        uj.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{j.parent}")])
        uj.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{j.child}")])
        uj.CreateLocalPos0Attr(p0)
        uj.CreateLocalRot0Attr(gquat(q0))
        uj.CreateLocalPos1Attr(p1)
        uj.CreateLocalRot1Attr(gquat(q1))
        if j.loop:
            uj.CreateExcludeFromArticulationAttr(True)
            if j.break_force is not None:
                uj.CreateBreakForceAttr(float(j.break_force))
            if j.break_torque is not None:
                uj.CreateBreakTorqueAttr(float(j.break_torque))
        if j.kind == "fixed":
            continue
        angular = j.kind == "revolute"
        scale = 180.0 / math.pi if angular else 1.0  # USD angles are degrees
        if j.lower is not None:
            uj.CreateLowerLimitAttr(float((j.lower - j.rest) * scale))
        if j.upper is not None:
            uj.CreateUpperLimitAttr(float((j.upper - j.rest) * scale))
        drive = UsdPhysics.DriveAPI.Apply(uj.GetPrim(), "angular" if angular else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(j.stiffness / scale))  # USD angular gains are per degree
        drive.CreateDampingAttr(float(j.damping / scale))
        drive.CreateTargetPositionAttr(0.0)
        PhysxSchema.PhysxJointAPI.Apply(uj.GetPrim()).CreateArmatureAttr(float(j.armature))

    root_link = stage.GetPrimAtPath(f"{prim_path}/{spec.links[0].name}")
    if moving:
        if spec.fixed:
            weld = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/world_weld")
            weld.CreateBody1Rel().SetTargets([root_link.GetPath()])
            world = UsdGeom.Xformable(root_link).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t, q = world.ExtractTranslation(), world.ExtractRotationQuat()
            weld.CreateLocalPos0Attr(Gf.Vec3f(*t))
            weld.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), *q.GetImaginary()))
            weld.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            weld.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            art_prim = root.GetPrim()  # fixed base: root API on an ancestor of the world joint and the links
        else:
            art_prim = root_link  # floating base: the first link is the articulation's root link
        UsdPhysics.ArticulationRootAPI.Apply(art_prim)
        art = PhysxSchema.PhysxArticulationAPI.Apply(art_prim)
        art.CreateEnabledSelfCollisionsAttr(spec.self_collision)
        art.CreateSolverPositionIterationCountAttr(spec.pos_iters)
        art.CreateSolverVelocityIterationCountAttr(spec.vel_iters)
        art.CreateSleepThresholdAttr(spec.sleep_threshold)
        art.CreateStabilizationThresholdAttr(spec.stabilization_threshold)
    return root.GetPrim()


def _arena_object(spec: Linkage):
    """The linkage as an Arena object: an articulation, or a rigid object when nothing moves."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg, RigidObjectCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class LinkageSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_linkage)
        linkage_name: str = ""

    js = spec.moving_joints

    class LinkageObject(Object):
        def _generate_articulation_cfg(self):
            per = lambda f: {j.name: float(f(j)) for j in js}  # noqa: E731
            cfg = ArticulationCfg(
                prim_path=self.prim_path,
                spawn=self.spawner_cfg,
                init_state=ArticulationCfg.InitialStateCfg(joint_pos=per(lambda j: j.init - j.rest),
                                                           joint_vel={".*": 0.0}),
                actuators={
                    "linkkit": ImplicitActuatorCfg(
                        joint_names_expr=[j.name for j in js],
                        stiffness=per(lambda j: j.stiffness),
                        damping=per(lambda j: j.damping),
                        armature=per(lambda j: j.armature),
                        friction=per(lambda j: j.friction),
                        dynamic_friction=per(lambda j: j.dynamic_friction),
                        viscous_friction=per(lambda j: j.viscous_friction),
                    )
                },
            )
            return self._add_initial_pose_to_cfg(cfg)

        def _generate_rigid_cfg(self):
            return self._add_initial_pose_to_cfg(RigidObjectCfg(prim_path=self.prim_path, spawn=self.spawner_cfg))

    return LinkageObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION if js else ObjectType.RIGID,
        spawner_cfg=LinkageSpawnerCfg(linkage_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


# -- hooking yam_arena (no fork): scene objects, states, reset and step -------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_linkkit_patched", False):
        return
    base_object, base_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Linkage):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return base_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = base_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _linkage_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object, object_states
    scene._linkkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_linkkit_patched", False):
        return
    base_ensure, base_reset, base_step = cls._ensure_sim, cls.reset, cls._task_step

    def _ensure_sim(self, *a, **k):
        out = base_ensure(self, *a, **k)
        if self._env is not None and _SPECS:
            _apply_defaults(self._env)  # runs inside reset(), before env.reset() and the settle steps
        return out

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _CTX.update(env=self._env, emb=self, t=0.0, n=0)
        for h in _HISTORIES:
            h.reset()
        tick = Tick(self._env, 0.0, 0, self)
        for fn in _RESET:
            fn(tick)
        return obs

    def _task_step(self, *a, **k):
        _CTX.update(env=self._env, emb=self, t=float(self._t), n=_CTX["n"] + 1)
        for h in _HISTORIES:
            h._tick(float(self._t))
        tick = Tick(self._env, float(self._t), _CTX["n"], self)
        for fn in _STEP:
            fn(tick)
        return base_step(self, *a, **k)

    cls._ensure_sim, cls.reset, cls._task_step = _ensure_sim, reset, _task_step
    cls._linkkit_patched = True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _Hook(importlib.abc.MetaPathFinder):
    """Patches the yam_arena modules above right after they are first imported."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _TARGETS:
            return None
        sys.meta_path.remove(self)
        try:
            found = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if found is None or found.loader is None:
            return None
        run = found.loader.exec_module

        def exec_module(module):
            run(module)
            _TARGETS[fullname](module)

        found.loader.exec_module = exec_module
        return found


def install() -> None:
    """Hook yam_arena now (modules already imported) or on import. Idempotent."""
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Hook) for f in sys.meta_path):
        sys.meta_path.insert(0, _Hook())


install()
