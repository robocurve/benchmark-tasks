"""mechkit: multi-box parts, hinges, sliders and breakable bonds for yam_arena task files.

Drop this file next to a task file and import it there; it adds mechanisms to the stock yam_arena runtime by
patching it in memory at import time (nothing on disk changes), so the standard command still runs the task:

    inspect-robots run --embodiment yam_arena -E task=tasks/<task>/<task>.py ...

Pure Python at import (numpy and yam_arena.task_file): a task file that uses it still loads, and runs its offline
self-test, without Isaac Sim. Isaac Lab and pxr are imported only once the simulator builds the scene.

Model. Every part is one ordinary yam_arena rigid body (free, with a mass) or static collider (fixed, no mass),
built from several boxes. Joints are plain USD Physics / PhysX joints authored between those bodies after the
scene is built (no articulations): RevoluteJoint and PrismaticJoint with limits and a DriveAPI spring/damper, and
FixedJoint with physics:breakForce / physics:breakTorque for bonds. A joint's two bodies never collide with each
other. Every readout is computed from the bodies' poses in the ``objects`` dict that score() receives, so it is
geometry a reviewer could measure on a photo, and the offline self-test can build states by hand.

Public API (angles in the API are degrees unless the name says rad; lengths m; forces N):

    import sys; from pathlib import Path; sys.path.insert(0, str(Path(__file__).resolve().parent))
    import mechkit as mk

    kit = mk.Kit()
    kit.part(name, pos, boxes, mass=None, color=GREY, yaw=0.0, quat=None)      # boxes: [mk.box(size, at, ...)]
    kit.hinge(name, parent, child, anchor, axis, lower_deg, upper_deg, start_deg=0, stiffness=0, rest_deg=0,
              damping=0, friction=0)                         # N*m/rad, N*m*s/rad, friction N*m
    kit.slider(name, parent, child, axis, lower, upper, start=0, stiffness=0, rest=0, damping=0, friction=0)
    kit.bond(name, parent, child, break_force=inf, break_torque=inf, anchor=None, slip_mm=2, slip_deg=2,
             twist_axis=None, twist_release_deg=None, twist_stiffness=1.0)   # N, N*m about the child, N*m/rad
    kit.on_step(fn)                                          # fn(objects) every control step, sim only
    kit.release(name)                                        # switch a bond off now (sim; re-glued at reset)
    OBJECTS = [*kit.objects(), ...]

    kit.q(objects, joint) / kit.q_deg(objects, hinge) / kit.joints(objects)   # joint values from body poses
    kit.broken(objects, bond)                                # moved apart > slip (latched per episode)
    kit.pose(objects, part) -> (pos, quat_xyzw)              # the part origin's world pose
    kit.point(objects, part, local_xyz)                      # a point fixed on the part, in world
    kit.fake_states(q=..., moved=..., broken=..., extra=...) # ObjectStates for offline self-tests
    mk.tcps(), mk.released(state, margin), mk.MEMORY, mk.new_episode()

parent=None (or a fixed part) means the joint holds the child to the world. Joint values are 0 in the reference
pose the parts are declared in; ``start``/``start_deg`` poses the child at spawn. Hinge angle is the child's
right-hand rotation about +axis relative to the parent; slider travel is along +axis.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass, field

import numpy as np

from yam_arena.task_file import GREY, Box, ObjectState

__version__ = "0.2"
INF = float("inf")
STICK_RAD = math.radians(0.5)  # friction model: elastic "stick" travel before a joint slips (hinge)
STICK_M = 0.0005  # (slider)
_BIG = 3.0e38  # PhysX "unbreakable"
MEMORY: dict = {}  # per-episode scratch for tasks; cleared at every reset (and by new_episode())
_EP: dict = {"env": None, "emb": None, "step": 0}


# -- small rigid-transform maths (xyzw quaternions, 4x4 matrices) --------------------------------------------------


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError(f"axis {v!r} has zero length")
    return v / n


def quat_to_mat(q) -> np.ndarray:
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def mat_to_quat(m) -> np.ndarray:
    m = np.asarray(m, dtype=float)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[3] >= 0 else -1)


def axis_angle(axis, angle: float) -> np.ndarray:
    """3x3 rotation of ``angle`` rad about ``axis``."""
    x, y, z = _unit(axis)
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def yaw_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _T(rot=None, pos=(0.0, 0.0, 0.0)) -> np.ndarray:
    t = np.eye(4)
    if rot is not None:
        t[:3, :3] = rot
    t[:3, 3] = pos
    return t


def _inv(t) -> np.ndarray:
    r, p = t[:3, :3], t[:3, 3]
    return _T(r.T, -r.T @ p)


def _rot_angle(r) -> float:
    return float(math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2))))


def _frame_with_x(axis) -> np.ndarray:
    """A rotation whose x column is ``axis`` (the joint frame: USD joints move along/about their local X)."""
    x = _unit(axis)
    helper = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
    y = _unit(np.cross(helper, x))
    return np.column_stack([x, y, np.cross(x, y)])


# -- declarations -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class BoxSpec:
    size: tuple
    at: tuple = (0.0, 0.0, 0.0)
    color: tuple | None = None
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


def box(size, at=(0.0, 0.0, 0.0), color=None, yaw: float = 0.0, quat=None) -> BoxSpec:
    """One box of a part: edge lengths ``size`` (m), centre ``at`` in the part frame, turned ``yaw`` (rad) about
    the part's z (or by ``quat`` xyzw); ``color`` overrides the part's."""
    size = tuple(float(s) for s in size)
    if len(size) != 3 or min(size) <= 0:
        raise ValueError(f"box size must be 3 positive lengths, got {size!r}")
    q = tuple(float(c) for c in (quat if quat is not None else yaw_quat(yaw)))
    return BoxSpec(size, tuple(float(c) for c in at), None if color is None else tuple(color), q)


@dataclass(frozen=True)
class MechPart(Box):
    """A kit part as a task-file object. ``pos``/``size`` are its world bounding box at spawn (so offline tools
    that treat it as a Box see roughly the right footprint); the kit uses ``origin``/``origin_quat``/``boxes``."""

    boxes: tuple = ()
    origin: tuple = (0.0, 0.0, 0.0)  # spawn pose of the part frame (world)
    origin_quat: tuple = (0.0, 0.0, 0.0, 1.0)
    kit_id: int = 0


@dataclass
class _Part:
    name: str
    ref: np.ndarray  # 4x4 reference pose (all joints at 0)
    boxes: tuple
    mass: float | None
    color: tuple
    lo: np.ndarray = field(default=None)  # local AABB of the geometry (part frame)
    hi: np.ndarray = field(default=None)
    parent_joint: str | None = None

    @property
    def free(self) -> bool:
        return self.mass is not None

    @property
    def centre(self) -> np.ndarray:
        return (self.lo + self.hi) / 2


@dataclass
class _Joint:
    name: str
    kind: str  # "hinge" | "slider" | "bond"
    parent: str | None
    child: str
    anchor: np.ndarray  # world, reference pose
    axis: np.ndarray  # world, reference pose (bond: unused)
    lower: float = -INF  # rad / m
    upper: float = INF
    start: float = 0.0
    stiffness: float = 0.0  # SI per rad / per m
    rest: float = 0.0
    damping: float = 0.0
    friction: float = 0.0
    break_force: float = INF
    break_torque: float = INF
    slip_m: float = 0.002
    slip_rad: float = math.radians(2.0)
    twist_axis: np.ndarray | None = None  # twist bond: turns on a spring about this axis, lets go past the limit
    twist_limit: float = INF  # rad
    twist_stiffness: float = 0.0  # N*m/rad


_KITS: list = []


class Kit:
    """The mechanisms of one task file. Declare parts first, then joints, then put ``kit.objects()`` in OBJECTS."""

    def __init__(self) -> None:
        self.parts: dict[str, _Part] = {}
        self.joints_: dict[str, _Joint] = {}
        self._steps: list = []
        self._released: set = set()
        self.id = len(_KITS)
        _KITS.append(self)

    # -- declaring --------------------------------------------------------------------------------------------

    def part(self, name, pos, boxes, mass=None, color=GREY, yaw: float = 0.0, quat=None) -> str:
        """A rigid body made of ``boxes`` (see :func:`box`), its frame at world ``pos`` in the reference pose.
        ``mass`` None: fixed in place (static); otherwise a free body of that mass (kg)."""
        if not str(name).isidentifier():
            raise ValueError(f"part name {name!r} must be a Python identifier")
        if name in self.parts:
            raise ValueError(f"part {name!r} declared twice")
        boxes = tuple(b if isinstance(b, BoxSpec) else box(*b) for b in boxes)
        if not boxes:
            raise ValueError(f"part {name!r} needs at least one box")
        if mass is not None and not mass > 0:
            raise ValueError(f"part {name!r}: mass must be positive or None")
        q = quat if quat is not None else yaw_quat(yaw)
        p = _Part(name, _T(quat_to_mat(q), np.asarray(pos, dtype=float)), boxes, mass, tuple(color))
        corners = []
        for b in boxes:
            r = quat_to_mat(b.quat)
            for sx in (-0.5, 0.5):
                for sy in (-0.5, 0.5):
                    for sz in (-0.5, 0.5):
                        corners.append(np.asarray(b.at) + r @ (np.array([sx, sy, sz]) * b.size))
        corners = np.array(corners)
        p.lo, p.hi = corners.min(axis=0), corners.max(axis=0)
        self.parts[name] = p
        return name

    def _joint(self, j: _Joint) -> str:
        if j.name in self.joints_:
            raise ValueError(f"joint {j.name!r} declared twice")
        if j.child not in self.parts or (j.parent is not None and j.parent not in self.parts):
            raise ValueError(f"joint {j.name!r}: declare parts {j.parent!r} and {j.child!r} first")
        child = self.parts[j.child]
        if not child.free:
            raise ValueError(f"joint {j.name!r}: the child {j.child!r} must be a free part (give it a mass)")
        if child.parent_joint is not None:
            raise ValueError(f"part {j.child!r} already has a parent joint ({child.parent_joint!r})")
        if j.parent is not None and self._ancestors(j.parent) & {j.child}:
            raise ValueError(f"joint {j.name!r} closes a loop")
        child.parent_joint = j.name
        self.joints_[j.name] = j
        return j.name

    def _ancestors(self, part: str) -> set:
        out = set()
        while part is not None:
            out.add(part)
            pj = self.parts[part].parent_joint
            part = self.joints_[pj].parent if pj else None
        return out

    def hinge(self, name, parent, child, anchor, axis, lower_deg, upper_deg, start_deg=0.0, stiffness=0.0,
              rest_deg=0.0, damping=0.0, friction=0.0) -> str:
        """Revolute joint through world ``anchor`` along world ``axis`` (reference pose). Spring torque
        -stiffness*(q-rest) - damping*dq (N*m/rad, N*m*s/rad). ``friction`` (N*m) is Coulomb joint friction:
        the joint does not move until the other torques on it exceed ``friction``, then slides against it, and it
        stays where it is left (e.g. a creased carton flap). Note: PhysX caps viscous ``damping`` on light bodies,
        so damping alone does not hold a flap against gravity; use ``friction``."""
        lo, hi, s = math.radians(lower_deg), math.radians(upper_deg), math.radians(start_deg)
        if not lo <= s <= hi:
            raise ValueError(f"hinge {name!r}: start {start_deg} outside [{lower_deg}, {upper_deg}]")
        return self._joint(_Joint(name, "hinge", parent, child, np.asarray(anchor, float), _unit(axis), lo, hi, s,
                                  float(stiffness), math.radians(rest_deg), float(damping), float(friction)))

    def slider(self, name, parent, child, axis, lower, upper, start=0.0, stiffness=0.0, rest=0.0,
               damping=0.0, friction=0.0) -> str:
        """Prismatic joint along world ``axis``. Spring force -stiffness*(d-rest) - damping*dd (N/m, N*s/m); a
        ``rest`` beyond a stop preloads the child against that stop. ``friction`` (N): Coulomb, as for hinges."""
        if not lower <= start <= upper:
            raise ValueError(f"slider {name!r}: start {start} outside [{lower}, {upper}]")
        anchor = self.parts[child].ref[:3, 3] if child in self.parts else np.zeros(3)
        return self._joint(_Joint(name, "slider", parent, child, np.array(anchor, float), _unit(axis),
                                  float(lower), float(upper), float(start), float(stiffness), float(rest),
                                  float(damping), float(friction)))

    def bond(self, name, parent, child, break_force=INF, break_torque=INF, anchor=None, slip_mm=2.0,
             slip_deg=2.0, twist_axis=None, twist_release_deg=None, twist_stiffness=1.0) -> str:
        """Rigid glue that breaks when its constraint force exceeds ``break_force`` (N) or its constraint torque,
        taken about the CHILD's origin (PhysX measures it about body0, which the kit makes the child), exceeds
        ``break_torque`` (N*m). ``anchor`` (world point, default the child's origin) is where the glue sits.
        ``broken()`` reports it once the pair has moved apart by more than slip_mm / slip_deg.

        Twist bond (``twist_axis`` and ``twist_release_deg`` given): the glue yields in twist like a sheared
        paste: it turns about ``twist_axis`` through ``anchor`` on a spring of ``twist_stiffness`` (N*m/rad) and
        lets go for good once the twist passes ``twist_release_deg``, i.e. at a twist torque of about
        twist_stiffness * radians(twist_release_deg). Pull (break_force) and bending (break_torque) still apply."""
        a = self.parts[child].ref[:3, 3] if anchor is None and child in self.parts else anchor
        if break_torque < INF and parent is not None and parent in self.parts and self.parts[parent].free:
            print(f"[mechkit] warning: bond {name!r}: break_torque between two free parts was unreliable in sim "
                  "(broke under ~60% of its threshold); use a twist bond or break_force", file=sys.stderr)
        j = _Joint(name, "bond", parent, child, np.asarray(a, float), np.array([1.0, 0, 0]),
                   break_force=float(break_force), break_torque=float(break_torque),
                   slip_m=slip_mm / 1000.0, slip_rad=math.radians(slip_deg))
        if twist_axis is not None:
            if twist_release_deg is None or not twist_release_deg > 0 or not twist_stiffness > 0:
                raise ValueError(f"bond {name!r}: a twist bond needs twist_release_deg > 0 and twist_stiffness > 0")
            j.twist_axis, j.axis = _unit(twist_axis), _unit(twist_axis)
            j.twist_limit, j.twist_stiffness = math.radians(twist_release_deg), float(twist_stiffness)
        return self._joint(j)

    def on_step(self, fn) -> None:
        """Call ``fn(objects)`` after every control step in sim, before success()/score() (e.g. a release rule)."""
        self._steps.append(fn)

    # -- kinematics -------------------------------------------------------------------------------------------

    def _motion(self, j: _Joint, value: float) -> np.ndarray:
        """The joint's motion at ``value`` as a world transform about its reference anchor/axis."""
        if j.kind == "hinge":
            r = axis_angle(j.axis, value)
            return _T(r, j.anchor - r @ j.anchor)
        if j.kind == "slider":
            return _T(None, j.axis * value)
        if j.twist_axis is not None and value:
            r = axis_angle(j.twist_axis, value)
            return _T(r, j.anchor - r @ j.anchor)
        return np.eye(4)

    def _fk(self, q: dict | None = None, moved: dict | None = None) -> dict[str, np.ndarray]:
        q, moved = q or {}, moved or {}
        out: dict[str, np.ndarray] = {}

        def solve(name):
            if name in out:
                return out[name]
            p = self.parts[name]
            if name in moved:
                pos, quat = moved[name]
                out[name] = _T(quat_to_mat(quat), pos)
            elif p.parent_joint is None:
                out[name] = p.ref.copy()
            else:
                j = self.joints_[p.parent_joint]
                mp = np.eye(4) if j.parent is None else solve(j.parent) @ _inv(self.parts[j.parent].ref)
                out[name] = mp @ self._motion(j, q.get(j.name, j.start)) @ p.ref
            return out[name]

        for n in self.parts:
            solve(n)
        return out

    def _spawn_poses(self) -> dict[str, np.ndarray]:
        return self._fk()

    def objects(self) -> list[MechPart]:
        """The parts as task-file objects (spawned at their joints' start values)."""
        out = []
        for name, t in self._spawn_poses().items():
            p = self.parts[name]
            lo, hi = _world_aabb(t, p.lo, p.hi)
            fl = lambda v: tuple(float(c) for c in v)  # plain floats: Isaac turns numpy float64 into doubles
            out.append(MechPart(name=name, pos=fl((lo + hi) / 2), color=fl(p.color), mass=p.mass,
                                size=fl(np.maximum(hi - lo, 1e-4)), boxes=p.boxes, origin=fl(t[:3, 3]),
                                origin_quat=fl(mat_to_quat(t[:3, :3])), kit_id=self.id))
        return out

    # -- readouts ---------------------------------------------------------------------------------------------

    def _pose_T(self, objects, part: str) -> np.ndarray:
        p = self.parts[part]
        st = objects[part]
        r = quat_to_mat(st.quat_xyzw)
        return _T(r, np.asarray(st.pos, float) - r @ p.centre)

    def pose(self, objects, part: str):
        """(origin position, quat xyzw) of ``part`` in world, from its ObjectState."""
        t = self._pose_T(objects, part)
        return t[:3, 3].copy(), mat_to_quat(t[:3, :3])

    def point(self, objects, part: str, local_xyz) -> np.ndarray:
        """World position of a point fixed in ``part``'s frame."""
        t = self._pose_T(objects, part)
        return t[:3, :3] @ np.asarray(local_xyz, float) + t[:3, 3]

    def _joint_motion(self, objects, j: _Joint) -> np.ndarray:
        """J = T_p_ref T_p^-1 T_c T_c_ref^-1: the joint's world-frame motion since the reference pose."""
        tc = self._pose_T(objects, j.child)
        if j.parent is None:
            rel = tc
        else:
            rel = self.parts[j.parent].ref @ _inv(self._pose_T(objects, j.parent)) @ tc
        return rel @ _inv(self.parts[j.child].ref)

    def q(self, objects, name: str) -> float:
        """Hinge angle (rad) or slider travel (m), from the two bodies' poses."""
        j = self.joints_[name]
        m = self._joint_motion(objects, j)
        if j.kind == "hinge" or j.twist_axis is not None:
            quat = mat_to_quat(m[:3, :3])
            return float(2 * math.atan2(float(np.dot(quat[:3], j.axis)), float(quat[3])))
        if j.kind == "slider":
            return float(np.dot(m[:3, 3], j.axis))
        raise ValueError(f"{name!r} is a plain bond; use broken() / slip()")

    def q_deg(self, objects, name: str) -> float:
        return math.degrees(self.q(objects, name))

    def joints(self, objects) -> dict[str, float]:
        """Every hinge (rad) and slider (m) value."""
        return {n: self.q(objects, n) for n, j in self.joints_.items() if j.kind != "bond"}

    def slip(self, objects, name: str) -> tuple[float, float]:
        """(m, rad) a bonded pair has moved apart, measured at the bond's anchor."""
        j = self.joints_[name]
        m = self._joint_motion(objects, j)
        r = m[:3, :3]
        if j.twist_axis is not None:  # the allowed twist does not count as slip
            r = r @ axis_angle(j.twist_axis, self.q(objects, name)).T
        return float(np.linalg.norm(m[:3, :3] @ j.anchor + m[:3, 3] - j.anchor)), _rot_angle(r)

    def broken(self, objects, name: str) -> bool:
        """The bond has let go: the pair moved apart > slip_mm or > slip_deg, now or earlier this episode (sim),
        or it was released."""
        key = ("mechkit_broken", self.id, name)
        if MEMORY.get(key):
            return True
        j = self.joints_[name]
        d, a = self.slip(objects, name)
        twisted = j.twist_axis is not None and abs(self.q(objects, name)) > j.twist_limit
        if d > j.slip_m or a > j.slip_rad or twisted or name in self._released:
            MEMORY[key] = True
            return True
        return False

    # -- offline states ---------------------------------------------------------------------------------------

    def fake_states(self, q: dict | None = None, moved: dict | None = None, broken=(), extra: dict | None = None,
                    deg: bool = False) -> dict[str, ObjectState]:
        """ObjectStates for every part posed by forward kinematics: ``q`` joint values (rad/m, or degrees for
        hinges with ``deg=True``; unlisted joints at start), ``moved`` {part: (origin pos, quat xyzw)} absolute
        overrides (children follow), ``broken`` bond names whose child is lifted 1 cm off its parent, ``extra``
        other ObjectStates to include."""
        q = dict(q or {})
        if deg:
            q = {k: (v if self.joints_[k].kind == "slider" else math.radians(v)) for k, v in q.items()}
        moved = dict(moved or {})
        for b in broken:
            j = self.joints_[b]
            t = self._fk(q, moved)[j.child]
            moved.setdefault(j.child, (t[:3, 3] + np.array([0, 0, 0.01]), mat_to_quat(t[:3, :3])))
        poses = self._fk(q, moved)
        out = {n: self._state(n, t) for n, t in poses.items()}
        out.update(extra or {})
        return out

    def _state(self, name: str, t: np.ndarray) -> ObjectState:
        p = self.parts[name]
        lo, hi = _world_aabb(t, p.lo, p.hi)
        z = np.zeros(3)
        return ObjectState(pos=t[:3, :3] @ p.centre + t[:3, 3], quat_xyzw=mat_to_quat(t[:3, :3]), lin_vel=z,
                           ang_vel=z.copy(), low=lo, high=hi)

    # -- sim side ---------------------------------------------------------------------------------------------

    def release(self, name: str) -> None:
        """Switch bond ``name`` off now (sim). It is glued again at the next episode reset."""
        self._released.add(name)
        _set_joint_enabled(self, name, False)


def _world_aabb(t, lo, hi):
    pts = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = pts @ t[:3, :3].T + t[:3, 3]
    return w.min(axis=0), w.max(axis=0)


def new_episode() -> None:
    """Forget per-episode latches (the sim calls this at every reset; self-tests call it between cases)."""
    MEMORY.clear()
    for k in _KITS:
        k._released.clear()


# -- grippers ------------------------------------------------------------------------------------------------------

TCP_PAST_GRIPPER = 0.134  # TCP: this far from the gripper link toward the finger-tip midpoint


def tcps() -> list:
    """World positions of both grippers' TCPs (left, right); [] offline."""
    env = _EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        g = pos[names.index("gripper")]
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]) / 2
        out.append(g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * TCP_PAST_GRIPPER)
    return out


def released(state, margin: float = 0.025) -> bool:
    """Neither TCP within ``margin`` of the object's bounding box (always True offline)."""
    lo, hi = np.asarray(state.low) - margin, np.asarray(state.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in tcps())


# -- yam_arena hooks (in memory) -----------------------------------------------------------------------------------


def _all_parts() -> dict:
    return {n: (k, p) for k in _KITS for n, p in k.parts.items()}


def _prim_paths(stage, env_path: str) -> dict:
    out = {}
    for name in _all_parts():
        path = f"{env_path}/{name}"
        if stage.GetPrimAtPath(path).IsValid():
            out[name] = path
    return out


def _author_joint(stage, kit: Kit, j: _Joint, paths: dict, env_T: np.ndarray, suffix: str = "") -> str | None:
    from pxr import Gf, Sdf, UsdPhysics

    if j.child not in paths or (j.parent is not None and j.parent not in paths):
        return None
    poses = kit._spawn_poses()
    parent_free = j.parent is not None and kit.parts[j.parent].free
    # Frame 0 lives on the parent body, or in the sim world when the parent is the world or a fixed part.
    tp = env_T @ poses[j.parent] if parent_free else np.eye(4)
    mp = poses[j.parent] @ _inv(kit.parts[j.parent].ref) if j.parent is not None else np.eye(4)  # parent motion
    if j.kind == "bond" and j.twist_axis is None:
        f_ref = _T(kit.parts[j.child].ref[:3, :3], j.anchor)
    else:
        f_ref = _T(_frame_with_x(j.axis), j.anchor)
    frame = env_T @ mp @ f_ref  # the joint frame on the parent, at spawn
    tc0 = env_T @ mp @ kit.parts[j.child].ref  # the child where it would be with this joint at 0
    f0 = _inv(tp) @ frame
    f1 = _inv(tc0) @ frame
    path = f"{paths[j.child].rsplit('/', 1)[0]}/mechkit_joints/{j.name}{suffix}"
    twist = j.kind == "bond" and j.twist_axis is not None
    if j.kind == "hinge" or twist:
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    elif j.kind == "slider":
        joint = UsdPhysics.PrismaticJoint.Define(stage, path)
    else:
        joint = UsdPhysics.FixedJoint.Define(stage, path)
    if j.kind == "bond":
        # PhysX measures a joint's constraint torque about its body0 (the world origin when body0 is empty), so a
        # bond puts the child as body0: break_torque is then taken about the child's origin.
        f0, f1 = f1, f0
        joint.CreateBody0Rel().SetTargets([Sdf.Path(paths[j.child])])
        if parent_free:
            joint.CreateBody1Rel().SetTargets([Sdf.Path(paths[j.parent])])
    else:
        if parent_free:
            joint.CreateBody0Rel().SetTargets([Sdf.Path(paths[j.parent])])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(paths[j.child])])

    def gq(r):
        x, y, z, w = mat_to_quat(r)
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in f0[:3, 3]]))
    joint.CreateLocalRot0Attr().Set(gq(f0[:3, :3]))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in f1[:3, 3]]))
    joint.CreateLocalRot1Attr().Set(gq(f1[:3, :3]))
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    prim = joint.GetPrim()
    if j.kind in ("hinge", "slider"):
        joint.CreateAxisAttr().Set("X")
        scale = 180.0 / math.pi if j.kind == "hinge" else 1.0  # USD angles are degrees
        joint.CreateLowerLimitAttr().Set(float(j.lower * scale))
        joint.CreateUpperLimitAttr().Set(float(j.upper * scale))
        if j.stiffness > 0 or j.damping > 0 or j.friction > 0:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular" if j.kind == "hinge" else "linear")
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(float((j.stiffness + _k_stick(j)) / scale))
            drive.CreateDampingAttr().Set(float(j.damping / scale))
            drive.CreateTargetPositionAttr().Set(float(_drive_target(j, j.start) * scale))
            drive.CreateTargetVelocityAttr().Set(0.0)
    else:
        joint.CreateBreakForceAttr().Set(float(min(j.break_force, _BIG)))
        joint.CreateBreakTorqueAttr().Set(float(min(j.break_torque, _BIG)))
        if twist:
            lim = math.degrees(j.twist_limit) + 10.0
            joint.CreateAxisAttr().Set("X")
            joint.CreateLowerLimitAttr().Set(-lim)
            joint.CreateUpperLimitAttr().Set(lim)
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(float(j.twist_stiffness * math.pi / 180.0))
            drive.CreateDampingAttr().Set(float(0.02 * j.twist_stiffness * math.pi / 180.0))
            drive.CreateTargetPositionAttr().Set(0.0)
    # A world-attached joint whose parent is a fixed part: keep the child from colliding with that part too.
    if j.parent is not None and not parent_free:
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths[j.child]))
        fp.CreateFilteredPairsRel().AddTarget(Sdf.Path(paths[j.parent]))
    return path


def _k_stick(j: _Joint) -> float:
    """Stiffness of the friction 'stick' spring: friction reached after STICK travel."""
    return j.friction / (STICK_RAD if j.kind == "hinge" else STICK_M) if j.friction > 0 else 0.0


def _drive_target(j: _Joint, stick: float) -> float:
    """One drive = the user spring (to rest) plus the friction stick spring (to the current stick point)."""
    ks, kf = j.stiffness, _k_stick(j)
    return (ks * j.rest + kf * stick) / (ks + kf) if ks + kf > 0 else j.rest


def _friction_step(kit: Kit, states: dict) -> None:
    """Elasto-plastic Coulomb friction: the stick point follows the joint once it has slipped past STICK."""
    from pxr import UsdPhysics

    for j in kit.joints_.values():
        if j.kind == "bond" or j.friction <= 0 or j.child not in states:
            continue
        key = (kit.id, j.name)
        path = _SIM["joint_paths"].get(key)
        if not path:
            continue
        q = kit.q(states, j.name)
        stick = _SIM["stick"].get(key, j.start)
        lim = STICK_RAD if j.kind == "hinge" else STICK_M
        new = min(max(stick, q - lim), q + lim)
        if new != stick:
            _SIM["stick"][key] = new
            scale = 180.0 / math.pi if j.kind == "hinge" else 1.0
            drive = UsdPhysics.DriveAPI(_SIM["stage"].GetPrimAtPath(path), "angular" if j.kind == "hinge" else "linear")
            drive.GetTargetPositionAttr().Set(float(_drive_target(j, new) * scale))


def _friction_reset() -> None:
    from pxr import UsdPhysics

    for (kid, name), path in _SIM["joint_paths"].items():
        j = _KITS[kid].joints_[name]
        if j.kind == "bond" or j.friction <= 0:
            continue
        _SIM["stick"][(kid, name)] = j.start
        scale = 180.0 / math.pi if j.kind == "hinge" else 1.0
        drive = UsdPhysics.DriveAPI(_SIM["stage"].GetPrimAtPath(path), "angular" if j.kind == "hinge" else "linear")
        drive.GetTargetPositionAttr().Set(float(_drive_target(j, j.start) * scale))


_SIM: dict = {"stage": None, "paths": {}, "env_T": np.eye(4), "joint_paths": {}, "gen": 0, "stick": {}}


def _author_all(stage, env_path: str) -> None:
    from pxr import UsdGeom

    paths = _prim_paths(stage, env_path)
    if not paths:
        return
    env_prim = stage.GetPrimAtPath(env_path)
    m = UsdGeom.Xformable(env_prim).ComputeLocalToWorldTransform(0)
    env_T = _T(None, (float(m[3][0]), float(m[3][1]), float(m[3][2])))
    _SIM.update(stage=stage, paths=paths, env_T=env_T)
    for k in _KITS:
        for j in k.joints_.values():
            p = _author_joint(stage, k, j, paths, env_T)
            if p:
                _SIM["joint_paths"][(k.id, j.name)] = p
    print(f"[mechkit {__version__}] parts {sorted(paths)}; joints {sorted(n for _, n in _SIM['joint_paths'])}",
          flush=True)


def _set_joint_enabled(kit: Kit, name: str, on: bool) -> None:
    path = _SIM["joint_paths"].get((kit.id, name))
    if not path or _SIM["stage"] is None:
        return
    from pxr import UsdPhysics

    UsdPhysics.Joint(_SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(bool(on))


def _reglue() -> None:
    """Re-create every bond that broke or was released last episode (a broken PhysX joint stays broken)."""
    stage = _SIM["stage"]
    if stage is None:
        return
    for k in _KITS:
        for j in k.joints_.values():
            key = (k.id, j.name)
            if j.kind != "bond" or key not in _SIM["joint_paths"]:
                continue
            if not (MEMORY.get(("mechkit_broken", k.id, j.name)) or j.name in k._released):
                continue
            stage.RemovePrim(_SIM["joint_paths"][key])
            _SIM["gen"] += 1
            p = _author_joint(stage, k, j, _SIM["paths"], _SIM["env_T"], suffix=f"_r{_SIM['gen']}")
            if p:
                _SIM["joint_paths"][key] = p


def _spawn_part_fn(mod):
    """A spawner (decorated like yam_arena's own) that builds a kit part from its boxes."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim import schemas
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone, create_prim, get_current_stage

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        spec = _PENDING[prim_path.rsplit("/", 1)[-1]]
        stage = get_current_stage()
        create_prim(prim_path, prim_type="Xform", translation=translation, orientation=orientation, stage=stage)
        coll = mod._contact_props() if spec.free else mod.PhysxCollisionPropertiesCfg()
        for i, b in enumerate(spec.boxes):
            color = b.color if b.color is not None else spec.color
            bcfg = sim_utils.CuboidCfg(
                size=tuple(b.size),
                collision_props=coll,
                physics_material=mod._grippy(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=mod.linear_rgb(color), roughness=0.4),
            )
            spawn_cuboid.__wrapped__(f"{prim_path}/box{i}", bcfg, translation=tuple(float(v) for v in b.at),
                                     orientation=tuple(float(v) for v in b.quat))
        if spec.free:
            schemas.define_rigid_body_properties(
                prim_path, mod.PhysxRigidBodyPropertiesCfg(solver_position_iteration_count=16,
                                                           solver_velocity_iteration_count=2), stage=stage)
            schemas.define_mass_properties(prim_path, sim_utils.MassPropertiesCfg(mass=spec.mass), stage=stage)
        return stage.GetPrimAtPath(prim_path)

    return spawn


_PENDING: dict = {}


def _patch_scene(mod) -> None:
    if getattr(mod, "_mechkit", False):
        return
    mod._mechkit = True
    original = mod.task_object
    spawn = _spawn_part_fn(mod)

    def task_object(spec):
        if not isinstance(spec, MechPart):
            return original(spec)
        _PENDING[spec.name] = spec
        cfg = mod.sim_utils.CuboidCfg(func=spawn, size=tuple(float(v) for v in spec.size))
        return mod.Object(
            name=spec.name,
            object_type=mod.ObjectType.RIGID if spec.free else mod.ObjectType.BASE,
            spawner_cfg=cfg,
            initial_pose=mod.Pose(position_xyz=tuple(float(v) for v in spec.origin),
                                  rotation_xyzw=tuple(float(v) for v in spec.origin_quat)),
        )

    mod.task_object = task_object

    from isaaclab.scene import interactive_scene as isc

    cls = isc.InteractiveScene
    if not getattr(cls, "_mechkit", False):
        init0 = cls.__init__

        def __init__(self, *a, **k):
            init0(self, *a, **k)
            try:
                _author_all(self.stage, self.env_prim_paths[0])
            except Exception as e:  # noqa: BLE001
                print(f"[mechkit] joint authoring failed: {e!r}", flush=True)
                raise

        cls.__init__ = __init__
        cls._mechkit = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_mechkit", False):
        return
    cls._mechkit = True
    ensure0, reset0, step0 = cls._ensure_sim, cls.reset, cls._task_step

    def _ensure_sim(self, *a, **k):
        out = ensure0(self, *a, **k)
        jointed = {n for kit in _KITS for n, p in kit.parts.items()}
        self._free_objects = tuple(n for n in self._free_objects if n not in jointed)  # no jitter on mechanisms
        return out

    def reset(self, *a, **k):
        if _SIM["stage"] is not None:
            _friction_reset()
        obs = reset0(self, *a, **k)
        _reglue()
        new_episode()
        _EP.update(env=self._env, emb=self, step=0)
        return obs

    def _task_step(self, *a, **k):
        _EP.update(env=self._env, emb=self)
        _EP["step"] += 1
        if any(kit._steps or any(j.kind == "bond" or j.friction > 0 for j in kit.joints_.values()) for kit in _KITS):
            from yam_arena.scene import object_states

            names = [n for n in self.task.names]
            states = object_states(self._env, names)
            for kit in _KITS:
                for j in kit.joints_.values():
                    if j.kind == "bond" and j.child in states and (j.parent is None or j.parent in states):
                        if kit.broken(states, j.name) and j.twist_axis is not None and j.name not in kit._released:
                            kit.release(j.name)  # a twist bond lets go for good past its limit
                _friction_step(kit, states)
                for fn in kit._steps:
                    fn(states)
        return step0(self, *a, **k)

    cls._ensure_sim, cls.reset, cls._task_step = _ensure_sim, reset, _task_step


class _AfterImport(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Runs a patch right after a module is first imported (the module itself is loaded unchanged)."""

    def __init__(self, name: str, patch) -> None:
        self.name, self.patch, self._busy = name, patch, False

    def find_spec(self, fullname, path=None, target=None):
        if fullname != self.name or self._busy:
            return None
        self._busy = True
        try:
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        finally:
            self._busy = False
        if spec is None or spec.loader is None:
            return None
        inner, patch = spec.loader, self.patch

        class _Loader(importlib.abc.Loader):
            def create_module(self, s):
                return inner.create_module(s)

            def exec_module(self, module):
                inner.exec_module(module)
                patch(module)

        spec.loader = _Loader()
        return spec


def _when_imported(name: str, patch) -> None:
    if name in sys.modules:
        patch(sys.modules[name])
    elif not any(isinstance(f, _AfterImport) and f.name == name for f in sys.meta_path):
        sys.meta_path.insert(0, _AfterImport(name, patch))


_when_imported("yam_arena.scene", _patch_scene)
_when_imported("yam_arena.inspect_embodiment", _patch_embodiment)
