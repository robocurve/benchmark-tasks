"""labkit: extra object kinds, scoring helpers and demo poses for yam_arena task files.

yam_arena's task files spawn single primitives (Box, Cylinder, Sphere) and models from Arena's library. This module
adds, without changing yam_arena:

- :class:`Container`: an open-top square container (bottle, tub, tray, bin), its floor and four walls one body,
  with an optional coloured label band round its middle (visual only). Its ``pos`` is the centre of its floor.
- :class:`Marked`: a solid box with flat coloured marks on chosen faces (a crack, a label, a warning patch). Marks
  are visual only, so a mark on the back face is hidden from a camera that sees only the front.
- :class:`Model`: a 3D model from a USD file (convert STL/OBJ with tools/convert_mesh.py on the GPU machine), with
  optional invisible box :class:`Prim` colliders (a fixed machine drawn from its CAD, collided as boxes).
- :class:`Shaped`: a rigid body built from box and cylinder :class:`Prim` colliders, drawn with a mesh (a printed
  part). With ``snap_to`` it starts welded to another :class:`Shaped` by a joint that breaks above ``break_force``
  (N) or ``break_torque`` (N m): a 3D print's support, a moulding's sprue, a breakaway tab. :func:`snapped_off`
  tells whether that joint has let go; :func:`body_point` gives a point fixed in a body (its mesh's centre).
- :class:`Button`: a push button, a loose heavy housing with a cap on top that presses in up to ``travel``
  against a spring (a prismatic joint with a spring drive).
- :class:`Lamp`: a status lamp's lit face, a visual-only disc that stays hidden until any button has been pressed
  ``latch_at`` deep, and goes dark again at every reset. A lamp with a ``group`` lights only when that group is the
  trial's active one (:func:`set_lamp_group`), so a task can show a different answer on different trials.
- :class:`Branch`: a demo step whose waypoints depend on which lamp group lit (what the cameras show), for an
  open-loop demo that must act on an answer it only sees mid-trial.
- :class:`Hinge`: a lid on a fixed base, on a revolute joint (``axis``, ``limits``, optional ``spring``) that starts
  at any angle; with a :class:`Latch`, its state says whether the lid is latched (pressed past a detent near closed,
  and not opened past a tolerance since). For a centrifuge lid, a sharps-container lid, a door.
- :class:`Slider`: a panel on a fixed frame, on a prismatic joint (a fume-hood sash, a drawer front, a cabinet door
  that slides). By default the panel is counterweighted (no gravity), so it stays where it is left.
- :class:`Hand`: a person's open hand, palm up, and forearm, a fixed prop of boxes, with a glove that is on or off
  (per task, or per trial with :func:`set_glove`) and a target zone on the palm (:meth:`Hand.on_palm`,
  :func:`handed_over`) for scoring a hand-over.
- :class:`Towel`: a small cloth, a grid of thin rigid tiles joined at their shared edges by joints that fold and
  twist a little but don't stretch; its state gives every tile's centre, for scoring how much of an area it covers
  (:meth:`TowelState.coverage`).

Timed lamps: :func:`switch_lamp_group` (called from an on_reset hook) lights a lamp group at a chosen time into the
trial, with no button (an alarm panel that turns red 20 s in); the schedule clears at every reset, and
:func:`lit_group` (so a demo :class:`Branch`) sees each switch once it happens. :func:`trial_time` gives the seconds
since the reset, :func:`lamp_group_at` what the schedule shows at a given time.

:func:`trial_seed` gives the seed the current reset used (None if unseeded), for tasks that vary a hidden state.

Importing this module hooks ``yam_arena.scene`` (to spawn these and read their state) and the embodiment's step and
reset (to light the lamps, and to start each trial's bookkeeping through :func:`on_reset` and :class:`Trial`).
``score()`` and ``success()`` then get an ``ObjectState`` for each of these (its body pose; ``low``/``high`` from
its box), a :class:`ButtonState` for a button, and a :class:`HingeState`, :class:`SliderState`, :class:`HandState`
or :class:`TowelState` for the kinds above.

Grippers: every time the scorer reads the objects, labkit also reads both grippers' TCPs (:func:`gripper_points`), so
:func:`released` can require that no gripper is still holding a part (offline tests set them with
:func:`set_gripper_points`).

Scoring helpers: :func:`tilt_deg`, :func:`lifted`, :func:`moved`, :func:`inside`, :func:`count_inside`,
:func:`all_resting`, :func:`handed_over` and :class:`Trial` (flags that latch during a trial and clear at reset).
Offline tests play a trial's clock and a lid's angles with :func:`tick` and :func:`hinge_step`, as each control step
does in the simulator.

Demo helpers: a :class:`Pose` waypoint carries its own TCP rotation, which ``yam_arena.demo.ArmPlan`` follows
(see :func:`forward_grip`, :func:`rolled` and :func:`ready_pose`).

Pure Python at import time; Isaac Sim is imported lazily, once yam_arena's scene module loads. Copy this file into
each task folder (tools/new_task.py does) so every pull request is self-contained.

Status (2026-11-07): Container, Button and Lamp come from a draft task and pass offline checks only; Marked and
Model are new. None has run in Isaac Sim yet: the first GPU session must check that each one spawns.
Added 2026-11-09 (offline checks only): Lamp ``group`` with :func:`set_lamp_group` / :func:`lit_group`,
:func:`trial_seed`, and the demo :class:`Branch`. The first GPU run of a task that uses them must check that only the
active group's lamp lights, that the reset seed reaches the task, and that a Branch follows the lit lamp.
Added 2026-11-15 (offline checks only; needs a GPU check): :class:`Shaped`, breakable ``snap_to`` joints and
:class:`Model` colliders. The first GPU session must check that a snapped body holds its parent's weight and a pull
below ``break_force``, lets go above ``break_torque``, and is welded again at the next reset (a broken PhysX joint
stays broken, so labkit redefines it after every reset); that Model colliders are invisible but collide; and that :func:`gripper_points` finds each robot's "gripper" body and
puts the TCP between the finger pads.
Added 2026-11-10 (offline checks only; needs a GPU check): :class:`Hinge` with :class:`Latch`, :class:`Slider`,
:class:`Hand`, :class:`Towel` and the timed lamp switch. None has run in Isaac Sim. The first GPU session must check
that each spawns where its start state says (lid angle, panel position, flat towel), that a hinge or slider joint
moves within its limits and its spring pulls toward ``rest`` (not toward ``start``), that the fixed bases stay put,
that the glove shows and hides at reset, that a timed switch lights the lamp at its time, and that the towel is
stable at 240 Hz and can be picked up by an edge.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import demo, ik
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import Box, ObjectState
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

__all__ = [
    "Bag", "Bottle", "Branch", "Custom", "Button", "Cloth", "ClothState", "ButtonState", "Container", "Hand", "HandState", "Hinge", "HingeState", "Lamp",
    "Latch", "Live", "Mark", "latest_objects", "Marked", "Model", "Pose", "Prim", "Shaped", "body_point", "gripper_points", "released",
    "set_gripper_points", "snapped_off", "Slider", "SliderState", "Towel", "TowelState", "Trial",
    "all_resting", "count_inside", "forward_grip", "glove_on", "handed_over", "hinge_step", "inside", "lamp_group_at",
    "lamp_switches", "lifted", "lit_group", "moved", "on_reset", "ready_pose", "rolled", "set_glove", "set_lamp_group",
    "switch_lamp_group", "tick", "tilt_deg", "trial_seed", "trial_time",
]

Color = tuple[float, float, float]
X_AXIS, Y_AXIS, Z_AXIS = np.eye(3)
FACES = ("front", "back", "left", "right", "top")  # front faces the robot (-x); left is +y
PROUD = 0.0006  # how far a visual band or mark stands off its face (m)
AXES = {"x": X_AXIS, "y": Y_AXIS, "z": Z_AXIS}
GRAVITY = 9.81
SKIN = (0.93, 0.74, 0.62)
NITRILE = (0.30, 0.45, 0.90)  # a blue nitrile glove
BASE_MASS = 2.0  # kg: a hinge's or slider's base (welded to the world, so it only steadies the solver)
MAX_LINKS = 64  # PhysX's limit on the links of one articulation


def _box_corners(center_local, size, pos, quat_xyzw) -> np.ndarray:
    """The 8 corners (world) of a box of ``size`` centred at ``center_local`` in a body at ``pos``/``quat``."""
    sx, sy, sz = size
    c = np.asarray(center_local, float)
    local = np.array([[i * sx / 2, j * sy / 2, k * sz / 2] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) + c
    return local @ quat_xyzw_to_matrix(np.asarray(quat_xyzw, float)).T + np.asarray(pos, float)


def _yaw_quat(yaw: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def _yaw_matrix(yaw: float) -> np.ndarray:
    return axis_angle_matrix(Z_AXIS, yaw)


def _turn(axis: str, angle: float) -> np.ndarray:
    """Rotation by ``angle`` (rad, right-handed) about the body axis ``axis`` ("x", "y" or "z")."""
    return axis_angle_matrix(AXES[axis], angle)


def _union_box(*corner_sets) -> tuple[np.ndarray, np.ndarray]:
    c = np.vstack(corner_sets)
    return c.min(0), c.max(0)


# -- object kinds ---------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Container(Box):
    """An open-top square container: ``size`` is its outside (x, y, height), ``wall`` its wall and floor thickness,
    ``color`` its body and ``label`` (None: no band) the band round it, from ``band[0]`` to ``band[1]`` above its
    bottom. ``pos`` is its floor's centre. Loose with a ``mass``; fixed without one."""

    wall: float = 0.003
    label: Color | None = None
    band: tuple[float, float] = (0.035, 0.075)

    def _check(self) -> None:
        super()._check()
        if not 0 < 2 * self.wall < min(self.size[:2]) or not self.wall < self.size[2]:
            raise ValueError(f"{self.name}: wall {self.wall} doesn't fit size {self.size}")
        if self.label is not None and not 0 <= self.band[0] < self.band[1] <= self.size[2]:
            raise ValueError(f"{self.name}: band {self.band} must lie within the height {self.size[2]}")

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        """The 8 outside corners, world frame, for a body origin (floor centre) at ``pos``."""
        return _box_corners((0.0, 0.0, self.size[2] / 2 - self.wall / 2), self.size, pos, quat_xyzw)

    def holds(self, state: ObjectState, point, margin: float = 0.0) -> bool:
        """Whether ``point`` (world) is inside this container, as it lies in ``state``."""
        local = quat_xyzw_to_matrix(state.quat_xyzw).T @ (np.asarray(point, float) - state.pos)
        inner_x, inner_y = self.size[0] / 2 - self.wall - margin, self.size[1] / 2 - self.wall - margin
        return bool(abs(local[0]) < inner_x and abs(local[1]) < inner_y and 0 < local[2] < self.size[2] - self.wall)


Bottle = Container  # the name the first draft task used


@dataclass(frozen=True)
class Mark:
    """A flat coloured patch on one face of a :class:`Marked` box: ``size`` (width, height) and ``at`` (offset from
    the face's centre) in the face's own plane. Front/back: width along y, height along z. Left/right: width along x,
    height along z. Top: width along x, height along y."""

    face: str
    size: tuple[float, float]
    color: Color
    at: tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class Marked(Box):
    """A solid box with :class:`Mark` patches on its faces. ``pos`` is its centre."""

    marks: tuple[Mark, ...] = ()

    def _check(self) -> None:
        super()._check()
        for m in self.marks:
            if m.face not in FACES:
                raise ValueError(f"{self.name}: mark face {m.face!r} must be one of {FACES}")
            (w, h), (u, v) = m.size, m.at
            fw, fh = self._face_extent(m.face)
            if w <= 0 or h <= 0 or abs(u) + w / 2 > fw / 2 + 1e-9 or abs(v) + h / 2 > fh / 2 + 1e-9:
                raise ValueError(f"{self.name}: a {m.face} mark of {m.size} at {m.at} runs off the face ({fw}, {fh})")

    def _face_extent(self, face: str) -> tuple[float, float]:
        sx, sy, sz = self.size
        return {"front": (sy, sz), "back": (sy, sz), "left": (sx, sz), "right": (sx, sz), "top": (sx, sy)}[face]

    def mark_box(self, m: Mark) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """(size, centre) of a mark's thin visual box, in the body frame."""
        sx, sy, sz = self.size
        (w, h), (u, v) = m.size, m.at
        return {
            "front": ((PROUD, w, h), (-(sx + PROUD) / 2, u, v)),
            "back": ((PROUD, w, h), ((sx + PROUD) / 2, u, v)),
            "left": ((w, PROUD, h), (u, (sy + PROUD) / 2, v)),
            "right": ((w, PROUD, h), (u, -(sy + PROUD) / 2, v)),
            "top": ((w, h, PROUD), (u, v, (sz + PROUD) / 2)),
        }[m.face]

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        return _box_corners((0.0, 0.0, 0.0), self.size, pos, quat_xyzw)


@dataclass(frozen=True)
class Model(Box):
    """A 3D model from a USD file (absolute path in ``usd``), ``scale`` applied. Give ``size`` as its bounding box
    after scaling and ``pos`` as that box's centre; the USD's origin must sit at the box centre (tools/convert_mesh.py
    recentres it). Loose with a ``mass``; fixed without one. ``color`` is ignored: the model keeps its own look."""

    usd: str = ""
    scale: float = 1.0
    colliders: tuple[Prim, ...] = ()  # invisible boxes in the model's frame (its box centre), after scaling

    def _check(self) -> None:
        super()._check()
        for prim in self.colliders:
            prim.check(self.name)
        if not self.usd.endswith((".usd", ".usda", ".usdc")):
            raise ValueError(f"{self.name}: usd must be a .usd/.usda/.usdc file path, got {self.usd!r}")
        if not self.scale > 0:
            raise ValueError(f"{self.name}: scale must be positive")

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        return _box_corners((0.0, 0.0, 0.0), self.size, pos, quat_xyzw)


@dataclass(frozen=True)
class Prim:
    """A collider primitive in a body's frame: ``kind`` "box" (``size`` its x, y, z edges), "cylinder" (along its own
    z; ``size`` (radius, radius, height)) or "capsule" (a cylinder with round ends, along its own z; ``size`` (radius,
    radius, length of the straight part)), centred at ``at`` and turned by ``rpy`` (roll, pitch, yaw in radians, about
    the body's x, then y, then z)."""

    kind: str
    at: tuple[float, float, float]
    size: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def check(self, owner: str) -> None:
        if self.kind not in ("box", "cylinder", "capsule"):
            raise ValueError(f"{owner}: a Prim's kind must be 'box', 'cylinder' or 'capsule', got {self.kind!r}")
        if any(v <= 0 for v in self.size) or (self.kind != "box" and self.size[0] != self.size[1]):
            raise ValueError(f"{owner}: Prim size {self.size} must be positive (a cylinder's: radius, radius, height)")

    @property
    def rot(self) -> np.ndarray:
        r, p, y = self.rpy
        return _turn("z", y) @ _turn("y", p) @ _turn("x", r)

    @property
    def extent(self) -> tuple[float, float, float]:
        """Its own bounding box's edges, before turning."""
        if self.kind == "box":
            return tuple(self.size)
        tall = self.size[2] + (2 * self.size[0] if self.kind == "capsule" else 0.0)
        return (2 * self.size[0], 2 * self.size[0], tall)

    def corners(self) -> np.ndarray:
        """The 8 corners of its bounding box, body frame."""
        local = _box_corners((0, 0, 0), self.extent, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        return local @ self.rot.T + np.asarray(self.at, float)


@dataclass(frozen=True)
class Shaped(Box):
    """A rigid body made of :class:`Prim` colliders (``parts``, body frame), drawn with the mesh in ``mesh`` (a
    .usd/.usda/.usdc path whose geometry is in the body frame, metres; empty: the colliders are drawn instead) in
    ``color``. ``pos`` is the body's origin, not its centre; ``size`` and ``box_at`` (body frame) are its bounding box,
    which :meth:`corners` and the state's ``low``/``high`` use. Loose with a ``mass``, spread over the colliders.

    ``snap_to`` names another :class:`Shaped` (listed before this one in OBJECTS) that this body starts welded to,
    by a fixed joint that breaks once it carries more than ``break_force`` (N) or ``break_torque`` (N m). The two
    don't collide while joined. ``friction`` (static, dynamic) overrides the kit's rubber-like default."""

    parts: tuple[Prim, ...] = ()
    mesh: str = ""
    box_at: tuple[float, float, float] = (0.0, 0.0, 0.0)
    snap_to: str | None = None
    break_force: float = math.inf
    break_torque: float = math.inf
    friction: tuple[float, float] | None = None

    def _check(self) -> None:
        super()._check()
        if not self.parts:
            raise ValueError(f"{self.name}: give at least one collider Prim in parts")
        for prim in self.parts:
            prim.check(self.name)
        if self.mesh and not self.mesh.endswith((".usd", ".usda", ".usdc")):
            raise ValueError(f"{self.name}: mesh must be a .usd/.usda/.usdc path, got {self.mesh!r}")
        if self.snap_to is not None:
            if self.snap_to == self.name or self.mass is None:
                raise ValueError(f"{self.name}: a snapped body must be loose (a mass) and snap to another body")
            if not (self.break_force > 0 and self.break_torque > 0):
                raise ValueError(f"{self.name}: break_force and break_torque must be positive")
            if math.isinf(self.break_force) and math.isinf(self.break_torque):
                raise ValueError(f"{self.name}: a snap that can't break: give break_force or break_torque")
        lo, hi = _union_box(*(p.corners() for p in self.parts))
        b_lo, b_hi = np.asarray(self.box_at) - np.asarray(self.size) / 2, np.asarray(self.box_at) + np.asarray(self.size) / 2
        if np.any(lo < b_lo - 1e-6) or np.any(hi > b_hi + 1e-6):
            raise ValueError(f"{self.name}: colliders {lo.round(4)}..{hi.round(4)} stick out of the bounding box "
                             f"{b_lo.round(4)}..{b_hi.round(4)} (size, box_at)")

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        return _box_corners(self.box_at, self.size, pos, quat_xyzw)

    @property
    def rot(self) -> np.ndarray:
        return _yaw_matrix(self.yaw)


def body_point(state: ObjectState, local) -> np.ndarray:
    """The world position of ``local`` (a point fixed in the body, body frame) as the body lies in ``state``."""
    return np.asarray(state.pos, float) + quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float)) @ np.asarray(local, float)


def snapped_off(child: ObjectState, parent: ObjectState, child_spec: Shaped, parent_spec: Shaped,
                d: float = 0.005) -> bool:
    """Whether a snapped body has let go of its parent: its bounding-box centre has moved more than ``d`` (m) from
    where it started, measured in the parent's frame (so carrying the two together doesn't count)."""
    def rel(c_pos, c_rot, p_pos, p_rot):
        return p_rot.T @ (np.asarray(c_pos, float) + c_rot @ np.asarray(child_spec.box_at, float) - np.asarray(p_pos, float))

    now = rel(child.pos, quat_xyzw_to_matrix(np.asarray(child.quat_xyzw, float)), parent.pos,
              quat_xyzw_to_matrix(np.asarray(parent.quat_xyzw, float)))
    start = rel(child_spec.pos, child_spec.rot, parent_spec.pos, parent_spec.rot)
    return bool(np.linalg.norm(now - start) > d)


@dataclass(frozen=True)
class Button(Box):
    """A push button: a housing (``size``, ``pos`` its centre, ``mass`` heavy enough not to slide when pressed,
    ``color``) with a cap (``cap`` x, y, z, ``cap_color``) on top that presses in up to ``travel`` against a ``rate``
    N/m spring. Lamps light once the cap has gone in ``latch_at``, and stay lit until the next reset."""

    cap: tuple[float, float, float] = (0.04, 0.04, 0.015)
    cap_color: Color = (0.15, 0.35, 0.85)
    cap_mass: float = 0.01
    travel: float = 0.008
    latch_at: float = 0.0056
    rate: float = 300.0
    damping: float = 4.0

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: give the housing a mass (it stands loose on the table)")
        if not 0 < self.latch_at <= self.travel:
            raise ValueError(f"{self.name}: latch_at must be within the cap's travel")

    @property
    def cap_top(self) -> float:
        """Height of the cap's top at rest (m)."""
        return self.pos[2] + self.size[2] / 2 + self.cap[2]


@dataclass(frozen=True)
class Lamp(Box):
    """A status lamp's lit face: a visual-only disc (``radius``, ``size[2]`` thick) in ``color``, hidden until a
    button latches. Put the dark lamp under it as an ordinary fixed ``Cylinder``. With a ``group``, it lights only
    if that group is the trial's active one (:func:`set_lamp_group`)."""

    radius: float = 0.015
    group: str | None = None

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a lamp is fixed (no mass)")


@dataclass(frozen=True)
class ButtonState(ObjectState):
    """The housing's pose and how far the cap is pressed in (m, 0 at rest)."""

    travel: float = 0.0


# -- jointed objects: a hinge and a slider ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Latch:
    """A latch on a :class:`Hinge`. It catches once the lid comes within ``detent`` (rad) of closed (angle 0) and
    reports latched until the lid opens past ``tol`` (rad); it does not hold the lid shut. A ``detent`` below 0 means
    the lid must be pressed past flush, into overtravel the hinge's ``limits`` allow below 0, against a ``spring``
    that keeps the lid's own weight from sinking it there (:class:`Hinge` checks the spring is strong enough)."""

    detent: float = 0.03
    tol: float = 0.10

    def step(self, angle: float, was: bool) -> bool:
        """Whether the latch is caught at ``angle``, given whether it was caught at the step before."""
        return bool(angle <= self.detent or (was and angle <= self.tol))


@dataclass(frozen=True)
class Hinge(Box):
    """A lid on a fixed base. The base is a box (``size``, ``pos`` its centre, ``color``; welded in place, so no
    ``mass``); the lid is a box (``lid`` x, y, z, ``lid_color``, ``lid_mass``) on a revolute joint about ``axis``
    ("x", "y" or "z", in the base's frame) through ``pivot`` (base frame; default: the base's back top edge).
    ``lid_at`` is the lid box's centre from the pivot when closed (default: lying on the base's top, reaching forward
    from the pivot). Angle 0 is closed, the geometry as given; a positive angle turns the lid about +``axis``, which
    for the default back-edge y hinge lifts its front. ``limits`` (rad) bound the angle, ``start`` is where each trial
    starts, ``spring`` (N m/rad, None for none) pulls toward ``rest`` (rad: 0 a self-closing lid, an open angle a
    gas strut), ``damping`` (N m s/rad) and ``friction`` (N m) slow it. ``latch`` (a :class:`Latch`) adds a latched
    flag to its state. The lid and base don't collide with each other: the joint limits are the stops."""

    lid: tuple[float, float, float] = (0.12, 0.12, 0.01)
    lid_color: Color = (0.85, 0.85, 0.88)
    lid_mass: float = 0.05
    axis: str = "y"
    pivot: tuple[float, float, float] | None = None
    lid_at: tuple[float, float, float] | None = None
    limits: tuple[float, float] = (0.0, 1.9)
    start: float = 0.0
    spring: float | None = None
    rest: float = 0.0
    damping: float = 0.01
    friction: float = 0.0
    latch: Latch | None = None

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a hinge's base is fixed (no mass); the lid's is lid_mass")
        if self.axis not in AXES:
            raise ValueError(f"{self.name}: axis must be one of {tuple(AXES)}, got {self.axis!r}")
        if any(s <= 0 for s in self.lid) or not self.lid_mass > 0:
            raise ValueError(f"{self.name}: lid size and lid_mass must be positive")
        lo, hi = self.limits
        if not lo <= 0.0 <= hi or not lo < hi:
            raise ValueError(f"{self.name}: limits {self.limits} must run from at most 0 (closed) to above it")
        if not lo <= self.start <= hi:
            raise ValueError(f"{self.name}: start {self.start} is outside the limits {self.limits}")
        if (self.spring is not None and self.spring < 0) or self.damping < 0 or self.friction < 0:
            raise ValueError(f"{self.name}: spring, damping and friction can't be negative")
        if self.latch is None:
            return
        d, tol = self.latch.detent, self.latch.tol
        if not lo <= d < tol <= hi:
            raise ValueError(f"{self.name}: the latch needs limits[0] <= detent < tol <= limits[1], got "
                             f"{self.limits}, detent {d}, tol {tol}")
        if d < 0:
            sink = max(0.0, -self.weight_torque(d))  # the lid's weight, closing it
            hold = (self.spring or 0.0) * (self.rest - d)
            if hold <= 1.5 * sink:
                raise ValueError(f"{self.name}: with the detent below flush, the lid's weight alone would latch it: "
                                 f"give it a spring of over {1.5 * sink / max(self.rest - d, 1e-9):.3f} N m/rad "
                                 f"toward a rest at or above 0")

    @property
    def pivot_point(self) -> np.ndarray:
        """The hinge line's point, base frame."""
        if self.pivot is not None:
            return np.asarray(self.pivot, float)
        return np.array([self.size[0] / 2, 0.0, self.size[2] / 2])

    @property
    def lid_offset(self) -> np.ndarray:
        """The lid box's centre from the pivot when closed, base frame."""
        if self.lid_at is not None:
            return np.asarray(self.lid_at, float)
        return np.array([-self.lid[0] / 2, 0.0, self.lid[2] / 2])

    def lid_pose(self, angle: float) -> tuple[np.ndarray, np.ndarray]:
        """(centre, rotation matrix) of the lid box, world frame, at ``angle``."""
        yaw, turn = _yaw_matrix(self.yaw), _turn(self.axis, angle)
        pivot = np.asarray(self.pos, float) + yaw @ self.pivot_point
        return pivot + yaw @ turn @ self.lid_offset, yaw @ turn

    def lid_corners(self, angle: float) -> np.ndarray:
        centre, rot = self.lid_pose(angle)
        return _box_corners((0.0, 0.0, 0.0), self.lid, centre, ik.matrix_to_quat_xyzw(rot))

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        """The base's 8 corners, world frame."""
        return _box_corners((0.0, 0.0, 0.0), self.size, pos, quat_xyzw)

    def weight_torque(self, angle: float) -> float:
        """The lid's weight as a torque about the hinge at ``angle`` (N m; positive opens it)."""
        arm = _turn(self.axis, angle) @ self.lid_offset  # base frame; yaw doesn't tilt the vertical
        return float(np.cross(arm, (0.0, 0.0, -self.lid_mass * GRAVITY)) @ AXES[self.axis])

    def state_at(self, angle: float, latched: bool | None = None) -> HingeState:
        """The state with the lid at ``angle``. ``latched`` None: as a trial starting at that angle reports it."""
        pos, quat = np.asarray(self.pos, float), _yaw_quat(self.yaw)
        centre, rot = self.lid_pose(angle)
        low, high = _union_box(self.corners(pos, quat), self.lid_corners(angle))
        if latched is None:
            latched = self.latch is not None and self.latch.step(angle, False)
        return HingeState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=low, high=high,
                          angle=float(angle), latched=bool(latched), lid_pos=centre,
                          lid_quat=ik.matrix_to_quat_xyzw(rot))


@dataclass(frozen=True)
class HingeState(ObjectState):
    """The base's pose; ``low``/``high`` bound base and lid. ``angle`` is the lid's (rad, 0 closed), ``latched``
    the latch's flag (always False without a latch), ``lid_pos``/``lid_quat`` the lid box's centre and rotation."""

    angle: float = 0.0
    latched: bool = False
    lid_pos: np.ndarray | None = None
    lid_quat: np.ndarray | None = None


@dataclass(frozen=True)
class Slider(Box):
    """A panel on a fixed frame. The frame is a box (``size``, ``pos`` its centre, ``color``; welded in place, so no
    ``mass``); the panel is a box (``panel``, ``panel_color``, ``panel_mass``) on a prismatic joint along ``axis``
    ("x", "y" or "z", frame axes). ``panel_at`` is the panel's centre from the frame's at position 0 (default: just in
    front of the frame's front face, centred). Position 0 is the geometry as given; ``limits`` (m) bound it,
    ``start`` is where each trial starts, ``spring`` (N/m, None for none) pulls toward ``rest``, ``damping``
    (N s/m) and ``friction`` (N) slow it. ``balanced`` (default) leaves the panel weightless, like a counterweighted
    sash that stays where it's left; without it a vertical panel falls to a limit unless a spring holds it."""

    panel: tuple[float, float, float] = (0.01, 0.30, 0.20)
    panel_color: Color = (0.70, 0.82, 0.92)
    panel_mass: float = 0.3
    axis: str = "z"
    panel_at: tuple[float, float, float] | None = None
    limits: tuple[float, float] = (0.0, 0.20)
    start: float = 0.0
    spring: float | None = None
    rest: float = 0.0
    damping: float = 5.0
    friction: float = 0.0
    balanced: bool = True

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a slider's frame is fixed (no mass); the panel's is panel_mass")
        if self.axis not in AXES:
            raise ValueError(f"{self.name}: axis must be one of {tuple(AXES)}, got {self.axis!r}")
        if any(s <= 0 for s in self.panel) or not self.panel_mass > 0:
            raise ValueError(f"{self.name}: panel size and panel_mass must be positive")
        lo, hi = self.limits
        if not lo < hi or not lo <= self.start <= hi:
            raise ValueError(f"{self.name}: start {self.start} must lie within limits {self.limits} (low < high)")
        if (self.spring is not None and self.spring < 0) or self.damping < 0 or self.friction < 0:
            raise ValueError(f"{self.name}: spring, damping and friction can't be negative")

    @property
    def panel_offset(self) -> np.ndarray:
        """The panel's centre from the frame's at position 0, frame axes."""
        if self.panel_at is not None:
            return np.asarray(self.panel_at, float)
        return np.array([-(self.size[0] + self.panel[0]) / 2 - 0.002, 0.0, 0.0])

    def panel_center(self, position: float) -> np.ndarray:
        """The panel's centre, world frame, at ``position``."""
        local = self.panel_offset + position * AXES[self.axis]
        return np.asarray(self.pos, float) + _yaw_matrix(self.yaw) @ local

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        """The frame's 8 corners, world frame."""
        return _box_corners((0.0, 0.0, 0.0), self.size, pos, quat_xyzw)

    def state_at(self, position: float) -> SliderState:
        pos, quat = np.asarray(self.pos, float), _yaw_quat(self.yaw)
        centre = self.panel_center(position)
        low, high = _union_box(self.corners(pos, quat), _box_corners((0.0, 0.0, 0.0), self.panel, centre, quat))
        return SliderState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=low, high=high,
                           position=float(position), panel_pos=centre)


@dataclass(frozen=True)
class SliderState(ObjectState):
    """The frame's pose; ``low``/``high`` bound frame and panel. ``position`` is the panel's (m along the axis from
    the geometry as given), ``panel_pos`` its centre."""

    position: float = 0.0
    panel_pos: np.ndarray | None = None


# -- a person's hand -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Hand(Box):
    """A person's open hand, palm up, with its forearm: a fixed prop of boxes (all colliders). ``size`` is the palm
    (length, width, thickness) and ``pos`` its centre; at yaw 0 the fingers point to -x (toward the arms) and the
    forearm runs back along +x, its top level with the palm's. ``finger`` is (length, thickness) of the four fingers,
    ``forearm`` (length, width, height); ``side`` "right" puts the thumb on +y, "left" on -y. ``color`` is the skin.
    The glove (``glove`` its colour) is a visual shell over palm, fingers, thumb and a cuff on the wrist: on at the
    start of every trial if ``gloved``, or set per trial with :func:`set_glove` from an on_reset hook."""

    size: tuple[float, float, float] = (0.09, 0.085, 0.025)
    color: Color = SKIN
    finger: tuple[float, float] = (0.075, 0.018)
    forearm: tuple[float, float, float] = (0.20, 0.065, 0.055)
    side: str = "right"
    glove: Color = NITRILE
    gloved: bool = False

    GAP = 0.003  # between fingers (m)
    CUFF = 0.05  # how far the glove's cuff runs up the forearm (m)

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a hand is a fixed prop (no mass)")
        if self.side not in ("right", "left"):
            raise ValueError(f"{self.name}: side must be 'right' or 'left', got {self.side!r}")
        if any(v <= 0 for v in (*self.finger, *self.forearm)):
            raise ValueError(f"{self.name}: finger and forearm sizes must be positive")
        if self.finger[1] > self.size[2] or self.forearm[2] < self.size[2]:
            raise ValueError(f"{self.name}: fingers must be no thicker than the palm, the forearm no thinner")
        if (self.size[1] - 3 * self.GAP) / 4 <= 0.004:
            raise ValueError(f"{self.name}: the palm is too narrow for four fingers")

    def parts(self) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]]:
        """Each collider box as (centre, size) in the hand's frame (origin the palm's centre, before yaw)."""
        px, py, pz = self.size
        top = pz / 2
        fl, ft = self.finger
        fw = (py - 3 * self.GAP) / 4
        out = {"palm": ((0.0, 0.0, 0.0), (px, py, pz))}
        for k in range(4):
            out[f"finger{k}"] = ((-(px + fl) / 2, -py / 2 + fw / 2 + k * (fw + self.GAP), top - ft / 2), (fl, fw, ft))
        tl, tw = 0.6 * fl, 1.2 * fw
        s = 1.0 if self.side == "right" else -1.0
        out["thumb"] = ((-px / 2 + tl / 2 - 0.01, s * (py / 2 + tw / 2), top - ft / 2), (tl, tw, ft))
        length, width, height = self.forearm
        out["forearm"] = ((px / 2 + length / 2, 0.0, top - height / 2), (length, width, height))
        return out

    def glove_parts(self) -> dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]]:
        """The glove's visual boxes as (centre, size) in the hand's frame: palm, fingers and thumb a shade larger
        than the skin, and a cuff round the start of the forearm."""
        grow = 2 * PROUD
        out = {name: (at, tuple(s + grow for s in size)) for name, (at, size) in self.parts().items()
               if name != "forearm"}
        (_, _, fz), (_, width, height) = self.parts()["forearm"]
        cuff = min(self.CUFF, self.forearm[0])
        out["cuff"] = ((self.size[0] / 2 + cuff / 2, 0.0, fz), (cuff, width + grow, height + grow))
        return out

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        """The corners of every collider box, world frame."""
        return np.vstack([_box_corners(at, size, pos, quat_xyzw) for at, size in self.parts().values()])

    @property
    def palm_top(self) -> float:
        """The palm's top surface height (m)."""
        return float(self.pos[2]) + self.size[2] / 2

    def on_palm(self, point, height: float = 0.06, margin: float = 0.0) -> bool:
        """Whether ``point`` (world) is over the palm (``margin`` in from its edges) and no more than ``height``
        above its top: the target zone for an object handed over."""
        local = _yaw_matrix(self.yaw).T @ (np.asarray(point, float) - np.asarray(self.pos, float))
        hx, hy = self.size[0] / 2 - margin, self.size[1] / 2 - margin
        return bool(abs(local[0]) < hx and abs(local[1]) < hy and self.size[2] / 2 < local[2] <= self.size[2] / 2
                    + height)

    def palm_zone(self, height: float = 0.06, margin: float = 0.0):
        """(centre (x, y), half sizes (x, y) along the hand's own axes, z range) of the target zone; equal to
        :func:`inside`'s region when the hand isn't yawed. Prefer :meth:`on_palm`, which follows the yaw."""
        return ((float(self.pos[0]), float(self.pos[1])), (self.size[0] / 2 - margin, self.size[1] / 2 - margin),
                (self.palm_top, self.palm_top + height))

    def state(self) -> HandState:
        """The hand as the scorer gets it: fixed, with the glove on or off as this trial has it."""
        pos, quat = np.asarray(self.pos, float), _yaw_quat(self.yaw)
        low, high = _union_box(self.corners(pos, quat))
        return HandState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=low, high=high,
                         gloved=glove_on(self))


@dataclass(frozen=True)
class HandState(ObjectState):
    """The palm's centre and the hand's yaw (fixed); ``low``/``high`` bound hand and forearm; ``gloved`` whether
    the glove shows this trial."""

    gloved: bool = False


def handed_over(objects, name: str, hand: Hand, height: float = 0.06, margin: float = 0.0,
                max_speed: float = 0.01) -> bool:
    """Object ``name``'s centre is in ``hand``'s palm zone (:meth:`Hand.on_palm`) and it is still. Whether the
    gripper has let go is the task's to check (an arm still holding it there is not a hand-over)."""
    s = objects[name]
    return hand.on_palm(s.pos, height, margin) and s.resting(max_speed=max_speed)


# -- a towel ---------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Towel(Box):
    """A small cloth: ``tiles`` (nx, ny) thin rigid tiles in a grid, neighbours joined at their shared edge by a
    joint that folds about the edge (up to ``max_fold`` rad) and twists a little about the line between their
    centres (up to ``max_twist``), with no stretch or shear. ``size`` is the whole towel (x, y, thickness), ``pos``
    its centre lying flat, ``mass`` the whole towel's (required). Each joint has a weak spring back to flat
    (``stiffness`` N m/rad, ``damping``, ``armature`` for stability at 240 Hz). Tiles are ``gap`` smaller than
    their pitch so diagonal neighbours don't touch when flat. It is never jittered at reset (yam_arena moves rigid
    bodies only), so check_task treats it as fixed. At most 64 tiles (PhysX's articulation limit)."""

    size: tuple[float, float, float] = (0.20, 0.20, 0.004)
    color: Color = (0.92, 0.92, 0.95)
    tiles: tuple[int, int] = (6, 6)
    gap: float = 0.001
    max_fold: float = 2.0
    max_twist: float = 0.5
    stiffness: float = 0.002
    damping: float = 0.0005
    armature: float = 2e-6
    friction: float = 0.9
    self_collision: bool = True

    @property
    def free(self) -> bool:
        return False

    def _check(self) -> None:
        super()._check()
        nx, ny = self.tiles
        if self.mass is None:
            raise ValueError(f"{self.name}: give the towel a mass (the whole towel's, kg)")
        if int(nx) != nx or int(ny) != ny or nx < 2 or ny < 2 or nx * ny > MAX_LINKS:
            raise ValueError(f"{self.name}: tiles must be whole numbers, at least 2 x 2 and at most {MAX_LINKS} "
                             f"in all, got {self.tiles}")
        px, py = self.pitch
        if not 0 <= self.gap < min(px, py) / 4 or self.size[2] >= min(px, py) / 2:
            raise ValueError(f"{self.name}: tiles of {px:.3f} x {py:.3f} m need a gap under a quarter and a "
                             f"thickness under half of that")
        if not 0 < self.max_fold <= math.pi or not 0 <= self.max_twist <= math.pi / 2:
            raise ValueError(f"{self.name}: max_fold must be in (0, pi], max_twist in [0, pi/2]")
        if min(self.stiffness, self.damping, self.armature, self.friction) < 0:
            raise ValueError(f"{self.name}: stiffness, damping, armature and friction can't be negative")

    @property
    def pitch(self) -> tuple[float, float]:
        """Centre-to-centre spacing of the tiles along x and y (m)."""
        return self.size[0] / self.tiles[0], self.size[1] / self.tiles[1]

    @property
    def centre_tile(self) -> tuple[int, int]:
        """The tile (i, j) the articulation is rooted at."""
        return self.tiles[0] // 2, self.tiles[1] // 2

    def tile_ids(self) -> list[tuple[int, int]]:
        """Every tile's (i, j), i along x, in the order a state's ``points`` lists them."""
        return [(i, j) for i in range(self.tiles[0]) for j in range(self.tiles[1])]

    @staticmethod
    def tile_name(i: int, j: int) -> str:
        return f"tile_{i}_{j}"

    def edges(self) -> list[tuple[tuple[int, int], tuple[int, int], bool]]:
        """Every pair of neighbouring tiles as (parent, child, in_tree). The tree joints (a spine along x through the
        centre tile and ribs along y off it) form the articulation; the others close the grid's loops as ordinary
        joints."""
        nx, ny = self.tiles
        ic, jc = self.centre_tile
        out = [((i - 1 if i > ic else i + 1, jc), (i, jc), True) for i in range(nx) if i != ic]
        out += [((i, j - 1 if j > jc else j + 1), (i, j), True) for i in range(nx) for j in range(ny) if j != jc]
        out += [((i - 1, j), (i, j), False) for i in range(1, nx) for j in range(ny) if j != jc]
        return out

    def start_points(self) -> np.ndarray:
        """Each tile's starting centre, world frame, (nx * ny, 3) in :meth:`tile_ids` order."""
        nx, ny = self.tiles
        px, py = self.pitch
        local = np.array([((i - (nx - 1) / 2) * px, (j - (ny - 1) / 2) * py, 0.0) for i, j in self.tile_ids()])
        return local @ _yaw_matrix(self.yaw).T + np.asarray(self.pos, float)

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        """The flat towel's 8 corners, world frame."""
        return _box_corners((0.0, 0.0, 0.0), self.size, pos, quat_xyzw)

    def state_at(self, points=None, quats=None, lin_vel=None, ang_vel=None) -> TowelState:
        """The state with tiles at ``points`` (n, 3) turned by ``quats`` (n, 4, xyzw); by default lying flat where it
        starts, still. ``lin_vel``/``ang_vel`` (n, 3) per tile; the state reports the fastest tile's."""
        n = self.tiles[0] * self.tiles[1]
        pts = self.start_points() if points is None else np.asarray(points, float).reshape(n, 3)
        qs = np.tile(_yaw_quat(self.yaw), (n, 1)) if quats is None else np.asarray(quats, float).reshape(n, 4)
        lin = np.zeros((n, 3)) if lin_vel is None else np.asarray(lin_vel, float).reshape(n, 3)
        ang = np.zeros((n, 3)) if ang_vel is None else np.asarray(ang_vel, float).reshape(n, 3)
        px, py = self.pitch
        tile = (px - self.gap, py - self.gap, self.size[2])
        low, high = _union_box(*(_box_corners((0.0, 0.0, 0.0), tile, p, q) for p, q in zip(pts, qs)))
        root = self.tile_ids().index(self.centre_tile)
        return TowelState(pos=pts.mean(0), quat_xyzw=qs[root], lin_vel=lin[np.argmax(np.linalg.norm(lin, axis=1))],
                          ang_vel=ang[np.argmax(np.linalg.norm(ang, axis=1))], low=low, high=high, points=pts,
                          quats=qs, shape=tuple(self.tiles), pitch=self.pitch)


@dataclass(frozen=True)
class TowelState(ObjectState):
    """The towel: ``pos`` the mean of its tile centres, ``quat_xyzw`` the centre tile's, ``lin_vel``/``ang_vel`` the
    fastest tile's (so ``resting()`` means every tile is still), ``low``/``high`` bound every tile. ``points`` are the
    tile centres and ``quats`` their rotations, (nx * ny, 3) and (nx * ny, 4) in :meth:`Towel.tile_ids` order."""

    points: np.ndarray | None = None
    quats: np.ndarray | None = None
    shape: tuple[int, int] = (0, 0)
    pitch: tuple[float, float] = (0.0, 0.0)

    def grid(self) -> np.ndarray:
        """Tile centres as (nx, ny, 3): ``grid()[i, j]`` is tile (i, j)."""
        return self.points.reshape(*self.shape, 3)

    def corner_tiles(self) -> np.ndarray:
        """The four corner tiles' centres: (0, 0), (nx-1, 0), (0, ny-1), (nx-1, ny-1)."""
        g = self.grid()
        return np.array([g[0, 0], g[-1, 0], g[0, -1], g[-1, -1]])

    def fraction_inside(self, center, half_xy, z_range) -> float:
        """The share of tile centres :func:`inside` the region."""
        return float(np.mean([inside(p, center, half_xy, z_range) for p in self.points]))

    def coverage(self, center, half_xy, z_range=None, step: float = 0.004) -> float:
        """The share of the rectangle ``center`` +- ``half_xy`` (x, y; m) that the towel covers seen from above,
        counting only tiles whose centre lies within ``z_range`` (e.g. on the bench, not held up), sampled every
        ``step`` m. Each tile covers its full pitch (gaps included), so a flat towel covers its own footprint."""
        hx, hy = half_xy
        nx, ny = max(1, math.ceil(2 * hx / step)), max(1, math.ceil(2 * hy / step))
        xs = center[0] - hx + (np.arange(nx) + 0.5) * (2 * hx / nx)
        ys = center[1] - hy + (np.arange(ny) + 0.5) * (2 * hy / ny)
        samples = np.stack(np.meshgrid(xs, ys, indexing="ij"), -1).reshape(-1, 2)
        covered = np.zeros(len(samples), bool)
        half_u, half_v = self.pitch[0] / 2 + 1e-9, self.pitch[1] / 2 + 1e-9
        for p, q in zip(self.points, self.quats):
            if z_range is not None and not z_range[0] < p[2] < z_range[1]:
                continue
            rot = quat_xyzw_to_matrix(q)
            uv = np.column_stack([rot[:2, 0], rot[:2, 1]])  # the tile's own x and y, seen from above
            if abs(np.linalg.det(uv)) < 1e-6:  # standing on edge: covers nothing
                continue
            a, b = np.linalg.solve(uv, (samples - p[:2]).T)
            covered |= (np.abs(a) <= half_u) & (np.abs(b) <= half_v)
        return float(covered.mean())



@dataclass(frozen=True)
class Cloth(Box):
    """True cloth: a PhysX surface deformable, a triangle mesh whose every node moves, so it drapes, folds and
    wrinkles anywhere (unlike :class:`Towel`, which is rigid tiles). Use it when handling fabric is the skill.

    ``size`` is the cloth as it starts, before ``yaw``: its footprint (x, y) and the height of the stack. ``pos`` is
    the centre of that box. With ``quarter=True`` the cloth is ONE sheet of (2 * size[0], 2 * size[1]) that starts
    folded in half twice, four layers ``layer_gap`` apart: the first fold's two folded edges run along the start
    shape's +x side, the second fold, which wraps all four layers, along its +y side, and the -x and -y sides are
    loose edges. With ``quarter=False`` it is a single flat layer of ``size``. The start shape is also the rest
    shape, so the folds stay as pressed creases.

    ``edge`` is the mesh spacing (m): smaller bends rounder but costs simulation time (nodes = sheet area / edge^2).
    ``thickness``, ``density``, ``youngs``, ``poisson``, ``bend`` and the dampings are the fabric; ``friction`` is its
    static and dynamic friction. ``rest_offset`` is half the gap two layers keep between them (self-collision is on
    for a folded cloth), ``contact_offset`` how far out contacts are generated. It has no ``mass`` (its mass comes
    from ``density``), so yam_arena never jitters it. Status: offline only; it has not run in Isaac Sim."""

    size: tuple[float, float, float] = (0.28, 0.23, 0.010)
    color: Color = (0.95, 0.95, 0.93)
    quarter: bool = True
    edge: float = 0.007
    thickness: float = 0.0006
    layer_gap: float = 0.003
    density: float = 440.0  # kg/m^3: 244 g/m^2 fabric 0.55 mm thick
    youngs: float = 1.0e6  # Pa; Isaac Lab's cloth example value
    poisson: float = 0.25
    bend: float = 1.0e3  # surface bend stiffness. Isaac Lab's example value (1e6) makes a folded cloth spring open like a book
    damping: float = 0.1
    bend_damping: float = 0.1
    friction: float = 0.6
    rest_offset: float = 0.001
    contact_offset: float = 0.004
    iterations: int = 24
    self_collide: bool = False  # a single flat sheet: turn on if it will be folded or rolled onto itself

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Cloth takes no mass; set density and thickness")
        if not 0.002 <= self.edge <= 0.03:
            raise ValueError(f"{self.name}: edge {self.edge} m is outside 2 to 30 mm")
        if self.quarter and self.layer_gap < 2 * self.rest_offset:
            raise ValueError(f"{self.name}: layer_gap {self.layer_gap} is under twice rest_offset {self.rest_offset}, "
                             "so the layers would start pressed into each other")

    @property
    def sheet(self) -> tuple[float, float]:
        """The unfolded sheet's size (m)."""
        k = 2 if self.quarter else 1
        return k * self.size[0], k * self.size[1]

    @property
    def cells(self) -> tuple[int, int]:
        """Mesh cells along the sheet's two sides (even when folded, so each fold line is a line of nodes)."""
        step = 2 if self.quarter else 1
        return tuple(max(step, step * round(side / self.edge / step)) for side in self.sheet)

    @property
    def node_count(self) -> int:
        return (self.cells[0] + 1) * (self.cells[1] + 1)

    @property
    def sheet_mass(self) -> float:
        """kg, from density, thickness and the sheet's area."""
        return self.density * self.thickness * self.sheet[0] * self.sheet[1]

    def mesh(self, world: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """(nodes (n, 3), triangles (m, 3)) of the start shape. ``world=False`` gives the part frame (before yaw,
        about ``pos``). Node (i, j) of the flat sheet is row ``i * (cells[1] + 1) + j``."""
        (cu, cv), (su, sv) = self.cells, self.sheet
        u = np.linspace(0.0, su, cu + 1)[:, None] * np.ones((1, cv + 1))
        v = np.ones((cu + 1, 1)) * np.linspace(0.0, sv, cv + 1)[None, :]
        if self.quarter:
            hu, hv = su / 2, sv / 2
            a = np.where(np.isclose(u, hu), 0.5, (u > hu).astype(float))  # which side of the first fold (0.5: on it)
            b = np.where(np.isclose(v, hv), 0.5, (v > hv).astype(float))  # which side of the second fold
            layer = np.where(b == 0.5, 1.5, np.where(b == 0.0, a, 3.0 - a))
            x, y, z = np.minimum(u, su - u) - hu / 2, np.minimum(v, sv - v) - hv / 2, (layer - 1.5) * self.layer_gap
            # Round the folds so no two nodes coincide (PhysX rejects such a mesh): the first fold's lines bulge half
            # a gap; on the second fold the outer pair of layers (0 and 3) turns outside the inner pair (1 and 2).
            x = x + np.where(a == 0.5, 0.5 * self.layer_gap, 0.0)
            y = y + np.where(b == 0.5, (1.5 - a) * self.layer_gap, 0.0)
        else:
            x, y, z = u - su / 2, v - sv / 2, np.zeros_like(u)
        nodes = np.stack([x, y, z], -1).reshape(-1, 3)
        idx = np.arange((cu + 1) * (cv + 1)).reshape(cu + 1, cv + 1)
        p, q, r, t = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
        faces = np.concatenate([np.stack([p, q, r], 1), np.stack([p, r, t], 1)])
        if world:
            nodes = nodes @ _yaw_matrix(self.yaw).T + np.asarray(self.pos, float)
        return nodes, faces

    def side_nodes(self, side: str) -> np.ndarray:
        """Rows (into the nodes) of the nodes along one side of the folded start shape, before yaw: ``"+y"`` is the
        fold that wraps all four layers, ``"+x"`` the first fold's edges, ``"-x"`` and ``"-y"`` the loose edges."""
        nodes, _ = self.mesh(world=False)
        axis, sign = "xy".index(side[1]), 1.0 if side[0] == "+" else -1.0
        return np.flatnonzero(sign * nodes[:, axis] > self.size[axis] / 2 - self.edge * 0.6)

    def state_at(self, nodes=None, node_vel=None) -> ClothState:
        """The state with the nodes at ``nodes`` (n, 3), moving at ``node_vel``; by default as it starts, still."""
        pts = self.mesh()[0] if nodes is None else np.asarray(nodes, float).reshape(-1, 3)
        vel = np.zeros_like(pts) if node_vel is None else np.asarray(node_vel, float).reshape(-1, 3)
        fastest = vel[np.argmax(np.linalg.norm(vel, axis=1))]
        return ClothState(pos=pts.mean(0), quat_xyzw=_yaw_quat(self.yaw), lin_vel=fastest, ang_vel=np.zeros(3),
                          low=pts.min(0), high=pts.max(0), nodes=pts, node_vel=vel)


@dataclass(frozen=True)
class ClothState(ObjectState):
    """A cloth: ``nodes`` (n, 3) are its mesh nodes and ``node_vel`` their velocities; ``pos`` is their mean,
    ``low``/``high`` bound them, and ``lin_vel`` is the fastest node's, so ``resting()`` means every node is still."""

    nodes: np.ndarray | None = None
    node_vel: np.ndarray | None = None

    def fraction_inside(self, center, half_xy, z_range) -> float:
        """The share of nodes :func:`inside` the region."""
        return float(np.mean([inside(p, center, half_xy, z_range) for p in self.nodes]))

    def covers(self, point, radius: float = 0.01) -> tuple[bool, bool]:
        """(cloth below, cloth above): whether any node lies within ``radius`` (x, y) of ``point`` and lower, or
        higher, than it. A wrapped object has cloth both below and above all along it."""
        near = self.nodes[np.linalg.norm(self.nodes[:, :2] - np.asarray(point, float)[:2], axis=1) <= radius]
        return bool((near[:, 2] < point[2]).any()), bool((near[:, 2] > point[2]).any())


@dataclass(frozen=True)
class Bag(Cloth):
    """A plastic bag (a bin liner): one surface-deformable film, a tube ``width`` across when flat and ``length``
    long, sealed along its bottom and open at its mouth, lying flat with its two layers ``layer_gap`` apart. Before
    ``yaw`` it lies along x with its mouth at -x. ``folds`` (0, 1 or 2) folds it lengthwise in half that many times,
    as a liner comes off its roll; each fold lays the +y part over the top. ``pos`` is the centre of the start shape's
    footprint, its bottom on ``pos[2]`` minus half ``size[2]``; ``size`` must be the start shape's bounding box (checked).

    The two sides of the flat tube are folds of the film (as on a bag with no side seams); the bottom row of nodes is
    shared by both layers, which is the seal. :meth:`hem_nodes` are the mouth's edge, :meth:`bottom_nodes` the seal.
    Mass comes from ``density`` and ``thickness``: real liner film is 6 to 15 microns; the simulated film may be
    thicker (a stiffer, heavier-duty liner), with density lowered to keep the real bag's mass."""

    width: float = 0.38
    length: float = 0.43
    folds: int = 0
    quarter: bool = False
    self_collide: bool = True

    def _check(self) -> None:
        if self.quarter:
            raise ValueError(f"{self.name}: a Bag is folded with folds, not quarter")
        if self.folds not in (0, 1, 2):
            raise ValueError(f"{self.name}: folds must be 0, 1 or 2")
        super()._check()
        lo, hi = self._start_nodes().min(0), self._start_nodes().max(0)
        if not np.allclose(hi - lo, self.size, atol=0.004):
            raise ValueError(f"{self.name}: size {self.size} is not the start shape's box {tuple((hi - lo).round(4))}")

    @property
    def sheet(self) -> tuple[float, float]:
        return 2 * self.width, self.length

    @property
    def cells(self) -> tuple[int, int]:
        """(cells round the tube, cells along it). Round the tube is a multiple of 8, so the sides and both fold lines
        fall on node columns."""
        across = max(2, 2 * round(self.width / self.edge / 2))  # per layer: even
        across = 4 * max(1, round(across / 4))  # two folds put fold lines at quarter widths
        return 2 * across, max(2, round(self.length / self.edge))

    @property
    def node_count(self) -> int:
        nu, nv = self.cells
        return nu * nv + nu // 2 + 1  # nv rows round the tube, then the sealed bottom row (one layer's worth)

    def _grid(self):
        """Index of the node at (round, along) ``(i, j)``: i in 0..nu-1 round the tube from the -y side, along the
        front (top) layer to the +y side and back along the back layer; j in 0..nv from the mouth to the seal."""
        nu, nv = self.cells
        half = nu // 2

        def index(i: int, j: int) -> int:
            i %= nu
            if j < nv:
                return i * nv + j
            k = i if i <= half else nu - i  # back-layer seal nodes are the front's
            return nu * nv + k
        return index

    def _flat(self) -> np.ndarray:
        """Node positions of the flat bag (before folds), centred: x along, y across, z layer."""
        nu, nv = self.cells
        half, g = nu // 2, self.layer_gap
        pts = np.zeros((self.node_count, 3))
        index = self._grid()
        for j in range(nv + 1):
            x = -self.length / 2 + self.length * j / nv
            for i in range(nu):
                if j == nv and i > half:
                    continue
                if i <= half:  # front layer, then the +y side fold at i == half
                    y = -self.width / 2 + self.width * i / half
                    z = 0.0 if (i in (0, half) or j == nv) else g / 2
                else:
                    y = self.width / 2 - self.width * (i - half) / half
                    z = -g / 2
                pts[index(i, j)] = (x, y, z)
        return pts

    def _start_nodes(self) -> np.ndarray:
        """The start shape (before yaw, about ``pos``): the flat bag with its lengthwise folds, recentred so its
        footprint is centred at the origin and its bottom at -size[2] / 2."""
        p = self._flat()
        g = self.layer_gap
        lines = [0.0, -self.width / 4][: self.folds]
        for f in lines:
            top = p[:, 2].max()
            over = p[:, 1] > f + 1e-9
            on = np.isclose(p[:, 1], f)
            p[over, 1] = 2 * f - p[over, 1]
            p[over, 2] = 2 * top + g - p[over, 2]
            p[on, 2] = p[on, 2] + (top + g / 2 - p[on, 2]) * 0.5  # round the fold: its line rises halfway
            p[on, 1] = p[on, 1] + g  # and bulges out past the stack's edge
        lo, hi = p.min(0), p.max(0)
        return p - (lo + hi) / 2

    def mesh(self, world: bool = True) -> tuple[np.ndarray, np.ndarray]:
        nu, nv = self.cells
        index = self._grid()
        faces = []
        for i in range(nu):
            for j in range(nv):
                a, b, c, d = index(i, j), index(i + 1, j), index(i + 1, j + 1), index(i, j + 1)
                for tri in ((a, b, c), (a, c, d)):
                    if len(set(tri)) == 3:
                        faces.append(tri)
        nodes = self._start_nodes()
        if world:
            nodes = nodes @ _yaw_matrix(self.yaw).T + np.asarray(self.pos, float)
        return nodes, np.array(faces)

    def hem_nodes(self) -> np.ndarray:
        """Rows of the nodes round the mouth's edge, in order round it."""
        index = self._grid()
        return np.array([index(i, 0) for i in range(self.cells[0])])

    def bottom_nodes(self) -> np.ndarray:
        """Rows of the nodes along the bottom seal."""
        index = self._grid()
        return np.array([index(i, self.cells[1]) for i in range(self.cells[0] // 2 + 1)])



@dataclass(frozen=True)
class Custom(Box):
    """An object the task file builds and reads itself (for a mechanism the other kinds can't express, such as a
    chain of free bodies on joints). ``build(stage, prim_path, origin)`` authors its prims under ``prim_path`` (an
    Xform already at ``pos``, the world origin of its frame); ``reader(environment, spec, prim_root)`` returns its
    state each step; ``starter(spec)`` its state at the start (offline and for fixed reads). It has no mass (it is
    spawned as a static asset whose bodies PhysX simulates), so yam_arena never jitters it. ``size`` is its bounding
    box about ``pos`` (checks only)."""

    build: object = None
    reader: object = None
    starter: object = None

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Custom object takes no mass")
        if not (callable(self.build) and callable(self.reader) and callable(self.starter)):
            raise ValueError(f"{self.name}: give build, reader and starter callables")


KitObject = Container | Marked | Model | Shaped | Button | Lamp | Hinge | Slider | Hand | Towel | Cloth | Custom


# -- scoring helpers ---------------------------------------------------------------------------------------------------


def tilt_deg(quat_xyzw) -> float:
    """Angle between a body's z axis and the vertical (degrees)."""
    x, y, _, _ = quat_xyzw
    return math.degrees(math.acos(min(1.0, max(-1.0, 1.0 - 2.0 * (x * x + y * y)))))


def lifted(state: ObjectState, spec, dz: float = 0.02) -> bool:
    """Risen more than ``dz`` (m) above where ``spec`` starts."""
    return float(state.pos[2]) - float(spec.pos[2]) > dz


def moved(state: ObjectState, spec, d: float = 0.02) -> bool:
    """Shifted more than ``d`` (m) across the table from where ``spec`` starts."""
    return float(np.hypot(state.pos[0] - spec.pos[0], state.pos[1] - spec.pos[1])) > d


def inside(point, center, half_xy, z_range) -> bool:
    """``point`` within ``half_xy`` (x, y half widths, m) of ``center`` (x, y) and between ``z_range`` heights."""
    x, y, z = (float(v) for v in point)
    return bool(abs(x - center[0]) < half_xy[0] and abs(y - center[1]) < half_xy[1] and z_range[0] < z < z_range[1])


def count_inside(objects, names, center, half_xy, z_range) -> int:
    """How many of ``names`` have their centre :func:`inside` the region."""
    return sum(inside(objects[n].pos, center, half_xy, z_range) for n in names)


def all_resting(objects, names, max_speed: float = 0.01, max_spin: float = 0.5) -> bool:
    return all(objects[n].resting(max_speed=max_speed, max_spin=max_spin) for n in names)


# -- demo waypoints with their own rotation ------------------------------------------------------------------------------


_GRIPS: dict = {}  # arm -> TCP (world, m), read with the objects every step; empty offline unless a test sets it


def gripper_points() -> dict:
    """Each arm's TCP (world, m) as of the last time the objects were read (empty if never read)."""
    return dict(_GRIPS)


def set_gripper_points(**points) -> None:
    """Set the TCPs :func:`released` checks (offline tests): ``set_gripper_points(left=(x, y, z), right=...)``."""
    _GRIPS.clear()
    _GRIPS.update({k: np.asarray(v, float) for k, v in points.items()})


def released(state: ObjectState, clear: float = 0.02) -> bool:
    """No gripper holds the object: every known TCP is more than ``clear`` (m) outside its bounding box. The pad tips
    reach 13 mm past the TCP, so 2 cm keeps them off it. True if no TCP has been read (nothing to check)."""
    lo, hi = np.asarray(state.low, float), np.asarray(state.high, float)
    for p in _GRIPS.values():
        if np.linalg.norm(np.maximum(0.0, np.maximum(lo - p, p - hi))) <= clear:
            return False
    return True


@dataclass
class Pose(Waypoint):
    """A demo waypoint whose TCP rotation (3x3, world frame) is ``rot`` instead of a straight-down grip."""

    rot: np.ndarray | None = None


def forward_grip(yaw_deg: float = 0.0) -> np.ndarray:
    """The TCP rotation of a grip pointing horizontally forward (+x, turned ``yaw_deg`` about the vertical), its
    fingers closing along world y: the straight-down grip pitched up 90 degrees."""
    return axis_angle_matrix(Z_AXIS, math.radians(yaw_deg)) @ axis_angle_matrix(Y_AXIS, -math.pi / 2) @ grip_rotation()


def rolled(rot: np.ndarray, deg: float) -> np.ndarray:
    """``rot`` rolled ``deg`` about the world x axis: for a forward grip, a turn of the wrist that tips what it holds
    sideways (positive tips its top toward -y)."""
    return axis_angle_matrix(X_AXIS, math.radians(deg)) @ rot


def ready_pose(arm: str, duration: float = 1.5, label: str = "back to ready") -> Pose:
    """A waypoint at the arm's ready pose. The IK is warm-started, so passing through it also puts the arm back on
    the joint branch a forward grip needs after a straight-down move."""
    from yam_arena import station
    from yam_arena.constants import FINGER_OPEN, READY

    pos, rot = ik.tcp_pose(np.array(READY, dtype=float), np.array(station.BASES[arm]))
    return Pose(duration, tuple(pos), FINGER_OPEN, label, rot=rot)


@dataclass
class Branch:
    """A demo step that plays ``options[key]`` once reached, where ``key`` is the lamp group lit at that moment
    (:func:`lit_group`; ``default`` if none is lit), whether a button or a timed switch (:func:`switch_lamp_group`)
    lit it. Every option is padded to the longest one's duration by holding its last pose, so the plan's length
    doesn't depend on the answer. The choice is made once per trial (demo() builds a fresh Branch every trial)."""

    options: dict
    default: str

    def __post_init__(self) -> None:
        if self.default not in self.options:
            raise ValueError(f"Branch default {self.default!r} is not one of {sorted(self.options)}")
        self._chosen = None

    @property
    def duration(self) -> float:
        return max(sum(w.duration for w in ws) for ws in self.options.values())

    def chosen(self) -> list:
        if self._chosen is None:
            key = lit_group() or self.default
            ws = list(self.options.get(key, self.options[self.default]))
            pad = self.duration - sum(w.duration for w in ws)
            if pad > 1e-9 and ws:
                last = ws[-1]
                ws.append(Pose(pad, last.pos, last.finger, "hold", rot=getattr(last, "rot", None)))
            self._chosen = ws
        return self._chosen


@dataclass
class Live:
    """A demo step planned from what the simulator shows when the arm reaches it: ``plan(objects)`` gets the latest
    object states (name -> state, as score() gets them; offline, every object's start state) and returns the poses
    to play, which are padded to ``duration`` by holding the last one (the plan's length stays fixed). The review
    rules allow a scripted demo to read simulator state to plan; it never moves objects. Planned once per trial."""

    duration: float
    plan: object  # callable(objects) -> list of Pose
    label: str = "live"

    def __post_init__(self) -> None:
        self._chosen = None

    def chosen(self) -> list:
        if self._chosen is None:
            objects = dict(_LATEST) or {n: start_state(spec) for n, spec in _SPECS.items()}
            ws = list(self.plan(objects))
            total = sum(w.duration for w in ws)
            if total > self.duration + 1e-9:
                raise ValueError(f"{self.label}: its plan takes {total:.2f} s, more than its {self.duration} s")
            if self.duration - total > 1e-9 and ws:
                last = ws[-1]
                ws.append(Pose(self.duration - total, last.pos, last.finger, "hold", rot=getattr(last, "rot", None)))
            self._chosen = ws
        return self._chosen


def latest_objects() -> dict:
    """The object states as last read in the simulator (empty offline)."""
    return dict(_LATEST)


def _flat(waypoints, t: float):
    """The waypoints to walk at time ``t``: a :class:`Branch` is replaced by its chosen option once reached."""
    out, elapsed = [], 0.0
    for w in waypoints:
        if isinstance(w, Branch | Live):
            if t < elapsed:  # not reached yet: don't choose before the arm gets there
                break
            out.extend(w.chosen())
        else:
            out.append(w)
        elapsed += w.duration
    return out


def _plan_target(self, t: float):
    """``ArmPlan.target`` that lets a :class:`Pose` waypoint bring its own rotation, and plays :class:`Branch`."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in _flat(self.waypoints, t):
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            finger = prev_finger + s * (w.finger - prev_finger)
            return prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s), finger, w.label
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


demo.ArmPlan.target = _plan_target

# -- trial bookkeeping ------------------------------------------------------------------------------------------------

_SPECS: dict[str, KitObject] = {}
_LATEST: dict = {}  # the object states as last read (Live demo steps plan from them)
_RESET: list = []
_LIT = {"on": False}
_GROUP: dict = {"active": None}
_SEED: dict = {"seed": None}
_CLOCK: dict = {"t": 0.0}
_SWITCHES: list = []  # this trial's timed lamp switches: [at (s), group, applied]
_LATCH: dict = {}  # hinge name -> latched at the last step
_GLOVE: dict = {}  # hand name -> glove on this trial (absent: the Hand's ``gloved``)
STEP_S = 0.1  # the control period, if the embodiment doesn't say (yam_arena runs at 10 Hz)


def set_lamp_group(group: str | None) -> None:
    """Make ``group`` the lamp group that lights when a button latches this trial (call it from an on_reset hook)."""
    _GROUP["active"] = group


def lit_group() -> str | None:
    """The lamp group lit right now (None before any button latches or timed switch): what the cameras show."""
    return _GROUP["active"] if _LIT["on"] else None


def trial_seed():
    """The seed the current trial's reset used, or None for an unseeded reset."""
    return _SEED["seed"]


def trial_time() -> float:
    """Seconds since this trial's reset (the embodiment's clock, advanced every control step)."""
    return _CLOCK["t"]


def switch_lamp_group(group: str | None, at: float) -> None:
    """At ``at`` seconds into this trial, light lamp ``group`` and put out the rest (``group`` None: put every lamp
    out), with or without a button. Call it from an on_reset hook: every reset clears the schedule. Several calls
    queue several switches (green at 0, red at 20). The lamps change on the first control step at or after ``at``
    and the cameras show it from the step after, so a demo :class:`Branch` that reads it should come at least 0.2 s
    later. A task that lights lamps by timer should give every lamp a ``group``, and not also light them by button."""
    at = float(at)
    if not (math.isfinite(at) and at >= 0):
        raise ValueError(f"switch time must be a finite number of seconds >= 0, got {at!r}")
    _SWITCHES.append([at, group, False])
    _SWITCHES.sort(key=lambda s: s[0])
    tick(_CLOCK["t"])  # one due now (at 0, from the reset hook) applies at once


def lamp_switches() -> list[tuple[float, str | None]]:
    """This trial's timed lamp switches as (time, group), in time order."""
    return [(at, group) for at, group, _ in _SWITCHES]


def lamp_group_at(t: float) -> str | None:
    """The group this trial's timed switches have lit by ``t`` seconds (None before the first, or with none)."""
    lit = None
    for at, group, _ in _SWITCHES:
        if at <= t + 1e-9:
            lit = group
    return lit


def tick(t: float, environment=None) -> bool:
    """Move the trial clock to ``t`` seconds and apply the timed lamp switches now due; True if one applied. Every
    control step calls it in the simulator; offline tests call it to play a trial's clock."""
    _CLOCK["t"] = float(t)
    changed = False
    for switch in _SWITCHES:
        at, group, applied = switch
        if not applied and at <= t + 1e-9:
            switch[2] = changed = True
            _GROUP["active"], _LIT["on"] = group, group is not None
    if changed and environment is not None:
        _show_lamps(environment, _LIT["on"])
    return changed


def hinge_step(spec: Hinge, angle: float) -> HingeState:
    """The hinge's state at ``angle`` (rad), its latch carried over from the last call this trial: what every
    control step does with the joint's angle. Offline tests call it to play a lid's angles through a trial."""
    was = _LATCH.get(spec.name, False)
    latched = spec.latch is not None and spec.latch.step(angle, was)
    _LATCH[spec.name] = latched
    return spec.state_at(angle, latched)


def set_glove(hand: Hand | str, on: bool) -> None:
    """Show or hide ``hand``'s glove this trial (call it from an on_reset hook; every reset restores its
    ``gloved``)."""
    _GLOVE[hand if isinstance(hand, str) else hand.name] = bool(on)


def glove_on(hand: Hand) -> bool:
    """Whether ``hand``'s glove shows this trial."""
    return _GLOVE.get(hand.name, hand.gloved)


def on_reset(fn):
    """Run ``fn()`` after every reset, before the trial's first step."""
    _RESET.append(fn)
    return fn


@on_reset
def _start_trial() -> None:
    """Clear the last trial's clock, timed switches, latches and gloves (registered first, so a task's own on_reset
    hooks run after it)."""
    _CLOCK["t"] = 0.0
    _GRIPS.clear()
    _LATEST.clear()
    _SWITCHES.clear()
    _LATCH.clear()
    _GLOVE.clear()
    _LIT["on"] = False


class Trial(dict):
    """Per-trial flags that start at ``defaults`` and are restored at every reset:
    ``TRIAL = Trial(pressed=False, early=False)``, then ``TRIAL["pressed"] = True`` inside ``score()``."""

    def __init__(self, **defaults):
        super().__init__(defaults)
        self._defaults = dict(defaults)
        on_reset(self.reset)

    def reset(self) -> None:
        self.clear()
        self.update(self._defaults)


# -- Arena objects (Isaac Sim) ----------------------------------------------------------------------------------------


def _look(stage, path: str, color: Color, roughness: float = 0.5) -> str:
    import isaaclab.sim as sim_utils

    from yam_arena.station import linear_rgb

    vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=roughness)
    vis.func(path, vis)
    return path


def _cube(stage, path: str, size, at, look: str | None, physics: str | None = None, offset: float = 0.005) -> None:
    """A box under a body: a collider (contacts from ``offset`` out) if ``physics`` (a physics material) is given,
    else visual only."""
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in at)))
    cube.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in size)))
    if physics is not None:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(offset)
        contact.CreateRestOffsetAttr(0.0)
        bind_physics_material(path, physics, stage=stage)
    if look is not None:
        bind_visual_material(path, look, stage=stage)


def _grippy(prim_path: str, friction: tuple[float, float] | None = None) -> str:
    import isaaclab.sim as sim_utils

    from yam_arena.constants import GRIPPY_FRICTION

    static, dynamic = friction or GRIPPY_FRICTION
    physics = f"{prim_path}/Looks/physics"
    grippy = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
    grippy.func(physics, grippy)
    return physics


def _container_parts(prim_path: str, spec: Container) -> None:
    """The four walls (colliders) and the label band (visual) on a container whose floor is already spawned."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    sx, sy, h = spec.size
    w = spec.wall
    tall = h - w
    zc = w / 2 + tall / 2
    body = _look(stage, f"{prim_path}/Looks/body", spec.color, 0.4)
    physics = _grippy(prim_path)
    walls = {
        "front": ((w, sy, tall), (-(sx - w) / 2, 0.0, zc)),
        "back": ((w, sy, tall), ((sx - w) / 2, 0.0, zc)),
        "left": ((sx - 2 * w, w, tall), (0.0, (sy - w) / 2, zc)),
        "right": ((sx - 2 * w, w, tall), (0.0, -(sy - w) / 2, zc)),
    }
    for name, (size, at) in walls.items():
        _cube(stage, f"{prim_path}/walls/{name}", size, at, body, physics)
    if spec.label is None:
        return
    label = _look(stage, f"{prim_path}/Looks/label", spec.label, 0.6)
    lo, hi = spec.band
    band_h, band_z = hi - lo, (lo + hi) / 2 - w / 2
    for name, (size, at) in {
        "front": ((PROUD, sy, band_h), (-(sx + PROUD) / 2, 0.0, band_z)),
        "back": ((PROUD, sy, band_h), ((sx + PROUD) / 2, 0.0, band_z)),
        "left": ((sx, PROUD, band_h), (0.0, (sy + PROUD) / 2, band_z)),
        "right": ((sx, PROUD, band_h), (0.0, -(sy + PROUD) / 2, band_z)),
    }.items():
        _cube(stage, f"{prim_path}/band/{name}", size, at, label)


def _mark_parts(prim_path: str, spec: Marked) -> None:
    """The visual patches on a marked box whose body is already spawned."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for i, m in enumerate(spec.marks):
        size, at = spec.mark_box(m)
        _cube(stage, f"{prim_path}/marks/m{i}_{m.face}", size, at, _look(stage, f"{prim_path}/Looks/mark{i}", m.color))


def _button_parts(prim_path: str, spec: Button, translation=None, orientation=None):
    """The button as an articulation: housing body, cap body, and the cap's sprung prismatic joint."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    physics = _grippy(prim_path)

    def body(path: str, at, mass: float):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp().Set(Gf.Vec3d(*at))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr(mass)

    housing, cap = f"{prim_path}/housing", f"{prim_path}/cap"
    rest = (0.0, 0.0, spec.size[2] / 2 + spec.cap[2] / 2)
    body(housing, (0.0, 0.0, 0.0), spec.mass)
    _cube(stage, f"{housing}/box", spec.size, (0.0, 0.0, 0.0), _look(stage, f"{prim_path}/Looks/housing", spec.color),
          physics)
    body(cap, rest, spec.cap_mass)
    _cube(stage, f"{cap}/box", spec.cap, (0.0, 0.0, 0.0), _look(stage, f"{prim_path}/Looks/cap", spec.cap_color),
          physics)
    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{cap}/press")
    joint.CreateAxisAttr("Z")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(housing)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(cap)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*rest))
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLowerLimitAttr(-spec.travel)
    joint.CreateUpperLimitAttr(0.0)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(spec.rate)
    drive.CreateDampingAttr(spec.damping)
    drive.CreateTargetPositionAttr(0.0)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(housing))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(housing))
    art.CreateEnabledSelfCollisionsAttr(False)  # the cap sinks into the housing; the joint limit stops it
    art.CreateSolverPositionIterationCountAttr(32)
    return root.GetPrim()


def _hand_parts(prim_path: str, spec: Hand) -> None:
    """Fingers, thumb and forearm (colliders) and the glove (visual, shown or hidden at reset) on a hand whose palm
    is already spawned."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    skin = _look(stage, f"{prim_path}/Looks/skin", spec.color, 0.6)
    physics = _grippy(prim_path)
    UsdGeom.Xform.Define(stage, f"{prim_path}/skin")
    for name, (at, size) in spec.parts().items():
        if name != "palm":  # the palm is the body's own box
            _cube(stage, f"{prim_path}/skin/{name}", size, at, skin, physics)
    glove = _look(stage, f"{prim_path}/Looks/glove", spec.glove, 0.35)
    shell = UsdGeom.Xform.Define(stage, f"{prim_path}/glove")
    for name, (at, size) in spec.glove_parts().items():
        _cube(stage, f"{prim_path}/glove/{name}", size, at, glove)
    if not spec.gloved:
        UsdGeom.Imageable(shell.GetPrim()).MakeInvisible()


def _root_xform(stage, prim_path: str, translation, orientation):
    from pxr import Gf, UsdGeom

    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)  # Isaac Lab 3: (x, y, z, w)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    return root


def _rigid(stage, path: str, at, rot: np.ndarray, mass: float, gravity: bool = True):
    """A rigid body Xform at ``at`` turned by ``rot`` (3x3) in its parent's frame."""
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    x, y, z, w = ik.matrix_to_quat_xyzw(rot)
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(float(v) for v in at)))
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(w), float(x), float(y),
                                                                               float(z)))
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
    physx = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    if not gravity:
        physx.CreateDisableGravityAttr(True)
    return physx


def _weld_to_world(stage, prim_path: str, body: str) -> None:
    """Weld ``body`` to the world where it stands, making its articulation fixed-base. The articulation root API
    sits on the parent Xform: PhysX treats a fixed joint on a root-API body as an ordinary joint, not a fixed base
    (Isaac Lab's fix_root_link does the same)."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    weld = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld")
    weld.CreateBody1Rel().SetTargets([Sdf.Path(body)])
    world = UsdGeom.Xformable(stage.GetPrimAtPath(body)).ComputeLocalToWorldTransform(0)
    t, q = world.ExtractTranslation(), world.ExtractRotationQuat()
    weld.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in t)))
    weld.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()), *(float(v) for v in q.GetImaginary())))
    weld.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    weld.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))


def _jointed_parts(prim_path: str, spec: Hinge | Slider, translation=None, orientation=None):
    """A hinge or slider as a fixed-base articulation: the base welded to the world, the lid or panel on a revolute
    ("hinge") or prismatic ("slide") joint whose zero is ``spec.rest``, so a zero drive target is the spring's rest."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    stage = get_current_stage()
    root = _root_xform(stage, prim_path, translation, orientation)
    physics = _grippy(prim_path)
    base = f"{prim_path}/base"
    _rigid(stage, base, (0.0, 0.0, 0.0), np.eye(3), BASE_MASS)
    _cube(stage, f"{base}/box", spec.size, (0.0, 0.0, 0.0), _look(stage, f"{prim_path}/Looks/base", spec.color),
          physics)
    hinge = isinstance(spec, Hinge)
    if hinge:
        part, size, color, mass = f"{prim_path}/lid", spec.lid, spec.lid_color, spec.lid_mass
        _rigid(stage, part, spec.pivot_point, _turn(spec.axis, spec.start), mass)
        _cube(stage, f"{part}/box", size, spec.lid_offset, _look(stage, f"{prim_path}/Looks/lid", color), physics)
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{part}/hinge")
        at0, rot0 = spec.pivot_point, ik.matrix_to_quat_xyzw(_turn(spec.axis, spec.rest))
        lower, upper = (math.degrees(v - spec.rest) for v in spec.limits)  # USD angles are degrees
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateStiffnessAttr((spec.spring or 0.0) * math.pi / 180)  # USD angular gains are per degree
        drive.CreateDampingAttr(spec.damping * math.pi / 180)
    else:
        part, size, color, mass = f"{prim_path}/panel", spec.panel, spec.panel_color, spec.panel_mass
        at = spec.panel_offset + spec.start * AXES[spec.axis]
        _rigid(stage, part, at, np.eye(3), mass, gravity=not spec.balanced)
        _cube(stage, f"{part}/box", size, (0.0, 0.0, 0.0), _look(stage, f"{prim_path}/Looks/panel", color), physics)
        joint = UsdPhysics.PrismaticJoint.Define(stage, f"{part}/slide")
        at0, rot0 = spec.panel_offset + spec.rest * AXES[spec.axis], np.array([0.0, 0.0, 0.0, 1.0])
        lower, upper = (v - spec.rest for v in spec.limits)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateStiffnessAttr(spec.spring or 0.0)
        drive.CreateDampingAttr(spec.damping)
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    joint.CreateAxisAttr(spec.axis.upper())
    joint.CreateBody0Rel().SetTargets([Sdf.Path(base)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(part)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in at0)))
    joint.CreateLocalRot0Attr(Gf.Quatf(float(rot0[3]), float(rot0[0]), float(rot0[1]), float(rot0[2])))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLowerLimitAttr(float(lower))
    joint.CreateUpperLimitAttr(float(upper))
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(False)  # lid and base may touch at the hinge; the joint limits are the stops
    art.CreateSolverPositionIterationCountAttr(32)
    _weld_to_world(stage, prim_path, base)
    return root.GetPrim()


def _towel_parts(prim_path: str, spec: Towel, translation=None, orientation=None):
    """The towel as a floating articulation rooted at its centre tile (``translation``): a tree of edge joints, plus
    loop-closing joints kept out of the articulation."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    stage = get_current_stage()
    root = _root_xform(stage, prim_path, translation, orientation)
    look = _look(stage, f"{prim_path}/Looks/cloth", spec.color, 0.8)
    physics = f"{prim_path}/Looks/physics"
    cloth = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    cloth.func(physics, cloth)
    px, py = spec.pitch
    ic, jc = spec.centre_tile
    n = spec.tiles[0] * spec.tiles[1]

    def tile(i: int, j: int) -> str:
        return f"{prim_path}/{spec.tile_name(i, j)}"

    for i, j in spec.tile_ids():
        body = _rigid(stage, tile(i, j), ((i - ic) * px, (j - jc) * py, 0.0), np.eye(3), spec.mass / n)
        body.CreateMaxDepenetrationVelocityAttr(1.0)
        body.CreateLinearDampingAttr(0.5)
        body.CreateAngularDampingAttr(2.0)
        _cube(stage, f"{tile(i, j)}/box", (px - spec.gap, py - spec.gap, spec.size[2]), (0.0, 0.0, 0.0), look,
              physics, offset=0.002)

    quarter = Gf.Quatf(math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    for (a, b, in_tree) in spec.edges():
        along_x = a[1] == b[1]
        sign = 1.0 if (b[0] - a[0] if along_x else b[1] - a[1]) > 0 else -1.0
        half = np.array([px / 2, 0.0, 0.0]) if along_x else np.array([0.0, py / 2, 0.0])
        joint = UsdPhysics.Joint.Define(stage, f"{tile(*b)}/{'edge' if in_tree else 'loop'}_{a[0]}_{a[1]}")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(tile(*a))])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(tile(*b))])
        frame = Gf.Quatf(1, 0, 0, 0) if along_x else quarter  # joint x along the axis between the two tiles
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in sign * half)))
        joint.CreateLocalRot0Attr(frame)
        joint.CreateLocalPos1Attr(Gf.Vec3f(*(float(v) for v in -sign * half)))
        joint.CreateLocalRot1Attr(frame)
        if not in_tree:
            joint.CreateExcludeFromArticulationAttr(True)
        prim = joint.GetPrim()
        for dof in ("transX", "transY", "transZ", "rotZ"):  # no stretch, no shear in the cloth's plane
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)  # low > high locks the axis
        for dof, most in (("rotY", spec.max_fold), ("rotX", spec.max_twist)):  # fold about the edge; twist
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(-math.degrees(most))
            lim.CreateHighAttr(math.degrees(most))
            drive = UsdPhysics.DriveAPI.Apply(prim, dof)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(spec.stiffness * math.pi / 180)
            drive.CreateDampingAttr(spec.damping * math.pi / 180)
            drive.CreateTargetPositionAttr(0.0)

    # The root API on the centre tile itself, so it is the root link (on a plain Xform PhysX picks its own).
    centre = stage.GetPrimAtPath(tile(ic, jc))
    UsdPhysics.ArticulationRootAPI.Apply(centre)
    art = PhysxSchema.PhysxArticulationAPI.Apply(centre)
    art.CreateEnabledSelfCollisionsAttr(spec.self_collision)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _prim_shape(stage, path: str, prim: Prim, look: str | None, physics: str, visible: bool = True) -> None:
    """One :class:`Prim` collider under a body (or a fixed model), drawn in ``look`` only if ``visible``."""
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    x, y, z, w = (float(v) for v in matrix_to_quat_xyzw(prim.rot))
    if prim.kind == "box":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in prim.at)))
        geom.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
        geom.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in prim.size)))
    else:
        geom = (UsdGeom.Cylinder if prim.kind == "cylinder" else UsdGeom.Capsule).Define(stage, path)
        geom.CreateRadiusAttr(float(prim.size[0]))
        geom.CreateHeightAttr(float(prim.size[2]))
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in prim.at)))
        geom.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    contact = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
    contact.CreateContactOffsetAttr(0.002)
    contact.CreateRestOffsetAttr(0.0)
    bind_physics_material(path, physics, stage=stage)
    if not visible:
        UsdGeom.Imageable(geom.GetPrim()).MakeInvisible()  # still collides: visibility doesn't reach PhysX
    elif look is not None:
        bind_visual_material(path, look, stage=stage)


def _snap_path(prim_path: str, n: int) -> str:
    return f"{prim_path}/snap_{n}"


_WELDS: dict = {"n": 0}  # how many times the snaps have been (re)defined: each redefinition gets a new prim name


def _snap_joint(stage, prim_path: str, spec: Shaped, n: int = 0) -> str:
    """Weld ``spec``'s body to its ``snap_to`` body where both start (from their specs, so the order they spawn in
    doesn't matter), as a fixed joint with break thresholds."""
    from pxr import Gf, Sdf, UsdPhysics

    parent = _SPECS[spec.snap_to]
    parent_path = f"{prim_path.rsplit('/', 1)[0]}/{spec.snap_to}"
    rp, rc = parent.rot, spec.rot
    at0 = rp.T @ (np.asarray(spec.pos, float) - np.asarray(parent.pos, float))
    x, y, z, w = ik.matrix_to_quat_xyzw(rp.T @ rc)
    joint = UsdPhysics.FixedJoint.Define(stage, _snap_path(prim_path, n))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(prim_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in at0)))
    joint.CreateLocalRot0Attr(Gf.Quatf(float(w), float(x), float(y), float(z)))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateBreakForceAttr(float(min(spec.break_force, 3.0e38)))
    joint.CreateBreakTorqueAttr(float(min(spec.break_torque, 3.0e38)))
    return joint.GetPath().pathString


def _shaped_parts(prim_path: str, spec: Shaped, translation=None, orientation=None):
    """A :class:`Shaped` body: one rigid body (if loose) with its colliders, its mesh, and its snap joint."""
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import PhysxSchema, UsdGeom, UsdPhysics

    stage = get_current_stage()
    root = _root_xform(stage, prim_path, translation, orientation)
    prim = root.GetPrim()
    if spec.mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(spec.mass))
        body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        body.CreateSolverPositionIterationCountAttr(16)
        body.CreateMaxDepenetrationVelocityAttr(1.0)
    physics = _grippy(prim_path, spec.friction)
    look = _look(stage, f"{prim_path}/Looks/body", spec.color, 0.55)
    for i, part in enumerate(spec.parts):
        _prim_shape(stage, f"{prim_path}/collider_{i}", part, look, physics, visible=not spec.mesh)
    if spec.mesh:
        vis = UsdGeom.Xform.Define(stage, f"{prim_path}/visual")
        vis.GetPrim().GetReferences().AddReference(spec.mesh)
        bind_visual_material(f"{prim_path}/visual", look, stage=stage)
    if spec.snap_to is not None:
        _snap_joint(stage, prim_path, spec, _WELDS["n"])
    return prim


def _model_colliders(prim_path: str, spec: Model) -> None:
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    physics = _grippy(prim_path)
    for i, part in enumerate(spec.colliders):
        _prim_shape(stage, f"{prim_path}/collider_{i}", part, None, physics, visible=False)


def _cloth_object(spec: Cloth):
    """An Isaac Lab-Arena deformable object for ``spec``: its own folded mesh as a PhysX surface deformable."""
    import isaaclab.sim as sim_utils
    import trimesh
    from isaaclab.sim.spawners.meshes import meshes as mesh_spawners
    from isaaclab.sim.utils import clone, get_current_stage
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_arena.utils.pose import Pose as ArenaPose
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxSurfaceDeformableBodyMaterialCfg

    from yam_arena.station import linear_rgb

    nodes, faces = spec.mesh(world=False)
    centre = nodes.mean(0)  # Arena resets a deformable's nodes about its prim origin, so put the origin at the centroid

    @clone
    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        mesh = trimesh.Trimesh(vertices=nodes - centre, faces=faces, process=False)
        stage = get_current_stage()
        mesh_spawners._spawn_mesh_geom_from_mesh(prim_path, cfg, mesh, translation, orientation, None, stage=stage)
        if spec.quarter or isinstance(spec, Bag):  # a bag's side folds and lengthwise folds are creases too
            # By default PhysX takes a cloth's rest bend angles as flat, so a folded cloth with any stiffness springs
            # open. "restShapeDefault" takes them from the spawn shape instead: the folds become pressed creases.
            from pxr import Sdf, Usd

            done = 0
            for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
                if prim.GetAttribute("omniphysics:restShapePoints").IsValid():
                    attr = prim.GetAttribute("omniphysics:restBendAnglesDefault")
                    if not attr.IsValid():
                        attr = prim.CreateAttribute("omniphysics:restBendAnglesDefault", Sdf.ValueTypeNames.Token, False,
                                                    Sdf.VariabilityUniform)
                    attr.Set("restShapeDefault")
                    done += 1
            print(f"[labkit] cloth {spec.name}: rest bend angles taken from the folded shape on {done} prim(s)", flush=True)
        return stage.GetPrimAtPath(prim_path)

    material = PhysxSurfaceDeformableBodyMaterialCfg(
        density=spec.density, surface_thickness=spec.thickness, poissons_ratio=spec.poisson, youngs_modulus=spec.youngs,
        surface_bend_stiffness=spec.bend, elasticity_damping=spec.damping, bend_damping=spec.bend_damping,
        static_friction=spec.friction, dynamic_friction=spec.friction)
    cfg = sim_utils.MeshRectangleCfg(
        func=spawn, size=(float(spec.sheet[0]), float(spec.sheet[1])), edge_refinement=1.0, physics_material=material,
        deformable_props=PhysxDeformableBodyPropertiesCfg(solver_position_iteration_count=int(spec.iterations),
                                                          linear_damping=0.0,
                                                          self_collision=True if (spec.quarter or spec.self_collide) else None),
        collision_props=[PhysxCollisionCfg(rest_offset=spec.rest_offset, contact_offset=spec.contact_offset)],
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.8))
    origin = centre @ _yaw_matrix(spec.yaw).T + np.asarray(spec.pos, float)
    return DeformableObject(name=spec.name, spawner_cfg=cfg,
                            initial_pose=ArenaPose(position_xyz=tuple(float(v) for v in origin),
                                                   rotation_xyzw=tuple(float(v) for v in _yaw_quat(spec.yaw))))


def _custom_object(spec: Custom):
    """A :class:`Custom` object: a static Arena asset whose spawner calls the task's ``build``."""
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose as ArenaPose

    def spawn(prim_path, cfg, *args, translation=None, orientation=None, **kwargs):
        stage = get_current_stage()
        root = _root_xform(stage, prim_path, translation, orientation)
        spec.build(stage, prim_path, np.asarray(translation if translation is not None else spec.pos, float))
        return root.GetPrim()

    @configclass
    class CustomSpawnerCfg(SpawnerCfg):
        func: object = clone(spawn)

    pose = ArenaPose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=CustomSpawnerCfg(), initial_pose=pose)


def _arena_object(spec: KitObject):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose as ArenaPose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    if isinstance(spec, Cloth):
        return _cloth_object(spec)
    if isinstance(spec, Custom):
        return _custom_object(spec)
    pose = ArenaPose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(float(v) for v in _yaw_quat(spec.yaw)))
    kind = ObjectType.RIGID if spec.mass is not None else ObjectType.BASE
    body = dict(
        collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0],
                                                        dynamic_friction=GRIPPY_FRICTION[1]),
    )
    if spec.mass is not None:
        body.update(rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass))

    if isinstance(spec, Container | Marked | Hand):
        parts = _container_parts if isinstance(spec, Container) else _hand_parts if isinstance(spec, Hand) \
            else _mark_parts

        @clone
        def spawn_with_parts(prim_path: str, cfg, *args, **kwargs):
            prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)  # the body and its first collider
            parts(prim_path, spec)
            return prim

        size = (spec.size[0], spec.size[1], spec.wall) if isinstance(spec, Container) else spec.size
        cfg = sim_utils.CuboidCfg(
            size=size, func=spawn_with_parts,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4), **body)
        return Object(name=spec.name, object_type=kind, spawner_cfg=cfg, initial_pose=pose)

    if isinstance(spec, Model):
        extra = {}
        if spec.colliders:
            from isaaclab.sim.spawners.from_files.from_files import spawn_from_usd

            @clone
            def spawn_with_colliders(prim_path: str, cfg, *args, **kwargs):
                prim = spawn_from_usd.__wrapped__(prim_path, cfg, *args, **kwargs)
                _model_colliders(prim_path, spec)
                return prim

            extra["func"] = spawn_with_colliders
        cfg = sim_utils.UsdFileCfg(usd_path=spec.usd, scale=(spec.scale,) * 3, **extra, **{
            k: v for k, v in body.items() if k != "physics_material"})
        return Object(name=spec.name, object_type=kind, spawner_cfg=cfg, initial_pose=pose)

    if isinstance(spec, Shaped):

        @configclass
        class ShapedSpawnerCfg(SpawnerCfg):
            func: object = clone(lambda prim_path, cfg, *a, translation=None, orientation=None, **k: _shaped_parts(
                prim_path, spec, translation, orientation))

        return Object(name=spec.name, object_type=kind, spawner_cfg=ShapedSpawnerCfg(), initial_pose=pose)

    if isinstance(spec, Lamp):
        cfg = sim_utils.CylinderCfg(
            radius=spec.radius,
            height=spec.size[2],
            visible=False,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.2,
                                                        emissive_color=linear_rgb(spec.color)),
        )
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)

    if isinstance(spec, Hinge | Slider):
        joint = "hinge" if isinstance(spec, Hinge) else "slide"

        @configclass
        class JointSpawnerCfg(SpawnerCfg):
            func: object = clone(lambda prim_path, cfg, *a, translation=None, orientation=None, **k: _jointed_parts(
                prim_path, spec, translation, orientation))

        class JointObject(Object):
            def _generate_articulation_cfg(self):
                # The implicit actuator is the spring (stiffness 0 without one); the joint's zero is spec.rest, so
                # the zero target Isaac Lab holds for an undriven joint is the spring's rest.
                act = ImplicitActuatorCfg(joint_names_expr=[joint], stiffness=spec.spring or 0.0,
                                          damping=spec.damping, friction=spec.friction)
                cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators={"joint": act})
                cfg = self._add_initial_pose_to_cfg(cfg)
                cfg.init_state.joint_pos = {joint: spec.start - spec.rest}
                return cfg

        return JointObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=JointSpawnerCfg(),
                           initial_pose=pose)

    if isinstance(spec, Towel):

        @configclass
        class TowelSpawnerCfg(SpawnerCfg):
            func: object = clone(lambda prim_path, cfg, *a, translation=None, orientation=None, **k: _towel_parts(
                prim_path, spec, translation, orientation))

        class TowelObject(Object):
            def _generate_articulation_cfg(self):
                act = ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=spec.stiffness, damping=spec.damping,
                                          armature=spec.armature)
                cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators={"cloth": act})
                return self._add_initial_pose_to_cfg(cfg)

        root = spec.start_points()[spec.tile_ids().index(spec.centre_tile)]
        return TowelObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=TowelSpawnerCfg(),
                           initial_pose=ArenaPose(position_xyz=tuple(float(v) for v in root),
                                                  rotation_xyzw=tuple(float(v) for v in _yaw_quat(spec.yaw))))

    @configclass
    class ButtonSpawnerCfg(SpawnerCfg):
        func: object = clone(lambda prim_path, cfg, *a, translation=None, orientation=None, **k: _button_parts(
            prim_path, spec, translation, orientation))

    class ButtonObject(Object):
        def _generate_articulation_cfg(self):
            # The implicit actuator is the spring: its target is the cap's rest stop.
            spring = ImplicitActuatorCfg(joint_names_expr=["press"], stiffness=spec.rate, damping=spec.damping)
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators={"spring": spring})
            return self._add_initial_pose_to_cfg(cfg)

    return ButtonObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=ButtonSpawnerCfg(),
                        initial_pose=pose)


# -- state and lamps (Isaac Sim) --------------------------------------------------------------------------------------


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _box_state(spec, pos, quat, lin=np.zeros(3), ang=np.zeros(3)) -> ObjectState:
    corners = spec.corners(pos, quat) if hasattr(spec, "corners") else _box_corners((0, 0, 0), spec.size, pos, quat)
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(lin, float),
                       ang_vel=np.asarray(ang, float), low=corners.min(0), high=corners.max(0))


def start_state(spec) -> ObjectState:
    """The state ``spec`` starts in, at rest: what a fixed object always reports, and what tests can build on."""
    if isinstance(spec, Hinge | Slider):
        return spec.state_at(spec.start)
    if isinstance(spec, Towel | Cloth):
        return spec.state_at()
    if isinstance(spec, Custom):
        return spec.starter(spec)
    if isinstance(spec, Hand):
        return spec.state()
    s = _box_state(spec, np.asarray(spec.pos, float), _yaw_quat(spec.yaw))
    if isinstance(spec, Button):
        half = np.asarray(spec.size, float) / 2
        return ButtonState(pos=s.pos, quat_xyzw=s.quat_xyzw, lin_vel=s.lin_vel, ang_vel=s.ang_vel,
                           low=s.pos - half, high=s.pos + half, travel=0.0)
    return s


def _towel_state(environment, spec: Towel) -> TowelState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    names = list(asset.body_names)
    idx = [names.index(spec.tile_name(i, j)) for i, j in spec.tile_ids()]
    return spec.state_at(_np(asset.data.body_pos_w)[0][idx] - origin, _np(asset.data.body_quat_w)[0][idx],
                         _np(asset.data.body_lin_vel_w)[0][idx], _np(asset.data.body_ang_vel_w)[0][idx])


_CLOTH_NODES: dict[str, int] = {}


def _cloth_state(environment, spec: Cloth) -> ClothState:
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    nodes = _np(base.arena_world.get_nodal_positions_w(spec.name)[0]) - origin
    vel = _np(base.arena_world.get_nodal_velocities_w(spec.name)[0])
    if spec.name not in _CLOTH_NODES:  # say once what the simulator built, to compare with spec.node_count
        _CLOTH_NODES[spec.name] = len(nodes)
        print(f"[labkit] cloth {spec.name}: {len(nodes)} nodes in the simulator, {spec.node_count} in the mesh", flush=True)
    return spec.state_at(nodes, vel)


def _state(environment, spec: KitObject) -> ObjectState:
    if isinstance(spec, Lamp | Hand):
        return start_state(spec)
    if isinstance(spec, Towel):
        return _towel_state(environment, spec)
    if isinstance(spec, Cloth):
        return _cloth_state(environment, spec)
    if isinstance(spec, Custom):
        root = getattr(environment.unwrapped.scene, "env_prim_paths", ["/World/envs/env_0"])[0]
        return spec.reader(environment, spec, f"{root}/{spec.name}")
    base = environment.unwrapped
    if isinstance(spec, Hinge | Slider):  # the base is welded in place: only the joint moves
        asset = base.scene[spec.name]
        joint = "hinge" if isinstance(spec, Hinge) else "slide"
        q = float(_np(asset.data.joint_pos)[0][list(asset.joint_names).index(joint)]) + spec.rest
        return hinge_step(spec, q) if isinstance(spec, Hinge) else spec.state_at(q)
    if spec.mass is None:
        return start_state(spec)
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    if not isinstance(spec, Button):
        return _box_state(spec, pos, quat, vel[:3], vel[3:6])
    q = _np(asset.data.joint_pos)[0]
    travel = max(0.0, -float(q[list(asset.joint_names).index("press")]))
    half = np.asarray(spec.size, float) / 2
    return ButtonState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pos - half, high=pos + half,
                       travel=travel)


_TRACE = {"n": 0}
_JOINTS: dict = {}  # each arm's joint positions as last read (for LABKIT_TRACE_FULL)


def _trace(states) -> None:
    """With LABKIT_TRACE=<file>, append every 10th read of the objects as a JSON line: each loose object's pose
    and both TCPs (a probe for tuning a demo against what the simulator really does)."""
    import os

    path = os.environ.get("LABKIT_TRACE")
    if not path:
        return
    full = bool(os.environ.get("LABKIT_TRACE_FULL"))  # also every cloth node and both arms' joints, every 4th read
    _TRACE["n"] += 1
    every = int(os.environ.get("LABKIT_TRACE_EVERY", "4" if full else "10"))
    if _TRACE["n"] % every != 1:
        return
    import json

    row = {"read": _TRACE["n"] - 1, "tcp": {k: np.round(v, 4).tolist() for k, v in _GRIPS.items()}, "obj": {}}
    for name, st in states.items():
        spec = _SPECS.get(name)
        if spec is not None and getattr(spec, "mass", None) is not None:
            row["obj"][name] = {"pos": np.round(st.pos, 4).tolist(), "quat": np.round(st.quat_xyzw, 4).tolist()}
        if full and isinstance(spec, Cloth):
            row["obj"][name] = {"nodes": np.round(st.nodes, 4).tolist()}
        if isinstance(spec, Custom) and getattr(st, "pins", None) is not None:  # a jointed mechanism's parts
            row["obj"][name] = {"pins": np.round(st.pins, 4).tolist(), "quats": np.round(st.quats, 4).tolist()}
            for extra in ("crank", "rear"):  # a drivetrain's angles, if it has them
                if hasattr(st, extra):
                    row["obj"][name][extra] = round(float(getattr(st, extra)), 4)
    if full:
        row["joints"] = {arm: {k: round(v, 4) for k, v in q.items()} for arm, q in _JOINTS.items()}
        row["t"] = round(_CLOCK["t"], 3)
    with open(path, "a") as f:
        f.write(json.dumps(row) + "\n")


def _read_grippers(environment) -> None:
    """Both arms' TCPs (the gripper link's pose plus the TCP offset), env frame, into :func:`gripper_points`."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    for arm in ("left", "right"):
        try:
            asset = base.scene[f"{arm}_robot"]
            i = list(asset.body_names).index("gripper")
        except (KeyError, ValueError):
            continue
        pos = _np(asset.data.body_pos_w)[0][i] - origin
        quat = _np(asset.data.body_quat_w)[0][i]  # Isaac Lab 3: (x, y, z, w)
        _GRIPS[arm] = pos + quat_xyzw_to_matrix(quat) @ np.asarray(TCP_OFFSET, float)
        _JOINTS[arm] = dict(zip(asset.joint_names, _np(asset.data.joint_pos)[0].tolist()))


def _set_visible(environment, name: str, show: bool, child: str = "") -> None:
    """Show or hide object ``name``'s prim (or its ``child``) in env 0."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    root = getattr(environment.unwrapped.scene, "env_prim_paths", ["/World/envs/env_0"])[0]
    prim = stage.GetPrimAtPath(f"{root}/{name}")
    if not prim.IsValid():  # fall back to finding it by name under env 0
        prim = next((p for p in stage.Traverse() if p.GetName() == name and str(p.GetPath()).startswith(root)), prim)
    if prim.IsValid() and child:
        prim = stage.GetPrimAtPath(prim.GetPath().AppendPath(child))
    if prim.IsValid():
        image = UsdGeom.Imageable(prim)
        image.MakeVisible() if show else image.MakeInvisible()


def _show_lamps(environment, on: bool) -> None:
    for name, spec in _SPECS.items():
        if isinstance(spec, Lamp):
            _set_visible(environment, name, on and (spec.group is None or spec.group == _GROUP["active"]))
    _LIT["on"] = on


def _show_gloves(environment) -> None:
    for name, spec in _SPECS.items():
        if isinstance(spec, Hand):
            _set_visible(environment, name, glove_on(spec), "glove")


def _after_step(environment, t: float | None = None) -> None:
    tick(_CLOCK["t"] + STEP_S if t is None else t, environment)  # timed lamp switches
    if _LIT["on"]:
        return
    for spec in _SPECS.values():
        if isinstance(spec, Button) and _state(environment, spec).travel >= spec.latch_at:
            _show_lamps(environment, True)
            return


def _set_joint_starts(environment) -> None:
    """Put every hinge and slider at its ``start``. Isaac Lab-Arena's reset leaves their joints at 0 (the spring's
    rest, clamped to the limits): a sharps lid meant to start open started shut, a gas-strut lid meant to start shut
    started open (GPU check, 9 Nov 2026)."""
    base = getattr(environment, "unwrapped", environment)
    for name, spec in _SPECS.items():
        if isinstance(spec, Hinge | Slider):
            asset = base.scene[name]
            asset.write_joint_position_to_sim_index(position=asset.data.default_joint_pos.torch.clone())
            asset.write_joint_velocity_to_sim_index(velocity=asset.data.default_joint_vel.torch.clone())


def _reweld(environment) -> None:
    """Weld every snapped :class:`Shaped` to its parent again. A PhysX joint that broke stays broken through a
    reset, so after the first reset each snap joint is removed and defined anew (under a new name) where the reset
    put the bodies back."""
    snapped = [(n, s) for n, s in _SPECS.items() if isinstance(s, Shaped) and s.snap_to is not None]
    if not snapped:
        return
    _WELDS["resets"] = _WELDS.get("resets", 0) + 1
    if _WELDS["resets"] == 1:  # the joints defined at spawn are still whole
        return
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    root = getattr(environment.unwrapped.scene, "env_prim_paths", ["/World/envs/env_0"])[0]
    old, _WELDS["n"] = _WELDS["n"], _WELDS["n"] + 1
    for name, spec in snapped:
        path = f"{root}/{name}"
        stage.RemovePrim(_snap_path(path, old))
        _snap_joint(stage, path, spec, _WELDS["n"])


def _after_reset(environment) -> None:
    _set_joint_starts(environment)
    _reweld(environment)
    _show_lamps(environment, False)
    for fn in _RESET:  # labkit's _start_trial first, then the task's hooks (which may schedule lamps, gloves)
        fn()
    _show_lamps(environment, _LIT["on"])  # a timed switch due at 0 lights from the first frame
    _show_gloves(environment)


# -- hooking yam_arena ------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_labkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def task_object_hook(spec):
        if isinstance(spec, KitObject):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def object_states_hook(environment, names):
        _read_grippers(environment)
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
        _LATEST.clear()
        _LATEST.update(out)
        _trace(out)
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object_hook, object_states_hook
    scene._labkit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_labkit", False):
        return
    task_step, reset = cls._task_step, cls.reset

    def task_step_hook(self, *args, **kwargs):
        _after_step(self._env, getattr(self, "_t", None))
        return task_step(self, *args, **kwargs)

    def reset_hook(self, *args, **kwargs):
        obs = reset(self, *args, **kwargs)
        _SEED["seed"] = kwargs.get("seed")
        _after_reset(self._env)
        return obs

    cls._task_step, cls.reset = task_step_hook, reset_hook
    cls._labkit = True


class _Patcher(importlib.abc.MetaPathFinder):
    """Patches yam_arena's scene and embodiment modules as they load (they need Isaac Sim, which starts after the
    task file is read)."""

    patches = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.patches:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module, patch = spec.loader.exec_module, self.patches[fullname]

        def exec_and_patch(module):
            exec_module(module)
            patch(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Patch modules already imported, and the rest as they load. Safe to call twice."""
    for name, patch in _Patcher.patches.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Patcher) for f in sys.meta_path):
        sys.meta_path.insert(0, _Patcher())


install()
