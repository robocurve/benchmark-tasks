"""labkit: extra object kinds, scoring helpers and demo poses for yam_arena task files.

yam_arena's task files spawn single primitives (Box, Cylinder, Sphere) and models from Arena's library. This module
adds, without changing yam_arena:

- :class:`Container`: an open-top square container (bottle, tub, tray, bin), its floor and four walls one body,
  with an optional coloured label band round its middle (visual only). Its ``pos`` is the centre of its floor.
- :class:`Marked`: a solid box with flat coloured marks on chosen faces (a crack, a label, a warning patch). Marks
  are visual only, so a mark on the back face is hidden from a camera that sees only the front.
- :class:`Model`: a 3D model from a USD file (convert STL/OBJ with tools/convert_mesh.py on the GPU machine).
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

Scoring helpers: :func:`tilt_deg`, :func:`lifted`, :func:`moved`, :func:`inside`, :func:`count_inside`,
:func:`all_resting`, :func:`handed_over` and :class:`Trial` (flags that latch during a trial and clear at reset).
Offline tests play a trial's clock and a lid's angles with :func:`tick` and :func:`hinge_step`, as each control step
does in the simulator.

Demo helpers: a :class:`Pose` waypoint carries its own TCP rotation, which ``yam_arena.demo.ArmPlan`` follows
(see :func:`forward_grip`, :func:`rolled` and :func:`ready_pose`).

Pure Python at import time; Isaac Sim is imported lazily, once yam_arena's scene module loads. Copy this file into
each task folder (tools/new_task.py does) so every pull request is self-contained.

Added for rearm-wound-drain-bulb (14 Nov 2026, offline only; not yet in the kit's template, needs a GPU check):
- :class:`Prop`: a fixed prop made of meshes generated in code (:func:`ellipsoid_mesh`, :func:`cylinder_mesh`,
  :func:`tube_mesh`), each visual only unless ``collides``, plus an optional box collider. A mesh part can be
  rescaled each step with :func:`scale_part` (a bulb that visibly flattens).
- :class:`Slider` options: ``panel_visible`` (False: a collider only), ``frame_collides`` (False: the frame is
  visual only) and ``no_collide_with`` (names of other sliders whose panels this panel passes through).
- :class:`Hinge` option ``lid_mesh``: a generated mesh (convex) in place of the lid's box.
- :func:`on_step`: a task hook run every control step with the environment, before scoring;
  :func:`hold_joint` (alias :func:`hold_slider`) / :func:`release_slider`: hold a slider or hinge where it is with a
  stiff drive (task code standing in for a mechanism: a sealed bulb's trapped vacuum, a push-fit cap's grip), and let
  it go again. Holds clear at reset.
- :class:`Harness` (added 2026-11-15 for outlet-into-box; offline checks only, never run in Isaac Sim): a device on
  stiff wires that stay bent (jointed capsules with elastic-perfectly-plastic joints, :func:`plastic_targets` applied
  every control step). The first GPU run must check that it spawns straight and stable, that the loop-closing
  terminal joints hold, which joint style ("d6" or "alternate") Isaac Lab accepts, and that a bend stays bent.

- :class:`Harness` ``looks`` and :class:`Part` ``metallic`` (added 2026-11-19; offline checks only, not yet run in
  Isaac Sim): ``looks`` are :class:`Part` meshes drawn on the device and never colliders (a screw slot drawn on a
  strap); ``metallic`` is the look's metalness, passed to the surface only when it is not 0. The first GPU run must
  check that the harness spawns as before with looks on it and how a metallic part renders.

Status (2026-11-07): Container, Button and Lamp come from a draft task and pass offline checks only; Marked and
Model are new. None has run in Isaac Sim yet: the first GPU session must check that each one spawns.
Added 2026-11-09 (offline checks only): Lamp ``group`` with :func:`set_lamp_group` / :func:`lit_group`,
:func:`trial_seed`, and the demo :class:`Branch`. The first GPU run of a task that uses them must check that only the
active group's lamp lights, that the reset seed reaches the task, and that a Branch follows the lit lamp.
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
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

__all__ = [
    "Bottle", "Branch", "Built", "Button", "ButtonState", "Container", "Hand", "HandState", "Hinge", "HingeState", "Lamp",
    "Latch", "Mark", "Marked", "Model", "Part", "Pose", "Prop", "Slider", "SliderState", "Towel", "TowelState", "Trial",
    "all_resting", "count_inside", "cylinder_mesh", "ellipsoid_mesh", "forward_grip", "glove_on", "handed_over",
    "HangingBulb", "BulbState", "Harness", "HarnessPart", "HarnessState", "HarnessWire", "harness_rest",
    "plastic_targets", "hinge_step", "hold_joint", "hold_slider", "inside", "release_joint", "lamp_group_at", "lamp_switches", "lifted", "lit_group", "moved", "on_reset",
    "on_step", "ready_pose", "release_slider", "rolled", "scale_part", "set_glove", "set_lamp_group",
    "switch_lamp_group", "tick", "tilt_deg", "trial_seed", "trial_time", "tube_mesh",
    "on_physics", "place", "push", "push_link_all",
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

    def _check(self) -> None:
        super()._check()
        if not self.usd.endswith((".usd", ".usda", ".usdc")):
            raise ValueError(f"{self.name}: usd must be a .usd/.usda/.usdc file path, got {self.usd!r}")
        if not self.scale > 0:
            raise ValueError(f"{self.name}: scale must be positive")

    def corners(self, pos, quat_xyzw) -> np.ndarray:
        return _box_corners((0.0, 0.0, 0.0), self.size, pos, quat_xyzw)


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
    lid_mesh: tuple | None = None  # (points, counts, indices) from a *_mesh helper, centred like the lid box

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a hinge's base is fixed (no mass); the lid's is lid_mass")
        if self.lid_mesh is not None:
            _check_mesh(self.name, self.lid_mesh)
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
    panel_visible: bool = True
    frame_collides: bool = True
    no_collide_with: tuple[str, ...] = ()

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a slider's frame is fixed (no mass); the panel's is panel_mass")
        if self.name in self.no_collide_with:
            raise ValueError(f"{self.name}: no_collide_with names the slider itself")
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


# -- meshes generated in code, and fixed props made of them --------------------------------------------------------------
# A mesh is (points, counts, indices): points as a tuple of (x, y, z) in metres, counts the vertices of each face, and
# indices the faces' vertices in order (counter-clockwise seen from outside). Tuples, so specs stay hashable.


def _mesh(points, faces) -> tuple:
    pts = tuple(tuple(round(float(v), 7) for v in p) for p in points)
    return pts, tuple(len(f) for f in faces), tuple(int(i) for f in faces for i in f)


def _check_mesh(name: str, mesh) -> None:
    if not (isinstance(mesh, tuple) and len(mesh) == 3):
        raise ValueError(f"{name}: a mesh is (points, counts, indices)")
    points, counts, indices = mesh
    if sum(counts) != len(indices) or any(c < 3 for c in counts):
        raise ValueError(f"{name}: the mesh's face counts don't match its indices")
    if indices and (min(indices) < 0 or max(indices) >= len(points)):
        raise ValueError(f"{name}: the mesh indexes a point it doesn't have")


def ellipsoid_mesh(a: float, b: float, c: float, n: int = 20) -> tuple:
    """A closed ellipsoid centred at the origin with semi-axes ``a`` (x), ``b`` (y), ``c`` (z): ``n`` segments round,
    ``n // 2`` rings from pole to pole. Convex."""
    rings = max(3, n // 2)
    pts = [(0.0, 0.0, -c)]
    for i in range(1, rings):
        phi = -math.pi / 2 + math.pi * i / rings
        for j in range(n):
            th = 2 * math.pi * j / n
            pts.append((a * math.cos(phi) * math.cos(th), b * math.cos(phi) * math.sin(th), c * math.sin(phi)))
    pts.append((0.0, 0.0, c))
    top, faces = len(pts) - 1, []
    for j in range(n):
        faces.append((0, 1 + (j + 1) % n, 1 + j))
    for i in range(rings - 2):
        r0, r1 = 1 + i * n, 1 + (i + 1) * n
        for j in range(n):
            k = (j + 1) % n
            faces.append((r0 + j, r0 + k, r1 + k, r1 + j))
    last = 1 + (rings - 2) * n
    for j in range(n):
        faces.append((last + j, last + (j + 1) % n, top))
    return _mesh(pts, faces)


def cylinder_mesh(radius: float, height: float, n: int = 16) -> tuple:
    """A closed upright cylinder centred at the origin (axis z). Convex."""
    h = height / 2
    pts = [(radius * math.cos(2 * math.pi * j / n), radius * math.sin(2 * math.pi * j / n), z) for z in (-h, h)
           for j in range(n)]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(j, (j + 1) % n, n + (j + 1) % n, n + j) for j in range(n)]
    return _mesh(pts, faces)


def tube_mesh(path, radius: float, n: int = 8) -> tuple:
    """A tube of ``radius`` along ``path`` (a list of 3-D points), open at both ends. Not convex: visual only."""
    path = [np.asarray(p, float) for p in path]
    if len(path) < 2:
        raise ValueError("a tube needs at least two points")
    pts, ref = [], np.array([0.0, 0.0, 1.0])
    for i, p in enumerate(path):
        d = path[min(i + 1, len(path) - 1)] - path[max(i - 1, 0)]
        d /= np.linalg.norm(d) + 1e-12
        if abs(d @ ref) > 0.9:
            ref = np.array([1.0, 0.0, 0.0])
        u = np.cross(d, ref)
        u /= np.linalg.norm(u)
        v = np.cross(d, u)
        pts += [tuple(p + radius * (math.cos(2 * math.pi * j / n) * u + math.sin(2 * math.pi * j / n) * v))
                for j in range(n)]
    faces = [(i * n + j, i * n + (j + 1) % n, (i + 1) * n + (j + 1) % n, (i + 1) * n + j)
             for i in range(len(path) - 1) for j in range(n)]
    return _mesh(pts, faces)


@dataclass(frozen=True)
class Part:
    """One mesh of a :class:`Prop`: ``mesh`` in the part's frame, placed at ``at`` (prop frame, m), in ``color``.
    ``collides`` makes it a convex-hull collider (only for convex meshes)."""

    name: str
    mesh: tuple
    at: tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: Color = (0.7, 0.7, 0.7)
    collides: bool = False
    roughness: float = 0.5
    metallic: float = 0.0  # 0: plastic, paint; 1: bare metal (it then shows its surroundings more than its colour)


@dataclass(frozen=True)
class Prop(Box):
    """A fixed prop built from :class:`Part` meshes made in code (a patient, a bulb's visible body, tubing). ``pos``
    is its origin; ``size`` is a box collider round the origin when ``box_collider`` (else nothing collides but the
    parts marked ``collides``). Visual parts can be rescaled each step (:func:`scale_part`)."""

    parts: tuple = ()
    box_collider: bool = False

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a prop is fixed (no mass)")
        names = [p.name for p in self.parts]
        if len(set(names)) != len(names) or not all(n.isidentifier() for n in names):
            raise ValueError(f"{self.name}: part names must be unique identifiers")
        for p in self.parts:
            _check_mesh(f"{self.name}.{p.name}", p.mesh)


@dataclass(frozen=True)
class Built(Box):
    """A rigid body made in code (a hand tool, a screw): its own box (``size``, a collider, in ``color``) plus
    :class:`Part` meshes, each visual only unless it ``collides`` (then a convex-hull collider on the same body).
    ``pos`` is the box's centre and the body's origin; it needs a ``mass``. ``driven`` makes it kinematic: it collides
    and pushes other things but never moves by itself, only where task code puts it with :func:`place` (from an
    :func:`on_physics` hook, every physics step); its mass is then unused. ``friction`` is (static, dynamic) in place
    of the kit's grippy default."""

    parts: tuple = ()
    driven: bool = False
    friction: tuple | None = None
    apart_from: tuple = ()  # names of other bodies this one never collides with (task code stands in for that contact)
    friction_mix: str | None = None  # how its friction combines with what it touches ("min": the lower of the two
    #                                  wins, so with friction (0, 0) it slides on everything); None: PhysX's average

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a built body needs a mass (a driven one too: it is how its pose is read)")
        names = [p.name for p in self.parts]
        if len(set(names)) != len(names) or not all(n.isidentifier() for n in names):
            raise ValueError(f"{self.name}: part names must be unique identifiers")
        for p in self.parts:
            _check_mesh(f"{self.name}.{p.name}", p.mesh)


@dataclass(frozen=True)
class HangingBulb(Box):
    """A closed-suction drain bulb hanging free: one articulation, its root welded at ``pos`` (the hanging point,
    above the bulb), turned ``yaw``. Joints, by name:

    - ``swing_u``, ``swing_v``: the bulb swings about the hanging point (about its own x, then y), against a soft
      ``swing_spring`` (N m/rad) standing in for its strap and tubing, within ``max_swing`` (rad).
    - ``squeeze_left``, ``squeeze_right``: its two faces (invisible paddles ``paddle``, at +-``face_at`` on its y axis)
      slide in by up to ``travel`` against ``spring`` (N/m each); left moves to negative positions, right to positive.
    - ``cap_hinge``: the cap (a generated mesh ``cap_mesh`` at ``cap_at`` from the hinge) on the port's edge
      (``cap_pivot``, core frame), sprung toward ``cap_rest`` by ``cap_spring``, within ``cap_limits``, starting at
      ``cap_start``; ``cap_latch`` flags it seated.

    The bulb's own frame ("core") has its origin at the hanging point: the bulb's centre is ``drop`` below it, the
    port (``port`` box) on its top. ``parts`` are extra generated meshes carried by the core (visual unless marked),
    for example the visible bulb, scaled each step with :func:`scale_part` (child path ``core/<part>``)."""

    drop: float = 0.16
    swing_spring: float = 1.5
    swing_damping: float = 0.05
    max_swing: float = 0.7
    core_mass: float = 0.04
    paddle: tuple[float, float, float] = (0.06, 0.003, 0.08)
    face_at: float = 0.0225
    travel: float = 0.016
    spring: float = 625.0
    face_damping: float = 2.0
    face_mass: float = 0.02
    port: tuple[float, float, float] = (0.012, 0.012, 0.010)
    port_z: float = 0.053  # the port box's centre above the bulb's centre
    port_color: Color = (0.95, 0.95, 0.94)
    cap_mesh: tuple | None = None
    cap_size: tuple[float, float, float] = (0.016, 0.016, 0.006)  # bounds of the cap (for its box if no mesh)
    cap_pivot: tuple[float, float] = (-0.006, 0.005)  # (y, z) of the hinge line from the port box's centre
    cap_at: tuple[float, float, float] = (0.0, 0.008, 0.003)  # the cap's centre from the hinge when seated
    cap_mass: float = 0.003
    cap_limits: tuple[float, float] = (0.0, 2.0)
    cap_start: float = 2.0
    cap_rest: float = 2.0
    cap_spring: float = 0.002
    cap_damping: float = 0.002
    cap_latch: Latch = Latch(detent=0.035, tol=0.12)
    cap_color: Color = (0.98, 0.98, 0.98)
    parts: tuple = ()

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: the hanging point is fixed (no mass); give core_mass instead")
        lo, hi = self.cap_limits
        if not (lo <= self.cap_start <= hi and lo <= self.cap_latch.detent < self.cap_latch.tol <= hi):
            raise ValueError(f"{self.name}: cap start or latch outside cap_limits {self.cap_limits}")
        if self.cap_mesh is not None:
            _check_mesh(f"{self.name}.cap_mesh", self.cap_mesh)
        for p in self.parts:
            _check_mesh(f"{self.name}.{p.name}", p.mesh)
        if not 0 < self.travel < self.face_at:
            raise ValueError(f"{self.name}: the faces would cross (travel {self.travel}, face_at {self.face_at})")

    JOINTS = ("swing_u", "swing_v", "squeeze_left", "squeeze_right", "cap_hinge")

    def joint_rest(self, joint: str) -> float:
        return self.cap_rest if joint == "cap_hinge" else 0.0

    def joint_spring(self, joint: str) -> float:
        return {"swing_u": self.swing_spring, "swing_v": self.swing_spring, "squeeze_left": self.spring,
                "squeeze_right": self.spring, "cap_hinge": self.cap_spring}[joint]

    @property
    def port_offset(self) -> np.ndarray:
        """The port box's centre in the core frame."""
        return np.array([0.0, 0.0, -self.drop + self.port_z])

    @property
    def cap_hinge_point(self) -> np.ndarray:
        return self.port_offset + np.array([0.0, self.cap_pivot[0], self.cap_pivot[1]])

    def state_at(self, core_pos, core_quat_xyzw, joints: dict, latched: bool | None = None) -> BulbState:
        """The bulb's state from its core pose (world) and joint values (``joints``: name -> value, absolute)."""
        R = quat_xyzw_to_matrix(np.asarray(core_quat_xyzw, float))
        p = np.asarray(core_pos, float)
        centre = p + R @ np.array([0.0, 0.0, -self.drop])
        mouth = p + R @ (self.port_offset + np.array([0.0, 0.0, self.port[2] / 2]))
        cap = float(joints.get("cap_hinge", self.cap_start))
        if latched is None:
            latched = self.cap_latch.step(cap, False)
        half = np.array([self.size[0], self.size[1], self.size[2]]) / 2
        return BulbState(pos=p, quat_xyzw=np.asarray(core_quat_xyzw, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                         low=centre - half, high=centre + half, centre=centre, mouth=mouth,
                         squeeze_left=float(joints.get("squeeze_left", 0.0)),
                         squeeze_right=float(joints.get("squeeze_right", 0.0)), cap_angle=cap,
                         cap_latched=bool(latched),
                         swing=(float(joints.get("swing_u", 0.0)), float(joints.get("swing_v", 0.0))))


@dataclass(frozen=True)
class BulbState(ObjectState):
    """A :class:`HangingBulb`: ``pos``/``quat_xyzw`` are its core frame (origin at the hanging point); ``centre`` and
    ``mouth`` (the port's open top) in the world; the faces' and cap's joint values; ``swing`` (u, v) in rad."""

    centre: np.ndarray | None = None
    mouth: np.ndarray | None = None
    squeeze_left: float = 0.0
    squeeze_right: float = 0.0
    cap_angle: float = 0.0
    cap_latched: bool = False
    swing: tuple = (0.0, 0.0)


# -- a device on stiff wires ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class HarnessWire:
    """One wire of a :class:`Harness`: from ``root`` (harness frame, i.e. relative to the anchor) to ``terminal``
    (device frame), in ``segments`` rigid capsule links of ``radius``. Straight at the start, unless ``points`` gives
    its start shape: segments + 1 joint points (harness frame), root first, terminal last, all links the same length.
    Whatever the start shape, it is the wire's relaxed shape (zero bend)."""

    name: str
    root: tuple
    terminal: tuple
    segments: int
    radius: float
    color: Color = (0.1, 0.1, 0.1)
    link_mass: float = 0.0005
    points: tuple | None = None


@dataclass(frozen=True)
class HarnessPart:
    """A box of the device's body, in the device frame (visual and collider)."""

    name: str
    size: tuple
    at: tuple
    color: Color


@dataclass(frozen=True)
class Harness(Box):
    """A device (an outlet, a switch) hanging on stiff wires that stay bent: an anchor welded at ``pos`` (where the
    cable's sheath ends), ``wires`` running from it, and a rigid device (``parts``, ``device_mass``) whose origin starts
    at ``device_at`` (harness frame, upright, axes as the world's) with every wire ending on it.

    The wires are elastic-perfectly-plastic, like annealed copper: each joint has a stiff spring (``stiffness`` N m/rad,
    ``damping``, ``armature`` for stability at 240 Hz) toward a rest angle that labkit drags along each control step
    whenever the joint is bent more than ``yield_angle`` past it (:func:`plastic_targets`). Unloaded, a joint springs
    back by at most ``yield_angle`` and keeps the rest of its bend. ``style`` "d6": each joint bends both ways (two
    rotational degrees of freedom, no twist); "alternate": one revolute joint per node, its axis alternating between
    the link's y and z (half the bending resolution in each plane, but plain revolute joints).

    One articulation: anchor -> each wire's links; the first wire's last link -> the device by a fixed joint in the
    tree, the other wires' last links -> the device by fixed joints kept out of it (they close loops). The last
    ``end_filter`` links of each wire don't collide with the device (they are screwed down under its terminals). At
    most 64 links. ``yaw`` must be 0. It is never jittered at reset."""

    wires: tuple = ()
    parts: tuple = ()
    device_at: tuple = (0.0, 0.0, 0.0)
    device_mass: float = 0.07
    stiffness: float = 7.0
    damping: float = 0.03
    armature: float = 5e-5
    yield_angle: float = 0.008
    max_bend: float = math.radians(90.0)
    style: str = "d6"
    end_filter: int = 2
    wire_friction: float = 0.4
    torque_cap: float | None = None  # N m: each wire joint's drive torque limit (copper's yield moment); None = none
    # The copper rule (copper.py): with ``flow_m`` set, ``yield_angle`` and ``torque_cap`` are not used. Each joint
    # stands for ``segment`` metres of wire; its two axes are one bend; it yields at copper's flow moment, which rises
    # with the plastic curvature the joint has been through (``flow_kp`` 1/m -> ``flow_m`` N m), and its drive torque is
    # capped at that moment every control step.
    segment: float = 0.0
    flow_kp: tuple = ()
    flow_m: tuple = ()
    rigid_root: bool = False  # each wire's first link is fixed to the anchor: a short stub clamped in the cable's sheath
    # The torque cap is the flow moment this much plastic curvature (1/m) ahead. The rule runs once per control step, and
    # between updates a joint at its cap bends freely: with the cap at the flow moment itself a bend made in 0.3 s
    # overshot by 38%; 0.3/m ahead keeps every bend within 12% of copper's curve from 0.3 s to 4 s (laptop emulation).
    cap_ahead: float = 0.3
    # The plastic curvature (1/m) every joint starts with, wire by wire from the cable to the outlet: the bends of the
    # start shape (old folds start work-hardened). Empty: every joint starts soft.
    hardened0: tuple = ()
    # :class:`Part` meshes drawn on the device (device frame): looks only, never colliders (a screw slot drawn on a
    # strap, a label). They add nothing to the device's shape, mass or state.
    looks: tuple = ()

    @property
    def free(self) -> bool:
        return False

    def _check(self) -> None:
        super()._check()
        if not self.wires or not self.parts:
            raise ValueError(f"{self.name}: a harness needs wires and device parts")
        if self.yaw != 0.0:
            raise ValueError(f"{self.name}: a harness can't be turned (yaw must be 0)")
        if self.style not in ("d6", "alternate"):
            raise ValueError(f"{self.name}: style must be 'd6' or 'alternate', got {self.style!r}")
        if 2 + sum(w.segments for w in self.wires) > MAX_LINKS:
            raise ValueError(f"{self.name}: {2 + sum(w.segments for w in self.wires)} links, PhysX allows {MAX_LINKS}")
        if self.torque_cap is not None and self.torque_cap <= 0:
            raise ValueError(f"{self.name}: torque_cap must be positive (or None)")
        if min(self.stiffness, self.damping, self.armature, self.yield_angle) < 0 or self.device_mass <= 0:
            raise ValueError(f"{self.name}: stiffness, damping, armature, yield_angle >= 0 and a device mass, please")
        if len(self.flow_kp) != len(self.flow_m) or (self.flow_m and (self.segment <= 0 or len(self.flow_m) < 2)):
            raise ValueError(f"{self.name}: the copper rule needs flow_kp and flow_m of one length and a segment > 0")
        names = [w.name for w in self.wires]
        if len(set(names)) != len(names) or any(w.segments < 2 for w in self.wires):
            raise ValueError(f"{self.name}: wire names must be unique and each wire needs at least 2 links")
        taken = {*names, *(p.name for p in self.parts), "physics", "wire_physics"}
        for p in self.looks:
            if p.collides or p.name in taken or not p.name.isidentifier():
                raise ValueError(f"{self.name}: look {p.name!r} must be drawn only (collides=False), with a name of its own")
            taken.add(p.name)
            _check_mesh(f"{self.name}.{p.name}", p.mesh)
        for w in self.wires:
            if w.points is None:
                continue
            pts = np.asarray(w.points, float)
            chords = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            if len(pts) != w.segments + 1 or chords.min() <= 0:
                raise ValueError(f"{self.name}/{w.name}: points must be segments + 1 distinct points")
            if not self.flow_m and np.ptp(chords) > 1e-6:  # the copper rule allows a half link at each end
                raise ValueError(f"{self.name}/{w.name}: points must be segments + 1 points with equal link lengths")
            if not (np.allclose(pts[0], w.root, atol=1e-6) and np.allclose(pts[-1], self.terminal_local(w), atol=1e-6)):
                raise ValueError(f"{self.name}/{w.name}: points must start at root and end at the terminal")

    @staticmethod
    def link_name(wire: str, i: int) -> str:
        return f"{wire}_{i:02d}"

    def terminal_local(self, wire: HarnessWire) -> np.ndarray:
        """The wire's terminal in the harness frame at the start."""
        return np.asarray(self.device_at, float) + np.asarray(wire.terminal, float)

    def start_points(self, wire: HarnessWire) -> np.ndarray:
        """The wire's joint points at the start, harness frame (segments + 1, 3)."""
        if wire.points is not None:
            return np.asarray(wire.points, float)
        a, b = np.asarray(wire.root, float), self.terminal_local(wire)
        return a + np.linspace(0.0, 1.0, wire.segments + 1)[:, None] * (b - a)

    def link_length(self, wire: HarnessWire) -> float:
        return float(np.mean(np.linalg.norm(np.diff(self.start_points(wire), axis=0), axis=1)))

    def link_lengths(self, wire: HarnessWire) -> np.ndarray:
        """Each link's length (segments,): equal unless the wire's points give a half link at each end."""
        return np.linalg.norm(np.diff(self.start_points(wire), axis=0), axis=1)

    def joint_count(self) -> int:
        """Bending joints in all: one per link, less each wire's fixed first link when ``rigid_root``."""
        return sum(w.segments - (1 if self.rigid_root else 0) for w in self.wires)

    def link_frames(self, wire: HarnessWire) -> list:
        """Each link's rotation (3x3) at the start: x along the link, root to terminal; y and z carried from link to
        link with the least turn (no twist), starting level (y horizontal) at the root."""
        pts = self.start_points(wire)
        frames = []
        for i in range(wire.segments):
            u = pts[i + 1] - pts[i]
            u = u / np.linalg.norm(u)
            if not frames:
                side = np.cross(Z_AXIS, u)
                side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-6 else Y_AXIS
            else:
                prev = frames[-1]
                side = prev[:, 1] - (prev[:, 1] @ u) * u  # the last link's y, made square to this link
                side = side / np.linalg.norm(side)
            frames.append(np.column_stack([u, side, np.cross(u, side)]))
        return frames

    def wire_frame(self, wire: HarnessWire) -> np.ndarray:
        """The first link's rotation at the start (every link's, for a straight wire)."""
        return self.link_frames(wire)[0]

    def link_centres(self, wire: HarnessWire) -> np.ndarray:
        """Each link's centre at the start, harness frame (segments, 3)."""
        pts = self.start_points(wire)
        return (pts[:-1] + pts[1:]) / 2

    def dof_count(self) -> int:
        per_joint = 2 if self.style == "d6" else 1
        return per_joint * self.joint_count()

    def device_corners(self, pos, quat_xyzw) -> np.ndarray:
        rot = quat_xyzw_to_matrix(quat_xyzw)
        corners = [_box_corners(p.at, p.size, np.zeros(3), (0.0, 0.0, 0.0, 1.0)) for p in self.parts]
        return np.vstack(corners) @ rot.T + np.asarray(pos, float)

    def state_at(self, device_pos=None, device_quat=None, links: dict | None = None, lin_vel=None,
                 ang_vel=None) -> HarnessState:
        """The state with the device at ``device_pos`` / ``device_quat`` (world) and each wire's links at
        ``links[name] = (centres (n, 3), quats (n, 4))`` (world); by default everything where it starts."""
        origin = np.asarray(self.pos, float)
        dpos = origin + np.asarray(self.device_at, float) if device_pos is None else np.asarray(device_pos, float)
        dquat = np.array([0.0, 0.0, 0.0, 1.0]) if device_quat is None else np.asarray(device_quat, float)
        polylines = {}
        for w in self.wires:
            half = self.link_lengths(w)[:, None] / 2
            if links is None or w.name not in links:
                c = self.link_centres(w) + origin
                axes = np.array([f[:, 0] for f in self.link_frames(w)])
            else:
                c, q = (np.asarray(v, float) for v in links[w.name])
                axes = np.array([quat_xyzw_to_matrix(qi)[:, 0] for qi in q])
            polylines[w.name] = np.vstack([c - half * axes, c[-1:] + half[-1:] * axes[-1:]])
        corners = self.device_corners(dpos, dquat)
        return HarnessState(pos=dpos, quat_xyzw=dquat, lin_vel=np.zeros(3) if lin_vel is None else np.asarray(lin_vel),
                            ang_vel=np.zeros(3) if ang_vel is None else np.asarray(ang_vel), low=corners.min(0),
                            high=corners.max(0), polylines=polylines)


@dataclass(frozen=True)
class HarnessState(ObjectState):
    """A harness: ``pos``/``quat_xyzw`` the device's origin and rotation, ``lin_vel``/``ang_vel`` the device's,
    ``low``/``high`` bound the device; ``polylines[wire]`` each wire's joint points from root to terminal (n + 1, 3),
    world."""

    polylines: dict | None = None


def plastic_targets(q, target, theta_y: float) -> np.ndarray:
    """Elastic-perfectly-plastic joints: each rest angle in ``target`` is dragged to within ``theta_y`` of the joint
    angle ``q`` (both vectors, rad)."""
    q, target = np.asarray(q, float), np.asarray(target, float)
    return np.clip(target, q - theta_y, q + theta_y)


KitObject = Container | Marked | Model | Button | Lamp | Hinge | Slider | Hand | Towel | Prop | HangingBulb | Harness | Built


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


def _flat(waypoints, t: float):
    """The waypoints to walk at time ``t``: a :class:`Branch` is replaced by its chosen option once reached."""
    out, elapsed = [], 0.0
    for w in waypoints:
        if isinstance(w, Branch):
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
_RESET: list = []
_LIT = {"on": False}
_GROUP: dict = {"active": None}
_SEED: dict = {"seed": None}
_CLOCK: dict = {"t": 0.0}
_SWITCHES: list = []  # this trial's timed lamp switches: [at (s), group, applied]
_LATCH: dict = {}  # hinge name -> latched at the last step
_GLOVE: dict = {}  # hand name -> glove on this trial (absent: the Hand's ``gloved``)
_STEP: list = []  # task hooks run every control step: fn(environment)
_HELD: dict = {}  # (object name, joint name) -> the value it is held at
HOLD_STIFFNESS = 20000.0  # N/m: a held slider gives under 1 mm to 20 N
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


def on_step(fn):
    """Run ``fn(environment)`` after every control step, before ``success()`` and ``score()`` see the states. It can
    read states with ``labkit.state(environment, name)`` and act on the scene (:func:`hold_slider`,
    :func:`scale_part`). Offline tests call it with ``environment`` None."""
    _STEP.append(fn)
    return fn


def state(environment, name: str):
    """The current state of kit object ``name`` (what ``score()`` gets for it)."""
    return _state(environment, _SPECS[name])


_PHYSICS: list = []
PHYSICS_ERRORS: dict = {}  # what failed in place()/push(), by name: the first error of each kind, for the task's log


def on_physics(fn):
    """Run ``fn(environment)`` before every physics step (the simulator takes several per control step): for task
    code that stands in for a mechanism and must keep up with moving bodies (a screw riding on a part, a thread).
    It needs at least one :class:`Built` body in the scene (the hook rides on its ``write_data_to_sim``, which Isaac
    Lab calls before each physics step)."""
    _PHYSICS.append(fn)
    return fn


def _hook_physics(environment) -> None:
    if not _PHYSICS:
        return
    base = environment.unwrapped
    name = next((n for n, sp in _SPECS.items() if isinstance(sp, Built)), None)
    if name is None:
        return
    anchor = base.scene[name]
    if getattr(anchor, "_labkit_physics", False):
        return
    orig = anchor.write_data_to_sim

    def write(*args, **kwargs):
        out = orig(*args, **kwargs)
        for fn in _PHYSICS:
            fn(environment)
        return out

    anchor.write_data_to_sim = write
    anchor._labkit_physics = True


def place(environment, name: str, pos, quat_xyzw) -> bool:
    """Put :class:`Built` body ``name`` at ``pos`` (world, m) turned by ``quat_xyzw``, now. For a ``driven`` body
    this is how it moves. Returns False (and notes why in PHYSICS_ERRORS) if the simulator refused."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins)[0]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin[:3]), *np.asarray(quat_xyzw, float)]], dtype=torch.float32,
                        device=base.device)
    err = None
    for fn_name in ("write_root_link_pose_to_sim_index", "write_root_pose_to_sim_index", "write_root_link_pose_to_sim",
                    "write_root_pose_to_sim"):
        fn = getattr(asset, fn_name, None)
        if fn is None:
            continue
        for call in (lambda f=fn: f(root_pose=pose), lambda f=fn: f(pose)):
            try:
                call()
                return True
            except Exception as exc:  # noqa: BLE001 - try the next spelling of the same write
                err = exc
    PHYSICS_ERRORS.setdefault(f"place {name}", f"{type(err).__name__}: {err}" if err else "no pose writer found")
    return False


def push(environment, name: str, force, at) -> bool:
    """Push free :class:`Built` body ``name`` with ``force`` (N, world) at world point ``at`` for the coming physics
    step. Returns False (and notes why in PHYSICS_ERRORS) if the simulator refused."""
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins)[0][:3]
    force = np.asarray(force, float)
    try:
        import warp as wp

        com = _np(asset.data.body_com_pos_w)[0].reshape(-1)[:3] - origin if hasattr(asset.data, "body_com_pos_w") \
            else _np(asset.data.root_link_pose_w)[0][:3] - origin
        torque = np.cross(np.asarray(at, float) - com, force)
        asset.root_view.apply_forces_and_torques_at_position(
            force_data=wp.array(force.astype(np.float32), dtype=wp.float32, device=base.device),
            torque_data=wp.array(torque.astype(np.float32), dtype=wp.float32, device=base.device),
            position_data=None, indices=asset._ALL_INDICES, is_global=True)
        return True
    except Exception as exc:  # noqa: BLE001 - a push is an assist: the trial goes on without it
        PHYSICS_ERRORS.setdefault(f"push {name}", f"{type(exc).__name__}: {exc}")
        return False


def push_link(environment, name: str, link: int, force, at) -> bool:
    """Push link number ``link`` (its place in the body's ``body_names``) of jointed body ``name`` with ``force`` (N,
    world) at world point ``at`` for the coming physics step. Returns False (and notes why in PHYSICS_ERRORS) if the
    simulator refused."""
    return push_link_all(environment, name, link, [(force, at)])


def push_link_all(environment, name: str, link: int, pushes) -> bool:
    """Several pushes [(force, world point), ...] on one link of jointed body ``name`` for the coming physics step, sent
    as one force and one torque. An empty list sends none (which clears the last one)."""
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins)[0][:3]
    try:
        import warp as wp

        n = len(asset.body_names)
        com = _np(asset.data.body_com_pos_w)[0].reshape(n, -1)[link][:3] - origin
        forces, torques = np.zeros((n, 3), np.float32), np.zeros((n, 3), np.float32)
        for f, p in pushes:  # summed: a second call in the same physics step would replace the first one's force
            f = np.asarray(f, float)
            forces[link] += f
            torques[link] += np.cross(np.asarray(p, float) - com, f)
        err = None
        for shape in ((n * 3,), (n, 3), (1, n, 3)):  # the tensor API takes one of these for a jointed body
            try:
                asset.root_view.apply_forces_and_torques_at_position(
                    force_data=wp.array(forces.reshape(shape), dtype=wp.float32, device=base.device),
                    torque_data=wp.array(torques.reshape(shape), dtype=wp.float32, device=base.device),
                    position_data=None, indices=asset._ALL_INDICES, is_global=True)
                return True
            except Exception as exc:  # noqa: BLE001
                err = exc
        raise err
    except Exception as exc:  # noqa: BLE001 - a push is an assist: the trial goes on without it
        PHYSICS_ERRORS.setdefault(f"push_link {name}", f"{type(exc).__name__}: {exc}")
        return False


def _joints(spec) -> tuple:
    """The joint names of a jointed kit object, in the order its docs give them."""
    if isinstance(spec, HangingBulb):
        return HangingBulb.JOINTS
    return ("hinge",) if isinstance(spec, Hinge) else ("slide",)


def _joint_rest(spec, joint: str) -> float:
    return spec.joint_rest(joint) if isinstance(spec, HangingBulb) else spec.rest


def _joint_spring(spec, joint: str) -> float:
    return spec.joint_spring(joint) if isinstance(spec, HangingBulb) else (spec.spring or 0.0)


def _angular(spec, joint: str) -> bool:
    return isinstance(spec, Hinge) or (isinstance(spec, HangingBulb) and not joint.startswith("squeeze"))


def held(name: str, joint: str | None = None) -> float | None:
    """The value joint ``joint`` of ``name`` is held at (default: its only joint), or None if it is free."""
    return _HELD.get((name, joint or _joints(_SPECS[name])[0]))


def _call_first(obj, names, value) -> None:
    """Call the first of ``names`` that ``obj`` has, with ``value`` by keyword then by position (Isaac Lab 3 renamed
    many writers to ``*_index`` with keyword arguments)."""
    for n in names:
        fn = getattr(obj, n, None)
        if fn is None:
            continue
        for kw in ("stiffness", "target", "position", None):
            try:
                return fn(value) if kw is None else fn(**{kw: value})
            except TypeError:
                continue
    raise RuntimeError(f"none of {names} worked on {type(obj).__name__}: check the Isaac Lab articulation API")


def _apply_drives(environment, name: str) -> None:
    """Write every joint drive of ``name``: held joints stiff at their held value, the rest on their own springs
    toward rest. Whole vectors, in the articulation's joint order, so no joint ids are needed."""
    import torch

    spec = _SPECS[name]
    asset = getattr(environment, "unwrapped", environment).scene[name]
    stiff, goal = [], []
    for joint in asset.joint_names:
        value = _HELD.get((name, joint))
        if value is None:
            stiff.append(_joint_spring(spec, joint))
            goal.append(0.0)
        else:
            stiff.append(HOLD_STIFFNESS_ANGULAR if _angular(spec, joint) else HOLD_STIFFNESS)
            goal.append(value - _joint_rest(spec, joint))
    writers = (("write_joint_stiffness_to_sim_index", "write_joint_stiffness_to_sim"),
               ("set_joint_position_target_index", "set_joint_position_target"))
    for names in writers:  # check both exist before changing anything, so a failure leaves the springs as they were
        if not any(hasattr(asset, n) for n in names):
            raise RuntimeError(f"the articulation has none of {names}")
    _call_first(asset, writers[0], torch.tensor([stiff], dtype=torch.float32, device=asset.device))
    _call_first(asset, writers[1], torch.tensor([goal], dtype=torch.float32, device=asset.device))


_PINNED: set = set()  # objects whose drives couldn't be rewritten: their held joints are re-set every step instead
HOLD_STIFFNESS_ANGULAR = 5.0  # N m/rad: a held hinge gives under 1 degree to 0.1 N m


def _joint_value(environment, name: str, joint: str) -> float:
    s = state(environment, name)
    if isinstance(_SPECS[name], HangingBulb):
        return {"squeeze_left": s.squeeze_left, "squeeze_right": s.squeeze_right, "cap_hinge": s.cap_angle,
                "swing_u": s.swing[0], "swing_v": s.swing[1]}[joint]
    return s.angle if isinstance(_SPECS[name], Hinge) else s.position


def hold_joint(environment, name: str, value: float | None = None, joint: str | None = None) -> None:
    """Hold joint ``joint`` of ``name`` (default: its only joint) at ``value`` (m or rad; default where it is now),
    both ways, until :func:`release_joint` or the next reset. First choice: a stiff drive (the joints' stiffness and
    targets rewritten through Isaac Lab's articulation API). If this Isaac Lab doesn't have those writers, the held
    joints are instead put back at their values every control step with ``write_joint_position_to_sim_index``, the
    call labkit's reset already relies on; between steps a spring may move them, so read a held joint with
    :func:`held`. Offline (``environment`` None) it only records the hold."""
    spec = _SPECS[name]
    joint = joint or _joints(spec)[0]
    if value is None:
        value = _joint_value(environment, name, joint)
    _HELD[(name, joint)] = float(value)
    if environment is None:
        return
    if name not in _PINNED:
        try:
            _apply_drives(environment, name)
            return
        except (RuntimeError, AttributeError, TypeError) as err:
            print(f"labkit: holding {name} by re-setting its joints each step ({err})", flush=True)
            _PINNED.add(name)
    _pin(environment, name)


def _pin(environment, name: str) -> None:
    """Put the held joints of ``name`` back at their held values, at rest; its other joints are left as they are."""
    spec = _SPECS[name]
    asset = getattr(environment, "unwrapped", environment).scene[name]
    pos = getattr(asset.data.joint_pos, "torch", asset.data.joint_pos).clone()
    vel = getattr(asset.data.joint_vel, "torch", asset.data.joint_vel).clone()
    for i, joint in enumerate(asset.joint_names):
        if (name, joint) in _HELD:
            pos[:, i] = _HELD[(name, joint)] - _joint_rest(spec, joint)
            vel[:, i] = 0.0
    asset.write_joint_position_to_sim_index(position=pos)
    asset.write_joint_velocity_to_sim_index(velocity=vel)


def release_joint(environment, name: str, joint: str | None = None) -> None:
    """Give joint ``joint`` of ``name`` (default: its only joint) back its own spring."""
    joint = joint or _joints(_SPECS[name])[0]
    _HELD.pop((name, joint), None)
    if environment is not None and name not in _PINNED:
        _apply_drives(environment, name)


hold_slider, release_slider = hold_joint, release_joint  # the names the first version used


def scale_part(environment, name: str, part: str, scale, at=None) -> None:
    """Rescale part ``part`` of prop ``name`` (x, y, z factors about the part's own origin), and move it to ``at``
    (prop frame) if given. Visual parts only: a collider would not follow. Offline (``environment`` None) it does
    nothing."""
    if environment is None:
        return
    from pxr import Gf, UsdGeom

    prim = _find_prim(environment, name, part)
    if prim is None:
        raise RuntimeError(f"no prim for {name}/{part}")
    xform = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    (ops.get("xformOp:scale") or xform.AddScaleOp()).Set(Gf.Vec3f(*(float(v) for v in scale)))
    if at is not None:
        (ops.get("xformOp:translate") or xform.AddTranslateOp()).Set(Gf.Vec3d(*(float(v) for v in at)))


@on_reset
def _start_trial() -> None:
    """Clear the last trial's clock, timed switches, latches and gloves (registered first, so a task's own on_reset
    hooks run after it)."""
    _CLOCK["t"] = 0.0
    _SWITCHES.clear()
    _LATCH.clear()
    _GLOVE.clear()
    _HELD.clear()
    _PINNED.clear()
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


def _look(stage, path: str, color: Color, roughness: float = 0.5, metallic: float = 0.0) -> str:
    import isaaclab.sim as sim_utils

    from yam_arena.station import linear_rgb

    metal = {"metallic": float(metallic)} if metallic else {}  # only named when asked for: other looks are as before
    vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=roughness, **metal)
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


def _mesh_prim(stage, path: str, mesh, at, look: str | None, physics: str | None = None,
               offset: float = 0.003) -> None:
    """A generated mesh under a body or prop, translated to ``at`` with a unit scale op (for :func:`scale_part`): a
    convex-hull collider if ``physics`` is given, else visual only."""
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, Vt

    points, counts, indices = mesh
    geom = UsdGeom.Mesh.Define(stage, path)
    geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    geom.CreateFaceVertexCountsAttr(Vt.IntArray(list(counts)))
    geom.CreateFaceVertexIndicesAttr(Vt.IntArray(list(indices)))
    geom.CreateSubdivisionSchemeAttr("none")
    geom.CreateDoubleSidedAttr(True)
    geom.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in at)))
    geom.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    if physics is not None:
        prim = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        contact.CreateContactOffsetAttr(offset)
        contact.CreateRestOffsetAttr(0.0)
        bind_physics_material(path, physics, stage=stage)
    if look is not None:
        bind_visual_material(path, look, stage=stage)


def _prop_parts(prim_path: str, spec: Prop, translation=None, orientation=None):
    """A fixed prop: its generated meshes (visual, or convex colliders when marked) and an optional box collider."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    root = _root_xform(stage, prim_path, translation, orientation)
    physics = _grippy(prim_path)
    if spec.box_collider:
        _cube(stage, f"{prim_path}/collider", spec.size, (0.0, 0.0, 0.0), None, physics)
    for p in spec.parts:
        look = _look(stage, f"{prim_path}/Looks/{p.name}", p.color, p.roughness, p.metallic)
        _mesh_prim(stage, f"{prim_path}/{p.name}", p.mesh, p.at, look, physics if p.collides else None)
    return root.GetPrim()


def _grippy(prim_path: str, friction: tuple[float, float] | None = None, mix: str | None = None) -> str:
    import isaaclab.sim as sim_utils

    from yam_arena.constants import GRIPPY_FRICTION

    static, dynamic = friction or GRIPPY_FRICTION
    physics = f"{prim_path}/Looks/physics"
    grippy = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic,
                                            **({"friction_combine_mode": mix} if mix else {}))
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


def _built_parts(prim_path: str, spec: Built) -> None:
    """A built body's meshes, on the body whose box is already spawned: visual, or convex colliders when marked."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    physics = _grippy(prim_path, spec.friction, spec.friction_mix)
    for p in spec.parts:
        look = _look(stage, f"{prim_path}/Looks/{p.name}", p.color, p.roughness, p.metallic)
        _mesh_prim(stage, f"{prim_path}/{p.name}", p.mesh, p.at, look, physics if p.collides else None, offset=0.002)
    if spec.apart_from:  # no contact between this body and those: PhysX filters the pair
        from pxr import Sdf, UsdPhysics

        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(prim_path)).CreateFilteredPairsRel()
        for other in spec.apart_from:
            pairs.AddTarget(Sdf.Path(f"{prim_path.rsplit('/', 1)[0]}/{other}"))


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
    frame_physics = physics if isinstance(spec, Hinge) or spec.frame_collides else None
    _cube(stage, f"{base}/box", spec.size, (0.0, 0.0, 0.0), _look(stage, f"{prim_path}/Looks/base", spec.color),
          frame_physics)
    hinge = isinstance(spec, Hinge)
    if hinge:
        part, size, color, mass = f"{prim_path}/lid", spec.lid, spec.lid_color, spec.lid_mass
        _rigid(stage, part, spec.pivot_point, _turn(spec.axis, spec.start), mass)
        look = _look(stage, f"{prim_path}/Looks/lid", color)
        if spec.lid_mesh is not None:
            _mesh_prim(stage, f"{part}/mesh", spec.lid_mesh, spec.lid_offset, look, physics)
        else:
            _cube(stage, f"{part}/box", size, spec.lid_offset, look, physics)
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
        look = _look(stage, f"{prim_path}/Looks/panel", color) if spec.panel_visible else None
        _cube(stage, f"{part}/box", size, (0.0, 0.0, 0.0), look, physics)
        if not spec.panel_visible:
            from pxr import UsdGeom

            UsdGeom.Imageable(stage.GetPrimAtPath(f"{part}/box")).MakeInvisible()
        if spec.no_collide_with:  # sibling sliders' panels: pass through each other (one bulb, two faces)
            parent = prim_path.rsplit("/", 1)[0]
            pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(part)).CreateFilteredPairsRel()
            for other in spec.no_collide_with:
                pairs.AddTarget(Sdf.Path(f"{parent}/{other}/panel"))
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


def _bulb_parts(prim_path: str, spec: HangingBulb, translation=None, orientation=None):
    """The hanging bulb as one articulation: anchor (welded at the hanging point) -swing_u-> gimbal -swing_v-> core
    (the bulb: port collider, generated parts), and on the core: two faces (prismatic squeeze_left / squeeze_right)
    and the cap (revolute cap_hinge). Joint zeros are the springs' rests, so zero drive targets are the rests."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    stage = get_current_stage()
    root = _root_xform(stage, prim_path, translation, orientation)
    physics = _grippy(prim_path)
    anchor, gimbal, core = f"{prim_path}/anchor", f"{prim_path}/gimbal", f"{prim_path}/core"
    _rigid(stage, anchor, (0.0, 0.0, 0.0), np.eye(3), BASE_MASS)
    _rigid(stage, gimbal, (0.0, 0.0, 0.0), np.eye(3), 0.005)
    _rigid(stage, core, (0.0, 0.0, 0.0), np.eye(3), spec.core_mass)
    # The anchor and gimbal have no shapes to take inertia from, and the core's only collider is its small port: set
    # their mass properties outright, the core's centre of mass at the bulb's centre (a 40 g oval about 6 x 5 x 10 cm).
    for path, inertia, com in ((anchor, 1e-4, (0.0, 0.0, 0.0)), (gimbal, 1e-6, (0.0, 0.0, 0.0)),
                               (core, 3e-5, (0.0, 0.0, -spec.drop))):
        mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(path))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    _cube(stage, f"{core}/port", spec.port, spec.port_offset, _look(stage, f"{prim_path}/Looks/port", spec.port_color),
          physics)
    for p in spec.parts:
        look = _look(stage, f"{prim_path}/Looks/{p.name}", p.color, p.roughness, p.metallic)
        _mesh_prim(stage, f"{core}/{p.name}", p.mesh, p.at, look, physics if p.collides else None)

    def joint(kind, path, axis, body0, body1, at0, rot0, lower, upper, stiffness, damping, linear=False):
        j = (UsdPhysics.PrismaticJoint if linear else UsdPhysics.RevoluteJoint).Define(stage, path)
        j.CreateAxisAttr(axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in at0)))
        j.CreateLocalRot0Attr(Gf.Quatf(float(rot0[3]), float(rot0[0]), float(rot0[1]), float(rot0[2])))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(float(lower))
        j.CreateUpperLimitAttr(float(upper))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear" if linear else "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(stiffness if linear else stiffness * math.pi / 180)  # USD angular gains: per degree
        drive.CreateDampingAttr(damping if linear else damping * math.pi / 180)
        drive.CreateTargetPositionAttr(0.0)

    ident = np.array([0.0, 0.0, 0.0, 1.0])
    swing = math.degrees(spec.max_swing)
    joint("swing", f"{gimbal}/swing_u", "X", anchor, gimbal, (0, 0, 0), ident, -swing, swing, spec.swing_spring,
          spec.swing_damping)
    joint("swing", f"{core}/swing_v", "Y", gimbal, core, (0, 0, 0), ident, -swing, swing, spec.swing_spring,
          spec.swing_damping)
    for link, sign, lo, hi in (("face_left", 1.0, -spec.travel, 0.0), ("face_right", -1.0, 0.0, spec.travel)):
        at = (0.0, sign * spec.face_at, -spec.drop)
        path = f"{prim_path}/{link}"
        _rigid(stage, path, at, np.eye(3), spec.face_mass, gravity=False)
        _cube(stage, f"{path}/box", spec.paddle, (0.0, 0.0, 0.0), None, physics)
        UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/box")).MakeInvisible()
        joint("squeeze", f"{path}/squeeze_{link.split('_')[1]}", "Y", core, path, at, ident, lo, hi, spec.spring,
              spec.face_damping, linear=True)
    cap = f"{prim_path}/cap"
    hinge_at = spec.cap_hinge_point
    _rigid(stage, cap, hinge_at, _turn("x", spec.cap_start), spec.cap_mass)
    look = _look(stage, f"{prim_path}/Looks/cap", spec.cap_color)
    if spec.cap_mesh is not None:
        _mesh_prim(stage, f"{cap}/mesh", spec.cap_mesh, spec.cap_at, look, physics)
    else:
        _cube(stage, f"{cap}/box", spec.cap_size, spec.cap_at, look, physics)
    lo, hi = (math.degrees(v - spec.cap_rest) for v in spec.cap_limits)
    joint("cap", f"{cap}/cap_hinge", "X", core, cap, hinge_at, ik.matrix_to_quat_xyzw(_turn("x", spec.cap_rest)), lo,
          hi, spec.cap_spring, spec.cap_damping)
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(False)  # faces, cap and port overlap by design; the joint limits are the stops
    art.CreateSolverPositionIterationCountAttr(32)
    _weld_to_world(stage, prim_path, anchor)
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


def _capsule(stage, path: str, radius: float, length: float, look: str | None, physics: str | None,
             offset: float = 0.001) -> None:
    """A capsule along the body's x axis, ``length`` between its cap centres, as a collider (and visual)."""
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import PhysxSchema, UsdGeom, UsdPhysics

    cap = UsdGeom.Capsule.Define(stage, path)
    cap.CreateAxisAttr("X")
    cap.CreateRadiusAttr(float(radius))
    cap.CreateHeightAttr(float(length))
    if physics is not None:
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim())
        contact.CreateContactOffsetAttr(offset)
        contact.CreateRestOffsetAttr(0.0)
        bind_physics_material(path, physics, stage=stage)
    if look is not None:
        bind_visual_material(path, look, stage=stage)


def _quatf(q_xyzw):
    from pxr import Gf

    x, y, z, w = (float(v) for v in q_xyzw)
    return Gf.Quatf(w, x, y, z)


def _harness_parts(prim_path: str, spec: Harness, translation=None, orientation=None):
    """The harness as one fixed-base articulation (see :class:`Harness`). Every joint's zero is the straight start,
    so zero drive targets are the unbent wire."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    stage = get_current_stage()
    root = _root_xform(stage, prim_path, translation, orientation)
    anchor, device = f"{prim_path}/anchor", f"{prim_path}/device"
    _rigid(stage, anchor, (0.0, 0.0, 0.0), np.eye(3), BASE_MASS)
    mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(anchor))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-4, 1e-4, 1e-4))  # no shapes to take inertia from
    _rigid(stage, device, spec.device_at, np.eye(3), spec.device_mass)
    physics = _grippy(prim_path)
    for p in spec.parts:
        _cube(stage, f"{device}/{p.name}", p.size, p.at, _look(stage, f"{prim_path}/Looks/{p.name}", p.color),
              physics, offset=0.002)
    for p in spec.looks:  # drawn only: no physics material is passed, so the mesh gets no collision API
        _mesh_prim(stage, f"{device}/{p.name}", p.mesh, p.at,
                   _look(stage, f"{prim_path}/Looks/{p.name}", p.color, p.roughness, p.metallic))
    import isaaclab.sim as sim_utils

    wire_physics = f"{prim_path}/Looks/wire_physics"  # insulation on plastic and on itself: less grippy than pads
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=spec.wire_friction, dynamic_friction=spec.wire_friction)
    mat.func(wire_physics, mat)
    ident = np.array([0.0, 0.0, 0.0, 1.0])

    def lock(prim, dofs) -> None:
        for dof in dofs:
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)  # low > high locks the axis

    def bend_joint(path: str, body0: str, body1: str, pos0, rot0, half: float, k: int) -> None:
        if spec.style == "d6":
            j = UsdPhysics.Joint.Define(stage, path)
        else:
            j = UsdPhysics.RevoluteJoint.Define(stage, path)
            j.CreateAxisAttr("Y" if k % 2 == 0 else "Z")
            j.CreateLowerLimitAttr(-math.degrees(spec.max_bend))
            j.CreateUpperLimitAttr(math.degrees(spec.max_bend))
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in pos0)))
        j.CreateLocalRot0Attr(_quatf(rot0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-half, 0.0, 0.0))
        j.CreateLocalRot1Attr(_quatf(ident))
        prim = j.GetPrim()
        if spec.style == "d6":
            lock(prim, ("transX", "transY", "transZ", "rotX"))  # no stretch, no twist
            dofs = ("rotY", "rotZ")
            for dof in dofs:
                lim = UsdPhysics.LimitAPI.Apply(prim, dof)
                lim.CreateLowAttr(-math.degrees(spec.max_bend))
                lim.CreateHighAttr(math.degrees(spec.max_bend))
        else:
            dofs = ("angular",)
        for dof in dofs:
            drive = UsdPhysics.DriveAPI.Apply(prim, dof)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(spec.stiffness * math.pi / 180)  # USD angular gains are per degree
            drive.CreateDampingAttr(spec.damping * math.pi / 180)
            drive.CreateTargetPositionAttr(0.0)

    def fixed_joint(path: str, body0: str, body1: str, pos0, rot0, half: float) -> None:
        j = UsdPhysics.FixedJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in pos0)))
        j.CreateLocalRot0Attr(_quatf(rot0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-half, 0.0, 0.0))
        j.CreateLocalRot1Attr(_quatf(ident))

    for n, w in enumerate(spec.wires):
        lengths, frames = spec.link_lengths(w), spec.link_frames(w)
        full = float(np.max(lengths))
        look = _look(stage, f"{prim_path}/Looks/{w.name}", w.color, 0.4)
        links = [f"{prim_path}/{spec.link_name(w.name, i)}" for i in range(w.segments)]
        for i, (path, c) in enumerate(zip(links, spec.link_centres(w))):
            length = float(lengths[i])
            body = _rigid(stage, path, c, frames[i], w.link_mass * length / full)  # link_mass is a whole link's
            body.CreateMaxDepenetrationVelocityAttr(0.5)
            body.CreateLinearDampingAttr(0.5)
            body.CreateAngularDampingAttr(2.0)
            _capsule(stage, f"{path}/wire", w.radius, length, look, wire_physics)
            # Joint frames: frame 1 is the link's own; frame 0 in the parent is turned so both agree at the start, so
            # a zero angle is the start shape (a pre-bent wire is relaxed as it spawns).
            if i == 0 and spec.rigid_root:
                fixed_joint(f"{path}/clamp", anchor, path, w.root, ik.matrix_to_quat_xyzw(frames[0]), length / 2)
            elif i == 0:
                bend_joint(f"{path}/bend", anchor, path, w.root, ik.matrix_to_quat_xyzw(frames[0]), length / 2, 0)
            else:
                rel = ik.matrix_to_quat_xyzw(frames[i - 1].T @ frames[i])
                bend_joint(f"{path}/bend", links[i - 1], path, (float(lengths[i - 1]) / 2, 0.0, 0.0), rel, length / 2, i)
        end = UsdPhysics.FixedJoint.Define(stage, f"{links[-1]}/{'terminal' if n == 0 else 'loop_terminal'}")
        end.CreateBody0Rel().SetTargets([Sdf.Path(links[-1])])
        end.CreateBody1Rel().SetTargets([Sdf.Path(device)])
        end.CreateLocalPos0Attr(Gf.Vec3f(float(lengths[-1]) / 2, 0.0, 0.0))
        end.CreateLocalRot0Attr(_quatf(ident))
        end.CreateLocalPos1Attr(Gf.Vec3f(*(float(v) for v in w.terminal)))
        # The device starts upright, so device frame = harness frame: the last link's start rotation.
        end.CreateLocalRot1Attr(_quatf(ik.matrix_to_quat_xyzw(frames[-1])))
        if n > 0:
            end.CreateExcludeFromArticulationAttr(True)
        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(device)).CreateFilteredPairsRel()
        for path in links[-spec.end_filter:] if spec.end_filter else []:
            pairs.AddTarget(Sdf.Path(path))
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(True)  # wires push on each other and on the device; jointed neighbours skip
    art.CreateSolverPositionIterationCountAttr(64)
    art.CreateSolverVelocityIterationCountAttr(4)
    _weld_to_world(stage, prim_path, anchor)
    return root.GetPrim()


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

    if isinstance(spec, Built):

        @clone
        def spawn_built(prim_path: str, cfg, *args, **kwargs):
            prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)  # the body and its own box
            _built_parts(prim_path, spec)
            return prim

        static, dynamic = spec.friction or GRIPPY_FRICTION
        cfg = sim_utils.CuboidCfg(
            size=tuple(spec.size), func=spawn_built,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4),
            collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=static, dynamic_friction=dynamic,
                **({"friction_combine_mode": spec.friction_mix} if spec.friction_mix else {})),
            rigid_props=PhysxRigidBodyPropertiesCfg(kinematic_enabled=True) if spec.driven
            else PhysxRigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass))
        return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg, initial_pose=pose)

    if isinstance(spec, Model):
        cfg = sim_utils.UsdFileCfg(usd_path=spec.usd, scale=(spec.scale,) * 3, **{
            k: v for k, v in body.items() if k != "physics_material"})
        return Object(name=spec.name, object_type=kind, spawner_cfg=cfg, initial_pose=pose)

    if isinstance(spec, Lamp):
        cfg = sim_utils.CylinderCfg(
            radius=spec.radius,
            height=spec.size[2],
            visible=False,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.2,
                                                        emissive_color=linear_rgb(spec.color)),
        )
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)

    if isinstance(spec, HangingBulb):

        @configclass
        class BulbSpawnerCfg(SpawnerCfg):
            func: object = clone(lambda prim_path, cfg, *a, translation=None, orientation=None, **k: _bulb_parts(
                prim_path, spec, translation, orientation))

        class BulbObject(Object):
            def _generate_articulation_cfg(self):
                # Implicit actuators are the springs: the swing (strap and tubing), the faces, the cap's tether. Each
                # joint's zero is its rest, so the zero target Isaac Lab holds is the spring's rest.
                acts = {
                    "swing": ImplicitActuatorCfg(joint_names_expr=["swing_.*"], stiffness=spec.swing_spring,
                                                 damping=spec.swing_damping),
                    "squeeze": ImplicitActuatorCfg(joint_names_expr=["squeeze_.*"], stiffness=spec.spring,
                                                   damping=spec.face_damping),
                    "cap": ImplicitActuatorCfg(joint_names_expr=["cap_hinge"], stiffness=spec.cap_spring,
                                               damping=spec.cap_damping),
                }
                cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=acts)
                cfg = self._add_initial_pose_to_cfg(cfg)
                cfg.init_state.joint_pos = {"swing_.*": 0.0, "squeeze_.*": 0.0,
                                            "cap_hinge": spec.cap_start - spec.cap_rest}
                return cfg

        return BulbObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=BulbSpawnerCfg(),
                          initial_pose=pose)

    if isinstance(spec, Harness):

        @configclass
        class HarnessSpawnerCfg(SpawnerCfg):
            func: object = clone(lambda prim_path, cfg, *a, translation=None, orientation=None, **k: _harness_parts(
                prim_path, spec, translation, orientation))

        class HarnessObject(Object):
            def _generate_articulation_cfg(self):
                # Every joint is a wire bend: one implicit actuator, its targets the plastic rest angles (zero =
                # straight), which labkit's step hook drags along (_harness_plastic).
                # With torque_cap, a joint's drive can't push back harder than copper's yield moment at any PhysX
                # substep (the plastic rule itself only runs once per control step).
                fields = getattr(ImplicitActuatorCfg, "__dataclass_fields__", {})
                key = "effort_limit_sim" if not fields or "effort_limit_sim" in fields else "effort_limit"
                first = (float(np.interp(spec.cap_ahead, spec.flow_kp, spec.flow_m)) if spec.flow_m
                         else spec.torque_cap)  # the copper rule raises it as joints harden
                cap = {} if first is None else {key: float(first)}
                act = ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=spec.stiffness, damping=spec.damping,
                                          armature=spec.armature, **cap)
                cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators={"wire": act})
                cfg = self._add_initial_pose_to_cfg(cfg)
                cfg.init_state.joint_pos = {".*": 0.0}
                return cfg

        return HarnessObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=HarnessSpawnerCfg(),
                             initial_pose=pose)

    if isinstance(spec, Prop):

        @configclass
        class PropSpawnerCfg(SpawnerCfg):
            func: object = clone(lambda prim_path, cfg, *a, translation=None, orientation=None, **k: _prop_parts(
                prim_path, spec, translation, orientation))

        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=PropSpawnerCfg(), initial_pose=pose)

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
    if isinstance(spec, HangingBulb):
        return spec.state_at(np.asarray(spec.pos, float), _yaw_quat(spec.yaw), {})
    if isinstance(spec, Towel):
        return spec.state_at()
    if isinstance(spec, Harness):
        return spec.state_at()
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


def _harness_state(environment, spec: Harness) -> HarnessState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    names = list(asset.body_names)
    pos, quat = _np(asset.data.body_pos_w)[0] - origin, _np(asset.data.body_quat_w)[0]
    links = {}
    for w in spec.wires:
        idx = [names.index(spec.link_name(w.name, i)) for i in range(w.segments)]
        links[w.name] = (pos[idx], quat[idx])
    d = names.index("device")
    return spec.state_at(pos[d], quat[d], links, _np(asset.data.body_lin_vel_w)[0][d],
                         _np(asset.data.body_ang_vel_w)[0][d])


_PLASTIC: dict = {}  # harness name -> its joints' current rest angles (rad), cleared at reset


def harness_rest(name: str) -> np.ndarray | None:
    """The plastic rest angles of harness ``name``'s joints (the permanent bends), or None before its first step."""
    return _PLASTIC.get(name)


def _harness_plastic(environment) -> None:
    """Every control step: drag each harness joint's rest angle to within its yield angle of where the joint is, and
    give the actuators the new rest angles as targets (whole vectors, in the articulation's joint order)."""
    import torch

    base = getattr(environment, "unwrapped", environment)
    for name, spec in _SPECS.items():
        if not isinstance(spec, Harness):
            continue
        asset = base.scene[name]
        q = _np(asset.data.joint_pos)[0]
        rest = _PLASTIC.get(name, np.zeros_like(q))
        if spec.flow_m:  # the copper rule: a joint's axes are one bend, it hardens, and its torque cap follows
            import copper

            per = 2 if spec.style == "d6" else 1
            groups = np.arange(len(q)).reshape(-1, per)  # a joint's axes sit next to each other in the joint order
            acc = _HARDENED.get(name)
            if acc is None or len(acc) != len(groups):
                acc = _start_hardening(asset, spec, groups)
            new, acc, flow = copper.plastic_step(q, rest, acc, groups, spec.stiffness, spec.segment,
                                                 np.asarray(spec.flow_kp), np.asarray(spec.flow_m))
            _HARDENED[name] = acc
            cap = np.interp(acc + spec.cap_ahead, np.asarray(spec.flow_kp), np.asarray(spec.flow_m))
            _write_effort_limits(asset, np.repeat(cap, per))
        else:
            new = plastic_targets(q, rest, spec.yield_angle)
        _PLASTIC[name] = new
        if not np.array_equal(new, rest) or name not in _WROTE:
            _call_first(asset, ("set_joint_position_target_index", "set_joint_position_target"),
                        torch.tensor(new[None, :], dtype=torch.float32, device=asset.device))
            _WROTE.add(name)


_WROTE: set = set()
_HARDENED: dict = {}  # harness name -> the plastic curvature each joint has been through (1/m), cleared at reset
_LIMIT_WRITER: dict = {}  # articulation class -> (writer name, keyword) that worked


def _start_hardening(asset, spec: Harness, groups) -> np.ndarray:
    """Each joint's starting hardening, in the articulation's joint order. ``hardened0`` is in the order the joints
    were made (wire by wire, cable to outlet), which USD names bend, bend_0, bend_1, ...: match by name."""
    if len(spec.hardened0) != len(groups):
        return np.zeros(len(groups))
    made = []
    for g in groups:
        stem = str(asset.joint_names[int(g[0])]).split(":")[0]
        tail = stem.rsplit("_", 1)[-1]
        made.append(int(tail) + 1 if "_" in stem and tail.isdigit() else 0)
    if sorted(made) != list(range(len(groups))):  # names not as expected: fall back to the order as it is
        made = list(range(len(groups)))
    return np.asarray(spec.hardened0, float)[made]


def harness_hardening(name: str) -> np.ndarray | None:
    """The plastic curvature (1/m) each joint of harness ``name`` has been through, or None before its first step."""
    return _HARDENED.get(name)


def _write_effort_limits(asset, values) -> str:
    """Write a torque limit per joint axis (N m, in the joint order) while the sim runs. Returns the writer used, or
    raises if none works. The first working writer is remembered for the articulation's class."""
    import torch

    value = torch.tensor(np.asarray(values, np.float32)[None, :], dtype=torch.float32, device=asset.device)
    known = _LIMIT_WRITER.get(type(asset).__name__)
    tries = [known] if known else [(f, kw) for f in ("write_joint_effort_limit_to_sim_index",
                                                       "write_joint_effort_limit_to_sim")
                                    for kw in ("limits", "effort_limits", None)]
    errors = []
    for fn_name, kw in tries:
        fn = getattr(asset, fn_name, None)
        if fn is None:
            continue
        try:
            fn(value) if kw is None else fn(**{kw: value})
        except TypeError as err:
            errors.append(f"{fn_name}({kw}): {err}")
            continue
        _LIMIT_WRITER[type(asset).__name__] = (fn_name, kw)
        return f"{fn_name}({kw or 'positional'})"
    raise RuntimeError(f"no effort-limit writer worked on {type(asset).__name__}: {errors or 'none found'}")

UNCAPPED = 1.0e6  # N m: what set_harness_torque_cap writes for "no cap"


def set_harness_torque_cap(environment, name: str, cap: float | None) -> dict:
    """Set every joint drive's torque limit of harness ``name`` while the sim runs (None: no cap), and read it back.
    Returns which writer worked and what the articulation now reports, or raises if no writer worked."""
    import torch

    base = getattr(environment, "unwrapped", environment)
    asset = base.scene[name]
    n = len(asset.joint_names)
    value = torch.full((1, n), UNCAPPED if cap is None else float(cap), dtype=torch.float32, device=asset.device)
    errors = []
    for fn_name in ("write_joint_effort_limit_to_sim_index", "write_joint_effort_limit_to_sim"):
        fn = getattr(asset, fn_name, None)
        if fn is None:
            continue
        for kw in ("limits", "effort_limits", None):
            try:
                fn(value) if kw is None else fn(**{kw: value})
            except TypeError as err:
                errors.append(f"{fn_name}({kw}): {err}")
                continue
            back = None
            for attr in ("joint_effort_limits", "joint_effort_limit"):
                got = getattr(asset.data, attr, None)
                if got is not None:
                    try:  # only a read-back for the log: the write itself already worked
                        back = [float(v) for v in _np(got)[0][:3]]
                    except Exception as err:  # noqa: BLE001
                        back = f"unreadable: {type(err).__name__}"
                    break
            return {"writer": f"{fn_name}({kw or 'positional'})", "set": None if cap is None else float(cap),
                    "reads_back_first3": back}
    raise RuntimeError(f"no effort-limit writer worked on {type(asset).__name__}: {errors or 'none found'}")


def _state(environment, spec: KitObject) -> ObjectState:
    if isinstance(spec, Lamp | Hand):
        return start_state(spec)
    if isinstance(spec, Towel):
        return _towel_state(environment, spec)
    if isinstance(spec, Harness):
        return _harness_state(environment, spec)
    base = environment.unwrapped
    if isinstance(spec, HangingBulb):  # the anchor is welded; read the core's pose and every joint
        asset = base.scene[spec.name]
        q = _np(asset.data.joint_pos)[0]
        joints = {n: float(q[i]) + spec.joint_rest(n) for i, n in enumerate(asset.joint_names)}
        pose = _np(asset.data.body_link_pose_w)[0][list(asset.body_names).index("core")]
        was = _LATCH.get(spec.name, False)
        latched = spec.cap_latch.step(joints.get("cap_hinge", spec.cap_start), was)
        _LATCH[spec.name] = latched
        return spec.state_at(pose[:3] - _np(base.scene.env_origins)[0], pose[3:7], joints, latched)
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


_PRIMS: dict = {}  # (name, child) -> prim, found once


def _find_prim(environment, name: str, child: str = ""):
    """Object ``name``'s prim (or its ``child``) in env 0, or None."""
    if (name, child) in _PRIMS and _PRIMS[(name, child)].IsValid():
        return _PRIMS[(name, child)]
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    root = getattr(environment.unwrapped.scene, "env_prim_paths", ["/World/envs/env_0"])[0]
    prim = stage.GetPrimAtPath(f"{root}/{name}")
    if not prim.IsValid():  # fall back to finding it by name under env 0
        prim = next((p for p in stage.Traverse() if p.GetName() == name and str(p.GetPath()).startswith(root)), prim)
    if prim.IsValid() and child:
        prim = stage.GetPrimAtPath(prim.GetPath().AppendPath(child))
    if not prim.IsValid():
        return None
    _PRIMS[(name, child)] = prim
    return prim


def _set_visible(environment, name: str, show: bool, child: str = "") -> None:
    """Show or hide object ``name``'s prim (or its ``child``) in env 0."""
    from pxr import UsdGeom

    prim = _find_prim(environment, name, child)
    if prim is not None:
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
    if environment is not None:
        _harness_plastic(environment)  # before the task's hooks, so they see this step's permanent bends
    for fn in _STEP:
        fn(environment)
    for name in list(_PINNED):  # holds kept by re-setting the joints (see hold_joint)
        if environment is not None and any(key[0] == name for key in _HELD):
            _pin(environment, name)
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
        if isinstance(spec, Hinge | Slider | HangingBulb | Harness):
            asset = base.scene[name]
            asset.write_joint_position_to_sim_index(position=asset.data.default_joint_pos.torch.clone())
            asset.write_joint_velocity_to_sim_index(velocity=asset.data.default_joint_vel.torch.clone())
        if isinstance(spec, Harness):  # the wires come back straight: no permanent bends from the last trial
            import torch

            n = len(asset.joint_names)
            _PLASTIC[name] = np.zeros(n)  # set outright: joint_pos may still read the last trial until a step runs
            _HARDENED.pop(name, None)  # and no work hardening from the last trial
            _call_first(asset, ("set_joint_position_target_index", "set_joint_position_target"),
                        torch.zeros((1, n), dtype=torch.float32, device=asset.device))
            _WROTE.add(name)


def _after_reset(environment) -> None:
    for name, joint in list(_HELD):  # a hold from the last trial: springs back before the joints reset
        release_joint(environment, name, joint)
    _set_joint_starts(environment)
    _show_lamps(environment, False)
    for fn in _RESET:  # labkit's _start_trial first, then the task's hooks (which may schedule lamps, gloves)
        fn()
    _show_lamps(environment, _LIT["on"])  # a timed switch due at 0 lights from the first frame
    _show_gloves(environment)
    _hook_physics(environment)


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
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
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
