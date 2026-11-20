"""mechkit: real mechanisms for ``yam_arena`` task files. One file, copied into a task's folder.

A task file imports this module and declares, next to yam_arena's own ``Box`` / ``Cylinder`` objects:

- ``body(name, shapes=[box(...), cylinder(...), capsule(...), sphere(...)], pos=..., mass=...)``: one rigid body
  made of several primitive shapes (each with its own colour). With a mass it is free; without, fixed. A shape
  with ``solid=False`` is only drawn (bristles, a water surface); one with ``glow`` gives off part of its colour
  as light of its own and reads as bright in a shadow too (froth). ``kinematic=True`` makes a body that nothing
  pushes and that the task moves itself with ``write_pose`` (a bubble, an indicator).
- ``hinge(...)`` and ``slide(...)``: a revolute or a prismatic joint between two bodies, or between a body and
  the world, with travel limits and either a spring (stiffness, rest value, damping) or dry friction. With
  ``stop=<stiffness>`` the end stops are springs, which makes them gauges: ``stop_load(name)`` is the torque
  or force a stop carries, as long as one of the joint's two bodies is free to move (see the notes below).
- ``weld(...)``: a rigid joint that holds two bodies together as they were built, until ``cut(name)`` takes it away
  (a sausage cut through, a seal broken); ``joint(name)`` is how far the two have come apart at the weld (m). A
  weld declared with ``made=False`` is not there at the start: ``make(name)`` joins the two bodies as they stand at
  that moment (tines stuck into a piece of food) and ``cut(name)`` parts them again.
- ``show(name, shape, False)`` stops drawing one shape of a body (a film wiped off) until the next reset;
  ``set_mass(name, kg)`` gives a free body another mass until the next reset (a vessel that has been emptied);
  ``push(name, force, torque)`` puts a computed load on a free body until the next call (buoyancy, drag).
- ``@on_reset`` / ``@on_step`` functions, called once per episode / once per control step before scoring, for
  what the rigid simulation cannot do by itself (a snap hook, a detent, a state hidden per trial). An
  ``@on_reset`` function may take the trial's seed; ``set_joint`` / ``write_pose`` put bodies where it wants.
- in ``score()`` / ``success()``: ``joint(name)`` is the joint's travel (rad or m) from the pose it was built in,
  ``pose(name)`` a body's frame, ``gripper(side)`` a gripper's grasp point and opening, ``now()`` the seconds
  since the reset, all as of the control step being scored.

The task then runs with the stock embodiment and scorer:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...

How: importing this module needs no simulator. It registers a hook that runs when ``yam_arena.scene`` is first
imported (after Isaac Sim has started) and wraps three of its functions: ``task_object`` (to build the bodies and
author the joints as USD physics joints), ``object_states`` (to read the bodies and run the step hooks right
before yam_arena scores the step) and ``env_cfg_callback`` (PhysX buffer sizes that fit one environment). It also
wraps ``YamArenaEmbodiment.reset``: the reset hooks run right after the simulator has put every body back and
before it settles, and jointed bodies are left out of the seeded layout jitter. yam_arena itself is not modified.

Units: metres, kilograms, seconds, radians. World frame: x away from the arms, y left, z up. A body's frame is
at ``pos`` with the rotation ``quat`` (x, y, z, w) or ``yaw``; shapes are placed in that frame.

Physics, as authored here: free bodies get 32 position / 4 velocity solver iterations and a 1 m/s depenetration
cap; colliders a 2 mm contact offset and zero rest offset. Joints are maximal-coordinate PhysX joints between
separate rigid bodies (the two bodies of a joint do not collide with each other). A hinge's limits lie within
half a turn either way of the built pose (PhysX clamps them).

Dry friction has two states. While a joint moves, it is a drive towards zero speed that can push no harder than
the friction value: constant and against the motion. Once a joint has moved less than its ``give`` (half a degree,
0.2 mm) in a control step, it is held: a spring to the place it stopped at, as stiff as it takes to reach the
friction value ``give`` away, and no stronger than the friction. A load that takes it further than ``give`` tears
it loose and it moves again. The kit switches the states once per control step. (0.2 first had the moving state
only. That lets a loaded joint creep, 0.07 rad/s at 0.6 of the friction in run spike3, and lets a joint on a body
that is being squeezed drift with no load at all: a 6 g handle on a pinched stove turned 10 degrees in 12 s in run
n2g, with the nearest finger 27 mm away.)

Measured in the simulator (240 Hz; mech/tasks/spike_parts/task.py holds these numbers and checks them, run spike6
of 2026-11-17 unless another is named):
- Dry friction against motion: a 200 g disc spun to 5 rad/s on 0.002 N m stopped after 0.989 rad (1.000 by
  hand); a bar swinging down under 1.2 of its friction of 0.05 N m stopped where a friction of 0.054 N m would
  stop it. At rest: bars loaded at 0.6 of their friction, with a give of 0.3, 0.5 and 2 degrees, sagged
  0.6 x give + 0.00077 rad and then did not move (0.00000 rad in 9 s). That extra sag is why no give below
  0.3 degrees is taken: with less, a load well under the friction would tear the joint loose.
- A sprung stop (``stop=``) under a bar lying on it by its own weight sat 0.0015 rad past what its spring says, at
  5 and at 50 N m/rad, with 0.1 and 0.39 N m: ``stop_load`` then reads stiffness x 0.0015 rad too much (8 % at
  5 N m/rad and 0.1 N m, 78 % at 50). Under a balanced body twisted about its own axis it read true: 0.157,
  0.455 and 0.954 N m for 0.15, 0.45 and 0.95 N m on a 50 N m/rad stop (run n2h, the stove check's thread).
  Check a stop against a known load before a rule hangs on its reading.
  Between two bodies that are both held, a stop's travel overstates the torque. A gripper turned a stove 40
  degrees against a stop of 10 N m/rad while two fingertips pressed the cartridge under it down with 30 N: the
  stop stood 11.8 degrees in (2.05 N m by its spring) when the cartridge let go, and the same hold by itself, a
  known torque on the cartridge and no hand on the stove, let go between 0.4 and 0.6 N m (runs n2v and n2t of
  2026-11-18). Why is not established. Read a stop as a torque only when one of its two bodies is free to move.
- A kinematic body is where ``write_pose`` put it at the next step, to 0.00 mm; a shape with ``solid=False`` and
  one that ``passes`` a body are not pushed by what they overlap.
- ``show``: a shape that is hidden fades from the cameras' pictures over several frames (4 control steps after 60
  film tiles were hidden, a third of their pixels still showed; after 130 steps none; run n8d of 2026-11-18).
- A small thing that vanishes or jumps while nothing around it changes can stay in the pictures for seconds: the
  renderer builds each picture from the frames before it and has nothing to tell it that this one moved. A 10 mm
  lamp tile on a hob's black glass, hidden with ``show`` or sunk into the hob as a kinematic body, was still drawn
  6 s later; moved 3 cm aside it was drawn in both places (runs l7b to l7g of 2026-11-18). Of 16 tiles sunk at
  once (10 to 60 mm, glowing and not, on black glass, on a light strip, on the bench), between 3 % and 80 % of
  their pixels still showed a second later, differently every time (run ghost2). ``redraw`` clears it: in ten
  rounds only the three that reset that part of the renderer left no tile showing after 0.25 s. Things that an
  arm moves, and things as large as a plate, did not show the fault. Look at the picture after the change.
- The same at a reset, where every body jumps back to where it was built: the first pictures of a trial showed
  what the trial before had left, fading over 2 to 3 s (run l7g3 of 2026-11-18: three gnocchi left afloat by one
  run on the water of the next; a pan pushed 74 mm as a band along the pan's rim). Since 0.7 the kit makes the
  renderer forget its earlier frames from the simulator's reset until the scene has settled, then renders
  ``RESET_FRAMES`` pictures with its memory on again before the first one is handed out. Every task built on the
  kit gets this at every reset.
- ``set_mass``: a cup taken from 199 g to 12 g read 12 g in the simulator and rose under an upward pull of 1 N
  that had not moved it before; after the next reset it weighed 199 g again (run n8a).
- There is no see-through shape. A surface with an opacity below 1 was drawn opaque by the renderer (real-time
  path tracing; run n8a); with the renderer's fractional opacity turned on from the start, a sheet of opacity 0.2
  still let 0.1 % of what lay under it through (run n8c). Water is better drawn as a line at its level.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import inspect
import math
import sys
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from yam_arena import task_file

__version__ = "0.9"

GRIPPY = (1.2, 1.0)  # static, dynamic friction: yam_arena's own default for graspable objects
SOLVER_ITERATIONS = (32, 4)  # position, velocity
CONTACT_OFFSET, REST_OFFSET = 0.002, 0.0
MAX_DEPENETRATION = 1.0  # m/s
PAIR_CAPACITY = 2**18  # PhysX GPU pair buffers for one environment (the defaults do not fit a 24 GB GPU)
GIVE = {"hinge": math.radians(0.5), "slide": 0.0002}  # rad, m: how far a held joint with dry friction yields before it is torn loose
GIVE_LEAST = {"hinge": math.radians(0.3), "slide": 0.0001}  # a held hinge sags 0.04 degrees more than its spring says (run spike4): below this it would not hold
HOLD_SETTLE_S = 0.01  # the damping of the spring that holds a joint: its stiffness times this many seconds
CREEP_SPEED = {"hinge": 0.005, "slide": 0.0005}  # rad/s, m/s: below it the friction of a moving joint is proportional to the speed
STOP_DAMPING_S = 0.02  # a sprung stop's default damping: its stiffness times this many seconds
TCP_IN_GRIPPER = (0.0, 0.0, 0.134)  # the grasp point between the finger pads, in the arm's `gripper` link frame (yam_arena)


# --- small rotation helpers (numpy only) ------------------------------------------------------------------------


def quat_to_matrix(q) -> np.ndarray:
    """Rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = (float(c) for c in q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    """Unit quaternion (x, y, z, w), w >= 0, of a rotation matrix."""
    m = np.asarray(r, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
        q4 = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q4[i], q4[j], q4[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
        q = tuple(q4)
    n = math.sqrt(sum(c * c for c in q))
    q = tuple(c / n for c in q)
    return tuple(-c for c in q) if q[3] < 0 else q  # type: ignore[return-value]


def axis_angle(axis, angle: float) -> np.ndarray:
    """Rotation by ``angle`` rad about ``axis`` (right-hand rule)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def frame_with_x(axis) -> np.ndarray:
    """A rotation whose first column is ``axis`` (the other two are any perpendicular pair)."""
    x = np.asarray(axis, dtype=float)
    x = x / np.linalg.norm(x)
    helper = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(helper, x)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


# --- shapes and bodies ----------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Shape:
    """One primitive collider of a body, in the body's frame."""

    kind: str  # box | cylinder | capsule | sphere
    dims: tuple[float, ...]  # box: x, y, z edges; cylinder, capsule: radius, length of the straight part; sphere: radius
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # cylinders and capsules run along their local z
    color: tuple[float, float, float] | None = None  # sRGB; None: the body's colour
    solid: bool = True  # False: drawn only, nothing touches it and it weighs nothing
    passes: tuple[str, ...] = ()  # bodies this solid shape goes through without touching (soft bristles against a part)
    friction: tuple[float, float] | None = None  # (static, dynamic) of this shape; None: the body's
    friction_combine: str | None = None  # average, min, multiply or max; None: the body's
    glow: float = 0.0  # 0 to 1: the share of its colour it gives off as light of its own (froth, a lamp): bright in shadow too
    soft: tuple[float, float] | None = None  # (N/m, N s/m): a compliant contact that gives this much; see ``soften``

    def half_extents(self) -> np.ndarray:
        """Half sizes of the shape's box in its own frame."""
        if self.kind == "box":
            return np.array(self.dims) / 2
        if self.kind == "sphere":
            return np.full(3, self.dims[0])
        radius, length = self.dims
        return np.array([radius, radius, length / 2 + (radius if self.kind == "capsule" else 0.0)])

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """(low, high) of the shape's bounding box in the body frame."""
        r, half = quat_to_matrix(self.quat), self.half_extents()
        if self.kind == "sphere":
            reach = half
        else:
            reach = np.abs(r) @ half
        centre = np.array(self.pos, dtype=float)
        return centre - reach, centre + reach


_AXIS_QUAT = {"z": (0.0, 0.0, 0.0, 1.0), "x": (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)), "y": (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))}


def _surface(shape: Shape, friction, friction_combine, glow: float = 0.0) -> Shape:
    """``shape`` with a friction of its own (a foot that slides where the rest of the body grips) and its glow."""
    if friction is None and friction_combine is None and not glow:
        return shape
    if friction_combine not in (None, "average", "min", "multiply", "max"):
        raise ValueError(f"friction_combine must be average, min, multiply or max, got {friction_combine!r}")
    if not 0.0 <= glow <= 1.0:
        raise ValueError(f"glow must lie between 0 and 1, got {glow!r}")
    from dataclasses import replace

    return replace(shape, friction=None if friction is None else tuple(float(v) for v in friction), friction_combine=friction_combine, glow=float(glow))


def soften(shape: Shape, stiffness: float, damping: float) -> Shape:
    """``shape`` with a compliant contact: whatever presses on it goes in by force / ``stiffness`` (N/m), damped by
    ``damping`` (N s/m), instead of being stopped at its surface (PhysX compliant contact on the shape's material; a
    soft cake's side squeezed by a gripper). Only the shape's own contacts give; check the give in the simulator
    against a known load before a rule hangs on it (0.9, mech/tasks/cupcake_candles)."""
    if not (stiffness > 0 and damping >= 0):
        raise ValueError(f"soften: stiffness must be positive and damping not negative, got {stiffness!r}, {damping!r}")
    from dataclasses import replace

    return replace(shape, soft=(float(stiffness), float(damping)))


def _positive(values, what: str) -> tuple[float, ...]:
    out = tuple(float(v) for v in values)
    if not all(math.isfinite(v) and v > 0 for v in out):
        raise ValueError(f"{what} must be positive, got {values!r}")
    return out


def box(size, pos=(0.0, 0.0, 0.0), color=None, yaw: float = 0.0, quat=None, solid: bool = True, passes=(), friction=None, friction_combine=None, glow: float = 0.0) -> Shape:
    """A box with edges ``size`` centred at ``pos``, turned ``yaw`` about the body's z (or by ``quat``).
    ``solid=False`` (any shape): drawn only. ``passes=("name", ...)``: solid, except against those bodies, which it
    goes through (what is soft in reality and would shove a light part if it were rigid). ``friction=(static,
    dynamic)`` and ``friction_combine`` give the shape a surface of its own (see ``body``). ``glow`` (0 to 1): the
    share of its colour the shape gives off as light of its own, so that it reads as bright in a shadow too."""
    return _surface(Shape("box", _positive(size, "box size"), tuple(float(v) for v in pos), tuple(quat) if quat else yaw_quat(yaw), color, bool(solid), tuple(passes)), friction, friction_combine, glow)


def cylinder(radius: float, length: float, pos=(0.0, 0.0, 0.0), axis: str = "z", color=None, quat=None, solid: bool = True, passes=(), friction=None, friction_combine=None, glow: float = 0.0) -> Shape:
    """A cylinder of ``length`` along the body's ``axis`` (x, y or z), centred at ``pos``."""
    return _surface(Shape("cylinder", _positive((radius, length), "cylinder radius and length"), tuple(float(v) for v in pos), tuple(quat) if quat else _AXIS_QUAT[axis], color, bool(solid), tuple(passes)), friction, friction_combine, glow)


def capsule(radius: float, length: float, pos=(0.0, 0.0, 0.0), axis: str = "z", color=None, quat=None, solid: bool = True, passes=(), friction=None, friction_combine=None, glow: float = 0.0) -> Shape:
    """A capsule: a cylinder of ``length`` with a half sphere on each end."""
    return _surface(Shape("capsule", _positive((radius, length), "capsule radius and length"), tuple(float(v) for v in pos), tuple(quat) if quat else _AXIS_QUAT[axis], color, bool(solid), tuple(passes)), friction, friction_combine, glow)


def sphere(radius: float, pos=(0.0, 0.0, 0.0), color=None, solid: bool = True, passes=(), friction=None, friction_combine=None, glow: float = 0.0) -> Shape:
    return _surface(Shape("sphere", _positive((radius,), "sphere radius"), tuple(float(v) for v in pos), (0.0, 0.0, 0.0, 1.0), color, bool(solid), tuple(passes)), friction, friction_combine, glow)


@dataclass(frozen=True)
class Body(task_file.Box):
    """A rigid body of several shapes. A ``task_file.Box`` to yam_arena's loader; ``size`` is its bounding box."""

    shapes: tuple[Shape, ...] = ()
    quat: tuple[float, float, float, float] | None = None  # full orientation; None: ``yaw`` about the vertical
    friction: tuple[float, float] = GRIPPY
    linear_damping: float = 0.0
    angular_damping: float = 0.05
    kinematic: bool = False  # nothing pushes it; the task moves it with write_pose
    friction_combine: str = "average"  # how its friction meets the other body's: average, min, multiply or max

    def rotation(self) -> np.ndarray:
        return quat_to_matrix(self.quat if self.quat is not None else yaw_quat(self.yaw))

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """(low, high) of all shapes in the body frame."""
        lows, highs = zip(*(s.bounds() for s in self.shapes))
        return np.min(lows, axis=0), np.max(highs, axis=0)

    def to_world(self, point) -> np.ndarray:
        """A body-frame point in the world, at the pose the body is built in."""
        return self.rotation() @ np.asarray(point, dtype=float) + np.array(self.pos, dtype=float)


def body(name: str, shapes, pos, *, mass: float | None = None, yaw: float = 0.0, quat=None, color=task_file.GREY,
         friction=GRIPPY, linear_damping: float = 0.0, angular_damping: float = 0.05, kinematic: bool = False,
         friction_combine: str = "average") -> Body:  # fmt: skip
    """One rigid body made of ``shapes``; list it in the task file's ``OBJECTS``. ``mass`` None fixes it in place.
    ``kinematic=True`` (without a mass): it stays where ``write_pose`` puts it and no force moves it.
    ``friction`` is (static, dynamic); with ``friction_combine="min"`` a slippery body stays slippery whatever
    touches it (the default averages the two bodies' values: the table's are 0.5)."""
    shapes = tuple(shapes)
    if not shapes or not all(isinstance(s, Shape) for s in shapes):
        raise ValueError(f"{name}: shapes must be a non-empty list of box(), cylinder(), capsule(), sphere()")
    if kinematic and mass is not None:
        raise ValueError(f"{name}: a kinematic body has no mass (it is moved, not pushed)")
    if mass is not None and not any(s.solid for s in shapes):
        raise ValueError(f"{name}: a free body needs at least one solid shape")
    if friction_combine not in ("average", "min", "multiply", "max"):
        raise ValueError(f"{name}: friction_combine must be average, min, multiply or max, got {friction_combine!r}")
    lows, highs = zip(*(s.bounds() for s in shapes))
    size = tuple(float(v) for v in (np.max(highs, axis=0) - np.min(lows, axis=0)))
    if quat is not None:
        n = math.sqrt(sum(float(c) ** 2 for c in quat))
        if not 0.99 < n < 1.01:
            raise ValueError(f"{name}: quat must be a unit quaternion (x, y, z, w), got {quat!r}")
        quat = tuple(float(c) / n for c in quat)
    made = Body(name=name, pos=tuple(float(v) for v in pos), color=tuple(color), mass=mass, yaw=float(yaw), size=size, shapes=shapes,
                quat=quat, friction=tuple(friction), linear_damping=float(linear_damping), angular_damping=float(angular_damping),
                kinematic=bool(kinematic), friction_combine=friction_combine)  # fmt: skip
    _claim()
    if name in MECH.bodies:
        raise ValueError(f"body name {name!r} is used twice")
    MECH.bodies[name] = made
    return made


# --- joints -------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Joint:
    """A hinge (travel in rad) or a slide (travel in m) of ``child`` relative to ``parent`` (None: the world), or a
    weld that holds the two together as built. ``anchor`` and ``axis`` are given in the world as the bodies are
    built; travel 0 is that pose. A weld's travel is how far the two have come apart at the anchor (m)."""

    name: str
    kind: str  # hinge | slide | weld
    child: str
    parent: str | None
    anchor: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float
    upper: float
    stiffness: float = 0.0  # N m/rad or N/m, pulling towards ``rest``
    rest: float = 0.0
    damping: float = 0.0  # N m s/rad or N s/m
    friction: float = 0.0  # N m or N of dry friction
    stop: float = 0.0  # N m/rad or N/m of the end stops; 0: hard stops
    stop_damping: float = 0.0  # N m s/rad or N s/m of the end stops
    give: float = 0.0  # rad or m a joint held by its dry friction yields before it is torn loose
    made: bool = True  # a weld only: False if it is not there at the start of an episode (see ``make``)


def _joint(kind: str, name: str, child: str, parent: str | None, anchor, axis, limits, spring, rest, damping,
           friction: float = 0.0, stop: float = 0.0, stop_damping: float | None = None, give: float | None = None) -> Joint:  # fmt: skip
    axis = np.asarray(axis, dtype=float)
    length = float(np.linalg.norm(axis))
    if not length > 1e-9:
        raise ValueError(f"{name}: axis must not be zero")
    if limits is None:  # a free hinge: it turns on through any number of turns
        if kind != "hinge":
            raise ValueError(f"{name}: only a hinge can be free (limits=None)")
        if spring > 0 or friction > 0 or stop > 0:
            raise ValueError(f"{name}: a free hinge takes damping only (a spring's or a friction hold's target would wrap at half a turn)")
        limits = (-math.inf, math.inf)
    lower, upper = (float(v) for v in limits)
    if not lower <= upper:
        raise ValueError(f"{name}: limits must be (lower, upper) with lower <= upper, got {limits!r}")
    if spring < 0 or damping < 0 or friction < 0 or stop < 0 or (stop_damping is not None and stop_damping < 0):
        raise ValueError(f"{name}: spring, damping, friction and stop must not be negative")
    if friction > 0 and (spring > 0 or damping > 0):
        raise ValueError(f"{name}: a joint has either dry friction or a spring with damping (one drive per joint), not both")
    if kind == "hinge" and not (lower == -math.inf and upper == math.inf) and not (-math.pi <= lower and upper <= math.pi):
        raise ValueError(f"{name}: a hinge's limits must lie within half a turn either way of the built pose (PhysX clamps them there)")
    _claim()
    if name in MECH.joints:
        raise ValueError(f"joint name {name!r} is used twice")
    made = Joint(name, kind, child, parent, tuple(float(v) for v in anchor), tuple(float(v) for v in axis / length), lower, upper,
                 float(spring), float(rest), float(damping), float(friction), float(stop),
                 float(stop * STOP_DAMPING_S if stop_damping is None else stop_damping), float(GIVE.get(kind, 0.0) if give is None else give))  # fmt: skip
    if friction > 0 and not made.give >= GIVE_LEAST[kind]:
        raise ValueError(f"{name}: give must be at least {GIVE_LEAST[kind]:g} (a held joint sags a little more than its spring says)")
    MECH.joints[name] = made
    return made


def hinge(name: str, child: str, parent: str | None, anchor, axis, limits, *, spring: float = 0.0, rest: float = 0.0, damping: float = 0.0,
          friction: float = 0.0, stop: float = 0.0, stop_damping: float | None = None, give: float | None = None) -> Joint:  # fmt: skip
    """A revolute joint: ``child`` turns about the line through ``anchor`` along ``axis`` (world, as built) within
    ``limits`` = (lower, upper) rad from the built pose, or freely through any number of turns with ``limits=None``
    (then with ``damping`` only: a nut on a thread; its travel counts the turns). ``spring`` N m/rad pulls it to ``rest`` rad, or
    ``friction`` N m holds it where it is left (it yields up to ``give`` rad before it is torn loose).
    ``stop`` N m/rad makes the end stops springs (see ``stop_load``)."""
    return _joint("hinge", name, child, parent, anchor, axis, limits, spring, rest, damping, friction, stop, stop_damping, give)


def slide(name: str, child: str, parent: str | None, axis, limits, *, anchor=None, spring: float = 0.0, rest: float = 0.0, damping: float = 0.0,
          friction: float = 0.0, stop: float = 0.0, stop_damping: float | None = None, give: float | None = None) -> Joint:  # fmt: skip
    """A prismatic joint: ``child`` slides along ``axis`` (world, as built) within ``limits`` = (lower, upper) m
    from the built pose. ``spring`` N/m pulls it to ``rest`` m, or ``friction`` N holds it (it yields up to
    ``give`` m before it is torn loose). ``anchor`` defaults to the child's origin. ``stop`` N/m makes the end
    stops springs (see ``stop_load``)."""
    _claim()
    if anchor is None:
        if child not in MECH.bodies:
            raise ValueError(f"{name}: declare body {child!r} before its slide, or give an anchor")
        anchor = MECH.bodies[child].pos
    return _joint("slide", name, child, parent, anchor, axis, limits, spring, rest, damping, friction, stop, stop_damping, give)


def weld(name: str, child: str, parent: str | None, *, anchor=None, made: bool = True) -> Joint:
    """A rigid joint: ``child`` stays fixed to ``parent`` (None: the world) as the two were built, until
    ``cut(name)`` takes the joint away. ``anchor`` (world, as built; the child's origin if None) is where the joint
    sits. ``joint(name)`` is how far the two have come apart there (m): 0 while the weld holds, apart from what
    the solver lets it give under load. The two bodies touch each other like any two bodies, while the weld holds
    and after it is cut: build them so that they do not reach into each other.

    ``made=False``: the weld is not there when an episode starts. ``make(name)`` joins the two bodies as they
    stand at that moment, ``cut(name)`` parts them, and they can be joined again. ``joint(name)`` is then how far
    they have come apart since they were joined, and 0 while they are not joined."""
    _claim()
    if anchor is None:
        if child not in MECH.bodies:
            raise ValueError(f"{name}: declare body {child!r} before its weld, or give an anchor")
        anchor = MECH.bodies[child].pos
    from dataclasses import replace

    j = _joint("weld", name, child, parent, anchor, (1.0, 0.0, 0.0), (0.0, 0.0), 0.0, 0.0, 0.0)
    if not made:
        j = MECH.joints[name] = replace(j, made=False)
        MECH.cut.add(name)
    return j


def hold_point(value: float, previous: float | None, held: float | None, give: float) -> float | None:
    """Where a joint with dry friction is held from now on, or None while it moves. ``value``: its travel now;
    ``previous``: a control step ago (None: it has just been put at ``value``, at rest); ``held``: where it was
    held until now (None: it was moving). A held joint stays held while it is within ``give`` of that place; a
    moving one is held where it is once it has moved less than ``give`` in a control step."""
    if held is not None:
        return float(held) if abs(value - held) <= give else None
    if previous is None or abs(value - previous) <= give:
        return float(value)
    return None


def joint_pose(joint: Joint, value: float, parent_pose=None) -> tuple[np.ndarray, np.ndarray]:
    """(position, rotation matrix) of ``joint``'s child when the joint's travel is ``value`` and its parent stands
    at ``parent_pose`` (None: where it was built)."""
    (tp0, rp0), (tc0, rc0) = built_pose(joint.parent), built_pose(joint.child)
    tp, rp = (tp0, rp0) if parent_pose is None else (np.asarray(parent_pose[0], dtype=float), np.asarray(parent_pose[1], dtype=float))
    axis, anchor = np.array(joint.axis), np.array(joint.anchor)
    if joint.kind == "hinge":
        turn = axis_angle(axis, value)
        position, rotation = anchor + turn @ (tc0 - anchor), turn @ rc0
    else:
        position, rotation = tc0 + axis * value, rc0
    carried = rp @ rp0.T  # how the parent has turned since it was built
    return tp + carried @ (position - tp0), carried @ rotation


def joint_travel(joint: Joint, parent_pose, child_pose, parent_built, child_built) -> float:
    """Travel of ``joint`` (rad or m) given the two bodies' poses now and as built; a pose is (position, rotation
    matrix), and the world's pose is (zeros, identity)."""
    (tp, rp), (tc, rc), (tp0, rp0), (tc0, rc0) = parent_pose, child_pose, parent_built, child_built
    if joint.kind == "weld":  # how far the anchor as a point of the child is from the anchor as a point of the parent
        anchor = np.array(joint.anchor)
        return float(np.linalg.norm((rc @ (rc0.T @ (anchor - tc0)) + tc) - (rp @ (rp0.T @ (anchor - tp0)) + tp)))
    axis_p = rp0.T @ np.array(joint.axis)  # the axis in the parent's frame (fixed there)
    if joint.kind == "slide":
        anchor_c = rc0.T @ (np.array(joint.anchor) - tc0)  # the anchor as a point of the child
        now = rp.T @ (rc @ anchor_c + tc - tp)
        built = rp0.T @ (np.array(joint.anchor) - tp0)
        return float(axis_p @ (now - built))
    turn = (rp.T @ rc) @ (rp0.T @ rc0).T  # the child's rotation relative to the parent since it was built, in the parent frame
    sin_part = 0.5 * np.array([turn[2, 1] - turn[1, 2], turn[0, 2] - turn[2, 0], turn[1, 0] - turn[0, 1]])
    return float(math.atan2(float(axis_p @ sin_part), 0.5 * (float(np.trace(turn)) - 1.0)))


def unwrapped(angle: float, previous: float | None) -> float:
    """``angle`` (any representative of it) as the value nearest to ``previous``: a hinge read every control step
    keeps counting through +-180 degrees."""
    if previous is None:
        return angle
    return angle + 2.0 * math.pi * round((previous - angle) / (2.0 * math.pi))


# --- registry, hooks and what score() reads -----------------------------------------------------------------


@dataclass
class _Registry:
    bodies: dict[str, Body] = field(default_factory=dict)
    joints: dict[str, Joint] = field(default_factory=dict)
    reset_hooks: list[Callable] = field(default_factory=list)
    step_hooks: list[Callable] = field(default_factory=list)
    poses: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)  # body -> (position, rotation) this step
    velocities: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    travel: dict[str, float] = field(default_factory=dict)
    grippers: dict[str, tuple[np.ndarray, float]] = field(default_factory=dict)  # side -> (grasp point, opening 0..1)
    written: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)  # poses written since the bodies were last read
    held: dict[str, float | None] = field(default_factory=dict)  # joint with dry friction -> where it is held (rad or m); None: it moves
    before: dict[str, float] = field(default_factory=dict)  # joint with dry friction -> its travel a control step ago
    thrown: set[str] = field(default_factory=set)  # bodies given a velocity since the bodies were last read
    hidden: set[tuple[str, int]] = field(default_factory=set)  # (body, shape) not drawn at present (see ``show``)
    reweighed: set[str] = field(default_factory=set)  # bodies whose mass is not the built one at present (see ``set_mass``)
    pushed: set[str] = field(default_factory=set)  # bodies that carry a force or torque set with ``push``
    cut: set[str] = field(default_factory=set)  # welds that are not there at present (see ``cut`` and ``make``)
    joined: dict[str, np.ndarray] = field(default_factory=dict)  # weld made in this episode -> the child's origin in the parent's frame when it was made
    redraw: int = 0  # control steps for which the renderer still forgets its earlier frames (see ``redraw``)
    body_prims: dict[str, str] = field(default_factory=dict)  # body -> its prim in the simulator's stage, once built
    joint_prims: dict[str, str] = field(default_factory=dict)  # joint -> its prim in the simulator's stage, once built
    environment: Any = None
    steps: int = 0
    time: float = 0.0  # seconds since the reset, as of the control step being scored
    seed: int | None = None  # the trial's seed, as the embodiment's reset received it
    owner: int | None = None  # id of the globals of the task module whose declarations these are

    def clear(self) -> None:
        """Forget everything declared."""
        self.bodies.clear(), self.joints.clear(), self.reset_hooks.clear(), self.step_hooks.clear(), self.joint_prims.clear(), self.body_prims.clear()
        self.hidden.clear(), self.reweighed.clear(), self.pushed.clear(), self.cut.clear(), self.joined.clear()
        self.redraw = 0
        self.new_episode()
        self.environment, self.seed, self.owner = None, None, None

    def new_episode(self) -> None:
        """Forget what was read from the simulator."""
        self.poses.clear(), self.velocities.clear(), self.travel.clear(), self.grippers.clear(), self.written.clear()
        self.held.clear(), self.before.clear(), self.thrown.clear()
        self.steps, self.time = 0, 0.0


MECH = _Registry()
WORLD_POSE = (np.zeros(3), np.eye(3))


def _claim() -> None:
    """Start a clean registry when the declaring module changes. yam_arena executes a task file afresh on every
    load, so its first declaration (from a new module object) drops what an earlier load, or another task file
    in the same process, had declared. A helper module that declares for the task file importing it (shared
    parts of a scene) sets ``MECHKIT_HELPER = True`` in its globals: its frames are skipped like the kit's own (0.9)."""
    frame = sys._getframe(1)
    while frame is not None and (frame.f_globals.get("__name__") == __name__ or frame.f_globals.get("MECHKIT_HELPER") is True):
        frame = frame.f_back
    owner = id(frame.f_globals) if frame is not None else None
    if owner != MECH.owner:
        MECH.clear()
        MECH.owner = owner


def on_reset(fn: Callable) -> Callable:
    """Decorator: ``fn()`` or ``fn(seed)`` runs at the start of every episode, right after the simulator has put
    every body back where it was built and before the scene settles. It resets the task's own state and may
    call ``set_joint`` and ``write_pose`` (a start state that differs from trial to trial). ``seed`` is the
    trial's seed, None when the run gave none."""
    _claim()
    MECH.reset_hooks.append(fn)
    return fn


def _run_reset_hooks() -> None:
    for hook in MECH.reset_hooks:
        try:
            wants_seed = any(p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD) for p in inspect.signature(hook).parameters.values())
        except (TypeError, ValueError):
            wants_seed = False
        hook(MECH.seed) if wants_seed else hook()


def on_step(fn: Callable) -> Callable:
    """Decorator: ``fn()`` runs after every control step, once the bodies have been read and before yam_arena
    scores the step. It may call ``pose``, ``joint``, ``velocity`` and ``write_pose``."""
    _claim()
    MECH.step_hooks.append(fn)
    return fn


def built_pose(name: str | None) -> tuple[np.ndarray, np.ndarray]:
    if name is None:
        return WORLD_POSE
    spec = MECH.bodies[name]
    return np.array(spec.pos, dtype=float), spec.rotation()


def pose(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(position, rotation matrix) of a body's frame as of the control step being scored."""
    if name not in MECH.poses:
        raise KeyError(f"no pose for {name!r} yet: pose() is valid from the first control step on")
    return MECH.poses[name]


def velocity(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(linear m/s, angular rad/s) of a body, zeros for a fixed one."""
    return MECH.velocities.get(name, (np.zeros(3), np.zeros(3)))


def joint(name: str) -> float:
    """Travel of a joint (rad or m) from the pose it was built in, as of the control step being scored. A hinge
    counts on through full turns."""
    if name not in MECH.travel:
        raise KeyError(f"no value for joint {name!r} yet: joint() is valid from the first control step on")
    return MECH.travel[name]


def stop_load(name: str) -> float:
    """What a sprung end stop of joint ``name`` carries now (N m or N): its stiffness times the travel beyond the
    limit, positive at the upper stop, negative at the lower one, 0 between them. It is the stop's spring only,
    read once per control step: a blow shorter than that is missed. Measured true on a body that is free to move;
    between two bodies that are both held it overstated the torque (module notes)."""
    j = MECH.joints[name]
    if not j.stop > 0:
        raise ValueError(f"joint {name!r} has hard stops: declare it with stop=<stiffness> to read their load")
    travel = joint(name)
    return j.stop * (travel - j.upper) if travel > j.upper else j.stop * (travel - j.lower) if travel < j.lower else 0.0


def gripper(side: str) -> tuple[np.ndarray, float]:
    """(grasp point between the finger pads in the world, opening from 0 closed to 1 open) of the ``left`` or
    ``right`` arm, as of the control step being scored."""
    if side not in MECH.grippers:
        raise KeyError(f"no reading of the {side!r} gripper yet: gripper() is valid from the first control step on")
    return MECH.grippers[side]


def gripper_turn(side: str) -> np.ndarray:
    """The ``left`` or ``right`` gripper's rotation in the world, as of the control step being scored: its columns
    are the closing axis, the pads' width axis and the approach axis (0.9)."""
    if side not in GRIPPER_TURN:
        raise KeyError(f"no reading of the {side!r} gripper yet: gripper_turn() is valid from the first control step on")
    return GRIPPER_TURN[side]


def now() -> float:
    """Seconds since the reset, as of the control step being scored."""
    return MECH.time


def seed() -> int | None:
    """The trial's seed (None when the run gave none)."""
    return MECH.seed


def set_state(poses: dict | None = None, velocities: dict | None = None, *, joints: dict | None = None, grippers: dict | None = None,
              time: float | None = None) -> None:  # fmt: skip
    """Put a state in by hand: for tests of ``score()`` and of step hooks without the simulator. Bodies stand as
    built, then ``joints`` (name -> travel) place their children (parents first, in the order the joints were
    declared), then ``poses`` (name -> (position, rotation matrix)) override. ``grippers``: side -> (grasp
    point, opening); ``time``: seconds since the reset. What is not given keeps its last value, except poses."""
    MECH.poses = {name: built_pose(name) for name in MECH.bodies}
    for name, j in MECH.joints.items():
        if joints and name in joints:
            MECH.poses[j.child] = joint_pose(j, float(joints[name]), None if j.parent is None else MECH.poses[j.parent])
        elif j.parent is not None and name not in MECH.cut and (j.kind != "weld" or j.made):  # a child rides on a parent that was moved
            MECH.poses[j.child] = joint_pose(j, 0.0, MECH.poses[j.parent])
    for name, (position, rotation) in (poses or {}).items():
        MECH.poses[name] = (np.asarray(position, dtype=float), np.asarray(rotation, dtype=float))
    MECH.velocities = {name: (np.asarray(v[0], dtype=float), np.asarray(v[1], dtype=float)) for name, v in (velocities or {}).items()}
    MECH.travel.clear()
    _update_travel()
    for name, value in (joints or {}).items():
        if MECH.joints[name].child not in (poses or {}):
            MECH.travel[name] = float(value)  # exactly as given, also beyond half a turn
    for side, (point, opening) in (grippers or {}).items():
        MECH.grippers[side] = (np.asarray(point, dtype=float), float(opening))
    if time is not None:
        MECH.time = float(time)


def _update_travel() -> None:
    for name, j in MECH.joints.items():
        parent = MECH.poses[j.parent] if j.parent is not None else WORLD_POSE
        if j.kind == "weld" and not j.made:  # joined during the episode, or not at all
            where = MECH.joined.get(name)
            MECH.travel[name] = 0.0 if where is None or name in MECH.cut else float(np.linalg.norm(parent[0] + parent[1] @ where - MECH.poses[j.child][0]))
            continue
        travel = joint_travel(j, parent, MECH.poses[j.child], built_pose(j.parent), built_pose(j.child))
        MECH.travel[name] = unwrapped(travel, MECH.travel.get(name)) if j.kind == "hinge" else travel


def _hold_joints(travel: dict[str, float] | None = None, write: Callable[[Joint, float | None], None] | None = None, moving=()) -> None:
    """Once per control step, for every joint with dry friction: hold it if it has come to rest, let it go if it
    has been torn loose (see ``hold_point``), and tell the simulator when that changes. ``travel``: the joints'
    travel if not the last reading's; ``moving``: joints to start as moving although they have just been placed."""
    travel = MECH.travel if travel is None else travel
    write = _write_friction if write is None else write
    for name, j in MECH.joints.items():
        if j.friction > 0 and name in MECH.joint_prims and name in travel:
            first = name not in MECH.held  # of the episode: whatever the last one left in the simulator is replaced
            now = None if name in moving else hold_point(travel[name], MECH.before.get(name), MECH.held.get(name), j.give)
            if first or now != MECH.held[name]:
                write(j, now)
            MECH.held[name], MECH.before[name] = now, float(travel[name])


def _write_friction(j: Joint, held: float | None) -> None:
    """The drive of joint ``j`` in the simulator: a spring to ``held``, or (None) the brake of a moving joint."""
    from isaaclab.sim.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(MECH.joint_prims[j.name])
    instance, per_unit, to_usd = ("angular", math.radians(1.0), math.degrees) if j.kind == "hinge" else ("linear", 1.0, float)
    stiffness, damping, target = (prim.GetAttribute(f"drive:{instance}:physics:{what}") for what in ("stiffness", "damping", "targetPosition"))
    if not (stiffness.IsValid() and damping.IsValid() and target.IsValid()):
        raise RuntimeError(f"joint {j.name!r}: no drive at {MECH.joint_prims[j.name]} to make its friction with")
    spring, brake = _friction_drive(j)
    if held is None:
        stiffness.Set(0.0)
        damping.Set(float(brake * per_unit))
    else:
        target.Set(float(to_usd(held)))
        stiffness.Set(float(spring * per_unit))
        damping.Set(float(spring * HOLD_SETTLE_S * per_unit))


def set_spring(name: str, rest: float | None = None, most: float | None = None) -> None:
    """Move the rest position of the spring of joint ``name`` to ``rest`` (m or rad from the built pose) and cap the
    force it can give at ``most`` (N or N m), from the next physics step on; None leaves either as it is. For a task
    that makes a resistance of its own out of a spring: a stiff spring that is capped and whose rest follows the
    joint at a fixed distance holds like dry friction and slides at the cap, without the two states of the kit's
    own friction (which lets a loaded joint creep when small jerks tear it loose: mech/tasks/shield_seat, recording
    c1d). The simulator keeps both through a reset, and a rest left where the last trial pushed it pulls the joint
    there before the next trial's first step: put the rest back in an @on_reset function. Needs the simulator."""
    j = MECH.joints[name]
    if j.kind == "weld" or j.friction > 0 or j.stiffness <= 0:
        raise ValueError(f"{name}: only a hinge or slide with a spring has a rest position and a cap")
    if name not in MECH.joint_prims:
        raise RuntimeError("set_spring() needs the simulator: call it from an @on_step or @on_reset function")
    _write_spring(j, rest, most)


def _write_spring(j: Joint, rest: float | None, most: float | None) -> None:
    from isaaclab.sim.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(MECH.joint_prims[j.name])
    instance, to_usd = ("angular", math.degrees) if j.kind == "hinge" else ("linear", float)
    target, cap = (prim.GetAttribute(f"drive:{instance}:physics:{what}") for what in ("targetPosition", "maxForce"))
    if not (target.IsValid() and cap.IsValid()):
        raise RuntimeError(f"joint {j.name!r}: no drive at {MECH.joint_prims[j.name]}")
    if rest is not None:
        target.Set(float(to_usd(rest)))
    if most is not None:
        cap.Set(float(most))


def set_drive(name: str, stiffness: float | None = None, damping: float | None = None) -> None:
    """Give the spring of hinge or slide ``name`` another stiffness (N m/rad or N/m) and damping (N m s/rad or N s/m),
    from the next physics step on; None leaves either as it is. For a hold that changes with the state (a candle held
    the more firmly, the deeper it is in a cake). The simulator keeps both through a reset: put them back in an
    @on_reset function. Needs the simulator (0.9, mech/tasks/cupcake_candles)."""
    j = MECH.joints[name]
    if j.kind == "weld" or j.friction > 0 or j.stiffness <= 0:
        raise ValueError(f"{name}: only a hinge or slide with a spring has a stiffness to change")
    for what, value in (("stiffness", stiffness), ("damping", damping)):
        if value is not None and not value >= 0:
            raise ValueError(f"{name}: {what} must not be negative, got {value!r}")
    if name not in MECH.joint_prims:
        raise RuntimeError("set_drive() needs the simulator: call it from an @on_step or @on_reset function")
    from isaaclab.sim.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(MECH.joint_prims[name])
    instance, per_unit = ("angular", math.radians(1.0)) if j.kind == "hinge" else ("linear", 1.0)  # USD: N m/deg
    for what, value in (("stiffness", stiffness), ("damping", damping)):
        if value is None:
            continue
        attr = prim.GetAttribute(f"drive:{instance}:physics:{what}")
        if not attr.IsValid():
            raise RuntimeError(f"joint {name!r}: no drive {what} at {MECH.joint_prims[name]}")
        attr.Set(float(value * per_unit))


def set_limits(name: str, lower: float | None = None, upper: float | None = None) -> None:
    """Move the end stops of hinge or slide ``name`` to ``lower`` and ``upper`` (rad or m from the built pose; None
    leaves either as it is), from the next physics step on. A stop moved after a joint every control step makes a
    one-way lock: it moves on freely and cannot go back past where it was (a ratchet's pawl, a bar clamp's lock plate).
    The simulator keeps the stops through a reset, and a stop left where the last trial moved it would push the joint
    there before the next trial's first step: put the stops back in an @on_reset function. Needs the simulator."""
    j = MECH.joints[name]
    if j.kind == "weld":
        raise ValueError(f"{name}: a weld has no stops")
    if j.lower == -math.inf and j.upper == math.inf:
        raise ValueError(f"{name}: a free hinge has no stops to move (they would wrap at half a turn)")
    if name not in MECH.joint_prims:
        raise RuntimeError("set_limits() needs the simulator: call it from an @on_step or @on_reset function")
    if lower is not None and upper is not None and not lower < upper:
        raise ValueError(f"{name}: lower stop {lower} not under upper stop {upper} (PhysX refuses equal stops: give the joint a window)")
    from isaaclab.sim.utils.stage import get_current_stage

    prim = get_current_stage().GetPrimAtPath(MECH.joint_prims[j.name])
    to_usd = math.degrees if j.kind == "hinge" else float  # USD: revolute limits are in degrees
    writes = [(lower, "physics:lowerLimit"), (upper, "physics:upperLimit")]
    if lower is not None and upper is not None:  # each write reaches the simulator alone: no state between the two may have its stops crossed
        now_upper = prim.GetAttribute("physics:upperLimit").Get()
        if now_upper is not None and to_usd(lower) >= float(now_upper):  # the window moved up past itself: the upper stop first (lower first, PhysX refused the stops every such step: e2f)
            writes.reverse()
    for value, attribute in writes:
        if value is None:
            continue
        stop = prim.GetAttribute(attribute)
        if not stop.IsValid():
            raise RuntimeError(f"joint {j.name!r}: no {attribute} at {MECH.joint_prims[j.name]}")
        stop.Set(float(to_usd(value)))


def _friction_drive(j: Joint) -> tuple[float, float]:
    """(stiffness of the spring that holds joint ``j``, damping of the brake while it moves), per rad or m."""
    return j.friction / j.give, j.friction / CREEP_SPEED[j.kind]


def _draw_all() -> None:
    """What the last episode stopped drawing is drawn again (before the reset hooks, which may hide things anew)."""
    for name, shape in sorted(MECH.hidden):
        if name in MECH.body_prims:
            _write_visibility(name, shape, True)
    MECH.hidden.clear()


def _after_reset() -> None:
    """Start the dry friction of an episode: every such joint is held where the simulator's reset and the reset
    hooks have put it (nothing has been read yet), except one whose body a reset hook has thrown: that one moves."""

    def where(name: str | None):
        if name is None or not (MECH.bodies[name].free or MECH.bodies[name].kinematic):
            return built_pose(name)  # the world, or a fixed body: where it was built
        return _sim_pose(name)

    joints = {name: j for name, j in MECH.joints.items() if j.friction > 0 and name in MECH.joint_prims}
    travel = {name: joint_travel(j, where(j.parent), where(j.child), built_pose(j.parent), built_pose(j.child)) for name, j in joints.items()}
    MECH.held.clear(), MECH.before.clear()
    _hold_joints(travel, moving={name for name, j in joints.items() if j.child in MECH.thrown or j.parent in MECH.thrown})


def set_joint(name: str, value: float) -> None:
    """Put the child of joint ``name`` where the joint's travel is ``value`` (rad or m), at rest. For a start
    state from an ``@on_reset`` function; the parent must already stand where it will."""
    j = MECH.joints[name]
    if not j.lower <= value <= j.upper:
        raise ValueError(f"{name}: {value} is outside the joint's limits ({j.lower}, {j.upper})")
    parent = None
    if j.parent is not None and MECH.bodies[j.parent].free:
        parent = _sim_pose(j.parent)
    position, rotation = joint_pose(j, float(value), parent)
    write_pose(j.child, position, matrix_to_quat(rotation))
    MECH.travel[name] = float(value)  # the next reading counts turns from here


def cut(name: str) -> None:
    """Take the weld ``name`` away: from the next physics step on its two bodies are separate and touch each other
    like any two bodies (checked in the simulator: mech/tasks/spike_welds). A weld that was there from the start is
    back at the next reset. Without a simulator it only notes the cut, for tests of a task's rules."""
    j = MECH.joints[name]
    if j.kind != "weld":
        raise ValueError(f"{name} is a {j.kind}: only a weld can be cut")
    if name in MECH.cut:
        return
    if name in MECH.joint_prims:  # built in a simulator
        _write_joint_enabled(name, False)
    MECH.cut.add(name)
    MECH.joined.pop(name, None)


def make(name: str) -> None:
    """Join the two bodies of the weld ``name`` (declared with ``made=False``) as they stand now: from the next
    physics step on the child keeps its place on the parent, until ``cut(name)``. Call it from an @on_step
    function. Without a simulator it joins them at the poses last put in with ``set_state``."""
    j = MECH.joints[name]
    if j.kind != "weld" or j.made:
        raise ValueError(f"{name}: only a weld declared with made=False can be made during an episode")
    if name not in MECH.cut:
        return  # it is made already
    in_sim = name in MECH.joint_prims
    where = _sim_pose if in_sim else (lambda body: MECH.poses[body])
    (tp, rp), (tc, rc) = (WORLD_POSE if j.parent is None else where(j.parent)), where(j.child)
    if in_sim:
        fixed_parent = j.parent is None or not MECH.bodies[j.parent].free
        _write_joint_frame(name, WORLD_POSE if fixed_parent else (tp, rp), (tc, rc))
        _write_joint_enabled(name, True)
    MECH.cut.discard(name)
    MECH.joined[name] = rp.T @ (tc - tp)


def is_cut(name: str) -> bool:
    """Whether the weld ``name`` has been cut in this episode."""
    if MECH.joints[name].kind != "weld":
        raise ValueError(f"{name} is not a weld")
    return name in MECH.cut


def mend_all() -> None:
    """The welds as an episode starts with them: every weld that was built holds again, and none of those made
    during an episode is there. The kit calls this at every reset, when the simulator has put every body back where
    it was built (a weld put back between bodies that have moved apart would snap them together). A test without a
    simulator calls it itself."""
    for name, j in MECH.joints.items():
        if j.kind != "weld":
            continue
        if name in MECH.joint_prims and (name in MECH.cut) == j.made:  # it is not as the episode starts with it
            _write_joint_enabled(name, j.made)
        (MECH.cut.discard if j.made else MECH.cut.add)(name)
    MECH.joined.clear()


def _write_joint_frame(name: str, parent_pose, child_pose) -> None:
    """Move the parent's side of the joint's frame to where the child's side is now, so that the joint, switched
    on, holds the two bodies as they stand."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    prim = get_current_stage().GetPrimAtPath(MECH.joint_prims[name])
    if not prim.IsValid():
        raise RuntimeError(f"joint {name!r}: no prim at {MECH.joint_prims[name]} to move")
    joint = UsdPhysics.Joint(prim)
    (tp, rp), (tc, rc) = parent_pose, child_pose
    p1, q1 = joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get()
    r1 = quat_to_matrix((*q1.GetImaginary(), q1.GetReal()))
    local_pos, local_rot = rp.T @ (rc @ np.array(p1, dtype=float) + tc - tp), matrix_to_quat(rp.T @ rc @ r1)
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in local_pos)))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(float(local_rot[3]), Gf.Vec3f(*(float(v) for v in local_rot[:3]))))


def _write_joint_enabled(name: str, enabled: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    prim = get_current_stage().GetPrimAtPath(MECH.joint_prims[name])
    if not prim.IsValid():
        raise RuntimeError(f"joint {name!r}: no prim at {MECH.joint_prims[name]} to switch")
    UsdPhysics.Joint(prim).CreateJointEnabledAttr().Set(bool(enabled))


def show(name: str, shape: int, visible: bool = True) -> None:
    """Draw, or stop drawing, shape number ``shape`` (its place in the body's list) of body ``name``: a film that
    has been wiped off, a lamp that has gone out. It changes what the cameras see and nothing else: a solid shape
    still touches what it touched. Every shape is drawn again at the next reset."""
    spec = MECH.bodies[name]
    if not 0 <= shape < len(spec.shapes):
        raise ValueError(f"{name} has shapes 0 to {len(spec.shapes) - 1}, not {shape}")
    if visible == ((name, shape) not in MECH.hidden):
        return
    if name in MECH.body_prims:  # built in a simulator: tell its renderer
        _write_visibility(name, shape, visible)
    (MECH.hidden.discard if visible else MECH.hidden.add)((name, shape))


def shown(name: str, shape: int) -> bool:
    """Whether shape ``shape`` of body ``name`` is drawn at present."""
    return (name, shape) not in MECH.hidden


def _write_visibility(name: str, shape: int, visible: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    prim = get_current_stage().GetPrimAtPath(f"{MECH.body_prims[name]}/geometry/shape{shape}")
    if not prim.IsValid():
        raise RuntimeError(f"{name}: no shape {shape} at {MECH.body_prims[name]} to show or hide")
    imageable = UsdGeom.Imageable(prim)
    imageable.MakeVisible() if visible else imageable.MakeInvisible()


def mass(name: str) -> float:
    """The mass (kg) of free body ``name`` at present: the simulator's if it runs, else the built one."""
    spec = MECH.bodies[name]
    if not spec.free:
        raise ValueError(f"{name} is not a free body: it has no mass")
    if MECH.environment is None:
        return float(spec.mass)
    return float(MECH.environment.unwrapped.scene.rigid_objects[name].data.body_mass.torch[0, 0])


def set_mass(name: str, kilograms: float) -> None:
    """Give free body ``name`` another mass from now on: a vessel that has been emptied. Its moments of inertia
    change in the same proportion (as if what left had been spread like what stays). The built mass is back at the
    next reset. Needs the simulator: call it from an @on_step or @on_reset function."""
    env = MECH.environment
    if env is None:
        raise RuntimeError("set_mass() needs the simulator: call it from an @on_step or @on_reset function")
    spec = MECH.bodies[name]
    if not spec.free:
        raise ValueError(f"{name} is not a free body: it has no mass to change")
    if not (math.isfinite(kilograms) and kilograms > 0):
        raise ValueError(f"{name}: a mass must be positive, got {kilograms!r}")
    import torch

    obj = env.unwrapped.scene.rigid_objects[name]
    before = float(obj.data.body_mass.torch[0, 0])
    inertia = obj.data.body_inertia.torch[:1].clone() * (float(kilograms) / before)
    obj.set_masses_index(masses=torch.full((1, 1), float(kilograms), dtype=inertia.dtype, device=inertia.device))
    obj.set_inertias_index(inertias=inertia)
    (MECH.reweighed.discard if abs(kilograms - spec.mass) < 1e-9 else MECH.reweighed.add)(name)


REDRAW_SETTING = "/rtx-transient/post/dlss/forceParamReset"  # the renderer's anti-aliasing starts from nothing while this is on


def redraw(steps: int = 2) -> None:
    """Make the renderer build the cameras' next ``steps`` pictures without the frames before them. Call it at
    the control step at which a small drawn thing vanishes, appears or jumps (a lamp that goes out, a tile that is
    taken away): otherwise it can stay in the pictures for seconds (see the module's notes). For those steps the
    pictures have rougher edges; nothing else changes."""
    if steps < 1:
        raise ValueError(f"redraw: steps must be at least 1, got {steps}")
    MECH.redraw = max(MECH.redraw, int(steps))
    if MECH.environment is not None:
        _write_redraw(True)


def _tick_redraw() -> None:
    """Once per control step, before the task's step functions: one step less of forgetting; at none, the
    renderer may use its earlier frames again."""
    if MECH.redraw > 0:
        MECH.redraw -= 1
        if MECH.redraw == 0 and MECH.environment is not None:
            _write_redraw(False)


RESET_FRAMES = 8  # pictures rendered after a reset has settled, the renderer's memory on again, before the first is handed out


def _forget_at_reset() -> None:
    """From the simulator's reset until the scene has settled the renderer builds its pictures without the frames
    before them: every body has just jumped back to where it was built, and what the cameras showed before would
    otherwise stay in the new episode's first pictures for seconds (see the module's notes). A redraw the last
    episode still had running ends here."""
    MECH.redraw = 0
    _write_redraw(True)


def _first_picture(embodiment, observation):
    """The observation a reset hands out, taken afresh: the renderer may use its earlier frames again (unless a
    reset hook has asked for a redraw of its own, which runs out by itself), RESET_FRAMES pictures are rendered
    so that it has frames of the settled scene to build on, and the cameras are read again."""
    if MECH.redraw == 0:
        _write_redraw(False)
    for _ in range(RESET_FRAMES):
        embodiment._base.sim.render()
    return embodiment._observation()


def _write_redraw(on: bool) -> None:
    import carb

    carb.settings.get_settings().set(REDRAW_SETTING, bool(on))


def push(name: str, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)) -> None:
    """Put a force (N) and a torque (N m), both in the world's frame, on free body ``name`` at its centre of mass.
    They act until the next call for that body or the next reset: an @on_step function that computes a load
    (buoyancy, drag, a magnet) calls this every control step, so the load is that of the last reading held for one
    step. A spring made this way with stiffness k on a mass m, read every dt seconds, needs a damping c between
    k dt / 2 and 2 m / dt to come to rest; outside that it swings up. Needs the simulator."""
    env = MECH.environment
    if env is None:
        raise RuntimeError("push() needs the simulator: call it from an @on_step or @on_reset function")
    spec = MECH.bodies[name]
    if not spec.free:
        raise ValueError(f"{name} is not a free body: a force does not move it")
    import torch

    obj = env.unwrapped.scene.rigid_objects[name]
    load = torch.tensor([[[float(v) for v in force]], [[float(v) for v in torque]]], dtype=torch.float32, device=obj.device)
    obj.permanent_wrench_composer.set_forces_and_torques_index(forces=load[0:1], torques=load[1:2], is_global=True)
    (MECH.pushed.add if bool(load.any()) else MECH.pushed.discard)(name)


def _stop_pushing() -> None:
    """No body carries a computed load any more (before the reset hooks, which may set loads anew)."""
    for name in sorted(MECH.pushed):
        push(name)


def _restore_masses() -> None:
    """Every body weighs again what it was built with (before the reset hooks, which may change masses anew)."""
    for name in sorted(MECH.reweighed):
        set_mass(name, MECH.bodies[name].mass)


def write_pose(name: str, position, quat=None, lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)) -> None:
    """Put a free or kinematic body at ``position`` (world) with orientation ``quat`` (x, y, z, w; None keeps it)
    and, if it is free, the given velocities. For scripted holds and computed effects; say in the task's
    fidelity note what is written and when."""
    env = MECH.environment
    if env is None:
        raise RuntimeError("write_pose() needs the simulator: call it from an @on_step or @on_reset function")
    spec = MECH.bodies.get(name)
    if spec is not None and not (spec.free or spec.kinematic):
        raise ValueError(f"{name} is fixed: only a free or a kinematic body can be moved")
    import torch

    base = env.unwrapped
    obj = base.scene.rigid_objects[name]
    root = obj.data.root_pose_w.torch.clone()
    root[0, :3] = torch.tensor([float(v) for v in position], dtype=root.dtype, device=root.device) + base.scene.env_origins[0]
    if quat is not None:
        root[0, 3:] = torch.tensor([float(v) for v in quat], dtype=root.dtype, device=root.device)
    twist = torch.tensor([[*(float(v) for v in lin_vel), *(float(v) for v in ang_vel)]], dtype=root.dtype, device=root.device)
    obj.write_root_pose_to_sim_index(root_pose=root)
    if spec is None or not spec.kinematic:  # a kinematic body has no velocity to set
        obj.write_root_velocity_to_sim_index(root_velocity=twist)
        if any(float(v) != 0.0 for v in (*lin_vel, *ang_vel)):
            MECH.thrown.add(name)
    MECH.written[name] = (np.array([float(v) for v in position]), quat_to_matrix(root[0, 3:].detach().cpu().numpy().astype(float)))


def _sim_pose(name: str) -> tuple[np.ndarray, np.ndarray]:
    """A rigid body's pose now: what was written to it since the last reading, else the simulator's (environment 0)."""
    if name in MECH.written:
        return MECH.written[name]
    base = MECH.environment.unwrapped
    root = base.scene.rigid_objects[name].data.root_pose_w.torch[0].detach().cpu().numpy().astype(float)
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    return root[:3] - origin, quat_to_matrix(root[3:])


# --- the simulator side: runs only after Isaac Sim has started ------------------------------------------------


def _read_bodies(env) -> None:
    """Fill MECH.poses / velocities / travel from the simulator (environment 0)."""
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    rigid = base.scene.rigid_objects
    world = base.arena_world
    MECH.written.clear(), MECH.thrown.clear()
    for name, spec in MECH.bodies.items():
        if name in rigid:
            data = rigid[name].data
            root = data.root_pose_w.torch[0].detach().cpu().numpy().astype(float)
            vel = data.root_vel_w.torch[0].detach().cpu().numpy().astype(float)
            MECH.velocities[name] = (vel[:3], vel[3:])
        else:
            root = world.get_pose_w(name)[0].detach().cpu().numpy().astype(float)
        MECH.poses[name] = (root[:3] - origin, quat_to_matrix(root[3:]))
    _update_travel()
    _read_grippers(base, origin)


def _read_grippers(base, origin: np.ndarray) -> None:
    """Fill MECH.grippers from the two arms: the grasp point (TCP) and the fingers' opening."""
    from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN

    for side in ("left", "right"):
        try:
            robot = base.scene[f"{side}_robot"]
        except KeyError:
            continue
        ids = _ROBOT_IDS.get((id(base), side))
        if ids is None:
            ids = _ROBOT_IDS[(id(base), side)] = (robot.find_bodies("gripper")[0][0], robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
        link = robot.data.body_link_pose_w.torch[0, ids[0]].detach().cpu().numpy().astype(float)
        point = link[:3] - origin + quat_to_matrix(link[3:7]) @ np.array(TCP_IN_GRIPPER)
        opening = float(np.clip(robot.data.joint_pos.torch[0][ids[1]].mean().item() / FINGER_OPEN, 0.0, 1.0))
        MECH.grippers[side] = (point, opening)
        GRIPPER_TURN[side] = quat_to_matrix(link[3:7])


_ROBOT_IDS: dict = {}
GRIPPER_TURN: dict[str, np.ndarray] = {}  # side -> the gripper's rotation in the world (columns: closing, pads' width, approach)  # (environment, side) -> (index of the gripper link, indices of the finger joints)


def _spawner(spec: Body):
    """An Isaac Lab spawn function that builds ``spec``: a root prim (the rigid body, if free) with one collider
    prim per shape under ``geometry``, then the joints whose child is this body. Same prim layout as Isaac Lab's
    own shape spawners (root Xform, ``geometry`` below it), so Arena's bounding boxes and resets work unchanged."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim import schemas
    from isaaclab.sim.utils.prims import bind_physics_material, bind_visual_material, clone, create_prim
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    @clone
    def spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
        stage = get_current_stage()
        if stage.GetPrimAtPath(prim_path).IsValid():
            raise ValueError(f"A prim already exists at path: '{prim_path}'.")
        root = create_prim(prim_path, prim_type="Xform", translation=translation, orientation=orientation, stage=stage)
        geometry = f"{prim_path}/geometry"
        create_prim(geometry, prim_type="Xform", stage=stage)
        surfaces: dict[tuple, str] = {}

        def surface(shape: Shape) -> str:
            """The friction material of a shape: its own values or the body's; one prim per distinct surface."""
            key = (*(shape.friction or spec.friction), shape.friction_combine or spec.friction_combine, shape.soft)
            if key not in surfaces:
                surfaces[key] = f"{geometry}/friction{len(surfaces)}"
                material = sim_utils.RigidBodyMaterialCfg(static_friction=key[0], dynamic_friction=key[1], friction_combine_mode=key[2])
                material.func(surfaces[key], material)
                if shape.soft is not None:  # a compliant contact (see ``soften``)
                    from pxr import PhysxSchema

                    api = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(surfaces[key]))
                    api.CreateCompliantContactStiffnessAttr(float(shape.soft[0]))
                    api.CreateCompliantContactDampingAttr(float(shape.soft[1]))
            return surfaces[key]

        looks: dict[tuple, str] = {}
        for i, shape in enumerate(spec.shapes):
            path = f"{geometry}/shape{i}"
            half = shape.half_extents()
            if shape.kind == "box":
                kind, attributes, scale, extent = "Cube", {"size": 1.0}, tuple(shape.dims), np.full(3, 0.5)
            elif shape.kind == "sphere":
                kind, attributes, scale, extent = "Sphere", {"radius": shape.dims[0]}, None, half
            else:
                kind = "Cylinder" if shape.kind == "cylinder" else "Capsule"
                attributes, scale, extent = {"radius": shape.dims[0], "height": shape.dims[1], "axis": "Z"}, None, half
            prim = create_prim(path, prim_type=kind, translation=shape.pos, orientation=shape.quat, scale=scale, attributes=attributes, stage=stage)
            if kind != "Cube":  # round shapes are drawn as coarse polygons unless their refinement is raised
                prim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool).Set(True)
                prim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int).Set(2)
            UsdGeom.Boundable(prim).CreateExtentAttr([Gf.Vec3f(*(float(-v) for v in extent)), Gf.Vec3f(*(float(v) for v in extent))])
            if shape.solid:
                schemas.define_collision_properties(path, PhysxCollisionPropertiesCfg(contact_offset=CONTACT_OFFSET, rest_offset=REST_OFFSET), stage=stage)
                bind_physics_material(path, surface(shape), stage=stage)
                if shape.passes:  # no contact with the solid shapes of the named bodies
                    env = prim_path.rsplit("/", 1)[0]
                    others = [f"{env}/{name}/geometry/shape{k}" for name in shape.passes for k, other in enumerate(MECH.bodies[name].shapes) if other.solid]
                    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(others)
            colour = (*(shape.color if shape.color is not None else spec.color), shape.glow)  # red, green, blue, glow
            if colour not in looks:
                looks[colour] = f"{geometry}/colour{len(looks)}"
                lit = linear_rgb(colour[:3])
                look = sim_utils.PreviewSurfaceCfg(diffuse_color=lit, emissive_color=tuple(shape.glow * float(v) for v in lit), roughness=0.4)
                look.func(looks[colour], look)
            bind_visual_material(path, looks[colour], stage=stage)
        if spec.free:
            schemas.define_mass_properties(prim_path, sim_utils.MassPropertiesCfg(mass=spec.mass), stage=stage)
            schemas.define_rigid_body_properties(
                prim_path,
                PhysxRigidBodyPropertiesCfg(
                    solver_position_iteration_count=SOLVER_ITERATIONS[0], solver_velocity_iteration_count=SOLVER_ITERATIONS[1],
                    max_depenetration_velocity=MAX_DEPENETRATION, linear_damping=spec.linear_damping, angular_damping=spec.angular_damping,
                ),
                stage=stage,
            )  # fmt: skip
        elif spec.kinematic:
            schemas.define_rigid_body_properties(prim_path, PhysxRigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True), stage=stage)
        env_ns = prim_path.rsplit("/", 1)[0]
        for j in MECH.joints.values():
            if j.child == spec.name:
                _author_joint(stage, j, env_ns, UsdPhysics, Gf)
        MECH.body_prims[spec.name] = prim_path
        return root

    return spawn


def _author_joint(stage, j: Joint, env_ns: str, UsdPhysics, Gf) -> None:  # noqa: N803 - pxr module names
    """Write ``j`` as a USD physics joint under its child body. Angles in USD are degrees."""
    from pxr import PhysxSchema

    child_path = f"{env_ns}/{j.child}"
    parent_spec = MECH.bodies[j.parent] if j.parent is not None else None
    fixed_parent = parent_spec is None or not parent_spec.free  # a fixed body is the world to PhysX
    define = {"hinge": UsdPhysics.RevoluteJoint.Define, "slide": UsdPhysics.PrismaticJoint.Define, "weld": UsdPhysics.FixedJoint.Define}[j.kind]
    prim = define(stage, f"{child_path}/joint_{j.name}")
    frame = frame_with_x(j.axis)  # the joint frame in the world as built: x along the axis
    (tp, rp), (tc, rc) = (WORLD_POSE if fixed_parent else built_pose(j.parent)), built_pose(j.child)
    anchor = np.array(j.anchor)
    if not fixed_parent:
        prim.CreateBody0Rel().SetTargets([f"{env_ns}/{j.parent}"])
    prim.CreateBody1Rel().SetTargets([child_path])
    for which, (t, r) in (("0", (tp, rp)), ("1", (tc, rc))):
        local_pos, local_rot = r.T @ (anchor - t), matrix_to_quat(r.T @ frame)
        getattr(prim, f"CreateLocalPos{which}Attr")().Set(Gf.Vec3f(*(float(v) for v in local_pos)))
        getattr(prim, f"CreateLocalRot{which}Attr")().Set(Gf.Quatf(float(local_rot[3]), Gf.Vec3f(*(float(v) for v in local_rot[:3]))))
    if j.kind == "weld":  # no axis, no travel: the two frames stay on each other until the joint is switched off (cut)
        prim.CreateJointEnabledAttr(bool(j.made))
        # The simulator keeps the two bodies of a joint from touching each other, and goes on doing so when the joint
        # is switched off: the bodies of a cut weld sank into each other (sausage_cut, recordings up to n9t). Saying
        # at the cut that they may touch changes nothing either (recording weld2). So every weld says from the start
        # that its bodies touch, while it holds and after: checked in the simulator by mech/tasks/spike_welds.
        prim.CreateCollisionEnabledAttr(True)
        MECH.joint_prims[j.name] = str(prim.GetPrim().GetPath())
        PhysxSchema.PhysxJointAPI.Apply(prim.GetPrim())
        return
    prim.CreateAxisAttr("X")
    to_usd = math.degrees if j.kind == "hinge" else float  # USD: revolute limits and drive targets are in degrees
    prim.CreateLowerLimitAttr(to_usd(j.lower))
    prim.CreateUpperLimitAttr(to_usd(j.upper))
    instance = "angular" if j.kind == "hinge" else "linear"
    per_unit = math.radians(1.0) if j.kind == "hinge" else 1.0  # N m/rad -> N m/deg
    if j.stiffness > 0 or j.damping > 0:
        drive = UsdPhysics.DriveAPI.Apply(prim.GetPrim(), instance)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(j.stiffness * per_unit)
        drive.CreateDampingAttr(j.damping * per_unit)
        drive.CreateTargetPositionAttr(to_usd(j.rest))
        drive.CreateMaxForceAttr(1.0e6)
    elif j.friction > 0:
        # Dry friction: a drive that can push no harder than the friction value. As built it holds the joint where
        # it is (a spring that reaches the friction value ``give`` away); _hold_joints turns it into a brake
        # towards zero speed while the joint moves, and back.
        spring, _ = _friction_drive(j)
        drive = UsdPhysics.DriveAPI.Apply(prim.GetPrim(), instance)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(spring * per_unit)
        drive.CreateDampingAttr(spring * HOLD_SETTLE_S * per_unit)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateMaxForceAttr(j.friction)
    MECH.joint_prims[j.name] = str(prim.GetPrim().GetPath())
    if j.stop > 0:  # the end stops as springs. Unlike a drive's, a limit's stiffness is taken per radian (measured: run spike2)
        limit = PhysxSchema.PhysxLimitAPI.Apply(prim.GetPrim(), instance)
        limit.CreateStiffnessAttr(j.stop)
        limit.CreateDampingAttr(j.stop_damping)
    PhysxSchema.PhysxJointAPI.Apply(prim.GetPrim())


def _patch_scene(scene) -> None:
    """Wrap the three functions of ``yam_arena.scene`` named in the module docstring. Idempotent."""
    if getattr(scene, "_mechkit_patched", False):
        return
    stock_object, stock_states, stock_callback = scene.task_object, scene.object_states, scene.env_cfg_callback

    def task_object(spec):
        if not isinstance(spec, Body):
            return stock_object(spec)
        import isaaclab.sim as sim_utils
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

        props: dict[str, Any] = dict(collision_props=PhysxCollisionPropertiesCfg())
        if spec.free:  # Isaac Lab decides from these whether the prim is a rigid object; the values are set by the spawner
            props.update(rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass))
        elif spec.kinematic:
            props.update(rigid_props=PhysxRigidBodyPropertiesCfg(kinematic_enabled=True))
        cfg = sim_utils.CuboidCfg(func=_spawner(spec), size=tuple(spec.size), **props)
        return Object(
            name=spec.name,
            object_type=ObjectType.RIGID if spec.free or spec.kinematic else ObjectType.BASE,
            spawner_cfg=cfg,
            initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=tuple(spec.quat if spec.quat is not None else yaw_quat(spec.yaw))),
        )

    def object_states(env, names):
        if MECH.bodies and env is not None:
            MECH.environment = env
            MECH.steps += 1
            MECH.time = MECH.steps * float(env.unwrapped.step_dt)
            _read_bodies(env)
            _tick_redraw()
            if MECH.step_hooks:
                for hook in MECH.step_hooks:
                    hook()
                _read_bodies(env)  # a hook may have moved a body
            _hold_joints()
        return stock_states(env, names)

    def env_cfg_callback(control_hz=None):
        configure = stock_callback(control_hz)

        def configure_with_buffers(env_cfg):
            env_cfg = configure(env_cfg)
            try:
                from isaaclab_physx.physics import PhysxCfg

                physics = env_cfg.sim.physics
                if physics is None:
                    physics = env_cfg.sim.physics = PhysxCfg()
                if isinstance(physics, PhysxCfg):
                    for attr in ("gpu_found_lost_pairs_capacity", "gpu_found_lost_aggregate_pairs_capacity", "gpu_total_aggregate_pairs_capacity"):
                        if hasattr(physics, attr):
                            setattr(physics, attr, PAIR_CAPACITY)
            except ImportError:
                pass
            return env_cfg

        return configure_with_buffers

    scene.task_object, scene.object_states, scene.env_cfg_callback = task_object, object_states, env_cfg_callback
    scene._mechkit_patched = True


class _SceneHook(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` right after its first import (it can only be imported once Isaac Sim runs)."""

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, run = spec.loader, spec.loader.exec_module

        def exec_module(module):
            run(module)
            _patch_scene(module)

        loader.exec_module = exec_module  # type: ignore[method-assign]
        return spec


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _SceneHook) for f in sys.meta_path):
        sys.meta_path.insert(0, _SceneHook())
    from yam_arena import inspect_embodiment

    cls = inspect_embodiment.YamArenaEmbodiment
    if not getattr(cls, "_mechkit_patched", False):
        stock_reset = cls.reset

        def reset(self, scene, *, seed=None):
            if self.task is None or not (MECH.bodies or MECH.reset_hooks):
                return stock_reset(self, scene, seed=seed)
            MECH.new_episode()
            MECH.seed = seed
            self._ensure_sim(None)  # a task file has no goal: the same call the stock reset makes, so it is a no-op there
            _hook_env_reset(self._env)
            jointed = {name for j in MECH.joints.values() for name in (j.child, j.parent) if name is not None}
            self._free_objects = tuple(name for name in self._free_objects if name not in jointed)  # a shifted link would tear its joint
            return _first_picture(self, stock_reset(self, scene, seed=seed))

        cls.reset = reset
        cls._mechkit_patched = True


def _hook_env_reset(env) -> None:
    """Run the reset hooks right after the simulator's own reset: every body is back where it was built, nothing
    has moved yet, and the embodiment's settling steps come afterwards."""
    if getattr(env, "_mechkit_reset_hooked", False):
        return
    stock = env.reset

    def reset(*args, **kwargs):
        out = stock(*args, **kwargs)
        MECH.environment = env
        _draw_all()
        mend_all()
        _restore_masses()
        _stop_pushing()
        _forget_at_reset()
        _run_reset_hooks()
        _after_reset()
        return out

    env.reset = reset
    env._mechkit_reset_hooked = True


_install()
