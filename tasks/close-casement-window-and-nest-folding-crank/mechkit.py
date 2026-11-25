# SPDX-License-Identifier: MIT
"""mechkit: multi-part objects and spring joints for yam_arena task files.

yam_arena task files hold single primitives. A ``Mech`` is one object built from several shapes, which can carry moving
pieces: a ``Joint`` is a slide or a hinge, with end stops and an optional spring that returns it to its rest position.
That is enough for hollow fixtures, sockets, keyed parts, drawers, doors, lids, spring latches and buttons.

    from mechkit import Mech, Joint, box, cyl

    DRAWER = Mech("cabinet", pos=(0.25, 0.0, TOP), mass=None,
                  shapes=[box((0, 0, 0.05), (0.2, 0.2, 0.01), GREY)],
                  joints=[Joint("drawer", "slide", axis="x", at=(0, 0, 0.03), low=-0.12, high=0.0,
                                shapes=[box((0, 0, 0), (0.16, 0.16, 0.03), BLUE)])])
    OBJECTS = [DRAWER, ...]

    def score(objects):
        return 1.0 if objects["cabinet"].joints["drawer"] < -0.1 else 0.0

Positions in a Mech are metres from its ``pos``, turned by its ``yaw``. Joint values are measured from rest: metres for a
slide, radians for a hinge, positive along ``axis`` by the right-hand rule. ``mass=None`` fixes the object to the world.

Real geometry. Besides ``box`` and ``cyl`` (native colliders, now with any ``axis`` and ``tilt``), the builders ``rod``,
``lathe``, ``cone``, ``ball``, ``tube``, ``ring_profile``, ``bore``, ``extrude``, ``arc``, ``sweep`` and ``hull`` make
round, turned, hollow, concave and curved parts. Each returns a Shape of kind "mesh": a smooth visible mesh plus hidden
convex collision pieces (at most 64 vertices each, at least 0.5 mm thick), all computed in NumPy when the task file is
imported. A round pin in a round hole, with 1 mm clearance per side and a 2 mm entry chamfer:

    from mechkit import Mech, bore, lathe

    BLOCK = Mech("block", pos=(0.25, 0.0, TOP + 0.01), mass=None,
                 shapes=[bore((0, 0, 0), (0.05, 0.05, 0.02), 0.010, ORANGE, lead_in=0.002)])
    PIN = Mech("pin", pos=(0.25, 0.12, TOP + 0.03), mass=0.06,
               shapes=[lathe((0, 0, -0.03), [(0.006, 0.0), (0.009, 0.003), (0.009, 0.06)], BLUE)])

Measure with ``shape_aabb``, ``gap`` (signed clearance between two shapes' colliders), ``shape_volume`` and
``mass_props``; ``Mech.points()`` gives every collider point for bounds. Every consumer raises on an unknown shape kind.
See GEOMETRY.md beside this file for each builder and the rules.

A sprung piece sags a little under its own weight before anything touches it, by mass x g / spring for a slide, so give
score() a margin: measured in Isaac, a 30 g button on 150 N/m sits at -0.002 m. Fixed and free Mechs, slides and hinges have
all been run in Isaac: buttons press and return, hinges turn, multi-part blocks and jointed objects lift and carry.

Parts that stay where they are left: ``Joint(..., hold=1.5, hold_moving=1.2)`` gives a joint Coulomb friction (N for a
slide, N m for a hinge), and ``detents=(0.0, -0.01, -0.02)`` with a spring makes it click into the nearest stop.
``hold_report(mech)``, ``deadband(joint)`` and ``creep(joint, seconds)`` give the numbers for check.py. Measured in
Isaac (kit smoke run 2026-11-12): hold is an effort (breakaway within 10% of the set value whatever the mass), and
detents click. A hold is not a lock under a steady load: a vertical slide or tilting hinge held at 2 x its gravity load
crept about 1 mm or 1 deg in the first seconds and then about 0.03 mm/s or 0.14 deg/s. Let an end stop or a detent
spring carry a weight whose position is scored; see creep() and GEOMETRY.md.
The kind "mesh" builders are likewise unverified in Isaac until that run passes: hidden guide hulls, minThickness,
GPU cooking of the cap faces and the PhysX mass, COM and inertia against mass_props are checked only there.

Chained joints: ``Joint(..., parent="carriage")`` makes a joint ride on another joint of the same Mech (a hinge on a
slide, a latch on a lid), up to 3 joints deep; its ``at`` is then in the parent joint's moving frame. ``Mech.joint_pose``
and ``body_poses`` give each moving body's pose, and a joint never touches the body it rides on (see GEOMETRY.md).

Screw threads: ``screw("nut", shapes, lead=0.00175, low=-0.01, high=0.0, seat=-0.008)`` returns a carrier slide and a
threaded hinge riding on it. Before every physics step the kit writes the slide's position target from the hinge's
measured turn (travel = turns x lead), so turning the part advances it; past the ``seat`` the travel stops and the
hinge's hold grows with every turn. ``screw_states`` gives turns, travel and seated for score(). It is a kinematic thread
with real geometry for the parts, not a contact-resolved helix (GEOMETRY.md, "Screw threads").

Indicator lamps: ``lamp("status", at, radius, {"red": RED, "green": GREEN})`` gives look-only lens shapes; an on_step
hook calls ``set_lamp(MECH, "status", "red")`` to show one colour, and ``timetable`` / ``blink_code`` give blink codes.
Offline set_lamp only records the state; ``MechState.lamps`` reports it to score() (GEOMETRY.md, "Indicator lamps").

Gears: ``Joint("needle", ..., follows=Gear("knob", 0.4))`` (or ``follows=("knob", 0.4)``) drives the needle at 0.4 x
the knob's angle, a rack from a pinion (m/rad) or a pinion from a rack (rad/m), with optional ``offset`` and
``backlash``. It is a two-way spring coupling of finite ``stiffness`` written into both joints' position drives
before every physics step, so a blocked follower stalls the driver through the drive and nothing is placed by a
formula. Mech refuses a gear the 240 Hz step would not keep stable. ``gear_report``, ``gear_target`` and
``gear_slip`` for check.py and score(). Not meshing teeth: the fidelity note must say so (see Gear).

Contact: ``watch_touch("hot_plate")`` at import, then ``touch_state("hot_plate")`` in score() says whether a finger pad
of either arm touched it this episode (``touched``), the peak and latest normal force and which pad, read from PhysX
contact views before every physics step. ``watch_fingers()`` and ``finger_force(arm)`` give the net force on an arm's
fingers from anything (a fingertip contact sense). Offline both return what ``mock_touch`` / ``mock_finger_force``
scripted (untouched, 0 N by default). A fixed target with no rigid body falls back to a 2 mm pad-to-bounding-box
distance and reports source "proximity".

Scorer-side hooks: ``@on_step`` runs ``fn(env)`` after every control step, before scoring, and
``released(objects["part"])`` (in score() or an on_step hook) is True only when every finger pad of both arms is at
least 3 cm from the part. Offline, with no simulator, released() is always False. A demo never reads either.

The task file runs with the standard ``yam_arena`` embodiment and ``task_score`` scorer: importing this module adds Mech
support to ``yam_arena.scene`` and the on_step hook to ``yam_arena.inspect_embodiment`` as they load, and changes
nothing else. Keep this file beside the task file.
Importing it needs only NumPy and yam_arena; everything that needs Isaac Sim is imported when the simulator builds the scene.

Assets: all geometry is procedural, generated by this file; no third-party meshes, textures or other files ship with it.
"""

from __future__ import annotations

import dataclasses
import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any, NamedTuple

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Mech", "Joint", "MechState", "Shape", "box", "cyl", "rod", "lathe", "cone", "ball", "tube", "ring_profile",
           "bore", "extrude", "arc", "sweep", "hull", "mass_props", "shape_volume", "shape_points", "shape_aabb", "gap",
           "on_reset", "on_step", "deadband", "hold_report", "set_hold", "set_rest", "nearest_detent",
           "finger_points", "finger_clearance", "released", "gravity_lever", "gravity_load",
           "contact_offset", "max_drop", "creep", "creep_rate", "gravity_sign", "articulation_iterations",
           "body_poses", "contact_pairs", "self_gap", "MAX_CHAIN",
           "Thread", "ScrewState", "screw", "screw_law", "screw_hold", "screw_unwrap", "screw_states", "screw_report",
           "seat_travel", "lamp", "lamp_states", "set_lamp", "timetable", "blink_code", "step_count", "elapsed",
           "Gear", "GearTargets", "gear_law", "gear_gains", "gear_target", "gear_slip", "gear_report",
           "max_gear_stiffness", "Touch", "TouchState", "watch_touch", "watch_fingers", "touch_state", "finger_force",
           "mock_touch", "mock_finger_force", "clear_contact_mocks", "touch_report", "FINGER_LINKS"]

_RESET_HOOKS: list[Any] = []
_STEP_HOOKS: list[Any] = []
_ENV: dict[str, Any] = {"env": None}  # the live env, set before every on_step / on_reset run
_LIVE: dict[tuple[str, str], float] = {}  # (mech, joint) -> spring rest currently written by a detent
_SCREWS: dict[tuple[str, str], dict[str, float]] = {}  # (mech, threaded hinge) -> unwrapped theta, last reading, hold
_ARMED: dict[str, bool] = {"screws": False, "gears": False, "contacts": False}  # physics-step writes and contact
# reads run only between a finished reset and the next one
_LAMP_STATE: dict[tuple[str, str], str] = {}  # (mech, lamp) -> state set by set_lamp ("off" or a lit state)
_LAMP_PRIMS: dict[str, dict[str, dict[str, str]]] = {}  # mech -> lamp -> state -> lens prim path (env 0)
_LAMP_SHOWN: dict[str, bool] = {}  # lens prim path -> shown, so set_lamp only writes changes
_CLOCK: dict[str, float] = {"steps": 0, "seconds": 0.0}  # control steps and seconds since the last reset
G = 9.81
HOLD_MARGIN = 1.5  # a hold must beat gravity by this factor
HOLD_MOVING = 0.8  # default moving (dynamic) hold as a fraction of the static hold
# Creep of a hold under a steady gravity load. Kit smoke run 2026-11-12 (Isaac Sim, TGS, 240 Hz physics, 32 articulation
# position iterations, hold 2 x the load): a 0.1 kg vertical slide (0.98 N, 9.81 m/s^2 unbalanced) dropped 1.0 mm in
# the first 5 s, then 0.03 mm/s; a 0.1 kg hinge arm (0.029 N m on 1.21e-4 kg m^2, 243 rad/s^2) turned 0.85 deg, then
# 0.10 to 0.14 deg/s. creep() bounds both: a start allowance plus a rate proportional to the acceleration gravity would
# give the free joint, at 1.5 x the measured rate per unit acceleration. Proportionality to acceleration, and no help
# from a bigger hold, is the working model (velocity-level joint friction leaks a fraction of every step's gravity
# impulse, about g * dt / iterations^2 per second); the next smoke run tests it with 2x, 4x and 8x holds, armature,
# damping and 64 iterations. Neither armature nor extra iterations is credited until it is measured.
CREEP_START = {"slide": 0.0015, "hinge": math.radians(1.5)}  # m, rad: allowance for the first seconds
CREEP_PER_ACCEL = {"slide": 4.5e-6, "hinge": 1.5e-5}  # s: drift rate (m/s or rad/s) per m/s^2 or rad/s^2 unbalanced
CREEP_ITERATIONS = 64  # articulation position iterations mechkit authors on a Mech with a gravity-held joint


def on_reset(fn):
    """Register ``fn(env)`` to run whenever a yam_arena episode resets (clears latched scorer state across episodes)."""
    _RESET_HOOKS.append(fn)
    return fn


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored (same as on_reset's env).
    Scorer side only: use it to latch facts score() cannot see, such as whether the fingers have let go. A demo must
    never read what these hooks record."""
    _STEP_HOOKS.append(fn)
    return fn

Vec3 = tuple[float, float, float]
AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
GREY = (0.5, 0.5, 0.5)
IDENT = (1.0, 0.0, 0.0, 0.0)
KINDS = ("box", "cyl", "mesh")
FORMS = ("cyl", "lathe", "tube", "bore", "extrude", "arc", "sweep", "hull")


@dataclass(frozen=True)
class Shape:
    """A collider that also shows.

    kind "box": ``size`` is x, y, z edge lengths. kind "cyl": a native cylinder, ``size`` is (radius, radius, height)
    along ``axis``. kind "mesh" (made by the builders, never by hand): ``hulls`` holds convex collision pieces, each
    (points, counts, indices) in the shape frame, and ``visual`` the render mesh (points, counts, indices, corner
    normals); ``at`` is the centre of the collision bounding box and ``size`` its extents before ``tilt``.
    ``collide=False`` makes a look-only shape, ``visible=False`` an invisible collider (use dataclasses.replace).
    ``contact`` overrides the 0.0015 m contact offset. Rest offsets are always 0, so an offset larger than a fit's
    clearance only makes contacts early and never pushes parts apart: keep the default on fits, and see max_drop()
    before lowering it.

    ``lamp`` and ``lit`` are set by ``lamp()``: the shape is a lens of that indicator lamp, its unlit lens when ``lit``
    is empty, else the lens shown in state ``lit``. Lamp lenses are look-only box or cyl shapes."""

    kind: str
    at: Vec3
    size: Vec3
    color: Vec3 = GREY
    tilt: tuple[float, float, float, float] = IDENT  # rotation in the part frame, (w, x, y, z)
    axis: str = "z"
    form: str = ""
    hulls: tuple = field(default=(), repr=False)
    visual: tuple = field(default=(), repr=False)
    collide: bool = True
    visible: bool = True
    contact: float | None = None
    lamp: str = ""
    lit: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.lamp, str) or not isinstance(self.lit, str):
            raise ValueError(f"shape lamp and lit must be strings, got {self.lamp!r}, {self.lit!r}")
        if self.lit and not self.lamp:
            raise ValueError(f"a lit lens ({self.lit!r}) must name its lamp")
        if self.lamp:
            if self.collide or not self.visible:
                raise ValueError(f"lamp {self.lamp}: a lens is look-only (collide=False) and visible")
            if self.kind not in ("box", "cyl"):
                raise ValueError(f"lamp {self.lamp}: a lens is a box or a native cyl, got {self.kind!r}")
            if self.lit == "off":
                raise ValueError(f"lamp {self.lamp}: 'off' is the unlit lens; name lit states otherwise")
        if self.kind not in KINDS:
            raise ValueError(f"shape kind must be one of {KINDS}, got {self.kind!r}")
        if self.axis not in AXES:
            raise ValueError(f"shape axis must be x, y or z, got {self.axis!r}")
        if self.kind == "mesh":
            if self.form not in FORMS:
                raise ValueError(f"mesh form must be one of {FORMS}, got {self.form!r}")
            if not self.hulls:
                raise ValueError("a mesh shape needs its convex hulls")
            if not self.visual:
                raise ValueError("a mesh shape needs its visual mesh")
        elif self.hulls or self.visual or self.form:
            raise ValueError(f"only mesh shapes carry hulls, visual or form, not {self.kind!r}")
        if self.kind == "box" and self.axis != "z":
            raise ValueError("a box has no axis; turn it with tilt")
        if not (self.collide or self.visible):
            raise ValueError("a shape must collide, show, or both")
        if self.contact is not None and not 0.0 <= self.contact <= 0.02:
            raise ValueError(f"contact offset must be 0 to 0.02 m, got {self.contact!r}")


@dataclass(frozen=True)
class Thread:
    """What makes a hinge a screw (``Joint(..., thread=Thread(...))``, usually made by ``screw()``). The hinge rides on a
    coaxial carrier slide (its ``parent``); every physics step the kit sets the slide's target to ``lead`` x turns.

    ``lead``: travel along the axis per turn (m); positive for a right-hand thread (a turn counter-clockwise about the
    axis, by the right-hand rule, advances the part along +axis), negative for a left-hand one. ``seat``: the carrier
    travel (m) where the part meets what it clamps; turning further does not move it, and each turn past the seat adds
    ``tighten`` (N m per turn) to the hinge's hold, up to ``max_hold`` (None: no cap)."""

    lead: float
    seat: float | None = None
    tighten: float = 0.0
    max_hold: float | None = None

    def __post_init__(self) -> None:
        lead = _finite("thread: lead", self.lead)
        if lead == 0.0 or abs(lead) > 0.05:
            raise ValueError(f"thread: lead must be nonzero and at most 0.05 m per turn, got {lead}")
        if self.seat is not None and _finite("thread: seat", self.seat) == 0.0:
            raise ValueError("thread: seat must be a travel other than 0 (rest), or None")
        tighten = _finite("thread: tighten", self.tighten)
        if tighten < 0:
            raise ValueError(f"thread: tighten must be >= 0, got {tighten}")
        if tighten > 0 and self.seat is None:
            raise ValueError("thread: tighten needs a seat to count turns past")
        if self.max_hold is not None and _finite("thread: max_hold", self.max_hold) < 0:
            raise ValueError(f"thread: max_hold must be >= 0, got {self.max_hold}")


@dataclass(frozen=True)
class Gear:
    """What makes a joint follow another (``Joint(..., follows=Gear("knob", 6.0))``, or the tuple
    ``follows=("knob", 6.0)`` / ``("knob", 6.0, offset)``): a gear, belt or rack coupling inside one Mech.

    Law: follower = ``ratio`` x driver + ``offset``, both measured from rest like ``MechState.joints``. ``ratio`` is
    follower units per driver unit: rad/rad for two hinges (negative for an external mesh, which turns the other way),
    m/m for two slides, m/rad for a pinion driving a rack (its pitch radius) and rad/m for a rack driving a pinion.
    ``backlash`` is the total lost motion measured at the driver (rad or m): reverse the driver this far before the
    follower moves again.

    It is a two-way physical drive, not a formula that places the follower. Before every physics step the kit writes
    the follower's position drive toward the law (a spring of ``stiffness``, follower units: N m/rad or N/m) and the
    driver's position drive toward the matching point (stiffness x ratio^2), so each joint feels the other's reaction:
    a blocked follower stalls the driver after a wind up of about effort / (stiffness x ratio^2), and pushing the
    follower back drives the driver (no self-locking, unlike a screw). ``damping`` acts on the slip between them
    (None: critical for the pair). ``stiffness`` None picks half the stiffest value the coupling stays stable at."""

    driver: str
    ratio: float
    offset: float = 0.0
    backlash: float = 0.0
    stiffness: float | None = None
    damping: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.driver, str) or not self.driver:
            raise ValueError(f"gear: driver must be the name of another joint of the Mech, got {self.driver!r}")
        ratio = _finite("gear: ratio", self.ratio)
        if ratio == 0.0:
            raise ValueError("gear: ratio must be nonzero")
        object.__setattr__(self, "ratio", ratio)
        object.__setattr__(self, "offset", _finite("gear: offset", self.offset))
        backlash = _finite("gear: backlash", self.backlash)
        if backlash < 0:
            raise ValueError(f"gear: backlash must be >= 0, got {backlash}")
        object.__setattr__(self, "backlash", backlash)
        if self.stiffness is not None:
            k = _finite("gear: stiffness", self.stiffness)
            if k <= 0:
                raise ValueError(f"gear: stiffness must be > 0, got {k}")
            object.__setattr__(self, "stiffness", k)
        if self.damping is not None:
            c = _finite("gear: damping", self.damping)
            if c < 0:
                raise ValueError(f"gear: damping must be >= 0, got {c}")
            object.__setattr__(self, "damping", c)


@dataclass(frozen=True)
class Joint:
    """A moving piece. ``at`` is the joint's origin in the Mech frame and the moving piece's own frame at rest. ``low`` and
    ``high`` are its stops, which must include 0 (rest). ``spring`` is N/m or N m/rad toward rest; 0 leaves it free.
    ``friction`` is the contact friction of its shapes.

    ``hold`` is joint (Coulomb) friction: the effort, N for a slide or N m for a hinge, needed to start it moving, so a
    part stays where it is left. ``hold_moving`` is the effort while it moves (default 0.8 x hold). With a spring, the
    piece may rest anywhere within ``deadband(joint)`` = hold / spring of its rest. A hold on an unsprung vertical slide
    or a horizontal hinge must beat gravity by 1.5 times, or the joint is refused as one that would sag. Even then it
    creeps under that steady load (``creep(joint, seconds)``): let a stop or a detent spring carry a scored position.
    ``armature`` (kg for a slide, kg m^2 for a hinge, default 0) adds inertia in the joint, as a gear train would.

    ``detents`` are rest positions (m or rad, inside the stops, including 0) the spring snaps between: every control
    step the spring's rest moves to the stop nearest the joint, so the part clicks into the closest one. They need
    ``spring > 0``. Two stops give an over-centre toggle; present these as detent springs, never as snap fits.

    ``parent`` is the name of another joint of the same Mech that this one rides on (None: the Mech base). ``at`` and
    ``axis`` are then in the parent's moving frame (origin at the parent's body origin, axes lined up with the Mech
    frame while every joint is at rest, so rest origins add along the chain); chains
    are at most MAX_CHAIN (3) joints deep. A joint never collides with the body it rides on (``contact_pairs``).

    ``thread`` makes a hinge a screw on its carrier slide (see Thread and ``screw()``). A threaded hinge is authored
    without limits (it spins freely, like a cap on its neck) and its value is the kit's unwrapped turn angle; the travel
    stops are the carrier's ``low`` and ``high``. A carrier slide may have no shapes; any other joint needs one.

    ``follows`` makes this joint a follower geared to another joint of the same Mech (see Gear): a meter needle on a
    knob, a geared output on a crank, a rack on a pinion. Its own ``spring`` and ``damping`` still act toward rest."""

    name: str
    kind: str  # "slide" or "hinge"
    shapes: tuple[Shape, ...]
    axis: str = "z"
    at: Vec3 = (0.0, 0.0, 0.0)
    low: float = 0.0
    high: float = 0.01
    spring: float = 0.0
    damping: float = 5.0
    mass: float = 0.02
    friction: float | None = None
    hold: float = 0.0
    hold_moving: float | None = None
    detents: tuple[float, ...] = ()
    armature: float = 0.0
    parent: str | None = None
    thread: Thread | None = None
    follows: Gear | tuple | None = None

    def __post_init__(self) -> None:
        if self.follows is not None:
            g = self.follows
            if isinstance(g, (tuple, list)):
                if not 2 <= len(g) <= 3:
                    raise ValueError(f"joint {self.name}: follows takes (driver, ratio) or (driver, ratio, offset), "
                                     f"or a Gear; got {g!r}")
                g = Gear(*g)
            if not isinstance(g, Gear):
                raise ValueError(f"joint {self.name}: follows must be a Gear, a (driver, ratio[, offset]) tuple or "
                                 f"None, got {g!r}")
            if g.driver == self.name:
                raise ValueError(f"joint {self.name}: cannot follow itself")
            if self.thread is not None:
                raise ValueError(f"joint {self.name}: a threaded hinge cannot also follow a gear")
            object.__setattr__(self, "follows", g)
        if self.thread is not None:
            if not isinstance(self.thread, Thread):
                raise ValueError(f"joint {self.name}: thread must be a Thread or None, got {self.thread!r}")
            if self.kind != "hinge":
                raise ValueError(f"joint {self.name}: only a hinge carries a thread")
            if self.parent is None:
                raise ValueError(f"joint {self.name}: a threaded hinge rides on its carrier slide: give parent")
            if self.spring != 0 or self.detents:
                raise ValueError(f"joint {self.name}: a threaded hinge has no spring or detents")
        if self.parent is not None and (not isinstance(self.parent, str) or not self.parent):
            raise ValueError(f"joint {self.name}: parent must be the name of another joint of the Mech, or None, "
                             f"got {self.parent!r}")
        if self.parent == self.name:
            raise ValueError(f"joint {self.name}: cannot ride on itself")
        if self.kind not in ("slide", "hinge"):
            raise ValueError(f"joint {self.name}: kind must be slide or hinge, got {self.kind!r}")
        if self.axis not in AXES:
            raise ValueError(f"joint {self.name}: axis must be x, y or z, got {self.axis!r}")
        if not self.low <= 0.0 <= self.high or self.low == self.high:
            raise ValueError(f"joint {self.name}: stops {self.low}..{self.high} must surround 0, the rest position")
        if not self.shapes and self.kind != "slide":  # a shapeless slide is checked by its Mech (a thread carrier)
            raise ValueError(f"joint {self.name}: needs at least one shape")
        object.__setattr__(self, "shapes", tuple(self.shapes))
        armature = _finite(f"joint {self.name}: armature", self.armature)
        if armature < 0:
            raise ValueError(f"joint {self.name}: armature must be >= 0, got {armature}")
        _check_hold(self)

    @property
    def moving_hold(self) -> float:
        """The effort while moving: ``hold_moving``, or 0.8 x hold when it is not given."""
        return HOLD_MOVING * self.hold if self.hold_moving is None else float(self.hold_moving)


@dataclass(frozen=True)
class Mech(Box):
    """A multi-part object. Subclasses Box because task files accept only the primitive types; ``size`` is filled in as
    the bounding box at rest. Give ``mass`` for a free object, None to fix it to the world. ``iterations`` sets the
    articulation's solver position iterations (1 to 255); None lets mechkit choose (``articulation_iterations``)."""

    shapes: tuple[Shape, ...] = ()
    joints: tuple[Joint, ...] = ()
    friction: float = 1.0
    iterations: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "shapes", tuple(self.shapes))
        object.__setattr__(self, "joints", tuple(self.joints))
        if self.iterations is not None and (isinstance(self.iterations, bool) or not isinstance(self.iterations, int)
                                            or not 1 <= self.iterations <= 255):
            raise ValueError(f"mech {self.name}: iterations must be a whole number 1 to 255, got {self.iterations!r}")
        if not self.shapes:
            raise ValueError(f"mech {self.name}: needs at least one shape")
        names = [j.name for j in self.joints]
        if len(set(names)) != len(names):
            raise ValueError(f"mech {self.name}: joint names repeat: {names}")
        _check_chains(self)
        _check_threads(self)
        _check_gears(self)
        _check_lamps(self)
        corners =_local_corners(self, {j.name: 0.0 for j in self.joints})
        object.__setattr__(self, "size", tuple(float(v) for v in corners.max(0) - corners.min(0)))
        self._check()

    def points(self, values: dict[str, float] | None = None) -> np.ndarray:
        """Every collider bound point in the Mech frame at these joint values (rest by default): box corners, cylinder
        rims, hull vertices. Use it for footprints and clearances; ``size`` is only the bounding box."""
        return _local_points(self, values or {})

    def joint_pose(self, name: str, values: dict[str, float] | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(rotation 3x3, origin) of joint ``name``'s moving body in the Mech frame, through every joint it rides on,
        at these joint values (rest for any joint not given). Usable as a ``gap`` pose."""
        by = {j.name: j for j in self.joints}
        if name not in by:
            raise ValueError(f"mech {self.name}: no joint {name!r} (joints: {sorted(by)})")
        return _body_pose(self, by[name], values or {})


@dataclass(frozen=True)
class MechState(ObjectState):
    """An ObjectState plus each joint's value, and helpers to read a point in the Mech's own frame. ``bodies`` holds
    each joint body's world pose (rotation 3x3, origin), relative to the env origin like ``origin``, when the simulator
    or the replay filled it; ``body_poses(mech, state)`` computes the same from any state. ``screws`` holds a
    ScrewState per threaded hinge and ``lamps`` each lamp's state ("off" or a lit state name), when the simulator or
    the replay filled them; ``screw_states(mech, state)`` computes the screws from any state."""

    joints: dict[str, float] = field(default_factory=dict)
    origin: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bodies: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    screws: dict[str, Any] = field(default_factory=dict)
    lamps: dict[str, str] = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        return _quat_xyzw_matrix(self.quat_xyzw)

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, dtype=float) - self.origin)

    def to_world(self, local_point) -> np.ndarray:
        return self.rotation() @ np.asarray(local_point, dtype=float) + self.origin


# -- geometry, pure NumPy ----------------------------------------------------------------------------------------------


def _quat_wxyz_matrix(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat_xyzw_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return _quat_wxyz_matrix((w, x, y, z))


def _axis_matrix(axis: str, angle: float) -> np.ndarray:
    half = angle / 2
    v = np.array(AXES[axis]) * math.sin(half)
    return _quat_wxyz_matrix((math.cos(half), *v))


def _shape_corners(shape: Shape) -> np.ndarray:
    """Bound points of a shape in its parent frame: the 8 corners of a box or of a cylinder's bounding box (same order
    as always), or every hull vertex of a mesh (every visual vertex when it does not collide)."""
    if shape.kind == "mesh":
        if shape.collide:
            pts = np.vstack([np.asarray(h[0], dtype=float) for h in shape.hulls])
        else:
            pts = np.asarray(shape.visual[0], dtype=float)
        return pts @ _quat_wxyz_matrix(shape.tilt).T + np.array(shape.at)
    if shape.kind not in ("box", "cyl"):
        raise ValueError(f"unknown shape kind {shape.kind!r}")
    sx, sy, sz = shape.size
    half = np.array([sx, sy, sz / 2]) if shape.kind == "cyl" else np.array([sx, sy, sz]) / 2
    if shape.kind == "cyl" and shape.axis != "z":
        half = _canon([half], shape.axis)[0]
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], dtype=float)
    return signs * half @ _quat_wxyz_matrix(shape.tilt).T + np.array(shape.at)


def _joint_pose(joint: Joint, value: float) -> tuple[np.ndarray, np.ndarray]:
    """Rotation and origin of a moving piece in its parent's frame (the Mech frame for a joint on the base) at joint
    position ``value``. A chained joint's pose in the Mech frame is ``_body_pose``."""
    at = np.array(joint.at)
    if joint.kind == "slide":
        return np.eye(3), at + np.array(AXES[joint.axis]) * value
    return _axis_matrix(joint.axis, value), at


MAX_CHAIN = 3  # joints in one chain, counting the one on the base


def _chain(spec: Mech, joint: Joint) -> list[Joint]:
    """The joints from the base out to ``joint``, base side first (just [joint] for a joint on the base)."""
    by = {j.name: j for j in spec.joints}
    out = [joint]
    while out[-1].parent is not None:
        if len(out) > len(by):
            raise ValueError(f"mech {spec.name}: joint {joint.name}: its parents form a cycle")
        if out[-1].parent not in by:
            raise ValueError(f"mech {spec.name}: joint {out[-1].name}: no joint {out[-1].parent!r} to ride on "
                             f"(joints: {sorted(by)})")
        out.append(by[out[-1].parent])
    return out[::-1]


def _check_chains(spec: Mech) -> None:
    """Refuse unknown parents, cycles and chains over MAX_CHAIN deep, then holds a chain would overload."""
    by = {k.name: k for k in spec.joints}
    for j in spec.joints:
        seen, cur = {j.name}, j
        while cur.parent is not None:
            if cur.parent not in by:
                raise ValueError(f"mech {spec.name}: joint {cur.name}: no joint {cur.parent!r} to ride on "
                                 f"(joints: {sorted(by)})")
            if cur.parent in seen:
                raise ValueError(f"mech {spec.name}: joint {j.name}: its parents form a cycle through {cur.parent!r}")
            seen.add(cur.parent)
            cur = by[cur.parent]
        if len(seen) > MAX_CHAIN:
            raise ValueError(f"mech {spec.name}: joint {j.name} is {len(seen)} joints deep "
                             f"({' > '.join(k.name for k in _chain(spec, j))}); at most {MAX_CHAIN}")
    _check_chain_holds(spec)


def _check_chain_holds(spec: Mech) -> None:
    """Joint.__post_init__ checks a hold against the joint's own weight with its axis as declared. In a chain a joint
    also carries everything riding on it, and a hinge about x or y below it can turn its axis away from the declared
    one, so check a conservative load: the whole carried mass for a slide, the centre of mass's full distance from
    the axis for a hinge. A hold on a hinge that carries other joints against gravity is refused: not checked offline."""
    kids: dict[str, list[Joint]] = {}
    for j in spec.joints:
        if j.parent is not None:
            kids.setdefault(j.parent, []).append(j)

    def carried(name: str) -> list[Joint]:
        out = []
        for k in kids.get(name, []):
            out += [k, *carried(k.name)]
        return out

    for j in spec.joints:
        if j.hold <= 0 or j.spring > 0:
            continue
        rest = carried(j.name)
        tilting = any(a.kind == "hinge" and a.axis != "z" for a in _chain(spec, j)[:-1])
        if not rest and not tilting:
            continue  # Joint.__post_init__ already checked it
        if j.kind == "hinge":
            if rest and (j.axis != "z" or tilting):
                raise ValueError(f"joint {j.name}: a hold on a hinge that carries chained joints "
                                 f"({', '.join(k.name for k in rest)}) against gravity is not checked offline; carry "
                                 "the load on a stop or a spring")
            if not tilting:
                continue
            com, e = _joint_com(j), np.array(AXES[j.axis])
            load = j.mass * G * math.sqrt(max(float(com @ com - (com @ e) ** 2), 0.0))
        elif j.axis == "z" or tilting:
            load = (j.mass + sum(k.mass for k in rest)) * G
        else:
            continue  # a level slide on level ancestors: gravity does not load it
        if j.hold < HOLD_MARGIN * load:
            unit = "N" if j.kind == "slide" else "N m"
            raise ValueError(f"joint {j.name}: would sag: hold {j.hold:.4g} {unit} is under {HOLD_MARGIN} x the "
                             f"{load:.4g} {unit} gravity can put on it in its chain; raise hold to at least "
                             f"{HOLD_MARGIN * load:.4g}, add a spring, or carry the load on a stop")


def _body_pose(spec: Mech, joint: Joint, values: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    """Rotation and origin of ``joint``'s moving body in the Mech frame, through every joint it rides on."""
    if joint.parent is None:
        return _joint_pose(joint, values.get(joint.name, 0.0))
    rot, origin = np.eye(3), np.zeros(3)
    for j in _chain(spec, joint):
        r, o = _joint_pose(j, float(values.get(j.name, 0.0)))
        rot, origin = rot @ r, rot @ o + origin
    return rot, origin


def _local_corners(spec: Mech, values: dict[str, float]) -> np.ndarray:
    pts = [_shape_corners(s) for s in spec.shapes]
    for joint in spec.joints:
        rot, origin = _body_pose(spec, joint, values)
        pts += [_shape_corners(s) @ rot.T + origin for s in joint.shapes]
    return np.vstack(pts)


def body_poses(spec: Mech, state) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """World pose (rotation 3x3, origin relative to the env origin) of every joint body of ``spec`` in ``state`` (a
    MechState from score(), the simulator, the replay or an offline mock), from its joint values. Raises when the
    state lacks a joint value rather than assume rest."""
    missing = [j.name for j in spec.joints if j.name not in state.joints]
    if missing:
        raise ValueError(f"body_poses: mech {spec.name}: the state has no value for joints {missing}")
    return _world_bodies(spec, state.joints, state.origin, state.quat_xyzw)


def _world_bodies(spec: Mech, values, origin, quat_xyzw) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    rw, ow = _quat_xyzw_matrix(np.asarray(quat_xyzw, dtype=float)), np.asarray(origin, dtype=float)
    out = {}
    for j in spec.joints:
        rot, org = _body_pose(spec, j, values)
        out[j.name] = (rw @ rot, rw @ org + ow)
    return out


def contact_pairs(spec: Mech) -> list[tuple[str, str]]:
    """Pairs of the Mech's bodies ("base" and joint names) that collide with each other in the simulator. The
    articulation has self-collision on, and PhysX never collides the two bodies one joint connects, so every pair
    collides except a joint and the body it rides on (the base, or its parent joint)."""
    names = ["base", *(j.name for j in spec.joints)]
    jointed = {frozenset((j.name, j.parent or "base")) for j in spec.joints}
    return [(a, b) for i, a in enumerate(names) for b in names[i + 1:] if frozenset((a, b)) not in jointed]


def self_gap(spec: Mech, values: dict[str, float] | None = None) -> dict[tuple[str, str], float]:
    """Least ``gap`` (m) between the colliding shapes of every pair in ``contact_pairs`` at these joint values (rest by
    default); pairs where either body has no colliding shape are left out. Negative means the bodies overlap and
    PhysX will push them apart."""
    values = values or {}
    shapes = {"base": (list(spec.shapes), (np.eye(3), np.zeros(3)))}
    for j in spec.joints:
        shapes[j.name] = (list(j.shapes), _body_pose(spec, j, values))
    out = {}
    for a, b in contact_pairs(spec):
        sa = [s for s in shapes[a][0] if s.collide]
        sb = [s for s in shapes[b][0] if s.collide]
        if sa and sb:
            out[(a, b)] = min(gap(x, shapes[a][1], y, shapes[b][1]) for x in sa for y in sb)
    return out


# -- screw threads, pure NumPy -------------------------------------------------------------------------------------------

SEAT_TOL = 0.0005  # m: a screw counts as seated when its travel is this close to the seat
SCREW_SAG_MAX = 0.0005  # m: most a thread may give under the weight it carries (mass x g / carrier spring)
SCREW_STIFFNESS = 5.0e4  # N/m: default thread stiffness (the carrier's position drive)
SCREW_CARRIER_MASS = 0.004  # kg, with 1e-6 kg m^2 inertia: the shapeless carrier, as in the accepted cap chain
SCREW_HOLD_STEP = 0.002  # N m: the hinge hold is rewritten only when its law moves this far


@dataclass(frozen=True)
class ScrewState:
    """A threaded hinge for score(): ``turns`` (signed, from rest, measured), ``travel`` (m, the carrier's measured
    value), ``turns_past`` (turns beyond the seat by the coupling law, 0 before it or with no seat) and ``seated``
    (travel within SEAT_TOL of the seat). Score tightness by turns_past, never by a force."""

    turns: float
    travel: float
    turns_past: float
    seated: bool


def _threads(spec: Mech) -> list[tuple[Joint, Joint]]:
    """(threaded hinge, its carrier slide) for every screw of the Mech."""
    by = {j.name: j for j in spec.joints}
    return [(j, by[j.parent]) for j in spec.joints if j.thread is not None]


def _check_threads(spec: Mech) -> None:
    """A threaded hinge rides on a coaxial carrier slide that has a spring (the thread stiffness), no hold, no detents
    and no other rider; its seat lies inside the carrier's stops. Only a carrier may have no shapes."""
    carriers = set()
    for nut, c in _threads(spec):
        if c.kind != "slide":
            raise ValueError(f"joint {nut.name}: a threaded hinge rides on a slide (its carrier), not on {c.kind} "
                             f"{c.name}")
        if c.axis != nut.axis:
            raise ValueError(f"joint {nut.name}: the carrier {c.name} slides along {c.axis}, the thread turns about "
                             f"{nut.axis}; they must share the axis")
        e = np.array(AXES[nut.axis])
        at = np.array(nut.at, dtype=float)
        if np.abs(at - (at @ e) * e).max() > 1e-12:
            raise ValueError(f"joint {nut.name}: a threaded hinge sits on its carrier's axis; at {nut.at} is off it")
        if c.spring <= 0:
            raise ValueError(f"joint {nut.name}: the carrier {c.name} needs spring > 0, the thread stiffness its "
                             "position drive follows")
        if c.hold > 0 or c.detents or c.thread is not None:
            raise ValueError(f"joint {nut.name}: the carrier {c.name} takes no hold or detents")
        riders = [k.name for k in spec.joints if k.parent == c.name and k is not nut]
        if riders:
            raise ValueError(f"joint {nut.name}: the carrier {c.name} carries only its threaded hinge, not {riders}")
        if c.name in carriers:
            raise ValueError(f"joint {c.name}: carries two threads")
        carriers.add(c.name)
        t = nut.thread
        if t.seat is not None and not c.low <= t.seat <= c.high:
            raise ValueError(f"joint {nut.name}: seat {t.seat} lies outside the carrier stops {c.low}..{c.high}")
        if t.max_hold is not None and t.max_hold < nut.hold:
            raise ValueError(f"joint {nut.name}: max_hold {t.max_hold} is under the hold {nut.hold}")
        carried = c.mass + nut.mass
        sag = carried * G / c.spring
        if sag > SCREW_SAG_MAX:
            raise ValueError(f"joint {nut.name}: the thread gives {sag * 1000:.2f} mm under its own {carried:.3g} kg; "
                             f"raise the carrier spring to at least {carried * G / SCREW_SAG_MAX:.4g} N/m")
    for j in spec.joints:
        if not j.shapes and j.name not in carriers:
            raise ValueError(f"joint {j.name}: needs at least one shape (only a thread carrier may have none)")


def screw_unwrap(theta: float, last: float, q: float) -> tuple[float, float]:
    """(new unwrapped angle, new last reading) after the joint reads ``q`` (rad, possibly wrapped): theta moves by the
    shortest turn from ``last`` to ``q``. Correct while the joint turns under half a turn between readings."""
    return float(theta) + math.remainder(float(q) - float(last), 2.0 * math.pi), float(q)


def _screw_law(t: Thread, carrier: Joint, theta: float) -> tuple[float, float]:
    free = t.lead * float(theta) / (2.0 * math.pi)
    lo, hi, past = carrier.low, carrier.high, 0.0
    if t.seat is not None:
        if t.seat < 0:
            lo, past = max(lo, t.seat), max(0.0, t.seat - free) / abs(t.lead)
        else:
            hi, past = min(hi, t.seat), max(0.0, free - t.seat) / abs(t.lead)
    return float(min(max(free, lo), hi)), float(past)


def screw_law(spec: Mech, name: str, angle: float) -> tuple[float, float]:
    """The coupling law of threaded hinge ``name`` at unwrapped turn ``angle`` (rad): (travel target in m, turns past
    the seat). Travel is lead x angle / 2 pi, held inside the carrier's stops and never past the seat. This is what
    the simulator writes before every physics step and what the replay follows."""
    for nut, c in _threads(spec):
        if nut.name == name:
            return _screw_law(nut.thread, c, angle)
    raise ValueError(f"mech {spec.name}: no threaded hinge {name!r} (threads: {[n.name for n, _ in _threads(spec)]})")


def screw_hold(nut: Joint, turns_past: float) -> float:
    """The threaded hinge's hold (N m) at ``turns_past`` turns beyond its seat: its own hold plus tighten per turn,
    capped at max_hold."""
    if nut.thread is None:
        raise ValueError(f"joint {nut.name}: has no thread")
    h = nut.hold + nut.thread.tighten * max(0.0, float(turns_past))
    return float(h if nut.thread.max_hold is None else min(h, nut.thread.max_hold))


def _screw_state(nut: Joint, carrier: Joint, values) -> ScrewState:
    theta, travel = float(values[nut.name]), float(values[carrier.name])
    _, past = _screw_law(nut.thread, carrier, theta)
    seat = nut.thread.seat
    return ScrewState(turns=theta / (2.0 * math.pi), travel=travel, turns_past=past,
                      seated=seat is not None and abs(travel - seat) <= SEAT_TOL)


def screw_states(spec: Mech, state) -> dict[str, ScrewState]:
    """A ScrewState per threaded hinge of ``spec`` from a state's joint values (a MechState from score(), the
    simulator, the replay or an offline mock). Raises when a value is missing rather than assume rest."""
    out = {}
    for nut, c in _threads(spec):
        missing = [n for n in (nut.name, c.name) if n not in state.joints]
        if missing:
            raise ValueError(f"screw_states: mech {spec.name}: the state has no value for joints {missing}")
        out[nut.name] = _screw_state(nut, c, state.joints)
    return out


def screw(name: str, shapes, lead: float, axis: str = "z", at: Vec3 = (0.0, 0.0, 0.0), low: float = -0.01,
          high: float = 0.0, seat: float | None = None, tighten: float = 0.0, max_hold: float | None = None,
          hold: float = 0.0, past_turns: float = 1.0, mass: float = 0.02, stiffness: float = SCREW_STIFFNESS,
          damping: float | None = None, turn_damping: float = 0.01, friction: float | None = None,
          parent: str | None = None) -> tuple[Joint, Joint]:
    """A screw: (carrier slide ``name + "_travel"``, threaded hinge ``name`` carrying ``shapes``). Put both in the
    Mech's joints. ``at`` is the screw's origin (in ``parent``'s frame when given); ``low`` .. ``high`` are its travel
    stops (m, the end of the thread); the hinge's turn range covers them plus ``past_turns`` beyond the seat (it is
    not authored as a limit). ``hold`` (N m) is the running friction of the turn, ``tighten`` and ``max_hold`` its
    growth past the seat (Thread). ``stiffness`` (N/m) is the thread's axial stiffness; ``damping`` defaults to
    critical for the carried mass."""
    t = Thread(lead, seat, tighten, max_hold)
    _positive("screw", stiffness=stiffness, mass=mass, past_turns=past_turns)
    a0, a1 = sorted((2.0 * math.pi * low / lead, 2.0 * math.pi * high / lead))
    if seat is not None:
        s = 2.0 * math.pi * seat / lead
        a0, a1 = (min(a0, s - 2.0 * math.pi * past_turns), a1) if s < 0 else (a0, max(a1, s + 2.0 * math.pi * past_turns))
    carried = SCREW_CARRIER_MASS + float(mass)
    c_damp = 2.0 * math.sqrt(float(stiffness) * carried) if damping is None else float(damping)
    carrier = Joint(f"{name}_travel", "slide", (), axis=axis, at=tuple(map(float, at)), low=float(low),
                    high=float(high), spring=float(stiffness), damping=c_damp, mass=SCREW_CARRIER_MASS, parent=parent)
    nut = Joint(name, "hinge", tuple(shapes), axis=axis, low=float(a0), high=float(a1), damping=float(turn_damping),
                mass=float(mass), friction=friction, hold=float(hold), parent=carrier.name, thread=t)
    return carrier, nut


def seat_travel(spec: Mech, name: str, shape: Shape, pose=None, clearance: float = 0.0) -> float:
    """The carrier travel (m) at which the colliding shapes of threaded hinge ``name`` first come within ``clearance``
    of ``shape`` (at ``pose`` in the Mech frame, None for the Mech frame), turning from rest toward the nearer end of
    the carrier's travel that it meets. Steps by the gap itself, which never exceeds the true distance, so it never
    passes the contact. Build the Mech without a seat, find it, then rebuild with Thread(seat=...)."""
    pairs = [(n, c) for n, c in _threads(spec) if n.name == name]
    if not pairs:
        raise ValueError(f"seat_travel: mech {spec.name}: no threaded hinge {name!r}")
    nut, c = pairs[0]
    own = [s for s in nut.shapes if s.collide]
    if not own:
        raise ValueError(f"seat_travel: joint {name} has no colliding shape")
    clearance = _finite("seat_travel: clearance", clearance)
    found = []
    for end in (c.low, c.high):
        if end == 0.0:
            continue
        step = 1.0 if end > 0 else -1.0
        travel = 0.0
        for _ in range(10000):
            angle = 2.0 * math.pi * travel / nut.thread.lead
            pose_n = spec.joint_pose(name, {c.name: travel, name: angle})
            g = min(gap(s, pose_n, shape, pose) for s in own) - clearance
            if g <= 1e-7:
                found.append(travel)
                break
            travel += step * max(g, 1e-7)
            if abs(travel) > abs(end):
                break
    if not found:
        raise ValueError(f"seat_travel: joint {name} never comes within {clearance} m of the shape inside its stops "
                         f"{c.low}..{c.high}")
    return float(min(found, key=abs))


def screw_report(spec: Mech) -> list[str]:
    """One line per screw for check.py: lead and hand, travel stops in turns, the seat, the hold law and the sag."""
    out = []
    for nut, c in _threads(spec):
        t = nut.thread
        hand = "right" if t.lead > 0 else "left"
        turns = [x / t.lead for x in (c.low, c.high)]
        line = (f"{nut.name}: {hand}-hand thread, lead {abs(t.lead) * 1000:.3g} mm, travel {c.low * 1000:.4g}.."
                f"{c.high * 1000:.4g} mm ({min(turns):.3g}..{max(turns):.3g} turns), stiffness {c.spring:.4g} N/m, "
                f"sag {(c.mass + nut.mass) * G / c.spring * 1000:.3g} mm, hold {nut.hold:.4g} N m")
        if t.seat is not None:
            line += f", seat at {t.seat * 1000:.4g} mm ({t.seat / t.lead:.3g} turns)"
            if t.tighten > 0:
                cap = "" if t.max_hold is None else f", at most {t.max_hold:.4g}"
                line += f", +{t.tighten:.4g} N m per turn past it{cap}"
        out.append(line)
    return out


# -- gears: one joint driven by another at a ratio, pure NumPy --------------------------------------------------------
#
# The coupling is a spring and damper between the follower and ratio x driver (+ offset), split into two implicit
# position drives: the follower's drive pulls it toward the law, the driver's drive (stiffness x ratio^2) pulls it
# toward where the follower says it should be, so each feels the other's reaction (virtual work: driver effort =
# -ratio x follower effort). Both are written before every physics step from the joints' readings at that step, the
# hook the screw threads use. Each drive is implicit in its own joint and explicit in the other, so a stiff gear between
# two light joints can ring; Mech refuses a gear whose discrete coupling (modelled below the way the PhysX drives step)
# is not stable, and ``stiffness=None`` picks half the stiffest stable value.

GEAR_RATIO = {("hinge", "hinge"): (1e-3, 1e3), ("slide", "slide"): (1e-2, 1e2), ("hinge", "slide"): (1e-4, 0.5),
              ("slide", "hinge"): (2.0, 1e4)}  # (driver kind, follower kind): allowed |ratio|
GEAR_STIFFNESS_CAP = 1.0e5  # N m/rad or N/m: the stiffest gear the kit will pick or accept
GEAR_RHO = 0.999  # an oscillating mode of the discrete coupling must shrink at least this much per physics step


class GearTargets(NamedTuple):
    """One physics step of the gear law: each joint's position and velocity drive target, whether the teeth are
    engaged (outside the backlash) and the engaged strain (follower units) the drives act on."""

    follower_pos: float
    follower_vel: float
    driver_pos: float
    driver_vel: float
    engaged: bool
    strain: float


class _GearLink(NamedTuple):
    follower: Joint
    driver: Joint
    ratio: float
    offset: float
    backlash: float
    stiffness: float
    damping: float


def gear_law(ratio: float, offset: float, backlash: float, q_driver: float, v_driver: float, q_follower: float,
             v_follower: float, dt: float = 1.0 / 240.0) -> GearTargets:
    """Drive targets for one physics step of length ``dt`` from the joints' positions and velocities at its start.
    Each joint is predicted to the end of the step (q + v dt); the strain is ratio x driver + offset - follower, less
    half the backlash (scaled to the follower) on the side the teeth touch. Inside the backlash nothing is engaged: each
    target is the joint's own prediction, so the drives push nothing. Engaged, the follower's target is its prediction
    plus the strain and the driver's its prediction minus strain / ratio, with velocity targets ratio x driver speed
    and follower speed / ratio, so the follower's drive gives stiffness x strain and the driver's -ratio times that."""
    r = float(ratio)
    qd, qf = float(q_driver) + float(v_driver) * dt, float(q_follower) + float(v_follower) * dt
    s = r * qd + float(offset) - qf
    h = abs(r) * float(backlash) / 2.0
    if h > 0.0 and abs(s) <= h:
        return GearTargets(qf, float(v_follower), qd, float(v_driver), False, 0.0)
    strain = s - math.copysign(h, s)
    return GearTargets(qf + strain, r * float(v_driver), qd - strain / r, float(v_follower) / r, True, strain)


def _gear_inertia(j: Joint) -> float:
    """Inertia the joint's drive moves (kg or kg m^2), its armature included; its riders are left out (lighter is
    the harder case for the coupling's stability)."""
    return _joint_inertia(j) + float(j.armature)


def _gear_drive(links, own: dict, q: dict, v: dict, rest: dict, dt: float) -> dict[str, tuple[float, float, float,
                                                                                                    float]]:
    """{joint: (stiffness, position target, damping, velocity target)} of every geared joint's drive: its own spring
    toward its rest and damping toward 0 (``own`` = {joint: (spring, damping)}), plus every gear it is in, combined
    into one drive (stiffness-weighted target, damping-weighted velocity). This is what the simulator writes."""
    acc = {n: [float(k), float(k) * float(rest.get(n, 0.0)), float(c), 0.0] for n, (k, c) in own.items()}
    for g in links:
        f, d = g.follower.name, g.driver.name
        t = gear_law(g.ratio, g.offset, g.backlash, q[d], v[d], q[f], v[f], dt)
        r2 = g.ratio * g.ratio
        for n, k, c, pos, vel in ((f, g.stiffness, g.damping, t.follower_pos, t.follower_vel),
                                  (d, g.stiffness * r2, g.damping * r2, t.driver_pos, t.driver_vel)):
            a = acc.setdefault(n, [0.0, 0.0, 0.0, 0.0])
            a[0] += k
            a[1] += k * pos
            a[2] += c
            a[3] += c * vel
    return {n: (k, kt / k if k > 0 else 0.0, c, cv / c if c > 0 else 0.0) for n, (k, kt, c, cv) in acc.items()}


def _drive_step(drive: dict, inertia: dict, q: dict, v: dict, dt: float) -> tuple[dict, dict]:
    """One implicit step of each joint under its own drive (as a PhysX position drive steps): v1 = (I v0 + dt (K (T -
    q0) + C V)) / (I + dt C + dt^2 K), q1 = q0 + v1 dt."""
    q1, v1 = {}, {}
    for n, (k, t, c, vel) in drive.items():
        i = inertia[n]
        v1[n] = (i * v[n] + dt * (k * (t - q[n]) + c * vel)) / (i + dt * c + dt * dt * k)
        q1[n] = q[n] + v1[n] * dt
    return q1, v1


def _gear_matrix(links, own: dict, inertia: dict, dt: float) -> np.ndarray:
    """The linear map one physics step applies to (positions, velocities) of the geared joints, teeth engaged (no
    backlash, offsets and rests at 0)."""
    names = sorted(own)
    plain = [g._replace(offset=0.0, backlash=0.0) for g in links]
    n = len(names)
    m = np.zeros((2 * n, 2 * n))
    for col in range(2 * n):
        x = np.zeros(2 * n)
        x[col] = 1.0
        q, v = dict(zip(names, x[:n])), dict(zip(names, x[n:]))
        q1, v1 = _drive_step(_gear_drive(plain, own, q, v, {}, dt), inertia, q, v, dt)
        m[:, col] = [q1[k] for k in names] + [v1[k] for k in names]
    return m


def _gear_stable(m: np.ndarray) -> bool:
    """No mode grows, and every fast mode (turning more than a quarter cycle a step, the step-to-step ringing of two
    drives explicit in each other) shrinks by GEAR_RHO a step at least. A mode at +1 is the gear train turning freely,
    and a slow lightly damped mode is a physical oscillation; both are fine."""
    for lam in np.linalg.eigvals(m):
        a = abs(lam)
        if a > 1.0 + 1e-9:
            return False
        if a > GEAR_RHO and abs(np.angle(lam)) > math.pi / 2:
            return False
    return True


def _pair(follower: Joint, driver: Joint, g: Gear, k: float) -> _GearLink:
    """The link at stiffness k, damping as given or critical for the pair's reduced inertia (follower units)."""
    i_f, i_d = _gear_inertia(follower), _gear_inertia(driver) * g.ratio ** 2
    c = g.damping if g.damping is not None else 2.0 * math.sqrt(k * i_f * i_d / (i_f + i_d))
    return _GearLink(follower, driver, g.ratio, g.offset, g.backlash, float(k), float(c))


def _system_stable(links, joints: dict[str, Joint], dt: float = 1.0 / 240.0) -> bool:
    names = {n for g in links for n in (g.follower.name, g.driver.name)}
    own = {n: (joints[n].spring, joints[n].damping) for n in names}
    inertia = {n: _gear_inertia(joints[n]) for n in names}
    return _gear_stable(_gear_matrix(links, own, inertia, dt))


def max_gear_stiffness(follower: Joint, driver: Joint, ratio: float, damping: float | None = None,
                       dt: float = 1.0 / 240.0) -> float:
    """The stiffest gear (follower units, N m/rad or N/m) between these two joints that the discrete coupling stays
    stable at, up to GEAR_STIFFNESS_CAP; 0 when none is. Give the driver armature (a gear train's own inertia) or
    lighten the stiffness to stay under it."""
    g = Gear(driver.name, ratio, damping=damping)
    joints = {follower.name: follower, driver.name: driver}

    def ok(k: float) -> bool:
        return _system_stable([_pair(follower, driver, g, k)], joints, dt)

    lo, hi = 1e-7, GEAR_STIFFNESS_CAP
    if ok(hi):
        return hi
    if not ok(lo):
        return 0.0
    for _ in range(60):
        mid = math.sqrt(lo * hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return float(lo)


def _gear_rest(spec: Mech, links) -> dict[str, float]:
    """Each geared joint's value at rest by the law: drivers that follow nothing at 0, every follower at ratio x its
    driver + offset (through a gear train)."""
    by = {g.follower.name: g for g in links}
    out: dict[str, float] = {}

    def value(name: str) -> float:
        if name not in out:
            g = by.get(name)
            out[name] = 0.0 if g is None else g.ratio * value(g.driver.name) + g.offset
        return out[name]

    for g in links:
        value(g.follower.name)
        value(g.driver.name)
    return out


def _check_gears(spec: Mech) -> None:
    """Refuse unknown drivers, gear cycles, threads and carriers in a gear, ratios out of range, hinges past a turn
    either way, rest values outside the stops, backlash wider than half the driver's travel, and couplings the
    physics step would not keep stable; then resolve default stiffness and damping."""
    by = {j.name: j for j in spec.joints}
    carriers = {c.name for _, c in _threads(spec)}
    links = []
    for f in spec.joints:
        g = f.follows
        if g is None:
            continue
        if g.driver not in by:
            raise ValueError(f"mech {spec.name}: joint {f.name} follows {g.driver!r}, which is not a joint of the Mech "
                             f"(joints: {sorted(by)})")
        d = by[g.driver]
        for j in (f, d):
            if j.thread is not None or j.name in carriers:
                raise ValueError(f"mech {spec.name}: joint {j.name} is part of a screw thread and cannot be in a gear")
            if j.kind == "hinge" and (j.low < -2.0 * math.pi or j.high > 2.0 * math.pi):
                raise ValueError(f"mech {spec.name}: geared hinge {j.name} has stops {j.low:.4g}..{j.high:.4g} rad; "
                                 "a geared hinge stays within one turn either way of rest")
        seen, cur = {f.name}, d
        while True:
            if cur.name in seen:
                raise ValueError(f"mech {spec.name}: gears form a cycle through {cur.name!r}")
            seen.add(cur.name)
            if cur.follows is None:
                break
            cur = by[cur.follows.driver] if cur.follows.driver in by else None
            if cur is None:
                break
        lo, hi = GEAR_RATIO[(d.kind, f.kind)]
        if not lo <= abs(g.ratio) <= hi:
            unit = {"hinge": "rad", "slide": "m"}
            raise ValueError(f"mech {spec.name}: joint {f.name}: |ratio| {abs(g.ratio):.4g} {unit[f.kind]} per "
                             f"{unit[d.kind]} of {d.name} is outside {lo:g}..{hi:g} for a {d.kind} driving a {f.kind}")
        if g.backlash >= (d.high - d.low) / 2.0:
            raise ValueError(f"mech {spec.name}: joint {f.name}: backlash {g.backlash:.4g} is at least half the "
                             f"travel of {d.name} ({d.low:.4g}..{d.high:.4g})")
        k = g.stiffness
        if k is None:
            kmax = max_gear_stiffness(f, d, g.ratio, g.damping)
            if kmax <= 0.0:
                raise ValueError(f"mech {spec.name}: joint {f.name}: no stable gear stiffness to {d.name}; give the "
                                 "driver armature or more mass")
            k = kmax if kmax >= GEAR_STIFFNESS_CAP else 0.5 * kmax
        elif k > GEAR_STIFFNESS_CAP:
            raise ValueError(f"mech {spec.name}: joint {f.name}: gear stiffness {k:.4g} is over the cap "
                             f"{GEAR_STIFFNESS_CAP:g}")
        link = _pair(f, d, g, k)
        if not _system_stable([link], {f.name: f, d.name: d}):
            kmax = max_gear_stiffness(f, d, g.ratio, g.damping)
            raise ValueError(f"mech {spec.name}: joint {f.name}: a gear of stiffness {k:.4g} to {d.name} rings at "
                             f"240 Hz (the drives are explicit in each other); the stiffest stable is about "
                             f"{kmax:.4g}: lower stiffness, or give the lighter joint armature")
        links.append(link)
    if not links:
        object.__setattr__(spec, "_gears", ())
        return
    for name, value in _gear_rest(spec, links).items():
        j = by[name]
        if not j.low - 1e-12 <= value <= j.high + 1e-12:
            raise ValueError(f"mech {spec.name}: joint {name} rests at {value:.4g} by its gear law, outside its stops "
                             f"{j.low:.4g}..{j.high:.4g}; check the offset")
    if not _system_stable(links, by):
        raise ValueError(f"mech {spec.name}: its gears are stable one by one but not together (a joint in two "
                         "gears); lower their stiffness or give the shared joint armature")
    object.__setattr__(spec, "_gears", tuple(links))


def _gear_links(spec) -> tuple:
    return getattr(spec, "_gears", ())


def gear_gains(spec: Mech) -> dict[str, tuple[float, float]]:
    """{joint: (extra stiffness, extra damping)} every gear adds to its joints' drives: stiffness and damping on a
    follower, stiffness x ratio^2 and damping x ratio^2 on a driver. The simulator authors spring + extra."""
    out: dict[str, list[float]] = {}
    for g in _gear_links(spec):
        r2 = g.ratio * g.ratio
        for n, k, c in ((g.follower.name, g.stiffness, g.damping), (g.driver.name, g.stiffness * r2, g.damping * r2)):
            a = out.setdefault(n, [0.0, 0.0])
            a[0] += k
            a[1] += c
    return {n: (float(k), float(c)) for n, (k, c) in out.items()}


def gear_target(spec: Mech, follower: str, driver_value: float) -> float:
    """Where the law puts follower ``follower`` for a driver value (ratio x value + offset), ignoring compliance and
    backlash. For check.py and score() margins."""
    for g in _gear_links(spec):
        if g.follower.name == follower:
            return float(g.ratio * float(driver_value) + g.offset)
    raise ValueError(f"mech {spec.name}: no geared follower {follower!r}")


def gear_slip(spec: Mech, state) -> dict[str, float]:
    """{follower: its value minus where the law puts it} from a state's joint values: the gear's compliance under
    load plus any backlash taken up (follower units). Raises when a value is missing rather than assume rest."""
    out = {}
    for g in _gear_links(spec):
        missing = [n for n in (g.follower.name, g.driver.name) if n not in state.joints]
        if missing:
            raise ValueError(f"gear_slip: mech {spec.name}: the state has no value for joints {missing}")
        out[g.follower.name] = float(state.joints[g.follower.name]
                                     - (g.ratio * state.joints[g.driver.name] + g.offset))
    return out


def gear_report(spec: Mech) -> list[str]:
    """One line per gear for check.py: the law, stiffness and damping, what the driver feels, how far gravity sags the
    follower against the gear, and how far the driver winds up per unit of effort when the follower is blocked."""
    unit = {"hinge": ("rad", "N m"), "slide": ("m", "N")}
    out = []
    for g in _gear_links(spec):
        f, d = g.follower, g.driver
        (fu, fe), (du, de) = unit[f.kind], unit[d.kind]
        kd = g.stiffness * g.ratio ** 2
        line = (f"{f.name}: follows {d.name} x {g.ratio:.4g} {fu}/{du} + {g.offset:.4g} {fu}, stiffness "
                f"{g.stiffness:.4g} {fe}/{fu}, damping {g.damping:.4g}, driver feels {kd:.4g} {de}/{du}, "
                f"gravity sags the follower {gravity_load(f) / g.stiffness:.3g} {fu}, a blocked follower winds the "
                f"driver {1.0 / kd:.3g} {du} per {de}")
        if g.backlash > 0:
            line += f", backlash {g.backlash:.4g} {du} at the driver"
        if f.damping > 0:  # Joint's default damping (5.0) is heavy for a light follower: the driver feels it
            line += (f", the follower's own damping takes {f.damping * abs(g.ratio) ** 2:.3g} {de} at the driver "
                     f"per {du}/s (lower Joint damping on a light follower)")
        out.append(line)
    return out


# -- contact: has a finger pad touched an object, pure part ------------------------------------------------------------
#
# In the simulator the YAM arm USD carries PhysX contact reporting on every link (yam_arena's arm config activates
# contact sensors), but the inspect embodiment builds no ContactSensor, so Isaac Lab leaves PhysX contact processing
# off. A task that declares a watch turns it back on (the carb setting /physics/disableContactProcessing, as Isaac
# Lab's own ContactSensor does) and the kit reads PhysX rigid contact views built on the finger links: per watch, the
# normal force between each pad link and the target's rigid bodies (get_contact_force_matrix); per arm, the net
# normal force on each finger link from anything (get_net_contact_forces). Both are read before every physics step
# (the previous step's contacts) and once more after each control step, the way accepted cable and cutting kits read
# them. A target with no rigid body (a fixed Box, a jointless fixed Mech) cannot be a contact-view filter; its watch
# falls back to a pad-to-bounding-box distance under PROXIMITY_TOL, measured each control step, and says so.

FINGER_LINKS = ("tip_left", "tip_right")  # the YAM finger links; "gripper" (the palm body) may be watched too
TOUCH_LINKS = (*FINGER_LINKS, "gripper")
ARM_ASSETS = {"left": "left_robot", "right": "right_robot"}  # yam_arena scene keys
ARM_PRIMS = {"left": "LeftArm", "right": "RightArm"}  # yam_arena prim names under each env
TOUCH_MIN_FORCE = 0.05  # N: a pad touches when its normal force reaches this (yam_arena's stage scripts use 0.05 N)
PROXIMITY_TOL = 0.002  # m: fallback touch distance from a pad face point to the target's bounding box


@dataclass(frozen=True)
class Touch:
    """A contact watch (``watch_touch(...)``): whether the finger pads ``links`` of ``arms`` touch object ``target``
    (its rigid bodies, or only ``bodies``, which for a Mech are "base" and joint names), with a normal force of at
    least ``min_force`` N. ``name`` (default the target) is how score() asks for it."""

    target: str
    bodies: tuple[str, ...] = ()
    arms: tuple[str, ...] = ("left", "right")
    links: tuple[str, ...] = FINGER_LINKS
    min_force: float = TOUCH_MIN_FORCE
    name: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.target, str) or not self.target:
            raise ValueError(f"touch: target must be an object name, got {self.target!r}")
        for field_name, allowed in (("arms", tuple(ARM_ASSETS)), ("links", TOUCH_LINKS)):
            vals = getattr(self, field_name)
            vals = (vals,) if isinstance(vals, str) else tuple(vals)
            if not vals or len(set(vals)) != len(vals) or any(v not in allowed for v in vals):
                raise ValueError(f"touch {self.target}: {field_name} must be distinct names from {allowed}, "
                                 f"got {vals!r}")
            object.__setattr__(self, field_name, vals)
        bodies = (self.bodies,) if isinstance(self.bodies, str) else tuple(self.bodies)
        if any(not isinstance(b, str) or not b or "/" in b for b in bodies) or len(set(bodies)) != len(bodies):
            raise ValueError(f"touch {self.target}: bodies must be distinct body names, got {self.bodies!r}")
        object.__setattr__(self, "bodies", bodies)
        f = _finite(f"touch {self.target}: min_force", self.min_force)
        if f <= 0:
            raise ValueError(f"touch {self.target}: min_force must be > 0 N, got {f}")
        object.__setattr__(self, "min_force", f)
        object.__setattr__(self, "name", self.name or self.target)

    @property
    def pads(self) -> tuple[str, ...]:
        """The watched pads as "arm/link" keys, arm by arm."""
        return tuple(f"{a}/{k}" for a in self.arms for k in self.links)


@dataclass(frozen=True)
class TouchState:
    """What a watch saw since the last reset. ``touched``: some pad reached min_force (or, in the proximity
    fallback, came within PROXIMITY_TOL) at any physics step; ``touching``: one does now (the latest reading);
    ``peak`` and ``now``: largest normal force (N) over the episode and at the latest reading, over the pads asked
    for; ``pads`` and ``pads_now``: the same per "arm/link"; ``first_step``: the control step of the first touch;
    ``source``: "physx" (measured contact forces), "proximity" (distance fallback: forces are nan), "offline" (no
    simulator: the scripted mock, untouched unless mock_touch set it)."""

    name: str
    touched: bool
    touching: bool
    peak: float
    now: float
    first_step: int | None
    pads: dict[str, float]
    pads_now: dict[str, float]
    source: str


_TOUCHES: dict[str, Touch] = {}  # watch name -> Touch, registered by watch_touch at task import
_FINGER_SENSE: dict[str, bool] = {"on": False}  # watch_fingers() (or any watch) asks for the net finger force
_TOUCH_MOCK: dict[str, dict] = {}  # watch name -> scripted accumulator, offline only
_FINGER_MOCK: dict[str, float] = {}  # arm -> scripted net finger force, offline only


def watch_touch(target: str, bodies=(), arms=("left", "right"), links=FINGER_LINKS, min_force: float = TOUCH_MIN_FORCE,
                name: str | None = None) -> Touch:
    """Declare, at task import, a contact watch between finger pads and object ``target`` (see Touch); score() then
    reads ``touch_state(name)``. Watching also turns on the net finger force (``finger_force``). Declaring the same
    watch twice is harmless; a different watch under a used name raises."""
    t = Touch(target, bodies, arms, links, min_force, name or "")
    old = _TOUCHES.get(t.name)
    if old is not None and old != t:
        raise ValueError(f"watch_touch: a different watch is already named {t.name!r}: {old}")
    _TOUCHES[t.name] = t
    _FINGER_SENSE["on"] = True
    return t


def watch_fingers() -> None:
    """Declare, at task import, that the scorer or the closed-loop demo reads the net normal force on each arm's
    finger links (``finger_force``): the robot's own fingertip contact sense, with no object named."""
    _FINGER_SENSE["on"] = True


def contacts_wanted() -> bool:
    """True when the task declared a watch or the finger sense, so the simulator must process contacts."""
    return bool(_TOUCHES) or _FINGER_SENSE["on"]


def _new_touch_acc(t: Touch) -> dict:
    nan = float("nan")
    return {"peak": {p: 0.0 for p in t.pads}, "now": {p: 0.0 for p in t.pads}, "ever": {p: False for p in t.pads},
            "hit": {p: False for p in t.pads}, "first": None, "source": "offline", "nan": nan}


def _touch_update(acc: dict, t: Touch, step: int, forces: dict | None = None, hits: dict | None = None) -> None:
    """Fold one reading into a watch's accumulator: ``forces`` {pad: normal force N} (measured; a pad touches at
    min_force), or ``hits`` {pad: bool} (the proximity fallback, forces unknown: nan)."""
    for p in t.pads:
        if forces is not None:
            f = float(forces.get(p, 0.0))
            hit = f >= t.min_force
            acc["now"][p] = f
            acc["peak"][p] = max(acc["peak"][p], f) if not math.isnan(acc["peak"][p]) else f
        else:
            hit = bool((hits or {}).get(p, False))
            acc["now"][p] = acc["peak"][p] = float("nan")
        acc["hit"][p] = hit
        if hit:
            acc["ever"][p] = True
            if acc["first"] is None:
                acc["first"] = int(step)


def _touch_view(t: Touch, acc: dict, arm: str | None) -> TouchState:
    if arm is not None and arm not in t.arms:
        raise ValueError(f"touch_state: watch {t.name} does not watch the {arm} arm (arms: {t.arms})")
    pads = [p for p in t.pads if arm is None or p.startswith(arm + "/")]

    def top(vals) -> float:
        vals = [v for v in vals if not math.isnan(v)]
        return float(max(vals)) if vals else float("nan")

    ever = any(acc["ever"][p] for p in pads)
    return TouchState(name=t.name, touched=ever, touching=any(acc["hit"][p] for p in pads),
                      peak=top(acc["peak"][p] for p in pads), now=top(acc["now"][p] for p in pads),
                      first_step=acc["first"] if ever else None, pads={p: float(acc["peak"][p]) for p in pads},
                      pads_now={p: float(acc["now"][p]) for p in pads}, source=acc["source"])


def mock_touch(name: str, peak: float = 0.0, now: float = 0.0, arm: str = "right", link: str = "tip_left",
               step: int = 1) -> TouchState:
    """Script an offline answer for watch ``name`` (for check.py scorer cases): pad ``arm``/``link`` reached ``peak``
    N during the episode and reads ``now`` N at the end. Calls add up pad by pad; clear_contact_mocks() forgets them.
    Ignored while the simulator runs."""
    t = _touch(name)
    acc = _TOUCH_MOCK.setdefault(name, _new_touch_acc(t))
    pad = f"{arm}/{link}"
    if pad not in t.pads:
        raise ValueError(f"mock_touch: watch {name} has no pad {pad!r} (pads: {t.pads})")
    for f in (float(peak), float(now)):
        _touch_update(acc, t, step, forces={pad: f})
    acc["peak"][pad] = max(float(peak), float(now))
    return _touch_view(t, acc, None)


def mock_finger_force(arm: str, force: float) -> None:
    """Script the offline net finger force (N) of ``arm`` for scorer tests. Ignored while the simulator runs."""
    if arm not in ARM_ASSETS:
        raise ValueError(f"mock_finger_force: arm must be one of {tuple(ARM_ASSETS)}, got {arm!r}")
    _FINGER_MOCK[arm] = _finite("mock_finger_force: force", force)


def clear_contact_mocks() -> None:
    _TOUCH_MOCK.clear()
    _FINGER_MOCK.clear()


def _touch(name: str) -> Touch:
    if name not in _TOUCHES:
        raise ValueError(f"no contact watch {name!r}; declare it at import with watch_touch(...) (watches: "
                         f"{sorted(_TOUCHES)})")
    return _TOUCHES[name]


def touch_report() -> list[str]:
    """One line per watch for check.py: target, bodies, pads and threshold."""
    return [f"{t.name}: {', '.join(t.pads)} on {t.target}"
            + (f" bodies {', '.join(t.bodies)}" if t.bodies else " (every rigid body)")
            + f", touch at {t.min_force:g} N" for t in _TOUCHES.values()]


def _match_bodies(paths, target: str, bodies=()) -> list[str]:
    """Rigid-body prim paths of env 0 that belong to object ``target`` (a path segment equal to it), and, when
    ``bodies`` is given, whose last segment is one of them."""
    out = []
    for p in paths:
        segs = p.split("/")
        if target in segs[:-1] or segs[-1] == target:
            if not bodies or segs[-1] in bodies:
                out.append(p)
    return out


def _pad_paths(link_paths: dict[str, list[str]], t_arms, links) -> dict[str, str]:
    """{"arm/link": prim path} for each watched pad from each arm's link prim paths. Raises when one is missing."""
    out = {}
    for arm in t_arms:
        for link in links:
            hits = [p for p in link_paths.get(arm, []) if p.rsplit("/", 1)[-1] == link]
            if len(hits) != 1:
                raise RuntimeError(f"contact: {len(hits)} prims named {link!r} on the {arm} arm")
            out[f"{arm}/{link}"] = hits[0]
    return out


def _any_numpy(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _pad_forces(matrix, pads, n_filters: int) -> dict[str, float]:
    """{pad: |normal force| from the target} from a contact view's force matrix (sensors x filters x 3): each pad's
    force vectors from all the target's bodies are summed, then its length taken."""
    fm = _any_numpy(matrix).reshape(len(pads), n_filters, 3)
    return {p: float(np.linalg.norm(fm[i].sum(axis=0))) for i, p in enumerate(pads)}


def _net_force(net) -> float:
    """The arm's finger contact sense from a net-force view (finger links x 3): the largest finger's force. A max, not
    a sum, so two pads squeezing a part do not cancel."""
    f = _any_numpy(net).reshape(-1, 3)
    return float(np.linalg.norm(f, axis=1).max(initial=0.0))


# -- indicator lamps, pure -----------------------------------------------------------------------------------------------

LAMP_OFF = (0.16, 0.16, 0.16)  # unlit lens colour
LAMP_GLOW = 2.0  # emissive gain of a lit lens over its colour (linear)
LAMP_PROUD = 0.0002  # m: a lit lens is this much bigger than the unlit one on every side, so it covers it


def lamp(name: str, at, radius: float, colors: dict, off=LAMP_OFF, height: float = 0.003, axis: str = "z",
         tilt=IDENT, square: bool = False) -> tuple[Shape, ...]:
    """Lens shapes of indicator lamp ``name``: an unlit lens (colour ``off``) and one lit lens per state in ``colors``
    ({state: rgb}), all look-only and centred at ``at``. A lit lens is LAMP_PROUD bigger all round, so the one shown
    covers the unlit lens; only one shows at a time (``set_lamp``) and all start hidden. Round (a cyl along ``axis``)
    or ``square`` (a box ``2 radius`` across, ``height`` along z, turned by ``tilt``). Spread them into a shapes list:
    ``shapes=[housing, *lamp("status", (0, 0, 0.031), 0.005, {"red": RED, "green": GREEN})]``."""
    if not isinstance(name, str) or not name:
        raise ValueError(f"lamp: name must be a non-empty string, got {name!r}")
    _positive(f"lamp {name}", radius=radius, height=height)
    if not isinstance(colors, dict) or not colors:
        raise ValueError(f"lamp {name}: colors must be a non-empty {{state: rgb}} dict")
    for state, rgb in colors.items():
        if not isinstance(state, str) or not state or state == "off":
            raise ValueError(f"lamp {name}: state names are non-empty strings other than 'off', got {state!r}")
        if len(tuple(rgb)) != 3 or not all(0.0 <= float(c) <= 1.0 for c in rgb):
            raise ValueError(f"lamp {name}: colour of {state!r} must be (r, g, b) in 0..1, got {rgb!r}")

    def lens(grow: float, color, lit: str) -> Shape:
        r, h = float(radius) + grow, float(height) + 2.0 * grow
        s = (box(at, (2 * r, 2 * r, h), color, tilt) if square else cyl(at, r, h, color, axis, tilt))
        return dataclasses.replace(s, collide=False, lamp=name, lit=lit)

    return (lens(0.0, tuple(map(float, off)), ""),
            *(lens(LAMP_PROUD, tuple(map(float, rgb)), state) for state, rgb in colors.items()))


def _lamp_bodies(spec: Mech) -> dict[str, tuple[str, list[Shape]]]:
    """lamp name -> (body, its lens shapes), body being "base" or a joint name."""
    out: dict[str, tuple[str, list[Shape]]] = {}
    for body, shapes in [("base", spec.shapes), *((j.name, j.shapes) for j in spec.joints)]:
        for s in shapes:
            if s.lamp:
                if s.lamp in out and out[s.lamp][0] != body:
                    raise ValueError(f"mech {spec.name}: lamp {s.lamp} has lenses on two bodies, {out[s.lamp][0]} "
                                     f"and {body}")
                out.setdefault(s.lamp, (body, []))[1].append(s)
    return out


def _check_lamps(spec: Mech) -> None:
    for name, (_, lenses) in _lamp_bodies(spec).items():
        if sum(1 for s in lenses if not s.lit) != 1:
            raise ValueError(f"mech {spec.name}: lamp {name} needs exactly one unlit lens (build it with lamp())")
        lit = [s.lit for s in lenses if s.lit]
        if not lit or len(set(lit)) != len(lit):
            raise ValueError(f"mech {spec.name}: lamp {name} needs lit states with distinct names, got {lit}")


def lamp_states(spec: Mech) -> dict[str, tuple[str, ...]]:
    """{lamp name: its lit state names} for every lamp of the Mech; a lamp is also always "off"."""
    return {name: tuple(s.lit for s in lenses if s.lit) for name, (_, lenses) in _lamp_bodies(spec).items()}


def timetable(t: float, schedule, repeat: bool = True, after: str = "off") -> str:
    """The state at time ``t`` (any unit, the same as the durations) of ``schedule`` = [(duration, state), ...],
    repeating when ``repeat``, else ``after`` once it has run out. A boundary belongs to the next entry."""
    t = _finite("timetable: t", t)
    if t < 0:
        raise ValueError(f"timetable: t must be >= 0, got {t}")
    rows = [(_finite("timetable: duration", d), s) for d, s in schedule]
    if not rows or any(d <= 0 for d, _ in rows) or any(not isinstance(s, str) or not s for _, s in rows):
        raise ValueError("timetable: schedule needs (duration > 0, state name) rows")
    total = sum(d for d, _ in rows)
    if t >= total:
        if not repeat:
            return after
        t = math.fmod(t, total)
    acc = 0.0
    for d, s in rows:
        acc += d
        if t < acc:
            return s
    return rows[-1][1]  # rounding at the very end of a period


def blink_code(count: int, lit: str, on: float = 0.3, off: float = 0.3, gap: float = 1.5) -> list[tuple[float, str]]:
    """A timetable schedule that blinks ``count`` pulses of state ``lit`` (``on`` lit, ``off`` dark each), then stays
    dark ``gap`` more before it repeats: a blink code. Seconds by default; any unit that matches ``timetable``'s t."""
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError(f"blink_code: count must be a whole number >= 1, got {count!r}")
    _positive("blink_code", on=on, off=off)
    if _finite("blink_code: gap", gap) < 0:
        raise ValueError(f"blink_code: gap must be >= 0, got {gap}")
    rows: list[tuple[float, str]] = []
    for _ in range(count):
        rows += [(float(on), lit), (float(off), "off")]
    if gap > 0:
        rows[-1] = (float(off) + float(gap), "off")
    return rows


# -- joint hold (Coulomb friction) and detents, pure NumPy -------------------------------------------------------------


def _finite(what: str, value) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{what} must be a number, got {value!r}") from None
    if not math.isfinite(v):
        raise ValueError(f"{what} must be finite, got {value!r}")
    return v


def _joint_com(j: Joint) -> np.ndarray:
    """Centre of the joint's colliding volume in its own frame (the centre of its bound points if nothing collides)."""
    try:
        return mass_props(j.shapes)[1]
    except ValueError:
        pts = np.vstack([shape_points(s) for s in j.shapes])
        return (pts.min(0) + pts.max(0)) / 2


def gravity_lever(j: Joint, samples: int = 721) -> float:
    """Worst horizontal lever arm (m) of a hinge's centre of mass about its axis over its whole travel; 0 for a z
    hinge (gravity cannot turn it) and for a slide."""
    if j.kind != "hinge" or j.axis == "z":
        return 0.0
    com = _joint_com(j)
    across = 1 if j.axis == "x" else 0  # gravity along -z turns an x hinge by r_y, a y hinge by r_x
    angles = np.linspace(j.low, j.high, samples)
    return float(max(abs((_axis_matrix(j.axis, float(a)) @ com)[across]) for a in angles))


def gravity_load(j: Joint) -> float:
    """What gravity asks the joint to hold: weight (N) for a z slide, worst torque (N m) for an x or y hinge, else 0."""
    if j.kind == "slide":
        return j.mass * G if j.axis == "z" else 0.0
    return j.mass * G * gravity_lever(j)


def _check_hold(j: Joint) -> None:
    hold = _finite(f"joint {j.name}: hold", j.hold)
    if hold < 0:
        raise ValueError(f"joint {j.name}: hold must be >= 0, got {hold}")
    if j.hold_moving is not None:
        moving = _finite(f"joint {j.name}: hold_moving", j.hold_moving)
        if not 0.0 <= moving <= hold:
            raise ValueError(f"joint {j.name}: hold_moving {moving} must be 0 to hold ({hold})")
    stops = tuple(_finite(f"joint {j.name}: detent", s) for s in j.detents)
    if stops:
        if j.spring <= 0:
            raise ValueError(f"joint {j.name}: detents need a spring > 0 to snap into them")
        if len(set(stops)) != len(stops):
            raise ValueError(f"joint {j.name}: detents repeat: {stops}")
        if not any(abs(s) <= 1e-12 for s in stops):
            raise ValueError(f"joint {j.name}: detents must include 0, the rest position")
        bad = [s for s in stops if not j.low <= s <= j.high]
        if bad:
            raise ValueError(f"joint {j.name}: detents {bad} lie outside the stops {j.low}..{j.high}")
        object.__setattr__(j, "detents", tuple(sorted(stops)))
    if hold > 0 and j.spring == 0:
        load = gravity_load(j)
        if hold < HOLD_MARGIN * load:
            unit = "N" if j.kind == "slide" else "N m"
            raise ValueError(f"joint {j.name}: would sag: hold {hold:.4g} {unit} is under {HOLD_MARGIN} x the "
                             f"{load:.4g} {unit} gravity puts on it; raise hold to at least {HOLD_MARGIN * load:.4g}, "
                             "add a spring, or turn the axis")


def deadband(j: Joint) -> float:
    """How far from its rest (m or rad) a sprung joint with a hold may stay: hold / spring; inf with no spring. Put it
    in score() margins. A spring carries a steady load at a fixed sag, so a sprung joint never creeps out of this band;
    an unsprung joint holding gravity has no band and creeps instead (``creep``)."""
    return j.hold / j.spring if j.spring > 0 else math.inf


def gravity_sign(j: Joint) -> int:
    """Which way gravity moves the joint from rest, in its own sign convention: -1 for a z slide, the sign of the
    gravity torque about the axis at rest for an x or y hinge, 0 when gravity cannot move it."""
    if j.kind == "slide":
        return -1 if j.axis == "z" else 0
    if j.axis == "z":
        return 0
    com = _joint_com(j)
    torque = -com[1] if j.axis == "x" else com[0]  # (r x (0, 0, -m g)) along x is -m g r_y, along y +m g r_x
    return 0 if abs(torque) < 1e-9 else (1 if torque > 0 else -1)


def _joint_inertia(j: Joint) -> float:
    """Inertia the joint moves (kg for a slide, kg m^2 about the axis for a hinge), without armature."""
    if j.kind == "slide":
        return float(j.mass)
    e = np.array(AXES[j.axis])
    try:
        _, com, inertia = mass_props(j.shapes, j.mass)
        r = np.asarray(com, dtype=float)
        return float(e @ inertia @ e + j.mass * (r @ r - (r @ e) ** 2))
    except ValueError:  # nothing collides: a point mass at the centre of the bound points
        r = _joint_com(j)
        return float(j.mass * (r @ r - (r @ e) ** 2)) + 1e-9


def creep_rate(j: Joint) -> float:
    """Upper estimate of how fast (m/s or rad/s) an unsprung joint drifts while its hold keeps it against gravity
    off its stops: CREEP_PER_ACCEL x the acceleration gravity would give it free (load / inertia). 0 when a hold is not
    what carries a load (no hold, a spring, or no gravity load). Armature and extra iterations are not credited."""
    if j.hold <= 0 or j.spring > 0:
        return 0.0
    load = gravity_load(j)
    if load <= 0:
        return 0.0
    return CREEP_PER_ACCEL[j.kind] * load / _joint_inertia(j)


def creep(j: Joint, seconds: float) -> float:
    """Upper estimate of how far (m or rad, toward gravity_sign) a hold lets the joint drift under gravity in
    ``seconds`` of steady load: CREEP_START plus creep_rate x seconds; 0 when creep_rate is 0. A joint resting on an end
    stop that gravity presses it into does not creep. Add it to score() margins, or carry the load on a stop or spring."""
    rate = creep_rate(j)
    if rate <= 0:
        return 0.0
    return CREEP_START[j.kind] + rate * max(0.0, float(seconds))


def articulation_iterations(m: Mech) -> int | None:
    """Solver position iterations mechkit authors for the Mech's articulation: ``m.iterations`` when given, else
    CREEP_ITERATIONS (64) when a hold keeps some joint against gravity, else None (the simulator default, 32)."""
    if m.iterations is not None:
        return int(m.iterations)
    if any(creep_rate(j) > 0 for j in m.joints):
        return CREEP_ITERATIONS
    return None


def hold_report(m: Mech) -> list[str]:
    """One line per joint: what gravity asks of it against its hold, its deadband and its detents. For check.py logs."""
    out = []
    for j in m.joints:
        unit, travel = ("N", "m") if j.kind == "slide" else ("N m", "rad")
        load = gravity_load(j)
        line = f"{j.name}: {j.axis} {j.kind}, gravity {load:.4g} {unit}, hold {j.hold:.4g} {unit}"
        if j.hold > 0:
            line += f" ({j.moving_hold:.4g} moving)"
            if load > 0:
                line += f", hold/gravity {j.hold / load:.2f}"
        if j.spring > 0:
            line += f", spring {j.spring:.4g}, deadband {deadband(j):.4g} {travel}"
            if load > 0 and j.hold < load:
                line += f", sag {(load - j.hold) / j.spring:.4g} {travel}"
        if creep_rate(j) > 0:
            line += (f", creeps under gravity: up to {creep(j, 10.0):.3g} {travel} in 10 s, {creep(j, 60.0):.3g} "
                     f"{travel} in 60 s (carry a scored position on a stop or a detent spring)")
        if j.armature > 0:
            line += f", armature {j.armature:.4g}"
        if j.detents:
            line += f", detents {list(j.detents)}"
        out.append(line)
    return out


def nearest_detent(stops, q: float) -> float:
    """The stop nearest the joint value ``q`` (the lower one on a tie). With two stops the switch happens at their
    midpoint: an over-centre spring."""
    stops = sorted(float(s) for s in stops)
    if not stops:
        raise ValueError("nearest_detent: no stops")
    return min(stops, key=lambda s: abs(s - q))


def _detent_plan(mech: str, spec: Mech, values: dict[str, float], live: dict) -> list[tuple[str, float]]:
    """(joint, new rest) for every detent joint whose nearest stop differs from the rest last written."""
    out = []
    for j in spec.joints:
        if j.detents and j.name in values:
            best = nearest_detent(j.detents, values[j.name])
            if live.get((mech, j.name), 0.0) != best:
                out.append((j.name, best))
    return out


def _actuator_kwargs(j: Joint, spec: Mech | None = None) -> dict:
    """ImplicitActuatorCfg arguments for a joint. Without a hold or a gear they are exactly what mechkit always
    passed. A geared joint (``spec`` given) gets its gears' stiffness and damping on top of its own (gear_gains)."""
    kw: dict[str, Any] = {"joint_names_expr": [f"{j.name}_joint"], "stiffness": j.spring, "damping": j.damping}
    extra = gear_gains(spec).get(j.name) if spec is not None else None
    if extra is not None:
        kw.update(stiffness=float(j.spring + extra[0]), damping=float(j.damping + extra[1]))
    if j.hold > 0:
        kw.update(friction=float(j.hold), dynamic_friction=j.moving_hold)
    if j.armature > 0:
        kw["armature"] = float(j.armature)
    return kw


def _pad_points(pose, half_gap: float, origin, tcp, past: float) -> list[np.ndarray]:
    """Four points for one gripper: at the TCP and at the pad tips, 6 mm outside each pad face. ``pose`` is the
    gripper body's (x, y, z, qx, qy, qz, qw) in the world, ``origin`` the env origin."""
    pose = np.asarray(pose, dtype=float)
    r = _quat_xyzw_matrix(pose[3:7])
    pts = []
    for along in (0.0, past):
        centre = pose[:3] + r @ (np.asarray(tcp, dtype=float) + [0.0, 0.0, along]) - np.asarray(origin, dtype=float)
        pts += [centre + r[:, 0] * side * (half_gap + 0.006) for side in (1.0, -1.0)]
    return pts


def finger_clearance(points, part, lo=None, hi=None) -> float:
    """Least distance (m) from any of ``points`` to a part: to its world bounding box (``part.low`` .. ``part.high``) by
    default, or to the box ``lo`` .. ``hi`` in the part's own frame (a MechState's frame, or an ObjectState's centre
    and turn). 0 when a point is inside."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if lo is None and hi is None:
        lo, hi = np.asarray(part.low, dtype=float), np.asarray(part.high, dtype=float)
    else:
        if lo is None or hi is None:
            raise ValueError("finger_clearance: give both lo and hi, or neither")
        lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
        if hasattr(part, "to_local"):
            pts = np.array([part.to_local(p) for p in pts])
        else:
            rot = _quat_xyzw_matrix(np.asarray(part.quat_xyzw, dtype=float))
            pts = (pts - np.asarray(part.pos, dtype=float)) @ rot
    out = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    return float(np.linalg.norm(out, axis=1).min())


# -- real geometry: convex collision pieces and smooth visual meshes, pure NumPy -----------------------------------------
#
# Every builder below returns Shape(kind="mesh"). Its ``hulls`` are convex pieces, each (points, counts, indices) in the
# shape frame; its ``visual`` is (points, counts, indices, corner_normals) for the render mesh. Both are computed when the
# task file is imported, so offline checks see exactly what the simulator authors. Builders work in a canonical frame
# (u, v, w) with w along ``axis`` and map it to the shape frame with a cyclic permutation (a proper rotation).
# All helpers were written for this kit; ideas seen in accepted tasks are credited by name of the idea, not copied.

HULL_VERTEX_LIMIT = 64  # vertices per convex piece the physics engine's GPU path accepts
MAX_SIDES = 32  # a two-ring piece of 2 x 32 points sits exactly at the limit
COLLIDE_SIDES = 24
VISUAL_SIDES = 64
MIN_WIDTH = 0.0005  # thinnest convex piece accepted (m)
CREASE_DEG = 35.0
DEFAULT_CONTACT = 0.0015
_PERM = {"z": (0, 1, 2), "x": (1, 2, 0), "y": (2, 0, 1)}  # canonical (u, v, w) component k lands on shape axis _PERM[k]
_TINTS = ((0.9, 0.25, 0.2), (0.2, 0.75, 0.3), (0.2, 0.4, 0.9), (0.95, 0.8, 0.1), (0.8, 0.3, 0.8), (0.1, 0.75, 0.8))


def _canon(points, axis: str) -> np.ndarray:
    """Map canonical (u, v, w) points, w along ``axis``, into the shape frame."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    out = np.empty_like(p)
    for k, dst in enumerate(_PERM[axis]):
        out[:, dst] = p[:, k]
    return out


def _ring(r: float, n: int, flats: bool = False, w: float = 0.0, a0: float = 0.0) -> np.ndarray:
    """n points round the w axis at height w: vertices on r, or on r / cos(pi / n) so the flats sit at r."""
    rv = r / math.cos(math.pi / n) if flats else r
    t = a0 + 2.0 * math.pi * np.arange(n) / n
    return np.column_stack([rv * np.cos(t), rv * np.sin(t), np.full(n, float(w))])


def _loft_faces(n: int, rows: int, apex0: bool = False, apex1: bool = False) -> list[list[int]]:
    """Faces of ``rows`` stacked rings of n points (optional apex point before the first ring and after the last).
    Corresponding ring points must line up so every side quad is planar; winding is fixed by _orient."""
    off = 1 if apex0 else 0

    def ring(k):
        return [off + k * n + i for i in range(n)]

    faces = [[0, ring(0)[(i + 1) % n], ring(0)[i]] for i in range(n)] if apex0 else [ring(0)[::-1]]
    for k in range(rows - 1):
        a, b = ring(k), ring(k + 1)
        faces += [[a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]] for i in range(n)]
    last = ring(rows - 1)
    faces += [[last[i], last[(i + 1) % n], off + rows * n] for i in range(n)] if apex1 else [last]
    return faces


def _prism_faces(n: int) -> list[list[int]]:
    """Two n-gon caps plus n side quads (points: bottom ring then top ring)."""
    return _loft_faces(n, 2)


def _face_area_vector(p: np.ndarray, f) -> np.ndarray:
    """Newell's method: area times unit normal, right-hand rule over the face's point order."""
    q = p[list(f)]
    r = np.roll(q, -1, axis=0)
    return 0.5 * np.array([np.sum((q[:, 1] - r[:, 1]) * (q[:, 2] + r[:, 2])),
                           np.sum((q[:, 2] - r[:, 2]) * (q[:, 0] + r[:, 0])),
                           np.sum((q[:, 0] - r[:, 0]) * (q[:, 1] + r[:, 1]))])


def _orient(pts: np.ndarray, faces) -> list[list[int]]:
    """Wind every face of a convex piece counter-clockwise seen from outside."""
    c = pts.mean(0)
    out = []
    for f in faces:
        f = list(f)
        if _face_area_vector(pts, f) @ (pts[f].mean(0) - c) < 0:
            f = f[::-1]
        out.append(f)
    return out


def _tol(p: np.ndarray) -> float:
    return 1e-10 + 1e-8 * float(np.ptp(p, axis=0).max())


def _check_hull(pts, faces, what: str = "piece") -> float:
    """Refuse a piece the simulator would cook differently: over 64 vertices, open, non-planar or inward faces, not
    convex, unused points or thinner than 0.5 mm. Returns its volume."""
    p = np.asarray(pts, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all():
        raise ValueError(f"{what}: points must be finite (x, y, z)")
    if len(p) > HULL_VERTEX_LIMIT:
        raise ValueError(f"{what}: {len(p)} vertices, over the {HULL_VERTEX_LIMIT}-vertex hull limit")
    if len(p) < 4 or len(faces) < 4:
        raise ValueError(f"{what}: a convex piece needs at least 4 points and 4 faces")
    tol = _tol(p)
    c = p.mean(0)
    used, normals, volume, total = set(), [], 0.0, np.zeros(3)
    for f in faces:
        f = list(f)
        if len(f) < 3 or min(f) < 0 or max(f) >= len(p):
            raise ValueError(f"{what}: bad face {f}")
        used.update(f)
        a = _face_area_vector(p, f)
        area = float(np.linalg.norm(a))
        if area <= 1e-12 * float(np.ptp(p, axis=0).max()) ** 2:
            raise ValueError(f"{what}: degenerate face {f}")
        n = a / area
        fc = p[f].mean(0)
        d = float(n @ fc)
        if np.abs(p[f] @ n - d).max() > tol:
            raise ValueError(f"{what}: face {f} is not planar")
        if n @ (fc - c) <= 0:
            raise ValueError(f"{what}: face {f} is wound inward")
        if (p @ n - d).max() > tol:
            raise ValueError(f"{what}: not convex (a point lies outside face {f})")
        normals.append(n)
        volume += d * area / 3.0
        total += a
    if len(used) != len(p):
        raise ValueError(f"{what}: {len(p) - len(used)} points are not on any face")
    if np.linalg.norm(total) > 1e-6 * max(volume, 1e-30) ** (2.0 / 3.0):
        raise ValueError(f"{what}: surface is not closed")
    if volume <= 0:
        raise ValueError(f"{what}: volume is not positive")
    proj = p @ np.array(normals).T
    width = float((proj.max(0) - proj.min(0)).min())
    if width < MIN_WIDTH - 1e-12:
        raise ValueError(f"{what}: {width * 1000:.3f} mm thick, under the {MIN_WIDTH * 1000:.1f} mm minimum")
    return volume


def _hull2d(q, tol: float) -> list[int]:
    """Monotone-chain convex hull of 2-D points, counter-clockwise, collinear points dropped."""
    order = sorted(range(len(q)), key=lambda i: (q[i][0], q[i][1]))

    def cross(o, a, b):
        return (q[a][0] - q[o][0]) * (q[b][1] - q[o][1]) - (q[a][1] - q[o][1]) * (q[b][0] - q[o][0])

    lower: list[int] = []
    for i in order:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], i) <= tol:
            lower.pop()
        lower.append(i)
    upper: list[int] = []
    for i in reversed(order):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], i) <= tol:
            upper.pop()
        upper.append(i)
    return lower[:-1] + upper[:-1]


def _perp(n: np.ndarray) -> np.ndarray:
    e = np.zeros(3)
    e[int(np.argmin(np.abs(n)))] = 1.0
    u = np.cross(n, e)
    return u / np.linalg.norm(u)


def _hull_faces(points) -> tuple[np.ndarray, list[list[int]]]:
    """Convex hull of 4 to 64 points: (vertices, faces wound outward). Points inside the hull, inside a face or along an
    edge are dropped. Vectorised supporting-plane search over every point triple."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    n = len(p)
    if n < 4:
        raise ValueError("a hull needs at least 4 points")
    if n > HULL_VERTEX_LIMIT:
        raise ValueError(f"a hull takes at most {HULL_VERTEX_LIMIT} points, got {n}")
    if not np.isfinite(p).all():
        raise ValueError("hull points must be finite")
    scale = float(np.ptp(p, axis=0).max())
    sv = np.linalg.svd(p - p.mean(0), compute_uv=False)
    if scale <= 0 or sv[2] <= 1e-7 * sv[0]:
        raise ValueError("hull points are coplanar or degenerate")
    tol = _tol(p)
    tri = np.array([(i, j, k) for i in range(n) for j in range(i + 1, n) for k in range(j + 1, n)])
    nr = np.cross(p[tri[:, 1]] - p[tri[:, 0]], p[tri[:, 2]] - p[tri[:, 0]])
    ln = np.linalg.norm(nr, axis=1)
    ok = ln > 1e-9 * scale * scale
    tri, nr = tri[ok], nr[ok] / ln[ok, None]
    s = p @ nr.T - np.einsum("ij,ij->i", nr, p[tri[:, 0]])
    planes: dict[tuple, np.ndarray] = {}
    for sel, sign in (((s <= tol).all(0), 1.0), ((s >= -tol).all(0), -1.0)):
        for t in np.nonzero(sel)[0]:
            key = tuple(np.nonzero(np.abs(s[:, t]) <= tol)[0].tolist())
            if key not in planes:
                planes[key] = sign * nr[t]
    polys = []
    for on, nrm in planes.items():
        e1 = _perp(nrm)
        e2 = np.cross(nrm, e1)
        q = [(float(p[i] @ e1), float(p[i] @ e2)) for i in on]
        ring = _hull2d(q, tol * scale)
        if len(ring) >= 3:
            polys.append([on[k] for k in ring])
    keep = sorted({i for f in polys for i in f})
    remap = {old: new for new, old in enumerate(keep)}
    return p[keep], [[remap[i] for i in f] for f in polys]


# -- 2-D polygons: simple-polygon checks, ear clipping, Hertel-Mehlhorn merge --------------------------------------------


def _area2d(q) -> float:
    q = np.asarray(q, dtype=float)
    x, y = q[:, 0], q[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def _cross2(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _clean_outline(outline, what: str) -> np.ndarray:
    """Counter-clockwise outline with repeated and collinear points removed."""
    q = [tuple(map(float, pt)) for pt in outline]
    if any(len(pt) != 2 for pt in q) or not np.isfinite(np.array(q)).all():
        raise ValueError(f"{what}: outline points must be finite (u, v) pairs")
    scale = max(float(np.ptp(np.array(q), axis=0).max()), 1e-12) if q else 1.0
    eps = 1e-9 * scale
    q = [pt for i, pt in enumerate(q) if math.dist(pt, q[i - 1]) > eps] if len(q) > 1 else q
    changed = True
    while changed and len(q) >= 3:
        changed = False
        for i in range(len(q)):
            if abs(_cross2(q[i - 1], q[i], q[(i + 1) % len(q)])) <= eps * scale:
                del q[i]
                changed = True
                break
    if len(q) < 3:
        raise ValueError(f"{what}: outline needs at least 3 distinct, non-collinear points")
    if _area2d(q) < 0:
        q.reverse()
    q = np.array(q)
    _check_simple(q, what)
    return q


def _segments_touch(a, b, c, d, eps: float) -> bool:
    d1, d2 = _cross2(c, d, a), _cross2(c, d, b)
    d3, d4 = _cross2(a, b, c), _cross2(a, b, d)
    if ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)):
        return True

    def on(p, q, r, dd):  # r on segment pq
        return abs(dd) <= eps and min(p[0], q[0]) - 1e-12 <= r[0] <= max(p[0], q[0]) + 1e-12 \
            and min(p[1], q[1]) - 1e-12 <= r[1] <= max(p[1], q[1]) + 1e-12

    return on(c, d, a, d1) or on(c, d, b, d2) or on(a, b, c, d3) or on(a, b, d, d4)


def _check_simple(q: np.ndarray, what: str) -> None:
    n = len(q)
    eps = 1e-12 * max(float(np.ptp(q, axis=0).max()), 1e-12) ** 2
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue
            if _segments_touch(q[i], q[(i + 1) % n], q[j], q[(j + 1) % n], eps):
                raise ValueError(f"{what}: outline crosses itself (edges {i} and {j}); holes are not supported, "
                                 "use bore() or compose shapes")


def _in_triangle(p, a, b, c, eps: float) -> bool:
    return _cross2(a, b, p) >= -eps and _cross2(b, c, p) >= -eps and _cross2(c, a, p) >= -eps


def _ear_clip(q) -> list[tuple[int, int, int]]:
    """Triangulate a simple counter-clockwise polygon."""
    q = [tuple(pt) for pt in np.asarray(q, dtype=float)]
    eps = 1e-12 * max(float(np.ptp(np.array(q), axis=0).max()), 1e-12) ** 2
    idx = list(range(len(q)))
    tris = []
    while len(idx) > 3:
        m = len(idx)
        for k in range(m):
            a, b, c = idx[k - 1], idx[k], idx[(k + 1) % m]
            if _cross2(q[a], q[b], q[c]) <= eps:
                continue
            if any(_in_triangle(q[o], q[a], q[b], q[c], eps) for o in idx if o not in (a, b, c)
                   and q[o] not in (q[a], q[b], q[c])):
                continue
            tris.append((a, b, c))
            del idx[k]
            break
        else:
            raise ValueError("outline could not be triangulated (degenerate or self-intersecting)")
    tris.append(tuple(idx))
    return tris


def _merge_polys(a: list[int], b: list[int]) -> list[int] | None:
    """Join two counter-clockwise polygons across a shared edge, or None if they share none."""
    for i in range(len(a)):
        x, y = a[i], a[(i + 1) % len(a)]
        for j in range(len(b)):
            if b[j] == y and b[(j + 1) % len(b)] == x:
                ra = a[i + 1:] + a[:i + 1]  # starts at y, ends at x
                rb = b[j + 1:] + b[:j + 1]  # starts at x, ends at y
                return ra + rb[1:-1]
    return None


def _convex_poly(q, poly, eps: float) -> bool:
    n = len(poly)
    return all(_cross2(q[poly[i - 1]], q[poly[i]], q[poly[(i + 1) % n]]) >= -eps for i in range(n))


def _drop_straight(q, poly, eps: float) -> list[int]:
    out = list(poly)
    i = 0
    while i < len(out) and len(out) > 3:
        if abs(_cross2(q[out[i - 1]], q[out[i]], q[out[(i + 1) % len(out)]])) <= eps:
            del out[i]
        else:
            i += 1
    return out


def _width2d(q: np.ndarray) -> float:
    best = math.inf
    for i in range(len(q)):
        e = q[(i + 1) % len(q)] - q[i]
        ln = math.hypot(*e)
        nrm = np.array([e[1], -e[0]]) / ln
        best = min(best, float(np.ptp((q - q[i]) @ nrm)))
    return best


def _decompose(outline) -> tuple[list[np.ndarray], list[tuple[int, int, int]], np.ndarray]:
    """Split a simple outline into convex pieces: ear clipping, then a Hertel-Mehlhorn merge (at most 4x the optimal piece
    count), then any piece over 32 vertices is cut in two. Returns (pieces, triangles, cleaned outline)."""
    q = _clean_outline(outline, "outline")
    eps = 1e-12 * max(float(np.ptp(q, axis=0).max()), 1e-12) ** 2
    tris = _ear_clip(q)
    polys = [list(t) for t in tris]
    merged = True
    while merged:
        merged = False
        for i in range(len(polys)):
            for j in range(i + 1, len(polys)):
                m = _merge_polys(polys[i], polys[j])
                if m is not None and _convex_poly(q, m, eps):
                    polys[i] = m
                    del polys[j]
                    merged = True
                    break
            if merged:
                break
    out: list[list[int]] = []
    todo = [_drop_straight(q, p, eps) for p in polys]
    while todo:
        p = todo.pop()
        if len(p) > HULL_VERTEX_LIMIT // 2:
            half = len(p) // 2
            todo += [p[:half + 1], p[half:] + [p[0]]]
        else:
            out.append(p)
    pieces = [q[p] for p in out]
    for k, piece in enumerate(pieces):
        if _width2d(piece) < MIN_WIDTH:
            raise ValueError(f"outline splits into a convex piece {_width2d(piece) * 1000:.2f} mm wide, under "
                             f"{MIN_WIDTH * 1000:.1f} mm; build curved outlines with arc() or sweep()")
    return pieces, tris, q


# -- visual meshes ---------------------------------------------------------------------------------------------------------


def _area_vectors(p: np.ndarray, faces) -> np.ndarray:
    """Area vector (area times unit normal) of every face, by fan triangles; equal to Newell's method."""
    tri = np.array([(fi, f[0], f[i], f[i + 1]) for fi, f in enumerate(faces) for i in range(1, len(f) - 1)])
    out = np.zeros((len(faces), 3))
    np.add.at(out, tri[:, 0], 0.5 * np.cross(p[tri[:, 2]] - p[tri[:, 1]], p[tri[:, 3]] - p[tri[:, 1]]))
    return out


def _corner_normals(points, faces, crease_deg: float = CREASE_DEG) -> np.ndarray:
    """One normal per face corner: the area-weighted mean of the normals of faces sharing that vertex whose normal lies
    within ``crease_deg`` of this face's, so curved surfaces shade smooth and sharp edges stay sharp."""
    p = np.asarray(points, dtype=float)
    av = _area_vectors(p, faces)
    ln = np.linalg.norm(av, axis=1)
    unit = np.divide(av, ln[:, None], out=np.zeros_like(av), where=ln[:, None] > 0)
    cf = np.array([fi for fi, f in enumerate(faces) for _ in f])
    cv = np.array([v for f in faces for v in f])
    order = np.argsort(cv, kind="stable")
    _, start, size = np.unique(cv[order], return_index=True, return_counts=True)
    group = np.repeat(np.arange(len(start)), size)  # group of each sorted corner
    k = size[group]  # pairs per sorted corner
    first = np.repeat(np.cumsum(k) - k, k)
    within = np.arange(int(k.sum())) - first
    pair_corner = order[np.repeat(np.arange(len(order)), k)]
    pair_face = cf[order[np.repeat(start[group], k) + within]]
    keep = np.einsum("ij,ij->i", unit[cf[pair_corner]], unit[pair_face]) >= math.cos(math.radians(crease_deg))
    acc = np.zeros((len(cf), 3))
    np.add.at(acc, pair_corner[keep], av[pair_face[keep]])
    la = np.linalg.norm(acc, axis=1)
    return np.where(la[:, None] > 0, acc / np.where(la > 0, la, 1.0)[:, None], unit[cf])


def _revolve(loop, n: int, a0: float = 0.0, a1: float | None = None) -> tuple[np.ndarray, list[list[int]]]:
    """Revolve a closed (r, z) loop round the w axis with n segments; a partial turn (a1 given) gets flat end caps.
    Points with r = 0 become one point on the axis."""
    lp: list[tuple[float, float]] = []
    for r, z in loop:
        pt = (max(0.0, float(r)), float(z))
        if not lp or math.dist(pt, lp[-1]) > 1e-12:
            lp.append(pt)
    if len(lp) > 1 and math.dist(lp[0], lp[-1]) <= 1e-12:
        lp.pop()
    if _area2d(lp) < 0:
        lp.reverse()
    full = a1 is None
    na = n if full else n + 1
    t = a0 + 2 * math.pi * np.arange(n) / n if full else np.linspace(a0, a1, n + 1)
    pts: list[tuple[float, float, float]] = []
    ids: list[list[int]] = []
    for r, z in lp:
        if r <= 1e-12:
            ids.append([len(pts)] * na)
            pts.append((0.0, 0.0, z))
        else:
            ids.append(list(range(len(pts), len(pts) + na)))
            pts += [(r * math.cos(a), r * math.sin(a), z) for a in t]
    faces = []
    m = len(lp)
    for e in range(m):
        s, u = ids[e], ids[(e + 1) % m]
        if lp[e][0] <= 1e-12 and lp[(e + 1) % m][0] <= 1e-12:
            continue
        for k in range(n):
            k1 = (k + 1) % na
            f = [s[k], s[k1], u[k1], u[k]]
            f = [v for i, v in enumerate(f) if v != f[i - 1]]
            if len(f) >= 3:
                faces.append(f)
    if not full:
        cap0 = [ids[e][0] for e in range(m)]
        cap1 = [ids[e][-1] for e in range(m)]
        for a, b, c in _ear_clip(lp):
            if len({cap0[a], cap0[b], cap0[c]}) == 3:
                faces.append([cap0[a], cap0[b], cap0[c]])
                faces.append([cap1[c], cap1[b], cap1[a]])
    return np.array(pts), faces


class _Welder:
    """Builds a mesh from faces given as point lists, sharing identical points so normals can be smoothed."""

    def __init__(self):
        self.pts: list[tuple[float, float, float]] = []
        self.index: dict[tuple, int] = {}
        self.faces: list[list[int]] = []

    def face(self, points) -> None:
        f = []
        for pt in points:
            key = tuple(round(float(c), 12) for c in pt)
            if key not in self.index:
                self.index[key] = len(self.pts)
                self.pts.append(tuple(float(c) for c in pt))
            if not f or f[-1] != self.index[key]:
                f.append(self.index[key])
        if len(f) > 1 and f[0] == f[-1]:
            f.pop()
        if len(f) >= 3:
            self.faces.append(f)

    def mesh(self) -> tuple[np.ndarray, list[list[int]]]:
        return np.array(self.pts), self.faces


# -- assembling a mesh Shape -------------------------------------------------------------------------------------------


def _as_tuple(pts) -> tuple:
    return tuple(tuple(float(c) for c in pt) for pt in np.asarray(pts, dtype=float))


def _pack(pts, faces) -> tuple:
    return (_as_tuple(pts), tuple(len(f) for f in faces), tuple(int(i) for f in faces for i in f))


def _unpack(piece) -> tuple[np.ndarray, list[list[int]]]:
    pts, counts, idx = piece[0], piece[1], piece[2]
    faces, k = [], 0
    for c in counts:
        faces.append(list(idx[k:k + c]))
        k += c
    return np.asarray(pts, dtype=float).reshape(-1, 3), faces


def _validate_tilt(tilt) -> tuple[float, float, float, float]:
    t = tuple(map(float, tilt))
    if len(t) != 4 or not all(math.isfinite(c) for c in t) or math.sqrt(sum(c * c for c in t)) < 1e-9:
        raise ValueError(f"tilt must be a nonzero (w, x, y, z) quaternion, got {tilt!r}")
    return t


def _make_mesh(form: str, anchor, color, tilt, axis: str, pieces, visual, contact) -> Shape:
    """Recentre the pieces on their bounding-box centre, check every piece, compute corner normals and pack the Shape.
    ``pieces`` and ``visual`` are (points, faces) in the shape frame relative to ``anchor``, before ``tilt``."""
    tilt = _validate_tilt(tilt)
    pieces = [(np.asarray(p, dtype=float), f) for p, f in pieces]
    allp = np.vstack([p for p, _ in pieces])
    lo, hi = allp.min(0), allp.max(0)
    centre = (lo + hi) / 2
    at = np.asarray(anchor, dtype=float) + _quat_wxyz_matrix(tilt) @ centre
    hulls = []
    for k, (p, f) in enumerate(pieces):
        p = p - centre
        f = _orient(p, f)
        _check_hull(p, f, f"{form} piece {k}")
        hulls.append(_pack(p, f))
    vp, vf = visual
    vp = np.asarray(vp, dtype=float) - centre
    normals = _corner_normals(vp, vf)
    vis = _pack(vp, vf) + (_as_tuple(normals),)
    return Shape("mesh", tuple(float(c) for c in at), tuple(float(c) for c in hi - lo), tuple(map(float, color)), tilt,
                 axis, form, tuple(hulls), vis, True, True, contact)


def _positive(what: str, **values) -> None:
    for name, v in values.items():
        if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0):
            raise ValueError(f"{what}: {name} must be a positive number, got {v!r}")


def _sides(what: str, sides: int, low: int = 3, high: int = MAX_SIDES) -> int:
    if int(sides) != sides or not low <= sides <= high:
        raise ValueError(f"{what}: sides must be an integer {low} to {high}, got {sides!r}")
    return int(sides)


def _axis(what: str, axis: str) -> str:
    if axis not in AXES:
        raise ValueError(f"{what}: axis must be x, y or z, got {axis!r}")
    return axis


# -- builders ----------------------------------------------------------------------------------------------------------


def box(at, size, color=GREY, tilt=IDENT, contact=None) -> Shape:
    """A box of edge lengths ``size`` centred at ``at``. Native cube collider."""
    return Shape("box", tuple(map(float, at)), tuple(map(float, size)), tuple(color), tuple(map(float, tilt)),
                 contact=contact)


def cyl(at, radius, height, color=GREY, axis="z", tilt=IDENT, sides=0, contact=None) -> Shape:
    """A cylinder centred at ``at`` along ``axis``, turned by ``tilt``. sides=0 keeps the native cylinder collider;
    sides 3 to 32 gives a convex prism collider with its vertices on the radius (8 to 12 sides keep a loose round part
    from rolling forever) and a smooth 64-side visual."""
    _axis("cyl", axis)
    _positive("cyl", radius=radius, height=height)
    if not sides:
        return Shape("cyl", tuple(map(float, at)), (float(radius), float(radius), float(height)), tuple(color),
                     tuple(map(float, tilt)), axis, contact=contact)
    n = _sides("cyl", sides)
    r, h = float(radius), float(height)
    piece = (np.vstack([_ring(r, n, w=-h / 2), _ring(r, n, w=h / 2)]), _prism_faces(n))
    vis = _revolve([(0.0, -h / 2), (r, -h / 2), (r, h / 2), (0.0, h / 2)], VISUAL_SIDES)
    return _make_mesh("cyl", at, color, tilt, axis, [(_canon(piece[0], axis), piece[1])],
                      (_canon(vis[0], axis), vis[1]), contact)


def _quat_z_to(d) -> tuple[float, float, float, float]:
    """The quaternion (w, x, y, z) turning +z onto direction d."""
    d = np.asarray(d, dtype=float) / np.linalg.norm(d)
    if 1.0 + d[2] < 1e-12:
        return (0.0, 1.0, 0.0, 0.0)
    q = np.array([1.0 + d[2], -d[1], d[0], 0.0])
    return tuple(float(c) for c in q / np.linalg.norm(q))


def rod(p0, p1, radius, color=GREY, sides=0, contact=None) -> Shape:
    """A round bar from point p0 to point p1 (a cyl centred on the midpoint, tilted onto the bar)."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    length = float(np.linalg.norm(p1 - p0))
    _positive("rod", length=length, radius=radius)
    return cyl(tuple((p0 + p1) / 2), radius, length, color, "z", _quat_z_to(p1 - p0), sides, contact)


def _lathe(at, profile, vprofile, color, axis, tilt, sides, fit, contact, form="lathe") -> Shape:
    _axis(form, axis)
    n = _sides(form, sides)
    if fit not in ("inside", "outside"):
        raise ValueError(f"{form}: fit must be inside or outside, got {fit!r}")
    rows = [(0.0 if abs(float(r)) < 1e-12 else float(r), float(z)) for r, z in profile]
    if len(rows) < 2:
        raise ValueError(f"{form}: profile needs at least 2 (r, z) rows")
    for (r0, z0), (r1, z1) in zip(rows, rows[1:]):
        if not z1 > z0:
            raise ValueError(f"{form}: profile z must increase strictly, got {z0} then {z1}")
        if r0 <= 0 and r1 <= 0:
            raise ValueError(f"{form}: two consecutive rows on the axis")
    if any(r < 0 or not math.isfinite(r) for r, _ in rows):
        raise ValueError(f"{form}: profile radii must be >= 0")
    flats = fit == "outside"
    pieces = []
    for (r0, z0), (r1, z1) in zip(rows, rows[1:]):
        if r0 > 0 and r1 > 0:
            pieces.append((np.vstack([_ring(r0, n, flats, z0), _ring(r1, n, flats, z1)]), _loft_faces(n, 2)))
        elif r0 > 0:
            pieces.append((np.vstack([_ring(r0, n, flats, z0), [(0.0, 0.0, z1)]]), _loft_faces(n, 1, apex1=True)))
        else:
            pieces.append((np.vstack([[(0.0, 0.0, z0)], _ring(r1, n, flats, z1)]), _loft_faces(n, 1, apex0=True)))
    vrows = [(float(r), float(z)) for r, z in vprofile]
    loop = ([(0.0, vrows[0][1])] if vrows[0][0] > 0 else []) + vrows + ([(0.0, vrows[-1][1])] if vrows[-1][0] > 0 else [])
    vp, vf = _revolve(loop, VISUAL_SIDES)
    return _make_mesh(form, at, color, tilt, axis, [(_canon(p, axis), f) for p, f in pieces], (_canon(vp, axis), vf),
                      contact)


def lathe(at, profile, color=GREY, axis="z", tilt=IDENT, sides=COLLIDE_SIDES, fit="inside", contact=None) -> Shape:
    """A solid of revolution: ``profile`` is [(r, z), ...] with z strictly increasing (metres from ``at`` along
    ``axis``) and r >= 0; the ends close automatically and r = 0 makes an apex. One convex frustum per row pair.
    fit="inside" puts the vertices on r (the collider is never bigger than the part); "outside" puts the flats on r."""
    return _lathe(at, profile, profile, color, axis, tilt, sides, fit, contact)


def cone(at, r0, r1, height, color=GREY, axis="z", tilt=IDENT, sides=COLLIDE_SIDES, fit="inside", contact=None) -> Shape:
    """A cone or frustum centred at ``at``: radius r0 at -height/2 and r1 at +height/2 along ``axis`` (either may be 0)."""
    _positive("cone", height=height)
    h = float(height)
    return lathe(at, [(r0, -h / 2), (r1, h / 2)], color, axis, tilt, sides, fit, contact)


def ball(at, radius, color=GREY, tilt=IDENT, sides=COLLIDE_SIDES, rows=8, contact=None) -> Shape:
    """A sphere centred at ``at``: ``rows`` stacked frusta (vertices on the sphere) and a smooth visual."""
    _positive("ball", radius=radius)
    if int(rows) != rows or rows < 2:
        raise ValueError(f"ball: rows must be an integer >= 2, got {rows!r}")
    r = float(radius)

    def prof(k):
        return [(0.0 if i in (0, k) else r * math.sin(math.pi * i / k), -r * math.cos(math.pi * i / k))
                for i in range(k + 1)]

    return _lathe(at, prof(int(rows)), prof(32), color, "z", tilt, sides, "inside", contact)


def _sector_count(span: float, sides: int, segments) -> int:
    if segments is not None:
        if int(segments) != segments or segments < 1:
            raise ValueError(f"segments must be a positive integer, got {segments!r}")
        n = int(segments)
    else:
        n = sides if span >= 2 * math.pi - 1e-9 else max(1, math.ceil(span / (2 * math.pi / sides) - 1e-9))
    if span / n > 2 * math.pi / 3 + 1e-9:
        raise ValueError("each sector must span at most 120 degrees; use more segments")
    return n


def _merge_if_convex(pieces):
    """Greedily merge consecutive pieces whose union is convex (hull volume equals the sum) and fits 64 vertices."""
    out = [pieces[0]]
    for p, f in pieces[1:]:
        q, g = out[-1]
        pts = np.vstack([q, p])
        try:
            hp, hf = _hull_faces(pts)
        except ValueError:
            out.append((p, f))
            continue
        v_sum = _check_hull(q, _orient(q, g)) + _check_hull(p, _orient(p, f))
        v_hull = _check_hull(hp, hf)
        if abs(v_hull - v_sum) <= 1e-9 * v_sum:
            out[-1] = (hp, hf)
        else:
            out.append((p, f))
    return out


def ring_profile(at, rows, color=GREY, axis="z", tilt=IDENT, sides=COLLIDE_SIDES, a0=0.0, a1=2 * math.pi, floor=0.0,
                 segments=None, contact=None, form="tube") -> Shape:
    """A hollow round part from ``rows`` = [(r_in, r_out, z), ...] (z strictly increasing, metres from ``at`` along
    ``axis``). Collision is one convex wedge per sector and row pair (merged up the rows where the union stays convex).
    Inner vertices sit at r_in / cos(half sector), so the inner flats sit exactly at r_in and a hole is never smaller
    than nominal; outer vertices sit at r_out. a0..a1 gives a C clip or slotted ring. ``floor`` adds a disc of that
    thickness below the first row, making a cup."""
    _axis(form, axis)
    n_full = _sides(form, sides)
    rows = [(float(a), float(b), float(z)) for a, b, z in rows]
    if len(rows) < 2:
        raise ValueError(f"{form}: needs at least 2 rows")
    for (ri, ro, z), nxt in zip(rows, rows[1:] + [None]):
        if not 0 < ri < ro or not math.isfinite(ro):
            raise ValueError(f"{form}: each row needs 0 < r_in < r_out, got {ri}, {ro}")
        if nxt is not None and not nxt[2] > z:
            raise ValueError(f"{form}: row z must increase strictly")
    span = float(a1) - float(a0)
    if not 0 < span <= 2 * math.pi + 1e-12:
        raise ValueError(f"{form}: a1 - a0 must be in (0, 2 pi], got {span}")
    full = span >= 2 * math.pi - 1e-9
    n = _sector_count(span, n_full, segments)
    half = span / (2 * n)
    k_in = 1.0 / math.cos(half)
    pieces = []
    for k in range(n):
        t0, t1 = a0 + 2 * half * k, a0 + 2 * half * (k + 1)
        c0, s0, c1, s1 = math.cos(t0), math.sin(t0), math.cos(t1), math.sin(t1)

        def quad(ri, ro, z):
            ri = ri * k_in
            return [(ri * c0, ri * s0, z), (ro * c0, ro * s0, z), (ro * c1, ro * s1, z), (ri * c1, ri * s1, z)]

        stack = [(np.array(quad(*rows[j]) + quad(*rows[j + 1])), _loft_faces(4, 2)) for j in range(len(rows) - 1)]
        pieces += _merge_if_convex(stack)
    floor = float(floor)
    if floor < 0:
        raise ValueError(f"{form}: floor must be >= 0")
    if floor > 0:
        zb, zt = rows[0][2] - floor, rows[0][2]
        ro = rows[0][1]
        if full:
            pieces.append((np.vstack([_ring(ro, n_full, w=zb), _ring(ro, n_full, w=zt)]), _prism_faces(n_full)))
        else:
            per = max(1, min(n, int((math.pi + 1e-9) // (2 * half)), MAX_SIDES - 2))
            for k0 in range(0, n, per):
                ks = range(k0, min(n, k0 + per) + 1)
                rim = [(0.0, 0.0)] + [(ro * math.cos(a0 + 2 * half * k), ro * math.sin(a0 + 2 * half * k)) for k in ks]
                poly = np.array(rim)
                m = len(poly)
                pts = np.vstack([np.column_stack([poly, np.full(m, zb)]), np.column_stack([poly, np.full(m, zt)])])
                pieces.append((pts, _prism_faces(m)))
    outer = ([(0.0, rows[0][2] - floor), (rows[0][1], rows[0][2] - floor)] if floor > 0 else []) + \
        [(ro, z) for _, ro, z in rows]
    inner = [(ri, z) for ri, _, z in reversed(rows)] + ([(0.0, rows[0][2])] if floor > 0 else [])
    vp, vf = _revolve(outer + inner, VISUAL_SIDES if full else max(2, math.ceil(VISUAL_SIDES * span / (2 * math.pi))),
                      float(a0), None if full else float(a1))
    return _make_mesh(form, at, color, tilt, axis, [(_canon(p, axis), f) for p, f in pieces], (_canon(vp, axis), vf),
                      contact)


def tube(at, r_in, r_out, height, color=GREY, axis="z", tilt=IDENT, sides=COLLIDE_SIDES, a0=0.0, a1=2 * math.pi,
         lead_in=0.0, floor=0.0, contact=None) -> Shape:
    """A tube centred at ``at`` along ``axis``. lead_in adds a 45 degree entry chamfer at the top (+axis end) that opens
    the bore to r_in + lead_in; floor closes the bottom with a disc of that thickness (a cup); a0..a1 cuts a C."""
    _positive("tube", r_in=r_in, r_out=r_out, height=height)
    h, li, fl = float(height), float(lead_in), float(floor)
    if li < 0 or fl < 0:
        raise ValueError("tube: lead_in and floor must be >= 0")
    z0 = -h / 2 + fl
    if h / 2 - li - z0 < MIN_WIDTH:
        raise ValueError("tube: floor plus lead_in leave no straight wall")
    if r_in + li > r_out - MIN_WIDTH:
        raise ValueError("tube: lead_in cuts through the wall")
    rows = [(r_in, r_out, z0)]
    rows += [(r_in, r_out, h / 2 - li), (r_in + li, r_out, h / 2)] if li > 0 else [(r_in, r_out, h / 2)]
    return ring_profile(at, rows, color, axis, tilt, sides, a0, a1, fl, None, contact)


def _rect_hits(cu: float, cv: float, U: float, V: float, t: float) -> np.ndarray:
    """Where the ray from (cu, cv) at angle t leaves the rectangle |u| <= U/2, |v| <= V/2."""
    c, s = math.cos(t), math.sin(t)
    best = math.inf
    if c > 1e-15:
        best = min(best, (U / 2 - cu) / c)
    if c < -1e-15:
        best = min(best, (-U / 2 - cu) / c)
    if s > 1e-15:
        best = min(best, (V / 2 - cv) / s)
    if s < -1e-15:
        best = min(best, (-V / 2 - cv) / s)
    return np.array([cu + best * c, cv + best * s])


def _bore_polygon(cu, cv, U, V, t0, t1, rv) -> list[tuple[float, float]]:
    """Convex sector of the block round the hole: inner points at rv, the ray hits, and any corner between them,
    counter-clockwise."""
    corners = [(U / 2, V / 2), (-U / 2, V / 2), (-U / 2, -V / 2), (U / 2, -V / 2)]
    between = []
    for x, y in corners:
        a = math.atan2(y - cv, x - cu)
        while a < t0:
            a += 2 * math.pi
        if t0 + 1e-12 < a < t1 - 1e-12:
            between.append((a, (x, y)))
    pts = [(cu + rv * math.cos(t0), cv + rv * math.sin(t0)), tuple(_rect_hits(cu, cv, U, V, t0))]
    pts += [p for _, p in sorted(between)]
    pts += [tuple(_rect_hits(cu, cv, U, V, t1)), (cu + rv * math.cos(t1), cv + rv * math.sin(t1))]
    return pts


def bore(at, size, radius, color=GREY, axis="z", tilt=IDENT, sides=COLLIDE_SIDES, lead_in=0.0, centre=(0.0, 0.0),
         floor=0.0, contact=None) -> Shape:
    """A block of edge lengths ``size`` centred at ``at`` with a round hole of ``radius`` along ``axis``, its centre
    offset by ``centre`` (in the two block directions across the axis, in cyclic order after it). Each sector piece is
    (convex sector) intersect (rectangle) intersect (beyond the chord), so it is convex and no piece spans the hole;
    the flats sit exactly at ``radius``. lead_in chamfers the +axis entry; floor > 0 makes a blind hole."""
    _axis("bore", axis)
    n = _sides("bore", sides)
    size = tuple(map(float, size))
    _positive("bore", radius=radius, sx=size[0], sy=size[1], sz=size[2])
    perm = _PERM[axis]
    U, V, W = size[perm[0]], size[perm[1]], size[perm[2]]
    cu, cv = map(float, centre)
    r, li, fl = float(radius), float(lead_in), float(floor)
    if li < 0 or fl < 0:
        raise ValueError("bore: lead_in and floor must be >= 0")
    half = math.pi / n
    rmax = (r + li) / math.cos(half)
    wall = min(U / 2 - abs(cu), V / 2 - abs(cv)) - rmax
    if wall < MIN_WIDTH:
        raise ValueError(f"bore: wall beside the hole is {wall * 1000:.2f} mm, needs at least {MIN_WIDTH * 1000:.1f} mm")
    zb, zt = -W / 2 + fl, W / 2
    zc = zt - li
    if zc - zb < MIN_WIDTH or (fl > 0 and fl < MIN_WIDTH):
        raise ValueError("bore: floor plus lead_in leave no straight wall (or the floor is under 0.5 mm)")
    pieces = []
    for k in range(n):
        t0, t1 = 2 * half * k, 2 * half * (k + 1)

        def layer(rr, z):
            return [(u, v, z) for u, v in _bore_polygon(cu, cv, U, V, t0, t1, rr / math.cos(half))]

        pts = layer(r, zb) + (layer(r, zc) + layer(r + li, zt) if li > 0 else layer(r, zt))
        pieces.append(_hull_faces(pts))
    if fl > 0:
        slab = [(su * U / 2, sv * V / 2, z) for z in (-W / 2, zb) for su, sv in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        pieces.append((np.array(slab), _prism_faces(4)))
    wd = _Welder()
    nv = VISUAL_SIDES
    for k in range(nv):
        t0, t1 = 2 * math.pi * k / nv, 2 * math.pi * (k + 1) / nv
        pb = _bore_polygon(cu, cv, U, V, t0, t1, r)
        pt = _bore_polygon(cu, cv, U, V, t0, t1, r + li)
        wd.face([(u, v, zt) for u, v in pt])
        if fl == 0:
            wd.face([(u, v, zb) for u, v in reversed(pb)])
        for i in range(1, len(pb) - 2):  # the block's outside, between consecutive boundary points
            a, b = pb[i], pb[i + 1]
            wd.face([(a[0], a[1], -W / 2 if fl > 0 else zb), (b[0], b[1], -W / 2 if fl > 0 else zb),
                     (b[0], b[1], zt), (a[0], a[1], zt)])
        a, b = pb[-1], pb[0]  # the hole wall, inner t1 to inner t0
        if li > 0:
            wd.face([(a[0], a[1], zb), (b[0], b[1], zb), (b[0], b[1], zc), (a[0], a[1], zc)])
            ta, tb = pt[-1], pt[0]
            wd.face([(a[0], a[1], zc), (b[0], b[1], zc), (tb[0], tb[1], zt), (ta[0], ta[1], zt)])
        else:
            wd.face([(a[0], a[1], zb), (b[0], b[1], zb), (b[0], b[1], zt), (a[0], a[1], zt)])
    if fl > 0:
        wd.face([(U / 2, V / 2, -W / 2), (U / 2, -V / 2, -W / 2), (-U / 2, -V / 2, -W / 2), (-U / 2, V / 2, -W / 2)])
        for k in range(nv):  # the hole floor
            t0, t1 = 2 * math.pi * k / nv, 2 * math.pi * (k + 1) / nv
            wd.face([(cu, cv, zb), (cu + r * math.cos(t0), cv + r * math.sin(t0), zb),
                     (cu + r * math.cos(t1), cv + r * math.sin(t1), zb)])
    vp, vf = wd.mesh()
    return _make_mesh("bore", at, color, tilt, axis, [(_canon(p, axis), f) for p, f in pieces], (_canon(vp, axis), vf),
                      contact)


def extrude(at, outline, depth, color=GREY, axis="z", tilt=IDENT, contact=None) -> Shape:
    """A 2-D outline [(u, v), ...] in the plane across ``axis`` (u, v are the next two axes in cyclic order), extruded
    from -depth/2 to +depth/2. Convex or concave; clockwise outlines are reversed; self-intersecting outlines and holes
    are refused. Concave outlines split into convex pieces; curves made of many points become slivers, so build curved
    parts with arc() or sweep()."""
    _axis("extrude", axis)
    _positive("extrude", depth=depth)
    d = float(depth)
    if d < MIN_WIDTH:
        raise ValueError("extrude: depth under 0.5 mm")
    polys, tris, q = _decompose(outline)
    pieces = []
    for poly in polys:
        m = len(poly)
        pts = np.vstack([np.column_stack([poly, np.full(m, -d / 2)]), np.column_stack([poly, np.full(m, d / 2)])])
        pieces.append((pts, _prism_faces(m)))
    m = len(q)
    vp = np.vstack([np.column_stack([q, np.full(m, -d / 2)]), np.column_stack([q, np.full(m, d / 2)])])
    vf = [[c, b, a] for a, b, c in tris] + [[a + m, b + m, c + m] for a, b, c in tris]
    vf += [[i, (i + 1) % m, (i + 1) % m + m, i + m] for i in range(m)]
    return _make_mesh("extrude", at, color, tilt, axis, [(_canon(p, axis), f) for p, f in pieces],
                      (_canon(vp, axis), vf), contact)


def _rotate_onto(u: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotate u by the smallest rotation taking unit a onto unit b (parallel transport of a frame)."""
    k = np.cross(a, b)
    s = float(np.linalg.norm(k))
    c = float(a @ b)
    if s < 1e-15:
        return u
    k /= s
    return u * c + np.cross(k, u) * s + k * (k @ u) * (1 - c)


def _fillet(points: np.ndarray, radius: float, max_bend: float) -> np.ndarray:
    """Replace every bend sharper than max_bend with a circular fillet cut into pieces of at most max_bend."""
    pts = [points[0]]
    for i in range(1, len(points) - 1):
        p, a, b = points[i], points[i - 1], points[i + 1]
        d_in, d_out = (p - a) / np.linalg.norm(p - a), (b - p) / np.linalg.norm(b - p)
        bend = math.acos(max(-1.0, min(1.0, float(d_in @ d_out))))
        if bend <= max_bend + 1e-12:
            pts.append(p)
            continue
        if bend > math.radians(170):
            raise ValueError(f"sweep: bend at point {i} is over 170 degrees")
        rf = 1.5 * radius
        t = rf * math.tan(bend / 2)
        limit = 0.5 * min(np.linalg.norm(p - a), np.linalg.norm(b - p))
        if t > limit:
            t, rf = limit, limit / math.tan(bend / 2)
        if rf < 1.05 * radius:
            raise ValueError(f"sweep: bend at point {i} is too sharp for its segment lengths; lengthen the segments "
                             "or add points")
        k = math.ceil(bend / max_bend - 1e-9)
        s = p - d_in * t
        nrm = d_out - (d_out @ d_in) * d_in
        nrm /= np.linalg.norm(nrm)
        centre = s + nrm * rf
        for j in range(k + 1):
            psi = bend * j / k
            pts.append(centre - nrm * rf * math.cos(psi) + d_in * rf * math.sin(psi))
    pts.append(points[-1])
    return np.array(pts)


def _sweep_rings(points, radius: float, sides: int, max_bend_deg: float = 15.0):
    """Per segment of the (filleted) polyline: (start ring, end ring, frame u, frame v). Interior rings lie on the mitre
    plane; a parallel-transported frame makes segment j's end ring and segment j+1's start ring the same points."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    keep = [0] + [i for i in range(1, len(p)) if np.linalg.norm(p[i] - p[i - 1]) > 1e-9]
    p = p[keep]
    if len(p) < 2:
        raise ValueError("sweep: needs at least 2 distinct points")
    p = _fillet(p, radius, math.radians(max_bend_deg))
    d = p[1:] - p[:-1]
    d /= np.linalg.norm(d, axis=1)[:, None]
    u = _perp(d[0])
    phi = 2 * math.pi * np.arange(sides) / sides
    out = []
    for j in range(len(d)):
        if j:
            u = _rotate_onto(u, d[j - 1], d[j])
            u = u - d[j] * (u @ d[j])
            u /= np.linalg.norm(u)
        v = np.cross(d[j], u)
        circle = radius * (np.outer(np.cos(phi), u) + np.outer(np.sin(phi), v))
        m0 = d[j] if j == 0 else (d[j - 1] + d[j]) / np.linalg.norm(d[j - 1] + d[j])
        m1 = d[j] if j == len(d) - 1 else (d[j] + d[j + 1]) / np.linalg.norm(d[j] + d[j + 1])
        r0 = p[j] + circle - np.outer(circle @ m0 / (d[j] @ m0), d[j])
        r1 = p[j + 1] + circle - np.outer(circle @ m1 / (d[j] @ m1), d[j])
        out.append((r0, r1, u, v))
    return out, p, d


def _dome(base_centre, direction, u, v, radius, sides, rows):
    """A hemisphere's rings above a base ring (included first) and its apex, along ``direction``."""
    phi = 2 * math.pi * np.arange(sides) / sides
    rings = []
    for k in range(rows):
        a = 0.5 * math.pi * k / rows
        rings.append(base_centre + direction * radius * math.sin(a) +
                     radius * math.cos(a) * (np.outer(np.cos(phi), u) + np.outer(np.sin(phi), v)))
    return np.vstack(rings + [base_centre + direction * radius])


def sweep(at, points, radius, color=GREY, sides=12, max_bend_deg=15.0, ends="flat", tilt=IDENT, contact=None) -> Shape:
    """A round bar of ``radius`` along the polyline ``points`` (metres from ``at``). Each segment is one convex piece
    between mitred rings, so pieces meet exactly with no overlap; bends sharper than ``max_bend_deg`` get a fillet of
    about 1.5 x radius cut into small bends. ends="round" adds a hemisphere piece at each end."""
    _positive("sweep", radius=radius, max_bend_deg=max_bend_deg)
    n = _sides("sweep", sides)
    if ends not in ("flat", "round"):
        raise ValueError(f"sweep: ends must be flat or round, got {ends!r}")
    r = float(radius)
    rings, p, d = _sweep_rings(points, r, n, max_bend_deg)
    pieces = [(np.vstack([r0, r1]), _prism_faces(n)) for r0, r1, _, _ in rings]
    if ends == "round":
        for centre, direction, (_, _, u, v) in ((p[0], -d[0], rings[0]), (p[-1], d[-1], rings[-1])):
            pieces.append((_dome(centre, direction, u, v, r, n, 3), _loft_faces(n, 3, apex1=True)))
    nv = 24
    vr, _, _ = _sweep_rings(p, r, nv, 180.0)  # p is already filleted, so no further change
    wd = _Welder()
    ring_list = [vr[0][0]] + [r1 for _, r1, _, _ in vr]
    for a, b in zip(ring_list, ring_list[1:]):
        for i in range(nv):
            wd.face([a[i], a[(i + 1) % nv], b[(i + 1) % nv], b[i]])
    if ends == "flat":
        wd.face(list(ring_list[0][::-1]))
        wd.face(list(ring_list[-1]))
    else:
        # rings run counter-clockwise about the bar direction, so the start dome (pointing back) is wound reversed
        for centre, direction, (_, _, u, v), base, flip in ((p[0], -d[0], vr[0], ring_list[0], True),
                                                            (p[-1], d[-1], vr[-1], ring_list[-1], False)):
            dome = _dome(centre, direction, u, v, r, nv, 6)
            dome[:nv] = base
            for k in range(6):
                lo = dome[k * nv:(k + 1) * nv]
                hi = dome[(k + 1) * nv:(k + 2) * nv] if k < 5 else None
                for i in range(nv):
                    j = (i + 1) % nv
                    f = [lo[i], lo[j], hi[j], hi[i]] if hi is not None else [lo[i], lo[j], dome[-1]]
                    wd.face(f[::-1] if flip else f)
    vp, vf = wd.mesh()
    vf = _outward_visual(vp, vf)
    return _make_mesh("sweep", at, color, tilt, "z", pieces, (vp, vf), contact)


def _outward_visual(pts: np.ndarray, faces) -> list[list[int]]:
    """Flip the whole visual mesh if its signed volume is negative (a consistently wound closed mesh)."""
    vol = sum(float(pts[f[0]] @ np.cross(pts[f[i]], pts[f[i + 1]])) for f in faces for i in range(1, len(f) - 1))
    return faces if vol >= 0 else [f[::-1] for f in faces]


def arc(at, radius, a0, a1, color=GREY, section=0.003, axis="z", tilt=IDENT, segments=None, sides=12,
        contact=None) -> Shape:
    """A bar bent round ``axis`` at ``radius`` from angle a0 to a1 (radians, counter-clockwise about the axis).
    A float ``section`` is a round bar of that radius (a sweep); (w, h) is a rectangular bar w wide radially and h
    along the axis (annular-sector wedges). segments defaults to one per 15 degrees."""
    _axis("arc", axis)
    _positive("arc", radius=radius)
    span = float(a1) - float(a0)
    if not 0 < span <= 2 * math.pi + 1e-12:
        raise ValueError(f"arc: a1 - a0 must be in (0, 2 pi], got {span}")
    n = int(segments) if segments is not None else max(1, math.ceil(span / math.radians(15) - 1e-9))
    R = float(radius)
    if isinstance(section, (int, float)):
        _positive("arc", section=section)
        if section >= R:
            raise ValueError("arc: a round section must be smaller than the bend radius")
        t = np.linspace(float(a0), float(a1), n + 1)
        pts = _canon(np.column_stack([R * np.cos(t), R * np.sin(t), np.zeros(n + 1)]), axis)
        s = sweep(at, pts, float(section), color, sides, 180.0, "flat", tilt, contact)
        return dataclasses.replace(s, form="arc", axis=axis)
    w, h = map(float, section)
    _positive("arc", width=w, height=h)
    if w / 2 >= R:
        raise ValueError("arc: the section is wider than twice the bend radius")
    rows = [(R - w / 2, R + w / 2, -h / 2), (R - w / 2, R + w / 2, h / 2)]
    return ring_profile(at, rows, color, axis, tilt, MAX_SIDES, a0, a1, 0.0, n, contact, form="arc")


def hull(at, points, color=GREY, tilt=IDENT, contact=None) -> Shape:
    """One convex piece from 4 to 64 raw points (metres from ``at``); interior points are dropped and coplanar sets are
    refused. Flat shaded."""
    p, f = _hull_faces(points)
    return _make_mesh("hull", at, color, tilt, "z", [(p, f)], (p, f), contact)


# -- measuring shapes, pure NumPy --------------------------------------------------------------------------------------


def _usd_mesh_arrays(piece) -> tuple[list, list, list, list | None]:
    """(points, face vertex counts, face vertex indices, corner normals or None) for a hull piece or visual tuple."""
    normals = [tuple(n) for n in piece[3]] if len(piece) > 3 else None
    return [tuple(p) for p in piece[0]], list(piece[1]), list(piece[2]), normals


def _piece_moments(pts: np.ndarray, faces) -> tuple[float, np.ndarray, np.ndarray]:
    """Volume, first moment and second moment (integral of x x^T) of a closed outward mesh, by tetrahedra fanned from
    its vertex centroid."""
    c = pts.mean(0)
    tris = np.array([(f[0], f[i], f[i + 1]) for f in faces for i in range(1, len(f) - 1)])
    a, b, e = pts[tris[:, 0]] - c, pts[tris[:, 1]] - c, pts[tris[:, 2]] - c
    det = np.einsum("ij,ij->i", a, np.cross(b, e))
    vol = float(det.sum() / 6)
    s = a + b + e
    first = (det[:, None] * s).sum(0) / 24
    second = np.einsum("i,ij,ik->jk", det, a, a) + np.einsum("i,ij,ik->jk", det, b, b) + \
        np.einsum("i,ij,ik->jk", det, e, e) + np.einsum("i,ij,ik->jk", det, s, s)
    second /= 120
    second = second + np.outer(c, first) + np.outer(first, c) + vol * np.outer(c, c)
    return vol, first + vol * c, second


def _shape_moments(shape: Shape) -> tuple[float, np.ndarray, np.ndarray]:
    rot, at = _quat_wxyz_matrix(shape.tilt), np.asarray(shape.at, dtype=float)
    if shape.kind in ("box", "cyl"):
        if shape.kind == "box":
            vol = float(np.prod(shape.size))
            local = vol / 12 * np.diag(np.square(shape.size))
        else:
            r, h = shape.size[0], shape.size[2]
            vol = math.pi * r * r * h
            local = np.diag(_canon([(vol * r * r / 4, vol * r * r / 4, vol * h * h / 12)], shape.axis)[0])
        return vol, vol * at, rot @ local @ rot.T + vol * np.outer(at, at)
    if shape.kind == "mesh":
        vol, first, second = 0.0, np.zeros(3), np.zeros((3, 3))
        if shape.collide:
            for piece in shape.hulls:
                p, f = _unpack(piece)
                v, fi, se = _piece_moments(p @ rot.T + at, f)
                vol, first, second = vol + v, first + fi, second + se
        return vol, first, second
    raise ValueError(f"unknown shape kind {shape.kind!r}")


def shape_volume(shape: Shape) -> float:
    """Collider volume (m^3): exact for box and cyl, the sum of the convex pieces for a mesh."""
    return _shape_moments(shape)[0]


def mass_props(shapes, mass: float | None = None) -> tuple[float, np.ndarray, np.ndarray]:
    """(volume, centre of mass, 3x3 inertia about the centre of mass) of colliding shapes in one frame. The inertia is
    for unit density, or for ``mass`` spread evenly through the volume when given."""
    vol, first, second = 0.0, np.zeros(3), np.zeros((3, 3))
    for s in shapes:
        v, f, se = _shape_moments(s)
        vol, first, second = vol + v, first + f, second + se
    if vol <= 0:
        raise ValueError("mass_props: no colliding volume")
    com = first / vol
    cc = second - vol * np.outer(com, com)
    inertia = np.trace(cc) * np.eye(3) - cc
    if mass is not None:
        inertia = inertia * (float(mass) / vol)
    return vol, com, inertia


def shape_points(shape: Shape, n: int = 24) -> np.ndarray:
    """Points bounding the shape in its parent frame: box corners, two n-gon rims for a cyl (vertices on the radius),
    every hull vertex for a mesh (visual vertices when it does not collide)."""
    if shape.kind == "cyl":
        r, h = shape.size[0], shape.size[2]
        rim = np.vstack([_ring(r, n, w=-h / 2), _ring(r, n, w=h / 2)])
        return _canon(rim, shape.axis) @ _quat_wxyz_matrix(shape.tilt).T + np.array(shape.at)
    return _shape_corners(shape)


def shape_aabb(shape: Shape, joint: Joint | None = None, value: float = 0.0, mech: Mech | None = None,
               values: dict[str, float] | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(low, high) of the shape in the Mech frame; give the joint it rides on and that joint's value. For a chained
    joint also give its ``mech`` and the ``values`` of the joints it rides on (rest when left out)."""
    pts = shape_points(shape)
    if joint is not None:
        if mech is None:
            if joint.parent is not None:
                raise ValueError(f"shape_aabb: joint {joint.name} rides on {joint.parent!r}: pass mech= (and values=)")
            rot, origin = _joint_pose(joint, value)
        else:
            if joint not in mech.joints:
                raise ValueError(f"shape_aabb: joint {joint.name} is not a joint of mech {mech.name}")
            rot, origin = _body_pose(mech, joint, {**(values or {}), joint.name: value})
        pts = pts @ rot.T + origin
    return pts.min(0), pts.max(0)


def _local_points(spec, values) -> np.ndarray:
    pts = [shape_points(s) for s in spec.shapes]
    for joint in spec.joints:
        rot, origin = _body_pose(spec, joint, values)
        pts += [shape_points(s) @ rot.T + origin for s in joint.shapes]
    return np.vstack(pts)


def _edge_dirs(pts: np.ndarray, faces) -> np.ndarray:
    seen, out = set(), []
    for f in faces:
        for i in range(len(f)):
            a, b = f[i], f[(i + 1) % len(f)]
            key = (min(a, b), max(a, b))
            if key not in seen:
                seen.add(key)
                e = pts[b] - pts[a]
                out.append(e / np.linalg.norm(e))
    return np.array(out)


def _convex_parts(shape: Shape, pose) -> list[dict]:
    """The shape's colliders in the frame of ``pose`` as convex parts: polytopes (points, face normals, edge directions)
    or exact cylinders (centre, axis, radius, half height)."""
    if pose is None:
        rot, origin = np.eye(3), np.zeros(3)
    elif isinstance(pose[0], Joint):
        if pose[0].parent is not None:
            raise ValueError(f"gap: joint {pose[0].name} rides on {pose[0].parent!r}; a (joint, value) pose is only "
                             "for a joint on the base: pass mech.joint_pose(name, values)")
        rot, origin = _joint_pose(pose[0], float(pose[1]))
    else:
        rot, origin = np.asarray(pose[0], dtype=float), np.asarray(pose[1], dtype=float)
    tilt = rot @ _quat_wxyz_matrix(shape.tilt)
    at = rot @ np.asarray(shape.at, dtype=float) + origin
    if shape.kind == "cyl":
        a = tilt @ _canon([(0.0, 0.0, 1.0)], shape.axis)[0]
        return [{"cyl": True, "c": at, "a": a, "r": shape.size[0], "h": shape.size[2] / 2}]
    if shape.kind == "box":
        signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], dtype=float)
        pts = signs * np.array(shape.size) / 2 @ tilt.T + at
        return [{"cyl": False, "p": pts, "n": np.vstack([tilt.T, -tilt.T]), "e": tilt.T.copy()}]
    if shape.kind == "mesh":
        if not shape.collide:
            return []
        out = []
        for piece in shape.hulls:
            p, f = _unpack(piece)
            p = p @ tilt.T + at
            nrm = np.array([_face_area_vector(p, g) for g in f])
            nrm /= np.linalg.norm(nrm, axis=1)[:, None]
            out.append({"cyl": False, "p": p, "n": nrm, "e": _edge_dirs(p, f)})
        return out
    raise ValueError(f"unknown shape kind {shape.kind!r}")


def _support(part: dict, dirs: np.ndarray) -> np.ndarray:
    """max over the part of x . n, for each row n of ``dirs``."""
    if part["cyl"]:
        ad = dirs @ part["a"]
        return dirs @ part["c"] + part["h"] * np.abs(ad) + part["r"] * np.sqrt(np.clip(1 - ad * ad, 0, None))
    return (part["p"] @ dirs.T).max(0)


def _centre(part: dict) -> np.ndarray:
    return part["c"] if part["cyl"] else part["p"].mean(0)


def _pair_axes(a: dict, b: dict) -> np.ndarray:
    axes = [np.eye(3), (_centre(b) - _centre(a))[None]]
    dirs = []
    for part in (a, b):
        if part["cyl"]:
            dirs.append(part["a"][None])
        else:
            axes.append(part["n"])
            dirs.append(part["e"])
    da, db = dirs
    axes.append(np.cross(da[:, None, :], db[None, :, :]).reshape(-1, 3))
    for cy, other in ((a, b), (b, a)):
        if cy["cyl"]:  # radial directions from the cylinder axis toward the other part's points
            pts = other["p"] if not other["cyl"] else np.vstack([other["c"], other["c"] + other["a"] * other["h"],
                                                                 other["c"] - other["a"] * other["h"]])
            w = pts - cy["c"]
            axes.append(w - np.outer(w @ cy["a"], cy["a"]))
    ax = np.vstack(axes)
    ln = np.linalg.norm(ax, axis=1)
    return ax[ln > 1e-9] / ln[ln > 1e-9, None]


def gap(shape_a: Shape, pose_a, shape_b: Shape, pose_b) -> float:
    """Signed separation between two shapes' colliders (m): positive is clearance, negative is overlap depth.

    Each pose is None (the Mech frame), (rotation 3x3, origin) or (joint, value) for a shape riding on a joint on the
    base; for a chained joint pass ``mech.joint_pose(name, values)``, which is a (rotation, origin) pose. The
    value is the largest separating-axis gap over face normals, edge-pair cross products and cylinder axes, taken
    between every pair of convex pieces; it never exceeds the true distance, so it is safe for clearance checks.
    A native cyl is treated as an exact cylinder, so a coaxial pin of radius r in bore(radius=R) gives R - r."""
    pa, pb = _convex_parts(shape_a, pose_a), _convex_parts(shape_b, pose_b)
    if not pa or not pb:
        raise ValueError("gap: both shapes must collide")
    box_a = [(-_support(p, -np.eye(3)), _support(p, np.eye(3))) for p in pa]
    box_b = [(-_support(p, -np.eye(3)), _support(p, np.eye(3))) for p in pb]
    best = math.inf
    order = sorted(((float(np.max(np.maximum(lb - ha, la - hb))), i, j)
                    for i, (la, ha) in enumerate(box_a) for j, (lb, hb) in enumerate(box_b)))
    for bound, i, j in order:
        if bound >= best:
            break
        axes = _pair_axes(pa[i], pb[j])
        sep = np.maximum(-_support(pb[j], -axes) - _support(pa[i], axes),
                         -_support(pa[i], -axes) - _support(pb[j], axes))
        best = min(best, float(sep.max()))
    return best


PHYSICS_DT = 1.0 / 240.0  # the yam_arena physics step (s): sim dt 0.0041667 in every Isaac run so far
GRAVITY = 9.81


def contact_offset(shape: Shape) -> float:
    """The contact offset the simulator authors for ``shape`` (m): its ``contact`` override, else 0.0015."""
    return DEFAULT_CONTACT if shape.contact is None else float(shape.contact)


def max_drop(shape_a: Shape, shape_b: Shape, dt: float = PHYSICS_DT) -> float:
    """Highest free fall (m) after which the simulator still sees contact between the two shapes before they overlap.

    Rest offsets are 0, so PhysX makes a contact only once the gap is under the sum of the two contact offsets, and a
    part falling from rest through h arrives at v = sqrt(2 g h), moving v * dt per physics step. When that step is
    longer than the summed offsets the first contact can appear already overlapped and the solver throws the part out.
    Kit smoke run 2026-11-12: a 40 g pin with 0.5 mm offsets on both parts fell 19.6 mm onto its bore's lead in (a 2.6 mm
    step against 1.0 mm of offsets) and ended upside down beside the bore. Default offsets allow about 26 mm, 0.5 mm
    offsets on both parts about 2.9 mm. Find the fall with gap() from the start pose down to the first contact."""
    c = contact_offset(shape_a) + contact_offset(shape_b)
    return (c / dt) ** 2 / (2 * GRAVITY)


# -- building it in Isaac Sim ------------------------------------------------------------------------------------------

_MECHS: dict[str, Mech] = {}


def _author(prim_path: str, cfg: Any, translation=None, orientation=None, **_: Any):
    """Write the Mech to USD: an Xform holding a base body and one body per joint, all siblings under it. A chained
    joint's body0 is its parent joint's body; every body starts at its rest pose in the Mech frame."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, Vt

    from yam_arena.station import linear_rgb

    spec = _MECHS[cfg.mech_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    looks: dict[Vec3, str] = {}
    glows: dict[Vec3, str] = {}
    lamps: dict[str, dict[str, str]] = {}  # lamp -> state ("off" for the unlit lens) -> lens prim path

    def glow(color: Vec3) -> str:
        """A lit lens: its colour, also emissive (as the accepted lamp and flash markers author it)."""
        if color not in glows:
            path = f"{prim_path}/Looks/glow{len(glows)}"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), emissive_color=_glow_rgb(color),
                                              roughness=0.4)
            mat.func(path, mat)
            glows[color] = path
        return glows[color]

    def look(color: Vec3) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/color{len(looks)}"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4)
            mat.func(path, mat)
            looks[color] = path
        return looks[color]

    def material(tag: str, mu: float) -> str:
        path = f"{prim_path}/Looks/{tag}"
        mat = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu)
        mat.func(path, mat)
        return path

    def mesh_prim(path: str, piece):
        pts, counts, idx, normals = _usd_mesh_arrays(piece)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        lo, hi = np.min(pts, axis=0), np.max(pts, axis=0)
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))]))
        if normals is not None:
            m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in normals]))
            m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        return m

    def add_mesh(path: str, shape: Shape, phys: str) -> None:
        """A visible smooth mesh plus hidden convex-hull collision pieces under one Xform."""
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*shape.at))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*shape.tilt))
        if shape.visible:
            mesh_prim(f"{path}/visual", shape.visual)
            bind_visual_material(f"{path}/visual", look(shape.color))
        if shape.collide:
            show = os.environ.get("MECHKIT_SHOW_HULLS") == "1"
            for k, piece in enumerate(shape.hulls):
                hp = f"{path}/hull{k:02d}"
                h = mesh_prim(hp, piece)
                if not show:
                    h.CreatePurposeAttr(UsdGeom.Tokens.guide)
                prim = h.GetPrim()
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
                cv = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
                cv.CreateHullVertexLimitAttr(HULL_VERTEX_LIMIT)
                cv.CreateMinThicknessAttr(1e-5)
                c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                c.CreateContactOffsetAttr(DEFAULT_CONTACT if shape.contact is None else shape.contact)
                c.CreateRestOffsetAttr(0.0)
                bind_physics_material(hp, phys)
                if show:
                    bind_visual_material(hp, look(_TINTS[k % len(_TINTS)]))

    def add_shape(body: str, i: int, shape: Shape, phys: str) -> None:
        path = f"{body}/shape{i}"
        if shape.kind == "mesh":
            add_mesh(path, shape, phys)
            return
        if shape.kind == "cyl":
            geom = UsdGeom.Cylinder.Define(stage, path)
            geom.CreateRadiusAttr(shape.size[0])
            geom.CreateHeightAttr(shape.size[2])
            geom.CreateAxisAttr(shape.axis.upper())
            scale = Gf.Vec3f(1.0, 1.0, 1.0)
        elif shape.kind == "box":
            geom = UsdGeom.Cube.Define(stage, path)
            geom.CreateSizeAttr(1.0)
            scale = Gf.Vec3f(*shape.size)
        else:
            raise ValueError(f"unknown shape kind {shape.kind!r}")
        geom.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*shape.at))
        tw, tx, ty, tz = shape.tilt
        geom.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(tw, tx, ty, tz))
        geom.AddScaleOp().Set(scale)
        prim = geom.GetPrim()
        if shape.collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            contact.CreateContactOffsetAttr(DEFAULT_CONTACT if shape.contact is None else shape.contact)
            contact.CreateRestOffsetAttr(0.0)
        if not shape.visible:
            geom.CreatePurposeAttr(UsdGeom.Tokens.guide)
        bind_visual_material(path, glow(shape.color) if shape.lit else look(shape.color))
        if shape.collide:
            bind_physics_material(path, phys)
        if shape.lamp:  # a lamp lens: lit lenses start hidden, set_lamp shows one at a time
            lamps.setdefault(shape.lamp, {})[shape.lit or "off"] = path
            if shape.lit:
                UsdGeom.Imageable(prim).MakeInvisible()

    def add_body(name: str, at, mass: float | None, shapes=()) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*at))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        if mass is not None:
            UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
            mapi = UsdPhysics.MassAPI.Apply(xf.GetPrim())
            mapi.CreateMassAttr(float(mass))
            if not shapes and name != "base":  # a shapeless thread carrier: no collider, so give its inertia
                mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-6, 1e-6, 1e-6))
                mapi.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            if any(s.kind == "mesh" for s in shapes):  # box and cyl bodies author nothing new
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim())
                rb.CreateMaxDepenetrationVelocityAttr(1.0)
                rb.CreateSolverPositionIterationCountAttr(16)
                rb.CreateSolverVelocityIterationCountAttr(4)
        return path

    fixed = spec.mass is None
    base_mass = (spec.mass if not fixed else (1.0 if spec.joints else None))
    base = add_body("base", (0.0, 0.0, 0.0), base_mass, spec.shapes)
    phys = material("friction", spec.friction)
    for i, shape in enumerate(spec.shapes):
        add_shape(base, i, shape, phys)

    if spec.joints:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(base))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(base))
        art.CreateEnabledSelfCollisionsAttr(True)
        iterations = articulation_iterations(spec)
        if iterations is not None:  # None authors nothing: exactly what mechkit always wrote
            art.CreateSolverPositionIterationCountAttr(iterations)
    if fixed and spec.joints:
        anchor = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/anchor")
        anchor.CreateBody1Rel().SetTargets([Sdf.Path(base)])
        ax, ay, az = translation or (0.0, 0.0, 0.0)
        anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(ax, ay, az))
        anchor.CreateLocalRot0Attr().Set(Gf.Quatf(w, x, y, z))
        anchor.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        anchor.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    for joint in spec.joints:
        # a joint on the base starts at joint.at, exactly as always; a chained one at its rest origin in the Mech frame
        # (rest rotations are all identity), as plain floats
        at = joint.at if joint.parent is None else tuple(float(v) for v in _body_pose(spec, joint, {})[1])
        body = add_body(joint.name, at, joint.mass, joint.shapes)
        mu = spec.friction if joint.friction is None else joint.friction
        own = phys if joint.friction is None else material(f"friction_{joint.name}", mu)
        for i, shape in enumerate(joint.shapes):
            add_shape(body, i, shape, own)
        kind = UsdPhysics.PrismaticJoint if joint.kind == "slide" else UsdPhysics.RevoluteJoint
        j = kind.Define(stage, f"{prim_path}/{joint.name}_joint")
        # body0 is the body this joint rides on; localPos0 (joint.at) is in that body's frame
        j.CreateBody0Rel().SetTargets([Sdf.Path(base if joint.parent is None else f"{prim_path}/{joint.parent}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body)])
        j.CreateAxisAttr(joint.axis.upper())
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*joint.at))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        degrees = joint.kind == "hinge"
        if joint.thread is None:  # a threaded hinge spins free, as the accepted cap's spin joint; the carrier stops it
            j.CreateLowerLimitAttr(math.degrees(joint.low) if degrees else joint.low)
            j.CreateUpperLimitAttr(math.degrees(joint.high) if degrees else joint.high)
        UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if degrees else "linear")
    if lamps:
        _LAMP_PRIMS.setdefault(spec.name, lamps)  # the first one authored: env 0
        for states in lamps.values():
            for state, path in states.items():
                _LAMP_SHOWN.setdefault(path, state == "off")
    return stage.GetPrimAtPath(prim_path)


def _arena_object(spec: Mech):
    """The Mech as an Arena object; Isaac Lab's implicit actuators carry the springs."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class MechSpawnerCfg(SpawnerCfg):
        func: object = clone(_author)
        mech_name: str = ""

    class MechObject(Object):
        def _generate_articulation_cfg(self):
            def spring(j: Joint) -> ImplicitActuatorCfg:
                kw = _actuator_kwargs(j, spec)
                try:
                    return ImplicitActuatorCfg(**kw)
                except TypeError:  # an Isaac Lab without dynamic_friction: keep the static hold
                    if "dynamic_friction" not in kw:
                        raise
                    kw.pop("dynamic_friction")
                    print(f"mechkit: joint {j.name}: this Isaac Lab has no dynamic_friction; using hold only",
                          flush=True)
                    return ImplicitActuatorCfg(**kw)

            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg,
                                  actuators={j.name: spring(j) for j in spec.joints})
            return self._add_initial_pose_to_cfg(cfg)

    _MECHS[spec.name] = spec
    if spec.joints:
        kind = ObjectType.ARTICULATION
    else:
        kind = ObjectType.RIGID if spec.mass is not None else ObjectType.BASE
    half = spec.yaw / 2
    return MechObject(name=spec.name, object_type=kind, spawner_cfg=MechSpawnerCfg(mech_name=spec.name),
                      initial_pose=Pose(position_xyz=tuple(spec.pos),
                                        rotation_xyzw=(0.0, 0.0, math.sin(half), math.cos(half))))


def _to_numpy(value) -> np.ndarray:
    value = getattr(value, "torch", value)
    return value.detach().cpu().numpy().astype(np.float64)


def _mech_state(environment, spec: Mech) -> MechState:
    base = environment.unwrapped
    values: dict[str, float] = {}
    origin = np.array(spec.pos, dtype=float)
    half = spec.yaw / 2
    quat = np.array([0.0, 0.0, math.sin(half), math.cos(half)])
    lin, ang = np.zeros(3), np.zeros(3)
    entity = base.scene[spec.name] if spec.name in {*base.scene.rigid_objects, *base.scene.articulations} else None
    if entity is not None:
        env0 = _to_numpy(base.scene.env_origins)[0]
        pose = _to_numpy(entity.data.root_link_pose_w)[0]
        origin, quat = pose[:3] - env0, pose[3:]
        lin = _to_numpy(entity.data.root_link_lin_vel_w)[0]
        ang = _to_numpy(entity.data.root_link_ang_vel_w)[0]
        if spec.joints:
            pos = _to_numpy(entity.data.joint_pos)[0]
            values = {n.removesuffix("_joint"): float(v) for n, v in zip(entity.joint_names, pos, strict=True)}
            for nut, _ in _threads(spec):  # a threaded hinge reads as the kit's unwrapped turn angle
                live = _SCREWS.get((spec.name, nut.name))
                if live is not None and nut.name in values:
                    values[nut.name] = float(live["theta"])
    corners = _local_corners(spec, values) @ _quat_xyzw_matrix(quat).T + origin
    low, high = corners.min(0), corners.max(0)
    return MechState(pos=(low + high) / 2, quat_xyzw=quat, lin_vel=lin, ang_vel=ang, low=low, high=high,
                     joints=values, origin=origin, **_state_extras(spec, values, origin, quat))


def _state_extras(spec: Mech, values, origin, quat_xyzw) -> dict:
    """The MechState fields beyond the pose and joint values: ``bodies``, ``screws`` (when every screw joint has a
    value) and ``lamps``. Shared by the simulator and the MuJoCo replay."""
    threads = _threads(spec)
    have = all(n.name in values and c.name in values for n, c in threads)
    return {"bodies": _world_bodies(spec, values, origin, quat_xyzw),
            "screws": {n.name: _screw_state(n, c, values) for n, c in threads} if have else {},
            "lamps": {name: _LAMP_STATE.get((spec.name, name), "off") for name in lamp_states(spec)}}


def _patch_scene(scene) -> None:
    plain_object, plain_states, plain_make = scene.task_object, scene.object_states, scene.make_env

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, Mech) else plain_object(spec)

    def object_states(environment, names):
        mechs = {n: _MECHS[n] for n in names if n in _MECHS}
        out = plain_states(environment, [n for n in names if n not in mechs])
        out.update({n: _mech_state(environment, spec) for n, spec in mechs.items()})
        return out

    def make_env(*args, **kwargs):
        if contacts_wanted():  # before the simulator is configured, as the accepted live-wire kit does it
            _enable_contact_processing()
        env = plain_make(*args, **kwargs)
        orig_reset = env.reset

        def reset(*r_args, **r_kwargs):
            # no screw or gear writes and no contact reads from inside the reset; _reset_live re-arms
            _ARMED.update(screws=False, gears=False, contacts=False)
            res = orig_reset(*r_args, **r_kwargs)
            _ENV["env"] = env
            _reset_live(env)
            for fn in list(_RESET_HOOKS):
                fn(env)
            return res

        env.reset = reset
        return env

    scene.task_object, scene.object_states, scene.make_env = task_object, object_states, make_env


# -- run time: holds, detents, on_step and finger release ---------------------------------------------------------------


def _live_mechs(environment):
    """(name, spec, asset) for every jointed Mech in this env's scene."""
    scene = environment.unwrapped.scene
    names = set(scene.articulations)
    for name, spec in list(_MECHS.items()):
        if spec.joints and name in names:
            yield name, spec, scene[name]


def _joint_id(asset, joint: str) -> int:
    names = list(asset.joint_names)
    key = f"{joint}_joint"
    if key not in names:
        raise KeyError(f"mechkit: no joint {joint!r} in {names}")
    return names.index(key)


def _sim_tensor(asset, values):
    import torch

    return torch.tensor([list(values)], dtype=torch.float32, device=asset.device)


def _write_target(environment, mech: str, joint: str, value: float) -> None:
    asset = environment.unwrapped.scene[mech]
    i = _joint_id(asset, joint)
    v = _sim_tensor(asset, [float(value)])
    cmd = getattr(getattr(asset, "actuators", None), "target_command", None)
    if cmd is not None and hasattr(cmd, "set_position_index"):
        cmd.set_position_index(value=v, joint_ids=[i])
    else:
        asset.set_joint_position_target(v, joint_ids=[i])


def set_rest(environment, mech: str, joint: str, value: float) -> None:
    """Move a joint's spring rest (m or rad, measured like MechState.joints) at run time."""
    _write_target(environment, mech, joint, value)
    _LIVE[(mech, joint)] = float(value)


def set_hold(environment, mech: str, joint: str, static: float, moving: float | None = None) -> None:
    """Change a joint's hold (N or N m) at run time; ``moving`` defaults to 0.8 x ``static``."""
    static = _finite("set_hold: static", static)
    moving = HOLD_MOVING * static if moving is None else _finite("set_hold: moving", moving)
    if static < 0 or not 0.0 <= moving <= static:
        raise ValueError(f"set_hold: need 0 <= moving ({moving}) <= static ({static})")
    asset = environment.unwrapped.scene[mech]
    i = _joint_id(asset, joint)
    s, d = _sim_tensor(asset, [static]), _sim_tensor(asset, [moving])
    write = getattr(asset, "write_joint_friction_coefficient_to_sim_index", None)
    if write is None:
        write = asset.write_joint_friction_coefficient_to_sim
    try:
        write(joint_friction_coeff=s, joint_dynamic_friction_coeff=d, joint_ids=[i])
    except TypeError:  # older signature without the moving value
        write(joint_friction_coeff=s, joint_ids=[i])


def _detent_step(environment) -> None:
    for name, spec, asset in _live_mechs(environment):
        if not any(j.detents for j in spec.joints):
            continue
        q = _to_numpy(asset.data.joint_pos)[0]
        values = {n.removesuffix("_joint"): float(v) for n, v in zip(asset.joint_names, q, strict=True)}
        for joint, rest in _detent_plan(name, spec, values, _LIVE):
            set_rest(environment, name, joint, rest)


def _reset_live(environment) -> None:
    """Forget every rest a detent wrote and put detent joints' springs back at 0; put every screw and gear back at
    rest and re-arm its coupling; turn every lamp off; clear every contact watch; restart the step clock."""
    _LIVE.clear()
    for name, spec, _ in _live_mechs(environment):
        for j in spec.joints:
            if j.detents:
                set_rest(environment, name, j.name, 0.0)
    _CLOCK.update(steps=0, seconds=0.0)
    _reset_screws(environment)
    _reset_gears(environment)
    _reset_lamps()
    _reset_contacts(environment)


# -- run time: screw threads ----------------------------------------------------------------------------------------------


def _write_joint_rest(environment, mech: str, joints) -> None:
    """Put these joints at 0 with no velocity in the simulator (an Arena reset moves an articulation by its root only)."""
    asset = environment.unwrapped.scene[mech]
    ids = [_joint_id(asset, j) for j in joints]
    zero = _sim_tensor(asset, [0.0] * len(ids))
    pos = getattr(asset, "write_joint_position_to_sim_index", None) or asset.write_joint_position_to_sim
    vel = getattr(asset, "write_joint_velocity_to_sim_index", None) or asset.write_joint_velocity_to_sim
    pos(position=zero, joint_ids=ids)
    vel(velocity=zero, joint_ids=ids)


def _reset_screws(environment) -> None:
    _SCREWS.clear()
    armed = False
    for name, spec, _ in _live_mechs(environment):
        for nut, c in _threads(spec):
            _write_joint_rest(environment, name, (c.name, nut.name))
            _write_target(environment, name, c.name, 0.0)
            if nut.thread.tighten > 0:
                set_hold(environment, name, nut.name, nut.hold, nut.moving_hold)
            _SCREWS[(name, nut.name)] = {"theta": 0.0, "last": 0.0, "hold": float(nut.hold)}
            armed = True
    if armed:
        _hook_physics(environment)
        _ARMED["screws"] = True


def _hook_physics(environment) -> None:
    """Run the screw and gear couplings and the contact reads before every physics step: Isaac Lab calls the scene's
    write_data_to_sim before each 240 Hz substep, as the accepted cord and centrifuge kits hook it. Wraps once (its own
    flag)."""
    isc = environment.unwrapped.scene
    if getattr(isc, "_mechkit_physics", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _screw_physics(environment)
        _gear_physics(environment)
        _contact_poll(environment)
        return write(*a, **k)

    isc.write_data_to_sim, isc._mechkit_physics = write_data_to_sim, True


def _screw_physics(environment) -> None:
    """Every armed screw: unwrap the hinge's reading, write the carrier's target from the law, and move the hinge's
    hold when the turns past the seat change it."""
    if not _ARMED["screws"]:
        return
    for name, spec, asset in _live_mechs(environment):
        threads = _threads(spec)
        if not threads:
            continue
        q = _to_numpy(asset.data.joint_pos)[0]
        for nut, c in threads:
            live = _SCREWS.get((name, nut.name))
            if live is None:
                continue
            live["theta"], live["last"] = screw_unwrap(live["theta"], live["last"], float(q[_joint_id(asset, nut.name)]))
            target, past = _screw_law(nut.thread, c, live["theta"])
            _write_target(environment, name, c.name, target)
            if nut.thread.tighten > 0:
                hold = screw_hold(nut, past)
                if abs(hold - live["hold"]) >= SCREW_HOLD_STEP or (past == 0.0 and live["hold"] != nut.hold):
                    ratio = nut.moving_hold / nut.hold if nut.hold > 0 else HOLD_MOVING
                    set_hold(environment, name, nut.name, hold, ratio * hold)
                    live["hold"] = hold


# -- run time: gears ------------------------------------------------------------------------------------------------------


def _physics_dt(environment) -> float:
    dt = getattr(environment.unwrapped, "physics_dt", None)
    try:
        dt = float(dt)
    except (TypeError, ValueError):
        return PHYSICS_DT
    return dt if math.isfinite(dt) and dt > 0 else PHYSICS_DT


def _write_vel_target(environment, mech: str, joint: str, value: float) -> None:
    asset = environment.unwrapped.scene[mech]
    i = _joint_id(asset, joint)
    v = _sim_tensor(asset, [float(value)])
    cmd = getattr(getattr(asset, "actuators", None), "target_command", None)
    if cmd is not None and hasattr(cmd, "set_velocity_index"):
        cmd.set_velocity_index(value=v, joint_ids=[i])
    else:
        asset.set_joint_velocity_target(v, joint_ids=[i])


def _reset_gears(environment) -> None:
    """Put every geared joint at its rest by the law (drivers at 0, followers at ratio x driver + offset) with no
    velocity, its drive target there, and arm the coupling."""
    armed = False
    for name, spec, asset in _live_mechs(environment):
        links = _gear_links(spec)
        if not links:
            continue
        rest = _gear_rest(spec, links)
        names = sorted(rest)
        ids = [_joint_id(asset, n) for n in names]
        pos = getattr(asset, "write_joint_position_to_sim_index", None) or asset.write_joint_position_to_sim
        vel = getattr(asset, "write_joint_velocity_to_sim_index", None) or asset.write_joint_velocity_to_sim
        pos(position=_sim_tensor(asset, [float(rest[n]) for n in names]), joint_ids=ids)
        vel(velocity=_sim_tensor(asset, [0.0] * len(ids)), joint_ids=ids)
        for n in names:
            _write_target(environment, name, n, float(rest[n]))
            _write_vel_target(environment, name, n, 0.0)
        armed = True
    if armed:
        _hook_physics(environment)
        _ARMED["gears"] = True


def _gear_physics(environment) -> None:
    """Every armed gear: from the joints' positions and velocities now, write each geared joint's position and
    velocity drive targets (its own spring toward its rest, detent rests included, plus every gear it is in)."""
    if not _ARMED["gears"]:
        return
    dt = _physics_dt(environment)
    for name, spec, asset in _live_mechs(environment):
        links = _gear_links(spec)
        if not links:
            continue
        jn = [n.removesuffix("_joint") for n in asset.joint_names]
        q = dict(zip(jn, map(float, _to_numpy(asset.data.joint_pos)[0]), strict=True))
        v = dict(zip(jn, map(float, _to_numpy(asset.data.joint_vel)[0]), strict=True))
        by = {j.name: j for j in spec.joints}
        names = {n for g in links for n in (g.follower.name, g.driver.name)}
        own = {n: (by[n].spring, by[n].damping) for n in names}
        rest = {n: _LIVE.get((name, n), 0.0) for n in names}
        for n, (_, target, _, vel) in sorted(_gear_drive(links, own, q, v, rest, dt).items()):
            _write_target(environment, name, n, float(target))
            _write_vel_target(environment, name, n, float(vel))


# -- run time: contact watches ------------------------------------------------------------------------------------------

_CONTACT: dict[str, Any] = {"env": None, "views": {}, "fingers": {}, "acc": {}, "net": {}, "proximity": set(),
                            "error": None, "live": False}


def _enable_contact_processing() -> None:
    """Isaac Lab turns PhysX contact processing off unless a ContactSensor exists (the inspect embodiment builds
    none); switch it back on, as Isaac Lab's own ContactSensor does."""
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception as exc:  # noqa: BLE001 - outside Kit there is nothing to switch
        print(f"mechkit: could not enable contact processing: {exc!r}", flush=True)


def _physics_sim_view(environment):
    try:
        from isaaclab_physx.physics import PhysxManager

        view = PhysxManager.get_physics_sim_view()
    except ImportError:
        view = None
    if view is None:
        view = getattr(getattr(environment.unwrapped, "sim", None), "physics_sim_view", None)
    if view is None:
        raise RuntimeError("no PhysX simulation view")
    return view


def _arm_link_paths(environment, stage) -> dict[str, list[str]]:
    """Each arm's rigid-body link prim paths in env 0: from the articulation's PhysX view, else from the stage."""
    out: dict[str, list[str]] = {}
    scene = environment.unwrapped.scene
    for arm, key in ARM_ASSETS.items():
        paths = []
        try:
            lp = scene[key].root_physx_view.link_paths
            paths = list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        except Exception:  # noqa: BLE001 - fall back to the stage
            paths = []
        if not paths:
            from pxr import UsdPhysics

            paths = [p.GetPath().pathString for p in stage.Traverse()
                     if "/env_0/" in p.GetPath().pathString and f"/{ARM_PRIMS[arm]}/" in p.GetPath().pathString
                     and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        out[arm] = [str(p) for p in paths]
    return out


def _bind_contacts(environment) -> None:
    """Build the contact views once per env: one per watch (watched pads x the target's rigid bodies) and one per
    arm (its finger links, net force). A watch whose target has no rigid body uses the proximity fallback; any other
    failure is printed once and every watch falls back to proximity, with the finger sense unavailable (nan)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    _enable_contact_processing()
    c = _CONTACT
    c.update(env=None, views={}, fingers={}, proximity=set(), error=None)
    stage = get_current_stage()
    prims = [p for p in stage.Traverse() if "/env_0/" in p.GetPath().pathString]
    rigid = [p.GetPath().pathString for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    every = [p.GetPath().pathString for p in prims]
    targets = {}
    for t in _TOUCHES.values():
        paths = _match_bodies(rigid, t.target, t.bodies)
        if not paths:
            if not _match_bodies(every, t.target):
                raise ValueError(f"watch_touch {t.name}: no object {t.target!r} in the scene")
            if t.bodies:
                raise ValueError(f"watch_touch {t.name}: object {t.target} has no rigid bodies {t.bodies}")
            c["proximity"].add(t.name)
            print(f"mechkit: contact watch {t.name}: {t.target} has no rigid body (fixed); using the proximity "
                  f"fallback ({PROXIMITY_TOL * 1000:g} mm to its bounding box)", flush=True)
            continue
        targets[t.name] = paths
    try:
        sv = _physics_sim_view(environment)
        links = _arm_link_paths(environment, stage)
        for name, paths in targets.items():
            t = _TOUCHES[name]
            pads = _pad_paths(links, t.arms, t.links)
            sensors = [pads[p] for p in t.pads]
            view = sv.create_rigid_contact_view(sensors, filter_patterns=[paths] * len(sensors),
                                                max_contact_data_count=64 * len(sensors) * len(paths))
            c["views"][name] = (view, t.pads, len(paths))
        if _FINGER_SENSE["on"]:
            for arm in ARM_ASSETS:
                fp = _pad_paths(links, (arm,), FINGER_LINKS)
                c["fingers"][arm] = sv.create_rigid_contact_view([fp[f"{arm}/{k}"] for k in FINGER_LINKS])
        print(f"mechkit: contact views: {sorted(c['views'])} measured, {sorted(c['proximity'])} by proximity, finger "
              f"sense {'on' if c['fingers'] else 'off'}", flush=True)
    except Exception as exc:  # noqa: BLE001 - degrade to the stated fallback, loudly
        c["error"] = repr(exc)
        c["proximity"] |= set(targets)
        c["views"], c["fingers"] = {}, {}
        print(f"mechkit: contact views unavailable ({exc!r}); every watch uses the proximity fallback and "
              "finger_force() reads nan", flush=True)
    c["env"] = environment


def _reset_contacts(environment) -> None:
    """Clear every watch's episode record and arm the reads; build the views on the first reset of an env."""
    c = _CONTACT
    c["acc"] = {n: _new_touch_acc(t) for n, t in _TOUCHES.items()}
    c["net"] = {}
    if not contacts_wanted():
        c["live"] = False
        return
    if c["env"] is not environment:
        _bind_contacts(environment)
    for n, acc in c["acc"].items():
        acc["source"] = "proximity" if n in c["proximity"] else "physx"
    c["live"] = True
    _hook_physics(environment)
    _ARMED["contacts"] = True


def _arm_face_points(environment) -> dict[str, list[np.ndarray]]:
    """Each arm's pad face points (the finger_points pads with no 6 mm outset), world relative to the env origin."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _to_numpy(base.scene.env_origins)[0]
    out = {}
    for arm, key in ARM_ASSETS.items():
        robot = base.scene[key]
        pose = _to_numpy(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        jp, jn = _to_numpy(robot.data.joint_pos)[0], list(robot.joint_names)
        half_gap = max(abs(float(jp[jn.index(n)])) for n in ("joint7", "joint8") if n in jn)
        out[arm] = _pad_points(pose, half_gap - 0.006, origin, TCP_OFFSET, PAD_TIP_PAST_TCP)
    return out


def proximity_hits(face_points: dict, part, t: Touch, tol: float = PROXIMITY_TOL) -> dict[str, bool]:
    """The proximity fallback for one watch: {pad: True} for every pad of an arm whose pad face points come within
    ``tol`` of the part's world bounding box. It cannot tell the two fingers of an arm apart, so both count."""
    out = {}
    for arm in t.arms:
        near = finger_clearance(face_points[arm], part) <= tol
        for link in t.links:
            out[f"{arm}/{link}"] = near
    return out


def _contact_poll(environment, control: bool = False) -> None:
    """Fold the latest contacts into every watch and the finger sense: measured views every physics step (and once
    after each control step), the proximity fallback once per control step."""
    if not _ARMED["contacts"]:
        return
    c, step = _CONTACT, int(_CLOCK["steps"])
    dt = _physics_dt(environment)
    try:
        for name, (view, pads, nf) in c["views"].items():
            _touch_update(c["acc"][name], _TOUCHES[name], step,
                          forces=_pad_forces(view.get_contact_force_matrix(dt=dt), pads, nf))
        for arm, view in c["fingers"].items():
            c["net"][arm] = _net_force(view.get_net_contact_forces(dt=dt))
        if control and c["proximity"]:
            from yam_arena.scene import object_states

            faces = _arm_face_points(environment)
            for name in sorted(c["proximity"]):
                t = _TOUCHES[name]
                part = object_states(environment, [t.target])[t.target]
                _touch_update(c["acc"][name], t, step, hits=proximity_hits(faces, part, t))
    except Exception as exc:  # noqa: BLE001 - one report, then the reads stop rather than lie
        if c["error"] is None:
            c["error"] = repr(exc)
            print(f"mechkit: contact read failed ({exc!r}); contact watches stop updating", flush=True)
        _ARMED["contacts"] = False


def touch_state(name: str, arm: str | None = None) -> TouchState:
    """What contact watch ``name`` (declared with watch_touch) saw since the last reset, over both arms or only
    ``arm``. Scorer side (score() or an on_step hook); the closed-loop demo asks through ``ctx.touch_state``. With
    no simulator it returns the mock_touch script (untouched unless set), with source "offline"."""
    t = _touch(name)
    c = _CONTACT
    if c["live"] and name in c["acc"]:
        return _touch_view(t, c["acc"][name], arm)
    return _touch_view(t, _TOUCH_MOCK.get(name) or _new_touch_acc(t), arm)


def finger_force(arm: str) -> float:
    """The net normal contact force (N) on the ``arm`` arm's finger links at the latest reading: the larger of its two
    fingers, from anything they touch. The robot's own fingertip contact sense (the YAM has no such sensor; say so in
    the fidelity note). Needs watch_fingers() or a watch_touch(); nan when the simulator could not build the view.
    Offline it returns the mock_finger_force script (0 unless set)."""
    if arm not in ARM_ASSETS:
        raise ValueError(f"finger_force: arm must be one of {tuple(ARM_ASSETS)}, got {arm!r}")
    if not contacts_wanted():
        raise RuntimeError("finger_force: declare watch_fingers() (or a watch_touch) at task import")
    c = _CONTACT
    if c["live"]:
        return float(c["net"].get(arm, 0.0)) if c["fingers"] else float("nan")
    return float(_FINGER_MOCK.get(arm, 0.0))


# -- run time: indicator lamps ---------------------------------------------------------------------------------------------


def _glow_rgb(color) -> tuple[float, float, float]:
    """Emissive colour of a lit lens: the colour made linear, times LAMP_GLOW, as plain floats."""
    return tuple(float(LAMP_GLOW * (c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)) for c in color)


def set_lamp(mech, name: str, state: str | None = "off") -> None:
    """Show lamp ``name`` of ``mech`` (the Mech, or its name once the simulator has built it) in ``state``: one of
    its lit states, or "off" / None. Call it from an on_step or on_reset hook (scorer side). In the simulator it
    shows that state's lens and hides the others (USD visibility, only where it changes); offline it only records the
    state, which MechState.lamps reports. An unknown Mech, lamp or state raises. Every reset turns every lamp off."""
    spec = mech if isinstance(mech, Mech) else _MECHS.get(mech)
    if spec is None:
        raise ValueError(f"set_lamp: no Mech {mech!r}; pass the Mech itself")
    states = lamp_states(spec)
    if name not in states:
        raise ValueError(f"set_lamp: mech {spec.name} has no lamp {name!r} (lamps: {sorted(states)})")
    state = "off" if state is None else state
    if state != "off" and state not in states[name]:
        raise ValueError(f"set_lamp: lamp {name} has no state {state!r} (states: off, {', '.join(states[name])})")
    _LAMP_STATE[(spec.name, name)] = state
    prims = _LAMP_PRIMS.get(spec.name, {}).get(name)
    if prims:
        _show_lamp(prims, state)


def _show_lamp(prims: dict[str, str], state: str) -> None:
    todo = [(path, st == state) for st, path in prims.items() if st != "off" and _LAMP_SHOWN.get(path) != (st == state)]
    if not todo:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    for path, want in todo:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"set_lamp: no lens prim at {path}")
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if want else img.MakeInvisible()
        _LAMP_SHOWN[path] = want


def _reset_lamps() -> None:
    _LAMP_STATE.clear()
    for lamps in _LAMP_PRIMS.values():
        for prims in lamps.values():
            _show_lamp(prims, "off")


def step_count() -> int:
    """Control steps run since the last reset (counted before each on_step round; offline 0)."""
    return int(_CLOCK["steps"])


def elapsed() -> float:
    """Seconds since the last reset, counted in control steps of the env's step_dt (offline 0). Use it with
    ``timetable`` for blink codes. Raises when the env gave no step_dt."""
    t = float(_CLOCK["seconds"])
    if math.isnan(t):
        raise RuntimeError("elapsed: the env has no step_dt; use step_count() and the task's control rate")
    return t


def finger_points(environment=None) -> list[np.ndarray]:
    """Points on both arms' finger pads (world, relative to the env origin): at the TCP and at the pad tips, 6 mm
    outside each pad face. Scorer side only (an on_step hook or score()); a demo must never read them."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    environment = environment if environment is not None else _ENV["env"]
    if environment is None:
        raise RuntimeError("finger_points: no live env (only available while the simulator runs)")
    base = environment.unwrapped
    origin = _to_numpy(base.scene.env_origins)[0]
    pts: list[np.ndarray] = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _to_numpy(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        jp, jn = _to_numpy(robot.data.joint_pos)[0], list(robot.joint_names)
        half_gap = max(abs(float(jp[jn.index(n)])) for n in ("joint7", "joint8") if n in jn)
        pts += _pad_points(pose, half_gap, origin, TCP_OFFSET, PAD_TIP_PAST_TCP)
    return pts


_RELEASE_WARNED: list[bool] = []


def released(part, clearance: float = 0.03, lo=None, hi=None, environment=None) -> bool:
    """True when every finger pad point of both arms is at least ``clearance`` m from ``part`` (an ObjectState or
    MechState from score()'s objects): from its world bounding box by default, or from the box ``lo`` .. ``hi`` in its
    own frame. Callable from score() or an on_step hook while the simulator runs. No live env (offline dry runs) or
    any error counts as not released, so a scorer can never pay for a grip it could not check."""
    environment = environment if environment is not None else _ENV["env"]
    if environment is None:
        return False
    try:
        return finger_clearance(finger_points(environment), part, lo, hi) >= clearance
    except Exception as exc:  # noqa: BLE001 - an unknown grip never counts as released
        if not _RELEASE_WARNED:
            _RELEASE_WARNED.append(True)
            print(f"mechkit: finger release check unavailable: {exc!r}", flush=True)
        return False


def _run_step(environment) -> None:
    _ENV["env"] = environment
    if environment is None:
        return
    _CLOCK["steps"] += 1
    _CLOCK["seconds"] += float(getattr(environment.unwrapped, "step_dt", math.nan))
    _detent_step(environment)
    _contact_poll(environment, control=True)
    for fn in list(_STEP_HOOKS):
        fn(environment)


def _patch_embodiment(ie) -> None:
    """Run detents and on_step hooks after every control step, before the task is scored. Wraps (never replaces)
    whatever other kits put there; its own flag keeps it to one wrap."""
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_mechkit_patched", False):
        return
    base_task_step = cls._task_step

    def _task_step(self, *a, **k):
        _run_step(getattr(self, "_env", None))
        return base_task_step(self, *a, **k)

    cls._task_step = _task_step
    cls._mechkit_patched = True


class _AfterImport(importlib.abc.MetaPathFinder):
    """Patches yam_arena.scene and yam_arena.inspect_embodiment as each loads; both import only once Isaac Sim is up."""

    TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def __init__(self) -> None:
        self._pending = set(self.TARGETS)

    def find_spec(self, fullname, path=None, target=None):
        if fullname not in self._pending:
            return None
        self._pending.discard(fullname)
        if not self._pending and self in sys.meta_path:
            sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        if spec is None or spec.loader is None:
            return spec
        run, patch = spec.loader.exec_module, self.TARGETS[fullname]

        def exec_module(module):
            run(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


def _install() -> None:
    finder = _AfterImport()
    for name, patch in finder.TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
            finder._pending.discard(name)
    if finder._pending and not any(isinstance(f, _AfterImport) for f in sys.meta_path):
        sys.meta_path.insert(0, finder)


_install()
