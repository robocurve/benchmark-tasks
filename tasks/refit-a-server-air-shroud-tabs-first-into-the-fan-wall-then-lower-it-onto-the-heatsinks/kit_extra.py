"""kit_extra: two helpers for task files built on ``mechkit``, in one file that a task folder copies (or links, as it
links ``mechkit.py``) next to its task file. Standard library only; nothing here needs the simulator.

- ``Events``: when named conditions first held for a given time, in the trial's seconds; ``before(a, b)`` and
  ``order()`` for a score by the order of events (the plug was out before the hand went in), ``report()`` for the
  task's report.
- ``ToggleSpec`` and ``Toggle``: a two-position switch with an over-centre snap (a rocker, a toggle lever, a slide
  switch) whose spring is worked out from the maker's snap force and travel, with a lamp that is lit while it is on.

Neither declares anything in mechkit. The task file declares its bodies and joints itself (a declaration made from
this module would make mechkit start a new registry: see ``mechkit._claim``) and calls the helpers from its own
``@mk.on_reset`` and ``@mk.on_step`` functions:

    import mechkit as mk
    import kit_extra as kx

    SNAP = kx.ToggleSpec("hinge", force=..., travel=math.radians(...), arm=..., source="<maker's page, address>")
    mk.body("rocker", [mk.box(...), mk.box(..., color=LIT, glow=0.6)], pos=..., mass=...)  # shape 1: the lamp
    mk.hinge("switch", "rocker", "strip", anchor=..., axis=..., limits=SNAP.limits, spring=SNAP.stiffness,
             rest=SNAP.rest(True), damping=SNAP.critical_damping(inertia))
    SWITCH = kx.Toggle(mk, "switch", SNAP, start_on=True, lamp=("rocker", 1))
    EVENTS = kx.Events({"switched off": 0.5, "plug out": 0.5, "hand in": 0.2})

    @mk.on_reset
    def start():
        SWITCH.reset()
        EVENTS.clear()

    @mk.on_step
    def step():
        SWITCH.step()
        EVENTS.note("switched off", not SWITCH.on, mk.now())
        ...
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

__version__ = "0.1"

TIME_TOL = 1e-9  # s: control-step times are sums of 1/20 s, so a hold of 0.3 s from 0.1 + 0.2 to 0.6 must count


class Events:
    """When named conditions first held, each for its own time, in seconds since the reset.

    ``holds`` maps each event's name to its hold time (s). ``note(name, holds, now)`` once per control step for each
    name. A condition has happened once it has held at every note from a time ``t0`` to a time at least its hold
    later; its time is then ``t0``, the start of that run, and stays so for the rest of the trial: a later run does
    not move it, and the condition may stop holding afterwards. A run that breaks off sooner does not count (a hand
    that only brushes past). With a hold of 0 the first note at which it holds is its time. A hold is measured
    between notes: at 20 notes a second, a hold of 0.5 s takes 11 notes in a row.
    """

    def __init__(self, holds: Mapping[str, float]) -> None:
        self.holds: dict[str, float] = {}
        for name, hold in holds.items():
            if not isinstance(name, str) or not name:
                raise ValueError(f"an event's name must be a non-empty string, got {name!r}")
            hold = float(hold)
            if not (math.isfinite(hold) and hold >= 0.0):
                raise ValueError(f"{name}: the hold must be a finite number of seconds, at least 0, got {hold!r}")
            self.holds[name] = hold
        self.clear()

    def clear(self) -> None:
        """Forget the trial: call it from the task's ``@mk.on_reset`` function."""
        self._since: dict[str, float | None] = dict.fromkeys(self.holds)  # name -> start of the run that holds now
        self._first: dict[str, tuple[float, float]] = {}  # name -> (start of its first run that held, end of that hold)
        self._last: dict[str, float] = {}  # name -> time of its last note

    def note(self, name: str, holds: bool, now: float) -> None:
        """Whether condition ``name`` holds at time ``now`` (s since the reset): once per control step."""
        hold = self._hold(name)
        now = float(now)
        if not math.isfinite(now):
            raise ValueError(f"{name}: time must be a finite number of seconds, got {now!r}")
        last = self._last.get(name)
        if last is not None and now < last - TIME_TOL:
            raise ValueError(f"{name}: time went back from {last:g} s to {now:g} s: clear() the events at every reset")
        self._last[name] = now
        if not holds:
            self._since[name] = None
            return
        since = self._since[name]
        if since is None:
            since = self._since[name] = now
        if name not in self._first and now - since >= hold - TIME_TOL:
            self._first[name] = (since, now)

    def first(self, name: str) -> float | None:
        """When the run in which ``name`` first held for its hold started (s since the reset); None if it has not."""
        self._hold(name)
        done = self._first.get(name)
        return None if done is None else done[0]

    def happened(self, name: str) -> bool:
        return self.first(name) is not None

    def before(self, a: str, b: str) -> bool:
        """``a`` has happened, and ``b`` had not started by then: ``b`` has not happened, or its run started later.
        Two whose runs started at the same control step are neither before the other."""
        ta, tb = self.first(a), self.first(b)
        return ta is not None and (tb is None or ta < tb - TIME_TOL)

    def order(self) -> list[str]:
        """The names that have happened, earliest first (two that started at the same step as they were declared)."""
        rank = {name: i for i, name in enumerate(self.holds)}
        return sorted(self._first, key=lambda name: (self._first[name][0], rank[name]))

    def report(self) -> list[str]:
        """One line per event: those that happened in their order, then those that did not, as declared."""
        lines = []
        for name in self.order():
            since, done = self._first[name]
            hold = self.holds[name]
            lines.append(f"{name}: at {since:.2f} s" if hold == 0.0 else f"{name}: from {since:.2f} s (held {hold:g} s by {done:.2f} s)")
        lines += [f"{name}: not happened (hold {self.holds[name]:g} s)" for name in self.holds if name not in self._first]
        return lines

    def _hold(self, name: str) -> float:
        try:
            return self.holds[name]
        except KeyError:
            raise KeyError(f"no event {name!r}: declared are {list(self.holds)}") from None


@dataclass(frozen=True)
class ToggleSpec:
    """A two-position switch with an over-centre snap, from the maker's figures.

    The moving part (rocker, lever, slider) is built half way between its two positions. Its joint's travel is 0
    there and positive towards on: off is at ``-travel/2``, on at ``+travel/2``, and the joint's stops are those two
    positions (``limits``). While the switch is off its spring pushes the part towards off, while it is on towards
    on; the push is ``hold`` x the snap force with the part at its position and rises linearly to the full snap force
    at the middle, where the switch snaps over (``Toggle.step`` moves the spring's rest to the other side). So a
    finger has to give the maker's snap force at the press point to switch it, and a push under ``hold`` of it moves
    nothing. ``hold`` is a choice of the model (makers state the operating force, not its course over the travel).

    kind: "hinge" (travel in rad; ``arm``, the distance in m from the pivot to where a finger presses, turns the
    force into a torque) or "slide" (travel in m; no arm). force: N at the press point. source: where force and
    travel come from (the maker's page or datasheet, with its address).
    """

    kind: str
    force: float
    travel: float
    source: str
    arm: float | None = None
    hold: float = 0.5

    def __post_init__(self) -> None:
        if self.kind not in ("hinge", "slide"):
            raise ValueError(f"kind must be 'hinge' or 'slide', got {self.kind!r}")
        if not (math.isfinite(self.force) and self.force > 0.0):
            raise ValueError(f"force must be a positive number of newtons, got {self.force!r}")
        if not (math.isfinite(self.travel) and self.travel > 0.0):
            raise ValueError(f"travel must be positive (rad for a hinge, m for a slide), got {self.travel!r}")
        if not (isinstance(self.source, str) and self.source.strip()):
            raise ValueError("cite the figures: source must say where force and travel come from")
        if self.kind == "hinge" and not (self.arm is not None and math.isfinite(self.arm) and self.arm > 0.0):
            raise ValueError(f"a hinge needs arm, the pivot's distance from the press point (m), got {self.arm!r}")
        if self.kind == "slide" and self.arm is not None:
            raise ValueError("a slide takes no arm: its force acts along its travel")
        if not 0.0 <= self.hold < 1.0:
            raise ValueError(f"hold must be at least 0 and under 1 (a share of the snap force), got {self.hold!r}")
        if self.kind == "hinge" and self.rest(True) > math.pi:
            raise ValueError(f"the spring's rest would lie {math.degrees(self.rest(True)):.1f} deg from the middle: PhysX wraps a hinge's targets at half a turn")

    @property
    def half(self) -> float:
        """Half the travel: each position's distance from the middle (rad or m)."""
        return self.travel / 2.0

    @property
    def snap(self) -> float:
        """The spring's push at the middle: N m for a hinge, N for a slide."""
        return self.force * (self.arm if self.kind == "hinge" else 1.0)

    @property
    def stiffness(self) -> float:
        """N m/rad or N/m: from ``hold`` x the snap at a position to the full snap at the middle, over half the travel."""
        return self.snap * (1.0 - self.hold) / self.half

    @property
    def beyond(self) -> float:
        """How far the spring's rest lies past the position it pushes towards (rad or m)."""
        return self.hold * self.snap / self.stiffness

    @property
    def limits(self) -> tuple[float, float]:
        """The joint's stops: off and on (rad or m from the middle)."""
        return (-self.half, self.half)

    def position(self, on: bool) -> float:
        """Where the part rests while the switch is on or off (rad or m from the middle)."""
        return self.half if on else -self.half

    def rest(self, on: bool) -> float:
        """The spring's rest while the switch is on or off (rad or m from the middle)."""
        return self.half + self.beyond if on else -(self.half + self.beyond)

    def push(self, travel: float, on: bool) -> float:
        """The spring's push on the part at ``travel`` while the switch is on or off: N m or N, positive towards on."""
        return -self.stiffness * (float(travel) - self.rest(on))

    def press_force(self, travel: float, on: bool) -> float:
        """The force (N) a finger gives at the press point to hold the part at ``travel`` against its spring."""
        return abs(self.push(travel, on)) / (self.arm if self.kind == "hinge" else 1.0)

    def critical_damping(self, inertia: float) -> float:
        """The damping (N m s/rad or N s/m) that brings the part to its position without bouncing:
        2 sqrt(stiffness x inertia), with ``inertia`` the part's moment of inertia about the pivot (kg m2) or its mass (kg)."""
        if not (math.isfinite(inertia) and inertia > 0.0):
            raise ValueError(f"inertia must be positive, got {inertia!r}")
        return 2.0 * math.sqrt(self.stiffness * inertia)


class Toggle:
    """A two-position switch in a task, run from the task's own hooks: ``reset()`` from its ``@mk.on_reset`` function,
    ``step()`` from its ``@mk.on_step`` function.

    The switch is on while its part is past the middle of its travel. When ``step()`` finds that the part has
    crossed the middle, the switch changes over: in the simulator its spring's rest moves to the new side (the snap),
    and its lamp is drawn anew (``show`` the lit shape and hide the dark one, then ``redraw``, since a small lamp
    that goes out or comes on can otherwise stay as it was in the pictures for seconds: mechkit's notes). The hooks
    run once per control step, so a press that crosses the middle and lets go within one step does not switch.
    Without the simulator (rule tests with ``mk.set_state``) it keeps its state and the lamp's drawing in mechkit's
    registry and leaves the spring alone.

    mk: the task's mechkit module. joint: the switch's hinge or slide, declared by the task with ``spec``'s limits
    and stiffness (checked here). lamp, dark: (body, shape index) drawn only while on, and only while off.
    Check a switch in the simulator against known loads (the force at which it changes over) before a rule hangs on
    it, as mechkit's notes ask of its springs and stops.
    """

    def __init__(self, mk, joint: str, spec: ToggleSpec, *, start_on: bool, lamp: tuple[str, int] | None = None,
                 dark: tuple[str, int] | None = None) -> None:  # fmt: skip
        j = mk.MECH.joints.get(joint)
        if j is None:
            raise KeyError(f"no joint {joint!r}: declare the switch's {spec.kind} before its Toggle")
        if j.kind != spec.kind:
            raise ValueError(f"{joint} is a {j.kind}, the spec is for a {spec.kind}")
        if not (math.isclose(j.lower, -spec.half, rel_tol=1e-9) and math.isclose(j.upper, spec.half, rel_tol=1e-9)):
            raise ValueError(f"{joint}: declare it with limits=spec.limits {spec.limits}, not ({j.lower}, {j.upper})")
        if not math.isclose(j.stiffness, spec.stiffness, rel_tol=1e-9):
            raise ValueError(f"{joint}: declare it with spring=spec.stiffness ({spec.stiffness:g}), not {j.stiffness:g}")
        for part in (lamp, dark):
            if part is None:
                continue
            body, shape = part
            if body not in mk.MECH.bodies:
                raise KeyError(f"no body {body!r} for the switch's lamp")
            if not 0 <= shape < len(mk.MECH.bodies[body].shapes):
                raise ValueError(f"{body} has shapes 0 to {len(mk.MECH.bodies[body].shapes) - 1}, not {shape}")
        if lamp is not None and lamp == dark:
            raise ValueError("the lit and the dark lamp must be different shapes")
        self.mk, self.joint, self.spec = mk, joint, spec
        self.ends = (j.lower, j.upper)  # off and on exactly as declared: set_joint refuses a value past a stop
        self.start_on, self.lamp, self.dark = bool(start_on), lamp, dark
        self.on = self.start_on
        self.flips: list[tuple[float, bool]] = []  # (time, on) of every change-over in the trial

    def reset(self, on: bool | None = None) -> None:
        """Start a trial with the switch on or off (None: as declared): in the simulator the part at that position
        with its spring on that side (the simulator keeps a spring's rest through a reset), the lamp drawn to match,
        the change-overs forgotten. Pictures need no redraw here: mechkit renders a reset's first pictures afresh."""
        self.on = self.start_on if on is None else bool(on)
        self.flips = []
        if self._in_simulator():
            self.mk.set_spring(self.joint, rest=self.spec.rest(self.on))
            self.mk.set_joint(self.joint, self.ends[1] if self.on else self.ends[0])
        self._draw()

    def step(self) -> bool:
        """Read the part's travel; if it has crossed the middle, change over. Returns whether the switch is on."""
        on = self.mk.joint(self.joint) > 0.0
        if on != self.on:
            self.on = on
            self.flips.append((self.mk.now(), on))
            if self._in_simulator():
                self.mk.set_spring(self.joint, rest=self.spec.rest(on))
            if self._draw():
                self.mk.redraw()
        return self.on

    def report(self) -> list[str]:
        """The trial's change-overs, as lines for the task's report."""
        return [f"switched {'on' if on else 'off'} at {t:.2f} s" for t, on in self.flips]

    def _draw(self) -> bool:
        """Draw the lamp for the present state; whether there is a lamp."""
        if self.lamp is not None:
            self.mk.show(*self.lamp, self.on)
        if self.dark is not None:
            self.mk.show(*self.dark, not self.on)
        return self.lamp is not None or self.dark is not None

    def _in_simulator(self) -> bool:
        return self.mk.MECH.environment is not None
