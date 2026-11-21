"""ourkit.rules: mechanism rules that act on a Mech's joints after every control step (20 Hz), in Isaac and physcheck.

A rule reads the joints (position, and speed as the change over the last control step) and changes a joint's limits,
gains or spring target. Rules run after each step, before the task's own hooks; a reset puts back whatever they
changed before the settling steps, then calls each rule's ``reset``.

- :class:`Latch`: catch points on one joint. Engaged, a catch is a joint limit at that point: ``side="up"`` stops the
  part going past it in +q (a pawl over a ratchet tooth: the part can still be pulled back), ``"down"`` in -q,
  ``"both"`` holds it there. ``style="drive"`` holds with a stiff spring instead (two-sided only). Which catch
  engages, and when it frees, are rules of the task (speed, a button, a pull).
- :class:`Detent`: notches on a sprung joint: its spring target follows the nearest notch, so it snaps into and holds
  in each notch with up to ``rate x half the spacing`` (set_targets only: the GPU-proven call).
- :class:`Couple`: one joint drives another at a ratio, while a condition holds (a feed wheel, a linked flap).
- :class:`Weld`: holds a joint where it is when a rule says (seated, clicked), with a stiff spring, and lets go when
  pulled past ``release_n`` or when a rule says.
Every rule logs ``events``: (what, t s, q, speed) tuples in mm or deg.
"""

from __future__ import annotations

import math

from . import scene

HZ = 20.0
DT = 1.0 / HZ


class Ctx:
    """What a rule sees at one control step: the mech's state, all object states, time and step, each joint's
    position ``q(j)``, its position a step ago ``prev(j)`` and its speed over the step ``v(j)``."""

    def __init__(self, name: str, states, step: int, prev: dict):
        self.name, self.states, self.step = name, states, step
        self.state = states[name]
        self.t = round(step * DT, 3)
        self._prev = prev

    def q(self, j: str) -> float:
        return float(self.state.q[j])

    def prev(self, j: str) -> float:
        return float(self._prev.get(j, self.q(j)))

    def v(self, j: str) -> float:
        return (self.q(j) - self.prev(j)) / DT


def _fmt(kind: str, q: float) -> float:
    return round(q * 1000, 2) if kind == "slide" else round(math.degrees(q), 2)


class Rule:
    """Base: ``joint`` of the rule set's mech. ``reset(ctx)`` after a reset; ``step(ctx)`` after every step."""

    def __init__(self, joint: str, name: str | None = None):
        self.joint, self.name = joint, name or f"{type(self).__name__.lower()}:{joint}"
        self.events: list = []
        self.mech = ""

    def log(self, ctx: Ctx, what: str, q: float | None = None) -> None:
        kind = scene.MECHS[self.mech].joint(self.joint).kind if self.mech in scene.MECHS else "slide"
        qq = ctx.q(self.joint) if q is None else q
        self.events.append((what, ctx.t, _fmt(kind, qq), _fmt(kind, ctx.v(self.joint))))

    def reset(self, ctx: Ctx) -> None:
        self.events = []

    def step(self, ctx: Ctx) -> None:
        pass


class Rules:
    """The rules of one mech, run by the kit's hooks: ``Rules(SHADE, [Latch(...), ...])`` in the task file."""

    def __init__(self, mech, rules):
        self.mech = mech.name if hasattr(mech, "name") else str(mech)
        if isinstance(mech, scene.Mech):
            scene.MECHS.setdefault(self.mech, mech)  # rules work (and are testable) before a simulator builds it
        self.rules = list(rules)
        for r in self.rules:
            r.mech = self.mech
        self.prev: dict = {}
        self.steps = 0
        scene._RULES[self.mech] = self  # a task file loaded again replaces its rules

    def __getitem__(self, name: str) -> Rule:
        return next(r for r in self.rules if r.name == name or r.joint == name)

    def reset(self, states) -> None:
        self.steps = 0
        self.prev = dict(states[self.mech].q)
        ctx = Ctx(self.mech, states, 0, self.prev)
        for r in self.rules:
            r.reset(ctx)

    def step(self, states) -> None:
        self.steps += 1
        ctx = Ctx(self.mech, states, self.steps, self.prev)
        for r in self.rules:
            r.step(ctx)
        self.prev = dict(states[self.mech].q)

    def events(self) -> list:
        return sorted((e + (r.name,) for r in self.rules for e in r.events), key=lambda e: e[1])


# -- locks ------------------------------------------------------------------------------------------------------------

EPS = {"slide": 1e-4, "hinge": math.radians(0.05)}  # half-width of a two-sided limit lock


def lock_limits(mech: str, joint: str, at: float, side: str) -> None:
    j = scene.MECHS[mech].joint(joint)
    e = EPS[j.kind]
    lo, hi = {"up": (j.lower, at), "down": (at, j.upper), "both": (at - e, at + e)}[side]
    scene.set_limits(mech, joint, lo, hi)


def free_limits(mech: str, joint: str) -> None:
    j = scene.MECHS[mech].joint(joint)
    scene.set_limits(mech, joint, j.lower, j.upper)


LOCK_RATE = {"slide": 20000.0, "hinge": 200.0}  # N/m, N m/rad: a drive lock's spring (physcheck caps it for MuJoCo)


def lock_drive(mech: str, joint: str, at: float, rate: float | None = None, damping: float | None = None) -> float:
    """Hold a joint at ``at`` with a stiff spring; returns the rate used. The spring target it had is kept."""
    j = scene.MECHS[mech].joint(joint)
    k = rate or LOCK_RATE[j.kind]
    c = damping if damping is not None else 2.0 * math.sqrt(k * max(j.mass, 1e-3))
    scene.set_gains(mech, joint, k, c)
    held = scene.target_of(mech, joint)
    scene.set_targets(mech, {joint: at})
    scene._TARGET[(mech, f"{joint}#held")] = held
    return k


def free_drive(mech: str, joint: str) -> None:
    j = scene.MECHS[mech].joint(joint)
    scene.set_gains(mech, joint, j.rate, j.damping)
    scene.set_targets(mech, {joint: scene._TARGET.pop((mech, f"{joint}#held"), j.target)})


class Latch(Rule):
    """Catch points on a joint. ``points``: joint positions (m or rad). ``side``: "up", "down" or "both" (see the
    module docstring). A catch engages when the joint, coming from the free side, is within ``window`` of a point or
    crossed it during the last step, and ``engage(ctx, point)`` agrees (default: always); otherwise it runs past (an
    event). An engaged catch frees when ``release(ctx, point)`` says so (default: never). ``start``: the point engaged
    at reset (None: none). ``style``: "limits" (a joint limit: rigid, one- or two-sided) or "drive" (a stiff spring,
    two-sided, ``rate`` N/m or N m/rad). One catch at a time."""

    def __init__(self, joint: str, points, *, side: str = "up", engage=None, release=None, start=None,
                 window: float | None = None, style: str = "limits", rate: float | None = None, name=None):
        super().__init__(joint, name)
        if side not in ("up", "down", "both"):
            raise ValueError("side: up, down or both")
        if style == "drive" and side != "both":
            raise ValueError("a drive lock is two-sided: use side='both', or style='limits' for a pawl")
        self.points = sorted(float(p) for p in points)
        self.side, self.style, self.rate = side, style, rate
        self.engage_fn, self.release_fn = engage, release
        self.start, self.window = start, window
        self.engaged: float | None = None

    def _hold(self, at: float) -> None:
        if self.style == "drive":
            lock_drive(self.mech, self.joint, at, self.rate)
        else:
            lock_limits(self.mech, self.joint, at, self.side)

    def _free(self) -> None:
        if self.style == "drive":
            free_drive(self.mech, self.joint)
        else:
            free_limits(self.mech, self.joint)

    def reset(self, ctx: Ctx) -> None:
        super().reset(ctx)
        self.engaged = None
        if self.start is not None:
            self.engaged = float(self.start)
            self._hold(self.engaged)
            self.log(ctx, "engaged", self.engaged)

    def _window(self) -> float:
        if self.window is not None:
            return self.window
        kind = scene.MECHS[self.mech].joint(self.joint).kind
        return 0.003 if kind == "slide" else math.radians(1.0)

    def candidate(self, ctx: Ctx) -> float | None:
        """The catch the joint is reaching from its free side now (within the window, or crossed this step)."""
        q, p = ctx.q(self.joint), ctx.prev(self.joint)
        w = self._window()
        best = None
        for c in self.points:
            rising = self.side in ("up", "both") and (0.0 <= c - q <= w or p < c <= q)
            falling = self.side in ("down", "both") and (0.0 <= q - c <= w or q <= c < p)
            if rising or falling:
                if best is None or abs(c - q) < abs(best - q):
                    best = c
        return best

    def step(self, ctx: Ctx) -> None:
        if self.engaged is not None:
            if self.release_fn is not None and self.release_fn(ctx, self.engaged):
                self.log(ctx, "freed", self.engaged)
                self.engaged = None
                self._free()
            return
        c = self.candidate(ctx)
        if c is None:
            return
        if self.engage_fn is None or self.engage_fn(ctx, c):
            self.engaged = c
            self._hold(c)
            self.log(ctx, "engaged", c)
        elif ctx.prev(self.joint) < c <= ctx.q(self.joint) or ctx.q(self.joint) <= c < ctx.prev(self.joint):
            self.log(ctx, "ran past", c)


class Detent(Rule):
    """Notches on a sprung joint (``rate`` > 0): its spring target is the nearest of ``points`` (or the nearest within
    ``capture`` of it; outside every capture the target stays where it was). Holds in a notch with no command."""

    def __init__(self, joint: str, points, *, capture: float | None = None, name=None):
        super().__init__(joint, name)
        self.points = sorted(float(p) for p in points)
        self.capture = capture
        self.at: float | None = None

    def reset(self, ctx: Ctx) -> None:
        super().reset(ctx)
        self.at = None
        self.step(ctx)

    def step(self, ctx: Ctx) -> None:
        q = ctx.q(self.joint)
        near = min(self.points, key=lambda p: abs(p - q))
        if self.capture is not None and abs(near - q) > self.capture:
            return
        if near != self.at:
            self.at = near
            scene.set_targets(self.mech, {self.joint: near})
            self.log(ctx, "notch", near)


class Couple(Rule):
    """``driven`` follows ``joint`` (the driver): target = offset + ratio x (q_driver - its q at reset), sent while
    ``when(ctx)`` holds (default always); otherwise its target stays. ``offset`` defaults to the driven joint's target
    at reset. The driven joint needs a spring (``rate``) stiff enough to follow."""

    def __init__(self, joint: str, driven: str, ratio: float, *, offset: float | None = None, when=None, name=None):
        super().__init__(joint, name or f"couple:{joint}->{driven}")
        self.driven, self.ratio, self.offset, self.when = driven, float(ratio), offset, when
        self.q0 = 0.0
        self.base = 0.0
        self.on = False

    def reset(self, ctx: Ctx) -> None:
        super().reset(ctx)
        self.q0 = ctx.q(self.joint)
        self.base = self.offset if self.offset is not None else scene.target_of(self.mech, self.driven)
        self.on = False

    def step(self, ctx: Ctx) -> None:
        ok = self.when is None or bool(self.when(ctx))
        if ok != self.on:
            self.on = ok
            self.log(ctx, "coupled" if ok else "uncoupled")
        if ok:
            scene.set_targets(self.mech, {self.driven: self.base + self.ratio * (ctx.q(self.joint) - self.q0)})


class Weld(Rule):
    """Holds ``joint`` where it is once ``when(ctx)`` holds (seated, clicked shut), with a stiff spring of ``rate``.
    Frees when ``release(ctx)`` says so, or when the part is pulled off the weld point by more than ``release_n`` /
    ``rate`` (that is, a pull past ``release_n`` N or N m). ``start=True`` welds at reset."""

    def __init__(self, joint: str, when=None, *, release=None, release_n: float | None = None,
                 rate: float | None = None, start: bool = False, name=None):
        super().__init__(joint, name)
        self.when, self.release, self.release_n, self.rate, self.start = when, release, release_n, rate, start
        self.at: float | None = None
        self.k = 0.0

    def reset(self, ctx: Ctx) -> None:
        super().reset(ctx)
        self.at = None
        if self.start:
            self._weld(ctx)

    def _weld(self, ctx: Ctx) -> None:
        self.at = ctx.q(self.joint)
        self.k = lock_drive(self.mech, self.joint, self.at, self.rate)
        self.log(ctx, "welded")

    def step(self, ctx: Ctx) -> None:
        if self.at is None:
            if self.when is not None and self.when(ctx):
                self._weld(ctx)
            return
        pulled = self.release_n is not None and abs(ctx.q(self.joint) - self.at) * (self.rate or LOCK_RATE[
            scene.MECHS[self.mech].joint(self.joint).kind]) > self.release_n
        if pulled or (self.release is not None and self.release(ctx)):
            self.log(ctx, "released")
            self.at = None
            free_drive(self.mech, self.joint)
