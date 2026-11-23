"""mechkit: joint behaviors ("laws") for yamkit Mechanism links, so one kit covers toggles, ratchets, knobs, gauges,
pumps and interlocks.

    import yamkit as kit
    import mechkit as mk

    handle = kit.Link("handle", "revolute", "Y", anchor=(0, 0, 0.03), lower=0.0, upper=1.9, start=0.3,
                      law=mk.OverCenter(center=1.2, lock_at=1.8), meshes=..., colliders=...)
    ...
    mk.locked(objects, "clamp/handle")     # in the scorer

Every law sets the joint's drive each control step from the joint's position `q` and speed `qd`, the positions of the
mechanism's other joints (`J`, by link name) and a per-trial memory (reset with the scene). A law returns
(target, stiffness, effort, damping): PhysX's own (implicit) joint drive applies stiffness * (target - q) + effort -
damping * qd at every physics substep. Gains must be implicit like this: a PD computed once per 20 Hz control step
and held as an effort for 12 substeps is unstable on light links (sim pilot, Nov 9: an 80 g lever slammed between
its detent and its stop every step). yamkit gives every link with a law a PhysX drive (PhysicsDriveAPI) for this.
Constant pushes (OverCenter's snap, a Ratchet's load) are efforts; `hold` is a stiff drive where a part must not be
pushed back (a caught pawl, a blocked interlock).

Readouts for scorers: `state(key)` is the law's memory for this trial (e.g. "clicks", "level", "setting", "locked",
"forced"); the helpers below read the common ones. Units: rad for hinges, m for slides; torques N*m, forces N.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import yamkit as kit


def _pd(q, qd, target, k, c):
    return -k * (q - target) - c * qd


@dataclass(frozen=True)
class Free:
    """No behavior beyond friction-like `stay` drag (the base case, for links that other laws read)."""

    drag: float = 0.02

    def drive(self, key, q, qd, J, mem):
        return q, 0.0, 0.0, self.drag


@dataclass(frozen=True)
class OverCenter:
    """A bistable toggle: past `center` (toward upper) a torque/force `push` drives it to its upper stop, below it to
    the lower stop, so it snaps to one side or the other (toggle clamp, draw latch, ratchet-strap handle, bike
    quick-release, cam lever, stepladder spreader). Within `dead` of center it is free (the over-center hump).
    `locked` is set while q >= `lock_at` (the lever is seated over center); `ever_locked` latches it for the trial.
    With `sign=-1` the locking side is toward lower (lock_at then counts q <= lock_at)."""

    center: float
    lock_at: float
    push: float = 0.15
    dead: float = 0.03
    damp: float = 0.02
    sign: int = 1

    def drive(self, key, q, qd, J, mem):
        d = self.sign * (q - self.center)
        f = 0.0 if abs(d) < self.dead else math.copysign(self.push, d) * self.sign
        mem["locked"] = self.sign * (q - self.lock_at) >= 0
        mem["ever_locked"] = mem.get("ever_locked", False) or mem["locked"]
        return q, 0.0, f, self.damp


@dataclass(frozen=True)
class Ratchet:
    """A one-way joint with a pawl every `pitch` (ratchet strap, jack, cable puller, ratcheting clamp, zip-tie): it
    moves freely in the `sign` direction and is held at its furthest click going back. `clicks` counts teeth past the
    start. While the joint named `release` (another link of the mechanism) is past `release_at` (in that joint's
    units, in its `release_sign` direction), the pawl is lifted and the joint runs back freely, pulled by `back`
    (N or N*m, e.g. a load or a spring). `clicks_max` stops it (a strap at full tension)."""

    pitch: float
    sign: int = 1
    release: str | None = None
    release_at: float = 0.0
    release_sign: int = 1
    back: float = 0.0
    hold: float = 400.0
    clicks_max: int | None = None
    damp: float = 0.02

    def fresh(self, lk):
        return {"held": lk.start, "clicks": 0, "released": False}

    def drive(self, key, q, qd, J, mem):
        released = self.release is not None and self.release_sign * (J[self.release] - self.release_at) >= 0
        mem["released"] = released
        back = -self.sign * self.back
        if released:
            mem["held"] = q
            mem["clicks"] = int(math.floor(self.sign * (q - mem["start"]) / self.pitch + 1e-9))
            return q, 0.0, back, self.damp
        n = int(math.floor(self.sign * (q - mem["start"]) / self.pitch + 1e-9))
        if self.clicks_max is not None and n > self.clicks_max:
            n = self.clicks_max
            cap = mem["start"] + self.sign * self.clicks_max * self.pitch
            return cap, self.hold, back, self.damp
        if n > mem["clicks"]:
            mem["clicks"] = n
            mem["held"] = mem["start"] + self.sign * n * self.pitch
        if self.sign * (q - mem["held"]) < 0:  # pushed back onto the pawl
            return mem["held"], self.hold, back, self.damp
        return q, 0.0, back, self.damp


@dataclass(frozen=True)
class Detents:
    """A knob, selector or rocker with click positions (stove knob OFF/LOW/HIGH, rotary switch, fan speed, valve
    quarter-turn stops): within `zone` of a detent it is pulled into it with `pull`; between detents it is free with
    a little drag. `setting` is the index of the detent it sits in (None between). `labels` names the detents."""

    positions: tuple
    labels: tuple = ()
    zone: float = 0.12
    pull: float = 0.6
    damp: float = 0.03
    drag: float = 0.05

    def drive(self, key, q, qd, J, mem):
        i = min(range(len(self.positions)), key=lambda k: abs(q - self.positions[k]))
        p = self.positions[i]
        if abs(q - p) <= self.zone:
            mem["setting"] = i
            mem["label"] = self.labels[i] if self.labels else i
            return p, self.pull, 0.0, self.damp
        mem["setting"] = None
        mem["label"] = None
        return q, 0.0, 0.0, self.drag


@dataclass(frozen=True)
class Follow:
    """A part driven by another joint or a value, not by hand: gauge needles, indicator flags, a linkage's second
    bar, a guard that tracks a lever. Its target is `ratio` * J[source] + `offset` (or `fn(J, mem)` if given),
    clamped to [lo, hi]. By default (`kinematic=True`) the kit moves the joint to the target itself, at most `rate` per
    control step: in the sim an effort PD on a 10 g needle at 20 Hz is unstable and pins it at its stops. With
    `kinematic=False` it is pushed by `k`/`damp` instead (a part that must give way to a hand; 50 g or more)."""

    source: str | None = None
    ratio: float = 1.0
    offset: float = 0.0
    fn: object = None
    lo: float = -1e9
    hi: float = 1e9
    k: float = 0.5
    damp: float = 0.05
    kinematic: bool = True  # the kit sets the joint (rate-limited) instead of pushing it: stable for 10 g needles
    rate: float = 0.25  # most the joint moves per control step when kinematic (rad or m), so it sweeps, not jumps
    hold: float = 0.01  # soft stiffness holding a kinematic follower between steps

    def drive(self, key, q, qd, J, mem):
        t = self.fn(J, mem) if self.fn is not None else self.ratio * J[self.source] + self.offset
        t = min(max(t, self.lo), self.hi)
        mem["target"] = t
        return t, self.k, 0.0, self.damp


@dataclass(frozen=True)
class Pump:
    """A part raised in steps by working another joint (pallet-jack forks, hydraulic jack ram, bottle jack, grease or
    tire pump pressure): each full stroke of joint `handle` (from below `stroke_lo` to above `stroke_hi` and back)
    adds `per_stroke` to the level, up to `top`. This link is driven to `start + level`. While the joint `release` is
    past `release_at`, the level falls at `drop_rate` per step (lowering the forks, opening the jack's valve).
    `strokes` counts strokes; `level` is the raise above start."""

    handle: str
    stroke_lo: float
    stroke_hi: float
    per_stroke: float
    top: float
    release: str | None = None
    release_at: float = 0.0
    release_sign: int = 1
    drop_rate: float = 0.002
    k: float = 60.0
    damp: float = 4.0

    def fresh(self, lk):
        return {"level": 0.0, "strokes": 0, "armed": False}

    def drive(self, key, q, qd, J, mem):
        h = J[self.handle]
        if h <= self.stroke_lo:
            mem["armed"] = True
        elif h >= self.stroke_hi and mem["armed"]:
            mem["armed"] = False
            mem["strokes"] += 1
            mem["level"] = min(self.top, mem["level"] + self.per_stroke)
        if self.release is not None and self.release_sign * (J[self.release] - self.release_at) >= 0:
            mem["level"] = max(0.0, mem["level"] - self.drop_rate)
            mem["lowering"] = True
        else:
            mem["lowering"] = False
        return mem["start"] + mem["level"], self.k, 0.0, self.damp


@dataclass(frozen=True)
class Interlock:
    """A joint that is held (at where it was when the lock engaged) while `blocked(J)` is true, and runs `inner` (any
    other law, or free) when it is not: a door that won't open while the machine runs, a drawer that won't open while
    another is out, a trigger that won't squeeze with the pin in. `forced` counts steps on which something pushed
    against the lock by more than `force_tol` (rad or m) — useful to score "tried to force it"."""

    blocked: object
    inner: object = field(default_factory=Free)
    hold: float = 400.0
    force_tol: float = 0.02

    def drive(self, key, q, qd, J, mem):
        if self.blocked(J):
            at = mem.setdefault("held_at", q)
            if abs(q - at) > self.force_tol:
                mem["forced"] = mem.get("forced", 0) + 1
            mem["blocked"] = True
            return at, self.hold, 0.0, 1.0
        mem.pop("held_at", None)
        mem["blocked"] = False
        return self.inner.drive(key, q, qd, J, mem)


@dataclass(frozen=True)
class Spring:
    """A plain return spring with damping, as a law (so it can be an Interlock's `inner`): pulls to `rest`."""

    rest: float
    k: float = 0.3
    damp: float = 0.02

    def drive(self, key, q, qd, J, mem):
        return self.rest, self.k, 0.0, self.damp


# -- scorer readouts -------------------------------------------------------------------------------


def state(key: str) -> dict:
    """The law's memory for link `key` ("mechanism/link") in this trial."""
    return kit.law_state(key)


def locked(objects, key: str) -> bool:
    """An OverCenter link seated over center right now."""
    return bool(state(key).get("locked", False))


def clicks(objects, key: str) -> int:
    """A Ratchet's teeth taken up this trial (0 after a full release)."""
    return int(state(key).get("clicks", 0))


def setting(objects, key: str):
    """A Detents link's current detent label (or index), None between detents."""
    return state(key).get("label")


def level(objects, key: str) -> float:
    """A Pump link's raise above its start."""
    return float(state(key).get("level", 0.0))


def forced(objects, key: str) -> int:
    """Steps on which an Interlock was pushed against while blocked."""
    return int(state(key).get("forced", 0))
