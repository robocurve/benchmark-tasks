"""pivotkit: a pin pivot that holds only while a condition holds, for yamkit Mechanisms.

A sewing needle down through the fabric, a divider point seated in a punch mark, a compass point in the paper, a
pin through a template: while the pin is in, the free part can only turn about the pin point; while it is out, the
part slides and turns freely in its plane (or stays where it is, as the task chooses).

    import yamkit as kit
    import pivotkit as pv

    PIN = pv.Pivot("panel", engaged=lambda J: J["wheel"] < 0.32, free="slide",
                   resist=lambda J: (40.0, 0.3) if J["lifter"] < 0.05 else None)   # presser foot down: drags
    links = pv.planar(PIN, pin=(0.0, 0.0, 0.001), reach=((-0.05, 0.05), (-0.05, 0.05)), turn=(-0.5, 2.2),
                      mass=0.06, meshes=..., colliders=...)          # meshes/colliders in the part's frame at the pin
    machine = kit.Mechanism("machine", ..., links=(wheel, needle, *links))

    pv.engaged(objects, "machine/panel")      # in the scorer: the pin is in now
    pv.turned_deg(objects, "machine/panel")   # signed turn about the pin while it was in (ccw from above +)
    pv.slid(objects, "machine/panel")         # the part slid more than `slip` while the pin was in, ever this trial

How it works. `planar` builds the part as the end of a three-joint chain off the mechanism's base: a hinge about the
vertical through the pin point (`<name>_turn`), then a slide along the hinge's x (`<name>_x`), then a slide along its
y (the part, `<name>`). The part's pose is the turn about the pin after the two slides, so (x, y, turn) covers every
flat pose, and the pin point sits over the part's point (-x, -y) in the part's own frame (the part's origin at the
start). While the pin is in, the kit narrows both slides' joint limits to within `tol` of where they were when it
went in (the same trick as yamkit's locked latch: a hard stop holds where a drive does not), so only the hinge moves:
the part turns about the pin and nothing else. While it is out, the limits are the slides' own (`free="slide"`) or
all three joints are narrowed at their current values (`free="fixed"`). Damping on each joint (an implicit PhysX
drive, stable on light parts) is light by default, like a part sliding on a table, and `resist(J)` can raise it
(a presser foot clamping the fabric): a pushed part then drags rather than being locked.

Needs the yamkit copy with `Link.parent`, collider-less link inertia and law-set limits (mem["limits"]), added in
tasks/sewing-pivot-at-corner/yamkit.py (to be merged into the master). The part's joints have no gravity sag: the
chain carries the part flat at its start height, so give its underside a small gap (1 mm) above the surface it lies on.

Readouts (all reset with kit.episode()); `key` is "mechanism/part":
  engaged(objects, key)      the pin is in at this step
  turned_deg(objects, key)   signed rotation about the pin accumulated over steps when the pin was in at both ends
  free_turn_deg(objects, key)  unsigned rotation accumulated while the pin was out (turning off the pin), counted
                             through a `jitter` dead band (0.05 deg) so sim jitter at rest never adds up
  angle_deg(objects, key)    the part's turn from its start, now (signed, ccw from above +)
  slid(objects, key)         True once a slide moved more than `slip` from where the pin went in, while in
  max_slip_mm(objects, key)  the largest such slide this trial
  pin_offset(objects, key)   where the pin point is on the part now, (x, y) in m in the part's frame from where it
                             was at the start (0, 0 = the same point of the part as at the start)
  entered_at(objects, key)   pin_offset when the pin last went in (None if never)
  entries(objects, key)      how many times the pin went in
Offline tests call `step(key, J)` once per control step with the joints by link name (the sim's turn law does it).
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass

import numpy as np

import yamkit as kit


@dataclass(frozen=True)
class Pivot:
    """A conditional pin pivot for the part link `name`. `engaged(J)` says whether the pin is in, from the
    mechanism's joint positions by link name (e.g. a needle's handwheel angle). `free` is what the part does while the
    pin is out: "slide" (moves and turns freely in its plane) or "fixed" (stays put). `tol` (m) is how far the
    narrowed slide stops sit either side of where the pin went in (0.5 mm: a needle's play in its hole); `slip` (m)
    is how far from there counts as having slid while in (2 mm: four times `tol`, so only a real push through the
    stops counts). `slide_damp` (N*s/m) and `turn_damp` (N*m*s/rad) resist the part's motion, like friction on the
    surface it lies on; `resist(J)` may return (slide_damp, turn_damp) to use instead, or None."""

    name: str
    engaged: object
    free: str = "slide"
    tol: float = 0.0005
    slip: float = 0.002
    slide_damp: float = 2.0
    turn_damp: float = 0.01
    resist: object = None
    turn_tol: float = 0.004  # rad: the turn's narrowed stop either side when free="fixed" (0.2 degrees)
    jitter: float = math.radians(0.05)  # rad: free turn is counted only once it moves this far from where it was
    #   last counted (a dead band): sim jitter at rest (under 0.005 deg a step) never adds up, a real turn does
    def damps(self, J):
        r = self.resist(J) if self.resist is not None else None
        return r if r is not None else (self.slide_damp, self.turn_damp)


_STATE: dict[str, tuple] = {}  # "mechanism/part" -> (episode, state dict)
_PIVOTS: dict[str, Pivot] = {}  # part link name -> its Pivot (to find the key from a law)


def _fresh() -> dict:
    return {"engaged": False, "turned": 0.0, "free_turn": 0.0, "slid": False, "max_slip": 0.0, "entered_at": None,
            "entries": 0, "free_ref": None, "theta": None, "theta0": None, "s": (0.0, 0.0), "s0": None, "hold": None}


def state(key: str) -> dict:
    """The pivot's record for this trial (fresh after every scene reset)."""
    ep, st = _STATE.get(key, (None, None))
    if ep != kit.episode():
        st = _fresh()
        _STATE[key] = (kit.episode(), st)
    return st


def step(key: str, J: dict) -> dict:
    """Update the record from this control step's joint positions (by link name). Called by the sim's turn law once
    per step; offline tests call it directly."""
    pv = _PIVOTS[key.split("/")[-1]]
    st = state(key)
    th, s = J[f"{pv.name}_turn"], (J[f"{pv.name}_x"], J[pv.name])
    if st["theta0"] is None:
        st["theta0"], st["s0"], st["theta"] = th, s, th
    now = bool(pv.engaged(J))
    d = th - st["theta"]
    if now and st["engaged"]:
        st["turned"] += d
        st["free_ref"] = None
    else:  # free turn, through a dead band so jitter back and forth about one angle doesn't add up
        if st["free_ref"] is None:
            st["free_ref"] = st["theta"]
        if abs(th - st["free_ref"]) > pv.jitter:
            st["free_turn"] += abs(th - st["free_ref"])
            st["free_ref"] = th
    if now and not st["engaged"]:
        st["entries"] += 1
        st["entered_at"] = s
    if now:
        slip = math.hypot(s[0] - st["entered_at"][0], s[1] - st["entered_at"][1])
        st["max_slip"] = max(st["max_slip"], slip)
        st["slid"] = st["slid"] or slip > pv.slip
        st["hold"] = None
    elif pv.free == "fixed":
        if st["hold"] is None:
            st["hold"] = (th, s)
    st["engaged"], st["theta"], st["s"] = now, th, s
    return st


class _Joint:
    """The law on each of the chain's three joints (role "turn", "x" or "y"); see the module docstring."""

    sets_limits = True

    def __init__(self, pivot: Pivot, role: str, lower: float, upper: float):
        self.pivot, self.role, self.lower, self.upper = pivot, role, lower, upper

    def drive(self, key, q, qd, J, mem):
        part = key.rsplit("/", 1)[0] + "/" + self.pivot.name
        st = step(part, J) if self.role == "turn" else state(part)
        slide_d, turn_d = self.pivot.damps(J)
        mem["limits"] = None
        if self.role == "turn":
            if st["hold"] is not None:  # free="fixed" and the pin is out: the part stays where it is
                t = st["hold"][0]
                mem["limits"] = (max(self.lower, t - self.pivot.turn_tol), min(self.upper, t + self.pivot.turn_tol))
            return q, 0.0, 0.0, turn_d
        k = 0 if self.role == "x" else 1
        at = st["entered_at"] if st["engaged"] else (st["hold"][1] if st["hold"] is not None else None)
        if at is not None:  # the pin is in (or the part is held fixed): narrow this slide's stops around there
            mem["limits"] = (max(self.lower, at[k] - self.pivot.tol), min(self.upper, at[k] + self.pivot.tol))
        return q, 0.0, 0.0, slide_d


def planar(pivot: Pivot, pin, reach, turn, mass: float, meshes=(), colliders=(), start=(0.0, 0.0, 0.0),
           parent: str | None = None, link_mass: float = 0.005, slide_at=(0.0, 0.0)) -> tuple:
    """The part on its three-joint chain, as yamkit Links to put in a Mechanism's `links` (in this order). `pin` is the
    pin point in the base's frame; the part's meshes and colliders are in its own frame, with its origin at the pin
    (at the start: the part's point that the pin goes through when nothing has moved). `reach` is ((x lo, x hi),
    (y lo, y hi)) in m, how far the part can slide along its own axes (keep it within the surface it lies on);
    `turn` (lo, hi) in rad. `start` is (x, y, turn). `link_mass` (kg) is each of the two hidden intermediate links.
    `slide_at` (local addition, router task) is where on the part, (x, y) from the pin point in its frame, the slides'
    joint frames sit: PhysX applies the slides' damping there, so put it at the part's center, or a part pushed at
    one point drags as if held back at the pin point and swings round. The part's pose and the readouts are the same."""
    _PIVOTS[pivot.name] = pivot
    (xl, xh), (yl, yh) = reach
    n = pivot.name
    g = np.array([slide_at[0], slide_at[1], 0.0])
    if g.any():  # the part's meshes and colliders in its link frame, now at the slide point
        meshes = tuple(kit._mesh(np.asarray(m.points, float) - g, m.counts, m.indices, m.look) for m in meshes)
        colliders = tuple(dataclasses.replace(c, center=tuple(np.asarray(c.center, float) - g)) for c in colliders)
    at = tuple(np.asarray(pin, float) + g)
    return (
        kit.Link(f"{n}_turn", "revolute", "Z", tuple(pin), lower=turn[0], upper=turn[1], start=start[2],
                 mass=link_mass, damping=0.0, parent=parent, law=_Joint(pivot, "turn", turn[0], turn[1])),
        kit.Link(f"{n}_x", "prismatic", "X", at, lower=xl, upper=xh, start=start[0], mass=link_mass,
                 damping=0.0, parent=f"{n}_turn", law=_Joint(pivot, "x", xl, xh)),
        kit.Link(n, "prismatic", "Y", at, lower=yl, upper=yh, start=start[1], mass=mass, meshes=tuple(meshes),
                 colliders=tuple(colliders), damping=0.0, parent=f"{n}_x", law=_Joint(pivot, "y", yl, yh)),
    )


# -- scorer readouts -------------------------------------------------------------------------------


def engaged(objects, key: str) -> bool:
    return bool(state(key)["engaged"])


def turned_deg(objects, key: str) -> float:
    return math.degrees(state(key)["turned"])


def free_turn_deg(objects, key: str) -> float:
    return math.degrees(state(key)["free_turn"])


def angle_deg(objects, key: str) -> float:
    st = state(key)
    return 0.0 if st["theta0"] is None else math.degrees(st["theta"] - st["theta0"])


def slid(objects, key: str) -> bool:
    return bool(state(key)["slid"])


def max_slip_mm(objects, key: str) -> float:
    return 1000.0 * state(key)["max_slip"]


def pin_offset(objects, key: str) -> tuple:
    st = state(key)
    if st["s0"] is None:
        return (0.0, 0.0)
    return (st["s0"][0] - st["s"][0], st["s0"][1] - st["s"][1])


def entered_at(objects, key: str):
    st = state(key)
    if st["entered_at"] is None or st["s0"] is None:
        return None
    return (st["s0"][0] - st["entered_at"][0], st["s0"][1] - st["entered_at"][1])


def entries(objects, key: str) -> int:
    return int(state(key)["entries"])
