"""ringkit: elastic and split rings on a shaft (circlips, snap rings, O-rings), built on yamkit.

A ring sits on a shaft. Its inner radius is spring-loaded: left alone it closes toward its free (rest) radius, which is
smaller than the shaft, so it grips the shaft wherever it is, or springs into a groove when it is over one. A tool
spreads it (circlip pliers in its lug holes, a stretching cone for an O-ring); spread past the shaft it slides along
the shaft and turns about it with the tool. Spread past its elastic limit it takes a permanent set and is scrap.

    import yamkit as kit
    import ringkit as rk

    CLIP = rk.Ring("clip", end=(0, 0, 0.105), axis="-Z", shaft_d=0.040, free_id=0.0365, thickness=0.00175,
                   width=0.0045, length=0.05, grooves=(rk.Groove(at=0.0381, width=0.00185, diameter=0.0375),),
                   stop=0.039, limit_id=0.041, lug_gap=0.012, lug_out=0.0058, look=kit.Look((0.2, 0.2, 0.25)))
    fixture = kit.Mechanism("fixture", ..., links=(*rk.links(CLIP), ...))      # the shaft is part of the base
    pliers = rk.free_tool(kit.Mechanism("pliers", ..., mass=0.07, links=(jaw,)))  # a loose, articulated tool
    rk.attach("fixture", CLIP, rk.TipPair(("pliers", tip_a), ("pliers/jaw", tip_b)))

    rk.seated(objects, "fixture/clip")       # in the scorer

The model (all lengths in m; `s` is the ring's mid-plane position along the shaft, measured from the shaft's end face
into the shaft, so s = -thickness/2 is a ring lying on the end face):
- Shaft profile: radius shaft_d/2 for 0 < z <= length, a groove's bottom radius within each `Groove`, nothing past the
  end (z <= 0). The ring covers [s - t/2, s + t/2]; the radius it has to clear is the largest radius under it.
- Free: the ring closes to max(free radius + permanent set, the radius under it): gripping the land, closed into a
  groove, or (off the end) at its free radius. It does not move along the shaft or turn.
- Snap: a closed ring whose mid-plane is within half a groove width of the groove's middle sits in the groove (its own
  spring pushes it in); `stop` (optional) is a face the ring's far face cannot pass (the part it retains, a shoulder).
- Held by a tool (a spreader reports a `Grip`): the ring opens to what the tool asks (never smaller than the shaft
  under it allows), turns with the tool (unless cocked) and moves along the shaft toward where the tool puts it, but
  only as far as its radius clears the shaft by `clear` (diametral): a ring not spread far enough stays where it is,
  and the tool's tips slip out of it (the spreader then stops reporting a grip).
- Over-spread: the largest inner diameter ever reached is kept; past `limit_id` the excess becomes a permanent set
  (the free radius grows by it): such a ring may close loosely in its groove.
- Tilt and twist: the tool reports how far it tilts the ring's plane off square to the shaft and how far it twists
  the ring's two ends apart along the shaft. A ring that closes into a groove tilted more than `tilt_limit` degrees
  lands cocked (one side in the groove, the other on the land; not seated, it cannot turn), however near closed it
  was before; spreading it again frees it. The largest tilt and twist while held are kept for the scorer.
- Seated: in a groove, closed onto the groove bottom (inner radius within `seat_tol` of it), not cocked, not held.
- Reaction on the shaft (`grip_n` > 0): a ring closed onto the shaft grips it, and a tool turning it drags the shaft
  round with a torque about its axis until the ring slips (mu * grip * shaft radius); the ring's turn counts only as
  it slips on the shaft. See "reaction loads" below.

Spreaders (anything with `grip(ring, st, frame, objects) -> Grip | None`): `TipPair` is a pair of pliers tips in a
split ring's two lug holes. A tip engages when it is within `catch` of a lug hole across the shaft axis and its end
is inside the hole (below the ring's near face, at most `depth` past its far face). The ring's lug gap follows the
tips' spacing, so its inner radius grows by (tip spacing - rest lug gap) / (2 pi). Write another spreader for other
tools (an O-ring cone, fingers): it gets the ring spec, its state and the shaft frame, and returns a Grip or None.

Joints: `links(ring)` gives the ring's invisible, collider-less links for the fixture mechanism, all on its base:
"<name>_axial" (prismatic along the shaft: s), "<name>_turn" (revolute about it: the ring's turn, ccw seen from the
end) and "<name>" (prismatic: the opening, inner radius minus free radius). The kit writes them every control step
from its model, so `kit.joint(objects, "fixture/clip")` etc. read the ring too (one step behind the kit's readouts).
The ring is drawn with Isaac Lab visualization markers (segments of a band and two lug ears) at its current radius,
turn, tilt and twist, and once more at its start pose right after every reset, so it shows in the first camera frame;
drawing never affects scoring. New marker prims take about five rendered frames to appear, so a
fourth link, "<name>_shown", carries a mesh of the ring at its start pose for the first SHOW_STEPS steps and is then
moved to s = `hide` (inside the table or fixture). Use a matte look: a metallic dark ring mirrors the bright dome.

Free tools: `free_tool(mech)` marks a loose (massed) Mechanism so that its base body carries the articulation root
(otherwise PhysX roots a floating articulation at its most central body and the joint drives misbehave).
`HandleSpring` is a law for a tool's handle (a preloaded return spring).

Scorer readouts (key "fixture/clip"; all reset with kit.episode()):
  position(objects, key)       s now (m)               groove(objects, key)      index of the groove it sits in, or None
  seated(objects, key)         seated now (see above)  engaged(objects, key)     a tool holds it now
  inner_d(objects, key)        inner diameter now (m)  max_d(objects, key)       largest inner diameter this trial
  overspread(objects, key)     max_d past limit_id     set_mm(objects, key)      permanent set (mm of radius)
  tilt_deg / max_tilt_deg      plane off square         max_twist_deg             largest twist while held
  cocked(objects, key)         landed tilted in a groove
  turned_deg(objects, key)     net turn (angle span) while closed in a groove and not cocked (a seating check)
Offline tests call `update(objects)` once per control step with hand-made ObjectStates (fixture, tool parts).
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field, replace

import numpy as np
import yamkit as kit

# -- specs ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Groove:
    """A groove: its middle at `at` (m from the shaft's end face, into the shaft), `width` along the shaft and
    `diameter` at its bottom."""

    at: float
    width: float
    diameter: float


@dataclass(frozen=True)
class Ring:
    """An elastic or split ring on a shaft that is part of a mechanism's base. `end` is the shaft's end-face center in
    the base's frame and `axis` the direction into the shaft from there ("-Z" for an upright shaft with its end up;
    "X", "-Y", ...). `free_id` is the inner diameter at rest, `thickness` the ring's size along the shaft, `width` its
    radial section. `length` is how far along the shaft it can go (m from the end face); `start` its starting s
    (default: lying on the end face). `limit_id` is the elastic limit (None: no limit), `clear` the diametral
    clearance it needs to slide. A split ring (circlip) has lug holes `lug_gap` apart at rest, their centers `lug_out`
    beyond the inner radius, its split at `lug_angle` (rad, ccw seen from the end, from the shaft frame's first
    axis); a closed ring (O-ring) has lug_gap 0. See the module docstring for the model."""

    name: str
    end: tuple
    shaft_d: float
    free_id: float
    thickness: float
    width: float
    length: float
    axis: str = "-Z"
    grooves: tuple = ()
    stop: float | None = None
    start: float | None = None
    limit_id: float | None = None
    clear: float = 0.0002
    lug_gap: float = 0.0
    lug_out: float = 0.0
    lug_angle: float = math.pi
    tilt_limit: float = 4.0
    seat_tol: float = 0.00005
    grip_n: float = 0.0  # N: total radial force of the ring closed onto the bottom of its first groove (0: no drag)
    mu: float = 0.0  # friction coefficient between the ring and the shaft
    hide: float = 0.15  # m along the shaft from its end face where the start-pose placeholder mesh is put away
    look: kit.Look = kit.Look((0.05, 0.05, 0.06), 0.85)  # matte: a metallic dark ring reflects the dome and vanishes

    @property
    def s0(self) -> float:
        return -self.thickness / 2 if self.start is None else self.start


@dataclass(frozen=True)
class Grip:
    """What a tool holding the ring asks of it this step: `s` where it would put the ring (m along the shaft), `theta`
    the tool's angle about the shaft (rad, ccw from the end; only changes matter), `r` the inner radius it spreads the
    ring to (m; None: it does not spread it), `tilt` (deg) how far it holds the ring's plane off square, `tilt_axis`
    (unit vector, world) the axis of that tilt, `twist` (deg, signed) how far it twists the ring's ends apart."""

    s: float
    theta: float
    r: float | None = None
    tilt: float = 0.0
    tilt_axis: tuple = (1.0, 0.0, 0.0)
    twist: float = 0.0


@dataclass
class RingState:
    """The ring this trial (see the module docstring); lengths in m, angles in deg unless noted."""

    episode: int
    s: float
    r: float  # inner radius now
    theta: float = 0.0  # rad, ccw from the end
    set: float = 0.0  # permanent set (radius)
    max_r: float = 0.0
    engaged: bool = False
    grip: Grip | None = None
    theta_off: float = 0.0  # ring turn minus tool angle, kept from when the tool took hold
    tilt: float = 0.0
    tilt_axis: tuple = (1.0, 0.0, 0.0)
    twist: float = 0.0
    max_tilt: float = 0.0
    max_twist: float = 0.0
    cocked: bool = False
    groove: int | None = None
    turned: float = 0.0  # deg, net turn in the groove: the widest span of the ring's angle since it first closed into
    #                      a groove (not cocked); a back-and-forth wiggle or jitter adds nothing past its own span
    turn_at: float = 0.0  # deg, the ring's unwrapped angle (signed sum of its steps)
    turn_lo: float | None = None  # deg, smallest and largest unwrapped angle while closed in a groove
    turn_hi: float | None = None
    steps: int = 0
    wind: float = 0.0  # m at the lug holes: how far the tool has moved round the shaft since the ring last slipped,
    #                    while the ring sticks to the shaft (the hand and pliers take it up; see reaction loads)
    anchor: float | None = None  # rad: the tool's angle on the shaft where the ring would sit with no wind-up


@dataclass
class _Attach:
    mech: str
    ring: Ring
    spreader: object = None


_RINGS: dict[str, _Attach] = {}  # "mechanism/ring" -> attachment
_STATE: dict[str, RingState] = {}
_FREE: dict[str, tuple | None] = {}  # free tools: name -> pinned base center of mass


def attach(mech: str, ring: Ring, spreader=None) -> str:
    """Put `ring` on the shaft of mechanism `mech` (whose links include `links(ring)`), worked by `spreader`. Returns
    the readout key "mech/ring". Call at module level in the task file."""
    key = f"{mech}/{ring.name}"
    _RINGS[key] = _Attach(mech, ring, spreader)
    return key


def free_tool(mech, com=None):
    """Mark a loose (massed) Mechanism so its base carries the articulation root. `com` (base frame, m) pins the
    base's center of mass, e.g. at a tool's grip point: a two-pad grip barely resists turning about the pads' normal,
    so a hand tool whose weight sits forward of the pads pitches nose-down in the gripper (sim pilot, Nov 10: 57
    degrees). Returns the mechanism unchanged."""
    _FREE[mech.name] = com
    return mech


# -- shaft frame and profile -------------------------------------------------------------------------


def _letter(axis: str) -> tuple[str, int]:
    a = axis.strip().upper()
    return a[-1], (-1 if a.startswith("-") else 1)


def _unit(axis: str) -> np.ndarray:
    k, sg = _letter(axis)
    v = np.zeros(3)
    v["XYZ".index(k)] = sg
    return v


def _rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _local_frame(ring: Ring):
    """In the base's frame: u (into the shaft), e1, e2 with (e1, e2, -u) right-handed, so angles run ccw seen from
    the end."""
    u = _unit(ring.axis)
    out = -u
    e1 = np.array([0.0, 0.0, 1.0]) if abs(out[2]) < 0.5 else np.array([1.0, 0.0, 0.0])
    e1 = e1 - np.dot(e1, out) * out
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(out, e1)
    return u, e1, e2


@dataclass(frozen=True)
class Frame:
    """The shaft in world coordinates (relative to the env origin): end-face center `o`, into-shaft `u`, `e1`, `e2`."""

    o: np.ndarray
    u: np.ndarray
    e1: np.ndarray
    e2: np.ndarray

    def s_of(self, p) -> float:
        return float(np.dot(np.asarray(p, float) - self.o, self.u))

    def angle_of(self, p) -> float:
        d = np.asarray(p, float) - self.o
        return math.atan2(float(np.dot(d, self.e2)), float(np.dot(d, self.e1)))

    def radial(self, p) -> float:
        d = np.asarray(p, float) - self.o
        return float(np.linalg.norm(d - np.dot(d, self.u) * self.u))

    def point(self, s: float, ang: float, rad: float) -> np.ndarray:
        return self.o + s * self.u + rad * (math.cos(ang) * self.e1 + math.sin(ang) * self.e2)


def frame(objects, mech: str, ring: Ring) -> Frame:
    b = objects[mech]
    rot = _rotm(b.quat_xyzw)
    u, e1, e2 = _local_frame(ring)
    return Frame(np.asarray(b.pos, float) + rot @ np.asarray(ring.end, float), rot @ u, rot @ e1, rot @ e2)


def shaft_r(ring: Ring, z: float) -> float:
    """The shaft's radius at z (m from the end face): 0 off the end, a groove's bottom inside it."""
    if z <= 1e-6 or z > ring.length + 1e-6:
        return 0.0
    for g in ring.grooves:
        if abs(z - g.at) <= g.width / 2 + 1e-7:
            return g.diameter / 2
    return ring.shaft_d / 2


def span_r(ring: Ring, s: float) -> float:
    """The largest shaft radius under a ring at s (its section covers s +/- thickness/2)."""
    h = ring.thickness / 2 - 1e-6
    zs = np.linspace(s - h, s + h, 9)
    pts = [shaft_r(ring, float(z)) for z in zs]
    for g in ring.grooves:  # groove edges inside the span
        for e in (g.at - g.width / 2 - 1e-7, g.at + g.width / 2 + 1e-7):
            if s - h < e < s + h:
                pts.append(shaft_r(ring, e))
    return max(pts)


def _snap(ring: Ring, s: float):
    """(groove index, s at its middle) if a ring closing at s is pushed into a groove, else (None, s)."""
    for i, g in enumerate(ring.grooves):
        if abs(s - g.at) <= g.width / 2:
            return i, g.at
    return None, s


def _s_max(ring: Ring) -> float:
    hi = ring.length - ring.thickness / 2
    if ring.stop is not None:
        hi = min(hi, ring.stop - ring.thickness / 2)
    return hi


def _travel(ring: Ring, s: float, target: float, r: float) -> float:
    """Move from s toward target in 0.2 mm steps while the ring (inner radius r) clears the shaft by `clear`."""
    target = min(max(target, ring.s0), _s_max(ring))
    n = int(abs(target - s) / 0.0002) + 1
    ok = s
    for k in range(1, n + 1):
        z = s + (target - s) * k / n
        if span_r(ring, z) > r - ring.clear / 2 + 1e-9:
            break
        ok = z
    return ok


# -- per-step model --------------------------------------------------------------------------------


def state(key: str) -> RingState:
    """The ring's state this trial (a fresh one after every scene reset)."""
    st = _STATE.get(key)
    if st is None or st.episode != kit.episode():
        ring = _RINGS[key].ring
        r = max(ring.free_id / 2, span_r(ring, ring.s0))
        st = _STATE[key] = RingState(kit.episode(), ring.s0, r, max_r=r)
        st.groove = _in_groove(ring, st)
    return st


def _in_groove(ring: Ring, st: RingState):
    for i, g in enumerate(ring.grooves):
        if abs(st.s - g.at) <= (g.width - ring.thickness) / 2 + 1e-6:
            return i
    return None


def _step(key: str, grip: Grip | None) -> RingState:
    att = _RINGS[key]
    ring, st = att.ring, state(key)
    free_r = ring.free_id / 2 + st.set
    theta0 = st.theta
    if grip is not None:
        if not st.engaged:
            st.theta_off = st.theta - grip.theta
        under = span_r(ring, st.s)
        want = grip.r if grip.r is not None else free_r
        st.r = max(want, free_r, under)
        if st.cocked and st.r > ring.shaft_d / 2 + ring.clear / 2:
            st.cocked = False  # spread again: free of the land
        st.s = _travel(ring, st.s, grip.s, st.r)
        if not st.cocked:
            st.theta = grip.theta + st.theta_off - st.wind / (st.r + ring.lug_out)
        st.tilt, st.tilt_axis, st.twist = grip.tilt, tuple(grip.tilt_axis), grip.twist
        st.max_tilt = max(st.max_tilt, abs(grip.tilt))
        st.max_twist = max(st.max_twist, abs(grip.twist))
    st.engaged = grip is not None
    if grip is None:
        st.wind, st.anchor = 0.0, None
    st.grip = grip
    under = span_r(ring, st.s)
    closing_r = max(free_r, under)
    if grip is None or (grip.r is not None and grip.r <= closing_r):
        # nothing (or nothing more) spreading it: the ring closes onto what is under it, into a groove if over one
        if under > 0 and not st.cocked:
            g, s_g = _snap(ring, st.s)
            if g is not None and free_r <= ring.grooves[g].diameter / 2 + ring.seat_tol:
                if abs(st.tilt) > ring.tilt_limit:
                    st.cocked = True  # one side lands on the land (however slowly or from however near it closed)
                else:
                    st.s = s_g
            under = span_r(ring, st.s)
        st.r = ring.shaft_d / 2 if st.cocked else max(free_r, under)
    if grip is None and not st.cocked:
        st.tilt, st.twist = 0.0, 0.0  # let go square: nothing tilts it any more
    st.max_r = max(st.max_r, st.r)
    if ring.limit_id is not None and st.r > ring.limit_id / 2:
        st.set = max(st.set, st.r - ring.limit_id / 2)
    st.groove = _in_groove(ring, st)
    st.turn_at += math.degrees(_wrap(st.theta - theta0))
    if st.groove is not None and not st.cocked and st.r <= ring.grooves[st.groove].diameter / 2 + ring.seat_tol:
        st.turn_lo = st.turn_at if st.turn_lo is None else min(st.turn_lo, st.turn_at)
        st.turn_hi = st.turn_at if st.turn_hi is None else max(st.turn_hi, st.turn_at)
        st.turned = st.turn_hi - st.turn_lo
    st.steps += 1
    return st


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def update(objects) -> None:
    """Advance every attached ring by one control step (the sim calls this; offline tests call it themselves)."""
    for key, att in _RINGS.items():
        if att.mech not in objects:
            continue
        st = state(key)
        fr = frame(objects, att.mech, att.ring)
        grip = att.spreader.grip(att.ring, st, fr, objects) if att.spreader is not None else None
        _step(key, grip)


# -- spreaders ------------------------------------------------------------------------------------------


def lug_holes(ring: Ring, st: RingState, fr: Frame):
    """World positions of the two lug-hole centers (mid-plane) of a split ring now."""
    rad = st.r + ring.lug_out
    gap = ring.lug_gap + 2 * math.pi * (st.r - ring.free_id / 2)
    half = (gap / 2) / rad
    mid = st.theta + ring.lug_angle
    return fr.point(st.s, mid - half, rad), fr.point(st.s, mid + half, rad)


@dataclass(frozen=True)
class TipPair:
    """Two tool tips (circlip pliers) for a split ring's lug holes. `a` and `b` are (object key, tip end point in that
    object's frame); `down` is the direction the tips point in object a's frame (for tilt). A tip engages within
    `catch` (m) of a lug hole across the shaft axis with its end at least `enter` past the ring's near face and at
    most `depth` past its far face; both must engage. Held, the ring's far face sits at the tips' ends."""

    a: tuple
    b: tuple
    down: tuple = (0.0, 0.0, -1.0)
    catch: float = 0.004
    enter: float = 0.0005
    depth: float = 0.003

    def tips(self, objects):
        out = []
        for key, p in (self.a, self.b):
            o = objects[key]
            out.append(np.asarray(o.pos, float) + _rotm(o.quat_xyzw) @ np.asarray(p, float))
        return out

    def grip(self, ring: Ring, st: RingState, fr: Frame, objects):
        if self.a[0] not in objects or self.b[0] not in objects:
            return None
        ta, tb = self.tips(objects)
        # what the tips ask: the ring's far face at their ends, its lug gap their spacing, its turn their angle
        za, zb = fr.s_of(ta), fr.s_of(tb)
        da = (ta - fr.o) - np.dot(ta - fr.o, fr.u) * fr.u
        db = (tb - fr.o) - np.dot(tb - fr.o, fr.u) * fr.u
        gap = float(np.linalg.norm(da - db))
        r = ring.free_id / 2 + (gap - ring.lug_gap) / (2 * math.pi)
        mid = da + db
        theta = math.atan2(float(np.dot(mid, fr.e2)), float(np.dot(mid, fr.e1))) - ring.lug_angle
        s = (za + zb) / 2 - ring.thickness / 2
        # Where the lug holes are: a free ring's holes are where the ring is now. Once the tips are in, the ring
        # deforms, turns and slides with them as far as the shaft lets it, so check them against where it would be.
        probe = st
        if st.engaged:
            r_new = max(r, ring.free_id / 2 + st.set, span_r(ring, st.s))
            th_new = st.theta if st.cocked else theta + st.theta_off
            probe = replace(st, r=r_new, theta=th_new, s=_travel(ring, st.s, s, r_new))
        ha, hb = lug_holes(ring, probe, fr)

        def across(p, h):
            d = p - h
            return float(np.linalg.norm(d - np.dot(d, fr.u) * fr.u))

        pair = min(((ta, ha), (tb, hb)), ((ta, hb), (tb, ha)), key=lambda c: across(*c[0]) + across(*c[1]))
        near, far = probe.s - ring.thickness / 2, probe.s + ring.thickness / 2
        for p, h in pair:
            z = fr.s_of(p)
            if across(p, h) > self.catch or not (near + self.enter <= z <= far + self.depth):
                return None
        rot_a = _rotm(objects[self.a[0]].quat_xyzw)
        d = rot_a @ np.asarray(self.down, float)
        d /= np.linalg.norm(d)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(d, fr.u))))))
        ax = np.cross(fr.u, d)
        ax = tuple(ax / np.linalg.norm(ax)) if np.linalg.norm(ax) > 1e-9 else (1.0, 0.0, 0.0)
        twist = math.degrees(math.atan2(zb - za, max(gap, 1e-6)))
        return Grip(s=s, theta=theta, r=r, tilt=tilt, tilt_axis=ax, twist=twist)


@dataclass(frozen=True)
class HandleSpring:
    """A yamkit law for a tool handle: a return spring of `k` (N/m or N*m/rad) toward `rest`, which may lie past the
    joint's stop to preload it (rest = stop - preload / k), with implicit damping `damp`."""

    rest: float
    k: float
    damp: float = 1.0

    def drive(self, key, q, qd, J, mem):
        return self.rest, self.k, 0.0, self.damp


# -- reaction loads on the shaft --------------------------------------------------------------------
# A ring closed onto the shaft grips it with its own spring. Turned by the pliers' tips in its lug holes, it drags the
# shaft round with it until the friction at the shaft gives way: up to slip_force() at the lug holes, a torque of
# mu * grip * shaft radius about the shaft's axis. That torque acts on the shaft; the hand turning the pliers supplies
# it (the pliers themselves are not pushed back). A loose shaft whose footing gives way first turns with the pliers,
# and the ring does not turn on it. A tool pushing the ring sideways would push the shaft sideways too; that is not
# modelled (the turn is about the axis). While the ring is spread clear of the shaft (it slides only with `clear` to
# spare) it touches nothing and pushes nothing. Sticking is a stiff spring at the lug holes (DRAG_K), so that the
# wind-up is small next to a 15-degree check turn; DRAG_C damps it. Both act every physics step.

DRAG_K = 10000.0  # N/m: the stick, at the lug holes (2.5 mm of wind-up at 25 N)
DRAG_C = 20.0  # N*s/m


def grip_force(ring: Ring, st: RingState) -> float:
    """N: the ring's total radial force on the shaft now. Closed onto the shaft it grows with its stretch (linear
    elastic, `grip_n` at the groove bottom); held open by the tool it eases off over `seat_tol` and is 0 once the
    tool holds it clear; a cocked ring is jammed on the land and not modelled here (0)."""
    if ring.grip_n <= 0 or not ring.grooves or st.cocked:
        return 0.0
    under = span_r(ring, st.s)
    free_r = ring.free_id / 2 + st.set
    if under <= free_r:
        return 0.0
    n = ring.grip_n * (under - free_r) / (ring.grooves[0].diameter / 2 - ring.free_id / 2)
    if st.grip is not None and st.grip.r is not None:
        n *= min(1.0, max(0.0, (under + ring.seat_tol - st.grip.r) / ring.seat_tol))
    return n


def slip_force(ring: Ring, st: RingState) -> float:
    """N: the turning force at the lug holes at which the ring slips round the shaft (mu * grip * shaft radius / lug
    radius)."""
    return ring.mu * grip_force(ring, st) * span_r(ring, st.s) / (st.r + ring.lug_out)


def drag(ring: Ring, st: RingState, angle: float, dt: float) -> float:
    """One physics step of the stick-slip: `angle` (rad) is the tool's angle about the shaft in the shaft's frame (as
    Grip.theta). While the ring sticks, the tool's turn since it took hold is wind-up at the lug holes, and the ring
    passes the spring force of that wind-up to the shaft; past slip_force() the ring slips round and the wind-up stays
    at its limit. Returns the turning force (N at the lug holes, positive along the ring's turn) and keeps the
    wind-up in st.wind."""
    f_max = slip_force(ring, st) if st.engaged else 0.0
    if f_max <= 0.0:
        if st.wind:  # let go of the shaft: the ring stays where it is, the tool's offset takes up the wind-up
            st.theta_off -= st.wind / (st.r + ring.lug_out)
        st.wind, st.anchor = 0.0, None
        return 0.0
    lug = st.r + ring.lug_out
    if st.anchor is None:
        st.anchor = angle - st.wind / lug
    w_max = f_max / DRAG_K
    wind = _wrap(angle - st.anchor) * lug
    if abs(wind) > w_max:  # the ring slips round the shaft
        wind = math.copysign(w_max, wind)
        st.anchor = angle - wind / lug
    rate = (wind - st.wind) / dt
    st.wind = wind
    return max(-f_max, min(f_max, DRAG_K * wind + DRAG_C * rate))


def _body(base, key: str):
    """(articulation, body index) for an object key "mech" or "mech/link"."""
    mech, _, link = key.partition("/")
    art = base.scene[mech]
    return art, list(art.body_names).index(link or "base")


def _push(base, dt: float) -> None:
    """Before each physics step: each held ring's turning drag on its shaft."""
    import torch
    from types import SimpleNamespace

    np_ = lambda t: getattr(t, "torch", t)[0].detach().cpu().numpy().astype(np.float64)  # noqa: E731
    for key, att in _RINGS.items():
        st = state(key)
        sp = att.spreader
        if not st.engaged or not isinstance(sp, TipPair):
            drag(att.ring, st, 0.0, dt)
            continue
        try:
            parts = {k: _body(base, k) for k in (att.mech, sp.a[0], sp.b[0])}
        except (KeyError, ValueError):
            continue
        objs = {}
        for k, (art, i) in parts.items():
            d = art.data
            objs[k] = SimpleNamespace(pos=np_(d.body_link_pos_w)[i], quat_xyzw=np_(d.body_link_quat_w)[i])
        fr = frame(objs, att.mech, att.ring)
        ta, tb = sp.tips(objs)
        mid = (ta - fr.o) + (tb - fr.o)
        angle = math.atan2(float(np.dot(mid, fr.e2)), float(np.dot(mid, fr.e1))) - att.ring.lug_angle
        f = drag(att.ring, st, angle, dt)
        if f == 0.0:
            continue
        art, i = parts[att.mech]
        dev = getattr(art.data.body_link_pos_w, "torch", art.data.body_link_pos_w).device
        vec = lambda v: torch.tensor(np.asarray(v, np.float32).reshape(1, 1, 3), device=dev)  # noqa: E731
        # the ring's friction on the shaft: a torque about the shaft's axis (world frame); angles run ccw seen from
        # the end, i.e. about -u
        torque = -f * (st.r + att.ring.lug_out) * fr.u
        art.instantaneous_wrench_composer.add_forces_and_torques_index(torques=vec(torque), body_ids=[i],
                                                                       is_global=True)


def _hook_physics(base) -> None:
    """Run _push before every physics step (the scene writes the wrenches into the simulator in that call)."""
    scene = base.scene
    if getattr(scene, "_ringkit_push", False) or not any(a.ring.grip_n > 0 for a in _RINGS.values()):
        return
    scene._ringkit_push = True
    write = scene.write_data_to_sim
    dt = float(getattr(base, "physics_dt", 1 / 240))

    def write_data_to_sim(*a, **k):
        try:
            _push(base, dt)
        except Exception as e:  # noqa: BLE001
            if not _VIS.get("push_failed"):
                import traceback

                _note(f"ringkit: no reaction loads ({e!r})\n{traceback.format_exc()}")
            _VIS["push_failed"] = True
            raise
        return write(*a, **k)

    scene.write_data_to_sim = write_data_to_sim


# -- the ring's joints ------------------------------------------------------------------------------


SHOW_STEPS = 12  # control steps the placeholder mesh stands in for the markers (they show from about frame 5)


def _band_mesh(ring: Ring, r: float, n: int = 48) -> kit.Mesh:
    """The ring as a mesh at inner radius r in the shaft frame at s = 0 (the link's frame), split at lug_angle."""
    u, e1, e2 = _local_frame(ring)
    rm = r + ring.width / 2
    gap = 0.004 / rm if ring.lug_gap > 0 else 0.0
    angs = np.linspace(ring.lug_angle + gap / 2, ring.lug_angle + 2 * math.pi - gap / 2, n)
    path = [rm * (math.cos(a) * e1 + math.sin(a) * e2) for a in angs]
    if ring.lug_gap == 0:
        path.append(path[0])
    return kit.band(path, ring.thickness, ring.width, ring.look, normal=tuple(-u))


def links(ring: Ring, mass: float = 0.05) -> tuple:
    """The ring's joints for its fixture mechanism (see the module docstring): invisible, collider-less, gravity off;
    the kit writes their positions every step. A fourth link, "<name>_shown", carries a mesh of the ring at its start
    pose for the first SHOW_STEPS control steps (new marker prims take a few rendered frames to appear, and setup
    shots are taken at frame 2); the kit then moves it to s = `hide` (choose a point inside the
    table or the fixture)."""
    k, sg = _letter(ring.axis)
    _, e1, _ = _local_frame(ring)
    k1 = "XYZ"[int(np.argmax(np.abs(e1)))]
    s_hi = _s_max(ring)
    lo, hi = sorted((sg * ring.s0, sg * s_hi))
    common = dict(anchor=tuple(ring.end), mass=mass, damping=0.0, floating=True)
    return (kit.Link(f"{ring.name}_axial", "prismatic", k, lower=lo - 0.001, upper=hi + 0.001, start=sg * ring.s0,
                     **common),
            kit.Link(f"{ring.name}_turn", "revolute", k, lower=-math.pi - 0.01, upper=math.pi + 0.01, start=0.0,
                     **common),
            kit.Link(ring.name, "prismatic", k1, lower=-0.001, upper=0.02, start=0.0, **common),
            kit.Link(f"{ring.name}_shown", "prismatic", k, lower=min(lo, sg * ring.hide) - 0.001,
                     upper=max(hi, sg * ring.hide) + 0.001, start=sg * ring.s0,
                     meshes=(_band_mesh(ring, max(ring.free_id / 2, span_r(ring, ring.s0))),), **common))


def _joint_values(ring: Ring, st: RingState) -> dict:
    _, sg = _letter(ring.axis)
    shown = ring.s0 if st.steps <= SHOW_STEPS else ring.hide
    return {f"{ring.name}_axial": sg * st.s, f"{ring.name}_turn": -sg * _wrap(st.theta),
            ring.name: st.r - ring.free_id / 2, f"{ring.name}_shown": sg * shown}


def _write_joints(base) -> None:
    for key, att in _RINGS.items():
        try:
            art = base.scene[att.mech]
        except KeyError:
            continue
        names = list(art.joint_names)
        for jn, v in _joint_values(att.ring, state(key)).items():
            if f"{jn}_joint" in names:
                j = names.index(f"{jn}_joint")
                kit._put_joint(art, j, v)


# -- USD: inertia for the ring's links, the root on a free tool's base ---------------------------------

_orig_mech_usda = kit._mech_usda


def _mech_usda(mech):
    text = _orig_mech_usda(mech)
    ring_links = {f"{a.ring.name}{x}" for a in _RINGS.values() if a.mech == mech.name for x in ("_axial", "_turn", "", "_shown")}
    for name in ring_links:
        head = f'    def Xform "{name}" ('
        if head not in text:
            continue
        i = text.index(head)
        m = text.index("float physics:mass", i)
        eol = text.index("\n", m) + 1
        text = text[:eol] + "        float3 physics:diagonalInertia = (1e-06, 1e-06, 1e-06)\n" \
                            "        point3f physics:centerOfMass = (0, 0, 0)\n" + text[eol:]
    if mech.name in _FREE and mech.mass is not None:
        root = 'def Xform "Root" (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)\n{\n'
        if root not in text:
            raise RuntimeError("ringkit: unexpected yamkit USD layout for the root")
        text = text.replace(root, 'def Xform "Root"\n{\n', 1)
        old = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
               '"PhysxRigidBodyAPI"]\n    )\n    {\n')
        if old not in text:
            raise RuntimeError("ringkit: unexpected yamkit USD layout for the base")
        settings = [ln for ln in text.splitlines(True) if ln.startswith("    ") and "physxArticulation:" in ln
                    and not ln.startswith("        ")]
        text = "".join(ln for ln in text.splitlines(True) if ln not in settings or ln.startswith("        "))
        new = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
               '"PhysxRigidBodyAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n    )\n    {\n'
               + "".join("    " + ln for ln in settings))
        text = text.replace(old, new, 1)
        com = _FREE[mech.name]
        if com is not None:
            i = text.index('    def Xform "base" (')
            m = text.index("float physics:mass", i)
            eol = text.index("\n", m) + 1
            text = text[:eol] + f"        point3f physics:centerOfMass = {kit._fmt(com)}\n" + text[eol:]
    return text


kit._mech_usda = _mech_usda


# -- visuals ------------------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
SEGMENTS = 40


def _note(msg: str) -> None:
    """A visuals problem: to stderr, and to the file named by RINGKIT_LOG if set (run scripts filter stderr)."""
    print(msg, file=sys.stderr, flush=True)
    if os.environ.get("RINGKIT_LOG"):
        with open(os.environ["RINGKIT_LOG"], "a") as f:
            f.write(msg + "\n")


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        protos, index = {}, {}
        for key, att in _RINGS.items():
            lk = att.ring.look
            index[key] = len(protos)
            protos[f"ring{len(protos)}"] = sim_utils.CuboidCfg(
                size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(kit._linear(c) for c in lk.color), roughness=lk.roughness,
                    metallic=lk.metallic))
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/ringkit",
                                                                       markers=protos))
        _VIS["index"] = index
    except Exception as e:  # visuals are optional: scoring never depends on them
        _note(f"ringkit: no ring visuals ({e!r})")
        _VIS["failed"] = True
    return _VIS["markers"]


def _axis_angle(axis, ang) -> np.ndarray:
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


def _quat_xyzw(R) -> tuple:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return tuple(q)


def pieces(key: str, objects) -> list:
    """The drawn ring as (center, rotation matrix, size) boxes in world coordinates (relative to the env origin):
    band segments round the arc and, for a split ring, a lug ear at each end."""
    att = _RINGS[key]
    ring, st = att.ring, state(key)
    fr = frame(objects, att.mech, ring)
    rm = st.r + ring.width / 2
    split = ring.lug_gap > 0
    gap_vis = (0.004 + 2 * math.pi * (st.r - ring.free_id / 2)) if split else 0.0
    half = (gap_vis / 2) / rm
    a0, a1 = st.theta + ring.lug_angle + half, st.theta + ring.lug_angle + 2 * math.pi - half
    center = fr.o + st.s * fr.u
    tilt = _axis_angle(st.tilt_axis, math.radians(st.tilt)) if st.tilt else np.eye(3)
    dz = (math.tan(math.radians(st.twist)) * (ring.lug_gap or 0.01)) if st.twist else 0.0
    out = []
    n = SEGMENTS

    def put(ang, rad, f, size):  # size: (radial, along the arc, along the shaft)
        radial = math.cos(ang) * fr.e1 + math.sin(ang) * fr.e2
        tang = np.cross(-fr.u, radial)
        p = center + rad * radial + (f - 0.5) * dz * fr.u
        R = np.column_stack([radial, tang, -fr.u])  # right-handed: radial x tang = outward
        out.append((center + tilt @ (p - center), tilt @ R, size))

    for k in range(n):
        f = (k + 0.5) / n
        ang = a0 + f * (a1 - a0)
        put(ang, rm, f, (ring.width, rm * (a1 - a0) / n * 1.15, ring.thickness))
    if split:
        ear = ring.lug_out + 0.0025
        for f, ang in ((0.0, a0 + 0.0025 / rm), (1.0, a1 - 0.0025 / rm)):
            put(ang, st.r + ear / 2, f, (ear, 0.005, ring.thickness))
    return out


def _draw(objects, origin) -> None:
    m = _markers()
    if m is None:
        return
    tr, qs, sc, ids = [], [], [], []
    for key in _RINGS:
        if _RINGS[key].mech not in objects:
            continue
        for c, R, size in pieces(key, objects):
            tr.append(tuple(c + origin)), qs.append(_quat_xyzw(R)), sc.append(size), ids.append(_VIS["index"][key])
    if not tr:
        return
    if os.environ.get("RINGKIT_LOG") and not _VIS.get("noted"):
        _VIS["noted"] = True
        _note(f"ringkit: drawing {len(tr)} pieces, first at {np.round(tr[0], 4).tolist()} size {sc[0]}")
    # this Isaac Lab takes (x, y, z, w) quaternions, as it reports them (pourkit's markers verified it)
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                marker_indices=np.array(ids, int))


# -- hook into yam_arena.scene (after yamkit's) ------------------------------------------------------


def _patch(scene) -> None:
    if getattr(scene, "_ringkit", False):
        return
    scene._ringkit = True
    inner = scene.object_states
    last = {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if key[1] is None or key != last["key"]:  # once per control step
            last["key"] = key
            update(out)
            _hook_physics(base)
            _write_joints(base)
            if os.environ.get("RINGKIT_DEBUG"):
                for k in _RINGS:
                    st = state(k)
                    _note(f"ringkit {k} s={st.s * 1000:.2f}mm id={2 * st.r * 1000:.2f}mm "
                          f"th={math.degrees(st.theta):.1f} eng={st.engaged} g={st.groove} cock={st.cocked} "
                          f"tilt={st.tilt:.1f} tw={st.twist:.1f} turned={st.turned:.1f} set={st.set * 1000:.2f}")
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    import traceback

                    _note(f"ringkit: visuals off ({e!r})\n{traceback.format_exc()}")
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


def _draw_start(env) -> None:
    """Draw every ring at its start pose right after a reset, so the settling steps and the reset renders that follow
    show it in the first camera frame (new marker prims take a few rendered frames to appear)."""
    from types import SimpleNamespace

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    objs = {}
    for att in _RINGS.values():
        try:
            d = base.scene[att.mech].data
        except KeyError:
            continue
        i = list(base.scene[att.mech].body_names).index("base")
        pick = lambda t: getattr(t, "torch", t)[0, i].detach().cpu().numpy().astype(np.float64)  # noqa: E731
        objs[att.mech] = SimpleNamespace(pos=pick(d.body_link_pos_w) - origin, quat_xyzw=pick(d.body_link_quat_w))
    keep = dict(_STATE)
    _STATE.clear()  # a fresh ring at its start pose; the trial's own state is untouched
    try:
        _draw(objs, origin)
    finally:
        _STATE.clear()
        _STATE.update(keep)


def _patch_embodiment() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_ringkit_start", False):
        return
    cls._ringkit_start = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_ringkit_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            try:
                _draw_start(env)
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                _note(f"ringkit: no start visuals ({e!r})")
            return out

        env.reset, env._ringkit_reset = reset_and_draw, True

    cls._ensure_sim = _ensure_sim


_patch_embodiment()


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that also adds the ring step after yamkit's patch
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- scorer readouts --------------------------------------------------------------------------------


def position(objects, key: str) -> float:
    """The ring's mid-plane along the shaft, m from the end face (into the shaft)."""
    return state(key).s


def groove(objects, key: str):
    """Index of the groove the ring's section lies in now (closed or not), or None."""
    return state(key).groove


def seated(objects, key: str) -> bool:
    """In a groove, closed onto its bottom, not cocked and not held by a tool."""
    st, ring = state(key), _RINGS[key].ring
    return (st.groove is not None and not st.cocked and not st.engaged
            and st.r <= ring.grooves[st.groove].diameter / 2 + ring.seat_tol)


def engaged(objects, key: str) -> bool:
    """A tool holds the ring now."""
    return state(key).engaged


def inner_d(objects, key: str) -> float:
    return 2 * state(key).r


def max_d(objects, key: str) -> float:
    """The largest inner diameter the ring reached this trial (m)."""
    return 2 * state(key).max_r


def overspread(objects, key: str) -> bool:
    """Spread past its elastic limit at any point this trial (irreversible: it has taken a set)."""
    ring = _RINGS[key].ring
    return ring.limit_id is not None and state(key).max_r > ring.limit_id / 2 + 1e-9


def set_mm(objects, key: str) -> float:
    return 1000 * state(key).set


def tilt_deg(objects, key: str) -> float:
    return abs(state(key).tilt)


def max_tilt_deg(objects, key: str) -> float:
    return state(key).max_tilt


def max_twist_deg(objects, key: str) -> float:
    return state(key).max_twist


def cocked(objects, key: str) -> bool:
    return state(key).cocked


def turned_deg(objects, key: str) -> float:
    """Net degrees the ring turned while closed in a groove and not cocked, this trial (a seating check): the span
    between the smallest and largest angle it reached there, so a wiggle back and forth counts only its width."""
    return state(key).turned
