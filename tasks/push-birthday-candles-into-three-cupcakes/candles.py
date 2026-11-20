"""A birthday candle stuck in something soft (a cupcake, the Play-Doh in the egg cup), as the simulation holds it:
shared by the task file and its trial scenes.

A rigid body cannot hold a candle that has gone into it, and a weld would hold it rigidly at any depth. So every
candle carries three hidden bodies at its tip (rods inside the candle, carried weightless), joined in a chain:

    host --weld, made when the tip goes in--> hole --hinge (level axis 1)--> ring --hinge (level axis 2)--> sleeve
         --slide along the candle--> candle

- A free candle carries the chain along: its two hinges are sprung stiff and its slide rests at 0, so the three
  bodies stay at the tip, inside the candle.
- When the tip has gone into a host's top (checked once per control step: at most 2.5 mm in by then, inside the
  top's rim, the candle within 45 degrees of the host's axis) the weld joins the hole to the host as they stand: the
  hole is the line along which the candle went in, and the point where it went in is where it turns.
- Along its length the candle meets a resistance and no wall: a stiff spring on the slide, capped at the resistance,
  whose rest follows the candle (it holds like dry friction and slides at the cap): the host's ``resist_top`` while
  the tip is less than ``top_depth`` in (a cupcake's icing cap), its ``resist_body`` beyond; the same on the way out.
- Sideways the host holds it by the two hinges' springs, through the point where it went in, the more the deeper: a
  power law through the host's stiffness at 8 and at 15 mm, written to both hinges every control step. The springs
  rest along the candle's own direction at the last moment it went deeper: a crooked push leaves a crooked candle.
- Leaning more than the host's ``tear_deg`` from that direction it tears out, and pulled back to the surface it comes out:
  the weld is cut and the candle is free again; a cupcake keeps the hole (a candle going in there later meets no
  icing).

A candle is two pieces, built whole: the lower 38 mm (which go through the hosts: a candle is at most 35 mm in a
cupcake) and the upper 25 mm with the wick, which touch the hosts like anything else. Between them, 38 mm from the tip
(just under the pads of a hand that pinches it 1 cm under the wick end, where a pinched candle bends most), two
sprung hinges stand for the wax's bending and the cake's give at the tip; their springs' torque is the bending moment, and at
``SNAP_MOMENT`` a weld between the pieces is cut: the candle has snapped, the lower piece stays where it is (in a
cupcake, say) and the upper one in the hand. The hidden bodies touch nothing. Numbers: the one table in the task's README, all assumed until
the owner's kitchen test.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import mechkit as mk
from mechkit import body, cylinder, hinge, slide, weld

GRAVITY = 9.81
MECHKIT_HELPER = True  # declares for the task file that imports it (mechkit._claim)

# --- the candle (README, "Materials": source beside each) ---
CANDLE_L, CANDLE_R, CANDLE_MASS = 0.063, 0.0025, 0.001  # Wilton 2 1/2 in birthday candle (a)
# The simulated candle weighs 10 g (a stand-in): a 0.4 g piece pinched with 12 N went through the pads (run w7c-t2,
# the kit's lesson on light parts between a hand and a joint). Its weight over the real 1 g is carried for it every
# step, so it stands, leans and falls under the real candle's weight, only more slowly (ten times the inertia).
STANDIN_MASS = 0.010
WICK_L = 0.004  # drawn only (a)
PASSING_L = 0.038  # the lower piece, which goes through the hosts; the candle breaks there
SNAP_MOMENT = 0.02  # N m (a): the bending moment at which the wax snaps
# N m/rad (a): the candle's bending and the cake's give at the tip, lumped (the hidden pivot holds the tip rigidly
# sideways). The wax alone (paraffin, E 0.2 GPa, 40 mm free) would give 0.46 and snap at 2.5 degrees: a 1 mm drift of
# the hand; 0.15 snaps it at 7.6 degrees, about 4 mm of drift with the tip in the cake (a crooked push, not a tremor)
BEND_K = 0.15
BEND_MOST = math.radians(30.0)
HOLD_AT = 0.008  # m: the depth at which a host's k_at is given

# --- how the chain is built (not material numbers) ---
HIDDEN_MASS, HIDDEN_L, HIDDEN_R = 0.002, 0.020, 0.001  # each hidden rod, carried weightless
SLIDE_K = 5000.0  # N/m: the capped spring of the slide (0.2 mm of give at 1 N, 0.6 mm at 3 N)
SLIDE_DAMP_S, SLIDE_SLACK = 0.001, 0.00005  # its damping (stiffness x s: 5 N s/m; 50 let a 1 g candle creep at 0.9 of its resistance, run w7c-a2); how far past the give before its rest follows
FREE_K, FREE_DAMP_S = 0.01, 0.01  # N m/rad and s: the hinges' spring while the candle is free (keeps the chain in line)
HOLD_DAMP_S = 0.03  # s: the hinges' damping while the candle is in a host, stiffness x this
ENTRY_BELOW, ENTRY_LEAN_DEG = 0.0025, 45.0  # how far in the tip may be when the weld is made; the most lean then
OUT_DEPTH, OUT_AFTER = 0.0003, 0.001  # pulled out: back within 0.3 mm of the surface, after being 1 mm in
OLD_HOLE = 0.003  # a candle going in within 3 mm of an earlier hole meets no icing
DEEPER = 0.0002  # m: going deeper by more than this since the deepest so far moves the springs' rest to the candle


@dataclass
class Host:
    """Something soft a candle can stand in. ``top``: the height of its top surface over the body's origin, along
    the body's z; ``rim_inside``: how far from that axis a tip may go in (m)."""

    name: str
    top: float
    rim_inside: float
    resist_top: float  # N along the candle while its tip is less than top_depth in
    top_depth: float
    resist_body: float  # N beyond
    k_at: float  # N m/rad against leaning at HOLD_AT (8 mm)
    tear_deg: float
    k_15: float = 0.0  # N m/rad at 15 mm: with k_at, a power law through both (0: rising with the square)


HOSTS: dict[str, Host] = {}


def host(name: str, top: float, rim_inside: float, *, resist_top: float, top_depth: float, resist_body: float, k_at: float,
         tear_deg: float, k_15: float = 0.0) -> Host:  # fmt: skip
    h = Host(name, float(top), float(rim_inside), float(resist_top), float(top_depth), float(resist_body), float(k_at), float(tear_deg), float(k_15))
    HOSTS[name] = h
    return h


HOLD_AT_15 = 0.015  # m: the depth at which a host's k_15 is given


def hold_exponent(h: Host) -> float:
    """The power of the depth by which the hold rises: through k_at at 8 mm and k_15 at 15 mm, or 2."""
    return math.log(h.k_15 / h.k_at) / math.log(HOLD_AT_15 / HOLD_AT) if h.k_15 > 0 else 2.0


def hold_stiffness(depth: float, h: Host) -> float:
    """N m/rad with which host ``h`` holds a candle ``depth`` m in against leaning."""
    d = max(float(depth), 0.0)
    return h.k_at * (d / HOLD_AT) ** hold_exponent(h)


def resistance(depth: float, h: Host, old_hole: bool = False) -> float:
    """N along the candle at ``depth`` m in."""
    return h.resist_body if old_hole or depth >= h.top_depth else h.resist_top


def follow(rest: float, travel: float, cap: float, k: float = SLIDE_K) -> float:
    """Where the slide's capped spring rests from now on: where it rested, unless the candle has gone past the give
    and the slack from there; then the give behind the candle (the spring pulls at its cap against the motion)."""
    give = cap / k
    if travel < rest - give - SLIDE_SLACK:
        return travel + give
    if travel > rest + give + SLIDE_SLACK:
        return travel - give
    return rest


@dataclass
class Candle:
    """One candle and its state in a trial."""

    name: str
    hosts: tuple[str, ...]  # what it can go into
    built_in: str | None = None  # the host it is built in, else None
    built_depth: float = 0.0
    host: str | None = None  # the host it is in now
    entry_below: float = 0.0  # how far in the tip was when the weld was made
    entry_travel: float = 0.0  # the slide's travel then
    rest: float = 0.0  # the slide spring's rest
    cap: float = 0.0
    k_now: float = -1.0
    deepest: float = 0.0
    old_hole: bool = False
    lean_rest: tuple[float, float] = (0.0, 0.0)  # the two hinges' rests: the candle's direction when it last went deeper
    slide_k: float = SLIDE_K
    mass: float = 0.001  # kg: the simulated candle's (both pieces)
    follows: bool = True  # False: the slide's rest stays where it was put (a trial of the resistance)
    events: list = field(default_factory=list)

    @property
    def tip_travel(self) -> float:
        """The slide's travel at which the chain sits at the candle's tip (where a free candle carries it): 0 for a
        candle built free, its built depth for one built standing in a host."""
        return self.built_depth if self.built_in is not None else 0.0

    snapped: bool = False

    @property
    def parts(self) -> tuple[str, str, str]:
        return f"{self.name}_hole", f"{self.name}_ring", f"{self.name}_sleeve"

    @property
    def hidden(self) -> tuple[str, ...]:
        return (*self.parts, f"{self.name}_bend", f"{self.name}_bend2")


CANDLES: dict[str, Candle] = {}
HOLES: dict[str, list] = {}  # host -> points (in its frame, from its top centre) where a candle came out again


def declare(name: str, tip, quat, hosts, color, *, built_in: str | None = None, built_depth: float = 0.0, others=(), bands=(),
            mass: float = STANDIN_MASS, slide_k: float = SLIDE_K, slide_damp_s: float = SLIDE_DAMP_S, follows: bool = True,
            friction=mk.GRIPPY, friction_combine: str = "average") -> Candle:  # fmt: skip
    """A candle whose tip (the lower end of its axis) stands at ``tip`` turned by ``quat`` (its axis is the body's z),
    with its hidden chain and a weld (not made) to each of ``hosts``, except the one to ``built_in``, which holds
    from the start: the candle is built ``built_depth`` deep in it. ``others``: further bodies the hidden chain goes
    through, and the candle's lower part too (shapes that take a host's place). ``bands``: (height from the tip, colour)
    of thin drawn rings round the candle."""
    rot = mk.quat_to_matrix(quat)
    axis = rot[:, 2]
    tip = np.asarray(tip, dtype=float)
    top = f"{name}_top"
    hole, ring, sleeve = (f"{name}_hole", f"{name}_ring", f"{name}_sleeve")
    # the two pieces go through each other: they are not jointed directly (the bending joint's bodies lie between) and
    # their faces meet at the break (run w7c-b3: touching there, they bent each other and two candles snapped at rest)
    lower = cylinder(CANDLE_R, PASSING_L, pos=(0.0, 0.0, PASSING_L / 2), passes=(*hosts, *others, top), color=color)
    upper = cylinder(CANDLE_R, CANDLE_L - PASSING_L, pos=(0.0, 0.0, (CANDLE_L - PASSING_L) / 2), passes=(name,), color=color)
    wick = cylinder(0.0005, WICK_L, pos=(0.0, 0.0, CANDLE_L - PASSING_L + WICK_L / 2), color=(0.05, 0.05, 0.05), solid=False)
    low_rings = [cylinder(CANDLE_R + 0.0001, 0.0015, pos=(0.0, 0.0, z), color=c, solid=False) for z, c in bands if z < PASSING_L]
    top_rings = [cylinder(CANDLE_R + 0.0001, 0.0015, pos=(0.0, 0.0, z - PASSING_L), color=c, solid=False) for z, c in bands if z >= PASSING_L]
    share = PASSING_L / CANDLE_L
    body(name, [lower, *low_rings], pos=tuple(tip), quat=tuple(quat), mass=mass * share, color=color, friction=tuple(friction),
         friction_combine=friction_combine)  # fmt: skip
    root = tip + axis * PASSING_L  # where the candle bends and breaks
    body(top, [upper, wick, *top_rings], pos=tuple(root), quat=tuple(quat), mass=mass * (1 - share), color=color, friction=tuple(friction),
         friction_combine=friction_combine)  # fmt: skip
    bend, bend2 = f"{name}_bend", f"{name}_bend2"
    for part, extra in ((bend, (bend2, top)), (bend2, (bend, top))):
        body(part, [cylinder(HIDDEN_R, 0.008, pos=(0.0, 0.0, 0.0), passes=(name, top, hole, ring, sleeve, *hosts, *others, *extra), color=color)],
             pos=tuple(root), quat=tuple(quat), mass=HIDDEN_MASS, color=color)
    weld(f"{name}_whole", bend, name)  # cut when it snaps
    hinge(f"{name}_flex1", bend2, bend, anchor=tuple(root), axis=tuple(rot[:, 0]), limits=(-BEND_MOST, BEND_MOST), spring=BEND_K, damping=BEND_K * 0.002)
    hinge(f"{name}_flex2", top, bend2, anchor=tuple(root), axis=tuple(rot[:, 1]), limits=(-BEND_MOST, BEND_MOST), spring=BEND_K, damping=BEND_K * 0.002)
    pivot = tip + axis * built_depth if built_in is not None else tip  # where it went in
    # every body of one candle goes through every other (run w7c-b4: the pivot's rods, 15 to 35 mm over the tip, and the
    # bending joint's, 34 to 42 mm, touched and pushed the candle 1 mm up and bent it until two candles snapped at rest)
    through = (name, f"{name}_top", f"{name}_bend", f"{name}_bend2", *hosts, *others)
    for part, extra in ((hole, (ring, sleeve)), (ring, (hole, sleeve)), (sleeve, (hole, ring))):
        body(part, [cylinder(HIDDEN_R, HIDDEN_L, pos=(0.0, 0.0, HIDDEN_L / 2), passes=(*through, *extra), color=color)],
             pos=tuple(pivot), quat=tuple(quat), mass=HIDDEN_MASS, color=color)
    hinge(f"{name}_tilt1", ring, hole, anchor=tuple(pivot), axis=tuple(rot[:, 0]), limits=(-math.pi / 2, math.pi / 2),
          spring=FREE_K, damping=FREE_K * FREE_DAMP_S)
    hinge(f"{name}_tilt2", sleeve, ring, anchor=tuple(pivot), axis=tuple(rot[:, 1]), limits=(-math.pi / 2, math.pi / 2),
          spring=FREE_K, damping=FREE_K * FREE_DAMP_S)
    slide(f"{name}_depth", name, sleeve, axis=tuple(axis), limits=(-0.040, 0.040), spring=slide_k, damping=slide_k * slide_damp_s)
    for h in hosts:
        weld(f"{name}_in_{h}", hole, h, made=(h == built_in))
    c = Candle(name, tuple(hosts), built_in, float(built_depth), slide_k=float(slide_k), follows=bool(follows), mass=float(mass))
    CANDLES[name] = c
    return c


def hidden_shapes() -> list[tuple[str, int]]:
    return [(part, 0) for c in CANDLES.values() for part in c.hidden]


def bending(c: Candle) -> float:
    """The bending moment (N m) the candle's two bending springs carry."""
    a, b = mk.joint(f"{c.name}_flex1"), mk.joint(f"{c.name}_flex2")
    return BEND_K * math.hypot(a, b)


def _set_hold(c: Candle, k: float, damp_s: float) -> None:
    for t in ("tilt1", "tilt2"):
        mk.set_drive(f"{c.name}_{t}", stiffness=k, damping=k * damp_s)
    c.k_now = k


def reset() -> None:
    """At every reset (in the simulator): each candle free, or in the host it was built in; the chains' springs as
    built for that."""
    HOLES.clear()
    for c in CANDLES.values():
        c.host, c.events, c.deepest, c.old_hole, c.rest, c.k_now, c.lean_rest, c.snapped = None, [], 0.0, False, 0.0, -1.0, (0.0, 0.0), False
        if c.built_in is not None:
            c.host, c.entry_below, c.entry_travel, c.deepest = c.built_in, c.built_depth, 0.0, c.built_depth
        if mk.MECH.environment is None:
            continue
        h = HOSTS.get(c.host) if c.host else None
        c.cap = resistance(c.built_depth, h) if h else 3.0
        mk.set_spring(f"{c.name}_depth", rest=0.0, most=c.cap)
        for t in ("tilt1", "tilt2"):
            mk.set_spring(f"{c.name}_{t}", rest=0.0)
        _set_hold(c, hold_stiffness(c.built_depth, h), HOLD_DAMP_S) if h else _set_hold(c, FREE_K, FREE_DAMP_S)


def weightless() -> None:
    """The hidden bodies weigh nothing, and the candle's pieces only their share of the real candle's weight."""
    for c in CANDLES.values():
        for part in c.hidden:
            mk.push(part, force=(0.0, 0.0, HIDDEN_MASS * GRAVITY))
        for piece in (c.name, f"{c.name}_top"):
            extra = mk.MECH.bodies[piece].mass * (1.0 - CANDLE_MASS / c.mass)
            if extra > 0.0:
                mk.push(piece, force=(0.0, 0.0, extra * GRAVITY))


def tip_and_axis(name: str) -> tuple[np.ndarray, np.ndarray]:
    p, r = mk.pose(name)
    return np.asarray(p, dtype=float), np.asarray(r, dtype=float)[:, 2]


def host_top(name: str) -> tuple[np.ndarray, np.ndarray]:
    """(the middle of host ``name``'s top in the world, its rotation)."""
    p, r = mk.pose(name)
    r = np.asarray(r, dtype=float)
    return np.asarray(p, dtype=float) + r @ np.array([0.0, 0.0, HOSTS[name].top]), r


def depth_of(c: Candle) -> float:
    """How far the candle's tip is in, along its hole (m): from the slide's travel since the weld was made."""
    return c.entry_below + (c.entry_travel - mk.joint(f"{c.name}_depth"))


def lean_from_hole(c: Candle) -> float:
    """Degrees between the candle's axis and its hole's (the direction it went in along)."""
    _, a = tip_and_axis(c.name)
    _, rh = mk.pose(c.parts[0])
    return math.degrees(math.acos(float(np.clip(a @ np.asarray(rh)[:, 2], -1.0, 1.0))))


def _direction(t1: float, t2: float) -> np.ndarray:
    """The candle's axis in its hole's frame for the two hinges' travels: turned about the hole's x, then the ring's y."""
    c1, s1, c2, s2 = math.cos(t1), math.sin(t1), math.cos(t2), math.sin(t2)
    return np.array([s2, -s1 * c2, c1 * c2])


def lean_from_rest(c: Candle) -> float:
    """Degrees between the candle's axis and the direction its host's springs rest along."""
    now = _direction(mk.joint(f"{c.name}_tilt1"), mk.joint(f"{c.name}_tilt2"))
    rest = _direction(*c.lean_rest)
    return math.degrees(math.acos(float(np.clip(now @ rest, -1.0, 1.0))))


def _rest_along_candle(c: Candle) -> None:
    c.lean_rest = (mk.joint(f"{c.name}_tilt1"), mk.joint(f"{c.name}_tilt2"))
    mk.set_spring(f"{c.name}_tilt1", rest=c.lean_rest[0])
    mk.set_spring(f"{c.name}_tilt2", rest=c.lean_rest[1])


def lean_from_vertical(name: str) -> float:
    _, a = tip_and_axis(name)
    return math.degrees(math.acos(float(np.clip(a[2], -1.0, 1.0))))


def _free(c: Candle, why: str) -> None:
    mk.cut(f"{c.name}_in_{c.host}")
    c.events.append({"t": round(mk.now(), 2), "event": why, "host": c.host, "depth_mm": round(1000 * depth_of(c), 2)})
    c.host, c.rest, c.cap, c.lean_rest = None, c.tip_travel, 3.0, (0.0, 0.0)
    mk.set_spring(f"{c.name}_depth", rest=c.rest, most=c.cap)  # the chain back to the tip
    for t in ("tilt1", "tilt2"):
        mk.set_spring(f"{c.name}_{t}", rest=0.0)
    _set_hold(c, FREE_K, FREE_DAMP_S)


def step(host_ok=lambda name: True) -> None:
    """Once per control step (in the simulator): entries, the resistance along the candle, the hold against leaning,
    tearing out and pulling out. ``host_ok(name)``: whether a candle can go into that host now (not squashed)."""
    for c in CANDLES.values():
        if not c.snapped and bending(c) >= SNAP_MOMENT:
            mk.cut(f"{c.name}_whole")
            c.snapped = True
            c.events.append({"t": round(mk.now(), 2), "event": "snapped", "host": c.host, "moment_Nm": round(bending(c), 4)})
        tip, axis = tip_and_axis(c.name)
        if c.host is None:
            for name in c.hosts:
                if not host_ok(name):
                    continue
                top, rot = host_top(name)
                local = rot.T @ (tip - top)
                below = -float(local[2])
                lean = math.degrees(math.acos(float(np.clip(rot[:, 2] @ axis, -1.0, 1.0))))
                if 0.0 <= below <= ENTRY_BELOW and math.hypot(local[0], local[1]) <= HOSTS[name].rim_inside and lean <= ENTRY_LEAN_DEG:
                    mk.make(f"{c.name}_in_{name}")
                    c.host, c.entry_below, c.entry_travel = name, below, mk.joint(f"{c.name}_depth")
                    c.old_hole = any(math.hypot(*(np.asarray(p)[:2] - local[:2])) <= OLD_HOLE for p in HOLES.get(name, []))
                    c.rest, c.deepest, c.k_now = c.entry_travel, below, -1.0
                    c.cap = resistance(below, HOSTS[name], c.old_hole)
                    mk.set_spring(f"{c.name}_depth", rest=c.rest, most=c.cap)
                    _rest_along_candle(c)
                    c.events.append({"t": round(mk.now(), 2), "event": "in", "host": name, "below_mm": round(1000 * below, 2),
                                     "off_middle_mm": round(1000 * math.hypot(local[0], local[1]), 2), "lean_deg": round(lean, 2), "old_hole": c.old_hole})
                    break
            if c.host is None:
                continue
        h = HOSTS[c.host]
        depth = depth_of(c)
        if depth > c.deepest + DEEPER:  # it went deeper: the cake closes round it as it stands now
            _rest_along_candle(c)
        c.deepest = max(c.deepest, depth)
        if lean_from_rest(c) > h.tear_deg:
            _free(c, "torn out")
            continue
        if c.deepest >= OUT_AFTER and depth <= OUT_DEPTH:
            top, rot = host_top(c.host)
            HOLES.setdefault(c.host, []).append(rot.T @ (tip - top))
            _free(c, "pulled out")
            continue
        k = hold_stiffness(depth, h)
        if c.k_now < 0 or abs(k - c.k_now) > 0.02 * max(k, 1e-9):
            _set_hold(c, k, HOLD_DAMP_S)
        cap = resistance(depth, h, c.old_hole)
        rest = follow(c.rest, mk.joint(f"{c.name}_depth"), cap, c.slide_k) if c.follows else c.rest
        if rest != c.rest or cap != c.cap:
            mk.set_spring(f"{c.name}_depth", rest=rest, most=cap)
            c.rest, c.cap = rest, cap


def reading(c: Candle) -> dict:
    out = {"in": c.host, "lean_vertical_deg": round(lean_from_vertical(c.name), 2), "bending_Nm": round(bending(c), 5), "snapped": c.snapped}
    if c.host is not None:
        out.update(depth_mm=round(1000 * depth_of(c), 2), lean_hole_deg=round(lean_from_hole(c), 2), lean_rest_deg=round(lean_from_rest(c), 2),
                   k=round(c.k_now, 6), cap=c.cap)
    return out
