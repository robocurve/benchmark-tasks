"""pourkit: liquids that pour between containers, for a yam_arena task file (works with yamkit).

The simulator has no fluids, so the liquid here is a scored flow model, not particles. A `Vessel` gives a prop
(a yamkit `Prop`, or any task object) a liquid-holding interior: stacked cylinder sections in the prop's own frame, an
opening at the top, and its starting contents in mL by liquid name. After every control step the kit reads each
vessel's pose and moves liquid:

- How much a vessel can hold at its current tilt is the volume of its interior below the lowest point of its rim
  (worked out once per vessel on a grid, for tilts 0 to 180 degrees). Liquid above that pours out over that lowest
  rim point (the lip), at most `max_flow` mL/s, as a well-mixed share of what the vessel holds. A vessel tipped over
  empties itself the same way; one filled past its brim overflows.
- The stream falls straight down from the lip. It lands in the vessel whose opening (a disk at its rim, tilted no
  more than `receive_tilt` degrees) the plumb line from the lip crosses first below the lip; else it is spilled. With
  `splash_drop` set, a stream falling farther than that (lip to the receiving rim) loses `splash` of itself to spill,
  since a liquid poured from high splashes.
- Every transfer is logged: an ordered list of `Pour`s (source, destination or "spill", first and last step, the
  amount of each liquid, and what the destination held just before the pour began).

Scorer API (call after the kit has updated for the step, which it does inside the scene's object_states, before
success() and score()): `volume(objects, "beaker")`, `volume(objects, "beaker", "acid")`, `contents(objects, name)`,
`spilled(objects)` / `spilled(objects, "acid")`, `pours(objects)`, `poured(objects, src, dst, liquid)`,
`flowing(objects, name)` and `tilt(objects, name)` in degrees. Everything is deterministic: plain arithmetic on the
object poses the scorer also gets.

Visuals: each vessel's liquid is drawn as a colored column inside it (one cylinder per section, along the vessel's
axis, so it tilts with the vessel rather than staying level), and each active stream as a thin vertical cylinder from
the lip to where it lands, in the liquid's color. They are Isaac Lab visualization markers (USD point instancers),
so the cameras see them; they have no physics. The vessels themselves are ordinary rigid props: give a container a
solid collider (the liquid is not physical), so nothing can be pushed into it.

Episodes: the contents and history go back to the start whenever yamkit's `episode()` changes (the simulator reset
the scene). `reset()` does that by hand. The starting contents are also drawn right after every scene reset (the
embodiment's env.reset), before the first rendered frame, so a video does not open on empty containers.

Demo helper: `pour(...)` returns `yamkit.Tilted` waypoints that tip a held container about its lip, so the lip stays
over a chosen point while the grip pitches (or rolls); `lip(...)` gives the lip's world position at the grip.

Limits: containers are stacked cylinders (no spouts or square tanks); a stream has no sideways speed, so a fast
swing does not throw liquid; the liquid's weight stays with the container; liquid never wets, sticks or splashes onto
anything but a spill total; anything between a lip and an opening (a hand, a lid) does not block the stream; the
drawn liquid tilts with its glass instead of staying level. A lid or cap would need the task to block pouring itself.

Tests: `update(objects)` is the step function the hook calls; offline tests call it with hand-made ObjectStates.

Usage in task.py (next to yamkit.py and pourkit.py):

    import yamkit as kit
    import pourkit as pk
    pk.setup([pk.Vessel("beaker", sections=((0.035, 0.09),), floor=-0.045, fill={"water": 200.0}),
              pk.Vessel("bottle", sections=((0.02, 0.065), (0.011, 0.02)), floor=-0.04, fill={"acid": 40.0})],
             liquids={"water": (0.55, 0.75, 0.95), "acid": (0.95, 0.92, 0.70)})
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

import yamkit as kit

ML = 1e6  # mL per cubic metre
SPILL = "spill"


# -- configuration ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Vessel:
    """The liquid-holding interior of the task object `name`. `sections` are (inner radius, height) pairs in metres,
    stacked along the object's +z from `floor` (the inside bottom, in the object's frame: relative to the object's
    `pos`). The top section's top is the rim. `mouth` is the opening's radius (default: the top section's).
    `fill` is the starting contents, mL per liquid. `receive_tilt` (degrees) is how far it may lean and still catch a
    stream. `max_flow` caps how fast it pours, mL/s (default 10 mL/s per cm² of opening). `seal` (from
    decant-off-the-sediment) is the mL of unsettled liquid that must stay between a settled layer (see `setup`'s
    `settled`) and the lip: while more than that is left, the vessel pours only unsettled liquid; past it, the rest
    pours well mixed with the settled layer."""

    name: str
    sections: tuple
    floor: float
    fill: dict = field(default_factory=dict)
    mouth: float | None = None
    receive_tilt: float = 60.0
    max_flow: float | None = None
    seal: float = 0.0
    # The settled layer's creep (from decant-off-the-sediment). While the vessel leans past `slide_tilt`
    # degrees with less than `slide_below` mL of unsettled liquid left above the settled layer, the layer slides from
    # the floor toward the shoulder: its front moves 1/`slide_s` of the way per second, sped up by the stream dragging
    # it (times 1 + `slide_gain` x the outflow as a fraction of `flow_cap`). Below `slide_tilt` it stops where it is.
    # Once the front reaches the shoulder (front >= 1) the seal is gone: what pours is well mixed, and past
    # `slurry_tilt` degrees the settled layer runs out of the neck on its own at `slurry_flow` mL/s.
    slide_below: float = 0.0
    slide_tilt: float = 75.0
    slide_s: float = 5.0
    slide_gain: float = 2.0
    slurry_tilt: float = 85.0
    slurry_flow: float = 2.0
    # Glugging (from decant-off-the-sediment): tipped past `glug_tilt` degrees (neck pointing down; 0 = off),
    # air gulps back up the neck and the churn lifts the settled layer: its front moves up to 1/`glug_s` of the way per second
    # (scaled by how far past `glug_tilt`, full at 10 degrees past), whatever is left above it.
    glug_tilt: float = 0.0
    glug_s: float = 0.5
    # Leaking (from springform-clasp-before-batter): `leak(objects)` gives mL/s the vessel loses at this
    # step through a gap in its wall or floor (0 when sealed), well mixed, to the spill total (logged as a pour to
    # "spill"). `leak_at` is where it runs out, in the object's frame: the run is drawn from there to the table, with
    # a puddle on the table that grows with what has leaked.
    leak: object = None
    leak_at: tuple = (0.0, 0.0, 0.0)

    @property
    def height(self) -> float:
        return float(sum(h for _, h in self.sections))

    @property
    def mouth_r(self) -> float:
        return float(self.mouth if self.mouth is not None else self.sections[-1][0])

    @property
    def capacity(self) -> float:
        """mL to the brim."""
        return float(sum(math.pi * r * r * h for r, h in self.sections) * ML)

    @property
    def flow_cap(self) -> float:
        """mL/s."""
        if self.max_flow is not None:
            return float(self.max_flow)
        return 10.0 * math.pi * (self.mouth_r * 100) ** 2


@dataclass(frozen=True)
class Pour:
    """One continuous transfer: liquid from `source` into `dest` (a vessel or "spill") from control step `start` to
    `end` (inclusive; steps count from 1 in each trial). `amounts` is mL per liquid; `before` is what `dest` held,
    mL per liquid, just before the pour began (empty for a spill)."""

    source: str
    dest: str
    start: int
    end: int
    amounts: dict
    before: dict

    @property
    def total(self) -> float:
        return float(sum(self.amounts.values()))


_CFG = {"vessels": {}, "liquids": {}, "splash_drop": None, "splash": 0.0, "dt": 0.05, "settled": (), "docks": {}, "limits": {}}
_TABLES: dict[str, tuple[np.ndarray, np.ndarray]] = {}  # vessel -> (tilt degrees, mL held at that tilt)


class _State:
    def __init__(self):
        self.episode = kit.episode()
        self.step = 0
        self.contents = {n: {k: float(v) for k, v in vs.fill.items()} for n, vs in _CFG["vessels"].items()}
        self.spilled: dict[str, float] = {}
        self.pours: list[Pour] = []
        self.open: dict[str, int] = {}  # source -> index in pours of its pour still running at the last step
        self.flowing: dict[str, bool] = {}
        self.tilt: dict[str, float] = {}
        self.front: dict[str, float] = {}  # vessel -> its settled layer's front, 0 on the floor to 1 at the shoulder
        self.streams: dict[str, tuple] = {}  # vessel -> (lip xyz, landing z, liquid) for the visuals
        self.leaking: dict[str, tuple] = {}  # vessel -> (leak point xyz, liquid) while it leaks, for the visuals
        self.leaked: dict[str, float] = {}  # vessel -> mL leaked so far this trial


_S = _State()


def setup(vessels, liquids: dict | None = None, splash_drop: float | None = None, splash: float = 0.0,
          control_hz: float = 20.0, settled: tuple = (), docks: dict | None = None,
          limits: dict | None = None) -> None:
    """Declare the task's vessels (list of `Vessel`) and liquid colors ({name: sRGB}). `splash_drop` (m) and
    `splash` (fraction) turn on splash losses for streams that fall farther than `splash_drop`. `control_hz` is the
    run's control rate (the commands here use 20). `settled` names liquids that lie at a vessel's bottom (a sediment):
    they pour last (see `Vessel.seal`) and are drawn below the rest when upright, behind it when tipped neck-down.
    `docks` (from water-cooler-bottle-flip) maps a vessel to a function of the step's objects that returns
    (receiver, seal mL) while the vessel's mouth is seated in that receiver, else None: a seated vessel pours only into
    its receiver, and only until the receiver holds `seal` mL (an inverted bottle in a water cooler's crown feeds the
    reservoir until the water covers its neck, then air can no longer get in and it stops). `limits` (same
    task) caps what a receiver may hold, mL: liquid poured into it past that is counted as spill."""
    global _S
    _CFG["vessels"] = {v.name: v for v in vessels}
    _CFG["liquids"] = dict(liquids or {})
    for v in vessels:
        for k in v.fill:
            _CFG["liquids"].setdefault(k, (0.6, 0.8, 1.0))
    _CFG["splash_drop"], _CFG["splash"], _CFG["dt"] = splash_drop, float(splash), 1.0 / control_hz
    _CFG["settled"] = tuple(settled)
    _CFG["docks"] = dict(docks or {})
    _CFG["limits"] = dict(limits or {})
    _TABLES.clear()
    _S = _State()


def reset() -> None:
    """Back to the starting contents, no history."""
    global _S
    _S = _State()


# -- geometry ----------------------------------------------------------------------------------


def _rotm(q) -> np.ndarray:
    x, y, z, w = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _table(v: Vessel) -> tuple[np.ndarray, np.ndarray]:
    """mL the vessel holds when tilted 0..180 degrees: its interior below the lowest point of its rim (a grid of
    equal-volume cells in each section, so it is exact for an upright vessel and close otherwise)."""
    if v.name in _TABLES:
        return _TABLES[v.name]
    pts, wts = [], []
    z0 = 0.0
    for r, h in v.sections:
        nr, nphi, nz = 14, 36, max(6, int(round(h / 0.0025)))
        rr = r * np.sqrt((np.arange(nr) + 0.5) / nr)  # equal-area rings
        phi = 2 * np.pi * (np.arange(nphi) + 0.5) / nphi
        zz = z0 + h * (np.arange(nz) + 0.5) / nz
        R, P, Z = np.meshgrid(rr, phi, zz, indexing="ij")
        pts.append(np.stack([R * np.cos(P), R * np.sin(P), Z], -1).reshape(-1, 3))
        wts.append(np.full(R.size, math.pi * r * r * h * ML / R.size))
        z0 += h
    p, w = np.concatenate(pts), np.concatenate(wts)
    angles = np.arange(0.0, 180.5, 1.0)
    held = []
    for a in np.radians(angles):
        hz = -p[:, 0] * math.sin(a) + p[:, 2] * math.cos(a)  # height after tipping toward +x
        lip = v.height * math.cos(a) - v.mouth_r * math.sin(a)
        held.append(float(w[hz < lip].sum()))
    held = np.minimum.accumulate(np.array(held))  # never more at a steeper tilt
    held[0] = v.capacity
    _TABLES[v.name] = (angles, held)
    return _TABLES[v.name]


def held_at(v: Vessel | str, tilt_deg: float) -> float:
    """mL the vessel can hold at this tilt (degrees from upright)."""
    v = _CFG["vessels"][v] if isinstance(v, str) else v
    a, h = _table(v)
    return float(np.interp(tilt_deg, a, h))


def _pose(objects, v: Vessel):
    s = objects[v.name]
    rot = _rotm(s.quat_xyzw)
    axis = rot[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, axis[2]))))
    rim = np.asarray(s.pos, float) + rot @ np.array([0.0, 0.0, v.floor + v.height])
    down = np.array([0.0, 0.0, -1.0]) - axis * (-axis[2])  # straight down, projected onto the rim's plane
    n = np.linalg.norm(down)
    lip = rim + (down / n * v.mouth_r if n > 1e-6 else np.array([v.mouth_r, 0.0, 0.0]))
    return rot, axis, tilt, rim, lip


def _landing(objects, source: str, lip: np.ndarray):
    """(vessel, its rim height) the plumb line from `lip` falls into first, or (None, None)."""
    best, best_z = None, None
    for name, v in _CFG["vessels"].items():
        if name == source or name not in objects:
            continue
        _, axis, tilt, rim, _ = _pose(objects, v)
        if tilt > v.receive_tilt or axis[2] <= 1e-6:
            continue
        t = float(np.dot(lip - rim, axis) / axis[2])  # drop from the lip to the rim's plane
        if t <= 0:
            continue
        hit = lip - np.array([0.0, 0.0, t])
        if np.linalg.norm(hit - rim) < v.mouth_r and (best is None or hit[2] > best_z):
            best, best_z = name, float(hit[2])
    return best, best_z


# -- the step ----------------------------------------------------------------------------------


def update(objects) -> None:
    """Advance the liquids by one control step from the vessels' poses in `objects` (name -> ObjectState). The scene
    hook calls this once per step; tests call it directly."""
    global _S
    if _S.episode != kit.episode():
        _S = _State()
    _S.step += 1
    dt = _CFG["dt"]
    moves = []  # (source, dest, mL per liquid)
    _S.streams = {}
    for name, v in _CFG["vessels"].items():
        if name not in objects:
            continue
        _, _, tilt, _, lip = _pose(objects, v)
        _S.tilt[name] = tilt
        have = _S.contents[name]
        total = sum(have.values())
        excess = total - held_at(v, tilt)
        out = min(excess, v.flow_cap * dt) if excess > 1e-9 else 0.0
        dock = _CFG["docks"][name](objects) if name in _CFG["docks"] else None
        if dock is not None:  # seated in a receiver: pours into it only while it has room below the seal level
            room = max(0.0, dock[1] - sum(_S.contents[dock[0]].values()))
            out = min(out, room)
        front = _S.front.get(name, 0.0)
        sed = sum(q for k, q in have.items() if k in _CFG["settled"])
        clear = total - sed
        if v.slide_below > 0 and sed > 1e-9 and tilt > v.slide_tilt and clear < v.slide_below:
            front += dt / v.slide_s * (1.0 + v.slide_gain * out / (v.flow_cap * dt))
            _S.front[name] = front
        if v.glug_tilt > 0 and sed > 1e-9 and tilt > v.glug_tilt:  # glugging churns the settled layer up
            front += dt / v.glug_s * min((tilt - v.glug_tilt) / 10.0, 1.0)  # gentle just past it, full 10 degrees on
            _S.front[name] = front
        share = _share(v, have, out, front) if out > 1e-9 else {}
        if front >= 1.0 and tilt > v.slurry_tilt and sed > 1e-9:  # the sediment runs out of the neck on its own
            for k, q in have.items():
                if k in _CFG["settled"]:
                    extra = min(q - share.get(k, 0.0), v.slurry_flow * dt * q / sed)
                    if extra > 1e-9:
                        share[k] = share.get(k, 0.0) + extra
        out = sum(share.values())
        _S.flowing[name] = out > 1e-9
        if out <= 1e-9:
            continue
        dest, dest_z = (dock[0], lip[2]) if dock is not None else _landing(objects, name, lip)
        if dest is not None and _CFG["splash_drop"] is not None and lip[2] - dest_z > _CFG["splash_drop"]:
            lost = {k: q * _CFG["splash"] for k, q in share.items()}
            moves.append((name, SPILL, lost))
            share = {k: q - lost[k] for k, q in share.items()}
        moves.append((name, dest or SPILL, share))
        main = max(share, key=share.get)
        _S.streams[name] = (lip, dest_z if dest is not None else _floor_z(lip), main)
    _apply(moves)
    leaks = []  # leaking vessels lose liquid after the step's pours (a stream that lands in a leaking pan leaks out)
    _S.leaking = {}
    for name, v in _CFG["vessels"].items():
        if v.leak is None or name not in objects:
            continue
        have = _S.contents[name]
        total = sum(have.values())
        rate = float(v.leak(objects))
        if rate <= 0 or total <= 1e-9:
            continue
        out = min(total, rate * dt)
        leaks.append((name, SPILL, {k: q * out / total for k, q in have.items() if q > 0}))
        _S.leaked[name] = _S.leaked.get(name, 0.0) + out
        s = objects[name]
        at = np.asarray(s.pos, float) + _rotm(s.quat_xyzw) @ np.asarray(v.leak_at, float)
        _S.leaking[name] = (at, max(have, key=have.get))
    _apply(leaks)


def _apply(moves) -> None:
    """Move liquid for (source, dest, mL per liquid) transfers and log them as pours."""
    for src, dst, amt in moves:  # take everything out first, then pour in: a step's order does not matter
        for k, q in amt.items():
            _S.contents[src][k] = max(0.0, _S.contents[src][k] - q)
    for src, dst, amt in moves:
        before = dict(_S.contents[dst]) if dst != SPILL else {}
        if dst == SPILL:
            for k, q in amt.items():
                _S.spilled[k] = _S.spilled.get(k, 0.0) + q
        else:
            limit = _CFG["limits"].get(dst)
            if limit is not None:  # past its limit a receiver overflows: the rest is spill
                room = max(0.0, limit - sum(_S.contents[dst].values()))
                tot = sum(amt.values())
                if tot > room + 1e-12:
                    for k, q in amt.items():
                        _S.spilled[k] = _S.spilled.get(k, 0.0) + q * (tot - room) / tot
                    amt = {k: q * room / tot for k, q in amt.items()}
            for k, q in amt.items():
                _S.contents[dst][k] = _S.contents[dst].get(k, 0.0) + q
        key = f"{src}>{dst}"
        i = _S.open.get(key)
        if i is not None and _S.pours[i].end == _S.step - 1:
            p = _S.pours[i]
            merged = {k: p.amounts.get(k, 0.0) + amt.get(k, 0.0) for k in set(p.amounts) | set(amt)}
            _S.pours[i] = Pour(src, dst, p.start, _S.step, merged, p.before)
        else:
            _S.pours.append(Pour(src, dst, _S.step, _S.step, dict(amt), before))
            _S.open[key] = len(_S.pours) - 1


def _share(v: Vessel, have: dict, out: float, front: float = 0.0) -> dict:
    """mL per liquid in `out` mL poured from contents `have`: well mixed, except that settled liquids stay behind
    while more than `v.seal` mL of the rest is left and their front has not reached the shoulder; the part of `out`
    past that is well mixed with them."""
    total = sum(have.values())
    sed = {k: q for k, q in have.items() if k in _CFG["settled"] and q > 0}
    if not sed or front >= 1.0:
        return {k: q * out / total for k, q in have.items() if q > 0}
    clear = {k: q for k, q in have.items() if k not in sed and q > 0}
    c = sum(clear.values())
    first = min(out, max(0.0, c - v.seal))  # clear liquid only
    rest = out - first
    left = {k: q - (q / c * first if c > 0 else 0.0) for k, q in clear.items()}
    left.update(sed)
    lt = sum(left.values())
    share = {k: (q / c * first if c > 0 else 0.0) for k, q in clear.items()}
    for k, q in left.items():
        share[k] = share.get(k, 0.0) + (q * rest / lt if lt > 0 else 0.0)
    return {k: q for k, q in share.items() if q > 0}


def _floor_z(lip) -> float:
    return float(_CFG.get("floor_z", lip[2] - 0.1))


def set_floor(z: float) -> None:
    """Where a spilled stream is drawn to (the table top's height); visual only."""
    _CFG["floor_z"] = float(z)


# -- scorer API ------------------------------------------------------------------------------


def volume(objects, name: str, liquid: str | None = None) -> float:
    """mL in vessel `name` (of one `liquid`, or all) as of this step."""
    have = _S.contents.get(name, {})
    return float(have.get(liquid, 0.0) if liquid else sum(have.values()))


def contents(objects, name: str) -> dict:
    """mL per liquid in vessel `name`."""
    return dict(_S.contents.get(name, {}))


def spilled(objects=None, liquid: str | None = None) -> float:
    """mL spilled so far this trial (of one `liquid`, or all)."""
    return float(_S.spilled.get(liquid, 0.0) if liquid else sum(_S.spilled.values()))


def pours(objects=None) -> tuple:
    """Every `Pour` so far this trial, in the order they began."""
    return tuple(_S.pours)


def poured(objects, source: str, dest: str, liquid: str | None = None) -> float:
    """mL moved from `source` into `dest` ("spill" for spills) so far this trial."""
    return float(sum(p.amounts.get(liquid, 0.0) if liquid else p.total for p in _S.pours
                     if p.source == source and p.dest == dest))


def flowing(objects, name: str) -> bool:
    """Whether vessel `name` poured during this step."""
    return bool(_S.flowing.get(name, False))


def front(objects, name: str) -> float:
    """How far the vessel's settled layer has slid: 0 on the floor, 1 at the shoulder (it pours from there)."""
    return float(_S.front.get(name, 0.0))


def tilt(objects, name: str) -> float:
    """Vessel `name`'s tilt from upright, degrees, as of this step."""
    return float(_S.tilt.get(name, 0.0))


def leaked(objects, name: str) -> float:
    """mL vessel `name` has lost through its leak so far this trial (part of the spill total)."""
    return float(_S.leaked.get(name, 0.0))


def leaking(objects, name: str) -> bool:
    """Whether vessel `name` leaked during this step."""
    return name in _S.leaking


def step() -> int:
    """Control steps the kit has seen in this trial."""
    return _S.step


# -- visuals -----------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
STREAM_R = 0.0035  # m: a stream's drawn radius
NECK_DOWN = 70.0  # degrees: tipped further than this, a vessel's liquid is drawn from the rim end (it has run to the neck)
PUDDLE_T, PUDDLE_MAX = 0.003, 0.07  # m: a leak's puddle is drawn this thick, up to this radius
LEAD_FROM = 0.75  # a settled layer's front past this (of the way to the shoulder) is also drawn entering the shoulder


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        protos = {}
        for name, rgb in _CFG["liquids"].items():
            lin = tuple(kit._linear(c) for c in rgb)
            protos[f"liquid_{name}"] = sim_utils.CylinderCfg(
                radius=1.0, height=1.0, axis="Z",
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.15, opacity=1.0))  # opaque: a see-through liquid behind see-through glass does not render
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/pourkit",
                                                                       markers=protos))
        _VIS["index"] = {name: i for i, name in enumerate(_CFG["liquids"])}
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"pourkit: no liquid visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _layers(name: str, have: dict, inverted: bool) -> list:
    """(liquid, mL) layers in the order they fill the vessel from its floor (or from its rim when `inverted`):
    settled liquids lie on the floor, so they come first upright and last tipped neck-down. The two are one colour
    each: the main settled liquid and the main other liquid."""
    sed = sum(q for k, q in have.items() if k in _CFG["settled"])
    rest = sum(q for k, q in have.items() if k not in _CFG["settled"])
    out = []
    if sed > 0:
        out.append((max((k for k in have if k in _CFG["settled"]), key=have.get), sed))
    if rest > 0:
        out.append((max((k for k in have if k not in _CFG["settled"]), key=have.get), rest))
    if inverted:
        out.reverse()
    return out


def _draw(objects, origin) -> None:
    """Each vessel's liquid as up to two layers (settled, the rest) per section, filled from the floor when the
    vessel is upright-ish and from the rim when it is tipped past NECK_DOWN degrees (the liquid runs to the neck); then its
    stream. Unused slots are parked out of sight so the marker count stays fixed."""
    m = _markers()
    if m is None:
        return
    idx = _VIS["index"]
    first = next(iter(idx))
    tr, qs, sc, ids = [], [], [], []

    def hide(liq):
        tr.append((0.0, 0.0, -10.0)), qs.append((0, 0, 0, 1)), sc.append((1e-5,) * 3), ids.append(idx[liq])

    for name, v in _CFG["vessels"].items():
        have = _S.contents[name]
        s = objects.get(name)
        inverted = s is not None and _S.tilt.get(name, 0.0) > NECK_DOWN
        layers = _layers(name, have, inverted) if s is not None else []
        secs = list(v.sections)
        bases = np.cumsum([0.0] + [h for _, h in secs])[:-1]  # each section's start along the axis from the floor
        order = list(range(len(secs)))[::-1] if inverted else list(range(len(secs)))
        # walk the sections in fill order, laying each layer's volume
        slots = []  # (liquid, section index, start along the section in fill direction, length)
        li, left = 0, (layers[0][1] / ML if layers else 0.0)
        for i in order:
            r, h = secs[i]
            used = 0.0
            while li < len(layers) and used < h - 1e-6:
                part = min(h - used, left / (math.pi * r * r))
                if part > 1e-4:
                    slots.append((layers[li][0], i, used, part))
                used += part
                left -= part * math.pi * r * r
                if left <= 1e-12:
                    li += 1
                    left = layers[li][1] / ML if li < len(layers) else 0.0
                else:
                    break
        n_slots = 2 * len(secs)
        rot = _rotm(s.quat_xyzw) if s is not None else None
        for liq, i, start, part in slots[:n_slots]:
            r, h = secs[i]
            along = bases[i] + (h - start - part / 2 if inverted else start + part / 2)
            rr = r * 0.97
            if liq in _CFG["settled"] and inverted and i == 0:
                # the settled layer slides from the floor toward the shoulder (Vessel.slide_*): drawn as a band at
                # its front, a little wider than the wine so it shows through it
                along = part / 2 + min(1.0, _S.front.get(name, 0.0)) * max(0.0, h - part)
                rr = r * 0.995
            c = np.asarray(s.pos, float) + rot @ np.array([0.0, 0.0, v.floor + along])
            tr.append(tuple(c + origin)), qs.append(tuple(float(q) for q in s.quat_xyzw))
            sc.append((rr, rr, part)), ids.append(idx[liq])
        lead = 0
        front = _S.front.get(name, 0.0)
        sed_liq = next((liq for liq, *_ in slots if liq in _CFG["settled"]), None)
        if inverted and sed_liq is not None and front > LEAD_FROM and len(secs) > 1 and len(slots) < n_slots:
            # the settled layer's leading edge reaching the shoulder (from decant-off-the-sediment): for
            # the last part of its slide a dark sleeve grows from the body into the shoulder around the wine there,
            # where a wrist camera holding the bottle by its body sees it (the band itself is often behind the fingers)
            r1, h1 = secs[1]
            part = h1 * min(1.0, (front - LEAD_FROM) / (1.0 - LEAD_FROM))
            c = np.asarray(s.pos, float) + rot @ np.array([0.0, 0.0, v.floor + bases[1] + part / 2])
            tr.append(tuple(c + origin)), qs.append(tuple(float(q) for q in s.quat_xyzw))
            sc.append((r1 * 0.995, r1 * 0.995, part)), ids.append(idx[sed_liq])
            lead = 1
        for _ in range(n_slots - min(len(slots), n_slots) - lead):
            hide(first)
        stream = _S.streams.get(name) if s is not None else None
        if stream is None:
            hide(first)
        else:
            lip, z_end, liq = stream
            length = max(1e-3, float(lip[2] - z_end))
            tr.append(tuple(np.array([lip[0], lip[1], lip[2] - length / 2]) + origin))
            qs.append((0, 0, 0, 1)), sc.append((STREAM_R, STREAM_R, length)), ids.append(idx[liq])
        if v.leak is not None:  # a leak's run down to the table and its puddle (two slots, always present)
            run = _S.leaking.get(name) if s is not None else None
            if run is None:
                hide(first)
            else:
                at, liq = run
                z0 = _floor_z(at)
                length = max(1e-3, float(at[2] - z0))
                tr.append(tuple(np.array([at[0], at[1], z0 + length / 2]) + origin))
                qs.append((0, 0, 0, 1)), sc.append((STREAM_R * 0.8, STREAM_R * 0.8, length)), ids.append(idx[liq])
            vol = _S.leaked.get(name, 0.0)
            if s is None or vol <= 0.05:
                hide(first)
            else:
                at = np.asarray(s.pos, float) + _rotm(s.quat_xyzw) @ np.asarray(v.leak_at, float)
                main = max(have, key=have.get) if have and sum(have.values()) > 0 else next(iter(v.fill), first)
                r = min(PUDDLE_MAX, math.sqrt(vol / ML / (math.pi * PUDDLE_T)))
                tr.append(tuple(np.array([at[0], at[1], _floor_z(at) + PUDDLE_T / 2]) + origin))
                qs.append((0, 0, 0, 1)), sc.append((r, r, PUDDLE_T)), ids.append(idx.get(main, idx[first]))
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                marker_indices=np.array(ids, int))


# -- hook into yam_arena.scene (after yamkit's) ------------------------------------------------


def _patch(scene) -> None:
    if getattr(scene, "_pourkit", False):
        return
    scene._pourkit = True
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
            if os.environ.get("POURKIT_DEBUG"):
                print("pourkit", _S.step, {n: round(_S.tilt.get(n, 0), 1) for n in _CFG["vessels"]},
                      {n: {k: round(q, 1) for k, q in c.items()} for n, c in _S.contents.items()},
                      {k: round(q, 1) for k, q in _S.spilled.items()},
                      {n: round(f, 2) for n, f in _S.front.items()}, flush=True)
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"pourkit: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


# -- the reset draw (the embodiment's env.reset) ---------------------------------------------------
# object_states runs only after a step, so without this the first rendered frames of a trial show the markers as the
# last trial left them (or none): containers empty until the first step. Like levelhose/dispersekit/siphonkit, the
# starting contents are drawn right after every scene reset, before the first rendered frame, without touching the
# trial's state.


def start_objects(env, names) -> tuple[dict, np.ndarray]:
    """Poses of `names` straight from the sim (for drawing before the first step): a dict of ObjectState-like records
    (pos: a yamkit Prop's origin, else the bounding-box centre, as object_states gives them; quat, zero velocities,
    bounds), and the env origin. Names the sim does not know (grippers) are skipped."""
    from types import SimpleNamespace

    base = env.unwrapped
    world = base.arena_world
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)

    def arr(t) -> np.ndarray:
        return t.detach().cpu().numpy().astype(np.float64)

    out = {}
    for n in names:
        try:
            pose = arr(world.get_pose_w(n)[0])
        except Exception:  # noqa: BLE001  (grippers and missing objects are skipped)
            continue
        pos = pose[:3] - origin
        low = high = pos
        try:
            box = world.get_aabb_w(n)
            low, high = arr(box.min_point[0]) - origin, arr(box.max_point[0]) - origin
            if n not in kit._PROPS:
                pos = arr(world.get_centroid_w(n)[0]) - origin
        except Exception:  # noqa: BLE001
            pass
        out[n] = SimpleNamespace(pos=pos, quat_xyzw=pose[3:], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=low,
                                 high=high)
    return out, origin


def _draw_start(env) -> None:
    """Every vessel at its starting contents and pose, no streams, before the first rendered frame (the trial's own
    state is left alone: the first step still starts it afresh)."""
    global _S
    if not _CFG["vessels"]:
        return
    objs, origin = start_objects(env, list(_CFG["vessels"]))
    keep, _S = _S, _State()
    try:
        for name, v in _CFG["vessels"].items():
            if name in objs:
                _S.tilt[name] = _pose(objs, v)[2]
        _draw(objs, origin)
    finally:
        _S = keep


def _patch_embodiment() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_pourkit", False):
        return
    cls._pourkit = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_pourkit_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            if not _VIS["failed"]:
                try:
                    _draw_start(env)
                except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                    print(f"pourkit: no start visuals ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset, env._pourkit_reset = reset_and_draw, True

    cls._ensure_sim = _ensure_sim


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


_patch_embodiment()
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that also adds the pour step after yamkit's patch
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- demo helpers --------------------------------------------------------------------------------


def lip(v: Vessel | str, obj_pos, toward=(1.0, 0.0, 0.0)) -> np.ndarray:
    """World position of the rim point on the `toward` side (the lip it will pour from) of vessel `v` standing
    upright with its origin at `obj_pos`. Use it for `pour(...)`'s `lip_at_grip`."""
    v = _CFG["vessels"][v] if isinstance(v, str) else v
    d = np.array([toward[0], toward[1], 0.0], float)
    d /= np.linalg.norm(d)
    return np.asarray(obj_pos, float) + np.array([0.0, 0.0, v.floor + v.height]) + d * v.mouth_r


def pour(tcp_at_grip, lip_at_grip, over, *, roll=0.0, pitch=0.0, yaw=0.0, start_roll=None, start_pitch=None,
         to_roll=None, to_pitch=None, above: float = 0.04, steps: int = 8, duration: float = 6.0, hold: float = 2.0,
         back: float = 3.0, finger=0.0, label: str = "pour"):
    """`yamkit.Tilted` waypoints that pour from a held container by tipping it about its lip.

    At the grip the tool point was at `tcp_at_grip`, the grip at (`roll`, `pitch`, `yaw`) and the container's lip at
    `lip_at_grip` (world; see `lip`). The plan brings the container, tipped to (`start_roll`, `start_pitch`), with its
    lip `above` m over `over` (1.5 s), lowers it until the lip is at `over` (1.0 s), tips it on to (`to_roll`,
    `to_pitch`) in `steps` even steps over `duration` s while keeping the lip at `over`, holds `hold` s, tips it back
    to the start tilt over `back` s and lifts it `above` m again (1.0 s). Choose the start tilt below the angle at
    which the container begins to pour (see `held_at`) and high enough that its body clears the receiving rim.
    Pitch turns about world y (positive tips the top toward +x), roll about world x (see yamkit.Tilted). Grips
    hold best when the turn is about the axis the hand points along (a side grip turned like a doorknob)."""
    start_roll = roll if start_roll is None else start_roll
    start_pitch = pitch if start_pitch is None else start_pitch
    to_roll = start_roll if to_roll is None else to_roll
    to_pitch = start_pitch if to_pitch is None else to_pitch
    r0 = kit._rot_xyz(roll, pitch)
    d = np.asarray(lip_at_grip, float) - np.asarray(tcp_at_grip, float)
    over = np.asarray(over, float)
    up = np.array([0.0, 0.0, above])

    def at(s, lift=0.0):
        r, p = start_roll + s * (to_roll - start_roll), start_pitch + s * (to_pitch - start_pitch)
        q = kit._rot_xyz(r, p) @ r0.T  # how far the held container has turned since the grip
        return dict(pos=tuple(over + lift * up - q @ d), finger=finger, yaw=yaw, roll=r, pitch=p)

    out = [kit.Tilted(1.5, label=f"{label}: over the target", **at(0.0, 1.0)),
           kit.Tilted(1.0, label=f"{label}: lip down to the target", **at(0.0))]
    out += [kit.Tilted(duration / steps, label=f"{label}: tip", **at(k / steps)) for k in range(1, steps + 1)]
    out.append(kit.Tilted(hold, label=f"{label}: hold", **at(1.0)))
    n = max(2, steps // 2)
    out += [kit.Tilted(back / n, label=f"{label}: tip back", **at(k / n)) for k in range(n - 1, -1, -1)]
    out.append(kit.Tilted(1.0, label=f"{label}: lift away", **at(0.0, 1.0)))
    return out
