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
the scene). `reset()` does that by hand.

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
    stream. `max_flow` caps how fast it pours, mL/s (default 10 mL/s per cm² of opening)."""

    name: str
    sections: tuple
    floor: float
    fill: dict = field(default_factory=dict)
    mouth: float | None = None
    receive_tilt: float = 60.0
    max_flow: float | None = None
    accepts: object = None  # optional fn(objects) -> bool: False while a lid covers the opening (a stream then
    # lands on the lid and runs off as spill instead of going in)

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


_CFG = {"vessels": {}, "liquids": {}, "splash_drop": None, "splash": 0.0, "dt": 0.05}
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
        self.streams: dict[str, tuple] = {}  # vessel -> (lip xyz, landing z, liquid) for the visuals


_S = _State()


def setup(vessels, liquids: dict | None = None, splash_drop: float | None = None, splash: float = 0.0,
          control_hz: float = 20.0) -> None:
    """Declare the task's vessels (list of `Vessel`) and liquid colors ({name: sRGB}). `splash_drop` (m) and
    `splash` (fraction) turn on splash losses for streams that fall farther than `splash_drop`. `control_hz` is the
    run's control rate (the commands here use 20)."""
    global _S
    _CFG["vessels"] = {v.name: v for v in vessels}
    _CFG["liquids"] = dict(liquids or {})
    for v in vessels:
        for k in v.fill:
            _CFG["liquids"].setdefault(k, (0.6, 0.8, 1.0))
    _CFG["splash_drop"], _CFG["splash"], _CFG["dt"] = splash_drop, float(splash), 1.0 / control_hz
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
    """(vessel, its rim height) the plumb line from `lip` falls into first, or (None, None). A vessel whose `accepts`
    says it is closed still catches the stream first, on its lid, and the stream is spilled: (None, its rim height)."""
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
        if np.linalg.norm(hit - rim) < v.mouth_r and (best_z is None or hit[2] > best_z):
            best, best_z = name, float(hit[2])
    if best is not None and _CFG["vessels"][best].accepts is not None and not _CFG["vessels"][best].accepts(objects):
        return None, best_z  # onto the closed lid: it runs off
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
        _S.flowing[name] = out > 1e-9
        if out <= 1e-9:
            continue
        share = {k: q * out / total for k, q in have.items() if q > 0}
        dest, dest_z = _landing(objects, name, lip)
        if dest is not None and _CFG["splash_drop"] is not None and lip[2] - dest_z > _CFG["splash_drop"]:
            lost = {k: q * _CFG["splash"] for k, q in share.items()}
            moves.append((name, SPILL, lost))
            share = {k: q - lost[k] for k, q in share.items()}
        moves.append((name, dest or SPILL, share))
        main = max(share, key=share.get)
        _S.streams[name] = (lip, dest_z if dest_z is not None else _floor_z(lip), main)  # lands in, on a lid, or on the floor
    for src, dst, amt in moves:  # take everything out first, then pour in: a step's order does not matter
        for k, q in amt.items():
            _S.contents[src][k] = max(0.0, _S.contents[src][k] - q)
    for src, dst, amt in moves:
        before = dict(_S.contents[dst]) if dst != SPILL else {}
        if dst == SPILL:
            for k, q in amt.items():
                _S.spilled[k] = _S.spilled.get(k, 0.0) + q
        else:
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


def tilt(objects, name: str) -> float:
    """Vessel `name`'s tilt from upright, degrees, as of this step."""
    return float(_S.tilt.get(name, 0.0))


def step() -> int:
    """Control steps the kit has seen in this trial."""
    return _S.step


# -- visuals -----------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
STREAM_R = 0.0035  # m: a stream's drawn radius


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
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.15, opacity=0.85))
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/pourkit",
                                                                       markers=protos))
        _VIS["index"] = {name: i for i, name in enumerate(_CFG["liquids"])}
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"pourkit: no liquid visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(objects, origin) -> None:
    m = _markers()
    if m is None:
        return
    idx = _VIS["index"]
    tr, qs, sc, ids = [], [], [], []
    for name, v in _CFG["vessels"].items():
        have = _S.contents[name]
        total = sum(have.values())
        main = max(have, key=have.get) if have else next(iter(idx))
        s = objects.get(name)
        level = 0.0  # liquid height along the axis from the floor, filling sections bottom up
        left = total / ML
        for r, h in v.sections:
            part = min(h, left / (math.pi * r * r)) if left > 0 else 0.0
            left -= part * math.pi * r * r
            if s is None or part < 1e-4:
                tr.append((0.0, 0.0, -10.0)), qs.append((0, 0, 0, 1)), sc.append((1e-5,) * 3), ids.append(idx[main])
            else:
                rot = _rotm(s.quat_xyzw)
                c = np.asarray(s.pos, float) + rot @ np.array([0.0, 0.0, v.floor + level + part / 2])
                tr.append(tuple(c + origin)), qs.append(tuple(float(q) for q in s.quat_xyzw))
                sc.append((r * 0.97, r * 0.97, part)), ids.append(idx[main])
            level += h
        stream = _S.streams.get(name) if s is not None else None
        if stream is None:
            tr.append((0.0, 0.0, -10.0)), qs.append((0, 0, 0, 1)), sc.append((1e-5,) * 3), ids.append(idx[main])
        else:
            lip, z_end, liq = stream
            length = max(1e-3, float(lip[2] - z_end))
            tr.append(tuple(np.array([lip[0], lip[1], lip[2] - length / 2]) + origin))
            qs.append((0, 0, 0, 1)), sc.append((STREAM_R, STREAM_R, length)), ids.append(idx[liq])
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
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"pourkit: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


# -- draw the starting contents right after each scene reset, before the first rendered frame -----
# (object_states only runs after a step, so without this the first frames show the vessels empty: pop-in)


def _draw_start(env) -> None:
    from types import SimpleNamespace
    global _S
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    objs = {}
    for n in _CFG["vessels"]:
        try:
            pose = base.arena_world.get_pose_w(n)[0].detach().cpu().numpy().astype(np.float64)
        except Exception:  # noqa: BLE001
            continue
        objs[n] = SimpleNamespace(pos=pose[:3] - origin, quat_xyzw=pose[3:])
    keep, _S = _S, _State()
    try:
        _draw(objs, origin)
    finally:
        _S = keep  # the trial state is untouched


def _patch_embodiment() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_pourkit_start", False):
        return
    cls._pourkit_start = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_pourkit_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            try:
                _draw_start(env)
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                print(f"pourkit: no start visuals ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset, env._pourkit_reset = reset_and_draw, True

    cls._ensure_sim = _ensure_sim


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
