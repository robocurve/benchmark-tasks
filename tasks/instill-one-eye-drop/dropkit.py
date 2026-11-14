"""dropkit: a soft squeeze bottle that releases liquid one drop at a time (eye drops, ear drops, dropper bottles).
Import after yamkit:  import dropkit as dk

No fluid simulation: a deterministic, scored model of what a dropper bottle does, driven by the bottle's own state.
- The bottle is a yamkit Mechanism: a rigid core plus wall pads on sprung slides in opposite pairs (the squeeze
  faces). `squeeze` is the largest pair's mean inward travel s (m); the squeeze force on the walls is F = wall_k * s.
- Liquid leaves only with the tip pointing at least `tip_down_deg` below horizontal (tip up, a squeeze moves air
  only) and only while F is above `open_force`, the force that overcomes the tip's surface tension and the air
  space. Then it flows into a drop hanging at the tip at `flow` * (F - open_force) (uL/s per N): a firm hold below
  `open_force` releases nothing; a moderate squeeze grows one drop in about a second; squeezing harder or longer
  makes more. When the hanging drop reaches `drop_ul` it detaches and falls straight down.
- Easing off below `open_force` sucks a hanging, not yet detached drop back into the bottle (the walls spring back
  and draw air in through the tip), as a real bottle does.
- Each released drop is handed to the task's `land(objects, tip_world)` function, which says where it lands
  (a zone name, the landing point and anything else to log). The task scores from `drops()`.
Readouts: drops(), hanging(), squeeze_force(objects, spec), tip(objects, spec), tip_down(objects, spec).
Offline tests call update(objects) once per control step with hand-made ObjectStates and kit._JOINTS entries.
Visuals (optional, never scored): each landed drop as a blue bead at its landing point; the hanging drop at the tip.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

import yamkit as kit

DT = 0.05  # s per control step (20 Hz)


@dataclass(frozen=True)
class Bottle:
    """A dropper bottle: the Mechanism `name` with squeeze-pad links `pads`: one or more opposite pairs, each pair two
    (link name, sign) entries (sign * joint position is the pad's inward travel), so a bottle can be squeezed across
    any of its pairs; `tip` is the tip's end in the bottle's own frame and `axis` the bottle's tip direction in that
    frame. Units: m, N, uL, s."""

    name: str
    pads: tuple
    tip: tuple
    axis: tuple = (1.0, 0.0, 0.0)
    wall_k: float = 2000.0  # N/m per wall: 2 N per mm of travel
    open_force: float = 4.0  # N: below this nothing leaves the tip
    flow: float = 50.0  # uL/s per N above open_force
    drop_ul: float = 40.0  # one drop's volume
    tip_down_deg: float = 30.0
    capacity_ul: float = 10000.0  # liquid in the bottle (10 mL)


@dataclass
class Drop:
    """One released drop: control step, where its tip was, the landing zone and point, and task extras."""

    step: int
    tip: np.ndarray
    zone: str
    at: np.ndarray | None
    info: dict = field(default_factory=dict)


@dataclass
class _State:
    episode: int = -1
    key: object = None
    step: int = 0
    hanging: float = 0.0
    released: float = 0.0
    drops: list = field(default_factory=list)


_CFG: dict = {"bottle": None, "land": None}
_S = _State()


def setup(bottle: Bottle, land) -> None:
    """Register the task's bottle and its landing function land(objects, tip_world) -> (zone, point, info)."""
    _CFG["bottle"], _CFG["land"] = bottle, land


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tip(objects, spec: Bottle | None = None) -> np.ndarray:
    """The bottle tip's end in world coordinates."""
    spec = spec or _CFG["bottle"]
    s = objects[spec.name]
    return np.asarray(s.pos, float) + _rot(s.quat_xyzw) @ np.asarray(spec.tip, float)


def tip_dir(objects, spec: Bottle | None = None) -> np.ndarray:
    spec = spec or _CFG["bottle"]
    return _rot(objects[spec.name].quat_xyzw) @ np.asarray(spec.axis, float)


def tip_down(objects, spec: Bottle | None = None) -> bool:
    """The tip points at least tip_down_deg below horizontal."""
    spec = spec or _CFG["bottle"]
    return float(tip_dir(objects, spec)[2]) <= -math.sin(math.radians(spec.tip_down_deg))


def squeeze(objects, spec: Bottle | None = None) -> float:
    """The squeeze (m): the largest mean inward travel of an opposite pair of wall pads (0 at rest)."""
    spec = spec or _CFG["bottle"]
    pairs = (spec.pads,) if isinstance(spec.pads[0][0], str) else spec.pads
    return max(float(np.mean([max(0.0, sg * kit.joint(objects, f"{spec.name}/{p}").pos) for p, sg in pair]))
               for pair in pairs)


def squeeze_force(objects, spec: Bottle | None = None) -> float:
    spec = spec or _CFG["bottle"]
    return spec.wall_k * squeeze(objects, spec)


def _fresh_if_new():
    global _S
    if _S.episode != kit.episode():
        _S = _State(episode=kit.episode())


def update(objects) -> None:
    """Advance the bottle by one control step (call once per step; repeated calls with the same objects are ignored)."""
    spec = _CFG["bottle"]
    if spec is None:
        return
    _fresh_if_new()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else id(objects)
    if key == _S.key:
        return
    _S.key = key
    _S.step += 1
    if spec.name not in objects:
        return
    f = squeeze_force(objects, spec)
    if f < spec.open_force or not tip_down(objects, spec):
        _S.hanging = 0.0  # easing off (or turning the tip up) draws a hanging drop back in
        return
    left = spec.capacity_ul - _S.released - _S.hanging
    _S.hanging += min(max(0.0, left), spec.flow * (f - spec.open_force) * DT)
    while _S.hanging >= spec.drop_ul:  # a full drop detaches and falls
        _S.hanging -= spec.drop_ul
        _S.released += spec.drop_ul
        t = tip(objects, spec)
        zone, at, info = _CFG["land"](objects, t)
        _S.drops.append(Drop(_S.step, t, zone, None if at is None else np.asarray(at, float), dict(info or {})))


def drops() -> list:
    """The drops released this trial, in order."""
    _fresh_if_new()
    return list(_S.drops)


def hanging() -> float:
    """uL hanging at the tip right now (not yet a drop)."""
    _fresh_if_new()
    return _S.hanging


def steps() -> int:
    _fresh_if_new()
    return _S.step


# -- visuals ---------------------------------------------------------------------------------------------------------
_VIS = {"markers": None, "failed": False}
BEAD_R = 0.0021  # m: a 40 uL drop is a 4.2 mm bead
MAX_BEADS = 12
COLOR = (0.15, 0.45, 0.95)


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        lin = tuple(kit._linear(c) for c in COLOR)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/dropkit",
            markers={"drop": sim_utils.SphereCfg(radius=1.0, visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=lin, roughness=0.1))}))
    except Exception as e:  # noqa: BLE001  (visuals are optional)
        print(f"dropkit: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(objects, origin) -> None:
    m = _markers()
    if m is None:
        return
    spec = _CFG["bottle"]
    tr, sc = [], []
    for d in _S.drops[-MAX_BEADS:]:
        p = d.at if d.at is not None else d.tip
        tr.append(np.asarray(p, float) + origin), sc.append((BEAD_R, BEAD_R, BEAD_R * 0.6))
    if _S.hanging > 0 and spec.name in objects:
        r = BEAD_R * (_S.hanging / spec.drop_ul) ** (1 / 3)
        tr.append(tip(objects, spec) - np.array([0, 0, r]) + origin), sc.append((r, r, r))
    while len(tr) < MAX_BEADS + 1:  # park unused beads out of sight
        tr.append(np.array([0.0, 0.0, -10.0])), sc.append((1e-4, 1e-4, 1e-4))
    m.visualize(translations=np.array(tr, float), orientations=np.tile([0, 0, 0, 1.0], (len(tr), 1)),
                scales=np.array(sc, float), marker_indices=np.zeros(len(tr), int))


def _patch(scene) -> None:
    if getattr(scene, "_dropkit", False):
        return
    scene._dropkit = True
    inner = scene.object_states

    def object_states(env, names):
        out = inner(env, names)
        update(out)
        if os.environ.get("DROPKIT_DEBUG") and _CFG["bottle"] is not None and _CFG["bottle"].name in out:
            print("dropkit", _S.step, round(squeeze_force(out), 2), tip_down(out), round(_S.hanging, 1),
                  [(d.step, d.zone) for d in _S.drops], flush=True)
        try:
            _draw(out, env.unwrapped.scene.env_origins[0].detach().cpu().numpy())
        except Exception as e:  # noqa: BLE001  (never let visuals break a run)
            if not _VIS["failed"]:
                print(f"dropkit: visuals off ({e!r})", file=sys.stderr, flush=True)
            _VIS["failed"] = True
        return out

    scene.object_states = object_states


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that also adds the drop step after yamkit's patch (as pourkit does)
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())
