"""On the station table: an 18 V cordless drill-driver lying on its side with its flat battery pack on, and a charged
pack of the same kind lying 26 cm away, in line with the drill's rails. The instruction is "Swap the drill's battery
for the charged one."

The pack slides along the tool's T-rail and locks with a spring catch; the catch's button, on the pack's top face
ahead of the tool's foot, must be held in while the pack slides off. The charged pack slides on until its catch
clicks into the notch.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "Swap the drill's battery for the charged one." --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402
import taskkit as tk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Swap the drill's battery for the charged one."
SEAT = np.array([g.SEAT[0], g.SEAT[1], g.Z_C])
SPARE0 = SEAT + [0, g.SPARE_DY, 0]

DRILL = UsdBody("drill", usd="assets/drill.usda", pos=tuple(SEAT), size=(0.25, 0.32, g.W), mass=g.DRILL_MASS,
                friction=(0.8, 0.7), solver_iters=32)
FLAT = UsdBody("pack_flat", usd="assets/pack_flat.usda", pos=tuple(SEAT), size=(g.H, g.L, g.W), articulation=True,
               joint_pos=(("catch", 0.0),), drives=(("catch", g.LATCH_K, 2.0),), friction=(0.5, 0.4),
               solver_iters=32)
CHARGED = UsdBody("pack_charged", usd="assets/pack_charged.usda", pos=tuple(SPARE0), size=(g.H, g.L, g.W),
                  articulation=True, joint_pos=(("catch", 0.0),), drives=(("catch", g.LATCH_K, 2.0),),
                  friction=(0.5, 0.4), solver_iters=32)
OBJECTS = [DRILL, FLAT, CHARGED]

_STATE: dict = {}
_STILL = tk.Still()


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(flat=None, charged=None, released=False, still=0.0, why=[])
    _STILL.reset()


_reset_state()


def _on_rail(drill, pack) -> dict:
    """The pack's pose in the drill's frame: how far forward of home along the rail (y), how far off the rail line
    (x, z) and how twisted (degrees)."""
    Rd = tk.rotation(drill.quat_xyzw)
    rel = Rd.T @ (np.asarray(pack.pos) - np.asarray(drill.pos))
    Rp = Rd.T @ tk.rotation(pack.quat_xyzw)
    twist = float(np.degrees(np.arccos(np.clip((np.trace(Rp) - 1) / 2, -1, 1))))
    fwd = -float(rel[1])                             # +: forward of home (toward the chuck end)
    across = float(max(abs(rel[0]), abs(rel[2])))
    return {"fwd": fwd, "across": across, "twist": twist, "catch": float(-pack.joints["catch"]),
            "dx": float(rel[0]), "dz": float(rel[2])}


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    st = object_states(environment, ["drill", "pack_flat", "pack_charged"])
    d = st["drill"]
    for key, name in (("flat", "pack_flat"), ("charged", "pack_charged")):
        _STATE[key] = _on_rail(d, st[name])
    c = st["pack_charged"]
    Rc = tk.rotation(c.quat_xyzw)
    half = np.array([g.H / 2 + g.WALL_H, g.L / 2, g.W / 2]) + g.CLEAR
    tips = _STATE["tips"] = tk.fingertips(environment)
    _STATE["released"] = not any(np.all(np.abs(Rc.T @ (p - np.asarray(c.pos))) <= half) for p in tips)
    s = [_STILL.update(n, st[n].pos, st[n].quat_xyzw, st[n].lin_vel, st[n].ang_vel) for n in ("drill", "pack_flat",
                                                                                            "pack_charged")]
    _STATE["still"] = min(s)


def outcome(objects=None) -> str:
    """'swapped' (the charged pack home and latched, the flat pack off the rail, both let go and still), 'on' (flat
    pack off, charged pack on the rail by at least 3 cm), 'off' (flat pack off the rail), or 'other'."""
    f, c = _STATE["flat"], _STATE["charged"]
    if f is None:
        return "other"
    flat_off = f["fwd"] >= g.OFF or f["across"] > 0.02
    if not flat_off:
        return "other"
    lined = c["across"] <= g.ALIGN_TOL and c["twist"] <= 5.0
    if lined and abs(c["fwd"]) <= g.HOME_TOL and c["catch"] <= g.LATCHED and _STATE["released"] \
            and _STATE["still"] >= g.STILL_S:
        return "swapped"
    if lined and -g.HOME_TOL <= c["fwd"] <= g.FOOT_LEN - g.ENGAGED:
        return "on"
    return "off"


def success(objects) -> bool:
    return outcome() == "swapped"


def score(objects) -> float:
    """1.0 the charged pack is home on the drill (within 2 mm of the stop) with its catch up, no fingertip within
    1 cm of it, and the drill and both packs still for 1 s, the flat pack off the rail; 0.5 the flat pack is off and
    the charged pack is on the rail by at least 3 cm; 0.25 the flat pack is off the rail; 0 otherwise."""
    return {"swapped": 1.0, "on": 0.5, "off": 0.25}.get(outcome(), 0.0)
